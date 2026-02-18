/*
MIT License

Copyright (c) 2026 Raja Lehtihet and Wael El Oraiby

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

//! Solver execution and failure mapping.

use constraint_solver::{
    Compiler as SolverCompiler, EquationTrace as RawEquationTrace, Exp, NewtonRaphsonSolver,
    SolverError as RawSolverError, SolverRunDiagnostic as RawSolverRunDiagnostic,
};
use std::collections::{BTreeSet, HashMap};

use super::{ConstraintIssue, FailureKind, Model, SolveError, SolveFailureReport, SolveResult};

const OVERCONSTRAINED_RESIDUAL_THRESHOLD: f64 = 1e-7;

impl Model {
    /// Solves the model using Newton-Raphson without line search.
    pub fn solve(&self, seed: super::SolveSeed) -> Result<SolveResult, SolveError> {
        self.solve_internal(seed, false)
    }

    /// Solves the model using Newton-Raphson with line search.
    pub fn solve_with_line_search(
        &self,
        seed: super::SolveSeed,
    ) -> Result<SolveResult, SolveError> {
        self.solve_internal(seed, true)
    }

    fn solve_internal(
        &self,
        seed: super::SolveSeed,
        use_line_search: bool,
    ) -> Result<SolveResult, SolveError> {
        // Ensure caller can only modify public API variables.
        self.validate_seed(&seed)?;

        let super::SolveSeed {
            initial: seed_initial,
            unknowns: seed_unknowns,
        } = seed;

        // Start from declaration defaults, then apply caller overrides.
        let solver_name_set: BTreeSet<String> = self.solver_names.iter().cloned().collect();
        let mut initial = self.defaults.clone();
        initial.extend(seed_initial);
        // Backend rejects unknown names in the initial map.
        initial.retain(|name, _| solver_name_set.contains(name));

        let selected_unknowns: BTreeSet<String> = if seed_unknowns.is_empty() {
            self.solver_names.iter().cloned().collect()
        } else {
            let mut unknowns = seed_unknowns.clone();
            unknowns.extend(self.hidden_solver_names.iter().cloned());
            unknowns
        };
        let unknown_count = selected_unknowns.len();

        // Convert lowered symbolic equations to backend representation.
        let compiled =
            SolverCompiler::compile(&self.equations).map_err(SolveError::InternalCompile)?;

        let solver = if seed_unknowns.is_empty() {
            // Solve all variables when unknowns are not specified.
            NewtonRaphsonSolver::new(compiled)
        } else {
            // Always include hidden/local solver vars as unknowns.
            let unknowns: Vec<String> = selected_unknowns.iter().cloned().collect();
            let names: Vec<&str> = unknowns.iter().map(String::as_str).collect();
            NewtonRaphsonSolver::new_with_variables(compiled, &names)?
        };

        // Attach equation trace metadata so backend diagnostics carry origin context.
        let traces: Vec<Option<RawEquationTrace>> = self
            .equation_origins
            .iter()
            .enumerate()
            .map(|(idx, origin)| {
                Some(RawEquationTrace {
                    constraint_id: idx,
                    description: origin.description.clone(),
                })
            })
            .collect();
        let solver = solver.try_with_equation_traces(traces)?;

        // Choose solve mode requested by caller.
        let raw = if use_line_search {
            match solver.solve_with_line_search(initial) {
                Ok(solution) => solution,
                Err(err) => return Err(self.map_solver_error(err, unknown_count)),
            }
        } else {
            match solver.solve(initial) {
                Ok(solution) => solution,
                Err(err) => return Err(self.map_solver_error(err, unknown_count)),
            }
        };

        // Even when backend reports convergence, overconstrained systems can settle on a
        // least-squares fit with non-zero residual. Surface this as inconsistency.
        if self.equations.len() > unknown_count && raw.error > OVERCONSTRAINED_RESIDUAL_THRESHOLD {
            let residuals = self.compute_residuals(&raw.values);
            return Err(SolveError::Failure(self.failure_report_from_parts(
                FailureKind::OverconstrainedInconsistency,
                format!(
                    "Overconstrained system is inconsistent (least-squares residual {:.3e})",
                    raw.error
                ),
                raw.iterations,
                raw.error,
                raw.values.clone(),
                residuals,
                unknown_count,
            )));
        }

        Ok(SolveResult { raw })
    }

    fn validate_seed(&self, seed: &super::SolveSeed) -> Result<(), SolveError> {
        let public_name_set: BTreeSet<String> =
            self.public_flattened_names.iter().cloned().collect();
        let public_solver_set: BTreeSet<String> =
            self.public_solver_names.iter().cloned().collect();

        // Initial values may include only public names.
        for name in seed.initial.keys() {
            if !public_name_set.contains(name) {
                return Err(SolveError::UnknownVariable(name.clone()));
            }
        }

        // Explicit unknown set may include only public solver names.
        for name in &seed.unknowns {
            if !public_solver_set.contains(name) {
                return Err(SolveError::UnknownVariable(name.clone()));
            }
        }
        Ok(())
    }

    pub(crate) fn map_solver_error(&self, err: RawSolverError, unknown_count: usize) -> SolveError {
        match err {
            RawSolverError::NoConvergence(diag) => {
                let overconstrained = self.equations.len() > unknown_count;
                let kind = if overconstrained {
                    FailureKind::OverconstrainedInconsistency
                } else {
                    FailureKind::NonConvergence
                };
                SolveError::Failure(self.failure_report(kind, diag, unknown_count))
            }
            RawSolverError::SingularMatrix(diag) => SolveError::Failure(self.failure_report(
                FailureKind::SingularMatrix,
                diag,
                unknown_count,
            )),
            other => SolveError::Solver(other),
        }
    }

    fn failure_report(
        &self,
        kind: FailureKind,
        diag: RawSolverRunDiagnostic,
        unknown_count: usize,
    ) -> SolveFailureReport {
        self.failure_report_from_parts(
            kind,
            diag.message,
            diag.iterations,
            diag.error,
            diag.values,
            diag.residuals,
            unknown_count,
        )
    }

    fn failure_report_from_parts(
        &self,
        kind: FailureKind,
        message: String,
        iterations: usize,
        error: f64,
        values: HashMap<String, f64>,
        residuals: Vec<f64>,
        unknown_count: usize,
    ) -> SolveFailureReport {
        let mut ranked: Vec<(usize, f64)> = residuals
            .iter()
            .enumerate()
            .map(|(idx, value)| (idx, value.abs()))
            .collect();
        ranked.sort_by(|a, b| b.1.total_cmp(&a.1));

        let mut issues = Vec::new();
        for (eq_idx, magnitude) in ranked.into_iter().take(8) {
            let residual = residuals.get(eq_idx).copied().unwrap_or(0.0);
            let issue = if let Some(origin) = self.equation_origins.get(eq_idx) {
                ConstraintIssue {
                    equation_index: eq_idx,
                    residual,
                    magnitude,
                    description: origin.description.clone(),
                    file: origin.file.clone(),
                    line: origin.line,
                    column: origin.column,
                    snippet: origin.snippet.clone(),
                    pointer: origin.pointer.clone(),
                    traceback: origin.traceback.clone(),
                }
            } else {
                ConstraintIssue {
                    equation_index: eq_idx,
                    residual,
                    magnitude,
                    description: format!("equation #{eq_idx}"),
                    file: String::new(),
                    line: 0,
                    column: 0,
                    snippet: String::new(),
                    pointer: String::new(),
                    traceback: Vec::new(),
                }
            };
            issues.push(issue);
        }

        SolveFailureReport {
            kind,
            message,
            iterations,
            error,
            equation_count: self.equations.len(),
            unknown_count,
            overconstrained: self.equations.len() > unknown_count,
            values,
            residuals,
            issues,
        }
    }

    fn compute_residuals(&self, values: &HashMap<String, f64>) -> Vec<f64> {
        self.equations
            .iter()
            .map(|exp| evaluate_exp(exp, values).unwrap_or(f64::NAN))
            .collect()
    }
}

/// Attempts constant-fold evaluation using known defaults.
pub(crate) fn evaluate_exp(exp: &Exp, env: &HashMap<String, f64>) -> Option<f64> {
    // A minimal evaluator used only for compile-time default inference.
    match exp {
        Exp::Val(v) => Some(*v),
        Exp::Var(name) => env.get(name).copied(),
        Exp::Add(l, r) => Some(evaluate_exp(l, env)? + evaluate_exp(r, env)?),
        Exp::Sub(l, r) => Some(evaluate_exp(l, env)? - evaluate_exp(r, env)?),
        Exp::Mul(l, r) => Some(evaluate_exp(l, env)? * evaluate_exp(r, env)?),
        Exp::Div(l, r) => Some(evaluate_exp(l, env)? / evaluate_exp(r, env)?),
        Exp::Power(base, power) => Some(evaluate_exp(base, env)?.powf(*power)),
        Exp::Neg(v) => Some(-evaluate_exp(v, env)?),
        Exp::Sin(v) => Some(evaluate_exp(v, env)?.sin()),
        Exp::Cos(v) => Some(evaluate_exp(v, env)?.cos()),
        Exp::Ln(v) => Some(evaluate_exp(v, env)?.ln()),
        Exp::Exp(v) => Some(evaluate_exp(v, env)?.exp()),
    }
}

/// Collects variable names referenced in an expression.
pub(crate) fn collect_var_names(exp: &Exp, out: &mut BTreeSet<String>) {
    // DFS walk over the scalar expression tree.
    match exp {
        Exp::Val(_) => {}
        Exp::Var(name) => {
            out.insert(name.clone());
        }
        Exp::Add(l, r) | Exp::Sub(l, r) | Exp::Mul(l, r) | Exp::Div(l, r) => {
            collect_var_names(l, out);
            collect_var_names(r, out);
        }
        Exp::Power(base, _)
        | Exp::Neg(base)
        | Exp::Sin(base)
        | Exp::Cos(base)
        | Exp::Ln(base)
        | Exp::Exp(base) => {
            collect_var_names(base, out);
        }
    }
}
