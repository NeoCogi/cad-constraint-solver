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

//! Runtime model/solver types and failure reporting.

mod errors;
mod result;
mod seed;
mod solve;
mod trace;

use constraint_solver::Exp;
use std::collections::HashMap;

pub use errors::SolveError;
pub use result::SolveResult;
pub use seed::SolveSeed;
pub use trace::{ConstraintIssue, FailureKind, IssueTraceFrame, SolveFailureReport};

pub(crate) use solve::{collect_var_names, evaluate_exp};
pub(crate) use trace::EquationOrigin;

/// Declared public symbol shape.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SymbolType {
    /// A scalar (`f64`) symbol.
    Scalar,
    /// A 2D vector symbol (`rs_math3d::Vec2d`).
    Vec2d,
    /// A 3D vector symbol (`rs_math3d::Vec3d`).
    Vec3d,
}

/// A compiled DSL model ready to be solved.
///
/// `Model` stores lowered scalar equations and visibility-aware symbol metadata.
#[derive(Debug, Clone)]
pub struct Model {
    pub(crate) equations: Vec<Exp>,
    pub(crate) equation_origins: Vec<EquationOrigin>,
    // Public API symbols only (`export` declarations, or legacy top-level decls).
    pub(crate) public_symbols: HashMap<String, SymbolType>,
    // Default seed values for *all* flattened variables (public + local).
    pub(crate) defaults: HashMap<String, f64>,
    // All flattened declared variable names.
    pub(crate) flattened_names: Vec<String>,
    // All flattened variable names referenced by final equations.
    pub(crate) solver_names: Vec<String>,
    // Public flattened names (subset of `flattened_names`).
    pub(crate) public_flattened_names: Vec<String>,
    // Public flattened names that are referenced by equations.
    pub(crate) public_solver_names: Vec<String>,
    // Local/internal flattened names that are referenced by equations.
    pub(crate) hidden_solver_names: Vec<String>,
}

impl Model {
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn from_lowered_parts(
        equations: Vec<Exp>,
        equation_origins: Vec<EquationOrigin>,
        public_symbols: HashMap<String, SymbolType>,
        defaults: HashMap<String, f64>,
        flattened_names: Vec<String>,
        solver_names: Vec<String>,
        public_flattened_names: Vec<String>,
        public_solver_names: Vec<String>,
        hidden_solver_names: Vec<String>,
    ) -> Self {
        Self {
            equations,
            equation_origins,
            public_symbols,
            defaults,
            flattened_names,
            solver_names,
            public_flattened_names,
            public_solver_names,
            hidden_solver_names,
        }
    }

    /// Returns lowered scalar equations as `constraint_solver::Exp`.
    pub fn equations(&self) -> &[Exp] {
        &self.equations
    }

    /// Returns all flattened symbol component names declared by the DSL.
    ///
    /// This includes local/internal variables.
    pub fn flattened_names(&self) -> &[String] {
        &self.flattened_names
    }

    /// Returns flattened variable names that appear in solver equations.
    ///
    /// This includes local/internal variables.
    pub fn solver_names(&self) -> &[String] {
        &self.solver_names
    }

    /// Returns flattened *public* names.
    pub fn public_flattened_names(&self) -> &[String] {
        &self.public_flattened_names
    }

    /// Returns flattened *public* names used by equations.
    pub fn public_solver_names(&self) -> &[String] {
        &self.public_solver_names
    }

    /// Returns the declared public DSL type for a top-level symbol.
    pub fn symbol_type(&self, name: &str) -> Option<SymbolType> {
        self.public_symbols.get(name).copied()
    }

    /// Builds a default [`SolveSeed`] for public API variables.
    ///
    /// Local/internal variables are intentionally omitted from this seed.
    pub fn bootstrap_seed(&self) -> SolveSeed {
        let mut initial = HashMap::new();
        for name in &self.public_flattened_names {
            if let Some(value) = self.defaults.get(name) {
                initial.insert(name.clone(), *value);
            }
        }

        SolveSeed {
            initial,
            unknowns: self.public_solver_names.iter().cloned().collect(),
        }
    }
}
