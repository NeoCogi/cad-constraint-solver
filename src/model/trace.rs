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

//! Traceback and failure-report structures.

use std::collections::HashMap;

/// Classification for solver failure modes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FailureKind {
    /// Overconstrained system with no feasible solution under tolerance.
    OverconstrainedInconsistency,
    /// Solver failed to reduce residuals within iteration/damping limits.
    NonConvergence,
    /// Solver encountered a singular Jacobian/linear solve failure.
    SingularMatrix,
}

/// One `use ...` invocation frame in an issue traceback.
#[derive(Debug, Clone)]
pub struct IssueTraceFrame {
    /// Called constraint function name.
    pub function: String,
    /// File containing the `use` call site.
    pub file: String,
    /// 1-based source line for call site.
    pub line: usize,
    /// 1-based source column for call site.
    pub column: usize,
    /// Source line snippet at call site.
    pub snippet: String,
    /// Caret pointer for `snippet`.
    pub pointer: String,
}

/// One high-residual equation mapped back to DSL source.
#[derive(Debug, Clone)]
pub struct ConstraintIssue {
    /// Equation index in the lowered scalar equation list.
    pub equation_index: usize,
    /// Signed residual from solver diagnostics.
    pub residual: f64,
    /// Absolute residual magnitude (`abs(residual)`).
    pub magnitude: f64,
    /// Human-readable origin description (system/call path/component).
    pub description: String,
    /// Source file/path for the originating DSL constraint.
    pub file: String,
    /// 1-based source line for the originating DSL constraint.
    pub line: usize,
    /// 1-based source column for the originating DSL constraint.
    pub column: usize,
    /// Source line snippet.
    pub snippet: String,
    /// Caret pointer for `snippet`.
    pub pointer: String,
    /// `use` call chain frames leading to this constraint.
    pub traceback: Vec<IssueTraceFrame>,
}

/// Structured report returned for solver-side failures.
#[derive(Debug, Clone)]
pub struct SolveFailureReport {
    /// High-level failure classification.
    pub kind: FailureKind,
    /// Backend-provided failure message.
    pub message: String,
    /// Iteration count reached before failure.
    pub iterations: usize,
    /// Final residual norm.
    pub error: f64,
    /// Number of scalar equations in the lowered system.
    pub equation_count: usize,
    /// Number of unknowns selected for solving.
    pub unknown_count: usize,
    /// Whether the selected solve set is overconstrained.
    pub overconstrained: bool,
    /// Snapshot of variable values at failure time.
    pub values: HashMap<String, f64>,
    /// Raw residual vector from backend.
    pub residuals: Vec<f64>,
    /// Top residual equations mapped back to source locations.
    pub issues: Vec<ConstraintIssue>,
}

/// Internal source mapping attached to each lowered scalar equation.
#[derive(Debug, Clone)]
pub(crate) struct EquationOrigin {
    pub(crate) description: String,
    pub(crate) file: String,
    pub(crate) line: usize,
    pub(crate) column: usize,
    pub(crate) snippet: String,
    pub(crate) pointer: String,
    pub(crate) traceback: Vec<IssueTraceFrame>,
}
