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

//! Error types surfaced by compile/solve APIs.

use crate::CompileError;
use constraint_solver::{CompileError as SolverCompileError, SolverError as RawSolverError};
use std::fmt;

use super::SolveFailureReport;

/// Errors produced by compile or solve stages.
#[derive(Debug)]
pub enum SolveError {
    /// DSL compilation failed with source diagnostics.
    DslCompile(CompileError),
    /// A variable name was not declared or not publicly exposed.
    UnknownVariable(String),
    /// A requested value was not present in the final solution map.
    MissingValue(String),
    /// Lowered equations could not be compiled by `constraint-solver`.
    InternalCompile(SolverCompileError),
    /// Runtime solver failure with structured traceback data.
    Failure(SolveFailureReport),
    /// Runtime solve failure from the backend solver.
    Solver(RawSolverError),
}

impl fmt::Display for SolveError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SolveError::DslCompile(err) => write!(f, "{err}"),
            SolveError::UnknownVariable(name) => write!(f, "Unknown variable '{name}'"),
            SolveError::MissingValue(name) => write!(f, "Missing value for variable '{name}'"),
            SolveError::InternalCompile(err) => {
                write!(f, "Failed to compile solver equations: {err}")
            }
            SolveError::Failure(report) => write!(
                f,
                "{} ({:?}, residual {:.3e}, iterations {})",
                report.message, report.kind, report.error, report.iterations
            ),
            SolveError::Solver(err) => write!(f, "{err:?}"),
        }
    }
}

impl std::error::Error for SolveError {}

impl From<RawSolverError> for SolveError {
    fn from(value: RawSolverError) -> Self {
        SolveError::Solver(value)
    }
}

impl SolveError {
    /// Returns structured failure report when available.
    pub fn failure_report(&self) -> Option<&SolveFailureReport> {
        match self {
            SolveError::Failure(report) => Some(report),
            _ => None,
        }
    }
}
