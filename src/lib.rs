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

//! CAD-oriented DSL layer on top of `constraint-solver`.
//!
//! This crate provides:
//! - Typed DSL parsing (`scalar`, `vec2d`, `vec3d`, imports).
//! - Multi-file project composition via `import "..."`.
//! - Embedded standard-library DSL modules (`stdlib/std2d.dsl`, `stdlib/std3d.dsl`).
//! - Lowering/type-checking to scalar `constraint_solver::Exp` equations.
//! - Solve-time adapters for `rs_math3d` vector types.
//! - Source-mapped compile/solve diagnostics.
//!
//! Preferred system-call semantics:
//! - `system` parameters use `in`, `out`, `inout` direction modes.
//! - Calls use direct syntax `name(args...);`.
//! - Parameters alias caller arguments; repeated calls on the same variables accumulate constraints.
//! - Local declarations are isolated per invocation.
//! - Recursive calls are rejected.
//! - `out` parameters are readable in results but not externally seed-writable.
//! - Legacy keywords (`constraint_fn`, `use`, `export`, `local`) are rejected.

mod ast;
mod compiler;
mod diagnostics;
mod model;
mod parser;
mod project;
mod stdlib;

pub use ast::{
    BinOp, CallStmt, Decl, Expr, ExprKind, Field, Import, Param, ParamMode, Program, SourceSpan,
    Stmt, StmtKind, System, SystemStmt, SystemStmtKind, TypeName,
};
pub use diagnostics::CompileError;
pub use model::{
    ConstraintIssue, FailureKind, IssueTraceFrame, Model, SolveError, SolveFailureReport,
    SolveResult, SolveSeed, SymbolType,
};
pub use project::DslSource;
pub use stdlib::{
    STDLIB_STD2D_PATH, STDLIB_STD2D_SOURCE, STDLIB_STD3D_PATH, STDLIB_STD3D_SOURCE,
    compile_dsl_project_system_with_stdlib, compile_dsl_project_with_stdlib,
    merge_with_standard_library_sources, standard_library_sources,
};

use parser::parse_program;

/// Parses and compiles DSL source into a ready-to-solve [`Model`].
///
/// If the source contains a single `system`, that system is compiled.
/// If it contains no `system`, top-level statements are compiled.
/// If it contains multiple systems, use [`compile_dsl_system`] to select one.
///
/// # Errors
///
/// Returns [`CompileError`] with source line/column and caret highlight when
/// parsing or lowering fails.
pub fn compile_dsl(source: &str) -> Result<Model, CompileError> {
    let program = parse_program(source)?;
    if let Some(import) = program.imports.first() {
        return Err(CompileError::from_span_in_source(
            "Imports require project compilation (use compile_dsl_project...)",
            "<inline>",
            source,
            &import.span,
        ));
    }
    compiler::lower_program("<inline>", source, &program, None)
}

/// Parses and compiles one named `system` from source.
pub fn compile_dsl_system(source: &str, system_name: &str) -> Result<Model, CompileError> {
    let program = parse_program(source)?;
    if let Some(import) = program.imports.first() {
        return Err(CompileError::from_span_in_source(
            "Imports require project compilation (use compile_dsl_project...)",
            "<inline>",
            source,
            &import.span,
        ));
    }
    compiler::lower_program("<inline>", source, &program, Some(system_name))
}

/// Parses DSL source into a spanned AST [`Program`].
pub fn parse_dsl(source: &str) -> Result<Program, CompileError> {
    parse_program(source)
}

/// Compiles a project (multiple source files) starting from `entry_path`.
///
/// Imports are resolved lexically relative to each importer file path.
pub fn compile_dsl_project(entry_path: &str, sources: &[DslSource]) -> Result<Model, CompileError> {
    project::compile_dsl_project_impl(entry_path, None, sources)
}

/// Compiles one named `system` from a project rooted at `entry_path`.
pub fn compile_dsl_project_system(
    entry_path: &str,
    system_name: &str,
    sources: &[DslSource],
) -> Result<Model, CompileError> {
    project::compile_dsl_project_impl(entry_path, Some(system_name), sources)
}

/// Compiles a project using a custom source loader callback.
///
/// The loader receives normalized source paths and returns full source text.
pub fn compile_dsl_project_with_loader<F>(
    entry_path: &str,
    system_name: Option<&str>,
    loader: F,
) -> Result<Model, CompileError>
where
    F: FnMut(&str) -> Result<String, String>,
{
    project::compile_dsl_project_with_loader(entry_path, system_name, loader)
}

/// Compiles DSL and returns a bootstrap solve seed in one call.
pub fn compile_with_bootstrap(source: &str) -> Result<(Model, SolveSeed), CompileError> {
    let model = compile_dsl(source)?;
    let seed = model.bootstrap_seed();
    Ok((model, seed))
}

/// Convenience function that compiles and solves in one step.
pub fn solve_dsl(source: &str, seed: SolveSeed) -> Result<SolveResult, SolveError> {
    let model = compile_dsl(source).map_err(SolveError::DslCompile)?;
    model.solve(seed)
}

#[cfg(test)]
mod tests;
