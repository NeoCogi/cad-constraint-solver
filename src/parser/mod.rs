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

//! `nom` parser for the DSL.
//!
//! The grammar supports:
//! - top-level `import "path.dsl";` declarations
//! - top-level `system` definitions with optional directional parameters
//! - top-level declarations/constraints when no named `system` is used
//! - call statements as `name(args...);`
//!
//! Expressions support:
//! - numeric literals and identifiers
//! - vector literals (`[a, b]`, `[a, b, c]`)
//! - unary negation
//! - binary `+ - * /`
//! - function calls
//! - field access (`.x .y .z`)

mod expr;
mod statements;
mod utils;

use crate::ast::{Program, SourceSpan, Span};
use crate::diagnostics::CompileError;
use nom::{
    IResult,
    combinator::all_consuming,
    error::{VerboseError, VerboseErrorKind},
    multi::many0,
    sequence::delimited,
};

use self::statements::{TopItem, top_item};
use self::utils::ws0;

type PResult<'a, O> = IResult<Span<'a>, O, VerboseError<Span<'a>>>;

/// Parses full DSL source into a spanned AST program.
pub fn parse_program(source: &str) -> Result<Program, CompileError> {
    parse_program_in_source(source, "<inline>")
}

/// Parses full DSL source while tagging diagnostics with a source name/path.
pub(crate) fn parse_program_in_source(
    source: &str,
    source_name: &str,
) -> Result<Program, CompileError> {
    let input = Span::new(source);
    // `all_consuming` ensures trailing garbage is treated as syntax error.
    let (_, items) = match all_consuming(delimited(ws0, many0(top_item), ws0))(input) {
        Ok(v) => v,
        Err(err) => return Err(parse_error_to_compile_error(err, source_name, source)),
    };

    let mut imports = Vec::new();
    let mut systems = Vec::new();
    let mut statements = Vec::new();

    for item in items {
        match item {
            TopItem::Import(import) => imports.push(import),
            TopItem::System(system) => systems.push(system),
            TopItem::Stmt(stmt) => statements.push(stmt),
        }
    }

    Ok(Program {
        imports,
        systems,
        statements,
    })
}

/// Converts a `nom` verbose error to crate-level compile diagnostics.
fn parse_error_to_compile_error(
    err: nom::Err<VerboseError<Span<'_>>>,
    source_name: &str,
    source: &str,
) -> CompileError {
    match err {
        nom::Err::Incomplete(_) => CompileError::message_in_file("Incomplete input", source_name),
        nom::Err::Error(e) | nom::Err::Failure(e) => {
            // Use the deepest recorded parser error as the diagnostic anchor.
            if let Some((span, kind)) = e.errors.last() {
                let span = SourceSpan::from_bounds(*span, *span);
                let detail = match kind {
                    VerboseErrorKind::Context(ctx) => format!("Syntax error: expected {ctx}"),
                    VerboseErrorKind::Char(c) => format!("Syntax error: expected '{c}'"),
                    VerboseErrorKind::Nom(kind) => format!("Syntax error near {kind:?}"),
                };
                CompileError::from_span_in_source(detail, source_name, source, &span)
            } else {
                CompileError::message_in_file("Syntax error", source_name)
            }
        }
    }
}
