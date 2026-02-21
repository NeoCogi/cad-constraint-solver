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

//! Statement and top-level item parsers.

use crate::ast::{
    CallStmt, Decl, Expr, Import, Param, ParamMode, SourceSpan, Span, Stmt, StmtKind, System,
    SystemStmt, SystemStmtKind,
};
use nom::Parser;
use nom::{
    branch::alt,
    bytes::complete::tag,
    combinator::{map, opt},
    error::context,
    multi::{many0, separated_list0},
    sequence::{delimited, preceded},
};

use super::PResult;
use super::expr::expr;
use super::utils::{identifier, string_literal, type_name, ws, ws_char};

/// One top-level AST item.
pub(super) enum TopItem {
    Import(Import),
    System(System),
    Stmt(Stmt),
}

/// Parses one top-level item.
pub(super) fn top_item(input: Span<'_>) -> PResult<'_, TopItem> {
    // Try definitions first to avoid treating their bodies as statements.
    alt((
        map(import_stmt, TopItem::Import),
        map(system_def, TopItem::System),
        map(statement, TopItem::Stmt),
    ))
    .parse(input)
}

/// Parses one top-level import (`import "path.dsl";`).
fn import_stmt(input: Span<'_>) -> PResult<'_, Import> {
    let start = input;
    let (input, _) = ws(context("'import'", tag("import"))).parse(input)?;
    let (input, path) = context("import path", ws(string_literal)).parse(input)?;
    let (input, _) = context("';'", ws_char(';')).parse(input)?;
    let span = SourceSpan::from_bounds(start, input);
    Ok((input, Import { path, span }))
}

/// Parses a top-level `system` definition.
fn system_def(input: Span<'_>) -> PResult<'_, System> {
    let start = input;
    let (input, _) = ws(context("'system'", tag("system"))).parse(input)?;
    let (input, name) = context("system name", ws(identifier)).parse(input)?;
    let (input, params) = opt(delimited(
        ws_char('('),
        separated_list0(ws_char(','), system_param),
        context("')'", ws_char(')')),
    ))
    .parse(input)?;
    let (input, body) = delimited(
        ws_char('{'),
        many0(system_statement),
        context("'}'", ws_char('}')),
    )
    .parse(input)?;
    let span = SourceSpan::from_bounds(start, input);
    Ok((
        input,
        System {
            name,
            params: params.unwrap_or_default(),
            body,
            span,
        },
    ))
}

/// Parses one system parameter (`in|out|inout <type> <name>`).
fn system_param(input: Span<'_>) -> PResult<'_, Param> {
    let start = input;
    let (input, mode) = context("parameter mode", ws(param_mode)).parse(input)?;
    let (input, ty) = context("parameter type", ws(type_name)).parse(input)?;
    let (input, name) = context("parameter name", ws(identifier)).parse(input)?;
    let span = SourceSpan::from_bounds(start, input);
    Ok((
        input,
        Param {
            mode,
            name,
            ty,
            span,
        },
    ))
}

/// Parses one parameter mode keyword.
fn param_mode(input: Span<'_>) -> PResult<'_, ParamMode> {
    alt((
        map(tag("inout"), |_| ParamMode::InOut),
        map(tag("in"), |_| ParamMode::In),
        map(tag("out"), |_| ParamMode::Out),
    ))
    .parse(input)
}

/// Parses one statement and trailing semicolon.
pub(super) fn statement(input: Span<'_>) -> PResult<'_, Stmt> {
    let start = input;
    // A statement can be declaration, equality, or callable invocation.
    let (input, kind) = alt((
        map(decl_stmt, StmtKind::Decl),
        map(constraint_stmt, |(lhs, rhs)| StmtKind::ConstraintEq {
            lhs,
            rhs,
        }),
        map(call_stmt, StmtKind::Call),
    ))
    .parse(input)?;
    let (input, _) = context("';'", ws_char(';')).parse(input)?;
    let span = SourceSpan::from_bounds(start, input);
    Ok((input, Stmt { kind, span }))
}

/// Parses one system statement and trailing semicolon.
pub(super) fn system_statement(input: Span<'_>) -> PResult<'_, SystemStmt> {
    let start = input;
    let (input, kind) = alt((
        map(decl_stmt, SystemStmtKind::Decl),
        map(constraint_stmt, |(lhs, rhs)| SystemStmtKind::ConstraintEq {
            lhs,
            rhs,
        }),
        map(call_stmt, SystemStmtKind::Call),
    ))
    .parse(input)?;
    let (input, _) = context("';'", ws_char(';')).parse(input)?;
    let span = SourceSpan::from_bounds(start, input);
    Ok((input, SystemStmt { kind, span }))
}

/// Parses a declaration statement body.
pub(super) fn decl_stmt(input: Span<'_>) -> PResult<'_, Decl> {
    let start = input;
    // Grammar: `<type> <identifier> (= <expr>)?`
    let (input, ty) = context("type", ws(type_name)).parse(input)?;
    let (input, name) = context("identifier", ws(identifier)).parse(input)?;
    let (input, init) = opt(preceded(ws_char('='), expr)).parse(input)?;
    let span = SourceSpan::from_bounds(start, input);
    Ok((
        input,
        Decl {
            ty,
            name,
            init,
            span,
        },
    ))
}

/// Parses an equality statement body (`lhs == rhs`).
fn constraint_stmt(input: Span<'_>) -> PResult<'_, (Expr, Expr)> {
    // Grammar: `<expr> == <expr>`
    let (input, lhs) = context("left expression", expr).parse(input)?;
    let (input, _) = context("'=='", ws(tag("=="))).parse(input)?;
    let (input, rhs) = context("right expression", expr).parse(input)?;
    Ok((input, (lhs, rhs)))
}

/// Parses direct call syntax (`name(args...)`).
fn call_stmt(input: Span<'_>) -> PResult<'_, CallStmt> {
    let start = input;
    let (input, name) = context("function name", ws(identifier)).parse(input)?;
    let (input, args) = delimited(
        ws_char('('),
        separated_list0(ws_char(','), expr),
        context("')'", ws_char(')')),
    )
    .parse(input)?;
    let span = SourceSpan::from_bounds(start, input);
    Ok((input, CallStmt { name, args, span }))
}
