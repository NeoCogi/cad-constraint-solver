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
    ConstraintFn, Decl, Expr, Import, Param, SourceSpan, Span, Stmt, StmtKind, System, SystemStmt,
    SystemStmtKind, UseStmt, Visibility,
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
    ConstraintFn(ConstraintFn),
    System(System),
    LegacyStmt(Stmt),
}

/// Parses one top-level item.
pub(super) fn top_item(input: Span<'_>) -> PResult<'_, TopItem> {
    // Try definitions first to avoid treating their bodies as legacy statements.
    alt((
        map(import_stmt, TopItem::Import),
        map(constraint_fn_def, TopItem::ConstraintFn),
        map(system_def, TopItem::System),
        map(statement, TopItem::LegacyStmt),
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

/// Parses a top-level `constraint_fn` definition.
fn constraint_fn_def(input: Span<'_>) -> PResult<'_, ConstraintFn> {
    let start = input;
    let (input, _) = ws(context("'constraint_fn'", tag("constraint_fn"))).parse(input)?;
    let (input, name) = context("function name", ws(identifier)).parse(input)?;
    let (input, params) = delimited(
        ws_char('('),
        separated_list0(ws_char(','), param),
        context("')'", ws_char(')')),
    )
    .parse(input)?;
    let (input, body) =
        delimited(ws_char('{'), many0(statement), context("'}'", ws_char('}'))).parse(input)?;
    let span = SourceSpan::from_bounds(start, input);
    Ok((
        input,
        ConstraintFn {
            name,
            params,
            body,
            span,
        },
    ))
}

/// Parses one function parameter (`name: type`).
fn param(input: Span<'_>) -> PResult<'_, Param> {
    let start = input;
    let (input, name) = context("parameter name", ws(identifier)).parse(input)?;
    let (input, _) = context("':'", ws_char(':')).parse(input)?;
    let (input, ty) = context("parameter type", ws(type_name)).parse(input)?;
    let span = SourceSpan::from_bounds(start, input);
    Ok((input, Param { name, ty, span }))
}

/// Parses a top-level `system` definition.
fn system_def(input: Span<'_>) -> PResult<'_, System> {
    let start = input;
    let (input, _) = ws(context("'system'", tag("system"))).parse(input)?;
    let (input, name) = context("system name", ws(identifier)).parse(input)?;
    let (input, body) = delimited(
        ws_char('{'),
        many0(system_statement),
        context("'}'", ws_char('}')),
    )
    .parse(input)?;
    let span = SourceSpan::from_bounds(start, input);
    Ok((input, System { name, body, span }))
}

/// Parses one legacy statement and trailing semicolon.
pub(super) fn statement(input: Span<'_>) -> PResult<'_, Stmt> {
    let start = input;
    // A statement can be declaration, constraint, or `use`.
    let (input, kind) = alt((
        map(decl_stmt, StmtKind::Decl),
        map(constraint_stmt, |(lhs, rhs)| StmtKind::ConstraintEq {
            lhs,
            rhs,
        }),
        map(use_stmt, StmtKind::Use),
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
        map(export_decl_stmt, |decl| SystemStmtKind::Decl {
            visibility: Visibility::Export,
            decl,
        }),
        map(local_decl_stmt, |decl| SystemStmtKind::Decl {
            visibility: Visibility::Local,
            decl,
        }),
        // Bare declarations inside a system default to local visibility.
        map(decl_stmt, |decl| SystemStmtKind::Decl {
            visibility: Visibility::Local,
            decl,
        }),
        map(constraint_stmt, |(lhs, rhs)| SystemStmtKind::ConstraintEq {
            lhs,
            rhs,
        }),
        map(use_stmt, SystemStmtKind::Use),
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

/// Parses `export <decl>`.
fn export_decl_stmt(input: Span<'_>) -> PResult<'_, Decl> {
    let (input, _) = ws(context("'export'", tag("export"))).parse(input)?;
    decl_stmt(input)
}

/// Parses `local <decl>`.
fn local_decl_stmt(input: Span<'_>) -> PResult<'_, Decl> {
    let (input, _) = ws(context("'local'", tag("local"))).parse(input)?;
    decl_stmt(input)
}

/// Parses a constraint statement body (`constraint lhs == rhs`).
fn constraint_stmt(input: Span<'_>) -> PResult<'_, (Expr, Expr)> {
    // Grammar: `constraint <expr> == <expr>`
    let (input, _) = ws(context("constraint keyword", tag("constraint"))).parse(input)?;
    let (input, lhs) = context("left expression", expr).parse(input)?;
    let (input, _) = context("'=='", ws(tag("=="))).parse(input)?;
    let (input, rhs) = context("right expression", expr).parse(input)?;
    Ok((input, (lhs, rhs)))
}

/// Parses `use <name>(args...)`.
pub(super) fn use_stmt(input: Span<'_>) -> PResult<'_, UseStmt> {
    let start = input;
    let (input, _) = ws(context("'use'", tag("use"))).parse(input)?;
    let (input, name) = context("function name", ws(identifier)).parse(input)?;
    let (input, args) = delimited(
        ws_char('('),
        separated_list0(ws_char(','), expr),
        context("')'", ws_char(')')),
    )
    .parse(input)?;
    let span = SourceSpan::from_bounds(start, input);
    Ok((input, UseStmt { name, args, span }))
}
