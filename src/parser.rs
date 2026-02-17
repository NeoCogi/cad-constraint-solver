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
//! - top-level `constraint_fn` definitions
//! - top-level `system` definitions
//! - legacy top-level declarations/constraints
//!
//! Expressions support:
//! - numeric literals and identifiers
//! - vector literals (`[a, b]`, `[a, b, c]`)
//! - unary negation
//! - binary `+ - * /`
//! - function calls
//! - field access (`.x .y .z`)

use crate::ast::{
    BinOp, ConstraintFn, Decl, Expr, ExprKind, Field, Param, Program, SourceSpan, Span, Stmt,
    StmtKind, System, SystemStmt, SystemStmtKind, TypeName, UseStmt, Visibility,
};
use crate::diagnostics::CompileError;
use nom::Parser;
use nom::{
    IResult,
    branch::alt,
    bytes::complete::{tag, take_while, take_while1},
    character::complete::{char, multispace1, one_of},
    combinator::{all_consuming, map, map_res, opt, recognize, value},
    error::{VerboseError, VerboseErrorKind, context},
    multi::{many0, separated_list0, separated_list1},
    number::complete::recognize_float,
    sequence::{delimited, pair, preceded},
};

type PResult<'a, O> = IResult<Span<'a>, O, VerboseError<Span<'a>>>;

enum TopItem {
    ConstraintFn(ConstraintFn),
    System(System),
    LegacyStmt(Stmt),
}

/// Parses full DSL source into a spanned AST program.
pub fn parse_program(source: &str) -> Result<Program, CompileError> {
    let input = Span::new(source);
    // `all_consuming` ensures trailing garbage is treated as syntax error.
    let (_, items) = match all_consuming(delimited(ws0, many0(top_item), ws0))(input) {
        Ok(v) => v,
        Err(err) => return Err(parse_error_to_compile_error(err, source)),
    };

    let mut constraint_fns = Vec::new();
    let mut systems = Vec::new();
    let mut statements = Vec::new();

    for item in items {
        match item {
            TopItem::ConstraintFn(def) => constraint_fns.push(def),
            TopItem::System(system) => systems.push(system),
            TopItem::LegacyStmt(stmt) => statements.push(stmt),
        }
    }

    Ok(Program {
        constraint_fns,
        systems,
        statements,
    })
}

/// Converts a `nom` verbose error to crate-level compile diagnostics.
fn parse_error_to_compile_error(
    err: nom::Err<VerboseError<Span<'_>>>,
    source: &str,
) -> CompileError {
    match err {
        nom::Err::Incomplete(_) => CompileError::message_only("Incomplete input"),
        nom::Err::Error(e) | nom::Err::Failure(e) => {
            // Use the deepest recorded parser error as the diagnostic anchor.
            if let Some((span, kind)) = e.errors.last() {
                let span = SourceSpan::from_bounds(*span, *span);
                let detail = match kind {
                    VerboseErrorKind::Context(ctx) => format!("Syntax error: expected {ctx}"),
                    VerboseErrorKind::Char(c) => format!("Syntax error: expected '{c}'"),
                    VerboseErrorKind::Nom(kind) => format!("Syntax error near {kind:?}"),
                };
                CompileError::from_span(detail, source, &span)
            } else {
                CompileError::message_only("Syntax error")
            }
        }
    }
}

/// Parses one top-level item.
fn top_item(input: Span<'_>) -> PResult<'_, TopItem> {
    // Try definitions first to avoid treating their bodies as legacy statements.
    alt((
        map(constraint_fn_def, TopItem::ConstraintFn),
        map(system_def, TopItem::System),
        map(statement, TopItem::LegacyStmt),
    ))
    .parse(input)
}

/// Parses one legacy statement and trailing semicolon.
fn statement(input: Span<'_>) -> PResult<'_, Stmt> {
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
fn system_statement(input: Span<'_>) -> PResult<'_, SystemStmt> {
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
fn decl_stmt(input: Span<'_>) -> PResult<'_, Decl> {
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
fn use_stmt(input: Span<'_>) -> PResult<'_, UseStmt> {
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

/// Top-level expression parser.
fn expr(input: Span<'_>) -> PResult<'_, Expr> {
    parse_add_sub(input)
}

/// Parses left-associative `+`/`-`.
fn parse_add_sub(input: Span<'_>) -> PResult<'_, Expr> {
    let (mut input, mut left) = parse_mul_div(input)?;
    loop {
        let (next, op) = opt(alt((ws_char('+'), ws_char('-')))).parse(input)?;
        let Some(op_char) = op else {
            break;
        };

        // Left-associative fold: `a-b-c` becomes `(a-b)-c`.
        let (next, right) = parse_mul_div(next)?;
        let op = if op_char == '+' {
            BinOp::Add
        } else {
            BinOp::Sub
        };
        let span = left.span.merge(&right.span);
        left = Expr {
            kind: ExprKind::Binary {
                op,
                left: Box::new(left),
                right: Box::new(right),
            },
            span,
        };
        input = next;
    }
    Ok((input, left))
}

/// Parses left-associative `*`/`/`.
fn parse_mul_div(input: Span<'_>) -> PResult<'_, Expr> {
    let (mut input, mut left) = parse_unary(input)?;
    loop {
        let (next, op) = opt(alt((ws_char('*'), ws_char('/')))).parse(input)?;
        let Some(op_char) = op else {
            break;
        };

        // Left-associative fold: `a/b/c` becomes `(a/b)/c`.
        let (next, right) = parse_unary(next)?;
        let op = if op_char == '*' {
            BinOp::Mul
        } else {
            BinOp::Div
        };
        let span = left.span.merge(&right.span);
        left = Expr {
            kind: ExprKind::Binary {
                op,
                left: Box::new(left),
                right: Box::new(right),
            },
            span,
        };
        input = next;
    }
    Ok((input, left))
}

/// Parses unary operators.
fn parse_unary(input: Span<'_>) -> PResult<'_, Expr> {
    let start = input;
    // Unary operators are parsed recursively to support chains like `---x`.
    if let Ok((input, _)) = ws_char('-').parse(input) {
        let (input, inner) = parse_unary(input)?;
        let span = SourceSpan::from_bounds(start, input);
        return Ok((
            input,
            Expr {
                kind: ExprKind::UnaryNeg(Box::new(inner)),
                span,
            },
        ));
    }
    parse_postfix(input)
}

/// Parses postfix field access chains (`expr.x.y`).
fn parse_postfix(input: Span<'_>) -> PResult<'_, Expr> {
    let (mut input, mut expr) = parse_primary(input)?;
    loop {
        let (next, field) = opt(preceded(ws_char('.'), ws(one_of("xyz")))).parse(input)?;
        let Some(field_char) = field else {
            break;
        };

        // Repeatedly consume field suffixes (`p.x`, `v.y.z`, ...).
        let field = match field_char {
            'x' => Field::X,
            'y' => Field::Y,
            'z' => Field::Z,
            _ => unreachable!("field parser is constrained to xyz"),
        };
        let mut span = expr.span.clone();
        span.end = next.location_offset();
        expr = Expr {
            kind: ExprKind::Field {
                base: Box::new(expr),
                field,
            },
            span,
        };
        input = next;
    }
    Ok((input, expr))
}

/// Parses expression atoms.
fn parse_primary(input: Span<'_>) -> PResult<'_, Expr> {
    alt((
        parse_vector_literal,
        parse_parenthesized,
        parse_number,
        parse_ident_or_call,
    ))
    .parse(input)
}

/// Parses vector literal expressions.
fn parse_vector_literal(input: Span<'_>) -> PResult<'_, Expr> {
    let start = input;
    // Semantic checks (length 2/3) are handled by lowering phase.
    let (input, _) = ws_char('[').parse(input)?;
    let (input, elements) = separated_list1(ws_char(','), expr).parse(input)?;
    let (input, _) = context("']'", ws_char(']')).parse(input)?;
    let span = SourceSpan::from_bounds(start, input);
    Ok((
        input,
        Expr {
            kind: ExprKind::VectorLit(elements),
            span,
        },
    ))
}

/// Parses parenthesized expressions.
fn parse_parenthesized(input: Span<'_>) -> PResult<'_, Expr> {
    let start = input;
    let (input, inner) = delimited(ws_char('('), expr, ws_char(')')).parse(input)?;
    let mut inner = inner;
    // Preserve outer range for better diagnostics around parenthesized terms.
    inner.span = SourceSpan::from_bounds(start, input);
    Ok((input, inner))
}

/// Parses numeric literal expressions.
fn parse_number(input: Span<'_>) -> PResult<'_, Expr> {
    let start = input;
    let (input, n) = ws(map_res(recognize_float, |s: Span<'_>| {
        s.fragment().parse::<f64>()
    }))
    .parse(input)?;
    let span = SourceSpan::from_bounds(start, input);
    Ok((
        input,
        Expr {
            kind: ExprKind::Number(n),
            span,
        },
    ))
}

/// Parses either identifier or function call expression.
fn parse_ident_or_call(input: Span<'_>) -> PResult<'_, Expr> {
    let start = input;
    let (input, name) = ws(identifier).parse(input)?;
    let (input, args) = opt(delimited(
        ws_char('('),
        separated_list0(ws_char(','), expr),
        ws_char(')'),
    ))
    .parse(input)?;

    let span = SourceSpan::from_bounds(start, input);
    // A name followed by `(...)` is parsed as call, otherwise identifier.
    let kind = if let Some(args) = args {
        ExprKind::Call { name, args }
    } else {
        ExprKind::Ident(name)
    };

    Ok((input, Expr { kind, span }))
}

/// Parses declaration type tokens.
fn type_name(input: Span<'_>) -> PResult<'_, TypeName> {
    alt((
        map(tag("scalar"), |_| TypeName::Scalar),
        map(tag("vec2d"), |_| TypeName::Vec2),
        map(tag("vec2f"), |_| TypeName::Vec2),
        map(tag("vec3d"), |_| TypeName::Vec3),
        map(tag("vec3f"), |_| TypeName::Vec3),
    ))
    .parse(input)
}

/// Parses identifiers (`[A-Za-z_][A-Za-z0-9_]*`).
fn identifier(input: Span<'_>) -> PResult<'_, String> {
    map(
        recognize(pair(
            take_while1(is_ident_start),
            take_while(is_ident_continue),
        )),
        |s: Span<'_>| s.fragment().to_string(),
    )
    .parse(input)
}

/// Returns whether a char can start an identifier.
fn is_ident_start(c: char) -> bool {
    c == '_' || c.is_ascii_alphabetic()
}

/// Returns whether a char can continue an identifier.
fn is_ident_continue(c: char) -> bool {
    c == '_' || c.is_ascii_alphanumeric()
}

/// Skips zero-or-more whitespace/comments.
fn ws0(input: Span<'_>) -> PResult<'_, ()> {
    // Treat spaces/newlines and comments uniformly as trivia.
    value((), many0(alt((value((), multispace1), comment)))).parse(input)
}

/// Parses line comments (`// ...` and `# ...`).
fn comment(input: Span<'_>) -> PResult<'_, ()> {
    // Accept shell-style and C++-style single-line comments.
    value(
        (),
        alt((
            pair(tag("//"), opt(nom::character::complete::not_line_ending)),
            pair(tag("#"), opt(nom::character::complete::not_line_ending)),
        )),
    )
    .parse(input)
}

/// Wraps a parser with leading/trailing whitespace/comment skipping.
fn ws<'a, O, P>(mut parser: P) -> impl FnMut(Span<'a>) -> PResult<'a, O>
where
    P: FnMut(Span<'a>) -> PResult<'a, O>,
{
    // This helper keeps individual grammar rules free from manual trivia handling.
    move |input| delimited(ws0, &mut parser, ws0)(input)
}

/// Parses a specific character token with surrounding whitespace/comments.
fn ws_char<'a>(c: char) -> impl FnMut(Span<'a>) -> PResult<'a, char> {
    ws(char(c))
}
