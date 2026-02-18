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

//! Expression parser.

use crate::ast::{BinOp, Expr, ExprKind, Field, SourceSpan, Span};
use nom::Parser;
use nom::{
    branch::alt,
    combinator::{map_res, opt},
    error::context,
    multi::{separated_list0, separated_list1},
    number::complete::recognize_float,
    sequence::{delimited, preceded},
};

use super::PResult;
use super::utils::{identifier, ws, ws_char};

/// Top-level expression parser.
pub(super) fn expr(input: Span<'_>) -> PResult<'_, Expr> {
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
        let (next, field) = opt(preceded(
            ws_char('.'),
            ws(nom::character::complete::one_of("xyz")),
        ))
        .parse(input)?;
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
