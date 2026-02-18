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

//! Parser trivia and lexical helpers.

use crate::ast::{Span, TypeName};
use nom::Parser;
use nom::{
    branch::alt,
    bytes::complete::{tag, take_while, take_while1},
    character::complete::{char, multispace1},
    combinator::{map, opt, recognize, value},
    error::context,
    multi::many0,
    sequence::pair,
};

use super::PResult;

/// Parses a simple double-quoted string literal.
///
/// Escape processing is intentionally minimal: import paths are consumed as
/// raw text between quotes.
pub(super) fn string_literal(input: Span<'_>) -> PResult<'_, String> {
    map(
        nom::sequence::delimited(
            char('"'),
            take_while(|c| c != '"' && c != '\n' && c != '\r'),
            context("closing quote", char('"')),
        ),
        |s: Span<'_>| s.fragment().to_string(),
    )
    .parse(input)
}

/// Parses declaration type tokens.
pub(super) fn type_name(input: Span<'_>) -> PResult<'_, TypeName> {
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
pub(super) fn identifier(input: Span<'_>) -> PResult<'_, String> {
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
pub(super) fn ws0(input: Span<'_>) -> PResult<'_, ()> {
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
pub(super) fn ws<'a, O, P>(mut parser: P) -> impl FnMut(Span<'a>) -> PResult<'a, O>
where
    P: FnMut(Span<'a>) -> PResult<'a, O>,
{
    // This helper keeps grammar rules free from manual trivia handling.
    move |input| nom::sequence::delimited(ws0, &mut parser, ws0)(input)
}

/// Parses a specific character token with surrounding whitespace/comments.
pub(super) fn ws_char<'a>(c: char) -> impl FnMut(Span<'a>) -> PResult<'a, char> {
    ws(char(c))
}
