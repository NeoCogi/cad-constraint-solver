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

//! Compile-time diagnostics with source snippets and caret pointers.

use crate::ast::SourceSpan;
use std::fmt;

/// Rich compile error returned by parser/lowering stages.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CompileError {
    /// Human-readable error message.
    pub message: String,
    /// Source file/path label (`""` when unavailable).
    pub file: String,
    /// 1-based line number (`0` when unavailable).
    pub line: usize,
    /// 1-based column number (`0` when unavailable).
    pub column: usize,
    /// Source line snippet where the error occurred.
    pub snippet: String,
    /// Caret pointer aligned to `snippet`.
    pub pointer: String,
}

impl CompileError {
    /// Creates an error with no location information.
    pub fn message_only(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
            file: String::new(),
            line: 0,
            column: 0,
            snippet: String::new(),
            pointer: String::new(),
        }
    }

    /// Creates an error with source-file label but no line/column info.
    pub fn message_in_file(message: impl Into<String>, file: impl Into<String>) -> Self {
        Self {
            message: message.into(),
            file: file.into(),
            line: 0,
            column: 0,
            snippet: String::new(),
            pointer: String::new(),
        }
    }

    /// Creates a source-mapped diagnostic from a [`SourceSpan`].
    pub fn from_span(message: impl Into<String>, source: &str, span: &SourceSpan) -> Self {
        Self::from_span_in_source(message, "<inline>", source, span)
    }

    /// Creates a source-mapped diagnostic from a [`SourceSpan`] and source label.
    pub fn from_span_in_source(
        message: impl Into<String>,
        file: impl Into<String>,
        source: &str,
        span: &SourceSpan,
    ) -> Self {
        let message = message.into();
        let file = file.into();
        // Pull the exact source line where the parser/lowering reported the span.
        let snippet = source
            .lines()
            .nth(span.line.saturating_sub(1))
            .unwrap_or("")
            .to_string();
        // Compute a safe caret range even when spans extend past line boundaries.
        let line_len = snippet.chars().count();
        let pointer_column = span.column.saturating_sub(1).min(line_len);
        let requested_len = span.len().max(1);
        let max_len = line_len.saturating_sub(pointer_column).max(1);
        let pointer_len = requested_len.min(max_len);
        // Render a fixed-width caret marker under the highlighted fragment.
        let pointer = format!("{}{}", " ".repeat(pointer_column), "^".repeat(pointer_len));

        Self {
            message,
            file,
            line: span.line,
            column: span.column,
            snippet,
            pointer,
        }
    }
}

impl fmt::Display for CompileError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.line == 0 || self.column == 0 {
            if self.file.is_empty() {
                return write!(f, "{}", self.message);
            }
            return write!(f, "{} ({})", self.message, self.file);
        }

        let location = if self.file.is_empty() {
            format!("line {}, column {}", self.line, self.column)
        } else {
            format!("{}:{}:{}", self.file, self.line, self.column)
        };

        write!(
            f,
            "{}\n --> {}\n  |\n{:>3} | {}\n  | {}",
            self.message, location, self.line, self.snippet, self.pointer
        )
    }
}

impl std::error::Error for CompileError {}
