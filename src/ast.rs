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

//! AST definitions for the DSL with precise source spans.
//!
//! The parser creates this AST first. A later lowering phase type-checks and
//! converts it to scalar `constraint_solver::Exp` equations.

use nom_locate::LocatedSpan;

/// Parser input span type carrying byte offsets and line/column info.
pub type Span<'a> = LocatedSpan<&'a str>;

/// Source range and anchor position for diagnostics.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SourceSpan {
    /// Start byte offset (inclusive).
    pub start: usize,
    /// End byte offset (exclusive).
    pub end: usize,
    /// 1-based line number.
    pub line: usize,
    /// 1-based UTF-8 column.
    pub column: usize,
}

impl SourceSpan {
    /// Creates a source span from parser start/end positions.
    pub fn from_bounds(start: Span<'_>, end: Span<'_>) -> Self {
        Self {
            start: start.location_offset(),
            end: end.location_offset(),
            line: start.location_line() as usize,
            column: start.get_utf8_column(),
        }
    }

    /// Returns span length in bytes.
    pub fn len(&self) -> usize {
        self.end.saturating_sub(self.start)
    }

    /// Returns a span that starts at `self` and ends at `other`.
    pub fn merge(&self, other: &Self) -> Self {
        Self {
            start: self.start,
            end: other.end,
            line: self.line,
            column: self.column,
        }
    }
}

/// User-facing declaration types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TypeName {
    /// Scalar (`f64`) value.
    Scalar,
    /// 2D vector (`vec2d` or `vec2f` tokens).
    Vec2,
    /// 3D vector (`vec3d` or `vec3f` tokens).
    Vec3,
}

/// Binary arithmetic operators.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BinOp {
    /// Addition (`+`).
    Add,
    /// Subtraction (`-`).
    Sub,
    /// Multiplication (`*`).
    Mul,
    /// Division (`/`).
    Div,
}

/// Swizzle fields for vector components.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Field {
    /// X component.
    X,
    /// Y component.
    Y,
    /// Z component.
    Z,
}

/// Expression node variants.
#[derive(Debug, Clone, PartialEq)]
pub enum ExprKind {
    /// Numeric literal.
    Number(f64),
    /// Identifier reference.
    Ident(String),
    /// Vector literal (`[a, b]` or `[a, b, c]`).
    VectorLit(Vec<Expr>),
    /// Unary negation.
    UnaryNeg(Box<Expr>),
    /// Binary operation.
    Binary {
        /// Operator kind.
        op: BinOp,
        /// Left operand.
        left: Box<Expr>,
        /// Right operand.
        right: Box<Expr>,
    },
    /// Function call.
    Call {
        /// Function name.
        name: String,
        /// Call arguments.
        args: Vec<Expr>,
    },
    /// Field access (`.x`, `.y`, `.z`).
    Field {
        /// Base expression.
        base: Box<Expr>,
        /// Requested field.
        field: Field,
    },
}

/// Spanned expression node.
#[derive(Debug, Clone, PartialEq)]
pub struct Expr {
    /// Expression payload.
    pub kind: ExprKind,
    /// Source location for diagnostics.
    pub span: SourceSpan,
}

/// Declaration statement payload.
#[derive(Debug, Clone, PartialEq)]
pub struct Decl {
    /// Declared type.
    pub ty: TypeName,
    /// Symbol name.
    pub name: String,
    /// Optional initializer expression.
    pub init: Option<Expr>,
    /// Source location for diagnostics.
    pub span: SourceSpan,
}

/// Statement variants.
#[derive(Debug, Clone, PartialEq)]
pub enum StmtKind {
    /// Declaration statement.
    Decl(Decl),
    /// Equality constraint statement (`constraint lhs == rhs;`).
    ConstraintEq { lhs: Expr, rhs: Expr },
    /// Reusable constraint function invocation (`use name(args...);`).
    Use(UseStmt),
}

/// Spanned statement node.
#[derive(Debug, Clone, PartialEq)]
pub struct Stmt {
    /// Statement payload.
    pub kind: StmtKind,
    /// Source location for diagnostics.
    pub span: SourceSpan,
}

/// Constraint function invocation.
#[derive(Debug, Clone, PartialEq)]
pub struct UseStmt {
    /// Function name.
    pub name: String,
    /// Call arguments.
    pub args: Vec<Expr>,
    /// Source location for diagnostics.
    pub span: SourceSpan,
}

/// Function parameter declaration.
#[derive(Debug, Clone, PartialEq)]
pub struct Param {
    /// Parameter name.
    pub name: String,
    /// Parameter type.
    pub ty: TypeName,
    /// Source location for diagnostics.
    pub span: SourceSpan,
}

/// User-defined reusable constraint function.
#[derive(Debug, Clone, PartialEq)]
pub struct ConstraintFn {
    /// Function name.
    pub name: String,
    /// Typed parameter list.
    pub params: Vec<Param>,
    /// Function body statements.
    pub body: Vec<Stmt>,
    /// Source location for diagnostics.
    pub span: SourceSpan,
}

/// Visibility mode for system declarations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Visibility {
    /// Exposed to public API setters/getters.
    Export,
    /// Internal variable; not directly mutable by callers.
    Local,
}

/// System-scoped statement variants.
#[derive(Debug, Clone, PartialEq)]
pub enum SystemStmtKind {
    /// System declaration with explicit visibility.
    Decl {
        /// Public vs local visibility.
        visibility: Visibility,
        /// Declaration payload.
        decl: Decl,
    },
    /// Equality constraint statement.
    ConstraintEq { lhs: Expr, rhs: Expr },
    /// Reusable constraint function invocation.
    Use(UseStmt),
}

/// Spanned system statement node.
#[derive(Debug, Clone, PartialEq)]
pub struct SystemStmt {
    /// Statement payload.
    pub kind: SystemStmtKind,
    /// Source location for diagnostics.
    pub span: SourceSpan,
}

/// Named system definition block.
#[derive(Debug, Clone, PartialEq)]
pub struct System {
    /// System name.
    pub name: String,
    /// System body statements.
    pub body: Vec<SystemStmt>,
    /// Source location for diagnostics.
    pub span: SourceSpan,
}

/// Full parsed program.
#[derive(Debug, Clone, PartialEq)]
pub struct Program {
    /// Top-level reusable constraint functions.
    pub constraint_fns: Vec<ConstraintFn>,
    /// Top-level named systems.
    pub systems: Vec<System>,
    /// Legacy top-level statements (when no explicit `system` is used).
    pub statements: Vec<Stmt>,
}
