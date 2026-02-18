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

//! Expression lowering and compile-time default inference.

use super::*;

impl LowerContext {
    /// Recursively lowers an AST expression into a typed value expression.
    pub(super) fn lower_expr(&self, expr: &Expr) -> Result<ValueExp, CompileError> {
        match &expr.kind {
            ExprKind::Number(v) => Ok(ValueExp::Scalar(Exp::val(*v))),
            ExprKind::Ident(name) => {
                // Identifier lookup is scope-driven (params, locals, exported vars).
                let Some(value) = self.resolve_binding(name) else {
                    return Err(self.error_at(format!("Unknown identifier '{name}'"), &expr.span));
                };
                Ok(value)
            }
            ExprKind::VectorLit(items) => self.lower_vector_literal(items, &expr.span),
            ExprKind::UnaryNeg(inner) => {
                // Unary negation is supported for scalar and vectors.
                let value = self.lower_expr(inner)?;
                Ok(match value {
                    ValueExp::Scalar(e) => ValueExp::Scalar(Exp::neg(e)),
                    ValueExp::Vec2(v) => {
                        ValueExp::Vec2([Exp::neg(v[0].clone()), Exp::neg(v[1].clone())])
                    }
                    ValueExp::Vec3(v) => ValueExp::Vec3([
                        Exp::neg(v[0].clone()),
                        Exp::neg(v[1].clone()),
                        Exp::neg(v[2].clone()),
                    ]),
                })
            }
            ExprKind::Binary { op, left, right } => {
                // Binary operations are dispatched based on operand types.
                let left = self.lower_expr(left)?;
                let right = self.lower_expr(right)?;
                self.lower_binary(*op, left, right, &expr.span)
            }
            ExprKind::Call { name, args } => self.lower_call(name, args, &expr.span),
            ExprKind::Field { base, field } => {
                // Field access projects a vector component to scalar.
                let base = self.lower_expr(base)?;
                match (base, field) {
                    (ValueExp::Vec2(v), Field::X) => Ok(ValueExp::Scalar(v[0].clone())),
                    (ValueExp::Vec2(v), Field::Y) => Ok(ValueExp::Scalar(v[1].clone())),
                    (ValueExp::Vec2(_), Field::Z) => {
                        Err(self.error_at("Field '.z' is invalid for vec2d", &expr.span))
                    }
                    (ValueExp::Vec3(v), Field::X) => Ok(ValueExp::Scalar(v[0].clone())),
                    (ValueExp::Vec3(v), Field::Y) => Ok(ValueExp::Scalar(v[1].clone())),
                    (ValueExp::Vec3(v), Field::Z) => Ok(ValueExp::Scalar(v[2].clone())),
                    (ValueExp::Scalar(_), _) => {
                        Err(self.error_at("Field access requires vector value", &expr.span))
                    }
                }
            }
        }
    }

    pub(super) fn lower_vector_literal(
        &self,
        items: &[Expr],
        span: &SourceSpan,
    ) -> Result<ValueExp, CompileError> {
        // Keep literal dimensions explicit to avoid implicit widening/narrowing.
        if !(2..=3).contains(&items.len()) {
            return Err(self.error_at(
                format!(
                    "Vector literal must have 2 or 3 elements, found {}",
                    items.len()
                ),
                span,
            ));
        }

        let mut lowered = Vec::with_capacity(items.len());
        for item in items {
            let value = self.lower_expr(item)?;
            // Literal components must each be scalar.
            let Some(scalar) = value.scalar() else {
                return Err(self.error_at("Vector literal elements must be scalar", &item.span));
            };
            lowered.push(scalar);
        }

        Ok(match lowered.len() {
            2 => ValueExp::Vec2([lowered[0].clone(), lowered[1].clone()]),
            3 => ValueExp::Vec3([lowered[0].clone(), lowered[1].clone(), lowered[2].clone()]),
            _ => unreachable!("checked literal length"),
        })
    }

    /// Lowers binary expressions with type-directed dispatch.
    pub(super) fn lower_binary(
        &self,
        op: BinOp,
        left: ValueExp,
        right: ValueExp,
        span: &SourceSpan,
    ) -> Result<ValueExp, CompileError> {
        // Arithmetic legality depends on both operator and operand shapes.
        match op {
            BinOp::Add => self.binary_add_sub(true, left, right, span),
            BinOp::Sub => self.binary_add_sub(false, left, right, span),
            BinOp::Mul => self.binary_mul(left, right, span),
            BinOp::Div => self.binary_div(left, right, span),
        }
    }

    /// Lowers `+` and `-` for scalar/vector operands.
    fn binary_add_sub(
        &self,
        is_add: bool,
        left: ValueExp,
        right: ValueExp,
        span: &SourceSpan,
    ) -> Result<ValueExp, CompileError> {
        match (left, right) {
            (ValueExp::Scalar(l), ValueExp::Scalar(r)) => Ok(ValueExp::Scalar(if is_add {
                Exp::add(l, r)
            } else {
                Exp::sub(l, r)
            })),
            (ValueExp::Vec2(l), ValueExp::Vec2(r)) => Ok(ValueExp::Vec2([
                if is_add {
                    Exp::add(l[0].clone(), r[0].clone())
                } else {
                    Exp::sub(l[0].clone(), r[0].clone())
                },
                if is_add {
                    Exp::add(l[1].clone(), r[1].clone())
                } else {
                    Exp::sub(l[1].clone(), r[1].clone())
                },
            ])),
            (ValueExp::Vec3(l), ValueExp::Vec3(r)) => Ok(ValueExp::Vec3([
                if is_add {
                    Exp::add(l[0].clone(), r[0].clone())
                } else {
                    Exp::sub(l[0].clone(), r[0].clone())
                },
                if is_add {
                    Exp::add(l[1].clone(), r[1].clone())
                } else {
                    Exp::sub(l[1].clone(), r[1].clone())
                },
                if is_add {
                    Exp::add(l[2].clone(), r[2].clone())
                } else {
                    Exp::sub(l[2].clone(), r[2].clone())
                },
            ])),
            (l, r) => Err(self.error_at(
                format!(
                    "Operator '{}' is not defined for {} and {}",
                    if is_add { "+" } else { "-" },
                    l.type_name(),
                    r.type_name()
                ),
                span,
            )),
        }
    }

    /// Lowers `*` for scalar-scalar and scalar-vector operations.
    fn binary_mul(
        &self,
        left: ValueExp,
        right: ValueExp,
        span: &SourceSpan,
    ) -> Result<ValueExp, CompileError> {
        match (left, right) {
            (ValueExp::Scalar(l), ValueExp::Scalar(r)) => Ok(ValueExp::Scalar(Exp::mul(l, r))),
            // Allow scalar-vector scaling in either operand order.
            (ValueExp::Vec2(v), ValueExp::Scalar(s)) | (ValueExp::Scalar(s), ValueExp::Vec2(v)) => {
                Ok(ValueExp::Vec2([
                    Exp::mul(v[0].clone(), s.clone()),
                    Exp::mul(v[1].clone(), s),
                ]))
            }
            // Allow scalar-vector scaling in either operand order.
            (ValueExp::Vec3(v), ValueExp::Scalar(s)) | (ValueExp::Scalar(s), ValueExp::Vec3(v)) => {
                Ok(ValueExp::Vec3([
                    Exp::mul(v[0].clone(), s.clone()),
                    Exp::mul(v[1].clone(), s.clone()),
                    Exp::mul(v[2].clone(), s),
                ]))
            }
            (l, r) => Err(self.error_at(
                format!(
                    "Operator '*' is not defined for {} and {}",
                    l.type_name(),
                    r.type_name()
                ),
                span,
            )),
        }
    }

    /// Lowers `/` for scalar-scalar and vector-scalar operations.
    fn binary_div(
        &self,
        left: ValueExp,
        right: ValueExp,
        span: &SourceSpan,
    ) -> Result<ValueExp, CompileError> {
        match (left, right) {
            (ValueExp::Scalar(l), ValueExp::Scalar(r)) => Ok(ValueExp::Scalar(Exp::div(l, r))),
            // Vector division is only defined by scalar denominator.
            (ValueExp::Vec2(v), ValueExp::Scalar(s)) => Ok(ValueExp::Vec2([
                Exp::div(v[0].clone(), s.clone()),
                Exp::div(v[1].clone(), s),
            ])),
            // Vector division is only defined by scalar denominator.
            (ValueExp::Vec3(v), ValueExp::Scalar(s)) => Ok(ValueExp::Vec3([
                Exp::div(v[0].clone(), s.clone()),
                Exp::div(v[1].clone(), s.clone()),
                Exp::div(v[2].clone(), s),
            ])),
            (l, r) => Err(self.error_at(
                format!(
                    "Operator '/' is not defined for {} and {}",
                    l.type_name(),
                    r.type_name()
                ),
                span,
            )),
        }
    }

    /// Attempts to evaluate constant initializers and store solved defaults.
    pub(super) fn capture_defaults(
        &mut self,
        canonical_base: &str,
        ty: SymbolType,
        rhs: &ValueExp,
    ) {
        // Defaults are best-effort: only expressions resolvable from known defaults
        // are captured. Non-constant initializers are left as zero-seeded values.
        let values = match rhs {
            ValueExp::Scalar(e) => evaluate_exp(e, &self.defaults).map(|v| vec![v]),
            ValueExp::Vec2(v) => {
                let x = evaluate_exp(&v[0], &self.defaults);
                let y = evaluate_exp(&v[1], &self.defaults);
                match (x, y) {
                    (Some(x), Some(y)) => Some(vec![x, y]),
                    _ => None,
                }
            }
            ValueExp::Vec3(v) => {
                let x = evaluate_exp(&v[0], &self.defaults);
                let y = evaluate_exp(&v[1], &self.defaults);
                let z = evaluate_exp(&v[2], &self.defaults);
                match (x, y, z) {
                    (Some(x), Some(y), Some(z)) => Some(vec![x, y, z]),
                    _ => None,
                }
            }
        };

        if let Some(values) = values {
            // Write solved initializer values back into flattened default slots.
            for (component, value) in flattened_component_names(canonical_base, ty)
                .into_iter()
                .zip(values.into_iter())
            {
                self.defaults.insert(component, value);
            }
        }
    }
}
