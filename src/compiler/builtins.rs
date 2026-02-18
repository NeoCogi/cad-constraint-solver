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

//! Lowering for builtin expression functions.

use super::*;

impl LowerContext {
    /// Lowers builtin function calls.
    ///
    /// Supported builtins:
    /// - `dot2(vec2d, vec2d)`
    /// - `dot3(vec3d, vec3d)`
    /// - `length2(vec2d)`
    /// - `length3(vec3d)`
    /// - `sin/cos/ln/exp(scalar)`
    pub(super) fn lower_call(
        &self,
        name: &str,
        args: &[Expr],
        span: &SourceSpan,
    ) -> Result<ValueExp, CompileError> {
        let mut lowered = Vec::with_capacity(args.len());
        for arg in args {
            lowered.push(self.lower_expr(arg)?);
        }

        // Builtins are explicit scalar expansions because `constraint_solver::Exp`
        // does not include vector primitives.
        match name {
            "dot2" => {
                let [a, b] = expect_2_args(lowered, name, self, span)?;
                let (a, b) = match (a, b) {
                    (ValueExp::Vec2(a), ValueExp::Vec2(b)) => (a, b),
                    (l, r) => {
                        return Err(self.error_at(
                            format!(
                                "dot2 expects (vec2d, vec2d), got ({}, {})",
                                l.type_name(),
                                r.type_name()
                            ),
                            span,
                        ));
                    }
                };
                Ok(ValueExp::Scalar(dot2_exp(&a, &b)))
            }
            "dot3" => {
                let [a, b] = expect_2_args(lowered, name, self, span)?;
                let (a, b) = match (a, b) {
                    (ValueExp::Vec3(a), ValueExp::Vec3(b)) => (a, b),
                    (l, r) => {
                        return Err(self.error_at(
                            format!(
                                "dot3 expects (vec3d, vec3d), got ({}, {})",
                                l.type_name(),
                                r.type_name()
                            ),
                            span,
                        ));
                    }
                };
                Ok(ValueExp::Scalar(dot3_exp(&a, &b)))
            }
            "length2" => {
                let [a] = expect_1_arg(lowered, name, self, span)?;
                let a = match a {
                    ValueExp::Vec2(v) => v,
                    other => {
                        return Err(self.error_at(
                            format!("length2 expects vec2d, got {}", other.type_name()),
                            span,
                        ));
                    }
                };
                // length(v) = sqrt(dot(v,v)).
                let d = dot2_exp(&a, &a);
                Ok(ValueExp::Scalar(Exp::power(d, 0.5)))
            }
            "length3" => {
                let [a] = expect_1_arg(lowered, name, self, span)?;
                let a = match a {
                    ValueExp::Vec3(v) => v,
                    other => {
                        return Err(self.error_at(
                            format!("length3 expects vec3d, got {}", other.type_name()),
                            span,
                        ));
                    }
                };
                // length(v) = sqrt(dot(v,v)).
                let d = dot3_exp(&a, &a);
                Ok(ValueExp::Scalar(Exp::power(d, 0.5)))
            }
            "sin" | "cos" | "ln" | "exp" => {
                let [a] = expect_1_arg(lowered, name, self, span)?;
                let a = match a {
                    ValueExp::Scalar(v) => v,
                    other => {
                        return Err(self.error_at(
                            format!("{name} expects scalar, got {}", other.type_name()),
                            span,
                        ));
                    }
                };
                let out = match name {
                    "sin" => Exp::sin(a),
                    "cos" => Exp::cos(a),
                    "ln" => Exp::ln(a),
                    "exp" => Exp::exp(a),
                    _ => unreachable!(),
                };
                Ok(ValueExp::Scalar(out))
            }
            _ => Err(self.error_at(format!("Unknown function '{name}'"), span)),
        }
    }
}

/// Validates and extracts exactly one argument.
fn expect_1_arg(
    mut args: Vec<ValueExp>,
    name: &str,
    ctx: &LowerContext,
    span: &SourceSpan,
) -> Result<[ValueExp; 1], CompileError> {
    // Keep function arity checks centralized for consistent diagnostics.
    if args.len() != 1 {
        return Err(ctx.error_at(
            format!("{name} expects exactly 1 argument, found {}", args.len()),
            span,
        ));
    }
    Ok([args.remove(0)])
}

/// Validates and extracts exactly two arguments.
fn expect_2_args(
    mut args: Vec<ValueExp>,
    name: &str,
    ctx: &LowerContext,
    span: &SourceSpan,
) -> Result<[ValueExp; 2], CompileError> {
    // Keep function arity checks centralized for consistent diagnostics.
    if args.len() != 2 {
        return Err(ctx.error_at(
            format!("{name} expects exactly 2 arguments, found {}", args.len()),
            span,
        ));
    }
    Ok([args.remove(0), args.remove(0)])
}

/// Builds scalar `dot(vec2, vec2)` expansion.
fn dot2_exp(a: &[Exp; 2], b: &[Exp; 2]) -> Exp {
    Exp::add(
        Exp::mul(a[0].clone(), b[0].clone()),
        Exp::mul(a[1].clone(), b[1].clone()),
    )
}

/// Builds scalar `dot(vec3, vec3)` expansion.
fn dot3_exp(a: &[Exp; 3], b: &[Exp; 3]) -> Exp {
    Exp::add(
        Exp::add(
            Exp::mul(a[0].clone(), b[0].clone()),
            Exp::mul(a[1].clone(), b[1].clone()),
        ),
        Exp::mul(a[2].clone(), b[2].clone()),
    )
}
