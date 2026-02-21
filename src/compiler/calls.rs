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

//! Lowering for callable invocations and equality constraints.

use super::*;
use crate::ast::{ExprKind, ParamMode};

impl LowerContext {
    /// Instantiates and lowers a callable invocation (`name(args...);`).
    pub(super) fn lower_call_stmt(&mut self, call_stmt: &CallStmt) -> Result<(), CompileError> {
        if let Some(system_entry) = self.systems.get(&call_stmt.name).cloned() {
            return self.lower_system_call(call_stmt, system_entry);
        }
        Err(self.error_at(
            format!("Unknown callable '{}'", call_stmt.name),
            &call_stmt.span,
        ))
    }

    fn lower_system_call(
        &mut self,
        call_stmt: &CallStmt,
        system_entry: SystemEntry,
    ) -> Result<(), CompileError> {
        let called = system_entry.system;

        if self
            .call_stack
            .iter()
            .any(|frame| frame.function == called.name)
        {
            return Err(self.error_at(
                format!(
                    "Recursive callable invocation is not supported ('{}')",
                    called.name
                ),
                &call_stmt.span,
            ));
        }

        if call_stmt.args.len() != called.params.len() {
            return Err(self.error_at(
                format!(
                    "system '{}' expects {} arguments, found {}",
                    called.name,
                    called.params.len(),
                    call_stmt.args.len()
                ),
                &call_stmt.span,
            ));
        }

        let mut lowered_args = Vec::with_capacity(call_stmt.args.len());
        for arg in &call_stmt.args {
            lowered_args.push(self.lower_expr(arg)?);
        }

        let mut param_scope = HashMap::new();
        for (idx, ((param, arg_expr), arg_value)) in called
            .params
            .iter()
            .zip(call_stmt.args.iter())
            .zip(lowered_args.into_iter())
            .enumerate()
        {
            let expected = map_decl_type(param.ty);
            if !matches_symbol_type(&arg_value, expected) {
                return Err(self.error_at(
                    format!(
                        "Argument {} for system '{}' expects {}, got {}",
                        idx + 1,
                        called.name,
                        symbol_type_name(expected),
                        arg_value.type_name()
                    ),
                    &arg_expr.span,
                ));
            }
            if matches!(param.mode, ParamMode::Out | ParamMode::InOut)
                && !is_assignable_argument(arg_expr)
            {
                return Err(self.error_at(
                    format!(
                        "Argument {} for '{}' {} parameter '{}' must be a variable identifier",
                        idx + 1,
                        called.name,
                        if param.mode == ParamMode::Out {
                            "out"
                        } else {
                            "inout"
                        },
                        param.name
                    ),
                    &arg_expr.span,
                ));
            }
            if param_scope.contains_key(&param.name) {
                return Err(self.error_at(
                    format!(
                        "Duplicate parameter name '{}' in system '{}'",
                        param.name, called.name
                    ),
                    &param.span,
                ));
            }
            param_scope.insert(param.name.clone(), arg_value);
        }

        let doc = self.current_doc();
        let marker =
            CompileError::from_span_in_source("call", &doc.path, &doc.source, &call_stmt.span);
        self.call_stack.push(IssueTraceFrame {
            function: called.name.clone(),
            file: doc.path.clone(),
            line: call_stmt.span.line,
            column: call_stmt.span.column,
            snippet: marker.snippet,
            pointer: marker.pointer,
        });
        self.push_scope(param_scope);
        self.push_doc(system_entry.doc.clone());

        self.invocation_counter += 1;
        let namespace_prefix = format!("__sys_{}_{}_", called.name, self.invocation_counter);

        for stmt in &called.body {
            self.lower_called_system_stmt(stmt, &namespace_prefix)?;
        }

        self.pop_doc();
        self.pop_scope();
        let _ = self.call_stack.pop();
        Ok(())
    }

    /// Lowers one `lhs == rhs;` statement.
    pub(super) fn lower_constraint(&mut self, lhs: &Expr, rhs: &Expr) -> Result<(), CompileError> {
        // Both sides are type-lowered first, then expanded to scalar equations.
        let lhs = self.lower_expr(lhs)?;
        let rhs_value = self.lower_expr(rhs)?;
        self.push_equality(lhs, rhs_value, &rhs.span)
    }

    /// Converts typed equality into one or more scalar equations.
    pub(super) fn push_equality(
        &mut self,
        lhs: ValueExp,
        rhs: ValueExp,
        span: &SourceSpan,
    ) -> Result<(), CompileError> {
        match (lhs, rhs) {
            (ValueExp::Scalar(l), ValueExp::Scalar(r)) => {
                // Scalar equality: one residual equation.
                self.push_scalar_equation(Exp::sub(l, r), span, None);
                Ok(())
            }
            (ValueExp::Vec2(l), ValueExp::Vec2(r)) => {
                // Vector equality expands component-wise.
                self.push_scalar_equation(Exp::sub(l[0].clone(), r[0].clone()), span, Some("x"));
                self.push_scalar_equation(Exp::sub(l[1].clone(), r[1].clone()), span, Some("y"));
                Ok(())
            }
            (ValueExp::Vec3(l), ValueExp::Vec3(r)) => {
                // Vector equality expands component-wise.
                self.push_scalar_equation(Exp::sub(l[0].clone(), r[0].clone()), span, Some("x"));
                self.push_scalar_equation(Exp::sub(l[1].clone(), r[1].clone()), span, Some("y"));
                self.push_scalar_equation(Exp::sub(l[2].clone(), r[2].clone()), span, Some("z"));
                Ok(())
            }
            (l, r) => Err(self.error_at(
                format!(
                    "Type mismatch in equality: left is {}, right is {}",
                    l.type_name(),
                    r.type_name()
                ),
                span,
            )),
        }
    }
}

fn is_assignable_argument(expr: &Expr) -> bool {
    matches!(expr.kind, ExprKind::Ident(_))
}
