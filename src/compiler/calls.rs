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

//! Lowering for `use` calls and equality constraints.

use super::*;

impl LowerContext {
    /// Instantiates and lowers a reusable `use constraint_fn(...)` call.
    pub(super) fn lower_use(&mut self, use_stmt: &UseStmt) -> Result<(), CompileError> {
        let Some(function_entry) = self.functions.get(&use_stmt.name).cloned() else {
            return Err(self.error_at(
                format!("Unknown constraint function '{}'", use_stmt.name),
                &use_stmt.span,
            ));
        };
        let function = function_entry.function;

        if self
            .call_stack
            .iter()
            .any(|frame| frame.function == function.name)
        {
            return Err(self.error_at(
                format!(
                    "Recursive constraint_fn invocation is not supported ('{}')",
                    function.name
                ),
                &use_stmt.span,
            ));
        }

        if use_stmt.args.len() != function.params.len() {
            return Err(self.error_at(
                format!(
                    "constraint_fn '{}' expects {} arguments, found {}",
                    function.name,
                    function.params.len(),
                    use_stmt.args.len()
                ),
                &use_stmt.span,
            ));
        }

        // Evaluate call arguments in caller scope first.
        let mut lowered_args = Vec::with_capacity(use_stmt.args.len());
        for arg in &use_stmt.args {
            lowered_args.push(self.lower_expr(arg)?);
        }

        // Build invocation-local scope: params are aliases to lowered args.
        let mut param_scope = HashMap::new();
        for (idx, (param, arg)) in function
            .params
            .iter()
            .zip(lowered_args.into_iter())
            .enumerate()
        {
            let expected = map_decl_type(param.ty);
            if !matches_symbol_type(&arg, expected) {
                let arg_span = &use_stmt.args[idx].span;
                return Err(self.error_at(
                    format!(
                        "Argument {} for '{}' expects {}, got {}",
                        idx + 1,
                        function.name,
                        symbol_type_name(expected),
                        arg.type_name()
                    ),
                    arg_span,
                ));
            }
            if param_scope.contains_key(&param.name) {
                return Err(self.error_at(
                    format!(
                        "Duplicate parameter name '{}' in constraint_fn '{}'",
                        param.name, function.name
                    ),
                    &param.span,
                ));
            }
            param_scope.insert(param.name.clone(), arg);
        }

        let doc = self.current_doc();
        let marker =
            CompileError::from_span_in_source("use", &doc.path, &doc.source, &use_stmt.span);
        self.call_stack.push(IssueTraceFrame {
            function: function.name.clone(),
            file: doc.path.clone(),
            line: use_stmt.span.line,
            column: use_stmt.span.column,
            snippet: marker.snippet,
            pointer: marker.pointer,
        });
        self.push_scope(param_scope);
        self.push_doc(function_entry.doc.clone());

        self.invocation_counter += 1;
        let namespace_prefix = format!("__fn_{}_{}_", function.name, self.invocation_counter);

        for stmt in &function.body {
            self.lower_function_stmt(stmt, &namespace_prefix)?;
        }

        self.pop_doc();
        self.pop_scope();
        let _ = self.call_stack.pop();
        Ok(())
    }

    /// Lowers one `constraint lhs == rhs;` statement.
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
