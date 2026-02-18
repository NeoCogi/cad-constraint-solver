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

//! Declaration and statement lowering.

use super::*;

impl LowerContext {
    /// Registers one declared variable in current scope and metadata stores.
    pub(super) fn register_declaration(
        &mut self,
        binding_name: &str,
        canonical_base: &str,
        ty: SymbolType,
        expose_name: Option<&str>,
        span: &SourceSpan,
    ) -> Result<(), CompileError> {
        if self.current_scope().contains_key(binding_name) {
            return Err(self.error_at(
                format!("Duplicate declaration for '{binding_name}' in the same scope"),
                span,
            ));
        }

        self.current_scope_mut()
            .insert(binding_name.to_string(), variable_value(canonical_base, ty));

        // Every declared symbol gets default storage for all flattened components.
        for flat_name in flattened_component_names(canonical_base, ty) {
            self.defaults.entry(flat_name.clone()).or_insert(0.0);
            self.flattened_names.push(flat_name);
        }

        if let Some(public_name) = expose_name {
            if self.public_symbols.contains_key(public_name) {
                return Err(self.error_at(
                    format!("Duplicate exported declaration for '{public_name}'"),
                    span,
                ));
            }
            self.public_symbols.insert(public_name.to_string(), ty);
            self.public_flattened_names
                .extend(flattened_component_names(canonical_base, ty));
        }

        Ok(())
    }

    /// Lowers one declaration and optional initializer.
    pub(super) fn lower_decl(
        &mut self,
        decl: &Decl,
        canonical_base: &str,
        expose_name: Option<&str>,
    ) -> Result<(), CompileError> {
        let ty = map_decl_type(decl.ty);
        self.register_declaration(&decl.name, canonical_base, ty, expose_name, &decl.span)?;

        if let Some(init) = &decl.init {
            // Treat declaration initializers as implicit equality constraints.
            let lhs = self
                .resolve_binding(&decl.name)
                .expect("declaration binding inserted above");
            let rhs = self.lower_expr(init)?;
            self.push_equality(lhs, rhs.clone(), &init.span)?;
            // If initializer is constant-foldable, store it as a better default seed.
            self.capture_defaults(canonical_base, ty, &rhs);
        }

        Ok(())
    }

    /// Lowers one statement from a `system` block.
    pub(super) fn lower_system_stmt(&mut self, stmt: &SystemStmt) -> Result<(), CompileError> {
        match &stmt.kind {
            SystemStmtKind::Decl { visibility, decl } => match visibility {
                Visibility::Export => self.lower_decl(decl, &decl.name, Some(&decl.name)),
                Visibility::Local => self.lower_decl(decl, &decl.name, None),
            },
            SystemStmtKind::ConstraintEq { lhs, rhs } => self.lower_constraint(lhs, rhs),
            SystemStmtKind::Use(call) => self.lower_use(call),
        }
    }

    /// Lowers one legacy top-level statement.
    pub(super) fn lower_legacy_stmt(&mut self, stmt: &Stmt) -> Result<(), CompileError> {
        match &stmt.kind {
            StmtKind::Decl(decl) => self.lower_decl(decl, &decl.name, Some(&decl.name)),
            StmtKind::ConstraintEq { lhs, rhs } => self.lower_constraint(lhs, rhs),
            StmtKind::Use(call) => self.lower_use(call),
        }
    }

    /// Lowers one statement from a `constraint_fn` body.
    pub(super) fn lower_function_stmt(
        &mut self,
        stmt: &Stmt,
        namespace_prefix: &str,
    ) -> Result<(), CompileError> {
        match &stmt.kind {
            StmtKind::Decl(decl) => {
                // Function-local declarations are namespaced per invocation to avoid
                // collisions between multiple `use` sites.
                let canonical_base = format!("{namespace_prefix}{}", decl.name);
                self.lower_decl(decl, &canonical_base, None)
            }
            StmtKind::ConstraintEq { lhs, rhs } => self.lower_constraint(lhs, rhs),
            StmtKind::Use(call) => self.lower_use(call),
        }
    }
}
