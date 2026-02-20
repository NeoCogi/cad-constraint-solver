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

//! Lowering context and core state management.

use super::*;

/// Lowering context for one compilation unit.
///
/// Holds system registry, scope stack, emitted equations, and inferred defaults.
pub(super) struct LowerContext {
    pub(super) systems: HashMap<String, SystemEntry>,
    pub(super) scopes: Vec<HashMap<String, ValueExp>>,
    pub(super) public_symbols: HashMap<String, SymbolType>,
    pub(super) equations: Vec<Exp>,
    pub(super) equation_origins: Vec<EquationOrigin>,
    pub(super) defaults: HashMap<String, f64>,
    pub(super) flattened_names: Vec<String>,
    pub(super) seed_flattened_names: Vec<String>,
    pub(super) public_flattened_names: Vec<String>,
    pub(super) unknown_selectable_flattened_names: Vec<String>,
    pub(super) invocation_counter: usize,
    pub(super) call_stack: Vec<IssueTraceFrame>,
    pub(super) current_system_name: Option<String>,
    pub(super) doc_stack: Vec<Rc<SourceDocument>>,
}

impl LowerContext {
    /// Creates a fresh lowering context bound to source text.
    pub(super) fn new(
        initial_doc: Rc<SourceDocument>,
        systems: HashMap<String, SystemEntry>,
    ) -> Self {
        Self {
            systems,
            scopes: Vec::new(),
            public_symbols: HashMap::new(),
            equations: Vec::new(),
            equation_origins: Vec::new(),
            defaults: HashMap::new(),
            flattened_names: Vec::new(),
            seed_flattened_names: Vec::new(),
            public_flattened_names: Vec::new(),
            unknown_selectable_flattened_names: Vec::new(),
            invocation_counter: 0,
            call_stack: Vec::new(),
            current_system_name: None,
            doc_stack: vec![initial_doc],
        }
    }

    pub(super) fn current_doc(&self) -> &SourceDocument {
        self.doc_stack
            .last()
            .expect("lowering always executes with an active source document")
            .as_ref()
    }

    pub(super) fn push_doc(&mut self, doc: Rc<SourceDocument>) {
        self.doc_stack.push(doc);
    }

    pub(super) fn pop_doc(&mut self) {
        let _ = self.doc_stack.pop();
    }

    /// Finalizes lowering and produces a [`Model`].
    pub(super) fn build(self) -> Model {
        let mut solver_vars = BTreeSet::new();
        // Track only variables that survive lowering into final equations.
        for equation in &self.equations {
            collect_var_names(equation, &mut solver_vars);
        }

        let public_set: BTreeSet<String> = self.public_flattened_names.iter().cloned().collect();
        let unknown_selectable_set: BTreeSet<String> = self
            .unknown_selectable_flattened_names
            .iter()
            .cloned()
            .collect();
        let mut public_solver = Vec::new();
        let mut hidden_solver = Vec::new();
        for name in &solver_vars {
            if unknown_selectable_set.contains(name) {
                public_solver.push(name.clone());
            } else if !public_set.contains(name) {
                hidden_solver.push(name.clone());
            }
        }

        Model::from_lowered_parts(
            self.equations,
            self.equation_origins,
            self.public_symbols,
            self.defaults,
            self.flattened_names,
            solver_vars.into_iter().collect(),
            self.seed_flattened_names,
            self.public_flattened_names,
            public_solver,
            hidden_solver,
        )
    }

    /// Creates a source-mapped compile error.
    pub(super) fn error_at(&self, message: impl Into<String>, span: &SourceSpan) -> CompileError {
        let doc = self.current_doc();
        CompileError::from_span_in_source(message, &doc.path, &doc.source, span)
    }

    pub(super) fn current_context_description(&self, component: Option<&str>) -> String {
        let mut segments = Vec::new();
        if let Some(system) = &self.current_system_name {
            segments.push(format!("system {system}"));
        }
        if !self.call_stack.is_empty() {
            let call_chain = self
                .call_stack
                .iter()
                .map(|frame| {
                    format!(
                        "{}@{}:{}:{}",
                        frame.function, frame.file, frame.line, frame.column
                    )
                })
                .collect::<Vec<_>>()
                .join(" -> ");
            segments.push(format!("call {call_chain}"));
        }

        let mut description = if segments.is_empty() {
            "constraint".to_string()
        } else {
            format!("constraint [{}]", segments.join(" | "))
        };
        if let Some(component) = component {
            description.push_str(&format!(" component {component}"));
        }
        description
    }

    pub(super) fn equation_origin(
        &self,
        span: &SourceSpan,
        component: Option<&str>,
    ) -> EquationOrigin {
        let doc = self.current_doc();
        let marker = CompileError::from_span_in_source("constraint", &doc.path, &doc.source, span);
        EquationOrigin {
            description: self.current_context_description(component),
            file: doc.path.clone(),
            line: span.line,
            column: span.column,
            snippet: marker.snippet,
            pointer: marker.pointer,
            traceback: self.call_stack.clone(),
        }
    }

    pub(super) fn push_scalar_equation(
        &mut self,
        equation: Exp,
        span: &SourceSpan,
        component: Option<&str>,
    ) {
        self.equations.push(equation);
        self.equation_origins
            .push(self.equation_origin(span, component));
    }

    pub(super) fn push_scope(&mut self, scope: HashMap<String, ValueExp>) {
        self.scopes.push(scope);
    }

    pub(super) fn pop_scope(&mut self) {
        let _ = self.scopes.pop();
    }

    pub(super) fn current_scope(&self) -> &HashMap<String, ValueExp> {
        self.scopes
            .last()
            .expect("lowering always executes within at least one scope")
    }

    pub(super) fn current_scope_mut(&mut self) -> &mut HashMap<String, ValueExp> {
        self.scopes
            .last_mut()
            .expect("lowering always executes within at least one scope")
    }

    pub(super) fn resolve_binding(&self, name: &str) -> Option<ValueExp> {
        for scope in self.scopes.iter().rev() {
            if let Some(value) = scope.get(name) {
                return Some(value.clone());
            }
        }
        None
    }
}
