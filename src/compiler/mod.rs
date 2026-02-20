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

//! DSL lowering from parsed AST into scalar solver equations.

mod builtins;
mod calls;
mod context;
mod decls;
mod expr;

use crate::CompileError;
use crate::ast::{
    BinOp, CallStmt, Decl, Expr, ExprKind, Field, Param, ParamMode, Program, SourceSpan, Stmt,
    StmtKind, System, SystemStmt, SystemStmtKind, TypeName,
};
use crate::model::{
    EquationOrigin, IssueTraceFrame, Model, SymbolType, collect_var_names, evaluate_exp,
};
use crate::project::{ModuleUnit, SourceDocument};
use constraint_solver::Exp;
use std::collections::{BTreeSet, HashMap};
use std::rc::Rc;

use self::context::LowerContext;

/// Internal typed expression used during lowering.
#[derive(Debug, Clone)]
pub(super) enum ValueExp {
    Scalar(Exp),
    Vec2([Exp; 2]),
    Vec3([Exp; 3]),
}

#[derive(Debug, Clone)]
pub(super) struct SystemEntry {
    pub(super) system: System,
    pub(super) doc: Rc<SourceDocument>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum PublicRole {
    TopLevel,
    In,
    Out,
    InOut,
}

impl PublicRole {
    pub(super) fn allows_seed(self) -> bool {
        matches!(
            self,
            PublicRole::TopLevel | PublicRole::In | PublicRole::InOut
        )
    }

    pub(super) fn allows_unknown_selection(self) -> bool {
        matches!(
            self,
            PublicRole::TopLevel | PublicRole::Out | PublicRole::InOut
        )
    }
}

impl ValueExp {
    /// Human-readable type label used in diagnostics.
    fn type_name(&self) -> &'static str {
        match self {
            ValueExp::Scalar(_) => "scalar",
            ValueExp::Vec2(_) => "vec2d",
            ValueExp::Vec3(_) => "vec3d",
        }
    }

    /// Extracts scalar expression when this value is scalar.
    fn scalar(self) -> Option<Exp> {
        match self {
            ValueExp::Scalar(exp) => Some(exp),
            _ => None,
        }
    }
}

/// Collects named systems across modules and validates duplicate names.
fn collect_systems(modules: &[ModuleUnit]) -> Result<Vec<SystemEntry>, CompileError> {
    let mut systems: Vec<SystemEntry> = Vec::new();
    let mut seen = HashMap::<String, Rc<SourceDocument>>::new();
    for module in modules {
        for system in &module.program.systems {
            if seen.contains_key(&system.name) {
                return Err(CompileError::from_span_in_source(
                    format!("Duplicate system '{}'", system.name),
                    &module.doc.path,
                    &module.doc.source,
                    &system.span,
                ));
            }
            seen.insert(system.name.clone(), module.doc.clone());
            systems.push(SystemEntry {
                system: system.clone(),
                doc: module.doc.clone(),
            });
        }
    }
    Ok(systems)
}

/// Selects the system that should be compiled.
fn pick_system(
    modules: &[ModuleUnit],
    systems: &[SystemEntry],
    selected_system: Option<&str>,
) -> Result<Option<SystemEntry>, CompileError> {
    if systems.is_empty() {
        if let Some(name) = selected_system {
            return Err(CompileError::message_only(format!(
                "Requested system '{name}' was not found"
            )));
        }
        return Ok(None);
    }

    if modules
        .iter()
        .any(|module| !module.program.statements.is_empty())
    {
        let first = &systems[0];
        return Err(CompileError::from_span_in_source(
            "Top-level statements cannot be mixed with named system definitions",
            &first.doc.path,
            &first.doc.source,
            &first.system.span,
        ));
    }

    if let Some(requested) = selected_system {
        let system = systems
            .iter()
            .cloned()
            .find(|entry| entry.system.name == requested)
            .ok_or_else(|| {
                CompileError::message_only(format!("Requested system '{requested}' was not found"))
            })?;
        return Ok(Some(system));
    }

    if systems.len() > 1 {
        let second = &systems[1];
        return Err(CompileError::from_span_in_source(
            "Multiple systems found; use compile_dsl_system(...) to select one",
            &second.doc.path,
            &second.doc.source,
            &second.system.span,
        ));
    }

    Ok(systems.first().cloned())
}

/// Lowers loaded modules into a solve-ready model.
pub(crate) fn lower_modules(
    modules: &[ModuleUnit],
    selected_system: Option<&str>,
) -> Result<Model, CompileError> {
    let systems = collect_systems(modules)?;
    let system_map: HashMap<String, SystemEntry> = systems
        .iter()
        .cloned()
        .map(|entry| (entry.system.name.clone(), entry))
        .collect();
    let initial_doc = modules
        .first()
        .map(|module| module.doc.clone())
        .unwrap_or_else(|| {
            Rc::new(SourceDocument {
                path: "<inline>".to_string(),
                source: String::new(),
            })
        });
    let mut ctx = LowerContext::new(initial_doc, system_map);
    ctx.push_scope(HashMap::new());

    if let Some(system) = pick_system(modules, &systems, selected_system)? {
        ctx.current_system_name = Some(system.system.name.clone());
        ctx.push_doc(system.doc.clone());
        ctx.lower_system_params(&system.system.params)?;
        // Process system statements in source order.
        for stmt in &system.system.body {
            ctx.lower_system_stmt(stmt)?;
        }
        ctx.pop_doc();
        ctx.current_system_name = None;
    } else {
        let has_top_level = modules
            .iter()
            .any(|module| !module.program.statements.is_empty());
        if !has_top_level {
            return Err(CompileError::message_only(
                "No system or top-level statements to compile",
            ));
        }
        // Top-level mode: declarations are considered public.
        for module in modules {
            if module.program.statements.is_empty() {
                continue;
            }
            ctx.push_doc(module.doc.clone());
            for stmt in &module.program.statements {
                ctx.lower_top_level_stmt(stmt)?;
            }
            ctx.pop_doc();
        }
    }

    ctx.pop_scope();
    Ok(ctx.build())
}

/// Lowers a parsed program into a solve-ready model.
pub(crate) fn lower_program(
    source_name: &str,
    source: &str,
    program: &Program,
    selected_system: Option<&str>,
) -> Result<Model, CompileError> {
    let module = ModuleUnit {
        doc: Rc::new(SourceDocument {
            path: source_name.to_string(),
            source: source.to_string(),
        }),
        program: program.clone(),
    };
    lower_modules(&[module], selected_system)
}

/// Maps parser type token to public symbol type.
pub(super) fn map_decl_type(ty: TypeName) -> SymbolType {
    match ty {
        TypeName::Scalar => SymbolType::Scalar,
        TypeName::Vec2 => SymbolType::Vec2d,
        TypeName::Vec3 => SymbolType::Vec3d,
    }
}

/// Returns human-readable symbol type name.
pub(super) fn symbol_type_name(ty: SymbolType) -> &'static str {
    match ty {
        SymbolType::Scalar => "scalar",
        SymbolType::Vec2d => "vec2d",
        SymbolType::Vec3d => "vec3d",
    }
}

/// Returns whether a lowered value matches expected symbol type.
pub(super) fn matches_symbol_type(value: &ValueExp, expected: SymbolType) -> bool {
    matches!(
        (value, expected),
        (ValueExp::Scalar(_), SymbolType::Scalar)
            | (ValueExp::Vec2(_), SymbolType::Vec2d)
            | (ValueExp::Vec3(_), SymbolType::Vec3d)
    )
}

/// Returns flattened solver component names for a base symbol.
pub(super) fn flattened_component_names(name: &str, ty: SymbolType) -> Vec<String> {
    match ty {
        SymbolType::Scalar => vec![name.to_string()],
        SymbolType::Vec2d => vec![format!("{name}.x"), format!("{name}.y")],
        SymbolType::Vec3d => vec![
            format!("{name}.x"),
            format!("{name}.y"),
            format!("{name}.z"),
        ],
    }
}

/// Converts a symbol to its typed value expression.
pub(super) fn variable_value(name: &str, ty: SymbolType) -> ValueExp {
    match ty {
        SymbolType::Scalar => ValueExp::Scalar(Exp::var(name)),
        SymbolType::Vec2d => {
            ValueExp::Vec2([Exp::var(format!("{name}.x")), Exp::var(format!("{name}.y"))])
        }
        SymbolType::Vec3d => ValueExp::Vec3([
            Exp::var(format!("{name}.x")),
            Exp::var(format!("{name}.y")),
            Exp::var(format!("{name}.z")),
        ]),
    }
}
