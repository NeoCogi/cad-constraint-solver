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

//! Project source loading and import graph traversal.

use crate::CompileError;
use crate::ast::Program;
use crate::parser::parse_program_in_source;
use std::collections::HashMap;
use std::rc::Rc;

use super::paths::resolve_import_path;

/// One DSL source unit in a multi-file project compile.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DslSource {
    /// Logical source path used by `import "..."` resolution and diagnostics.
    pub path: String,
    /// Full DSL content.
    pub source: String,
}

impl DslSource {
    /// Creates a source unit.
    pub fn new(path: impl Into<String>, source: impl Into<String>) -> Self {
        Self {
            path: path.into(),
            source: source.into(),
        }
    }
}

/// In-memory source document used for diagnostics and provenance.
#[derive(Debug, Clone)]
pub(crate) struct SourceDocument {
    pub(crate) path: String,
    pub(crate) source: String,
}

/// Parsed module unit loaded from one source file.
#[derive(Debug, Clone)]
pub(crate) struct ModuleUnit {
    pub(crate) doc: Rc<SourceDocument>,
    pub(crate) program: Program,
}

/// Loads and parses a full project import graph in dependency order.
pub(crate) fn load_project_modules<F>(
    entry_path: &str,
    loader: &mut F,
) -> Result<Vec<ModuleUnit>, CompileError>
where
    F: FnMut(&str) -> Result<String, String>,
{
    let mut loaded: HashMap<String, ModuleUnit> = HashMap::new();
    let mut load_order: Vec<String> = Vec::new();
    let mut stack: Vec<String> = Vec::new();
    load_module_recursive(entry_path, loader, &mut loaded, &mut load_order, &mut stack)?;

    let mut modules = Vec::with_capacity(load_order.len());
    for path in load_order {
        let module = loaded
            .remove(&path)
            .expect("module must exist after successful recursive load");
        modules.push(module);
    }
    Ok(modules)
}

fn load_module_recursive<F>(
    path: &str,
    loader: &mut F,
    loaded: &mut HashMap<String, ModuleUnit>,
    load_order: &mut Vec<String>,
    stack: &mut Vec<String>,
) -> Result<(), CompileError>
where
    F: FnMut(&str) -> Result<String, String>,
{
    if loaded.contains_key(path) {
        return Ok(());
    }

    stack.push(path.to_string());

    let source = loader(path).map_err(|message| {
        CompileError::message_in_file(format!("Failed to load source '{path}': {message}"), path)
    })?;
    let program = parse_program_in_source(&source, path)?;

    for import in &program.imports {
        let resolved = resolve_import_path(path, &import.path);
        if stack.iter().any(|item| item == &resolved) {
            let mut cycle = stack.clone();
            cycle.push(resolved.clone());
            return Err(CompileError::from_span_in_source(
                format!("Import cycle detected: {}", cycle.join(" -> ")),
                path,
                &source,
                &import.span,
            ));
        }
        if loaded.contains_key(&resolved) {
            continue;
        }

        if let Err(err) = load_module_recursive(&resolved, loader, loaded, load_order, stack) {
            if err.line == 0 && err.column == 0 && err.file == resolved {
                return Err(CompileError::from_span_in_source(
                    format!("Failed to load import '{}': {}", import.path, err.message),
                    path,
                    &source,
                    &import.span,
                ));
            }
            return Err(err);
        }
    }

    let _ = stack.pop();
    loaded.insert(
        path.to_string(),
        ModuleUnit {
            doc: Rc::new(SourceDocument {
                path: path.to_string(),
                source,
            }),
            program,
        },
    );
    load_order.push(path.to_string());
    Ok(())
}
