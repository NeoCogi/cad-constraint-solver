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

//! Multi-file project compilation pipeline.

mod loader;
mod paths;

pub use loader::DslSource;
pub(crate) use loader::{ModuleUnit, SourceDocument, load_project_modules};
pub(crate) use paths::normalize_path_str;

use crate::CompileError;
use crate::compiler::lower_modules;
use crate::model::Model;
use std::collections::HashMap;

pub(crate) fn compile_dsl_project_impl(
    entry_path: &str,
    system_name: Option<&str>,
    sources: &[DslSource],
) -> Result<Model, CompileError> {
    let mut by_path: HashMap<String, String> = HashMap::new();
    for source in sources {
        let normalized = normalize_path_str(&source.path);
        if by_path.contains_key(&normalized) {
            return Err(CompileError::message_in_file(
                format!("Duplicate source path '{normalized}'"),
                normalized,
            ));
        }
        by_path.insert(normalized, source.source.clone());
    }

    let entry_path = normalize_path_str(entry_path);
    if !by_path.contains_key(&entry_path) {
        return Err(CompileError::message_in_file(
            format!("Entry source '{entry_path}' was not provided"),
            entry_path,
        ));
    }

    compile_dsl_project_with_loader(&entry_path, system_name, |path| {
        by_path
            .get(path)
            .cloned()
            .ok_or_else(|| format!("source '{path}' was not provided"))
    })
}

pub(crate) fn compile_dsl_project_with_loader<F>(
    entry_path: &str,
    system_name: Option<&str>,
    mut loader: F,
) -> Result<Model, CompileError>
where
    F: FnMut(&str) -> Result<String, String>,
{
    let entry_path = normalize_path_str(entry_path);
    let modules = load_project_modules(&entry_path, &mut loader)?;
    lower_modules(&modules, system_name)
}
