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

//! Embedded DSL standard-library sources (`std2d`, `std3d`) and helpers.
//!
//! The standard library is shipped as plain DSL files and can be merged into
//! project sources when calling `compile_dsl_project*` APIs.

use crate::{CompileError, DslSource, Model, compile_dsl_project, compile_dsl_project_system};
use std::collections::HashSet;

use crate::project::normalize_path_str;

/// Canonical project path for the embedded 2D standard library.
pub const STDLIB_STD2D_PATH: &str = "stdlib/std2d.dsl";

/// Canonical project path for the embedded 3D standard library.
pub const STDLIB_STD3D_PATH: &str = "stdlib/std3d.dsl";

/// Embedded source text for `stdlib/std2d.dsl`.
pub const STDLIB_STD2D_SOURCE: &str = include_str!("../stdlib/std2d.dsl");

/// Embedded source text for `stdlib/std3d.dsl`.
pub const STDLIB_STD3D_SOURCE: &str = include_str!("../stdlib/std3d.dsl");

/// Returns the embedded standard-library sources as project units.
pub fn standard_library_sources() -> Vec<DslSource> {
    vec![
        DslSource::new(STDLIB_STD2D_PATH, STDLIB_STD2D_SOURCE),
        DslSource::new(STDLIB_STD3D_PATH, STDLIB_STD3D_SOURCE),
    ]
}

/// Merges user-provided sources with embedded standard-library sources.
///
/// User sources take precedence when a normalized path collides with one of
/// the embedded standard-library paths.
pub fn merge_with_standard_library_sources(user_sources: &[DslSource]) -> Vec<DslSource> {
    let mut merged = user_sources.to_vec();
    let mut seen: HashSet<String> = user_sources
        .iter()
        .map(|src| normalize_path_str(&src.path))
        .collect();

    for std_source in standard_library_sources() {
        let normalized = normalize_path_str(&std_source.path);
        if seen.insert(normalized) {
            merged.push(std_source);
        }
    }
    merged
}

/// Compiles a project after injecting embedded `std2d` / `std3d` sources.
pub fn compile_dsl_project_with_stdlib(
    entry_path: &str,
    sources: &[DslSource],
) -> Result<Model, CompileError> {
    let merged = merge_with_standard_library_sources(sources);
    compile_dsl_project(entry_path, &merged)
}

/// Compiles one named system from a project after injecting embedded stdlib.
pub fn compile_dsl_project_system_with_stdlib(
    entry_path: &str,
    system_name: &str,
    sources: &[DslSource],
) -> Result<Model, CompileError> {
    let merged = merge_with_standard_library_sources(sources);
    compile_dsl_project_system(entry_path, system_name, &merged)
}
