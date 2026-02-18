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

//! Filesystem-like path helpers for DSL imports.

use std::path::{Component, Path, PathBuf};

/// Resolves an import path relative to its importer source file.
pub(crate) fn resolve_import_path(importer_path: &str, import_path: &str) -> String {
    let import_path = Path::new(import_path);
    let joined = if import_path.is_absolute() {
        import_path.to_path_buf()
    } else {
        let base = Path::new(importer_path)
            .parent()
            .unwrap_or_else(|| Path::new("."));
        base.join(import_path)
    };
    normalize_path(&joined).to_string_lossy().to_string()
}

/// Normalizes a source path string.
pub(crate) fn normalize_path_str(path: &str) -> String {
    normalize_path(Path::new(path))
        .to_string_lossy()
        .to_string()
}

/// Performs lexical `.` / `..` normalization without touching the filesystem.
fn normalize_path(path: &Path) -> PathBuf {
    let mut normalized = PathBuf::new();
    let mut absolute = false;
    for component in path.components() {
        match component {
            Component::Prefix(prefix) => normalized.push(prefix.as_os_str()),
            Component::RootDir => {
                normalized.push(Path::new("/"));
                absolute = true;
            }
            Component::CurDir => {}
            Component::ParentDir => {
                if !normalized.pop() && !absolute {
                    normalized.push("..");
                }
            }
            Component::Normal(part) => normalized.push(part),
        }
    }

    if normalized.as_os_str().is_empty() {
        if absolute {
            normalized.push(Path::new("/"));
        } else {
            normalized.push(".");
        }
    }
    normalized
}
