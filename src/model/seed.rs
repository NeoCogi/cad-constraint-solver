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

//! Solve seed builder.

use rs_math3d::{Vec2d, Vec2f, Vec3d, Vec3f};
use std::collections::{BTreeSet, HashMap};

/// Solve-time values and unknown-variable selection.
///
/// This builder owns:
/// - `initial`: scalar values keyed by flattened variable name.
/// - `unknowns`: a subset of flattened names that the solver should optimize.
#[derive(Debug, Clone, Default)]
pub struct SolveSeed {
    pub(crate) initial: HashMap<String, f64>,
    pub(crate) unknowns: BTreeSet<String>,
}

impl SolveSeed {
    /// Creates an empty seed.
    pub fn new() -> Self {
        Self::default()
    }

    /// Sets a scalar parameter or initial value.
    pub fn param_scalar(mut self, name: &str, value: f64) -> Self {
        self.initial.insert(name.to_string(), value);
        self
    }

    /// Sets a scalar initial value and marks it as unknown.
    pub fn unknown_scalar(mut self, name: &str, guess: f64) -> Self {
        self.initial.insert(name.to_string(), guess);
        self.unknowns.insert(name.to_string());
        self
    }

    /// Sets a `vec2d` value by writing `<name>.x` and `<name>.y`.
    pub fn param_vec2d(mut self, name: &str, value: Vec2d) -> Self {
        self.initial.insert(format!("{name}.x"), value.x);
        self.initial.insert(format!("{name}.y"), value.y);
        self
    }

    /// Sets a `vec2d` initial value and marks both components as unknown.
    pub fn unknown_vec2d(mut self, name: &str, guess: Vec2d) -> Self {
        self = self.param_vec2d(name, guess);
        self.unknowns.insert(format!("{name}.x"));
        self.unknowns.insert(format!("{name}.y"));
        self
    }

    /// Sets a `vec2f` value by converting components to `f64`.
    pub fn param_vec2f(mut self, name: &str, value: Vec2f) -> Self {
        self.initial.insert(format!("{name}.x"), value.x as f64);
        self.initial.insert(format!("{name}.y"), value.y as f64);
        self
    }

    /// Sets a `vec2f` initial value and marks both components as unknown.
    pub fn unknown_vec2f(mut self, name: &str, guess: Vec2f) -> Self {
        self = self.param_vec2f(name, guess);
        self.unknowns.insert(format!("{name}.x"));
        self.unknowns.insert(format!("{name}.y"));
        self
    }

    /// Sets a `vec3d` value by writing `<name>.x`, `<name>.y`, and `<name>.z`.
    pub fn param_vec3d(mut self, name: &str, value: Vec3d) -> Self {
        self.initial.insert(format!("{name}.x"), value.x);
        self.initial.insert(format!("{name}.y"), value.y);
        self.initial.insert(format!("{name}.z"), value.z);
        self
    }

    /// Sets a `vec3d` initial value and marks all components as unknown.
    pub fn unknown_vec3d(mut self, name: &str, guess: Vec3d) -> Self {
        self = self.param_vec3d(name, guess);
        self.unknowns.insert(format!("{name}.x"));
        self.unknowns.insert(format!("{name}.y"));
        self.unknowns.insert(format!("{name}.z"));
        self
    }

    /// Sets a `vec3f` value by converting components to `f64`.
    pub fn param_vec3f(mut self, name: &str, value: Vec3f) -> Self {
        self.initial.insert(format!("{name}.x"), value.x as f64);
        self.initial.insert(format!("{name}.y"), value.y as f64);
        self.initial.insert(format!("{name}.z"), value.z as f64);
        self
    }

    /// Sets a `vec3f` initial value and marks all components as unknown.
    pub fn unknown_vec3f(mut self, name: &str, guess: Vec3f) -> Self {
        self = self.param_vec3f(name, guess);
        self.unknowns.insert(format!("{name}.x"));
        self.unknowns.insert(format!("{name}.y"));
        self.unknowns.insert(format!("{name}.z"));
        self
    }
}
