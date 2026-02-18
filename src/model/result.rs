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

//! Typed view over solver output values.

use constraint_solver::Solution as RawSolution;
use rs_math3d::{Vec2d, Vec2f, Vec3d, Vec3f};
use std::collections::HashMap;

use super::SolveError;

/// Typed view over the raw solver output.
#[derive(Debug, Clone)]
pub struct SolveResult {
    pub(crate) raw: RawSolution,
}

impl SolveResult {
    /// Returns the raw `constraint_solver::Solution`.
    pub fn raw(&self) -> &RawSolution {
        &self.raw
    }

    /// Returns all solved scalar values by flattened variable name.
    pub fn values(&self) -> &HashMap<String, f64> {
        &self.raw.values
    }

    /// Returns a scalar value by flattened name (for example `"r"` or `"p.x"`).
    pub fn scalar(&self, name: &str) -> Result<f64, SolveError> {
        self.raw
            .values
            .get(name)
            .copied()
            .ok_or_else(|| SolveError::MissingValue(name.to_string()))
    }

    /// Returns a `vec2d` by reading `<name>.x` and `<name>.y`.
    pub fn vec2d(&self, name: &str) -> Result<Vec2d, SolveError> {
        Ok(Vec2d::new(
            self.scalar(&format!("{name}.x"))?,
            self.scalar(&format!("{name}.y"))?,
        ))
    }

    /// Returns a `vec2f` by reading `<name>.x` and `<name>.y` and casting to `f32`.
    pub fn vec2f(&self, name: &str) -> Result<Vec2f, SolveError> {
        Ok(Vec2f::new(
            self.scalar(&format!("{name}.x"))? as f32,
            self.scalar(&format!("{name}.y"))? as f32,
        ))
    }

    /// Returns a `vec3d` by reading `<name>.x`, `<name>.y`, and `<name>.z`.
    pub fn vec3d(&self, name: &str) -> Result<Vec3d, SolveError> {
        Ok(Vec3d::new(
            self.scalar(&format!("{name}.x"))?,
            self.scalar(&format!("{name}.y"))?,
            self.scalar(&format!("{name}.z"))?,
        ))
    }

    /// Returns a `vec3f` by reading `<name>.x`, `<name>.y`, and `<name>.z` and casting to `f32`.
    pub fn vec3f(&self, name: &str) -> Result<Vec3f, SolveError> {
        Ok(Vec3f::new(
            self.scalar(&format!("{name}.x"))? as f32,
            self.scalar(&format!("{name}.y"))? as f32,
            self.scalar(&format!("{name}.z"))? as f32,
        ))
    }
}
