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

//! CAD-oriented DSL layer on top of `constraint-solver`.
//!
//! This crate provides:
//! - A typed DSL parser (`scalar`, `vec2d`, `vec3d`).
//! - Reusable `constraint_fn` blocks and `use` composition.
//! - Named `system` blocks with `export`/`local` declaration visibility.
//! - Lowering from typed DSL expressions to scalar `constraint_solver::Exp`.
//! - Solve-time adapters for `rs_math3d` vector types.
//! - Rich compile diagnostics with line/column/caret output.
//!
//! # Pipeline
//!
//! 1. Parse DSL into AST with source spans.
//! 2. Lower/type-check into scalar equations (`f(x) - g(x) = 0`).
//! 3. Compile with `constraint_solver::Compiler`.
//! 4. Solve with `NewtonRaphsonSolver`.
//!
//! # Variable Flattening
//!
//! `constraint-solver` accepts scalar variable names only. Vector symbols are
//! flattened:
//! - `p: vec2d` -> `p.x`, `p.y`
//! - `n: vec3d` -> `n.x`, `n.y`, `n.z`
//!
//! `SolveSeed` handles this flattening for `Vec2d/Vec3d` and `Vec2f/Vec3f`.

mod ast;
mod diagnostics;
mod parser;

pub use ast::{
    BinOp, ConstraintFn, Decl, Expr, ExprKind, Field, Param, Program, SourceSpan, Stmt, StmtKind,
    System, SystemStmt, SystemStmtKind, TypeName, UseStmt, Visibility,
};
use constraint_solver::{
    CompileError as SolverCompileError, Compiler as SolverCompiler, Exp, NewtonRaphsonSolver,
    Solution as RawSolution, SolverError as RawSolverError,
};
pub use diagnostics::CompileError;
use parser::parse_program;
use rs_math3d::{Vec2d, Vec2f, Vec3d, Vec3f};
use std::collections::{BTreeSet, HashMap};
use std::fmt;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SymbolType {
    /// A scalar (`f64`) symbol.
    Scalar,
    /// A 2D vector symbol (`rs_math3d::Vec2d`).
    Vec2d,
    /// A 3D vector symbol (`rs_math3d::Vec3d`).
    Vec3d,
}

/// A compiled DSL model ready to be solved.
///
/// `Model` stores lowered scalar equations and visibility-aware symbol metadata.
#[derive(Debug, Clone)]
pub struct Model {
    equations: Vec<Exp>,
    // Public API symbols only (`export` declarations, or legacy top-level decls).
    public_symbols: HashMap<String, SymbolType>,
    // Default seed values for *all* flattened variables (public + local).
    defaults: HashMap<String, f64>,
    // All flattened declared variable names.
    flattened_names: Vec<String>,
    // All flattened variable names referenced by final equations.
    solver_names: Vec<String>,
    // Public flattened names (subset of `flattened_names`).
    public_flattened_names: Vec<String>,
    // Public flattened names that are referenced by equations.
    public_solver_names: Vec<String>,
    // Local/internal flattened names that are referenced by equations.
    hidden_solver_names: Vec<String>,
}

impl Model {
    /// Returns lowered scalar equations as `constraint_solver::Exp`.
    pub fn equations(&self) -> &[Exp] {
        &self.equations
    }

    /// Returns all flattened symbol component names declared by the DSL.
    ///
    /// This includes local/internal variables.
    pub fn flattened_names(&self) -> &[String] {
        &self.flattened_names
    }

    /// Returns flattened variable names that appear in solver equations.
    ///
    /// This includes local/internal variables.
    pub fn solver_names(&self) -> &[String] {
        &self.solver_names
    }

    /// Returns flattened *public* names.
    pub fn public_flattened_names(&self) -> &[String] {
        &self.public_flattened_names
    }

    /// Returns flattened *public* names used by equations.
    pub fn public_solver_names(&self) -> &[String] {
        &self.public_solver_names
    }

    /// Returns the declared public DSL type for a top-level symbol.
    pub fn symbol_type(&self, name: &str) -> Option<SymbolType> {
        self.public_symbols.get(name).copied()
    }

    /// Builds a default [`SolveSeed`] for public API variables.
    ///
    /// Local/internal variables are intentionally omitted from this seed.
    pub fn bootstrap_seed(&self) -> SolveSeed {
        let mut initial = HashMap::new();
        for name in &self.public_flattened_names {
            if let Some(value) = self.defaults.get(name) {
                initial.insert(name.clone(), *value);
            }
        }

        SolveSeed {
            initial,
            unknowns: self.public_solver_names.iter().cloned().collect(),
        }
    }

    /// Solves the model using Newton-Raphson without line search.
    pub fn solve(&self, seed: SolveSeed) -> Result<SolveResult, SolveError> {
        self.solve_internal(seed, false)
    }

    /// Solves the model using Newton-Raphson with line search.
    pub fn solve_with_line_search(&self, seed: SolveSeed) -> Result<SolveResult, SolveError> {
        self.solve_internal(seed, true)
    }

    fn solve_internal(
        &self,
        seed: SolveSeed,
        use_line_search: bool,
    ) -> Result<SolveResult, SolveError> {
        // Ensure caller can only modify public API variables.
        self.validate_seed(&seed)?;

        // Start from declaration defaults, then apply caller overrides.
        let solver_name_set: BTreeSet<String> = self.solver_names.iter().cloned().collect();
        let mut initial = self.defaults.clone();
        initial.extend(seed.initial);
        // Backend rejects unknown names in the initial map.
        initial.retain(|name, _| solver_name_set.contains(name));

        // Convert lowered symbolic equations to backend representation.
        let compiled =
            SolverCompiler::compile(&self.equations).map_err(SolveError::InternalCompile)?;

        let solver = if seed.unknowns.is_empty() {
            // Solve all variables when unknowns are not specified.
            NewtonRaphsonSolver::new(compiled)
        } else {
            // Always include hidden/local solver vars as unknowns.
            let mut unknowns = seed.unknowns;
            unknowns.extend(self.hidden_solver_names.iter().cloned());
            let unknowns: Vec<String> = unknowns.into_iter().collect();
            let names: Vec<&str> = unknowns.iter().map(String::as_str).collect();
            NewtonRaphsonSolver::new_with_variables(compiled, &names)?
        };

        // Choose solve mode requested by caller.
        let raw = if use_line_search {
            solver.solve_with_line_search(initial)?
        } else {
            solver.solve(initial)?
        };

        Ok(SolveResult { raw })
    }

    fn validate_seed(&self, seed: &SolveSeed) -> Result<(), SolveError> {
        let public_name_set: BTreeSet<String> =
            self.public_flattened_names.iter().cloned().collect();
        let public_solver_set: BTreeSet<String> =
            self.public_solver_names.iter().cloned().collect();

        // Initial values may include only public names.
        for name in seed.initial.keys() {
            if !public_name_set.contains(name) {
                return Err(SolveError::UnknownVariable(name.clone()));
            }
        }

        // Explicit unknown set may include only public solver names.
        for name in &seed.unknowns {
            if !public_solver_set.contains(name) {
                return Err(SolveError::UnknownVariable(name.clone()));
            }
        }
        Ok(())
    }
}

/// Solve-time values and unknown-variable selection.
///
/// This builder owns:
/// - `initial`: scalar values keyed by flattened variable name.
/// - `unknowns`: a subset of flattened names that the solver should optimize.
#[derive(Debug, Clone, Default)]
pub struct SolveSeed {
    initial: HashMap<String, f64>,
    unknowns: BTreeSet<String>,
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

/// Typed view over the raw solver output.
#[derive(Debug, Clone)]
pub struct SolveResult {
    raw: RawSolution,
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

/// Errors produced by compile or solve stages.
#[derive(Debug)]
pub enum SolveError {
    /// DSL compilation failed with source diagnostics.
    DslCompile(CompileError),
    /// A variable name was not declared or not publicly exposed.
    UnknownVariable(String),
    /// A requested value was not present in the final solution map.
    MissingValue(String),
    /// Lowered equations could not be compiled by `constraint-solver`.
    InternalCompile(SolverCompileError),
    /// Runtime solve failure from the backend solver.
    Solver(RawSolverError),
}

impl fmt::Display for SolveError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SolveError::DslCompile(err) => write!(f, "{err}"),
            SolveError::UnknownVariable(name) => write!(f, "Unknown variable '{name}'"),
            SolveError::MissingValue(name) => write!(f, "Missing value for variable '{name}'"),
            SolveError::InternalCompile(err) => {
                write!(f, "Failed to compile solver equations: {err}")
            }
            SolveError::Solver(err) => write!(f, "{err:?}"),
        }
    }
}

impl std::error::Error for SolveError {}

impl From<RawSolverError> for SolveError {
    fn from(value: RawSolverError) -> Self {
        SolveError::Solver(value)
    }
}

/// Parses and compiles DSL source into a ready-to-solve [`Model`].
///
/// If the source contains a single `system`, that system is compiled.
/// If it contains no `system`, legacy top-level statements are compiled.
///
/// # Errors
///
/// Returns [`CompileError`] with source line/column and caret highlight when
/// parsing or lowering fails.
pub fn compile_dsl(source: &str) -> Result<Model, CompileError> {
    let program = parse_program(source)?;
    lower_program(source, &program, None)
}

/// Parses and compiles one named `system` from source.
pub fn compile_dsl_system(source: &str, system_name: &str) -> Result<Model, CompileError> {
    let program = parse_program(source)?;
    lower_program(source, &program, Some(system_name))
}

/// Parses DSL source into a spanned AST [`Program`].
pub fn parse_dsl(source: &str) -> Result<Program, CompileError> {
    parse_program(source)
}

/// Compiles DSL and returns a bootstrap solve seed in one call.
pub fn compile_with_bootstrap(source: &str) -> Result<(Model, SolveSeed), CompileError> {
    let model = compile_dsl(source)?;
    let seed = model.bootstrap_seed();
    Ok((model, seed))
}

/// Convenience function that compiles and solves in one step.
pub fn solve_dsl(source: &str, seed: SolveSeed) -> Result<SolveResult, SolveError> {
    let model = compile_dsl(source).map_err(SolveError::DslCompile)?;
    model.solve(seed)
}

/// Internal typed expression used during lowering.
#[derive(Debug, Clone)]
enum ValueExp {
    Scalar(Exp),
    Vec2([Exp; 2]),
    Vec3([Exp; 3]),
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

/// Lowering context for one compilation unit.
///
/// Holds function registry, scope stack, emitted equations, and inferred defaults.
struct LowerContext<'a> {
    source: &'a str,
    functions: HashMap<String, ConstraintFn>,
    scopes: Vec<HashMap<String, ValueExp>>,
    public_symbols: HashMap<String, SymbolType>,
    equations: Vec<Exp>,
    defaults: HashMap<String, f64>,
    flattened_names: Vec<String>,
    public_flattened_names: Vec<String>,
    invocation_counter: usize,
    call_stack: Vec<String>,
}

impl<'a> LowerContext<'a> {
    /// Creates a fresh lowering context bound to source text.
    fn new(source: &'a str, functions: HashMap<String, ConstraintFn>) -> Self {
        Self {
            source,
            functions,
            scopes: Vec::new(),
            public_symbols: HashMap::new(),
            equations: Vec::new(),
            defaults: HashMap::new(),
            flattened_names: Vec::new(),
            public_flattened_names: Vec::new(),
            invocation_counter: 0,
            call_stack: Vec::new(),
        }
    }

    /// Finalizes lowering and produces a [`Model`].
    fn build(self) -> Model {
        let mut solver_vars = BTreeSet::new();
        // Track only variables that survive lowering into final equations.
        for equation in &self.equations {
            collect_var_names(equation, &mut solver_vars);
        }

        let public_set: BTreeSet<String> = self.public_flattened_names.iter().cloned().collect();
        let mut public_solver = Vec::new();
        let mut hidden_solver = Vec::new();
        for name in &solver_vars {
            if public_set.contains(name) {
                public_solver.push(name.clone());
            } else {
                hidden_solver.push(name.clone());
            }
        }

        Model {
            equations: self.equations,
            public_symbols: self.public_symbols,
            defaults: self.defaults,
            flattened_names: self.flattened_names,
            solver_names: solver_vars.into_iter().collect(),
            public_flattened_names: self.public_flattened_names,
            public_solver_names: public_solver,
            hidden_solver_names: hidden_solver,
        }
    }

    /// Creates a source-mapped compile error.
    fn error_at(&self, message: impl Into<String>, span: &SourceSpan) -> CompileError {
        CompileError::from_span(message, self.source, span)
    }

    fn push_scope(&mut self, scope: HashMap<String, ValueExp>) {
        self.scopes.push(scope);
    }

    fn pop_scope(&mut self) {
        let _ = self.scopes.pop();
    }

    fn current_scope(&self) -> &HashMap<String, ValueExp> {
        self.scopes
            .last()
            .expect("lowering always executes within at least one scope")
    }

    fn current_scope_mut(&mut self) -> &mut HashMap<String, ValueExp> {
        self.scopes
            .last_mut()
            .expect("lowering always executes within at least one scope")
    }

    fn resolve_binding(&self, name: &str) -> Option<ValueExp> {
        for scope in self.scopes.iter().rev() {
            if let Some(value) = scope.get(name) {
                return Some(value.clone());
            }
        }
        None
    }

    /// Registers one declared variable in current scope and metadata stores.
    fn register_declaration(
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
    fn lower_decl(
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
    fn lower_system_stmt(&mut self, stmt: &SystemStmt) -> Result<(), CompileError> {
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
    fn lower_legacy_stmt(&mut self, stmt: &Stmt) -> Result<(), CompileError> {
        match &stmt.kind {
            StmtKind::Decl(decl) => self.lower_decl(decl, &decl.name, Some(&decl.name)),
            StmtKind::ConstraintEq { lhs, rhs } => self.lower_constraint(lhs, rhs),
            StmtKind::Use(call) => self.lower_use(call),
        }
    }

    /// Lowers one statement from a `constraint_fn` body.
    fn lower_function_stmt(
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

    /// Instantiates and lowers a reusable `use constraint_fn(...)` call.
    fn lower_use(&mut self, use_stmt: &UseStmt) -> Result<(), CompileError> {
        let Some(function) = self.functions.get(&use_stmt.name).cloned() else {
            return Err(self.error_at(
                format!("Unknown constraint function '{}'", use_stmt.name),
                &use_stmt.span,
            ));
        };

        if self.call_stack.iter().any(|name| name == &function.name) {
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

        self.call_stack.push(function.name.clone());
        self.push_scope(param_scope);

        self.invocation_counter += 1;
        let namespace_prefix = format!("__fn_{}_{}_", function.name, self.invocation_counter);

        for stmt in &function.body {
            self.lower_function_stmt(stmt, &namespace_prefix)?;
        }

        self.pop_scope();
        let _ = self.call_stack.pop();
        Ok(())
    }

    /// Lowers one `constraint lhs == rhs;` statement.
    fn lower_constraint(&mut self, lhs: &Expr, rhs: &Expr) -> Result<(), CompileError> {
        // Both sides are type-lowered first, then expanded to scalar equations.
        let lhs = self.lower_expr(lhs)?;
        let rhs_value = self.lower_expr(rhs)?;
        self.push_equality(lhs, rhs_value, &rhs.span)
    }

    /// Converts typed equality into one or more scalar equations.
    fn push_equality(
        &mut self,
        lhs: ValueExp,
        rhs: ValueExp,
        span: &SourceSpan,
    ) -> Result<(), CompileError> {
        match (lhs, rhs) {
            (ValueExp::Scalar(l), ValueExp::Scalar(r)) => {
                // Scalar equality: one residual equation.
                self.equations.push(Exp::sub(l, r));
                Ok(())
            }
            (ValueExp::Vec2(l), ValueExp::Vec2(r)) => {
                // Vector equality expands component-wise.
                self.equations.push(Exp::sub(l[0].clone(), r[0].clone()));
                self.equations.push(Exp::sub(l[1].clone(), r[1].clone()));
                Ok(())
            }
            (ValueExp::Vec3(l), ValueExp::Vec3(r)) => {
                // Vector equality expands component-wise.
                self.equations.push(Exp::sub(l[0].clone(), r[0].clone()));
                self.equations.push(Exp::sub(l[1].clone(), r[1].clone()));
                self.equations.push(Exp::sub(l[2].clone(), r[2].clone()));
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

    /// Recursively lowers an AST expression into a typed value expression.
    fn lower_expr(&self, expr: &Expr) -> Result<ValueExp, CompileError> {
        match &expr.kind {
            ExprKind::Number(v) => Ok(ValueExp::Scalar(Exp::val(*v))),
            ExprKind::Ident(name) => {
                // Identifier lookup is scope-driven (params, locals, exported vars).
                let Some(value) = self.resolve_binding(name) else {
                    return Err(self.error_at(format!("Unknown identifier '{name}'"), &expr.span));
                };
                Ok(value)
            }
            ExprKind::VectorLit(items) => self.lower_vector_literal(items, &expr.span),
            ExprKind::UnaryNeg(inner) => {
                // Unary negation is supported for scalar and vectors.
                let value = self.lower_expr(inner)?;
                Ok(match value {
                    ValueExp::Scalar(e) => ValueExp::Scalar(Exp::neg(e)),
                    ValueExp::Vec2(v) => {
                        ValueExp::Vec2([Exp::neg(v[0].clone()), Exp::neg(v[1].clone())])
                    }
                    ValueExp::Vec3(v) => ValueExp::Vec3([
                        Exp::neg(v[0].clone()),
                        Exp::neg(v[1].clone()),
                        Exp::neg(v[2].clone()),
                    ]),
                })
            }
            ExprKind::Binary { op, left, right } => {
                // Binary operations are dispatched based on operand types.
                let left = self.lower_expr(left)?;
                let right = self.lower_expr(right)?;
                self.lower_binary(*op, left, right, &expr.span)
            }
            ExprKind::Call { name, args } => self.lower_call(name, args, &expr.span),
            ExprKind::Field { base, field } => {
                // Field access projects a vector component to scalar.
                let base = self.lower_expr(base)?;
                match (base, field) {
                    (ValueExp::Vec2(v), Field::X) => Ok(ValueExp::Scalar(v[0].clone())),
                    (ValueExp::Vec2(v), Field::Y) => Ok(ValueExp::Scalar(v[1].clone())),
                    (ValueExp::Vec2(_), Field::Z) => {
                        Err(self.error_at("Field '.z' is invalid for vec2d", &expr.span))
                    }
                    (ValueExp::Vec3(v), Field::X) => Ok(ValueExp::Scalar(v[0].clone())),
                    (ValueExp::Vec3(v), Field::Y) => Ok(ValueExp::Scalar(v[1].clone())),
                    (ValueExp::Vec3(v), Field::Z) => Ok(ValueExp::Scalar(v[2].clone())),
                    (ValueExp::Scalar(_), _) => {
                        Err(self.error_at("Field access requires vector value", &expr.span))
                    }
                }
            }
        }
    }

    fn lower_vector_literal(
        &self,
        items: &[Expr],
        span: &SourceSpan,
    ) -> Result<ValueExp, CompileError> {
        // Keep literal dimensions explicit to avoid implicit widening/narrowing.
        if !(2..=3).contains(&items.len()) {
            return Err(self.error_at(
                format!(
                    "Vector literal must have 2 or 3 elements, found {}",
                    items.len()
                ),
                span,
            ));
        }

        let mut lowered = Vec::with_capacity(items.len());
        for item in items {
            let value = self.lower_expr(item)?;
            // Literal components must each be scalar.
            let Some(scalar) = value.scalar() else {
                return Err(self.error_at("Vector literal elements must be scalar", &item.span));
            };
            lowered.push(scalar);
        }

        Ok(match lowered.len() {
            2 => ValueExp::Vec2([lowered[0].clone(), lowered[1].clone()]),
            3 => ValueExp::Vec3([lowered[0].clone(), lowered[1].clone(), lowered[2].clone()]),
            _ => unreachable!("checked literal length"),
        })
    }

    /// Lowers binary expressions with type-directed dispatch.
    fn lower_binary(
        &self,
        op: BinOp,
        left: ValueExp,
        right: ValueExp,
        span: &SourceSpan,
    ) -> Result<ValueExp, CompileError> {
        // Arithmetic legality depends on both operator and operand shapes.
        match op {
            BinOp::Add => self.binary_add_sub(true, left, right, span),
            BinOp::Sub => self.binary_add_sub(false, left, right, span),
            BinOp::Mul => self.binary_mul(left, right, span),
            BinOp::Div => self.binary_div(left, right, span),
        }
    }

    /// Lowers `+` and `-` for scalar/vector operands.
    fn binary_add_sub(
        &self,
        is_add: bool,
        left: ValueExp,
        right: ValueExp,
        span: &SourceSpan,
    ) -> Result<ValueExp, CompileError> {
        match (left, right) {
            (ValueExp::Scalar(l), ValueExp::Scalar(r)) => Ok(ValueExp::Scalar(if is_add {
                Exp::add(l, r)
            } else {
                Exp::sub(l, r)
            })),
            (ValueExp::Vec2(l), ValueExp::Vec2(r)) => Ok(ValueExp::Vec2([
                if is_add {
                    Exp::add(l[0].clone(), r[0].clone())
                } else {
                    Exp::sub(l[0].clone(), r[0].clone())
                },
                if is_add {
                    Exp::add(l[1].clone(), r[1].clone())
                } else {
                    Exp::sub(l[1].clone(), r[1].clone())
                },
            ])),
            (ValueExp::Vec3(l), ValueExp::Vec3(r)) => Ok(ValueExp::Vec3([
                if is_add {
                    Exp::add(l[0].clone(), r[0].clone())
                } else {
                    Exp::sub(l[0].clone(), r[0].clone())
                },
                if is_add {
                    Exp::add(l[1].clone(), r[1].clone())
                } else {
                    Exp::sub(l[1].clone(), r[1].clone())
                },
                if is_add {
                    Exp::add(l[2].clone(), r[2].clone())
                } else {
                    Exp::sub(l[2].clone(), r[2].clone())
                },
            ])),
            (l, r) => Err(self.error_at(
                format!(
                    "Operator '{}' is not defined for {} and {}",
                    if is_add { "+" } else { "-" },
                    l.type_name(),
                    r.type_name()
                ),
                span,
            )),
        }
    }

    /// Lowers `*` for scalar-scalar and scalar-vector operations.
    fn binary_mul(
        &self,
        left: ValueExp,
        right: ValueExp,
        span: &SourceSpan,
    ) -> Result<ValueExp, CompileError> {
        match (left, right) {
            (ValueExp::Scalar(l), ValueExp::Scalar(r)) => Ok(ValueExp::Scalar(Exp::mul(l, r))),
            // Allow scalar-vector scaling in either operand order.
            (ValueExp::Vec2(v), ValueExp::Scalar(s)) | (ValueExp::Scalar(s), ValueExp::Vec2(v)) => {
                Ok(ValueExp::Vec2([
                    Exp::mul(v[0].clone(), s.clone()),
                    Exp::mul(v[1].clone(), s),
                ]))
            }
            // Allow scalar-vector scaling in either operand order.
            (ValueExp::Vec3(v), ValueExp::Scalar(s)) | (ValueExp::Scalar(s), ValueExp::Vec3(v)) => {
                Ok(ValueExp::Vec3([
                    Exp::mul(v[0].clone(), s.clone()),
                    Exp::mul(v[1].clone(), s.clone()),
                    Exp::mul(v[2].clone(), s),
                ]))
            }
            (l, r) => Err(self.error_at(
                format!(
                    "Operator '*' is not defined for {} and {}",
                    l.type_name(),
                    r.type_name()
                ),
                span,
            )),
        }
    }

    /// Lowers `/` for scalar-scalar and vector-scalar operations.
    fn binary_div(
        &self,
        left: ValueExp,
        right: ValueExp,
        span: &SourceSpan,
    ) -> Result<ValueExp, CompileError> {
        match (left, right) {
            (ValueExp::Scalar(l), ValueExp::Scalar(r)) => Ok(ValueExp::Scalar(Exp::div(l, r))),
            // Vector division is only defined by scalar denominator.
            (ValueExp::Vec2(v), ValueExp::Scalar(s)) => Ok(ValueExp::Vec2([
                Exp::div(v[0].clone(), s.clone()),
                Exp::div(v[1].clone(), s),
            ])),
            // Vector division is only defined by scalar denominator.
            (ValueExp::Vec3(v), ValueExp::Scalar(s)) => Ok(ValueExp::Vec3([
                Exp::div(v[0].clone(), s.clone()),
                Exp::div(v[1].clone(), s.clone()),
                Exp::div(v[2].clone(), s),
            ])),
            (l, r) => Err(self.error_at(
                format!(
                    "Operator '/' is not defined for {} and {}",
                    l.type_name(),
                    r.type_name()
                ),
                span,
            )),
        }
    }

    /// Lowers builtin function calls.
    ///
    /// Supported builtins:
    /// - `dot2(vec2d, vec2d)`
    /// - `dot3(vec3d, vec3d)`
    /// - `length2(vec2d)`
    /// - `length3(vec3d)`
    /// - `sin/cos/ln/exp(scalar)`
    fn lower_call(
        &self,
        name: &str,
        args: &[Expr],
        span: &SourceSpan,
    ) -> Result<ValueExp, CompileError> {
        let mut lowered = Vec::with_capacity(args.len());
        for arg in args {
            lowered.push(self.lower_expr(arg)?);
        }

        // Builtins are explicit scalar expansions because `constraint_solver::Exp`
        // does not include vector primitives.
        match name {
            "dot2" => {
                let [a, b] = expect_2_args(lowered, name, self, span)?;
                let (a, b) = match (a, b) {
                    (ValueExp::Vec2(a), ValueExp::Vec2(b)) => (a, b),
                    (l, r) => {
                        return Err(self.error_at(
                            format!(
                                "dot2 expects (vec2d, vec2d), got ({}, {})",
                                l.type_name(),
                                r.type_name()
                            ),
                            span,
                        ));
                    }
                };
                Ok(ValueExp::Scalar(dot2_exp(&a, &b)))
            }
            "dot3" => {
                let [a, b] = expect_2_args(lowered, name, self, span)?;
                let (a, b) = match (a, b) {
                    (ValueExp::Vec3(a), ValueExp::Vec3(b)) => (a, b),
                    (l, r) => {
                        return Err(self.error_at(
                            format!(
                                "dot3 expects (vec3d, vec3d), got ({}, {})",
                                l.type_name(),
                                r.type_name()
                            ),
                            span,
                        ));
                    }
                };
                Ok(ValueExp::Scalar(dot3_exp(&a, &b)))
            }
            "length2" => {
                let [a] = expect_1_arg(lowered, name, self, span)?;
                let a = match a {
                    ValueExp::Vec2(v) => v,
                    other => {
                        return Err(self.error_at(
                            format!("length2 expects vec2d, got {}", other.type_name()),
                            span,
                        ));
                    }
                };
                // length(v) = sqrt(dot(v,v)).
                let d = dot2_exp(&a, &a);
                Ok(ValueExp::Scalar(Exp::power(d, 0.5)))
            }
            "length3" => {
                let [a] = expect_1_arg(lowered, name, self, span)?;
                let a = match a {
                    ValueExp::Vec3(v) => v,
                    other => {
                        return Err(self.error_at(
                            format!("length3 expects vec3d, got {}", other.type_name()),
                            span,
                        ));
                    }
                };
                // length(v) = sqrt(dot(v,v)).
                let d = dot3_exp(&a, &a);
                Ok(ValueExp::Scalar(Exp::power(d, 0.5)))
            }
            "sin" | "cos" | "ln" | "exp" => {
                let [a] = expect_1_arg(lowered, name, self, span)?;
                let a = match a {
                    ValueExp::Scalar(v) => v,
                    other => {
                        return Err(self.error_at(
                            format!("{name} expects scalar, got {}", other.type_name()),
                            span,
                        ));
                    }
                };
                let out = match name {
                    "sin" => Exp::sin(a),
                    "cos" => Exp::cos(a),
                    "ln" => Exp::ln(a),
                    "exp" => Exp::exp(a),
                    _ => unreachable!(),
                };
                Ok(ValueExp::Scalar(out))
            }
            _ => Err(self.error_at(format!("Unknown function '{name}'"), span)),
        }
    }

    /// Attempts to evaluate constant initializers and store solved defaults.
    fn capture_defaults(&mut self, canonical_base: &str, ty: SymbolType, rhs: &ValueExp) {
        // Defaults are best-effort: only expressions resolvable from known defaults
        // are captured. Non-constant initializers are left as zero-seeded values.
        let values = match rhs {
            ValueExp::Scalar(e) => evaluate_exp(e, &self.defaults).map(|v| vec![v]),
            ValueExp::Vec2(v) => {
                let x = evaluate_exp(&v[0], &self.defaults);
                let y = evaluate_exp(&v[1], &self.defaults);
                match (x, y) {
                    (Some(x), Some(y)) => Some(vec![x, y]),
                    _ => None,
                }
            }
            ValueExp::Vec3(v) => {
                let x = evaluate_exp(&v[0], &self.defaults);
                let y = evaluate_exp(&v[1], &self.defaults);
                let z = evaluate_exp(&v[2], &self.defaults);
                match (x, y, z) {
                    (Some(x), Some(y), Some(z)) => Some(vec![x, y, z]),
                    _ => None,
                }
            }
        };

        if let Some(values) = values {
            // Write solved initializer values back into flattened default slots.
            for (component, value) in flattened_component_names(canonical_base, ty)
                .into_iter()
                .zip(values.into_iter())
            {
                self.defaults.insert(component, value);
            }
        }
    }
}

/// Builds function map and validates duplicate names.
fn collect_constraint_fns(
    source: &str,
    program: &Program,
) -> Result<HashMap<String, ConstraintFn>, CompileError> {
    let mut map = HashMap::new();
    for def in &program.constraint_fns {
        if map.contains_key(&def.name) {
            return Err(CompileError::from_span(
                format!("Duplicate constraint_fn '{}'", def.name),
                source,
                &def.span,
            ));
        }
        map.insert(def.name.clone(), def.clone());
    }
    Ok(map)
}

/// Selects the system that should be compiled.
fn pick_system<'a>(
    source: &str,
    program: &'a Program,
    selected_system: Option<&str>,
) -> Result<Option<&'a System>, CompileError> {
    if program.systems.is_empty() {
        if let Some(name) = selected_system {
            return Err(CompileError::message_only(format!(
                "Requested system '{name}' was not found"
            )));
        }
        return Ok(None);
    }

    if !program.statements.is_empty() {
        return Err(CompileError::from_span(
            "Top-level statements cannot be mixed with named system definitions",
            source,
            &program.systems[0].span,
        ));
    }

    if let Some(requested) = selected_system {
        let system = program
            .systems
            .iter()
            .find(|system| system.name == requested)
            .ok_or_else(|| {
                CompileError::message_only(format!("Requested system '{requested}' was not found"))
            })?;
        return Ok(Some(system));
    }

    if program.systems.len() > 1 {
        return Err(CompileError::from_span(
            "Multiple systems found; use compile_dsl_system(...) to select one",
            source,
            &program.systems[1].span,
        ));
    }

    Ok(program.systems.first())
}

/// Lowers a parsed program into a solve-ready model.
fn lower_program(
    source: &str,
    program: &Program,
    selected_system: Option<&str>,
) -> Result<Model, CompileError> {
    let functions = collect_constraint_fns(source, program)?;
    let mut ctx = LowerContext::new(source, functions);
    ctx.push_scope(HashMap::new());

    if let Some(system) = pick_system(source, program, selected_system)? {
        // Process system statements in source order.
        for stmt in &system.body {
            ctx.lower_system_stmt(stmt)?;
        }
    } else {
        if program.statements.is_empty() {
            return Err(CompileError::message_only(
                "No system or top-level statements to compile",
            ));
        }
        // Legacy mode: top-level declarations are considered public.
        for stmt in &program.statements {
            ctx.lower_legacy_stmt(stmt)?;
        }
    }

    ctx.pop_scope();
    Ok(ctx.build())
}

/// Validates and extracts exactly one argument.
fn expect_1_arg(
    mut args: Vec<ValueExp>,
    name: &str,
    ctx: &LowerContext<'_>,
    span: &SourceSpan,
) -> Result<[ValueExp; 1], CompileError> {
    // Keep function arity checks centralized for consistent diagnostics.
    if args.len() != 1 {
        return Err(ctx.error_at(
            format!("{name} expects exactly 1 argument, found {}", args.len()),
            span,
        ));
    }
    Ok([args.remove(0)])
}

/// Validates and extracts exactly two arguments.
fn expect_2_args(
    mut args: Vec<ValueExp>,
    name: &str,
    ctx: &LowerContext<'_>,
    span: &SourceSpan,
) -> Result<[ValueExp; 2], CompileError> {
    // Keep function arity checks centralized for consistent diagnostics.
    if args.len() != 2 {
        return Err(ctx.error_at(
            format!("{name} expects exactly 2 arguments, found {}", args.len()),
            span,
        ));
    }
    Ok([args.remove(0), args.remove(0)])
}

/// Builds scalar `dot(vec2, vec2)` expansion.
fn dot2_exp(a: &[Exp; 2], b: &[Exp; 2]) -> Exp {
    Exp::add(
        Exp::mul(a[0].clone(), b[0].clone()),
        Exp::mul(a[1].clone(), b[1].clone()),
    )
}

/// Builds scalar `dot(vec3, vec3)` expansion.
fn dot3_exp(a: &[Exp; 3], b: &[Exp; 3]) -> Exp {
    Exp::add(
        Exp::add(
            Exp::mul(a[0].clone(), b[0].clone()),
            Exp::mul(a[1].clone(), b[1].clone()),
        ),
        Exp::mul(a[2].clone(), b[2].clone()),
    )
}

/// Maps parser type token to public symbol type.
fn map_decl_type(ty: TypeName) -> SymbolType {
    match ty {
        TypeName::Scalar => SymbolType::Scalar,
        TypeName::Vec2 => SymbolType::Vec2d,
        TypeName::Vec3 => SymbolType::Vec3d,
    }
}

/// Returns human-readable symbol type name.
fn symbol_type_name(ty: SymbolType) -> &'static str {
    match ty {
        SymbolType::Scalar => "scalar",
        SymbolType::Vec2d => "vec2d",
        SymbolType::Vec3d => "vec3d",
    }
}

/// Returns whether a lowered value matches expected symbol type.
fn matches_symbol_type(value: &ValueExp, expected: SymbolType) -> bool {
    matches!(
        (value, expected),
        (ValueExp::Scalar(_), SymbolType::Scalar)
            | (ValueExp::Vec2(_), SymbolType::Vec2d)
            | (ValueExp::Vec3(_), SymbolType::Vec3d)
    )
}

/// Returns flattened solver component names for a base symbol.
fn flattened_component_names(name: &str, ty: SymbolType) -> Vec<String> {
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
fn variable_value(name: &str, ty: SymbolType) -> ValueExp {
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

/// Attempts constant-fold evaluation using known defaults.
fn evaluate_exp(exp: &Exp, env: &HashMap<String, f64>) -> Option<f64> {
    // A minimal evaluator used only for compile-time default inference.
    match exp {
        Exp::Val(v) => Some(*v),
        Exp::Var(name) => env.get(name).copied(),
        Exp::Add(l, r) => Some(evaluate_exp(l, env)? + evaluate_exp(r, env)?),
        Exp::Sub(l, r) => Some(evaluate_exp(l, env)? - evaluate_exp(r, env)?),
        Exp::Mul(l, r) => Some(evaluate_exp(l, env)? * evaluate_exp(r, env)?),
        Exp::Div(l, r) => Some(evaluate_exp(l, env)? / evaluate_exp(r, env)?),
        Exp::Power(base, power) => Some(evaluate_exp(base, env)?.powf(*power)),
        Exp::Neg(v) => Some(-evaluate_exp(v, env)?),
        Exp::Sin(v) => Some(evaluate_exp(v, env)?.sin()),
        Exp::Cos(v) => Some(evaluate_exp(v, env)?.cos()),
        Exp::Ln(v) => Some(evaluate_exp(v, env)?.ln()),
        Exp::Exp(v) => Some(evaluate_exp(v, env)?.exp()),
    }
}

/// Collects variable names referenced in an expression.
fn collect_var_names(exp: &Exp, out: &mut BTreeSet<String>) {
    // DFS walk over the scalar expression tree.
    match exp {
        Exp::Val(_) => {}
        Exp::Var(name) => {
            out.insert(name.clone());
        }
        Exp::Add(l, r) | Exp::Sub(l, r) | Exp::Mul(l, r) | Exp::Div(l, r) => {
            collect_var_names(l, out);
            collect_var_names(r, out);
        }
        Exp::Power(base, _)
        | Exp::Neg(base)
        | Exp::Sin(base)
        | Exp::Cos(base)
        | Exp::Ln(base)
        | Exp::Exp(base) => {
            collect_var_names(base, out);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reports_line_and_column_for_compile_errors() {
        let src = "scalar x;\nconstraint y == 1;";
        let err = compile_dsl(src).expect_err("compile should fail");
        assert_eq!(err.line, 2);
        assert_eq!(err.column, 12);
        assert!(err.pointer.contains('^'));
        assert!(err.to_string().contains("Unknown identifier 'y'"));
    }

    #[test]
    fn reports_parse_error_for_bad_parameter_syntax() {
        let src = "constraint_fn bad(a scalar) { constraint a == 1; }";
        let err = parse_dsl(src).expect_err("parse should fail");
        assert_eq!(err.line, 1);
        assert!(err.column > 0);
        assert!(err.message.contains("Syntax error"));
        assert!(err.pointer.contains('^'));
    }

    #[test]
    fn reports_parse_error_for_missing_semicolon() {
        let src = "system s { export scalar r constraint r == 1; }";
        let err = parse_dsl(src).expect_err("parse should fail");
        assert_eq!(err.line, 1);
        assert!(err.column > 0);
        assert!(err.message.contains("Syntax error"));
        assert!(err.pointer.contains('^'));
    }

    #[test]
    fn reports_parse_error_for_unclosed_use_call() {
        let src = "system s { use foo(1, 2; }";
        let err = parse_dsl(src).expect_err("parse should fail");
        assert_eq!(err.line, 1);
        assert!(err.column > 0);
        assert!(err.message.contains("Syntax error"));
        assert!(err.pointer.contains('^'));
    }

    #[test]
    fn supports_hash_comments() {
        let src = r#"
            # top-level comment
            system s {
                export scalar x; # inline comment
                constraint x == 2; # another comment
            }
        "#;

        let model = compile_dsl(src).expect("compile");
        let result = model.solve(model.bootstrap_seed()).expect("solve");
        let x = result.scalar("x").expect("x");
        assert!((x - 2.0).abs() < 1e-8);
    }

    #[test]
    fn solves_simple_vec2_constraints() {
        let src = "vec2d p;\nconstraint p == [3.0, 4.0];";
        let model = compile_dsl(src).expect("compile");
        let seed = model
            .bootstrap_seed()
            .unknown_vec2d("p", Vec2d::new(0.1, 0.2));
        let result = model.solve(seed).expect("solve");
        let p = result.vec2d("p").expect("vec2d");
        assert!((p.x - 3.0).abs() < 1e-8);
        assert!((p.y - 4.0).abs() < 1e-8);
    }

    #[test]
    fn solves_simple_vec3_constraints() {
        let src = r#"
            vec3d n;
            constraint n == [0.0, 0.0, 1.0];
            constraint dot3(n, [0.0, 0.0, 1.0]) == 1.0;
            constraint length3(n) == 1.0;
        "#;

        let model = compile_dsl(src).expect("compile");
        let seed = model
            .bootstrap_seed()
            .unknown_vec3d("n", Vec3d::new(0.2, -0.1, 0.9));
        let result = model.solve(seed).expect("solve");
        let n = result.vec3d("n").expect("vec3d");
        assert!(n.x.abs() < 1e-8);
        assert!(n.y.abs() < 1e-8);
        assert!((n.z - 1.0).abs() < 1e-8);
    }

    #[test]
    fn solves_composed_system_with_local_and_exported_vars() {
        let src = r#"
            constraint_fn tangent_to_line(center: vec2d, r: scalar, n: vec2d, d: scalar, side: scalar) {
                constraint dot2(n, center) + d == side * r;
            }

            constraint_fn equal_radius_tangent(c1: vec2d, c2: vec2d, r: scalar) {
                constraint dot2(c2 - c1, c2 - c1) == (2 * r) * (2 * r);
            }

            system circles {
                export vec2d c1;
                export vec2d c2;
                export scalar r;
                local vec2d n = [0, 1];

                use tangent_to_line(c1, r, n, 0, 1);
                use tangent_to_line(c1, r, n, -10, -1);
                use tangent_to_line(c2, r, n, 0, 1);
                use tangent_to_line(c2, r, n, -10, -1);
                use equal_radius_tangent(c1, c2, r);
                constraint c1.x == 0;
            }
        "#;

        let model = compile_dsl(src).expect("compile");
        assert_eq!(model.symbol_type("c1"), Some(SymbolType::Vec2d));
        assert_eq!(model.symbol_type("c2"), Some(SymbolType::Vec2d));
        assert_eq!(model.symbol_type("r"), Some(SymbolType::Scalar));
        assert_eq!(model.symbol_type("n"), None);
        assert!(!model.public_flattened_names().iter().any(|n| n == "n.x"));

        let seed = model
            .bootstrap_seed()
            .unknown_vec2d("c1", Vec2d::new(0.0, 4.0))
            .unknown_vec2d("c2", Vec2d::new(9.0, 6.0))
            .unknown_scalar("r", 4.5);

        let result = model.solve_with_line_search(seed).expect("solve");
        let r = result.scalar("r").expect("r");
        let c1 = result.vec2d("c1").expect("c1");
        let c2 = result.vec2d("c2").expect("c2");

        assert!((r - 5.0).abs() < 1e-6);
        assert!((c1.y - 5.0).abs() < 1e-6);
        assert!((c2.y - 5.0).abs() < 1e-6);
        assert!(c1.x.abs() < 1e-6);
        assert!((c2.x.abs() - 10.0).abs() < 1e-5);
    }

    #[test]
    fn rejects_setting_local_variable_from_public_api() {
        let src = r#"
            system s {
                export scalar x;
                local scalar hidden;
                constraint hidden == x + 1;
                constraint x == 2;
            }
        "#;

        let model = compile_dsl(src).expect("compile");
        let seed = model.bootstrap_seed().param_scalar("hidden", 12.0);
        let err = model.solve(seed).expect_err("solve should fail");
        assert!(matches!(err, SolveError::UnknownVariable(name) if name == "hidden"));
    }
}
