# cad-constraint-solver

`cad-constraint-solver` is a CAD-oriented DSL layer on top of:
- [`constraint-solver`](https://crates.io/crates/constraint-solver) for scalar nonlinear solving
- [`rs-math3d`](https://crates.io/crates/rs-math3d) for vector types (`Vec2d`, `Vec3d`, `Vec2f`, `Vec3f`)

It parses typed DSL source, lowers it into scalar `constraint_solver::Exp` equations, and solves via Newton-Raphson.

## Features

- Typed DSL symbols: `scalar`, `vec2d`/`vec2f`, `vec3d`/`vec3f`
- Preferred callable model: parameterized `system` blocks with `in` / `out` / `inout` parameters
- Direct call statements: `helper(args...);`
- Multi-file project support via top-level `import "path.dsl";`
- Built-in math/vector helpers: `dot2`, `dot3`, `length2`, `length3`, `sin`, `cos`, `ln`, `exp`
- Solve seed helpers for scalar/vec2/vec3 values and unknown selection
- Rich diagnostics:
  - compile-time `line`, `column`, `snippet`, `pointer`
  - solve-time failure reports with issue tracebacks to source constraints
- Comment support: `# ...` and `// ...`
- Strict syntax: legacy `constraint_fn`, `use`, `export`, and `local` keywords are rejected

## Installation

Add to `Cargo.toml`:

```toml
[dependencies]
cad-constraint-solver = "0.1.0"
```

## DSL Quick Reference

### Declarations

```text
scalar x;
vec2d p;
vec3d n = [0, 0, 1];
```

### Systems (Preferred)

```text
system helper(in scalar a, out vec2d p, inout scalar k) {
    constraint p == [a, k];
}
```

- `in`: caller-provided input
- `out`: solver-produced output
- `inout`: caller-provided value that can also be solved/updated

### Statements

```text
constraint <expr> == <expr>;
callable_name(args...);   # preferred call syntax
```

Legacy constructs are not supported:
- `constraint_fn ...`
- `use name(args...);`
- `export <decl>;`
- `local <decl>;`

### Structure

```text
import "constraints/common.dsl";
system helper(in scalar a, out scalar b) { ... }
system main(out scalar x) { helper(1, x); }
```

### Expressions

- numbers, identifiers
- vector literals: `[a, b]`, `[a, b, c]`
- unary `-`
- binary `+`, `-`, `*`, `/`
- function calls
- field access `.x`, `.y`, `.z`

## Built-in Functions

- `dot2(vec2, vec2) -> scalar`
- `dot3(vec3, vec3) -> scalar`
- `length2(vec2) -> scalar`
- `length3(vec3) -> scalar`
- `sin(scalar)`, `cos(scalar)`, `ln(scalar)`, `exp(scalar)`

## System Parameter Semantics

### Call behavior

- Parameters alias caller arguments (no copy).
- Reusing the same variable(s) across calls accumulates constraints on those unknowns.
- Recursion is rejected (direct and indirect).

### `in` / `out` / `inout`

- `in` values are externally seedable.
- `out` values are externally readable in results, but not externally seedable.
- `inout` values are both seedable and readable.

Example:

```text
system pin(in scalar target, out scalar x) {
    constraint x == target;
}

system main(out scalar v) {
    pin(5, v);
}
```

If you try to seed `v` from the external API (`param_scalar("v", ...)`), compilation/solve API rejects it as non-writable.

Recursive call example (compile error):

```text
system rec(inout scalar x) {
    rec(x);  # recursive invocation is rejected
}
```

### Out/InOut call-site rule

For `out` and `inout` parameters, call arguments must be variable identifiers.

Valid:

```text
producer(out_val);
```

Invalid:

```text
producer(1 + 2);   # compile error
```

## Usage

### Basic Compile + Solve

```rust
use cad_constraint_solver::{compile_dsl_system, SolveError};

let src = r#"
system main(out scalar x) {
    constraint x == 3;
}
"#;

let model = compile_dsl_system(src, "main")?;
let result = model.solve(model.bootstrap_seed())?;
assert!((result.scalar("x")? - 3.0).abs() < 1e-8);
# Ok::<(), SolveError>(())
```

### Compose Systems

```rust
use cad_constraint_solver::compile_dsl_system;
use rs_math3d::Vec2d;

let src = r#"
system tangent_to_line(in vec2d center, in scalar r, in vec2d n, in scalar d, in scalar side) {
    constraint dot2(n, center) + d == side * r;
}

system circles(inout vec2d c1, inout vec2d c2, inout scalar r) {
    scalar n_y = 1;
    vec2d n = [0, n_y];

    tangent_to_line(c1, r, n, 0, 1);
    tangent_to_line(c1, r, n, -10, -1);
    tangent_to_line(c2, r, n, 0, 1);
    tangent_to_line(c2, r, n, -10, -1);

    constraint dot2(c2 - c1, c2 - c1) == (2 * r) * (2 * r);
    constraint c1.x == 0;
}
"#;

let model = compile_dsl_system(src, "circles")?;
let seed = model
    .bootstrap_seed()
    .unknown_vec2d("c1", Vec2d::new(0.0, 4.0))
    .unknown_vec2d("c2", Vec2d::new(9.0, 6.0))
    .unknown_scalar("r", 4.5);

let result = model.solve_with_line_search(seed)?;
assert!((result.scalar("r")? - 5.0).abs() < 1e-6);
# Ok::<(), cad_constraint_solver::SolveError>(())
```

### Multi-File Project Compile

```rust
use cad_constraint_solver::{compile_dsl_project_system, DslSource};

let sources = vec![
    DslSource::new(
        "constraints/pin.dsl",
        r#"
        system pin(in scalar target, out scalar x) {
            constraint x == target;
        }
        "#,
    ),
    DslSource::new(
        "systems/main.dsl",
        r#"
        import "../constraints/pin.dsl";

        system main(out scalar v) {
            pin(5, v);
        }
        "#,
    ),
];

let model = compile_dsl_project_system("systems/main.dsl", "main", &sources)?;
let result = model.solve(model.bootstrap_seed())?;
assert!((result.scalar("v")? - 5.0).abs() < 1e-8);
# Ok::<(), cad_constraint_solver::SolveError>(())
```

## Public API Overview

- Parsing/compilation:
  - `parse_dsl(source) -> Result<Program, CompileError>`
  - `compile_dsl(source) -> Result<Model, CompileError>`
  - `compile_dsl_system(source, name) -> Result<Model, CompileError>`
  - `compile_dsl_project(entry_path, sources) -> Result<Model, CompileError>`
  - `compile_dsl_project_system(entry_path, name, sources) -> Result<Model, CompileError>`
  - `compile_dsl_project_with_loader(entry_path, system_name, loader) -> Result<Model, CompileError>`
  - `compile_with_bootstrap(source) -> Result<(Model, SolveSeed), CompileError>`
- Solve:
  - `model.solve(seed)`
  - `model.solve_with_line_search(seed)`
  - `solve_dsl(source, seed)` convenience wrapper
- Seeding:
  - `SolveSeed::param_scalar`, `unknown_scalar`
  - `SolveSeed::param_vec2d`/`unknown_vec2d`
  - `SolveSeed::param_vec3d`/`unknown_vec3d`
  - `Vec2f`/`Vec3f` helpers are also provided
- Readback:
  - `SolveResult::scalar`, `vec2d`, `vec3d`, `vec2f`, `vec3f`

## Variables and Flattening

`constraint-solver` operates on scalar variable names. Vector symbols are flattened:

- `p: vec2d` -> `p.x`, `p.y`
- `n: vec3d` -> `n.x`, `n.y`, `n.z`

`SolveSeed` handles flattening for you.

## Diagnostics and Issue Locations

### Compile Errors (`CompileError`)

Parser/lowering errors include exact source location info:
- `message`
- `file` (source path/label)
- `line` (1-based)
- `column` (1-based)
- `snippet` (source line text)
- `pointer` (caret marker)

### Solve Failures (`SolveError::Failure`)

When solving fails, you get a structured `SolveFailureReport`:
- `kind`: `OverconstrainedInconsistency`, `NonConvergence`, or `SingularMatrix`
- `equation_count`, `unknown_count`, `overconstrained`
- `residuals`, solver `values`
- `issues`: top residual equations with source mapping and call traceback

Important location detail:
- currently, issue locations are anchored to the lowered equality span used during compilation.
- for `constraint lhs == rhs`, this maps to the RHS expression span, so columns usually point at the start of the RHS expression.

Example:

```text
system inconsistent(out scalar x) {
    constraint x == 1;
    constraint x == 2;
}
```

For a top residual on the second equation, the reported location is anchored near `2` (the RHS), not the `constraint` keyword.

## License

MIT, Copyright (c) 2026 Raja Lehtihet and Wael El Oraiby.
