# cad-constraint-solver

`cad-constraint-solver` is a CAD-oriented DSL layer on top of:
- [`constraint-solver`](https://crates.io/crates/constraint-solver) for scalar nonlinear solving
- [`rs-math3d`](https://crates.io/crates/rs-math3d) for vector types (`Vec2d`, `Vec3d`, `Vec2f`, `Vec3f`)

It parses typed DSL source, lowers it into scalar `constraint_solver::Exp` equations, and solves via Newton-Raphson.

## Features

- Typed DSL symbols: `scalar`, `vec2d`/`vec2f`, `vec3d`/`vec3f`
- Reusable `constraint_fn` blocks and `use` composition
- Multi-file project support via top-level `import "path.dsl";`
- Named `system` blocks with `export` (public API) and `local` (internal) visibility
- Built-in math/vector helpers: `dot2`, `dot3`, `length2`, `length3`, `sin`, `cos`, `ln`, `exp`
- Solve seed helpers for scalar/vec2/vec3 values and unknown selection
- Rich diagnostics:
  - compile-time `line`, `column`, `snippet`, `pointer`
  - solve-time failure reports with issue tracebacks to source constraints
- Comment support: `# ...` and `// ...`

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

Inside a `system`:
- `export <decl>;` exposes variable(s) through the public API
- `local <decl>;` keeps variable(s) internal
- bare declarations default to local

### Constraints and Composition

```text
constraint <expr> == <expr>;
use some_constraint_fn(args...);
```

### Structure

```text
import "constraints/common.dsl";
constraint_fn name(param: type, ...) { ... }
system system_name { ... }
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

## Usage

### Basic Compile + Solve

```rust
use cad_constraint_solver::{compile_dsl, SolveError};
use rs_math3d::Vec2d;

let src = r#"
system s {
    export vec2d p;
    constraint p == [3.0, 4.0];
}
"#;

let model = compile_dsl(src)?;

let seed = model
    .bootstrap_seed()
    .unknown_vec2d("p", Vec2d::new(0.0, 0.0));

let result = model.solve(seed)?;
let p = result.vec2d("p")?;
assert!((p.x - 3.0).abs() < 1e-8);
assert!((p.y - 4.0).abs() < 1e-8);
# Ok::<(), SolveError>(())
```

### Compose Reusable Constraints

```rust
use cad_constraint_solver::compile_dsl;
use rs_math3d::Vec2d;

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

let model = compile_dsl(src)?;
let seed = model
    .bootstrap_seed()
    .unknown_vec2d("c1", Vec2d::new(0.0, 4.0))
    .unknown_vec2d("c2", Vec2d::new(9.0, 6.0))
    .unknown_scalar("r", 4.5);

let result = model.solve_with_line_search(seed)?;
let r = result.scalar("r")?;
assert!((r - 5.0).abs() < 1e-6);
# Ok::<(), cad_constraint_solver::SolveError>(())
```

### Multi-File Project Compile

```rust
use cad_constraint_solver::{compile_dsl_project, DslSource};
use rs_math3d::Vec2d;

let sources = vec![
    DslSource::new(
        "constraints/2d.dsl",
        r#"
        constraint_fn pin(p: vec2d) {
            constraint p == [3, 4];
        }
        "#,
    ),
    DslSource::new(
        "systems/main.dsl",
        r#"
        import "../constraints/2d.dsl";
        system s {
            export vec2d p;
            use pin(p);
        }
        "#,
    ),
];

let model = compile_dsl_project("systems/main.dsl", &sources)?;
let result = model.solve(
    model
        .bootstrap_seed()
        .unknown_vec2d("p", Vec2d::new(0.0, 0.0)),
)?;
let p = result.vec2d("p")?;
assert!((p.x - 3.0).abs() < 1e-8);
assert!((p.y - 4.0).abs() < 1e-8);
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

`SolveSeed` handles this flattening for you.

Visibility rules:
- only `export` variables are writable from the external API
- `local` variables remain internal
- attempting to set internal variables through the public API yields `SolveError::UnknownVariable`

## Diagnostics and Issue Locations

### Compile Errors (`CompileError`)

Parser/lowering errors include exact source location info:
- `message`
- `file` (source path/label)
- `line` (1-based)
- `column` (1-based)
- `snippet` (source line text)
- `pointer` (caret marker)

Example handling:

```rust
use cad_constraint_solver::compile_dsl;

let src = "scalar x;\nconstraint y == 1;";
let err = compile_dsl(src).unwrap_err();

println!("{}", err.message);
println!("file={}", err.file);
println!("line={}, col={}", err.line, err.column);
println!("{}", err.snippet);
println!("{}", err.pointer);
```

### Solve Failures (`SolveError::Failure`)

When solving fails, you get a structured `SolveFailureReport`:
- `kind`: `OverconstrainedInconsistency`, `NonConvergence`, or `SingularMatrix`
- `equation_count`, `unknown_count`, `overconstrained`
- `residuals`, solver `values`
- `issues`: top residual equations, each with:
  - `description` (includes system/use-call context)
  - `file`, `line`, `column`, `snippet`, `pointer`
  - `traceback` frames (`use` call chain with file/line/column/snippet/pointer)
  - `residual`, `magnitude`

Example handling:

```rust
use cad_constraint_solver::{compile_dsl, SolveError};

let src = "system bad {\nexport scalar x;\nconstraint x == 0;\nconstraint x == 1;\n}";
let model = compile_dsl(src).unwrap();
let err = model
    .solve(model.bootstrap_seed().unknown_scalar("x", 0.0))
    .unwrap_err();

if let SolveError::Failure(report) = err {
    println!("failure: {:?} residual={}", report.kind, report.error);
    for issue in &report.issues {
        println!(
            "eq#{}, {}:{}:{}: {}",
            issue.equation_index, issue.file, issue.line, issue.column, issue.description
        );
        println!("{}", issue.snippet);
        println!("{}", issue.pointer);
        for frame in &issue.traceback {
            println!(
                "  via use {} at {}:{}:{}",
                frame.function, frame.file, frame.line, frame.column
            );
        }
    }
}
```

Important location detail:
- currently, issue locations are anchored to the lowered equality span used during compilation.
- for `constraint lhs == rhs`, this maps to the RHS expression span, so columns usually point at the start of the RHS expression (not necessarily the `constraint` keyword).

Example:

```text
DSL source:
1 | system bad {
2 | export scalar x;
3 | constraint x == 0;
4 | constraint x == 1;
5 | }

Typical issue location for line 3:
line   = 3
column = 17
snippet: "constraint x == 0;"
pointer: "                ^"
```

In this example, the caret is under `0` (the RHS start), not under the `constraint` keyword.

## License

MIT, Copyright (c) 2026 Raja Lehtihet and Wael El Oraiby.
