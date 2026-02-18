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

//! Crate unit tests.

use super::*;
use constraint_solver::{
    SolverError as RawSolverError, SolverRunDiagnostic as RawSolverRunDiagnostic,
};
use rs_math3d::{Vec2d, Vec3d};
use std::collections::HashMap;

fn first_caret_column(pointer: &str) -> Option<usize> {
    pointer.chars().position(|ch| ch == '^').map(|idx| idx + 1)
}

fn assert_parse_error_case(case_name: &str, source: &str, expected_line: usize) {
    let err = parse_dsl(source).expect_err("parse should fail");
    assert_eq!(
        err.line, expected_line,
        "{case_name}: unexpected error line"
    );
    assert!(err.column > 0, "{case_name}: expected non-zero column");
    assert!(
        err.message.contains("Syntax error") || err.message.contains("Incomplete input"),
        "{case_name}: unexpected message '{}'",
        err.message
    );

    let expected_snippet = source
        .lines()
        .nth(err.line.saturating_sub(1))
        .unwrap_or_default();
    assert_eq!(
        err.snippet, expected_snippet,
        "{case_name}: snippet should match source line"
    );
    assert!(
        err.pointer.contains('^'),
        "{case_name}: missing caret pointer"
    );
    assert_eq!(
        first_caret_column(&err.pointer),
        Some(err.column),
        "{case_name}: caret column mismatch"
    );
}

#[test]
fn reports_line_and_column_for_compile_errors() {
    let src = "scalar x;\nconstraint y == 1;";
    let err = compile_dsl(src).expect_err("compile should fail");
    assert_eq!(err.line, 2);
    assert_eq!(err.column, 12);
    assert_eq!(err.snippet, "constraint y == 1;");
    assert_eq!(first_caret_column(&err.pointer), Some(err.column));
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
fn reports_parse_errors_for_exhaustive_invalid_forms() {
    let cases = vec![
        ("missing system name", "system { export scalar x; }", 1usize),
        (
            "missing constraint_fn name",
            "constraint_fn (x: scalar) { constraint x == 1; }",
            1,
        ),
        (
            "missing colon in parameter",
            "constraint_fn bad(x scalar) { constraint x == 1; }",
            1,
        ),
        (
            "missing closing paren in parameter list",
            "constraint_fn bad(x: scalar { constraint x == 1; }",
            1,
        ),
        (
            "missing semicolon after declaration",
            "system s { export scalar x constraint x == 1; }",
            1,
        ),
        (
            "missing semicolon in constraint_fn body",
            "constraint_fn bad(x: scalar) { constraint x == 1 }",
            1,
        ),
        (
            "missing lhs expression",
            "system s { export scalar x; constraint == 1; }",
            1,
        ),
        (
            "missing rhs expression",
            "system s { export scalar x; constraint x == ; }",
            1,
        ),
        (
            "malformed binary rhs",
            "system s { export scalar x; constraint x == 1 + ; }",
            1,
        ),
        ("unclosed use call", "system s { use foo(1, 2; }", 1),
        ("trailing comma in use args", "system s { use foo(1,); }", 1),
        ("missing comma in use args", "system s { use foo(1 2); }", 1),
        (
            "missing closing bracket in vector literal",
            "system s { vec2d p; constraint p == [1, 2; }",
            1,
        ),
        (
            "invalid swizzle component",
            "system s { vec2d p; constraint p.w == 1; }",
            1,
        ),
        (
            "invalid identifier in declaration",
            "system s { export scalar 1x; }",
            1,
        ),
        ("unknown type token", "system s { vector x; }", 1),
        (
            "unmatched parenthesis in expression",
            "system s { export scalar x; constraint (x == 1; }",
            1,
        ),
        (
            "missing opening system brace",
            "system s export scalar x; }",
            1,
        ),
        (
            "missing closing system brace",
            "system s { export scalar x;",
            1,
        ),
        (
            "trailing garbage after valid system",
            "system s { export scalar x; } trailing",
            1,
        ),
        ("random garbage input", "@@@", 1),
        (
            "multiline missing rhs expression",
            "system s {\nexport scalar x;\nconstraint x == ;\n}",
            1,
        ),
        (
            "multiline missing closing vector bracket",
            "system s {\nvec2d p;\nconstraint p == [1, 2;\n}",
            1,
        ),
    ];

    for (case_name, source, expected_line) in cases {
        assert_parse_error_case(case_name, source, expected_line);
    }
}

#[test]
fn parses_top_level_imports() {
    let src = r#"
            import "constraints/common.dsl";
            import "../shared/math.dsl";
            scalar x;
            constraint x == 1;
        "#;
    let program = parse_dsl(src).expect("parse");
    assert_eq!(program.imports.len(), 2);
    assert_eq!(program.imports[0].path, "constraints/common.dsl");
    assert_eq!(program.imports[1].path, "../shared/math.dsl");
}

#[test]
fn rejects_imports_in_single_source_compile_api() {
    let src = r#"
            import "common.dsl";
            system s { export scalar x; constraint x == 1; }
        "#;
    let err = compile_dsl(src).expect_err("compile should fail");
    assert!(err.message.contains("Imports require project compilation"));
    assert_eq!(err.file, "<inline>");
    assert_eq!(err.line, 2);
    assert_eq!(err.column, 13);
}

#[test]
fn compiles_and_solves_multi_file_project() {
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

    let model = compile_dsl_project("systems/main.dsl", &sources).expect("compile");
    let result = model
        .solve(
            model
                .bootstrap_seed()
                .unknown_vec2d("p", Vec2d::new(0.0, 0.0)),
        )
        .expect("solve");
    let p = result.vec2d("p").expect("p");
    assert!((p.x - 3.0).abs() < 1e-8);
    assert!((p.y - 4.0).abs() < 1e-8);
}

#[test]
fn reports_missing_import_with_callsite_location() {
    let sources = vec![DslSource::new(
        "systems/main.dsl",
        r#"
            import "../constraints/missing.dsl";
            system s {
                export scalar x;
                constraint x == 1;
            }
            "#,
    )];

    let err = compile_dsl_project("systems/main.dsl", &sources).expect_err("compile should fail");
    assert!(err.message.contains("Failed to load import"));
    assert_eq!(err.file, "systems/main.dsl");
    assert_eq!(err.line, 2);
    assert_eq!(err.column, 13);
    assert!(err.snippet.contains("import"));
    assert_eq!(first_caret_column(&err.pointer), Some(err.column));
}

#[test]
fn reports_import_cycle_with_source_location() {
    let sources = vec![
        DslSource::new("a.dsl", r#"import "b.dsl";"#),
        DslSource::new("b.dsl", r#"import "a.dsl";"#),
    ];

    let err = compile_dsl_project("a.dsl", &sources).expect_err("compile should fail");
    assert!(err.message.contains("Import cycle detected"));
    assert_eq!(err.file, "b.dsl");
    assert_eq!(err.line, 1);
    assert_eq!(err.column, 1);
    assert_eq!(first_caret_column(&err.pointer), Some(err.column));
}

#[test]
fn reports_duplicate_constraint_fn_across_files() {
    let sources = vec![
        DslSource::new(
            "constraints/a.dsl",
            r#"
                constraint_fn c(v: scalar) {
                    constraint v == 0;
                }
                "#,
        ),
        DslSource::new(
            "constraints/b.dsl",
            r#"
                constraint_fn c(v: scalar) {
                    constraint v == 1;
                }
                "#,
        ),
        DslSource::new(
            "systems/main.dsl",
            r#"
                import "../constraints/a.dsl";
                import "../constraints/b.dsl";
                system s {
                    export scalar x;
                    use c(x);
                }
                "#,
        ),
    ];

    let err = compile_dsl_project("systems/main.dsl", &sources).expect_err("compile should fail");
    assert!(err.message.contains("Duplicate constraint_fn 'c'"));
    assert_eq!(err.file, "constraints/b.dsl");
    assert_eq!(err.line, 2);
    assert_eq!(err.column, 17);
}

#[test]
fn solve_trace_reports_origin_file_and_use_callsite() {
    let sources = vec![
        DslSource::new(
            "constraints/common.dsl",
            r#"
                constraint_fn contradictory(v: scalar) {
                    constraint v == 0;
                    constraint v == 1;
                }
                "#,
        ),
        DslSource::new(
            "systems/main.dsl",
            r#"
                import "../constraints/common.dsl";
                system traced {
                    export scalar x;
                    use contradictory(x);
                }
                "#,
        ),
    ];

    let model = compile_dsl_project("systems/main.dsl", &sources).expect("compile");
    let err = model
        .solve(model.bootstrap_seed().unknown_scalar("x", 0.5))
        .expect_err("solve should fail");
    let report = err
        .failure_report()
        .expect("expected structured failure report");

    assert!(!report.issues.is_empty());
    assert!(
        report
            .issues
            .iter()
            .any(|issue| issue.file == "constraints/common.dsl")
    );
    assert!(
        report
            .issues
            .iter()
            .any(|issue| issue.traceback.iter().any(|frame| {
                frame.function == "contradictory"
                    && frame.file == "systems/main.dsl"
                    && frame.line == 5
            }))
    );
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

#[test]
fn reports_overconstrained_inconsistency_with_trace() {
    let src = "system bad {\nexport scalar x;\nconstraint x == 0;\nconstraint x == 1;\n}";

    let model = compile_dsl(src).expect("compile");
    let err = model
        .solve(model.bootstrap_seed().unknown_scalar("x", 0.5))
        .expect_err("solve should fail");

    let report = err
        .failure_report()
        .expect("expected structured failure report");
    assert_eq!(report.kind, FailureKind::OverconstrainedInconsistency);
    assert!(report.overconstrained);
    assert!(report.equation_count > report.unknown_count);
    let first = report
        .issues
        .iter()
        .find(|issue| issue.line == 3)
        .expect("missing issue for line 3");
    assert_eq!(first.column, 17);
    assert_eq!(first.snippet, "constraint x == 0;");
    assert_eq!(first_caret_column(&first.pointer), Some(first.column));

    let second = report
        .issues
        .iter()
        .find(|issue| issue.line == 4)
        .expect("missing issue for line 4");
    assert_eq!(second.column, 17);
    assert_eq!(second.snippet, "constraint x == 1;");
    assert_eq!(first_caret_column(&second.pointer), Some(second.column));
}

#[test]
fn reports_non_convergence_with_trace() {
    let src = "system impossible {\nexport scalar x;\nconstraint exp(x) == -1;\n}";

    let model = compile_dsl(src).expect("compile");
    let err = model
        .solve(model.bootstrap_seed().unknown_scalar("x", 0.0))
        .expect_err("solve should fail");

    let report = err
        .failure_report()
        .expect("expected structured failure report");
    assert!(!report.overconstrained);
    assert!(matches!(
        report.kind,
        FailureKind::NonConvergence | FailureKind::SingularMatrix
    ));
    let issue = report
        .issues
        .iter()
        .find(|issue| issue.line == 3)
        .expect("missing issue for line 3");
    assert_eq!(issue.column, 22);
    assert_eq!(issue.snippet, "constraint exp(x) == -1;");
    assert_eq!(first_caret_column(&issue.pointer), Some(issue.column));
    assert!(issue.magnitude >= 0.0);
}

#[test]
fn reports_non_convergence_with_line_search_trace() {
    let src = "system impossible {\nexport scalar x;\nconstraint exp(x) == -1;\n}";

    let model = compile_dsl(src).expect("compile");
    let err = model
        .solve_with_line_search(model.bootstrap_seed().unknown_scalar("x", 0.0))
        .expect_err("line-search solve should fail");

    let report = err
        .failure_report()
        .expect("expected structured failure report");
    assert!(!report.overconstrained);
    assert!(matches!(
        report.kind,
        FailureKind::NonConvergence | FailureKind::SingularMatrix
    ));
    let issue = report
        .issues
        .iter()
        .find(|issue| issue.line == 3)
        .expect("missing issue for line 3");
    assert_eq!(issue.column, 22);
    assert_eq!(issue.snippet, "constraint exp(x) == -1;");
    assert_eq!(first_caret_column(&issue.pointer), Some(issue.column));
}

#[test]
fn failure_report_absent_on_non_failure_errors() {
    let src = r#"
            system s {
                export scalar x;
                local scalar hidden;
                constraint hidden == x + 1;
                constraint x == 2;
            }
        "#;

    let model = compile_dsl(src).expect("compile");
    let err = model
        .solve(model.bootstrap_seed().param_scalar("hidden", 12.0))
        .expect_err("solve should fail");
    assert!(matches!(err, SolveError::UnknownVariable(ref name) if name == "hidden"));
    assert!(err.failure_report().is_none());
}

#[test]
fn classifies_all_failure_kinds() {
    let src = r#"
            system s {
                export scalar x;
                constraint x == 0;
            }
        "#;
    let model = compile_dsl(src).expect("compile");

    let make_diag = |msg: &str| RawSolverRunDiagnostic {
        message: msg.to_string(),
        iterations: 7,
        error: 3.0,
        values: HashMap::new(),
        residuals: vec![3.0],
    };

    let singular = model.map_solver_error(RawSolverError::SingularMatrix(make_diag("singular")), 1);
    let singular = singular
        .failure_report()
        .expect("expected singular failure report");
    assert_eq!(singular.kind, FailureKind::SingularMatrix);

    let non_converged = model.map_solver_error(
        RawSolverError::NoConvergence(make_diag("no convergence")),
        1,
    );
    let non_converged = non_converged
        .failure_report()
        .expect("expected non-convergence report");
    assert_eq!(non_converged.kind, FailureKind::NonConvergence);

    let overconstrained = model.map_solver_error(
        RawSolverError::NoConvergence(make_diag("overconstrained")),
        0,
    );
    let overconstrained = overconstrained
        .failure_report()
        .expect("expected overconstrained report");
    assert_eq!(
        overconstrained.kind,
        FailureKind::OverconstrainedInconsistency
    );
}

#[test]
fn failure_issues_are_sorted_by_residual_magnitude() {
    let src = r#"
            system bad {
                export scalar x;
                constraint x == 0;
                constraint x == 2;
                constraint x == 10;
            }
        "#;

    let model = compile_dsl(src).expect("compile");
    let err = model
        .solve(model.bootstrap_seed().unknown_scalar("x", 0.0))
        .expect_err("solve should fail");
    let report = err
        .failure_report()
        .expect("expected structured failure report");
    assert!(!report.issues.is_empty());
    for pair in report.issues.windows(2) {
        assert!(pair[0].magnitude >= pair[1].magnitude);
    }
}

#[test]
fn trace_contains_system_and_use_context() {
    let src = "constraint_fn contradictory(v: scalar) {\nconstraint v == 0;\nconstraint v == 1;\n}\n\nsystem traced {\nexport scalar x;\nuse contradictory(x);\n}";

    let model = compile_dsl(src).expect("compile");
    let err = model
        .solve(model.bootstrap_seed().unknown_scalar("x", 0.5))
        .expect_err("solve should fail");
    let report = err
        .failure_report()
        .expect("expected structured failure report");
    assert!(!report.issues.is_empty());
    assert!(
        report
            .issues
            .iter()
            .any(|issue| issue.description.contains("system traced"))
    );
    assert!(
        report
            .issues
            .iter()
            .any(|issue| issue.description.contains("use contradictory"))
    );
    assert!(report.issues.iter().any(|issue| {
        (issue.line == 2 || issue.line == 3)
            && issue.column == 17
            && issue.snippet.starts_with("constraint v ==")
            && first_caret_column(&issue.pointer) == Some(issue.column)
    }));
}
