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

fn compile_stdlib_entry_system(entry_source: &str, system_name: &str) -> Model {
    let sources = vec![DslSource::new("systems/main.dsl", entry_source)];
    compile_dsl_project_system_with_stdlib("systems/main.dsl", system_name, &sources)
        .expect("compile")
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
    let src = "scalar x;\ny == 1;";
    let err = compile_dsl(src).expect_err("compile should fail");
    assert_eq!(err.line, 2);
    assert_eq!(err.column, 1);
    assert_eq!(err.snippet, "y == 1;");
    assert_eq!(first_caret_column(&err.pointer), Some(err.column));
    assert!(err.to_string().contains("Unknown identifier 'y'"));
}

#[test]
fn reports_parse_error_for_bad_parameter_syntax() {
    let src = "system bad(in scalar a out scalar b) { b == a; }";
    let err = parse_dsl(src).expect_err("parse should fail");
    assert_eq!(err.line, 1);
    assert!(err.column > 0);
    assert!(err.message.contains("Syntax error"));
    assert!(err.pointer.contains('^'));
}

#[test]
fn reports_parse_error_for_missing_semicolon() {
    let src = "system s(out scalar r) { scalar y r == y; }";
    let err = parse_dsl(src).expect_err("parse should fail");
    assert_eq!(err.line, 1);
    assert!(err.column > 0);
    assert!(err.message.contains("Syntax error"));
    assert!(err.pointer.contains('^'));
}

#[test]
fn reports_parse_error_for_unclosed_call() {
    let src = "system s(out scalar x) { foo(1, 2; }";
    let err = parse_dsl(src).expect_err("parse should fail");
    assert_eq!(err.line, 1);
    assert!(err.column > 0);
    assert!(err.message.contains("Syntax error"));
    assert!(err.pointer.contains('^'));
}

#[test]
fn reports_parse_errors_for_exhaustive_invalid_forms() {
    let cases = vec![
        (
            "missing system name",
            "system (out scalar x) { x == 1; }",
            1usize,
        ),
        (
            "missing comma in parameter list",
            "system s(in scalar a out scalar b) { b == a; }",
            1,
        ),
        (
            "unsupported constraint_fn keyword",
            "constraint_fn f(x: scalar) { x == 1; }",
            1,
        ),
        (
            "legacy constraint keyword",
            "system s(out scalar x) { constraint x == 1; }",
            1,
        ),
        (
            "unsupported use keyword",
            "system s(out scalar x) { use foo(x); }",
            1,
        ),
        (
            "unsupported export declaration keyword",
            "system s(out scalar x) { export scalar y; }",
            1,
        ),
        (
            "unsupported local declaration keyword",
            "system s(out scalar x) { local scalar y; }",
            1,
        ),
        (
            "missing closing paren in parameter list",
            "system s(in scalar a, out scalar b { b == a; }",
            1,
        ),
        (
            "missing semicolon after declaration",
            "system s(out scalar x) { scalar y x == y; }",
            1,
        ),
        (
            "missing lhs expression",
            "system s(out scalar x) { == 1; }",
            1,
        ),
        (
            "missing rhs expression",
            "system s(out scalar x) { x == ; }",
            1,
        ),
        (
            "malformed binary rhs",
            "system s(out scalar x) { x == 1 + ; }",
            1,
        ),
        ("unclosed call", "system s(out scalar x) { foo(1, 2; }", 1),
        (
            "trailing comma in call args",
            "system s(out scalar x) { foo(1,); }",
            1,
        ),
        (
            "missing comma in call args",
            "system s(out scalar x) { foo(1 2); }",
            1,
        ),
        (
            "missing closing bracket in vector literal",
            "system s(out scalar x) { vec2d p; p == [1, 2; }",
            1,
        ),
        (
            "invalid swizzle component",
            "system s(out scalar x) { vec2d p; p.w == 1; }",
            1,
        ),
        (
            "invalid identifier in declaration",
            "system s(out scalar x) { scalar 1x; }",
            1,
        ),
        (
            "unknown type token",
            "system s(out scalar x) { vector y; }",
            1,
        ),
        (
            "unmatched parenthesis in expression",
            "system s(out scalar x) { (x == 1; }",
            1,
        ),
        (
            "missing opening system brace",
            "system s(out scalar x) x == 1; }",
            1,
        ),
        (
            "missing closing system brace",
            "system s(out scalar x) { scalar y;",
            1,
        ),
        (
            "trailing garbage after valid system",
            "system s(out scalar x) { x == 1; } trailing",
            1,
        ),
        ("random garbage input", "@@@", 1),
        (
            "multiline missing rhs expression",
            "system s(out scalar x) {\nx == ;\n}",
            1,
        ),
        (
            "multiline missing closing vector bracket",
            "system s(out scalar x) {\nvec2d p;\np == [1, 2;\n}",
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
            x == 1;
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
            system s(out scalar x) { x == 1; }
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
                system pin(out vec2d p) {
                    p == [3, 4];
                }
                "#,
        ),
        DslSource::new(
            "systems/main.dsl",
            r#"
                import "../constraints/2d.dsl";
                system s(inout vec2d p) {
                    pin(p);
                }
                "#,
        ),
    ];

    let model = compile_dsl_project_system("systems/main.dsl", "s", &sources).expect("compile");
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
            system s(out scalar x) {
                x == 1;
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
fn reports_duplicate_system_across_files() {
    let sources = vec![
        DslSource::new(
            "constraints/a.dsl",
            r#"
                system c(inout scalar v) {
                    v == 0;
                }
                "#,
        ),
        DslSource::new(
            "constraints/b.dsl",
            r#"
                system c(inout scalar v) {
                    v == 1;
                }
                "#,
        ),
        DslSource::new(
            "systems/main.dsl",
            r#"
                import "../constraints/a.dsl";
                import "../constraints/b.dsl";
                system main(out scalar x) {
                    c(x);
                }
                "#,
        ),
    ];

    let err = compile_dsl_project("systems/main.dsl", &sources).expect_err("compile should fail");
    assert!(err.message.contains("Duplicate system 'c'"));
    assert_eq!(err.file, "constraints/b.dsl");
    assert_eq!(err.line, 2);
    assert_eq!(err.column, 17);
}

#[test]
fn solve_trace_reports_origin_file_and_callsite() {
    let sources = vec![
        DslSource::new(
            "constraints/common.dsl",
            r#"
                system contradictory(inout scalar v) {
                    v == 0;
                    v == 1;
                }
                "#,
        ),
        DslSource::new(
            "systems/main.dsl",
            r#"
                import "../constraints/common.dsl";
                system traced(inout scalar x) {
                    contradictory(x);
                }
                "#,
        ),
    ];

    let model =
        compile_dsl_project_system("systems/main.dsl", "traced", &sources).expect("compile");
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
                    && frame.line >= 4
            }))
    );
}

#[test]
fn supports_hash_comments() {
    let src = r#"
            # top-level comment
            system s(out scalar x) {
                scalar temp; # inline comment
                temp == x; # another comment
                x == 2; # another comment
            }
        "#;

    let model = compile_dsl_system(src, "s").expect("compile");
    let result = model.solve(model.bootstrap_seed()).expect("solve");
    let x = result.scalar("x").expect("x");
    assert!((x - 2.0).abs() < 1e-8);
}

#[test]
fn solves_simple_vec2_constraints() {
    let src = "vec2d p;\np == [3.0, 4.0];";
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
            n == [0.0, 0.0, 1.0];
            dot3(n, [0.0, 0.0, 1.0]) == 1.0;
            length3(n) == 1.0;
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
fn solves_composed_system_with_parameters_and_locals() {
    let src = r#"
            system tangent_to_line(in vec2d center, in scalar r, in vec2d n, in scalar d, in scalar side) {
                dot2(n, center) + d == side * r;
            }

            system equal_radius_tangent(in vec2d c1, in vec2d c2, in scalar r) {
                dot2(c2 - c1, c2 - c1) == (2 * r) * (2 * r);
            }

            system circles(inout vec2d c1, inout vec2d c2, inout scalar r) {
                vec2d n = [0, 1];

                tangent_to_line(c1, r, n, 0, 1);
                tangent_to_line(c1, r, n, -10, -1);
                tangent_to_line(c2, r, n, 0, 1);
                tangent_to_line(c2, r, n, -10, -1);
                equal_radius_tangent(c1, c2, r);
                c1.x == 0;
            }
        "#;

    let model = compile_dsl_system(src, "circles").expect("compile");
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
            system s(out scalar x) {
                scalar hidden;
                hidden == x + 1;
                x == 2;
            }
        "#;

    let model = compile_dsl(src).expect("compile");
    let seed = model.bootstrap_seed().param_scalar("hidden", 12.0);
    let err = model.solve(seed).expect_err("solve should fail");
    assert!(matches!(err, SolveError::UnknownVariable(name) if name == "hidden"));
}

#[test]
fn reports_overconstrained_inconsistency_with_trace() {
    let src = "system bad(out scalar x) {\nx == 0;\nx == 1;\n}";

    let model = compile_dsl(src).expect("compile");
    let err = model
        .solve(model.bootstrap_seed())
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
        .find(|issue| issue.line == 2)
        .expect("missing issue for line 2");
    assert_eq!(first.column, 6);
    assert_eq!(first.snippet, "x == 0;");
    assert_eq!(first_caret_column(&first.pointer), Some(first.column));

    let second = report
        .issues
        .iter()
        .find(|issue| issue.line == 3)
        .expect("missing issue for line 3");
    assert_eq!(second.column, 6);
    assert_eq!(second.snippet, "x == 1;");
    assert_eq!(first_caret_column(&second.pointer), Some(second.column));
}

#[test]
fn reports_non_convergence_with_trace() {
    let src = "system impossible(out scalar x) {\nexp(x) == -1;\n}";

    let model = compile_dsl(src).expect("compile");
    let err = model
        .solve(model.bootstrap_seed())
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
        .find(|issue| issue.line == 2)
        .expect("missing issue for line 2");
    assert_eq!(issue.column, 11);
    assert_eq!(issue.snippet, "exp(x) == -1;");
    assert_eq!(first_caret_column(&issue.pointer), Some(issue.column));
    assert!(issue.magnitude >= 0.0);
}

#[test]
fn reports_non_convergence_with_line_search_trace() {
    let src = "system impossible(out scalar x) {\nexp(x) == -1;\n}";

    let model = compile_dsl(src).expect("compile");
    let err = model
        .solve_with_line_search(model.bootstrap_seed())
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
        .find(|issue| issue.line == 2)
        .expect("missing issue for line 2");
    assert_eq!(issue.column, 11);
    assert_eq!(issue.snippet, "exp(x) == -1;");
    assert_eq!(first_caret_column(&issue.pointer), Some(issue.column));
}

#[test]
fn failure_report_absent_on_non_failure_errors() {
    let src = r#"
            system s(out scalar x) {
                scalar hidden;
                hidden == x + 1;
                x == 2;
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
            system s(out scalar x) {
                x == 0;
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
            system bad(out scalar x) {
                x == 0;
                x == 2;
                x == 10;
            }
        "#;

    let model = compile_dsl(src).expect("compile");
    let err = model
        .solve(model.bootstrap_seed())
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
fn trace_contains_system_and_call_context() {
    let src = "system contradictory(inout scalar v) {\nv == 0;\nv == 1;\n}\n\nsystem traced(inout scalar x) {\ncontradictory(x);\n}";

    let model = compile_dsl_system(src, "traced").expect("compile");
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
            .any(|issue| issue.description.contains("call contradictory"))
    );
    assert!(report.issues.iter().any(|issue| {
        (issue.line == 2 || issue.line == 3)
            && issue.column == 6
            && issue.snippet.starts_with("v ==")
            && first_caret_column(&issue.pointer) == Some(issue.column)
    }));
}

#[test]
fn parses_system_parameters_with_directions() {
    let src = r#"
        system helper(in scalar a, out vec2d p, inout scalar k) {
            a == k;
            p == [a, k];
        }
    "#;
    let program = parse_dsl(src).expect("parse");
    let system = program.systems.first().expect("system");
    assert_eq!(system.params.len(), 3);
    assert_eq!(system.params[0].mode, ParamMode::In);
    assert_eq!(system.params[1].mode, ParamMode::Out);
    assert_eq!(system.params[2].mode, ParamMode::InOut);
}

#[test]
fn solves_parameterized_system_call_without_use_keyword() {
    let src = r#"
        system pin(in scalar target, out scalar x) {
            x == target;
        }

        system main(out scalar v) {
            pin(5, v);
        }
    "#;

    let model = compile_dsl_system(src, "main").expect("compile");
    let result = model.solve(model.bootstrap_seed()).expect("solve");
    let v = result.scalar("v").expect("v");
    assert!((v - 5.0).abs() < 1e-8);
}

#[test]
fn rejects_setting_out_parameter_from_public_seed() {
    let src = r#"
        system s(out scalar x) {
            x == 1;
        }
    "#;
    let model = compile_dsl(src).expect("compile");
    let err = model
        .solve(model.bootstrap_seed().param_scalar("x", 3.0))
        .expect_err("solve should fail");
    assert!(matches!(err, SolveError::UnknownVariable(ref name) if name == "x"));
}

#[test]
fn rejects_non_identifier_out_argument_for_system_call() {
    let src = r#"
        system producer(out scalar x) {
            x == 1;
        }

        system main(out scalar y) {
            producer(1 + 2);
            y == 0;
        }
    "#;
    let err = compile_dsl_system(src, "main").expect_err("compile should fail");
    assert!(err.message.contains("must be a variable identifier"));
}

#[test]
fn rejects_recursive_system_calls() {
    let src = r#"
        system rec(inout scalar x) {
            rec(x);
        }

        system main(out scalar x) {
            rec(x);
            x == 1;
        }
    "#;
    let err = compile_dsl_system(src, "main").expect_err("compile should fail");
    assert!(
        err.message
            .contains("Recursive callable invocation is not supported")
    );
}

#[test]
fn provides_embedded_standard_library_sources() {
    let sources = standard_library_sources();
    assert!(sources.iter().any(|src| src.path == STDLIB_STD2D_PATH));
    assert!(sources.iter().any(|src| src.path == STDLIB_STD3D_PATH));
    assert!(
        sources
            .iter()
            .find(|src| src.path == STDLIB_STD2D_PATH)
            .map(|src| src.source.contains("system c2_distance_pp"))
            .unwrap_or(false)
    );
    assert!(
        sources
            .iter()
            .find(|src| src.path == STDLIB_STD3D_PATH)
            .map(|src| src.source.contains("system c3_distance_pp"))
            .unwrap_or(false)
    );
}

#[test]
fn solves_project_with_embedded_std2d_library() {
    let sources = vec![DslSource::new(
        "systems/main.dsl",
        r#"
            import "../stdlib/std2d.dsl";

            system main(inout vec2d a, inout vec2d b, in scalar d) {
                c2_distance_pp(a, b, d);
                a == [0, 0];
                b.y == 0;
            }
        "#,
    )];

    let model = compile_dsl_project_system_with_stdlib("systems/main.dsl", "main", &sources)
        .expect("compile");
    let seed = model
        .bootstrap_seed()
        .unknown_vec2d("a", Vec2d::new(0.0, 0.0))
        .unknown_vec2d("b", Vec2d::new(4.0, 0.1))
        .param_scalar("d", 5.0);
    let result = model.solve(seed).expect("solve");
    let b = result.vec2d("b").expect("b");
    assert!((b.x - 5.0).abs() < 1e-6);
    assert!(b.y.abs() < 1e-6);
}

#[test]
fn solves_project_with_embedded_std3d_library() {
    let sources = vec![DslSource::new(
        "systems/main.dsl",
        r#"
            import "../stdlib/std3d.dsl";

            system main(inout vec3d p) {
                c3_point_on_plane(p, [0, 0, 0], [0, 0, 1]);
                p.x == 2;
                p.y == -1;
            }
        "#,
    )];

    let model = compile_dsl_project_system_with_stdlib("systems/main.dsl", "main", &sources)
        .expect("compile");
    let seed = model
        .bootstrap_seed()
        .unknown_vec3d("p", Vec3d::new(2.1, -1.2, 0.8));
    let result = model.solve(seed).expect("solve");
    let p = result.vec3d("p").expect("p");
    assert!((p.x - 2.0).abs() < 1e-8);
    assert!((p.y + 1.0).abs() < 1e-8);
    assert!(p.z.abs() < 1e-8);
}

#[test]
fn std2d_simple_point_on_circle() {
    let src = r#"
        import "../stdlib/std2d.dsl";

        system main(inout vec2d p, in vec2d c, in scalar r) {
            c2_point_on_circle(p, c, r);
            p.y == 0;
        }
    "#;
    let model = compile_stdlib_entry_system(src, "main");
    let seed = model
        .bootstrap_seed()
        .unknown_vec2d("p", Vec2d::new(4.9, 0.2))
        .param_vec2d("c", Vec2d::new(0.0, 0.0))
        .param_scalar("r", 5.0);
    let result = model.solve(seed).expect("solve");
    let p = result.vec2d("p").expect("p");
    assert!((p.x - 5.0).abs() < 1e-6);
    assert!(p.y.abs() < 1e-6);
}

#[test]
fn std2d_simple_diameter_constraint() {
    let src = r#"
        import "../stdlib/std2d.dsl";

        system main(inout vec2d a, inout vec2d b) {
            a == [0, 0];
            c2_horizontal(a, b);
            c2_diameter(a, b, 10);
        }
    "#;
    let model = compile_stdlib_entry_system(src, "main");
    let seed = model
        .bootstrap_seed()
        .unknown_vec2d("a", Vec2d::new(0.1, -0.2))
        .unknown_vec2d("b", Vec2d::new(9.7, 0.2));
    let result = model.solve(seed).expect("solve");
    let b = result.vec2d("b").expect("b");
    assert!((b.x - 10.0).abs() < 1e-6);
    assert!(b.y.abs() < 1e-6);
}

#[test]
fn std2d_simple_line_circle_tangent() {
    let src = r#"
        import "../stdlib/std2d.dsl";

        system main(inout vec2d center, in scalar r) {
            c2_tangent_line_circle([0, 0], [1, 0], center, r, 1);
            center.x == 3;
        }
    "#;
    let model = compile_stdlib_entry_system(src, "main");
    let seed = model
        .bootstrap_seed()
        .unknown_vec2d("center", Vec2d::new(2.8, 1.7))
        .param_scalar("r", 2.0);
    let result = model.solve(seed).expect("solve");
    let c = result.vec2d("center").expect("center");
    assert!((c.x - 3.0).abs() < 1e-6);
    assert!((c.y - 2.0).abs() < 1e-6);
}

#[test]
fn std2d_simple_circle_circle_intersection() {
    let src = r#"
        import "../stdlib/std2d.dsl";

        system main(inout vec2d p) {
            c2_point_on_circle(p, [0, 0], 5);
            c2_point_on_circle(p, [8, 0], 5);
        }
    "#;
    let model = compile_stdlib_entry_system(src, "main");
    let seed = model
        .bootstrap_seed()
        .unknown_vec2d("p", Vec2d::new(4.2, 2.8));
    let result = model.solve(seed).expect("solve");
    let p = result.vec2d("p").expect("p");
    assert!((p.x - 4.0).abs() < 1e-6);
    assert!((p.y - 3.0).abs() < 1e-6);
}

#[test]
fn std2d_medium_hdist_vdist_and_equal_length() {
    let src = r#"
        import "../stdlib/std2d.dsl";

        system main(inout vec2d a, inout vec2d b, inout vec2d c, inout vec2d d) {
            a == [0, 0];
            c2_hdist(a, b, 4);
            c2_vdist(a, b, 3);

            c == [1, 1];
            c2_horizontal(c, d);
            c2_equal_length(a, b, c, d);
        }
    "#;
    let model = compile_stdlib_entry_system(src, "main");
    let seed = model
        .bootstrap_seed()
        .unknown_vec2d("a", Vec2d::new(0.2, -0.1))
        .unknown_vec2d("b", Vec2d::new(3.7, 3.3))
        .unknown_vec2d("c", Vec2d::new(1.1, 1.1))
        .unknown_vec2d("d", Vec2d::new(5.8, 0.9));
    let result = model.solve(seed).expect("solve");
    let b = result.vec2d("b").expect("b");
    let d = result.vec2d("d").expect("d");
    assert!((b.x - 4.0).abs() < 1e-6);
    assert!((b.y - 3.0).abs() < 1e-6);
    assert!((d.x - 6.0).abs() < 1e-6);
    assert!((d.y - 1.0).abs() < 1e-6);
}

#[test]
fn std2d_medium_midpoint_collinear_and_parallel_perpendicular() {
    let src = r#"
        import "../stdlib/std2d.dsl";

        system main(
            inout vec2d a,
            inout vec2d b,
            inout vec2d m,
            inout vec2d p,
            inout vec2d b1
        ) {
            a == [0, 0];
            b == [10, 0];
            c2_midpoint(m, a, b);
            c2_collinear(p, a, b);
            p.x == 7;

            c2_parallel([0, 0], [2, 0], [1, 1], b1);
            c2_perpendicular([1, 1], b1, [0, 0], [0, 1]);
            c2_distance_pp([1, 1], b1, 4);
        }
    "#;
    let model = compile_stdlib_entry_system(src, "main");
    let seed = model
        .bootstrap_seed()
        .unknown_vec2d("a", Vec2d::new(-0.1, 0.1))
        .unknown_vec2d("b", Vec2d::new(9.7, -0.2))
        .unknown_vec2d("m", Vec2d::new(4.5, 0.2))
        .unknown_vec2d("p", Vec2d::new(7.0, 0.5))
        .unknown_vec2d("b1", Vec2d::new(5.1, 1.2));
    let result = model.solve(seed).expect("solve");
    let m = result.vec2d("m").expect("m");
    let p = result.vec2d("p").expect("p");
    let b1 = result.vec2d("b1").expect("b1");
    assert!((m.x - 5.0).abs() < 1e-6);
    assert!(m.y.abs() < 1e-6);
    assert!((p.x - 7.0).abs() < 1e-6);
    assert!(p.y.abs() < 1e-6);
    assert!((b1.x - 5.0).abs() < 1e-6);
    assert!((b1.y - 1.0).abs() < 1e-6);
}

#[test]
fn std2d_medium_angle_equal_angle_ratio_diff_concentric_radius() {
    let src = r#"
        import "../stdlib/std2d.dsl";

        system main(inout vec2d q, inout vec2d r, inout vec2d p) {
            c2_angle_ll([0, 0], [1, 0], [0, 0], q, 1.57079632679);
            c2_distance_pp([0, 0], q, 2);

            c2_equal_angle([0, 0], [1, 0], [0, 1], [0, 0], [2, 0], r);
            c2_distance_pp([0, 0], r, 2);

            c2_length_ratio([0, 0], [6, 0], [0, 1], [3, 1], 2);
            c2_length_diff([0, 0], [6, 0], [0, 1], [3, 1], 3);

            c2_concentric([1, 2], [1, 2]);
            c2_radius([1, 2], p, 3);
            c2_point_on_circle(p, [1, 2], 3);
            p.y == 2;
        }
    "#;
    let model = compile_stdlib_entry_system(src, "main");
    let seed = model
        .bootstrap_seed()
        .unknown_vec2d("q", Vec2d::new(0.2, 1.9))
        .unknown_vec2d("r", Vec2d::new(-0.1, 2.1))
        .unknown_vec2d("p", Vec2d::new(3.8, 2.2));
    let result = model.solve(seed).expect("solve");
    let q = result.vec2d("q").expect("q");
    let r = result.vec2d("r").expect("r");
    let p = result.vec2d("p").expect("p");
    assert!(q.x.abs() < 1e-6);
    assert!((q.y - 2.0).abs() < 1e-6);
    assert!(r.x.abs() < 1e-6);
    assert!((r.y - 2.0).abs() < 1e-6);
    assert!((p.x - 4.0).abs() < 1e-6);
    assert!((p.y - 2.0).abs() < 1e-6);
}

#[test]
fn std2d_high_two_equal_circles_tangent_to_two_lines_and_each_other() {
    let src = r#"
        import "../stdlib/std2d.dsl";

        system main(inout vec2d c1, inout vec2d c2, inout scalar r) {
            c2_tangent_line_circle([0, 0], [1, 0], c1, r, 1);
            c2_tangent_line_circle([0, 10], [1, 10], c1, r, -1);
            c2_tangent_line_circle([0, 0], [1, 0], c2, r, 1);
            c2_tangent_line_circle([0, 10], [1, 10], c2, r, -1);
            c2_tangent_circle_circle(c1, r, c2, r, 1);
            c1.x == 0;
        }
    "#;
    let model = compile_stdlib_entry_system(src, "main");
    let seed = model
        .bootstrap_seed()
        .unknown_vec2d("c1", Vec2d::new(0.0, 4.4))
        .unknown_vec2d("c2", Vec2d::new(9.0, 5.8))
        .unknown_scalar("r", 4.2);
    let result = model.solve_with_line_search(seed).expect("solve");
    let c1 = result.vec2d("c1").expect("c1");
    let c2 = result.vec2d("c2").expect("c2");
    let r = result.scalar("r").expect("r");
    assert!((r - 5.0).abs() < 1e-6);
    assert!((c1.y - 5.0).abs() < 1e-6);
    assert!((c2.y - 5.0).abs() < 1e-6);
    assert!(c1.x.abs() < 1e-6);
    assert!((c2.x - 10.0).abs() < 1e-5);
}

#[test]
fn std2d_high_circle_intersection_pair_with_symmetry() {
    let src = r#"
        import "../stdlib/std2d.dsl";

        system main(inout vec2d p1, inout vec2d p2) {
            c2_point_on_circle(p1, [0, 0], 5);
            c2_point_on_circle(p1, [8, 0], 5);
            c2_point_on_circle(p2, [0, 0], 5);
            c2_point_on_circle(p2, [8, 0], 5);
            c2_symmetric_about_line(p1, p2, [0, 0], [1, 0]);
        }
    "#;
    let model = compile_stdlib_entry_system(src, "main");
    let seed = model
        .bootstrap_seed()
        .unknown_vec2d("p1", Vec2d::new(4.0, 2.9))
        .unknown_vec2d("p2", Vec2d::new(4.0, -2.7));
    let result = model.solve(seed).expect("solve");
    let p1 = result.vec2d("p1").expect("p1");
    let p2 = result.vec2d("p2").expect("p2");
    assert!((p1.x - 4.0).abs() < 1e-6);
    assert!((p2.x - 4.0).abs() < 1e-6);
    assert!((p1.y + p2.y).abs() < 1e-6);
    assert!((p1.y.abs() - 3.0).abs() < 1e-6);
}

#[test]
fn std3d_simple_point_on_line_and_plane_signed_distance() {
    let src = r#"
        import "../stdlib/std3d.dsl";

        system main(inout vec3d p, inout vec3d q) {
            c3_point_on_line(p, [0, 0, 0], [1, 0, 0]);
            c3_distance_pp([0, 0, 0], p, 5);

            c3_distance_point_plane_signed(q, [0, 0, 0], [0, 0, 1], 2);
            q.x == 1;
            q.y == -3;
        }
    "#;
    let model = compile_stdlib_entry_system(src, "main");
    let seed = model
        .bootstrap_seed()
        .unknown_vec3d("p", Vec3d::new(4.6, 0.4, -0.3))
        .unknown_vec3d("q", Vec3d::new(1.2, -2.8, 1.8));
    let result = model.solve(seed).expect("solve");
    let p = result.vec3d("p").expect("p");
    let q = result.vec3d("q").expect("q");
    assert!((p.x - 5.0).abs() < 1e-6);
    assert!(p.y.abs() < 1e-6);
    assert!(p.z.abs() < 1e-6);
    assert!((q.x - 1.0).abs() < 1e-6);
    assert!((q.y + 3.0).abs() < 1e-6);
    assert!((q.z - 2.0).abs() < 1e-6);
}

#[test]
fn std3d_medium_parallel_perpendicular_and_angle_constraints() {
    let src = r#"
        import "../stdlib/std3d.dsl";

        system main(inout vec3d b1, inout vec3d c1, inout vec3d v1, inout vec3d l1) {
            c3_parallel_lines([0, 0, 0], [1, 0, 0], [0, 1, 0], b1);
            c3_perpendicular_lines([0, 0, 0], [1, 0, 0], [0, 0, 0], c1);
            c3_distance_pp([0, 1, 0], b1, 4);
            c3_distance_pp([0, 0, 0], c1, 3);
            c1.y == 0;

            c3_angle_ll([0, 0, 0], [1, 0, 0], [0, 0, 0], v1, 1.57079632679);
            c3_distance_pp([0, 0, 0], v1, 2);
            v1.z == 0;

            c3_angle_line_plane([0, 0, 0], l1, [0, 0, 1], 0);
            c3_distance_pp([0, 0, 0], l1, 3);
            l1.y == 0;
        }
    "#;
    let model = compile_stdlib_entry_system(src, "main");
    let seed = model
        .bootstrap_seed()
        .unknown_vec3d("b1", Vec3d::new(4.7, 1.2, 0.2))
        .unknown_vec3d("c1", Vec3d::new(0.1, 0.2, 2.9))
        .unknown_vec3d("v1", Vec3d::new(0.2, 1.8, 0.3))
        .unknown_vec3d("l1", Vec3d::new(2.9, 0.2, 0.4));
    let result = model.solve(seed).expect("solve");
    let b1 = result.vec3d("b1").expect("b1");
    let c1 = result.vec3d("c1").expect("c1");
    let v1 = result.vec3d("v1").expect("v1");
    let l1 = result.vec3d("l1").expect("l1");
    assert!((b1.x - 4.0).abs() < 1e-6);
    assert!((b1.y - 1.0).abs() < 1e-6);
    assert!(b1.z.abs() < 1e-6);
    assert!(c1.x.abs() < 1e-6);
    assert!(c1.y.abs() < 1e-6);
    assert!((c1.z - 3.0).abs() < 1e-6);
    assert!(v1.x.abs() < 1e-6);
    assert!((v1.y - 2.0).abs() < 1e-6);
    assert!(v1.z.abs() < 1e-6);
    assert!((l1.x - 3.0).abs() < 1e-6);
    assert!(l1.y.abs() < 1e-6);
    assert!(l1.z.abs() < 1e-6);
}

#[test]
fn std3d_medium_tangent_and_distance_constraints() {
    let src = r#"
        import "../stdlib/std3d.dsl";

        system main(inout vec3d c1, inout vec3d c2, inout vec3d p) {
            c3_tangent_sphere_plane(c1, 2, [0, 0, 0], [0, 0, 1], 1);
            c3_tangent_sphere_sphere(c1, 2, c2, 3, 1);
            c1 == [0, 0, 2];
            c2.y == 0;
            c2.z == 2;

            c3_distance_point_line(p, [0, 0, 0], [1, 0, 0], 5);
            p.x == 2;
            p.y == 4;
        }
    "#;
    let model = compile_stdlib_entry_system(src, "main");
    let seed = model
        .bootstrap_seed()
        .unknown_vec3d("c1", Vec3d::new(0.2, -0.1, 2.1))
        .unknown_vec3d("c2", Vec3d::new(4.8, 0.3, 2.2))
        .unknown_vec3d("p", Vec3d::new(2.1, 3.9, 2.8));
    let result = model.solve(seed).expect("solve");
    let c2 = result.vec3d("c2").expect("c2");
    let p = result.vec3d("p").expect("p");
    assert!((c2.x - 5.0).abs() < 1e-6);
    assert!(c2.y.abs() < 1e-6);
    assert!((c2.z - 2.0).abs() < 1e-6);
    assert!((p.x - 2.0).abs() < 1e-6);
    assert!((p.y - 4.0).abs() < 1e-6);
    assert!((p.z - 3.0).abs() < 1e-6);
}

#[test]
fn std3d_high_line_line_and_plane_plane_and_sphere_constraints() {
    let src = r#"
        import "../stdlib/std3d.dsl";

        system main(
            inout vec3d b0,
            inout vec3d b1,
            inout vec3d p2,
            inout vec3d n2,
            inout vec3d c2,
            inout scalar r1,
            inout scalar r2
        ) {
            c3_distance_line_line([0, 0, 0], [1, 0, 0], b0, b1, 3);
            c3_parallel_lines([0, 0, 0], [1, 0, 0], b0, b1);
            b0.x == 0;
            b1.x == 1;
            b0.z == 0;
            b1.z == 0;

            c3_distance_plane_plane([0, 0, 0], [0, 0, 1], p2, n2, 4);
            c3_parallel_planes([0, 0, 1], n2);
            c3_perpendicular_planes([0, 0, 1], [1, 0, 0]);
            p2.x == 0;
            p2.y == 0;

            c3_concentric_spheres([1, 2, 3], c2);
            c3_equal_radius(r1, r2);
            r1 == 2;
            c3_point_on_sphere([3, 2, 3], c2, r2);
        }
    "#;
    let model = compile_stdlib_entry_system(src, "main");
    let seed = model
        .bootstrap_seed()
        .unknown_vec3d("b0", Vec3d::new(0.1, 2.8, 0.1))
        .unknown_vec3d("b1", Vec3d::new(0.9, 3.1, -0.2))
        .unknown_vec3d("p2", Vec3d::new(0.2, -0.1, 3.8))
        .unknown_vec3d("n2", Vec3d::new(0.0, 0.0, 0.7))
        .unknown_vec3d("c2", Vec3d::new(1.1, 2.1, 2.8))
        .unknown_scalar("r1", 2.2)
        .unknown_scalar("r2", 1.9);
    let result = model.solve_with_line_search(seed).expect("solve");
    let b0 = result.vec3d("b0").expect("b0");
    let b1 = result.vec3d("b1").expect("b1");
    let p2 = result.vec3d("p2").expect("p2");
    let c2 = result.vec3d("c2").expect("c2");
    let r1 = result.scalar("r1").expect("r1");
    let r2 = result.scalar("r2").expect("r2");
    assert!((b0.y - 3.0).abs() < 1e-5);
    assert!((b1.y - 3.0).abs() < 1e-5);
    assert!(b0.z.abs() < 1e-6);
    assert!(b1.z.abs() < 1e-6);
    assert!((p2.z - 4.0).abs() < 1e-5);
    assert!((c2.x - 1.0).abs() < 1e-6);
    assert!((c2.y - 2.0).abs() < 1e-6);
    assert!((c2.z - 3.0).abs() < 1e-6);
    assert!((r1 - 2.0).abs() < 1e-6);
    assert!((r2 - 2.0).abs() < 1e-6);
}
