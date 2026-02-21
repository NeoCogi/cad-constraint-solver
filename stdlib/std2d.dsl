# ============================================================================
# std2d.dsl
#
# 2D geometric constraint standard library for cad-constraint-solver.
#
# Design goals:
# - Use only equation constraints that map cleanly to scalar residuals.
# - Prefer squared-distance forms where practical for better numeric behavior.
# - Keep constraints composable: each system focuses on one geometric relation.
# - Use explicit branch selectors (`side`, `mode`) for ambiguous constraints.
#
# Parameter convention:
# - All parameters are `in` because systems are read-only relation templates.
# - Passing variables still works: variables remain unknowns in caller context.
#
# Branch convention:
# - `side`: +1 / -1, selects one side of an oriented line.
# - `mode`: +1 for external tangency, -1 for internal tangency.
# ============================================================================

# ----------------------------------------------------------------------------
# c2_coincident
# Enforces two points to be identical.
# ----------------------------------------------------------------------------
system c2_coincident(in vec2d a, in vec2d b) {
    a == b;
}

# ----------------------------------------------------------------------------
# c2_horizontal
# Constrains segment (a,b) to be horizontal.
# ----------------------------------------------------------------------------
system c2_horizontal(in vec2d a, in vec2d b) {
    a.y == b.y;
}

# ----------------------------------------------------------------------------
# c2_vertical
# Constrains segment (a,b) to be vertical.
# ----------------------------------------------------------------------------
system c2_vertical(in vec2d a, in vec2d b) {
    a.x == b.x;
}

# ----------------------------------------------------------------------------
# c2_distance_pp
# Point-point distance constraint.
# Uses squared-distance form: |b-a|^2 = dist^2.
# ----------------------------------------------------------------------------
system c2_distance_pp(in vec2d a, in vec2d b, in scalar dist) {
    dot2(b - a, b - a) == dist * dist;
}

# ----------------------------------------------------------------------------
# c2_hdist
# Signed horizontal distance constraint: b.x - a.x = dx.
# ----------------------------------------------------------------------------
system c2_hdist(in vec2d a, in vec2d b, in scalar dx) {
    b.x - a.x == dx;
}

# ----------------------------------------------------------------------------
# c2_vdist
# Signed vertical distance constraint: b.y - a.y = dy.
# ----------------------------------------------------------------------------
system c2_vdist(in vec2d a, in vec2d b, in scalar dy) {
    b.y - a.y == dy;
}

# ----------------------------------------------------------------------------
# c2_midpoint
# Enforces m to be the midpoint of segment (a,b).
# ----------------------------------------------------------------------------
system c2_midpoint(in vec2d m, in vec2d a, in vec2d b) {
    2 * m == a + b;
}

# ----------------------------------------------------------------------------
# c2_point_on_line
# Constrains p on infinite line through (l0,l1).
# Formula: cross(l1-l0, p-l0) = 0.
# ----------------------------------------------------------------------------
system c2_point_on_line(in vec2d p, in vec2d l0, in vec2d l1) {
    (l1.x - l0.x) * (p.y - l0.y) - (l1.y - l0.y) * (p.x - l0.x) == 0;
}

# ----------------------------------------------------------------------------
# c2_collinear
# Enforces points (a,b,c) to be collinear.
# ----------------------------------------------------------------------------
system c2_collinear(in vec2d a, in vec2d b, in vec2d c) {
    c2_point_on_line(c, a, b);
}

# ----------------------------------------------------------------------------
# c2_parallel
# Enforces line (a0,a1) parallel to line (b0,b1).
# Formula: cross(d1,d2) = 0.
# ----------------------------------------------------------------------------
system c2_parallel(in vec2d a0, in vec2d a1, in vec2d b0, in vec2d b1) {
    (a1.x - a0.x) * (b1.y - b0.y) - (a1.y - a0.y) * (b1.x - b0.x) == 0;
}

# ----------------------------------------------------------------------------
# c2_perpendicular
# Enforces line (a0,a1) perpendicular to line (b0,b1).
# Formula: dot(d1,d2) = 0.
# ----------------------------------------------------------------------------
system c2_perpendicular(in vec2d a0, in vec2d a1, in vec2d b0, in vec2d b1) {
    dot2(a1 - a0, b1 - b0) == 0;
}

# ----------------------------------------------------------------------------
# c2_angle_ll
# Constrains angle between two lines to `angle_rad` (radians).
# Formula: dot(u,v) = cos(theta) * |u| * |v|.
# ----------------------------------------------------------------------------
system c2_angle_ll(
    in vec2d a0,
    in vec2d a1,
    in vec2d b0,
    in vec2d b1,
    in scalar angle_rad
) {
    dot2(a1 - a0, b1 - b0)
        == cos(angle_rad) * length2(a1 - a0) * length2(b1 - b0);
}

# ----------------------------------------------------------------------------
# c2_equal_angle
# Enforces equality between angle(a1-a0, a2-a0) and angle(b1-b0, b2-b0).
# Uses cross-multiplied cosine equality to avoid explicit division.
# ----------------------------------------------------------------------------
system c2_equal_angle(
    in vec2d a0,
    in vec2d a1,
    in vec2d a2,
    in vec2d b0,
    in vec2d b1,
    in vec2d b2
) {
    dot2(a1 - a0, a2 - a0) * length2(b1 - b0) * length2(b2 - b0)
        == dot2(b1 - b0, b2 - b0) * length2(a1 - a0) * length2(a2 - a0);
}

# ----------------------------------------------------------------------------
# c2_point_on_circle
# Enforces p to lie on circle(center,radius).
# ----------------------------------------------------------------------------
system c2_point_on_circle(in vec2d p, in vec2d center, in scalar radius) {
    dot2(p - center, p - center) == radius * radius;
}

# ----------------------------------------------------------------------------
# c2_radius
# Circle radius by center and one point on circumference.
# ----------------------------------------------------------------------------
system c2_radius(in vec2d center, in vec2d rim, in scalar radius) {
    c2_distance_pp(center, rim, radius);
}

# ----------------------------------------------------------------------------
# c2_diameter
# Diameter by two opposite points on circumference.
# ----------------------------------------------------------------------------
system c2_diameter(in vec2d a, in vec2d b, in scalar diameter) {
    c2_distance_pp(a, b, diameter);
}

# ----------------------------------------------------------------------------
# c2_concentric
# Enforces two circle centers to coincide.
# ----------------------------------------------------------------------------
system c2_concentric(in vec2d c1, in vec2d c2) {
    c1 == c2;
}

# ----------------------------------------------------------------------------
# c2_equal_length
# Enforces |b-a| = |d-c| using squared form.
# ----------------------------------------------------------------------------
system c2_equal_length(in vec2d a, in vec2d b, in vec2d c, in vec2d d) {
    dot2(b - a, b - a) == dot2(d - c, d - c);
}

# ----------------------------------------------------------------------------
# c2_length_ratio
# Enforces |b-a| = ratio * |d-c|.
# Squared form avoids explicit root division.
# ----------------------------------------------------------------------------
system c2_length_ratio(
    in vec2d a,
    in vec2d b,
    in vec2d c,
    in vec2d d,
    in scalar ratio
) {
    dot2(b - a, b - a) == ratio * ratio * dot2(d - c, d - c);
}

# ----------------------------------------------------------------------------
# c2_length_diff
# Enforces |b-a| - |d-c| = delta.
# ----------------------------------------------------------------------------
system c2_length_diff(
    in vec2d a,
    in vec2d b,
    in vec2d c,
    in vec2d d,
    in scalar delta
) {
    length2(b - a) - length2(d - c) == delta;
}

# ----------------------------------------------------------------------------
# c2_tangent_line_circle
# Tangency of circle(center,radius) to line(l0,l1).
# Signed form:
#   cross(line_dir, center-l0) = side * radius * |line_dir|
# side = +1 / -1 picks one of the two tangent offsets.
# ----------------------------------------------------------------------------
system c2_tangent_line_circle(
    in vec2d l0,
    in vec2d l1,
    in vec2d center,
    in scalar radius,
    in scalar side
) {
    (l1.x - l0.x) * (center.y - l0.y) - (l1.y - l0.y) * (center.x - l0.x)
        == side * radius * length2(l1 - l0);
}

# ----------------------------------------------------------------------------
# c2_tangent_circle_circle
# Tangency of two circles (c1,r1) and (c2,r2).
# mode = +1 external tangency (distance = r1 + r2)
# mode = -1 internal tangency (distance = r1 - r2)
# ----------------------------------------------------------------------------
system c2_tangent_circle_circle(
    in vec2d c1,
    in scalar r1,
    in vec2d c2,
    in scalar r2,
    in scalar mode
) {
    dot2(c2 - c1, c2 - c1) == (r1 + mode * r2) * (r1 + mode * r2);
}

# ----------------------------------------------------------------------------
# c2_symmetric_about_line
# Reflects point p to p_ref across infinite line (l0,l1).
# Conditions:
# - midpoint(p,p_ref) lies on line
# - segment (p,p_ref) is perpendicular to line
# ----------------------------------------------------------------------------
system c2_symmetric_about_line(in vec2d p, in vec2d p_ref, in vec2d l0, in vec2d l1) {
    c2_point_on_line((p + p_ref) / 2, l0, l1);
    dot2(p_ref - p, l1 - l0) == 0;
}
