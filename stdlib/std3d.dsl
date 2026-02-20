# ============================================================================
# std3d.dsl
#
# 3D geometric constraint standard library for cad-constraint-solver.
#
# Design goals:
# - Keep constraints equation-based and composable.
# - Use helper scalar parameters when parallelism/projection is needed.
# - Prefer squared-distance forms where practical.
#
# Parameter convention:
# - All parameters are `in` because systems are read-only relation templates.
# - Passing variables still works: variables remain unknowns in caller context.
#
# Branch convention:
# - `side`: +1 / -1 for signed sidedness relative to oriented normals.
# - `mode`: +1 external tangency, -1 internal tangency.
# ============================================================================

# ----------------------------------------------------------------------------
# c3_coincident
# Enforces two 3D points to coincide.
# ----------------------------------------------------------------------------
system c3_coincident(in vec3d a, in vec3d b) {
    constraint a == b;
}

# ----------------------------------------------------------------------------
# c3_distance_pp
# Point-point distance: |b-a|^2 = dist^2.
# ----------------------------------------------------------------------------
system c3_distance_pp(in vec3d a, in vec3d b, in scalar dist) {
    constraint dot3(b - a, b - a) == dist * dist;
}

# ----------------------------------------------------------------------------
# c3_midpoint
# Enforces m as midpoint of segment (a,b).
# ----------------------------------------------------------------------------
system c3_midpoint(in vec3d m, in vec3d a, in vec3d b) {
    constraint 2 * m == a + b;
}

# ----------------------------------------------------------------------------
# c3_equal_length
# Enforces |b-a| = |d-c| using squared form.
# ----------------------------------------------------------------------------
system c3_equal_length(in vec3d a, in vec3d b, in vec3d c, in vec3d d) {
    constraint dot3(b - a, b - a) == dot3(d - c, d - c);
}

# ----------------------------------------------------------------------------
# c3_unit_direction
# Enforces a vector to be unit length.
# ----------------------------------------------------------------------------
system c3_unit_direction(in vec3d v) {
    constraint dot3(v, v) == 1;
}

# ----------------------------------------------------------------------------
# c3_parallel_vectors
# Enforces v1 parallel to v2 via scalar proportionality: v1 = k * v2.
# ----------------------------------------------------------------------------
system c3_parallel_vectors(in vec3d v1, in vec3d v2) {
    scalar k;
    constraint v1 == k * v2;
}

# ----------------------------------------------------------------------------
# c3_perpendicular_vectors
# Enforces vectors to be perpendicular.
# ----------------------------------------------------------------------------
system c3_perpendicular_vectors(in vec3d v1, in vec3d v2) {
    constraint dot3(v1, v2) == 0;
}

# ----------------------------------------------------------------------------
# c3_point_on_line
# Enforces p on infinite line through (l0,l1) using affine parameter t.
# ----------------------------------------------------------------------------
system c3_point_on_line(in vec3d p, in vec3d l0, in vec3d l1) {
    scalar t;
    constraint p == l0 + t * (l1 - l0);
}

# ----------------------------------------------------------------------------
# c3_point_on_plane
# Enforces p on plane defined by point p0 and normal n.
# ----------------------------------------------------------------------------
system c3_point_on_plane(in vec3d p, in vec3d p0, in vec3d n) {
    constraint dot3(n, p - p0) == 0;
}

# ----------------------------------------------------------------------------
# c3_parallel_lines
# Enforces line (a0,a1) parallel to line (b0,b1).
# ----------------------------------------------------------------------------
system c3_parallel_lines(in vec3d a0, in vec3d a1, in vec3d b0, in vec3d b1) {
    c3_parallel_vectors(a1 - a0, b1 - b0);
}

# ----------------------------------------------------------------------------
# c3_perpendicular_lines
# Enforces line (a0,a1) perpendicular to line (b0,b1).
# ----------------------------------------------------------------------------
system c3_perpendicular_lines(in vec3d a0, in vec3d a1, in vec3d b0, in vec3d b1) {
    constraint dot3(a1 - a0, b1 - b0) == 0;
}

# ----------------------------------------------------------------------------
# c3_parallel_line_plane
# Enforces line (l0,l1) parallel to plane by requiring direction dot normal = 0.
# ----------------------------------------------------------------------------
system c3_parallel_line_plane(in vec3d l0, in vec3d l1, in vec3d n) {
    constraint dot3(l1 - l0, n) == 0;
}

# ----------------------------------------------------------------------------
# c3_perpendicular_line_plane
# Enforces line (l0,l1) perpendicular to plane by making line direction
# parallel to plane normal.
# ----------------------------------------------------------------------------
system c3_perpendicular_line_plane(in vec3d l0, in vec3d l1, in vec3d n) {
    c3_parallel_vectors(l1 - l0, n);
}

# ----------------------------------------------------------------------------
# c3_parallel_planes
# Enforces two plane normals parallel.
# ----------------------------------------------------------------------------
system c3_parallel_planes(in vec3d n1, in vec3d n2) {
    c3_parallel_vectors(n1, n2);
}

# ----------------------------------------------------------------------------
# c3_perpendicular_planes
# Enforces two plane normals perpendicular.
# ----------------------------------------------------------------------------
system c3_perpendicular_planes(in vec3d n1, in vec3d n2) {
    constraint dot3(n1, n2) == 0;
}

# ----------------------------------------------------------------------------
# c3_angle_ll
# Constrains angle between two line directions to angle_rad (radians).
# dot(u,v) = cos(theta) * |u| * |v|
# ----------------------------------------------------------------------------
system c3_angle_ll(
    in vec3d a0,
    in vec3d a1,
    in vec3d b0,
    in vec3d b1,
    in scalar angle_rad
) {
    constraint dot3(a1 - a0, b1 - b0)
        == cos(angle_rad) * length3(a1 - a0) * length3(b1 - b0);
}

# ----------------------------------------------------------------------------
# c3_angle_line_plane
# Angle between line and plane (not line and normal).
# If theta is line-plane angle, then:
#   dot(d,n) = sin(theta) * |d| * |n|
# ----------------------------------------------------------------------------
system c3_angle_line_plane(in vec3d l0, in vec3d l1, in vec3d n, in scalar angle_rad) {
    constraint dot3(l1 - l0, n) == sin(angle_rad) * length3(l1 - l0) * length3(n);
}

# ----------------------------------------------------------------------------
# c3_angle_planes
# Constrains angle between two plane normals to angle_rad.
# ----------------------------------------------------------------------------
system c3_angle_planes(in vec3d n1, in vec3d n2, in scalar angle_rad) {
    constraint dot3(n1, n2) == cos(angle_rad) * length3(n1) * length3(n2);
}

# ----------------------------------------------------------------------------
# c3_point_on_sphere
# Enforces p on sphere(center,radius).
# ----------------------------------------------------------------------------
system c3_point_on_sphere(in vec3d p, in vec3d center, in scalar radius) {
    constraint dot3(p - center, p - center) == radius * radius;
}

# ----------------------------------------------------------------------------
# c3_tangent_sphere_plane
# Enforces sphere(center,radius) tangent to plane(p0,n).
# Signed form: dot(n, center-p0) = side * radius * |n|
# ----------------------------------------------------------------------------
system c3_tangent_sphere_plane(
    in vec3d center,
    in scalar radius,
    in vec3d p0,
    in vec3d n,
    in scalar side
) {
    constraint dot3(n, center - p0) == side * radius * length3(n);
}

# ----------------------------------------------------------------------------
# c3_tangent_sphere_sphere
# Sphere-sphere tangency.
# mode = +1 external tangency, mode = -1 internal tangency.
# ----------------------------------------------------------------------------
system c3_tangent_sphere_sphere(
    in vec3d c1,
    in scalar r1,
    in vec3d c2,
    in scalar r2,
    in scalar mode
) {
    constraint dot3(c2 - c1, c2 - c1) == (r1 + mode * r2) * (r1 + mode * r2);
}

# ----------------------------------------------------------------------------
# c3_concentric_spheres
# Enforces two sphere centers to coincide.
# ----------------------------------------------------------------------------
system c3_concentric_spheres(in vec3d c1, in vec3d c2) {
    constraint c1 == c2;
}

# ----------------------------------------------------------------------------
# c3_equal_radius
# Enforces equality of two scalar radii.
# ----------------------------------------------------------------------------
system c3_equal_radius(in scalar r1, in scalar r2) {
    constraint r1 == r2;
}

# ----------------------------------------------------------------------------
# c3_distance_point_plane_signed
# Signed point-plane offset:
#   dot(n, p-p0) = dist * |n|
# dist is signed with respect to oriented normal n.
# ----------------------------------------------------------------------------
system c3_distance_point_plane_signed(
    in vec3d p,
    in vec3d p0,
    in vec3d n,
    in scalar dist
) {
    constraint dot3(n, p - p0) == dist * length3(n);
}

# ----------------------------------------------------------------------------
# c3_distance_point_line
# Point-line shortest distance.
# Construction:
# - foot point q on line(l0,l1): q = l0 + t*(l1-l0)
# - (p-q) perpendicular to line direction
# - |p-q| = dist
# ----------------------------------------------------------------------------
system c3_distance_point_line(in vec3d p, in vec3d l0, in vec3d l1, in scalar dist) {
    scalar t;

    constraint dot3(p - (l0 + t * (l1 - l0)), l1 - l0) == 0;
    constraint dot3(p - (l0 + t * (l1 - l0)), p - (l0 + t * (l1 - l0))) == dist * dist;
}

# ----------------------------------------------------------------------------
# c3_distance_line_line
# Shortest distance between two infinite lines.
# Construction:
# - p on first line, q on second line
# - connecting segment s = p-q perpendicular to both directions
# - |s| = dist
# ----------------------------------------------------------------------------
system c3_distance_line_line(
    in vec3d a0,
    in vec3d a1,
    in vec3d b0,
    in vec3d b1,
    in scalar dist
) {
    scalar t1;
    scalar t2;

    constraint dot3((a0 + t1 * (a1 - a0)) - (b0 + t2 * (b1 - b0)), a1 - a0) == 0;
    constraint dot3((a0 + t1 * (a1 - a0)) - (b0 + t2 * (b1 - b0)), b1 - b0) == 0;
    constraint dot3(
        (a0 + t1 * (a1 - a0)) - (b0 + t2 * (b1 - b0)),
        (a0 + t1 * (a1 - a0)) - (b0 + t2 * (b1 - b0))
    ) == dist * dist;
}

# ----------------------------------------------------------------------------
# c3_distance_plane_plane
# Signed distance between two planes (p1,n1) and (p2,n2).
# Includes parallel normal condition and signed offset equation.
# ----------------------------------------------------------------------------
system c3_distance_plane_plane(
    in vec3d p1,
    in vec3d n1,
    in vec3d p2,
    in vec3d n2,
    in scalar dist
) {
    c3_parallel_planes(n1, n2);
    constraint dot3(n1, p2 - p1) == dist * length3(n1);
}
