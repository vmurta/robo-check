#include "narrow-phase.hu"
#include "test-narrow-phase.hu"
#include <iostream>

#define NUM_NP_TEST_CONFS 1024

void unit_tests() {
    // Test isclose
    bool passed = true;
    if (isclose(1, 2)) {
        passed = false;
        printf("Failed isclose(1, 2)\n");
    }

    if (!isclose(1, 1 + 1e-11)) {
        passed = false;
        printf("Failed isclose(1, 1 + 1e-11)\n");
    }

    if (!isclose(1e11, 1e11 + 1)) {
        passed = false;
        printf("Failed isclose(1e11, 1e11 + 1)\n");
    }

    if (passed) {
        printf("All isclose tests passed\n");
    }

    Vector3f pts[100];
    Vector3f pt0(1, 2, 3);
    pts[0] = pt0;
    Vector3f pt1(2, 2, 3.5);
    pts[1] = pt1;
    Vector3f pt2(2, 2.5, 3);
    pts[2] = pt2;
    Vector3f pt3(1, 3, 4);
    pts[3] = pt3;
    Vector3f pt4(4, 7, 2);
    pts[4] = pt4;
    Vector3f pt5(5, 1, 3);
    pts[5] = pt5;
    Vector3f pt6(100, 300, 400);
    pts[6] = pt6;
    Vector3f pt7(400, 700, 200);
    pts[7] = pt7;
    Vector3f pt8(500, 100, 300);
    pts[8] = pt8;

    // Test veq
    passed = true;
    if (!veq(pt0, pt0)) {
        passed = false;
        printf("Failed veq(pt0, pt0)\n");
    }

    if (veq(pt0, pt1)) {
        passed = false;
        printf("Failed veq(pt0, pt1)\n");
    }

    if (passed) {
        printf("All veq tests passed\n");
    }

    // Test teq
    Triangle t1 = {0, 1, 2};
    Triangle t2 = {3, 4, 5};
    Triangle t3 = {6, 7, 8};

    passed = true;
    if (!teq(t1, pts, t1, pts)) {
        passed = false;
        printf("Failed teq(t1, t1)\n");
    }

    if (teq(t1, pts, t2, pts)) {
        passed = false;
        printf("Failed teq(t1, t2)\n");
    }

    if (passed) {
        printf("All teq tests passed\n");
    }

    // Test compute_plane
    Vector3f N1;
    float d1;
    compute_plane(t1, pts, &N1, &d1);

    passed = true;
    Vector3f N1_corr(-0.25, 0.5, 0.5);
    if (!veq(N1, N1_corr)) {
        passed = false;
        printf("Failed veq(N1, N1_corr)\n");
    }

    if (!isclose(d1, -2.25)) {
        passed = false;
        printf("Failed isclose(d1, -2.25)\n");
    }

    if (passed) {
        printf("All compute_plane tests passed\n");
    }

    // Test compute_plane_sep
    compute_plane_sep(pts[t1.v1].x, pts[t1.v1].y, pts[t1.v1].z, pts[t1.v2].x, pts[t1.v2].y, pts[t1.v2].z, pts[t1.v3].x, pts[t1.v3].y, pts[t1.v3].z, &(N1.x), &(N1.y), &N1.z, &d1);

    passed = true;
    if (!veq(N1, N1_corr)) {
        passed = false;
        printf("Failed veq(N1, N1_corr)\n");
    }

    if (!isclose(d1, -2.25)) {
        passed = false;
        printf("Failed isclose(d1, -2.25)\n");
    }

    if (passed) {
        printf("All compute_plane_sep tests passed\n");
    }

    // Test compute_signed_dists
    Vector3f dists12 = compute_signed_dists(N1, d1, t2, pts);

    passed = true;
    Vector3f dists_corr(1, 1.25, -1.5);
    if (!veq(dists12, dists_corr)) {
        passed = false;
        printf("Failed veq(dists12, dists_corr)\n");
    }

    if (passed) {
        printf("All compute_signed_dists tests passed\n");
    }

    // Test compute_signed_dists_sep
    compute_signed_dists_sep(N1.x, N1.y, N1.z, d1, pts[t2.v1].x, pts[t2.v1].y, pts[t2.v1].z, pts[t2.v2].x, pts[t2.v2].y, pts[t2.v2].z, pts[t2.v3].x, pts[t2.v3].y, pts[t2.v3].z, &(dists12.x), &(dists12.y), &(dists12.z));

    passed = true;
    if (!veq(dists12, dists_corr)) {
        passed = false;
        printf("Failed veq(dists12, dists_corr)\n");
    }

    if (passed) {
        printf("All compute_signed_dists_sep tests passed\n");
    }

    // Test early exit
    passed = true;
    if (no_overlap(dists12)) {
        passed = false;
        printf("Failed no_overlap(dists12)\n");
    }

    if (!no_overlap(compute_signed_dists(N1, d1, t3, pts))) {
        passed = false;
        printf("Failed no_overlap(compute_signed_dists(N1, d1, t3, pts))\n");
    }

    if (!no_overlap(compute_signed_dists(N1, d1, t1, pts))) {
        passed = false;
        printf("Failed no_overlap(compute_signed_dists(N1, d1, t1, pts))\n");
    }

    if (passed) {
        printf("All early-exit tests passed\n");
    }

    // Test early exit sep
    passed = true;
    if (no_overlap_sep(dists12.x, dists12.y, dists12.z)) {
        passed = false;
        printf("Failed no_overlap(dists12)\n");
    }

    Vector3f dists13 = compute_signed_dists(N1, d1, t3, pts);
    if (!no_overlap_sep(dists13.x, dists13.y, dists13.z)) {
        passed = false;
        printf("Failed no_overlap(compute_signed_dists(N1, d1, t3, pts))\n");
    }

    Vector3f dists11 = compute_signed_dists(N1, d1, t1, pts);
    if (!no_overlap_sep(dists11.x, dists11.y, dists11.z)) {
        passed = false;
        printf("Failed no_overlap(compute_signed_dists(N1, d1, t1, pts))\n");
    }

    if (passed) {
        printf("All early-exit sep tests passed\n");
    }

    // Test la_solve
    float x1, x2;
    la_solve(2, 3, 4, 5, 12, 22, &x1, &x2);

    passed = true;
    if (!isclose(x1, 3)) {
        passed = false;
        printf("Failed la_solve(2, 3, 4, 5, 12, 22, &x1, &x2)\n");
    }

    if (!isclose(x2, 2)) {
        passed = false;
        printf("Failed la_solve(2, 3, 4, 5, 12, 22, &x1, &x2)\n");
    }

    la_solve(0, 3, 4, 5, 12, 24, &x1, &x2);
    if (!isclose(x1, 1)) {
        passed = false;
        printf("Failed la_solve(0, 3, 4, 5, 12, 24, &x1, &x2)\n");
    }

    if (!isclose(x2, 4)) {
        passed = false;
        printf("Failed la_solve(0, 3, 4, 5, 12, 24, &x1, &x2)\n");
    }

    if (passed) {
        printf("All la_solve tests passed\n");
    }

    // Test compute_intersect_line
    Vector3f D, O;
    Vector3f XY_plane(0, 0, 1);
    Vector3f YZ_plane(1, 0, 0);
    Vector3f XZ_plane(0, 1, 0);

    passed = true;
    compute_intersect_line(XY_plane, -1, YZ_plane, -2, &D, &O);
    Vector3f D_XYYZ_corr(0, 1, 0);
    if (!veq(D, D_XYYZ_corr)) {
        passed = false;
        printf("Failed veq(D, D_XYYZ_corr)\n");
    }

    Vector3f O_XYYZ_corr(2, 0, 1);
    if (!veq(O, O_XYYZ_corr)) {
        passed = false;
        printf("Failed veq(O, O_XYYZ_corr)\n");
    }

    compute_intersect_line(YZ_plane, -1, XZ_plane, -2, &D, &O);
    Vector3f D_YZXZ_corr(0, 0, 1);
    if (!veq(D, D_YZXZ_corr)) {
        passed = false;
        printf("Failed veq(D, D_YZXZ_corr)\n");
    }

    Vector3f O_YZXZ_corr(1, 2, 0);
    if (!veq(O, O_YZXZ_corr)) {
        passed = false;
        printf("Failed veq(O, O_YZXZ_corr)\n");
    }

    compute_intersect_line(XZ_plane, -1, XY_plane, -2, &D, &O);
    Vector3f D_XZXY_corr(1, 0, 0);
    if (!veq(D, D_XZXY_corr)) {
        passed = false;
        printf("Failed veq(D, D_XZXY_corr)\n");
    }

    Vector3f O_XZXY_corr(0, 1, 2);
    if (!veq(O, O_XZXY_corr)) {
        passed = false;
        printf("Failed veq(O, O_XZXY_corr)\n");
    }

    // Example from https://math.stackexchange.com/questions/475953/how-to-calculate-the-intersection-of-two-planes
    Vector3f N2(2, 3, -2);
    Vector3f N3(1, 2, 1);
    compute_intersect_line(N2, 2, N3, -1, &D, &O);

    Vector3f D_N2N3_corr(7, -4, 1);
    if (!veq(D, D_N2N3_corr)) {
        passed = false;
        printf("Failed veq(D, D_N2N3_corr)\n");
    }

    Vector3f O_N2N3_corr(-7, 4, 0);
    if (!veq(O, O_N2N3_corr)) {
        passed = false;
        printf("Failed veq(O, O_N2N3_corr)\n");
    }

    if (passed) {
        printf("All compute_intersect_line tests passed\n");
    }

    // Test compute_intersect_line_sep
    passed = true;
    compute_intersect_line_sep(XY_plane.x, XY_plane.y, XY_plane.z, -1, YZ_plane.x, YZ_plane.y, YZ_plane.z, -2, &(D.x), &(D.y), &(D.z), &(O.x), &(O.y), &(O.z));
    if (!veq(D, D_XYYZ_corr)) {
        passed = false;
        printf("Failed veq(D, D_XYYZ_corr)\n");
    }

    if (!veq(O, O_XYYZ_corr)) {
        passed = false;
        printf("Failed veq(O, O_XYYZ_corr)\n");
    }

    compute_intersect_line_sep(YZ_plane.x, YZ_plane.y, YZ_plane.z, -1, XZ_plane.x, XZ_plane.y, XZ_plane.z, -2, &(D.x), &(D.y), &(D.z), &(O.x), &(O.y), &(O.z));
    if (!veq(D, D_YZXZ_corr)) {
        passed = false;
        printf("Failed veq(D, D_YZXZ_corr)\n");
    }

    if (!veq(O, O_YZXZ_corr)) {
        passed = false;
        printf("Failed veq(O, O_YZXZ_corr)\n");
    }

    compute_intersect_line_sep(XZ_plane.x, XZ_plane.y, XZ_plane.z, -1, XY_plane.x, XY_plane.y, XY_plane.z, -2, &(D.x), &(D.y), &(D.z), &(O.x), &(O.y), &(O.z));
    if (!veq(D, D_XZXY_corr)) {
        passed = false;
        printf("Failed veq(D, D_XZXY_corr)\n");
    }

    if (!veq(O, O_XZXY_corr)) {
        passed = false;
        printf("Failed veq(O, O_XZXY_corr)\n");
    }


    // Example from https://math.stackexchange.com/questions/475953/how-to-calculate-the-intersection-of-two-planes
    compute_intersect_line_sep(N2.x, N2.y, N2.z, 2, N3.x, N3.y, N3.z, -1, &(D.x), &(D.y), &(D.z), &(O.x), &(O.y), &(O.z));

    if (!veq(D, D_N2N3_corr)) {
        passed = false;
        printf("Failed veq(D, D_N2N3_corr)\n");
    }

    if (!veq(O, O_N2N3_corr)) {
        passed = false;
        printf("Failed veq(O, O_N2N3_corr)\n");
    }

    if (passed) {
        printf("All compute_intersect_line_sep tests passed\n");
    }

    // Test project_vertex
    Vector3f V_pv(1, 2, 2);
    Vector3f O_pv(2, 2, 4);
    Vector3f D_pv(3, 4, 5);

    passed = true;
    if (!isclose(project_vertex(V_pv, D_pv, O_pv), -13)) {
        passed = false;
        printf("Failed project_vertex(V_pv, D_pv, O_pv)\n");
    }

    if (passed) {
        printf("All project_vertex tests passed\n");
    }

    // Test project_vertex_sep
    passed = true;
    if (!isclose(project_vertex_sep(V_pv.x, V_pv.y, V_pv.z, D_pv.x, D_pv.y, D_pv.z, O_pv.x, O_pv.y, O_pv.z), -13)) {
        passed = false;
        printf("Failed project_vertex_sep(V_pv, D_pv, O_pv)\n");
    }

    if (passed) {
        printf("All project_vertex_sep tests passed\n");
    }


    // Test canonicalize_triangle
    Vector3f dists1(-1, -1, 1);
    Vector3f dists2(-1, 1, 1);
    Vector3f dists3(-1, 1, -1);
    Vector3f dists4(1, 1, -1);
    Vector3f dists5(1, -1, -1);
    Vector3f dists6(1, -1, 1);
    Triangle t_ct = {0, 1, 2};

    Vector3f dists_n1n1_corr(-1, 1, -1);
    Vector3f dists_11_corr(1, -1, 1);

    Triangle new_t;
    Vector3f new_dists;
    passed = true;

    canonicalize_triangle(t_ct, dists1, &new_t, &new_dists);
    if (!teq(new_t, pts, {0, 2, 1}, pts)) {
        passed = false;
        printf("Failed canonicalize_triangle(t_ct, dists1, &new_t, &new_dists)\n");
    }

    if (!veq(new_dists, dists_n1n1_corr)) {
        passed = false;
        printf("Failed canonicalize_triangle(t_ct, dists1, &new_t, &new_dists)\n");
    }

    canonicalize_triangle(t_ct, dists2, &new_t, &new_dists);
    if (!teq(new_t, pts, {1, 0, 2}, pts)) {
        passed = false;
        printf("Failed canonicalize_triangle(t_ct, dists2, &new_t, &new_dists)\n");
    }

    if (!veq(new_dists, dists_11_corr)) {
        passed = false;
        printf("Failed canonicalize_triangle(t_ct, dists2, &new_t, &new_dists)\n");
    }

    canonicalize_triangle(t_ct, dists3, &new_t, &new_dists);
    if (!teq(new_t, pts, {0, 1, 2}, pts)) {
        passed = false;
        printf("Failed canonicalize_triangle(t_ct, dists3, &new_t, &new_dists)\n");
    }

    if (!veq(new_dists, dists_n1n1_corr)) {
        passed = false;
        printf("Failed canonicalize_triangle(t_ct, dists3, &new_t, &new_dists)\n");
    }

    canonicalize_triangle(t_ct, dists4, &new_t, &new_dists);
    if (!teq(new_t, pts, {0, 2, 1}, pts)) {
        passed = false;
        printf("Failed canonicalize_triangle(t_ct, dists4, &new_t, &new_dists)\n");
    }

    if (!veq(new_dists, dists_11_corr)) {
        passed = false;
        printf("Failed canonicalize_triangle(t_ct, dists4, &new_t, &new_dists)\n");
    }

    canonicalize_triangle(t_ct, dists5, &new_t, &new_dists);
    if (!teq(new_t, pts, {1, 0, 2}, pts)) {
        passed = false;
        printf("Failed canonicalize_triangle(t_ct, dists5, &new_t, &new_dists)\n");
    }

    if (!veq(new_dists, dists_n1n1_corr)) {
        passed = false;
        printf("Failed canonicalize_triangle(t_ct, dists5, &new_t, &new_dists)\n");
    }

    canonicalize_triangle(t_ct, dists6, &new_t, &new_dists);
    if (!teq(new_t, pts, {0, 1, 2}, pts)) {
        passed = false;
        printf("Failed canonicalize_triangle(t_ct, dists6, &new_t, &new_dists)\n");
    }

    if (!veq(new_dists, dists_11_corr)) {
        passed = false;
        printf("Failed canonicalize_triangle(t_ct, dists6, &new_t, &new_dists)\n");
    }

    if (passed) {
        printf("All canonicalize_triangle tests passed\n");
    }

    // Test canonicalize_triangle_sep
    passed = true;
    int v1, v2, v3;
    canonicalize_triangle_sep(dists1.x, dists1.y, dists1.z, &v1, &v2, &v3);
    if (!teq({v1, v2, v3}, pts, {0, 2, 1}, pts)) {
        passed = false;
        printf("Failed canonicalize_triangle_sep(t_ct, dists1, &new_t, &new_dists)\n");
    }

    canonicalize_triangle_sep(dists2.x, dists2.y, dists2.z, &v1, &v2, &v3);
    if (!teq({v1, v2, v3}, pts, {1, 0, 2}, pts)) {
        passed = false;
        printf("Failed canonicalize_triangle_sep(t_ct, dists2, &new_t, &new_dists)\n");
    }

    canonicalize_triangle_sep(dists3.x, dists3.y, dists3.z, &v1, &v2, &v3);
    if (!teq({v1, v2, v3}, pts, {0, 1, 2}, pts)) {
        passed = false;
        printf("Failed canonicalize_triangle_sep(t_ct, dists3, &new_t, &new_dists)\n");
    }

    canonicalize_triangle_sep(dists4.x, dists4.y, dists4.z, &v1, &v2, &v3);
    if (!teq({v1, v2, v3}, pts, {0, 2, 1}, pts)) {
        passed = false;
        printf("Failed canonicalize_triangle_sep(t_ct, dists4, &new_t, &new_dists)\n");
    }

    canonicalize_triangle_sep(dists5.x, dists5.y, dists5.z, &v1, &v2, &v3);
    if (!teq({v1, v2, v3}, pts, {1, 0, 2}, pts)) {
        passed = false;
        printf("Failed canonicalize_triangle_sep(t_ct, dists5, &new_t, &new_dists)\n");
    }

    canonicalize_triangle_sep(dists6.x, dists6.y, dists6.z, &v1, &v2, &v3);
    if (!teq({v1, v2, v3}, pts, {0, 1, 2}, pts)) {
        passed = false;
        printf("Failed canonicalize_triangle_sep(t_ct, dists6, &new_t, &new_dists)\n");
    }

    if (passed) {
        printf("All canonicalize_triangle_sep tests passed\n");
    }

    // Test compute_parametric_variable
    float d2;
    compute_plane(t2, pts, &N2, &d2);
    Vector3f dists21 = compute_signed_dists(N2, d2, t1, pts);
    compute_intersect_line(N1, d1, N2, d2, &D, &O);

    Triangle ct1, ct2;
    Vector3f cdists12, cdists21;
    canonicalize_triangle(t1, dists21, &ct1, &cdists21);
    canonicalize_triangle(t2, dists12, &ct2, &cdists12);

    passed = true;
    if (!isclose(compute_parametric_variable(pts[ct1.v1], pts[ct1.v2], cdists21.x, cdists21.y, D, O), 108.607147)) {
        passed = false;
        printf("Failed compute_parametric_variable(pts[ct1.v1], pts[ct1.v2], cdists21.x, cdists21.y, D, O)\n");
    }

    if (!isclose(compute_parametric_variable(pts[ct1.v2], pts[ct1.v3], cdists21.y, cdists21.z, D, O), 143.744751)) {
        passed = false;
        printf("Failed compute_parametric_variable(pts[ct1.v2], pts[ct1.v3], cdists21.y, cdists21.z, D, O)\n");
    }

    if (!isclose(compute_parametric_variable(pts[ct2.v1], pts[ct2.v2], cdists12.x, cdists12.y, D, O), 130.328568)) {
        passed = false;
        printf("Failed compute_parametric_variable(pts[ct2.v1], pts[ct2.v2], cdists12.x, cdists12.y, D, O)\n");
    }

    if (!isclose(compute_parametric_variable(pts[ct2.v2], pts[ct2.v3], cdists12.y, cdists12.z, D, O), 88.860390)) {
        passed = false;
        printf("Failed compute_parametric_variable(pts[ct2.v2], pts[ct2.v3], cdists12.y, cdists12.z, D, O)\n");
    }

    if (passed) {
        printf("All compute_parametric_variable tests passed\n");
    }

    // Test compute_parametric_variable_sep
    passed = true;
    if (!isclose(compute_parametric_variable_sep(pts[ct1.v1].x, pts[ct1.v1].y, pts[ct1.v1].z, pts[ct1.v2].x, pts[ct1.v2].y, pts[ct1.v2].z, cdists21.x, cdists21.y, D.x, D.y, D.z, O.x, O.y, O.z), 108.607147)) {
        passed = false;
        printf("Failed compute_parametric_variable_sep(pts[ct1.v1], pts[ct1.v2], cdists21.x, cdists21.y, D, O)\n");
    }

    if (!isclose(compute_parametric_variable_sep(pts[ct1.v2].x, pts[ct1.v2].y, pts[ct1.v2].z, pts[ct1.v3].x, pts[ct1.v3].y, pts[ct1.v3].z, cdists21.y, cdists21.z, D.x, D.y, D.z, O.x, O.y, O.z), 143.744751)) {
        passed = false;
        printf("Failed compute_parametric_variable_sep(pts[ct1.v2], pts[ct1.v3], cdists21.y, cdists21.z, D, O)\n");
    }

    if (!isclose(compute_parametric_variable_sep(pts[ct2.v1].x, pts[ct2.v1].y, pts[ct2.v1].z, pts[ct2.v2].x, pts[ct2.v2].y, pts[ct2.v2].z, cdists12.x, cdists12.y, D.x, D.y, D.z, O.x, O.y, O.z), 130.328568)) {
        passed = false;
        printf("Failed compute_parametric_variable_sep(pts[ct2.v1], pts[ct2.v2], cdists12.x, cdists12.y, D, O)\n");
    }

    if (!isclose(compute_parametric_variable_sep(pts[ct2.v2].x, pts[ct2.v2].y, pts[ct2.v2].z, pts[ct2.v3].x, pts[ct2.v3].y, pts[ct2.v3].z, cdists12.y, cdists12.z, D.x, D.y, D.z, O.x, O.y, O.z), 88.860390)) {
        passed = false;
        printf("Failed compute_parametric_variable_sep(pts[ct2.v2], pts[ct2.v3], cdists12.y, cdists12.z, D, O)\n");
    }

    if (passed) {
        printf("All compute_parametric_variable_sep tests passed\n");
    }

    // Test is_coplanar
    passed = true;

    if (is_coplanar(N1, d1, N2, d2)) {
        passed = false;
        printf("Failed is_coplanar(N1, d1, N2, d2)\n");
    }

    N3.x = N1.x * 2;
    N3.y = N1.y * 2;
    N3.z = N1.z * 2;

    if (!is_coplanar(N1, d1, N3, d1 * 2)) {
        passed = false;
        printf("Failed is_coplanar(N1, d1, N3, d1 * 2)\n");
    }

    if (is_coplanar(N1, d1, N3, 0)) {
        passed = false;
        printf("Failed is_coplanar(N1, d1, N3, 0)\n");
    }

    N3.x = 0;

    Vector3f N4 = {0, N1.y, N1.z};
    if (!is_coplanar(N4, d1, N3, d1 * 2)) {
        passed = false;
        printf("Failed is_coplanar(N4, d1, N3, d1 * 2)\n");
    }

    if (passed) {
        printf("All is_coplanar tests passed\n");
    }

    // Test is_coplanar_sep
    passed = true;

    if (is_coplanar_sep(N1.x, N1.y, N1.z, d1, N2.x, N2.y, N2.z, d2)) {
        passed = false;
        printf("Failed is_coplanar_sep(N1, d1, N2, d2)\n");
    }

    N3.x =  N1.x * 2;
    if (!is_coplanar_sep(N1.x, N1.y, N1.z, d1, N3.x, N3.y, N3.z, d1 * 2)) {
        passed = false;
        printf("Failed is_coplanar_sep(N1, d1, N3, d1 * 2)\n");
    }

    if (is_coplanar_sep(N1.x, N1.y, N1.z, d1, N3.x, N3.y, N3.z, 0)) {
        passed = false;
        printf("Failed is_coplanar_sep(N1, d1, N3, 0)\n");
    }

    N3.x = 0;

    if (!is_coplanar_sep(N4.x, N4.y, N4.z, d1, N3.x, N3.y, N3.z, d1 * 2)) {
        passed = false;
        printf("Failed is_coplanar_sep(N4, d1, N3, d1 * 2)\n");
    }

    if (passed) {
        printf("All is_coplanar_sep tests passed\n");
    }
}

void test_baseline() {
    Triangle rob_trs[4];
    rob_trs[0] = {0, 1, 2};
    rob_trs[1] = {0, 1, 3};
    rob_trs[2] = {0, 2, 3};
    rob_trs[3] = {1, 2, 3};

    Triangle obs_trs[4];
    obs_trs[0] = {0, 1, 2};
    obs_trs[1] = {0, 1, 3};
    obs_trs[2] = {0, 2, 3};
    obs_trs[3] = {1, 2, 3};

    Vector3f rob_pts[8];
    Vector3f pt0(0, 0, 0);
    Vector3f pt1(0, 1, 2);
    Vector3f pt2(0, 2, 0);
    Vector3f pt3(2, 2, 2);
    Vector3f pt4(100, 0, 0);
    Vector3f pt5(100, 1, 2);
    Vector3f pt6(100, 2, 0);
    Vector3f pt7(102, 2, 2);
    rob_pts[0] = pt0;
    rob_pts[1] = pt1;
    rob_pts[2] = pt2;
    rob_pts[3] = pt3;
    rob_pts[4] = pt4;
    rob_pts[5] = pt5;
    rob_pts[6] = pt6;
    rob_pts[7] = pt7;

    Vector3f obs_pts[4];
    Vector3f pto0(1, 0.1, 0);
    Vector3f pto1(1.5, 1, 2.2);
    Vector3f pto2(1, 2.4, 0);
    Vector3f pto3(3.3, 2, 2);
    obs_pts[0] = pto0;
    obs_pts[1] = pto1;
    obs_pts[2] = pto2;
    obs_pts[3] = pto3;

    bool res[2];
    narrowPhaseBaseline(2, 4, 4, 4, 4, rob_trs, rob_pts, obs_trs, obs_pts, res);

    bool passed = true;
    if (res[0]) {
        passed = false;
        printf("Failed test_baseline res[0]\n");
    }

    if (!res[1]) {
        passed = false;
        printf("Failed test_baseline res[1]\n");
    }

    if (passed) {
        printf("All test_baseline tests passed\n");
    }
}

void test_single_triangle() {
    Vector3f rob_pts[3];
    Vector3f ptr0(1.441547, -14.800514, 62.841087);
    rob_pts[0] = ptr0;
    Vector3f ptr1(-4.215309, 8.199282, 23.057938);
    rob_pts[1] = ptr1;
    Vector3f ptr2(1.883977, -15.487457, 62.381035);
    rob_pts[2] = ptr2;

    Vector3f obj_pts[3];
    Vector3f pto0(1.681669, 2.616245, 1.069425);
    obj_pts[0] = pto0;
    Vector3f pto1(3.561536, 0.677467, 1.707230);
    obj_pts[1] = pto1;
    Vector3f pto2(1.172210, 2.534812, 1.852433);
    obj_pts[2] = pto2;

    Triangle rob_trs[1];
    rob_trs[0] = {0, 1, 2};

    Triangle obs_trs[1];
    obs_trs[0] = {0, 1, 2};

    bool res[1];
    res[0] = false;
    narrowPhaseBaseline(1, 1, 3, 1, 3, rob_trs, rob_pts, obs_trs, obj_pts, res);

    bool passed = true;
    if (!res[0]) {
        passed = false;
        printf("Failed test_single_triangle CPU res[0]\n");
    }

    res[0] = false;
    narrowPhase_unopt(1, 1, 3, 1, 3, rob_trs, rob_pts, obs_trs, obj_pts, res);
    if (!res[0]) {
        passed = false;
        printf("Failed test_single_triangle GPU unoptimized res[0]\n");
    }

    res[0] = false;
    narrowPhase(1, 1, 3, 1, 3, rob_trs, rob_pts, obs_trs, obj_pts, res);
    if (!res[0]) {
        passed = false;
        printf("Failed test_single_triangle GPU optimized res[0]\n");
    }

    if (passed) {
        printf("All test_single_triangle tests passed\n");
    }

}


void test_gpu() {
    Triangle rob_trs[4];
    rob_trs[0] = {0, 1, 2};
    rob_trs[1] = {0, 1, 3};
    rob_trs[2] = {0, 2, 3};
    rob_trs[3] = {1, 2, 3};

    Triangle obs_trs[4];
    obs_trs[0] = {0, 1, 2};
    obs_trs[1] = {0, 1, 3};
    obs_trs[2] = {0, 2, 3};
    obs_trs[3] = {1, 2, 3};

    Vector3f pt0(0, 0, 0);
    Vector3f pt1(0, 1, 2);
    Vector3f pt2(0, 2, 0);
    Vector3f pt3(2, 2, 2);
    Vector3f pt4(100, 0, 0);
    Vector3f pt5(100, 1, 2);
    Vector3f pt6(100, 2, 0);
    Vector3f pt7(102, 2, 2);

    Vector3f obs_pts[4];
    Vector3f pto0(1, 0.1, 0);
    Vector3f pto1(1.5, 1, 2.2);
    Vector3f pto2(1, 2.4, 0);
    Vector3f pto3(3.3, 2, 2);
    obs_pts[0] = pto0;
    obs_pts[1] = pto1;
    obs_pts[2] = pto2;
    obs_pts[3] = pto3;

    // Generate the correct number of configurations
    Vector3f real_rob_pts[NUM_NP_TEST_CONFS * 4];
    for (int i = 0; i < NUM_NP_TEST_CONFS; i++) {
        if (i % 2 == 0) {
            real_rob_pts[i * 4] = pt0;
            real_rob_pts[i * 4 + 1] = pt1;
            real_rob_pts[i * 4 + 2] = pt2;
            real_rob_pts[i * 4 + 3] = pt3;
        } else {
            real_rob_pts[i * 4] = pt4;
            real_rob_pts[i * 4 + 1] = pt5;
            real_rob_pts[i * 4 + 2] = pt6;
            real_rob_pts[i * 4 + 3] = pt7;
        }
    }

    bool res[NUM_NP_TEST_CONFS];
    for (int i = 0; i < NUM_NP_TEST_CONFS; i++) {
        res[i] = false;
    }
    narrowPhase_unopt(NUM_NP_TEST_CONFS, 4, 4, 4, 4, rob_trs, real_rob_pts, obs_trs, obs_pts, res);

    bool passed = true;
    for (int i = 0; i < NUM_NP_TEST_CONFS; i++) {
        if (res[i] != i % 2) {
            passed = false;
            printf("Error checking unopt conf %d\n", i);
            break;
        }

        // Reset res
        res[i] = false;
    }


    narrowPhase(NUM_NP_TEST_CONFS, 4, 4, 4, 4, rob_trs, real_rob_pts, obs_trs, obs_pts, res);
    for (int i = 0; i < NUM_NP_TEST_CONFS; i++) {
        if (res[i] != i % 2) {
            passed = false;
            printf("Error checking opt conf %d\n", i);
            break;
        }
        // Set res to prepare for next test
        res[i] = true;
    }

    // If all configurations are already valid, don't even run the narrow phase
    for (int i = 0; i < NUM_NP_TEST_CONFS; i++) {
        res[i] = true;
    }
    narrowPhase(NUM_NP_TEST_CONFS, 4, 4, 4, 4, rob_trs, real_rob_pts, obs_trs, obs_pts, res);
    for (int i = 0; i < NUM_NP_TEST_CONFS; i++) {
        if (!res[i]) {
            passed = false;
            printf("Error checking conf %d despite broad\n", i);
            break;
        }
    }
    if (passed) {
        printf("All GPU tests successful\n");
    }

}

int main() {
    unit_tests();
    test_single_triangle();
    test_baseline();
    test_gpu();
    return 0;
}
