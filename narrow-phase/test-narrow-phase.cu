#include "narrow-phase.hu"
#include "test-narrow-phase.hu"
#include <iostream>

#define NUM_NP_TEST_CONFS 256

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

    // Test early exit
    passed = true;
    if (no_overlap(dists12)) {
        passed = false;
        printf("Failed no_overlap(dists12)\n");
    }

    if (yes_overlap(dists12)) {
        passed = false;
        printf("Failed yes_overlap(dists12)\n");
    }

    if (!no_overlap(compute_signed_dists(N1, d1, t3, pts))) {
        passed = false;
        printf("Failed no_overlap(compute_signed_dists(N1, d1, t3, pts))\n");
    }

    if (yes_overlap(compute_signed_dists(N1, d1, t3, pts))) {
        passed = false;
        printf("Failed yes_overlap(compute_signed_dists(N1, d1, t3, pts))\n");
    }

    if (!no_overlap(compute_signed_dists(N1, d1, t1, pts))) {
        passed = false;
        printf("Failed no_overlap(compute_signed_dists(N1, d1, t1, pts))\n");
    }

    if (!yes_overlap(compute_signed_dists(N1, d1, t1, pts))) {
        passed = false;
        printf("Failed yes_overlap(compute_signed_dists(N1, d1, t1, pts))\n");
    }

    if (passed) {
        printf("All early-exit tests passed\n");
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

    // Test compute_parametric_variable
    // float d2;
    // compute_plane(t2, pts, &N2, &d2);
    // Vector3f dists21 = compute_signed_dists(N2, d2, t1, pts);

    // Triangle ct1, ct2;
    // Vector3f cdists12, cdists21;
    // canonicalize_triangle(t1, dists21, &ct1, &cdists21);
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
    narrowPhase(NUM_NP_TEST_CONFS, 4, 4, 4, 4, rob_trs, rob_pts, obs_trs, obs_pts, res);

    bool passed = true;
    for (int i = 0; i < NUM_NP_TEST_CONFS; i++) {
        if (res[i] != i % 2) {
            passed = false;
            printf("Error checking conf %d\n", i);
            break;
        }
    }

    if (passed) {
        printf("All GPU tests successful\n");
    }

}

// __global__ void kern() {
//     printf("foo ");
// }

int main() {
    // kern<<<1, 256>>>();
    // cudaDeviceSynchronize();
    unit_tests();
    // test_baseline();
    test_gpu();
    return 0;
}