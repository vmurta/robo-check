#include "narrow-phase.hu"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

// Machine epsilon for floats is 1e-7
// https://en.wikipedia.org/wiki/Machine_epsilon#Values_for_standard_hardware_arithmetics
#define TOL 1e-6
#define BLOCK_SIZE 128
#define VERBOSE 0

__host__ __device__ bool isclose(float v1, float v2) {

    if (abs(v1) < TOL && abs(v2) < TOL) {
        return true;
    } else if (abs(v1) < TOL) {
        return false;
    } else if (abs(v2) < TOL) {
        return false;
    } else {
        return abs((v1 - v2) / v1) < TOL;
    }
}

__host__ __device__ bool veq(const Vector3f v1, const Vector3f v2) {
    return isclose(v1.x, v2.x) && isclose(v1.y, v2.y) && isclose(v1.z, v2.z);
}

__host__ __device__ bool teq(const Triangle self_tr, const Vector3f *self_pts,
        const Triangle other_tr, const Vector3f *other_pts) {
    return veq(self_pts[self_tr.v1], other_pts[other_tr.v1]) &&
        veq(self_pts[self_tr.v2], other_pts[other_tr.v2]) &&
        veq(self_pts[self_tr.v3], other_pts[other_tr.v3]);
}

__host__ __device__ void compute_plane(const Triangle tr, const Vector3f *pts, Vector3f *N,
    float *d) {
    Vector3f v2_v1(pts[tr.v2].x - pts[tr.v1].x, pts[tr.v2].y - pts[tr.v1].y,
        pts[tr.v2].z - pts[tr.v1].z);
    Vector3f v3_v2(pts[tr.v3].x - pts[tr.v2].x, pts[tr.v3].y - pts[tr.v2].y,
        pts[tr.v3].z - pts[tr.v2].z);

    N->x = v2_v1.y * v3_v2.z - v2_v1.z * v3_v2.y;
    N->y = v2_v1.z * v3_v2.x - v2_v1.x * v3_v2.z;
    N->z = v2_v1.x * v3_v2.y - v2_v1.y * v3_v2.x;

    *d = -1 * (N->x * pts[tr.v1].x + N->y * pts[tr.v1].y + N->z * pts[tr.v1].z);
}

__host__ __device__ void compute_plane_sep(const float pt1_x, const float pt1_y, const float pt1_z, const float pt2_x, const float pt2_y, const float pt2_z, const float pt3_x, const float pt3_y, const float pt3_z, float *Nx, float *Ny, float *Nz, float *d) {
    float v2_v1_x = pt2_x - pt1_x;
    float v2_v1_y = pt2_y - pt1_y;
    float v2_v1_z = pt2_z - pt1_z;

    float v3_v2_x = pt3_x - pt2_x;
    float v3_v2_y = pt3_y - pt2_y;
    float v3_v2_z = pt3_z - pt2_z;

    *Nx = v2_v1_y * v3_v2_z - v2_v1_z * v3_v2_y;
    *Ny = v2_v1_z * v3_v2_x - v2_v1_x * v3_v2_z;
    *Nz = v2_v1_x * v3_v2_y - v2_v1_y * v3_v2_x;

    *d = -1 * (*Nx * pt1_x + *Ny * pt1_y + *Nz * pt1_z);
}

__host__ __device__ Vector3f compute_signed_dists(const Vector3f N, const float d, const Triangle tr,
        const Vector3f *pts) {
    Vector3f dists;
    dists.x = N.x * pts[tr.v1].x + N.y * pts[tr.v1].y + N.z * pts[tr.v1].z + d;
    dists.y = N.x * pts[tr.v2].x + N.y * pts[tr.v2].y + N.z * pts[tr.v2].z + d;
    dists.z = N.x * pts[tr.v3].x + N.y * pts[tr.v3].y + N.z * pts[tr.v3].z + d;
    return dists;
}

__host__ __device__ void compute_signed_dists_sep(const float Nx, const float Ny, const float Nz, const float d, const float pt1_x, const float pt1_y, const float pt1_z, const float pt2_x, const float pt2_y, const float pt2_z, const float pt3_x, const float pt3_y, const float pt3_z, float* dists_x, float* dists_y, float* dists_z) {
    *dists_x = Nx * pt1_x + Ny * pt1_y + Nz * pt1_z + d;
    *dists_y = Nx * pt2_x + Ny * pt2_y + Nz * pt2_z + d;
    *dists_z = Nx * pt3_x + Ny * pt3_y + Nz * pt3_z + d;
}

__host__ __device__ bool no_overlap(const Vector3f dists) {
    bool gz = dists.x >= TOL || dists.y >= TOL || dists.z >= TOL;
    bool lz = dists.x <= -1 * TOL || dists.y <= -1 * TOL || dists.z <= -1 * TOL;

    return !(gz && lz);
}

__host__ __device__ bool no_overlap_sep(const float dists_x, const float dists_y, const float dists_z) {
    bool gz = dists_x >= TOL || dists_y >= TOL || dists_z >= TOL;
    float neg_tol = -1 * TOL;
    bool lz = dists_x <= neg_tol || dists_y <= neg_tol || dists_z <= neg_tol;

    return !(gz && lz);
}

// TODO: investigate a more numerically stable way to do this
__host__ __device__ void la_solve(const float A1, const float A2, const float A3, const float A4,
        const float b1, const float b2, float *x1, float *x2) {

    if (isclose(A1, 0)) {
        *x2 = b1 / A2;
        *x1 = (b2 - A4 * *x2) / A3;

    } else {
        float A1A4 = A1 * A4;
        float A2A3 = A2 * A3;
        float A3b1 = A3 * b1;
        float A1b2 = A1 * b2;

        *x2 = (A3b1 - A1b2) / (A2A3 - A1A4);
        *x1 = (b1 - A2 * *x2) / A1;
    }
}

__host__ __device__ void compute_intersect_line(const Vector3f N1, const float d1,
        const Vector3f N2, const float d2, Vector3f *D, Vector3f *O) {

    D->x = N1.y * N2.z - N1.z * N2.y;
    D->y = N1.z * N2.x - N1.x * N2.z;
    D->z = N1.x * N2.y - N1.y * N2.x;

    // Set t = 1
    float x1, x2;
    if (!isclose(D->z, 0)) {
        la_solve(N1.x, N1.y, N2.x, N2.y, -d1, -d2, &x1, &x2);
        O->x = x1;
        O->y = x2;
        O->z = 0;

    } else if (!isclose(D->y, 0)) {
        la_solve(N1.x, N1.z, N2.x, N2.z, -d1, -d2, &x1, &x2);
        O->x = x1;
        O->y = 0;
        O->z = x2;

    } else {
        la_solve(N1.y, N1.z, N2.y, N2.z, -d1, -d2, &x1, &x2);
        O->x = 0;
        O->y = x1;
        O->z = x2;
    }
}

__host__ __device__ void compute_intersect_line_sep(const float N1_x, const float N1_y, const float N1_z, const float d1, const float N2_x, const float N2_y, const float N2_z, const float d2, float *Dx, float *Dy, float *Dz, float *Ox, float *Oy, float *Oz) {

    *Dx = N1_y * N2_z - N1_z * N2_y;
    *Dy = N1_z * N2_x - N1_x * N2_z;
    *Dz = N1_x * N2_y - N1_y * N2_x;

    // Set t = 1
    float x1, x2;
    if (!isclose(*Dz, 0)) {
        la_solve(N1_x, N1_y, N2_x, N2_y, -d1, -d2, &x1, &x2);
        *Ox = x1;
        *Oy = x2;
        *Oz = 0;

    } else if (!isclose(*Dy, 0)) {
        la_solve(N1_x, N1_z, N2_x, N2_z, -d1, -d2, &x1, &x2);
        *Ox = x1;
        *Oy = 0;
        *Oz = x2;

    } else {
        la_solve(N1_y, N1_z, N2_y, N2_z, -d1, -d2, &x1, &x2);
        *Ox = 0;
        *Oy = x1;
        *Oz = x2;
    }
}

__host__ __device__ float project_vertex(const Vector3f V, const Vector3f D, const Vector3f O) {
    return D.x * (V.x - O.x) + D.y * (V.y - O.y) + D.z * (V.z - O.z);
}

__host__ __device__ float project_vertex_sep(const float Vx, const float Vy, const float Vz, const float Dx, const float Dy, const float Dz, const float Ox, const float Oy, const float Oz) {
    return Dx * (Vx - Ox) + Dy * (Vy - Oy) + Dz * (Vz - Oz);
}

__host__ __device__ void canonicalize_triangle(const Triangle t, const Vector3f dists, Triangle *new_t, Vector3f *new_dists) {
    if (dists.x > 0 && dists.y > 0 || dists.x < 0 && dists.y < 0) {
        new_t->v1 = t.v1;
        new_t->v2 = t.v3;
        new_t->v3 = t.v2;

        new_dists->x = dists.x;
        new_dists->y = dists.z;
        new_dists->z = dists.y;
    } else if (dists.x > 0 && dists.z > 0 || dists.x < 0 && dists.z < 0) {
        new_t->v1 = t.v1;
        new_t->v2 = t.v2;
        new_t->v3 = t.v3;

        new_dists->x = dists.x;
        new_dists->y = dists.y;
        new_dists->z = dists.z;
    } else {
        new_t->v1 = t.v2;
        new_t->v2 = t.v1;
        new_t->v3 = t.v3;

        new_dists->x = dists.y;
        new_dists->y = dists.x;
        new_dists->z = dists.z;
    }
}

__host__ __device__ void canonicalize_triangle_sep(const float dists_x, const float dists_y, const float dists_z, int *v1, int *v2, int *v3) {
    if (dists_x > 0 && dists_y > 0 || dists_x < 0 && dists_y < 0) {
        *v1 = 0;
        *v2 = 2;
        *v3 = 1;

    } else if (dists_x > 0 && dists_z > 0 || dists_x < 0 && dists_z < 0) {
        *v1 = 0;
        *v2 = 1;
        *v3 = 2;

    } else {
        *v1 = 1;
        *v2 = 0;
        *v3 = 2;
    }
}

__host__ __device__ float compute_parametric_variable(const Vector3f v0, const Vector3f v1,
        const float d0, const float d1, const Vector3f D, const Vector3f O) {
    float p_v0 = project_vertex(v0, D, O);
    float p_v1 = project_vertex(v1, D, O);

    return p_v0 + (p_v1 - p_v0) * d0 / (d0 - d1);
}

__host__ __device__ float compute_parametric_variable_sep(const float v0_x, const float v0_y, const float v0_z, const float v1_x, const float v1_y, const float v1_z, const float d0, const float d1, const float Dx, const float Dy, const float Dz, const float Ox, const float Oy, const float Oz) {
    float p_v0 = project_vertex_sep(v0_x, v0_y, v0_z, Dx, Dy, Dz, Ox, Oy, Oz);
    float p_v1 = project_vertex_sep(v1_x, v1_y, v1_z, Dx, Dy, Dz, Ox, Oy, Oz);

    return p_v0 + (p_v1 - p_v0) * d0 / (d0 - d1);
}

__host__ __device__ bool is_coplanar(const Vector3f N1, const float d1, const Vector3f N2, const float d2) {
    float ratio;
    bool started_ratio = false;
    for (int i = 0; i < 4; i++) {
        float p1, p2;
        if (i == 0) {
            p1 = N1.x;
            p2 = N2.x;
        } else if (i == 1) {
            p1 = N1.y;
            p2 = N2.y;
        } else if (i == 2) {
            p1 = N1.z;
            p2 = N2.z;
        } else {
            p1 = d1;
            p2 = d2;
        }

        bool p1_0 = isclose(p1, 0);
        bool p2_0 = isclose(p2, 0);

        if (p1_0 ^ p2_0) {
            return false;
        }

        else if (p1_0 && p2_0) {
            continue;
        }

        else if (!started_ratio) {
            ratio = p1 / p2;
            started_ratio = true;
        }

        else if (!isclose(ratio, p1 / p2)) {
            return false;
        }
    }

    return true;
}


__host__ __device__ bool is_coplanar_sep(const float N1_x, const float N1_y, const float N1_z, const float d1, const float N2_x, const float N2_y, const float N2_z, const float d2) {
    float ratio;
    bool started_ratio = false;
    for (int i = 0; i < 4; i++) {
        float p1, p2;
        if (i == 0) {
            p1 = N1_x;
            p2 = N2_x;
        } else if (i == 1) {
            p1 = N1_y;
            p2 = N2_y;
        } else if (i == 2) {
            p1 = N1_z;
            p2 = N2_z;
        } else {
            p1 = d1;
            p2 = d2;
        }

        bool p1_0 = isclose(p1, 0);
        bool p2_0 = isclose(p2, 0);

        if (p1_0 ^ p2_0) {
            return false;
        }

        else if (p1_0 && p2_0) {
            continue;
        }

        else if (!started_ratio) {
            ratio = p1 / p2;
            started_ratio = true;
        }

        else if (!isclose(ratio, p1 / p2)) {
            return false;
        }
    }

    return true;
}


// true if no collision
void narrowPhaseBaseline(int num_confs, int num_rob_trs, int num_rob_pts,
    int num_obs_trs, int num_obs_pts, const Triangle *rob_trs,
    const Vector3f *rob_pts, const Triangle *obs_trs, const Vector3f *obs_pts,
    bool *valid_conf) {

    for (int i = 0; i < num_confs; i++) {
        bool valid = true;

        // True only if we require coplanar analysis to determine whether or
        // not these intersect
        bool req_coplanar = false;
        for (int j = 0; j < num_rob_trs; j++) {
            Vector3f Nr;
            float dr;
            compute_plane(rob_trs[j], &rob_pts[i * num_rob_pts], &Nr, &dr);

            for (int k = 0; k < num_obs_trs; k++) {
                Vector3f distO = compute_signed_dists(Nr, dr, obs_trs[k], obs_pts);
                if (no_overlap(distO)) {
                    continue;
                }

                Vector3f No;
                float do_;
                compute_plane(obs_trs[k], obs_pts, &No, &do_);

                if (is_coplanar(Nr, dr, No, do_)) {
                    req_coplanar = true;
                    continue;
                }

                Vector3f distR = compute_signed_dists(No, do_, rob_trs[j], &rob_pts[i * num_rob_pts]);
                if (no_overlap(distR)) {
                    continue;
                }

                Vector3f D, O;
                compute_intersect_line(Nr, dr, No, do_, &D, &O);

                Triangle ctr, cto;
                Vector3f cdr, cdo;
                canonicalize_triangle(rob_trs[j], distR, &ctr, &cdr);
                canonicalize_triangle(obs_trs[k], distO, &cto, &cdo);

                float t_r01 = compute_parametric_variable(rob_pts[i * num_rob_pts + ctr.v1],
                    rob_pts[i * num_rob_pts + ctr.v2], cdr.x, cdr.y, D, O);

                float t_r12 = compute_parametric_variable(rob_pts[i * num_rob_pts + ctr.v2],
                    rob_pts[i * num_rob_pts + ctr.v3], cdr.y, cdr.z, D, O);

                float t_o01 = compute_parametric_variable(obs_pts[cto.v1],
                    obs_pts[cto.v2], cdo.x, cdo.y, D, O);

                float t_o12 = compute_parametric_variable(obs_pts[cto.v2],
                    obs_pts[cto.v3], cdo.y, cdo.z, D, O);

                // There is no overlap
                if (min(t_r01, t_r12) > max(t_o01, t_o12)) {
                    continue;

                // Also no overlap
                } else if (min(t_o01, t_o12) > max(t_r01, t_r12)) {
                    continue;

                // There is overlap
                } else {
                    valid = false;
                    req_coplanar = false;
                    break;
                }
            }

            // Stop if we found a collision
            if (!valid)
                break;
        }

        if (req_coplanar)
            printf("Error: require coplanar intersection for configuration: %d\n", i);

        valid_conf[i] = valid;
    }
}

__global__ void narrowPhaseKernel_sep(int num_confs, int num_rob_trs, int num_rob_pts,
        int num_obs_trs, int num_obs_pts, const int *rob_trs_1, const int * rob_trs_2,
        const int *rob_trs_3, const float *rob_pts_x, const float *rob_pts_y,
        const float *rob_pts_z, const int *obs_trs_1, const int *obs_trs_2,
        const int *obs_trs_3, const float *obs_pts_x, const float *obs_pts_y,
        const float *obs_pts_z, bool *valid_conf) {

    int tx = threadIdx.x;
    int i = blockDim.x * blockIdx.x + tx;
    __shared__ float rob[3][3][BLOCK_SIZE];
    __shared__ float obs[3][3][BLOCK_SIZE];
    __shared__ float Nr[3][BLOCK_SIZE];
    __shared__ float dr[BLOCK_SIZE];
    __shared__ float distO[3][BLOCK_SIZE];
    __shared__ float No[3][BLOCK_SIZE];
    __shared__ float do_[BLOCK_SIZE];
    __shared__ float distR[3][BLOCK_SIZE];
    __shared__ float D[3][BLOCK_SIZE];
    __shared__ float O[3][BLOCK_SIZE];
    __shared__ int rv1[BLOCK_SIZE];
    __shared__ int rv2[BLOCK_SIZE];
    __shared__ int rv3[BLOCK_SIZE];
    __shared__ int ov1[BLOCK_SIZE];
    __shared__ int ov2[BLOCK_SIZE];
    __shared__ int ov3[BLOCK_SIZE];

    if (i < num_confs) {
        bool valid = true;

        // True only if we require coplanar analysis to determine whether or
        // not these intersect
        bool req_coplanar = false;
        for (int j = 0; j < num_rob_trs; j++) {
            // Load the robot triangle
            rob[0][0][tx] = rob_pts_x[i * num_rob_pts + rob_trs_1[j]];
            rob[0][1][tx] = rob_pts_y[i * num_rob_pts + rob_trs_1[j]];
            rob[0][2][tx] = rob_pts_z[i * num_rob_pts + rob_trs_1[j]];
            rob[1][0][tx] = rob_pts_x[i * num_rob_pts + rob_trs_2[j]];
            rob[1][1][tx] = rob_pts_y[i * num_rob_pts + rob_trs_2[j]];
            rob[1][2][tx] = rob_pts_z[i * num_rob_pts + rob_trs_2[j]];
            rob[2][0][tx] = rob_pts_x[i * num_rob_pts + rob_trs_3[j]];
            rob[2][1][tx] = rob_pts_y[i * num_rob_pts + rob_trs_3[j]];
            rob[2][2][tx] = rob_pts_z[i * num_rob_pts + rob_trs_3[j]];

            // Compute the plane of the robot triangle
            compute_plane_sep(rob[0][0][tx], rob[0][1][tx], rob[0][2][tx], rob[1][0][tx], rob[1][1][tx], rob[1][2][tx], rob[2][0][tx], rob[2][1][tx], rob[2][2][tx], &(Nr[0][tx]), &(Nr[1][tx]), &(Nr[2][tx]), &(dr[tx]));

            for (int k = 0; k < num_obs_trs; k++) {
                // Load the obstacle triangle
                obs[0][0][tx] = obs_pts_x[obs_trs_1[k]];
                obs[0][1][tx] = obs_pts_y[obs_trs_1[k]];
                obs[0][2][tx] = obs_pts_z[obs_trs_1[k]];
                obs[1][0][tx] = obs_pts_x[obs_trs_2[k]];
                obs[1][1][tx] = obs_pts_y[obs_trs_2[k]];
                obs[1][2][tx] = obs_pts_z[obs_trs_2[k]];
                obs[2][0][tx] = obs_pts_x[obs_trs_3[k]];
                obs[2][1][tx] = obs_pts_y[obs_trs_3[k]];
                obs[2][2][tx] = obs_pts_z[obs_trs_3[k]];

                // Compute the distances between the robot plane and the obstacle triangle
                compute_signed_dists_sep(Nr[0][tx], Nr[1][tx], Nr[2][tx], dr[tx], obs[0][0][tx], obs[0][1][tx], obs[0][2][tx], obs[1][0][tx], obs[1][1][tx], obs[1][2][tx], obs[2][0][tx], obs[2][1][tx], obs[2][2][tx], &(distO[0][tx]), &(distO[1][tx]), &(distO[2][tx]));

                // Early exit if there is definitely no overlap
                if (no_overlap_sep(distO[0][tx], distO[1][tx], distO[2][tx])) {
                    continue;
                }

                // Compute the plane of the obstacle triangle
                compute_plane_sep(obs[0][0][tx], obs[0][1][tx], obs[0][2][tx], obs[1][0][tx], obs[1][1][tx], obs[1][2][tx], obs[2][0][tx], obs[2][1][tx], obs[2][2][tx], &(No[0][tx]), &(No[1][tx]), &(No[2][tx]), &(do_[tx]));

                // Compute the distances between the obstacle plane and the robot triangle
                compute_signed_dists_sep(No[0][tx], No[1][tx], No[2][tx], do_[tx], rob[0][0][tx], rob[0][1][tx], rob[0][2][tx], rob[1][0][tx], rob[1][1][tx], rob[1][2][tx], rob[2][0][tx], rob[2][1][tx], rob[2][2][tx], &(distR[0][tx]), &(distR[1][tx]), &(distR[2][tx]));

                // Early exit if there is definitely no overlap
                if (no_overlap_sep(distR[0][tx], distR[1][tx], distR[2][tx])) {
                    continue;
                }

                // Make sure these two triangles are not coplanar
                if (is_coplanar_sep(Nr[0][tx], Nr[1][tx], Nr[2][tx], dr[tx], No[0][tx], No[1][tx], No[2][tx], do_[tx])) {
                    req_coplanar = true;
                    continue;
                }

                // Compute the intersection line of these two planes
                compute_intersect_line_sep(Nr[0][tx], Nr[1][tx], Nr[2][tx], dr[tx], No[0][tx], No[1][tx], No[2][tx], do_[tx], &(D[0][tx]), &(D[1][tx]), &(D[2][tx]), &(O[0][tx]), &(O[1][tx]), &(O[2][tx]));

                // Canonicalize both triangles so that v1 and v3 are on one side of the line, and v2 is on the other
                canonicalize_triangle_sep(distR[0][tx], distR[1][tx], distR[2][tx], &(rv1[tx]), &(rv2[tx]), &(rv3[tx]));
                canonicalize_triangle_sep(distO[0][tx], distO[1][tx], distO[2][tx], &(ov1[tx]), &(ov2[tx]), &(ov3[tx]));

                // Compute the intersection between the side of the triangle and the line
                float t_r01 = compute_parametric_variable_sep(rob[rv1[tx]][0][tx], rob[rv1[tx]][1][tx], rob[rv1[tx]][2][tx], rob[rv2[tx]][0][tx], rob[rv2[tx]][1][tx], rob[rv2[tx]][2][tx], distR[rv1[tx]][tx], distR[rv2[tx]][tx], D[0][tx], D[1][tx], D[2][tx], O[0][tx], O[1][tx], O[2][tx]);
                float t_r12 = compute_parametric_variable_sep(rob[rv2[tx]][0][tx], rob[rv2[tx]][1][tx], rob[rv2[tx]][2][tx], rob[rv3[tx]][0][tx], rob[rv3[tx]][1][tx], rob[rv3[tx]][2][tx], distR[rv2[tx]][tx], distR[rv3[tx]][tx], D[0][tx], D[1][tx], D[2][tx], O[0][tx], O[1][tx], O[2][tx]);
                float t_o01 = compute_parametric_variable_sep(obs[ov1[tx]][0][tx], obs[ov1[tx]][1][tx], obs[ov1[tx]][2][tx], obs[ov2[tx]][0][tx], obs[ov2[tx]][1][tx], obs[ov2[tx]][2][tx], distO[ov1[tx]][tx], distO[ov2[tx]][tx], D[0][tx], D[1][tx], D[2][tx], O[0][tx], O[1][tx], O[2][tx]);
                float t_o12 = compute_parametric_variable_sep(obs[ov2[tx]][0][tx], obs[ov2[tx]][1][tx], obs[ov2[tx]][2][tx], obs[ov3[tx]][0][tx], obs[ov3[tx]][1][tx], obs[ov3[tx]][2][tx], distO[ov2[tx]][tx], distO[ov3[tx]][tx], D[0][tx], D[1][tx], D[2][tx], O[0][tx], O[1][tx], O[2][tx]);

                // There is no overlap
                if (min(t_r01, t_r12) >= max(t_o01, t_o12)) {
                    continue;

                // Also no overlap
                } else if (min(t_o01, t_o12) >= max(t_r01, t_r12)) {
                    continue;

                // There is overlap
                } else {
                    valid = false;
                    req_coplanar = false;
                    break;
                }
            }

            // Stop if we found a collision
            if (!valid)
                break;
        }

        if (req_coplanar)
            printf("Error: require coplanar intersection for configuration: %d\n", i);

        valid_conf[i] = valid;
    }
}

__global__ void narrowPhaseKernel(int num_confs, int num_rob_trs, int num_rob_pts,
        int num_obs_trs, int num_obs_pts, const Triangle *rob_trs,
        const Vector3f *rob_pts, const Triangle *obs_trs, const Vector3f *obs_pts,
        bool *valid_conf) {

    int i = blockDim.x * blockIdx.x + threadIdx.x;

    if (i < num_confs) {
        bool valid = true;
        // printf("First robot vertex is: %f, %f, %f\n", rob_pts[i * num_rob_pts].x,
        //     rob_pts[i * num_rob_pts].y, rob_pts[i * num_rob_pts].z);
        // True only if we require coplanar analysis to determine whether or
        // not these intersect
        bool req_coplanar = false;
        for (int j = 0; j < num_rob_trs; j++) {
            Vector3f Nr;
            float dr;
            //delete these when testing
            Triangle t = rob_trs[j];
            const Vector3f* robot_pts = &rob_pts[i * num_rob_pts];
            Vector3f *pNr = &Nr;
            float *pdr = &dr;
            compute_plane(t, robot_pts, pNr, pdr);
            // compute_plane(rob_trs[j], &rob_pts[i * num_rob_pts], &Nr, &dr);

            for (int k = 0; k < num_obs_trs; k++) {
                Vector3f distO = compute_signed_dists(Nr, dr, obs_trs[k], obs_pts);
                if (no_overlap(distO)) {
                    continue;
                }

                Vector3f No;
                float do_;
                compute_plane(obs_trs[k], obs_pts, &No, &do_);

                if (is_coplanar(Nr, dr, No, do_)) {
                    req_coplanar = true;
                    continue;
                }

                Vector3f distR = compute_signed_dists(No, do_, rob_trs[j], &rob_pts[i * num_rob_pts]);
                if (no_overlap(distR)) {
                    continue;
                }

                Vector3f D, O;
                compute_intersect_line(Nr, dr, No, do_, &D, &O);

                Triangle ctr, cto;
                Vector3f cdr, cdo;
                canonicalize_triangle(rob_trs[j], distR, &ctr, &cdr);
                canonicalize_triangle(obs_trs[k], distO, &cto, &cdo);

                float t_r01 = compute_parametric_variable(rob_pts[i * num_rob_pts + ctr.v1],
                    rob_pts[i * num_rob_pts + ctr.v2], cdr.x, cdr.y, D, O);

                float t_r12 = compute_parametric_variable(rob_pts[i * num_rob_pts + ctr.v2],
                    rob_pts[i * num_rob_pts + ctr.v3], cdr.y, cdr.z, D, O);

                float t_o01 = compute_parametric_variable(obs_pts[cto.v1],
                    obs_pts[cto.v2], cdo.x, cdo.y, D, O);

                float t_o12 = compute_parametric_variable(obs_pts[cto.v2],
                    obs_pts[cto.v3], cdo.y, cdo.z, D, O);

                // There is no overlap
                if (min(t_r01, t_r12) >= max(t_o01, t_o12)) {
                    continue;

                // Also no overlap
                } else if (min(t_o01, t_o12) >= max(t_r01, t_r12)) {
                    continue;

                // There is overlap
                } else {
                    // if (i > 9980){
                    //     printf("Found collision at obstacle triangle %d and robot triangle %d\nt_r01: %f, t_r12: %f, t_o01: %f, t_o12: %f\n", k, j,  t_r01, t_r12, t_o01, t_o12);
                    //     printf("Robot Triangle 0 coordinates: (%f, %f, %f), (%f, %f, %f), (%f, %f, %f)\nObstacle Triangle %d coordinates: (%f, %f, %f), (%f, %f, %f), (%f, %f, %f)\n", 
                    //             rob_pts[t.v1].x, rob_pts[t.v1].y, rob_pts[t.v1].z, rob_pts[t.v2].x, rob_pts[t.v2].y, rob_pts[t.v2].z, rob_pts[t.v3].x, rob_pts[t.v3].y, rob_pts[t.v3].z,
                    //             k, obs_pts[obs_trs[k].v1].x, obs_pts[obs_trs[k].v1].y, obs_pts[obs_trs[k].v1].z, obs_pts[obs_trs[k].v2].x, obs_pts[obs_trs[k].v2].y, obs_pts[obs_trs[k].v2].z, obs_pts[obs_trs[k].v3].x, obs_pts[obs_trs[k].v3].y, obs_pts[obs_trs[k].v3].z);
                    // }
                    req_coplanar = false;
                    valid = false;
                    break;
                }
            }

            // Stop if we found a collision
            if (!valid)
                break;
        }

        if (req_coplanar)
            printf("Error: require coplanar intersection for configuration: %d\n", i);
        
        valid_conf[i] = valid;
    }
}

void narrowPhase_unopt(int num_confs, int num_rob_trs, int num_rob_pts,
        int num_obs_trs, int num_obs_pts, const Triangle *rob_trs,
        const Vector3f *rob_pts, const Triangle *obs_trs, const Vector3f *obs_pts,
        bool *valid_conf) {

    int device_count;
    if (cudaGetDeviceCount(&device_count) != 0) {
        printf("CUDA not loaded properly\n");
    } else {
        printf("CUDA loaded for %d device(s)\n", device_count);
    }
    cudaDeviceSynchronize();
    fflush(stdout);
    cudaError_t err;
    #if VERBOSE
        err = cudaGetLastError();
        printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));
    #endif

    // Copy the data onto the device
    Triangle *d_rob_trs;
    cudaMalloc(&d_rob_trs, num_rob_trs * sizeof(Triangle));
    cudaMemcpy(d_rob_trs, rob_trs, num_rob_trs * sizeof(Triangle), cudaMemcpyHostToDevice);
    cudaDeviceSynchronize();
    fflush(stdout);
    #if VERBOSE
        err = cudaGetLastError();
        printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));
    #endif

    Vector3f *d_rob_pts;
    cudaMalloc(&d_rob_pts, num_confs * num_rob_pts * sizeof(Vector3f));
    cudaMemcpy(d_rob_pts, rob_pts, num_confs * num_rob_pts * sizeof(Vector3f), cudaMemcpyHostToDevice);
    cudaDeviceSynchronize();
    fflush(stdout);
    #if VERBOSE
        err = cudaGetLastError();
        printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));
    #endif

    Triangle *d_obs_trs;
    cudaMalloc(&d_obs_trs, num_obs_trs * sizeof(Triangle));
    cudaMemcpy(d_obs_trs, obs_trs, num_obs_trs * sizeof(Triangle), cudaMemcpyHostToDevice);
    cudaDeviceSynchronize();
    fflush(stdout);
    #if VERBOSE
        err = cudaGetLastError();
        printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));
    #endif

    Vector3f *d_obs_pts;
    cudaMalloc(&d_obs_pts, num_obs_pts * sizeof(Vector3f));
    cudaMemcpy(d_obs_pts, obs_pts, num_obs_pts * sizeof(Vector3f), cudaMemcpyHostToDevice);
    cudaDeviceSynchronize();
    fflush(stdout);
    #if VERBOSE
        err = cudaGetLastError();
        printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));
    #endif

    bool *d_valid_conf;
    cudaMalloc(&d_valid_conf, num_confs * sizeof(bool));
    cudaDeviceSynchronize();
    fflush(stdout);
    #if VERBOSE
        err = cudaGetLastError();
        printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));
    #endif

    // Call the kernel;
    narrowPhaseKernel<<<(num_confs - 1) / BLOCK_SIZE + 1, BLOCK_SIZE>>>(num_confs, num_rob_trs,
        num_rob_pts, num_obs_trs, num_obs_pts, d_rob_trs, d_rob_pts, d_obs_trs,
        d_obs_pts, d_valid_conf);

    cudaDeviceSynchronize();
    fflush(stdout);
    #if VERBOSE
        err = cudaGetLastError();
        printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));
    #endif

    // Copy the data back
    cudaMemcpy(valid_conf, d_valid_conf, num_confs * sizeof(bool), cudaMemcpyDeviceToHost);
    cudaDeviceSynchronize();
    fflush(stdout);
    #if VERBOSE
        err = cudaGetLastError();
        printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));
    #endif

    // Free the memory
    cudaFree(d_rob_trs);
    cudaDeviceSynchronize();
    fflush(stdout);
    #if VERBOSE
        err = cudaGetLastError();
        printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));
    #endif

    cudaFree(d_rob_pts);
    cudaDeviceSynchronize();
    fflush(stdout);
    #if VERBOSE
        err = cudaGetLastError();
        printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));
    #endif

    cudaFree(d_obs_trs);
    cudaDeviceSynchronize();
    fflush(stdout);
    #if VERBOSE
        err = cudaGetLastError();
        printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));
    #endif

    cudaFree(d_obs_pts);
    cudaDeviceSynchronize();
    fflush(stdout);
    #if VERBOSE
        err = cudaGetLastError();
        printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));
    #endif

    cudaFree(d_valid_conf);
    cudaDeviceSynchronize();
    fflush(stdout);
    err = cudaGetLastError();
    printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));

}

void narrowPhase(int num_confs, int num_rob_trs, int num_rob_pts,
        int num_obs_trs, int num_obs_pts, const Triangle *rob_trs,
        const Vector3f *rob_pts, const Triangle *obs_trs, const Vector3f *obs_pts,
        bool *valid_conf) {

    // First copy everything to struct-of-arrays;
    int *rob_trs_1 = (int*) malloc(num_rob_trs * sizeof(int));
    int *rob_trs_2 = (int*) malloc(num_rob_trs * sizeof(int));
    int *rob_trs_3 = (int*) malloc(num_rob_trs * sizeof(int));
    float *rob_pts_x = (float*) malloc(num_confs * num_rob_pts * sizeof(float));
    float *rob_pts_y = (float*) malloc(num_confs * num_rob_pts * sizeof(float));
    float *rob_pts_z = (float*) malloc(num_confs * num_rob_pts * sizeof(float));
    int *obs_trs_1 = (int*) malloc(num_obs_trs * sizeof(int));
    int *obs_trs_2 = (int*) malloc(num_obs_trs * sizeof(int));
    int *obs_trs_3 = (int*) malloc(num_obs_trs * sizeof(int));
    float *obs_pts_x = (float*) malloc(num_obs_pts * sizeof(float));
    float *obs_pts_y = (float*) malloc(num_obs_pts * sizeof(float));
    float *obs_pts_z = (float*) malloc(num_obs_pts * sizeof(float));

    for (int i = 0; i < num_rob_trs; i++) {
        rob_trs_1[i] = rob_trs[i].v1;
        rob_trs_2[i] = rob_trs[i].v2;
        rob_trs_3[i] = rob_trs[i].v3;
    }

    for (int i = 0; i < num_confs * num_rob_pts; i++) {
        rob_pts_x[i] = rob_pts[i].x;
        rob_pts_y[i] = rob_pts[i].y;
        rob_pts_z[i] = rob_pts[i].z;
    }

    for (int i = 0; i < num_obs_trs; i++) {
        obs_trs_1[i] = obs_trs[i].v1;
        obs_trs_2[i] = obs_trs[i].v2;
        obs_trs_3[i] = obs_trs[i].v3;
    }

    for (int i = 0; i < num_obs_pts; i++) {
        obs_pts_x[i] = obs_pts[i].x;
        obs_pts_y[i] = obs_pts[i].y;
        obs_pts_z[i] = obs_pts[i].z;
    }

    int device_count;
    if (cudaGetDeviceCount(&device_count) != 0) {
        printf("CUDA not loaded properly\n");
    } else {
        printf("CUDA loaded for %d device(s)\n", device_count);
    }
    cudaDeviceSynchronize();
    fflush(stdout);
    cudaError_t err;
    #if VERBOSE
        err = cudaGetLastError();
        printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));
    #endif

    // Copy the data onto the device
    int *d_rob_trs_1;
    int *d_rob_trs_2;
    int *d_rob_trs_3;
    cudaMalloc(&d_rob_trs_1, num_rob_trs * sizeof(int));
    cudaMalloc(&d_rob_trs_2, num_rob_trs * sizeof(int));
    cudaMalloc(&d_rob_trs_3, num_rob_trs * sizeof(int));
    cudaMemcpy(d_rob_trs_1, rob_trs_1, num_rob_trs * sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(d_rob_trs_2, rob_trs_2, num_rob_trs * sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(d_rob_trs_3, rob_trs_3, num_rob_trs * sizeof(int), cudaMemcpyHostToDevice);
    cudaDeviceSynchronize();
    fflush(stdout);
    #if VERBOSE
        err = cudaGetLastError();
        printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));
    #endif

    float *d_rob_pts_x;
    float *d_rob_pts_y;
    float *d_rob_pts_z;
    cudaMalloc(&d_rob_pts_x, num_confs * num_rob_pts * sizeof(float));
    cudaMalloc(&d_rob_pts_y, num_confs * num_rob_pts * sizeof(float));
    cudaMalloc(&d_rob_pts_z, num_confs * num_rob_pts * sizeof(float));
    cudaMemcpy(d_rob_pts_x, rob_pts_x, num_confs * num_rob_pts * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_rob_pts_y, rob_pts_y, num_confs * num_rob_pts * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_rob_pts_z, rob_pts_z, num_confs * num_rob_pts * sizeof(float), cudaMemcpyHostToDevice);
    cudaDeviceSynchronize();
    fflush(stdout);
    #if VERBOSE
        err = cudaGetLastError();
        printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));
    #endif

    int *d_obs_trs_1;
    int *d_obs_trs_2;
    int *d_obs_trs_3;
    cudaMalloc(&d_obs_trs_1, num_obs_trs * sizeof(int));
    cudaMalloc(&d_obs_trs_2, num_obs_trs * sizeof(int));
    cudaMalloc(&d_obs_trs_3, num_obs_trs * sizeof(int));
    cudaMemcpy(d_obs_trs_1, obs_trs_1, num_obs_trs * sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(d_obs_trs_2, obs_trs_2, num_obs_trs * sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(d_obs_trs_3, obs_trs_3, num_obs_trs * sizeof(int), cudaMemcpyHostToDevice);
    cudaDeviceSynchronize();
    fflush(stdout);
    #if VERBOSE
        err = cudaGetLastError();
        printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));
    #endif

    float *d_obs_pts_x;
    float *d_obs_pts_y;
    float *d_obs_pts_z;
    cudaMalloc(&d_obs_pts_x, num_confs * num_obs_pts * sizeof(float));
    cudaMalloc(&d_obs_pts_y, num_confs * num_obs_pts * sizeof(float));
    cudaMalloc(&d_obs_pts_z, num_confs * num_obs_pts * sizeof(float));
    cudaMemcpy(d_obs_pts_x, obs_pts_x, num_confs * num_obs_pts * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_obs_pts_y, obs_pts_y, num_confs * num_obs_pts * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_obs_pts_z, obs_pts_z, num_confs * num_obs_pts * sizeof(float), cudaMemcpyHostToDevice);
    cudaDeviceSynchronize();
    fflush(stdout);
    #if VERBOSE
        err = cudaGetLastError();
        printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));
    #endif

    bool *d_valid_conf;
    cudaMalloc(&d_valid_conf, num_confs * sizeof(bool));
    cudaDeviceSynchronize();
    fflush(stdout);
    #if VERBOSE
        err = cudaGetLastError();
        printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));
    #endif

    // Call the kernel;
    narrowPhaseKernel_sep<<<(num_confs - 1) / BLOCK_SIZE + 1, BLOCK_SIZE>>>(num_confs, num_rob_trs,
        num_rob_pts, num_obs_trs, num_obs_pts, d_rob_trs_1, d_rob_trs_2, d_rob_trs_3, d_rob_pts_x, d_rob_pts_y, d_rob_pts_z, d_obs_trs_1, d_obs_trs_2, d_obs_trs_3, d_obs_pts_x, d_obs_pts_y, d_obs_pts_z, d_valid_conf);

    cudaDeviceSynchronize();

    fflush(stdout);
    #if VERBOSE
        err = cudaGetLastError();
        printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));
    #endif

    // Copy the data back
    cudaMemcpy(valid_conf, d_valid_conf, num_confs * sizeof(bool), cudaMemcpyDeviceToHost);
    cudaDeviceSynchronize();
    fflush(stdout);
    #if VERBOSE
        err = cudaGetLastError();
        printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));
    #endif

    // Free the memory
    free(rob_trs_1);
    free(rob_trs_2);
    free(rob_trs_3);
    cudaFree(d_rob_trs_1);
    cudaFree(d_rob_trs_2);
    cudaFree(d_rob_trs_3);
    cudaDeviceSynchronize();
    fflush(stdout);
    #if VERBOSE
        err = cudaGetLastError();
        printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));
    #endif

    free(rob_pts_x);
    free(rob_pts_y);
    free(rob_pts_z);
    cudaFree(d_rob_pts_x);
    cudaFree(d_rob_pts_y);
    cudaFree(d_rob_pts_z);
    cudaDeviceSynchronize();
    fflush(stdout);
    #if VERBOSE
        err = cudaGetLastError();
        printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));
    #endif

    free(obs_trs_1);
    free(obs_trs_2);
    free(obs_trs_3);
    cudaFree(d_obs_trs_1);
    cudaFree(d_obs_trs_2);
    cudaFree(d_obs_trs_3);
    cudaDeviceSynchronize();
    fflush(stdout);
    #if VERBOSE
        err = cudaGetLastError();
        printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));
    #endif

    free(obs_pts_x);
    free(obs_pts_y);
    free(obs_pts_z);
    cudaFree(d_obs_pts_x);
    cudaFree(d_obs_pts_y);
    cudaFree(d_obs_pts_z);
    cudaDeviceSynchronize();
    fflush(stdout);
    #if VERBOSE
        err = cudaGetLastError();
        printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));
    #endif

    cudaFree(d_valid_conf);
    cudaDeviceSynchronize();
    fflush(stdout);
    err = cudaGetLastError();
    printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));

}

// Note: Only the d_rob_points should be passed in as device pointers!
void narrowPhase_sep(int num_confs, int num_rob_trs, int num_rob_pts,
        int num_obs_trs, int num_obs_pts, const Triangle *rob_trs,
        const float *d_rob_pts_x, const float *d_rob_pts_y, const float *d_rob_pts_z,
        const Triangle *obs_trs, const Vector3f *obs_pts,
        bool *valid_conf) {

    // First copy everything to struct-of-arrays;
    int *rob_trs_1 = (int*) malloc(num_rob_trs * sizeof(int));
    int *rob_trs_2 = (int*) malloc(num_rob_trs * sizeof(int));
    int *rob_trs_3 = (int*) malloc(num_rob_trs * sizeof(int));
    int *obs_trs_1 = (int*) malloc(num_obs_trs * sizeof(int));
    int *obs_trs_2 = (int*) malloc(num_obs_trs * sizeof(int));
    int *obs_trs_3 = (int*) malloc(num_obs_trs * sizeof(int));
    float *obs_pts_x = (float*) malloc(num_obs_pts * sizeof(float));
    float *obs_pts_y = (float*) malloc(num_obs_pts * sizeof(float));
    float *obs_pts_z = (float*) malloc(num_obs_pts * sizeof(float));


    // FIXME - These are sent on the GPU
    for (int i = 0; i < num_rob_trs; i++) {
        rob_trs_1[i] = rob_trs[i].v1;
        rob_trs_2[i] = rob_trs[i].v2;
        rob_trs_3[i] = rob_trs[i].v3;
    }
    for (int i = 0; i < num_obs_trs; i++) {
        obs_trs_1[i] = obs_trs[i].v1;
        obs_trs_2[i] = obs_trs[i].v2;
        obs_trs_3[i] = obs_trs[i].v3;
    }
    for (int i = 0; i < num_obs_pts; i++) {
        obs_pts_x[i] = obs_pts[i].x;
        obs_pts_y[i] = obs_pts[i].y;
        obs_pts_z[i] = obs_pts[i].z;
    }

    int device_count;
    if (cudaGetDeviceCount(&device_count) != 0) {
        printf("CUDA not loaded properly\n");
    } else {
        printf("CUDA loaded for %d device(s)\n", device_count);
    }
    cudaDeviceSynchronize();
    fflush(stdout);
    cudaError_t err;
    #if VERBOSE
        err = cudaGetLastError();
        printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));
    #endif

    // Copy the data onto the device
    int *d_rob_trs_1;
    int *d_rob_trs_2;
    int *d_rob_trs_3;
    cudaMalloc(&d_rob_trs_1, num_rob_trs * sizeof(int));
    cudaMalloc(&d_rob_trs_2, num_rob_trs * sizeof(int));
    cudaMalloc(&d_rob_trs_3, num_rob_trs * sizeof(int));
    cudaMemcpy(d_rob_trs_1, rob_trs_1, num_rob_trs * sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(d_rob_trs_2, rob_trs_2, num_rob_trs * sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(d_rob_trs_3, rob_trs_3, num_rob_trs * sizeof(int), cudaMemcpyHostToDevice);
    cudaDeviceSynchronize();

    fflush(stdout);
    #if VERBOSE
        err = cudaGetLastError();
        printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));
    #endif

    int *d_obs_trs_1;
    int *d_obs_trs_2;
    int *d_obs_trs_3;
    cudaMalloc(&d_obs_trs_1, num_obs_trs * sizeof(int));
    cudaMalloc(&d_obs_trs_2, num_obs_trs * sizeof(int));
    cudaMalloc(&d_obs_trs_3, num_obs_trs * sizeof(int));
    cudaMemcpy(d_obs_trs_1, obs_trs_1, num_obs_trs * sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(d_obs_trs_2, obs_trs_2, num_obs_trs * sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(d_obs_trs_3, obs_trs_3, num_obs_trs * sizeof(int), cudaMemcpyHostToDevice);
    cudaDeviceSynchronize();
    fflush(stdout);
    #if VERBOSE
        err = cudaGetLastError();
        printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));
    #endif

    float *d_obs_pts_x;
    float *d_obs_pts_y;
    float *d_obs_pts_z;
    cudaMalloc(&d_obs_pts_x, num_confs * num_obs_pts * sizeof(float));
    cudaMalloc(&d_obs_pts_y, num_confs * num_obs_pts * sizeof(float));
    cudaMalloc(&d_obs_pts_z, num_confs * num_obs_pts * sizeof(float));
    std::cerr << "Time = 2.24"<< std::endl;
    cudaMemcpy(d_obs_pts_x, obs_pts_x, num_confs * num_obs_pts * sizeof(float), cudaMemcpyHostToDevice);
    std::cerr << "Time = 2.25"<< std::endl;
    cudaMemcpy(d_obs_pts_y, obs_pts_y, num_confs * num_obs_pts * sizeof(float), cudaMemcpyHostToDevice);
    std::cerr << "Time = 2.26"<< std::endl;
    cudaMemcpy(d_obs_pts_z, obs_pts_z, num_confs * num_obs_pts * sizeof(float), cudaMemcpyHostToDevice);
    
    cudaDeviceSynchronize();
    fflush(stdout);
    #if VERBOSE
        err = cudaGetLastError();
        printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));
    #endif


    std::cerr << "Time = 2.3"<< std::endl;

    bool *d_valid_conf;
    cudaMalloc(&d_valid_conf, num_confs * sizeof(bool));
    cudaDeviceSynchronize();
    fflush(stdout);
    #if VERBOSE
        err = cudaGetLastError();
        printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));
    #endif

    std::cerr << "Time = 2"<< std::endl;

    // Call the kernel;
    narrowPhaseKernel_sep<<<(num_confs - 1) / BLOCK_SIZE + 1, BLOCK_SIZE>>>(num_confs, num_rob_trs,
        num_rob_pts, num_obs_trs, num_obs_pts, d_rob_trs_1, d_rob_trs_2, d_rob_trs_3, d_rob_pts_x, d_rob_pts_y, d_rob_pts_z, d_obs_trs_1, d_obs_trs_2, d_obs_trs_3, d_obs_pts_x, d_obs_pts_y, d_obs_pts_z, d_valid_conf);

    cudaDeviceSynchronize();
    std::cerr << "Time = "<< std::endl;

    fflush(stdout);
    #if VERBOSE
        err = cudaGetLastError();
        printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));
    #endif

    // Copy the data back
    cudaMemcpy(valid_conf, d_valid_conf, num_confs * sizeof(bool), cudaMemcpyDeviceToHost);
    cudaDeviceSynchronize();
    fflush(stdout);
    #if VERBOSE
        err = cudaGetLastError();
        printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));
    #endif

    // Free the memory
    free(rob_trs_1);
    free(rob_trs_2);
    free(rob_trs_3);
    cudaFree(d_rob_trs_1);
    cudaFree(d_rob_trs_2);
    cudaFree(d_rob_trs_3);
    cudaDeviceSynchronize();
    fflush(stdout);
    #if VERBOSE
        err = cudaGetLastError();
        printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));
    #endif

    free(obs_trs_1);
    free(obs_trs_2);
    free(obs_trs_3);
    cudaFree(d_obs_trs_1);
    cudaFree(d_obs_trs_2);
    cudaFree(d_obs_trs_3);
    cudaDeviceSynchronize();
    fflush(stdout);
    #if VERBOSE
        err = cudaGetLastError();
        printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));
    #endif

    free(obs_pts_x);
    free(obs_pts_y);
    free(obs_pts_z);
    cudaFree(d_obs_pts_x);
    cudaFree(d_obs_pts_y);
    cudaFree(d_obs_pts_z);
    cudaDeviceSynchronize();
    fflush(stdout);
    #if VERBOSE
        err = cudaGetLastError();
        printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));
    #endif

    cudaFree(d_valid_conf);
    cudaDeviceSynchronize();
    fflush(stdout);
    err = cudaGetLastError();
    printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));

}
