#include "narrow-phase.hu"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

extern __constant__ Eigen::Vector3f base_robot_vertices[NUM_ROB_VERTICES];
extern __constant__ Triangle base_robot_triangles[MAX_NUM_ROBOT_TRIANGLES];
extern __constant__ Eigen::Vector3f base_obs_vertices[NUM_ROB_VERTICES];
extern __constant__ Triangle base_obs_triangles[MAX_NUM_ROBOT_TRIANGLES];

extern __constant__ float base_rob_x[NUM_ROB_VERTICES];
extern __constant__ float base_rob_y[NUM_ROB_VERTICES];
extern __constant__ float base_rob_z[NUM_ROB_VERTICES];
extern __constant__ int base_rob_tri_v1[MAX_NUM_ROBOT_TRIANGLES];
extern __constant__ int base_rob_tri_v2[MAX_NUM_ROBOT_TRIANGLES];
extern __constant__ int base_rob_tri_v3[MAX_NUM_ROBOT_TRIANGLES];
extern __constant__ float base_obs_x[NUM_ROB_VERTICES];
extern __constant__ float base_obs_y[NUM_ROB_VERTICES];
extern __constant__ float base_obs_z[NUM_ROB_VERTICES];
extern __constant__ int base_obs_tri_v1[MAX_NUM_ROBOT_TRIANGLES];
extern __constant__ int base_obs_tri_v2[MAX_NUM_ROBOT_TRIANGLES];
extern __constant__ int base_obs_tri_v3[MAX_NUM_ROBOT_TRIANGLES];

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

__host__ __device__ bool veq(const Eigen::Vector3f v1, const Eigen::Vector3f v2) {
    return isclose(v1(0), v2(0)) && isclose(v1(1), v2(1)) && isclose(v1(2), v2(2));
}

__host__ __device__ bool teq(const Triangle self_tr, const Eigen::Vector3f *self_pts,
        const Triangle other_tr, const Eigen::Vector3f *other_pts) {
    return veq(self_pts[self_tr.v1], other_pts[other_tr.v1]) &&
        veq(self_pts[self_tr.v2], other_pts[other_tr.v2]) &&
        veq(self_pts[self_tr.v3], other_pts[other_tr.v3]);
}

__host__ __device__ void compute_plane(const Triangle tr, const Eigen::Vector3f *pts, Eigen::Vector3f *N,
    float *d) {
    Eigen::Vector3f v2_v1(pts[tr.v2](0) - pts[tr.v1](0), pts[tr.v2](1) - pts[tr.v1](1),
        pts[tr.v2](2) - pts[tr.v1](2));
    Eigen::Vector3f v3_v2(pts[tr.v3](0) - pts[tr.v2](0), pts[tr.v3](1) - pts[tr.v2](1),
        pts[tr.v3](2) - pts[tr.v2](2));

    (*N)(0) = v2_v1(1) * v3_v2(2) - v2_v1(2) * v3_v2(1);
    (*N)(1) = v2_v1(2) * v3_v2(0) - v2_v1(0) * v3_v2(2);
    (*N)(2) = v2_v1(0) * v3_v2(1) - v2_v1(1) * v3_v2(0);

    *d = -1 * (*N)(0) * pts[tr.v1](0) + (*N)(1) * pts[tr.v1](1) + (*N)(2) * pts[tr.v1](2);
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

__host__ __device__ void compute_plane(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2, const Eigen::Vector3f &v3, Eigen::Vector3f *N, float *d) {
    Eigen::Vector3f v2_v1(v2[0] - v1[0], v2[1] - v1[1], v2[2] - v1[2]);
    Eigen::Vector3f v3_v2(v3[0] - v2[0], v3[1] - v2[1], v3[2] - v2[2]);

    (*N)[0] = v2_v1[1] * v3_v2[2] - v2_v1[2] * v3_v2[1];
    (*N)[1] = v2_v1[2] * v3_v2[0] - v2_v1[0] * v3_v2[2];
    (*N)[2] = v2_v1[0] * v3_v2[1] - v2_v1[1] * v3_v2[0];

    *d = -1 * ((*N)[0] * v1[0] + (*N)[1] * v1[1] + (*N)[2] * v1[2]);
}

__host__ __device__ Eigen::Vector3f compute_signed_dists(const Eigen::Vector3f N, const float d, const Eigen::Vector3f &v1, const Eigen::Vector3f &v2, const Eigen::Vector3f &v3) {
    Eigen::Vector3f dists;
    dists(0) = N(0) * v1(0) + N(1) * v1(1) + N(2) * v1(2) + d;
    dists(1) = N(0) * v2(0) + N(1) * v2(1) + N(2) * v2(2) + d;
    dists(2) = N(0) * v3(0) + N(1) * v3(1) + N(2) * v3(2) + d;
    return dists;
}

__host__ __device__ void compute_intersect_line(const Eigen::Vector3f N1, const float d1,
        const Eigen::Vector3f N2, const float d2, Eigen::Vector3f *D, Eigen::Vector3f *O) {
    (*D)(0) = N1(1) * N2(2) - N1(2) * N2(1);
    (*D)(1) = N1(2) * N2(0) - N1(0) * N2(2);
    (*D)(2) = N1(0) * N2(1) - N1(1) * N2(0);

    float den = (*D)(0) * (*D)(0) + (*D)(1) * (*D)(1) + (*D)(2) * (*D)(2);

    (*O)(0) = (d2 * N1(0) - d1 * N2(0)) / den;
    (*O)(1) = (d2 * N1(1) - d1 * N2(1)) / den;
    (*O)(2) = (d2 * N1(2) - d1 * N2(2)) / den;
}

__host__ __device__ Eigen::Vector3f compute_signed_dists(const Eigen::Vector3f N, const float d, const Triangle tr,
        const Eigen::Vector3f *pts) {
    Eigen::Vector3f dists;
    dists(0) = N(0) * pts[tr.v1](0) + N(1) * pts[tr.v1](1) + N(2) * pts[tr.v1](2) + d;
    dists(1) = N(0) * pts[tr.v2](0) + N(1) * pts[tr.v2](1) + N(2) * pts[tr.v2](2) + d;
    dists(2) = N(0) * pts[tr.v3](0) + N(1) * pts[tr.v3](1) + N(2) * pts[tr.v3](2) + d;
    return dists;
}

__host__ __device__ void compute_signed_dists_sep(const float Nx, const float Ny, const float Nz, const float d, 
            const float pt1_x, const float pt1_y, const float pt1_z,
            const float pt2_x, const float pt2_y, const float pt2_z, 
            const float pt3_x, const float pt3_y, const float pt3_z, 
            float* dists_x, float* dists_y, float* dists_z) {
    *dists_x = Nx * pt1_x + Ny * pt1_y + Nz * pt1_z + d;
    *dists_y = Nx * pt2_x + Ny * pt2_y + Nz * pt2_z + d;
    *dists_z = Nx * pt3_x + Ny * pt3_y + Nz * pt3_z + d;
}

__host__ __device__ bool no_overlap(const Eigen::Vector3f dists) {
    bool gz = dists(0) >= TOL || dists(1) >= TOL || dists(2) >= TOL;
    bool lz = dists(0) <= -1 * TOL || dists(1) <= -1 * TOL || dists(2) <= -1 * TOL;

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

__host__ __device__ float project_vertex(const Eigen::Vector3f V, const Eigen::Vector3f D, const Eigen::Vector3f O) {
    return D(0) * (V(0) - O(0)) + D(1) * (V(1) - O(1)) + D(2) * (V(2) - O(2));
}

__host__ __device__ float project_vertex_sep(const float Vx, const float Vy, const float Vz, const float Dx, const float Dy, const float Dz, const float Ox, const float Oy, const float Oz) {
    return Dx * (Vx - Ox) + Dy * (Vy - Oy) + Dz * (Vz - Oz);
}

__host__ __device__ void canonicalize_triangle(const Triangle t, const Eigen::Vector3f dists, Triangle *new_t, Eigen::Vector3f *new_dists) {
    if (dists(0) > 0 && dists(1) > 0 || dists(0) < 0 && dists(1) < 0) {
        new_t->v1 = t.v1;
        new_t->v2 = t.v3;
        new_t->v3 = t.v2;

        (*new_dists)(0) = dists(0);
        (*new_dists)(1) = dists(2);
        (*new_dists)(2) = dists(1);
    } else if (dists(0) > 0 && dists(2) > 0 || dists(0) < 0 && dists(2) < 0) {
        new_t->v1 = t.v1;
        new_t->v2 = t.v2;
        new_t->v3 = t.v3;

        (*new_dists)(0) = dists(0);
        (*new_dists)(1) = dists(1);
        (*new_dists)(2) = dists(2);
    } else {
        new_t->v1 = t.v2;
        new_t->v2 = t.v1;
        new_t->v3 = t.v3;

        (*new_dists)(0) = dists(1);
        (*new_dists)(1) = dists(0);
        (*new_dists)(2) = dists(2);
    }
}

__host__ __device__ void canonicalize_triangle_sep(const float dists_x, const float dists_y, const float dists_z, int *v1, int *v2, int *v3) {
    if (dists_x > 0 && dists_y> 0 || dists_x < 0 && dists_y < 0) {
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

__host__ __device__ float compute_parametric_variable(const Eigen::Vector3f v0, const Eigen::Vector3f v1,
        const float d0, const float d1, const Eigen::Vector3f D, const Eigen::Vector3f O) {
    float p_v0 = project_vertex(v0, D, O);
    float p_v1 = project_vertex(v1, D, O);

    return p_v0 + (p_v1 - p_v0) * d0 / (d0 - d1);
}

__host__ __device__ float compute_parametric_variable_sep(const float v0_x, const float v0_y, const float v0_z, const float v1_x, const float v1_y, const float v1_z, const float d0, const float d1, const float Dx, const float Dy, const float Dz, const float Ox, const float Oy, const float Oz) {
    float p_v0 = project_vertex_sep(v0_x, v0_y, v0_z, Dx, Dy, Dz, Ox, Oy, Oz);
    float p_v1 = project_vertex_sep(v1_x, v1_y, v1_z, Dx, Dy, Dz, Ox, Oy, Oz);

    return p_v0 + (p_v1 - p_v0) * d0 / (d0 - d1);
}

__host__ __device__ bool is_coplanar(const Eigen::Vector3f N1, const float d1, const Eigen::Vector3f N2, const float d2) {
    float ratio;
    bool started_ratio = false;
    for (int i = 0; i < 4; i++) {
        float p1, p2;
        if (i == 0) {
            p1 = N1(0);
            p2 = N2(0);
        } else if (i == 1) {
            p1 = N1(1);
            p2 = N2(1);
        } else if (i == 2) {
            p1 = N1(2);
            p2 = N2(2);
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
    const Eigen::Vector3f *rob_pts, const Triangle *obs_trs, const Eigen::Vector3f *obs_pts,
    bool *valid_conf) {

    for (int i = 0; i < num_confs; i++) {
        bool valid = true;

        // True only if we require coplanar analysis to determine whether or
        // not these intersect
        bool req_coplanar = false;
        for (int j = 0; j < num_rob_trs; j++) {
            Eigen::Vector3f Nr;
            float dr;
            compute_plane(rob_trs[j], &rob_pts[i * num_rob_pts], &Nr, &dr);

            for (int k = 0; k < num_obs_trs; k++) {
                Eigen::Vector3f distO = compute_signed_dists(Nr, dr, obs_trs[k], obs_pts);
                if (no_overlap(distO)) {
                    continue;
                }

                Eigen::Vector3f No;
                float do_;
                compute_plane(obs_trs[k], obs_pts, &No, &do_);

                if (is_coplanar(Nr, dr, No, do_)) {
                    req_coplanar = true;
                    continue;
                }

                Eigen::Vector3f distR = compute_signed_dists(No, do_, rob_trs[j], &rob_pts[i * num_rob_pts]);
                if (no_overlap(distR)) {
                    continue;
                }

                Eigen::Vector3f D, O;
                compute_intersect_line(Nr, dr, No, do_, &D, &O);

                Triangle ctr, cto;
                Eigen::Vector3f cdr, cdo;
                canonicalize_triangle(rob_trs[j], distR, &ctr, &cdr);
                canonicalize_triangle(obs_trs[k], distO, &cto, &cdo);

                float t_r01 = compute_parametric_variable(rob_pts[i * num_rob_pts + ctr.v1],
                    rob_pts[i * num_rob_pts + ctr.v2], cdr(0), cdr(1), D, O);

                float t_r12 = compute_parametric_variable(rob_pts[i * num_rob_pts + ctr.v2],
                    rob_pts[i * num_rob_pts + ctr.v3], cdr(1), cdr(2), D, O);

                float t_o01 = compute_parametric_variable(obs_pts[cto.v1],
                    obs_pts[cto.v2], cdo(0), cdo(1), D, O);

                float t_o12 = compute_parametric_variable(obs_pts[cto.v2],
                    obs_pts[cto.v3], cdo(1), cdo(2), D, O);

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
        int num_obs_trs, int num_obs_pts, const float *rob_pts_x, const float *rob_pts_y,
        const float *rob_pts_z, bool *valid_conf) {

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
        if (valid_conf[i])
            return;

        bool valid = true;

        // True only if we require coplanar analysis to determine whether or
        // not these intersect
        bool req_coplanar = false;
        for (int j = 0; j < num_rob_trs; j++) {
            // Load the robot triangle
            rob[0][0][tx] = rob_pts_x[i * num_rob_pts + base_rob_tri_v1[j]];
            rob[0][1][tx] = rob_pts_y[i * num_rob_pts + base_rob_tri_v1[j]];
            rob[0][2][tx] = rob_pts_z[i * num_rob_pts + base_rob_tri_v1[j]];
            rob[1][0][tx] = rob_pts_x[i * num_rob_pts + base_rob_tri_v2[j]];
            rob[1][1][tx] = rob_pts_y[i * num_rob_pts + base_rob_tri_v2[j]];
            rob[1][2][tx] = rob_pts_z[i * num_rob_pts + base_rob_tri_v2[j]];
            rob[2][0][tx] = rob_pts_x[i * num_rob_pts + base_rob_tri_v3[j]];
            rob[2][1][tx] = rob_pts_y[i * num_rob_pts + base_rob_tri_v3[j]];
            rob[2][2][tx] = rob_pts_z[i * num_rob_pts + base_rob_tri_v3[j]];

            // Compute the plane of the robot triangle
            compute_plane_sep(rob[0][0][tx], rob[0][1][tx], rob[0][2][tx], rob[1][0][tx], rob[1][1][tx], rob[1][2][tx], rob[2][0][tx], rob[2][1][tx], rob[2][2][tx], &(Nr[0][tx]), &(Nr[1][tx]), &(Nr[2][tx]), &(dr[tx]));

            for (int k = 0; k < num_obs_trs; k++) {
                // Load the obstacle triangle
                obs[0][0][tx] = base_obs_x[base_obs_tri_v1[k]];
                obs[0][1][tx] = base_obs_y[base_obs_tri_v1[k]];
                obs[0][2][tx] = base_obs_z[base_obs_tri_v1[k]];
                obs[1][0][tx] = base_obs_x[base_obs_tri_v2[k]];
                obs[1][1][tx] = base_obs_y[base_obs_tri_v2[k]];
                obs[1][2][tx] = base_obs_z[base_obs_tri_v2[k]];
                obs[2][0][tx] = base_obs_x[base_obs_tri_v3[k]];
                obs[2][1][tx] = base_obs_y[base_obs_tri_v3[k]];
                obs[2][2][tx] = base_obs_z[base_obs_tri_v3[k]];

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

__global__ void narrowPhaseKernel_coarse(int num_confs, int num_rob_trs, int num_rob_pts,
        int num_obs_trs, int num_obs_pts, const float *rob_pts_x, const float *rob_pts_y,
        const float *rob_pts_z, bool *valid_conf) {

    int tx = threadIdx.x;
    int ty = threadIdx.y;
    int bidx = ty * COARSEN_SZ + tx;
    int i = blockIdx.x * CONFS_PER_BLOCK + ty;
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
    __shared__ bool valid[CONFS_PER_BLOCK];

    if (i < num_confs) {
        if (valid_conf[i])
            return;

        valid[ty] = true;

        // True only if we require coplanar analysis to determine whether or
        // not these intersect
        bool req_coplanar = false;
        for (int j = 0; j < num_rob_trs; j++) {
            // Load the robot triangle
            rob[0][0][bidx] = rob_pts_x[i * num_rob_pts + base_rob_tri_v1[j]];
            rob[0][1][bidx] = rob_pts_y[i * num_rob_pts + base_rob_tri_v1[j]];
            rob[0][2][bidx] = rob_pts_z[i * num_rob_pts + base_rob_tri_v1[j]];
            rob[1][0][bidx] = rob_pts_x[i * num_rob_pts + base_rob_tri_v2[j]];
            rob[1][1][bidx] = rob_pts_y[i * num_rob_pts + base_rob_tri_v2[j]];
            rob[1][2][bidx] = rob_pts_z[i * num_rob_pts + base_rob_tri_v2[j]];
            rob[2][0][bidx] = rob_pts_x[i * num_rob_pts + base_rob_tri_v3[j]];
            rob[2][1][bidx] = rob_pts_y[i * num_rob_pts + base_rob_tri_v3[j]];
            rob[2][2][bidx] = rob_pts_z[i * num_rob_pts + base_rob_tri_v3[j]];

            // Compute the plane of the robot triangle
            compute_plane_sep(rob[0][0][bidx], rob[0][1][bidx], rob[0][2][bidx], rob[1][0][bidx], rob[1][1][bidx], rob[1][2][bidx], rob[2][0][bidx], rob[2][1][bidx], rob[2][2][bidx], &(Nr[0][bidx]), &(Nr[1][bidx]), &(Nr[2][bidx]), &(dr[bidx]));

            for (int kk = 0; kk < num_obs_trs; kk += COARSEN_SZ) {
                // All threads should stop if a collision was found
                // __syncwarp();
                if (!valid[ty])
                    break;

                int k = kk + tx;
                if (k < num_obs_trs) {
                    // Load the obstacle triangle
                    obs[0][0][bidx] = base_obs_x[base_obs_tri_v1[k]];
                    obs[0][1][bidx] = base_obs_y[base_obs_tri_v1[k]];
                    obs[0][2][bidx] = base_obs_z[base_obs_tri_v1[k]];
                    obs[1][0][bidx] = base_obs_x[base_obs_tri_v2[k]];
                    obs[1][1][bidx] = base_obs_y[base_obs_tri_v2[k]];
                    obs[1][2][bidx] = base_obs_z[base_obs_tri_v2[k]];
                    obs[2][0][bidx] = base_obs_x[base_obs_tri_v3[k]];
                    obs[2][1][bidx] = base_obs_y[base_obs_tri_v3[k]];
                    obs[2][2][bidx] = base_obs_z[base_obs_tri_v3[k]];

                    // Compute the distances between the robot plane and the obstacle triangle
                    compute_signed_dists_sep(Nr[0][bidx], Nr[1][bidx], Nr[2][bidx], dr[bidx], obs[0][0][bidx], obs[0][1][bidx], obs[0][2][bidx], obs[1][0][bidx], obs[1][1][bidx], obs[1][2][bidx], obs[2][0][bidx], obs[2][1][bidx], obs[2][2][bidx], &(distO[0][bidx]), &(distO[1][bidx]), &(distO[2][bidx]));

                    // Early exit if there is definitely no overlap
                    if (no_overlap_sep(distO[0][bidx], distO[1][bidx], distO[2][bidx])) {
                        continue;
                    }

                    // Compute the plane of the obstacle triangle
                    compute_plane_sep(obs[0][0][bidx], obs[0][1][bidx], obs[0][2][bidx], obs[1][0][bidx], obs[1][1][bidx], obs[1][2][bidx], obs[2][0][bidx], obs[2][1][bidx], obs[2][2][bidx], &(No[0][bidx]), &(No[1][bidx]), &(No[2][bidx]), &(do_[bidx]));

                    // Compute the distances between the obstacle plane and the robot triangle
                    compute_signed_dists_sep(No[0][bidx], No[1][bidx], No[2][bidx], do_[bidx], rob[0][0][bidx], rob[0][1][bidx], rob[0][2][bidx], rob[1][0][bidx], rob[1][1][bidx], rob[1][2][bidx], rob[2][0][bidx], rob[2][1][bidx], rob[2][2][bidx], &(distR[0][bidx]), &(distR[1][bidx]), &(distR[2][bidx]));

                    // Early exit if there is definitely no overlap
                    if (no_overlap_sep(distR[0][bidx], distR[1][bidx], distR[2][bidx])) {
                        continue;
                    }

                    // Make sure these two triangles are not coplanar
                    if (is_coplanar_sep(Nr[0][bidx], Nr[1][bidx], Nr[2][bidx], dr[bidx], No[0][bidx], No[1][bidx], No[2][bidx], do_[bidx])) {
                        req_coplanar = true;
                        continue;
                    }

                    // Compute the intersection line of these two planes
                    compute_intersect_line_sep(Nr[0][bidx], Nr[1][bidx], Nr[2][bidx], dr[bidx], No[0][bidx], No[1][bidx], No[2][bidx], do_[bidx], &(D[0][bidx]), &(D[1][bidx]), &(D[2][bidx]), &(O[0][bidx]), &(O[1][bidx]), &(O[2][bidx]));

                    // Canonicalize both triangles so that v1 and v3 are on one side of the line, and v2 is on the other
                    canonicalize_triangle_sep(distR[0][bidx], distR[1][bidx], distR[2][bidx], &(rv1[bidx]), &(rv2[bidx]), &(rv3[bidx]));
                    canonicalize_triangle_sep(distO[0][bidx], distO[1][bidx], distO[2][bidx], &(ov1[bidx]), &(ov2[bidx]), &(ov3[bidx]));

                    // Compute the intersection between the side of the triangle and the line
                    float t_r01 = compute_parametric_variable_sep(rob[rv1[bidx]][0][bidx], rob[rv1[bidx]][1][bidx], rob[rv1[bidx]][2][bidx], rob[rv2[bidx]][0][bidx], rob[rv2[bidx]][1][bidx], rob[rv2[bidx]][2][bidx], distR[rv1[bidx]][bidx], distR[rv2[bidx]][bidx], D[0][bidx], D[1][bidx], D[2][bidx], O[0][bidx], O[1][bidx], O[2][bidx]);
                    float t_r12 = compute_parametric_variable_sep(rob[rv2[bidx]][0][bidx], rob[rv2[bidx]][1][bidx], rob[rv2[bidx]][2][bidx], rob[rv3[bidx]][0][bidx], rob[rv3[bidx]][1][bidx], rob[rv3[bidx]][2][bidx], distR[rv2[bidx]][bidx], distR[rv3[bidx]][bidx], D[0][bidx], D[1][bidx], D[2][bidx], O[0][bidx], O[1][bidx], O[2][bidx]);
                    float t_o01 = compute_parametric_variable_sep(obs[ov1[bidx]][0][bidx], obs[ov1[bidx]][1][bidx], obs[ov1[bidx]][2][bidx], obs[ov2[bidx]][0][bidx], obs[ov2[bidx]][1][bidx], obs[ov2[bidx]][2][bidx], distO[ov1[bidx]][bidx], distO[ov2[bidx]][bidx], D[0][bidx], D[1][bidx], D[2][bidx], O[0][bidx], O[1][bidx], O[2][bidx]);
                    float t_o12 = compute_parametric_variable_sep(obs[ov2[bidx]][0][bidx], obs[ov2[bidx]][1][bidx], obs[ov2[bidx]][2][bidx], obs[ov3[bidx]][0][bidx], obs[ov3[bidx]][1][bidx], obs[ov3[bidx]][2][bidx], distO[ov2[bidx]][bidx], distO[ov3[bidx]][bidx], D[0][bidx], D[1][bidx], D[2][bidx], O[0][bidx], O[1][bidx], O[2][bidx]);

                    // There is no overlap
                    if (min(t_r01, t_r12) >= max(t_o01, t_o12)) {
                        continue;

                    // Also no overlap
                    } else if (min(t_o01, t_o12) >= max(t_r01, t_r12)) {
                        continue;

                    // There is overlap
                    } else {
                        valid[ty] = false;
                        req_coplanar = false;
                    }
                }
            }

            // Stop if we found a collision
            __syncwarp();
            if (!valid[ty])
                break;
        }

        if (req_coplanar)
            printf("Error: require coplanar intersection for configuration: %d\n", i);

        __syncthreads();
        if (tx == 0) {
            valid_conf[i] = valid[ty];
        }
    }
}

__global__ void narrowPhaseKernel(int num_confs, int num_rob_trs, int num_rob_pts,
        int num_obs_trs, int num_obs_pts, const Triangle *rob_trs,
        const Eigen::Vector3f *rob_pts, const Triangle *obs_trs, const Eigen::Vector3f *obs_pts,
        bool *valid_conf) {

    int i = blockDim.x * blockIdx.x + threadIdx.x;

    if (i < num_confs) {
        if (valid_conf[i])
            return;

        bool valid = true;

        // True only if we require coplanar analysis to determine whether or
        // not these intersect
        bool req_coplanar = false;
        for (int j = 0; j < num_rob_trs; j++) {
            Eigen::Vector3f Nr;
            float dr;
            //delete these when testing
            Triangle t = rob_trs[j];
            const Eigen::Vector3f* robot_pts = &rob_pts[i * num_rob_pts];
            Eigen::Vector3f *pNr = &Nr;
            float *pdr = &dr;
            compute_plane(t, robot_pts, pNr, pdr);

            for (int k = 0; k < num_obs_trs; k++) {
                Eigen::Vector3f distO = compute_signed_dists(Nr, dr, obs_trs[k], obs_pts);
                if (no_overlap(distO)) {
                    continue;
                }

                Eigen::Vector3f No;
                float do_;
                compute_plane(obs_trs[k], obs_pts, &No, &do_);

                if (is_coplanar(Nr, dr, No, do_)) {
                    req_coplanar = true;
                    continue;
                }

                Eigen::Vector3f distR = compute_signed_dists(No, do_, rob_trs[j], &rob_pts[i * num_rob_pts]);
                if (no_overlap(distR)) {
                    continue;
                }

                Eigen::Vector3f D, O;
                compute_intersect_line(Nr, dr, No, do_, &D, &O);

                Triangle ctr, cto;
                Eigen::Vector3f cdr, cdo;
                canonicalize_triangle(rob_trs[j], distR, &ctr, &cdr);
                canonicalize_triangle(obs_trs[k], distO, &cto, &cdo);

                float t_r01 = compute_parametric_variable(rob_pts[i * num_rob_pts + ctr.v1],
                    rob_pts[i * num_rob_pts + ctr.v2], cdr(0), cdr(1), D, O);

                float t_r12 = compute_parametric_variable(rob_pts[i * num_rob_pts + ctr.v2],
                    rob_pts[i * num_rob_pts + ctr.v3], cdr(1), cdr(2), D, O);

                float t_o01 = compute_parametric_variable(obs_pts[cto.v1],
                    obs_pts[cto.v2], cdo(0), cdo(1), D, O);

                float t_o12 = compute_parametric_variable(obs_pts[cto.v2],
                    obs_pts[cto.v3], cdo(1), cdo(2), D, O);

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

void narrowPhase_unopt(int num_confs, int num_rob_trs, int num_rob_pts,
        int num_obs_trs, int num_obs_pts, const Triangle *rob_trs,
        const Eigen::Vector3f *rob_pts, const Triangle *obs_trs, const Eigen::Vector3f *obs_pts,
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

    Eigen::Vector3f *d_rob_pts;
    cudaMalloc(&d_rob_pts, num_confs * num_rob_pts * sizeof(Eigen::Vector3f));
    cudaMemcpy(d_rob_pts, rob_pts, num_confs * num_rob_pts * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice);
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

    Eigen::Vector3f *d_obs_pts;
    cudaMalloc(&d_obs_pts, num_obs_pts * sizeof(Eigen::Vector3f));
    cudaMemcpy(d_obs_pts, obs_pts, num_obs_pts * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice);
    cudaDeviceSynchronize();
    fflush(stdout);
    #if VERBOSE
        err = cudaGetLastError();
        printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));
    #endif

    bool *d_valid_conf;
    cudaMalloc(&d_valid_conf, num_confs * sizeof(bool));
    cudaMemcpy(d_valid_conf, valid_conf, num_confs * sizeof(bool), cudaMemcpyHostToDevice);
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
        const Eigen::Vector3f *rob_pts, const Triangle *obs_trs, const Eigen::Vector3f *obs_pts,
        bool *valid_conf) {

    bool *d_valid_conf;
    cudaMalloc(&d_valid_conf, num_confs * sizeof(bool));
    cudaMemcpy(d_valid_conf, valid_conf, num_confs * sizeof(bool), cudaMemcpyHostToDevice);
    cudaDeviceSynchronize();

    // Note: This function is currently a no-op because the narrow phase is called
    // directly from the broad-phase-fused kernels. The coarse and sep kernel variants
    // are invoked from broadPhaseFused_sep() and broadPhaseFused() respectively.

    cudaFree(d_valid_conf);
}
