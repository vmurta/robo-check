#include "narrow-phase.hu"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

// Machine epsilon for floats is 1e-7
// https://en.wikipedia.org/wiki/Machine_epsilon#Values_for_standard_hardware_arithmetics
#define TOL 1e-6
#define BLOCK_SIZE 128

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

__host__ __device__ void canonicalize_triangle_sep(const int t_v1, const int t_v2, const int t_v3, const float dists_x, const float dists_y, const float dists_z, int *new_t_v1, int *new_t_v2, int *new_t_v3, float *new_dists_x, float *new_dists_y, float *new_dists_z) {
    if (dists_x > 0 && dists_y > 0 || dists_x < 0 && dists_y < 0) {
        *new_t_v1 = t_v1;
        *new_t_v2 = t_v3;
        *new_t_v3 = t_v2;

        *new_dists_x = dists_x;
        *new_dists_y = dists_z;
        *new_dists_z = dists_y;
    } else if (dists_x > 0 && dists_z > 0 || dists_x < 0 && dists_z < 0) {
        *new_t_v1 = t_v1;
        *new_t_v2 = t_v2;
        *new_t_v3 = t_v3;

        *new_dists_x = dists_x;
        *new_dists_y = dists_y;
        *new_dists_z = dists_z;
    } else {
        *new_t_v1 = t_v2;
        *new_t_v2 = t_v1;
        *new_t_v3 = t_v3;

        *new_dists_x = dists_y;
        *new_dists_y = dists_x;
        *new_dists_z = dists_z;
    }
}

__host__ __device__ float compute_parametric_variable(const Vector3f v0, const Vector3f v1,
        const float d0, const float d1, const Vector3f D, const Vector3f O) {
    float p_v0 = project_vertex(v0, D, O);
    float p_v1 = project_vertex(v1, D, O);

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
                    // if (j == 56){
                        // printf("Found collision at obstacle triangle %d and robot triangle %d\nt_r01: %f, t_r12: %f, t_o01: %f, t_o12: %f\n", k, j,  t_r01, t_r12, t_o01, t_o12);
                        printf("Robot Triangle 0 coordinates: (%f, %f, %f), (%f, %f, %f), (%f, %f, %f)\nObstacle Triangle %d coordinates: (%f, %f, %f), (%f, %f, %f), (%f, %f, %f)\n", 
                                rob_pts[t.v1].x, rob_pts[t.v1].y, rob_pts[t.v1].z, rob_pts[t.v2].x, rob_pts[t.v2].y, rob_pts[t.v2].z, rob_pts[t.v3].x, rob_pts[t.v3].y, rob_pts[t.v3].z,
                                k, obs_pts[obs_trs[k].v1].x, obs_pts[obs_trs[k].v1].y, obs_pts[obs_trs[k].v1].z, obs_pts[obs_trs[k].v2].x, obs_pts[obs_trs[k].v2].y, obs_pts[obs_trs[k].v2].z, obs_pts[obs_trs[k].v3].x, obs_pts[obs_trs[k].v3].y, obs_pts[obs_trs[k].v3].z);
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

void narrowPhase(int num_confs, int num_rob_trs, int num_rob_pts,
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
    cudaError_t err = cudaGetLastError();
    printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));

    // Copy the data onto the device
    Triangle *d_rob_trs;
    cudaMalloc(&d_rob_trs, num_rob_trs * sizeof(Triangle));
    cudaMemcpy(d_rob_trs, rob_trs, num_rob_trs * sizeof(Triangle), cudaMemcpyHostToDevice);
    cudaDeviceSynchronize();
    fflush(stdout);
    err = cudaGetLastError();
    printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));

    Vector3f *d_rob_pts;
    cudaMalloc(&d_rob_pts, num_confs * num_rob_pts * sizeof(Vector3f));
    cudaMemcpy(d_rob_pts, rob_pts, num_confs * num_rob_pts * sizeof(Vector3f), cudaMemcpyHostToDevice);
    cudaDeviceSynchronize();
    fflush(stdout);
    err = cudaGetLastError();
    printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));

    Triangle *d_obs_trs;
    cudaMalloc(&d_obs_trs, num_obs_trs * sizeof(Triangle));
    cudaMemcpy(d_obs_trs, obs_trs, num_obs_trs * sizeof(Triangle), cudaMemcpyHostToDevice);
    cudaDeviceSynchronize();
    fflush(stdout);
    err = cudaGetLastError();
    printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));

    Vector3f *d_obs_pts;
    cudaMalloc(&d_obs_pts, num_obs_pts * sizeof(Vector3f));
    cudaMemcpy(d_obs_pts, obs_pts, num_obs_pts * sizeof(Vector3f), cudaMemcpyHostToDevice);
    cudaDeviceSynchronize();
    fflush(stdout);
    err = cudaGetLastError();
    printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));

    bool *d_valid_conf;
    cudaMalloc(&d_valid_conf, num_confs * sizeof(bool));
    cudaDeviceSynchronize();
    fflush(stdout);
    err = cudaGetLastError();
    printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));

    // Call the kernel;
    narrowPhaseKernel<<<(num_confs - 1) / BLOCK_SIZE + 1, BLOCK_SIZE>>>(num_confs, num_rob_trs,
        num_rob_pts, num_obs_trs, num_obs_pts, d_rob_trs, d_rob_pts, d_obs_trs,
        d_obs_pts, d_valid_conf);

    cudaDeviceSynchronize();
    fflush(stdout);
    err = cudaGetLastError();
    printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));

    // Copy the data back
    cudaMemcpy(valid_conf, d_valid_conf, num_confs * sizeof(bool), cudaMemcpyDeviceToHost);
    cudaDeviceSynchronize();
    fflush(stdout);
    err = cudaGetLastError();
    printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));

    // Free the memory
    cudaFree(d_rob_trs);
    cudaDeviceSynchronize();
    fflush(stdout);
    err = cudaGetLastError();
    printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));

    cudaFree(d_rob_pts);
    cudaDeviceSynchronize();
    fflush(stdout);
    err = cudaGetLastError();
    printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));

    cudaFree(d_obs_trs);
    cudaDeviceSynchronize();
    fflush(stdout);
    err = cudaGetLastError();
    printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));

    cudaFree(d_obs_pts);
    cudaDeviceSynchronize();
    fflush(stdout);
    err = cudaGetLastError();
    printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));

    cudaFree(d_valid_conf);
    cudaDeviceSynchronize();
    fflush(stdout);
    err = cudaGetLastError();
    printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));

}
