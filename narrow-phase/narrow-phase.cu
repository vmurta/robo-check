#include "narrow-phase.hu"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#define TOL 1e-10
#define BLOCK_SIZE 128

bool isclose(float v1, float v2) {

    if (abs(v1) < TOL && abs(v2) < TOL) {
        return true;
    } else if (abs(v1) < TOL) {
        return false;
    } else if (abs(v2) < TOL) {
        return false;
    } else {
        return abs((v1 - v2) / v1) < 1e-10;
    }
}

bool veq(const Vector3f v1, const Vector3f v2) {
    return isclose(v1.x, v2.x) && isclose(v1.y, v2.y) && isclose(v1.z, v2.z);
}

bool teq(const Triangle self_tr, const Vector3f *self_pts,
        const Triangle other_tr, const Vector3f *other_pts) {
    return veq(self_pts[self_tr.v1], other_pts[other_tr.v1]) &&
        veq(self_pts[self_tr.v2], other_pts[other_tr.v2]) &&
        veq(self_pts[self_tr.v3], other_pts[other_tr.v3]);
}

void compute_plane(const Triangle tr, const Vector3f *pts, Vector3f *N,
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

Vector3f compute_signed_dists(const Vector3f N, const float d, const Triangle tr,
        const Vector3f *pts) {
    Vector3f dists;
    dists.x = N.x * pts[tr.v1].x + N.y * pts[tr.v1].y + N.z * pts[tr.v1].z + d;
    dists.y = N.x * pts[tr.v2].x + N.y * pts[tr.v2].y + N.z * pts[tr.v2].z + d;
    dists.z = N.x * pts[tr.v3].x + N.y * pts[tr.v3].y + N.z * pts[tr.v3].z + d;
    return dists;
}

bool no_overlap(const Vector3f dists) {
    bool gz = dists.x > TOL || dists.y > TOL || dists.z > TOL;
    bool lz = dists.x < -1 * TOL || dists.y < -1 * TOL || dists.z < -1 * TOL;

    return !(gz && lz);
}

bool yes_overlap(const Vector3f dists) {
    return isclose(dists.x, 0) || isclose(dists.y, 0) || isclose(dists.z, 0);
}

// TODO: investigate a more numerically stable way to do this
void la_solve(const float A1, const float A2, const float A3, const float A4,
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

void compute_intersect_line(const Vector3f N1, const float d1,
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

float project_vertex(const Vector3f V, const Vector3f D, const Vector3f O) {
    return D.x * (V.x - O.x) + D.y * (V.y - O.y) + D.z * (V.z - O.z);
}

void canonicalize_triangle(const Triangle t, const Vector3f dists, Triangle *new_t, Vector3f *new_dists) {
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

float compute_parametric_variable(const Vector3f v0, const Vector3f v1,
        const float d0, const float d1, const Vector3f D, const Vector3f O) {
    float p_v0 = project_vertex(v0, D, O);
    float p_v1 = project_vertex(v1, D, O);

    return p_v0 + (p_v1 - p_v0) * d0 / (d0 - d1);
}


// true if no collision
void narrowPhaseBaseline(int num_confs, int num_rob_trs, int num_rob_pts,
    int num_obs_trs, int num_obs_pts, const Triangle *rob_trs,
    const Vector3f *rob_pts, const Triangle *obs_trs, const Vector3f *obs_pts,
    bool *valid_conf) {

    for (int i = 0; i < num_confs; i++) {
        bool valid = true;
        for (int j = 0; j < num_rob_trs; j++) {
            Vector3f Nr;
            float dr;
            compute_plane(rob_trs[j], &rob_pts[i * num_rob_pts], &Nr, &dr);

            for (int k = 0; k < num_obs_trs; k++) {
                Vector3f distR = compute_signed_dists(Nr, dr, obs_trs[k], obs_pts);
                // If no overlap is not detected (i.e., there may be an overlap,
                // We will interpret it for now as *there is definitely an overlap*
                if (!no_overlap(distR)) {
                    valid = false;
                    break;
                }
            }

            // Stop if we found a collision
            if (!valid)
                break;
        }
        valid_conf[i] = valid;
    }
}

__global__ void narrowPhaseKernel(int num_confs, int num_rob_trs, int num_rob_pts,
        int num_obs_trs, int num_obs_pts, const Triangle *rob_trs,
        const Vector3f *rob_pts, const Triangle *obs_trs, const Vector3f *obs_pts,
        bool *valid_conf) {

    int i = blockDim.x * blockIdx.x + threadIdx.x;
    if (i < num_confs) {
        valid_conf[i] = true;
    }
}

__global__ void kern() {
    printf("foo ");
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

    // Copy the data onto the device
    Triangle *d_rob_trs;
    cudaMalloc(&d_rob_trs, num_rob_trs * sizeof(Triangle));
    cudaMemcpy(d_rob_trs, rob_trs, num_rob_trs * sizeof(Triangle), cudaMemcpyHostToDevice);

    Vector3f *d_rob_pts;
    cudaMalloc(&d_rob_pts, num_confs * num_rob_pts * sizeof(Vector3f));
    cudaMemcpy(d_rob_pts, rob_pts, num_confs * num_rob_pts * sizeof(Vector3f), cudaMemcpyHostToDevice);

    Triangle *d_obs_trs;
    cudaMalloc(&d_obs_trs, num_obs_trs * sizeof(Triangle));
    cudaMemcpy(d_obs_trs, obs_trs, num_obs_trs * sizeof(Triangle), cudaMemcpyHostToDevice);
    Vector3f *d_obs_pts;

    cudaMalloc(&d_obs_pts, num_obs_pts * sizeof(Vector3f));
    cudaMemcpy(d_obs_pts, obs_pts, num_obs_pts * sizeof(Vector3f), cudaMemcpyHostToDevice);

    bool *d_valid_conf;
    cudaMalloc(&d_valid_conf, num_confs * sizeof(bool));

    // Call the kernel
    const int numBlocks = (num_confs - 1) / BLOCK_SIZE + 1;

    dim3 a(numBlocks, 1, 1);
    dim3 b(BLOCK_SIZE, 1, 1);
    narrowPhaseKernel<<<(num_confs - 1) / BLOCK_SIZE + 1, BLOCK_SIZE>>>(num_confs, num_rob_trs,
        num_rob_pts, num_obs_trs, num_obs_pts, d_rob_trs, d_rob_pts, d_obs_trs,
        d_obs_pts, d_valid_conf);
    // kern<<<numBlocks, BLOCK_SIZE>>>();
    // kern<<<1, 256>>>();

    cudaDeviceSynchronize();
    fflush(stdout);
    cudaError_t err = cudaGetLastError();
    printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));

    // Copy the data back
    cudaMemcpy(valid_conf, d_valid_conf, num_confs * sizeof(bool), cudaMemcpyDeviceToHost);

    // Free the memory
    cudaFree(d_rob_trs);
    cudaFree(d_rob_pts);
    cudaFree(d_obs_trs);
    cudaFree(d_rob_pts);
    cudaFree(d_valid_conf);
}
