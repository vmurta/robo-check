#pragma once
#include "./Utils.h"
// NOTE: everything below between the stars had the following headers in the initial version of this file, need to reevaluate which ones are actually needed
// #define NUM_CONFS_PER_BLOCK 32
// #define MEGA_BLOCK_SIZE 32
// #define TRIANGLE_BUFFER_SIZE 128
#define TOL 1e-6

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

//TODO: figure out how these functions work
//it seems like they just reorder the vertices of the triangle so that... 
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

//TODO: template float / double
__host__ __device__ void canonicalize_triangle ( Eigen::Vector3d &v1, Eigen::Vector3d &v2, Eigen::Vector3d &v3, Eigen::Vector3d &dists){
    
    if (dists(0) > 0 && dists(1) > 0 || dists(0) < 0 && dists(1) < 0) {
        Eigen::Vector3d swap = v2;
        v2 = v3;
        v3 = swap;

        swap = dists;
        dists(1) = swap(2);
        dists(2) = swap(1);
    } else if (dists(0) > 0 && dists(2) > 0 || dists(0) < 0 && dists(2) < 0) {
        // do nothing

    } else {
        Eigen::Vector3d swap = v1;
        v1 = v2;
        v2 = swap;

        swap = dists;
        dists(0) = swap(1);
        dists(1) = swap(0);
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


// TODO: investigate a more numerically stable way to do this
// TODO: template this to use doubles / floats
__host__ __device__ void la_solve(const double A1, const double A2, const double A3, const double A4,
        const double b1, const double b2, double *x1, double *x2) {

    if (isclose(A1, 0)) {
        *x2 = b1 / A2;
        *x1 = (b2 - A4 * *x2) / A3;

    } else {
        double A1A4 = A1 * A4;
        double A2A3 = A2 * A3;
        double A3b1 = A3 * b1;
        double A1b2 = A1 * b2;

        *x2 = (A3b1 - A1b2) / (A2A3 - A1A4);
        *x1 = (b1 - A2 * *x2) / A1;
    }
}

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
//TODO: template this to use doubles
__host__ __device__ Eigen::Vector3d compute_signed_dists(   const Eigen::Vector3d N, const double d, 
                                                            const Eigen::Vector3d &v1, const Eigen::Vector3d &v2, const Eigen::Vector3d &v3) {
    Eigen::Vector3d dists;
    dists(0) = N(0) * v1(0) + N(1) * v1(1) + N(2) * v1(2) + d;
    dists(1) = N(0) * v2(0) + N(1) * v2(1) + N(2) * v2(2) + d;
    dists(2) = N(0) * v3(0) + N(1) * v3(1) + N(2) * v3(2) + d;
    return dists;
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

//TODO: template this to use doubles
__host__ __device__ bool no_overlap(const Eigen::Vector3d dists) {
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


__host__ __device__ void compute_intersect_line(const Eigen::Vector3d N1, const double d1,
        const Eigen::Vector3d N2, const double d2, Eigen::Vector3d *D, Eigen::Vector3d *O) {

    (*D)(0) = N1(1) * N2(2) - N1(2) * N2(1);
    (*D)(1) = N1(2) * N2(0) - N1(0) * N2(2);
    (*D)(2) = N1(0) * N2(1) - N1(1) * N2(0);

    // Set t = 1
    double x1, x2;
    if (!isclose((*D)(2), 0)) {
        la_solve(N1(0), N1(1), N2(0), N2(1), -d1, -d2, &x1, &x2);
        (*O)(0) = x1;
        (*O)(1) = x2;
        (*O)(2) = 0;

    } else if (!isclose((*D)(1), 0)) {
        la_solve(N1(0), N1(2), N2(0), N2(2), -d1, -d2, &x1, &x2);
        (*O)(0) = x1;
        (*O)(1) = 0;
        (*O)(2) = x2;

    } else {
        la_solve(N1(1), N1(2), N2(1), N2(2), -d1, -d2, &x1, &x2);
        (*O)(0) = 0;
        (*O)(1) = x1;
        (*O)(2) = x2;
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

__host__ __device__ float project_vertex(const Eigen::Vector3f &V, const Eigen::Vector3f &D, const Eigen::Vector3f &O) {
    return D(0) * (V(0) - O(0)) + D(1) * (V(1) - O(1)) + D(2) * (V(2) - O(2));
}

__host__ __device__ double project_vertex(const Eigen::Vector3d &V, const Eigen::Vector3d &D, const Eigen::Vector3d &O) {
    return D(0) * (V(0) - O(0)) + D(1) * (V(1) - O(1)) + D(2) * (V(2) - O(2));
}

__host__ __device__ float project_vertex_sep(const float Vx, const float Vy, const float Vz, const float Dx, const float Dy, const float Dz, const float Ox, const float Oy, const float Oz) {
    return Dx * (Vx - Ox) + Dy * (Vy - Oy) + Dz * (Vz - Oz);
}

__host__ __device__ float compute_parametric_variable(const Eigen::Vector3f v0, const Eigen::Vector3f v1,
        const float d0, const float d1, const Eigen::Vector3f D, const Eigen::Vector3f O) {
    float p_v0 = project_vertex(v0, D, O);
    float p_v1 = project_vertex(v1, D, O);

    return p_v0 + (p_v1 - p_v0) * d0 / (d0 - d1);
}

//TODO: template this to use doubles
__host__ __device__ double compute_parametric_variable(const Eigen::Vector3d v0, const Eigen::Vector3d v1,
        const double d0, const double d1, const Eigen::Vector3d D, const Eigen::Vector3d O) {
    double p_v0 = project_vertex(v0, D, O);
    double p_v1 = project_vertex(v1, D, O);

    return p_v0 + (p_v1 - p_v0) * d0 / (d0 - d1);
}
// __host__ __device__
// float compute_parametric_variable(
//     const Eigen::Vector3f &v0,
//     const Eigen::Vector3f &v1,
//     float d0, float d1,
//     const Eigen::Vector3f &D,
//     const Eigen::Vector3f &O,
//     bool delete_me = false)
// {
//     float p_v0 = project_vertex(v0, D, O);
//     float p_v1 = project_vertex(v1, D, O);

//     if (delete_me) {
//         printf("p_v0: %f, p_v1: %f, d0: %f, d1: %f\n", p_v0, p_v1, d0, d1);
//     }
//     float denom = d0 - d1;
//     if (fabsf(denom) < 1e-6f) {
//         return p_v0; // fallback
//     }

//     float t = d0 / denom;

//     return p_v0 + (p_v1 - p_v0) * t;
// }

__host__ __device__ float compute_parametric_variable_sep(const float v0_x, const float v0_y, const float v0_z, const float v1_x, const float v1_y, const float v1_z, const float d0, const float d1, const float Dx, const float Dy, const float Dz, const float Ox, const float Oy, const float Oz) {
    float p_v0 = project_vertex_sep(v0_x, v0_y, v0_z, Dx, Dy, Dz, Ox, Oy, Oz);
    float p_v1 = project_vertex_sep(v1_x, v1_y, v1_z, Dx, Dy, Dz, Ox, Oy, Oz);

    return p_v0 + (p_v1 - p_v0) * d0 / (d0 - d1);
}


// #ifndef COALESCE
/***************************************************************************************************************************************/

extern __constant__ Eigen::Vector3f base_robot_vertices[NUM_ROB_VERTICES];
extern __constant__ Triangle base_robot_triangles[MAX_NUM_ROBOT_TRIANGLES];
extern __constant__ Eigen::Vector3f base_obs_vertices[NUM_ROB_VERTICES];
extern __constant__ Triangle base_obs_triangles[MAX_NUM_ROBOT_TRIANGLES];



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

__host__ __device__ bool is_coplanar(const Eigen::Vector3d N1, const double d1, const Eigen::Vector3d N2, const double d2) {
    double ratio;
    bool started_ratio = false;
    for (int i = 0; i < 4; i++) {
        double p1, p2;
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

//TODO: template this to use doubles
__host__ __device__ void compute_plane(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2, const Eigen::Vector3f &v3, Eigen::Vector3d &N, double &d) {
    Eigen::Vector3d v2_v1(v2[0] - v1[0], v2[1] - v1[1], v2[2] - v1[2]);
    Eigen::Vector3d v3_v2(v3[0] - v2[0], v3[1] - v2[1], v3[2] - v2[2]);

    N[0] = v2_v1[1] * v3_v2[2] - v2_v1[2] * v3_v2[1];
    N[1] = v2_v1[2] * v3_v2[0] - v2_v1[0] * v3_v2[2];
    N[2] = v2_v1[0] * v3_v2[1] - v2_v1[1] * v3_v2[0];

    d = -1 * (N[0] * v1[0] + N[1] * v1[1] + N[2] * v1[2]);
}

//TODO: template this to use doubles
__host__ __device__ void compute_plane(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2, const Eigen::Vector3d &v3, Eigen::Vector3d &N, double &d) {
    Eigen::Vector3d v2_v1(v2[0] - v1[0], v2[1] - v1[1], v2[2] - v1[2]);
    Eigen::Vector3d v3_v2(v3[0] - v2[0], v3[1] - v2[1], v3[2] - v2[2]);

    N[0] = v2_v1[1] * v3_v2[2] - v2_v1[2] * v3_v2[1];
    N[1] = v2_v1[2] * v3_v2[0] - v2_v1[0] * v3_v2[2];
    N[2] = v2_v1[0] * v3_v2[1] - v2_v1[1] * v3_v2[0];

    d = -1 * (N[0] * v1[0] + N[1] * v1[1] + N[2] * v1[2]);
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


__host__ __device__ AABB generateTriangleAABB(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2, const Eigen::Vector3f &p3){
    AABB aabb;
    aabb.x_min = min(p1[0], min(p2[0], p3[0]));
    aabb.y_min = min(p1[1], min(p2[1], p3[1]));
    aabb.z_min = min(p1[2], min(p2[2], p3[2]));
    aabb.x_max = max(p1[0], max(p2[0], p3[0]));
    aabb.y_max = max(p1[1], max(p2[1], p3[1]));
    aabb.z_max = max(p1[2], max(p2[2], p3[2]));
    return aabb;
}

void generateTriAABBs(const std::vector<Triangle> &triangles, const std::vector<Eigen::Vector3f> &points, std::vector<AABB> &aabbs){
    for (int i = 0; i < triangles.size(); i++){
        aabbs.push_back(generateTriangleAABB(points[triangles[i].v1], points[triangles[i].v2], points[triangles[i].v3]));
    }
}

//TODO: make it work assuming that the OBBs of the triangles already intersect (should be able to save some steps)
//TODO: make it work with templated floats/doubles
__device__ bool triangles_valid(    Eigen::Vector3f f_rob_v1, Eigen::Vector3f f_rob_v2,  Eigen::Vector3f f_rob_v3,
                                    Eigen::Vector3f f_obs_v1, Eigen::Vector3f f_obs_v2, Eigen::Vector3f f_obs_v3) {       
    
    Eigen::Vector3d rob_v1 = f_rob_v1.cast <double> ();
    Eigen::Vector3d rob_v2 = f_rob_v2.cast <double> ();
    Eigen::Vector3d rob_v3 = f_rob_v3.cast <double> ();
    Eigen::Vector3d obs_v1 = f_obs_v1.cast <double> ();
    Eigen::Vector3d obs_v2 = f_obs_v2.cast <double> ();
    Eigen::Vector3d obs_v3 = f_obs_v3.cast <double> ();
    Eigen::Vector3d Nr;
    double dr;
    bool valid = true;
    bool req_coplanar = false;

    compute_plane(f_rob_v1, f_rob_v2, f_rob_v3, Nr, dr);

    Eigen::Vector3d distO = compute_signed_dists(Nr, dr, obs_v1, obs_v2, obs_v3);

    // Eigen::Vector3f distO = compute_signed_dists(Nr, dr, base_obs_vertices[obs_tri.v1], base_obs_vertices[obs_tri.v2], base_obs_vertices[obs_tri.v3]);
    if (no_overlap(distO)) {
        return true;
    }

    Eigen::Vector3d No;
    double do_;
    compute_plane(obs_v1, obs_v2, obs_v3, No, do_);

    //TODO: i think this code can't handle coplanar triangles? return false if that's the case
    if (is_coplanar(Nr, dr, No, do_)) {
        req_coplanar = true;
        // printf("coplanar triangles detected, returning false\n");
        return false;
    }

    Eigen::Vector3d distR = compute_signed_dists(No, do_, rob_v1, rob_v2, rob_v3);
    if (no_overlap(distR)) {
        return true;
    }

    Eigen::Vector3d D, O;
    compute_intersect_line(Nr, dr, No, do_, &D, &O);

    //TODO: can we pre-canonicalize triangles in the mesh?
    canonicalize_triangle(rob_v1, rob_v2, rob_v3, distR);
    canonicalize_triangle(obs_v1, obs_v2, obs_v3, distO);

    double t_r01 = compute_parametric_variable(rob_v1,
        rob_v2, distR[0], distR[1], D, O);

    double t_r12 = compute_parametric_variable(rob_v2,
        rob_v3, distR[1], distR[2], D, O);

    double t_o01 = compute_parametric_variable(obs_v1,
        obs_v2, distO[0], distO[1], D, O);

    double t_o12 = compute_parametric_variable(obs_v2,
        obs_v3, distO[1], distO[2], D, O);

    // There is no overlap
    if (min(t_r01, t_r12) > max(t_o01, t_o12)) {
        return true;

    // Also no overlap
    } else if (min(t_o01, t_o12) > max(t_r01, t_r12)) {
        return true;

    // There is overlap
    } else {
        // printf("triangles overlap, returning false with values t_r01=%f, t_r12=%f, t_o01=%f, t_o12=%f\n", t_r01, t_r12, t_o01, t_o12);
        req_coplanar = false;
        return false;
    }
}
