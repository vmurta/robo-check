#include "MegaKernel.hu"

#define NUM_CONFS_PER_BLOCK 32
#define MEGA_BLOCK_SIZE 32
#define TRIANGLE_BUFFER_SIZE 128

// Set -DLOCAL_TESTING=1 to run CPU tests on local machine (not on rai)
__constant__ Vector3f base_robot_vertices[NUM_ROB_VERTICES];
__constant__ Triangle base_robot_triangles[MAX_NUM_ROBOT_TRIANGLES];
__constant__ Vector3f base_obs_vertices[NUM_ROB_VERTICES];
__constant__ Triangle base_obs_triangles[MAX_NUM_ROBOT_TRIANGLES];

#if(LOCAL_TESTING == 1)
#include <fcl/fcl.h>
#include "../Utils.h"
#endif

#if(LOCAL_TESTING == 1)
inline bool verticesEqual(const Vector3f &v1, const fcl::Vector3f &v2){
  return (fabs(v1.x -v2[0]) +
            fabs(v1.y -v2[1]) +
            fabs(v1.z -v2[2]) < 1e-5);
  // return false;
}

void generateAABBBaseline_fcl(fcl::Vector3f* vertices, unsigned int numVertices,
                    unsigned int numConfigs, AABB* botBounds)
{
    // Loop over every configuration
    for(int i = 0; i < numConfigs; ++i)
    {
        // Loop over every vertex in each configuration
        unsigned int configOffset = i * numVertices;
        botBounds[i].x_min = vertices[configOffset][0];
        botBounds[i].y_min = vertices[configOffset][1];
        botBounds[i].z_min = vertices[configOffset][2];
        botBounds[i].x_max = vertices[configOffset][0];
        botBounds[i].y_max = vertices[configOffset][1];
        botBounds[i].z_max = vertices[configOffset][2];
        for(int j = 0; j < numVertices; ++j)
        {
            botBounds[i].x_min = min(botBounds[i].x_min, vertices[configOffset + j][0]);
            botBounds[i].y_min = min(botBounds[i].y_min, vertices[configOffset + j][1]);
            botBounds[i].z_min = min(botBounds[i].z_min, vertices[configOffset + j][2]);
            botBounds[i].x_max = max(botBounds[i].x_max, vertices[configOffset + j][0]);
            botBounds[i].y_max = max(botBounds[i].y_max, vertices[configOffset + j][1]);
            botBounds[i].z_max = max(botBounds[i].z_max, vertices[configOffset + j][2]);
        }
    }
}
#endif


inline __device__ bool overlaps(const AABB &a1, const AABB &a2){
    return (a1.x_min <= a2.x_max && a1.x_max >= a2.x_min) &&
           (a1.y_min <= a2.y_max && a1.y_max >= a2.y_min) &&
           (a1.z_min <= a2.z_max && a1.z_max >= a2.z_min);
}

__host__ __device__ AABB generateTriangleAABB(const Vector3f &p1, const Vector3f &p2, const Vector3f &p3){
    AABB aabb;
    aabb.x_min = min(p1.x, min(p2.x, p3.x));
    aabb.y_min = min(p1.y, min(p2.y, p3.y));
    aabb.z_min = min(p1.z, min(p2.z, p3.z));
    aabb.x_max = max(p1.x, max(p2.x, p3.x));
    aabb.y_max = max(p1.y, max(p2.y, p3.y));
    aabb.z_max = max(p1.z, max(p2.z, p3.z));
    return aabb;
}

void generateTriAABBs(const std::vector<Triangle> &triangles, const std::vector<Vector3f> &points, std::vector<AABB> &aabbs){
    for (int i = 0; i < triangles.size(); i++){
        aabbs.push_back(generateTriangleAABB(points[triangles[i].v1], points[triangles[i].v2], points[triangles[i].v3]));
    }
}
__device__ bool triangles_valid(const Triangle &rob_tri, const Triangle &obs_tri,
                                    const Vector3f *rob_pts) {
    Vector3f Nr;
    float dr;
    bool valid = true;
    bool req_coplanar = false;

    compute_plane(rob_tri, rob_pts, &Nr, &dr);

    if  (blockIdx.x == 24 && threadIdx.x == 16){
        // printf("Obstacle triangle indices are %d, %d, %d\n", obs_tri.v1, obs_tri.v2, obs_tri.v3);
        // if (-1089013080 == obs_tri.v1) {
        //     printf("yes really \n");
        // }
    }
    Vector3f obs_v1 = base_obs_vertices[obs_tri.v1];
    Vector3f obs_v2 = base_obs_vertices[obs_tri.v2];
    Vector3f obs_v3 = base_obs_vertices[obs_tri.v3];

    Vector3f distO = compute_signed_dists(Nr, dr, obs_v1, obs_v2, obs_v3);

    // Vector3f distO = compute_signed_dists(Nr, dr, base_obs_vertices[obs_tri.v1], base_obs_vertices[obs_tri.v2], base_obs_vertices[obs_tri.v3]);
    if (no_overlap(distO)) {
        return true;
    }

    Vector3f No;
    float do_;
    compute_plane(obs_v1, obs_v2, obs_v3, &No, &do_);

    if (is_coplanar(Nr, dr, No, do_)) {
        req_coplanar = true;
        return true;
    }

    Vector3f distR = compute_signed_dists(No, do_, rob_tri, rob_pts);
    if (no_overlap(distR)) {
        return true;
    }

    Vector3f D, O;
    compute_intersect_line(Nr, dr, No, do_, &D, &O);

    Triangle ctr, cto;
    Vector3f cdr, cdo;
    canonicalize_triangle(rob_tri, distR, &ctr, &cdr);
    canonicalize_triangle(obs_tri, distO, &cto, &cdo);

    float t_r01 = compute_parametric_variable(rob_pts[ctr.v1],
        rob_pts[ctr.v2], cdr.x, cdr.y, D, O);

    float t_r12 = compute_parametric_variable(rob_pts[ctr.v2],
        rob_pts[ctr.v3], cdr.y, cdr.z, D, O);

    float t_o01 = compute_parametric_variable(base_obs_vertices[cto.v1],
        base_obs_vertices[cto.v2], cdo.x, cdo.y, D, O);

    float t_o12 = compute_parametric_variable(base_obs_vertices[cto.v2],
        base_obs_vertices[cto.v3], cdo.y, cdo.z, D, O);

    // There is no overlap
    if (min(t_r01, t_r12) > max(t_o01, t_o12)) {
        return true;

    // Also no overlap
    } else if (min(t_o01, t_o12) > max(t_r01, t_r12)) {
        return true;

    // There is overlap
    } else {
        req_coplanar = false;
        return false;
    }

    return valid;
}

__global__ void MegaKernel(const  Configuration *configs, const AABB *p_obsAABB, const AABB *obs_tri_AABBs,

                                     bool *valid_confs, const int _num_configs){

    //stage one variables
    ////////////////////////////////////////////////////////////////////////////////
    __shared__ Vector3f transformed_vertices[NUM_ROB_VERTICES];
    const int num_configs = _num_configs;
    size_t config_idx;
    __shared__ Vector3f smin[MEGA_BLOCK_SIZE];
    __shared__ Vector3f smax[MEGA_BLOCK_SIZE];
    Vector3f transformed_robot_vertex;
    Matrix3f rotation_matrix;
    Vector3f translation_vector;

    // local vectors for keeping track of AABBs
    Vector3f tmin;
    Vector3f tmax;

    Configuration conf;
    AABB obsAABB = *p_obsAABB;
    AABB robAABB;



    //stage two variables
    ////////////////////////////////////////////////////////////////////////////////
    __shared__ bool isTriangleValids[MAX_NUM_ROBOT_TRIANGLES];
    __shared__ AABB rob_tri_AABBs[MAX_NUM_ROBOT_TRIANGLES];
    AABB rob_tri_AABB;


    // stage three variables
    ////////////////////////////////////////////////////////////////////////////////

    // keeps track of which triangles to check for in the narrow phase
    // can replace these with indices if memory becomes an issue
    int curr_tri_index;
    __shared__ Triangle invalid_rob_tris[TRIANGLE_BUFFER_SIZE * 2]; // buffer for robot triangles whose AABBs intersect with obstacle triangle AABB
    __shared__ Triangle invalid_obs_tris[TRIANGLE_BUFFER_SIZE * 2]; // corresponding obstacle triangle buffer
    // __shared__ AABB obs_tri_AABBs[MAX_NUM_ROBOT_TRIANGLES];
    __shared__ int num_invalid_tris;
    __shared__ bool early_exit;
    __shared__ bool valid;

    int total_num_tris = 0;
    int conf_num_tris = 0;

    // load obs AABBs into shared memory
    // for (int i = threadIdx.x; i < MAX_NUM_ROBOT_TRIANGLES; i += blockDim.x){
    //     obs_tri_AABBs[i] = _obs_tri_AABBs[i];
    // }
    __syncthreads();
    for (int i = 0; i < NUM_CONFS_PER_BLOCK; i++){
        if(threadIdx.x == 0){
            // printf("Num triangle intersection performed in previous config: %d\n", conf_num_tris);
        }
        conf_num_tris = 0;
        config_idx = blockIdx.x * NUM_CONFS_PER_BLOCK + i;
        if (config_idx >= num_configs) break;
        conf = configs[config_idx];
        // stage one
        // transformation and big AABB checking
        rotation_matrix = createRotationMatrix(conf);
        translation_vector = Vector3f(conf.x, conf.y, conf.z);
        // Matrix4f transform_matrix = createTransformationMatrix(configs[config_idx]);
        valid = true;
        for(int vertex_idx = threadIdx.x ; vertex_idx < NUM_ROB_VERTICES; vertex_idx += MEGA_BLOCK_SIZE)
        {
            transformed_vertices[vertex_idx] = transformVector(base_robot_vertices[vertex_idx], rotation_matrix, translation_vector);
        }
        __syncthreads();
        // // local variables for AABB computation
        // Vector3f tmin = transformed_vertices[0];
        // Vector3f tmax = transformed_vertices[0];

        // if(threadIdx.x == 0){

        //     for(int j = 0; j < NUM_ROB_VERTICES; j++) {
        //         Vector3f v = transformed_vertices[j];
        //         tmin.x = fminf(tmin.x, v.x);
        //         tmin.y = fminf(tmin.y, v.y);
        //         tmin.z = fminf(tmin.z, v.z);
        //         tmax.x = fmaxf(tmax.x, v.x);
        //         tmax.y = fmaxf(tmax.y, v.y);
        //         tmax.z = fmaxf(tmax.z, v.z);
        //     }

        //     AABB robAABB = {tmin.x, tmin.y, tmin.z, tmax.x, tmax.y, tmax.z};
        //     if (overlaps(robAABB, obsAABB)){
        //         valid = false;
        //         valid_confs[config_idx] = false;
        //     }
        // }
        // __syncthreads();
        // if (!valid) continue;

        // local variables for AABB computation
        tmin = transformed_vertices[0];
        tmax = transformed_vertices[0];

        // compute AABB for each vertex using thread-local variables
        for(int j = threadIdx.x; j < NUM_ROB_VERTICES; j += MEGA_BLOCK_SIZE) {
            Vector3f v = transformed_vertices[j];
            tmin.x = fminf(tmin.x, v.x);
            tmin.y = fminf(tmin.y, v.y);
            tmin.z = fminf(tmin.z, v.z);
            tmax.x = fmaxf(tmax.x, v.x);
            tmax.y = fmaxf(tmax.y, v.y);
            tmax.z = fmaxf(tmax.z, v.z);
        }

        // perform reduction to compute final AABB
        int tid = threadIdx.x;
        smin[tid] = tmin;
        smax[tid] = tmax;
        __syncthreads();

        //TODO:  check if this AI generated is correct
        for(int s = MEGA_BLOCK_SIZE / 2; s > 0; s >>= 1) {
            if(tid < s) {
                smin[tid].x = fminf(smin[tid].x, smin[tid + s].x);
                smin[tid].y = fminf(smin[tid].y, smin[tid + s].y);
                smin[tid].z = fminf(smin[tid].z, smin[tid + s].z);
                smax[tid].x = fmaxf(smax[tid].x, smax[tid + s].x);
                smax[tid].y = fmaxf(smax[tid].y, smax[tid + s].y);
                smax[tid].z = fmaxf(smax[tid].z, smax[tid + s].z);
            }
            __syncthreads();
        }

        // TODO: optimize this, maybe make a function to just take in the raw values
        robAABB = {smin[0].x, smin[0].y, smin[0].z, smax[0].x, smax[0].y, smax[0].z};
        if (!overlaps(robAABB, obsAABB)){
            // valid_confs[config_idx] = true;
            continue;
        }

        // stage two
        // small robot AABB vs big obs AABB
        // generate an AABB for each robot triangle, and check if it
        // overlaps the big obs AABB

        // __syncthreads();
        // //TODO: experiment to see if its faster to load rob tri AABBs into shared memory
        // // during stage one, or to have a separate loop.
        for (int j = threadIdx.x; j < MAX_NUM_ROBOT_TRIANGLES; j+=MEGA_BLOCK_SIZE){
            // collaboratively generate AABB
            rob_tri_AABB = generateTriangleAABB(   transformed_vertices[base_robot_triangles[j].v1],
                                                        transformed_vertices[base_robot_triangles[j].v2],
                                                        transformed_vertices[base_robot_triangles[j].v3]);
            rob_tri_AABBs[j] = rob_tri_AABB;

            if (!overlaps(rob_tri_AABB, obsAABB)){
                isTriangleValids[j] = true;
            } else {
                valid = false;
                conf_num_tris++;
                total_num_tris++;
                isTriangleValids[j] = false;
            }
        }
        // if none of the smaller robot triangle AABB's overlapped the obstacle
        // continue to the next configuration
        __syncthreads();
        if (valid){
            // valid_confs[config_idx] = true;
            continue;
        }
        __syncthreads();
        // stage three
        // small robot AABB vs small obs AABB
        num_invalid_tris = 0;
        valid = true;
        __syncthreads();
        // if (threadIdx.x == 0 && config_idx == 86){
        //     // for (int j = 0; j < MAX_NUM_ROBOT_TRIANGLES; j++){
        //     //     printf("isTriangleValids[%d] = %d\n", j, isTriangleValids[j]);
        //     // }int
        //         int j = 751;
        //         int k = 976;
        //         printf("rob_tri_AABBs[%d] = (%f, %f, %f, %f, %f, %f)\n", j, rob_tri_AABBs[j].x_min, rob_tri_AABBs[j].y_min, rob_tri_AABBs[j].z_min, rob_tri_AABBs[j].x_max, rob_tri_AABBs[j].y_max, rob_tri_AABBs[j].z_max);
        //         printf("obs_tri_AABBs[%d] = (%f, %f, %f, %f, %f, %f)\n", k, obs_tri_AABBs[k].x_min, obs_tri_AABBs[k].y_min, obs_tri_AABBs[k].z_min, obs_tri_AABBs[k].x_max, obs_tri_AABBs[k].y_max, obs_tri_AABBs[k].z_max);
        //     // }
        // }
        for (int j = 0; j < MAX_NUM_ROBOT_TRIANGLES; j++){
            if (isTriangleValids[j]){
                continue;
            }
            if (!valid){
                break;
            }
                                         // round up to the next multiple of block dim
            for (int k = threadIdx.x; k < (MAX_NUM_ROBOT_TRIANGLES + MEGA_BLOCK_SIZE) ; k+= MEGA_BLOCK_SIZE){
            // for (int k = blockIdx.x; k < (MAX_NUM_ROBOT_TRIANGLES +(MEGA_BLOCK_SIZE - (MAX_NUM_ROBOT_TRIANGLES % MEGA_BLOCK_SIZE))); k+= MEGA_BLOCK_SIZE){
                // __syncthreads();
                if (!valid){
                    break;
                }


                // if (k < MAX_NUM_ROBOT_TRIANGLES){
                //     // TODO: copy obs_tri into shared memory
                //     if (overlaps(rob_tri_AABBs[j], obs_tri_AABBs[k])){
                //         valid = false;
                //     }
                // }

                // if we haven't overflowed the shared memory
                if(num_invalid_tris < TRIANGLE_BUFFER_SIZE){

                    // __syncthreads();
                    if (k < MAX_NUM_ROBOT_TRIANGLES){
                        if (overlaps(rob_tri_AABBs[j], obs_tri_AABBs[k])){
                            // valid = false;
                            curr_tri_index = atomicAdd(&num_invalid_tris, 1);
                            invalid_rob_tris[curr_tri_index] = base_robot_triangles[j];
                            invalid_obs_tris[curr_tri_index] = base_obs_triangles[k];
                        }
                    }
                // if we have overflowed shared memory
                } else {
                    // __syncthreads();
                    k--;
                    // do triangle triangle collision checking on all pairs found so far
                    // this flushes the buffer
                    for (int l = threadIdx.x; l < num_invalid_tris; l+=MEGA_BLOCK_SIZE){
                        // __syncthreads();
                        if (!valid){
                            break;
                        }
                        if (!triangles_valid(invalid_rob_tris[l], invalid_obs_tris[l],
                                                            transformed_vertices)){
                            valid = false;
                            valid_confs[config_idx] = false;
                        }

                    }
                    // reset the number of invalid triangles
                    __syncthreads();
                    num_invalid_tris = 0;
                    __syncthreads();
                }
            }
        }

        // final triangle triangle intersection test
        __syncthreads();
        for (int l = threadIdx.x; l < num_invalid_tris; l+=MEGA_BLOCK_SIZE){
            // __syncthreads();
            if (!valid){
                break;
            }
            if (!triangles_valid(invalid_rob_tris[l], invalid_obs_tris[l],
                                                    transformed_vertices)){
                valid = false;
                valid_confs[config_idx] = false;
            }
        }
    }
    // printf("total number of triangle intersections for block was %d\n", total_num_tris);
}

void CallMegaKernel(std::vector<Configuration> configs, bool *valid_confs){
    int device_count;
    if (cudaGetDeviceCount(&device_count) != 0) std::cout << "CUDA not loaded properly" << std::endl;

    //Load Robot
    std::vector<Vector3f> rob_vertices;
    std::vector<Triangle> rob_triangles;
    loadOBJFile(ROB_FILE, rob_vertices, rob_triangles);
    std::cout << "Robot has " << rob_vertices.size() << " vertices " <<std::endl;
    std::cout << "Robot has " << rob_triangles.size() << " triangles " <<std::endl;

    //Load Obstacles
    std::vector<Vector3f> obs_vertices;
    std::vector<Triangle> obs_triangles;
    loadOBJFile(OBS_FILE, obs_vertices, obs_triangles);
    std::cout << "Obstacle has " << obs_vertices.size() << " vertices " <<std::endl;
    std::cout << "Obstacle has " << obs_triangles.size() << " triangles " <<std::endl;

    // checkCudaCall(cudaMalloc(&d_rob_transformed_points, rob_vertices.size() * configs.size() * sizeof(Vector3f)));
    // checkCudaCall(cudaMalloc(&d_rob_triangles, rob_triangles.size() * sizeof(Triangle)));
    // checkCudaMem(cudaMemcpy(d_rob_points, rob_vertices.data(), rob_vertices.size() * sizeof(Vector3f), cudaMemcpyHostToDevice));
    // checkCudaMem(cudaMemcpy(d_rob_triangles, rob_triangles.data(), rob_triangles.size() * sizeof(Triangle), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpyToSymbol(base_robot_vertices, rob_vertices.data(), rob_vertices.size() * sizeof(Vector3f)));
    checkCudaMem(cudaMemcpyToSymbol(base_robot_triangles, rob_triangles.data(), rob_triangles.size() * sizeof(Triangle)));
    std::cout << "Copied the robot vertices and triangles " << std::endl;

    checkCudaMem(cudaMemcpyToSymbol(base_obs_vertices, obs_vertices.data(), obs_vertices.size() * sizeof(Vector3f)));
    checkCudaMem(cudaMemcpyToSymbol(base_obs_triangles, obs_triangles.data(), obs_triangles.size() * sizeof(Triangle)));
    // Vector3f *d_obs_points;
    // Triangle *d_obs_triangles;

    // checkCudaCall(cudaMalloc(&d_obs_points, obs_vertices.size() * sizeof(Vector3f)));
    // checkCudaCall(cudaMalloc(&d_obs_triangles, obs_triangles.size() * sizeof(Triangle)));
    // checkCudaMem(cudaMemcpy(d_obs_points, obs_vertices.data(), obs_vertices.size() * sizeof(Vector3f), cudaMemcpyHostToDevice));
    // checkCudaMem(cudaMemcpy(d_obs_triangles, obs_triangles.data(), obs_triangles.size() * sizeof(Triangle), cudaMemcpyHostToDevice));
    std::cout << "Copied the obstacle vertices and triangles " << std::endl;


    Configuration *d_configs;
    checkCudaCall(cudaMalloc(&d_configs, configs.size() * sizeof(Configuration)));
    checkCudaMem(cudaMemcpy(d_configs, configs.data(), configs.size() * sizeof(Configuration), cudaMemcpyHostToDevice));
    std::cout << "Copied the configurations " << std::endl;


    bool *valid_conf_d;
    AABB *obstacle_AABB_d;
    AABB *obstacle_tris_AABB_d;

    checkCudaCall(cudaMalloc(&valid_conf_d, configs.size() * sizeof(bool)));
    checkCudaCall(cudaMalloc(&obstacle_AABB_d, sizeof(AABB)));
    checkCudaCall(cudaMalloc(&obstacle_tris_AABB_d, sizeof(AABB) * obs_triangles.size()));

    // Move obstacle to AABB (on CPU since we only have 1)
    AABB *obstacle_AABB = new AABB();
    std::vector<AABB> obstacle_tris_AABB;
    obstacle_tris_AABB.reserve(obs_triangles.size());

    generateAABBBaseline(obs_vertices.data(), obs_vertices.size(), 1, obstacle_AABB);
    generateTriAABBs(obs_triangles, obs_vertices, obstacle_tris_AABB);
    std::cout << "Generated " << obstacle_tris_AABB.size() << " obstacle AABBs " << std::endl;
    checkCudaCall(cudaMemcpy(obstacle_AABB_d, obstacle_AABB, sizeof(AABB), cudaMemcpyHostToDevice));
    checkCudaCall(cudaMemcpy(obstacle_tris_AABB_d, obstacle_tris_AABB.data(), sizeof(AABB) * obs_triangles.size(), cudaMemcpyHostToDevice));
    bool *arr_of_true = new bool[configs.size()];
    for (int i = 0; i < configs.size(); i++){
        arr_of_true[i] = true;
    }
    checkCudaCall(cudaMemcpy(valid_conf_d, arr_of_true, configs.size() * sizeof(bool), cudaMemcpyHostToDevice));

    // checkCudaCall(cudaDeviceSynchronize());
    std::cout << "About to call mega kernel" << std::endl;
    MegaKernel<<<(configs.size() - 1) / (NUM_CONFS_PER_BLOCK) + 1, MEGA_BLOCK_SIZE>>>(d_configs, obstacle_AABB_d, obstacle_tris_AABB_d, valid_conf_d, configs.size());

    // checkCudaCall(cudaDeviceSynchronize());
    checkCudaMem(cudaMemcpy(valid_confs, valid_conf_d, configs.size() * sizeof(bool), cudaMemcpyDeviceToHost));

    checkCudaCall(cudaFree(d_configs));
    checkCudaCall(cudaFree(obstacle_AABB_d));
    checkCudaCall(cudaFree(obstacle_tris_AABB_d));
    checkCudaCall(cudaFree(valid_conf_d));
    std::cout << "Copied back memory and synchronized" << std::endl;
}