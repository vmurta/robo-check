#include "MegaKernel.hu"
#include "narrow-phase/Triangle.hu"

#define NUM_CONFS_PER_BLOCK 32
#define MEGA_BLOCK_SIZE 32
#define TRIANGLE_BUFFER_SIZE 128

#ifdef MEGA_CONSTANT
// POD layout identical to Eigen::Vector3f so the mesh can be stored in constant memory
// (Eigen::Vector3f itself has a non-trivial constructor, which CUDA rejects for __constant__)
struct ConstantPoint {
    float x;
    float y;
    float z;
};

__constant__ ConstantPoint mega_robot_vertices[NUM_ROB_VERTICES];
__constant__ Triangle mega_robot_triangles[MAX_NUM_ROBOT_TRIANGLES];
#define ROBOT_VERTICES mega_robot_vertices
#define ROBOT_TRIANGLES mega_robot_triangles
#else
#define ROBOT_VERTICES d_robot_vertices
#define ROBOT_TRIANGLES d_robot_triangles
#endif


inline __device__ bool overlaps(const AABB &a1, const AABB &a2){
    return (a1.x_min <= a2.x_max && a1.x_max >= a2.x_min) &&
           (a1.y_min <= a2.y_max && a1.y_max >= a2.y_min) &&
           (a1.z_min <= a2.z_max && a1.z_max >= a2.z_min);
}

__host__ __device__ AABB generateTriangleAABB(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2, const Eigen::Vector3f &p3){
    AABB aabb;
    aabb.x_min = min(p1.x(), min(p2.x(), p3.x()));
    aabb.y_min = min(p1.y(), min(p2.y(), p3.y()));
    aabb.z_min = min(p1.z(), min(p2.z(), p3.z()));
    aabb.x_max = max(p1.x(), max(p2.x(), p3.x()));
    aabb.y_max = max(p1.y(), max(p2.y(), p3.y()));
    aabb.z_max = max(p1.z(), max(p2.z(), p3.z()));
    return aabb;
}

void generateTriAABBs(const std::vector<Triangle> &triangles, const std::vector<Eigen::Vector3f> &points, std::vector<AABB> &aabbs){
    for (int i = 0; i < triangles.size(); i++){
        aabbs.push_back(generateTriangleAABB(points[triangles[i].v1], points[triangles[i].v2], points[triangles[i].v3]));
    }
}

__device__ bool triangles_valid(const Triangle &rob_tri, const Triangle &obs_tri,
                                    const Eigen::Vector3f *rob_pts,
                                    const Eigen::Vector3f *obs_vertices) {
    Eigen::Vector3f Nr;
    float dr;
    bool valid = true;
    bool req_coplanar = false;

    compute_plane(rob_tri, rob_pts, &Nr, &dr);

    Eigen::Vector3f obs_v1 = obs_vertices[obs_tri.v1];
    Eigen::Vector3f obs_v2 = obs_vertices[obs_tri.v2];
    Eigen::Vector3f obs_v3 = obs_vertices[obs_tri.v3];

    Eigen::Vector3f distO = compute_signed_dists(Nr, dr, obs_v1, obs_v2, obs_v3);

    if (no_overlap(distO)) {
        return true;
    }

    Eigen::Vector3f No;
    float do_;
    compute_plane(obs_v1, obs_v2, obs_v3, &No, &do_);

    if (is_coplanar(Nr, dr, No, do_)) {
        req_coplanar = true;
        return true;
    }

    Eigen::Vector3f distR = compute_signed_dists(No, do_, rob_tri, rob_pts);
    if (no_overlap(distR)) {
        return true;
    }

    Eigen::Vector3f D, O;
    compute_intersect_line(Nr, dr, No, do_, &D, &O);

    Triangle ctr, cto;
    Eigen::Vector3f cdr, cdo;
    canonicalize_triangle(rob_tri, distR, &ctr, &cdr);
    canonicalize_triangle(obs_tri, distO, &cto, &cdo);

    float t_r01 = compute_parametric_variable(rob_pts[ctr.v1],
        rob_pts[ctr.v2], cdr.x(), cdr.y(), D, O);

    float t_r12 = compute_parametric_variable(rob_pts[ctr.v2],
        rob_pts[ctr.v3], cdr.y(), cdr.z(), D, O);

    float t_o01 = compute_parametric_variable(obs_vertices[cto.v1],
        obs_vertices[cto.v2], cdo.x(), cdo.y(), D, O);

    float t_o12 = compute_parametric_variable(obs_vertices[cto.v2],
        obs_vertices[cto.v3], cdo.y(), cdo.z(), D, O);

    if (min(t_r01, t_r12) > max(t_o01, t_o12)) {
        return true;
    } else if (min(t_o01, t_o12) > max(t_r01, t_r12)) {
        return true;
    } else {
        req_coplanar = false;
        return false;
    }

    return valid;
}

__global__ void MegaKernel(const Configuration *configs, const AABB *p_obsAABB, const AABB *obs_tri_AABBs,
                                     bool *valid_confs, const int _num_configs,
#ifdef MEGA_CONSTANT
                                     const Eigen::Vector3f *d_obs_vertices, const Triangle *d_obs_triangles){
#else
                                     const Eigen::Vector3f *d_robot_vertices, const Triangle *d_robot_triangles,
                                     const Eigen::Vector3f *d_obs_vertices, const Triangle *d_obs_triangles){
#endif

    //stage one variables
    ////////////////////////////////////////////////////////////////////////////////
    __shared__ Eigen::Vector3f transformed_vertices[NUM_ROB_VERTICES];
    const int num_configs = _num_configs;
    size_t config_idx;
    __shared__ Eigen::Vector3f smin[MEGA_BLOCK_SIZE];
    __shared__ Eigen::Vector3f smax[MEGA_BLOCK_SIZE];
    Eigen::Vector3f transformed_robot_vertex;
    Eigen::Matrix3f rotation_matrix;
    Eigen::Vector3f translation_vector;

    // local vectors for keeping track of AABBs
    Eigen::Vector3f tmin;
    Eigen::Vector3f tmax;

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

    int curr_tri_index;
    __shared__ Triangle invalid_rob_tris[TRIANGLE_BUFFER_SIZE * 2];
    __shared__ Triangle invalid_obs_tris[TRIANGLE_BUFFER_SIZE * 2];
    __shared__ int num_invalid_tris;
    __shared__ bool early_exit;
    __shared__ bool valid;

    int total_num_tris = 0;
    int conf_num_tris = 0;

    __syncthreads();
    for (int i = 0; i < NUM_CONFS_PER_BLOCK; i++){
        conf_num_tris = 0;
        config_idx = blockIdx.x * NUM_CONFS_PER_BLOCK + i;
        if (config_idx >= num_configs) break;
        conf = configs[config_idx];
        // stage one
        rotation_matrix = createRotationMatrix(conf);
        translation_vector = Eigen::Vector3f(conf.x, conf.y, conf.z);
        valid = true;
        for(int vertex_idx = threadIdx.x ; vertex_idx < NUM_ROB_VERTICES; vertex_idx += MEGA_BLOCK_SIZE)
        {
#ifdef MEGA_CONSTANT
            Eigen::Vector3f robot_vertex(ROBOT_VERTICES[vertex_idx].x, ROBOT_VERTICES[vertex_idx].y, ROBOT_VERTICES[vertex_idx].z);
            transformed_vertices[vertex_idx] = transformVector(robot_vertex, rotation_matrix, translation_vector);
#else
            transformed_vertices[vertex_idx] = transformVector(ROBOT_VERTICES[vertex_idx], rotation_matrix, translation_vector);
#endif
        }
        __syncthreads();

        // compute AABB using parallel reduction
        tmin = transformed_vertices[0];
        tmax = transformed_vertices[0];

        for(int j = threadIdx.x; j < NUM_ROB_VERTICES; j += MEGA_BLOCK_SIZE) {
            Eigen::Vector3f v = transformed_vertices[j];
            tmin.x() = fminf(tmin.x(), v.x());
            tmin.y() = fminf(tmin.y(), v.y());
            tmin.z() = fminf(tmin.z(), v.z());
            tmax.x() = fmaxf(tmax.x(), v.x());
            tmax.y() = fmaxf(tmax.y(), v.y());
            tmax.z() = fmaxf(tmax.z(), v.z());
        }

        int tid = threadIdx.x;
        smin[tid] = tmin;
        smax[tid] = tmax;
        __syncthreads();

        for(int s = MEGA_BLOCK_SIZE / 2; s > 0; s >>= 1) {
            if(tid < s) {
                smin[tid].x() = fminf(smin[tid].x(), smin[tid + s].x());
                smin[tid].y() = fminf(smin[tid].y(), smin[tid + s].y());
                smin[tid].z() = fminf(smin[tid].z(), smin[tid + s].z());
                smax[tid].x() = fmaxf(smax[tid].x(), smax[tid + s].x());
                smax[tid].y() = fmaxf(smax[tid].y(), smax[tid + s].y());
                smax[tid].z() = fmaxf(smax[tid].z(), smax[tid + s].z());
            }
            __syncthreads();
        }

        robAABB = {smin[0].x(), smin[0].y(), smin[0].z(), smax[0].x(), smax[0].y(), smax[0].z()};
        if (!overlaps(robAABB, obsAABB)){
            continue;
        }

        // stage two
        for (int j = threadIdx.x; j < MAX_NUM_ROBOT_TRIANGLES; j+=MEGA_BLOCK_SIZE){
            rob_tri_AABB = generateTriangleAABB(   transformed_vertices[ROBOT_TRIANGLES[j].v1],
                                                        transformed_vertices[ROBOT_TRIANGLES[j].v2],
                                                        transformed_vertices[ROBOT_TRIANGLES[j].v3]);
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
        __syncthreads();
        if (valid){
            continue;
        }
        __syncthreads();
        // stage three
        num_invalid_tris = 0;
        valid = true;
        __syncthreads();
        for (int j = 0; j < MAX_NUM_ROBOT_TRIANGLES; j++){
            if (isTriangleValids[j]){
                continue;
            }
            if (!valid){
                break;
            }
            for (int k = threadIdx.x; k < (MAX_NUM_ROBOT_TRIANGLES + MEGA_BLOCK_SIZE) ; k+= MEGA_BLOCK_SIZE){
                if (!valid){
                    break;
                }

                if(num_invalid_tris < TRIANGLE_BUFFER_SIZE){
                    if (k < MAX_NUM_ROBOT_TRIANGLES){
                        if (overlaps(rob_tri_AABBs[j], obs_tri_AABBs[k])){
                            curr_tri_index = atomicAdd(&num_invalid_tris, 1);
                            invalid_rob_tris[curr_tri_index] = ROBOT_TRIANGLES[j];
                            invalid_obs_tris[curr_tri_index] = d_obs_triangles[k];
                        }
                    }
                } else {
                    k--;
                    int num_to_do = (num_invalid_tris / MEGA_BLOCK_SIZE) * MEGA_BLOCK_SIZE;
                    for (int l = threadIdx.x; l < num_to_do; l+=MEGA_BLOCK_SIZE){
                        if (!valid){
                            break;
                        }
                        if (!triangles_valid(invalid_rob_tris[l], invalid_obs_tris[l],
                                                            transformed_vertices, d_obs_vertices)){
                            valid = false;
                            valid_confs[config_idx] = false;
                        }
                    }
                    __syncthreads();
                    if (threadIdx.x == 0){
                        num_invalid_tris = num_invalid_tris - num_to_do;
                    }
                    __syncthreads();
                }
            }
        }

        __syncthreads();
        for (int l = threadIdx.x; l < num_invalid_tris; l+=MEGA_BLOCK_SIZE){
            if (!valid){
                break;
            }
            if (!triangles_valid(invalid_rob_tris[l], invalid_obs_tris[l],
                                                    transformed_vertices, d_obs_vertices)){
                valid = false;
                valid_confs[config_idx] = false;
            }
        }
    }
}

void CallMegaKernel(std::vector<Configuration> configs, bool *valid_confs, const char *rob_file, const char *obs_file){
    int device_count;
    if (cudaGetDeviceCount(&device_count) != 0) std::cout << "CUDA not loaded properly" << std::endl;

    std::vector<Eigen::Vector3f> rob_vertices;
    std::vector<Triangle> rob_triangles;
    loadOBJFile(rob_file, rob_vertices, rob_triangles);
    std::cout << "Robot has " << rob_vertices.size() << " vertices " <<std::endl;
    std::cout << "Robot has " << rob_triangles.size() << " triangles " <<std::endl;

    std::vector<Eigen::Vector3f> obs_vertices;
    std::vector<Triangle> obs_triangles;
    loadOBJFile(obs_file, obs_vertices, obs_triangles);
    std::cout << "Obstacle has " << obs_vertices.size() << " vertices " <<std::endl;
    std::cout << "Obstacle has " << obs_triangles.size() << " triangles " <<std::endl;

    Eigen::Vector3f *d_obs_vertices;
    Triangle *d_obs_triangles;

#ifdef MEGA_CONSTANT
    checkCudaMem(cudaMemcpyToSymbol(mega_robot_vertices, rob_vertices.data(), rob_vertices.size() * sizeof(ConstantPoint)));
    checkCudaMem(cudaMemcpyToSymbol(mega_robot_triangles, rob_triangles.data(), rob_triangles.size() * sizeof(Triangle)));
    std::cout << "Copied the robot vertices and triangles to constant memory" << std::endl;
#else
    Eigen::Vector3f *d_robot_vertices;
    Triangle *d_robot_triangles;

    checkCudaCall(cudaMalloc(&d_robot_vertices, rob_vertices.size() * sizeof(Eigen::Vector3f)));
    checkCudaMem(cudaMemcpy(d_robot_vertices, rob_vertices.data(), rob_vertices.size() * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaCall(cudaMalloc(&d_robot_triangles, rob_triangles.size() * sizeof(Triangle)));
    checkCudaMem(cudaMemcpy(d_robot_triangles, rob_triangles.data(), rob_triangles.size() * sizeof(Triangle), cudaMemcpyHostToDevice));
    std::cout << "Copied the robot vertices and triangles " << std::endl;
#endif

    checkCudaCall(cudaMalloc(&d_obs_vertices, obs_vertices.size() * sizeof(Eigen::Vector3f)));
    checkCudaMem(cudaMemcpy(d_obs_vertices, obs_vertices.data(), obs_vertices.size() * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaCall(cudaMalloc(&d_obs_triangles, obs_triangles.size() * sizeof(Triangle)));
    checkCudaMem(cudaMemcpy(d_obs_triangles, obs_triangles.data(), obs_triangles.size() * sizeof(Triangle), cudaMemcpyHostToDevice));
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

    std::cout << "About to call mega kernel" << std::endl;
#ifdef MEGA_CONSTANT
    MegaKernel<<<(configs.size() - 1) / (NUM_CONFS_PER_BLOCK) + 1, MEGA_BLOCK_SIZE>>>(
        d_configs, obstacle_AABB_d, obstacle_tris_AABB_d, valid_conf_d, configs.size(),
        d_obs_vertices, d_obs_triangles);
#else
    MegaKernel<<<(configs.size() - 1) / (NUM_CONFS_PER_BLOCK) + 1, MEGA_BLOCK_SIZE>>>(
        d_configs, obstacle_AABB_d, obstacle_tris_AABB_d, valid_conf_d, configs.size(),
        d_robot_vertices, d_robot_triangles, d_obs_vertices, d_obs_triangles);
#endif

    checkCudaMem(cudaMemcpy(valid_confs, valid_conf_d, configs.size() * sizeof(bool), cudaMemcpyDeviceToHost));

    checkCudaCall(cudaFree(d_configs));
    checkCudaCall(cudaFree(obstacle_AABB_d));
    checkCudaCall(cudaFree(obstacle_tris_AABB_d));
    checkCudaCall(cudaFree(valid_conf_d));
#ifdef MEGA_CONSTANT
    checkCudaCall(cudaFree(d_obs_vertices));
    checkCudaCall(cudaFree(d_obs_triangles));
#else
    checkCudaCall(cudaFree(d_robot_vertices));
    checkCudaCall(cudaFree(d_robot_triangles));
    checkCudaCall(cudaFree(d_obs_vertices));
    checkCudaCall(cudaFree(d_obs_triangles));
#endif
    std::cout << "Copied back memory and synchronized" << std::endl;
}
