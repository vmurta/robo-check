#include "MegaKernel.hu"
#include "narrow-phase/Triangle.hu"
#include <memory>

#define NUM_CONFS_PER_BLOCK 32
#define MEGA_BLOCK_SIZE 32
#define TRIANGLE_BUFFER_SIZE 128


inline __device__ bool overlaps(const AABB &a1, const AABB &a2){
    return (a1.x_min <= a2.x_max && a1.x_max >= a2.x_min) &&
           (a1.y_min <= a2.y_max && a1.y_max >= a2.y_min) &&
           (a1.z_min <= a2.z_max && a1.z_max >= a2.z_min);
}

// These are deliberately marked inline so that MegaKernel.o remains self-contained
// (Full-Integration-Test links against it without Triangle.o) while still coexisting
// with the non-inline definitions in Triangle.o when both are linked together (BVH).
inline __host__ __device__ AABB generateTriangleAABB(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2, const Eigen::Vector3f &p3){
    AABB aabb;
    aabb.x_min = min(p1.x(), min(p2.x(), p3.x()));
    aabb.y_min = min(p1.y(), min(p2.y(), p3.y()));
    aabb.z_min = min(p1.z(), min(p2.z(), p3.z()));
    aabb.x_max = max(p1.x(), max(p2.x(), p3.x()));
    aabb.y_max = max(p1.y(), max(p2.y(), p3.y()));
    aabb.z_max = max(p1.z(), max(p2.z(), p3.z()));
    return aabb;
}

inline void generateTriAABBs(const std::vector<Triangle> &triangles, const std::vector<Eigen::Vector3f> &points, std::vector<AABB> &aabbs){
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
                                     const Eigen::Vector3f *d_robot_vertices, const Triangle *d_robot_triangles,
                                     const Eigen::Vector3f *d_obs_vertices, const Triangle *d_obs_triangles){

    //stage one variables
    ////////////////////////////////////////////////////////////////////////////////
    __shared__ Eigen::Vector3f transformed_vertices[792];
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
    __shared__ bool isTriangleValids[1008];
    __shared__ AABB rob_tri_AABBs[1008];
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
        for(int vertex_idx = threadIdx.x ; vertex_idx < 792; vertex_idx += MEGA_BLOCK_SIZE)
        {
            transformed_vertices[vertex_idx] = transformVector(d_robot_vertices[vertex_idx], rotation_matrix, translation_vector);
        }
        __syncthreads();

        // compute AABB using parallel reduction
        tmin = transformed_vertices[0];
        tmax = transformed_vertices[0];

        for(int j = threadIdx.x; j < 792; j += MEGA_BLOCK_SIZE) {
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
        for (int j = threadIdx.x; j < 1008; j+=MEGA_BLOCK_SIZE){
            rob_tri_AABB = generateTriangleAABB(   transformed_vertices[d_robot_triangles[j].v1],
                                                        transformed_vertices[d_robot_triangles[j].v2],
                                                        transformed_vertices[d_robot_triangles[j].v3]);
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
        for (int j = 0; j < 1008; j++){
            if (isTriangleValids[j]){
                continue;
            }
            if (!valid){
                break;
            }
            for (int k = threadIdx.x; k < (1008 + MEGA_BLOCK_SIZE) ; k+= MEGA_BLOCK_SIZE){
                if (!valid){
                    break;
                }

                if(num_invalid_tris < TRIANGLE_BUFFER_SIZE){
                    if (k < 1008){
                        if (overlaps(rob_tri_AABBs[j], obs_tri_AABBs[k])){
                            curr_tri_index = atomicAdd(&num_invalid_tris, 1);
                            invalid_rob_tris[curr_tri_index] = d_robot_triangles[j];
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

double mega_naive(const BVNode_soa& rob_BVH, const BVNode_soa& obs_BVH,
                  const MeshData& rob_mesh, const MeshData& obs_mesh,
                  const std::vector<Configuration>& confs,
                  std::vector<bool>& valid, bool dry_run) {
    cudaEvent_t start, stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);

    const int num_confs = confs.size();
    valid.assign(num_confs, false);
    if (num_confs == 0) {
        return 0.0;
    }

    std::unique_ptr<bool[]> raw_valid(new bool[num_confs]);
    for (int i = 0; i < num_confs; ++i) {
        raw_valid.get()[i] = true;
    }

    std::cout << "Rob mesh has " << rob_mesh.vertices.size() << " vertices, "
              << rob_mesh.triangles.size() << " triangles" << std::endl;
    std::cout << "Obs mesh has " << obs_mesh.vertices.size() << " vertices, "
              << obs_mesh.triangles.size() << " triangles" << std::endl;

    cudaEventRecord(start, 0);

    Eigen::Vector3f *d_obs_vertices;
    Triangle *d_obs_triangles;
    Eigen::Vector3f *d_robot_vertices;
    Triangle *d_robot_triangles;

    checkCudaCall(cudaMalloc(&d_robot_vertices, rob_mesh.vertices.size() * sizeof(Eigen::Vector3f)));
    checkCudaMem(cudaMemcpy(d_robot_vertices, rob_mesh.vertices.data(), rob_mesh.vertices.size() * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaCall(cudaMalloc(&d_robot_triangles, rob_mesh.triangles.size() * sizeof(Triangle)));
    checkCudaMem(cudaMemcpy(d_robot_triangles, rob_mesh.triangles.data(), rob_mesh.triangles.size() * sizeof(Triangle), cudaMemcpyHostToDevice));

    checkCudaCall(cudaMalloc(&d_obs_vertices, obs_mesh.vertices.size() * sizeof(Eigen::Vector3f)));
    checkCudaMem(cudaMemcpy(d_obs_vertices, obs_mesh.vertices.data(), obs_mesh.vertices.size() * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaCall(cudaMalloc(&d_obs_triangles, obs_mesh.triangles.size() * sizeof(Triangle)));
    checkCudaMem(cudaMemcpy(d_obs_triangles, obs_mesh.triangles.data(), obs_mesh.triangles.size() * sizeof(Triangle), cudaMemcpyHostToDevice));

    Configuration *d_configs;
    checkCudaCall(cudaMalloc(&d_configs, num_confs * sizeof(Configuration)));
    checkCudaMem(cudaMemcpy(d_configs, confs.data(), num_confs * sizeof(Configuration), cudaMemcpyHostToDevice));

    bool *valid_conf_d;
    AABB *obstacle_AABB_d;
    AABB *obstacle_tris_AABB_d;

    checkCudaCall(cudaMalloc(&valid_conf_d, num_confs * sizeof(bool)));
    checkCudaCall(cudaMalloc(&obstacle_AABB_d, sizeof(AABB)));
    checkCudaCall(cudaMalloc(&obstacle_tris_AABB_d, sizeof(AABB) * obs_mesh.triangles.size()));

    AABB *obstacle_AABB = new AABB();
    std::vector<AABB> obstacle_tris_AABB;
    obstacle_tris_AABB.reserve(obs_mesh.triangles.size());

    generateAABBBaseline(const_cast<Eigen::Vector3f*>(obs_mesh.vertices.data()), obs_mesh.vertices.size(), 1, obstacle_AABB);
    generateTriAABBs(obs_mesh.triangles, obs_mesh.vertices, obstacle_tris_AABB);
    std::cout << "Generated " << obstacle_tris_AABB.size() << " obstacle AABBs" << std::endl;

    checkCudaCall(cudaMemcpy(obstacle_AABB_d, obstacle_AABB, sizeof(AABB), cudaMemcpyHostToDevice));
    checkCudaCall(cudaMemcpy(obstacle_tris_AABB_d, obstacle_tris_AABB.data(), sizeof(AABB) * obs_mesh.triangles.size(), cudaMemcpyHostToDevice));
    checkCudaCall(cudaMemcpy(valid_conf_d, raw_valid.get(), num_confs * sizeof(bool), cudaMemcpyHostToDevice));

    delete obstacle_AABB;

    cudaDeviceSynchronize();
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    float duration = 0;
    cudaEventElapsedTime(&duration, start, stop);
    std::cout << "MegaKernel allocation and transfer to GPU took " << duration << " ms." << std::endl;

    const int gridSize = (num_confs - 1) / NUM_CONFS_PER_BLOCK + 1;

    auto launch_mega = [&]() {
        MegaKernel<<<gridSize, MEGA_BLOCK_SIZE>>>(
            d_configs, obstacle_AABB_d, obstacle_tris_AABB_d, valid_conf_d, num_confs,
            d_robot_vertices, d_robot_triangles, d_obs_vertices, d_obs_triangles);
    };

    if (dry_run) {
        MegaKernel<<<1, MEGA_BLOCK_SIZE>>>(
            d_configs, obstacle_AABB_d, obstacle_tris_AABB_d, valid_conf_d, num_confs,
            d_robot_vertices, d_robot_triangles, d_obs_vertices, d_obs_triangles);
        checkCudaMem(cudaGetLastError());
        checkCudaMem(cudaDeviceSynchronize());
        std::cout << "MegaKernel dry run completed successfully." << std::endl;
    }

    cudaEventRecord(start, 0);
    launch_mega();
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&duration, start, stop);
    std::cout << "MegaKernel GPU kernel took " << duration << " ms for " << num_confs << " configurations." << std::endl;

    cudaEventRecord(start, 0);
    checkCudaMem(cudaGetLastError());
    checkCudaMem(cudaMemcpy(raw_valid.get(), valid_conf_d, num_confs * sizeof(bool), cudaMemcpyDeviceToHost));
    cudaDeviceSynchronize();
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&duration, start, stop);
    std::cout << "MegaKernel copying results from GPU took " << duration << " ms." << std::endl;

    for (int i = 0; i < num_confs; ++i) {
        valid[i] = raw_valid.get()[i];
    }

    checkCudaCall(cudaFree(d_configs));
    checkCudaCall(cudaFree(obstacle_AABB_d));
    checkCudaCall(cudaFree(obstacle_tris_AABB_d));
    checkCudaCall(cudaFree(valid_conf_d));
    checkCudaCall(cudaFree(d_robot_vertices));
    checkCudaCall(cudaFree(d_robot_triangles));
    checkCudaCall(cudaFree(d_obs_vertices));
    checkCudaCall(cudaFree(d_obs_triangles));

    return duration;
}

void CallMegaKernel(std::vector<Configuration> configs, bool *valid_confs, const char *rob_file, const char *obs_file){
    std::vector<Eigen::Vector3f> rob_vertices;
    std::vector<Triangle> rob_triangles;
    loadOBJFile(rob_file, rob_vertices, rob_triangles);

    std::vector<Eigen::Vector3f> obs_vertices;
    std::vector<Triangle> obs_triangles;
    loadOBJFile(obs_file, obs_vertices, obs_triangles);

    MeshData rob_mesh;
    rob_mesh.vertices = rob_vertices;
    rob_mesh.triangles = rob_triangles;

    MeshData obs_mesh;
    obs_mesh.vertices = obs_vertices;
    obs_mesh.triangles = obs_triangles;

    BVNode_soa empty_rob(0);
    BVNode_soa empty_obs(0);

    for (int i = 0; i < configs.size(); i++) {
        valid_confs[i] = true;
    }

    std::vector<bool> valid;
    mega_naive(empty_rob, empty_obs, rob_mesh, obs_mesh, configs, valid);

    for (int i = 0; i < configs.size(); i++) {
        valid_confs[i] = valid[i];
    }
}
