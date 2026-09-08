#include <fstream>
#include <sstream>
#include <iostream>

template <size_t N>
void readArticulatedConfigurationFromFile(const std::string& filename, std::vector<articulated_conf<N>>& confs) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        articulated_conf<N> conf;
        bool ok = true;
        for (size_t i = 0; i < N; ++i) {
            if (!(iss >> conf.joints[i])) {
                ok = false;
                break;
            }
        }
        if (ok) {
            confs.push_back(conf);
        }
    }
}

template <size_t N>
__host__ __device__ void forwardKinematics(const articulated_conf<N>& conf,
                                           const JointParams* joints,
                                           Eigen::Matrix3f* link_R,
                                           Eigen::Vector3f* link_T) {
    link_R[0] = Eigen::Matrix3f::Identity();
    link_T[0] = Eigen::Vector3f::Zero();

    for (size_t i = 0; i < N; ++i) {
        Eigen::Matrix3f Rj = axisAngleToRotation(joints[i].axis, conf[i]);
        link_R[i + 1] = link_R[i] * joints[i].origin_R * Rj;
        link_T[i + 1] = link_T[i] + link_R[i] * joints[i].origin_T;
    }
}

static __device__ __forceinline__ bool trianglesCollide(const Eigen::Matrix3f& link_R, const Eigen::Vector3f& link_T,
                                                 int link_vert_offset,
                                                 int rob_tri_idx, int obs_tri_idx,
                                                 const Eigen::Vector3f* pRob_verts, const Triangle* pRob_tris,
                                                 const Eigen::Vector3f* pObs_verts, const Triangle* pObs_tris) {
    Triangle rob_tri = pRob_tris[rob_tri_idx];
    Triangle obs_tri = pObs_tris[obs_tri_idx];

    Eigen::Vector3f rob_v0 = pRob_verts[link_vert_offset + rob_tri.v1];
    Eigen::Vector3f rob_v1 = pRob_verts[link_vert_offset + rob_tri.v2];
    Eigen::Vector3f rob_v2 = pRob_verts[link_vert_offset + rob_tri.v3];
    Eigen::Vector3f obs_v0 = pObs_verts[obs_tri.v1];
    Eigen::Vector3f obs_v1 = pObs_verts[obs_tri.v2];
    Eigen::Vector3f obs_v2 = pObs_verts[obs_tri.v3];

    rob_v0 = link_R * rob_v0 + link_T;
    rob_v1 = link_R * rob_v1 + link_T;
    rob_v2 = link_R * rob_v2 + link_T;

    return !triangles_valid(rob_v0, rob_v1, rob_v2, obs_v0, obs_v1, obs_v2);
}

static __device__ bool linkCollides(const Eigen::Matrix3f& link_R, const Eigen::Vector3f& link_T,
                             int rob_root, int link_vert_offset, int link_tri_offset,
                             const Eigen::Matrix3f* pRob_R, const Eigen::Vector3f* pRob_T,
                             const Eigen::Vector3f* pRob_dim, const int16_t* pRob_first_child,
                             const Eigen::Vector3f* pRob_verts, const Triangle* pRob_tris,
                             const Eigen::Matrix3f* pR_obs, const Eigen::Vector3f* pT_obs,
                             const Eigen::Vector3f* pObs_dim, const int16_t* pObs_first_child,
                             const Eigen::Vector3f* pObs_verts, const Triangle* pObs_tris) {
    const int STACK_SIZE = 512;
    const int NUM_CHILDREN = 4;
    const float epsilon = 1e-6f;

    int stack_rob[STACK_SIZE];
    int stack_obs[STACK_SIZE];
    int sp = 0;

    stack_rob[sp] = rob_root;
    stack_obs[sp] = 0;
    ++sp;

    while (sp > 0) {
        --sp;
        int rob_node = stack_rob[sp];
        int obs_node = stack_obs[sp];

        int rob_fc = pRob_first_child[rob_node];
        int obs_fc = pObs_first_child[obs_node];

        Eigen::Matrix3f node_R = pRob_R[rob_node];
        Eigen::Vector3f node_T = pRob_T[rob_node];
        Eigen::Vector3f b = pRob_dim[rob_node];

        Eigen::Matrix3f world_R = link_R * node_R;
        Eigen::Vector3f world_T = link_T + link_R * node_T;

        Eigen::Matrix3f R_obs_abs = pR_obs[obs_node];
        Eigen::Vector3f T_obs_abs = pT_obs[obs_node];
        Eigen::Vector3f a = pObs_dim[obs_node];

        Eigen::Matrix3f B = R_obs_abs.transpose() * world_R;
        Eigen::Matrix3f Bf = B.cwiseAbs();
        Bf.array() += epsilon;
        Eigen::Vector3f T = (world_T - T_obs_abs).transpose() * R_obs_abs;

        if (!obbOverlap(a, b, B, Bf, T)) {
            continue;
        }

        if (rob_fc < 0 && obs_fc < 0) {
            int rob_tri = -rob_fc - 1 + link_tri_offset;
            int obs_tri = -obs_fc - 1;
            if (trianglesCollide(link_R, link_T, link_vert_offset, rob_tri, obs_tri,
                                 pRob_verts, pRob_tris, pObs_verts, pObs_tris)) {
                return true;
            }
        } else if (rob_fc < 0) {
            for (int k = 0; k < NUM_CHILDREN; ++k) {
                int ochild = obs_fc + k;
                if (pObs_first_child[ochild] == 0) continue;
                if (sp >= STACK_SIZE) return true;
                stack_rob[sp] = rob_node;
                stack_obs[sp] = ochild;
                ++sp;
            }
        } else if (obs_fc < 0) {
            for (int k = 0; k < NUM_CHILDREN; ++k) {
                int rchild = rob_fc + k;
                if (pRob_first_child[rchild] == 0) continue;
                if (sp >= STACK_SIZE) return true;
                stack_rob[sp] = rchild;
                stack_obs[sp] = obs_node;
                ++sp;
            }
        } else {
            for (int k = 0; k < NUM_CHILDREN; ++k) {
                int rchild = rob_fc + k;
                if (pRob_first_child[rchild] == 0) continue;
                for (int j = 0; j < NUM_CHILDREN; ++j) {
                    int ochild = obs_fc + j;
                    if (pObs_first_child[ochild] == 0) continue;
                    if (sp >= STACK_SIZE) return true;
                    stack_rob[sp] = rchild;
                    stack_obs[sp] = ochild;
                    ++sp;
                }
            }
        }
    }
    return false;
}

template <size_t N>
__global__ void d_bvh_articulated(const Eigen::Matrix3f* pR_obs, const Eigen::Vector3f* pT_obs,
                                  const Eigen::Vector3f* pObs_dim, const int16_t* pObs_first_child,
                                  size_t num_obs_nodes,
                                  const Eigen::Vector3f* pObs_verts, const Triangle* pObs_tris,
                                  const Eigen::Matrix3f* pRob_R, const Eigen::Vector3f* pRob_T,
                                  const Eigen::Vector3f* pRob_dim, const int16_t* pRob_first_child,
                                  const Eigen::Vector3f* pRob_verts, const Triangle* pRob_tris,
                                  const int* pLinkOffset, const int* pLinkVertOffset, const int* pLinkTriOffset,
                                  const JointParams* pJoints,
                                  const articulated_conf<N>* pConf, size_t num_confs,
                                  bool* pdisjoint) {
    size_t index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= num_confs) {
        return;
    }

    articulated_conf<N> conf = pConf[index];

    Eigen::Matrix3f link_R[N + 1];
    Eigen::Vector3f link_T[N + 1];
    forwardKinematics(conf, pJoints, link_R, link_T);

    bool collision = false;
    for (size_t l = 0; l < N + 1; ++l) {
        int start = pLinkOffset[l];
        int end = pLinkOffset[l + 1];
        if (start >= end) {
            continue;
        }
        if (linkCollides(link_R[l], link_T[l], start,
                         pLinkVertOffset[l], pLinkTriOffset[l],
                         pRob_R, pRob_T, pRob_dim, pRob_first_child,
                         pRob_verts, pRob_tris,
                         pR_obs, pT_obs, pObs_dim, pObs_first_child,
                         pObs_verts, pObs_tris)) {
            collision = true;
            break;
        }
    }

    pdisjoint[index] = !collision;
}

template <size_t N>
double bvh_articulated(const std::string& robot_urdf_path,
                       const BVNode_soa& obs_BVH, const MeshData& obs_mesh,
                       const std::vector<articulated_conf<N>>& confs,
                       std::vector<bool>& valid, bool dry_run) {
    cudaEvent_t start, stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);

    const size_t num_confs = confs.size();
    valid.assign(num_confs, false);
    if (num_confs == 0) {
        return 0.0;
    }

    ArticulatedRobot robot;
    if (!parseSerialChainURDF(robot_urdf_path, robot)) {
        std::cerr << "Failed to parse URDF: " << robot_urdf_path << std::endl;
        return 0.0;
    }
    if (robot.num_joints != N) {
        std::cerr << "URDF has " << robot.num_joints
                  << " joints, but bvh_articulated was instantiated with N=" << N << std::endl;
        return 0.0;
    }

    std::vector<Eigen::Matrix3f> rob_R;
    std::vector<Eigen::Vector3f> rob_T;
    std::vector<Eigen::Vector3f> rob_dim;
    std::vector<int16_t> rob_first_child;
    std::vector<Eigen::Vector3f> rob_verts;
    std::vector<Triangle> rob_tris;

    std::vector<int> link_offset(N + 2, 0);
    std::vector<int> link_vert_offset(N + 2, 0);
    std::vector<int> link_tri_offset(N + 2, 0);

    for (size_t l = 0; l < robot.num_joints + 1; ++l) {
        link_offset[l] = static_cast<int>(rob_first_child.size());
        link_vert_offset[l] = static_cast<int>(rob_verts.size());
        link_tri_offset[l] = static_cast<int>(rob_tris.size());

        if (!robot.link_meshes[l].empty()) {
            BVNode_soa bvh = BVH_n_ary_hierarchy_from_mesh(robot.link_meshes[l].c_str(), 2);
            for (size_t i = 0; i < bvh.size; ++i) {
                rob_R.push_back(bvh.pR[i]);
                rob_T.push_back(bvh.pT[i]);
                rob_dim.push_back(bvh.pDim[i]);
                rob_first_child.push_back(bvh.first_child[i]);
            }

            MeshData md;
            loadOBJFile(robot.link_meshes[l], md.vertices, md.triangles);
            rob_verts.insert(rob_verts.end(), md.vertices.begin(), md.vertices.end());
            rob_tris.insert(rob_tris.end(), md.triangles.begin(), md.triangles.end());
        }
    }
    link_offset[N + 1] = static_cast<int>(rob_first_child.size());
    link_vert_offset[N + 1] = static_cast<int>(rob_verts.size());
    link_tri_offset[N + 1] = static_cast<int>(rob_tris.size());

    std::vector<JointParams> joints = robot.joints;

    const int blockSize = 32;
    const int gridSize = static_cast<int>((num_confs + blockSize - 1) / blockSize);

    Eigen::Matrix3f* d_R_obs;
    Eigen::Vector3f* d_T_obs;
    Eigen::Vector3f* d_Obs_dim;
    int16_t* d_Obs_first_child;
    Eigen::Vector3f* d_Obs_verts;
    Triangle* d_Obs_tris;

    Eigen::Matrix3f* d_Rob_R;
    Eigen::Vector3f* d_Rob_T;
    Eigen::Vector3f* d_Rob_dim;
    int16_t* d_Rob_first_child;
    Eigen::Vector3f* d_Rob_verts;
    Triangle* d_Rob_tris;

    int* d_LinkOffset;
    int* d_LinkVertOffset;
    int* d_LinkTriOffset;
    JointParams* d_Joints;
    articulated_conf<N>* d_Conf;
    bool* d_disjoint;

    cudaEventRecord(start, 0);

    cudaMalloc((void**)&d_R_obs, obs_BVH.size * sizeof(Eigen::Matrix3f));
    cudaMalloc((void**)&d_T_obs, obs_BVH.size * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Obs_dim, obs_BVH.size * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Obs_first_child, obs_BVH.size * sizeof(int16_t));
    cudaMalloc((void**)&d_Obs_verts, obs_mesh.vertices.size() * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Obs_tris, obs_mesh.triangles.size() * sizeof(Triangle));

    cudaMalloc((void**)&d_Rob_R, rob_R.size() * sizeof(Eigen::Matrix3f));
    cudaMalloc((void**)&d_Rob_T, rob_T.size() * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Rob_dim, rob_dim.size() * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Rob_first_child, rob_first_child.size() * sizeof(int16_t));
    cudaMalloc((void**)&d_Rob_verts, rob_verts.size() * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Rob_tris, rob_tris.size() * sizeof(Triangle));

    cudaMalloc((void**)&d_LinkOffset, (N + 2) * sizeof(int));
    cudaMalloc((void**)&d_LinkVertOffset, (N + 2) * sizeof(int));
    cudaMalloc((void**)&d_LinkTriOffset, (N + 2) * sizeof(int));
    cudaMalloc((void**)&d_Joints, N * sizeof(JointParams));
    cudaMalloc((void**)&d_Conf, num_confs * sizeof(articulated_conf<N>));
    cudaMalloc((void**)&d_disjoint, gridSize * blockSize * sizeof(bool));

    checkCudaMem(cudaMemcpy(d_R_obs, obs_BVH.pR, obs_BVH.size * sizeof(Eigen::Matrix3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_T_obs, obs_BVH.pT, obs_BVH.size * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_dim, obs_BVH.pDim, obs_BVH.size * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_first_child, obs_BVH.first_child, obs_BVH.size * sizeof(int16_t), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_verts, obs_mesh.vertices.data(), obs_mesh.vertices.size() * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_tris, obs_mesh.triangles.data(), obs_mesh.triangles.size() * sizeof(Triangle), cudaMemcpyHostToDevice));

    checkCudaMem(cudaMemcpy(d_Rob_R, rob_R.data(), rob_R.size() * sizeof(Eigen::Matrix3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_T, rob_T.data(), rob_T.size() * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_dim, rob_dim.data(), rob_dim.size() * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_first_child, rob_first_child.data(), rob_first_child.size() * sizeof(int16_t), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_verts, rob_verts.data(), rob_verts.size() * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_tris, rob_tris.data(), rob_tris.size() * sizeof(Triangle), cudaMemcpyHostToDevice));

    checkCudaMem(cudaMemcpy(d_LinkOffset, link_offset.data(), (N + 2) * sizeof(int), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_LinkVertOffset, link_vert_offset.data(), (N + 2) * sizeof(int), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_LinkTriOffset, link_tri_offset.data(), (N + 2) * sizeof(int), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Joints, joints.data(), N * sizeof(JointParams), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Conf, confs.data(), num_confs * sizeof(articulated_conf<N>), cudaMemcpyHostToDevice));
    cudaDeviceSynchronize();

    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    float duration = 0;
    cudaEventElapsedTime(&duration, start, stop);
    std::cout << "Articulated BVH allocation and transfer to GPU took " << duration << " ms." << std::endl;

    auto launch = [&]() {
        d_bvh_articulated<N><<<gridSize, blockSize>>>(
            d_R_obs, d_T_obs, d_Obs_dim, d_Obs_first_child, obs_BVH.size,
            d_Obs_verts, d_Obs_tris,
            d_Rob_R, d_Rob_T, d_Rob_dim, d_Rob_first_child,
            d_Rob_verts, d_Rob_tris,
            d_LinkOffset, d_LinkVertOffset, d_LinkTriOffset,
            d_Joints, d_Conf, num_confs,
            d_disjoint);
    };

    if (dry_run) {
        launch();
        checkCudaMem(cudaGetLastError());
        checkCudaMem(cudaDeviceSynchronize());
        std::cout << "Articulated BVH dry run completed successfully." << std::endl;
    }

    cudaEventRecord(start, 0);
    launch();
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&duration, start, stop);
    std::cout << "Articulated BVH GPU kernel took " << duration << " ms for "
              << num_confs << " configurations." << std::endl;

    std::unique_ptr<bool[]> disjoint(new bool[num_confs]);
    checkCudaMem(cudaMemcpy(disjoint.get(), d_disjoint, num_confs * sizeof(bool), cudaMemcpyDeviceToHost));
    cudaDeviceSynchronize();

    for (size_t i = 0; i < num_confs; ++i) {
        valid[i] = disjoint[i];
    }

    cudaFree(d_R_obs);
    cudaFree(d_T_obs);
    cudaFree(d_Obs_dim);
    cudaFree(d_Obs_first_child);
    cudaFree(d_Obs_verts);
    cudaFree(d_Obs_tris);
    cudaFree(d_Rob_R);
    cudaFree(d_Rob_T);
    cudaFree(d_Rob_dim);
    cudaFree(d_Rob_first_child);
    cudaFree(d_Rob_verts);
    cudaFree(d_Rob_tris);
    cudaFree(d_LinkOffset);
    cudaFree(d_LinkVertOffset);
    cudaFree(d_LinkTriOffset);
    cudaFree(d_Joints);
    cudaFree(d_Conf);
    cudaFree(d_disjoint);

    return duration;
}
