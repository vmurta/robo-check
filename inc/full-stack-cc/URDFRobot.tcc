#include <fstream>
#include <iostream>

template <size_t N>
__host__ __device__ void forwardKinematicsTree(const articulated_conf<N>& conf,
                                               int num_links,
                                               const int* link_parent,
                                               const int* link_order,
                                               const JointParams* joint_origin,
                                               const Eigen::Vector3f* joint_axis,
                                               const int* joint_angle_idx,
                                               Eigen::Matrix3f* link_R,
                                               Eigen::Vector3f* link_T) {
    for (int k = 0; k < num_links; ++k) {
        int l = link_order[k];
        int p = link_parent[l];
        if (p < 0) {
            link_R[l] = Eigen::Matrix3f::Identity();
            link_T[l] = Eigen::Vector3f::Zero();
            continue;
        }

        float angle = 0.0f;
        int ai = joint_angle_idx[l];
        if (ai >= 0) {
            angle = conf[ai];
        }

        Eigen::Matrix3f Rj = axisAngleToRotation(joint_axis[l], angle);
        link_R[l] = link_R[p] * joint_origin[l].origin_R * Rj;
        link_T[l] = link_T[p] + link_R[p] * joint_origin[l].origin_T;
    }
}

template <size_t N>
__global__ void d_bvh_urdf(const Eigen::Matrix3f* pR_obs, const Eigen::Vector3f* pT_obs,
                           const Eigen::Vector3f* pObs_dim, const int32_t* pObs_first_child,
                           size_t num_obs_nodes,
                           const Eigen::Vector3f* pObs_verts, const Triangle* pObs_tris,
                           const Eigen::Matrix3f* pRob_R, const Eigen::Vector3f* pRob_T,
                           const Eigen::Vector3f* pRob_dim, const int32_t* pRob_first_child,
                           const Eigen::Vector3f* pRob_verts, const Triangle* pRob_tris,
                           const int* pLinkOffset, const int* pLinkVertOffset, const int* pLinkTriOffset,
                           int num_links,
                           const int* pLinkParent, const int* pLinkOrder,
                           const JointParams* pJointOrigin, const Eigen::Vector3f* pJointAxis,
                           const int* pJointAngleIdx,
                           const articulated_conf<N>* pConf, size_t num_confs,
                           bool* pdisjoint,
                           unsigned long long* overflowCounter) {
    size_t index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= num_confs) {
        return;
    }

    articulated_conf<N> conf = pConf[index];

    Eigen::Matrix3f link_R[URDF_MAX_LINKS];
    Eigen::Vector3f link_T[URDF_MAX_LINKS];
    forwardKinematicsTree(conf, num_links, pLinkParent, pLinkOrder,
                          pJointOrigin, pJointAxis, pJointAngleIdx,
                          link_R, link_T);

    bool collision = false;
    for (int l = 0; l < num_links; ++l) {
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
                         pObs_verts, pObs_tris, overflowCounter)) {
            collision = true;
            break;
        }
    }

    pdisjoint[index] = !collision;
}

template <size_t N>
double bvh_urdf(const std::string& robot_urdf_path,
                const BVNode_soa<int32_t>& obs_BVH, const MeshData& obs_mesh,
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

    URDFRobot robot;
    if (!parseFullURDF(robot_urdf_path, robot)) {
        std::cerr << "Failed to parse URDF: " << robot_urdf_path << std::endl;
        return 0.0;
    }
    if (robot.num_joints != static_cast<int>(N)) {
        std::cerr << "URDF has " << robot.num_joints
                  << " movable joints, but bvh_urdf was instantiated with N=" << N << std::endl;
        return 0.0;
    }
    if (robot.num_links > URDF_MAX_LINKS) {
        std::cerr << "URDF has " << robot.num_links << " links, exceeding URDF_MAX_LINKS="
                  << URDF_MAX_LINKS << std::endl;
        return 0.0;
    }

    const int num_links = robot.num_links;

    std::vector<Eigen::Matrix3f> rob_R;
    std::vector<Eigen::Vector3f> rob_T;
    std::vector<Eigen::Vector3f> rob_dim;
    std::vector<int32_t> rob_first_child;
    std::vector<Eigen::Vector3f> rob_verts;
    std::vector<Triangle> rob_tris;

    std::vector<int> link_offset(num_links + 1, 0);
    std::vector<int> link_vert_offset(num_links + 1, 0);
    std::vector<int> link_tri_offset(num_links + 1, 0);

    for (int l = 0; l < num_links; ++l) {
        link_offset[l] = static_cast<int>(rob_first_child.size());
        link_vert_offset[l] = static_cast<int>(rob_verts.size());
        link_tri_offset[l] = static_cast<int>(rob_tris.size());

        if (!robot.link_meshes[l].empty()) {
            BVNode_soa<int32_t> bvh = BVH_n_ary_hierarchy_from_mesh<int32_t>(robot.link_meshes[l].c_str(), 2);
            const int base = static_cast<int>(rob_first_child.size());
            for (size_t i = 0; i < bvh.size; ++i) {
                rob_R.push_back(bvh.pR[i]);
                rob_T.push_back(bvh.pT[i]);
                rob_dim.push_back(bvh.pDim[i]);
                // first_child pointers are indices relative to this link's own
                // BVH; rebase internal pointers onto the concatenated array.
                int32_t fc = (int32_t)bvh.first_child[i];
                if (fc > 0) {
                    fc += base;
                }
                rob_first_child.push_back(fc);
            }

            MeshData md;
            loadOBJFile(robot.link_meshes[l], md.vertices, md.triangles);
            rob_verts.insert(rob_verts.end(), md.vertices.begin(), md.vertices.end());
            rob_tris.insert(rob_tris.end(), md.triangles.begin(), md.triangles.end());
        }
    }
    link_offset[num_links] = static_cast<int>(rob_first_child.size());
    link_vert_offset[num_links] = static_cast<int>(rob_verts.size());
    link_tri_offset[num_links] = static_cast<int>(rob_tris.size());

    const int blockSize = 32;
    const int gridSize = static_cast<int>((num_confs + blockSize - 1) / blockSize);

    Eigen::Matrix3f* d_R_obs;
    Eigen::Vector3f* d_T_obs;
    Eigen::Vector3f* d_Obs_dim;
    int32_t* d_Obs_first_child;
    Eigen::Vector3f* d_Obs_verts;
    Triangle* d_Obs_tris;

    Eigen::Matrix3f* d_Rob_R;
    Eigen::Vector3f* d_Rob_T;
    Eigen::Vector3f* d_Rob_dim;
    int32_t* d_Rob_first_child;
    Eigen::Vector3f* d_Rob_verts;
    Triangle* d_Rob_tris;

    int* d_LinkOffset;
    int* d_LinkVertOffset;
    int* d_LinkTriOffset;
    int* d_LinkParent;
    int* d_LinkOrder;
    JointParams* d_JointOrigin;
    Eigen::Vector3f* d_JointAxis;
    int* d_JointAngleIdx;
    articulated_conf<N>* d_Conf;
    bool* d_disjoint;
    unsigned long long* d_overflow;

    cudaEventRecord(start, 0);

    cudaMalloc((void**)&d_R_obs, obs_BVH.size * sizeof(Eigen::Matrix3f));
    cudaMalloc((void**)&d_T_obs, obs_BVH.size * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Obs_dim, obs_BVH.size * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Obs_first_child, obs_BVH.size * sizeof(int32_t));
    cudaMalloc((void**)&d_Obs_verts, obs_mesh.vertices.size() * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Obs_tris, obs_mesh.triangles.size() * sizeof(Triangle));

    cudaMalloc((void**)&d_Rob_R, rob_R.size() * sizeof(Eigen::Matrix3f));
    cudaMalloc((void**)&d_Rob_T, rob_T.size() * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Rob_dim, rob_dim.size() * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Rob_first_child, rob_first_child.size() * sizeof(int32_t));
    cudaMalloc((void**)&d_Rob_verts, rob_verts.size() * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Rob_tris, rob_tris.size() * sizeof(Triangle));

    cudaMalloc((void**)&d_LinkOffset, (num_links + 1) * sizeof(int));
    cudaMalloc((void**)&d_LinkVertOffset, (num_links + 1) * sizeof(int));
    cudaMalloc((void**)&d_LinkTriOffset, (num_links + 1) * sizeof(int));
    cudaMalloc((void**)&d_LinkParent, num_links * sizeof(int));
    cudaMalloc((void**)&d_LinkOrder, num_links * sizeof(int));
    cudaMalloc((void**)&d_JointOrigin, num_links * sizeof(JointParams));
    cudaMalloc((void**)&d_JointAxis, num_links * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_JointAngleIdx, num_links * sizeof(int));
    cudaMalloc((void**)&d_Conf, num_confs * sizeof(articulated_conf<N>));
    cudaMalloc((void**)&d_disjoint, gridSize * blockSize * sizeof(bool));
    cudaMalloc((void**)&d_overflow, sizeof(unsigned long long));

    checkCudaMem(cudaMemcpy(d_R_obs, obs_BVH.pR, obs_BVH.size * sizeof(Eigen::Matrix3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_T_obs, obs_BVH.pT, obs_BVH.size * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_dim, obs_BVH.pDim, obs_BVH.size * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_first_child, obs_BVH.first_child, obs_BVH.size * sizeof(int32_t), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_verts, obs_mesh.vertices.data(), obs_mesh.vertices.size() * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_tris, obs_mesh.triangles.data(), obs_mesh.triangles.size() * sizeof(Triangle), cudaMemcpyHostToDevice));

    checkCudaMem(cudaMemcpy(d_Rob_R, rob_R.data(), rob_R.size() * sizeof(Eigen::Matrix3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_T, rob_T.data(), rob_T.size() * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_dim, rob_dim.data(), rob_dim.size() * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_first_child, rob_first_child.data(), rob_first_child.size() * sizeof(int32_t), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_verts, rob_verts.data(), rob_verts.size() * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_tris, rob_tris.data(), rob_tris.size() * sizeof(Triangle), cudaMemcpyHostToDevice));

    checkCudaMem(cudaMemcpy(d_LinkOffset, link_offset.data(), (num_links + 1) * sizeof(int), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_LinkVertOffset, link_vert_offset.data(), (num_links + 1) * sizeof(int), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_LinkTriOffset, link_tri_offset.data(), (num_links + 1) * sizeof(int), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_LinkParent, robot.link_parent.data(), num_links * sizeof(int), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_LinkOrder, robot.link_order.data(), num_links * sizeof(int), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_JointOrigin, robot.joint_origin.data(), num_links * sizeof(JointParams), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_JointAxis, robot.joint_axis.data(), num_links * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_JointAngleIdx, robot.joint_angle_idx.data(), num_links * sizeof(int), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Conf, confs.data(), num_confs * sizeof(articulated_conf<N>), cudaMemcpyHostToDevice));
    cudaDeviceSynchronize();

    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    float duration = 0;
    cudaEventElapsedTime(&duration, start, stop);
    std::cout << "URDF BVH allocation and transfer to GPU took " << duration << " ms." << std::endl;

    auto launch = [&]() {
        d_bvh_urdf<N><<<gridSize, blockSize>>>(
            d_R_obs, d_T_obs, d_Obs_dim, d_Obs_first_child, obs_BVH.size,
            d_Obs_verts, d_Obs_tris,
            d_Rob_R, d_Rob_T, d_Rob_dim, d_Rob_first_child,
            d_Rob_verts, d_Rob_tris,
            d_LinkOffset, d_LinkVertOffset, d_LinkTriOffset,
            num_links,
            d_LinkParent, d_LinkOrder,
            d_JointOrigin, d_JointAxis, d_JointAngleIdx,
            d_Conf, num_confs,
            d_disjoint, d_overflow);
    };

    if (dry_run) {
        checkCudaMem(cudaMemset(d_overflow, 0, sizeof(unsigned long long)));
        launch();
        checkCudaMem(cudaGetLastError());
        checkCudaMem(cudaDeviceSynchronize());
        std::cout << "URDF BVH dry run completed successfully." << std::endl;
    }

    checkCudaMem(cudaMemset(d_overflow, 0, sizeof(unsigned long long)));
    cudaEventRecord(start, 0);
    launch();
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&duration, start, stop);
    std::cout << "URDF BVH GPU kernel took " << duration << " ms for "
              << num_confs << " configurations." << std::endl;
    unsigned long long h_overflow = 0;
    checkCudaMem(cudaMemcpy(&h_overflow, d_overflow, sizeof(unsigned long long), cudaMemcpyDeviceToHost));
    if (h_overflow != 0) {
        std::cerr << "WARNING: " << h_overflow
                  << " conservative stack-overflow early exits (potential false positives)"
                  << std::endl;
    }

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
    cudaFree(d_LinkParent);
    cudaFree(d_LinkOrder);
    cudaFree(d_JointOrigin);
    cudaFree(d_JointAxis);
    cudaFree(d_JointAngleIdx);
    cudaFree(d_Conf);
    cudaFree(d_disjoint);
    cudaFree(d_overflow);

    return duration;
}
