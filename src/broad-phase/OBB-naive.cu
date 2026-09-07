#include "OBB-naive.hu"

// is there a way to specify children with pointer arithmetic that respects coalescing?
// maybe bring in all the children into shared memory first, then use pointers?
__global__ void d_obb_dyn_1box( const Eigen::Matrix3f* pR_obs, const Eigen::Vector3f* pT_obs,
                                const Eigen::Matrix3f* pR_rob, const Eigen::Vector3f* pT_rob,
                                const Eigen::Vector3f* pRob_dim, const Eigen::Vector3f* pObs_dim,
                                const Eigen::Matrix3f* pRob_conf_rot, const Eigen::Vector3f* pRob_conf_trans,
                                bool* pdisjoint) {

    size_t index = blockIdx.x * blockDim.x + threadIdx.x;
    Eigen::Matrix3f R_obs_abs = pR_obs[index]; // rotation of B wrt origin
    Eigen::Vector3f T_obs_abs = pT_obs[index]; // translation of B wrt origin

    Eigen::Matrix3f R_rob_abs = pR_rob[index]; // rotation of A wrt origin
    Eigen::Vector3f T_rob_abs = pT_rob[index]; // translation of A wrt origin

    Eigen::Vector3f b = pRob_dim[index]; // half dimensions of box A
    Eigen::Vector3f a = pObs_dim[index]; // half dimensions of box B

    Eigen::Matrix3f R_conf = pRob_conf_rot[index]; // rotation of robot wrt world
    Eigen::Vector3f T_conf = pRob_conf_trans[index]; // translation of robot wrt world


    float t; // distance between centers of the two boxes as projected onto the axis
    const float epsilon = 1e-6f; // small value to avoid numerical issues

    //Calculate relative rotation of B wrt A
    //TODO: precompute inverse rotations of A
    // Take the absolute value of the rotation matrix B, add epsilon to avoid numerical issues
    Eigen::Matrix3f B = R_obs_abs.transpose() * (R_conf * R_rob_abs); // rotation of A wrt B
    Eigen::Matrix3f Bf = B.cwiseAbs();
    Bf.array() += epsilon;

    Eigen::Vector3f T = (T_conf + R_conf * T_rob_abs - T_obs_abs).transpose() * R_obs_abs; // translation of A wrt B
    // first tests: cross product of axes within the same box
    // (always resulting in the third axis of the box)
    ////////////////////////////////////////////////////////////////////////////////
    // A1 x A2 = A0
    t = fabsf(T[0]);

    //Since L = A0 is a unit vector (as it is the cross product of unit vectors), no need to multiply t
    // t dot L = t
    // \sum |a_i A^i * L | = a_0 + 0 + 0
    // \sum |b_i B^i * L | = b_i B^i * A0 = first element of each column vector of Bf = Bf.row(0).dot(b)

    if(t > (a[0] + Bf.row(0).dot(b))){
        pdisjoint[index] = true;
        return;
    }

  // B1 x B2 = B0
    t = fabsf(B.col(0).dot(T));

    if(t > (b[0] + Bf.col(0).dot(a))){
        pdisjoint[index] = true;
        return;
    }

    // A2 x A0 = A1
    t = fabsf(T[1]);

    if(t > (a[1] + Bf.row(1).dot(b))){
        pdisjoint[index] = true;
        return;
    }

    // A0 x A1 = A2
    t =fabsf(T[2]);

    if(t > (a[2] + Bf.row(2).dot(b))){
        pdisjoint[index] = true;
        return;
    }

    // B2 x B0 = B1
    t = fabsf(B.col(1).dot(T));

    if(t > (b[1] + Bf.col(1).dot(a))){
        pdisjoint[index] = true;
        return;
    }

    // B0 x B1 = B2
    t = fabsf(B.col(2).dot(T));

    if(t > (b[2] + Bf.col(2).dot(a))){
        pdisjoint[index] = true;
        return;
    }

    // A0 x B0
    t = fabsf(T[2] * B(1, 0) - T[1] * B(2, 0));

    if(t > (a[1] * Bf(2, 0) + a[2] * Bf(1, 0) +
            b[1] * Bf(0, 2) + b[2] * Bf(0, 1))){
        pdisjoint[index] = true;
        return;
    }

    // A0 x B1
    t = fabsf(T[2] * B(1, 1) - T[1] * B(2, 1));

    if(t > (a[1] * Bf(2, 1) + a[2] * Bf(1, 1) +
            b[0] * Bf(0, 2) + b[2] * Bf(0, 0))){
        pdisjoint[index] = true;
        return;
    }

    // A0 x B2
    t = fabsf(T[2] * B(1, 2) - T[1] * B(2, 2));

    if(t > (a[1] * Bf(2, 2) + a[2] * Bf(1, 2) +
            b[0] * Bf(0, 1) + b[1] * Bf(0, 0))){
        pdisjoint[index] = true;
        return;
    }

    // A1 x B0
    t = fabsf(T[0] * B(2, 0) - T[2] * B(0, 0));

    if(t > (a[0] * Bf(2, 0) + a[2] * Bf(0, 0) +
            b[1] * Bf(1, 2) + b[2] * Bf(1, 1))){
        pdisjoint[index] = true;
        return;
    }

    // A1 x B1
    t = fabsf(T[0] * B(2, 1) - T[2] * B(0, 1));

    if(t > (a[0] * Bf(2, 1) + a[2] * Bf(0, 1) +
            b[0] * Bf(1, 2) + b[2] * Bf(1, 0))){
        pdisjoint[index] = true;
        return;
    }

    // A1 x B2
    t = fabsf(T[0] * B(2, 2) - T[2] * B(0, 2));

    if(t > (a[0] * Bf(2, 2) + a[2] * Bf(0, 2) +
            b[0] * Bf(1, 1) + b[1] * Bf(1, 0))){
        pdisjoint[index] = true;
        return;
    }

    // A2 x B0
    t = fabsf(T[1] * B(0, 0) - T[0] * B(1, 0));

    if(t > (a[0] * Bf(1, 0) + a[1] * Bf(0, 0) +
            b[1] * Bf(2, 2) + b[2] * Bf(2, 1))){
        pdisjoint[index] = true;
        return;
    }

    // A2 x B1
    t = fabsf(T[1] * B(0, 1) - T[0] * B(1, 1));

    if(t > (a[0] * Bf(1, 1) + a[1] * Bf(0, 1) +
            b[0] * Bf(2, 2) + b[2] * Bf(2, 0))){
        pdisjoint[index] = true;
        return;
    }
    // A2 x B2
    t = fabsf(T[1] * B(0, 2) - T[0] * B(1, 2));

    if(t > (a[0] * Bf(1, 2) + a[1] * Bf(0, 2) +
            b[0] * Bf(2, 1) + b[1] * Bf(2, 0))){
        pdisjoint[index] = true;
        return;
    }

    // pdisjoint[index] = false;
}

// high level broad phase test -- runs top level obb for transformed robot and static obstacles
// does not assume robot BVH is wrt any obstacle
double broad_naive_1() {
    cudaEvent_t start, stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);

    // Test OBB disjoint function

    // Load Robot and Obstacle BVH
    std::cout << "Rob BVH:" << std::endl;
    OBB_soa rob_BVH = hierarchy_from_mesh("/home/victor/Projects/robo-check/data/models/alpha1.0/robot.obj");
    std::cout << "Obs BVH:" << std::endl;
    OBB_soa obs_BVH = hierarchy_from_mesh("/home/victor/Projects/robo-check/data/models/alpha1.0/obstacle.obj");

    // Load Configurations
    const int num_confs = 100000;
    std::vector<Configuration> confs;
    confs.reserve(num_confs);
    // std::cout << "Reading configurations from file..." << std::endl;
    readConfigurationFromFile("/home/victor/Projects/robo-check/data/configurations/hard_confs100,000.conf", confs);
    Eigen::Matrix3f rob_rotations[num_confs];
    Eigen::Vector3f rob_translations[num_confs];
    for (int i = 0; i < num_confs; ++i) {
        rob_rotations[i] = createRotationMatrix(confs[i]);
        rob_translations[i] = Eigen::Vector3f(confs[i].x, confs[i].y, confs[i].z);
    }
    // std::cout << "Created Rotation matrices from configurations" << std::endl;

    bool disjoint[num_confs] = {false};
    Eigen::Matrix3f* d_R_obs;
    Eigen::Vector3f* d_T_obs;
    Eigen::Matrix3f* d_R_rob;
    Eigen::Vector3f* d_T_rob;
    Eigen::Vector3f* d_Rob_dim;
    Eigen::Vector3f* d_Obs_dim;
    Eigen::Matrix3f* d_Rob_conf_rot;
    Eigen::Vector3f* d_Rob_conf_trans;
    bool* pdisjoint;
    // std::cout << "Allocated host memory" << std::endl;

    // Allocate memory for device pointers
    cudaMalloc((void**)&d_R_obs, num_confs * sizeof(Eigen::Matrix3f));
    cudaMalloc((void**)&d_T_obs, num_confs * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_R_rob, num_confs * sizeof(Eigen::Matrix3f));
    cudaMalloc((void**)&d_T_rob, num_confs * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Rob_dim, num_confs * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Obs_dim, num_confs * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Rob_conf_rot, num_confs * sizeof(Eigen::Matrix3f));
    cudaMalloc((void**)&d_Rob_conf_trans, num_confs * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&pdisjoint, num_confs * sizeof(bool));
    // std::cout << "Allocated device memory" << std::endl;

    //Make one dummy duplicates of the top level box for each configuration
    std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f>> R_obs(num_confs);
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> T_obs(num_confs);
    std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f>> R_rob(num_confs);
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> T_rob(num_confs);
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> rob_dim(num_confs);
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> obs_dim(num_confs);
    // std::cout << "Top level robot box size:" << rob_BVH.pDim[0] << std::endl;
    // std::cout << "Top level robot box rotation:" << std::endl << rob_BVH.pR[0] << std::endl;
    // std::cout << "Top level robot box translation:" << rob_BVH.pT[0] << std::endl;
    // std::cout << "Top level obstacle box size:" << rob_BVH.pDim[0] << std::endl;
    // std::cout << "Top level obstacle box rotation:" << obs_BVH.pR[0] << std::endl;
    // std::cout << "Top level obstacle box translation:" << obs_BVH.pT[0] << std::endl;

    for (int i = 0; i < num_confs; ++i) {
        obs_dim[i] = obs_BVH.pDim[0];
        R_obs[i] = obs_BVH.pR[0];
        T_obs[i] = obs_BVH.pT[0];

        rob_dim[i] = rob_BVH.pDim[0];
        R_rob[i] = rob_BVH.pR[0];
        T_rob[i] = rob_BVH.pT[0];
    }

    //print out obs OBB coords
    // std::cout << "Obstacle OBB coords:" << std::endl;
    // {
    //     std::vector<Eigen::Vector3f> vertices = obs_BVH.getBoxVertices(0);
    //     for (auto p : vertices){
    //         std::cout << p.transpose() << std::endl;
    //     }
    // }
    // std::cout << "Obstacle obb stats:" << std::endl;
    // std::cout << " Center: " << obs_BVH.pT[0].transpose()
    //             << " Half-dimensions: " << obs_BVH.pDim[0].transpose() << std::endl
    //             << " Rotation: " << std::endl << obs_BVH.pR[0] << std::endl;
    // std::cout << " Trimmed off top level boxes" << std::endl;
    // Copy data to device
    // TIMEIT("Copying data to device memory",
        cudaDeviceSynchronize();
        checkCudaMem(cudaMemcpy(d_R_obs, R_obs.data(), num_confs * sizeof(Eigen::Matrix3f), cudaMemcpyHostToDevice));
        checkCudaMem(cudaMemcpy(d_T_obs, T_obs.data(), num_confs * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
        checkCudaMem(cudaMemcpy(d_R_rob, R_rob.data(), num_confs * sizeof(Eigen::Matrix3f), cudaMemcpyHostToDevice));
        checkCudaMem(cudaMemcpy(d_T_rob, T_rob.data(), num_confs * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
        checkCudaMem(cudaMemcpy(d_Rob_conf_rot, rob_rotations, num_confs * sizeof(Eigen::Matrix3f), cudaMemcpyHostToDevice));
        checkCudaMem(cudaMemcpy(d_Rob_conf_trans, rob_translations, num_confs * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
        checkCudaMem(cudaMemcpy(d_Rob_dim, rob_dim.data(), num_confs * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
        checkCudaMem(cudaMemcpy(d_Obs_dim, obs_dim.data(), num_confs * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
        checkCudaMem(cudaMemcpy(pdisjoint, disjoint, num_confs * sizeof(bool), cudaMemcpyHostToDevice));
        cudaDeviceSynchronize();
    // )

    // Launch kernel with correct grid size
    const int blockSize = 32;
    const int gridSize = (num_confs + blockSize - 1) / blockSize; // ceil division
    // const int gridSize = (num_confs + blockSize - 1) / blockSize; // ceil division


    cudaEventRecord(start);
    auto cpu_start = std::chrono::high_resolution_clock::now();
        d_obb_dyn_1box<<<gridSize, blockSize>>>(d_R_obs, d_T_obs, d_R_rob, d_T_rob, d_Obs_dim, d_Rob_dim, d_Rob_conf_rot, d_Rob_conf_trans, pdisjoint);
    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    float duration = 0.0f;

    cudaEventElapsedTime(&duration, start, stop);
        cudaDeviceSynchronize();
        checkCudaMem(cudaGetLastError());
        // Copy result back to host (num_confs * sizeof(bool))
        checkCudaMem(cudaMemcpy(disjoint, pdisjoint, num_confs * sizeof(bool), cudaMemcpyDeviceToHost));
    auto cpu_end = std::chrono::high_resolution_clock::now();
    double cpu_duration = std::chrono::duration<double, std::milli>(cpu_end - cpu_start).count();
    std::vector<ConfigurationTagged> cpuCollisions(num_confs);
    // TIMEIT("Running Collision check on CPU", checkConfsCPU(cpuCollisions, confs);)
    checkConfsCPU(cpuCollisions, confs, "/home/victor/Projects/robo-check/data/models/alpha1.0/robot.obj",
        "/home/victor/Projects/robo-check/data/models/alpha1.0/obstacle.obj");

    //TODO: make the checker a function inside Utils
    // // Check result
    // size_t true_positives = 0; // num disjoint that are valid
    // size_t false_positives = 0; // num disjoint that are not valid (should be 0)
    // size_t false_negatives = 0; // num not disjoint that are valid (likely to be high since this is broad phase)
    // size_t true_negatives = 0; // num not disjoint that are not valid (unsure how many of these will be)
    // if obb is disjoint, then we know for sure there is no collision
    // verify that the cpu said the same thing
    // for (int i = 0; i < num_confs; ++i) {
    //     if (disjoint[i]) {
    //         if (cpuCollisions[i].valid) {
    //             true_positives++;
    //         } else {
    //             false_positives++;
    //             std::cout << "False positive at configuration " << i << ": "
    //                       << "Position (" << confs[i].x << ", " << confs[i].y << ", " << confs[i].z << "), " <<
    //                       "Orientation (roll: " << confs[i].roll << ", pitch: " << confs[i].pitch << ", yaw: " << confs[i].yaw << ")" << std::endl;

    //             //apply transformation to blank obb
    //             // rob_BVH.set(0, rob_rotations[i], rob_translations[i], rob_BVH.pDim[0]);
    //             std::vector<Eigen::Vector3f> vertices = rob_BVH.getBoxVertices(0);
    //             Eigen::Matrix3f R = rob_rotations[i];
    //             Eigen::Vector3f T = rob_translations[i];

    //             // for (auto p : vertices){
    //             //     std::cout << p.transpose() << " --> ";
    //             //     std::cout << (R * p + T).transpose() << std::endl;
    //             // }
    //             std::cout << "Invalid Robot Transform:" << std::endl;
    //             std::cout << pythonifyEigenMatrix(createHomogeneousMatrix(confs[i])) << std::endl;

    //         }
    //     }
    //     else {
    //         if (!cpuCollisions[i].valid) {
    //             true_negatives++;
    //         } else {
    //             false_negatives++;
    //         }
    //     }

    // }
    // std::cout << "Out of " << num_confs << " configurations, " << true_positives << " were true positives and " << false_positives << " were false positives." << std::endl;
    // std::cout << "Out of " << num_confs << " configurations, " << true_negatives << " were true negatives and " << false_negatives << " were false negatives." << std::endl;

    // int cpu_positives = 0;
    // int cpu_negatives = 0;
    // for (const auto& c : cpuCollisions) {
    //     if (c.valid) {
    //         cpu_positives++;
    //     } else {
    //         cpu_negatives++;
    //     }
    // }
    // std::cout << "CPU Collision checker found " << cpu_positives << " valid collisions and " << cpu_negatives << " invalid collisions." << std::endl;

    // Free device memory
    cudaFree(d_R_obs);
    cudaFree(d_T_obs);
    cudaFree(d_R_rob);
    cudaFree(d_T_rob);
    cudaFree(d_Rob_dim);
    cudaFree(d_Obs_dim);
    cudaFree(d_Rob_conf_rot);
    cudaFree(d_Rob_conf_trans);
    cudaFree(pdisjoint);

    return duration;
}