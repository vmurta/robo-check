#include "Edge.hu"
#include "OBB-BVH-naive.hu"

constexpr int EDGE_BLOCK_SIZE = 32;

__global__ void d_bvh_edges(const RigidObstacleSoA obs, const RigidRobotSoA rob,
                            const EdgeSoA edges, size_t num_edges,
                            const EdgeKernelOut out) {

    if (obs.num_nodes == 0) {
        printf("Error: obs.num_nodes is zero. Exiting kernel.\n");
        return;
    }

    extern __shared__ char shared_mem[];

    size_t smem_offset = 0;
    int16_t* sObs_first_child = reinterpret_cast<int16_t*>(shared_mem + smem_offset);
    smem_offset += obs.num_nodes * sizeof(int16_t);
    int16_t* sRob_first_child = reinterpret_cast<int16_t*>(shared_mem + smem_offset);

    // Load the first_child arrays into shared memory once per block; they are
    // read-only for the lifetime of the kernel.
    for (uint16_t i = threadIdx.x; i < rob.num_nodes; i += blockDim.x){
        sRob_first_child[i] = rob.first_child[i];
    }
    for (uint16_t i = threadIdx.x; i < obs.num_nodes; i += blockDim.x){
        sObs_first_child[i] = obs.first_child[i];
    }
    __syncwarp(0xFFFFFFFFu);

    const float4 qo0 = obs.Rq[0];
    const Quat q_obs_root = {qo0.x, qo0.y, qo0.z, qo0.w}; // rotation of B wrt origin
    Eigen::Vector3f T_obs_abs_root = obs.T[0]; // translation of B wrt origin

    const float4 qr0 = rob.Rq[0];
    const Quat q_rob_root = {qr0.x, qr0.y, qr0.z, qr0.w}; // rotation of A wrt origin
    Eigen::Vector3f T_rob_abs_root = rob.T[0]; // translation of A wrt origin

    Eigen::Vector3f b_root = rob.dim[0]; // half dimensions of box A
    Eigen::Vector3f a_root = obs.dim[0]; // half dimensions of box B

    const float epsilon = 1e-6f; // small value to avoid numerical issues

    float4 q_conf; // rotation of robot wrt world (per sample)
    Eigen::Vector3f T_conf; // translation of robot wrt world (per sample)
    Quat q_obs_abs; // rotation of B wrt origin (per node)
    Eigen::Vector3f T_obs_abs;
    Quat q_rob_abs;
    Eigen::Vector3f T_rob_abs;
    Eigen::Vector3f b; // half dimensions of box A (per node)
    Eigen::Vector3f a; // half dimensions of box B (per node)
    Quat qB;                // rotation of A wrt B (quatSAT)
    Eigen::Vector3f T;      // translation of A wrt B

    // intent: for each i in rob_obb_pend, need to check all children of rob_obb_pend[i] against all children of obs_obb_pend[j]
    // maybe should do 32 * num layers, back of envelope says that should be an upper limit
    constexpr int MAX_BUFFER = EDGE_BLOCK_SIZE * 128;
    __shared__ uint16_t rob_obb_pend[MAX_BUFFER]; // arbitrary buffer size, should experiment with this
    __shared__ uint16_t obs_obb_pend[MAX_BUFFER];
    __shared__ int num_obb_pend;

    // Set (from any thread, all writers set true) when the paper triangle
    // test (or its fallback) finds an intersecting leaf pair for the current
    // sample; the block-serial traversal breaks early on it.
    __shared__ bool s_collision;

    //TODO: need failsafe if this overflows

    constexpr int BATCH = EDGE_BLOCK_SIZE; // edges pulled per queue transaction (one per thread)
    __shared__ uint32_t s_batch_start;
    __shared__ uint32_t s_num_pend;
    __shared__ float4 s_pend_s_quat[BATCH];
    __shared__ float4 s_pend_e_quat[BATCH];
    __shared__ Eigen::Vector3f s_pend_s_trans[BATCH];
    __shared__ Eigen::Vector3f s_pend_e_trans[BATCH];
    __shared__ Eigen::Vector3f s_pend_s_euler[BATCH];
    __shared__ Eigen::Vector3f s_pend_e_euler[BATCH];
    __shared__ uint32_t s_pend_nsteps[BATCH];
    __shared__ EdgeType s_pend_type[BATCH];
    __shared__ uint32_t s_pend_flags[BATCH];
    __shared__ uint32_t s_pend_idx[BATCH];

    // Bitpacked result word for this batch: one bit per edge. The batch is
    // always 32-aligned (BATCH == EDGE_BLOCK_SIZE), so an entire batch maps
    // to a single uint32 word owned exclusively by this block; thread 0
    // flushes it.
    __shared__ uint32_t s_disjoint_word;

    int16_t conf_offset = (threadIdx.x >> 4)-2; // divide by 16 to see if thread works on the 0th pair or 1st pair of pending boxes
    int16_t rob_child_idx = (threadIdx.x >> 2) & 0x3; // divide by 4, then mod by 4to see which child of the robot box this thread is assigned to
    int16_t obs_child_idx = threadIdx.x & 0x3; // mod 4 to see which child of the obstacle box this thread is assigned to

    while (true) {
        __syncwarp(0xFFFFFFFFu);
        uint32_t batch_start = 0;
        if (threadIdx.x == 0) {
            batch_start = atomicAdd(out.g_next_edge, BATCH);
            s_num_pend = 0;
            s_disjoint_word = 0;
        }
        batch_start = __shfl_sync(0xFFFFFFFFu, batch_start, 0);
        if (batch_start >= num_edges) {
            return;
        }

        // parallel outermost OBB check: one edge per thread. Every sample
        // t = j/nsteps is root-checked; the edge is pushed to the pending
        // list as soon as any sample overlaps (or is retired as valid when
        // all of them are root-disjoint).
        const uint32_t index = batch_start + threadIdx.x;
        if (index < num_edges) {
            const uint32_t nsteps = edges.nsteps[index];
            bool root_overlap = false;
            for (uint32_t j = 0; j <= nsteps; ++j) {
                const float t = (nsteps > 0) ? (float)j / (float)nsteps : 0.0f;
                interpolateEdgeState(edges.s_quat[index], edges.s_trans[index],
                                     edges.e_quat[index], edges.e_trans[index],
                                     edges.s_euler[index], edges.e_euler[index],
                                     edges.type[index], t, q_conf, T_conf);

                // Calculate the relative rotation of B wrt A purely in
                // quaternion space (quatSAT: no matrix materialized).
                const Quat qc = {q_conf.x, q_conf.y, q_conf.z, q_conf.w};
                computeRelTransformQuat(q_obs_root, T_obs_abs_root,
                                        q_rob_root, T_rob_abs_root,
                                        qc, T_conf, qB, T);

                // initial per sample outermost bounding box check
                if (obbOverlapQuat(qB, T, a_root, b_root, epsilon)) {
                    root_overlap = true;
                    break;
                }
            }

            if (!root_overlap) {
                atomicOr(&s_disjoint_word, 1u << threadIdx.x);
            } else {
                uint32_t pos = atomicAdd(&s_num_pend, 1);
                s_pend_s_quat[pos] = edges.s_quat[index];
                s_pend_e_quat[pos] = edges.e_quat[index];
                s_pend_s_trans[pos] = edges.s_trans[index];
                s_pend_e_trans[pos] = edges.e_trans[index];
                s_pend_s_euler[pos] = edges.s_euler[index];
                s_pend_e_euler[pos] = edges.e_euler[index];
                s_pend_nsteps[pos] = nsteps;
                s_pend_type[pos] = edges.type[index];
                s_pend_flags[pos] = edges.flags[index];
                s_pend_idx[pos] = index;
            }
        } // end outermost OBB check
        __syncwarp(0xFFFFFFFFu);

        // Flush the bitpacked results of the outermost check before any early
        // continue: bits are only ever set, so re-ORing below is idempotent.
        if (threadIdx.x == 0) {
            out.pdisjoint[batch_start >> 5] |= s_disjoint_word;
        }


        if (s_num_pend == 0) {
            continue;
        }

        for (uint32_t i = 0; i < s_num_pend; i++) {
            __syncwarp(0xFFFFFFFFu);
            const float4 s_quat = s_pend_s_quat[i];
            const float4 e_quat = s_pend_e_quat[i];
            const Eigen::Vector3f s_trans = s_pend_s_trans[i];
            const Eigen::Vector3f e_trans = s_pend_e_trans[i];
            const Eigen::Vector3f s_euler = s_pend_s_euler[i];
            const Eigen::Vector3f e_euler = s_pend_e_euler[i];
            const uint32_t nsteps = s_pend_nsteps[i];
            const EdgeType type = s_pend_type[i];
            const uint32_t flags = s_pend_flags[i];
            const uint32_t index = s_pend_idx[i];

            // Verdict accumulators for this edge (uniform across the block
            // after the __syncthreads that end each sample traversal).
            bool edge_collision = false;
            float first_invalid_t = -1.0f;

            // Validate every sample t = j/nsteps, j in [0, nsteps], breaking
            // out on the first colliding sample.
            for (uint32_t j = 0; j <= nsteps; ++j) {
                const float t = (nsteps > 0) ? (float)j / (float)nsteps : 0.0f;
                interpolateEdgeState(s_quat, s_trans,
                                     e_quat, e_trans,
                                     s_euler, e_euler,
                                     type, t, q_conf, T_conf);
                // The sample quaternion is constant for this sample:
                // convert once and reuse across every node-pair test.
                const Quat qc = {q_conf.x, q_conf.y, q_conf.z, q_conf.w};

                //reset pending OBB lists
                if (threadIdx.x == 0){
                    num_obb_pend = 1;
                    s_collision = false;
                    rob_obb_pend[0] = 0; // root
                    obs_obb_pend[0] = 0; // root
                }

                __syncwarp(0xFFFFFFFFu);

                // intent: for each i in rob_obb_pend, need to check all children of rob_obb_pend[i] against all children of obs_obb_pend[j]
                // these should be 4-ary trees, meaning we grab up to two pending boxes at a time so that we have 16 threads doing the children 
                // of each pair of boxes. 
                while(true){
                    __syncwarp(0xFFFFFFFFu);
                    if (s_collision) {
                        break;
                    }
                    if (num_obb_pend == 0){
                        break;
                    }

                    int pend_idx = num_obb_pend + conf_offset;
                    if (num_obb_pend >= MAX_BUFFER - 32){
                        if (threadIdx.x == 0) {
                            printf("obb overflow with %d\n boxes on edge with sample rotation quaternion \n\r \
                                    %f, %f, %f, %f, \n and translation \n \n with block index %d\n \
                                    %f, %f, %f \n", 
                                    num_obb_pend, q_conf.x, q_conf.y, q_conf.z, q_conf.w,
                                    T_conf[0], T_conf[1], T_conf[2], blockIdx.x);
                        }
                        break;
                    }

                    __syncwarp(0xFFFFFFFFu);

                    if(threadIdx.x == 0){
                        if (num_obb_pend > 1) {num_obb_pend -= 2;} 
                        else {num_obb_pend = 0;}
                    }

                    __syncwarp(0xFFFFFFFFu);
                    if (pend_idx < 0) {
                        continue;
                    }

                    unsigned mask = __activemask();
                    
                    int rob_obb_par_idx = rob_obb_pend[pend_idx];
                    int obs_obb_par_idx = obs_obb_pend[pend_idx];
                    
                    int rob_obb_idx = sRob_first_child[rob_obb_par_idx] + rob_child_idx;
                    int obs_obb_idx = sObs_first_child[obs_obb_par_idx] + obs_child_idx;

                    int rob_first_child_idx = sRob_first_child[rob_obb_idx];
                    int obs_first_child_idx = sObs_first_child[obs_obb_idx];

                    __syncwarp(mask);

                    // if either is a dummy node, skip check
                    //TODO: see if this is safe to do
                    if (rob_first_child_idx == 0 || obs_first_child_idx == 0){
                        continue;
                    }

                    const float4 qof = obs.Rq[obs_obb_idx];
                    q_obs_abs = {qof.x, qof.y, qof.z, qof.w};
                    T_obs_abs = obs.T[obs_obb_idx];
                    const float4 qrf = rob.Rq[rob_obb_idx];
                    q_rob_abs = {qrf.x, qrf.y, qrf.z, qrf.w};
                    T_rob_abs = rob.T[rob_obb_idx];
                    b = rob.dim[rob_obb_idx];
                    a = obs.dim[obs_obb_idx];

                    computeRelTransformQuat(q_obs_abs, T_obs_abs, q_rob_abs, T_rob_abs,
                                            qc, T_conf, qB, T);

                    if (!obbOverlapQuat(qB, T, a, b, epsilon)) {
                        continue;
                    }

                    if (rob_first_child_idx > 0 && obs_first_child_idx > 0) {
                        // add children to pending lists
                        int pos = atomicAdd(&num_obb_pend, 1);
                        obs_obb_pend[pos] = obs_obb_idx;
                        rob_obb_pend[pos] = rob_obb_idx;
                    } 

                    //TODO: when one box is a leaf and the other is not, the leaf's
                    // parent is pushed back into the pending list, so the same
                    // parent can be pushed multiple times for the same non-leaf
                    // box (duplicate work, but harmless for correctness).
                    else if (rob_first_child_idx < 0) {
                        // both are leaves
                        if (obs_first_child_idx < 0) {
                            // Chang & Kim triangle test in rectangle-local
                            // coordinates, reusing this thread's B/T from the
                            // OBB overlap test above.
                            const int rob_tri = -(rob_first_child_idx + 1);
                            const int obs_tri = -(obs_first_child_idx + 1);
                            const Eigen::Vector3f& dimObs = obs.dim[obs_obb_idx];
                            const Eigen::Vector3f& dimRob = rob.dim[rob_obb_idx];
                            const Eigen::Matrix3f B = quatToMatrix(qB); // narrow phase still uses the matrix form
                            const int verdict = paperTriTri(dimObs(0), dimObs(1), obs.a[obs_obb_idx],
                                                            dimRob(0), dimRob(1), rob.a[rob_obb_idx],
                                                            B, T);
                            if (verdict > 0) {
                                s_collision = true;
                            } else if (verdict < 0) {
                                // borderline/coplanar: full world-frame test
                                const Triangle& rt = rob.tris[rob_tri];
                                const Triangle& ot = obs.tris[obs_tri];
                                const Eigen::Vector3f rv0 = quatRotateVec(qc, rob.verts[rt.v1]) + T_conf;
                                const Eigen::Vector3f rv1 = quatRotateVec(qc, rob.verts[rt.v2]) + T_conf;
                                const Eigen::Vector3f rv2 = quatRotateVec(qc, rob.verts[rt.v3]) + T_conf;
                                if (!triangles_valid_f(rv0, rv1, rv2,
                                                       obs.verts[ot.v1], obs.verts[ot.v2], obs.verts[ot.v3])) {
                                    s_collision = true;
                                }
                            }
                        }
                        // robot is leaf, obstacle is not
                        else {
                            int pos = atomicAdd(&num_obb_pend, 1);
                            obs_obb_pend[pos] = obs_obb_idx;
                            rob_obb_pend[pos] = rob_obb_par_idx; 
                        }
                    } 
                    // obstacle is leaf, robot is not
                    else if (obs_first_child_idx < 0) {

                        int pos = atomicAdd(&num_obb_pend, 1);
                        obs_obb_pend[pos] = obs_obb_par_idx; // just shove the parent back in, recurse only on robot children
                        rob_obb_pend[pos] = rob_obb_idx;
                    }
                } // end while true over single sample

                __syncwarp(0xFFFFFFFFu);
                if (s_collision) {
                    edge_collision = true;
                    first_invalid_t = t;
                    break;
                }
            } // end for over samples of one edge

            __syncwarp(0xFFFFFFFFu);
            if (edge_collision) {
                if (flags & EDGE_FLAG_REPORT_FIRST_INVALID_T) {
                    if (threadIdx.x == 0) {
                        out.pfirst_invalid_t[index] = first_invalid_t;
                    }
                }
            } else {
                if (threadIdx.x == 0) {
                    s_disjoint_word |= 1u << (index & 31);
                }
            }
            __syncwarp(0xFFFFFFFFu);
        } // end for over pending edges in batch

        // Flush the batch's bitpacked word once (one 4-byte write instead of
        // 32 threads storing the same byte).
        if (threadIdx.x == 0) {
            out.pdisjoint[batch_start >> 5] |= s_disjoint_word;
        }
    }
    return;
}

double bvh_edges(const BVNode_soa<>& rob_BVH, const BVNode_soa<>& obs_BVH,
                 const MeshData& rob_mesh, const MeshData& obs_mesh,
                 const EdgeBatch& edges,
                 std::vector<bool>& valid, std::vector<float>& first_invalid_t, bool dry_run) {
    cudaEvent_t start, stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);

    std::cout << "Rob BVH size: " << rob_BVH.size << std::endl;
    std::cout << "Obs BVH size: " << obs_BVH.size << std::endl;

    const int num_edges = (int)edges.size;
    const size_t num_words = (num_edges + 31) / 32;
    valid.assign(num_edges, false);
    first_invalid_t.assign(num_edges, -1.0f);
    if (num_edges == 0) {
        return 0.0;
    }

    // Bitpacked results: bit i of disjoint[i >> 5] is the result of edge i.
    std::unique_ptr<uint32_t[]> disjoint(new uint32_t[num_words]());

    // Rotations are stored as unit quaternions (x, y, z, w) in float4
    // (see bvh_naive).
    std::vector<float4> obs_q(obs_BVH.size);
    std::vector<float4> rob_q(rob_BVH.size);
    for (size_t i = 0; i < obs_BVH.size; ++i) {
        const Eigen::Quaternionf q(obs_BVH.pR[i]);
        obs_q[i] = make_float4(q.x(), q.y(), q.z(), q.w());
    }
    for (size_t i = 0; i < rob_BVH.size; ++i) {
        const Eigen::Quaternionf q(rob_BVH.pR[i]);
        rob_q[i] = make_float4(q.x(), q.y(), q.z(), q.w());
    }

    const int blockSize = 32;
    // Persistent blocks pulling edges from a global work queue. Size the grid
    // so every SM is fully occupied (the kernel fits multiple blocks per SM).
    int device;
    cudaGetDevice(&device);
    int num_sms = 0;
    cudaDeviceGetAttribute(&num_sms, cudaDevAttrMultiProcessorCount, device);

    // Dynamic shared memory: just the two first_child arrays (read-only work set).
    const size_t smem_size = (obs_BVH.size + rob_BVH.size) * sizeof(int16_t);

    int blocks_per_sm = 0;
    cudaOccupancyMaxActiveBlocksPerMultiprocessor(&blocks_per_sm, d_bvh_edges, blockSize, smem_size);
    if (blocks_per_sm < 1) blocks_per_sm = 1;
    const int max_blocks = (num_edges + blockSize - 1) / blockSize;
    const int persistent_blocks = num_sms * blocks_per_sm;
    const int gridSize = (max_blocks < persistent_blocks) ? max_blocks : persistent_blocks;


    float4* d_Q_obs;
    Eigen::Vector3f* d_T_obs;
    float4* d_Q_rob;
    Eigen::Vector3f* d_T_rob;
    Eigen::Vector3f* d_Rob_dim;
    Eigen::Vector3f* d_Obs_dim;
    float4* d_Edge_s_quat;
    float4* d_Edge_e_quat;
    Eigen::Vector3f* d_Edge_s_trans;
    Eigen::Vector3f* d_Edge_e_trans;
    Eigen::Vector3f* d_Edge_s_euler;
    Eigen::Vector3f* d_Edge_e_euler;
    uint32_t* d_Edge_nsteps;
    EdgeType* d_Edge_type;
    uint32_t* d_Edge_flags;
    Eigen::Vector3f* d_Rob_vertices;
    Eigen::Vector3f* d_Obs_vertices;
    Triangle * d_Rob_triangles;
    Triangle * d_Obs_triangles;
    int16_t* d_Obs_first_child;
    int16_t* d_Rob_first_child;
    float* d_Obs_a;
    float* d_Rob_a;
    uint32_t* pdisjoint;
    float* d_first_invalid_t;
    uint32_t* d_next_edge;

    // Allocate memory for device pointers
    cudaEventRecord(start, 0);
    cudaMalloc((void**)&d_Q_obs, obs_BVH.size * sizeof(float4));
    cudaMalloc((void**)&d_T_obs, obs_BVH.size * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Q_rob, rob_BVH.size * sizeof(float4));
    cudaMalloc((void**)&d_T_rob, rob_BVH.size * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Rob_dim, rob_BVH.size * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Obs_dim, obs_BVH.size * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Edge_s_quat, num_edges * sizeof(float4));
    cudaMalloc((void**)&d_Edge_e_quat, num_edges * sizeof(float4));
    cudaMalloc((void**)&d_Edge_s_trans, num_edges * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Edge_e_trans, num_edges * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Edge_s_euler, num_edges * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Edge_e_euler, num_edges * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Edge_nsteps, num_edges * sizeof(uint32_t));
    cudaMalloc((void**)&d_Edge_type, num_edges * sizeof(EdgeType));
    cudaMalloc((void**)&d_Edge_flags, num_edges * sizeof(uint32_t));
    cudaMalloc((void**)&d_Obs_first_child, obs_BVH.size * sizeof(int16_t));
    cudaMalloc((void**)&d_Rob_first_child, rob_BVH.size * sizeof(int16_t));
    cudaMalloc((void**)&d_Obs_a, obs_BVH.size * sizeof(float));
    cudaMalloc((void**)&d_Rob_a, rob_BVH.size * sizeof(float));
    cudaMalloc((void**)&d_Rob_vertices, rob_mesh.vertices.size() * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Obs_vertices, obs_mesh.vertices.size() * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Rob_triangles, rob_mesh.triangles.size() * sizeof(Triangle));
    cudaMalloc((void**)&d_Obs_triangles, obs_mesh.triangles.size() * sizeof(Triangle));
    cudaMalloc((void**)&pdisjoint, num_words * sizeof(uint32_t));
    cudaMalloc((void**)&d_first_invalid_t, num_edges * sizeof(float));
    cudaMalloc((void**)&d_next_edge, sizeof(uint32_t));
    checkCudaMem(cudaMemset(d_next_edge, 0, sizeof(uint32_t)));
    
    cudaDeviceSynchronize();
    checkCudaMem(cudaMemcpy(d_Q_obs, obs_q.data(), obs_BVH.size * sizeof(float4), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_T_obs, obs_BVH.pT, obs_BVH.size * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Q_rob, rob_q.data(), rob_BVH.size * sizeof(float4), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_T_rob, rob_BVH.pT, rob_BVH.size * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_dim, rob_BVH.pDim, rob_BVH.size * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_dim, obs_BVH.pDim, obs_BVH.size * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_first_child, rob_BVH.first_child, rob_BVH.size * sizeof(int16_t), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_first_child, obs_BVH.first_child, obs_BVH.size * sizeof(int16_t), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_a, obs_BVH.pA, obs_BVH.size * sizeof(float), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_a, rob_BVH.pA, rob_BVH.size * sizeof(float), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_vertices, rob_mesh.vertices.data(), rob_mesh.vertices.size() * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_vertices, obs_mesh.vertices.data(), obs_mesh.vertices.size() * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_triangles, rob_mesh.triangles.data(), rob_mesh.triangles.size() * sizeof(Triangle), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_triangles, obs_mesh.triangles.data(), obs_mesh.triangles.size() * sizeof(Triangle), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemset(pdisjoint, 0, num_words * sizeof(uint32_t)));
    cudaDeviceSynchronize();
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    float duration = 0;
    cudaEventElapsedTime(&duration, start, stop);
    std::cout << "Initial allocation and transfer to GPU took " << duration << " ms." << std::endl;

    // Copy edges over separately
    cudaEventRecord(start, 0);
    checkCudaMem(cudaMemcpy(d_Edge_s_quat, edges.s_quat, num_edges * sizeof(float4), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Edge_e_quat, edges.e_quat, num_edges * sizeof(float4), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Edge_s_trans, edges.s_trans, num_edges * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Edge_e_trans, edges.e_trans, num_edges * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Edge_s_euler, edges.s_euler, num_edges * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Edge_e_euler, edges.e_euler, num_edges * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Edge_nsteps, edges.nsteps, num_edges * sizeof(uint32_t), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Edge_type, edges.type, num_edges * sizeof(EdgeType), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Edge_flags, edges.flags, num_edges * sizeof(uint32_t), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_first_invalid_t, first_invalid_t.data(), num_edges * sizeof(float), cudaMemcpyHostToDevice));
    cudaDeviceSynchronize();
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&duration, start, stop);
    std::cout << "Copying edges to GPU took " << duration << " ms." << std::endl;

    std::cout << "obs_BVH.size: " << obs_BVH.size << ", rob_BVH.size: " << rob_BVH.size << std::endl;

    RigidObstacleSoA obsSoA;
    obsSoA.R = nullptr; obsSoA.Rq = d_Q_obs; obsSoA.T = d_T_obs; obsSoA.dim = d_Obs_dim;
    obsSoA.first_child = d_Obs_first_child; obsSoA.a = d_Obs_a;
    obsSoA.verts = d_Obs_vertices; obsSoA.tris = d_Obs_triangles;
    obsSoA.num_nodes = obs_BVH.size;
    RigidRobotSoA robSoA;
    robSoA.R = nullptr; robSoA.Rq = d_Q_rob; robSoA.T = d_T_rob; robSoA.dim = d_Rob_dim;
    robSoA.first_child = d_Rob_first_child; robSoA.a = d_Rob_a;
    robSoA.verts = d_Rob_vertices; robSoA.tris = d_Rob_triangles;
    robSoA.num_nodes = rob_BVH.size;
    robSoA.conf_rot = nullptr; robSoA.conf_trans = nullptr;
    EdgeSoA edgeSoA;
    edgeSoA.s_quat = d_Edge_s_quat; edgeSoA.e_quat = d_Edge_e_quat;
    edgeSoA.s_trans = d_Edge_s_trans; edgeSoA.e_trans = d_Edge_e_trans;
    edgeSoA.s_euler = d_Edge_s_euler; edgeSoA.e_euler = d_Edge_e_euler;
    edgeSoA.nsteps = d_Edge_nsteps; edgeSoA.type = d_Edge_type; edgeSoA.flags = d_Edge_flags;
    EdgeKernelOut outSoA;
    outSoA.pdisjoint = pdisjoint; outSoA.pfirst_invalid_t = d_first_invalid_t;
    outSoA.g_next_edge = d_next_edge;
    outSoA.overflowCounter = nullptr; // rigid kernels report overflows via printf

    auto launch_bvh_edges = [&]() {
        d_bvh_edges<<<gridSize, blockSize, smem_size>>>(
            obsSoA, robSoA, edgeSoA, static_cast<size_t>(num_edges), outSoA);
    };

    // Dynamic shared memory above 48KB requires opting in once per kernel.
    // The opt-in limit is the total (static + dynamic) shared memory per
    // block, so the static usage of this kernel must be subtracted.
    if (smem_size > 48 * 1024) {
        cudaFuncAttributes attr;
        checkCudaMem(cudaFuncGetAttributes(&attr, d_bvh_edges));
        int optin = 0;
        checkCudaMem(cudaDeviceGetAttribute(&optin, cudaDevAttrMaxSharedMemoryPerBlockOptin, 0));
        const int maxDyn = optin - (int)attr.sharedSizeBytes;
        if (smem_size > (size_t)maxDyn) {
            std::cerr << "d_bvh_edges: need " << smem_size << " B dynamic shared memory, but "
                      << maxDyn << " B is available after static usage ("
                      << attr.sharedSizeBytes << " B)." << std::endl;
            exit(1);
        }
        checkCudaMem(cudaFuncSetAttribute(d_bvh_edges, cudaFuncAttributeMaxDynamicSharedMemorySize, maxDyn));
        std::cout << "Opted d_bvh_edges into " << maxDyn << " B dynamic shared memory (needs "
                  << smem_size << " B)." << std::endl;
    }

    // Dry run: single-block launch over one batch, untimed, purely to get the
    // kernel loaded onto the device (the work queue would otherwise drain the
    // entire workload in this block).
    if (dry_run) {
        const size_t dry_edges = (num_edges < (size_t)blockSize) ? num_edges : (size_t)blockSize;
        d_bvh_edges<<<1, blockSize, smem_size>>>(
            obsSoA, robSoA, edgeSoA, dry_edges, outSoA);
        checkCudaMem(cudaGetLastError());
        checkCudaMem(cudaDeviceSynchronize());
        std::cout << "BVH Edges dry run completed successfully." << std::endl;
        // Reset the work queue so the timed launch starts from edge 0
        checkCudaMem(cudaMemset(d_next_edge, 0, sizeof(uint32_t)));
                checkCudaMem(cudaMemset(pdisjoint, 0, num_words * sizeof(uint32_t)));
        checkCudaMem(cudaMemcpy(d_first_invalid_t, first_invalid_t.data(), num_edges * sizeof(float), cudaMemcpyHostToDevice));
    }

    cudaEventRecord(start, 0);
    launch_bvh_edges();
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&duration, start, stop);
    std::cout << "BVH Edges GPU kernel took " << duration << " ms for " << num_edges << " edges." << std::endl;

    // Profiling: phase totals are summed across blocks (each block reports
    // once at termination); report the per-block average, which is a
    // block-serial view and NOT comparable to the wall-clock kernel time.
    // Copy result back to host
    cudaEventRecord(start, 0);
    checkCudaMem(cudaGetLastError());

    checkCudaMem(cudaMemcpy(disjoint.get(), pdisjoint, num_words * sizeof(uint32_t), cudaMemcpyDeviceToHost));
    checkCudaMem(cudaMemcpy(first_invalid_t.data(), d_first_invalid_t, num_edges * sizeof(float), cudaMemcpyDeviceToHost));
    cudaDeviceSynchronize();
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&duration, start, stop);
    std::cout << "Copying results from GPU took " << duration << " ms." << std::endl;

    for (int i = 0; i < num_edges; ++i) {
        valid[i] = (disjoint[i >> 5] >> (i & 31)) & 1u;
    }

    // Free device memory
    cudaFree(d_Q_obs);
    cudaFree(d_T_obs);
    cudaFree(d_Q_rob);
    cudaFree(d_T_rob);
    cudaFree(d_Rob_dim);
    cudaFree(d_Obs_dim);
    cudaFree(d_Edge_s_quat);
    cudaFree(d_Edge_e_quat);
    cudaFree(d_Edge_s_trans);
    cudaFree(d_Edge_e_trans);
    cudaFree(d_Edge_s_euler);
    cudaFree(d_Edge_e_euler);
    cudaFree(d_Edge_nsteps);
    cudaFree(d_Edge_type);
    cudaFree(d_Edge_flags);
    cudaFree(d_Obs_first_child);
    cudaFree(d_Rob_first_child);
    cudaFree(d_Obs_a);
    cudaFree(d_Rob_a);
    cudaFree(d_Rob_vertices);
    cudaFree(d_Obs_vertices);
    cudaFree(d_Rob_triangles);
    cudaFree(d_Obs_triangles);
    cudaFree(pdisjoint);
    cudaFree(d_first_invalid_t);
    cudaFree(d_next_edge);

    return duration;
}

double checkEdgesCPU(const std::vector<Edge>& edges,
                     std::vector<bool>& valid, std::vector<float>& first_invalid_t,
                     const std::string& robot_filename, const std::string& obstacle_filename) {
    std::vector<fcl::Vector3f> rob_vertices;
    std::vector<fcl::Triangle> rob_triangles;
    std::vector<fcl::Vector3f> obs_vertices;
    std::vector<fcl::Triangle> obs_triangles;

    loadOBJFileFCL(robot_filename, rob_vertices, rob_triangles);
    loadOBJFileFCL(obstacle_filename, obs_vertices, obs_triangles);

    std::shared_ptr<fcl::BVHModel<fcl::OBBRSSf>> rob_mesh(new fcl::BVHModel<fcl::OBBRSSf>);
    rob_mesh->beginModel(rob_triangles.size(), rob_vertices.size());
    rob_mesh->addSubModel(rob_vertices, rob_triangles);
    rob_mesh->endModel();

    std::shared_ptr<fcl::BVHModel<fcl::OBBRSSf>> obs_mesh(new fcl::BVHModel<fcl::OBBRSSf>);
    obs_mesh->beginModel(obs_triangles.size(), obs_vertices.size());
    obs_mesh->addSubModel(obs_vertices, obs_triangles);
    obs_mesh->endModel();

    fcl::CollisionObjectf rob_col_obj(rob_mesh);
    fcl::CollisionObjectf obs_col_obj(obs_mesh);

    const size_t num_edges = edges.size();
    valid.assign(num_edges, false);
    first_invalid_t.assign(num_edges, -1.0f);

    auto cpu_start_time = std::chrono::high_resolution_clock::now();

    size_t checked_samples = 0;
    for (size_t i = 0; i < num_edges; ++i) {
        const Edge& e = edges[i];
        if (e.type != EDGE_TYPE_LINEAR_6D) {
            std::cerr << "checkEdgesCPU: only EDGE_TYPE_LINEAR_6D edges are supported for CPU ground truth"
                      << std::endl;
            exit(1);
        }

        Configuration s, en;
        s.x = e.s_trans(0); s.y = e.s_trans(1); s.z = e.s_trans(2);
        s.pitch = e.s_euler(0); s.yaw = e.s_euler(1); s.roll = e.s_euler(2);
        en.x = e.e_trans(0); en.y = e.e_trans(1); en.z = e.e_trans(2);
        en.pitch = e.e_euler(0); en.yaw = e.e_euler(1); en.roll = e.e_euler(2);

        valid[i] = true;
        const uint32_t n = e.nsteps;
        for (uint32_t j = 0; j <= n; ++j) {
            const float t = (n > 0) ? (float)j / (float)n : 0.0f;
            Configuration c;
            c.x = s.x + (en.x - s.x) * t;
            c.y = s.y + (en.y - s.y) * t;
            c.z = s.z + (en.z - s.z) * t;
            c.pitch = s.pitch + (en.pitch - s.pitch) * t;
            c.yaw = s.yaw + (en.yaw - s.yaw) * t;
            c.roll = s.roll + (en.roll - s.roll) * t;

            fcl::Transform3f transform = configurationToTransform(c);
            rob_col_obj.setTransform(transform);

            fcl::CollisionRequestf request(1);
            fcl::CollisionResultf result;
            fcl::collide(&obs_col_obj, &rob_col_obj, request, result);
            ++checked_samples;

            if (result.isCollision()) {
                valid[i] = false;
                first_invalid_t[i] = t;
                break;
            }
        }
    }

    auto cpu_end_time = std::chrono::high_resolution_clock::now();
    const double cpu_duration = std::chrono::duration<double, std::milli>(cpu_end_time - cpu_start_time).count();
    std::cout << "cpu edge validation execution time: " << cpu_duration << " ms for " << num_edges
              << " edges (" << checked_samples << " samples checked, early-out)." << std::endl;
    return cpu_duration;
}

// ===========================================================================
// d_bvh_edges_collab: collaborative edge validation.
//
// Design (one 32-thread block == one warp, BATCH = 32 edges):
//
//   Phase A - pooled root checks. Round r: lane l root-checks sample
//   (edge l, j = r) of its edge. All lanes do identical work per round, so
//   there is no divergence until edges run out of samples (MIN_EDGE_NSTEPS
//   guarantees the first 8 rounds are fully populated). Samples whose root
//   boxes overlap are pushed onto a shared candidate ring as a packed
//   (uint8 local edge id, uint24 j). Round r+1 only starts when the ring has
//   32 free slots (backpressure: no candidate is ever dropped). Dead edges
//   skip their remaining rounds entirely.
//
//   Phase B - candidate drain. The warp splits into two 16-lane engines
//   (the same split d_bvh_naive uses for its two pend pairs), each owning
//   its own pend buffers. Engine g pops (edge, j) from the candidate ring
//   and runs the exact naive traversal body for that sample, one pend-pair
//   expansion per lockstep iteration. Engines advance in lockstep (one
//   shared loop; no divergent subgroup loops), so there is no engine-level
//   divergence - an idle engine simply does no work that iteration.
//
//   Early exit: s_edge_dead is a sticky bitmask. A leaf-leaf collision (or a
//   conservative pend-buffer overflow) sets the bit via atomicOr and records
//   t via atomicMin on the float bits (t >= 0, so IEEE ordering == integer
//   ordering). Dead edges are skipped at ring pop and phase A stops feeding
//   them; in-flight traversals always run to completion, so the recorded
//   first_invalid_t is exactly the earliest colliding sample (the ring is
//   FIFO by round, so any skipped sample has a larger t than every checked
//   one).
//
//   Phases alternate in spill passes (bounded ring): phase A fills, phase B
//   drains, repeat until every round has been checked. An edge with all its
//   samples root-disjoint never pays a single traversal step.
//
// Shared memory: 2x1024-entry per-engine pend queues, 512-entry candidate
// ring, edge metadata + endpoints, plus the dynamic first_child caches.
//
// Synchronization note: the block is exactly one warp (32 threads), so every
// barrier in this kernel is __syncwarp(0xFFFFFFFFu) instead of __syncthreads.
// This is only legal because every barrier site sits in warp-converged
// control flow (divergent regions contain no barriers), and it avoids the
// SM's barrier unit entirely - which matters once several blocks share an SM.
// If this kernel is ever launched with a different block size, these must go
// back to __syncthreads.
// ===========================================================================
namespace {

constexpr int EDGE_COLLAB_BLOCK = 32;
constexpr int MAX_BUF_G = 1024;     // pend pairs per engine (halved: 4 blocks/SM)
constexpr int CAND_CAP = 512;       // candidate ring entries (power of two)
constexpr uint32_t NO_ITEM = 0xFFFFFFFFu;

// ---------------------------------------------------------------------------
// Out-of-line rare paths: kept out of the hot drain loop so their code (the
// double-precision fallback triangle tests, printf plumbing) doesn't bloat
// the loop body, its branch-management overhead, or its register pressure.

// Leaf-leaf borderline/coplanar case: full world-frame triangle test.
// On collision: mark the edge dead and record the earliest invalid t.
__device__ __noinline__ void edgeLeafFallback(const Quat& q_c, const Eigen::Vector3f& T_c,
                                              int rob_tri, int obs_tri,
                                              const Eigen::Vector3f* __restrict__ rob_verts,
                                              const Triangle* __restrict__ rob_tris,
                                              const Eigen::Vector3f* __restrict__ obs_verts,
                                              const Triangle* __restrict__ obs_tris,
                                              uint32_t e, uint32_t j, uint32_t nsteps,
                                              uint32_t* s_edge_dead, float* s_edge_first_t,
                                              bool* s_eng_collision) {
    const Triangle& rt = rob_tris[rob_tri];
    const Triangle& ot = obs_tris[obs_tri];
    const Eigen::Vector3f rv0 = quatRotateVec(q_c, rob_verts[rt.v1]) + T_c;
    const Eigen::Vector3f rv1 = quatRotateVec(q_c, rob_verts[rt.v2]) + T_c;
    const Eigen::Vector3f rv2 = quatRotateVec(q_c, rob_verts[rt.v3]) + T_c;
    if (!triangles_valid_f(rv0, rv1, rv2,
                           obs_verts[ot.v1], obs_verts[ot.v2], obs_verts[ot.v3])) {
        const float t = (nsteps > 0) ? (float)j / (float)nsteps : 0.0f;
        atomicOr(s_edge_dead, 1u << e);
        atomicMin((unsigned int*)(s_edge_first_t + e), __float_as_uint(t));
        *s_eng_collision = true;
    }
}

// Conservative pend-overflow report (printf plumbing out of the hot loop).
__device__ __noinline__ void edgeOverflowReport(int num_pend, int edge_idx, uint32_t j, uint32_t block_idx) {
    printf("obb overflow with %d boxes on edge %d sample %u (block %d): treating as collision\n",
           num_pend, edge_idx, j, block_idx);
}

} // namespace

__global__ void d_bvh_edges_collab(const RigidObstacleSoA obs, const RigidRobotSoA rob,
                                   const EdgeSoA edges, size_t num_edges,
                                   const EdgeKernelOut out) {

    if (obs.num_nodes == 0) {
        printf("Error: obs.num_nodes is zero. Exiting kernel.\n");
        return;
    }

    extern __shared__ char shared_mem[];

    size_t smem_offset = 0;
    int16_t* sObs_first_child = reinterpret_cast<int16_t*>(shared_mem + smem_offset);
    smem_offset += obs.num_nodes * sizeof(int16_t);
    int16_t* sRob_first_child = reinterpret_cast<int16_t*>(shared_mem + smem_offset);

    // Load the first_child arrays into shared memory once per block; they are
    // read-only for the lifetime of the kernel.
    for (uint16_t i = threadIdx.x; i < rob.num_nodes; i += blockDim.x){
        sRob_first_child[i] = rob.first_child[i];
    }
    for (uint16_t i = threadIdx.x; i < obs.num_nodes; i += blockDim.x){
        sObs_first_child[i] = obs.first_child[i];
    }
    __syncwarp(0xFFFFFFFFu);

    const float4 qo0 = obs.Rq[0];
    const Quat q_obs_root = {qo0.x, qo0.y, qo0.z, qo0.w}; // rotation of B wrt origin
    Eigen::Vector3f T_obs_abs_root = obs.T[0]; // translation of B wrt origin

    const float4 qr0 = rob.Rq[0];
    const Quat q_rob_root = {qr0.x, qr0.y, qr0.z, qr0.w}; // rotation of A wrt origin
    Eigen::Vector3f T_rob_abs_root = rob.T[0]; // translation of A wrt origin

    Eigen::Vector3f b_root = rob.dim[0]; // half dimensions of box A
    Eigen::Vector3f a_root = obs.dim[0]; // half dimensions of box B

    const float epsilon = 1e-6f; // small value to avoid numerical issues

    // Per-lane working registers, reused across phases (never assumed to
    // hold a value across a __syncthreads).
    float4 q_conf;              // sample rotation wrt world (phase A)
    Eigen::Vector3f T_conf;     // sample translation wrt world (phase A)
    Quat q_obs_abs;           // obstacle node rotation (work step, per pair)
    Eigen::Vector3f T_obs_abs;  // obstacle node translation
    Quat q_rob_abs;           // robot node rotation
    Eigen::Vector3f T_rob_abs;  // robot node translation
    Eigen::Vector3f b;          // half dims of the robot box (per pair)
    Eigen::Vector3f a;          // half dims of the obstacle box (per pair)
    Quat qB;                    // robot orientation wrt obstacle (quatSAT)
    Eigen::Vector3f T;          // robot position wrt obstacle (SAT input)

    // ---- batch/edge metadata, indexed by LOCAL edge id (== lane) ----
    __shared__ uint32_t s_edge_idx[EDGE_COLLAB_BLOCK];     // global edge index of each local edge
    __shared__ uint32_t s_edge_nsteps[EDGE_COLLAB_BLOCK];  // nsteps: sample j runs 0..nsteps, t = j/nsteps
    __shared__ EdgeType s_edge_type[EDGE_COLLAB_BLOCK];    // LINEAR_6D or SE3_SLERP (interpolation scheme)
    __shared__ uint32_t s_edge_flags[EDGE_COLLAB_BLOCK];   // e.g. REPORT_FIRST_INVALID_T
    __shared__ float4 s_s_quat[EDGE_COLLAB_BLOCK];         // start rotation (unit quat x,y,z,w)
    __shared__ float4 s_e_quat[EDGE_COLLAB_BLOCK];         // end rotation
    __shared__ Eigen::Vector3f s_s_trans[EDGE_COLLAB_BLOCK]; // start position
    __shared__ Eigen::Vector3f s_e_trans[EDGE_COLLAB_BLOCK]; // end position
    __shared__ Eigen::Vector3f s_s_euler[EDGE_COLLAB_BLOCK]; // start (pitch, yaw, roll) for LINEAR_6D
    __shared__ Eigen::Vector3f s_e_euler[EDGE_COLLAB_BLOCK]; // end   (pitch, yaw, roll)

    // Sticky per-edge verdicts: bit l of s_edge_dead marks local edge l as
    // invalid; s_edge_first_t[l] holds the earliest colliding sample's t.
    __shared__ uint32_t s_edge_dead;                       // 32-bit death mask (one bit per local edge)
    __shared__ float s_edge_first_t[EDGE_COLLAB_BLOCK];    // earliest invalid t per edge (init FLT_MAX)

    // Candidate ring: root-overlapping samples waiting for traversal, packed
    // as (local edge id << 24) | j. head/tail are MONOTONIC counters (pop /
    // push counts, reset each batch); the physical slot is the counter
    // masked with CAND_CAP-1. Occupancy = tail - head, bounded by phase A's
    // backpressure check, so no live entry is ever overwritten.
    __shared__ uint32_t s_trav[CAND_CAP];  // ring storage
    __shared__ uint32_t s_trav_head;       // next item to pop (count of pops so far)
    __shared__ uint32_t s_trav_tail;       // next free slot (count of pushes so far)

    // Per-engine traversal state (engine g = threadIdx.x >> 4).
    __shared__ uint32_t s_eng_edge[2];     // local edge id the engine is traversing (NO_ITEM = idle)
    __shared__ uint32_t s_eng_j[2];        // sample index j of that item
    __shared__ int s_eng_pend_idx[2][2];   // [g][0/1] = pairs to expand THIS iteration (-1 = none)
    __shared__ int s_num_obb_pend[2];      // depth of the engine's pair stack (work left for this sample)
    __shared__ bool s_eng_collision[2];    // collision found for the current sample (item will be dropped)
    __shared__ uint16_t s_rob_pend[2][MAX_BUF_G]; // engine's stack of (robot, obstacle) node pairs
    __shared__ uint16_t s_obs_pend[2][MAX_BUF_G];
    // Current item's sample state (interpolated once per item by the
    // manager, read by the 16 work-step lanes every iteration).
    __shared__ float4 s_eng_q[2];          // sample rotation quat
    __shared__ Eigen::Vector3f s_eng_t[2]; // sample translation

    __shared__ uint32_t s_batch_start;     // first global edge index of this batch
    __shared__ uint32_t s_num_valid;       // edges in this batch (<= 32; tail batch may be short)

    // Persistent-block work queue: each block pulls a batch of edge indices
    // from a global atomic counter until the queue is exhausted. Phase
    // timers mirror d_bvh_naive (sums across blocks, see the notes there).
    while (true) {
        // ---- pull the next batch of 32 edge indices ----
        __syncwarp(0xFFFFFFFFu);
        uint32_t batch_start = 0;
        if (threadIdx.x == 0) {
            batch_start = atomicAdd(out.g_next_edge, EDGE_COLLAB_BLOCK); // claim the next 32 edges
        }
        batch_start = __shfl_sync(0xFFFFFFFFu, batch_start, 0);
        if (batch_start >= num_edges) {
            return;
        }

        // ---- reset per-batch state (thread 0, then broadcast) ----
        if (threadIdx.x == 0) {
            const size_t rem = num_edges - batch_start;
            s_num_valid = (uint32_t)((rem < EDGE_COLLAB_BLOCK) ? rem : EDGE_COLLAB_BLOCK); // short tail batch?
            s_edge_dead = 0;               // no edges dead yet
            s_trav_head = 0;               // candidate ring empty
            s_trav_tail = 0;
            s_eng_edge[0] = NO_ITEM;       // both engines idle
            s_eng_edge[1] = NO_ITEM;
            s_num_obb_pend[0] = 0;         // no traversal state yet
            s_num_obb_pend[1] = 0;
            s_eng_collision[0] = false;
            s_eng_collision[1] = false;
        }
        __syncwarp(0xFFFFFFFFu);

        // Load the shared edge buffer: lane l (the local edge id) copies its
        // edge's metadata + endpoints from global memory so every phase can
        // read them from shared. Lanes past num_valid leave their slots
        // uninitialized; they are gated out by the `l < num_valid` checks.
        const uint32_t l = threadIdx.x;            // local edge id for this lane
        const uint32_t num_valid = s_num_valid;    // edges actually present in this batch (<= 32)
        s_edge_first_t[l] = FLT_MAX;               // earliest colliding t; min'd down by atomicMin on collision
        uint32_t local_edge_nsteps = 0;            // this lane's nsteps, for the warp reduction below
        if (l < num_valid) {
            const uint32_t idx = batch_start + l;  // global edge index for this lane
            local_edge_nsteps = edges.nsteps[idx];
            s_edge_nsteps[l] = local_edge_nsteps;
            s_edge_idx[l] = idx;
            s_edge_type[l] = edges.type[idx];
            s_edge_flags[l] = edges.flags[idx];
            s_s_quat[l] = edges.s_quat[idx];
            s_e_quat[l] = edges.e_quat[idx];
            s_s_trans[l] = edges.s_trans[idx];
            s_e_trans[l] = edges.e_trans[idx];
            s_s_euler[l] = edges.s_euler[idx];
            s_e_euler[l] = edges.e_euler[idx];
        }
        __syncwarp(0xFFFFFFFFu);

        // Highest sample index any edge in this batch needs: drives the
        // pooled root-check round loop. A warp reduction (all lanes must
        // call it converged, which holds here) gives every lane the same
        // value, keeping the round loop control uniform - a thread-0-only
        // register would diverge the warp and deadlock at the loop's
        // __syncthreads. Invalid lanes contribute 0, which is harmless.
        // Highest sample index any edge in this batch needs: drives the
        // pooled root-check round loop. A warp-wide max (shfl-xor butterfly)
        // gives every lane the same value, keeping the round loop control
        // uniform - a thread-0-only register would diverge the warp and
        // deadlock at the loop's __syncwarp. Invalid lanes contribute 0,
        // which is harmless.
        uint32_t max_round = local_edge_nsteps;
        #pragma unroll
        for (int off = 16; off > 0; off >>= 1) {
            max_round = max(max_round, __shfl_xor_sync(0xFFFFFFFFu, max_round, off));
        }
        __syncwarp(0xFFFFFFFFu);

        // Spill passes: root-check pooled rounds until the candidate ring
        // fills up (backpressure, no lost candidates), drain it with the two
        // engines, repeat. The pipelining window keeps drains large and
        // amortizes their fixed cost; dead edges are skipped by phase A on
        // the next pass, so long runs of all-colliding edges still die in
        // the first drain.
        uint32_t round = 0;                       // current sample index j being root-checked
        while (true) {
            __syncwarp(0xFFFFFFFFu);

            // ---- Phase A: two pooled root-check rounds per __syncwarp
            // (halves the per-round barrier cost). The backpressure check
            // reserves room for BOTH rounds' pushes (<= 64) up front; the
            // second round reads a tail counter that undercounts by at most
            // 32, which the doubled slack absorbs.
            for (; round <= max_round; ) {
                const uint32_t count = s_trav_tail - s_trav_head; // ring occupancy (exact: prev round synced)
                if (count > CAND_CAP - 2 * EDGE_COLLAB_BLOCK) {
                    break; // not enough room for two rounds: go drain
                }
                // ---- round A ----
                if (l < num_valid && round <= s_edge_nsteps[l] && !((s_edge_dead >> l) & 1u)) {
                    const uint32_t nsteps = s_edge_nsteps[l];
                    const float t = (nsteps > 0) ? (float)round / (float)nsteps : 0.0f; // sample parameter t = j/nsteps
                    interpolateEdgeState(s_s_quat[l], s_s_trans[l],
                                         s_e_quat[l], s_e_trans[l],
                                         s_s_euler[l], s_e_euler[l],
                                         s_edge_type[l], t, q_conf, T_conf);
                    const Quat qc = {q_conf.x, q_conf.y, q_conf.z, q_conf.w};
                    computeRelTransformQuat(q_obs_root, T_obs_abs_root,
                                            q_rob_root, T_rob_abs_root,
                                            qc, T_conf, qB, T);
                    if (obbOverlapQuat(qB, T, a_root, b_root, epsilon)) {
                        // Root boxes overlap: this sample needs real traversal.
                        // Reserve the next ring slot and store (edge, j).
                        const uint32_t pos = atomicAdd(&s_trav_tail, 1);
                        s_trav[pos & (CAND_CAP - 1)] = (l << 24) | round; //circular buffer, pack lane and round into one uint32_t
                    }
                }
                ++round;
                // ---- round B (no sync in between: tail undercount is absorbed by the slack) ----
                if (round <= max_round) {
                    if (l < num_valid && round <= s_edge_nsteps[l] && !((s_edge_dead >> l) & 1u)) {
                        const uint32_t nsteps = s_edge_nsteps[l];
                        const float t = (nsteps > 0) ? (float)round / (float)nsteps : 0.0f;
                        interpolateEdgeState(s_s_quat[l], s_s_trans[l],
                                             s_e_quat[l], s_e_trans[l],
                                             s_s_euler[l], s_e_euler[l],
                                             s_edge_type[l], t, q_conf, T_conf);
                        const Quat qc = {q_conf.x, q_conf.y, q_conf.z, q_conf.w};
                        computeRelTransformQuat(q_obs_root, T_obs_abs_root,
                                                q_rob_root, T_rob_abs_root,
                                                qc, T_conf, qB, T);
                        if (obbOverlapQuat(qB, T, a_root, b_root, epsilon)) {
                            const uint32_t pos = atomicAdd(&s_trav_tail, 1);
                            s_trav[pos & (CAND_CAP - 1)] = (l << 24) | round;
                        }
                    }
                    ++round;
                }
                __syncwarp(0xFFFFFFFFu);
            }
            __syncwarp(0xFFFFFFFFu);

            // ---- Phase B: drain the candidates (two lockstep 16-lane
            // engines). Two __syncwarp per iteration: the two MANAGER
            // LANES (thread 0 for engine 0, thread 16 for engine 1) run
            // their engine's state machine in parallel - the engines touch
            // disjoint shared state, and the shared candidate ring is popped
            // with a CAS, so nothing needs to be serialized through a single
            // thread. Then each engine expands one pend pair with its 16
            // lanes (4x4 children). No subgroup barriers are needed.
            if (s_trav_head != s_trav_tail) { // ring non-empty: candidates to traverse
                do {
                __syncwarp(0xFFFFFFFFu); // make the previous work step's pushes/death writes visible to the manager lanes

                // ============ MANAGER (one lane per engine) ============
                // Lane (g, 0) runs engine g's state machine; the two lanes
                // execute in parallel and never write the same shared
                // location (all s_eng_* are indexed by g). The following
                // __syncwarp broadcasts both lanes' decisions to all 32
                // lanes. Per engine:
                //   1. idle  -> pop the next live candidate (seed traversal)
                //   2. done/collided/overflow -> drop the current item
                //   3. mid-traversal -> schedule the top pend pair
                {
                    const int g = threadIdx.x >> 4;
                    if ((threadIdx.x & 15) == 0) { // manager lane of this engine
                        s_eng_pend_idx[g][0] = -1; // default: nothing for the work step to do
                        s_eng_pend_idx[g][1] = -1;
                        if (s_eng_edge[g] == NO_ITEM) {
                            // Engine idle: pop the next candidate whose edge
                            // is still alive, discarding dead-edge entries.
                            // The pop is a CAS on the monotonic head counter:
                            // the other manager may be popping concurrently,
                            // so losing the CAS just means retrying (tail is
                            // constant during phase B, so a simple h >= tail
                            // check tells us the ring is empty).
                            while (true) {
                                const uint32_t h = s_trav_head;              // candidate slot we would take
                                if (h >= s_trav_tail) break;                 // ring empty
                                if (atomicCAS(&s_trav_head, h, h + 1) != h) continue; // lost the race: retry
                                const uint32_t item = s_trav[h & (CAND_CAP - 1)]; // we own slot h
                                const uint32_t e = item >> 24;              // unpack local edge id
                                if (!((s_edge_dead >> e) & 1u)) {
                                    const uint32_t j = item & 0xFFFFFFu;    // unpack sample index
                                    const uint32_t nsteps = s_edge_nsteps[e];
                                    const float t = (nsteps > 0) ? (float)j / (float)nsteps : 0.0f;
                                    // Seed the traversal: the root pair (0,0)
                                    // is already proven to overlap by phase A,
                                    // so traversal starts at its children.
                                    s_eng_edge[g] = e;          // this engine now owns this sample
                                    s_eng_j[g] = j;             // remember j for t (= j/nsteps)
                                    s_num_obb_pend[g] = 1;      // pend stack holds the root pair
                                    s_rob_pend[g][0] = 0;       // root robot node
                                    s_obs_pend[g][0] = 0;       // root obstacle node
                                    s_eng_collision[g] = false; // no collision found yet for this sample
                                    // Interpolate the sample state once per
                                    // item into shared, in pure scalar math
                                    // (no Eigen temporaries -> no local-memory
                                    // traffic in the manager path), reused by
                                    // every pend-pair expansion.
                                    const Eigen::Vector3f& s0 = s_s_trans[e];
                                    const Eigen::Vector3f& s1 = s_e_trans[e];
                                    s_eng_t[g] = Eigen::Vector3f(s0(0) + (s1(0) - s0(0)) * t,
                                                                  s0(1) + (s1(1) - s0(1)) * t,
                                                                  s0(2) + (s1(2) - s0(2)) * t);
                                    float4 q;
                                    if (s_edge_type[e] == EDGE_TYPE_SE3_SLERP) {
                                        q = quatSlerp(s_s_quat[e], s_e_quat[e], t);
                                    } else {
                                        const Eigen::Vector3f& r0 = s_s_euler[e];
                                        const Eigen::Vector3f& r1 = s_e_euler[e];
                                        Configuration c;
                                        c.x = 0.0f; c.y = 0.0f; c.z = 0.0f;
                                        c.pitch = r0(0) + (r1(0) - r0(0)) * t;
                                        c.yaw   = r0(1) + (r1(1) - r0(1)) * t;
                                        c.roll  = r0(2) + (r1(2) - r0(2)) * t;
                                        q = configurationToQuat(c);
                                    }
                                    s_eng_q[g] = q;
                                    break;
                                }
                                // Edge already dead: discard and keep popping.
                            }
                        }
                        if (s_eng_edge[g] != NO_ITEM) {
                            const uint32_t e = s_eng_edge[g];
                            // NOTE: an in-flight traversal is never dropped
                            // just because its edge died elsewhere - it runs
                            // to completion so its collision (possibly at an
                            // earlier t) is recorded. Dead edges are skipped
                            // only at ring pop, and since the ring is
                            // round-ordered (FIFO by push order), the
                            // atomicMin first_invalid_t is the true earliest
                            // colliding sample.
                            if (s_eng_collision[g]) {
                                // The work step found a leaf-leaf collision:
                                // edge already marked dead + t recorded there.
                                // Discard the item.
                                s_eng_edge[g] = NO_ITEM;
                                s_num_obb_pend[g] = 0;
                                s_eng_collision[g] = false;
                            } else if (s_num_obb_pend[g] >= MAX_BUF_G - 16) {
                                // Pend stack about to overflow: conservatively
                                // report a collision for this sample (safer
                                // than d_bvh_naive's break-and-mark-valid).
                                const uint32_t nsteps = s_edge_nsteps[e];
                                const float t = (nsteps > 0) ? (float)s_eng_j[g] / (float)nsteps : 0.0f;
                                edgeOverflowReport(s_num_obb_pend[g], s_edge_idx[e], s_eng_j[g], blockIdx.x);
                                atomicOr(&s_edge_dead, 1u << e);   // kill the edge...
                                atomicMin((unsigned int*)&s_edge_first_t[e], __float_as_uint(t)); // ...and record t
                                s_eng_edge[g] = NO_ITEM;
                                s_num_obb_pend[g] = 0;
                            } else if (s_num_obb_pend[g] > 0) {
                                // Mid-traversal: hand the TOP TWO pairs (LIFO,
                                // like d_bvh_naive) to the work step this
                                // iteration, popping them from the stack
                                // (the work step re-pushes their children).
                                const int num = s_num_obb_pend[g];
                                s_eng_pend_idx[g][0] = num - 1; // top pair
                                if (num >= 2) {
                                    s_eng_pend_idx[g][1] = num - 2; // second pair
                                    s_num_obb_pend[g] = num - 2;
                                } else {
                                    s_num_obb_pend[g] = 0; // only one pair left
                                }
                            } else {
                                // Stack empty and no collision: this sample
                                // traversed clean, mark it done.
                                s_eng_edge[g] = NO_ITEM;
                            }
                        }
                    }
                }
                __syncwarp(0xFFFFFFFFu); // broadcast the manager lanes' decisions to all lanes

                // ================= WORK STEP (all 32 lanes) =================
                // Each engine (16 lanes) expands up to TWO pend pairs this
                // iteration: two 8-lane groups, each group covering one
                // pair's 4x4 child combinations with two combos per lane
                // (8 lanes x 2 = 16; the two combos are independent, giving
                // ILP to hide the divergent global-load latency). Structure
                // mirrors d_bvh_naive's inner loop; both engines run the
                // same code with g selecting their own shared state.
                {
                    const int g = threadIdx.x >> 4;    // engine id (0 or 1)
                    const int gl = threadIdx.x & 15;   // lane within the engine (0..15)
                    const int grp = gl >> 3;           // which scheduled pair this 8-lane group expands
                    const int gl8 = gl & 7;            // lane within the group (0..7)
                    const int pend_idx = s_eng_pend_idx[g][grp]; // pair scheduled by the manager (-1 = idle this iteration)
                    if (pend_idx >= 0) {
                        const uint32_t e = s_eng_edge[g];      // local edge id of the sample being traversed
                        const float4& q_c = s_eng_q[g];        // sample state, interpolated once by the manager
                        const Eigen::Vector3f& T_c = s_eng_t[g];
                        const Quat qc = {q_c.x, q_c.y, q_c.z, q_c.w};

                        const int rob_obb_par_idx = s_rob_pend[g][pend_idx]; // parent robot node of this pair
                        const int obs_obb_par_idx = s_obs_pend[g][pend_idx]; // parent obstacle node

                        // Two 4x4 child combos per lane: combo = gl8 and gl8+8.
                        #pragma unroll
                        for (int k = 0; k < 2; ++k) {
                            const int combo = gl8 + (k << 3);
                            const int rob_child_idx = (combo >> 2) & 0x3; // 4x4 mapping: rows = robot child
                            const int obs_child_idx = combo & 0x3;         //                  cols = obstacle child

                            const int rob_obb_idx = sRob_first_child[rob_obb_par_idx] + rob_child_idx; // robot child node
                            const int obs_obb_idx = sObs_first_child[obs_obb_par_idx] + obs_child_idx; // obstacle child node

                            const int rob_first_child_idx = sRob_first_child[rob_obb_idx]; // child's first_child slot
                            const int obs_first_child_idx = sObs_first_child[obs_obb_idx]; // (>0 internal, <0 leaf, 0 dummy)

                            // if either is a dummy node, skip check (nested
                            // if: no continue here - it would skip the
                            // loop-bottom __syncwarp and deadlock)
                            if (rob_first_child_idx != 0 && obs_first_child_idx != 0) {
                                const float4 qof = obs.Rq[obs_obb_idx];
                                q_obs_abs = {qof.x, qof.y, qof.z, qof.w}; // load the two boxes...
                                T_obs_abs = obs.T[obs_obb_idx];
                                const float4 qrf = rob.Rq[rob_obb_idx];
                                q_rob_abs = {qrf.x, qrf.y, qrf.z, qrf.w};
                                T_rob_abs = rob.T[rob_obb_idx];
                                b = rob.dim[rob_obb_idx];          // ...their half dimensions...
                                a = obs.dim[obs_obb_idx];

                                // ...and test them with the 15-axis SAT,
                                // reusing the sample state cached by the manager.
                                computeRelTransformQuat(q_obs_abs, T_obs_abs, q_rob_abs, T_rob_abs,
                                                        qc, T_c, qB, T);

                                if (obbOverlapQuat(qB, T, a, b, epsilon)) {
                                    if (rob_first_child_idx > 0 && obs_first_child_idx > 0) {
                                        // Internal-internal: push the child
                                        // pair onto this engine's pend stack.
                                        int pos = atomicAdd(&s_num_obb_pend[g], 1);
                                        s_obs_pend[g][pos] = (uint16_t)obs_obb_idx;
                                        s_rob_pend[g][pos] = (uint16_t)rob_obb_idx;
                                    }

                                    //TODO: when one box is a leaf and the other
                                    // is not, the leaf's parent is pushed back
                                    // into the pending list (duplicate work,
                                    // harmless).
                                    else if (rob_first_child_idx < 0) {
                                        // both are leaves
                                        if (obs_first_child_idx < 0) {
                                            const int rob_tri = -(rob_first_child_idx + 1); // leaf encoding: -(tri+1)
                                            const int obs_tri = -(obs_first_child_idx + 1);
                                            const Eigen::Vector3f& dimObs = obs.dim[obs_obb_idx];
                                            const Eigen::Vector3f& dimRob = rob.dim[rob_obb_idx];
                                            const Eigen::Matrix3f B = quatToMatrix(qB); // narrow phase still uses the matrix form
                                            const int verdict = paperTriTri(dimObs(0), dimObs(1), obs.a[obs_obb_idx],
                                                                            dimRob(0), dimRob(1), rob.a[rob_obb_idx],
                                                                            B, T);
                                            if (verdict > 0) {
                                                // Triangles intersect: the edge is
                                                // invalid. Mark it dead (any lane may
                                                // do it) and record t with atomicMin
                                                // so the EARLIEST colliding sample
                                                // survives concurrent writes.
                                                const float t = (s_edge_nsteps[e] > 0) ? (float)s_eng_j[g] / (float)s_edge_nsteps[e] : 0.0f;
                                                atomicOr(&s_edge_dead, 1u << e);
                                                atomicMin((unsigned int*)&s_edge_first_t[e], __float_as_uint(t));
                                                s_eng_collision[g] = true;
                                            } else if (verdict < 0) {
                                                // borderline/coplanar: full
                                                // world-frame test (out of line).
                                                edgeLeafFallback(qc, T_c, rob_tri, obs_tri,
                                                                 rob.verts, rob.tris,
                                                                 obs.verts, obs.tris,
                                                                 e, s_eng_j[g], s_edge_nsteps[e],
                                                                 &s_edge_dead, s_edge_first_t, &s_eng_collision[g]);
                                            }
                                        }
                                        // robot is leaf, obstacle is not:
                                        // descend the obstacle by re-pushing
                                        // (rob leaf, obs child).
                                        else {
                                            int pos = atomicAdd(&s_num_obb_pend[g], 1);
                                            s_obs_pend[g][pos] = (uint16_t)obs_obb_idx;
                                            s_rob_pend[g][pos] = (uint16_t)rob_obb_par_idx;
                                        }
                                    }
                                    // obstacle is leaf, robot is not: descend
                                    // the robot by re-pushing (rob child, obs leaf).
                                    else if (obs_first_child_idx < 0) {
                                        int pos = atomicAdd(&s_num_obb_pend[g], 1);
                                        s_obs_pend[g][pos] = (uint16_t)obs_obb_par_idx;
                                        s_rob_pend[g][pos] = (uint16_t)rob_obb_idx;
                                    }
                                }
                            }
                        }
                    }
                }
            // Drain loop condition, evaluated uniformly by every lane from
            // manager-written state (settled at the post-manager
            // __syncwarp): keep going while the ring has candidates or an
            // engine still owns an item. One work step stale (a final no-op
            // iteration is possible) - same trade as the old shared flag.
            } while ((s_trav_head != s_trav_tail) ||
                     (s_eng_edge[0] != NO_ITEM) || (s_eng_edge[1] != NO_ITEM));
                __syncwarp(0xFFFFFFFFu); // engine writes visible before the next phase-A round
            }


            if (round > max_round) {
                break; // every sample round has been root-checked and the ring is drained
            }
        }

        // ================= BATCH EPILOGUE (thread 0) =================
        // Write this batch's verdicts: valid bits for edges that never died,
        // first_invalid_t for dead edges that asked for it (REPORT flag).
        __syncwarp(0xFFFFFFFFu);
        if (threadIdx.x == 0) {
            uint32_t word = 0; // batch's bitpacked result (1 = edge valid)
            for (uint32_t i = 0; i < num_valid; ++i) {
                if (!((s_edge_dead >> i) & 1u)) {
                    word |= (1u << i); // local edge i survived every sample -> valid
                } else if (s_edge_flags[i] & EDGE_FLAG_REPORT_FIRST_INVALID_T) {
                    out.pfirst_invalid_t[s_edge_idx[i]] = s_edge_first_t[i]; // earliest colliding t
                }
            }
            out.pdisjoint[batch_start >> 5] |= word; // batch is 32-aligned: one uint32 per batch
        }
        __syncwarp(0xFFFFFFFFu);
    }
    return;
}

double bvh_edges_collab(const BVNode_soa<>& rob_BVH, const BVNode_soa<>& obs_BVH,
                        const MeshData& rob_mesh, const MeshData& obs_mesh,
                        const EdgeBatch& edges,
                        std::vector<bool>& valid, std::vector<float>& first_invalid_t, bool dry_run) {
    cudaEvent_t start, stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);

    std::cout << "Rob BVH size: " << rob_BVH.size << std::endl;
    std::cout << "Obs BVH size: " << obs_BVH.size << std::endl;

    const int num_edges = (int)edges.size;
    const size_t num_words = (num_edges + 31) / 32;
    valid.assign(num_edges, false);
    first_invalid_t.assign(num_edges, -1.0f);
    if (num_edges == 0) {
        return 0.0;
    }

    // Bitpacked results: bit i of disjoint[i >> 5] is the result of edge i.
    std::unique_ptr<uint32_t[]> disjoint(new uint32_t[num_words]());

    // Rotations are stored as unit quaternions (x, y, z, w) in float4
    // (see bvh_naive).
    std::vector<float4> obs_q(obs_BVH.size);
    std::vector<float4> rob_q(rob_BVH.size);
    for (size_t i = 0; i < obs_BVH.size; ++i) {
        const Eigen::Quaternionf q(obs_BVH.pR[i]);
        obs_q[i] = make_float4(q.x(), q.y(), q.z(), q.w());
    }
    for (size_t i = 0; i < rob_BVH.size; ++i) {
        const Eigen::Quaternionf q(rob_BVH.pR[i]);
        rob_q[i] = make_float4(q.x(), q.y(), q.z(), q.w());
    }

    const int blockSize = 32;
    // Persistent blocks pulling edges from a global work queue. Size the grid
    // so every SM is fully occupied (the kernel fits multiple blocks per SM).
    int device;
    cudaGetDevice(&device);
    int num_sms = 0;
    cudaDeviceGetAttribute(&num_sms, cudaDevAttrMultiProcessorCount, device);

    // Dynamic shared memory: just the two first_child arrays (read-only work set).
    const size_t smem_size = (obs_BVH.size + rob_BVH.size) * sizeof(int16_t);

    int blocks_per_sm = 0;
    cudaOccupancyMaxActiveBlocksPerMultiprocessor(&blocks_per_sm, d_bvh_edges_collab, blockSize, smem_size);
    if (blocks_per_sm < 1) blocks_per_sm = 1;
    const int max_blocks = (num_edges + blockSize - 1) / blockSize;
    const int persistent_blocks = num_sms * blocks_per_sm;
    const int gridSize = (max_blocks < persistent_blocks) ? max_blocks : persistent_blocks;


    float4* d_Q_obs;
    Eigen::Vector3f* d_T_obs;
    float4* d_Q_rob;
    Eigen::Vector3f* d_T_rob;
    Eigen::Vector3f* d_Rob_dim;
    Eigen::Vector3f* d_Obs_dim;
    float4* d_Edge_s_quat;
    float4* d_Edge_e_quat;
    Eigen::Vector3f* d_Edge_s_trans;
    Eigen::Vector3f* d_Edge_e_trans;
    Eigen::Vector3f* d_Edge_s_euler;
    Eigen::Vector3f* d_Edge_e_euler;
    uint32_t* d_Edge_nsteps;
    EdgeType* d_Edge_type;
    uint32_t* d_Edge_flags;
    Eigen::Vector3f* d_Rob_vertices;
    Eigen::Vector3f* d_Obs_vertices;
    Triangle * d_Rob_triangles;
    Triangle * d_Obs_triangles;
    int16_t* d_Obs_first_child;
    int16_t* d_Rob_first_child;
    float* d_Obs_a;
    float* d_Rob_a;
    uint32_t* pdisjoint;
    float* d_first_invalid_t;
    uint32_t* d_next_edge;

    // Allocate memory for device pointers
    cudaEventRecord(start, 0);
    cudaMalloc((void**)&d_Q_obs, obs_BVH.size * sizeof(float4));
    cudaMalloc((void**)&d_T_obs, obs_BVH.size * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Q_rob, rob_BVH.size * sizeof(float4));
    cudaMalloc((void**)&d_T_rob, rob_BVH.size * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Rob_dim, rob_BVH.size * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Obs_dim, obs_BVH.size * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Edge_s_quat, num_edges * sizeof(float4));
    cudaMalloc((void**)&d_Edge_e_quat, num_edges * sizeof(float4));
    cudaMalloc((void**)&d_Edge_s_trans, num_edges * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Edge_e_trans, num_edges * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Edge_s_euler, num_edges * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Edge_e_euler, num_edges * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Edge_nsteps, num_edges * sizeof(uint32_t));
    cudaMalloc((void**)&d_Edge_type, num_edges * sizeof(EdgeType));
    cudaMalloc((void**)&d_Edge_flags, num_edges * sizeof(uint32_t));
    cudaMalloc((void**)&d_Obs_first_child, obs_BVH.size * sizeof(int16_t));
    cudaMalloc((void**)&d_Rob_first_child, rob_BVH.size * sizeof(int16_t));
    cudaMalloc((void**)&d_Obs_a, obs_BVH.size * sizeof(float));
    cudaMalloc((void**)&d_Rob_a, rob_BVH.size * sizeof(float));
    cudaMalloc((void**)&d_Rob_vertices, rob_mesh.vertices.size() * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Obs_vertices, obs_mesh.vertices.size() * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Rob_triangles, rob_mesh.triangles.size() * sizeof(Triangle));
    cudaMalloc((void**)&d_Obs_triangles, obs_mesh.triangles.size() * sizeof(Triangle));
    cudaMalloc((void**)&pdisjoint, num_words * sizeof(uint32_t));
    cudaMalloc((void**)&d_first_invalid_t, num_edges * sizeof(float));
    cudaMalloc((void**)&d_next_edge, sizeof(uint32_t));
    checkCudaMem(cudaMemset(d_next_edge, 0, sizeof(uint32_t)));
    
    cudaDeviceSynchronize();
    checkCudaMem(cudaMemcpy(d_Q_obs, obs_q.data(), obs_BVH.size * sizeof(float4), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_T_obs, obs_BVH.pT, obs_BVH.size * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Q_rob, rob_q.data(), rob_BVH.size * sizeof(float4), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_T_rob, rob_BVH.pT, rob_BVH.size * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_dim, rob_BVH.pDim, rob_BVH.size * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_dim, obs_BVH.pDim, obs_BVH.size * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_first_child, rob_BVH.first_child, rob_BVH.size * sizeof(int16_t), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_first_child, obs_BVH.first_child, obs_BVH.size * sizeof(int16_t), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_a, obs_BVH.pA, obs_BVH.size * sizeof(float), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_a, rob_BVH.pA, rob_BVH.size * sizeof(float), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_vertices, rob_mesh.vertices.data(), rob_mesh.vertices.size() * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_vertices, obs_mesh.vertices.data(), obs_mesh.vertices.size() * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_triangles, rob_mesh.triangles.data(), rob_mesh.triangles.size() * sizeof(Triangle), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_triangles, obs_mesh.triangles.data(), obs_mesh.triangles.size() * sizeof(Triangle), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemset(pdisjoint, 0, num_words * sizeof(uint32_t)));
    cudaDeviceSynchronize();
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    float duration = 0;
    cudaEventElapsedTime(&duration, start, stop);
    std::cout << "Initial allocation and transfer to GPU took " << duration << " ms." << std::endl;

    // Copy edges over separately
    cudaEventRecord(start, 0);
    checkCudaMem(cudaMemcpy(d_Edge_s_quat, edges.s_quat, num_edges * sizeof(float4), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Edge_e_quat, edges.e_quat, num_edges * sizeof(float4), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Edge_s_trans, edges.s_trans, num_edges * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Edge_e_trans, edges.e_trans, num_edges * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Edge_s_euler, edges.s_euler, num_edges * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Edge_e_euler, edges.e_euler, num_edges * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Edge_nsteps, edges.nsteps, num_edges * sizeof(uint32_t), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Edge_type, edges.type, num_edges * sizeof(EdgeType), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Edge_flags, edges.flags, num_edges * sizeof(uint32_t), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_first_invalid_t, first_invalid_t.data(), num_edges * sizeof(float), cudaMemcpyHostToDevice));
    cudaDeviceSynchronize();
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&duration, start, stop);
    std::cout << "Copying edges to GPU took " << duration << " ms." << std::endl;

    std::cout << "obs_BVH.size: " << obs_BVH.size << ", rob_BVH.size: " << rob_BVH.size << std::endl;

    RigidObstacleSoA obsSoA;
    obsSoA.R = nullptr; obsSoA.Rq = d_Q_obs; obsSoA.T = d_T_obs; obsSoA.dim = d_Obs_dim;
    obsSoA.first_child = d_Obs_first_child; obsSoA.a = d_Obs_a;
    obsSoA.verts = d_Obs_vertices; obsSoA.tris = d_Obs_triangles;
    obsSoA.num_nodes = obs_BVH.size;
    RigidRobotSoA robSoA;
    robSoA.R = nullptr; robSoA.Rq = d_Q_rob; robSoA.T = d_T_rob; robSoA.dim = d_Rob_dim;
    robSoA.first_child = d_Rob_first_child; robSoA.a = d_Rob_a;
    robSoA.verts = d_Rob_vertices; robSoA.tris = d_Rob_triangles;
    robSoA.num_nodes = rob_BVH.size;
    robSoA.conf_rot = nullptr; robSoA.conf_trans = nullptr;
    EdgeSoA edgeSoA;
    edgeSoA.s_quat = d_Edge_s_quat; edgeSoA.e_quat = d_Edge_e_quat;
    edgeSoA.s_trans = d_Edge_s_trans; edgeSoA.e_trans = d_Edge_e_trans;
    edgeSoA.s_euler = d_Edge_s_euler; edgeSoA.e_euler = d_Edge_e_euler;
    edgeSoA.nsteps = d_Edge_nsteps; edgeSoA.type = d_Edge_type; edgeSoA.flags = d_Edge_flags;
    EdgeKernelOut outSoA;
    outSoA.pdisjoint = pdisjoint; outSoA.pfirst_invalid_t = d_first_invalid_t;
    outSoA.g_next_edge = d_next_edge;
    outSoA.overflowCounter = nullptr; // rigid kernels report overflows via printf

    auto launch_bvh_edges_collab = [&]() {
        d_bvh_edges_collab<<<gridSize, blockSize, smem_size>>>(
            obsSoA, robSoA, edgeSoA, static_cast<size_t>(num_edges), outSoA);
    };

    // Dynamic shared memory above 48KB requires opting in once per kernel.
    // The opt-in limit is the total (static + dynamic) shared memory per
    // block, so the static usage of this kernel must be subtracted.
    if (smem_size > 48 * 1024) {
        cudaFuncAttributes attr;
        checkCudaMem(cudaFuncGetAttributes(&attr, d_bvh_edges_collab));
        int optin = 0;
        checkCudaMem(cudaDeviceGetAttribute(&optin, cudaDevAttrMaxSharedMemoryPerBlockOptin, 0));
        const int maxDyn = optin - (int)attr.sharedSizeBytes;
        if (smem_size > (size_t)maxDyn) {
            std::cerr << "d_bvh_edges_collab: need " << smem_size << " B dynamic shared memory, but "
                      << maxDyn << " B is available after static usage ("
                      << attr.sharedSizeBytes << " B)." << std::endl;
            exit(1);
        }
        checkCudaMem(cudaFuncSetAttribute(d_bvh_edges_collab, cudaFuncAttributeMaxDynamicSharedMemorySize, maxDyn));
        std::cout << "Opted d_bvh_edges_collab into " << maxDyn << " B dynamic shared memory (needs "
                  << smem_size << " B)." << std::endl;
    }

    // Dry run: single-block launch over one batch, untimed, purely to get the
    // kernel loaded onto the device (the work queue would otherwise drain the
    // entire workload in this block).
    if (dry_run) {
        const size_t dry_edges = (num_edges < (size_t)blockSize) ? num_edges : (size_t)blockSize;
        d_bvh_edges_collab<<<1, blockSize, smem_size>>>(
            obsSoA, robSoA, edgeSoA, dry_edges, outSoA);
        checkCudaMem(cudaGetLastError());
        checkCudaMem(cudaDeviceSynchronize());
        std::cout << "BVH Edges Collab dry run completed successfully." << std::endl;
        // Reset the work queue so the timed launch starts from edge 0
        checkCudaMem(cudaMemset(d_next_edge, 0, sizeof(uint32_t)));
                checkCudaMem(cudaMemset(pdisjoint, 0, num_words * sizeof(uint32_t)));
        checkCudaMem(cudaMemcpy(d_first_invalid_t, first_invalid_t.data(), num_edges * sizeof(float), cudaMemcpyHostToDevice));
    }

    cudaEventRecord(start, 0);
    launch_bvh_edges_collab();
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&duration, start, stop);
    std::cout << "BVH Edges GPU kernel took " << duration << " ms for " << num_edges << " edges." << std::endl;

    // Profiling: phase totals are summed across blocks (each block reports
    // once at termination); report the per-block average, which is a
    // block-serial view and NOT comparable to the wall-clock kernel time.
    // Copy result back to host
    cudaEventRecord(start, 0);
    checkCudaMem(cudaGetLastError());

    checkCudaMem(cudaMemcpy(disjoint.get(), pdisjoint, num_words * sizeof(uint32_t), cudaMemcpyDeviceToHost));
    checkCudaMem(cudaMemcpy(first_invalid_t.data(), d_first_invalid_t, num_edges * sizeof(float), cudaMemcpyDeviceToHost));
    cudaDeviceSynchronize();
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&duration, start, stop);
    std::cout << "Copying results from GPU took " << duration << " ms." << std::endl;

    for (int i = 0; i < num_edges; ++i) {
        valid[i] = (disjoint[i >> 5] >> (i & 31)) & 1u;
    }

    // Free device memory
    cudaFree(d_Q_obs);
    cudaFree(d_T_obs);
    cudaFree(d_Q_rob);
    cudaFree(d_T_rob);
    cudaFree(d_Rob_dim);
    cudaFree(d_Obs_dim);
    cudaFree(d_Edge_s_quat);
    cudaFree(d_Edge_e_quat);
    cudaFree(d_Edge_s_trans);
    cudaFree(d_Edge_e_trans);
    cudaFree(d_Edge_s_euler);
    cudaFree(d_Edge_e_euler);
    cudaFree(d_Edge_nsteps);
    cudaFree(d_Edge_type);
    cudaFree(d_Edge_flags);
    cudaFree(d_Obs_first_child);
    cudaFree(d_Rob_first_child);
    cudaFree(d_Obs_a);
    cudaFree(d_Rob_a);
    cudaFree(d_Rob_vertices);
    cudaFree(d_Obs_vertices);
    cudaFree(d_Rob_triangles);
    cudaFree(d_Obs_triangles);
    cudaFree(pdisjoint);
    cudaFree(d_first_invalid_t);
    cudaFree(d_next_edge);

    return duration;
}


// ===========================================================================
// d_bvh_edges_articulated: edge-validation version of d_bvh_articulated
// (modernized: one warp per 32-thread block, per-warp work-queue pulls, no
// device-side profiling, quatSAT layouts). Each lane owns one edge and
// root-checks its samples (t = j/nsteps) with incremental FK; edges with any
// root-overlapping sample enter the warp-serial phase, which re-runs the
// samples in order with the full articulated traversal (same math as
// d_bvh_articulated) and breaks on the first colliding sample.
// ===========================================================================
template <size_t N, typename RobChildT, typename ObsChildT, NodeLayout LAYOUT>
__global__ void d_bvh_edges_articulated(const ObstacleSoA<ObsChildT> obs,
                                        const RobotSoA<RobChildT> rob,
                                        const ArticulatedEdgeSoA<N> edges, size_t num_edges,
                                        const EdgeKernelOut out) {
    typedef typename ArticQueueIdx<RobChildT>::Q QRob;
    typedef typename ArticQueueIdx<ObsChildT>::Q QObs;
    constexpr int BLOCK_SIZE = 32;
    constexpr int MAX_BUFFER = 256;
    constexpr int MAX_TRI = 96;
    constexpr int TRI_BATCH = 64;
    constexpr int num_links = (int)N + 1;
    const float epsilon = 1e-6f;

    // Scene root node (loaded once per block).
    Eigen::Matrix3f R_obs_abs_root;
    Eigen::Vector3f T_obs_abs_root, a_root;
    float a_root_unused;
    Quat q_obs_root;
    if (LAYOUT == NodeLayout::QuatSAT) {
        const float4 qo0 = obs.nodes.Rq[0];
        q_obs_root = {qo0.x, qo0.y, qo0.z, qo0.w};
        T_obs_abs_root = obs.nodes.T[0];
        a_root = obs.nodes.dim[0];
    } else {
        loadNode<LAYOUT>(obs.nodes, 0, R_obs_abs_root, T_obs_abs_root, a_root, a_root_unused);
    }

    // Per-warp traversal state (this block IS one warp).
    __shared__ QRob s_rob_pend[MAX_BUFFER];
    __shared__ QObs s_obs_pend[MAX_BUFFER];
    __shared__ int s_num_obb_pend;
    __shared__ QRob s_tri_rob[MAX_TRI];
    __shared__ QObs s_tri_obs[MAX_TRI];
    __shared__ int s_num_tri;
    __shared__ bool s_collision;

    // Per-warp pending-edge queue.
    __shared__ articulated_conf<N> s_pend_s[BLOCK_SIZE];
    __shared__ articulated_conf<N> s_pend_e[BLOCK_SIZE];
    __shared__ uint32_t s_pend_nsteps[BLOCK_SIZE];
    __shared__ uint32_t s_pend_flags[BLOCK_SIZE];
    __shared__ uint32_t s_pend_idx[BLOCK_SIZE];
    __shared__ uint32_t s_num_pend;
    __shared__ uint32_t s_disjoint_word;

    const int lane = threadIdx.x;
    const int16_t conf_offset   = (lane >> 4) - 2;
    const int16_t rob_child_idx = (lane >> 2) & 0x3;
    const int16_t obs_child_idx = lane & 0x3;

    while (true) {
        __syncwarp();
        uint32_t batch_start = 0;
        if (lane == 0) {
            batch_start = atomicAdd(out.g_next_edge, BLOCK_SIZE);
            s_num_pend = 0;
            s_disjoint_word = 0;
        }
        batch_start = __shfl_sync(0xffffffffu, batch_start, 0);
        if (batch_start >= num_edges) {
            return;
        }

        // ---- parallel outermost check: one edge per lane ----------------
        const uint32_t index = batch_start + lane;
        bool root_overlap = false;
        if (index < num_edges) {
            const articulated_conf<N> es = edges.s[index];
            const articulated_conf<N> ee = edges.e[index];
            const uint32_t nsteps = edges.nsteps[index];

            for (uint32_t j = 0; j <= nsteps; ++j) {
                const float t = (nsteps > 0) ? (float)j / (float)nsteps : 0.0f;
                articulated_conf<N> q;
                interpolateArticulatedEdge<N>(es, ee, t, q);

                Eigen::Matrix3f link_R = Eigen::Matrix3f::Identity();
                Eigen::Vector3f link_T = Eigen::Vector3f::Zero();
                bool any = false;
                for (int l = 0; l < num_links && !any; ++l) {
                    if (rob.linkOffset[l + 1] > rob.linkOffset[l]) {
                        const int rob_root = rob.linkOffset[l];
                        Eigen::Matrix3f robRootR, B;
                        Eigen::Vector3f robRootT, robRootDim, T;
                        float robRootA;
                        if (LAYOUT == NodeLayout::QuatSAT) {
                            Quat qRoot, qB;
                            loadNodeQuat<LAYOUT>(rob.nodes, rob_root, qRoot, robRootT, robRootDim, robRootA);
                            const Quat qLink = matrixToQuat(link_R);
                            computeRelTransformQuat(q_obs_root, T_obs_abs_root,
                                                    qRoot, robRootT,
                                                    qLink, link_T, qB, T);
                            any = obbOverlapQuat(qB, T, a_root, robRootDim, epsilon);
                        } else {
                            loadNode<LAYOUT>(rob.nodes, rob_root, robRootR, robRootT, robRootDim, robRootA);
                            computeRelTransformNoBf(R_obs_abs_root, T_obs_abs_root,
                                                    robRootR, robRootT,
                                                    link_R, link_T, B, T);
                            any = obbOverlapAbs(a_root, robRootDim, B, epsilon, T);
                        }
                    }
                    if (l + 1 < num_links) {
                        const JointParams& jp = rob.joints[l];
                        const Eigen::Matrix3f Rj = axisAngleToRotation(jp.axis, q[l]);
                        const Eigen::Matrix3f nextR = link_R * jp.origin_R * Rj;
                        const Eigen::Vector3f nextT = link_T + link_R * jp.origin_T;
                        link_R = nextR;
                        link_T = nextT;
                    }
                }
                if (any) {
                    root_overlap = true;
                    break;
                }
            }

            if (!root_overlap) {
                atomicOr(&s_disjoint_word, 1u << lane);
            } else {
                const uint32_t pos = atomicAdd(&s_num_pend, 1);
                s_pend_s[pos] = es;
                s_pend_e[pos] = ee;
                s_pend_nsteps[pos] = nsteps;
                s_pend_flags[pos] = edges.flags[index];
                s_pend_idx[pos] = index;
            }
        }
        __syncwarp();
        if (lane == 0) {
            out.pdisjoint[batch_start >> 5] |= s_disjoint_word;
        }

        // ---- per-warp serial phase: samples in order, early exit ---------
        for (uint32_t k = 0; k < s_num_pend; ++k) {
            const articulated_conf<N> es = s_pend_s[k];
            const articulated_conf<N> ee = s_pend_e[k];
            const uint32_t nsteps = s_pend_nsteps[k];
            const uint32_t flags = s_pend_flags[k];
            const uint32_t idx = s_pend_idx[k];

            bool edge_collision = false;
            float first_t = -1.0f;

            for (uint32_t j = 0; j <= nsteps; ++j) {
                const float t = (nsteps > 0) ? (float)j / (float)nsteps : 0.0f;
                articulated_conf<N> q;
                interpolateArticulatedEdge<N>(es, ee, t, q);

                if (lane == 0) {
                    s_collision = false;
                }
                __syncwarp();

                // FK accumulators are declared inside the (duplicated)
                // traversal body below; broadcast from lane 0.
                const uint32_t owner = 0;

            // FK accumulators: meaningful only on the owner lane; broadcast
            // to the whole warp at each link (transform matrices are computed
            // once per config, by its owner lane).
            Eigen::Matrix3f ownR = Eigen::Matrix3f::Identity();
            Eigen::Vector3f ownT = Eigen::Vector3f::Zero();

            for (int l = 0; l < num_links; ++l) {
                Eigen::Matrix3f link_R;
                Eigen::Vector3f link_T;
                //TODO: remove hardcoded numbers here
#pragma unroll
                for (int i = 0; i < 9; ++i) {
                    link_R.data()[i] = __shfl_sync(0xffffffffu, ownR.data()[i], owner, 32);
                }
#pragma unroll
                for (int i = 0; i < 3; ++i) {
                    link_T[i] = __shfl_sync(0xffffffffu, ownT[i], owner, 32);
                }

                // Pure-quat broad phase: convert the FK matrix to a unit
                // quaternion once per link (per config), not per node pair.
                Quat q_link;
                if (LAYOUT == NodeLayout::QuatSAT) {
                    q_link = matrixToQuat(link_R);
                }
                const bool hasMesh = (rob.linkOffset[l + 1] > rob.linkOffset[l]);
                // Inline root test: the edge kernel has no precomputed
                // per-link mask, so the (link root, scene root) pair is
                // tested right here (same math as the parallel phase).
                bool rootHit = false;
                if (hasMesh) {
                    const int rob_root = rob.linkOffset[l];
                    Eigen::Matrix3f robRootR, rB;
                    Eigen::Vector3f robRootT, robRootDim, rT;
                    float robRootA;
                    if (LAYOUT == NodeLayout::QuatSAT) {
                        Quat qRoot, qB;
                        loadNodeQuat<LAYOUT>(rob.nodes, rob_root, qRoot, robRootT, robRootDim, robRootA);
                        const Quat qLink = matrixToQuat(link_R);
                        computeRelTransformQuat(q_obs_root, T_obs_abs_root,
                                                qRoot, robRootT,
                                                qLink, link_T, qB, rT);
                        rootHit = obbOverlapQuat(qB, rT, a_root, robRootDim, epsilon);
                    } else {
                        loadNode<LAYOUT>(rob.nodes, rob_root, robRootR, robRootT, robRootDim, robRootA);
                        computeRelTransformNoBf(R_obs_abs_root, T_obs_abs_root,
                                                robRootR, robRootT,
                                                link_R, link_T, rB, rT);
                        rootHit = obbOverlapAbs(a_root, robRootDim, rB, epsilon, rT);
                    }
                }

                // Seed the frontier with the (link root, scene root) pair;
                // root overlap was proven in the parallel phase (mask).
                if (lane == 0) {
                    s_num_obb_pend = 0;
                    s_num_tri = 0;
                    if (hasMesh && rootHit) {
                        s_rob_pend[0] = (QRob)rob.linkOffset[l];
                        s_obs_pend[0] = 0;
                        s_num_obb_pend = 1;
                    }
                }
                __syncwarp();

                if (s_num_obb_pend > 0) {
                    while (true) {
                        __syncwarp();
                        if (s_collision) {
                            break;
                        }
                        // Incremental triangle phase: flush candidates once
                        // TRI_BATCH accumulate (bounds the buffer, gives
                        // early exit at this granularity).
                        if (s_num_tri >= TRI_BATCH) {
                            d_articulated_tri_phase<RobChildT, ObsChildT, QRob, QObs, LAYOUT>(
                                s_tri_rob, s_tri_obs, s_num_tri,
                                link_R, link_T,
                                rob.linkVertOffset[l], rob.linkTriOffset[l],
                                obs, rob,
                                &s_collision);
                            if (lane == 0) {
                                s_num_tri = 0;
                            }
                            continue;
                        }
                        if (s_num_obb_pend == 0) {
                            break;
                        }
                        if (s_num_obb_pend >= MAX_BUFFER - 32) {
                            if (lane == 0) {
                                atomicAdd(out.overflowCounter, 1ull);
                                s_collision = true;
                            }
                            break;
                        }

                        // pend_idx from the pre-pop count: the two newest
                        // pairs are expanded by 16 lanes each. The count is
                        // already visible from the loop-top syncwarp; one
                        // syncwarp after lane 0's pop makes it visible to
                        // the expanders.
                        const int pend_idx = s_num_obb_pend + conf_offset;

                        if (lane == 0) {
                            if (s_num_obb_pend > 1) {
                                s_num_obb_pend -= 2;
                            } else {
                                s_num_obb_pend = 0;
                            }
                        }
                        __syncwarp();

                        if (pend_idx < 0) {
                            continue;
                        }

                        const int rob_obb_par_idx = (int)s_rob_pend[pend_idx];
                        const int obs_obb_par_idx = (int)s_obs_pend[pend_idx];
                        const int rob_fc_par = rob.first_child[rob_obb_par_idx];
                        const int obs_fc_par = obs.first_child[obs_obb_par_idx];

                        // Leaf-aware child selection: a leaf stays paired
                        // with each child of the other node (also handles
                        // leaf roots).
                        int rob_obb_idx = rob_obb_par_idx;
                        int obs_obb_idx = obs_obb_par_idx;
                        bool active = true;
                        if (rob_fc_par < 0 && obs_fc_par < 0) {
                            if (rob_child_idx != 0 || obs_child_idx != 0) {
                                active = false;
                            }
                        } else if (rob_fc_par < 0) {
                            if (rob_child_idx != 0) {
                                active = false;
                            } else {
                                obs_obb_idx = obs_fc_par + obs_child_idx;
                            }
                        } else if (obs_fc_par < 0) {
                            if (obs_child_idx != 0) {
                                active = false;
                            } else {
                                rob_obb_idx = rob_fc_par + rob_child_idx;
                            }
                        } else {
                            rob_obb_idx = rob_fc_par + rob_child_idx;
                            obs_obb_idx = obs_fc_par + obs_child_idx;
                        }
                        if (!active) {
                            continue;
                        }

                        const int rob_first_child_idx = rob.first_child[rob_obb_idx];
                        const int obs_first_child_idx = obs.first_child[obs_obb_idx];
                        if (rob_first_child_idx == 0 || obs_first_child_idx == 0) {
                            continue; // dummy (padding) node
                        }

                        if (LAYOUT == NodeLayout::QuatSAT) {
                            Quat qObs, qRob, qB;
                            Eigen::Vector3f T_obs_abs, T_rob_abs, b, a;
                            float aObs, aRob;
                            loadNodeQuat<LAYOUT>(obs.nodes, obs_obb_idx, qObs, T_obs_abs, a, aObs);
                            loadNodeQuat<LAYOUT>(rob.nodes, rob_obb_idx, qRob, T_rob_abs, b, aRob);
                            Eigen::Vector3f T;
                            computeRelTransformQuat(qObs, T_obs_abs, qRob, T_rob_abs,
                                                    q_link, link_T, qB, T);
                            if (!obbOverlapQuat(qB, T, a, b, epsilon)) {
                                continue;
                            }
                        } else {
                            Eigen::Matrix3f R_obs_abs, R_rob_abs;
                            Eigen::Vector3f T_obs_abs, T_rob_abs, b, a;
                            float aObs, aRob;
                            loadNode<LAYOUT>(obs.nodes, obs_obb_idx, R_obs_abs, T_obs_abs, a, aObs);
                            loadNode<LAYOUT>(rob.nodes, rob_obb_idx, R_rob_abs, T_rob_abs, b, aRob);

                            Eigen::Matrix3f B;
                            Eigen::Vector3f T;
                            computeRelTransformNoBf(R_obs_abs, T_obs_abs, R_rob_abs, T_rob_abs,
                                                    link_R, link_T, B, T);
                            if (!obbOverlapAbs(a, b, B, epsilon, T)) {
                                continue;
                            }
                        }

                        if (rob_first_child_idx < 0 && obs_first_child_idx < 0) {
                            // Both leaves: defer to the triangle phase.
                            const int pos = atomicAdd(&s_num_tri, 1);
                            if (pos < MAX_TRI) {
                                s_tri_rob[pos] = (QRob)rob_obb_idx;
                                s_tri_obs[pos] = (QObs)obs_obb_idx;
                            } else if (lane == 0) {
                                atomicAdd(out.overflowCounter, 1ull);
                                s_collision = true; // conservative
                            }
                        } else {
                            // One or both internal: expand (leaf-aware).
                            const int pos = atomicAdd(&s_num_obb_pend, 1);
                            if (pos < MAX_BUFFER) {
                                s_obs_pend[pos] = (QObs)obs_obb_idx;
                                s_rob_pend[pos] = (QRob)rob_obb_idx;
                            } else if (lane == 0) {
                                atomicAdd(out.overflowCounter, 1ull);
                                s_collision = true; // conservative
                            }
                        }
                    } // OBB traversal of this link

                    __syncwarp();

                    // Final triangle phase for the leftover candidates.
                    if (!s_collision && s_num_tri > 0) {
                        d_articulated_tri_phase<RobChildT, ObsChildT, QRob, QObs, LAYOUT>(
                                s_tri_rob, s_tri_obs, s_num_tri,
                                link_R, link_T,
                                rob.linkVertOffset[l], rob.linkTriOffset[l],
                                obs, rob,
                                &s_collision);
                    }
                    __syncwarp();
                    if (lane == 0) {
                        s_num_tri = 0;
                    }
                }
                __syncwarp();
                if (s_collision) {
                    break; // this link collides: the configuration is in collision
                }

                // Advance the owner lane's FK to the next link.
                if (l + 1 < num_links) {
                    const JointParams& jp = rob.joints[l];
                    const Eigen::Matrix3f Rj = axisAngleToRotation(jp.axis, q[l]);
                    const Eigen::Matrix3f nextR = ownR * jp.origin_R * Rj;
                    const Eigen::Vector3f nextT = ownT + ownR * jp.origin_T;
                    ownR = nextR;
                    ownT = nextT;
                }
            } // links

                __syncwarp();
                if (s_collision) {
                    edge_collision = true;
                    first_t = t;
                    break;
                }
            } // samples of this edge

            __syncwarp();
            if (edge_collision) {
                if (flags & EDGE_FLAG_REPORT_FIRST_INVALID_T) {
                    if (lane == 0) {
                        out.pfirst_invalid_t[idx] = first_t;
                    }
                }
            } else {
                if (lane == 0) {
                    s_disjoint_word |= 1u << (idx & 31);
                }
            }
            __syncwarp();
        } // pending edges

        __syncwarp();
        if (lane == 0) {
            out.pdisjoint[batch_start >> 5] |= s_disjoint_word;
        }
    }
}

// ---------------------------------------------------------------------------
// bvh_edges_articulated: host wrapper mirroring bvh_articulated (same URDF
// parsing, per-link BVH concatenation, layout packing, and dispatch) with the
// articulated edge batch added.
// ---------------------------------------------------------------------------
template <size_t N>
double bvh_edges_articulated(const std::string& robot_urdf_path,
                             const BVNode_soa<int32_t>& obs_BVH, const MeshData& obs_mesh,
                             const ArticulatedEdgeBatch<N>& edges,
                             std::vector<bool>& valid, std::vector<float>& first_invalid_t,
                             bool dry_run, int kernel_mode) {
    cudaEvent_t start, stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);

    const size_t num_edges = edges.size;
    const size_t num_words = (num_edges + 31) / 32;
    valid.assign(num_edges, false);
    first_invalid_t.assign(num_edges, -1.0f);
    if (num_edges == 0) {
        return 0.0;
    }

    ArticulatedRobot robot;
    if (!parseSerialChainURDF(robot_urdf_path, robot)) {
        std::cerr << "Failed to parse URDF: " << robot_urdf_path << std::endl;
        return 0.0;
    }
    if (robot.num_joints != N) {
        std::cerr << "URDF has " << robot.num_joints << " joints, but N=" << N << std::endl;
        return 0.0;
    }
    const int num_links = (int)(robot.num_joints + 1);

    std::vector<Eigen::Matrix3f> rob_R;
    std::vector<Eigen::Vector3f> rob_T;
    std::vector<Eigen::Vector3f> rob_dim;
    std::vector<int32_t> rob_first_child;
    std::vector<float> rob_a;
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
                int32_t fc = (int32_t)bvh.first_child[i];
                if (fc > 0) fc += base;
                rob_first_child.push_back(fc);
                rob_a.push_back(bvh.pA[i]);
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

    std::vector<JointParams> joints = robot.joints;

    // ---- variant selection (same bits as bvh_articulated; bit 1 ignored) --
    const bool want16 = (kernel_mode & 1) != 0;
    const bool rob16 = want16 && (rob_first_child.size() < 32768);
    const bool obs16 = want16 && (obs_BVH.size < 32768);
    NodeLayout layout = static_cast<NodeLayout>((kernel_mode >> 2) & 3);
    if (kernel_mode & 64) layout = NodeLayout::QuatSAT;
    static const char* kLayoutName[5] = {"matrix", "quat", "vecR", "quat+TD", "quatSAT"};
    std::cout << "bvh_edges_articulated: kernel variant = rob<"
              << (rob16 ? "int16" : "int32") << "> obs<"
              << (obs16 ? "int16" : "int32") << "> layout="
              << kLayoutName[static_cast<int>(layout)] << std::endl;

    std::vector<int16_t> obs_first_child16, rob_first_child16;
    if (obs16) {
        obs_first_child16.reserve(obs_BVH.size);
        for (size_t i = 0; i < obs_BVH.size; ++i) obs_first_child16.push_back((int16_t)obs_BVH.first_child[i]);
    }
    if (rob16) {
        rob_first_child16.reserve(rob_first_child.size());
        for (size_t i = 0; i < rob_first_child.size(); ++i) rob_first_child16.push_back((int16_t)rob_first_child[i]);
    }

    // ---- node-data packing (layouts 1-4) ----------------------------------
    std::vector<float4> obs_Rq, obs_Rp, obs_TD, rob_Rq, rob_Rp, rob_TD;
    if (layout == NodeLayout::Quat || layout == NodeLayout::QuatTD || layout == NodeLayout::QuatSAT) {
        obs_Rq.reserve(obs_BVH.size);
        for (size_t i = 0; i < obs_BVH.size; ++i) {
            const Eigen::Quaternionf q(obs_BVH.pR[i]);
            obs_Rq.push_back(make_float4(q.x(), q.y(), q.z(), q.w()));
        }
        rob_Rq.reserve(rob_R.size());
        for (size_t i = 0; i < rob_R.size(); ++i) {
            const Eigen::Quaternionf q(rob_R[i]);
            rob_Rq.push_back(make_float4(q.x(), q.y(), q.z(), q.w()));
        }
    }
    if (layout == NodeLayout::VecR) {
        obs_Rp.reserve(obs_BVH.size * 3);
        for (size_t i = 0; i < obs_BVH.size; ++i) {
            const Eigen::Matrix3f& R = obs_BVH.pR[i];
            obs_Rp.push_back(make_float4(R(0,0), R(0,1), R(0,2), 0.0f));
            obs_Rp.push_back(make_float4(R(1,0), R(1,1), R(1,2), 0.0f));
            obs_Rp.push_back(make_float4(R(2,0), R(2,1), R(2,2), 0.0f));
        }
        rob_Rp.reserve(rob_R.size() * 3);
        for (size_t i = 0; i < rob_R.size(); ++i) {
            const Eigen::Matrix3f& R = rob_R[i];
            rob_Rp.push_back(make_float4(R(0,0), R(0,1), R(0,2), 0.0f));
            rob_Rp.push_back(make_float4(R(1,0), R(1,1), R(1,2), 0.0f));
            rob_Rp.push_back(make_float4(R(2,0), R(2,1), R(2,2), 0.0f));
        }
    }
    if (layout == NodeLayout::QuatTD || layout == NodeLayout::QuatSAT) {
        obs_TD.reserve(obs_BVH.size * 2);
        for (size_t i = 0; i < obs_BVH.size; ++i) {
            const Eigen::Vector3f& T = obs_BVH.pT[i];
            const Eigen::Vector3f& d = obs_BVH.pDim[i];
            obs_TD.push_back(make_float4(T.x(), T.y(), T.z(), d.x()));
            obs_TD.push_back(make_float4(d.y(), d.z(), obs_BVH.pA[i], 0.0f));
        }
        rob_TD.reserve(rob_R.size() * 2);
        for (size_t i = 0; i < rob_R.size(); ++i) {
            const Eigen::Vector3f& T = rob_T[i];
            const Eigen::Vector3f& d = rob_dim[i];
            rob_TD.push_back(make_float4(T.x(), T.y(), T.z(), d.x()));
            rob_TD.push_back(make_float4(d.y(), d.z(), rob_a[i], 0.0f));
        }
    }

    const int blockSize = 32;
    const size_t max_blocks = (num_edges + blockSize - 1) / blockSize;

    int device;
    checkCudaMem(cudaGetDevice(&device));
    int num_sms = 0;
    checkCudaMem(cudaDeviceGetAttribute(&num_sms, cudaDevAttrMultiProcessorCount, device));

    // ---- device allocations (same set as bvh_articulated + edge batch) ----
    Eigen::Matrix3f* d_R_obs = nullptr; Eigen::Vector3f* d_T_obs; Eigen::Vector3f* d_Obs_dim;
    int32_t* d_Obs_first_child32 = nullptr; int16_t* d_Obs_first_child16 = nullptr;
    Eigen::Vector3f* d_Obs_verts; Triangle* d_Obs_tris; float* d_Obs_a;
    Eigen::Matrix3f* d_Rob_R = nullptr; Eigen::Vector3f* d_Rob_T; Eigen::Vector3f* d_Rob_dim;
    int32_t* d_Rob_first_child32 = nullptr; int16_t* d_Rob_first_child16 = nullptr;
    float* d_Rob_a; Eigen::Vector3f* d_Rob_verts; Triangle* d_Rob_tris;
    int* d_LinkOffset; int* d_LinkVertOffset; int* d_LinkTriOffset;
    JointParams* d_Joints;
    float4* d_Obs_Rq = nullptr; float4* d_Obs_Rp = nullptr; float4* d_Obs_TD = nullptr;
    float4* d_Rob_Rq = nullptr; float4* d_Rob_Rp = nullptr; float4* d_Rob_TD = nullptr;
    articulated_conf<N>* d_Edge_s; articulated_conf<N>* d_Edge_e;
    uint32_t* d_Edge_nsteps; uint32_t* d_Edge_flags;
    uint32_t* pdisjoint; float* d_first_invalid_t; uint32_t* d_next_edge;
    unsigned long long* d_overflow;

    cudaEventRecord(start, 0);
    cudaMalloc((void**)&d_R_obs, obs_BVH.size * sizeof(Eigen::Matrix3f));
    cudaMalloc((void**)&d_T_obs, obs_BVH.size * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Obs_dim, obs_BVH.size * sizeof(Eigen::Vector3f));
    if (obs16) cudaMalloc((void**)&d_Obs_first_child16, obs_BVH.size * sizeof(int16_t));
    else       cudaMalloc((void**)&d_Obs_first_child32, obs_BVH.size * sizeof(int32_t));
    cudaMalloc((void**)&d_Obs_verts, obs_mesh.vertices.size() * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Obs_tris, obs_mesh.triangles.size() * sizeof(Triangle));
    cudaMalloc((void**)&d_Obs_a, obs_BVH.size * sizeof(float));
    cudaMalloc((void**)&d_Rob_R, rob_R.size() * sizeof(Eigen::Matrix3f));
    cudaMalloc((void**)&d_Rob_T, rob_T.size() * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Rob_dim, rob_dim.size() * sizeof(Eigen::Vector3f));
    if (rob16) cudaMalloc((void**)&d_Rob_first_child16, rob_first_child.size() * sizeof(int16_t));
    else       cudaMalloc((void**)&d_Rob_first_child32, rob_first_child.size() * sizeof(int32_t));
    cudaMalloc((void**)&d_Rob_a, rob_a.size() * sizeof(float));
    cudaMalloc((void**)&d_Rob_verts, rob_verts.size() * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Rob_tris, rob_tris.size() * sizeof(Triangle));
    cudaMalloc((void**)&d_LinkOffset, (num_links + 1) * sizeof(int));
    cudaMalloc((void**)&d_LinkVertOffset, (num_links + 1) * sizeof(int));
    cudaMalloc((void**)&d_LinkTriOffset, (num_links + 1) * sizeof(int));
    cudaMalloc((void**)&d_Joints, N * sizeof(JointParams));
    if (layout == NodeLayout::Quat || layout == NodeLayout::QuatTD || layout == NodeLayout::QuatSAT) {
        cudaMalloc((void**)&d_Obs_Rq, obs_Rq.size() * sizeof(float4));
        cudaMalloc((void**)&d_Rob_Rq, rob_Rq.size() * sizeof(float4));
    }
    if (layout == NodeLayout::VecR) {
        cudaMalloc((void**)&d_Obs_Rp, obs_Rp.size() * sizeof(float4));
        cudaMalloc((void**)&d_Rob_Rp, rob_Rp.size() * sizeof(float4));
    }
    if (layout == NodeLayout::QuatTD || layout == NodeLayout::QuatSAT) {
        cudaMalloc((void**)&d_Obs_TD, obs_TD.size() * sizeof(float4));
        cudaMalloc((void**)&d_Rob_TD, rob_TD.size() * sizeof(float4));
    }
    cudaMalloc((void**)&d_Edge_s, num_edges * sizeof(articulated_conf<N>));
    cudaMalloc((void**)&d_Edge_e, num_edges * sizeof(articulated_conf<N>));
    cudaMalloc((void**)&d_Edge_nsteps, num_edges * sizeof(uint32_t));
    cudaMalloc((void**)&d_Edge_flags, num_edges * sizeof(uint32_t));
    cudaMalloc((void**)&pdisjoint, num_words * sizeof(uint32_t));
    cudaMalloc((void**)&d_first_invalid_t, num_edges * sizeof(float));
    cudaMalloc((void**)&d_next_edge, sizeof(uint32_t));
    cudaMalloc((void**)&d_overflow, sizeof(unsigned long long));

    checkCudaMem(cudaMemcpy(d_R_obs, obs_BVH.pR, obs_BVH.size * sizeof(Eigen::Matrix3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_T_obs, obs_BVH.pT, obs_BVH.size * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_dim, obs_BVH.pDim, obs_BVH.size * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    if (obs16) checkCudaMem(cudaMemcpy(d_Obs_first_child16, obs_first_child16.data(), obs_BVH.size * sizeof(int16_t), cudaMemcpyHostToDevice));
    else       checkCudaMem(cudaMemcpy(d_Obs_first_child32, obs_BVH.first_child, obs_BVH.size * sizeof(int32_t), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_verts, obs_mesh.vertices.data(), obs_mesh.vertices.size() * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_tris, obs_mesh.triangles.data(), obs_mesh.triangles.size() * sizeof(Triangle), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_a, obs_BVH.pA, obs_BVH.size * sizeof(float), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_R, rob_R.data(), rob_R.size() * sizeof(Eigen::Matrix3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_T, rob_T.data(), rob_T.size() * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_dim, rob_dim.data(), rob_dim.size() * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    if (rob16) checkCudaMem(cudaMemcpy(d_Rob_first_child16, rob_first_child16.data(), rob_first_child.size() * sizeof(int16_t), cudaMemcpyHostToDevice));
    else       checkCudaMem(cudaMemcpy(d_Rob_first_child32, rob_first_child.data(), rob_first_child.size() * sizeof(int32_t), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_a, rob_a.data(), rob_a.size() * sizeof(float), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_verts, rob_verts.data(), rob_verts.size() * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_tris, rob_tris.data(), rob_tris.size() * sizeof(Triangle), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_LinkOffset, link_offset.data(), (num_links + 1) * sizeof(int), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_LinkVertOffset, link_vert_offset.data(), (num_links + 1) * sizeof(int), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_LinkTriOffset, link_tri_offset.data(), (num_links + 1) * sizeof(int), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Joints, joints.data(), N * sizeof(JointParams), cudaMemcpyHostToDevice));
    if (layout == NodeLayout::Quat || layout == NodeLayout::QuatTD || layout == NodeLayout::QuatSAT) {
        checkCudaMem(cudaMemcpy(d_Obs_Rq, obs_Rq.data(), obs_Rq.size() * sizeof(float4), cudaMemcpyHostToDevice));
        checkCudaMem(cudaMemcpy(d_Rob_Rq, rob_Rq.data(), rob_Rq.size() * sizeof(float4), cudaMemcpyHostToDevice));
    }
    if (layout == NodeLayout::VecR) {
        checkCudaMem(cudaMemcpy(d_Obs_Rp, obs_Rp.data(), obs_Rp.size() * sizeof(float4), cudaMemcpyHostToDevice));
        checkCudaMem(cudaMemcpy(d_Rob_Rp, rob_Rp.data(), rob_Rp.size() * sizeof(float4), cudaMemcpyHostToDevice));
    }
    if (layout == NodeLayout::QuatTD || layout == NodeLayout::QuatSAT) {
        checkCudaMem(cudaMemcpy(d_Obs_TD, obs_TD.data(), obs_TD.size() * sizeof(float4), cudaMemcpyHostToDevice));
        checkCudaMem(cudaMemcpy(d_Rob_TD, rob_TD.data(), rob_TD.size() * sizeof(float4), cudaMemcpyHostToDevice));
    }
    checkCudaMem(cudaMemcpy(d_Edge_s, edges.s, num_edges * sizeof(articulated_conf<N>), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Edge_e, edges.e, num_edges * sizeof(articulated_conf<N>), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Edge_nsteps, edges.nsteps, num_edges * sizeof(uint32_t), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Edge_flags, edges.flags, num_edges * sizeof(uint32_t), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemset(pdisjoint, 0, num_words * sizeof(uint32_t)));
    checkCudaMem(cudaMemcpy(d_first_invalid_t, first_invalid_t.data(), num_edges * sizeof(float), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemset(d_next_edge, 0, sizeof(uint32_t)));
    checkCudaMem(cudaMemset(d_overflow, 0, sizeof(unsigned long long)));
    cudaDeviceSynchronize();
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    float duration = 0;
    cudaEventElapsedTime(&duration, start, stop);
    std::cout << "Articulated edge BVH allocation and transfer took " << duration << " ms." << std::endl;

    // ---- SoA parameter bundles + dispatch --------------------------------
    ObstacleSoA<int16_t> obsSoA16;
    ObstacleSoA<int32_t> obsSoA32;
    const auto fillObs = [&](auto* o) {
        o->nodes.R = d_R_obs;
        o->nodes.Rq = d_Obs_Rq;
        o->nodes.Rp = d_Obs_Rp;
        o->nodes.TD = d_Obs_TD;
        o->nodes.T = d_T_obs;
        o->nodes.dim = d_Obs_dim;
        o->nodes.a = d_Obs_a;
        o->num_nodes = obs_BVH.size;
        o->verts = d_Obs_verts;
        o->tris = d_Obs_tris;
    };
    fillObs(&obsSoA16);
    fillObs(&obsSoA32);
    obsSoA16.first_child = d_Obs_first_child16;
    obsSoA32.first_child = d_Obs_first_child32;
    RobotSoA<int16_t> robSoA16;
    RobotSoA<int32_t> robSoA32;
    const auto fillRob = [&](auto* r) {
        r->nodes.R = d_Rob_R;
        r->nodes.Rq = d_Rob_Rq;
        r->nodes.Rp = d_Rob_Rp;
        r->nodes.TD = d_Rob_TD;
        r->nodes.T = d_Rob_T;
        r->nodes.dim = d_Rob_dim;
        r->nodes.a = d_Rob_a;
        r->linkOffset = d_LinkOffset;
        r->linkVertOffset = d_LinkVertOffset;
        r->linkTriOffset = d_LinkTriOffset;
        r->joints = d_Joints;
        r->verts = d_Rob_verts;
        r->tris = d_Rob_tris;
    };
    fillRob(&robSoA16);
    fillRob(&robSoA32);
    robSoA16.first_child = d_Rob_first_child16;
    robSoA32.first_child = d_Rob_first_child32;
    ArticulatedEdgeSoA<N> edgeSoA;
    edgeSoA.s = d_Edge_s;
    edgeSoA.e = d_Edge_e;
    edgeSoA.nsteps = d_Edge_nsteps;
    edgeSoA.flags = d_Edge_flags;
    EdgeKernelOut outSoA;
    outSoA.pdisjoint = pdisjoint;
    outSoA.pfirst_invalid_t = d_first_invalid_t;
    outSoA.g_next_edge = d_next_edge;
    outSoA.overflowCounter = d_overflow;

    const auto doLaunch = [&](auto kernel, const auto& obsS, const auto& robS, int blocks, size_t launchEdges) {
        kernel<<<blocks, blockSize>>>(obsS, robS, edgeSoA, launchEdges, outSoA);
    };
    int blocks_per_sm = 0;
    const auto occForLayout = [&](auto layoutTag) {
        constexpr NodeLayout L = decltype(layoutTag)::value;
        if (rob16 && obs16) {
            checkCudaMem(cudaOccupancyMaxActiveBlocksPerMultiprocessor(&blocks_per_sm, d_bvh_edges_articulated<N, int16_t, int16_t, L>, blockSize, 0));
        } else if (rob16) {
            checkCudaMem(cudaOccupancyMaxActiveBlocksPerMultiprocessor(&blocks_per_sm, d_bvh_edges_articulated<N, int16_t, int32_t, L>, blockSize, 0));
        } else if (obs16) {
            checkCudaMem(cudaOccupancyMaxActiveBlocksPerMultiprocessor(&blocks_per_sm, d_bvh_edges_articulated<N, int32_t, int16_t, L>, blockSize, 0));
        } else {
            checkCudaMem(cudaOccupancyMaxActiveBlocksPerMultiprocessor(&blocks_per_sm, d_bvh_edges_articulated<N, int32_t, int32_t, L>, blockSize, 0));
        }
    };
    switch (layout) {
        case NodeLayout::Matrix: occForLayout(std::integral_constant<NodeLayout, NodeLayout::Matrix>()); break;
        case NodeLayout::Quat:   occForLayout(std::integral_constant<NodeLayout, NodeLayout::Quat>()); break;
        case NodeLayout::VecR:   occForLayout(std::integral_constant<NodeLayout, NodeLayout::VecR>()); break;
        case NodeLayout::QuatTD: occForLayout(std::integral_constant<NodeLayout, NodeLayout::QuatTD>()); break;
        default:                 occForLayout(std::integral_constant<NodeLayout, NodeLayout::QuatSAT>()); break;
    }
    if (blocks_per_sm < 1) blocks_per_sm = 1;
    const int persistent_blocks = num_sms * blocks_per_sm;
    const int gridSize = (max_blocks < (size_t)persistent_blocks) ? (int)max_blocks : persistent_blocks;

    if (dry_run) {
        const size_t dryEdges = (num_edges < (size_t)blockSize) ? num_edges : (size_t)blockSize;
        const auto dry = [&](auto layoutTag) {
            constexpr NodeLayout L = decltype(layoutTag)::value;
            if (rob16 && obs16)       doLaunch(d_bvh_edges_articulated<N, int16_t, int16_t, L>, obsSoA16, robSoA16, 1, dryEdges);
            else if (rob16)           doLaunch(d_bvh_edges_articulated<N, int16_t, int32_t, L>, obsSoA32, robSoA16, 1, dryEdges);
            else if (obs16)           doLaunch(d_bvh_edges_articulated<N, int32_t, int16_t, L>, obsSoA16, robSoA32, 1, dryEdges);
            else                      doLaunch(d_bvh_edges_articulated<N, int32_t, int32_t, L>, obsSoA32, robSoA32, 1, dryEdges);
        };
        switch (layout) {
            case NodeLayout::Matrix: dry(std::integral_constant<NodeLayout, NodeLayout::Matrix>()); break;
            case NodeLayout::Quat:   dry(std::integral_constant<NodeLayout, NodeLayout::Quat>()); break;
            case NodeLayout::VecR:   dry(std::integral_constant<NodeLayout, NodeLayout::VecR>()); break;
            case NodeLayout::QuatTD: dry(std::integral_constant<NodeLayout, NodeLayout::QuatTD>()); break;
            default:                 dry(std::integral_constant<NodeLayout, NodeLayout::QuatSAT>()); break;
        }
        checkCudaMem(cudaGetLastError());
        checkCudaMem(cudaDeviceSynchronize());
        std::cout << "Articulated edge BVH dry run completed successfully." << std::endl;
        checkCudaMem(cudaMemset(d_next_edge, 0, sizeof(uint32_t)));
        checkCudaMem(cudaMemset(pdisjoint, 0, num_words * sizeof(uint32_t)));
        checkCudaMem(cudaMemcpy(d_first_invalid_t, first_invalid_t.data(), num_edges * sizeof(float), cudaMemcpyHostToDevice));
    }

    cudaEventRecord(start, 0);
    const auto launchMain = [&](auto layoutTag) {
        constexpr NodeLayout L = decltype(layoutTag)::value;
        if (rob16 && obs16)       doLaunch(d_bvh_edges_articulated<N, int16_t, int16_t, L>, obsSoA16, robSoA16, gridSize, num_edges);
        else if (rob16)           doLaunch(d_bvh_edges_articulated<N, int16_t, int32_t, L>, obsSoA32, robSoA16, gridSize, num_edges);
        else if (obs16)           doLaunch(d_bvh_edges_articulated<N, int32_t, int16_t, L>, obsSoA16, robSoA32, gridSize, num_edges);
        else                      doLaunch(d_bvh_edges_articulated<N, int32_t, int32_t, L>, obsSoA32, robSoA32, gridSize, num_edges);
    };
    switch (layout) {
        case NodeLayout::Matrix: launchMain(std::integral_constant<NodeLayout, NodeLayout::Matrix>()); break;
        case NodeLayout::Quat:   launchMain(std::integral_constant<NodeLayout, NodeLayout::Quat>()); break;
        case NodeLayout::VecR:   launchMain(std::integral_constant<NodeLayout, NodeLayout::VecR>()); break;
        case NodeLayout::QuatTD: launchMain(std::integral_constant<NodeLayout, NodeLayout::QuatTD>()); break;
        default:                 launchMain(std::integral_constant<NodeLayout, NodeLayout::QuatSAT>()); break;
    }
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&duration, start, stop);
    std::cout << "Articulated edge BVH GPU kernel took " << duration << " ms for "
              << num_edges << " edges." << std::endl;

    unsigned long long h_overflow = 0;
    checkCudaMem(cudaMemcpy(&h_overflow, d_overflow, sizeof(unsigned long long), cudaMemcpyDeviceToHost));
    if (h_overflow != 0) {
        std::cerr << "WARNING: " << h_overflow
                  << " conservative pending-list overflows (potential false positives)" << std::endl;
    }

    std::unique_ptr<uint32_t[]> disjoint(new uint32_t[num_words]);
    checkCudaMem(cudaMemcpy(disjoint.get(), pdisjoint, num_words * sizeof(uint32_t), cudaMemcpyDeviceToHost));
    checkCudaMem(cudaMemcpy(first_invalid_t.data(), d_first_invalid_t, num_edges * sizeof(float), cudaMemcpyDeviceToHost));
    cudaDeviceSynchronize();
    for (size_t i = 0; i < num_edges; ++i) {
        valid[i] = ((disjoint[i >> 5] >> (i & 31)) & 1u) != 0;
    }

    cudaFree(d_R_obs); cudaFree(d_T_obs); cudaFree(d_Obs_dim);
    if (obs16) cudaFree(d_Obs_first_child16); else cudaFree(d_Obs_first_child32);
    cudaFree(d_Obs_verts); cudaFree(d_Obs_tris); cudaFree(d_Obs_a);
    cudaFree(d_Rob_R); cudaFree(d_Rob_T); cudaFree(d_Rob_dim);
    if (rob16) cudaFree(d_Rob_first_child16); else cudaFree(d_Rob_first_child32);
    cudaFree(d_Rob_a); cudaFree(d_Rob_verts); cudaFree(d_Rob_tris);
    cudaFree(d_LinkOffset); cudaFree(d_LinkVertOffset); cudaFree(d_LinkTriOffset);
    cudaFree(d_Joints);
    if (d_Obs_Rq) cudaFree(d_Obs_Rq);
    if (d_Obs_Rp) cudaFree(d_Obs_Rp);
    if (d_Obs_TD) cudaFree(d_Obs_TD);
    if (d_Rob_Rq) cudaFree(d_Rob_Rq);
    if (d_Rob_Rp) cudaFree(d_Rob_Rp);
    if (d_Rob_TD) cudaFree(d_Rob_TD);
    cudaFree(d_Edge_s); cudaFree(d_Edge_e); cudaFree(d_Edge_nsteps); cudaFree(d_Edge_flags);
    cudaFree(pdisjoint); cudaFree(d_first_invalid_t); cudaFree(d_next_edge); cudaFree(d_overflow);

    return duration;
}

// Explicit instantiation for the panda (N=7).
template double bvh_edges_articulated<7>(const std::string&, const BVNode_soa<int32_t>&, const MeshData&,
                                         const ArticulatedEdgeBatch<7>&, std::vector<bool>&,
                                         std::vector<float>&, bool, int);
