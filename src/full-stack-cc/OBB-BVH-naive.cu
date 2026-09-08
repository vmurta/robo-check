#include "OBB-BVH-naive.hu"

//TODO: Make a custom data type for this struct
BVNode_soa BVH_n_ary_hierarchy_from_mesh(const char* mesh_path, size_t power_of_2){
    // Load Robot
    std::vector<fcl::Vector3f> rob_vertices;
    std::vector<fcl::Triangle> rob_triangles;

    loadOBJFileFCL(mesh_path, rob_vertices, rob_triangles);

    // why is this a pointer???
    std::shared_ptr<fcl::BVHModel<fcl::OBB<float>>> rob_mesh(new fcl::BVHModel<fcl::OBB<float>>);
    rob_mesh->beginModel(rob_triangles.size(), rob_vertices.size());
    rob_mesh->addSubModel(rob_vertices, rob_triangles);
    rob_mesh->endModel();

    // Access OBB data from rob_mesh
    // rob_mesh->getNumBVs() gives the number of OBBs in the hierarchy
    size_t num_boxes = rob_mesh->getNumBVs();


    std::vector<size_t> leaf_depths = getBVHTreeDepths(*rob_mesh);
    size_t max_depth = *std::max_element(leaf_depths.begin(), leaf_depths.end());
    size_t reduced_depth = (max_depth + power_of_2 - 1) / power_of_2; // ceiling division to get number of n-ary levels

    std::vector<fcl::OBB<float>> flattened_n_ary;
    std::vector<int16_t> first_children;

    fcl::BVNode<fcl::OBB<float>> focus_node = rob_mesh->getBV(0); // start at root
    size_t parent_depth = 1;
    size_t curr_max_depth = parent_depth + power_of_2; // each n-ary level corresponds to power_of_2 binary levels

    std::vector<std::pair<int, size_t>> curr_frontier;
    curr_frontier.push_back(std::make_pair(0, parent_depth)); // pair of (node index, depth)

    size_t num_dummies = 0;
    size_t num_total_leafs = 0;
    for (size_t i = 0 ; i < reduced_depth; ++i){
        // turn current frontier into a stack for processing
        std::queue<std::pair<int, size_t>> node_queue;

        // push the current frontier nodes into the flattened structure
        for (const auto& pair : curr_frontier){
            focus_node = rob_mesh->getBV(pair.first);
            flattened_n_ary.push_back(focus_node.bv);
        }

        // update the first_children structure
        ////////////////////////////////////////////////////////////////////////////////////////////////
        // prev_first_primitive is used to track primitive index of the previous node in the loop
        // if we see the same primitive index twice in a row, we know it's a dummy node
        
        //TODO: very hacky, do beter at determining which nodes are dummy nodes
        int prev_first_primitive = 80000;
        size_t num_parent_nodes = 0;

        for (const auto& pair : curr_frontier){
            focus_node = rob_mesh->getBV(pair.first);
            if (focus_node.isLeaf()){
                if (prev_first_primitive == focus_node.first_primitive){
                    // this is a dummy node, indicate as such
                    first_children.push_back(0);
                    num_dummies++;
                } else {
                    // real leaf node
                    first_children.push_back(focus_node.first_child); 
                    prev_first_primitive = focus_node.first_primitive;
                    num_total_leafs++;
                }
            } else {
                prev_first_primitive = 80000; // reset

                first_children.push_back(flattened_n_ary.size() + (num_parent_nodes << power_of_2)); // index of first child in flattened structure
                num_parent_nodes++;
                node_queue.push(pair);
            }
        }

        curr_frontier.clear();

        while (!node_queue.empty()){
            auto [node_idx, depth] = node_queue.front();
            node_queue.pop();
            focus_node = rob_mesh->getBV(node_idx);
            if (depth >= curr_max_depth){
                // this node becomes a great^n grandchild
                curr_frontier.push_back(std::make_pair(node_idx, depth));

            } else {
                // expand this node's children
                if (focus_node.isLeaf()){
                    int num_copies = 2;

                    //NOTE: this part seems very error prone, need to check
                    for (int i = 0; i < num_copies; ++i){
                        node_queue.push(std::make_pair(node_idx, depth + 1));
                    }
                } else {
                    int left  = focus_node.leftChild();
                    int right = focus_node.rightChild();
                    node_queue.push(std::make_pair(left, depth + 1));
                    node_queue.push(std::make_pair(right, depth + 1));
                }
            }
        }
        parent_depth = curr_max_depth;
        curr_max_depth += power_of_2;
    }

    // update after the last iteration
    // turn current frontier into a stack for processing
    // push the current frontier nodes into the flattened structure
    for (const auto& pair : curr_frontier){
        focus_node = rob_mesh->getBV(pair.first);
        flattened_n_ary.push_back(focus_node.bv);
    }

    // update the first_children structure
    ////////////////////////////////////////////////////////////////////////////////////////////////
    // prev_first_primitive is used to track primitive index of the previous node in the loop
    // if we see the same primitive index twice in a row, we know it's a dummy node
    int prev_first_primitive = -1;

    for (const auto& pair : curr_frontier){
        //flush the completed fronter nodes into the flattened structure
        focus_node = rob_mesh->getBV(pair.first);
        if (focus_node.isLeaf()){
            if (prev_first_primitive == focus_node.first_primitive){
                first_children.push_back(0); // dummy node
                num_dummies++;
            } else {
                // real leaf node
                first_children.push_back(focus_node.first_child); // indicate leaf with negative numbers
                prev_first_primitive = focus_node.first_primitive;
                num_total_leafs++;
            }
        } else {
            std::cout << "Uh oh, shouldn't have parents anymore :(" <<std::endl;
            prev_first_primitive = -1; // reset

        }
    }

    Eigen::Matrix3f rotation;
    Eigen::Vector3f translation;
    Eigen::Vector3f half_dimensions;
    BVNode_soa result(flattened_n_ary.size());
    for (int i = 0; i < result.size; ++i) {
        fcl::OBB<float> obb = flattened_n_ary[i];
        rotation = obb.axis;
        translation = obb.To;
        half_dimensions = obb.extent;

        result.set(i, rotation, translation, half_dimensions, first_children[i]);
    }

    // delete rob_mesh manually to free memory
    rob_mesh.reset();

    return result;
}


__device__ __forceinline__ bool areTrianglesDisjoint (    uint32_t &num_bad_leaves, uint16_t *bad_rob_leaves, uint16_t *bad_obs_leaves, 
                                        const Eigen::Matrix3f &R_conf, const Eigen::Vector3f &T_conf,
                                        const Eigen::Vector3f *pRob_verts, const Triangle *pRob_tris,
                                        const Eigen::Vector3f *pObs_verts, const Triangle *pObs_tris)

{
    __shared__ bool all_disjoint;

    if (threadIdx.x == 0) {
        all_disjoint = true;
    }
    __syncthreads();

    size_t leaf_idx = threadIdx.x;
    while(leaf_idx < num_bad_leaves){
        int rob_tri_idx = bad_rob_leaves[leaf_idx];
        int obs_tri_idx = bad_obs_leaves[leaf_idx];

        Triangle rob_tri = pRob_tris[rob_tri_idx];
        Triangle obs_tri = pObs_tris[obs_tri_idx];
        Eigen::Vector3f rob_v0 = pRob_verts[rob_tri.v1];
        Eigen::Vector3f rob_v1 = pRob_verts[rob_tri.v2];
        Eigen::Vector3f rob_v2 = pRob_verts[rob_tri.v3];
        Eigen::Vector3f obs_v0 = pObs_verts[obs_tri.v1];
        Eigen::Vector3f obs_v1 = pObs_verts[obs_tri.v2];
        Eigen::Vector3f obs_v2 = pObs_verts[obs_tri.v3];

        // transform robot triangle vertices to world frame
        rob_v0 = R_conf * rob_v0 + T_conf;
        rob_v1 = R_conf * rob_v1 + T_conf;
        rob_v2 = R_conf * rob_v2 + T_conf;

        bool valid = triangles_valid_f(rob_v0, rob_v1, rob_v2, obs_v0, obs_v1, obs_v2);

        if (!valid) {
            all_disjoint = false;
        }
        leaf_idx += blockDim.x;
    }
    __syncthreads();
    return all_disjoint;
}

constexpr int BLOCK_SIZE = 32;

// Nanosecond wall-clock timer (%globaltimer), immune to SM clock throttling.
__device__ __forceinline__ unsigned long long globaltimer() {
    unsigned long long t;
    asm volatile("mov.u64 %0, %%globaltimer;" : "=l"(t));
    return t;
}
//assumes BVH of both trees have same depth // <-- does it?? I think currently it detects when something is a leaf appropriately
__global__ void d_bvh_naive   ( const Eigen::Matrix3f* __restrict__ pR_obs, const Eigen::Vector3f* __restrict__ pT_obs,
                                const Eigen::Matrix3f* __restrict__ pR_rob, const Eigen::Vector3f* __restrict__ pT_rob,
                                const Eigen::Vector3f* __restrict__ pObs_dim, const Eigen::Vector3f* __restrict__ pRob_dim,
                                const Eigen::Matrix3f* __restrict__ pRob_conf_rot, const Eigen::Vector3f* __restrict__ pRob_conf_trans,
                                const int16_t* __restrict__ pObs_first_child, const int16_t* __restrict__ pRob_first_child,
                                const Eigen::Vector3f * __restrict__ pRob_verts, const Triangle * __restrict__ pRob_tris, size_t num_rob_nodes,
                                const Eigen::Vector3f * __restrict__ pObs_verts, const Triangle * __restrict__ pObs_tris, size_t num_obs_nodes,
                                uint32_t* __restrict__ pdisjoint, size_t num_confs, uint32_t* __restrict__ g_next_conf,
                                unsigned long long* __restrict__ d_phase) {

    if (num_obs_nodes == 0) {
        printf("Error: num_obs_nodes is zero. Exiting kernel.\n");
        return;
    }

    extern __shared__ char shared_mem[];

    size_t smem_offset = 0;
    int16_t* sObs_first_child = reinterpret_cast<int16_t*>(shared_mem + smem_offset);
    smem_offset += num_obs_nodes * sizeof(int16_t);
    int16_t* sRob_first_child = reinterpret_cast<int16_t*>(shared_mem + smem_offset);

    // Load the first_child arrays into shared memory once per block; they are
    // read-only for the lifetime of the kernel.
    for (uint16_t i = threadIdx.x; i < num_rob_nodes; i += blockDim.x){
        sRob_first_child[i] = pRob_first_child[i];
    }
    for (uint16_t i = threadIdx.x; i < num_obs_nodes; i += blockDim.x){
        sObs_first_child[i] = pObs_first_child[i];
    }
    __syncthreads();

    Eigen::Matrix3f R_obs_abs_root = pR_obs[0]; // rotation of B wrt origin
    Eigen::Vector3f T_obs_abs_root = pT_obs[0]; // translation of B wrt origin

    Eigen::Matrix3f R_rob_abs_root = pR_rob[0]; // rotation of A wrt origin
    Eigen::Vector3f T_rob_abs_root = pT_rob[0]; // translation of A wrt origin

    Eigen::Vector3f b_root = pRob_dim[0]; // half dimensions of box A
    Eigen::Vector3f a_root = pObs_dim[0]; // half dimensions of box B

    const float epsilon = 1e-6f; // small value to avoid numerical issues

    Eigen::Matrix3f R_conf; // rotation of robot wrt world
    Eigen::Vector3f T_conf; // translation of robot wrt world
    Eigen::Matrix3f R_obs_abs; // rotation of B wrt origin (per node)
    Eigen::Vector3f T_obs_abs;
    Eigen::Matrix3f R_rob_abs;
    Eigen::Vector3f T_rob_abs;
    Eigen::Vector3f b; // half dimensions of box A (per node)
    Eigen::Vector3f a; // half dimensions of box B (per node)
    Eigen::Matrix3f B;      // rotation of A wrt B
    Eigen::Matrix3f Bf;     // absolute value of B (plus epsilon)
    Eigen::Vector3f T;      // translation of A wrt B

    // intent: for each i in rob_obb_pend, need to check all children of rob_obb_pend[i] against all children of obs_obb_pend[j]
    // maybe should do 32 * num layers, back of envelope says that should be an upper limit
    constexpr int MAX_BUFFER = BLOCK_SIZE * 128;
    __shared__ uint16_t rob_obb_pend[MAX_BUFFER]; // arbitrary buffer size, should experiment with this
    __shared__ uint16_t obs_obb_pend[MAX_BUFFER];
    __shared__ int num_obb_pend;

    // intent: for each i in rob_obb_pend, need to check triangle of bad_rob_leaves[i] against triangle of bad_obs_leaves[j]
    __shared__ uint16_t bad_rob_leaves[MAX_BUFFER];
    __shared__ uint16_t bad_obs_leaves[MAX_BUFFER];
    __shared__ uint32_t num_bad_leaves;
    // no obstacles need further testing

    //TODO: need failsafe if this overflows

    constexpr int BATCH = BLOCK_SIZE; // configs pulled per queue transaction (one per thread)
    __shared__ uint32_t s_batch_start;
    __shared__ uint32_t s_num_pend;
    __shared__ Eigen::Matrix3f s_pend_rot[BATCH];
    __shared__ Eigen::Vector3f s_pend_trans[BATCH];
    __shared__ uint32_t s_pend_idx[BATCH];

    // Bitpacked result word for this batch: one bit per config. The batch is
    // always 32-aligned (BATCH == BLOCK_SIZE), so an entire batch maps to a
    // single uint32 word owned exclusively by this block; thread 0 flushes it.
    __shared__ uint32_t s_disjoint_word;

    int16_t conf_offset = (threadIdx.x >> 4)-2; // divide by 16 to see if thread works on the 0th pair or 1st pair of pending boxes
    int16_t rob_child_idx = (threadIdx.x >> 2) & 0x3; // divide by 4, then mod by 4to see which child of the robot box this thread is assigned to
    int16_t obs_child_idx = threadIdx.x & 0x3; // mod 4 to see which child of the obstacle box this thread is assigned to

    // Persistent-block work queue: each block pulls a batch of configuration
    // indices from a global atomic counter until the queue is exhausted. The
    // batch's outermost OBB check runs in parallel (one config per thread),
    // then the block traverses the pending configs serially. Dynamic pulls
    // balance load across blocks instead of static contiguous chunking.
    //
    // Phase timers (profiling): thread 0 accumulates %globaltimer deltas for the
    // block-serial phases into d_phase[0..2] (pull+initial check, OBB
    // traversal, triangle tests).
    unsigned long long acc_init = 0, acc_trav = 0, acc_tri = 0;
    while (true) {
        __syncthreads();
        const unsigned long long t_phase0 = (threadIdx.x == 0) ? globaltimer() : 0;
        if (threadIdx.x == 0) {
            s_batch_start = atomicAdd(g_next_conf, BATCH);
            s_num_pend = 0;
            s_disjoint_word = 0;
        }
        __syncthreads();
        const uint32_t batch_start = s_batch_start;
        if (batch_start >= num_confs) {
            if (threadIdx.x == 0) {
                atomicAdd(&d_phase[0], acc_init);
                atomicAdd(&d_phase[1], acc_trav);
                atomicAdd(&d_phase[2], acc_tri);
            }
            return;
        }

        // parallel outermost OBB check: one config per thread
        const uint32_t index = batch_start + threadIdx.x;
        if (index < num_confs) {
            R_conf = pRob_conf_rot[index];
            T_conf = pRob_conf_trans[index];

            //Calculate relative rotation of B wrt A
            //TODO: precompute inverse rotations of A
            computeRelTransform(R_obs_abs_root, T_obs_abs_root,
                                R_rob_abs_root, T_rob_abs_root,
                                R_conf, T_conf, epsilon, B, Bf, T);

            //initial per conf outermost bounding box check
            if (!obbOverlap(a_root, b_root, B, Bf, T)) {
                atomicOr(&s_disjoint_word, 1u << threadIdx.x);
            } else {
                uint32_t pos = atomicAdd(&s_num_pend, 1);
                s_pend_rot[pos] = R_conf;
                s_pend_trans[pos] = T_conf;
                s_pend_idx[pos] = index;
            }
        } // end outermost OBB check
        __syncthreads();

        // Flush the bitpacked results of the outermost check before any early
        // continue: bits are only ever set, so re-ORing below is idempotent.
        if (threadIdx.x == 0) {
            pdisjoint[batch_start >> 5] |= s_disjoint_word;
        }

        // profiling
        if (threadIdx.x == 0) {
            acc_init += globaltimer() - t_phase0;
        }

        if (s_num_pend == 0) {
            continue;
        }

        for (uint32_t i = 0; i < s_num_pend; i++) {
            __syncthreads();
            const unsigned long long t_cfg = (threadIdx.x == 0) ? globaltimer() : 0;
            R_conf = s_pend_rot[i];
            T_conf = s_pend_trans[i];
            const uint32_t index = s_pend_idx[i];

            //reset pending OBB lists
            if (threadIdx.x == 0){
                num_obb_pend = 1;
                num_bad_leaves = 0;
                rob_obb_pend[0] = 0; // root
                obs_obb_pend[0] = 0; // root
            }

            __syncthreads();

            // intent: for each i in rob_obb_pend, need to check all children of rob_obb_pend[i] against all children of obs_obb_pend[j]
            // these should be 4-ary trees, meaning we grab up to two pending boxes at a time so that we have 16 threads doing the children 
            // of each pair of boxes. 
            bool collision_found = false;
            while(true){
                __syncthreads();
                if (num_obb_pend == 0){
                    break;
                }

                int pend_idx = num_obb_pend + conf_offset;
                if (num_obb_pend >= MAX_BUFFER - 32){
                    if (threadIdx.x == 0) {
                        printf("obb overflow with %d\n boxes on configuration with rotation matrix \n\r \
                                %f, %f, %f, \n %f, %f, %f, \n %f, %f, %f, \n and translation \n \n with block index %d\n \
                                %f, %f, %f \n", 
                                num_obb_pend, R_conf(0, 0), R_conf(0, 1), R_conf(0, 2),
                                R_conf(1, 0), R_conf(1, 1), R_conf(1, 2),
                                R_conf(2, 0), R_conf(2, 1), R_conf(2, 2),
                                T_conf[0], T_conf[1], T_conf[2], blockIdx.x);
                    }
                    break;
                }
                if (num_bad_leaves >= MAX_BUFFER - 32){
                    if (!areTrianglesDisjoint ( num_bad_leaves, bad_rob_leaves, bad_obs_leaves,
                                    R_conf, T_conf,
                                    pRob_verts,   pRob_tris, 
                                    pObs_verts,   pObs_tris)) {
                        collision_found = true;
                        break;
                    }
                    else if (threadIdx.x == 0) {
                        num_bad_leaves = 0;
                    }  
                }

                __syncthreads();

                if(threadIdx.x == 0){
                    if (num_obb_pend > 1) {num_obb_pend -= 2;} 
                    else {num_obb_pend = 0;}
                }

                __syncthreads();
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

                R_obs_abs = pR_obs[obs_obb_idx];
                T_obs_abs = pT_obs[obs_obb_idx];
                R_rob_abs = pR_rob[rob_obb_idx];
                T_rob_abs = pT_rob[rob_obb_idx];
                b = pRob_dim[rob_obb_idx];
                a = pObs_dim[obs_obb_idx];

                computeRelTransform(R_obs_abs, T_obs_abs, R_rob_abs, T_rob_abs, R_conf, T_conf, epsilon, B, Bf, T);

                if (!obbOverlap(a, b, B, Bf, T)) {
                    continue;
                }

                if (rob_first_child_idx > 0 && obs_first_child_idx > 0) {
                    // add children to pending lists
                    int pos = atomicAdd(&num_obb_pend, 1);
                    obs_obb_pend[pos] = obs_obb_idx;
                    rob_obb_pend[pos] = rob_obb_idx;
                } 

                //TODO: this is causing duplicates to be added to the bad leaves list, causing extra work down the line, as the duplicates grow exponentially.
                // this is caused by the fact that when one box is a leaf and the other is not, we add the parent of the leaf box back into the pending list, which can cause the same parent to be added multiple times for the same non-leaf box if multiple children of the non-leaf box overlap with the any of the leaf boxes of the parent

                else if (rob_first_child_idx < 0) {
                    // both are leaves
                    if (obs_first_child_idx < 0) {

                        // add leaves to bad leaves list
                        int pos = atomicAdd(&num_bad_leaves, 1);

                        int rob_tri_idx = -1 * (rob_first_child_idx + 1); 
                        int obs_tri_idx = -1 * (obs_first_child_idx + 1); //TODO: maybe consider having a leaf step instead of doing redundant checks
                        
                        bad_rob_leaves[pos] = rob_tri_idx; 
                        bad_obs_leaves[pos] = obs_tri_idx; //TODO: maybe consider having a leaf step instead of doing redundant checks
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
            } // end while true over single configuration

            __syncthreads();
            const unsigned long long t_tri = (threadIdx.x == 0) ? globaltimer() : 0;
            if (threadIdx.x == 0) {
                acc_trav += t_tri - t_cfg;
            }
            if (collision_found) {
                continue;
            }
            if (areTrianglesDisjoint ( num_bad_leaves, bad_rob_leaves, bad_obs_leaves,
                                    R_conf, T_conf,
                                    pRob_verts,   pRob_tris, 
                                    pObs_verts,   pObs_tris)) {
                if (threadIdx.x == 0) {
                    s_disjoint_word |= 1u << (index & 31);
                }
            }
            __syncthreads();
            if (threadIdx.x == 0) {
                acc_tri += globaltimer() - t_tri;
            }
        } // end for over pending configurations in batch

        // Flush the batch's bitpacked word once (one 4-byte write instead of
        // 32 threads storing the same byte).
        if (threadIdx.x == 0) {
            pdisjoint[batch_start >> 5] |= s_disjoint_word;
        }
    }
    return;
}

double bvh_naive(const BVNode_soa& rob_BVH, const BVNode_soa& obs_BVH,
                 const MeshData& rob_mesh, const MeshData& obs_mesh,
                 const std::vector<Configuration>& confs,
                 std::vector<bool>& valid, bool dry_run) {
    cudaEvent_t start, stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);

    std::cout << "Rob BVH size: " << rob_BVH.size << std::endl;
    std::cout << "Obs BVH size: " << obs_BVH.size << std::endl;

    const int num_confs = confs.size();
    const size_t num_words = (num_confs + 31) / 32;
    valid.assign(num_confs, false);
    if (num_confs == 0) {
        return 0.0;
    }

    // Bitpacked results: bit i of disjoint[i >> 5] is the result of config i.
    std::unique_ptr<uint32_t[]> disjoint(new uint32_t[num_words]());


    std::vector<Eigen::Matrix3f> rob_conf_r(num_confs);
    std::vector<Eigen::Vector3f> rob_conf_t(num_confs);
    for (int i = 0; i < num_confs; ++i) {
        rob_conf_r[i] = createRotationMatrix(confs[i]);
        rob_conf_t[i] = Eigen::Vector3f(confs[i].x, confs[i].y, confs[i].z);
    }

    const int blockSize = 32;
    // Persistent blocks pulling configs from a global work queue. Size the grid
    // so every SM is fully occupied (the kernel fits multiple blocks per SM).
    int device;
    cudaGetDevice(&device);
    int num_sms = 0;
    cudaDeviceGetAttribute(&num_sms, cudaDevAttrMultiProcessorCount, device);

    // Dynamic shared memory: just the two first_child arrays (read-only work set).
    const size_t smem_size = (obs_BVH.size + rob_BVH.size) * sizeof(int16_t);

    int blocks_per_sm = 0;
    cudaOccupancyMaxActiveBlocksPerMultiprocessor(&blocks_per_sm, d_bvh_naive, blockSize, smem_size);
    if (blocks_per_sm < 1) blocks_per_sm = 1;
    const int max_blocks = (num_confs + blockSize - 1) / blockSize;
    const int persistent_blocks = num_sms * blocks_per_sm;
    const int gridSize = (max_blocks < persistent_blocks) ? max_blocks : persistent_blocks;


    Eigen::Matrix3f* d_R_obs;
    Eigen::Vector3f* d_T_obs;
    Eigen::Matrix3f* d_R_rob;
    Eigen::Vector3f* d_T_rob;
    Eigen::Vector3f* d_Rob_dim;
    Eigen::Vector3f* d_Obs_dim;
    Eigen::Matrix3f* d_Rob_conf_rot;
    Eigen::Vector3f* d_Rob_conf_trans;
    Eigen::Vector3f* d_Rob_vertices;
    Eigen::Vector3f* d_Obs_vertices;
    Triangle * d_Rob_triangles;
    Triangle * d_Obs_triangles;
    int16_t* d_Obs_first_child;
    int16_t* d_Rob_first_child;
    uint32_t* pdisjoint;
    uint32_t* d_next_conf;
    uint64_t* d_phase;

    // Allocate memory for device pointers
    cudaEventRecord(start, 0);
    cudaMalloc((void**)&d_R_obs, obs_BVH.size * sizeof(Eigen::Matrix3f));
    cudaMalloc((void**)&d_T_obs, obs_BVH.size * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_R_rob, rob_BVH.size * sizeof(Eigen::Matrix3f));
    cudaMalloc((void**)&d_T_rob, rob_BVH.size * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Rob_dim, rob_BVH.size * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Obs_dim, obs_BVH.size * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Rob_conf_rot, num_confs * sizeof(Eigen::Matrix3f));
    cudaMalloc((void**)&d_Rob_conf_trans, num_confs * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Obs_first_child, obs_BVH.size * sizeof(int16_t));
    cudaMalloc((void**)&d_Rob_first_child, rob_BVH.size * sizeof(int16_t));
    cudaMalloc((void**)&d_Rob_vertices, rob_mesh.vertices.size() * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Obs_vertices, obs_mesh.vertices.size() * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Rob_triangles, rob_mesh.triangles.size() * sizeof(Triangle));
    cudaMalloc((void**)&d_Obs_triangles, obs_mesh.triangles.size() * sizeof(Triangle));
    cudaMalloc((void**)&pdisjoint, num_words * sizeof(uint32_t));
    cudaMalloc((void**)&d_next_conf, sizeof(uint32_t));
    checkCudaMem(cudaMemset(d_next_conf, 0, sizeof(uint32_t)));
    cudaMalloc((void**)&d_phase, 3 * sizeof(uint64_t));
    checkCudaMem(cudaMemset(d_phase, 0, 3 * sizeof(uint64_t)));

    cudaDeviceSynchronize();
    checkCudaMem(cudaMemcpy(d_R_obs, obs_BVH.pR, obs_BVH.size * sizeof(Eigen::Matrix3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_T_obs, obs_BVH.pT, obs_BVH.size * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_R_rob, rob_BVH.pR, rob_BVH.size * sizeof(Eigen::Matrix3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_T_rob, rob_BVH.pT, rob_BVH.size * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_dim, rob_BVH.pDim, rob_BVH.size * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_dim, obs_BVH.pDim, obs_BVH.size * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_first_child, rob_BVH.first_child, rob_BVH.size * sizeof(int16_t), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_first_child, obs_BVH.first_child, obs_BVH.size * sizeof(int16_t), cudaMemcpyHostToDevice));
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

    // Copy configurations over separately
    cudaEventRecord(start, 0);
    checkCudaMem(cudaMemcpy(d_Rob_conf_rot, rob_conf_r.data(), num_confs * sizeof(Eigen::Matrix3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_conf_trans, rob_conf_t.data(), num_confs * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    cudaDeviceSynchronize();
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&duration, start, stop);
    std::cout << "Copying configurations to GPU took " << duration << " ms." << std::endl;

    std::cout << "obs_BVH.size: " << obs_BVH.size << ", rob_BVH.size: " << rob_BVH.size << std::endl;

    auto launch_bvh_naive = [&]() {
        d_bvh_naive<<<gridSize, blockSize, smem_size>>>(
                                                d_R_obs, d_T_obs,
                                                d_R_rob, d_T_rob,
                                                d_Obs_dim, d_Rob_dim,
                                                d_Rob_conf_rot, d_Rob_conf_trans,
                                                d_Obs_first_child, d_Rob_first_child,
                                                d_Rob_vertices, d_Rob_triangles, rob_BVH.size,
                                                d_Obs_vertices, d_Obs_triangles, obs_BVH.size,
                                                pdisjoint, static_cast<size_t>(num_confs), d_next_conf, (unsigned long long*)d_phase);
    };

    // Dry run: single-block launch over one batch, untimed, purely to get the
    // kernel loaded onto the device (the work queue would otherwise drain the
    // entire workload in this block).
    if (dry_run) {
        const size_t dry_confs = (num_confs < (size_t)blockSize) ? num_confs : (size_t)blockSize;
        d_bvh_naive<<<1, blockSize, smem_size>>>(
                                                d_R_obs, d_T_obs,
                                                d_R_rob, d_T_rob,
                                                d_Obs_dim, d_Rob_dim,
                                                d_Rob_conf_rot, d_Rob_conf_trans,
                                                d_Obs_first_child, d_Rob_first_child,
                                                d_Rob_vertices, d_Rob_triangles, rob_BVH.size,
                                                d_Obs_vertices, d_Obs_triangles, obs_BVH.size,
                                                pdisjoint, dry_confs, d_next_conf, (unsigned long long*)d_phase);
        checkCudaMem(cudaGetLastError());
        checkCudaMem(cudaDeviceSynchronize());
        std::cout << "BVH Naive dry run completed successfully." << std::endl;
        // Reset the work queue so the timed launch starts from configuration 0
        checkCudaMem(cudaMemset(d_next_conf, 0, sizeof(uint32_t)));
        checkCudaMem(cudaMemset(d_phase, 0, 3 * sizeof(uint64_t)));
        checkCudaMem(cudaMemset(pdisjoint, 0, num_words * sizeof(uint32_t)));
    }

    cudaEventRecord(start, 0);
    launch_bvh_naive();
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&duration, start, stop);
    std::cout << "BVH Naive GPU kernel took " << duration << " ms for " << num_confs << " configurations." << std::endl;

    // Profiling: aggregate block-serial phase times (nanoseconds)
    uint64_t h_phase[3];
    checkCudaMem(cudaMemcpy(h_phase, d_phase, 3 * sizeof(uint64_t), cudaMemcpyDeviceToHost));
    const double ns_per_ms = 1e6;
    std::cout << "PHASES ms: init=" << (double)h_phase[0] / ns_per_ms
              << " traversal=" << (double)h_phase[1] / ns_per_ms
              << " triangles=" << (double)h_phase[2] / ns_per_ms << std::endl;

    // Copy result back to host (num_confs * sizeof(bool))
    //TODO deleteme
    for (int i = 0; i < 10000000; ++i) {
        i++;
    }
    cudaEventRecord(start, 0);
    checkCudaMem(cudaGetLastError());

    checkCudaMem(cudaMemcpy(disjoint.get(), pdisjoint, num_words * sizeof(uint32_t), cudaMemcpyDeviceToHost));
    cudaDeviceSynchronize();
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&duration, start, stop);
    std::cout << "Copying results from GPU took " << duration << " ms." << std::endl;

    for (int i = 0; i < num_confs; ++i) {
        valid[i] = (disjoint[i >> 5] >> (i & 31)) & 1u;
    }

    // Free device memory
    cudaFree(d_R_obs);
    cudaFree(d_T_obs);
    cudaFree(d_R_rob);
    cudaFree(d_T_rob);
    cudaFree(d_Rob_dim);
    cudaFree(d_Obs_dim);
    cudaFree(d_Rob_conf_rot);
    cudaFree(d_Rob_conf_trans);
    cudaFree(d_Obs_first_child);
    cudaFree(d_Rob_first_child);
    cudaFree(d_Rob_vertices);
    cudaFree(d_Obs_vertices);
    cudaFree(d_Rob_triangles);
    cudaFree(d_Obs_triangles);
    cudaFree(pdisjoint);
    cudaFree(d_next_conf);
    cudaFree(d_phase);

    return duration;
}
