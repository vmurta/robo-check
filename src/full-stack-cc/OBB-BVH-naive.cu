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


__device__ bool areTrianglesDisjoint (    uint32_t &num_bad_leaves, uint16_t *bad_rob_leaves, uint16_t *bad_obs_leaves, 
                                        const Eigen::Matrix3f &R_conf, const Eigen::Vector3f &T_conf,
                                        const Eigen::Vector3f *pRob_verts, const Triangle *pRob_tris,
                                        const Eigen::Vector3f *pObs_verts, const Triangle *pObs_tris)

{
    //TODO: go through leaves collaboratively with a queue instead of this
    //TODO: this doesn't actually work if there are more than BLOCK_SIZE bad leaves, need to add some sort of batching mechanism
    bool valid = true;
    size_t leaf_idx = threadIdx.x;
    bool all_valid = true;
    __syncthreads();

    while(leaf_idx < num_bad_leaves){
        unsigned mask = __activemask();
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

        bool valid = triangles_valid(rob_v0, rob_v1, rob_v2, obs_v0, obs_v1, obs_v2);

        if (!__all_sync(mask, valid)) {   
            all_valid = false;
            break;
        }
        leaf_idx += blockDim.x;
    }
    __syncthreads();
    return all_valid;
}

#define BLOCK_SIZE 32
//assumes BVH of both trees have same depth // <-- does it?? I think currently it detects when something is a leaf appropriately
__global__ void d_bvh_naive   ( const Eigen::Matrix3f* pR_obs, const Eigen::Vector3f* pT_obs,
                                const Eigen::Matrix3f* pR_rob, const Eigen::Vector3f* pT_rob,
                                const Eigen::Vector3f* pObs_dim, const Eigen::Vector3f* pRob_dim,
                                const Eigen::Matrix3f* pRob_conf_rot, const Eigen::Vector3f* pRob_conf_trans,
                                const int16_t* pObs_first_child, const int16_t* pRob_first_child,
                                const Eigen::Vector3f *pRob_verts, const Triangle *pRob_tris, size_t num_rob_nodes,
                                const Eigen::Vector3f *pObs_verts, const Triangle *pObs_tris, size_t num_obs_nodes,
                                bool* pdisjoint) {

    // extern __shared__      
    if (num_obs_nodes == 0) {
        printf("Error: num_obs_nodes is zero. Exiting kernel.\n");
        return;
    }                               
    size_t index = blockIdx.x * blockDim.x + threadIdx.x;

    extern __shared__ int16_t shared_mem[];
    // reinterpret shared memory as arrays of the appropriate typesintint16_t16_t
    int16_t* sObs_first_child = shared_mem;
    int16_t* sRob_first_child = sObs_first_child + num_obs_nodes;

    Eigen::Matrix3f R_obs_abs = pR_obs[0]; // rotation of B wrt origin
    Eigen::Vector3f T_obs_abs = pT_obs[0]; // translation of B wrt origin

    Eigen::Matrix3f R_rob_abs = pR_rob[0]; // rotation of A wrt origin
    Eigen::Vector3f T_rob_abs = pT_rob[0]; // translation of A wrt origin

    Eigen::Vector3f b = pRob_dim[0]; // half dimensions of box A
    Eigen::Vector3f a = pObs_dim[0]; // half dimensions of box B

    Eigen::Matrix3f R_conf = pRob_conf_rot[index]; // rotation of robot wrt world
    Eigen::Vector3f T_conf = pRob_conf_trans[index]; // translation of robot wrt world

    __syncthreads();

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
    __shared__ uint32_t num_confs_pend;
    if (threadIdx.x == 0){
        num_confs_pend = 0;
    }

    //TODO: need failsafe if this overflows
    __shared__ Eigen::Matrix3f rob_confs_pend_rot[BLOCK_SIZE];
    __shared__ Eigen::Vector3f rob_confs_pend_trans[BLOCK_SIZE];
    __shared__ uint32_t rob_confs_pend_indices[BLOCK_SIZE];

    __syncthreads();


    //initial per conf outermost bounding box check
    while (true) {

        t = fabsf(T[0]);

        //Since L = A0 is a unit vector (as it is the cross product of unit vectors), no need to multiply t
        // t dot L = t
        // \sum |a_i A^i * L | = a_0 + 0 + 0
        // \sum |b_i B^i * L | = b_i B^i * A0 = first element of each column vector of Bf = Bf.row(0).dot(b)

        // Test #1
        if(t > (a[0] + Bf.row(0).dot(b))){
            pdisjoint[index] = true;
            break;
        }

        // Test #2
        // B1 x B2 = B0
        t = fabsf(B.col(0).dot(T));

        if(t > (b[0] + Bf.col(0).dot(a))){
            pdisjoint[index] = true;
            break;
        }

        // Test #3
        // A2 x A0 = A1
        t = fabsf(T[1]);

        if(t > (a[1] + Bf.row(1).dot(b))){
            pdisjoint[index] = true;
            break;
        }

        // Test #4
        // A0 x A1 = A2
        t =fabsf(T[2]);

        if(t > (a[2] + Bf.row(2).dot(b))){
            pdisjoint[index] = true;
            break;
        }

        // Test #5
        // B2 x B0 = B1
        t = fabsf(B.col(1).dot(T));

        if(t > (b[1] + Bf.col(1).dot(a))){
            pdisjoint[index] = true;
            break;
        }

        // Test #6
        // B0 x B1 = B2
        t = fabsf(B.col(2).dot(T));

        if(t > (b[2] + Bf.col(2).dot(a))){
            pdisjoint[index] = true;
            break;
        }

        // Test #7
        // A0 x B0
        t = fabsf(T[2] * B(1, 0) - T[1] * B(2, 0));

        if(t > (a[1] * Bf(2, 0) + a[2] * Bf(1, 0) +
                b[1] * Bf(0, 2) + b[2] * Bf(0, 1))){
            pdisjoint[index] = true;
            break;
        }

        // Test #8
        // A0 x B1
        t = fabsf(T[2] * B(1, 1) - T[1] * B(2, 1));

        if(t > (a[1] * Bf(2, 1) + a[2] * Bf(1, 1) +
                b[0] * Bf(0, 2) + b[2] * Bf(0, 0))){
            pdisjoint[index] = true;
            break;
        }

        // Test #9
        // A0 x B2
        t = fabsf(T[2] * B(1, 2) - T[1] * B(2, 2));

        if(t > (a[1] * Bf(2, 2) + a[2] * Bf(1, 2) +
                b[0] * Bf(0, 1) + b[1] * Bf(0, 0))){
            pdisjoint[index] = true;
            break;
        }

        // Test #10
        // A1 x B0
        t = fabsf(T[0] * B(2, 0) - T[2] * B(0, 0));

        if(t > (a[0] * Bf(2, 0) + a[2] * Bf(0, 0) +
                b[1] * Bf(1, 2) + b[2] * Bf(1, 1))){
            pdisjoint[index] = true;
            break;
        }

        // Test #11
        // A1 x B1
        t = fabsf(T[0] * B(2, 1) - T[2] * B(0, 1));

        if(t > (a[0] * Bf(2, 1) + a[2] * Bf(0, 1) +
                b[0] * Bf(1, 2) + b[2] * Bf(1, 0))){
            pdisjoint[index] = true;
            break;
        }

        // Test #12
        // A1 x B2
        t = fabsf(T[0] * B(2, 2) - T[2] * B(0, 2));

        if(t > (a[0] * Bf(2, 2) + a[2] * Bf(0, 2) +
                b[0] * Bf(1, 1) + b[1] * Bf(1, 0))){
            pdisjoint[index] = true;
            break;
        }

        // Test #13
        // A2 x B0
        t = fabsf(T[1] * B(0, 0) - T[0] * B(1, 0));

        if(t > (a[0] * Bf(1, 0) + a[1] * Bf(0, 0) +
                b[1] * Bf(2, 2) + b[2] * Bf(2, 1))){
            pdisjoint[index] = true;
            break;
        }

        // Test #14
        // A2 x B1
        t = fabsf(T[1] * B(0, 1) - T[0] * B(1, 1));

        if(t > (a[0] * Bf(1, 1) + a[1] * Bf(0, 1) +
                b[0] * Bf(2, 2) + b[2] * Bf(2, 0))){
            pdisjoint[index] = true;
            break;
        }

        // Test #15
        // A2 x B2
        t = fabsf(T[1] * B(0, 2) - T[0] * B(1, 2));

        if(t > (a[0] * Bf(1, 2) + a[1] * Bf(0, 2) +
                b[0] * Bf(2, 1) + b[1] * Bf(2, 0))){
            pdisjoint[index] = true;
            break;
        }
        uint_fast16_t pos = atomicAdd(&num_confs_pend, 1);
        rob_confs_pend_rot[pos] = R_conf;
        rob_confs_pend_trans[pos] = T_conf;
        rob_confs_pend_indices[pos] = index;
        break;
    }
    __syncthreads();

    if (num_confs_pend == 0){
        // no pending configurations, we're done
        return;
    }

    // __shared__ int16_t rob_first_child[num_rob_nodes];
    // __shared__ int16_t obs_first_child[num_obs_nodes];

    for (uint16_t i = threadIdx.x; i < num_rob_nodes; i += BLOCK_SIZE){
        sRob_first_child[i] = pRob_first_child[i];
    }
    for (uint16_t i = threadIdx.x; i < num_obs_nodes; i += BLOCK_SIZE){
        sObs_first_child[i] = pObs_first_child[i];
    }

    __syncthreads();

        //todo: deleteme
    // for (uint16_t i = 0; i < num_rob_nodes; i++){
    //    if (threadIdx.x == 0){
    //         printf("Rob node %d has first child %d\n", i, sRob_first_child[i]);
    //    }
    // }

    //TODO: delete this? it doesn't seem to do anything?
    // if (threadIdx.x == 0){
    //     //check that the number of nodes with first_child < 0
    //     uint16_t rob_leaf_count = 0;
    //     for (uint16_t i = 0; i < num_rob_nodes; i++){
    //         if (sRob_first_child[i] < 0){
    //             rob_leaf_count++;
    //         }
    //     }
    //     uint16_t obs_leaf_count = 0;
    //     for (uint16_t i = 0; i < num_obs_nodes; i++){
    //         if (sObs_first_child[i] < 0){
    //             obs_leaf_count++;
    //         }
    //     }
    // }

    // intent: for each i in rob_obb_pend, need to check all children of rob_obb_pend[i] against all children of obs_obb_pend[j]

    // maybe should do 32 * num layers, back of envelope says that should be an upper limit
    #define MAX_BUFFER (BLOCK_SIZE * 128)
    __shared__ uint16_t rob_obb_pend[MAX_BUFFER]; // arbitrary buffer size, should experiment with this
    __shared__ uint16_t obs_obb_pend[MAX_BUFFER];
    __shared__ int num_obb_pend;

    // intent: for each i in rob_obb_pend, need to check triangle of bad_rob_leaves[i] against triangle of bad_obs_leaves[j]
    __shared__ uint16_t bad_rob_leaves[MAX_BUFFER];
    __shared__ uint16_t bad_obs_leaves[MAX_BUFFER];
    __shared__ uint32_t num_bad_leaves;
    // no obstacles need further testing

    //TODO: need failsafe if this overflows

    int16_t conf_offset = (threadIdx.x >> 4)-2; // divide by 16 to see if thread works on the 0th pair or 1st pair of pending boxes
    int16_t rob_child_idx = (threadIdx.x >> 2) & 0x3; // divide by 4, then mod by 4to see which child of the robot box this thread is assigned to
    int16_t obs_child_idx = threadIdx.x & 0x3; // mod 4 to see which child of the obstacle box this thread is assigned to
    for (uint8_t i = 0; i < num_confs_pend; i++){
        __syncthreads();
        R_conf = rob_confs_pend_rot[i];
        T_conf = rob_confs_pend_trans[i];
        if (num_confs_pend > 255){
            printf("Error, too many confs\n");
        }
        int global_conf_idx = rob_confs_pend_indices[i];

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
                // if (threadIdx.x == 0) {
                //     printf("bad leaves overflow with %d\n boxes on configuration with rotation matrix \n\r \
                //             %f, %f, %f, \n %f, %f, %f, \n %f, %f, %f, \n and translation \n \n with block index %d\n \
                //             %f, %f, %f \n", 
                //             num_bad_leaves, R_conf(0, 0), R_conf(0, 1), R_conf(0, 2),
                //             R_conf(1, 0), R_conf(1, 1), R_conf(1, 2),
                //             R_conf(2, 0), R_conf(2, 1), R_conf(2, 2),
                //             T_conf[0], T_conf[1], T_conf[2], blockIdx.x);
                // }
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

            // if (rob_obb_par_idx > 0) {
            //     if (rob_child_idx + obs_child_idx == 0 ) {
            //         printf("Error: rob_obb_par_idx is %d for pend_idx %d on Iteration %d of conf %d for thread %d on block %d. with pend_idx %d FATAL\n", rob_obb_par_idx, pend_idx, delete_me_num_iter, global_conf_idx, threadIdx.x, blockIdx.x, pend_idx);
            //     }
            // }
            int obs_obb_par_idx = obs_obb_pend[pend_idx];
            
            // if (rob_child_idx + obs_child_idx == 0 ) {
            //     printf("rob_obb_par_idx is %d and obs_obb_par_idx is %d for pend_idx %d on Iteration %d of conf %d for thread %d on block %d. with pend_idx %d\n", rob_obb_par_idx, obs_obb_par_idx, pend_idx, delete_me_num_iter, global_conf_idx, threadIdx.x, blockIdx.x, pend_idx);
            // }
            int rob_obb_idx = sRob_first_child[rob_obb_par_idx] + rob_child_idx;
            int obs_obb_idx = sObs_first_child[obs_obb_par_idx] + obs_child_idx;

            if (rob_obb_idx < 0) {
                if (rob_child_idx + obs_child_idx == 0 ) {
                    printf("Error: rob_obb_idx is negative (%d) for rob_obb_par_idx %d of conf %d for thread %d on block %d. with pend_idx %d FATAL\n", rob_obb_idx, rob_obb_par_idx, global_conf_idx, threadIdx.x, blockIdx.x, pend_idx);
                }
            }

            if (obs_obb_idx < 0) {
                if (rob_child_idx + obs_child_idx == 0) {
                    printf("Error: obs_obb_idx is negative (%d) for obs_obb_par_idx %d of conf %d for thread %d on block %d. with pend_idx %d FATAL\n", obs_obb_idx, obs_obb_par_idx, global_conf_idx, threadIdx.x, blockIdx.x, pend_idx);
                }
            }

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
            // if (obs_obb_idx >= static_cast<int>(num_obs_nodes)){
            //     if (threadIdx.x == 0){
            //         printf("obs_obb_idx %d >= num_obs_nodes %llu\n", obs_obb_idx, num_obs_nodes);
            //     }
            //     continue;
            // }
            // if (obs_obb_idx < 0){
            //     if (threadIdx.x == 0){
            //         printf("obs_obb_idx %d < 0\n", obs_obb_idx);
            //     }
            //     continue;
            // }
            a = pObs_dim[obs_obb_idx];

            B = R_obs_abs.transpose() * (R_conf * R_rob_abs); // rotation of A wrt B
            Bf = B.cwiseAbs(); // rotation of A wrt B
            Bf.array() += epsilon;

            T = (T_conf + R_conf * T_rob_abs - T_obs_abs).transpose() * R_obs_abs; // translation of A wrt B
            // first tests: cross product of axes within the same box
            // (always resulting in the third axis of the box)
            ////////////////////////////////////////////////////////////////////////////////
           
           
            // A1 x A2 = A0
            t = fabsf(T[0]);
            //Since L = A0 is a unit vector (as it is the cross product of unit vectors), no need to multiply t
            // t dot L = t
            // \sum |a_i A^i * L | = a_0 + 0 + 0
            // \sum |b_i B^i * L | = b_i B^i * A0 = first element of each column vector of Bf = Bf.row(0).dot(b)

            // // Test #1
            if(t > (a[0] + Bf.row(0).dot(b))){
                continue;
            }

            // Test #2
            // B1 x B2 = B0
            t = fabsf(B.col(0).dot(T));
            if(t > (b[0] + Bf.col(0).dot(a))){
                continue;
            }

            // Test #3
            // A2 x A0 = A1
            t = fabsf(T[1]);
            if(t > (a[1] + Bf.row(1).dot(b))){
                continue;
            }

            // Test #4
            // A0 x A1 = A2
            t = fabsf(T[2]);
            if(t > (a[2] + Bf.row(2).dot(b))){
                continue;
            }

            // Test #5
            // B2 x B0 = B1
            t = fabsf(B.col(1).dot(T));
            if(t > (b[1] + Bf.col(1).dot(a))){
                continue;
            }

            // Test #6
            // B0 x B1 = B2
            t = fabsf(B.col(2).dot(T));

            if(t > (b[2] + Bf.col(2).dot(a))){
                continue;
            }

            // Test #7
            // A0 x B0
            t = fabsf(T[2] * B(1, 0) - T[1] * B(2, 0));
            if(t > (a[1] * Bf(2, 0) + a[2] * Bf(1, 0) +
                    b[1] * Bf(0, 2) + b[2] * Bf(0, 1))){
                continue;
            }

            // Test #8
            // A0 x B1
            t = fabsf(T[2] * B(1, 1) - T[1] * B(2, 1));
            if(t > (a[1] * Bf(2, 1) + a[2] * Bf(1, 1) +
                    b[0] * Bf(0, 2) + b[2] * Bf(0, 0))){
                continue;
            }

            // Test #9
            // A0 x B2
            t = fabsf(T[2] * B(1, 2) - T[1] * B(2, 2));
            if(t > (a[1] * Bf(2, 2) + a[2] * Bf(1, 2) +
                    b[0] * Bf(0, 1) + b[1] * Bf(0, 0))){
                continue;
            }

            // Test #10
            // A1 x B0
            t = fabsf(T[0] * B(2, 0) - T[2] * B(0, 0));

            if(t > (a[0] * Bf(2, 0) + a[2] * Bf(0, 0) +
                    b[1] * Bf(1, 2) + b[2] * Bf(1, 1))){
                continue;
            }

            // Test #11
            // A1 x B1
            t = fabsf(T[0] * B(2, 1) - T[2] * B(0, 1));
            if(t > (a[0] * Bf(2, 1) + a[2] * Bf(0, 1) +
                    b[0] * Bf(1, 2) + b[2] * Bf(1, 0))){
                continue;
            }

            // Test #12
            // A1 x B2
            t = fabsf(T[0] * B(2, 2) - T[2] * B(0, 2));
            if(t > (a[0] * Bf(2, 2) + a[2] * Bf(0, 2) +
                    b[0] * Bf(1, 1) + b[1] * Bf(1, 0))){
                continue;
            }

            // Test #13
            // A2 x B0
            t = fabsf(T[1] * B(0, 0) - T[0] * B(1, 0));
            if(t > (a[0] * Bf(1, 0) + a[1] * Bf(0, 0) +
                    b[1] * Bf(2, 2) + b[2] * Bf(2, 1))){
                continue;
            }

            // Test #14
            // A2 x B1
            t = fabsf(T[1] * B(0, 1) - T[0] * B(1, 1));

            if(t > (a[0] * Bf(1, 1) + a[1] * Bf(0, 1) +
                    b[0] * Bf(2, 2) + b[2] * Bf(2, 0))){
                continue;
            }

            // Test #15
            // A2 x B2
            t = fabsf(T[1] * B(0, 2) - T[0] * B(1, 2));
            if(t > (a[0] * Bf(1, 2) + a[1] * Bf(0, 2) +
                    b[0] * Bf(2, 1) + b[1] * Bf(2, 0))){
                continue;
            }

            // printf("Thread %d in block %d found overlap at robot OBB %d and obstacle OBB %d\n\
            //         With rob_first_child_idx %d and obs_first_child_idx %d\n", 
            //         threadIdx.x, blockIdx.x, rob_obb_idx, obs_obb_idx, rob_first_child_idx, obs_first_child_idx);
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
                    
                    //TODO: delete me
                    // if (rob_obb_par_idx > 0) {
                    //     printf("Error: Rob leaf, obs non-leaf, rob_obb_par_idx is positive (%d) for rob_obb_idx %d and block %d on iteration %d of conf %d. FATAL\n", rob_obb_par_idx, rob_obb_idx, blockIdx.x, delete_me_num_iter, i);
                    //     // return;
                    // }
                    // just shove the parent back in, recurse only on obstacle children
                    // TODO: this is a weird hack, figure out a better way to do this
                }
            } 
            // obstacle is leaf, robot is not
            else if (obs_first_child_idx < 0) {

                int pos = atomicAdd(&num_obb_pend, 1);
                rob_obb_pend[pos] = rob_obb_idx;
                obs_obb_pend[pos] = obs_obb_par_idx; // just shove the parent back in, recurse only on robot children
                //TODO: delete me
                // if (rob_obb_par_idx >0 && blockIdx.x == 1 && global_conf_idx == 36) {
                //     printf("Error: Rob non-leaf, obs leaf, rob_obb_par_idx is positive (%d) for rob_obb_idx %d and block %d on iteration %d of conf %d. FATAL\n", rob_obb_par_idx, rob_obb_idx, blockIdx.x, delete_me_num_iter, i);
                //     // return;
                // }
            }
        } // end while true over single configuration


        __syncthreads();
        if (collision_found) {
            continue;
        }
//         if (areTrianglesDisjoint ( num_bad_leaves, bad_rob_leaves, bad_obs_leaves,
//                                 R_conf, T_conf,
//                                 pRob_verts,   pRob_tris, 
//                                 pObs_verts,   pObs_tris)) {
//             pdisjoint[global_conf_idx] = true;
//         }
//         __syncthreads();

// {
        if (num_bad_leaves == 0){
            if (threadIdx.x == 0) {
                pdisjoint[global_conf_idx] = true;
            }
            continue;
        } else {
            //TODO: go through leaves collaboratively with a queue instead of this
            //TODO: this doesn't actually work if there are more than BLOCK_SIZE bad leaves, need to add some sort of batching mechanism
            bool valid = true;
            size_t leaf_idx = threadIdx.x;
            bool all_valid = true;
            __syncthreads();

            while(leaf_idx < num_bad_leaves){
                unsigned mask = __activemask();
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

                bool valid = triangles_valid(rob_v0, rob_v1, rob_v2, obs_v0, obs_v1, obs_v2);

                if (!__all_sync(mask, valid)) {   
                    all_valid = false;
                    break;
                }
                leaf_idx += BLOCK_SIZE;
            }
            //TODO: Verify it's safe to synchreads here
            __syncthreads();
            if (all_valid && threadIdx.x == 0) {
                pdisjoint[global_conf_idx] = true;
            }
        }
        __syncthreads();
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
    valid.assign(num_confs, false);
    if (num_confs == 0) {
        return 0.0;
    }

    std::unique_ptr<bool[]> disjoint(new bool[num_confs]);
    for (int i = 0; i < num_confs; ++i) {
        disjoint.get()[i] = false;
    }


    std::vector<Eigen::Matrix3f> rob_conf_r(num_confs);
    std::vector<Eigen::Vector3f> rob_conf_t(num_confs);
    for (int i = 0; i < num_confs; ++i) {
        rob_conf_r[i] = createRotationMatrix(confs[i]);
        rob_conf_t[i] = Eigen::Vector3f(confs[i].x, confs[i].y, confs[i].z);
    }

    const int blockSize = 32;
    const int confs_per_block = blockSize;
    const int gridSize = (num_confs + confs_per_block - 1) / confs_per_block; // ceil division


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
    bool* pdisjoint;

    // Allocate memory for device pointers
    cudaEventRecord(start, 0);
    cudaMalloc((void**)&d_R_obs, obs_BVH.size * sizeof(Eigen::Matrix3f));
    cudaMalloc((void**)&d_T_obs, obs_BVH.size * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_R_rob, rob_BVH.size * sizeof(Eigen::Matrix3f));
    cudaMalloc((void**)&d_T_rob, rob_BVH.size * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Rob_dim, rob_BVH.size * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Obs_dim, obs_BVH.size * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Rob_conf_rot, gridSize * blockSize * sizeof(Eigen::Matrix3f));
    cudaMalloc((void**)&d_Rob_conf_trans, gridSize * blockSize * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Obs_first_child, obs_BVH.size * sizeof(int16_t));
    cudaMalloc((void**)&d_Rob_first_child, rob_BVH.size * sizeof(int16_t));
    cudaMalloc((void**)&d_Rob_vertices, rob_mesh.vertices.size() * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Obs_vertices, obs_mesh.vertices.size() * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Rob_triangles, rob_mesh.triangles.size() * sizeof(Triangle));
    cudaMalloc((void**)&d_Obs_triangles, obs_mesh.triangles.size() * sizeof(Triangle));
    cudaMalloc((void**)&pdisjoint, gridSize * blockSize * sizeof(bool));

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
    checkCudaMem(cudaMemcpy(pdisjoint, disjoint.get(), num_confs * sizeof(bool), cudaMemcpyHostToDevice));
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
        d_bvh_naive<<<gridSize, blockSize, (obs_BVH.size + rob_BVH.size) * sizeof(int16_t)>>>(
                                                d_R_obs, d_T_obs,
                                                d_R_rob, d_T_rob,
                                                d_Obs_dim, d_Rob_dim,
                                                d_Rob_conf_rot, d_Rob_conf_trans,
                                                d_Obs_first_child, d_Rob_first_child,
                                                d_Rob_vertices, d_Rob_triangles, rob_BVH.size,
                                                d_Obs_vertices, d_Obs_triangles, obs_BVH.size,
                                                pdisjoint);
    };

    // Dry run: single-block launch, untimed, purely to get the kernel loaded onto the device
    if (dry_run) {
        d_bvh_naive<<<1, blockSize, (obs_BVH.size + rob_BVH.size) * sizeof(int16_t)>>>(
                                                d_R_obs, d_T_obs,
                                                d_R_rob, d_T_rob,
                                                d_Obs_dim, d_Rob_dim,
                                                d_Rob_conf_rot, d_Rob_conf_trans,
                                                d_Obs_first_child, d_Rob_first_child,
                                                d_Rob_vertices, d_Rob_triangles, rob_BVH.size,
                                                d_Obs_vertices, d_Obs_triangles, obs_BVH.size,
                                                pdisjoint);
        checkCudaMem(cudaGetLastError());
        checkCudaMem(cudaDeviceSynchronize());
        std::cout << "BVH Naive dry run completed successfully." << std::endl;
    }

    cudaEventRecord(start, 0);
    launch_bvh_naive();
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&duration, start, stop);
    std::cout << "BVH Naive GPU kernel took " << duration << " ms for " << num_confs << " configurations." << std::endl;

    // Copy result back to host (num_confs * sizeof(bool))
    //TODO deleteme
    for (int i = 0; i < 10000000; ++i) {
        i++;
    }
    cudaEventRecord(start, 0);
    checkCudaMem(cudaGetLastError());

    checkCudaMem(cudaMemcpy(disjoint.get(), pdisjoint, num_confs * sizeof(bool), cudaMemcpyDeviceToHost));
    cudaDeviceSynchronize();
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&duration, start, stop);
    std::cout << "Copying results from GPU took " << duration << " ms." << std::endl;

    for (int i = 0; i < num_confs; ++i) {
        valid[i] = disjoint[i];
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

    return duration;
}
