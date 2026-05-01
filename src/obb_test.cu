#include <iostream>
#include <Eigen/Dense>
#include <fcl/fcl.h>
#include "./Utils.h"

#include "./Triangle.hu"
// taken from fcl/math/bv/obb-inl.h
/// @brief Check collision between two boxes: the first box is in configuration
/// (B, T) and its half dimension is set by a; the second box is in identity
/// configuration and its half dimension is set by b.

// We want this to detect whether two OBB's are disjoint.
// Presume as given that first box, A, is centered at the origin with half dimensions
// given by vector a, that is, the components of a are half the lengths of the box along each axis.
// We also assume that the axes of A are the standard basis vectors, i.e., A is axis-aligned.
// This means that for the second box, B, its rotation matrix B is given by the rotation of the box around the origin,
// with respect to the axes of A, and its translation T is the center of B in the coordinate system of A.
// The half dimensions of B are given by vector b, similarly to a.
// Since the rotation matrix B is with respect to the axes of A, the columns of B are the unit vectors of the axes of B in the coordinate system of A.
//
// The function will return true if the two boxes are disjoint, and false otherwise.


#define WARP_SIZE 32




//TODO: Make a custom data type for this struct
BVNode_soa BVH_n_ary_hierarchy_from_mesh(const char* mesh_path, size_t power_of_2 = 5){
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


// TODO: This assumes that B is a rotation matrix of B with respect to the axes of A
// we may want to calculate this dynamically, but for now, assume A axis aligned and centered at origin
OBB_soa hierarchy_from_mesh(const char* mesh_path){
    // Load Robot
    std::vector<fcl::Vector3f> rob_vertices;
    std::vector<fcl::Triangle> rob_triangles;


    loadOBJFileFCL(mesh_path, rob_vertices, rob_triangles);

    // why is this a pointer???
    std::shared_ptr<fcl::BVHModel<fcl::OBB<float>>> rob_mesh(new fcl::BVHModel<fcl::OBB<float>>);
    rob_mesh->beginModel(rob_triangles.size(), rob_vertices.size());
    rob_mesh->addSubModel(rob_vertices, rob_triangles);
    rob_mesh->endModel();

    getBVHTreeDepths(*rob_mesh);
    // Access OBB data from rob_mesh
    // rob_mesh->getNumBVs() gives the number of OBBs in the hierarchy
    size_t num_boxes = rob_mesh->getNumBVs();
    OBB_soa result(num_boxes);

    Eigen::Matrix3f rotation;
    Eigen::Vector3f translation;
    Eigen::Vector3f half_dimensions;

    //check to make sure all primitive ids are accounted for
    std::vector<bool> primitive_id_found(rob_mesh->num_tris, false);
    for (int i = 0; i < num_boxes; ++i) {

        fcl::OBB<float> obb = rob_mesh->getBV(i).bv;
        rotation = obb.axis;
        translation = obb.To;
        half_dimensions = obb.extent;
        auto node = rob_mesh->getBV(i);

        if (node.isLeaf()){
            // std::cout << "Node " << i << " is a leaf with type"  << typeid(obb).name() << std::endl;
            // std::cout << "It has axis :" << obb.axis << std::endl;
            // std::cout << "It has translation :" << obb.To.transpose() << std::endl;
            // std::cout << "It has half-dimensions :" << obb.extent.transpose() << std::endl;
            // fcl::Triangle triangle = rob_mesh->tri_indices[node.primitiveId()] ;
            // std::cout << "It has triangle " << triangle[0] << ", " << triangle[1] << ", " << triangle[2] << std::endl;
            primitive_id_found[node.primitiveId()] = true;
        }


        // if (i == 0){
        //     std::cout << "The outermost OBB has: " << std::endl;
        //     std::cout << "Height " << obb.height() << std::endl;
        //     std::cout << "Width " << obb.width() << std::endl;
        //     std::cout << "Depth " << obb.depth() << std::endl;

        // }
        // std::cout << "Rotation: " << rotation << std::endl;
        // std::cout << "Translation: " << translation.transpose() << std::endl;
        // std::cout << "Half Dimensions: " << half_dimensions.transpose() << std::endl;

        // // print out the values to verify
        // std::cout << "OBB " << i << ":\n";
        // std::cout << "Rotation:\n" << rotation << "\n";
        // std::cout << "Translation:\n" << translation.transpose() << "\n";
        // std::cout << "Half Dimensions:\n" << half_dimensions.transpose() << "\n";
        // std::cout << "-----------------------\n";
        result.set(i, rotation, translation, half_dimensions);
    }

    //verify all primitive ids were found
    for (size_t i = 0; i < primitive_id_found.size(); i++){
        if (!primitive_id_found[i]){
            std::cout << "Warning: Primitive ID " << i << " was not found in any leaf OBB." << std::endl;
        }
    }
    // fcl::OBB<float> outer_obb = rob_mesh->getBV(0).bv;
    // bool all_in = true;
    // for (fcl::Vector3f vertex : rob_vertices){
    //     if (!outer_obb.contain(vertex)){
    //         all_in = false;
    //         std::cout << vertex << " not in outermost OBB" << std::endl;
    //     }
    // }


    // delete rob_mesh manually to free memory
    rob_mesh.reset();

    //verify that the data was copied correctly
    // for (int i = 0; i < num_boxes; ++i) {
    //     std::cout << "Verifying OBB " << i << ":\n";
    //     std::cout << "Rotation:\n" << result.pR[i] << "\n";
    //     std::cout << "Translation:\n" << result.pT[i].transpose() << "\n";
    //     std::cout << "Half Dimensions:\n" << result.pDim[i].transpose() << "\n";
    //     std::cout << "-----------------------\n";
    // }



    return result;
}

// Note: This assumes that OBB B has rotation and translation with respect to A
// will this be feasible to use in practice? -- no, need to compute this dynamically
__global__ void d_obb_base(  const Eigen::Matrix3f* pB, const Eigen::Vector3f* pT,
                                const Eigen::Vector3f* pa, const Eigen::Vector3f* pb, bool* pdisjoint) {

    // Read data
    Eigen::Matrix3f B = pB[blockIdx.x]; // rotation of B with respect to A
    Eigen::Vector3f T = pT[blockIdx.x]; // translation of B from A
    Eigen::Vector3f a = pa[blockIdx.x]; // half dimensions of box A
    Eigen::Vector3f b = pb[blockIdx.x]; // half dimensions of box B

    float t; // distance between centers of the two boxes as projected onto the axis
    const float epsilon = 1e-5f; // small value to avoid numerical issues
    // Take the absolute value of the rotation matrix B, add epsilon to avoid numerical issues
    // TODO: make this read as efficient as possible
    Eigen::Matrix3f Bf = B.cwiseAbs(); // absolute value of rotation matrix B
    Bf.array() += epsilon;

    // first tests: cross product of axes within the same box
    // (always resulting in the third axis of the box)
    ////////////////////////////////////////////////////////////////////////////////
    // A1 x A2 = A0
    t = ((T[0] < 0.0) ? -T[0] : T[0]);

    //Since L = A0 is a unit vector (as it is the cross product of unit vectors), no need to multiply t
    // t dot L = t
    // \sum |a_i A^i * L | = a_0 + 0 + 0
    // \sum |b_i B^i * L | = b_i B^i * A0 = first element of each column vector of Bf = Bf.row(0).dot(b)
    if(t > (a[0] + Bf.row(0).dot(b)))
        pdisjoint[blockIdx.x] = true;
        return;

  // B1 x B2 = B0
    float s =  B.col(0).dot(T);
    t = ((s < 0.0) ? -s : s);

    if(t > (b[0] + Bf.col(0).dot(a)))
        pdisjoint[blockIdx.x] = true;
        return;

    // A2 x A0 = A1
    t = ((T[1] < 0.0) ? -T[1] : T[1]);

    if(t > (a[1] + Bf.row(1).dot(b)))
        pdisjoint[blockIdx.x] = true;
        return;

    // A0 x A1 = A2
    t =((T[2] < 0.0) ? -T[2] : T[2]);

    if(t > (a[2] + Bf.row(2).dot(b)))
        pdisjoint[blockIdx.x] = true;
        return;

    // B2 x B0 = B1
    s = B.col(1).dot(T);
    t = ((s < 0.0) ? -s : s);

    if(t > (b[1] + Bf.col(1).dot(a)))
        pdisjoint[blockIdx.x] = true;
        return;

    // B0 x B1 = B2
    s = B.col(2).dot(T);
    t = ((s < 0.0) ? -s : s);

    if(t > (b[2] + Bf.col(2).dot(a)))
        pdisjoint[blockIdx.x] = true;
        return;

    // A0 x B0
    s = T[2] * B(1, 0) - T[1] * B(2, 0);
    t = ((s < 0.0) ? -s : s);

    if(t > (a[1] * Bf(2, 0) + a[2] * Bf(1, 0) +
            b[1] * Bf(0, 2) + b[2] * Bf(0, 1)))
        pdisjoint[blockIdx.x] = true;
        return;

    // A0 x B1
    s = T[2] * B(1, 1) - T[1] * B(2, 1);
    t = ((s < 0.0) ? -s : s);

    if(t > (a[1] * Bf(2, 1) + a[2] * Bf(1, 1) +
            b[0] * Bf(0, 2) + b[2] * Bf(0, 0)))
        pdisjoint[blockIdx.x] = true;
        return;

    // A0 x B2
    s = T[2] * B(1, 2) - T[1] * B(2, 2);
    t = ((s < 0.0) ? -s : s);

    if(t > (a[1] * Bf(2, 2) + a[2] * Bf(1, 2) +
            b[0] * Bf(0, 1) + b[1] * Bf(0, 0)))
        pdisjoint[blockIdx.x] = true;
        return;

    // A1 x B0
    s = T[0] * B(2, 0) - T[2] * B(0, 0);
    t = ((s < 0.0) ? -s : s);

    if(t > (a[0] * Bf(2, 0) + a[2] * Bf(0, 0) +
            b[1] * Bf(1, 2) + b[2] * Bf(1, 1)))
        pdisjoint[blockIdx.x] = true;
        return;

    // A1 x B1
    s = T[0] * B(2, 1) - T[2] * B(0, 1);
    t = ((s < 0.0) ? -s : s);

    if(t > (a[0] * Bf(2, 1) + a[2] * Bf(0, 1) +
            b[0] * Bf(1, 2) + b[2] * Bf(1, 0)))
        pdisjoint[blockIdx.x] = true;
        return;

    // A1 x B2
    s = T[0] * B(2, 2) - T[2] * B(0, 2);
    t = ((s < 0.0) ? -s : s);

    if(t > (a[0] * Bf(2, 2) + a[2] * Bf(0, 2) +
            b[0] * Bf(1, 1) + b[1] * Bf(1, 0)))
        pdisjoint[blockIdx.x] = true;
        return;

    // A2 x B0
    s = T[1] * B(0, 0) - T[0] * B(1, 0);
    t = ((s < 0.0) ? -s : s);

    if(t > (a[0] * Bf(1, 0) + a[1] * Bf(0, 0) +
            b[1] * Bf(2, 2) + b[2] * Bf(2, 1)))
        pdisjoint[blockIdx.x] = true;
        return;

    // A2 x B1
    s = T[1] * B(0, 1) - T[0] * B(1, 1);
    t = ((s < 0.0) ? -s : s);

    if(t > (a[0] * Bf(1, 1) + a[1] * Bf(0, 1) +
            b[0] * Bf(2, 2) + b[2] * Bf(2, 0)))
        pdisjoint[blockIdx.x] = true;
        return;

    // A2 x B2
    s = T[1] * B(0, 2) - T[0] * B(1, 2);
    t = ((s < 0.0) ? -s : s);

    if(t > (a[0] * Bf(1, 2) + a[1] * Bf(0, 2) +
            b[0] * Bf(2, 1) + b[1] * Bf(2, 0)))
        pdisjoint[blockIdx.x] = true;
        return;

    pdisjoint[blockIdx.x] = false;

}

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


// Do TWO_STAGE_CF tests per thread
// only top level box
// used shared memory to enqueue
// check N configurations for full collision
#define TWO_STAGE_CF 29
#define BLOCK_SIZE 32
#define NUM_CONFS_PER_BLOCK_2S (TWO_STAGE_CF * BLOCK_SIZE)
__global__ void d_obb_coursened_two_stage ( const Eigen::Matrix3f* pR_obs, const Eigen::Vector3f* pT_obs,
                                const Eigen::Matrix3f* pR_rob, const Eigen::Vector3f* pT_rob,
                                const Eigen::Vector3f* pRob_dim, const Eigen::Vector3f* pObs_dim,
                                const Eigen::Matrix3f* pRob_conf_rot, const Eigen::Vector3f* pRob_conf_trans,
                                bool* pdisjoint) {

    uint32_t index = blockIdx.x * blockDim.x + threadIdx.x;
    Eigen::Matrix3f R_obs_abs = pR_obs[index]; // rotation of B wrt origin
    Eigen::Vector3f T_obs_abs = pT_obs[index]; // translation of B wrt origin

    Eigen::Matrix3f R_rob_abs = pR_rob[index]; // rotation of A wrt origin
    Eigen::Vector3f T_rob_abs = pT_rob[index]; // translation of A wrt origin

    Eigen::Vector3f b = pRob_dim[index]; // half dimensions of box A
    Eigen::Vector3f a = pObs_dim[index]; // half dimensions of box B



    float t; // distance between centers of the two boxes as projected onto the axis
    const float epsilon = 1e-6f; // small value to avoid numerical issues

    //Calculate relative rotation of B wrt A
    //TODO: precompute inverse rotations of A

    // Load configurations into shared memory
    __shared__ Eigen::Matrix3f shared_R_conf[NUM_CONFS_PER_BLOCK_2S];
    __shared__ Eigen::Vector3f shared_T_conf[NUM_CONFS_PER_BLOCK_2S];
    uint32_t local_conf_idx = threadIdx.x;
    uint32_t global_conf_idx = blockIdx.x * NUM_CONFS_PER_BLOCK_2S + local_conf_idx;
    for (; local_conf_idx < NUM_CONFS_PER_BLOCK_2S; local_conf_idx += BLOCK_SIZE){
        shared_R_conf[local_conf_idx] = pRob_conf_rot[global_conf_idx];
        global_conf_idx += BLOCK_SIZE;
    }

    local_conf_idx = threadIdx.x;
    global_conf_idx = blockIdx.x * NUM_CONFS_PER_BLOCK_2S + local_conf_idx;
    for (; local_conf_idx < NUM_CONFS_PER_BLOCK_2S; local_conf_idx += BLOCK_SIZE){
        shared_T_conf[local_conf_idx] = pRob_conf_trans[global_conf_idx];
        global_conf_idx += BLOCK_SIZE;
    }

    // memory-efficient approach: store only indices of potentially colliding configurations
    // recompute each time.
    __shared__ uint32_t overlap_count;
    __shared__ uint32_t overlap_indices[TWO_STAGE_CF * BLOCK_SIZE]; // arbitrarily chosen buffer size, should experiment with this
    if (threadIdx.x == 0){
        overlap_count = 0;
    }
    __syncthreads();



    //TODO: try computationally efficient approach
    // __shared__ Eigen::Matrix3f overlap_R_conf[TWO_STAGE_CF];
    // __shared__ Eigen::Vector3f overlap_T_conf[TWO_STAGE_CF];

    local_conf_idx = threadIdx.x;
    global_conf_idx = blockIdx.x * BLOCK_SIZE * TWO_STAGE_CF + local_conf_idx;
    Eigen::Matrix3f R_conf; // rotation of robot wrt world
    Eigen::Vector3f T_conf; // translation of robot wrt world

    Eigen::Matrix3f Bf;
    Eigen::Vector3f T;
    for (; local_conf_idx< NUM_CONFS_PER_BLOCK_2S; local_conf_idx += BLOCK_SIZE){
        R_conf = shared_R_conf[local_conf_idx];
        T_conf = shared_T_conf[local_conf_idx];

        // Take the absolute value of the rotation matrix B, add epsilon to avoid numerical issues
        // Eigen::Matrix3f B = R_obs_abs.transpose() * (R_conf * R_rob_abs); // rotation of A wrt B
        // Eigen::Matrix3f Bf = B.cwiseAbs();

        //TODO: we technically only need to compute Bf.row(0).dot(b) for this portion
        Bf = (R_obs_abs.transpose() * (R_conf * R_rob_abs)).cwiseAbs(); // rotation of A wrt B
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

        // Test #1
        if(t > (a[0] + Bf.row(0).dot(b))){
            pdisjoint[global_conf_idx] = true;
        } else {
            // potentially colliding configuration, store index
            uint32_t pos = atomicAdd(&overlap_count, 1);
            overlap_indices[pos] = local_conf_idx;
        }
        global_conf_idx += BLOCK_SIZE;
    }

    __syncthreads();
    local_conf_idx = threadIdx.x;
    uint32_t priv_overlap_count = overlap_count;
    uint32_t global_conf_offset = blockIdx.x * BLOCK_SIZE * TWO_STAGE_CF;
    // if (local_conf_idx == 0) {
    //     printf("Block %d: found %d potentially colliding configurations\n", blockIdx.x, overlap_count);
    // }
    // Now check potentially colliding configurations in detail
    for (; local_conf_idx < priv_overlap_count; local_conf_idx += BLOCK_SIZE){
        uint32_t conf_idx = overlap_indices[local_conf_idx];
        global_conf_idx = global_conf_offset + conf_idx;
        R_conf = shared_R_conf[conf_idx];
        T_conf = shared_T_conf[conf_idx];

        // Take the absolute value of the rotation matrix B, add epsilon to avoid numerical issues
        Eigen::Matrix3f B = R_obs_abs.transpose() * (R_conf * R_rob_abs); // rotation of A wrt B
        Bf = B.cwiseAbs();
        Bf.array() += epsilon;

        T = (T_conf + R_conf * T_rob_abs - T_obs_abs).transpose() * R_obs_abs; // translation of A wrt B

        // Test #2
        // B1 x B2 = B0
        t = fabsf(B.col(0).dot(T));

        if(t > (b[0] + Bf.col(0).dot(a))){
            pdisjoint[global_conf_idx] = true;
            continue;
        }

        // Test #3
        // A2 x A0 = A1
        t = fabsf(T[1]);

        if(t > (a[1] + Bf.row(1).dot(b))){
            pdisjoint[global_conf_idx] = true;
            continue;
        }

        // Test #4
        // A0 x A1 = A2
        t = fabsf(T[2]);

        if(t > (a[2] + Bf.row(2).dot(b))){
            pdisjoint[global_conf_idx] = true;
            continue;
        }

        // Test #5
        // B2 x B0 = B1
        t = fabsf(B.col(1).dot(T));

        if(t > (b[1] + Bf.col(1).dot(a))){
            pdisjoint[global_conf_idx] = true;
            continue;
        }

        // Test #6
        // B0 x B1 = B2
        t = fabsf(B.col(2).dot(T));

        if(t > (b[2] + Bf.col(2).dot(a))){
            pdisjoint[global_conf_idx] = true;
            continue;
        }

        // Test #7
        // A0 x B0
        t = fabsf(T[2] * B(1, 0) - T[1] * B(2, 0));

        if(t > (a[1] * Bf(2, 0) + a[2] * Bf(1, 0) +
                b[1] * Bf(0, 2) + b[2] * Bf(0, 1))){
            pdisjoint[global_conf_idx] = true;
            continue;
        }

        // Test #8
        // A0 x B1
        t = fabsf(T[2] * B(1, 1) - T[1] * B(2, 1));

        if(t > (a[1] * Bf(2, 1) + a[2] * Bf(1, 1) +
                b[0] * Bf(0, 2) + b[2] * Bf(0, 0))){
            pdisjoint[global_conf_idx] = true;
            continue;
        }

        // Test #9
        // A0 x B2
        t = fabsf(T[2] * B(1, 2) - T[1] * B(2, 2));

        if(t > (a[1] * Bf(2, 2) + a[2] * Bf(1, 2) +
                b[0] * Bf(0, 1) + b[1] * Bf(0, 0))){
            pdisjoint[global_conf_idx] = true;
            continue;
        }

        // Test #10
        // A1 x B0
        t = fabsf(T[0] * B(2, 0) - T[2] * B(0, 0));

        if(t > (a[0] * Bf(2, 0) + a[2] * Bf(0, 0) +
                b[1] * Bf(1, 2) + b[2] * Bf(1, 1))){
            pdisjoint[global_conf_idx] = true;
            continue;
        }

        // Test #11
        // A1 x B1
        t = fabsf(T[0] * B(2, 1) - T[2] * B(0, 1));

        if(t > (a[0] * Bf(2, 1) + a[2] * Bf(0, 1) +
                b[0] * Bf(1, 2) + b[2] * Bf(1, 0))){
            pdisjoint[global_conf_idx] = true;
            continue;
        }

        // Test #12
        // A1 x B2
        t = fabsf(T[0] * B(2, 2) - T[2] * B(0, 2));

        if(t > (a[0] * Bf(2, 2) + a[2] * Bf(0, 2) +
                b[0] * Bf(1, 1) + b[1] * Bf(1, 0))){
            pdisjoint[global_conf_idx] = true;
            continue;
        }

        // Test #13
        // A2 x B0
        t = fabsf(T[1] * B(0, 0) - T[0] * B(1, 0));

        if(t > (a[0] * Bf(1, 0) + a[1] * Bf(0, 0) +
                b[1] * Bf(2, 2) + b[2] * Bf(2, 1))){
            pdisjoint[global_conf_idx] = true;
            continue;
        }

        // Test #14
        // A2 x B1
        t = fabsf(T[1] * B(0, 1) - T[0] * B(1, 1));

        if(t > (a[0] * Bf(1, 1) + a[1] * Bf(0, 1) +
                b[0] * Bf(2, 2) + b[2] * Bf(2, 0))){
            pdisjoint[global_conf_idx] = true;
            continue;
        }

        // Test #15
        // A2 x B2
        t = fabsf(T[1] * B(0, 2) - T[0] * B(1, 2));

        if(t > (a[0] * Bf(1, 2) + a[1] * Bf(0, 2) +
                b[0] * Bf(2, 1) + b[1] * Bf(2, 0))){
            pdisjoint[global_conf_idx] = true;
            continue;
        }
    }

    // pdisjoint[index] = false;
}

#define ONE_STAGE_CF 32
#define NUM_CONFS_PER_BLOCK_1S (ONE_STAGE_CF * BLOCK_SIZE)
__global__ void d_obb_coursened_one_stage ( const Eigen::Matrix3f* pR_obs, const Eigen::Vector3f* pT_obs,
                                const Eigen::Matrix3f* pR_rob, const Eigen::Vector3f* pT_rob,
                                const Eigen::Vector3f* pRob_dim, const Eigen::Vector3f* pObs_dim,
                                const Eigen::Matrix3f* pRob_conf_rot, const Eigen::Vector3f* pRob_conf_trans,
                                bool* pdisjoint) {

    uint32_t index = blockIdx.x * blockDim.x + threadIdx.x;
    Eigen::Matrix3f R_obs_abs = pR_obs[index]; // rotation of B wrt origin
    Eigen::Vector3f T_obs_abs = pT_obs[index]; // translation of B wrt origin

    Eigen::Matrix3f R_rob_abs = pR_rob[index]; // rotation of A wrt origin
    Eigen::Vector3f T_rob_abs = pT_rob[index]; // translation of A wrt origin

    Eigen::Vector3f b = pRob_dim[index]; // half dimensions of box A
    Eigen::Vector3f a = pObs_dim[index]; // half dimensions of box B



    float t; // distance between centers of the two boxes as projected onto the axis
    const float epsilon = 1e-6f; // small value to avoid numerical issues

    //Calculate relative rotation of B wrt A
    //TODO: precompute inverse rotations of A

    // Load configurations into shared memory
    __shared__ Eigen::Matrix3f shared_R_conf[NUM_CONFS_PER_BLOCK_1S];
    __shared__ Eigen::Vector3f shared_T_conf[NUM_CONFS_PER_BLOCK_1S];
    uint32_t local_conf_idx = threadIdx.x;
    uint32_t global_conf_idx = blockIdx.x * NUM_CONFS_PER_BLOCK_1S + local_conf_idx;
    for (; local_conf_idx < NUM_CONFS_PER_BLOCK_1S; local_conf_idx += BLOCK_SIZE){
        shared_R_conf[local_conf_idx] = pRob_conf_rot[global_conf_idx];
        global_conf_idx += BLOCK_SIZE;
    }

    local_conf_idx = threadIdx.x;
    global_conf_idx = blockIdx.x * NUM_CONFS_PER_BLOCK_1S + local_conf_idx;
    for (; local_conf_idx < NUM_CONFS_PER_BLOCK_1S; local_conf_idx += BLOCK_SIZE){
        shared_T_conf[local_conf_idx] = pRob_conf_trans[global_conf_idx];
        global_conf_idx += BLOCK_SIZE;
    }
    __syncthreads();

    local_conf_idx = threadIdx.x;
    global_conf_idx = blockIdx.x * NUM_CONFS_PER_BLOCK_1S + local_conf_idx - BLOCK_SIZE;
    Eigen::Matrix3f R_conf; // rotation of robot wrt world
    Eigen::Vector3f T_conf; // translation of robot wrt world

    Eigen::Matrix3f B;
    Eigen::Matrix3f Bf;
    Eigen::Vector3f T;
    for (; local_conf_idx< NUM_CONFS_PER_BLOCK_1S; local_conf_idx += BLOCK_SIZE){
        R_conf = shared_R_conf[local_conf_idx];
        T_conf = shared_T_conf[local_conf_idx];
        global_conf_idx += BLOCK_SIZE;


        // Take the absolute value of the rotation matrix B, add epsilon to avoid numerical issues
        // Eigen::Matrix3f B = R_obs_abs.transpose() * (R_conf * R_rob_abs); // rotation of A wrt B
        // Eigen::Matrix3f Bf = B.cwiseAbs();

        //TODO: we technically only need to compute Bf.row(0).dot(b) for this portion
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

        // Test #1
        if(t > (a[0] + Bf.row(0).dot(b))){
            pdisjoint[global_conf_idx] = true;
            continue;
        }

        // Test #2
        // B1 x B2 = B0
        t = fabsf(B.col(0).dot(T));

        if(t > (b[0] + Bf.col(0).dot(a))){
            pdisjoint[global_conf_idx] = true;
            continue;
        }

        // Test #3
        // A2 x A0 = A1
        t = fabsf(T[1]);

        if(t > (a[1] + Bf.row(1).dot(b))){
            pdisjoint[global_conf_idx] = true;
            continue;
        }

        // Test #4
        // A0 x A1 = A2
        t = fabsf(T[2]);

        if(t > (a[2] + Bf.row(2).dot(b))){
            pdisjoint[global_conf_idx] = true;
            continue;
        }

        // Test #5
        // B2 x B0 = B1
        t = fabsf(B.col(1).dot(T));

        if(t > (b[1] + Bf.col(1).dot(a))){
            pdisjoint[global_conf_idx] = true;
            continue;
        }

        // Test #6
        // B0 x B1 = B2
        t = fabsf(B.col(2).dot(T));

        if(t > (b[2] + Bf.col(2).dot(a))){
            pdisjoint[global_conf_idx] = true;
            continue;
        }

        // Test #7
        // A0 x B0
        t = fabsf(T[2] * B(1, 0) - T[1] * B(2, 0));

        if(t > (a[1] * Bf(2, 0) + a[2] * Bf(1, 0) +
                b[1] * Bf(0, 2) + b[2] * Bf(0, 1))){
            pdisjoint[global_conf_idx] = true;
            continue;
        }

        // Test #8
        // A0 x B1
        t = fabsf(T[2] * B(1, 1) - T[1] * B(2, 1));

        if(t > (a[1] * Bf(2, 1) + a[2] * Bf(1, 1) +
                b[0] * Bf(0, 2) + b[2] * Bf(0, 0))){
            pdisjoint[global_conf_idx] = true;
            continue;
        }

        // Test #9
        // A0 x B2
        t = fabsf(T[2] * B(1, 2) - T[1] * B(2, 2));

        if(t > (a[1] * Bf(2, 2) + a[2] * Bf(1, 2) +
                b[0] * Bf(0, 1) + b[1] * Bf(0, 0))){
            pdisjoint[global_conf_idx] = true;
            continue;
        }

        // Test #10
        // A1 x B0
        t = fabsf(T[0] * B(2, 0) - T[2] * B(0, 0));

        if(t > (a[0] * Bf(2, 0) + a[2] * Bf(0, 0) +
                b[1] * Bf(1, 2) + b[2] * Bf(1, 1))){
            pdisjoint[global_conf_idx] = true;
            continue;
        }

        // Test #11
        // A1 x B1
        t = fabsf(T[0] * B(2, 1) - T[2] * B(0, 1));

        if(t > (a[0] * Bf(2, 1) + a[2] * Bf(0, 1) +
                b[0] * Bf(1, 2) + b[2] * Bf(1, 0))){
            pdisjoint[global_conf_idx] = true;
            continue;
        }

        // Test #12
        // A1 x B2
        t = fabsf(T[0] * B(2, 2) - T[2] * B(0, 2));

        if(t > (a[0] * Bf(2, 2) + a[2] * Bf(0, 2) +
                b[0] * Bf(1, 1) + b[1] * Bf(1, 0))){
            pdisjoint[global_conf_idx] = true;
            continue;
        }

        // Test #13
        // A2 x B0
        t = fabsf(T[1] * B(0, 0) - T[0] * B(1, 0));

        if(t > (a[0] * Bf(1, 0) + a[1] * Bf(0, 0) +
                b[1] * Bf(2, 2) + b[2] * Bf(2, 1))){
            pdisjoint[global_conf_idx] = true;
            continue;
        }

        // Test #14
        // A2 x B1
        t = fabsf(T[1] * B(0, 1) - T[0] * B(1, 1));

        if(t > (a[0] * Bf(1, 1) + a[1] * Bf(0, 1) +
                b[0] * Bf(2, 2) + b[2] * Bf(2, 0))){
            pdisjoint[global_conf_idx] = true;
            continue;
        }

        // Test #15
        // A2 x B2
        t = fabsf(T[1] * B(0, 2) - T[0] * B(1, 2));

        if(t > (a[0] * Bf(1, 2) + a[1] * Bf(0, 2) +
                b[0] * Bf(2, 1) + b[1] * Bf(2, 0))){
            pdisjoint[global_conf_idx] = true;
            continue;
        }
    }

    // pdisjoint[index] = false;
}

                                // is there a way to specify children with pointer arithmetic that respects coalescing?
                                // maybe bring in all the children into shared memory first, then use pointers?

//assumes BVH of both trees have same depth
__global__ void d_bvh_naive   ( const Eigen::Matrix3f* pR_obs, const Eigen::Vector3f* pT_obs,
                                const Eigen::Matrix3f* pR_rob, const Eigen::Vector3f* pT_rob,
                                const Eigen::Vector3f* pObs_dim, const Eigen::Vector3f* pRob_dim,
                                const Eigen::Matrix3f* pRob_conf_rot, const Eigen::Vector3f* pRob_conf_trans,
                                const int16_t* pObs_first_child, const int16_t* pRob_first_child,
                                const Eigen::Vector3f *pRob_verts, const Triangle *pRob_tris, size_t num_rob_nodes,
                                const Eigen::Vector3f *pObs_verts, const Triangle *pObs_tris, size_t num_obs_nodes,
                                bool* pdisjoint) {

    // extern __shared__                                     
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

    int conf_offset = (threadIdx.x >> 4)-2; // divide by 16 to see if thread works on the 0th pair or 1st pair of pending boxes
    int rob_child_idx = (threadIdx.x >> 2) & 0x3; // divide by 4, then mod by 4to see which child of the robot box this thread is assigned to
    int obs_child_idx = threadIdx.x & 0x3; // mod 4 to see which child of the obstacle box this thread is assigned to
    for (uint8_t i = 0; i < num_confs_pend; i++){
        __syncthreads();
        R_conf = rob_confs_pend_rot[i];
        T_conf = rob_confs_pend_trans[i];
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
        while(true){
            __syncthreads();
            if (num_obb_pend == 0){
                break;
            }


            int16_t pend_idx = num_obb_pend + conf_offset;
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

            int16_t rob_obb_par_idx = rob_obb_pend[pend_idx];
            int16_t obs_obb_par_idx = obs_obb_pend[pend_idx];

            int rob_obb_idx = sRob_first_child[rob_obb_par_idx] + rob_child_idx;
            int obs_obb_idx = sObs_first_child[obs_obb_par_idx] + obs_child_idx;

            int16_t rob_first_child_idx = sRob_first_child[rob_obb_idx];
            int16_t obs_first_child_idx = sObs_first_child[obs_obb_idx];

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
                    rob_obb_pend[pos] = rob_obb_par_idx; // just shove the parent back in, recurse only on obstacle children
                    // TODO: this is a weird hack, figure out a better way to do this
                }
            } 
            // obstacle is leaf, robot is not
            else if (obs_first_child_idx < 0) {

                int pos = atomicAdd(&num_obb_pend, 1);
                rob_obb_pend[pos] = rob_obb_idx;
                obs_obb_pend[pos] = obs_obb_par_idx; // just shove the parent back in, recurse only on robot children
            }
        } // end while true over single configuration
        __syncthreads();
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
    }
    return;
}

// Code for 1 single box test
void dummy_test() {
    // Test OBB disjoint function
    const int num_boxes = 1; // Example with one box

    Eigen::Matrix3f B = Eigen::Matrix3f::Identity(); // Identity rotation
    Eigen::Vector3f T(5.0f, 5.0f, 5.0f); // Translation of B from A
    Eigen::Vector3f a(1.0f, 1.0f, 1.0f); // Half dimensions of box A
    Eigen::Vector3f b(1.0f, 1.0f, 1.0f); // Half dimensions of box B
    bool disjoint[num_boxes] = {false};
    Eigen::Matrix3f* pB;
    Eigen::Vector3f* pT;
    Eigen::Vector3f* pa;
    Eigen::Vector3f* pb;
    bool* pdisjoint;

    // Allocate memory for device pointers
    cudaMalloc((void**)&pB, num_boxes * sizeof(Eigen::Matrix3f));
    cudaMalloc((void**)&pT, num_boxes * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&pa, num_boxes * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&pb, num_boxes * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&pdisjoint, num_boxes * sizeof(bool));


    // Copy data to device
    cudaMemcpy(pB, &B, sizeof(Eigen::Matrix3f), cudaMemcpyHostToDevice);
    cudaMemcpy(pT, &T, sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice);
    cudaMemcpy(pa, &a, sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice);
    cudaMemcpy(pb, &b, sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice);
    cudaMemcpy(pdisjoint, disjoint, sizeof(bool), cudaMemcpyHostToDevice);

    // Launch kernel
    d_obb_base<<<num_boxes, 1>>>(pB, pT, pa, pb, pdisjoint);

    // Copy result back to host
    cudaMemcpy(disjoint, pdisjoint, sizeof(bool), cudaMemcpyDeviceToHost);

    // Check result
    if (disjoint[0]) {
        std::cout << "Boxes are disjoint." << std::endl;
    } else {
        std::cout << "Boxes are not disjoint." << std::endl;
    }

    // Free device memory
    cudaFree(pB);
    cudaFree(pT);
    cudaFree(pa);
    cudaFree(pb);
    cudaFree(pdisjoint);
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
    checkConfsCPU(cpuCollisions, confs);
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
//TODO: idea -- half float for obb, full float for triangle overlap?

double broad_coarsened_shared_mem_2S() {
    // Similar setup as broad_naive_1 but using d_obb_coursened_two_stage kernel
    cudaEvent_t start, stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);
    // Load Robot and Obstacle BVH
    OBB_soa rob_BVH = hierarchy_from_mesh("/home/victor/Projects/robo-check/data/models/alpha1.0/robot.obj");
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
    const int blockSize = BLOCK_SIZE;
    const int confs_per_block = TWO_STAGE_CF * blockSize;
    const int gridSize = (num_confs + confs_per_block - 1) / confs_per_block; // ceil division

    // std::cout << "Launching coarsened_2S kernel with grid size " << gridSize << " and block size " << blockSize << std::endl;
    // std::cout << "Each block processes " << confs_per_block << " configurations for a total of " << gridSize * confs_per_block << " configurations." << std::endl;
    cudaEventRecord(start, 0);
    auto cpu_start = std::chrono::high_resolution_clock::now();
    d_obb_coursened_two_stage<<<gridSize, blockSize>>>(d_R_obs, d_T_obs, d_R_rob, d_T_rob, d_Obs_dim, d_Rob_dim, d_Rob_conf_rot, d_Rob_conf_trans, pdisjoint);
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    float duration = 0;
    cudaEventElapsedTime(&duration, start, stop);
    cudaDeviceSynchronize();
    auto cpu_end = std::chrono::high_resolution_clock::now();
    double cpu_duration = std::chrono::duration<double, std::milli>(cpu_end - cpu_start).count();
        checkCudaMem(cudaGetLastError());
        // Copy result back to host (num_confs * sizeof(bool))
        checkCudaMem(cudaMemcpy(disjoint, pdisjoint, num_confs * sizeof(bool), cudaMemcpyDeviceToHost));

    std::vector<ConfigurationTagged> cpuCollisions(num_confs);
    // TIMEIT("Running Collision check on CPU", checkConfsCPU(cpuCollisions, confs);)
    checkConfsCPU(cpuCollisions, confs);
    // Check result
    size_t true_positives = 0; // num disjoint that are valid
    size_t false_positives = 0; // num disjoint that are not valid (should be 0)
    size_t false_negatives = 0; // num not disjoint that are valid (likely to be high since this is broad phase)
    size_t true_negatives = 0; // num not disjoint that are not valid (unsure how many of these will be)
    // if obb is disjoint, then we know for sure there is no collision
    // verify that the cpu said the same thing
    for (int i = 0; i < num_confs; ++i) {
        if (disjoint[i]) {
            if (cpuCollisions[i].valid) {
                true_positives++;
            } else {
                false_positives++;
                std::cout << "False positive at configuration " << i << ": "
                          << "Position (" << confs[i].x << ", " << confs[i].y << ", " << confs[i].z << "), " <<
                          "Orientation (roll: " << confs[i].roll << ", pitch: " << confs[i].pitch << ", yaw: " << confs[i].yaw << ")" << std::endl;

                //apply transformation to blank obb
                // rob_BVH.set(0, rob_rotations[i], rob_translations[i], rob_BVH.pDim[0]);
                std::vector<Eigen::Vector3f> vertices = rob_BVH.getBoxVertices(0);
                Eigen::Matrix3f R = rob_rotations[i];
                Eigen::Vector3f T = rob_translations[i];

                // for (auto p : vertices){
                //     std::cout << p.transpose() << " --> ";
                //     std::cout << (R * p + T).transpose() << std::endl;
                // }
                std::cout << "Invalid Robot Transform:" << std::endl;
                std::cout << pythonifyEigenMatrix(createHomogeneousMatrix(confs[i])) << std::endl;

            }
        }
        else {
            if (!cpuCollisions[i].valid) {
                true_negatives++;
            } else {
                false_negatives++;
            }
        }

    }
    std::cout << "Out of " << num_confs << " configurations, " << true_positives << " were true positives and " << false_positives << " were false positives." << std::endl;
    std::cout << "Out of " << num_confs << " configurations, " << true_negatives << " were true negatives and " << false_negatives << " were false negatives." << std::endl;

    int cpu_positives = 0;
    int cpu_negatives = 0;
    for (const auto& c : cpuCollisions) {
        if (c.valid) {
            cpu_positives++;
        } else {
            cpu_negatives++;
        }
    }
    std::cout << "CPU Collision checker found " << cpu_positives << " valid collisions and " << cpu_negatives << " invalid collisions." << std::endl;

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

double broad_coarsened_shared_mem_1S() {
    // Similar setup as broad_naive_1 but using d_obb_coursened_two_stage kernel
        // Load Robot and Obstacle BVH
    cudaEvent_t start, stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);
    OBB_soa rob_BVH = hierarchy_from_mesh("/home/victor/Projects/robo-check/data/models/alpha1.0/robot.obj");
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

    //just make a bunch of copies of the top level box to test memory performance
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
    const int blockSize = BLOCK_SIZE;
    const int confs_per_block = ONE_STAGE_CF * blockSize;
    const int gridSize = (num_confs + confs_per_block - 1) / confs_per_block; // ceil division

    // std::cout << "Launching coarsened_1S kernel with grid size " << gridSize << " and block size " << blockSize << std::endl;
    // std::cout << "Each block processes " << confs_per_block << " configurations for a total of " << gridSize * confs_per_block << " configurations." << std::endl;

    cudaEventRecord(start, 0);
    auto cpu_start = std::chrono::high_resolution_clock::now();
        d_obb_coursened_one_stage<<<gridSize, blockSize>>>(d_R_obs, d_T_obs, d_R_rob, d_T_rob, d_Obs_dim, d_Rob_dim, d_Rob_conf_rot, d_Rob_conf_trans, pdisjoint);
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    float duration = 0;
    cudaEventElapsedTime(&duration, start, stop);
    cudaDeviceSynchronize();
    checkCudaMem(cudaGetLastError());
    // Copy result back to host (num_confs * sizeof(bool))
    checkCudaMem(cudaMemcpy(disjoint, pdisjoint, num_confs * sizeof(bool), cudaMemcpyDeviceToHost));
    auto cpu_end = std::chrono::high_resolution_clock::now();
    double cpu_duration = std::chrono::duration<double, std::milli>(cpu_end - cpu_start).count();
    std::vector<ConfigurationTagged> cpuCollisions(num_confs);
    // TIMEIT("Running Collision check on CPU", checkConfsCPU(cpuCollisions, confs);)
    // checkConfsCPU(cpuCollisions, confs);
    // // Check result
    // size_t true_positives = 0; // num disjoint that are valid
    // size_t false_positives = 0; // num disjoint that are not valid (should be 0)
    // size_t false_negatives = 0; // num not disjoint that are valid (likely to be high since this is broad phase)
    // size_t true_negatives = 0; // num not disjoint that are not valid (unsure how many of these will be)
    // // if obb is disjoint, then we know for sure there is no collision
    // // verify that the cpu said the same thing
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
    // // std::cout << "Out of " << num_confs << " configurations, " << true_positives << " were true positives and " << false_positives << " were false positives." << std::endl;
    // // std::cout << "Out of " << num_confs << " configurations, " << true_negatives << " were true negatives and " << false_negatives << " were false negatives." << std::endl;

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


double bvh_naive(std::string rob_file, std::string obs_file, std::string conf_file) {
    // Similar setup as broad_naive_1 but using d_obb_coursened_two_stage kernel
        // Load Robot and Obstacle BVH
    cudaEvent_t start, stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);

    BVNode_soa rob_BVH = BVH_n_ary_hierarchy_from_mesh(rob_file.c_str(), 2);
    BVNode_soa obs_BVH = BVH_n_ary_hierarchy_from_mesh(obs_file.c_str(), 2);

    
    std::cout << "Rob BVH size: " << rob_BVH.size << std::endl;
    std::cout << "Obs BVH size: " << obs_BVH.size << std::endl;

    //Load Robot
    std::vector<Eigen::Vector3f> rob_vertices;
    std::vector<Triangle> rob_triangles;
    loadOBJFile(rob_file, rob_vertices, rob_triangles);

    //Load Obstacles
    std::vector<Eigen::Vector3f> obs_vertices;
    std::vector<Triangle> obs_triangles;
    loadOBJFile(obs_file, obs_vertices, obs_triangles);

    // Load Configurations
    const int num_confs = 100000;
    std::vector<Configuration> confs;
    confs.reserve(num_confs);
    readConfigurationFromFile(conf_file, confs);
    Eigen::Matrix3f rob_conf_r[num_confs];
    Eigen::Vector3f rob_conf_t[num_confs];
    for (int i = 0; i < num_confs; ++i) {
        rob_conf_r[i] = createRotationMatrix(confs[i]);
        rob_conf_t[i] = Eigen::Vector3f(confs[i].x, confs[i].y, confs[i].z);
    }

    std::vector<ConfigurationTagged> cpuCollisions(num_confs);
    TIMEIT("Running Collision check on CPU", checkConfsCPU(cpuCollisions, confs, rob_file, obs_file);)
    bool disjoint[num_confs] = {false};

    const int blockSize = BLOCK_SIZE;
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
    // std::cout << "Allocated host memory" << std::endl;

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
    cudaMalloc((void**)&d_Rob_vertices, rob_vertices.size() * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Obs_vertices, obs_vertices.size() * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Rob_triangles, rob_triangles.size() * sizeof(Triangle));
    cudaMalloc((void**)&d_Obs_triangles, obs_triangles.size() * sizeof(Triangle));
    cudaMalloc((void**)&pdisjoint, num_confs * sizeof(bool));

    cudaDeviceSynchronize();
    checkCudaMem(cudaMemcpy(d_R_obs, obs_BVH.pR, obs_BVH.size * sizeof(Eigen::Matrix3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_T_obs, obs_BVH.pT, obs_BVH.size * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_R_rob, rob_BVH.pR, rob_BVH.size * sizeof(Eigen::Matrix3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_T_rob, rob_BVH.pT, rob_BVH.size * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_conf_rot, rob_conf_r, num_confs * sizeof(Eigen::Matrix3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_conf_trans, rob_conf_t, num_confs * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_dim, rob_BVH.pDim, rob_BVH.size * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_dim, obs_BVH.pDim, obs_BVH.size * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_first_child, rob_BVH.first_child, rob_BVH.size * sizeof(int16_t), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_first_child, obs_BVH.first_child, obs_BVH.size * sizeof(int16_t), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_vertices, rob_vertices.data(), rob_vertices.size() * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_vertices, obs_vertices.data(), obs_vertices.size() * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_triangles, rob_triangles.data(), rob_triangles.size() * sizeof(Triangle), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_triangles, obs_triangles.data(), obs_triangles.size() * sizeof(Triangle), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(pdisjoint, disjoint, num_confs * sizeof(bool), cudaMemcpyHostToDevice));
    cudaDeviceSynchronize();
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    float duration = 0;
    cudaEventElapsedTime(&duration, start, stop);
    std::cout << "Data allocation and transfer to GPU took " << duration << " ms." << std::endl;
    // )
    // Launch kernel with correct grid size


    // std::cout << "Launching coarsened_1S kernel with grid size " << gridSize << " and block size " << blockSize << std::endl;
    // std::cout << "Each block processes " << confs_per_block << " configurations for a total of " << gridSize * confs_per_block << " configurations." << std::endl;

    cudaEventRecord(start, 0);
    auto cpu_start = std::chrono::high_resolution_clock::now();
    d_bvh_naive<<<gridSize, blockSize, (obs_BVH.size + rob_BVH.size) * sizeof(int16_t)>>>(   
                                            d_R_obs, d_T_obs, 
                                            d_R_rob, d_T_rob, 
                                            d_Obs_dim, d_Rob_dim, 
                                            d_Rob_conf_rot, d_Rob_conf_trans,
                                            d_Obs_first_child, d_Rob_first_child, 
                                            d_Rob_vertices, d_Rob_triangles, rob_BVH.size,
                                            d_Obs_vertices, d_Obs_triangles, obs_BVH.size,
                                            pdisjoint);
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&duration, start, stop);
    std::cout << "BVH Naive GPU broad phase took " << duration << " ms for " << num_confs << " configurations." << std::endl;

    // Copy result back to host (num_confs * sizeof(bool))
    cudaEventRecord(start, 0);
    checkCudaMem(cudaGetLastError());
    checkCudaMem(cudaMemcpy(disjoint, pdisjoint, num_confs * sizeof(bool), cudaMemcpyDeviceToHost));
    cudaDeviceSynchronize();
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&duration, start, stop);
    std::cout << "Copying results from GPU took " << duration << " ms." << std::endl;


    auto cpu_end = std::chrono::high_resolution_clock::now();
    double cpu_duration = std::chrono::duration<double, std::milli>(cpu_end - cpu_start).count();

    // checkConfsCPU(cpuCollisions, confs);
    // Check result
    size_t true_positives = 0; // num disjoint that are valid
    size_t false_positives = 0; // num disjoint that are not valid (should be 0)
    size_t false_negatives = 0; // num not disjoint that are valid (likely to be high since this is broad phase)
    size_t true_negatives = 0; // num not disjoint that are not valid (unsure how many of these will be)
    // if obb is disjoint, then we know for sure there is no collision
    // verify that the cpu said the same thing
    for (int i = 0; i < num_confs; ++i) {
        if (disjoint[i]) {
            if (cpuCollisions[i].valid) {
                true_positives++;
            } else {
                false_positives++;
                // std::cout << "False positive at configuration " << i << ": "
                //           << "Position (" << confs[i].x << ", " << confs[i].y << ", " << confs[i].z << "), " <<
                //           "Orientation (roll: " << confs[i].roll << ", pitch: " << confs[i].pitch << ", yaw: " << confs[i].yaw << ")" << std::endl;

                //apply transformation to blank obb
                // rob_BVH.set(0, rob_rotations[i], rob_translations[i], rob_BVH.pDim[0]);
                // std::vector<Eigen::Vector3f> vertices = rob_BVH.getBoxVertices(0);
                // Eigen::Matrix3f R = rob_conf_r[i];
                // Eigen::Vector3f T = rob_conf_t[i];

                // for (auto p : vertices){
                //     std::cout << p.transpose() << " --> ";
                //     std::cout << (R * p + T).transpose() << std::endl;
                // }
                // std::cout << "Invalid Robot Transform:" << std::endl;
                // std::cout << pythonifyEigenMatrix(createHomogeneousMatrix(confs[i])) << std::endl;

            }
        }
        else {
            if (!cpuCollisions[i].valid) {
                true_negatives++;
            } else {
                false_negatives++;
                // std::cout << "False negative at configuration " << i << ": "
                //           << "Position (" << confs[i].x << ", " << confs[i].y << ", " << confs[i].z << "), " <<
                //           "Orientation (roll: " << confs[i].roll << ", pitch: " << confs[i].pitch << ", yaw: " << confs[i].yaw << ")" << std::endl;
            }
        }
    }
    std::cout << "for BVH traversal, Out of " << num_confs << " configurations, " << true_positives << " were true positives and " << false_positives << " were false positives." << std::endl;
    std::cout << "for BVH traversal, Out of " << num_confs << " configurations, " << true_negatives << " were true negatives and " << false_negatives << " were false negatives." << std::endl;

    int cpu_positives = 0;
    int cpu_negatives = 0;
    for (const auto& c : cpuCollisions) {
        if (c.valid) {
            cpu_positives++;
        } else {
            cpu_negatives++;
        }
    }
    std::cout << "CPU Collision checker found " << cpu_positives << " valid collisions and " << cpu_negatives << " invalid collisions." << std::endl;

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

int main(int argc, char** argv) {

    std::string rob_file  = "/home/victor/Projects/robo-check/data/models/alpha1.0/robot.obj";
    std::string obs_file  = "/home/victor/Projects/robo-check/data/models/alpha1.0/obstacle.obj";
    std::string conf_file = "/home/victor/Projects/robo-check/data/configurations/hard_confs100,000.conf";

    if (argc > 1) {rob_file  = argv[1];}
    if (argc > 2) {obs_file  = argv[2];}
    if (argc > 3) {conf_file = argv[3];}
    
    if (argc > 4) {
        std::cerr << "Usage: " << argv[0] << " [robot.obj] [obstacle.obj] [config.conf]\n";
        return 1;
    }

    // Validate files
    bool ok = true;
    ok &= check_file_exists(rob_file);
    ok &= check_file_exists(obs_file);
    ok &= check_file_exists(conf_file);

    if (!ok) {
        std::cerr << "Aborting due to invalid input files.\n";
        return 1;
    }

    // dummy run to "warm up" the GPU and avoid initialization overhead in timing

    // const size_t num_trials = 1000;
    // // double naive_times[num_trials];
    // double coarsened_1S_times[num_trials];
    // double coarsened_2S_times[num_trials];

    // flushCudaCache();

    //test BVH_nary_hierarchy_from_mesh
    // std::cout << "Starting Naive Broad Phase OBB Tests..." << std::endl;
    // OBB_soa rob_BVH = hierarchy_from_mesh("/home/victor/Projects/robo-check/data/models/alpha1.0/robot.obj");
    // std::cout << "Robot has "<< rob_BVH.size << " nodes." << std::endl;
    // std::cout<< "Robot 2nary BVH:" << std::endl;
    // BVH_n_ary_hierarchy_from_mesh("/home/victor/Projects/robo-check/data/models/alpha1.0/robot.obj", 2);
    // std::cout<< "Obstacle 2nary BVH:" << std::endl;
    // BVH_n_ary_hierarchy_from_mesh("/home/victor/Projects/robo-check/data/models/alpha1.0/obstacle.obj", 2);
    
    bvh_naive(rob_file, obs_file, conf_file);
    // broad_coarsened_shared_mem_1S();
    // broad_coarsened_shared_mem_2S();
    // for (size_t i = 0; i < num_trials; ++i) {
    //     std::cout << "\rIteration " << i + 1 << " / " << num_trials;
    //     naive_times[i] = broad_naive_1();
    //     std::cout.flush();
    // }
    // std::cout << std::endl;

    // flushCudaCache();
    // std::cout << "Starting Coarsened 2-Stage Broad Phase OBB Tests..." << std::endl;
    // broad_coarsened_shared_mem_2S();
    // for (size_t i = 0; i < num_trials; ++i) {
    //     std::cout << "\rIteration " << i + 1 << " / " << num_trials;
    //     coarsened_2S_times[i] = broad_coarsened_shared_mem_2S();
    //     std::cout.flush();
    // }

    // flushCudaCache();

    // std::cout << std::endl;
    // std::cout << "Starting Coarsened 1-Stage Broad Phase OBB Tests..." <<  std::endl;
    // broad_coarsened_shared_mem_1S();
    // for (size_t i = 0; i < num_trials; ++i) {
    //     std::cout << "\rIteration " << i + 1 << " / " << num_trials;
    //     coarsened_1S_times[i] = broad_coarsened_shared_mem_1S();
    //     std::cout.flush();
    // }
    // double avg_naive = std::accumulate(naive_times, naive_times + num_trials, 0.0) / num_trials;
    // double avg_coarsened_1S = std::accumulate(coarsened_1S_times, coarsened_1S_times + num_trials, 0.0) / num_trials;
    // double avg_coarsened_2S = std::accumulate(coarsened_2S_times, coarsened_2S_times + num_trials, 0.0) / num_trials;

    // std::cout << "Average time over " << num_trials << " trials:" << std::endl;
    // std::cout << " Naive Broad Phase OBB: " << avg_naive << " microseconds" << std::endl;
    // std::cout << " Coarsened 1-Stage Broad Phase OBB: " << avg_coarsened_1S << " microseconds" << std::endl;
    // std::cout << " Coarsened 2-Stage Broad Phase OBB: " << avg_coarsened_2S << " microseconds" << std::endl;
}

