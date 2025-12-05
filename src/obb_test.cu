#include <iostream>
#include <Eigen/Dense>
#include <fcl/fcl.h>
#include "./Utils.h"

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

    // Access OBB data from rob_mesh
    // rob_mesh->getNumBVs() gives the number of OBBs in the hierarchy
    size_t num_boxes = rob_mesh->getNumBVs();
    OBB_soa result(num_boxes);

    Eigen::Matrix3f rotation;
    Eigen::Vector3f translation;
    Eigen::Vector3f half_dimensions;
    for (int i = 0; i < num_boxes; ++i) {
        fcl::OBB<float> obb = rob_mesh->getBV(i).bv;
        rotation = obb.axis;
        translation = obb.To;
        half_dimensions = obb.extent;
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
    const float epsilon = 1e-6f; // small value to avoid numerical issues
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
__device__ inline bool contains(int val, const int* arr, size_t arr_size){
    for (size_t i = 0; i < arr_size; i++){
        if (arr[i] == val){
            return true;
        }
    }
    return false;
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

    Eigen::Vector3f a = pRob_dim[index]; // half dimensions of box A
    Eigen::Vector3f b = pObs_dim[index]; // half dimensions of box B

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

    Eigen::Vector3f a = pRob_dim[index]; // half dimensions of box A
    Eigen::Vector3f b = pObs_dim[index]; // half dimensions of box B


    
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

    Eigen::Vector3f a = pRob_dim[index]; // half dimensions of box A
    Eigen::Vector3f b = pObs_dim[index]; // half dimensions of box B


    
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
    OBB_soa rob_BVH = hierarchy_from_mesh("/home/victor/Projects/robo-check/data/models/alpha1.0/robot.obj");
    OBB_soa obs_BVH = hierarchy_from_mesh("/home/victor/Projects/robo-check/data/models/alpha1.0/obstacle.obj");

    // Load Configurations
    const int num_confs = 100000;
    std::vector<Configuration> confs;
    confs.reserve(num_confs);
    // std::cout << "Reading configurations from file..." << std::endl;
    readConfigurationFromFile("/home/victor/Projects/robo-check/data/configurations/easy_confs100,000.conf", confs);
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
    readConfigurationFromFile("/home/victor/Projects/robo-check/data/configurations/easy_confs100,000.conf", confs);
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
    readConfigurationFromFile("/home/victor/Projects/robo-check/data/configurations/easy_confs100,000.conf", confs);
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

int main() {
    // dummy run to "warm up" the GPU and avoid initialization overhead in timing

    const size_t num_trials = 1000;
    double naive_times[num_trials];
    double coarsened_1S_times[num_trials];
    double coarsened_2S_times[num_trials];

    flushCudaCache();

    std::cout << "Starting Naive Broad Phase OBB Tests..." << std::endl;
    broad_naive_1();
    for (size_t i = 0; i < num_trials; ++i) {
        std::cout << "\rIteration " << i + 1 << " / " << num_trials;
        naive_times[i] = broad_naive_1();
        std::cout.flush();
    }
    std::cout << std::endl;

    flushCudaCache();
    std::cout << "Starting Coarsened 2-Stage Broad Phase OBB Tests..." << std::endl;
    broad_coarsened_shared_mem_2S();
    for (size_t i = 0; i < num_trials; ++i) {
        std::cout << "\rIteration " << i + 1 << " / " << num_trials;
        coarsened_2S_times[i] = broad_coarsened_shared_mem_2S();
        std::cout.flush();
    }

    flushCudaCache();

    std::cout << std::endl;
    std::cout << "Starting Coarsened 1-Stage Broad Phase OBB Tests..." <<  std::endl;
    broad_coarsened_shared_mem_1S();
    for (size_t i = 0; i < num_trials; ++i) {
        std::cout << "\rIteration " << i + 1 << " / " << num_trials;
        coarsened_1S_times[i] = broad_coarsened_shared_mem_1S();
        std::cout.flush();
    }
    double avg_naive = std::accumulate(naive_times, naive_times + num_trials, 0.0) / num_trials;
    double avg_coarsened_1S = std::accumulate(coarsened_1S_times, coarsened_1S_times + num_trials, 0.0) / num_trials;
    double avg_coarsened_2S = std::accumulate(coarsened_2S_times, coarsened_2S_times + num_trials, 0.0) / num_trials;
    
    std::cout << "Average time over " << num_trials << " trials:" << std::endl;
    std::cout << " Naive Broad Phase OBB: " << avg_naive << " microseconds" << std::endl;
    std::cout << " Coarsened 1-Stage Broad Phase OBB: " << avg_coarsened_1S << " microseconds" << std::endl;
    std::cout << " Coarsened 2-Stage Broad Phase OBB: " << avg_coarsened_2S << " microseconds" << std::endl;
}

