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

        // // print out the values to verify
        // std::cout << "OBB " << i << ":\n";
        // std::cout << "Rotation:\n" << rotation << "\n";
        // std::cout << "Translation:\n" << translation.transpose() << "\n";
        // std::cout << "Half Dimensions:\n" << half_dimensions.transpose() << "\n";
        // std::cout << "-----------------------\n";
        result.set(i, rotation, translation, half_dimensions);
    }

    // delete rob_mesh manually to free memory
    rob_mesh.reset();

    //verify that the data was copied correctly
    for (int i = 0; i < num_boxes; ++i) {
        std::cout << "Verifying OBB " << i << ":\n";
        std::cout << "Rotation:\n" << result.pR[i] << "\n";
        std::cout << "Translation:\n" << result.pT[i].transpose() << "\n";
        std::cout << "Half Dimensions:\n" << result.pDim[i].transpose() << "\n";
        std::cout << "-----------------------\n";
    }



    return result;
}

// Note: This assumes that OBB B has rotation and translation with respect to A
// will this be feasible to use in practice?
__global__ void d_obbDisjoint(  const Eigen::Matrix3f* pB, const Eigen::Vector3f* pT,
                                const Eigen::Vector3f* pa, const Eigen::Vector3f* pb, bool* pdisjoint) {

    // Read data
    Eigen::Matrix3f B = pB[blockIdx.x];
    Eigen::Vector3f T = pT[blockIdx.x];
    Eigen::Vector3f a = pa[blockIdx.x];
    Eigen::Vector3f b = pb[blockIdx.x]; 

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

}


//TODO: idea -- half float for obb, full float for triangle overlap?
int main() {
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

    OBB_soa obb_data = hierarchy_from_mesh("/home/victor/Projects/robo-check/data/models/alpha1.0/robot.obj");
    obb_data.writeToFile("robot_OBB_BVH.obj");
    // Copy data to device
    cudaMemcpy(pB, &B, sizeof(Eigen::Matrix3f), cudaMemcpyHostToDevice);
    cudaMemcpy(pT, &T, sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice);
    cudaMemcpy(pa, &a, sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice);
    cudaMemcpy(pb, &b, sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice);
    cudaMemcpy(pdisjoint, disjoint, sizeof(bool), cudaMemcpyHostToDevice);
    
    // Launch kernel
    d_obbDisjoint<<<num_boxes, 1>>>(pB, pT, pa, pb, pdisjoint);

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

