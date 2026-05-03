#include <iostream>
#include <Eigen/Dense>
#include <fcl/fcl.h>
#include "Utils.h"
#include "OBB-BVH-naive.hu"
#include "OBB-single-buff.hu"
#include "OBB-double-buff.hu"
#include "OBB-naive.hu"

#include "Triangle.hu"
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

