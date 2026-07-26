#include <iostream>
#include <Eigen/Dense>
#include <fcl/fcl.h>
#include "Utils.h"
#include "OBB-BVH-naive.hu"
#include "OBB-single-buff.hu"
#include "OBB-double-buff.hu"
#include "OBB-naive.hu"

#include "Triangle.hu"

#define WARP_SIZE 32

void print_usage(const char *prog) {
    std::cout << "Usage: " << prog << " [options] [robot.obj] [obstacle.obj] [config.conf]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --algo <name>   Algorithm to run (default: bvh)" << std::endl;
    std::cout << "    bvh           BVH naive hierarchical traversal (default)" << std::endl;
    std::cout << "    obb-1s        OBB coarsened single-stage" << std::endl;
    std::cout << "    obb-2s        OBB coarsened two-stage" << std::endl;
    std::cout << "  -h, --help      Show this help message" << std::endl;
}

int main(int argc, char** argv) {

    std::string rob_file  = "./data/models/alpha1.0/robot.obj";
    std::string obs_file  = "./data/models/alpha1.0/obstacle.obj";
    std::string conf_file = "./data/configurations/hard_confs100,000.conf";
    std::string algo = "bvh";

    // Parse arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--algo" && i + 1 < argc) {
            algo = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return 0;
        } else if (arg[0] != '-') {
            // Positional arguments
            if (i == argc - 3 || (i + 2 < argc && std::string(argv[i+1])[0] != '-' && std::string(argv[i+2])[0] != '-')) {
                rob_file = argv[i];
            } else if (i == argc - 2 || (i + 1 < argc && std::string(argv[i+1])[0] != '-')) {
                obs_file = argv[i];
            } else if (i == argc - 1) {
                conf_file = argv[i];
            }
        } else {
            std::cerr << "Unknown argument: " << arg << std::endl;
            print_usage(argv[0]);
            return 1;
        }
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

    std::cout << "Robot:  " << rob_file << std::endl;
    std::cout << "Obstacle: " << obs_file << std::endl;
    std::cout << "Config: " << conf_file << std::endl;
    std::cout << "Algorithm: " << algo << std::endl;

    if (algo == "bvh") {
        bvh_naive(rob_file, obs_file, conf_file);
    } else if (algo == "obb-1s") {
        broad_coarsened_shared_mem_1S();
    } else if (algo == "obb-2s") {
        broad_coarsened_shared_mem_2S();
    } else {
        std::cerr << "Unknown algorithm: " << algo << std::endl;
        std::cerr << "Available algorithms: bvh, obb-1s, obb-2s" << std::endl;
        return 1;
    }
}
