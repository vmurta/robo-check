#include <iostream>
#include <chrono>
#include <Eigen/Dense>
#include <fcl/fcl.h>
#include "Utils.h"
#include "OBB-BVH-naive.hu"
#include "Edge.hu"
#include "OBB-single-buff.hu"
#include "OBB-double-buff.hu"
#include "OBB-naive.hu"

#include "Triangle.hu"

#define WARP_SIZE 32

static void runEdgeBenchmark(const std::string& rob_file, const std::string& obs_file,
                             const std::string& conf_file, float lvs, int max_edges,
                             bool dry_run, bool cpu_check, bool collab) {
    // Build BVH hierarchies and load mesh data
    BVNode_soa rob_BVH = BVH_n_ary_hierarchy_from_mesh(rob_file.c_str(), 2);
    BVNode_soa obs_BVH = BVH_n_ary_hierarchy_from_mesh(obs_file.c_str(), 2);

    MeshData rob_mesh;
    loadOBJFile(rob_file, rob_mesh.vertices, rob_mesh.triangles);

    MeshData obs_mesh;
    loadOBJFile(obs_file, obs_mesh.vertices, obs_mesh.triangles);

    // Load configurations and pair consecutive ones into edges
    std::vector<Configuration> confs;
    readConfigurationFromFile(conf_file, confs);

    const size_t max_e = (confs.size() > 1)
        ? ((max_edges > 0) ? std::min<size_t>((size_t)max_edges, confs.size() - 1) : (confs.size() - 1))
        : 0;
    std::vector<Edge> edges(max_e);
    for (size_t i = 0; i < max_e; ++i) {
        edges[i] = makeEdge(confs[i], confs[i + 1], EDGE_TYPE_LINEAR_6D, lvs);
        edges[i].flags = EDGE_FLAG_REPORT_FIRST_INVALID_T;
    }
    EdgeBatch batch(edges);

    size_t total_samples = 0;
    for (const auto& e : edges) total_samples += (size_t)e.nsteps + 1;
    std::cout << "Edge validation: " << max_e << " edges and " << total_samples
              << " samples (lvs=" << lvs << ")." << std::endl;

    std::vector<bool> valid;
    std::vector<float> first_invalid_t;
    if (collab) {
        bvh_edges_collab(rob_BVH, obs_BVH, rob_mesh, obs_mesh, batch, valid, first_invalid_t, dry_run);
    } else {
        bvh_edges(rob_BVH, obs_BVH, rob_mesh, obs_mesh, batch, valid, first_invalid_t, dry_run);
    }

    if (cpu_check) {
        std::vector<bool> cpuValid;
        std::vector<float> cpuFit;
        checkEdgesCPU(edges, cpuValid, cpuFit, rob_file, obs_file);

        size_t true_positives = 0;   // GPU valid, CPU valid
        size_t false_positives = 0;  // GPU valid, CPU invalid (should be 0)
        size_t true_negatives = 0;   // GPU invalid, CPU invalid
        size_t false_negatives = 0;  // GPU invalid, CPU valid (should be 0)
        for (size_t i = 0; i < edges.size(); ++i) {
            if (valid[i]) {
                if (cpuValid[i]) {
                    true_positives++;
                } else {
                    false_positives++;
                    std::cout << "False positive at edge " << i << ": "
                              << "from (" << confs[i].x << ", " << confs[i].y << ", " << confs[i].z << ") "
                              << "to (" << confs[i + 1].x << ", " << confs[i + 1].y << ", " << confs[i + 1].z << ")" << std::endl;
                }
            } else {
                if (!cpuValid[i]) {
                    true_negatives++;
                } else {
                    false_negatives++;
                    std::cout << "False negative at edge " << i << ": "
                              << "from (" << confs[i].x << ", " << confs[i].y << ", " << confs[i].z << ") "
                              << "to (" << confs[i + 1].x << ", " << confs[i + 1].y << ", " << confs[i + 1].z << ")" << std::endl;
                }
            }
        }

        std::cout << "for edge validation, Out of " << edges.size() << " edges, " << true_positives << " were true positives and " << false_positives << " were false positives." << std::endl;
        std::cout << "for edge validation, Out of " << edges.size() << " edges, " << true_negatives << " were true negatives and " << false_negatives << " were false negatives." << std::endl;

        size_t cpu_positives = 0;
        size_t cpu_negatives = 0;
        for (const bool v : cpuValid) {
            if (v) {
                cpu_positives++;
            } else {
                cpu_negatives++;
            }
        }
        std::cout << "CPU edge validator found " << cpu_positives << " valid edges and " << cpu_negatives << " invalid edges." << std::endl;
    }
}

void print_usage(const char *prog) {
    std::cout << "Usage: " << prog << " [options] [robot.obj] [obstacle.obj] [config.conf]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --algo <name>   Algorithm to run (default: bvh)" << std::endl;
    std::cout << "    bvh           BVH naive hierarchical traversal (default)" << std::endl;
    std::cout << "    edge          Edge validation (consecutive config pairs as edges)" << std::endl;
    std::cout << "    edge-collab   Collaborative edge validation (pooled root checks)" << std::endl;
    std::cout << "    obb-1s        OBB coarsened single-stage" << std::endl;
    std::cout << "    obb-2s        OBB coarsened two-stage" << std::endl;
    std::cout << "  --lvs <float>   Longest valid segment for edge nsteps (default: 0.05)" << std::endl;
    std::cout << "  --max-edges <n> Cap the number of edges (default: 10000, 0 = all)" << std::endl;
    std::cout << "  -h, --help      Show this help message" << std::endl;
    std::cout << "  --dry-run       Warm up the kernel with an untimed launch before timing" << std::endl;
    std::cout << "  --cpu-check     Also run the CPU collision check and report true/false positives/negatives" << std::endl;
}

int main(int argc, char** argv) {

    std::string rob_file  = "./data/models/alpha1.0/robot.obj";
    std::string obs_file  = "./data/models/alpha1.0/obstacle.obj";
    std::string conf_file = "./data/configurations/hard_confs100,000.conf";
    std::string algo = "bvh";
    bool dry_run = false;
    bool cpu_check = false;
    float lvs = 0.05f;
    int max_edges = 10000;

    // Parse arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--algo" && i + 1 < argc) {
            algo = argv[++i];
        } else if (arg == "--lvs" && i + 1 < argc) {
            lvs = std::stof(argv[++i]);
        } else if (arg == "--max-edges" && i + 1 < argc) {
            max_edges = std::stoi(argv[++i]);
        } else if (arg == "--dry-run") {
            dry_run = true;
        } else if (arg == "--cpu-check") {
            cpu_check = true;
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
        // Build BVH hierarchies and load mesh data
        // DEBUG: builder selection via env ROB_BUILDER / OBS_BUILDER (fcl|paper)
        const char* rob_builder = getenv("ROB_BUILDER");
        const char* obs_builder = getenv("OBS_BUILDER");
        BVNode_soa rob_BVH = (rob_builder && strcmp(rob_builder, "fcl") == 0)
            ? BVH_fcl_hierarchy_from_mesh(rob_file.c_str(), 2)
            : BVH_n_ary_hierarchy_from_mesh(rob_file.c_str(), 2);
        BVNode_soa obs_BVH = (obs_builder && strcmp(obs_builder, "fcl") == 0)
            ? BVH_fcl_hierarchy_from_mesh(obs_file.c_str(), 2)
            : BVH_n_ary_hierarchy_from_mesh(obs_file.c_str(), 2);

        MeshData rob_mesh;
        loadOBJFile(rob_file, rob_mesh.vertices, rob_mesh.triangles);

        MeshData obs_mesh;
        loadOBJFile(obs_file, obs_mesh.vertices, obs_mesh.triangles);

        // Load configurations
        std::vector<Configuration> confs;
        readConfigurationFromFile(conf_file, confs);

        std::vector<bool> valid;
        bvh_naive(rob_BVH, obs_BVH, rob_mesh, obs_mesh, confs, valid, dry_run);

        if (cpu_check) {
            std::vector<ConfigurationTagged> cpuCollisions(confs.size());
            checkConfsCPU(cpuCollisions, confs, rob_file, obs_file);

            size_t true_positives = 0;   // GPU valid, CPU valid
            size_t false_positives = 0;  // GPU valid, CPU invalid (should be 0)
            size_t true_negatives = 0;   // GPU invalid, CPU invalid
            size_t false_negatives = 0;  // GPU invalid, CPU valid (expected: broad phase)
            for (size_t i = 0; i < confs.size(); ++i) {
                if (valid[i]) {
                    if (cpuCollisions[i].valid) {
                        true_positives++;
                    } else {
                        false_positives++;
                                        std::cout << "False positive at configuration " << i << ": "
                          << "Position (" << confs[i].x << ", " << confs[i].y << ", " << confs[i].z << "), " <<
                          "Orientation (roll: " << confs[i].roll << ", pitch: " << confs[i].pitch << ", yaw: " << confs[i].yaw << ")" << std::endl;

                        // apply transformation to blank obb
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
                } else {
                    if (!cpuCollisions[i].valid) {
                        true_negatives++;
                    } else {
                        false_negatives++;
                        std::cout << "False negative at configuration " << i << ": "
                          << "Position (" << confs[i].x << ", " << confs[i].y << ", " << confs[i].z << "), " <<
                          "Orientation (roll: " << confs[i].roll << ", pitch: " << confs[i].pitch << ", yaw: " << confs[i].yaw << ")" << std::endl;
                    }
                }
            }
            
            std::cout << "for BVH traversal, Out of " << confs.size() << " configurations, " << true_positives << " were true positives and " << false_positives << " were false positives." << std::endl;
            std::cout << "for BVH traversal, Out of " << confs.size() << " configurations, " << true_negatives << " were true negatives and " << false_negatives << " were false negatives." << std::endl;

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
        }
    } else if (algo == "edge") {
        runEdgeBenchmark(rob_file, obs_file, conf_file, lvs, max_edges, dry_run, cpu_check, false);
    } else if (algo == "edge-collab") {
        runEdgeBenchmark(rob_file, obs_file, conf_file, lvs, max_edges, dry_run, cpu_check, true);
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
