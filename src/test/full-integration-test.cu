#include "../broad-phase/broad-phase-fused.hu"
#include "../narrow-phase/narrow-phase.hu"
#include "./MegaKernel.hu"

#include <fcl/fcl.h>
#include "../Utils.h"

#ifdef COALESCE

__constant__ float base_rob_x[NUM_ROB_VERTICES];
__constant__ float base_rob_y[NUM_ROB_VERTICES];
__constant__ float base_rob_z[NUM_ROB_VERTICES];
__constant__ int base_rob_tri_v1[MAX_NUM_ROBOT_TRIANGLES];
__constant__ int base_rob_tri_v2[MAX_NUM_ROBOT_TRIANGLES];
__constant__ int base_rob_tri_v3[MAX_NUM_ROBOT_TRIANGLES];
__constant__ float base_obs_x[NUM_ROB_VERTICES];
__constant__ float base_obs_y[NUM_ROB_VERTICES];
__constant__ float base_obs_z[NUM_ROB_VERTICES];
__constant__ int base_obs_tri_v1[MAX_NUM_ROBOT_TRIANGLES];
__constant__ int base_obs_tri_v2[MAX_NUM_ROBOT_TRIANGLES];
__constant__ int base_obs_tri_v3[MAX_NUM_ROBOT_TRIANGLES];

#else
__constant__ Vector3f base_robot_vertices[NUM_ROB_VERTICES];
__constant__ Triangle base_robot_triangles[MAX_NUM_ROBOT_TRIANGLES];
__constant__ Vector3f base_obs_vertices[NUM_ROB_VERTICES];
__constant__ Triangle base_obs_triangles[MAX_NUM_ROBOT_TRIANGLES];
#endif


inline bool verticesEqual(const Vector3f &v1, const fcl::Vector3f &v2){
  return (fabs(v1.x -v2[0]) +
            fabs(v1.y -v2[1]) +
            fabs(v1.z -v2[2]) < 1e-5);
  // return false;
}

void generateAABBBaseline_fcl(fcl::Vector3f* vertices, unsigned int numVertices,
                    unsigned int numConfigs, AABB* botBounds)
{
    // Loop over every configuration
    for(int i = 0; i < numConfigs; ++i)
    {
        // Loop over every vertex in each configuration
        unsigned int configOffset = i * numVertices;
        botBounds[i].x_min = vertices[configOffset][0];
        botBounds[i].y_min = vertices[configOffset][1];
        botBounds[i].z_min = vertices[configOffset][2];
        botBounds[i].x_max = vertices[configOffset][0];
        botBounds[i].y_max = vertices[configOffset][1];
        botBounds[i].z_max = vertices[configOffset][2];
        for(int j = 0; j < numVertices; ++j)
        {
            botBounds[i].x_min = min(botBounds[i].x_min, vertices[configOffset + j][0]);
            botBounds[i].y_min = min(botBounds[i].y_min, vertices[configOffset + j][1]);
            botBounds[i].z_min = min(botBounds[i].z_min, vertices[configOffset + j][2]);
            botBounds[i].x_max = max(botBounds[i].x_max, vertices[configOffset + j][0]);
            botBounds[i].y_max = max(botBounds[i].y_max, vertices[configOffset + j][1]);
            botBounds[i].z_max = max(botBounds[i].z_max, vertices[configOffset + j][2]);
        }
    }
}

bool verify_generateAABB(AABB* botBoundsBaseline, AABB* botBoundsParallel, const int numConfigs)
{
    int num_correct = 0;
    int num_incorrect = 0;
    float running_error = 0;
    for(int i = 0; i < numConfigs; ++i)
    {
        float error = fabs (botBoundsBaseline[i].x_min - botBoundsParallel[i].x_min) +
                fabs (botBoundsBaseline[i].y_min - botBoundsParallel[i].y_min) +
                fabs (botBoundsBaseline[i].z_min - botBoundsParallel[i].z_min) +
                fabs (botBoundsBaseline[i].x_max - botBoundsParallel[i].x_max) +
                fabs (botBoundsBaseline[i].y_max - botBoundsParallel[i].y_max) +
                fabs (botBoundsBaseline[i].z_max - botBoundsParallel[i].z_max);

        if (error < 1E-4){
            num_correct++;
        } else {
            num_incorrect++;
            running_error+=error;
            std::cout << "Baseline x_min: " << botBoundsBaseline[i].x_min << "\tGPU x_min " << botBoundsParallel[i].x_min << std::endl; 
            std::cout << "Baseline y_min: " << botBoundsBaseline[i].y_min << "\tGPU y_min " << botBoundsParallel[i].y_min << std::endl; 
            std::cout << "Baseline z_min: " << botBoundsBaseline[i].z_min << "\tGPU z_min " << botBoundsParallel[i].z_min << std::endl; 
            std::cout << "Baseline x_max: " << botBoundsBaseline[i].x_max << "\tGPU x_max " << botBoundsParallel[i].x_max << std::endl; 
            std::cout << "Baseline y_max: " << botBoundsBaseline[i].y_max << "\tGPU y_max " << botBoundsParallel[i].y_max << std::endl; 
            std::cout << "Baseline z_max: " << botBoundsBaseline[i].z_max << "\tGPU z_max " << botBoundsParallel[i].z_max << std::endl;
        }
    }
    float avg_error = running_error / num_incorrect;
    std::cout << "Num correct AABBs: " << num_correct <<std::endl;
    std::cout << "Num incorrect AABBs: " << num_incorrect <<std::endl;
    std::cout << "Average Error " << avg_error <<std::endl;
    return true;
}

void verifyConfs(bool *confs, size_t num_confs) {
    size_t numValidConfs = 0;

    for (size_t i = 0; i < num_confs; i++) {
        if (confs[i]) numValidConfs++;
    }

    // std::cout << "Valid configurations: " << numValidConfs << " (out of " << num_confs << ")" << std::endl;
}

void collisionCheckCPU(bool *valid, std::string confFile){

    // load configurations, should have  valids and 3010 invalids
    std::vector<Configuration> confs;
    readConfigurationFromFile(confFile, confs);
    // createAlphaBotConfigurations(confs, confs.size());

    //Load Robot
    std::vector<fcl::Vector3f> rob_vertices;
    std::vector<fcl::Triangle> rob_triangles;
    std::vector<fcl::Vector3f> obs_vertices;
    std::vector<fcl::Triangle> obs_triangles;

    loadOBJFileFCL("../models/alpha1.0/robot.obj", rob_vertices, rob_triangles);
    loadOBJFileFCL("../models/alpha1.0/obstacle.obj", obs_vertices, obs_triangles);

    std::cout << "robot has " << rob_vertices.size() << " vertices " <<std::endl;
    auto cpu_start_time = std::chrono::high_resolution_clock::now();

    std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<float>>> rob_mesh(new fcl::BVHModel<fcl::OBBRSS<float>>);
    rob_mesh->beginModel(rob_triangles.size(), rob_vertices.size());
    rob_mesh->addSubModel(rob_vertices, rob_triangles);
    rob_mesh->endModel();

    // Load Obstacle
    std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<float>>> obs_mesh(new fcl::BVHModel<fcl::OBBRSS<float>>);
    obs_mesh->beginModel(obs_triangles.size(), obs_vertices.size());
    obs_mesh->addSubModel(obs_vertices, obs_triangles);
    obs_mesh->endModel();

    fcl::CollisionObject<float> rob_col_obj(rob_mesh);
    fcl::CollisionObject<float> obs_col_obj(obs_mesh);


    // ************************************************************************//

    int num_valid = 0;
    int num_invalid = 0;
    // perform collision detection on each of the randomly generated configs
    for(int i = 0; i < confs.size(); i++){
      fcl::Transform3f transform = configurationToTransform(confs[i]);
      rob_col_obj.setTransform(transform);
        // if (i == 9999){
        //     std::cout << "CPU Configuration 9999" << std::endl;
        //     for ( auto vertex : rob_vertices){
        //         fcl::Vector3f transformed_vertex = transform * vertex;
        //         std:: cout << transformed_vertex[0] << ", " << transformed_vertex[1] << ", " << transformed_vertex[2] << std::endl;
        //     }
        // }
      // Define CollisionRequest and CollisionResult objects
      fcl::CollisionRequest<float> request;
      fcl::CollisionResult<float> result;

      // // Perform collision detection
      fcl::collide(&obs_col_obj, &rob_col_obj, request, result);

      // Check if collision occurred
      if (result.isCollision()) {
        valid[i]= false;
      } else {
        valid[i]=true;
      }
    }
    auto cpu_end_time = std::chrono::high_resolution_clock::now();
    auto cpu_elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(cpu_end_time - cpu_start_time);
    std::cout << "cpu collision detection execution time: " << cpu_elapsed_time.count() << " milliseconds" << std::endl;

}

void transformCPU(AABB* bot_bounds, std::vector<Configuration> &confs){
    //Load Robot
    std::vector<fcl::Vector3f> fcl_rob_vertices;
    std::vector<fcl::Triangle> fcl_rob_triangles;
    loadOBJFileFCL("./models/alpha1.0/robot.obj", fcl_rob_vertices, fcl_rob_triangles);
    std::cout << "robot has " << fcl_rob_vertices.size() << " vertices " <<std::endl;

    std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<float>>> rob_mesh(new fcl::BVHModel<fcl::OBBRSS<float>>);
    rob_mesh->beginModel(fcl_rob_triangles.size(), fcl_rob_vertices.size());
    rob_mesh->addSubModel(fcl_rob_vertices, fcl_rob_triangles);
    rob_mesh->endModel();
    std::cout << "loaded robot" <<std::endl;

    fcl::Vector3f* vertices = new fcl::Vector3f[confs.size() * NUM_ROB_VERTICES];

    for (int i = 0; i < confs.size(); i++){
        fcl::Transform3f transform = configurationToTransform(confs[i]);
        for (int j = 0; j < fcl_rob_vertices.size(); j++){
            vertices[i * fcl_rob_vertices.size() + j] = transform * fcl_rob_vertices[j];
        }
    }

    generateAABBBaseline_fcl(vertices, fcl_rob_vertices.size(), confs.size(), bot_bounds);
}

//TODO: refactor code to minimize loads by interleaving file reads and device memory operations
int main(int argc, char *argv[])
{
      // load configurations, should have 6990 valids and 3010 invalids
    std::vector<Configuration> confs;
        if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <configuration file>" << std::endl;
        return 1;
    }

    std::string confFile = argv[1];

    readConfigurationFromFile(confFile, confs);
    std::cout << "Read " << confs.size() << " configurations" << std::endl;

    Vector3f* gpu_transformed_vertices = new Vector3f[confs.size() * 792];
    AABB* bot_bounds_GPU = new AABB[confs.size()];

    bool *valid_conf = new bool[confs.size()];

    for (int i = 0; i < confs.size(); i++){
        valid_conf[i] = false;
    }
    bool *cpu_valid_conf = new bool[confs.size()];
    AABB* bot_bounds_CPU = new AABB[confs.size()];
    fcl::Vector3f* cpu_transformed_vertices = new fcl::Vector3f[confs.size() * 792];

    auto cpu_start_time = std::chrono::high_resolution_clock::now();
    collisionCheckCPU(cpu_valid_conf, confFile);
    auto cpu_end_time = std::chrono::high_resolution_clock::now();
    auto cpu_elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(cpu_end_time - cpu_start_time);
    std::cout << "collision check cpu execution time: " << cpu_elapsed_time.count() << " milliseconds" << std::endl;

    std::chrono::time_point<std::chrono::high_resolution_clock> start_time, end_time;
    start_time = std::chrono::high_resolution_clock::now();
    broadPhaseFused_sep(confs, valid_conf);
    // broadPhaseFused(confs, valid_conf);
    // CallMegaKernel(confs, valid_conf);
    end_time = std::chrono::high_resolution_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "collision GPU execution time: " << elapsed_time.count() << " milliseconds" << std::endl;

    int num_correct = 0;
    int num_true = 0;
    int num_correct_true = 0;
    for (int i = 0; i < confs.size(); i++){
        if (valid_conf[i]==true){
            num_true++;
            if (valid_conf[i]==cpu_valid_conf[i]){
                num_correct_true++;
            } else {
                // std::cout << "conf " << i << " is a false positive" << std::endl;
            }
        } else {
            if (valid_conf[i]==cpu_valid_conf[i]){
            } else {
                std::cout << "conf " << i << " is a false negative" << std::endl;
            }
        }
        if (valid_conf[i]==cpu_valid_conf[i]){
            num_correct++;
            // std::cout << "conf " << i << " is " << valid_conf[i] << std::endl;
        } else {
            // std::cout << "conf " << i << " is " << valid_conf[i] << " but should be " << cpu_valid_conf[i] << std::endl;
        }
    }
    std::cout << "Num valid configurations " << num_true << std::endl;
    std::cout << "Num correct collision detections " << num_correct << std::endl;
    std::cout << "Num correct collision detections (true) " << num_correct_true << std::endl;
    if (num_correct != confs.size()){
            std::cout << "\033[31m" << "KERNEL BROKEN " << "\033[0m" << std::endl;
    }


    verifyConfs(valid_conf, confs.size());

    // delete[](gpu_transformed_vertices);
    delete[](bot_bounds_GPU);
    delete[](valid_conf);
}
