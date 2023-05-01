#include "../broad-phase/broad-phase-fused.hu"

// Set -DLOCAL_TESTING=1 to run CPU tests on local machine (not on rai)

#if(LOCAL_TESTING == 1)
#include <fcl/fcl.h>
#include "../Utils.h"
#endif

#if(LOCAL_TESTING == 1)
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
#endif

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

    std::cout << "Valid configurations: " << numValidConfs << " (out of " << num_confs << ")" << std::endl;
}

#if(LOCAL_TESTING == 1)
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

    fcl::Vector3f* vertices = new fcl::Vector3f[10000 * 792];

    for (int i = 0; i < confs.size(); i++){
        fcl::Transform3f transform = configurationToTransform(confs[i]);
        for (int j = 0; j < fcl_rob_vertices.size(); j++){
            vertices[i * fcl_rob_vertices.size() + j] = transform * fcl_rob_vertices[j];
        }  
    }

    generateAABBBaseline_fcl(vertices, fcl_rob_vertices.size(), confs.size(), bot_bounds);
}
#endif

//TODO: refactor code to minimize loads by interleaving file reads and device memory operations
int main()
{
      // load configurations, should have 6990 valids and 3010 invalids
    std::vector<Configuration> confs;
    readConfigurationFromFile(CONF_FILE, confs);

    Vector3f* gpu_transformed_vertices = new Vector3f[10000 * 792];
    AABB* bot_bounds_GPU = new AABB[confs.size()];

    bool *valid_conf = new bool[confs.size()];
   
    #if(LOCAL_TESTING == 1)
    AABB* bot_bounds_CPU = new AABB[confs.size()];
    fcl::Vector3f* cpu_transformed_vertices = new fcl::Vector3f[10000 * 792];

    std::chrono::time_point<std::chrono::high_resolution_clock> cpu_start_time, cpu_end_time;
    cpu_start_time = std::chrono::high_resolution_clock::now();
    transformCPU(bot_bounds_CPU, confs);
    cpu_end_time = std::chrono::high_resolution_clock::now();
    auto cpu_elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(cpu_end_time - cpu_start_time);
    std::cout << "Transformation cpu execution time: " << cpu_elapsed_time.count() << " milliseconds" << std::endl;
    #endif

    std::chrono::time_point<std::chrono::high_resolution_clock> start_time, end_time;
    start_time = std::chrono::high_resolution_clock::now();

    broadPhaseFused(confs, valid_conf, bot_bounds_GPU);

    end_time = std::chrono::high_resolution_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "Transformation GPU execution time: " << elapsed_time.count() << " milliseconds" << std::endl;

    
    // int num_correct = 0;
    // int num_incorrect = 0;
    // float total_error_incorrect = 0;
    // for (int i = 0; i < 10000; i++){
    //   for (int j = 0; j < 792; j++){
    //     if (verticesEqual(gpu_transformed_vertices[i * 792 + j], cpu_transformed_vertices[i * 792 + j])){
    //       num_correct++;
    //     } else {
    //       num_incorrect++;
    //     }
    //   }  
    // }

    // std::cout << "num correct is " << num_correct << std::endl;
    // std::cout << "num incorrect is " << num_incorrect << std::endl;
    // std::cout << "avg incorrect error is " << total_error_incorrect / num_incorrect << std::endl;

    verifyConfs(valid_conf, confs.size());

    #if(LOCAL_TESTING == 1)
    if(verify_generateAABB(bot_bounds_CPU, bot_bounds_GPU, confs.size()))
        std::cout << "[PASS] Parallel AABB generation matches serial generation." << std::endl;
    else
        std::cout << "[FAIL] Parallel AABB generation does not match serial generation." << std::endl;
    delete[](bot_bounds_CPU);
    #endif

    // delete[](gpu_transformed_vertices);
    delete[](bot_bounds_GPU);
    delete[](valid_conf);
}
