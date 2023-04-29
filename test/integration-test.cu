#include <iostream>
#include <fcl/fcl.h>
#include <limits>
#include <vector>
#include "../Utils_rai.h"
#include "../transformation/transform.hu"
#include "../generate-AABB/generate-AABB.hu"
#include "../broad-phase/broad-phase.hu"
#include "../Utils.h"

#define CONF_FILE "./10,000samples.conf"
#define ROB_FILE "./models/alpha1.0/robot.obj"
#define OBS_FILE "./models/alpha1.0/obstacle.obj"
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
    for(int i = 0; i < numConfigs; ++i)
    {
        if(!(   fabs (botBoundsBaseline[i].x_min - botBoundsParallel[i].x_min) < 1e-4 &&
                fabs (botBoundsBaseline[i].y_min - botBoundsParallel[i].y_min) < 1e-4 &&
                fabs (botBoundsBaseline[i].z_min - botBoundsParallel[i].z_min) < 1e-4 &&
                fabs (botBoundsBaseline[i].x_max - botBoundsParallel[i].x_max) < 1e-4 &&
                fabs (botBoundsBaseline[i].y_max - botBoundsParallel[i].y_max) < 1e-4 &&
                fabs (botBoundsBaseline[i].z_max - botBoundsParallel[i].z_max) < 1e-4))
            return false;
    }
    return true;
}

void verifyConfs(bool *confs, size_t num_confs) {
    size_t numValidConfs = 0;

    for (size_t i = 0; i < num_confs; i++) {
        if (confs[i]) numValidConfs++;
    }

    std::cout << "Valid configurations: " << numValidConfs << " (out of " << num_confs << ")" << std::endl;
}

//takes in an allocated, empty array of vertices
// returns a filled one
void transformAndAABBOnGPU(AABB* bot_bounds, std::vector<Configuration> &confs){
    int device_count;
    if (cudaGetDeviceCount(&device_count) != 0) std::cout << "CUDA not loaded properly" << std::endl;

    //Load Robot
    std::vector<Vector3f> rob_vertices;
    std::vector<Triangle> rob_triangles;
    loadOBJFile(ROB_FILE, rob_vertices, rob_triangles);
    std::cout << "robot has " << rob_vertices.size() << " vertices " <<std::endl;


    std::vector<Vector3f> obs_vertices;
    std::vector<Triangle> obs_triangles;
    loadOBJFile(OBS_FILE, obs_vertices, obs_triangles);
    std::cout << "obstacle has " << obs_vertices.size() << " vertices " <<std::endl;

    Vector3f *d_rob_vertices;
    checkCudaCall(cudaMalloc(&d_rob_vertices, rob_vertices.size() * sizeof(Vector3f)));
    checkCudaMem(cudaMemcpy(d_rob_vertices, rob_vertices.data(), rob_vertices.size() * sizeof(Vector3f), cudaMemcpyHostToDevice));
    std::cout << "have copied the robot vertices " << std::endl;

    Vector3f *d_transformed_vertices;
    checkCudaCall(cudaMalloc(&d_transformed_vertices, rob_vertices.size() * sizeof(Vector3f) * confs.size()));
    std::cout << "have malloced the transformed vertices " << std::endl;

    Triangle *d_rob_triangles;
    checkCudaCall(cudaMalloc(&d_rob_triangles, rob_triangles.size() * sizeof(Triangle)));
    checkCudaMem(cudaMemcpy(d_rob_triangles, rob_triangles.data(), rob_triangles.size() * sizeof(Triangle), cudaMemcpyHostToDevice));
    std::cout << "have copied the robot triangles" << std::endl;

    Configuration *d_confs;
    checkCudaCall(cudaMalloc(&d_confs, confs.size() * sizeof(Configuration)));
    checkCudaMem(cudaMemcpy(d_confs, confs.data(), confs.size() * sizeof(Configuration), cudaMemcpyHostToDevice));
    std::cout << "have copied the configurations " << std::endl;

    AABB* d_bot_bounds;
    checkCudaCall(cudaMalloc(&d_bot_bounds, confs.size() * sizeof(AABB)));
    std::cout << "have malloced the AABBs " << std::endl;

    checkCudaCall(cudaDeviceSynchronize());
    std::cout << "have synchronized" << std::endl;

    dim3 dimGrid(ceil((float)(confs.size()) / AABB_BLOCK_SIZE), 1, 1);
    dim3 dimBlock(AABB_BLOCK_SIZE, 1, 1);

    // bitshifting right by 5 is the same as dividing by 2^5 (which is 32) and rounding up
    // also technically faster not that it matters very much
    genTransformedCopies<<<(confs.size() + 31)>> 5, 32>>>(d_confs, d_rob_vertices, d_transformed_vertices, 
                                                    confs.size(), rob_vertices.size());

    cudaError_t err = cudaGetLastError();
    printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));

    checkCudaCall(cudaDeviceSynchronize());

    generateAABBPrimitiveKernel<<<dimGrid, dimBlock>>>(d_transformed_vertices, rob_vertices.size(), 
                                                        confs.size(), d_bot_bounds);

    err = cudaGetLastError();
    printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));

    checkCudaCall(cudaDeviceSynchronize());

    // Move obstacle to AABB (on CPU since we only have 1)
    AABB *obstacle_bbox = new AABB();
    generateAABBBaseline(obs_vertices.data(), obs_vertices.size(), 1, obstacle_bbox);

    bool *valid_conf_d;
    bool *valid_conf = new bool[confs.size()];
    AABB *obstacle_bbox_d;
    checkCudaCall(cudaMalloc(&valid_conf_d, confs.size() * sizeof(bool)));
    checkCudaCall(cudaMalloc(&obstacle_bbox_d, sizeof(AABB)));
    checkCudaCall(cudaMemcpy(obstacle_bbox_d, obstacle_bbox, sizeof(AABB), cudaMemcpyHostToDevice));
    checkCudaCall(cudaDeviceSynchronize());

    // broadPhase(confs.size(), d_bot_bounds, obstacle_bbox_d, valid_conf_d);

    err = cudaGetLastError();
    printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));

    std::cout << "have called kernel " << std::endl;
    std:: cout << "about to synchronize" << std::endl;
    checkCudaCall(cudaDeviceSynchronize());

    std:: cout << "about to copy back vertices" << std::endl;
    cudaMemcpy(bot_bounds, d_bot_bounds, confs.size() * sizeof(AABB), cudaMemcpyDeviceToHost);
    cudaMemcpy(valid_conf, valid_conf_d, confs.size() * sizeof(bool), cudaMemcpyDeviceToHost);
    checkCudaCall(cudaDeviceSynchronize()); 

    verifyConfs(valid_conf, confs.size());
    checkCudaCall(cudaDeviceSynchronize());

    checkCudaCall(cudaFree(d_confs));
    checkCudaCall(cudaFree(d_rob_triangles));
    checkCudaCall(cudaFree(d_rob_vertices));
    checkCudaCall(cudaFree(d_transformed_vertices));
    checkCudaCall(cudaFree(d_bot_bounds));
    checkCudaCall(cudaFree(obstacle_bbox_d));
    checkCudaCall(cudaFree(valid_conf_d));
    checkCudaCall(cudaDeviceSynchronize());
    std::cout << "copied back memory and synchronized" << std::endl;
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

    fcl::Vector3f* vertices = new fcl::Vector3f[10000 * 792];

    for (int i = 0; i < confs.size(); i++){
    fcl::Transform3f transform = configurationToTransform(confs[i]);
    for (int j = 0; j < fcl_rob_vertices.size(); j++){
        vertices[i * fcl_rob_vertices.size() + j] = transform * fcl_rob_vertices[j];
    }  
    }

    generateAABBBaseline_fcl(vertices, fcl_rob_vertices.size(), confs.size(), bot_bounds);
}

//TODO: move robot to constant memory
//TODO: refactor code to minimize loads by interleaving file reads and device memory operations
int main()
{
      // load configurations, should have 6990 valids and 3010 invalids
    std::vector<Configuration> confs;
    readConfigurationFromFile(CONF_FILE, confs);

    Vector3f* gpu_transformed_vertices = new Vector3f[10000 * 792];
    fcl::Vector3f* cpu_transformed_vertices = new fcl::Vector3f[10000 * 792];
    AABB* bot_bounds_GPU = new AABB[confs.size()];
    AABB* bot_bounds_CPU = new AABB[confs.size()];

    std::chrono::time_point<std::chrono::high_resolution_clock> cpu_start_time, cpu_end_time;
    cpu_start_time = std::chrono::high_resolution_clock::now();
    transformCPU(bot_bounds_CPU, confs);
    cpu_end_time = std::chrono::high_resolution_clock::now();
    auto cpu_elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(cpu_end_time - cpu_start_time);
    std::cout << "Transformation cpu execution time: " << cpu_elapsed_time.count() << " milliseconds" << std::endl;

    std::chrono::time_point<std::chrono::high_resolution_clock> start_time, end_time;
    start_time = std::chrono::high_resolution_clock::now();

    transformAndAABBOnGPU(bot_bounds_GPU, confs);

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

    if(verify_generateAABB(bot_bounds_CPU, bot_bounds_GPU, confs.size()))
        std::cout << "[PASS] Parallel AABB generation matches serial generation." << std::endl;
    else
        std::cout << "[FAIL] Parallel AABB generation does not match serial generation." << std::endl;
    
}
