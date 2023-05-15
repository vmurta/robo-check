#include <iostream>
#include <fcl/fcl.h>
#include <limits>
#include <vector>
#include "../../Utils.h"
#include "../transform.hu"

inline bool verticesEqual(const Vector3f &v1, const fcl::Vector3f &v2){
  return (fabs(v1.x -v2[0]) +
            fabs(v1.y -v2[1]) +
            fabs(v1.z -v2[2]) < 1e-5);
  // return false;
}

//takes in an allocated, empty array of vertices
// returns a filled one
void transformGPU(Vector3f* vertices, std::vector<Configuration> &confs){
  int device_count;
  if (cudaGetDeviceCount(&device_count) != 0) std::cout << "CUDA not loaded properly" << std::endl;

  //Load Robot
  std::vector<Vector3f> rob_vertices;
  std::vector<Triangle> rob_triangles;
  loadOBJFile("./models/alpha1.0/robot.obj", rob_vertices, rob_triangles);
  std::cout << "robot has " << rob_vertices.size() << " vertices " <<std::endl;

  Vector3f *d_rob_vertices;
  checkCudaCall(cudaMalloc(&d_rob_vertices, rob_vertices.size() * sizeof(Vector3f)));
  checkCudaMem(cudaMemcpy(d_rob_vertices, rob_vertices.data(), rob_vertices.size() * sizeof(Vector3f), cudaMemcpyHostToDevice));
  std::cout << "have copied the robot vertices " << std::endl;

  Vector3f *d_transformed_vertices;
  checkCudaCall(cudaMalloc(&d_transformed_vertices, rob_vertices.size() * sizeof(Vector3f) * confs.size()));
  std::cout << "have malloced the transformed vertices " << std::endl;

  Configuration *d_confs;
  checkCudaCall(cudaMalloc(&d_confs, confs.size() * sizeof(Configuration)));
  checkCudaMem(cudaMemcpy(d_confs, confs.data(), confs.size() * sizeof(Configuration), cudaMemcpyHostToDevice));
  std::cout << "have copied the configurations " << std::endl;


  checkCudaCall(cudaDeviceSynchronize());
  std::cout << "have synchronized" << std::endl;

  // bitshifting right by 5 is the same as dividing by 2^5 (which is 32) and rounding up
  // also technically faster not that it matters very much
  genTransformedCopies<<<(confs.size() + 31)>> 5, 32>>>(d_confs, d_rob_vertices, d_transformed_vertices,
                                                  confs.size(), rob_vertices.size());

  std::cout << "have called kernel " << std::endl;
  std:: cout << "about to synchronize" << std::endl;
  checkCudaCall(cudaDeviceSynchronize());
  std:: cout << "about to copy back vertices" << std::endl;
  checkCudaMem(cudaMemcpy(vertices, d_transformed_vertices, rob_vertices.size()* confs.size() * sizeof(Vector3f), cudaMemcpyDeviceToHost));
  checkCudaCall(cudaDeviceSynchronize());
  checkCudaCall(cudaFree(d_confs));
  checkCudaCall(cudaFree(d_rob_vertices));
  checkCudaCall(cudaFree(d_transformed_vertices));
  checkCudaCall(cudaDeviceSynchronize());
  std::cout << " copied back memory and synchronized" << std::endl;
}

void transformCPU(fcl::Vector3f *vertices, std::vector<Configuration> &confs){
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

  for (int i = 0; i < confs.size(); i++){
    fcl::Transform3f transform = configurationToTransform(confs[i]);
    for (int j = 0; j < fcl_rob_vertices.size(); j++){
      vertices[i * fcl_rob_vertices.size() + j] = transform * fcl_rob_vertices[j];
    }
  }
}

//TODO: move robot to constant memory
//TODO: refactor code to minimize loads by interleaving file reads and device memory operations
int main()
{
      // load configurations, should have 6990 valids and 3010 invalids
    std::vector<Configuration> confs;
    readConfigurationFromFile(CONF_FILE, confs);

    Vector3f* gpu_transformed_vertices = new Vector3f[NUM_CONFS * NUM_ROB_VERTICES];
    fcl::Vector3f* cpu_transformed_vertices = new fcl::Vector3f[NUM_CONFS * NUM_ROB_VERTICES];

    transformCPU(cpu_transformed_vertices, confs);
    auto start_time = std::chrono::high_resolution_clock::now();
    transformGPU(gpu_transformed_vertices, confs);
    auto end_time = std::chrono::high_resolution_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "Transformation GPU execution time: " << elapsed_time.count() << " milliseconds" << std::endl;

    int num_correct = 0;
    int num_incorrect = 0;
    float total_error_incorrect = 0;
    for (int i = 0; i < NUM_CONFS; i++){
      for (int j = 0; j < NUM_ROB_VERTICES; j++){
        if (verticesEqual(gpu_transformed_vertices[i * NUM_ROB_VERTICES + j], cpu_transformed_vertices[i * NUM_ROB_VERTICES + j])){
          num_correct++;
        } else {
          num_incorrect++;
        }
      }
    }

    std::cout << "num correct is " << num_correct << std::endl;
    std::cout << "num incorrect is " << num_incorrect << std::endl;
    // std::cout << "avg incorrect error is " << total_error_incorrect / num_incorrect << std::endl;

}
