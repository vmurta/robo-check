#include <iostream>
#include <fcl/fcl.h>
#include <limits>
#include <vector>
#include "../../Utils.h"
#include "../transform.hu"

inline bool verticesEqual(const Vector3f &v1, const Vector3f &v2){
  return (fabs(transformed_vertices[i * fcl_rob_vertices.size() + j].x -transformed_vertex[0]) +
            fabs(transformed_vertices[i * fcl_rob_vertices.size() + j].y -transformed_vertex[1]) +
            fabs(transformed_vertices[i * fcl_rob_vertices.size() + j].z -transformed_vertex[2]) < 1e-5);
}

//takes in an allocated, empty array of vertices
// returns a filled one
void transformGPU(Vector3f* vertices){
  int device_count;
  if (cudaGetDeviceCount(&device_count) != 0) std::cout << "CUDA not loaded properly" << std::endl;
  // load configurations, should have 6990 valids and 3010 invalids
  std::vector<Configuration> confs;
  readConfigurationFromFile("10,000samples.conf", confs);
  // createAlphaBotConfigurations(confs, 10000);

  //Load Robot
  std::vector<Vector3f> rob_vertices;
  std::vector<Triangle> rob_triangles;
  loadOBJFile("../../models/alpha1.0/robot.obj", rob_vertices, rob_triangles);
  std::cout << "robot has " << rob_vertices.size() << " vertices " <<std::endl;

  // Load Obstacle
  std::vector<Vector3f> obs_vertices;
  std::vector<Triangle> obs_triangles;
  loadOBJFile("models/alpha1.0/obstacle.obj", obs_vertices, obs_triangles);
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
  checkCudaMem(cudaMemcpy(transformed_vertices, d_transformed_vertices, rob_vertices.size()* confs.size() * sizeof(Vector3f), cudaMemcpyDeviceToHost));
  checkCudaCall(cudaDeviceSynchronize()); 
  checkCudaCall(cudaFree(d_confs));
  checkCudaCall(cudaFree(d_rob_triangles));
  checkCudaCall(cudaFree(d_rob_vertices));
  checkCudaCall(cudaFree(d_transformed_vertices));
  checkCudaCall(cudaDeviceSynchronize());
  std::cout << " copied back memory and synchronized" << std::endl;

}

void transformCPU(Vector3f *vertices){
  //Load Robot
  std::vector<fcl::Vector3f> fcl_rob_vertices;
  std::vector<fcl::Triangle> fcl_rob_triangles;
  loadOBJFileFCL("../../models/alpha1.0/robot.obj", fcl_rob_vertices, fcl_rob_triangles);
  std::cout << "robot has " << fcl_rob_vertices.size() << " vertices " <<std::endl;

  std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<float>>> rob_mesh(new fcl::BVHModel<fcl::OBBRSS<float>>);
  rob_mesh->beginModel(fcl_rob_triangles.size(), fcl_rob_vertices.size());
  rob_mesh->addSubModel(fcl_rob_vertices, fcl_rob_triangles);
  rob_mesh->endModel();
  std::cout << "loaded robot" <<std::endl;

  for (int i = 0; i < confs.size(); i++){
    fcl::Transform3f transform = configurationToTransform(confs[i]);
    for (int j = 0; j < fcl_rob_vertices.size(); j++){
      vertices[i] = transform * fcl_rob_vertices[j];
    }  
  }
}

//TODO: move robot to constant memory
//TODO: refactor code to minimize loads by interleaving file reads and device memory operations
int main()
{
    
    Vector3f* gpu_transformed_vertices = new Vector3f[10000 * 792];
    Vector3f* cpu_transformed_vertices = new Vector3f[10000 * 792];

    transformCPU(cpu_transformed_vertices);
    transformGPU(gpu_transformed_vertices);
    
    int num_correct = 0;
    int num_incorrect = 0;
    float total_error_incorrect = 0;
    for (int i = 0; i < 10000; i++){
      for (int j = 0; j < 792; j++){
        if (verticesEqual(gpu_transformed_vertices[i * 792 + j], cpu_transformed_vertices[i * 792 + j])){
          num_correct++;
        } else {
          num_incorrect++;
          total_error_incorrect += fabs(transformed_vertices[i * fcl_rob_vertices.size() + j].x -transformed_vertex[0]) +
            fabs(transformed_vertices[i * fcl_rob_vertices.size() + j].y -transformed_vertex[1]) +
            fabs(transformed_vertices[i * fcl_rob_vertices.size() + j].z -transformed_vertex[2]);
        }
      }  
    }

    std::cout << "num correct is " << num_correct << std::endl;
    std::cout << "num incorrect is " << num_incorrect << std::endl;
    std::cout << "avg incorrect error is " << total_error_incorrect / num_incorrect << std::endl;
    
}