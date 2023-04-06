#include <iostream>
#include <limits>
#include <vector>
#include "Utils_rai.h"

void createAlphaBotConfigurations(std::vector<Configuration> &confs, int num_confs){
  // these are the max and min values for vertices in alpha1.0/robot.obj
    float x_min = 3.72119;
    float y_min = -11.0518;
    float z_min = -0.608012;
    float x_max = 65.9453;
    float y_max = 26.0984;
    float z_max = 18.6984;

    float x_range = x_max - x_min;
    float y_range = y_max - y_min;
    float z_range = z_max - z_min;

    generateConfs(confs, -x_range/20, x_range/20,
                         -y_range/20, y_range/20,
                         -z_range/20, z_range/20,
                         num_confs);

}

int main()
{
    int device_count;
    if (cudaGetDeviceCount(&device_count) != 0) std::cout << "CUDA not loaded properly" << std::endl;
    // load configurations, should have 6990 valids and 3010 invalids
    std::vector<Configuration> confs;
    readConfigurationFromFile("/src/10,000samples.conf", confs);
    createAlphaBotConfigurations(confs, 10000);

    //Load Robot
    std::vector<Vector3f> rob_vertices;
    std::vector<Triangle> rob_triangles;
    loadOBJFile("/src/models/alpha1.0/robot.obj", rob_vertices, rob_triangles);
    std::cout << "robot has " << rob_vertices.size() << " vertices " <<std::endl;

    /*std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<float>>> rob_mesh(new fcl::BVHModel<fcl::OBBRSS<float>>);
    rob_mesh->beginModel(rob_triangles.size(), rob_vertices.size());
    rob_mesh->addSubModel(rob_vertices, rob_triangles);
    rob_mesh->endModel();
    std::cout << "loaded robot" <<std::endl;*/

    // Load Obstacle
    std::vector<Vector3f> obs_vertices;
    std::vector<Triangle> obs_triangles;
    loadOBJFile("/src/models/alpha1.0/obstacle.obj", obs_vertices, obs_triangles);
    std::cout << "obstacle has " << obs_vertices.size() << " vertices " <<std::endl;
    return 0;
}
