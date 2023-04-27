#include <iostream>
#include <../fcl/fcl.h>
#include <limits>
#include "../Utils.h"

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
    // load configurations, should have 6990 valids and 3010 invalids
    std::vector<Configuration> confs;
    readConfigurationFromFile("10,000samples.conf", confs);
    // createAlphaBotConfigurations(confs, 10000);

    //Load Robot
    std::vector<fcl::Vector3f> rob_vertices;
    std::vector<fcl::Triangle> rob_triangles;
    loadOBJFileFCL("models/alpha1.0/robot.obj", rob_vertices, rob_triangles);
    std::cout << "robot has " << rob_vertices.size() << " vertices " <<std::endl;

    std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<float>>> rob_mesh(new fcl::BVHModel<fcl::OBBRSS<float>>);
    rob_mesh->beginModel(rob_triangles.size(), rob_vertices.size());
    rob_mesh->addSubModel(rob_vertices, rob_triangles);
    rob_mesh->endModel();
    std::cout << "loaded robot" <<std::endl;

    // Load Obstacle
    std::vector<fcl::Vector3f> obs_vertices;
    std::vector<fcl::Triangle> obs_triangles;
    std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<float>>> obs_mesh(new fcl::BVHModel<fcl::OBBRSS<float>>);
    loadOBJFileFCL("models/alpha1.0/obstacle.obj", obs_vertices, obs_triangles);
    obs_mesh->beginModel(obs_triangles.size(), obs_vertices.size());
    obs_mesh->addSubModel(obs_vertices, obs_triangles);
    obs_mesh->endModel();

    fcl::CollisionObject<float> rob_col_obj(rob_mesh);
    fcl::CollisionObject<float> obs_col_obj(obs_mesh);


    // ************************************************************************//

    int num_valid = 0;
    int num_invalid = 0;
    // perform collision detection on each of the randomly generated configs
    for(auto conf: confs){
      fcl::Transform3f transform = configurationToTransform(conf);
      rob_col_obj.setTransform(transform);

      // Define CollisionRequest and CollisionResult objects
      fcl::CollisionRequest<float> request;
      fcl::CollisionResult<float> result;

      // // Perform collision detection
      fcl::collide(&obs_col_obj, &rob_col_obj, request, result);

      // Check if collision occurred
      if (result.isCollision()) {
        conf.valid = false;
        ++num_invalid;
      } else {
        conf.valid = true;
        ++num_valid;
      }
    }

    // s//save configurations in output file
    // writeConfigurationToFile(confs, "10,000samples.conf");
    std::cout << "We had " << num_valid << " valid confs and " << num_invalid << " invalid confs." << std::endl;

    return 0;
}
