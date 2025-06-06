#include "Utils.h"

// This function taken from https://github.com/flexible-collision-library/fcl/issues/131 Github user dblanm
void loadOBJFileFCL(const char* filename, std::vector<fcl::Vector3f>& points, std::vector<fcl::Triangle>& triangles){
  FILE* file = fopen(filename, "rb");
  if(!file)
  {
    std::cerr << "file not exist" << std::endl;
    return;
  }

  bool has_normal = false;
  bool has_texture = false;
  char line_buffer[2000];
  while(fgets(line_buffer, 2000, file))
  {
    char* first_token = strtok(line_buffer, "\r\n\t ");
    if(!first_token || first_token[0] == '#' || first_token[0] == 0)
      continue;

    switch(first_token[0])
    {
    case 'v':
      {
        if(first_token[1] == 'n')
        {
          strtok(NULL, "\t ");
          strtok(NULL, "\t ");
          strtok(NULL, "\t ");
          has_normal = true;
        }
        else if(first_token[1] == 't')
        {
          strtok(NULL, "\t ");
          strtok(NULL, "\t ");
          has_texture = true;
        }
        else
        {
          fcl::FCL_REAL x = (fcl::FCL_REAL)atof(strtok(NULL, "\t "));
          fcl::FCL_REAL y = (fcl::FCL_REAL)atof(strtok(NULL, "\t "));
          fcl::FCL_REAL z = (fcl::FCL_REAL)atof(strtok(NULL, "\t "));
          fcl::Vector3f p(x, y, z);
          points.push_back(p);
        }
      }
      break;
    case 'f':
      {
        fcl::Triangle tri;
        char* data[30];
        int n = 0;
        while((data[n] = strtok(NULL, "\t \r\n")) != NULL)
        {
          if(strlen(data[n]))
            n++;
        }

        for(int t = 0; t < (n - 2); ++t)
        {
          if((!has_texture) && (!has_normal))
          {
            tri[0] = atoi(data[0]) - 1;
            tri[1] = atoi(data[1]) - 1;
            tri[2] = atoi(data[2]) - 1;
          }
          else
          {
            const char *v1;
            for(int i = 0; i < 3; i++)
            {
              // vertex ID
              if(i == 0)
                v1 = data[0];
              else
                v1 = data[t + i];

              tri[i] = atoi(v1) - 1;
            }
          }
          triangles.push_back(tri);
        }
      }
    }
  }
}

fcl::Transform3f configurationToTransform(const Configuration& config) {
    fcl::Transform3f out;
    out.setIdentity();
    fcl::Vector3f translation(config.x, config.y, config.z);
    
    fcl::Quaternionf rotation = Eigen::AngleAxisf(config.yaw, Eigen::Vector3f::UnitZ())
                              * Eigen::AngleAxisf(config.pitch, Eigen::Vector3f::UnitY())
                              * Eigen::AngleAxisf(config.roll, Eigen::Vector3f::UnitX());
    
    out.translation() = translation;
    out.rotate(rotation);
    return out;
}

void checkConfsCPU(std::vector<ConfigurationTagged> &out, const std::vector<Configuration> &confs){

    //Load Robot
    std::vector<fcl::Vector3f> rob_vertices;
    std::vector<fcl::Triangle> rob_triangles;
    std::vector<fcl::Vector3f> obs_vertices;
    std::vector<fcl::Triangle> obs_triangles;

    loadOBJFileFCL("models/alpha1.0/robot.obj", rob_vertices, rob_triangles);
    loadOBJFileFCL("models/alpha1.0/obstacle.obj", obs_vertices, obs_triangles);

    std::cout << "robot has " << rob_vertices.size() << " vertices " <<std::endl;
    auto cpu_start_time = std::chrono::high_resolution_clock::now();

    std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<float>>> rob_mesh(new fcl::BVHModel<fcl::OBBRSS<float>>);
    rob_mesh->beginModel(rob_triangles.size(), rob_vertices.size());
    rob_mesh->addSubModel(rob_vertices, rob_triangles);
    rob_mesh->endModel();
    std::cout << "loaded robot" <<std::endl;

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
      // std::cout << "starting conf " << i << std::endl;
      fcl::Transform3f transform = configurationToTransform(confs[i]);
      rob_col_obj.setTransform(transform);

      // Define CollisionRequest and CollisionResult objects
      fcl::CollisionRequest<float> request;
      fcl::CollisionResult<float> result;

      // // Perform collision detection
      fcl::collide(&obs_col_obj, &rob_col_obj, request, result);

      out[i] = makeTagged(confs[i]);
      // Check if collision occurred
      if (result.isCollision()) {
        out[i].valid= false;
      } else {
        out[i].valid=true;
      }
    }
    auto cpu_end_time = std::chrono::high_resolution_clock::now();
    auto cpu_elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(cpu_end_time - cpu_start_time);
    std::cout << "cpu collision detection execution time: " << cpu_elapsed_time.count() << " milliseconds for " <<confs.size() << " configurations." << std::endl;

}
