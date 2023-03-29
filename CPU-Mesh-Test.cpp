#include <iostream>
#include <fcl/fcl.h>
// #include <fcl/BVH_model.h>
// This function taken from https://github.com/flexible-collision-library/fcl/issues/131 Github user dblanm
void loadOBJFile(const char* filename, std::vector<fcl::Vector3f>& points, std::vector<fcl::Triangle>& triangles){
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

int main()
{
    // This AI generated part does not work, as we do not yet have a way to load
    // in .g format, which is BYU format meshes. 
    // We could either try to manually convert our meshes into obj format, or find 
    // some tool to automatically convert them in here.
    // ************************************************************************//
    // Load mesh 1 into BVHModel
    // fcl::BVHModel<fcl::OBBRSS<float>> robot;
    std::vector<fcl::Vector3f> rob_vertices;
    std::vector<fcl::Triangle> rob_triangles;
    loadOBJFile("models/alpha1.0/robot.obj", rob_vertices, rob_triangles);
    std::cout << "robot has " << rob_vertices.size() << " vertices " <<std::endl;

    // fcl::BVHModel *ptr = new fcl::BVHModel();
    std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<float>>> rob_mesh(new fcl::BVHModel<fcl::OBBRSS<float>>);
    rob_mesh->beginModel(rob_triangles.size(), rob_vertices.size());
    rob_mesh->addSubModel(rob_vertices, rob_triangles);
    rob_mesh->endModel();
    std::cout << "loaded robot" <<std::endl;

    std::vector<fcl::Vector3f> obs_vertices;
    std::vector<fcl::Triangle> obs_triangles;
    std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<float>>> obs_mesh(new fcl::BVHModel<fcl::OBBRSS<float>>);
    loadOBJFile("models/alpha1.0/obstacle.obj", obs_vertices, obs_triangles);
    obs_mesh->beginModel(obs_triangles.size(), obs_vertices.size());
    obs_mesh->addSubModel(obs_vertices, obs_triangles);
    obs_mesh->endModel();

    fcl::CollisionObject<float> rob_col_obj(rob_mesh);
    fcl::CollisionObject<float> obs_col_obj(obs_mesh);

    
    // ************************************************************************//

    // Create CollisionObject for obstacle
    // fcl::CollisionObject<fcl::OBBRSS<float>> co2(&obstacle);

    // Set transformation matrices if necessary
    fcl::Transform3<float> tf1 = fcl::Transform3<float>::Identity();
    fcl::Transform3<float> tf2 = fcl::Transform3<float>::Identity();
    obs_col_obj.setTransform(tf1);
    rob_col_obj.setTransform(tf2);

    // Define CollisionRequest and CollisionResult objects
    fcl::CollisionRequest<float> request;
    fcl::CollisionResult<float> result;

    // // Perform collision detection
    fcl::collide(&obs_col_obj, &rob_col_obj, request, result);

    // Check if collision occurred
    if (result.isCollision()) {
        std::cout << "Collision detected! That's bad :(" << std::endl;
    } else {
        std::cout << "No collision detected. That's good!" << std::endl;
    }

    tf2.translation() = fcl::Vector3<float>(0, -5,0);
    rob_col_obj.setTransform(tf2);

    fcl::collide(&obs_col_obj, &rob_col_obj, request, result);

    // Check if collision occurred
    if (result.isCollision()) {
        std::cout << "Collision detected! That's good!" << std::endl;
    } else {
        std::cout << "No collision detected. That's bad :(" << std::endl;
    }

    return 0;
}
