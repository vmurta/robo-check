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
