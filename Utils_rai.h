#pragma once

#include <fstream>
#include <iostream>
#include <random>
#include <string.h>

#define checkCudaCall(status) \
    do { \
        cudaError_t err = status; \
        if(err != cudaSuccess) { \
            fprintf(stderr, "CUDA Error in %s:%d at line %d: %s\n", \
                __FILE__, __LINE__, err, cudaGetErrorString(err)); \
        } \
    } while(0)

#define checkCudaMem(error) \
    do { \
        cudaError_t err = error; \
        if (err != cudaSuccess) { \
            fprintf(stderr, "CUDA error at %s:%d: %s\n", \
                __FILE__, __LINE__, cudaGetErrorString(err)); \
            exit(1); \
        } \
    } while (0)

struct Configuration {
    float x;
    float y;
    float z;
    float pitch;
    float yaw;
    float roll;
    bool valid;
};

struct Matrix4f {
    float m[4][4];
};

struct Vector3f {
  float x, y, z;
  __device__ __host__ Vector3f(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}\
  __device__ __host__ Vector3f() : x(0), y(0), z(0) {};
};

struct Triangle {
  int v1, v2, v3;
};

struct AABB
{
    float x_min;
    float y_min;
    float z_min;
    float x_max;
    float y_max;
    float z_max;
};

void writeConfigurationToFile(const std::vector<Configuration> &confs, const std::string& filename) {
    std::ofstream file(filename);
    if (file.is_open()) {
      for(auto config: confs ){
        file  << config.x << " "
              << config.y << " "
              << config.z << " "
              << config.pitch << " "
              << config.yaw << " "
              << config.roll << " "
              << config.valid << std::endl;
      }
      file.close();
    } else {
        throw std::runtime_error("Failed to open file " + filename);
    }
}

void readConfigurationFromFile(const std::string& filename, std::vector<Configuration> &confs) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file " + filename);
    }
    Configuration config;
    while (file >> config.x >> config.y >> config.z >> config.pitch >> config.yaw >> config.roll >> config.valid) {
        confs.push_back(config);
    }
    file.close();
}

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

//TODO: modify this to directly write to device memory
void loadOBJFile(const char* filename, std::vector<Vector3f>& points, std::vector<Triangle>& triangles){
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
          float x = (float)atof(strtok(NULL, "\t "));
          float y = (float)atof(strtok(NULL, "\t "));
          float z = (float)atof(strtok(NULL, "\t "));
          Vector3f p(x, y, z);
          points.push_back(p);
        }
      }
      break;
    case 'f':
      {
        Triangle tri;
        char* data[30];
        int n = 0;
        while((data[n] = strtok(NULL, "\t \r\n")) != NULL)
        {
          if(strlen(data[n]))
            n++;
        }

        for(int t = 0; t < (n - 2); ++t)
        {struct Matrix4f {
    float m[4][4];
};
          if((!has_texture) && (!has_normal))
          {
            tri.v1 = atoi(data[0]) - 1;
            tri.v2 = atoi(data[1]) - 1;
            tri.v3 = atoi(data[2]) - 1;
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

              if (i == 0)
                tri.v1 = atoi(v1) - 1;
              else if (i == 1)
                tri.v2 = atoi(v1) - 1;
              else
                tri.v3 = atoi(v1) - 1;
            }
          }
          triangles.push_back(tri);
        }
      }
    }
  }
}

void generateConfs(std::vector<Configuration> &confs, float x_min, float x_max,
                                                      float y_min, float y_max,
                                                      float z_min, float z_max,
                                                      int num_confs){
    // Define a uniform real distribution for x, y, z values
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis_x(x_min, x_max);
    std::uniform_real_distribution<float> dis_y(y_min, y_max);
    std::uniform_real_distribution<float> dis_z(z_min, z_max);

    // Define a uniform real distribution for yaw, pitch, roll values
    std::uniform_real_distribution<float> dis_rot(-M_PI, M_PI);

    // Generate num_confs Configurations
    for (int i = 0; i < num_confs; i++) {
        Configuration conf;
        conf.x = dis_x(gen);
        conf.y = dis_y(gen);
        conf.z = dis_z(gen);
        conf.pitch = dis_rot(gen);
        conf.yaw = dis_rot(gen);
        conf.roll = dis_rot(gen);
        conf.valid = false;
        confs.push_back(conf);
    }

}

/*fcl::Transform3f configurationToTransform(const Configuration& config) {
    fcl::Transform3f out;
    out.setIdentity();
    fcl::Vector3f translation(config.x, config.y, config.z);

    fcl::Quaternionf rotation = Eigen::AngleAxisf(config.roll, Eigen::Vector3f::UnitX())
                              * Eigen::AngleAxisf(config.pitch, Eigen::Vector3f::UnitY())
                              * Eigen::AngleAxisf(config.yaw, Eigen::Vector3f::UnitZ());

    out.translation() = translation;
    out.rotate(rotation);
    return out;
}*/

void printConfiguration(const Configuration& conf) {
    std::cout << "x: " << conf.x << std::endl;
    std::cout << "y: " << conf.y << std::endl;
    std::cout << "z: " << conf.z << std::endl;
    std::cout << "pitch: " << conf.pitch << std::endl;
    std::cout << "yaw: " << conf.yaw << std::endl;
    std::cout << "roll: " << conf.roll << std::endl;
    std::cout << "valid: " << conf.valid << std::endl;
}


