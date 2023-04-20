#pragma once
#include <fstream>
#include <iostream>
#include <random>
#include <string.h>
// #include <transform.h>

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
  __device__ __host__ Vector3f() : x(0), y(0), z(0) {}
  __device__ __host__ Vector3f(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
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
        {
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


// TODO: Make this part run in parallel
// IDEA: could have each block do blockDim.x number of transformations
// Have each thread precompute the transformationMatrix at start of block
// Then, the block loops through the list of transformation matrix, computing each 
// in parallel
__device__ Matrix4f createTransformationMatrix(Configuration config) {
    float x = config.x;
    float y = config.y;
    float z = config.z;
    float pitch = config.pitch;
    float yaw = config.yaw;
    float roll = config.roll;

    float cosPitch = cos(pitch);
    float sinPitch = sin(pitch);
    float cosYaw = cos(yaw);
    float sinYaw = sin(yaw);
    float cosRoll = cos(roll);
    float sinRoll = sin(roll);

    Matrix4f transform;
    transform.m[0][0] = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
    transform.m[0][1] = -cosYaw * sinRoll + sinYaw * sinPitch * cosRoll;
    transform.m[0][2] = sinYaw * cosPitch;
    transform.m[0][3] = x;
    transform.m[1][0] = cosPitch * sinRoll;
    transform.m[1][1] = cosPitch * cosRoll;
    transform.m[1][2] = -sinPitch;
    transform.m[1][3] = y;
    transform.m[2][0] = -sinYaw * cosRoll + cosYaw * sinPitch * sinRoll;
    transform.m[2][1] = sinYaw * sinRoll + cosYaw * sinPitch * cosRoll;
    transform.m[2][2] = cosYaw * cosPitch;
    transform.m[2][3] = z;
    transform.m[3][0] = 0;
    transform.m[3][1] = 0;
    transform.m[3][2] = 0;
    transform.m[3][3] = 1;

    return transform;
}

__device__ Vector3f transformVector(Vector3f v, Matrix4f M) {
    // Create a 4D homogeneous vector from the 3D vector
    float v_h[4] = {v.x, v.y, v.z, 1};

    // Compute the transformed 4D vector by matrix multiplication
    float v_h_prime[4] = {0};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            v_h_prime[i] += M.m[i][j] * v_h[j];
        }
    }

    // Convert the transformed 4D vector back to a 3D vector
    Vector3f v_prime = {
        v_h_prime[0] / v_h_prime[3],
        v_h_prime[1] / v_h_prime[3],
        v_h_prime[2] / v_h_prime[3]
    };

    return v_prime;
}
  
__global__ void genTransformedCopies(Configuration* confs,  Vector3f *base_robot_vertices,
                                     Vector3f* transformed_robot_vertices, int num_confs, 
                                    int num_robot_vertices){
    size_t conf_ind = blockIdx.x * blockDim.x + threadIdx.x;
    #define TRANS_SIZE 32

    __shared__ Matrix4f transforms[TRANS_SIZE];
    if (conf_ind < num_confs){
      transforms[threadIdx.x] = createTransformationMatrix(confs[conf_ind]);
    }
    // do TRANS_SIZE number of configurations, unless that would put us out of bounds
    size_t num_confs_to_do = blockIdx.x * blockDim.x + TRANS_SIZE < num_confs ? TRANS_SIZE 
                              : num_confs -  blockIdx.x * blockDim.x;

    size_t transformed_vertices_offset = num_robot_vertices * TRANS_SIZE * blockIdx.x;

    //for each configuration, write a transformed copy of the robot into global memory
    for (int i = 0; i < num_confs_to_do; ++i){
      //each thread computes a portion of the current transformation and writes it to global
      // TODO: do this in shared memory, tile and flush
      for(int rob_ind = threadIdx.x; rob_ind < num_robot_vertices; rob_ind += blockDim.x){
        transformed_robot_vertices[rob_ind + transformed_vertices_offset] = 
          transformVector(base_robot_vertices[rob_ind], transforms[i]);
      }
      // increment to the next transformation
      transformed_vertices_offset += num_robot_vertices;
    }
}
