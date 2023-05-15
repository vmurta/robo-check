#pragma once

#include <fstream>
#include <iostream>
#include <random>
#include <string.h>
#include <chrono>
#include <vector>
#include <limits>
#include <float.h>
#include <sstream>


#if LOCAL_TESTING == 1
    #ifndef CONF_FILE
        #define CONF_FILE "./easy_confs10,000.conf"
    #endif
    #define ROB_FILE "./models/alpha1.0/robot.obj"
    #define OBS_FILE "./models/alpha1.0/obstacle.obj"
#else
    #define CONF_FILE "src/10,000samples.conf"
    #define ROB_FILE "src/models/alpha1.0/robot.obj"
    #define OBS_FILE "src/models/alpha1.0/obstacle.obj"
#endif

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
};

struct ConfigurationTagged {
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

struct Matrix3f {
    float m[3][3];
};

struct Vector3f {
  float x, y, z;
  __device__ __host__ Vector3f(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}\
  __device__ __host__ Vector3f(){};
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

#define NUM_ROB_VERTICES 792
#define MAX_NUM_ROBOT_TRIANGLES 1008
extern __constant__ Vector3f base_robot_vertices[NUM_ROB_VERTICES];
extern __constant__ Triangle base_robot_triangles[MAX_NUM_ROBOT_TRIANGLES];
extern __constant__ Vector3f base_obs_vertices[NUM_ROB_VERTICES];
extern __constant__ Triangle base_obs_triangles[MAX_NUM_ROBOT_TRIANGLES];

// __constant__ float base_rob_x[NUM_ROB_VERTICES];
// __constant__ float base_rob_y[NUM_ROB_VERTICES];
// __constant__ float base_rob_z[NUM_ROB_VERTICES];
// __constant__ Triangle base_rob_tri_v1[MAX_NUM_ROBOT_TRIANGLES];
// __constant__ Triangle base_rob_tri_v2[MAX_NUM_ROBOT_TRIANGLES];
// __constant__ Triangle base_rob_tri_v3[MAX_NUM_ROBOT_TRIANGLES];
// __constant__ Vector3f base_obs_x[NUM_ROB_VERTICES];
// __constant__ Vector3f base_obs_y[NUM_ROB_VERTICES]; 
// __constant__ Vector3f base_obs_z[NUM_ROB_VERTICES];
// __constant__ Triangle base_obs_tri_v1[MAX_NUM_ROBOT_TRIANGLES];
// __constant__ Triangle base_obs_tri_v2[MAX_NUM_ROBOT_TRIANGLES];
// __constant__ Triangle base_obs_tri_v3[MAX_NUM_ROBOT_TRIANGLES];

void writeConfigurationToFileTagged(const std::vector<ConfigurationTagged> &confs, const std::string& filename);

void readConfigurationFromFileTagged(const std::string& filename, std::vector<ConfigurationTagged> &confs);
void readConfigurationFromFile(const std::string& filename, std::vector<Configuration> &confs);
ConfigurationTagged makeTagged(const Configuration& conf);

void createAlphaBotConfigurations(std::vector<Configuration> &confs, int num_confs, bool hard);
void loadOBJFile(const char* filename, std::vector<Vector3f>& points, std::vector<Triangle>& triangles);
void generateConfs(std::vector<Configuration> &confs, float x_min, float x_max,
                                                      float y_min, float y_max,
                                                      float z_min, float z_max,
                                                      int num_confs);

void printConfiguration(const Configuration& conf);
void printConfigurationTagged(const ConfigurationTagged& conf);