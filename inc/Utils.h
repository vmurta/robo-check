#pragma once
#include <fstream>
#include <iostream>
#include <random>
#include <string.h>
#include <chrono>
#include <vector>
#include <limits>
#include <stack>
#include <float.h>
#include <sstream>
#include <fcl/common/types.h>
#include <fcl/geometry/shape/utility.h>
#include <filesystem>
#include <fcl/fcl.h>

#include <Eigen/Dense>

#ifndef CONF_FILE
    #define CONF_FILE "./easy_confs10,000.conf"
#endif

#define COALESCE 1


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

// From:  https://github.com/sunbuny/timeit/blob/master/timeit.h
// we must use two marco to combine the variable name with __LINE__, eg. if we just write 'name##__LINE__'
// the '##' will take place before __LINE__
// Because the preprocessor will only expand the macros recursively if neither the stringizing operator #
// nor the token-pasting operator ## are applied to it
// see more: https://stackoverflow.com/questions/1597007/creating-c-macro-with-and-line-token-concatenation-with-positioning-macr
#define COMBINE_HELPER(X,Y) X##Y  // helper macro
#define COMBINE(X,Y) COMBINE_HELPER(X,Y)

#define TIMEIT_PRINT_HELPER(msg, line_num, ...)                                                                        \
    auto COMBINE(timeit_start,line_num) =std::chrono::high_resolution_clock::now();                                    \
    __VA_ARGS__                                                                                                        \
    auto COMBINE(timeit_end, line_num) = std::chrono::high_resolution_clock::now();                                    \
    auto COMBINE(timeit_duration, line_num) =                                                                          \
         std::chrono::duration<double, std::milli>(COMBINE(timeit_end,line_num) - COMBINE(timeit_start,line_num));     \
    std::cout << msg << " execution time: " <<  COMBINE(timeit_duration, line_num).count() << " ms"<< std::endl;

#ifndef DISABLE_TIMEIT
// use __COUNTER__ instead of __LINE__ to make the TIMEIT marco able to be nested
#define TIMEIT(msg,  ...) \
     TIMEIT_PRINT_HELPER(msg, __COUNTER__,  __VA_ARGS__)
#else
#define TIMEIT(msg, ...) __VA_ARGS__
#endif

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

struct OBB_soa {
    Eigen::Matrix3f *pR; // rotation matrix
    Eigen::Vector3f *pT; // translation vector
    Eigen::Vector3f *pDim; // half dimensions of box A
    size_t size;

    OBB_soa(size_t size) : size(size) {
        pR = new Eigen::Matrix3f[size];
        pT = new Eigen::Vector3f[size];
        pDim = new Eigen::Vector3f[size];
    }

    virtual ~OBB_soa() {
        delete[] pR;
        delete[] pT;
        delete[] pDim;
    }

    void set(size_t index, const Eigen::Matrix3f& R, const Eigen::Vector3f& T, const Eigen::Vector3f& dim) {
        if (index >= size) {
            throw std::out_of_range("Index out of range");
        }
        pR[index] = R;
        pT[index] = T;
        pDim[index] = dim;
    }

    std::vector<Eigen::Vector3f> getBoxVertices(size_t box_index) const {
        const Eigen::Matrix3f& R = pR[box_index];
        const Eigen::Vector3f& T = pT[box_index];
        const Eigen::Vector3f& dim = pDim[box_index];

        std::vector<Eigen::Vector3f> vertices(8);
        vertices[0] = T + R * Eigen::Vector3f(-dim.x(), -dim.y(), -dim.z());
        vertices[1] = T + R * Eigen::Vector3f(dim.x(), -dim.y(), -dim.z());
        vertices[2] = T + R * Eigen::Vector3f(dim.x(), dim.y(), -dim.z());
        vertices[3] = T + R * Eigen::Vector3f(-dim.x(), dim.y(), -dim.z());
        vertices[4] = T + R * Eigen::Vector3f(-dim.x(), -dim.y(), dim.z());
        vertices[5] = T + R * Eigen::Vector3f(dim.x(), -dim.y(), dim.z());
        vertices[6] = T + R * Eigen::Vector3f(dim.x(), dim.y(), dim.z());
        vertices[7] = T + R * Eigen::Vector3f(-dim.x(), dim.y(), dim.z());

        return vertices;
    }


    void writeToFile(const std::string& filename) const {
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error opening file: " << filename << std::endl;
            return;
        }
        
        // Write this as a .obj file
        for (size_t i = 0; i < size; ++i) {
            file << "o Box" << i << "\n";
            std::vector<Eigen::Vector3f> vertices = this->getBoxVertices(i);

            // Write vertices to file
            for (const auto& vertex : vertices) {
                file << "v " << vertex.x() << " " << vertex.y() << " " << vertex.z() << "\n";
            }

            // Write faces to file
            file << "f " << (i * 8 + 1) << " " << (i * 8 + 2) << " " << (i * 8 + 3) << " " << (i * 8 + 4) << "\n";
            file << "f " << (i * 8 + 5) << " " << (i * 8 + 6) << " " << (i * 8 + 7) << " " << (i * 8 + 8) << "\n";
            file << "f " << (i * 8 + 1) << " " << (i * 8 + 2) << " " << (i * 8 + 6) << " " << (i * 8 + 5) << "\n";
            file << "f " << (i * 8 + 2) << " " << (i * 8 + 3) << " " << (i * 8 + 7) << " " << (i * 8 + 6) << "\n";
            file << "f " << (i * 8 + 3) << " " << (i * 8 + 4) << " " << (i * 8 + 8) << " " << (i * 8 + 7) << "\n";
            file << "f " << (i * 8 + 4) << " " << (i * 8 + 1) << " " << (i * 8 + 5) << " " << (i * 8 + 8) << "\n";
        }
    }
};

// Extends OBB to include BVH tree information
// each node either has 0 or 32 children //TODO: confirm this is still true

struct BVNode_soa : OBB_soa {
    // using the same pattern as fcl::BVNodeBase 
    /// If the value is positive, it is the index of the first child bv node
    /// If the value is negative, it is -(primitive index + 1)
    /// Zero implies this node has no BVNode_soachildren and no primitives, used only for padding
    int16_t *first_child;

    BVNode_soa(size_t size) : OBB_soa(size) {
        first_child = new int16_t[size];
    }

    ~BVNode_soa() {
        delete[] first_child;
    }

    void set(size_t index, const Eigen::Matrix3f& R, const Eigen::Vector3f& T, const Eigen::Vector3f& dim, int16_t child) {
        OBB_soa::set(index, R, T, dim);
        first_child[index] = child;
    }

};

//TODO: move these somewhere outside of Utils.h
#define NUM_ROB_VERTICES 792
#define MAX_NUM_ROBOT_TRIANGLES 1008
extern __constant__ Eigen::Vector3f base_robot_vertices[NUM_ROB_VERTICES];
extern __constant__ Triangle base_robot_triangles[MAX_NUM_ROBOT_TRIANGLES];
extern __constant__ Eigen::Vector3f base_obs_vertices[NUM_ROB_VERTICES];
extern __constant__ Triangle base_obs_triangles[MAX_NUM_ROBOT_TRIANGLES];
extern __constant__ float base_rob_x[NUM_ROB_VERTICES];
extern __constant__ float base_rob_y[NUM_ROB_VERTICES];
extern __constant__ float base_rob_z[NUM_ROB_VERTICES];
extern __constant__ int base_rob_tri_v1[MAX_NUM_ROBOT_TRIANGLES];
extern __constant__ int base_rob_tri_v2[MAX_NUM_ROBOT_TRIANGLES];
extern __constant__ int base_rob_tri_v3[MAX_NUM_ROBOT_TRIANGLES];
extern __constant__ float base_obs_x[NUM_ROB_VERTICES];
extern __constant__ float base_obs_y[NUM_ROB_VERTICES]; 
extern __constant__ float base_obs_z[NUM_ROB_VERTICES];
extern __constant__ int base_obs_tri_v1[MAX_NUM_ROBOT_TRIANGLES];
extern __constant__ int base_obs_tri_v2[MAX_NUM_ROBOT_TRIANGLES];
extern __constant__ int base_obs_tri_v3[MAX_NUM_ROBOT_TRIANGLES];

void writeConfigurationToFileTagged(const std::vector<ConfigurationTagged> &confs, const std::string& filename);

void readConfigurationFromFileTagged(const std::string& filename, std::vector<ConfigurationTagged> &confs);
void readConfigurationFromFile(const std::string& filename, std::vector<Configuration> &confs);
ConfigurationTagged makeTagged(const Configuration& conf);

void createAlphaBotConfigurations(const std::string &model_path, std::vector<Configuration> &confs,
                                   int num_confs_in_collision, int total_num_confs);
void loadOBJFile(std::string filename,  std::vector<Eigen::Vector3f>& points, std::vector<Triangle>& triangles);
void loadOBJFile(std::string filename,  std::vector<float>& x, std::vector<float> &y, std::vector<float> &z,
                                        std::vector<int>& v1, std::vector<int>& v2, std::vector<int>& v3);
void loadGFile(std::string filename, std::vector<Eigen::Vector3f>& points, std::vector<Triangle>& triangles);

void generateConfs(std::vector<Configuration> &confs, float x_min, float x_max,
                                                      float y_min, float y_max,
                                                      float z_min, float z_max,
                                                      int num_confs, int offset = 0);

void printConfiguration(const Configuration& conf);
void printConfigurationTagged(const ConfigurationTagged& conf);

// This function taken from https://github.com/flexible-collision-library/fcl/issues/131 Github user dblanm
void loadOBJFileFCL(std::string filename, std::vector<fcl::Vector3f>& points, std::vector<fcl::Triangle>& triangles);
fcl::Transform3f configurationToTransform(const Configuration& config);


__device__ __host__ Eigen::Matrix3f createRotationMatrix(const Configuration &config);
__device__ __host__ Eigen::Matrix4f createHomogeneousMatrix(const Configuration &config);

template<typename Derived>
std::string pythonifyEigenMatrix(const Eigen::MatrixBase<Derived>& m)
{
    std::ostringstream oss;
    for (int i = 0; i < m.rows(); ++i) {
        oss << "[";
        for (int j = 0; j < m.cols(); ++j) {
            oss << m(i, j);
            if (j < m.cols() - 1) {
                oss << ", ";
            }
        }
        oss << "]";
        if (i < m.rows() - 1) {
            oss << ",\n";
        }
    }
    return oss.str();
}

void checkConfsCPU( std::vector<ConfigurationTagged> &out, const std::vector<Configuration> &confs, 
                    std::string robot_filename, std::string obstacle_filename);

void flushCudaCache();


bool check_file_exists(const std::string& path);

class tranform_soa {
    //spatial coordinates
    float *x;
    float *y;
    float *z;

    // quaternion values
    float *qx;
    float *qy;
    float *qz;
    float *qw;
    
    size_t size;

    public:
        tranform_soa(size_t size) : size(size) {
            x = new float[size];
            y = new float[size];
            z = new float[size];
            qx = new float[size];
            qy = new float[size];
            qz = new float[size];
            qw = new float[size];
        }   

        ~tranform_soa() {
            delete[] x;
            delete[] y;
            delete[] z;
            delete[] qx;
            delete[] qy;
            delete[] qz;
            delete[] qw;
        }

        void set(size_t index, const Configuration& conf) {
            if (index >= size) {
                throw std::out_of_range("Index out of range");
            }
            x[index] = conf.x;
            y[index] = conf.y;
            z[index] = conf.z;


            //TODO: double check this
            // Convert Euler angles to quaternion
            float cy = cos(conf.yaw * 0.5f);
            float sy = sin(conf.yaw * 0.5f);
            float cp = cos(conf.pitch * 0.5f);
            float sp = sin(conf.pitch * 0.5f);
            float cr = cos(conf.roll * 0.5f);
            float sr = sin(conf.roll * 0.5f);

            qx[index] = sr * cp * cy - cr * sp * sy;
            qy[index] = cr * sp * cy + sr * cp * sy;
            qz[index] = cr * cp * sy - sr * sp * cy;
            qw[index] = cr * cp * cy + sr * sp * sy;
        }

};


template <typename BV>
std::vector<size_t> getBVHTreeDepths(const fcl::BVHModel<BV>& model);

// TODO: This assumes that B is a rotation matrix of B with respect to the axes of A
// we may want to calculate this dynamically, but for now, assume A axis aligned and centered at origin
OBB_soa hierarchy_from_mesh(const char* mesh_path);