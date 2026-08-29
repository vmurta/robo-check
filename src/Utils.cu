#include "Utils.h"

#include <fstream>
#include <iostream>
#include <random>
#include <string.h>
#include <fstream>
#include <vector>
#include <string>
#include <stdexcept>
#include <algorithm>
#include <cassert>

void writeConfigurationToFileTagged(const std::vector<ConfigurationTagged> &confs, const std::string& filename) {
    std::ofstream file(filename);
    if (file.is_open()) {
        int validCount = 0;
        int invalidCount = 0;

        for (const auto& config : confs) {
            if (config.valid) {
                validCount++;
            } else {
                invalidCount++;
            }
        }

        file << "There are " << validCount << " valid configurations and " << invalidCount << " invalid configurations\n";

        for (const auto& config : confs) {
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

void readConfigurationFromFileTagged(const std::string& filename, std::vector<ConfigurationTagged> &confs) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file " + filename);
    }

    int validCount = 0;
    int invalidCount = 0;
    std::string firstLine;
    std::getline(file, firstLine);

    std::istringstream lineStream(firstLine);
    std::string word;

    // Parse the first line to extract the valid and invalid configuration counts
    while (lineStream >> word) {
        if (word == "valid") {
            lineStream >> validCount;
        } else if (word == "invalid") {
            lineStream >> invalidCount;
        }
    }

    ConfigurationTagged config;
    while (file >> config.x >> config.y >> config.z >> config.pitch >> config.yaw >> config.roll >> config.valid) {
        confs.push_back(config);
    }
    file.close();
}

//automatically detags
void readConfigurationFromFile(const std::string& filename, std::vector<Configuration> &confs) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file " + filename);
    }

    std::string firstLine;
    std::getline(file, firstLine);
    std::istringstream lineStream(firstLine);
    std::string word;

    Configuration config;
    while (file >> config.x >> config.y >> config.z >> config.pitch >> config.yaw >> config.roll) {
      std::string valid_str; // dump value
      file >> valid_str; // skip past this validity of the configuration
      confs.push_back(config);
    }

    file.close();
}

ConfigurationTagged makeTagged(const Configuration& config) {
    ConfigurationTagged tagged;
    tagged.x = config.x;
    tagged.y = config.y;
    tagged.z = config.z;
    tagged.pitch = config.pitch;
    tagged.yaw = config.yaw;
    tagged.roll = config.roll;
    tagged.valid = false;
    return tagged;
}

void loadGFile(std::string filename, std::vector<Eigen::Vector3f>& points, std::vector<Triangle>& triangles){
  FILE* file = fopen(filename.c_str(), "rb");
  if(!file)
  {
    std::cerr << "file not exist: " << filename << std::endl;
    return;
  }

  int numParts, numVerts, numPolys, numEdges;
  if(fscanf(file, "%d %d %d %d", &numParts, &numVerts, &numPolys, &numEdges) != 4)
  {
    std::cerr << "error reading BYU header" << std::endl;
    fclose(file);
    return;
  }

  for(int i = 0; i < numParts; ++i)
  {
    int start, end;
    fscanf(file, "%d %d", &start, &end);
  }

  for(int i = 0; i < numVerts; ++i)
  {
    float x, y, z;
    fscanf(file, "%f %f %f", &x, &y, &z);
    points.push_back(Eigen::Vector3f(x, y, z));
  }

  // consume the rest of the last vertex line
  char line_buffer[2000];
  fgets(line_buffer, 2000, file);

  for(int poly = 0; poly < numPolys; ++poly)
  {
    fgets(line_buffer, 2000, file);
    char* ptr = line_buffer;
    std::vector<int> indices;
    int idx;
    while(sscanf(ptr, "%d", &idx) == 1)
    {
      while(*ptr && *ptr != ' ' && *ptr != '\t' && *ptr != '\n') ++ptr;
      while(*ptr == ' ' || *ptr == '\t') ++ptr;
      if(idx < 0)
      {
        indices.push_back(-idx - 1);
        break;
      }
      indices.push_back(idx - 1);
    }
    for(size_t t = 1; t + 1 < indices.size(); ++t)
    {
      Triangle tri;
      tri.v1 = indices[0];
      tri.v2 = indices[t];
      tri.v3 = indices[t + 1];
      triangles.push_back(tri);
    }
  }

  fclose(file);
}

static bool checkSingleConfCPU(const Configuration &conf,
                               fcl::CollisionObject<float> &rob_col_obj,
                               fcl::CollisionObject<float> &obs_col_obj){
    fcl::Transform3f transform = configurationToTransform(conf);
    rob_col_obj.setTransform(transform);
    fcl::CollisionRequest<float> request(1);
    fcl::CollisionResult<float> result;
    fcl::collide(&obs_col_obj, &rob_col_obj, request, result);
    return result.isCollision();
}

//fills confs with num_confs_in_collision configurations that are in collision and
// total_num_confs - num_confs_in_collision configurations that are not in collision wrt the alpha obstacle
void createAlphaBotConfigurations(const std::string &model_path, std::vector<Configuration> &confs,
                                   int num_confs_in_collision, int total_num_confs){
    std::vector<Eigen::Vector3f> points;
    std::vector<Triangle> triangles;

    std::string ext = model_path.substr(model_path.find_last_of('.') + 1);
    if(ext == "g")
      loadGFile(model_path, points, triangles);
    else
      loadOBJFile(model_path, points, triangles);

    if(points.empty()){
      std::cerr << "no vertices loaded from " << model_path << std::endl;
      return;
    }

    float x_min = points[0].x(), x_max = points[0].x();
    float y_min = points[0].y(), y_max = points[0].y();
    float z_min = points[0].z(), z_max = points[0].z();
    for(size_t i = 1; i < points.size(); ++i){
      x_min = std::min(x_min, points[i].x());
      x_max = std::max(x_max, points[i].x());
      y_min = std::min(y_min, points[i].y());
      y_max = std::max(y_max, points[i].y());
      z_min = std::min(z_min, points[i].z());
      z_max = std::max(z_max, points[i].z());
    }

    float x_range = x_max - x_min;
    float y_range = y_max - y_min;
    float z_range = z_max - z_min;

    // Load FCL models once for collision checking
    std::vector<fcl::Vector3f> rob_vertices, obs_vertices;
    std::vector<fcl::Triangle> rob_triangles_fcl, obs_triangles_fcl;
    loadOBJFileFCL("/home/victor/Projects/robo-check/data/models/alpha1.0/robot.obj", rob_vertices, rob_triangles_fcl);
    loadOBJFileFCL("/home/victor/Projects/robo-check/data/models/alpha1.0/obstacle.obj", obs_vertices, obs_triangles_fcl);

    std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<float>>> rob_mesh(new fcl::BVHModel<fcl::OBBRSS<float>>);
    rob_mesh->beginModel(rob_triangles_fcl.size(), rob_vertices.size());
    rob_mesh->addSubModel(rob_vertices, rob_triangles_fcl);
    rob_mesh->endModel();

    std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<float>>> obs_mesh(new fcl::BVHModel<fcl::OBBRSS<float>>);
    obs_mesh->beginModel(obs_triangles_fcl.size(), obs_vertices.size());
    obs_mesh->addSubModel(obs_vertices, obs_triangles_fcl);
    obs_mesh->endModel();

    fcl::CollisionObject<float> rob_col_obj(rob_mesh);
    fcl::CollisionObject<float> obs_col_obj(obs_mesh);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis_rot(-M_PI, M_PI);

    int num_written = 0;

    // Generate configs that are in collision
    {
      std::uniform_real_distribution<float> dis_x(-x_range/2, x_range/2);
      std::uniform_real_distribution<float> dis_y(-y_range/2, y_range/2);
      std::uniform_real_distribution<float> dis_z(-z_range/2, z_range/2);
      int count = 0;
      while(count < num_confs_in_collision){
        Configuration conf;
        conf.x = dis_x(gen);
        conf.y = dis_y(gen);
        conf.z = dis_z(gen);
        conf.pitch = dis_rot(gen);
        conf.yaw = dis_rot(gen);
        conf.roll = dis_rot(gen);
        if(checkSingleConfCPU(conf, rob_col_obj, obs_col_obj)){
          confs[num_written++] = conf;
          count++;
        }
      }
    }

    // Generate configs that are not in collision
    {
      std::uniform_real_distribution<float> dis_x(-x_range * 10, x_range * 10);
      std::uniform_real_distribution<float> dis_y(-y_range * 10, y_range * 10);
      std::uniform_real_distribution<float> dis_z(-z_range * 10, z_range * 10);
      int num_not_in_collision = total_num_confs - num_confs_in_collision;
      int count = 0;
      while(count < num_not_in_collision){
        Configuration conf;
        conf.x = dis_x(gen);
        conf.y = dis_y(gen);
        conf.z = dis_z(gen);
        conf.pitch = dis_rot(gen);
        conf.yaw = dis_rot(gen);
        conf.roll = dis_rot(gen);
        if(!checkSingleConfCPU(conf, rob_col_obj, obs_col_obj)){
          confs[num_written++] = conf;
          count++;
        }
      }
    }

    assert(num_written == total_num_confs);
}

void loadOBJFile(std::string filename, std::vector<Eigen::Vector3f>& points, std::vector<Triangle>& triangles){
  FILE* file = fopen(filename.c_str(), "rb");
  if(!file)
  {
    std::cerr << "file not exist:" << filename << std::endl;
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
          Eigen::Vector3f p(x, y, z);
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

void loadOBJFile(std::string filename,  std::vector<float>& x, std::vector<float> &y, std::vector<float> &z,
                                        std::vector<int>& v1, std::vector<int>& v2, std::vector<int>& v3){

  FILE* file = fopen(filename.c_str(), "rb");
  if(!file)
  {
    std::cerr << "file not exist:" << filename << std::endl;
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
          float x_val = (float)atof(strtok(NULL, "\t "));
          float y_val = (float)atof(strtok(NULL, "\t "));
          float z_val = (float)atof(strtok(NULL, "\t "));
          x.push_back(x_val);
          y.push_back(y_val);
          z.push_back(z_val);
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
          v1.push_back(tri.v1);
          v2.push_back(tri.v2);
          v3.push_back(tri.v3);
        }
      }
    }
  }
}

void generateConfs(std::vector<Configuration> &confs, float x_min, float x_max,
                                                      float y_min, float y_max,
                                                      float z_min, float z_max,
                                                      int num_confs, int offset){
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
        confs[offset + i] = conf;
    }

}


__device__ __host__ Eigen::Matrix3f createRotationMatrix(const Configuration& config) {


    float cosB = cos(config.pitch);
    float sinB = sin(config.pitch);
    float cosA = cos(config.yaw);
    float sinA = sin(config.yaw);
    float cosC = cos(config.roll);
    float sinC = sin(config.roll);

    Eigen::Matrix3f rotate;
    rotate(0,0) = cosA * cosB;
    rotate(0,1) = cosA * sinB * sinC - sinA * cosC;
    rotate(0,2) = cosA *  sinB * cosC + sinA * sinC;
    rotate(1,0) = sinA * cosB;
    rotate(1,1) = sinA * sinB * sinC + cosA * cosC;
    rotate(1,2) = sinA * sinB * cosC - cosA * sinC;
    rotate(2,0) = -sinB;
    rotate(2,1) = cosB * sinC;
    rotate(2,2) = cosB * cosC;

    return rotate;
}

__device__ __host__ Eigen::Matrix4f createHomogeneousMatrix(const Configuration& config) {
    Eigen::Matrix4f homogeneous = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f rotation = createRotationMatrix(config);
    homogeneous.block<3,3>(0,0) = rotation;
    homogeneous(0,3) = config.x;
    homogeneous(1,3) = config.y;
    homogeneous(2,3) = config.z;
    return homogeneous;
}

void printConfiguration(const Configuration& conf) {
    std::cout << "x: " << conf.x << std::endl;
    std::cout << "y: " << conf.y << std::endl;
    std::cout << "z: " << conf.z << std::endl;
    std::cout << "pitch: " << conf.pitch << std::endl;
    std::cout << "yaw: " << conf.yaw << std::endl;
    std::cout << "roll: " << conf.roll << std::endl;
}

void printConfigurationTagged(const ConfigurationTagged& conf) {
    std::cout << "x: " << conf.x << std::endl;
    std::cout << "y: " << conf.y << std::endl;
    std::cout << "z: " << conf.z << std::endl;
    std::cout << "pitch: " << conf.pitch << std::endl;
    std::cout << "yaw: " << conf.yaw << std::endl;
    std::cout << "roll: " << conf.roll << std::endl;
    std::cout << "valid: " << conf.valid << std::endl;
}

// This function taken from https://github.com/flexible-collision-library/fcl/issues/131 Github user dblanm
void loadOBJFileFCL(std::string filename, std::vector<fcl::Vector3f>& points, std::vector<fcl::Triangle>& triangles){
  FILE* file = fopen(filename.c_str(), "rb");
  if(!file)
  {
    std::cerr << "file not exist:" << filename << std::endl;
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

    fcl::Matrix3f rotation = createRotationMatrix(config);
    out.translation() = translation;
    out.rotate(rotation);
    return out;
}

// taken from https://guillesanbri.com/CUDA-Benchmarks/
void flushCudaCache(){
  // Get size of L2 cache
  int device = 0;
  float *d_F = nullptr;
  int l2_size = 0;
  cudaGetDevice(&device);
  cudaDeviceGetAttribute(&l2_size, cudaDevAttrL2CacheSize, device);
  size_t sizeF = l2_size * 2;
  cudaMalloc((void **)&d_F, sizeF);
  cudaMemsetAsync((void *) d_F, 0, sizeF);
  cudaDeviceSynchronize();  
  cudaFree(d_F);
}
void checkConfsCPU( std::vector<ConfigurationTagged> &out, const std::vector<Configuration> &confs, 
                    std::string robot_filename, std::string obstacle_filename){

    //Load Robot
    std::vector<fcl::Vector3f> rob_vertices;
    std::vector<fcl::Triangle> rob_triangles;
    std::vector<fcl::Vector3f> obs_vertices;
    std::vector<fcl::Triangle> obs_triangles;

    loadOBJFileFCL(robot_filename, rob_vertices, rob_triangles);
    loadOBJFileFCL(obstacle_filename, obs_vertices, obs_triangles);

    // std::cout << "robot has " << rob_vertices.size() << " vertices " <<std::endl;

    std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<float>>> rob_mesh(new fcl::BVHModel<fcl::OBBRSS<float>>);
    rob_mesh->beginModel(rob_triangles.size(), rob_vertices.size());
    rob_mesh->addSubModel(rob_vertices, rob_triangles);
    rob_mesh->endModel();
    // std::cout << "loaded robot" <<std::endl;

    // Load Obstacle
    std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<float>>> obs_mesh(new fcl::BVHModel<fcl::OBBRSS<float>>);
    obs_mesh->beginModel(obs_triangles.size(), obs_vertices.size());
    obs_mesh->addSubModel(obs_vertices, obs_triangles);
    obs_mesh->endModel();

    fcl::CollisionObject<float> rob_col_obj(rob_mesh);
    fcl::CollisionObject<float> obs_col_obj(obs_mesh);

    
    auto cpu_start_time = std::chrono::high_resolution_clock::now();
    
    // ************************************************************************//

    // perform collision detection on each of the randomly generated configs
    for(int i = 0; i < confs.size(); i++){
      // std::cout << "starting conf " << i << std::endl;
      fcl::Transform3f transform = configurationToTransform(confs[i]);
      rob_col_obj.setTransform(transform);

      // Define CollisionRequest and CollisionResult objects
      fcl::CollisionRequest<float> request(1);
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
    double cpu_duration = std::chrono::duration<double, std::milli>(cpu_end_time - cpu_start_time).count();

    std::cout << "cpu collision detection execution time: " << cpu_duration << " ms for" <<confs.size() << " configurations." << std::endl;

}

bool check_file_exists(const std::string& path) {
    std::filesystem::path p(path);

    if (!std::filesystem::exists(p)) {
        std::cerr << "Error: file does not exist: " << path << "\n";
        return false;
    }

    if (!std::filesystem::is_regular_file(p)) {
        std::cerr << "Error: not a regular file: " << path << "\n";
        return false;
    }

    return true;
}

template <typename BV>
std::vector<size_t> getBVHTreeDepths(const fcl::BVHModel<BV>& model)
{
    const int n = model.getNumBVs();
    if (n == 0) return {};

    size_t max_depth = 0;

    // stack of (node_index, depth)
    std::stack<std::pair<int, size_t>> st;
    st.push({0, 1});   // root is depth 1

    std::vector<size_t> leaf_depths(n, 0);
    while (!st.empty()) {
        auto [node_idx, depth] = st.top();
        st.pop();
        max_depth = std::max(max_depth, depth);

        const auto& node = model.getBV(node_idx);
        if (node.isLeaf()) {
            leaf_depths[node_idx] = depth;
            continue;
        }       
        // std::cout << "Node " << node_idx << " is " << node.isLeaf() << " and has " << node.num_primitives << "\n";
        int left  = node.leftChild();
        int right = node.rightChild();

        if (right >= 0){
            st.push({right, depth + 1});
        }
        if (left >= 0) {
            st.push({left, depth + 1});
        }
    }
    return leaf_depths;
}

// TODO: This assumes that B is a rotation matrix of B with respect to the axes of A
// we may want to calculate this dynamically, but for now, assume A axis aligned and centered at origin
OBB_soa hierarchy_from_mesh(const char* mesh_path){
    // Load Robot
    std::vector<fcl::Vector3f> rob_vertices;
    std::vector<fcl::Triangle> rob_triangles;


    loadOBJFileFCL(mesh_path, rob_vertices, rob_triangles);

    // why is this a pointer???
    std::shared_ptr<fcl::BVHModel<fcl::OBB<float>>> rob_mesh(new fcl::BVHModel<fcl::OBB<float>>);
    rob_mesh->beginModel(rob_triangles.size(), rob_vertices.size());
    rob_mesh->addSubModel(rob_vertices, rob_triangles);
    rob_mesh->endModel();

    getBVHTreeDepths(*rob_mesh);
    // Access OBB data from rob_mesh
    // rob_mesh->getNumBVs() gives the number of OBBs in the hierarchy
    size_t num_boxes = rob_mesh->getNumBVs();
    OBB_soa result(num_boxes);

    Eigen::Matrix3f rotation;
    Eigen::Vector3f translation;
    Eigen::Vector3f half_dimensions;

    //check to make sure all primitive ids are accounted for
    std::vector<bool> primitive_id_found(rob_mesh->num_tris, false);
    for (int i = 0; i < num_boxes; ++i) {
        fcl::OBB<float> obb = rob_mesh->getBV(i).bv;
        rotation = obb.axis;
        translation = obb.To;
        half_dimensions = obb.extent;
        auto node = rob_mesh->getBV(i);

        if (node.isLeaf()){
          primitive_id_found[node.primitiveId()] = true;
        }

        result.set(i, rotation, translation, half_dimensions);
    }

    //verify all primitive ids were found
    for (size_t i = 0; i < primitive_id_found.size(); i++){
        if (!primitive_id_found[i]){
            std::cout << "Warning: Primitive ID " << i << " was not found in any leaf OBB." << std::endl;
        }
    }

    // delete rob_mesh manually to free memory
    rob_mesh.reset();

    return result;
}
