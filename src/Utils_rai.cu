#include <fstream>
#include <iostream>
#include <random>
#include <string.h>
#include <fstream>
#include <vector>
#include <string>
#include <stdexcept>
#include "Utils_rai.h"

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

    int validCount = 0;
    int invalidCount = 0;
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

void createAlphaBotConfigurations(std::vector<Configuration> &confs, int num_confs, bool hard){
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

    if(hard){
      generateConfs(confs,  -x_range/200, x_range/200,
                            -y_range/200, y_range/200,
                            -z_range/200, z_range/200,
                         num_confs);
    } else {
      generateConfs(confs,  -x_range * 10, x_range* 10,
                            -y_range * 10, y_range* 10,
                            -z_range * 10, z_range* 10,
                         num_confs);
    }

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

void loadOBJFile(const char* filename,  std::vector<float>& x, std::vector<float> &y, std::vector<float> &z,
                                        std::vector<int>& v1, std::vector<int>& v2, std::vector<int>& v3){

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
        confs[i] =conf;
    }

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