#include <iostream>
#include <cstdlib>
#include "Utils.h"

void usage(const char* prog){
    std::cerr << "Usage: " << prog << " <model_path> <num_confs_in_collision> <total_num_confs> <output_file>" << std::endl;
    std::cerr << "  model_path             - path to .obj or .g model file" << std::endl;
    std::cerr << "  num_confs_in_collision - number of configs verified to be in collision" << std::endl;
    std::cerr << "  total_num_confs        - total number of configurations to generate" << std::endl;
    std::cerr << "  output_file            - path to write the tagged .conf output" << std::endl;
}

int main(int argc, char* argv[]){
    if(argc != 5){
        usage(argv[0]);
        return 1;
    }

    std::string model_path = argv[1];
    int num_confs_in_collision = atoi(argv[2]);
    int total_num_confs = atoi(argv[3]);
    std::string output_file = argv[4];

    std::vector<Configuration> confs(total_num_confs);
    createAlphaBotConfigurations(model_path, confs, num_confs_in_collision, total_num_confs);

    std::vector<ConfigurationTagged> tagged(total_num_confs);
    checkConfsCPU(tagged, confs, model_path, "./data/models/alpha1.0/obstacle.obj");
    writeConfigurationToFileTagged(tagged, output_file);

    int valid = 0, invalid = 0;
    for(const auto &t : tagged){
      if(t.valid) valid++;
      else invalid++;
    }
    std::cout << "Wrote " << total_num_confs << " configurations to " << output_file
              << " (" << invalid << " in collision, " << valid << " not in collision)" << std::endl;
    return 0;
}
