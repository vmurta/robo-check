#include <iostream>
#include "Utils.h"

int main(){

    int num_confs = 10000;
    // std::vector<Configuration> easy_confs_ten_thousand(num_confs);
    // createAlphaBotConfigurations(easy_confs_ten_thousand, num_confs, false);
    // std::vector<ConfigurationTagged> easy_confs_ten_thousand_tagged(num_confs);
    // checkConfsCPU(easy_confs_ten_thousand_tagged, easy_confs_ten_thousand);
    // writeConfigurationToFileTagged(easy_confs_ten_thousand_tagged, "easy_confs10,000.conf");

    num_confs = 100000;
    std::vector<Configuration> easy_confs_hundred_thousand(num_confs);
    createAlphaBotConfigurations(easy_confs_hundred_thousand, num_confs, false);
    std::vector<ConfigurationTagged> easy_confs_hundred_thousand_tagged(num_confs);
    checkConfsCPU(easy_confs_hundred_thousand_tagged, easy_confs_hundred_thousand);
    writeConfigurationToFileTagged(easy_confs_hundred_thousand_tagged, "easy_confs100,000.conf");

    num_confs = 1000000;
    std::vector<Configuration> easy_confs_million(num_confs);
    createAlphaBotConfigurations(easy_confs_million, num_confs, false);
    std::vector<ConfigurationTagged> easy_confs_million_tagged(num_confs);
    checkConfsCPU(easy_confs_million_tagged, easy_confs_million);
    writeConfigurationToFileTagged(easy_confs_million_tagged, "easy_confs1,000,000.conf");

    num_confs = 10000;
    std::vector<Configuration> hard_confs_ten_thousand(num_confs);
    createAlphaBotConfigurations(hard_confs_ten_thousand, num_confs, false);
    std::vector<ConfigurationTagged> hard_confs_ten_thousand_tagged(num_confs);
    checkConfsCPU(hard_confs_ten_thousand_tagged, hard_confs_ten_thousand);
    writeConfigurationToFileTagged(hard_confs_ten_thousand_tagged, "hard_confs10,000.conf");
    num_confs = 1000000;
    std::vector<Configuration> hard_confs_hundred_thousand(num_confs);
    createAlphaBotConfigurations(hard_confs_hundred_thousand, num_confs, false);
    std::vector<ConfigurationTagged> hard_confs_hundred_thousand_tagged(num_confs);
    checkConfsCPU(hard_confs_hundred_thousand_tagged, hard_confs_hundred_thousand);
    writeConfigurationToFileTagged(hard_confs_hundred_thousand_tagged, "hard_confs100,000.conf");

    num_confs = 1000000;
    std::vector<Configuration> hard_confs_million(num_confs);
    createAlphaBotConfigurations(hard_confs_million, num_confs, false);
    std::vector<ConfigurationTagged> hard_confs_million_tagged(num_confs);
    checkConfsCPU(hard_confs_million_tagged, hard_confs_million);
    writeConfigurationToFileTagged(hard_confs_million_tagged, "hard_confs1,000,000.conf");
}