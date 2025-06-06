#pragma once
#include <fstream>
#include <fcl/common/types.h>
#include <fcl/geometry/shape/utility.h>
#include <fcl/fcl.h>
#include "Utils_rai.h"

// This function taken from https://github.com/flexible-collision-library/fcl/issues/131 Github user dblanm
void loadOBJFileFCL(const char* filename, std::vector<fcl::Vector3f>& points, std::vector<fcl::Triangle>& triangles);
fcl::Transform3f configurationToTransform(const Configuration& config);
void checkConfsCPU(std::vector<ConfigurationTagged> &out, const std::vector<Configuration> &confs);