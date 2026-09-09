NVCC := nvcc

COMMON_FLAGS := -lineinfo -Wno-deprecated-declarations --expt-relaxed-constexpr -diag-suppress 20012 -lfcl -lccd
OPT_FLAGS := -O3
DEBUG_FLAGS := -G -g -O0 -DLOCAL_TESTING=1

# Directory layout
SRC_DIR=./src
BROAD_DIR=${SRC_DIR}/broad-phase
AABB_DIR=${SRC_DIR}/generate-AABB
NAR_DIR=${SRC_DIR}/narrow-phase
INC_DIR=./inc
TEST_DIR=./test
PROF_DIR=./profiling

# Include paths
INCLUDES:=-I${INC_DIR} -I${INC_DIR}/full-stack-cc -I${INC_DIR}/broad-phase -I${INC_DIR}/generate-AABB -I${INC_DIR}/narrow-phase -I${SRC_DIR}/generate-AABB

CUFLAGS := $(COMMON_FLAGS) $(OPT_FLAGS)

ifeq ($(DEBUG),1)
    CUFLAGS := $(COMMON_FLAGS) $(DEBUG_FLAGS) 
endif

# Build directory
BUILD_DIR := build

# ---- Targets ----

# Library + benchmark binaries (default)
all: Generate-Tests profiling
.PHONY: clean all profiling

profiling: BVH

rtcd-bench: $(BUILD_DIR)/rtcd_bench.o $(BUILD_DIR)/Utils.o $(BUILD_DIR)/ArticulatedRobot.o $(BUILD_DIR)/OBB-BVH-naive.o $(BUILD_DIR)/Triangle.o
	$(NVCC) $(CXXFLAGS) $^ -o $@ $(LDFLAGS) $(CUFLAGS)

# ---- Build rules ----

Generate-Tests: $(BUILD_DIR)/generate-tests.o $(BUILD_DIR)/Utils.o
	$(NVCC) $(CXXFLAGS) $^ -o $@ $(LDFLAGS) $(CUFLAGS)

BVH: $(BUILD_DIR)/Utils.o $(BUILD_DIR)/OBB-BVH-naive.o $(BUILD_DIR)/OBB-single-buff.o $(BUILD_DIR)/OBB-double-buff.o $(BUILD_DIR)/OBB-naive.o $(BUILD_DIR)/Triangle.o $(BUILD_DIR)/obb_test.o
	$(NVCC) $(CXXFLAGS) $^ -o $@ $(LDFLAGS) $(CUFLAGS)

Debug: $(BUILD_DIR)/obb_test.o $(BUILD_DIR)/Utils.o $(BUILD_DIR)/OBB-BVH-naive.o $(BUILD_DIR)/OBB-single-buff.o $(BUILD_DIR)/OBB-double-buff.o $(BUILD_DIR)/OBB-naive.o $(BUILD_DIR)/Triangle.o
	$(NVCC) $(CXXFLAGS) $^ -o $@ $(LDFLAGS) $(CUFLAGS) -G -g -O0

# ---- Object file rules ----

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# src/ generic rules
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -dc $< -o $@

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cu | $(BUILD_DIR)
	$(NVCC) $(CUFLAGS) $(INCLUDES) -dc $< -o $@

# broad-phase/
$(BUILD_DIR)/%.o: $(BROAD_DIR)/%.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -dc $< -o $@

$(BUILD_DIR)/%.o: $(BROAD_DIR)/%.cu | $(BUILD_DIR)
	$(NVCC) $(CUFLAGS) $(INCLUDES) -dc $< -o $@

# narrow-phase/ (library)
$(BUILD_DIR)/%.o: $(NAR_DIR)/%.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -dc $< -o $@

$(BUILD_DIR)/%.o: $(NAR_DIR)/%.cu | $(BUILD_DIR)
	$(NVCC) $(CUFLAGS) $(INCLUDES) -dc $< -o $@

# profiling/
$(BUILD_DIR)/obb_test.o: $(PROF_DIR)/obb_test.cu | $(BUILD_DIR)
	$(NVCC) $(CUFLAGS) $(INCLUDES) -dc $< -o $@

$(BUILD_DIR)/rtcd_bench.o: $(PROF_DIR)/rtcd_bench.cu | $(BUILD_DIR)
	$(NVCC) $(CUFLAGS) $(INCLUDES) -dc $< -o $@

# ---- Explicit rules ----

$(BUILD_DIR)/generate-AABB.o: $(AABB_DIR)/generate-AABB.cu | $(BUILD_DIR)
	$(NVCC) $(CUFLAGS) -dc $< -o $@ $(INCLUDES)

$(BUILD_DIR)/generate-tests.o: $(TEST_DIR)/generate-tests.cu | $(BUILD_DIR)
	$(NVCC) $(CUFLAGS) -dc $< -o $@ $(INCLUDES)

$(BUILD_DIR)/OBB-BVH-naive.o: $(SRC_DIR)/full-stack-cc/OBB-BVH-naive.cu | $(BUILD_DIR)
	$(NVCC) $(CUFLAGS) $(INCLUDES) -dc $< -o $@

$(BUILD_DIR)/ArticulatedRobot.o: $(SRC_DIR)/full-stack-cc/ArticulatedRobot.cu | $(BUILD_DIR)
	$(NVCC) $(CUFLAGS) $(INCLUDES) -dc $< -o $@

$(BUILD_DIR)/URDFRobot.o: $(SRC_DIR)/full-stack-cc/URDFRobot.cu | $(BUILD_DIR)
	$(NVCC) $(CUFLAGS) $(INCLUDES) -dc $< -o $@

clean:
	rm -rf *.o Generate-Tests BVH Debug build
