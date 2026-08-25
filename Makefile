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
TEST_INCLUDES:=$(INCLUDES) -I${TEST_DIR}/narrow-phase

CUFLAGS := $(COMMON_FLAGS) $(OPT_FLAGS)

ifeq ($(DEBUG),1)
    CUFLAGS := $(COMMON_FLAGS) $(DEBUG_FLAGS) 
endif

# Build MegaKernel with the robot mesh stored in constant memory
# Usage: make MEGA_CONST=1
ifeq ($(MEGA_CONST),1)
    CUFLAGS += -DMEGA_CONSTANT
endif

# Build directory
BUILD_DIR := build

# ---- Targets ----

# Library + test binaries (default)
all: Full-Integration-Test Generate-Tests profiling
.PHONY: clean all tests profiling

tests: Test-Narrow-Phase
profiling: BVH

# ---- Build rules ----

Full-Integration-Test: $(BUILD_DIR)/full-integration-test.o $(BUILD_DIR)/Utils.o $(BUILD_DIR)/generate-AABB.o $(BUILD_DIR)/broad-phase-fused.o $(BUILD_DIR)/narrow-phase.o $(BUILD_DIR)/MegaKernel.o
	$(NVCC) $(CXXFLAGS) $^ -o $@ $(LDFLAGS) $(CUFLAGS)

Generate-Tests: $(BUILD_DIR)/generate-tests.o $(BUILD_DIR)/Utils.o
	$(NVCC) $(CXXFLAGS) $^ -o $@ $(LDFLAGS) $(CUFLAGS)

Test-Narrow-Phase: $(BUILD_DIR)/test-narrow-phase.o $(BUILD_DIR)/narrow-phase.o $(BUILD_DIR)/Triangle.o
	$(NVCC) $(CXXFLAGS) $^ -o $@ $(LDFLAGS) $(CUFLAGS)

BVH: $(BUILD_DIR)/Utils.o $(BUILD_DIR)/OBB-BVH-naive.o $(BUILD_DIR)/OBB-single-buff.o $(BUILD_DIR)/OBB-double-buff.o $(BUILD_DIR)/OBB-naive.o $(BUILD_DIR)/Triangle.o $(BUILD_DIR)/obb_test.o
	$(NVCC) $(CXXFLAGS) $^ -o $@ $(LDFLAGS) $(CUFLAGS)

Debug: $(BUILD_DIR)/obb_test.o $(BUILD_DIR)/Utils.o
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

# test/ (uses TEST_INCLUDES for test-narrow-phase.hu)
$(BUILD_DIR)/test-narrow-phase.o: $(TEST_DIR)/narrow-phase/test-narrow-phase.cu | $(BUILD_DIR)
	$(NVCC) $(CUFLAGS) $(TEST_INCLUDES) -dc $< -o $@

# profiling/
$(BUILD_DIR)/obb_test.o: $(PROF_DIR)/obb_test.cu | $(BUILD_DIR)
	$(NVCC) $(CUFLAGS) $(INCLUDES) -dc $< -o $@

# ---- Explicit rules ----

$(BUILD_DIR)/broad-phase-fused.o: $(BROAD_DIR)/broad-phase-fused.cu | $(BUILD_DIR)
	$(NVCC) $(CUFLAGS) -dc $< -o $@ $(INCLUDES)

$(BUILD_DIR)/full-integration-test.o: $(TEST_DIR)/full-integration-test.cu | $(BUILD_DIR)
	$(NVCC) $(CUFLAGS) $(INCLUDES) -dc $^ -o $@

$(BUILD_DIR)/generate-AABB.o: $(AABB_DIR)/generate-AABB.cu | $(BUILD_DIR)
	$(NVCC) $(CUFLAGS) -dc $< -o $@ $(INCLUDES)

$(BUILD_DIR)/generate-tests.o: $(TEST_DIR)/generate-tests.cu | $(BUILD_DIR)
	$(NVCC) $(CUFLAGS) -dc $< -o $@ $(INCLUDES)

$(BUILD_DIR)/narrow-phase.o: $(NAR_DIR)/narrow-phase.cu | $(BUILD_DIR)
	$(NVCC) $(CUFLAGS) -dc $< -o $@ $(INCLUDES)

$(BUILD_DIR)/MegaKernel.o: $(SRC_DIR)/full-stack-cc/MegaKernel.cu | $(BUILD_DIR)
	$(NVCC) $(CUFLAGS) $(INCLUDES) -dc $< -o $@

$(BUILD_DIR)/OBB-BVH-naive.o: $(SRC_DIR)/full-stack-cc/OBB-BVH-naive.cu | $(BUILD_DIR)
	$(NVCC) $(CUFLAGS) $(INCLUDES) -dc $< -o $@

clean:
	rm -rf *.o Full-Integration-Test Generate-Tests Test-Narrow-Phase BVH Debug build
