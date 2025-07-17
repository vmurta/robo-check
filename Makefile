CXX=nvcc
CXXFLAGS=
LDFLAGS=-lfcl -lccd

SRC_DIR=./src
BROAD_DIR=${SRC_DIR}/broad-phase
AABB_DIR=${SRC_DIR}/generate-AABB
NAR_DIR=${SRC_DIR}/narrow-phase
TRANS_DIR=${SRC_DIR}/transformation
TEST_DIR=./test
CU=nvcc
CUFLAGS=-DLOCAL_TESTING=1 -lineinfo -O3 -Wno-deprecated-declarations
# Default target
all: Full-Integration-Test Generate-Tests
.PHONY: clean all

# Ensure build directory exists
BUILD_DIR := build

# Update targets to use build directory object files
Full-Integration-Test: $(BUILD_DIR)/full-integration-test.o $(BUILD_DIR)/Utils.o $(BUILD_DIR)/generate-AABB.o $(BUILD_DIR)/broad-phase-fused.o $(BUILD_DIR)/narrow-phase.o  $(BUILD_DIR)/MegaKernel.o
	$(CU) $(CXXFLAGS) $^ -g -o $@ $(LDFLAGS) $(CUFLAGS)

Generate-Tests: $(BUILD_DIR)/generate-tests.o $(BUILD_DIR)/Utils.o
	$(CU) $(CXXFLAGS) $^ -g -o $@ $(LDFLAGS) $(CUFLAGS)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Pattern rules for object files in build directory from src directory
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -dc $< -o $@

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cu | $(BUILD_DIR)
	$(CU) $(CUFLAGS) -dc $< -o $@

# Update explicit rules to output to build directory

$(BUILD_DIR)/broad-phase-fused.o : ${BROAD_DIR}/broad-phase-fused.cu | $(BUILD_DIR)
	$(CU) $(CUFLAGS) -dc $< -o $@ -I${TRANS_DIR}

$(BUILD_DIR)/full-integration-test.o: ${TEST_DIR}/full-integration-test.cu | $(BUILD_DIR)
	$(CU) $(CUFLAGS) -dc $^ -o $@

$(BUILD_DIR)/transform.o: ${TRANS_DIR}/transform.cu | $(BUILD_DIR)
	$(CU) $(CUFLAGS) -dc $< -o $@ -I. 
	
$(BUILD_DIR)/generate-AABB.o: ${AABB_DIR}/generate-AABB.cu | $(BUILD_DIR)
	$(CU) $(CUFLAGS) -dc $< -o $@ -I. -I./generate-AABB

$(BUILD_DIR)/generate-tests.o: ${TEST_DIR}/generate-tests.cu | $(BUILD_DIR)
	$(CU) $(CUFLAGS) -dc $< -o $@ -I.

$(BUILD_DIR)/narrow-phase.o: ${NAR_DIR}/narrow-phase.cu | $(BUILD_DIR)
	$(CU) $(CUFLAGS) -dc $< -o $@ -I. -I./narrow-phase -I./broad-phase

$(BUILD_DIR)/MegaKernel.o: ${SRC_DIR}/MegaKernel.cu | $(BUILD_DIR)
	$(CU) $(CUFLAGS) -dc $< -o $@ -I. -I./narrow-phase -I./broad-phase


clean:
	rm -rf *.o Full-Integration-Test Generate-Tests build