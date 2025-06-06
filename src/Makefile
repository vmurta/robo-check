CXX=nvcc
CXXFLAGS=
LDFLAGS=-lfcl -lccd
BROAD_DIR=./broad-phase
AABB_DIR=./generate-AABB
NARROW_DIR=./narrow-phase
TRANS_DIR=./transformation
CU=nvcc
CUFLAGS=-DLOCAL_TESTING=1 -lineinfo -O3 -Wno-deprecated-declarations

all: GPU-Mesh-Test

CPU-Mesh-Test: CPU-Mesh-Test.o Utils.o Utils_rai.o
	$(CXX) $(CXXFLAGS) $^ $(LDFLAGS) -o $@

GPU-Mesh-Test: GPU-Mesh-Test.o
	$(CU) $(CXXFLAGS) $< -o $@

GPU-transform-test: GPU-transform-test.o Utils_rai.o Utils.o transform.o
	$(CU) $(CXXFLAGS) $^ -g -o $@ $(LDFLAGS) 

CPU-Sphere-Test: CPU-Sphere-Test.o 
	$(CXX) $(CXXFLAGS) $< $(LDFLAGS) -o $@

Integration-Test: integration-test.o Utils_rai.o Utils.o generate-AABB.o broad-phase-fused.o narrow-phase.o
	$(CU) $(CXXFLAGS) $^ -g -o $@ $(LDFLAGS) $(CUFLAGS)

Full-Integration-Test: full-integration-test.o Utils_rai.o Utils.o generate-AABB.o broad-phase-fused.o narrow-phase.o broad-phase.o MegaKernel.o
	$(CU) $(CXXFLAGS) $^ -g -o $@ $(LDFLAGS) $(CUFLAGS)

Generate-Tests: generate-tests.o Utils_rai.o Utils.o
	$(CU) $(CXXFLAGS) $^ -g -o $@ $(LDFLAGS) $(CUFLAGS)

# CPU-Sphere-Test.o: CPU-Sphere-Test.cpp
# 	$(CXX) $(CXXFLAGS) -dc $< -o $@

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -dc $< -o $@
%.o: %.cu
	$(CU) $(CUFLAGS) -dc $< -o $@

broad-phase.o : broad-phase/broad-phase.cu
	$(CU) $(CUFLAGS) -dc $< -o $@ -I. -I./transformation

broad-phase-fused.o : broad-phase/broad-phase-fused.cu 
	$(CU) $(CUFLAGS) -dc $< -o $@ -I./transformation -I./narrow-phase

full-integration-test.o: test/full-integration-test.cu
	$(CU) $(CUFLAGS) -dc $^ -o $@

integration-test.o: test/integration-test.cu
	$(CU) $(CUFLAGS) -dc $< -o $@ -I.

transform.o: transformation/transform.cu
	$(CU) $(CUFLAGS) -dc $< -o $@ -I. 
	
GPU-transform-test.o: transformation/testing/GPU-transform-test.cu
	$(CU) $(CUFLAGS) -dc $< -o $@ -I. 

generate-AABB.o: generate-AABB/generate-AABB.cu
	$(CU) $(CUFLAGS) -dc $< -o $@ -I. -I./generate-AABB

generate-tests.o: generate-tests.cu
	$(CU) $(CUFLAGS) -dc $< -o $@ -I.

CPU-Mesh-Test.o: test/CPU-Mesh-Test.cu
	$(CU) $(CUFLAGS) -dc $< -o $@ -I. -I./narrow-phase

narrow-phase.o: narrow-phase/narrow-phase.cu
	$(CU) $(CUFLAGS) -dc $< -o $@ -I. -I./narrow-phase -I./broad-phase

MegaKernel.o: test/MegaKernel.cu
	$(CU) $(CUFLAGS) -dc $< -o $@ -I. -I./narrow-phase -I./broad-phase

clean:
	rm -f *.o CPU-Sphere-Test CPU-Mesh-Test GPU-Mesh-Test GPU-transform-test CPU-Sphere-Test Integration-Test