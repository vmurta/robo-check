CXX=nvcc
CXXFLAGS=
LDFLAGS=-lfcl -lccd
BROAD_DIR=./broad-phase
AABB_DIR=./generate-AABB
NARROW_DIR=./narrow-phase
TRANS_DIR=./transformation
CU=nvcc

all: GPU-Mesh-Test

CPU-Mesh-Test: CPU-Mesh-Test.o Utils.o Utils_rai.o
	$(CXX) $(CXXFLAGS) $^ $(LDFLAGS) -o $@

GPU-Mesh-Test: GPU-Mesh-Test.o
	$(CU) $(CXXFLAGS) $< -o $@

GPU-transform-test: GPU-transform-test.o Utils_rai.o Utils.o transform.o
	$(CU) $(CXXFLAGS) $^ -g -o $@ $(LDFLAGS) 

CPU-Sphere-Test: CPU-Sphere-Test.o 
	$(CXX) $(CXXFLAGS) $< $(LDFLAGS) -o $@

Integration-Test: integration-test.o Utils_rai.o Utils.o generate-AABB.o broad-phase-fused.o
	$(CU) $(CXXFLAGS) $^ -g -o $@ $(LDFLAGS) -DLOCAL_TESTING=1


# CPU-Sphere-Test.o: CPU-Sphere-Test.cpp
# 	$(CXX) $(CXXFLAGS) -c $< -o $@

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
%.o: %.cu
	$(CU) -c $< -o $@

broad-phase.o : broad-phase/broad-phase.cu
	$(CU) -DLOCAL_TESTING=1 -c $< -o $@ -I. -I./transformation

broad-phase-fused.o : broad-phase/broad-phase-fused.cu transform.o
	$(CU) -DLOCAL_TESTING=1 -dc $< -o $@ -I./transformation

integration-test.o: test/integration-test.cu
	$(CU) -DLOCAL_TESTING=1 -c $< -o $@ -I.

transform.o: transformation/transform.cu
	$(CU) -DLOCAL_TESTING=1 -c $< -o $@ -I. 
	
GPU-transform-test.o: transformation/testing/GPU-transform-test.cu
	$(CU) -DLOCAL_TESTING=1 -c $< -o $@ -I. 

generate-AABB.o: generate-AABB/generate-AABB.cu
	$(CU) -DLOCAL_TESTING=1 -c $< -o $@ -I. -I./generate-AABB

CPU-Mesh-Test.o: test/CPU-Mesh-Test.cu
	$(CU) $(CXXFLAGS) -c $< -o $@


clean:
	rm -f *.o CPU-Sphere-Test CPU-Mesh-Test GPU-Mesh-Test GPU-transform-test CPU-Sphere-Test Integration-Test