CXX=nvcc
CXXFLAGS=
LDFLAGS=-lfcl -lccd
BROAD_DIR=./broad-phase
AABB_DIR=./generate-AABB
NARROW_DIR=./narrow-phase
TRANS_DIR=./transformation
CU=nvcc
CUFLAGS=-DLOCAL_TESTING=1 -lineinfo -Wno-deprecated-declarations

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

Full-Integration-Test: full-integration-test.o Utils_rai.o Utils.o generate-AABB.o broad-phase-fused.o narrow-phase.o
	$(CU) $(CXXFLAGS) $^ -g -o $@ $(LDFLAGS) $(CUFLAGS)
# CPU-Sphere-Test.o: CPU-Sphere-Test.cpp
# 	$(CXX) $(CXXFLAGS) -c $< -o $@

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
%.o: %.cu
	$(CU) $(CUFLAGS) -c $< -o $@

broad-phase.o : broad-phase/broad-phase.cu
	$(CU) $(CUFLAGS) -c $< -o $@ -I. -I./transformation

broad-phase-fused.o : broad-phase/broad-phase-fused.cu transform.o
	$(CU) $(CUFLAGS) -dc $< -o $@ -I./transformation -I./narrow-phase

full-integration-test.o: test/full-integration-test.cu
	$(CU) $(CUFLAGS) -c $< -o $@ -I.

integration-test.o: test/integration-test.cu
	$(CU) $(CUFLAGS) -c $< -o $@ -I.

transform.o: transformation/transform.cu
	$(CU) $(CUFLAGS) -c $< -o $@ -I. 
	
GPU-transform-test.o: transformation/testing/GPU-transform-test.cu
	$(CU) $(CUFLAGS) -c $< -o $@ -I. 

generate-AABB.o: generate-AABB/generate-AABB.cu
	$(CU) $(CUFLAGS) -c $< -o $@ -I. -I./generate-AABB

CPU-Mesh-Test.o: test/CPU-Mesh-Test.cu
	$(CU) $(CUFLAGS) -c $< -o $@ -I. -I./narrow-phase

narrow-phase.o: narrow-phase/narrow-phase.cu
	$(CU) $(CUFLAGS) -c $< -o $@ -I. -I./narrow-phase

clean:
	rm -f *.o CPU-Sphere-Test CPU-Mesh-Test GPU-Mesh-Test GPU-transform-test CPU-Sphere-Test Integration-Test