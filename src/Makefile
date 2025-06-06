CXX=nvcc
CXXFLAGS=
LDFLAGS=-lfcl -lccd
BROAD_DIR=./broad-phase
AABB_DIR=./generate-AABB
NARROW_DIR=./narrow-phase
TRANS_DIR=./transformation
CU=nvcc
CUFLAGS=-DLOCAL_TESTING=1 -lineinfo -O3 -Wno-deprecated-declarations

# CPU-Mesh-Test: CPU-Mesh-Test.o Utils.o
# 	$(CXX) $(CXXFLAGS) $^ $(LDFLAGS) -o $@


Full-Integration-Test: full-integration-test.o Utils.o generate-AABB.o broad-phase-fused.o narrow-phase.o broad-phase.o MegaKernel.o
	$(CU) $(CXXFLAGS) $^ -g -o $@ $(LDFLAGS) $(CUFLAGS)

Generate-Tests: generate-tests.o Utils.o
	$(CU) $(CXXFLAGS) $^ -g -o $@ $(LDFLAGS) $(CUFLAGS)



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

transform.o: transformation/transform.cu
	$(CU) $(CUFLAGS) -dc $< -o $@ -I. 
	
generate-AABB.o: generate-AABB/generate-AABB.cu
	$(CU) $(CUFLAGS) -dc $< -o $@ -I. -I./generate-AABB

generate-tests.o: generate-tests.cu
	$(CU) $(CUFLAGS) -dc $< -o $@ -I.

narrow-phase.o: narrow-phase/narrow-phase.cu
	$(CU) $(CUFLAGS) -dc $< -o $@ -I. -I./narrow-phase -I./broad-phase

MegaKernel.o: test/MegaKernel.cu
	$(CU) $(CUFLAGS) -dc $< -o $@ -I. -I./narrow-phase -I./broad-phase

clean:
	rm -f *.o CPU-Sphere-Test CPU-Mesh-Test GPU-transform-test CPU-Sphere-Test