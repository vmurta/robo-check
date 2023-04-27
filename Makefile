CXX=nvcc
CXXFLAGS=
LDFLAGS=-lfcl -lccd

CU=nvcc

all: GPU-Mesh-Test

CPU-Mesh-Test: CPU-Mesh-Test.o
	$(CXX) $(CXXFLAGS) $< $(LDFLAGS) -o $@

GPU-Mesh-Test: GPU-Mesh-Test.o
	$(CU) $(CXXFLAGS) $< -o $@

GPU-transform-test: GPU-transform-test.o Utils_rai.o Utils.o transform.o
	$(CU) $(CXXFLAGS) $^ -g -o $@ $(LDFLAGS) 

CPU-Sphere-Test: CPU-Sphere-Test.o
	$(CXX) $(CXXFLAGS) $< $(LDFLAGS) -o $@

# CPU-Sphere-Test.o: CPU-Sphere-Test.cpp
# 	$(CXX) $(CXXFLAGS) -c $< -o $@

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
%.o: %.cu
	$(CU) -c $< -o $@

transform.o: transformation/transform.cu
	$(CU) -c $< -o $@ -I. 
	
GPU-transform-test.o: transformation/testing/GPU-transform-test.cu
	$(CU) -c $< -o $@ -I. 



clean:
	rm -f *.o CPU-Sphere-Test
