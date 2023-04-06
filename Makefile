CXX=g++
CXXFLAGS=-Wall -Wextra -Wpedantic -std=c++11 -Wno-deprecated-declarations
LDFLAGS=-lfcl -lccd

all: GPU-Mesh-Test

CPU-Mesh-Test: CPU-Mesh-Test.o
	$(CXX) $(CXXFLAGS) $< $(LDFLAGS) -o $@

GPU-Mesh-Test: GPU-Mesh-Test.o
	$(CXX) $(CXXFLAGS) $< -o $@

CPU-Sphere-Test: CPU-Sphere-Test.o
	$(CXX) $(CXXFLAGS) $< $(LDFLAGS) -o $@

# CPU-Sphere-Test.o: CPU-Sphere-Test.cpp
# 	$(CXX) $(CXXFLAGS) -c $< -o $@

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f *.o CPU-Sphere-Test
