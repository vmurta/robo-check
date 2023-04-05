CXX=g++
CXXFLAGS=-Wall -Wextra -Wpedantic -std=c++11 -Wno-deprecated-declarations
LDFLAGS=-lfcl -lccd

all: CPU-Sphere-Test

CPU-Mesh-Test: CPU-Mesh-Test.o
	$(CXX) $(CXXFLAGS) $< $(LDFLAGS) -o $@

CPU-Sphere-Test: CPU-Sphere-Test.o
	$(CXX) $(CXXFLAGS) $< $(LDFLAGS) -o $@

<<<<<<< HEAD
=======
generate-test: generate-test.o
	$(CXX) $(CXXFLAGS) $< $(LDFLAGS) -o $@

# CPU-Sphere-Test.o: CPU-Sphere-Test.cpp
# 	$(CXX) $(CXXFLAGS) -c $< -o $@
>>>>>>> 783956b969c3fd7f18707f4220c9e28c1f94adcf
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f *.o CPU-Sphere-Test
