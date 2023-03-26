CXX=g++
CXXFLAGS=-Wall -Wextra -Wpedantic -std=c++11 -O3
LDFLAGS=-lfcl -lccd

#Make sure libfcl.so.0.7 is in here
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib


all: CPU-Sphere-Test 

CPU-Sphere-Test: CPU-Sphere-Test.o
	$(CXX) $(CXXFLAGS) $< $(LDFLAGS) -o $@

CPU-Sphere-Test.o: CPU-Sphere-Test.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f *.o CPU-Sphere-Test
