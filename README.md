# robo-check
A final project for ECE/CS 508 at UIUC, this is a CUDA library intended for performing collision detection for the express purpose of detecting if robots are in collision for the purpose of  sampling based motion planning.

## Resources
https://parasollab.web.illinois.edu/resources/mpbenchmarks/
Set of problems to test against

## Getting to run
Need CMAKE 3.14 at least to build fcl, follow the instructions here https://robots.uc3m.es/installation-guides/install-cmake.html if 

    cmake --version

spits out anything less than that.

### Eigen

    sudo apt install libeigen3-dev

Sudo should probably work, but i had to mess with the path since eigen hides things inside eigen3.
Add this to your .bashrc and then "source ~/.bashrc" if it isn't working
    
    export CPATH="/usr/include/eigen3"
 

### LibCCD
Clone this repo
https://github.com/danfis/libccd

It says it doesn't need any dependencies, it is lying. From within libccd/src directory:

    sudo apt install -y m4
    m4 -DUSE_DOUBLE ccd/config.h.m4 >ccd/config.h
    cd ..
    mkdir build && cd build
    cmake -G "Unix Makefiles" -DBUILD_SHARED_LIBS=ON ..
    make
    sudo make install

### FCL
Clone this repo
https://github.com/flexible-collision-library/fcl

cd into there, then

    mkdir build
    cd build
    cmake ..
    make -j4
    sudo make install