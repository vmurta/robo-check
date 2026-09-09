# robo-check
Originally a final project for ECE/CS 508 at UIUC, this is a CUDA library intended for performing collision detection for the express purpose of detecting if robots are in collision for the purpose of  sampling based motion planning.

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

I had issues with actually linking against the libary, so I needed to throw a 

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib

In my .bashrc as well, make sure that path is where libfcl.so.0.7 is

## Collision detection API

### OBB overlap helper
The 15 OBB separating-axis tests live in `inc/full-stack-cc/OBB-BVH-naive.hu`:

```cpp
__device__ __forceinline__ bool obbOverlap(
    const Eigen::Vector3f& a, const Eigen::Vector3f& b,   // half extents (obs, rob)
    const Eigen::Matrix3f& B, const Eigen::Matrix3f& Bf,  // relative rotation, abs+eps
    const Eigen::Vector3f& T);                            // relative translation
```
Returns `true` when the boxes overlap, `false` when a separating axis proves them disjoint.

### Articulated configuration
`articulated_conf<N>` (in `ArticulatedRobot.hu`) holds `N` joint angles (radians):
```cpp
articulated_conf<6> c;
c[0] = 0.1f;                 // set joint 0
```

### Serial-chain URDF (`bvh_articulated`)
```cpp
#include "ArticulatedRobot.hu"

ArticulatedRobot robot;
parseSerialChainURDF("robot.urdf", robot);               // bool

std::vector<articulated_conf<6>> confs;
readArticulatedConfigurationFromFile<6>("confs.txt", confs);

BVNode_soa obsBVH = BVH_n_ary_hierarchy_from_mesh("obs.obj", 2);
MeshData obsMesh; loadOBJFile("obs.obj", obsMesh.vertices, obsMesh.triangles);

std::vector<bool> valid;
bvh_articulated<6>("robot.urdf", obsBVH, obsMesh, confs, valid, /*dry_run=*/true);
```

### Full (branching) URDF (`bvh_urdf`)
Same call shape, but supports arbitrary kinematic trees:
```cpp
#include "URDFRobot.hu"

std::vector<bool> valid;
bvh_urdf<6>("robot.urdf", obsBVH, obsMesh, confs, valid, /*dry_run=*/true);
```

### Device-side helpers
```cpp
forwardKinematics<N>(conf, joints, link_R, link_T);                       // serial
forwardKinematicsTree<N>(conf, num_links, parent, order,                  // tree
                         origin, axis, angle_idx, link_R, link_T);
```

### Caveats
- `N` is the number of movable joints and must equal `robot.num_joints`.
- Both wrappers assume **4-ary** BVHs (`BVH_n_ary_hierarchy_from_mesh(..., 2)`); build the obstacle BVH with the same branching factor.
- `parseFullURDF` treats revolute/continuous as angle joints; fixed/other types are rigid. Max 64 links (`URDF_MAX_LINKS`).
- Collision meshes must be OBJ (`loadOBJFile`); UR5 meshes under `data/ur5/` are STL/DAE, and per-link collision `<origin>` offsets are not yet applied.

### Build
```bash
make build/ArticulatedRobot.o build/URDFRobot.o
```