#include <iostream>
#include <fcl/fcl.h>

int main()
{
    // This AI generated part does not work, as we do not yet have a way to load
    // in .g format, which is BYU format meshes. 
    // We could either try to manually convert our meshes into obj format, or find 
    // some tool to automatically convert them in here.
    //************************************************************************//
    // // Load mesh 1 into BVHModel
    // fcl::BVHModel<fcl::OBBRSS<double>> mesh1;
    // if (!fcl::loadOBJ("mesh1.obj", mesh1)) {
    //     std::cerr << "Failed to load mesh1.obj" << std::endl;
    //     return 1;
    // }

    // // Create CollisionObject for mesh1
    // fcl::CollisionObject<fcl::OBBRSS<double>> co1(&mesh1);

    // // Load mesh 2 into BVHModel
    // fcl::BVHModel<fcl::OBBRSS<double>> mesh2;
    // if (!fcl::loadOBJ("mesh2.obj", mesh2)) {
    //     std::cerr << "Failed to load mesh2.obj" << std::endl;
    //     return 1;
    // }
    //************************************************************************//

    // Create CollisionObject for mesh2
    fcl::CollisionObject<fcl::OBBRSS<double>> co2(&mesh2);

    // Set transformation matrices if necessary
    fcl::Transform3<double> tf1;
    fcl::Transform3<double> tf2;
    co1.setTransform(tf1);
    co2.setTransform(tf2);

    // Define CollisionRequest and CollisionResult objects
    fcl::CollisionRequest<double> request;
    fcl::CollisionResult<double> result;

    // Perform collision detection
    fcl::collide(&co1, &co2, request, result);

    // Check if collision occurred
    if (result.isCollision()) {
        std::cout << "Collision detected!" << std::endl;
    } else {
        std::cout << "No collision detected." << std::endl;
    }

    return 0;
}
