#include <iostream>
#include <fcl/fcl.h>

int main()
{
    // This AI generated part does not work, as we do not yet have a way to load
    // in .g format, which is BYU format meshes. 
    // We could either try to manually convert our meshes into obj format, or find 
    // some tool to automatically convert them in here.
    // ************************************************************************//
    // Load mesh 1 into BVHModel
    fcl::BVHModel<fcl::OBBRSS<double>> robot;
    if (!fcl::loadOBJ("./models/robot.obj", robot)) {
        std::cerr << "Failed to load robot.obj" << std::endl;
        return 1;
    }

    // Create CollisionObject for robot
    fcl::CollisionObject<fcl::OBBRSS<double>> co1(&robot);

    // Load mesh 2 into BVHModel
    fcl::BVHModel<fcl::OBBRSS<double>> obstacle;
    if (!fcl::loadOBJ("obstacle.obj", obstacle)) {
        std::cerr << "Failed to load obstacle.obj" << std::endl;
        return 1;
    }
    // ************************************************************************//

    // Create CollisionObject for obstacle
    fcl::CollisionObject<fcl::OBBRSS<double>> co2(&obstacle);

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
