#include <iostream>

#include <fcl/narrowphase/collision.h>
#include <fcl/geometry/shape/sphere.h>
int main()
{
    // Create two spheres for collision testing
    fcl::Sphere<double> sphere1(1.0);
    fcl::Sphere<double> sphere2(1.0);
    
    // Create a collision request object
    fcl::CollisionRequest<double> request;
    
    // Create a collision result object
    fcl::CollisionResult<double> result;
    
    // Perform collision detection
    fcl::collide(&sphere1, fcl::Transform3<double>(), &sphere2, fcl::Transform3<double>(), request, result);
    
    // Print the collision result
    std::cout << "Collision detected: " << result.isCollision() << std::endl;
    
    return 0;
}