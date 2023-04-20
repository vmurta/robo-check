#include "generate-AABB.hu"

void generateTestVertices(Vector3f* robPts)
{
    Vector3f pt0(2, 3, 4);
    Vector3f pt1(0, 1, 2);
    Vector3f pt2(0, 2, 0);

    Vector3f pt3(2, 2, 2);
    Vector3f pt4(100, 0, 1);
    Vector3f pt5(100, 1, 2);

    Vector3f pt6(100, 2, 0);
    Vector3f pt7(102, 2, 2);
    Vector3f pt8(102, 2, 80);

    Vector3f pt9(12, 22, 4);
    Vector3f pt10(16, 6, 2);
    Vector3f pt11(13, 21, 27);

    robPts[0] = pt0;
    robPts[1] = pt1;
    robPts[2] = pt2;
    robPts[3] = pt3;
    robPts[4] = pt4;
    robPts[5] = pt5;
    robPts[6] = pt6;
    robPts[7] = pt7;
    robPts[8] = pt8;
    robPts[9] = pt9;
    robPts[10] = pt10;
    robPts[11] = pt11;
}

void test_generateAABBBaseline(AABB* botBounds, const int numConfigs)
{
    const int numVertices = 6;
    Vector3f robPts[numVertices * numConfigs];

    generateTestVertices(robPts);

    generateAABBBaseline(robPts, numVertices, numConfigs, botBounds);

    // for(int i = 0; i < numConfigs; ++i)
    // {
    //     std::cout   << "(" << botBounds[i].x_max << " " << botBounds[i].x_min
    //                 << " " << botBounds[i].y_max << " " << botBounds[i].y_min
    //                 << " " << botBounds[i].z_max << " " << botBounds[i].z_min << ")" << std::endl;
    // }
}

void test_generateAABB(AABB* botBounds, const int numConfigs)
{
    const int numVertices = 6;
    Vector3f robPts[numVertices * numConfigs];

    generateTestVertices(robPts);

    generateAABB(robPts, numVertices, numConfigs, botBounds);

    // for(int i = 0; i < numConfigs; ++i)
    // {
    //     std::cout   << "(" << botBounds[i].x_max << " " << botBounds[i].x_min
    //                 << " " << botBounds[i].y_max << " " << botBounds[i].y_min
    //                 << " " << botBounds[i].z_max << " " << botBounds[i].z_min << ")" << std::endl;
    // }
}

bool verify_generateAABB(AABB* botBoundsBaseline, AABB* botBoundsParallel, const int numConfigs)
{
    for(int i = 0; i < numConfigs; ++i)
    {
        if(!(botBoundsBaseline[i].x_min == botBoundsParallel[i].x_min &&
           botBoundsBaseline[i].y_min == botBoundsParallel[i].y_min &&
           botBoundsBaseline[i].z_min == botBoundsParallel[i].z_min &&
           botBoundsBaseline[i].x_max == botBoundsParallel[i].x_max &&
           botBoundsBaseline[i].y_max == botBoundsParallel[i].y_max &&
           botBoundsBaseline[i].z_max == botBoundsParallel[i].z_max))
            return false;
    }
    return true;
}

int main()
{
    std::cout << "====AABB tests====" << std::endl;
    const int numConfigs = 2;
    std::cout << "Running AABB baseline test..." << std::endl;
    AABB botBoundsBaseline[numConfigs];
    test_generateAABBBaseline(botBoundsBaseline, numConfigs);
    std::cout << "Running AABB parallel kernel test..." << std::endl;
    AABB botBoundsParallel[numConfigs];
    test_generateAABB(botBoundsParallel, numConfigs);
    std::cout << "Verifying AABB parallel kernel test..." << std::endl;
    if(verify_generateAABB(botBoundsBaseline, botBoundsParallel, numConfigs))
        std::cout << "[PASS] Parallel implementation matches serial implementation." << std::endl;
    else
        std::cout << "[FAIL] Parallel implementation does not match serial implementation." << std::endl;
    std::cout << "==================" << std::endl;
    return 0;
}