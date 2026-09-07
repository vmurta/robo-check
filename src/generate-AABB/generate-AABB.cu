#include "generate-AABB.hu"

// generateAABBBaseline- Generate AABBs for all configurations serially
void generateAABBBaseline(Eigen::Vector3f* vertices, unsigned int numVertices, 
                    unsigned int numConfigs, AABB* botBounds) 
{
    // Loop over every configuration
    for(int i = 0; i < numConfigs; ++i)
    {
        // Loop over every vertex in each configuration
        unsigned int configOffset = i * numVertices;
        botBounds[i].x_min = vertices[configOffset](0);
        botBounds[i].y_min = vertices[configOffset](1);
        botBounds[i].z_min = vertices[configOffset](2);
        botBounds[i].x_max = vertices[configOffset](0);
        botBounds[i].y_max = vertices[configOffset](1);
        botBounds[i].z_max = vertices[configOffset](2);
        for(int j = 0; j < numVertices; ++j)
        {
            botBounds[i].x_min = min(botBounds[i].x_min, vertices[configOffset + j](0));
            botBounds[i].y_min = min(botBounds[i].y_min, vertices[configOffset + j](1));
            botBounds[i].z_min = min(botBounds[i].z_min, vertices[configOffset + j](2));
            botBounds[i].x_max = max(botBounds[i].x_max, vertices[configOffset + j](0));
            botBounds[i].y_max = max(botBounds[i].y_max, vertices[configOffset + j](1));
            botBounds[i].z_max = max(botBounds[i].z_max, vertices[configOffset + j](2));
        }
    }
}

// generateAABBBaseline- Generate AABBs for all configurations serially
void generateAABBBaseline(std::vector<float> &x, std::vector<float> &y, std::vector<float> &z, AABB* botBounds) 
{
    // Loop over every vertex in each configuration
    botBounds[0].x_min = FLT_MAX;
    botBounds[0].y_min = FLT_MAX;
    botBounds[0].z_min = FLT_MAX;
    botBounds[0].x_max = -FLT_MAX;
    botBounds[0].y_max = -FLT_MAX;
    botBounds[0].z_max = -FLT_MAX;
    for(int j = 0; j < x.size(); ++j)
    {
        botBounds[0].x_min = min(botBounds[0].x_min, x[j]);
        botBounds[0].y_min = min(botBounds[0].y_min, y[j]);
        botBounds[0].z_min = min(botBounds[0].z_min, z[j]);
        botBounds[0].x_max = max(botBounds[0].x_max, x[j]);
        botBounds[0].y_max = max(botBounds[0].y_max, y[j]);
        botBounds[0].z_max = max(botBounds[0].z_max, z[j]);
    }
}