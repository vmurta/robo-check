#include "Utils_rai.h"

// #define AABB_BLOCK_SIZE 32
#define AABB_BLOCK_SIZE_X 792
#define AABB_BLOCK_SIZE_Y 32

// generateAABBPrimitiveKernel - Basic, unoptimized parallel kernel to generate AABBs
//      - Single dimension block.
//      - Parallellizes over configurations i.e. each thread handles one configuration.
//      - Each thread loops over all vertices to calculate the AABB.
__global__ void generateAABBPrimitiveKernel(Vector3f* vertices, unsigned int numVertices, 
                    unsigned int num_configs, AABB* bot_bounds) 
{
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if(tid < num_configs)
    {
        unsigned int config_ofset = tid * numVertices;
        bot_bounds[tid].x_min = vertices[config_ofset].x;
        bot_bounds[tid].y_min = vertices[config_ofset].y;
        bot_bounds[tid].z_min = vertices[config_ofset].z;
        bot_bounds[tid].x_max = vertices[config_ofset].x;
        bot_bounds[tid].y_max = vertices[config_ofset].y;
        bot_bounds[tid].z_max = vertices[config_ofset].z;
        for(int j = 0; j < numVertices; ++j)
        {
            bot_bounds[tid].x_min = min(bot_bounds[tid].x_min, vertices[j].x);
            bot_bounds[tid].y_min = min(bot_bounds[tid].y_min, vertices[j].y);
            bot_bounds[tid].z_min = min(bot_bounds[tid].z_min, vertices[j].z);
            bot_bounds[tid].x_max = max(bot_bounds[tid].x_max, vertices[j].x);
            bot_bounds[tid].y_max = max(bot_bounds[tid].y_max, vertices[j].y);
            bot_bounds[tid].z_max = max(bot_bounds[tid].z_max, vertices[j].z);
        }
    }
}

// generateAABBKernel - Optimized parallel kernel to generate AABBs
//      - Two dimension block - each yDim corresponds to one config, each xDim corresponds to vertices.
//      - Uses reduction along xDim to calculate AABBs for each configuration.
__global__ void generateAABBKernel(Vector3f* vertices, unsigned int numVertices, 
                    unsigned int num_configs, AABB* bot_bounds) 
{    
    __shared__ Vector3f partialMin[2 * AABB_BLOCK_SIZE_X];
    __shared__ Vector3f partialMax[2 * AABB_BLOCK_SIZE_X];
    AABB bot_bounds_local;

    unsigned int ty = blockIdx.y * blockDim.y + threadIdx.y;
    unsigned int tx = threadIdx.x;
    unsigned int inputStartIdx = 2*blockIdx.x*blockDim.x*ty;
    
    unsigned int totalNumVertices = numVertices * num_configs;

    if(ty < num_configs)
    {
        if((inputStartIdx + tx) < totalNumVertices && tx < numVertices)
        {
            partialMin[tx] = vertices[inputStartIdx + tx];
            partialMax[tx] = vertices[inputStartIdx + tx];
            if((inputStartIdx + tx + blockDim.x) < totalNumVertices 
                && (tx + blockDim.x) < numVertices)
            {
                partialMin[tx + blockDim.x] = vertices[inputStartIdx + tx + blockDim.x];
                partialMax[tx + blockDim.x] = vertices[inputStartIdx + tx + blockDim.x];
            }
            else
            {
                partialMin[tx + blockDim.x] = Vector3f(0,0,0);
                partialMax[tx + blockDim.x] = Vector3f(0,0,0);
            }
        }
        else
        {
            partialMin[tx] = Vector3f(0,0,0);
            partialMin[tx + blockDim.x] = Vector3f(0,0,0);
            partialMax[tx] = Vector3f(0,0,0);
            partialMax[tx + blockDim.x] = Vector3f(0,0,0);
        }
        for(unsigned int stride = blockDim.x; stride >= 1; stride /= 2)
        {
            __syncthreads();
            if(tx < stride)
            {
                partialMin[tx].x = min(partialMin[tx].x, partialMin[tx + stride].x);
                partialMin[tx].y = min(partialMin[tx].y, partialMin[tx + stride].y);
                partialMin[tx].z = min(partialMin[tx].z, partialMin[tx + stride].z);
                partialMax[tx].x = max(partialMax[tx].x, partialMax[tx + stride].x);
                partialMax[tx].y = max(partialMax[tx].y, partialMax[tx + stride].y);
                partialMax[tx].z = max(partialMax[tx].z, partialMax[tx + stride].z);
            }
        }
        __syncthreads();

        //TODO: 
        // This algorithm reduces to an output size = blockDim.x for each config
        // Dynamically launch another kernel until only one block is launched?
        // Or reduce the output of this on the CPU. But that will require movement of data
        // from the device to host and back again for the broad phase.

        // The below code is a temporary solution which works only if one block is launched in the x dimension.

        if(tx == 0)
        {
            bot_bounds_local.x_min = partialMin[tx].x;
            bot_bounds_local.y_min = partialMin[tx].y;
            bot_bounds_local.z_min = partialMin[tx].z;
            bot_bounds_local.x_max = partialMax[tx].x;
            bot_bounds_local.y_max = partialMax[tx].y;
            bot_bounds_local.z_max = partialMax[tx].z;
            bot_bounds[ty] = bot_bounds_local;
        }
    }
}

// generateAABB- Generate AABBs for all configurations parallelly
void generateAABB(Vector3f* vertices, unsigned int numVertices, 
                    unsigned int num_configs, AABB* bot_bounds) 
{
    // dim3 dimGrid(ceil((float)(num_configs) / AABB_BLOCK_SIZE), 1, 1);
    // dim3 dimBlock(AABB_BLOCK_SIZE, 1, 1);
    // generateAABBPrimitiveKernel<<<dimGrid, dimBlock>>>(vertices, numVertices, num_configs, bot_bounds);
    dim3 dimGrid(ceil((float)(numVertices) / AABB_BLOCK_SIZE_X*2), ceil((float)(num_configs) / AABB_BLOCK_SIZE_Y), 1);
    dim3 dimBlock(AABB_BLOCK_SIZE_X, AABB_BLOCK_SIZE_Y, 1);
    generateAABBKernel<<<dimGrid, dimBlock>>>(vertices, numVertices, num_configs, bot_bounds);
}

// generateAABBBaseline- Generate AABBs for all configurations serially
void generateAABBBaseline(Vector3f* vertices, unsigned int numVertices, 
                    unsigned int num_configs, AABB* bot_bounds) 
{
    // Loop over every configuration
    for(int i = 0; i < num_configs; ++i)
    {
        // Loop over every vertex in each configuration
        unsigned int config_ofset = i * numVertices;
        bot_bounds[i].x_min = vertices[config_ofset].x;
        bot_bounds[i].y_min = vertices[config_ofset].y;
        bot_bounds[i].z_min = vertices[config_ofset].z;
        bot_bounds[i].x_max = vertices[config_ofset].x;
        bot_bounds[i].y_max = vertices[config_ofset].y;
        bot_bounds[i].z_max = vertices[config_ofset].z;
        for(int j = 0; j < numVertices; ++j)
        {
            bot_bounds[i].x_min = min(bot_bounds[i].x_min, vertices[j].x);
            bot_bounds[i].y_min = min(bot_bounds[i].y_min, vertices[j].y);
            bot_bounds[i].z_min = min(bot_bounds[i].z_min, vertices[j].z);
            bot_bounds[i].x_max = max(bot_bounds[i].x_max, vertices[j].x);
            bot_bounds[i].y_max = max(bot_bounds[i].y_max, vertices[j].y);
            bot_bounds[i].z_max = max(bot_bounds[i].z_max, vertices[j].z);
        }
    }
}