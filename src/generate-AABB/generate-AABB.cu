#include "generate-AABB.hu"

// generateAABBPrimitiveKernel - Basic, unoptimized parallel kernel to generate AABBs
//      - Single dimension block.
//      - Parallellizes over configurations i.e. each thread handles one configuration.
//      - Each thread loops over all vertices to calculate the AABB.
__global__ void generateAABBPrimitiveKernel(Eigen::Vector3f* vertices, unsigned int numVertices, 
                    unsigned int numConfigs, AABB* botBounds) 
{
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    AABB botBoundsLocal;
    Eigen::Vector3f vertex;

    if(tid < numConfigs)
    {
        unsigned int configOffset = tid * numVertices;
        botBoundsLocal.x_min = vertices[configOffset](0);
        botBoundsLocal.y_min = vertices[configOffset](1);
        botBoundsLocal.z_min = vertices[configOffset](2);
        // could just copy the x_min, y_min, z_min to x_max, y_max, z_max 
        // to avoid extra global memory access
        botBoundsLocal.x_max = vertices[configOffset](0);
        botBoundsLocal.y_max = vertices[configOffset](1);
        botBoundsLocal.z_max = vertices[configOffset](2);
        for(int j = 0; j < numVertices; ++j)
        {
            vertex = vertices[configOffset + j];
            botBoundsLocal.x_min = min(botBoundsLocal.x_min, vertex(0));
            botBoundsLocal.y_min = min(botBoundsLocal.y_min, vertex(1));
            botBoundsLocal.z_min = min(botBoundsLocal.z_min, vertex(2));
            botBoundsLocal.x_max = max(botBoundsLocal.x_max, vertex(0));
            botBoundsLocal.y_max = max(botBoundsLocal.y_max, vertex(1));
            botBoundsLocal.z_max = max(botBoundsLocal.z_max, vertex(2));
        }
        botBounds[tid] = botBoundsLocal;
    }
}

// generateAABBKernel - Optimized parallel kernel to generate AABBs
//      - Two dimension block - each yDim corresponds to one config, each xDim corresponds to vertices.
//      - Uses reduction along xDim to calculate AABBs for each configuration.
//TODO: Use struct of arrays instead of array of structs to improve memory access patterns.
__global__ void generateAABBKernel(Eigen::Vector3f* vertices, unsigned int numVertices, 
                    unsigned int numConfigs, AABB* botBounds) 
{    
    // __shared__ Eigen::Vector3f partialMin[2 * AABB_BLOCK_SIZE_X];
    // __shared__ Eigen::Vector3f partialMax[2 * AABB_BLOCK_SIZE_X];
    extern __shared__ Eigen::Vector3f sharedMem[];
    Eigen::Vector3f* partialMin = &sharedMem[0];
    Eigen::Vector3f* partialMax = &sharedMem[2 * AABB_BLOCK_SIZE_X];
    AABB botBoundsLocal;

    unsigned int ty = blockIdx.y * blockDim.y + threadIdx.y;
    unsigned int tx = threadIdx.x;
    unsigned int inputStartIdx = 2*blockIdx.x*blockDim.x*ty;
    
    unsigned int totalNumVertices = numVertices * numConfigs;

    if(ty < numConfigs)
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
                partialMin[tx + blockDim.x] = Eigen::Vector3f(0,0,0);
                partialMax[tx + blockDim.x] = Eigen::Vector3f(0,0,0);
            }
        }
        else
        {
            partialMin[tx] = Eigen::Vector3f(0,0,0);
            partialMin[tx + blockDim.x] = Eigen::Vector3f(0,0,0);
            partialMax[tx] = Eigen::Vector3f(0,0,0);
            partialMax[tx + blockDim.x] = Eigen::Vector3f(0,0,0);
        }
        for(unsigned int stride = blockDim.x; stride >= 1; stride /= 2)
        {
            __syncthreads();
            if(tx < stride)
            {
                partialMin[tx](0) = min(partialMin[tx](0), partialMin[tx + stride](0));
                partialMin[tx](1) = min(partialMin[tx](1), partialMin[tx + stride](1));
                partialMin[tx](2) = min(partialMin[tx](2), partialMin[tx + stride](2));
                partialMax[tx](0) = max(partialMax[tx](0), partialMax[tx + stride](0));
                partialMax[tx](1) = max(partialMax[tx](1), partialMax[tx + stride](1));
                partialMax[tx](2) = max(partialMax[tx](2), partialMax[tx + stride](2));
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
            botBoundsLocal.x_min = partialMin[tx](0);
            botBoundsLocal.y_min = partialMin[tx](1);
            botBoundsLocal.z_min = partialMin[tx](2);
            botBoundsLocal.x_max = partialMax[tx](0);
            botBoundsLocal.y_max = partialMax[tx](1);
            botBoundsLocal.z_max = partialMax[tx](2);
            botBounds[ty] = botBoundsLocal;
        }
    }
}

// generateAABB- Generate AABBs for all configurations parallelly
void generateAABB(Eigen::Vector3f* vertices, unsigned int numVertices, 
                    unsigned int numConfigs, AABB* botBounds) 
{
    int device_count;
    if (cudaGetDeviceCount(&device_count) != 0) {
        printf("CUDA not loaded properly\n");
    } else {
        printf("CUDA loaded for %d device(s)\n", device_count);
    }

    Eigen::Vector3f* d_vertices;
    cudaMalloc(&d_vertices, numConfigs * numVertices * sizeof(Eigen::Vector3f));
    cudaMemcpy(d_vertices, vertices, numConfigs * numVertices * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice);

    AABB* d_bot_bounds;
    cudaMalloc(&d_bot_bounds, numConfigs * sizeof(AABB));

    dim3 dimGrid(ceil((float)(numConfigs) / AABB_BLOCK_SIZE), 1, 1);
    dim3 dimBlock(AABB_BLOCK_SIZE, 1, 1);
    generateAABBPrimitiveKernel<<<dimGrid, dimBlock>>>(d_vertices, numVertices, numConfigs, d_bot_bounds);
    // dim3 dimGrid(ceil((float)(numVertices) / AABB_BLOCK_SIZE_X*2), ceil((float)(numConfigs) / AABB_BLOCK_SIZE_Y), 1);
    // dim3 dimBlock(AABB_BLOCK_SIZE_X, AABB_BLOCK_SIZE_Y, 1);
    // generateAABBKernel<<<dimGrid, dimBlock, 4 * AABB_BLOCK_SIZE_X>>>(d_vertices, numVertices, numConfigs, d_bot_bounds);

    cudaDeviceSynchronize();

    cudaError_t err = cudaGetLastError();
    printf("Status: %s: %s\n", cudaGetErrorName(err), cudaGetErrorString(err));

    // Copy the data back
    cudaMemcpy(botBounds, d_bot_bounds, numConfigs * sizeof(AABB), cudaMemcpyDeviceToHost);

    // Free the memory
    cudaFree(d_bot_bounds);
    cudaFree(d_vertices);
}

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