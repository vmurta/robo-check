#include "generate-AABB.hu"

#define AABB_BLOCK_SIZE 32
#define AABB_BLOCK_SIZE_X 792
#define AABB_BLOCK_SIZE_Y 32

// generateAABBPrimitiveKernel - Basic, unoptimized parallel kernel to generate AABBs
//      - Single dimension block.
//      - Parallellizes over configurations i.e. each thread handles one configuration.
//      - Each thread loops over all vertices to calculate the AABB.
__global__ void generateAABBPrimitiveKernel(Vector3f* vertices, unsigned int numVertices, 
                    unsigned int numConfigs, AABB* botBounds) 
{
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if(tid < numConfigs)
    {
        unsigned int configOffset = tid * numVertices;
        botBounds[tid].x_min = vertices[configOffset].x;
        botBounds[tid].y_min = vertices[configOffset].y;
        botBounds[tid].z_min = vertices[configOffset].z;
        botBounds[tid].x_max = vertices[configOffset].x;
        botBounds[tid].y_max = vertices[configOffset].y;
        botBounds[tid].z_max = vertices[configOffset].z;
        for(int j = 0; j < numVertices; ++j)
        {
            botBounds[tid].x_min = min(botBounds[tid].x_min, vertices[configOffset + j].x);
            botBounds[tid].y_min = min(botBounds[tid].y_min, vertices[configOffset + j].y);
            botBounds[tid].z_min = min(botBounds[tid].z_min, vertices[configOffset + j].z);
            botBounds[tid].x_max = max(botBounds[tid].x_max, vertices[configOffset + j].x);
            botBounds[tid].y_max = max(botBounds[tid].y_max, vertices[configOffset + j].y);
            botBounds[tid].z_max = max(botBounds[tid].z_max, vertices[configOffset + j].z);
        }
    }
}

// generateAABBKernel - Optimized parallel kernel to generate AABBs
//      - Two dimension block - each yDim corresponds to one config, each xDim corresponds to vertices.
//      - Uses reduction along xDim to calculate AABBs for each configuration.
__global__ void generateAABBKernel(Vector3f* vertices, unsigned int numVertices, 
                    unsigned int numConfigs, AABB* botBounds) 
{    
    // __shared__ Vector3f partialMin[2 * AABB_BLOCK_SIZE_X];
    // __shared__ Vector3f partialMax[2 * AABB_BLOCK_SIZE_X];
    extern __shared__ Vector3f sharedMem[];
    Vector3f* partialMin = &sharedMem[0];
    Vector3f* partialMax = &sharedMem[2 * AABB_BLOCK_SIZE_X];
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
            botBoundsLocal.x_min = partialMin[tx].x;
            botBoundsLocal.y_min = partialMin[tx].y;
            botBoundsLocal.z_min = partialMin[tx].z;
            botBoundsLocal.x_max = partialMax[tx].x;
            botBoundsLocal.y_max = partialMax[tx].y;
            botBoundsLocal.z_max = partialMax[tx].z;
            botBounds[ty] = botBoundsLocal;
        }
    }
}

// generateAABB- Generate AABBs for all configurations parallelly
void generateAABB(Vector3f* vertices, unsigned int numVertices, 
                    unsigned int numConfigs, AABB* botBounds) 
{
    int device_count;
    if (cudaGetDeviceCount(&device_count) != 0) {
        printf("CUDA not loaded properly\n");
    } else {
        printf("CUDA loaded for %d device(s)\n", device_count);
    }

    Vector3f* d_vertices;
    cudaMalloc(&d_vertices, numConfigs * numVertices * sizeof(Vector3f));
    cudaMemcpy(d_vertices, vertices, numConfigs * numVertices * sizeof(Vector3f), cudaMemcpyHostToDevice);

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
void generateAABBBaseline(Vector3f* vertices, unsigned int numVertices, 
                    unsigned int numConfigs, AABB* botBounds) 
{
    // Loop over every configuration
    for(int i = 0; i < numConfigs; ++i)
    {
        // Loop over every vertex in each configuration
        unsigned int configOffset = i * numVertices;
        botBounds[i].x_min = vertices[configOffset].x;
        botBounds[i].y_min = vertices[configOffset].y;
        botBounds[i].z_min = vertices[configOffset].z;
        botBounds[i].x_max = vertices[configOffset].x;
        botBounds[i].y_max = vertices[configOffset].y;
        botBounds[i].z_max = vertices[configOffset].z;
        for(int j = 0; j < numVertices; ++j)
        {
            botBounds[i].x_min = min(botBounds[i].x_min, vertices[configOffset + j].x);
            botBounds[i].y_min = min(botBounds[i].y_min, vertices[configOffset + j].y);
            botBounds[i].z_min = min(botBounds[i].z_min, vertices[configOffset + j].z);
            botBounds[i].x_max = max(botBounds[i].x_max, vertices[configOffset + j].x);
            botBounds[i].y_max = max(botBounds[i].y_max, vertices[configOffset + j].y);
            botBounds[i].z_max = max(botBounds[i].z_max, vertices[configOffset + j].z);
        }
    }
}

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