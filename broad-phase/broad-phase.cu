#include "broad-phase.hu"
#include "../generate-AABB/generate-AABB.hu"

#define BROAD_PHASE_TILE_SIZE 256

// Check if two objects are colliding along a certain dimension
inline __host__ __device__ bool dimensionCollides(float fstMin, float fstMax, float sndMin, float sndMax) {
    // Done without any control divergence!
    return sndMin > fstMax || sndMax < fstMin;
}

__global__ void broadPhaseKernel(int num_confs, const AABB *robots, const AABB *obstacle, bool *valid_conf) {  
    int numConfsPerThread = ceil((num_confs * 1.0) / (gridDim.x * blockDim.x));
    int threadNo = blockIdx.x * blockDim.x + threadIdx.x;

    // FIXME - Build a shared memory version
    __shared__ AABB robotTile[sizeof(AABB) * BROAD_PHASE_TILE_SIZE];

    // Due to the massive reuse it's fastest to store the obstacle AABB in registers
    AABB obstacleReg = *obstacle;

    // FIXME - Shared memory loading and usage (will need to coalesce as well)
    for (int i = threadNo * numConfsPerThread; i < min(num_confs, (threadNo + 1) * numConfsPerThread); i++) {
        // FIXME - Switch to direct tile loading
        AABB current = robots[i];
        
        // We can avoid ANY control divergence here!
        bool isNotValid = 
            dimensionCollides(obstacleReg.x_min, obstacleReg.x_max, current.x_min, current.x_max) &&
            dimensionCollides(obstacleReg.y_min, obstacleReg.y_max, current.y_min, current.y_max) &&
            dimensionCollides(obstacleReg.z_min, obstacleReg.z_max, current.z_min, current.z_max);
        valid_conf[i] = !isNotValid;
    }

}

void broadPhase(int num_confs, const AABB *robots, const AABB *obstacle, bool *valid_conf) {
    int gridDim = 100;
    int blockDim = BROAD_PHASE_TILE_SIZE;
    broadPhaseKernel<<<gridDim, blockDim>>>(num_confs, robots, obstacle, valid_conf);
    cudaDeviceSynchronize();
}

/*void broadPhaseBaseline(int num_confs, const AABB *robots, const AABB *obstacle, bool *valid_conf) {
    for (int i = 0; i < num_confs; i++) {
        AABB current = robots[i];
        
        // We can avoid ANY control divergence here!
        bool isNotValid =
            dimensionCollides(obstacle.x_min, obstacle.x_max, current.x_min, current.x_max) &&
            dimensionCollides(obstacle.y_min, obstacle.y_max, current.y_min, current.y_max) &&
            dimensionCollides(obstacle.z_min, obstacle.z_max, current.z_min, current.z_max);
        valid_conf[i] = !isNotValid;
    }
}*/

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

#define AABB_BLOCK_SIZE 32
#define AABB_BLOCK_SIZE_X 792
#define AABB_BLOCK_SIZE_Y 32

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

void test_broadPhase(AABB* botBounds, const int numConfigs)
{
    const int numVertices = 6;

    // Test a robot collides with itself
    Vector3f robPts[numVertices * numConfigs];
    bool validConf[numConfigs];

    generateTestVertices(robPts);

    generateAABB(robPts, numVertices, numConfigs, botBounds);

    // Test robot collides with itself
    broadPhase(numConfigs, botBounds, botBounds, validConf);

    size_t numValid = 0;
    for (int i = 0; i < numConfigs; i++) {
        std::cout << i << ": " << (validConf[i] == true) << std::endl;
    }
}

int main()
{
    std::cout << "====Borad Phase tests====" << std::endl;
    const int numConfigs = 2;
    std::cout << "Running broad phase parallel kernel test..." << std::endl;
    AABB botBoundsParallel[numConfigs];
    test_broadPhase(botBoundsParallel, numConfigs);

    std::cout << "==================" << std::endl;
    return 0;
}