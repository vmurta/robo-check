#include "broad-phase.hu"
#include "../generate-AABB/generate-AABB.hu"

#define BROAD_PHASE_TILE_SIZE 256

// Check if two objects are colliding along a certain dimension
inline __host__ __device__ bool dimensionCollides(float fstMin, float fstMax, float sndMin, float sndMax) {
    // Done without any control divergence!
    return !(sndMin > fstMax || sndMax < fstMin);
}

__global__ void broadPhaseKernel(int num_confs, const AABB *robots, const AABB *obstacle, bool *valid_conf) {  
    int numConfsPerThread = ceil((num_confs * 1.0) / (gridDim.x * blockDim.x));
    int threadNo = blockIdx.x * blockDim.x + threadIdx.x;

    // Due to the massive reuse it's fastest to store the obstacle AABB in registers
    AABB obstacleReg = *obstacle;

    for (int i = threadNo * numConfsPerThread; i < min(num_confs, (threadNo + 1) * numConfsPerThread); i++) {
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
}