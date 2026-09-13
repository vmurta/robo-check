/*
File: /home/victor/Projects/robo-check/performance_testing/abs_test.cu

Generates 1,000,000 random floats, stores tiles in shared memory,
takes absolute value in shared memory, and writes results back to global memory.
Uses a simple xorshift RNG per thread. Adjust BLOCK_SIZE or ITEMS_PER_THREAD
if you need different shared-memory footprint.
*/

#include <cstdio>
#include <cstdint>
#include <cuda_runtime.h>
#include <chrono>
#include <iostream>

#define ITEMS_PER_THREAD 8
#define BLOCK_SIZE 256
#define N_TOTAL 1000000

#define CUDA_CHECK(call)                                                   \
    do {                                                                   \
        cudaError_t err = call;                                            \
        if (err != cudaSuccess) {                                          \
            fprintf(stderr, "CUDA error %s:%d: %s\n", __FILE__, __LINE__,  \
                    cudaGetErrorString(err));                              \
            exit(1);                                                       \
        }                                                                  \
    } while (0)

// simple xorshift32 RNG
__device__ inline unsigned int xorshift32(unsigned int &state) {
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    return state;
}

__global__ void gen_abs_kernel(float* data, int N, unsigned int seed) {
    extern __shared__ float sdata[]; // dynamic shared memory: tile_size floats

    const int tid = threadIdx.x;
    const int lane_stride = blockDim.x; // distance between consecutive items of same thread in tile
    const int items_per_block = blockDim.x * ITEMS_PER_THREAD;
    // base index of this block's tile in global array
    int tile_start = blockIdx.x * items_per_block;

    // iterate tiles with grid-stride across blocks
    while (tile_start < N) {
        // per-thread local RNG state seeded with tile_start and thread id for variability
        unsigned int state = seed ^ (unsigned int)(tile_start + tid) ^ 0x9e3779b9u;

        __syncthreads();

        // take absolute value in shared memory (in-place)
        #pragma unroll
        int local_index = tid;
        int global_index = tile_start;
        for (int i = 0; i < ITEMS_PER_THREAD; ++i) {
            global_index += lane_stride;
            unsigned int rnd = xorshift32(state);
            // convert to float in [-1.0, 1.0)
            float f = (float)rnd / (float)UINT32_MAX;
            float val = f * 2.0f - 1.0f;
            // bounds check not necessary for shared memory but necessary when writing back
            data[global_index] = fabsf(val);
        }

        __syncthreads();

        // advance to next tile handled by this block in a grid-stride manner
        tile_start += gridDim.x * items_per_block;
    }
}

__global__ void gen_abs_ternary_kernel(float* data, int N, unsigned int seed) {
    extern __shared__ float sdata[]; // dynamic shared memory: tile_size floats

    const int tid = threadIdx.x;
    const int lane_stride = blockDim.x; // distance between consecutive items of same thread in tile
    const int items_per_block = blockDim.x * ITEMS_PER_THREAD;
    // base index of this block's tile in global array
    int tile_start = blockIdx.x * items_per_block;

    // iterate tiles with grid-stride across blocks
    while (tile_start < N) {
// iterate tiles with grid-stride across blocks
    while (tile_start < N) {
        // per-thread local RNG state seeded with tile_start and thread id for variability
        unsigned int state = seed ^ (unsigned int)(tile_start + tid) ^ 0x9e3779b9u;

        __syncthreads();

        // take absolute value in shared memory (in-place)
        #pragma unroll
        int local_index = tid;
        int global_index = tile_start;
        for (int i = 0; i < ITEMS_PER_THREAD; ++i) {
            global_index += lane_stride;
            unsigned int rnd = xorshift32(state);
            // convert to float in [-1.0, 1.0)
            float f = (float)rnd / (float)UINT32_MAX;
            float val = f * 2.0f - 1.0f;
            // bounds check not necessary for shared memory but necessary when writing back
            data[global_index] = (val < 0.0f) ? -val : val;
        }

        __syncthreads();

        // advance to next tile handled by this block in a grid-stride manner
        tile_start += gridDim.x * items_per_block;
    }
    }
}

float time_kernel(bool use_ternary) {
    const int N = N_TOTAL;
    const size_t bytes = N * sizeof(float);

    float* d_data = nullptr;
    CUDA_CHECK(cudaMalloc(&d_data, bytes));

    // Configure kernel
    const int blockSize = BLOCK_SIZE;
    const int itemsPerBlock = blockSize * ITEMS_PER_THREAD;
    int gridSize = (N + itemsPerBlock - 1) / itemsPerBlock;
    if (gridSize == 0) gridSize = 1;

    // limit grid to a reasonable number (optional)
    const int maxGrid = 65535;
    if (gridSize > maxGrid) gridSize = maxGrid;

    size_t shmem_bytes = itemsPerBlock * sizeof(float);

    unsigned int seed = 12345u;


    auto start = std::chrono::high_resolution_clock::now();
    if (use_ternary) {
        gen_abs_ternary_kernel<<<gridSize, blockSize, shmem_bytes>>>(d_data, N, seed);
    } else {
        gen_abs_kernel<<<gridSize, blockSize, shmem_bytes>>>(d_data, N, seed);
    }
    CUDA_CHECK(cudaGetLastError());
    CUDA_CHECK(cudaDeviceSynchronize());
    auto end = std::chrono::high_resolution_clock::now();
    float duration = std::chrono::duration<float, std::milli>(end - start).count();

    // copy back a few values to verify
    float* h = (float*)malloc(bytes);
    CUDA_CHECK(cudaMemcpy(h, d_data, bytes, cudaMemcpyDeviceToHost));

    // printf("First 16 values (absolute random floats):\n");
    // for (int i = 0; i < 16 && i < N; ++i) {
    //     printf("%d: %f\n", i, h[i]);
    // }

    free(h);
    CUDA_CHECK(cudaFree(d_data));
    return duration;
}

int main() {
    size_t num_iters = 10000;

    float simple_times[num_iters];
    printf("Timing gen_abs_kernel over %zu iterations...\n", num_iters);
    for (size_t i = 0; i < num_iters; ++i) {
        simple_times[i] = time_kernel(false);
    }

    float ternary_times[num_iters];
    printf("Timing gen_abs_ternary_kernel over %zu iterations...\n", num_iters);
    for (size_t i = 0; i < num_iters; ++i) {
        std::cout << "\rIteration " << i + 1 << " / " << num_iters;
        ternary_times[i] = time_kernel(true);
        std::cout.flush();
    }


    // compute average times
    float ternary_avg = 0.0f;
    float simple_avg = 0.0f;
    for (size_t i = 0; i < num_iters; ++i) {
        ternary_avg += ternary_times[i];
        simple_avg += simple_times[i];
    }
    std::cout << "Total times for ternary kernel: " << ternary_avg << " ms\n";
    std::cout << "Total times for simple kernel: " << simple_avg << " ms\n";
    std::cout << "Average time per iteration for ternary kernel: " << (ternary_avg / num_iters) << " ms\n";
    std::cout << "Average time per iteration for simple kernel: " << (simple_avg / num_iters) << " ms\n";

}