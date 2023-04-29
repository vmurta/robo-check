#include "transform.hu"
// TODO: Make this part run in parallel
// IDEA: could have each block do blockDim.x number of transformations
// Have each thread precompute the transformationMatrix at start of block
// Then, the block loops through the list of transformation matrix, computing each
// in parallel
__device__ Matrix4f createTransformationMatrix(Configuration config) {
    float x = config.x;
    float y = config.y;
    float z = config.z;
    float pitch = config.pitch;
    float yaw = config.yaw;
    float roll= config.roll;

    float cosB = cos(pitch);
    float sinB = sin(pitch);
    float cosA = cos(yaw);
    float sinA = sin(yaw);
    float cosC = cos(roll);
    float sinC = sin(roll);

    Matrix4f transform;
    transform.m[0][0] = cosA * cosB;
    transform.m[0][1] = cosA * sinB * sinC - sinA * cosC;
    transform.m[0][2] = cosA * sinB * cosC + sinA * sinC;
    transform.m[0][3] = x;
    transform.m[1][0] = sinA * cosB;
    transform.m[1][1] = sinA * sinB * sinC + cosA * cosC;
    transform.m[1][2] = sinA * sinB * cosC - cosA * sinC;
    transform.m[1][3] = y;
    transform.m[2][0] = -sinB;
    transform.m[2][1] = cosB * sinC;
    transform.m[2][2] = cosB * cosC;
    transform.m[2][3] = z;
    transform.m[3][0] = 0;
    transform.m[3][1] = 0;
    transform.m[3][2] = 0;
    transform.m[3][3] = 1;

    return transform;
}

__device__ Vector3f transformVector(Vector3f v, Matrix4f M) {
    // Create a 4D homogeneous vector from the 3D vector
    float v_h[4] = {v.x, v.y, v.z, 1};

    // Compute the transformed 4D vector by matrix multiplication
    float v_h_prime[4] = {0};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            v_h_prime[i] += M.m[i][j] * v_h[j];
        }
    }

    // Convert the transformed 4D vector back to a 3D vector
    Vector3f v_prime = {
        v_h_prime[0] / v_h_prime[3],
        v_h_prime[1] / v_h_prime[3],
        v_h_prime[2] / v_h_prime[3]
    };

    return v_prime;
}

__global__ void genTransformedCopies(Configuration* confs,  Vector3f *base_robot_vertices,
                                     Vector3f* transformed_robot_vertices, int num_confs,
                                    int num_robot_vertices){
    size_t conf_ind = blockIdx.x * blockDim.x + threadIdx.x;

    __shared__ Matrix4f transforms[TRANSFORM_SIZE];
    if (conf_ind < num_confs){
      transforms[threadIdx.x] = createTransformationMatrix(confs[conf_ind]);
    }
    __syncthreads();

    size_t transformed_vertices_offset = num_robot_vertices * TRANSFORM_SIZE * blockIdx.x + num_robot_vertices*threadIdx.x;

    // TODO: do this in shared memory, tile and flush
    for(int rob_ind = 0; rob_ind < num_robot_vertices; rob_ind++){
      transformed_robot_vertices[rob_ind + transformed_vertices_offset] =
        transformVector(base_robot_vertices[rob_ind], transforms[threadIdx.x]);
    }
    // // do TRANSFORM_SIZE number of configurations, unless that would put us out of bounds
    // size_t num_confs_to_do = blockIdx.x * blockDim.x + TRANSFORM_SIZE < num_confs ? TRANSFORM_SIZE
                              // : num_confs -  blockIdx.x * blockDim.x;
    // //for each configuration, write a transformed copy of the robot into global memory
    // for (int i = 0; i < num_confs_to_do; ++i){
    //   //each thread computes a portion of the current transformation and writes it to global
    //   // TODO: do this in shared memory, tile and flush
    //   for(int rob_ind = threadIdx.x; rob_ind < num_robot_vertices; rob_ind += blockDim.x){
    //     transformed_robot_vertices[rob_ind + transformed_vertices_offset] =
    //       transformVector(base_robot_vertices[rob_ind], transforms[i]);
    //   }
    //   // increment to the next transformation
    //   transformed_vertices_offset += num_robot_vertices;
    // }
}
