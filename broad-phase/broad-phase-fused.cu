#include "broad-phase-fused.hu"

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

__global__ void transformAndGenerateAABB(Configuration* confs,  Vector3f *base_robot_vertices,
                                     AABB* bot_bounds, int num_confs, int num_robot_vertices)
{
    size_t config_idx = blockIdx.x * blockDim.x + threadIdx.x;

    if(config_idx >= num_confs) return;

    Matrix4f transform_matrix = createTransformationMatrix(confs[config_idx]);

    AABB bot_bounds_local;
    bot_bounds_local.x_min = base_robot_vertices[0].x;
    bot_bounds_local.y_min = base_robot_vertices[0].y;
    bot_bounds_local.z_min = base_robot_vertices[0].z;
    bot_bounds_local.x_max = base_robot_vertices[0].x;
    bot_bounds_local.y_max = base_robot_vertices[0].y;
    bot_bounds_local.z_max = base_robot_vertices[0].z;

    Vector3f transformed_robot_vertex;
    for(int vertex_idx = 0; vertex_idx < num_robot_vertices; ++vertex_idx)
    {      
      transformed_robot_vertex = transformVector(base_robot_vertices[vertex_idx], transform_matrix);
      
      bot_bounds_local.x_min = min(bot_bounds_local.x_min, transformed_robot_vertex.x);
      bot_bounds_local.y_min = min(bot_bounds_local.y_min, transformed_robot_vertex.y);
      bot_bounds_local.z_min = min(bot_bounds_local.z_min, transformed_robot_vertex.z);
      bot_bounds_local.x_max = max(bot_bounds_local.x_max, transformed_robot_vertex.x);
      bot_bounds_local.y_max = max(bot_bounds_local.y_max, transformed_robot_vertex.y);
      bot_bounds_local.z_max = max(bot_bounds_local.z_max, transformed_robot_vertex.z);
    } 
    bot_bounds[config_idx] = bot_bounds_local;
}