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

#define MAX_NUM_ROBOT_VERTICES 1000
__constant__ Vector3f base_robot_vertices[MAX_NUM_ROBOT_VERTICES];

__global__ void transformAndGenerateAABBKernel(Configuration* confs,
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

void transformAndGenerateAABB(std::vector<Configuration> &confs, AABB* bot_bounds, bool *valid_conf)
{
    int device_count;
    if (cudaGetDeviceCount(&device_count) != 0) std::cout << "CUDA not loaded properly" << std::endl;

    //Load Robot
    std::vector<Vector3f> rob_vertices;
    std::vector<Triangle> rob_triangles;
    loadOBJFile(ROB_FILE, rob_vertices, rob_triangles);
    std::cout << "Robot has " << rob_vertices.size() << " vertices " <<std::endl;

    //Load Obstacles
    std::vector<Vector3f> obs_vertices;
    std::vector<Triangle> obs_triangles;
    loadOBJFile(OBS_FILE, obs_vertices, obs_triangles);
    std::cout << "Obstacle has " << obs_vertices.size() << " vertices " <<std::endl;

    //Load robot vertices to constant memory
    checkCudaMem(cudaMemcpyToSymbol(base_robot_vertices, rob_vertices.data(), rob_vertices.size() * sizeof(Vector3f)));
    std::cout << "Copied the robot vertices " << std::endl;

    Configuration *d_confs;
    checkCudaCall(cudaMalloc(&d_confs, confs.size() * sizeof(Configuration)));
    checkCudaMem(cudaMemcpy(d_confs, confs.data(), confs.size() * sizeof(Configuration), cudaMemcpyHostToDevice));
    std::cout << "Copied the configurations " << std::endl;

    AABB* d_bot_bounds;
    checkCudaCall(cudaMalloc(&d_bot_bounds, confs.size() * sizeof(AABB)));
    std::cout << "Malloced the AABBs " << std::endl;

    dim3 dimGridTransformKernel(ceil((float)(confs.size()) / TRANSFORM_BLOCK_SIZE), 1, 1);
    dim3 dimBlockTransformKernel(TRANSFORM_BLOCK_SIZE, 1, 1);
    transformAndGenerateAABBKernel<<<dimGridTransformKernel, dimBlockTransformKernel>>>(d_confs, d_bot_bounds, 
                                                    confs.size(), rob_vertices.size());
    checkCudaCall(cudaDeviceSynchronize());

    // Move obstacle to AABB (on CPU since we only have 1)
    AABB *obstacle_bbox = new AABB();
    generateAABBBaseline(obs_vertices.data(), obs_vertices.size(), 1, obstacle_bbox);

    bool *valid_conf_d;
    AABB *obstacle_bbox_d;
    checkCudaCall(cudaMalloc(&valid_conf_d, confs.size() * sizeof(bool)));
    checkCudaCall(cudaMalloc(&obstacle_bbox_d, sizeof(AABB)));
    checkCudaCall(cudaMemcpy(obstacle_bbox_d, obstacle_bbox, sizeof(AABB), cudaMemcpyHostToDevice));

    broadPhase(confs.size(), d_bot_bounds, obstacle_bbox_d, valid_conf_d);

    std::cout << "Completed kernel execution" << std::endl;
    
    checkCudaCall(cudaDeviceSynchronize());
    std::cout << "Synchronized" << std::endl;

    std:: cout << "Copying back results" << std::endl;
    cudaMemcpy(bot_bounds, d_bot_bounds, confs.size() * sizeof(AABB), cudaMemcpyDeviceToHost);
    cudaMemcpy(valid_conf, valid_conf_d, confs.size() * sizeof(bool), cudaMemcpyDeviceToHost);

    checkCudaCall(cudaFree(d_confs));
    checkCudaCall(cudaFree(d_bot_bounds));
    checkCudaCall(cudaFree(obstacle_bbox_d));
    checkCudaCall(cudaFree(valid_conf_d));
    std::cout << "Copied back memory and synchronized" << std::endl;
}