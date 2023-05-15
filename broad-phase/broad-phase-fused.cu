#include "broad-phase-fused.hu"
#include "../narrow-phase/narrow-phase.hu"
#define TRANSFORM_BLOCK_SIZE 32

extern __constant__ Vector3f base_robot_vertices[NUM_ROB_VERTICES];
extern __constant__ Triangle base_robot_triangles[MAX_NUM_ROBOT_TRIANGLES];
extern __constant__ Vector3f base_obs_vertices[NUM_ROB_VERTICES];
extern __constant__ Triangle base_obs_triangles[MAX_NUM_ROBOT_TRIANGLES];

__device__ Matrix4f createTransformationMatrix(const Configuration config) {


    float cosB = cos(config.pitch);
    float sinB = sin(config.pitch);
    float cosA = cos(config.yaw);
    float sinA = sin(config.yaw);
    float cosC = cos(config.roll);
    float sinC = sin(config.roll);

    Matrix4f transform;
    transform.m[0][0] = cosA * cosB;
    transform.m[0][1] = cosA * sinB * sinC - sinA * cosC;
    transform.m[0][2] = cosA *  sinB * cosC + sinA * sinC;
    transform.m[0][3] = config.x;
    transform.m[1][0] = sinA * cosB;
    transform.m[1][1] = sinA * sinB * sinC + cosA * cosC;
    transform.m[1][2] = sinA * sinB * cosC - cosA * sinC;
    transform.m[1][3] = config.y;
    transform.m[2][0] = -sinB;
    transform.m[2][1] = cosB * sinC;
    transform.m[2][2] = cosB * cosC;
    transform.m[2][3] = config.z;
    transform.m[3][0] = 0;
    transform.m[3][1] = 0;
    transform.m[3][2] = 0;
    transform.m[3][3] = 1;

    return transform;
}

__device__ Matrix3f createRotationMatrix(const Configuration config) {


    float cosB = cos(config.pitch);
    float sinB = sin(config.pitch);
    float cosA = cos(config.yaw);
    float sinA = sin(config.yaw);
    float cosC = cos(config.roll);
    float sinC = sin(config.roll);

    Matrix3f rotate;
    rotate.m[0][0] = cosA * cosB;
    rotate.m[0][1] = cosA * sinB * sinC - sinA * cosC;
    rotate.m[0][2] = cosA *  sinB * cosC + sinA * sinC;
    rotate.m[1][0] = sinA * cosB;
    rotate.m[1][1] = sinA * sinB * sinC + cosA * cosC;
    rotate.m[1][2] = sinA * sinB * cosC - cosA * sinC;
    rotate.m[2][0] = -sinB;
    rotate.m[2][1] = cosB * sinC;
    rotate.m[2][2] = cosB * cosC;

    return rotate;
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

    // printf("v_h_prime: %f %f %f %f\n", v_h_prime[0], v_h_prime[1], v_h_prime[2], v_h_prime[3]);
    // Convert the transformed 4D vector back to a 3D vector
    Vector3f v_prime = {
        v_h_prime[0],
        v_h_prime[1],
        v_h_prime[2]
    };

    return v_prime;
}

// Check if two objects are colliding along a certain dimension
inline __host__ __device__ bool dimensionCollides(float fstMin, float fstMax, float sndMin, float sndMax) {
    // Done without any control divergence!
    return fstMin <= sndMax && sndMin <= fstMax;
}

// __constant__ Triangle base_obs_triangles[2500];

__global__ void broadPhaseFusedKernel(Configuration *configs, const AABB *obstacle, Vector3f *transformed_robot_vertices,
                                     bool *valid_conf, const int num_configs, const int num_robot_vertices)
{
    size_t config_idx = blockIdx.x * blockDim.x + threadIdx.x;

    if(config_idx >= num_configs) return;

    Matrix4f transform_matrix = createTransformationMatrix(configs[config_idx]);

    AABB bot_bounds_local;
    bot_bounds_local.x_min = FLT_MAX;
    bot_bounds_local.y_min = FLT_MAX;
    bot_bounds_local.z_min = FLT_MAX;
    bot_bounds_local.x_max = -FLT_MAX;
    bot_bounds_local.y_max = -FLT_MAX;
    bot_bounds_local.z_max = -FLT_MAX;

    Vector3f transformed_robot_vertex;
    for(int vertex_idx = 0; vertex_idx < num_robot_vertices; ++vertex_idx)
    {
      transformed_robot_vertex = transformVector(base_robot_vertices[vertex_idx], transform_matrix);
      transformed_robot_vertices[config_idx * num_robot_vertices + vertex_idx] = transformed_robot_vertex;
      bot_bounds_local.x_min = min(bot_bounds_local.x_min, transformed_robot_vertex.x);
      bot_bounds_local.y_min = min(bot_bounds_local.y_min, transformed_robot_vertex.y);
      bot_bounds_local.z_min = min(bot_bounds_local.z_min, transformed_robot_vertex.z);
      bot_bounds_local.x_max = max(bot_bounds_local.x_max, transformed_robot_vertex.x);
      bot_bounds_local.y_max = max(bot_bounds_local.y_max, transformed_robot_vertex.y);
      bot_bounds_local.z_max = max(bot_bounds_local.z_max, transformed_robot_vertex.z);
    }
    // bot_bounds[config_idx] = bot_bounds_local;

    // Due to the massive reuse it's fastest to store the obstacle AABB in registers
    AABB obstacleReg = *obstacle;
    bool isNotValid =
            dimensionCollides(obstacleReg.x_min, obstacleReg.x_max, bot_bounds_local.x_min, bot_bounds_local.x_max) &&
            dimensionCollides(obstacleReg.y_min, obstacleReg.y_max, bot_bounds_local.y_min, bot_bounds_local.y_max) &&
            dimensionCollides(obstacleReg.z_min, obstacleReg.z_max, bot_bounds_local.z_min, bot_bounds_local.z_max);
    valid_conf[config_idx] = !isNotValid;
}

__global__ void broadPhaseFusedKernel_sep(Configuration *configs, const AABB *obstacle,
    float *rob_pts_x, float *rob_pts_y,
        float *rob_pts_z,
                                     bool *valid_conf, const int num_configs, const int num_robot_vertices)
{
    size_t config_idx = blockIdx.x * blockDim.x + threadIdx.x;

    if(config_idx >= num_configs) return;

    Matrix4f transform_matrix = createTransformationMatrix(configs[config_idx]);

    AABB bot_bounds_local;
    bot_bounds_local.x_min = FLT_MAX;
    bot_bounds_local.y_min = FLT_MAX;
    bot_bounds_local.z_min = FLT_MAX;
    bot_bounds_local.x_max = -FLT_MAX;
    bot_bounds_local.y_max = -FLT_MAX;
    bot_bounds_local.z_max = -FLT_MAX;

    Vector3f transformed_robot_vertex;
    for(int vertex_idx = 0; vertex_idx < num_robot_vertices; ++vertex_idx)
    {      
      transformed_robot_vertex = transformVector(base_robot_vertices[vertex_idx], transform_matrix);
      rob_pts_x[config_idx * num_robot_vertices + vertex_idx] = transformed_robot_vertex.x;
      rob_pts_y[config_idx * num_robot_vertices + vertex_idx] = transformed_robot_vertex.y;
      rob_pts_z[config_idx * num_robot_vertices + vertex_idx] = transformed_robot_vertex.z;
      
      bot_bounds_local.x_min = min(bot_bounds_local.x_min, transformed_robot_vertex.x);
      bot_bounds_local.y_min = min(bot_bounds_local.y_min, transformed_robot_vertex.y);
      bot_bounds_local.z_min = min(bot_bounds_local.z_min, transformed_robot_vertex.z);
      bot_bounds_local.x_max = max(bot_bounds_local.x_max, transformed_robot_vertex.x);
      bot_bounds_local.y_max = max(bot_bounds_local.y_max, transformed_robot_vertex.y);
      bot_bounds_local.z_max = max(bot_bounds_local.z_max, transformed_robot_vertex.z);
    } 
    // bot_bounds[config_idx] = bot_bounds_local;

    // Due to the massive reuse it's fastest to store the obstacle AABB in registers
    AABB obstacleReg = *obstacle;
    bool isNotValid = 
            dimensionCollides(obstacleReg.x_min, obstacleReg.x_max, bot_bounds_local.x_min, bot_bounds_local.x_max) &&
            dimensionCollides(obstacleReg.y_min, obstacleReg.y_max, bot_bounds_local.y_min, bot_bounds_local.y_max) &&
            dimensionCollides(obstacleReg.z_min, obstacleReg.z_max, bot_bounds_local.z_min, bot_bounds_local.z_max);
    valid_conf[config_idx] = !isNotValid;
}

void broadPhaseFused(std::vector<Configuration> &configs, bool *valid_conf)
{
    int device_count;
    if (cudaGetDeviceCount(&device_count) != 0) std::cout << "CUDA not loaded properly" << std::endl;

    //Load Robot
    std::vector<Vector3f> rob_vertices;
    std::vector<Triangle> rob_triangles;
    loadOBJFile(ROB_FILE, rob_vertices, rob_triangles);
    std::cout << "Robot has " << rob_vertices.size() << " vertices " <<std::endl;
    std::cout << "Robot has " << rob_triangles.size() << " triangles " <<std::endl;

    //Load Obstacles
    std::vector<Vector3f> obs_vertices;
    std::vector<Triangle> obs_triangles;
    loadOBJFile(OBS_FILE, obs_vertices, obs_triangles);
    std::cout << "Obstacle has " << obs_vertices.size() << " vertices " <<std::endl;
    std::cout << "Obstacle has " << obs_triangles.size() << " triangles " <<std::endl;

    // size_t count = 0;
    // for (const auto& triangle : obs_triangles) {
    //     if (count > 100){
    //         break;
    //     }
    //     std::cout << "v: " << triangle.v1 << ", v2: " << triangle.v2 << ", v3: " << triangle.v3 << std::endl;
    //     count++;
    // }
    // std::cout <base_robot_trianglesb_points;
    Vector3f *d_rob_transformed_points;
    Triangle *d_rob_triangles;
    Vector3f *d_rob_points;

    checkCudaCall(cudaMalloc(&d_rob_transformed_points, rob_vertices.size() * configs.size() * sizeof(Vector3f)));
    checkCudaCall(cudaMalloc(&d_rob_triangles, rob_triangles.size() * sizeof(Triangle)));
    // checkCudaMem(cudaMemcpy(d_rob_points, rob_vertices.data(), rob_vertices.size() * sizeof(Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_rob_triangles, rob_triangles.data(), rob_triangles.size() * sizeof(Triangle), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpyToSymbol(base_robot_vertices, rob_vertices.data(), rob_vertices.size() * sizeof(Vector3f)));
    // checkCudaMem(cudaMemcpyToSymbol(base_robot_triangles, rob_triangles.data(), rob_triangles.size() * sizeof(Triangle)));
    std::cout << "Copied the robot vertices and triangles " << std::endl;

    checkCudaMem(cudaMemcpyToSymbol(base_obs_vertices, obs_vertices.data(), obs_vertices.size() * sizeof(Vector3f)));
    Vector3f *d_obs_points;
    Triangle *d_obs_triangles;

    checkCudaCall(cudaMalloc(&d_obs_points, obs_vertices.size() * sizeof(Vector3f)));
    checkCudaCall(cudaMalloc(&d_obs_triangles, obs_triangles.size() * sizeof(Triangle)));
    checkCudaMem(cudaMemcpy(d_obs_points, obs_vertices.data(), obs_vertices.size() * sizeof(Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_obs_triangles, obs_triangles.data(), obs_triangles.size() * sizeof(Triangle), cudaMemcpyHostToDevice));
    std::cout << "Copied the obstacle vertices and triangles " << std::endl;


    Configuration *d_configs;
    checkCudaCall(cudaMalloc(&d_configs, configs.size() * sizeof(Configuration)));
    checkCudaMem(cudaMemcpy(d_configs, configs.data(), configs.size() * sizeof(Configuration), cudaMemcpyHostToDevice));
    std::cout << "Copied the configurations " << std::endl;

    // AABB* d_bot_bounds;
    // checkCudaCall(cudaMalloc(&d_bot_bounds, configs.size() * sizeof(AABB)));
    std::cout << "Malloced the AABBs " << std::endl;

    // Move obstacle to AABB (on CPU since we only have 1)
    AABB *obstacle_AABB = new AABB();
    generateAABBBaseline(obs_vertices.data(), obs_vertices.size(), 1, obstacle_AABB);

    bool *valid_conf_d;
    AABB *obstacle_AABB_d;
    checkCudaCall(cudaMalloc(&valid_conf_d, configs.size() * sizeof(bool)));
    checkCudaCall(cudaMalloc(&obstacle_AABB_d, sizeof(AABB)));
    checkCudaCall(cudaMemcpy(obstacle_AABB_d, obstacle_AABB, sizeof(AABB), cudaMemcpyHostToDevice));

    dim3 dimGridTransformKernel(ceil((float)(configs.size()) / TRANSFORM_BLOCK_SIZE), 1, 1);
    dim3 dimBlockTransformKernel(TRANSFORM_BLOCK_SIZE, 1, 1);
    broadPhaseFusedKernel<<<dimGridTransformKernel, dimBlockTransformKernel>>>( d_configs, obstacle_AABB_d,
                                                                                d_rob_transformed_points, valid_conf_d,
                                                                                configs.size(), rob_vertices.size());
    // checkCudaCall(cudaDeviceSynchronize());
    std::cout << "About to call narrow phase" << std::endl;

    // Vector3f test_rob_points[3];
    // test_rob_points[0] = {1.441547, -14.800514, 62.841087};
    // test_rob_points[1] = {-4.215309, 8.199282, 23.057938};
    // test_rob_points[2] = {1.883977, -15.487457, 62.381035};

    // Vector3f test_obs_points[3];
    // test_obs_points[0] = {1.681669, 2.616245, 1.069425};
    // test_obs_points[1] = {3.561536, 0.677467, 1.707230};
    // test_obs_points[2] = {1.172210, 2.534812, 1.852433};

    // Triangle test_rob_triangles = {0, 1, 2};
    // Triangle test_obs_triangles = {0, 1, 2};

    // Vector3f *d_test_rob_points;
    // Vector3f *d_test_obs_points;
    // Triangle *d_test_rob_triangles;
    // Triangle *d_test_obs_triangles;

    // checkCudaCall(cudaMalloc(&d_test_rob_points, 3 * sizeof(Vector3f)));
    // checkCudaCall(cudaMalloc(&d_test_rob_triangles, sizeof(Triangle)));
    // checkCudaCall(cudaMemcpy(d_test_rob_points, test_rob_points, 3 * sizeof(Vector3f), cudaMemcpyHostToDevice));
    // checkCudaCall(cudaMemcpy(d_test_rob_triangles, &test_rob_triangles, sizeof(Triangle), cudaMemcpyHostToDevice));


    // checkCudaCall(cudaMalloc(&d_test_obs_points, 3 * sizeof(Vector3f)));
    // checkCudaCall(cudaMalloc(&d_test_obs_triangles, sizeof(Triangle)));
    // checkCudaMem(cudaMemcpy(d_test_obs_points, test_obs_points, 3 * sizeof(Vector3f), cudaMemcpyHostToDevice));
    // checkCudaMem(cudaMemcpy(d_test_obs_triangles, &test_obs_triangles, sizeof(Triangle), cudaMemcpyHostToDevice));
    
    // narrowPhaseKernel<<<1, 1>>>(
    //     1, 1, 3, 1, 3, d_test_rob_triangles, d_test_rob_points, d_test_obs_triangles, d_test_obs_points,
    //     valid_conf_d);
    // checkCudaCall(cudaDeviceSynchronize());

    // checkCudaCall(cudaMemcpy(valid_conf, valid_conf_d , sizeof(bool), cudaMemcpyDeviceToHost));
    // checkCudaCall(cudaDeviceSynchronize());
    // std::cout << "configuration was " << valid_conf[0] << std::endl;
    
    auto start = std::chrono::high_resolution_clock::now();
    narrowPhaseKernel<<<(configs.size() - 1) / 128 + 1, 128>>>(
        configs.size(), rob_triangles.size(), rob_vertices.size(), obs_triangles.size(),
        obs_vertices.size(), d_rob_triangles, d_rob_transformed_points, d_obs_triangles, d_obs_points,
        valid_conf_d);
    
    checkCudaCall(cudaDeviceSynchronize());
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "Narrow phase took " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;
    checkCudaCall(cudaMemcpy(valid_conf, valid_conf_d , configs.size() * sizeof(bool), cudaMemcpyDeviceToHost));

    // narrowPhase(configs.size(), rob_triangles.size(), rob_vertices.size(), obs_triangles.size(),
    //         obs_vertices.size(), d_rob_triangles, d_rob_transformed_points, d_obs_triangles, d_obs_points,
    //         valid_conf);
    // broadPhase(configs.size(), d_bot_bounds, obstacle_AABB_d, valid_conf_d);

    std::cout << "Launched kernel execution" << std::endl;

    checkCudaCall(cudaDeviceSynchronize());
    std::cout << "Synchronized" << std::endl;

    // std:: cout << "Copying back results" << std::endl;
    // cudaMemcpy(bot_bounds, d_bot_bounds, configs.size() * sizeof(AABB), cudaMemcpyDeviceToHost);
    // cudaMemcpy(valid_conf, valid_conf_d, configs.size() * sizeof(bool), cudaMemcpyDeviceToHost);
    // checkCudaCall(cudaDeviceSynchronize());

    checkCudaCall(cudaFree(d_configs));
    // checkCudaCall(cudaFree(d_bot_bounds));
    checkCudaCall(cudaFree(d_rob_transformed_points));
    checkCudaCall(cudaFree(d_obs_points));
    checkCudaCall(cudaFree(d_rob_triangles));
    checkCudaCall(cudaFree(d_obs_triangles));
    checkCudaCall(cudaFree(obstacle_AABB_d));
    checkCudaCall(cudaFree(valid_conf_d));
    std::cout << "Copied back memory and synchronized" << std::endl;
}

void broadPhaseFused_sep(std::vector<Configuration> &configs, bool *valid_conf, AABB* bot_bounds)
{
    int device_count;
    if (cudaGetDeviceCount(&device_count) != 0) std::cout << "CUDA not loaded properly" << std::endl;

    //Load Robot
    std::vector<Vector3f> rob_vertices;
    std::vector<Triangle> rob_triangles;
    loadOBJFile(ROB_FILE, rob_vertices, rob_triangles);
    std::cout << "Robot has " << rob_vertices.size() << " vertices " <<std::endl;
    std::cout << "Robot has " << rob_triangles.size() << " triangles " <<std::endl;

    //Load Obstacles
    std::vector<Vector3f> obs_vertices;
    std::vector<Triangle> obs_triangles;
    loadOBJFile(OBS_FILE, obs_vertices, obs_triangles);
    std::cout << "Obstacle has " << obs_vertices.size() << " vertices " <<std::endl;
    std::cout << "Obstacle has " << obs_triangles.size() << " triangles " <<std::endl;

    size_t count = 0;
    for (const auto& triangle : obs_triangles) {
        if (count > 100){
            break;
        }
        std::cout << "v: " << triangle.v1 << ", v2: " << triangle.v2 << ", v3: " << triangle.v3 << std::endl;
        count++;
    }
    // std::cout <base_robot_trianglesb_points;
    float *d_rob_transformed_points_x;
    float *d_rob_transformed_points_y;
    float *d_rob_transformed_points_z;
    Triangle *d_rob_triangles;
    Vector3f *d_rob_points;

    checkCudaCall(cudaMalloc(&d_rob_transformed_points_x, rob_vertices.size() * configs.size() * sizeof(float)));
    checkCudaCall(cudaMalloc(&d_rob_transformed_points_y, rob_vertices.size() * configs.size() * sizeof(float)));
    checkCudaCall(cudaMalloc(&d_rob_transformed_points_z, rob_vertices.size() * configs.size() * sizeof(float)));
    checkCudaCall(cudaMalloc(&d_rob_triangles, rob_triangles.size() * sizeof(Triangle)));
    // checkCudaMem(cudaMemcpy(d_rob_points, rob_vertices.data(), rob_vertices.size() * sizeof(Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_rob_triangles, rob_triangles.data(), rob_triangles.size() * sizeof(Triangle), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpyToSymbol(base_robot_vertices, rob_vertices.data(), rob_vertices.size() * sizeof(Vector3f)));
    // checkCudaMem(cudaMemcpyToSymbol(base_robot_triangles, rob_triangles.data(), rob_triangles.size() * sizeof(Triangle)));
    std::cout << "Copied the robot vertices and triangles " << std::endl;
    
    checkCudaMem(cudaMemcpyToSymbol(base_obs_vertices, obs_vertices.data(), obs_vertices.size() * sizeof(Vector3f)));
    Vector3f *d_obs_points;
    Triangle *d_obs_triangles;

    checkCudaCall(cudaMalloc(&d_obs_points, obs_vertices.size() * sizeof(Vector3f)));
    checkCudaCall(cudaMalloc(&d_obs_triangles, obs_triangles.size() * sizeof(Triangle)));
    checkCudaMem(cudaMemcpy(d_obs_points, obs_vertices.data(), obs_vertices.size() * sizeof(Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_obs_triangles, obs_triangles.data(), obs_triangles.size() * sizeof(Triangle), cudaMemcpyHostToDevice));
    std::cout << "Copied the obstacle vertices and triangles " << std::endl;


    Configuration *d_configs;
    checkCudaCall(cudaMalloc(&d_configs, configs.size() * sizeof(Configuration)));
    checkCudaMem(cudaMemcpy(d_configs, configs.data(), configs.size() * sizeof(Configuration), cudaMemcpyHostToDevice));
    std::cout << "Copied the configurations " << std::endl;

    // AABB* d_bot_bounds;
    // checkCudaCall(cudaMalloc(&d_bot_bounds, configs.size() * sizeof(AABB)));
    std::cout << "Malloced the AABBs " << std::endl;

    // Move obstacle to AABB (on CPU since we only have 1)
    AABB *obstacle_AABB = new AABB();
    generateAABBBaseline(obs_vertices.data(), obs_vertices.size(), 1, obstacle_AABB);

    bool *valid_conf_d;
    AABB *obstacle_AABB_d;
    checkCudaCall(cudaMalloc(&valid_conf_d, configs.size() * sizeof(bool)));
    checkCudaCall(cudaMalloc(&obstacle_AABB_d, sizeof(AABB)));
    checkCudaCall(cudaMemcpy(obstacle_AABB_d, obstacle_AABB, sizeof(AABB), cudaMemcpyHostToDevice));

    dim3 dimGridTransformKernel(ceil((float)(configs.size()) / TRANSFORM_BLOCK_SIZE), 1, 1);
    dim3 dimBlockTransformKernel(TRANSFORM_BLOCK_SIZE, 1, 1);
    broadPhaseFusedKernel_sep<<<dimGridTransformKernel, dimBlockTransformKernel>>>( d_configs, obstacle_AABB_d,
                                                                                d_rob_transformed_points_x, d_rob_transformed_points_y, d_rob_transformed_points_z, valid_conf_d,
                                                                                configs.size(), rob_vertices.size());
    checkCudaCall(cudaDeviceSynchronize());
    std::cout << "About to call narrow phase" << std::endl;
    narrowPhase_sep(configs.size(), rob_triangles.size(), rob_vertices.size(), obs_triangles.size(), 
            obs_vertices.size(), rob_triangles.data(), d_rob_transformed_points_x, d_rob_transformed_points_y, d_rob_transformed_points_z, obs_triangles.data(), obs_vertices.data(),
            valid_conf);
    // broadPhase(configs.size(), d_bot_bounds, obstacle_AABB_d, valid_conf_d);

    std::cout << "Launched kernel execution" << std::endl;
    
    checkCudaCall(cudaDeviceSynchronize());
    std::cout << "Synchronized" << std::endl;

    std:: cout << "Copying back results" << std::endl;
    // cudaMemcpy(bot_bounds, d_bot_bounds, configs.size() * sizeof(AABB), cudaMemcpyDeviceToHost);
    // cudaMemcpy(valid_conf, valid_conf_d, configs.size() * sizeof(bool), cudaMemcpyDeviceToHost);
    checkCudaCall(cudaDeviceSynchronize());

    checkCudaCall(cudaFree(d_configs));
    // checkCudaCall(cudaFree(d_bot_bounds));
    checkCudaCall(cudaFree(d_rob_transformed_points_x));
    checkCudaCall(cudaFree(d_rob_transformed_points_y));
    checkCudaCall(cudaFree(d_rob_transformed_points_z));
    // checkCudaCall(cudaFree(d_obs_));
    checkCudaCall(cudaFree(d_obs_points));
    checkCudaCall(cudaFree(d_rob_triangles));
    checkCudaCall(cudaFree(d_obs_triangles));
    checkCudaCall(cudaFree(obstacle_AABB_d));
    checkCudaCall(cudaFree(valid_conf_d));
    std::cout << "Copied back memory and synchronized" << std::endl;
}