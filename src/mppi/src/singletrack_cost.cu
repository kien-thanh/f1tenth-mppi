#include "singletrack_cost.cuh"

__global__ void clearSurfaceKernel(cudaSurfaceObject_t surfObj, int width, int height, unsigned char value = 0)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x >= width || y >= height) return;

    surf2Dwrite(value, surfObj, x * sizeof(unsigned char), y);
}

__global__ void updateScanMapKernel(const float2 *points, cudaSurfaceObject_t surfObj, int width, int height, float resolution, float origin_x, float origin_y)
{
    int i = threadIdx.x + blockIdx.x * blockDim.x;
    // caller expects 1080 points (you can pass fewer and adjust)
    if (i >= 1080)
        return;

    int x_idx = static_cast<int>((points[i].x - origin_x) / resolution);
    int y_idx = static_cast<int>((points[i].y - origin_y) / resolution);

    if (x_idx < 0 || y_idx < 0 || x_idx >= width || y_idx >= height)
        return;

    unsigned char occupied_value = 100;

    for (int offset_x = -2; offset_x <= 2; ++offset_x)
    {
        for (int offset_y = -2; offset_y <= 2; ++offset_y)
        {
            int xx = x_idx + offset_x;
            int yy = y_idx + offset_y;
            if (xx >= 0 && yy >= 0 && xx < width && yy < height)
            {
                // surf2Dwrite takes byte offset horizontally
                surf2Dwrite(occupied_value, surfObj, xx * sizeof(unsigned char), yy);
            }
        }
    }
}

__global__ void debugReadSurface(cudaSurfaceObject_t surfObj,
                                 unsigned char *out,
                                 int x,
                                 int y)
{
    unsigned char value = 0;
    surf2Dread(&value, surfObj, x * sizeof(unsigned char), y);
    out[0] = value;
}

SingletrackCost::SingletrackCost(cudaStream_t stream)
{
    bindToStream(stream);
}

SingletrackCost::~SingletrackCost()
{
    if (surfObj_)
    {
        cudaDestroySurfaceObject(surfObj_);
        surfObj_ = 0;
    }
    if (surfArray_)
    {
        cudaFreeArray(surfArray_);
        surfArray_ = nullptr;
    }
}

void SingletrackCost::createDistanceField(const float *data, float resolution, int width, int height, float origin_x, float origin_y)
{
    distance_field_ = std::make_unique<Texture>(data, resolution, width, height, origin_x, origin_y);
    df_texObj_ = distance_field_->getTextureObject();
    df_resolution_ = distance_field_->getResolution();
    df_origin_x_ = distance_field_->getOriginX();
    df_origin_y_ = distance_field_->getOriginY();

    printf("Distance field created with resolution: %f, origin: (%f, %f)\n", df_resolution_, df_origin_x_, df_origin_y_);
}

void SingletrackCost::createAngleGrid(const float *data, float resolution, int width, int height, float origin_x, float origin_y)
{
    angle_grid_ = std::make_unique<Texture>(data, resolution, width, height, origin_x, origin_y);
    ag_texObj_ = angle_grid_->getTextureObject();
    ag_resolution_ = angle_grid_->getResolution();
    ag_origin_x_ = angle_grid_->getOriginX();
    ag_origin_y_ = angle_grid_->getOriginY();

    printf("Angle grid created with resolution: %f, origin: (%f, %f)\n", ag_resolution_, ag_origin_x_, ag_origin_y_);
}

void SingletrackCost::createCostmap(const float *data, float resolution, int width, int height, float origin_x, float origin_y)
{
    costmap_ = std::make_unique<Texture>(data, resolution, width, height, origin_x, origin_y);
    cm_texObj_ = costmap_->getTextureObject();
    cm_resolution_ = costmap_->getResolution();
    cm_origin_x_ = costmap_->getOriginX();
    cm_origin_y_ = costmap_->getOriginY();

    printf("Costmap created with resolution: %f, origin: (%f, %f)\n", cm_resolution_, cm_origin_x_, cm_origin_y_);
}

void SingletrackCost::createScanMap(int width, int height, float resolution, float origin_x, float origin_y)
{
    auto start = std::chrono::high_resolution_clock::now();

    sm_width_ = width;
    sm_height_ = height;
    sm_resolution_ = resolution;
    sm_origin_x_ = origin_x;
    sm_origin_y_ = origin_y;

    // Intilialize unoccupied scanmap
    std::vector<unsigned char> scanmap_data(width * height, 0);

    // Allocate CUDA array in device memory
    cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc<unsigned char>();
    cudaMallocArray(&surfArray_, &channelDesc, width, height, cudaArraySurfaceLoadStore);

    cudaMemcpy2DToArray(surfArray_, 0, 0, scanmap_data.data(), width * sizeof(unsigned char), width * sizeof(unsigned char), height, cudaMemcpyHostToDevice);

    // Create surface object for the CUDA array
    struct cudaResourceDesc resDesc;
    memset(&resDesc, 0, sizeof(resDesc));
    resDesc.resType = cudaResourceTypeArray;

    resDesc.res.array.array = surfArray_;
    cudaCreateSurfaceObject(&surfObj_, &resDesc);

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    std::cout << "Scanmap created with resolution: " << resolution << ", origin: (" << origin_x << ", " << origin_y << ") in " << duration << " ms" << std::endl;
}

void SingletrackCost::updateScanMap(const std::vector<float2> &h_points)
{
    dim3 block(16, 16);
    dim3 grid((sm_width_ + block.x - 1) / block.x, (sm_height_ + block.y - 1) / block.y);
    clearSurfaceKernel<<<grid, block>>>(surfObj_, sm_width_, sm_height_, 0);
    cudaError_t err = cudaDeviceSynchronize();
    if (err != cudaSuccess) {
        std::cerr << "CUDA error in clearSurfaceKernel: " << cudaGetErrorString(err) << std::endl;
        return;
    }

    int nrScanPoints = h_points.size();

    if (d_scanPoints_ == nullptr)
    {
        cudaMalloc(&d_scanPoints_, nrScanPoints * sizeof(float2));
    }

    cudaMemcpy(d_scanPoints_, h_points.data(), nrScanPoints * sizeof(float2), cudaMemcpyHostToDevice);
    updateScanMapKernel<<<9, 128>>>(d_scanPoints_, surfObj_, sm_width_, sm_height_, sm_resolution_, sm_origin_x_, sm_origin_y_);

    cudaDeviceSynchronize();
}

void SingletrackCost::getScanMapData(std::vector<unsigned char> &out_data, int width, int height)
{
    out_data.resize(width * height);
    cudaMemcpy2DFromArray(out_data.data(), width * sizeof(unsigned char), surfArray_, 0, 0, width * sizeof(unsigned char), height, cudaMemcpyDeviceToHost);
}

__device__ float SingletrackCost::computeStateCost(float *state, int timestep, float *theta_c, int *crash_status)
{
    float x = state[O_IND_CLASS(DYN_P, X_POS)];
    float y = state[O_IND_CLASS(DYN_P, Y_POS)];
    float steer = state[O_IND_CLASS(DYN_P, STEER)];
    float vel = state[O_IND_CLASS(DYN_P, VEL)];
    float yaw = state[O_IND_CLASS(DYN_P, YAW)];

    float cost = 0.0f;

    // --------------
    // Cost map
    // --------------
    float cm_cost = sampleTexture(cm_texObj_, x, y, cm_resolution_, cm_origin_x_, cm_origin_y_);

    // Crash condition
    if (cm_cost >= 99.0f)
    {
        return 1e6f;
    }

    // --------------
    // Scan
    // --------------
    int x_idx = static_cast<int>((x - sm_origin_x_) / sm_resolution_);
    int y_idx = static_cast<int>((y - sm_origin_y_) / sm_resolution_);

    // Out of bounds check
    if (x_idx < 0 || y_idx < 0 || x_idx >= sm_width_ || y_idx >= sm_height_)
    {
        return 1e6f;
    }

    unsigned char scanCrash = 0;
    surf2Dread(&scanCrash, surfObj_, x_idx * sizeof(unsigned char), y_idx);

    if (scanCrash >= 50.0f)
    {
        return 1e6f;
    }

    float distance = sampleTexture(df_texObj_, x, y, df_resolution_, df_origin_x_, df_origin_y_);
    float angle = sampleTexture(ag_texObj_, x, y, ag_resolution_, ag_origin_x_, ag_origin_y_);

    float c_dist = this->params_.c_distance;
    float c_vel = this->params_.c_velocity;
    float desired_vel = this->params_.desired_velocity;
    float c_costmap = this->params_.c_costmap;
    float c_angle = this->params_.c_angle;
    float c_latacc = this->params_.c_latacc; // lateral acceleration cost coefficient

    float ay = (vel * vel) * tanf(steer) / 0.33f;   // lateral acceleration

    cost = c_dist * (distance * distance) 
    + c_vel * (vel - desired_vel) * (vel - desired_vel) 
    + c_costmap * cm_cost 
    + c_angle * (fabsf(yaw - angle))
    + c_latacc * (ay * ay);
    return cost;
}

float SingletrackCost::computeStateCost(const Eigen::Ref<const output_array> state, int timestep, int *crash_status)
{
    (void)timestep;
    (void)crash_status;

    float x = state[O_IND_CLASS(DYN_P, X_POS)];
    float y = state[O_IND_CLASS(DYN_P, Y_POS)];
    (void)x;
    (void)y;
    return 0.0f;
}

__device__ float SingletrackCost::terminalCost(float *state, float *theta_c)
{
    (void)state;
    (void)theta_c;
    return 0.0f;
}

float SingletrackCost::terminalCost(const Eigen::Ref<const output_array> state)
{
    (void)state;
    return 0.0f;
}
