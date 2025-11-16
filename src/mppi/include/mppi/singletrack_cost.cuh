#pragma once

#ifndef SINGLETRACK_COST_CUH_
#define SINGLETRACK_COST_CUH_

#include <mppi/cost_functions/cost.cuh>
#include <singletrack_dynamics.cuh>
#include <texture.cuh>

__global__ void updateScanMapKernel(const float2 *points, cudaSurfaceObject_t surfObj, float origin_x, float origin_y, float resolution)
{
    int i = threadIdx.x + blockIdx.x * blockDim.x;

    if (i >= 1080)
        return;

    int x_idx = static_cast<int>((points[i].x - origin_x) / resolution);
    int y_idx = static_cast<int>((points[i].y - origin_y) / resolution);

    if (x_idx < 0 || y_idx < 0 || x_idx >= 130 || y_idx >= 113)
        return;

    // printf("Updating scan map at (%d, %d)\n", x_idx, y_idx);

    unsigned char occupied_value = 100;
    
    for (int offset_x = -2; offset_x <= 2; ++offset_x) {
        for (int offset_y = -2; offset_y <= 2; ++offset_y) {
            if (x_idx + offset_x >= 0 && y_idx + offset_y >= 0 && x_idx + offset_x < 130 && y_idx + offset_y < 113) {
                surf2Dwrite(occupied_value, surfObj, (x_idx + offset_x) * sizeof(unsigned char), y_idx + offset_y);
            }
        }
    }
}

__global__ void debugReadSurface(cudaSurfaceObject_t surfObj,
                                 unsigned char *out,
                                 int x,
                                 int y)
{
    unsigned char value;
    surf2Dread(&value, surfObj, x * sizeof(unsigned char), y);

    out[0] = value;
}

struct SingletrackCostParams : public CostParams<SingletrackDynamics::CONTROL_DIM>
{
    float c_distance = 10.0f;      // Cost coefficient for distance to obstacles
    float c_velocity = 10.0f;      // Cost coefficient for velocity tracking
    float desired_velocity = 0.5f; // Desired velocity for the vehicle
};

class SingletrackCost : public Cost<SingletrackCost, SingletrackCostParams, SingletrackDynamicsParams>
{
public:
    using DYN_P = Cost<SingletrackCost, SingletrackCostParams, SingletrackDynamicsParams>::TEMPLATED_DYN_PARAMS;

    SingletrackCost(cudaStream_t stream = nullptr) : distance_field_(nullptr)
    {
        bindToStream(stream);
    }

    __device__ float computeStateCost(float *s, int timestep = 0, float *theta_c = nullptr, int *crash_status = nullptr);

    float computeStateCost(const Eigen::Ref<const output_array> s, int timestep = 0, int *crash_status = nullptr);

    __device__ float terminalCost(float *s, float *theta_c);

    float terminalCost(const Eigen::Ref<const output_array> s);

    void createDistanceField(const float *data, float resolution, int width, int height, float origin_x, float origin_y)
    {
        distance_field_ = std::make_unique<Texture>(data, resolution, width, height, origin_x, origin_y);
        df_texObj_ = distance_field_->getTextureObject();
        df_resolution_ = distance_field_->getResolution();
        df_origin_x_ = distance_field_->getOriginX();
        df_origin_y_ = distance_field_->getOriginY();

        printf("Distance field created with resolution: %f, origin: (%f, %f)\n", df_resolution_, df_origin_x_, df_origin_y_);
    }

    void createCostmap(const float *data, float resolution, int width, int height, float origin_x, float origin_y)
    {
        costmap_ = std::make_unique<Texture>(data, resolution, width, height, origin_x, origin_y);
        cm_texObj_ = costmap_->getTextureObject();
        cm_resolution_ = costmap_->getResolution();
        cm_origin_x_ = costmap_->getOriginX();
        cm_origin_y_ = costmap_->getOriginY();

        printf("Costmap created with resolution: %f, origin: (%f, %f)\n", cm_resolution_, cm_origin_x_, cm_origin_y_);
    }

    void createScanMap(float resolution, int width, int height, float origin_x, float origin_y)
    {
        auto start = std::chrono::high_resolution_clock::now();

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

    void updateScanMap(const std::vector<float2> &h_points)
    {
        auto start = std::chrono::high_resolution_clock::now();

        float2 *d_points;
        cudaMalloc(&d_points, 1080 * sizeof(float2));
        cudaMemcpy(d_points, h_points.data(), 1080 * sizeof(float2), cudaMemcpyHostToDevice);

        updateScanMapKernel<<<9, 128>>>(d_points, surfObj_, 0.387, -4.08, 0.05f);

        cudaDeviceSynchronize();

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        cudaFree(d_points);

        // unsigned char* d_debug_output;
        // cudaMalloc(&d_debug_output, sizeof(unsigned char));
        // debugReadSurface<<<1, 1>>>(surfObj_, d_debug_output, 57, 46);
        // cudaDeviceSynchronize();

        // unsigned char debug_value;
        // cudaMemcpy(&debug_value, d_debug_output, sizeof(unsigned char), cudaMemcpyDeviceToHost);
        // cudaFree(d_debug_output);

        // std::vector<unsigned char> h_buffer(130 * 113);
        // cudaMemcpy2DFromArray(
        //     h_buffer.data(),             // dst pointer
        //     130 * sizeof(unsigned char), // dst pitch
        //     surfArray_,                  // src array
        //     0, 0,                        // offset inside the array
        //     130 * sizeof(unsigned char), // width in bytes
        //     113,                         // height
        //     cudaMemcpyDeviceToHost);

        // for (int y = 0; y < 113; ++y)
        // {
        //     for (int x = 0; x < 130; ++x)
        //         std::cout << (h_buffer[y * 130 + x] ? "#" : ".");
        //     std::cout << std::endl;
        // }

        // std::cout << "Debug value from surface: " << static_cast<int>(debug_value) << std::endl;
        // std::cout << "Scanmap updated with " << h_points.size() << " points in " << duration << " ms" << std::endl;
    }

private:
    std::unique_ptr<Texture> distance_field_;
    cudaTextureObject_t df_texObj_;
    float df_resolution_, df_origin_x_, df_origin_y_;

    std::unique_ptr<Texture> costmap_;
    cudaTextureObject_t cm_texObj_;
    float cm_resolution_, cm_origin_x_, cm_origin_y_;

    cudaArray_t surfArray_;
    cudaSurfaceObject_t surfObj_;
};

#if __CUDACC__
#include "singletrack_cost.cu"
#endif

#endif // SINGLETRACK_COST_CUH_
