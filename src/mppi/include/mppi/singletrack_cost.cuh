#pragma once

#ifndef SINGLETRACK_COST_CUH_
#define SINGLETRACK_COST_CUH_

#include <mppi/cost_functions/cost.cuh>
#include <singletrack_dynamics.cuh>
#include <texture.cuh>

struct SingletrackCostParams : public CostParams<SingletrackDynamics::CONTROL_DIM>
{
    float c_distance = 10.0f;   // Cost coefficient for distance to obstacles
    float c_velocity = 10.0f;   // Cost coefficient for velocity tracking
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

    void createDistanceField(const float* data, float resolution, int width, int height, float origin_x, float origin_y)
    {
        distance_field_ = std::make_unique<Texture>(data, resolution, width, height, origin_x, origin_y);
        df_texObj_ = distance_field_->getTextureObject();
        df_resolution_ = distance_field_->getResolution();
        df_origin_x_ = distance_field_->getOriginX();
        df_origin_y_ = distance_field_->getOriginY();

        printf("Distance field created with resolution: %f, origin: (%f, %f)\n", df_resolution_, df_origin_x_, df_origin_y_);
    }

    void createCostmap(const float* data, float resolution, int width, int height, float origin_x, float origin_y)
    {
        costmap_ =  std::make_unique<Texture>(data, resolution, width, height, origin_x, origin_y);
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

        auto end =  std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
      
        std::cout << "Scanmap created with resolution: " << resolution << ", origin: (" << origin_x << ", " << origin_y << ") in " << duration << " ms" << std::endl;
    }

    void updateScanMap(const std::vector<Eigen::Vector3f>& points)
    {
        auto start = std::chrono::high_resolution_clock::now();

        // Update scanmap based on the provided points
        for (const auto& point : points) {
            // Convert world coordinates to scanmap indices
            int x_idx = static_cast<int>((point(0) - cm_origin_x_) / cm_resolution_);
            int y_idx = static_cast<int>((point(1) - cm_origin_y_) / cm_resolution_);

            // Write occupied value (e.g., 255) to the surface object at the calculated indices
            unsigned char occupied_value = 255;
            surf2Dwrite(occupied_value, surfObj_, x_idx * sizeof(unsigned char), y_idx);
        }

        auto end =  std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        std::cout << "Scanmap updated with " << points.size() << " points in " << duration << " ms" << std::endl;
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
