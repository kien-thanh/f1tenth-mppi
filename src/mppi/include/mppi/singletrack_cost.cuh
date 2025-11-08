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

private:
    std::unique_ptr<Texture> distance_field_;
    cudaTextureObject_t df_texObj_;
    float df_resolution_, df_origin_x_, df_origin_y_;

    std::unique_ptr<Texture> costmap_;
    cudaTextureObject_t cm_texObj_;
    float cm_resolution_, cm_origin_x_, cm_origin_y_;
};

#if __CUDACC__
#include "singletrack_cost.cu"
#endif

#endif // SINGLETRACK_COST_CUH_
