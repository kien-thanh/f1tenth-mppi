#pragma once

#ifndef SINGLETRACK_COST_CUH_
#define SINGLETRACK_COST_CUH_

#include <mppi/cost_functions/cost.cuh>
#include <singletrack_dynamics.cuh>
#include <distance_field.cuh>

struct SingletrackCostParams : public CostParams<SingletrackDynamics::CONTROL_DIM>
{
    float width = 2.0f;  // width of the track
    float coeff = 10.0f; // cost coefficient
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
        distance_field_ = std::make_unique<DistanceField>(data, resolution, width, height, origin_x, origin_y);
        texObj_ = distance_field_->getTextureObject();
        resolution_ = distance_field_->getResolution();
        origin_x_ = distance_field_->getOriginX();
        origin_y_ = distance_field_->getOriginY();

        printf("Distance field created with resolution: %f, origin: (%f, %f)\n", resolution_, origin_x_, origin_y_);
    }

private:
    std::unique_ptr<DistanceField> distance_field_;
    cudaTextureObject_t texObj_;
    float resolution_, origin_x_, origin_y_;
};

#if __CUDACC__
#include "singletrack_cost.cu"
#endif

#endif // SINGLETRACK_COST_CUH_
