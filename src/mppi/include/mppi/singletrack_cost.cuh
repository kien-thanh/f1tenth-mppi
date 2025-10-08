#pragma once

#ifndef SINGLETRACK_COST_CUH_
#define SINGLETRACK_COST_CUH_

#include <mppi/cost_functions/cost.cuh>
#include <singletrack_dynamics.cuh>

struct SingletrackCostParams : public CostParams<SingletrackDynamics::CONTROL_DIM>
{
    float width = 2.0f;  // width of the track
    float coeff = 10.0f; // cost coefficient
};

class SingletrackCost : public Cost<SingletrackCost, SingletrackCostParams, SingletrackDynamicsParams>
{
public:
    using DYN_P = Cost<SingletrackCost, SingletrackCostParams, SingletrackDynamicsParams>::TEMPLATED_DYN_PARAMS;

    SingletrackCost(cudaStream_t stream = nullptr);

    __device__ float computeStateCost(float *s, int timestep = 0, float *theta_c = nullptr, int *crash_status = nullptr);

    float computeStateCost(const Eigen::Ref<const output_array> s, int timestep = 0, int *crash_status = nullptr);

    __device__ float terminalCost(float *s, float *theta_c);

    float terminalCost(const Eigen::Ref<const output_array> s);
};

#if __CUDACC__
#include "singletrack_cost.cu"
#endif

#endif // SINGLETRACK_COST_CUH_
