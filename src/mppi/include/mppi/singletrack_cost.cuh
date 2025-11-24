#pragma once

#include <mppi/cost_functions/cost.cuh>
#include <singletrack_dynamics.cuh>
#include <texture.cuh>

__global__ void updateScanMapKernel(const float2 *points, cudaSurfaceObject_t surfObj,
                                    float origin_x, float origin_y, float resolution);

__global__ void debugReadSurface(cudaSurfaceObject_t surfObj,
                                 unsigned char *out, int x, int y);

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
    using output_array = typename Cost<SingletrackCost, SingletrackCostParams, SingletrackDynamicsParams>::output_array;

    SingletrackCost(cudaStream_t stream = nullptr);
    ~SingletrackCost();

    __device__ float computeStateCost(float *s, int timestep = 0, float *theta_c = nullptr, int *crash_status = nullptr);

    float computeStateCost(const Eigen::Ref<const output_array> s, int timestep = 0, int *crash_status = nullptr);

    __device__ float terminalCost(float *s, float *theta_c);
    float terminalCost(const Eigen::Ref<const output_array> s);

    void createDistanceField(const float *data, float resolution, int width, int height, float origin_x, float origin_y);
    void createCostmap(const float *data, float resolution, int width, int height, float origin_x, float origin_y);
    void createScanMap(float resolution, int width, int height, float origin_x, float origin_y);

    void updateScanMap(const std::vector<float2> &h_points, int width, int height, float resolution, float origin_x, float origin_y);

    void getScanMapData(std::vector<unsigned char> &out_data, int width, int height);

private:
    std::unique_ptr<Texture> distance_field_;
    cudaTextureObject_t df_texObj_ = 0;
    float df_resolution_ = 0.0f, df_origin_x_ = 0.0f, df_origin_y_ = 0.0f;

    std::unique_ptr<Texture> costmap_;
    cudaTextureObject_t cm_texObj_ = 0;
    float cm_resolution_ = 0.0f, cm_origin_x_ = 0.0f, cm_origin_y_ = 0.0f;

    cudaArray_t surfArray_ = nullptr;
    cudaSurfaceObject_t surfObj_ = 0;

    float2* d_scanPoints_ = nullptr;
};
