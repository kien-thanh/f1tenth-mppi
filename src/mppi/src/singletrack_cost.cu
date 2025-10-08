#include <singletrack_cost.cuh>

SingletrackCost::SingletrackCost(cudaStream_t stream)
{
    bindToStream(stream);
}

__device__ float SingletrackCost::computeStateCost(float *state, int timestep, float *theta_c, int *crash_status)
{
    float x = state[O_IND_CLASS(DYN_P, X_POS)];
    float y = state[O_IND_CLASS(DYN_P, Y_POS)];
    float steer = state[O_IND_CLASS(DYN_P, STEER)];
    float vel = state[O_IND_CLASS(DYN_P, VEL)];
    float yaw = state[O_IND_CLASS(DYN_P, YAW)];

    float half_width = this->params_.width / 2.0f;

    float cost = 0.0f;

    if (abs(y) >= half_width)
    {
        cost = INFINITY;
    }

    else 
    {
        cost = y*y + steer*steer + yaw*yaw + (x - 2.0f)*(x - 2.0f);
    }

    return cost;
}

float SingletrackCost::computeStateCost(const Eigen::Ref<const output_array> state, int timestep, int *crash_status)
{
    float x = state[O_IND_CLASS(DYN_P, X_POS)];
    float y = state[O_IND_CLASS(DYN_P, Y_POS)];
    float steer = state[O_IND_CLASS(DYN_P, STEER)];
    float vel = state[O_IND_CLASS(DYN_P, VEL)];
    float yaw = state[O_IND_CLASS(DYN_P, YAW)];

    float half_width = this->params_.width / 2.0f;

    float cost = 0.0f;

    if (abs(y) >= half_width)
    {
        cost = INFINITY;
    }

    else 
    {
        cost = y*y + steer*steer + yaw*yaw + (x - 2.0f)*(x - 2.0f);
    }

    return cost;
}

__device__ float SingletrackCost::terminalCost(float *state, float *theta_c)
{
    return 0.0f;
}

float SingletrackCost::terminalCost(const Eigen::Ref<const output_array> state)
{
    return 0.0f;
}