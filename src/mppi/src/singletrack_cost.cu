#include <singletrack_cost.cuh>

__device__ float SingletrackCost::computeStateCost(float *state, int timestep, float *theta_c, int *crash_status)
{
    float x = state[O_IND_CLASS(DYN_P, X_POS)];
    float y = state[O_IND_CLASS(DYN_P, Y_POS)];
    float steer = state[O_IND_CLASS(DYN_P, STEER)];
    float vel = state[O_IND_CLASS(DYN_P, VEL)];
    float yaw = state[O_IND_CLASS(DYN_P, YAW)];

    float cost = 0.0f;

    float crash = sampleDistance(cm_texObj_, x, y, cm_resolution_, cm_origin_x_, cm_origin_y_);
    if (crash > 0.0f) {
        cost = 1e6f; // High cost for crashing
        if (crash_status != nullptr) {
            *crash_status = 1;
        }
        return cost;
    }

    float distance = sampleDistance(df_texObj_, x, y, df_resolution_, df_origin_x_, df_origin_y_);

    float c_dist = this->params_.c_distance;
    float c_vel = this->params_.c_velocity;
    float desired_vel = this->params_.desired_velocity;

    cost = c_dist * (distance * distance) + c_vel * (vel - desired_vel) * (vel - desired_vel);

    return cost;
}

float SingletrackCost::computeStateCost(const Eigen::Ref<const output_array> state, int timestep, int *crash_status)
{
    float x = state[O_IND_CLASS(DYN_P, X_POS)];
    float y = state[O_IND_CLASS(DYN_P, Y_POS)];
    float steer = state[O_IND_CLASS(DYN_P, STEER)];
    float vel = state[O_IND_CLASS(DYN_P, VEL)];
    float yaw = state[O_IND_CLASS(DYN_P, YAW)];

    float cost = 0.0f;

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