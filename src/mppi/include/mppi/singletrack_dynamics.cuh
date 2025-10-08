#ifndef SINGLETRACK_CUH_
#define SINGLETRACK_CUH_

#include <mppi/dynamics/dynamics.cuh>

struct SingletrackDynamicsParams : public DynamicsParams
{
    enum class StateIndex : int
    {
        X_POS = 0,
        Y_POS,
        STEER,
        VEL,
        YAW,
        NUM_STATES
    };

    enum class ControlIndex : int
    {
        STEER_SPEED = 0,
        ACCELERATION,
        NUM_CONTROLS
    };

    enum class OutputIndex : int
    {
        X_POS = 0,
        Y_POS,
        STEER,
        VEL,
        YAW,
        NUM_OUTPUTS
    };

    float wheelbase = 0.31f;

    SingletrackDynamicsParams() = default;
    SingletrackDynamicsParams(float wheelbase) : wheelbase(wheelbase) {};
};

using namespace MPPI_internal;

class SingletrackDynamics : public Dynamics<SingletrackDynamics, SingletrackDynamicsParams>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using PARENT_CLASS = Dynamics<SingletrackDynamics, SingletrackDynamicsParams>;
    SingletrackDynamics(float wheelbase = 0.31f, cudaStream_t stream = 0);

    std::string getDynamicsModelName() const override
    {
        return "Singletrack Model";
    }

    void computeDynamics(const Eigen::Ref<const state_array> &state, const Eigen::Ref<const control_array> &control, Eigen::Ref<state_array> state_der);

    bool computeGrad(const Eigen::Ref<const state_array> &state, const Eigen::Ref<const control_array> &control, Eigen::Ref<dfdx> A, Eigen::Ref<dfdu> B);

    __host__ __device__ float getWheelbase()
    {
        return params_.wheelbase;
    }

    __host__ __device__ float getGravity()
    {
        return gravity_;
    }


    void printState(const Eigen::Ref<const state_array>& state);
    void printState(float* state);
    void printParams();

    __device__ void computeDynamics(float* state, float* control, float* state_der, float* theta = nullptr);

    state_array stateFromMap(const std::map<std::string, float>& map) override;

protected:
    const float gravity_ = 9.81;
};

#if __CUDACC__
#include "singletrack_dynamics.cu"
#endif

#endif