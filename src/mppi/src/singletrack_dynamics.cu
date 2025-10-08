#include <singletrack_dynamics.cuh>
#include <mppi/utils/math_utils.h>

SingletrackDynamics::SingletrackDynamics(float wheelbase, cudaStream_t stream) : Dynamics<SingletrackDynamics, SingletrackDynamicsParams>(stream)
{
  this->params_ = SingletrackDynamicsParams(wheelbase);
}

bool SingletrackDynamics::computeGrad(const Eigen::Ref<const state_array> &state, const Eigen::Ref<const control_array> &control, Eigen::Ref<dfdx> A, Eigen::Ref<dfdu> B)
{
  A(0, 3) = cos(state(S_INDEX(YAW)));
  A(0, 4) = -state(S_INDEX(VEL)) * sin(state(S_INDEX(YAW)));
  A(1, 3) = sin(state(S_INDEX(YAW)));
  A(1, 4) = state(S_INDEX(VEL)) * cos(state(S_INDEX(YAW)));
  A(4, 2) = (state(S_INDEX(VEL))/this->params_.wheelbase ) / (cos(state(S_INDEX(STEER))) * cos(state(S_INDEX(STEER))));
  A(4, 3) = (1.0f/this->params_.wheelbase) * tan(state(S_INDEX(STEER)));

  B(2, 0) = 1.0f;
  B(3, 1) = 1.0f;

  return true;
}

void SingletrackDynamics::computeDynamics(const Eigen::Ref<const state_array> &state, const Eigen::Ref<const control_array> &control, Eigen::Ref<state_array> state_der)
{
  state_der(S_INDEX(X_POS)) = state(S_INDEX(VEL)) * cos(state(S_INDEX(YAW)));
  state_der(S_INDEX(Y_POS)) = state(S_INDEX(VEL)) * sin(state(S_INDEX(YAW)));
  state_der(S_INDEX(STEER)) = control(C_INDEX(STEER_SPEED));
  state_der(S_INDEX(VEL)) = control(C_INDEX(ACCELERATION));
  state_der(S_INDEX(YAW)) = (state(S_INDEX(VEL)) / this->params_.wheelbase) * tan(state(S_INDEX(STEER)));
}

__device__ void SingletrackDynamics::computeDynamics(float *state, float *control, float *state_der, float *theta_s)
{
  float steer = angle_utils::normalizeAngle(state[S_INDEX(STEER)]);
  float yaw = angle_utils::normalizeAngle(state[S_INDEX(YAW)]);

  const float l_wb = this->params_.wheelbase;

  // const float MAX_STEER = M_PI / 3.0f;  // 60 degrees
  // if (steer > MAX_STEER)
  //   steer = MAX_STEER;
  // else if (steer < -MAX_STEER)
  //   steer = -MAX_STEER;

  state_der[S_INDEX(X_POS)] = state[S_INDEX(VEL)] * cosf(yaw);
  state_der[S_INDEX(Y_POS)] = state[S_INDEX(VEL)] * sinf(yaw);
  state_der[S_INDEX(STEER)] = control[C_INDEX(STEER_SPEED)];
  state_der[S_INDEX(VEL)] = control[C_INDEX(ACCELERATION)];
  state_der[S_INDEX(YAW)] = (state[S_INDEX(VEL)] / l_wb) * tanf(steer);
}

void SingletrackDynamics::printState(const Eigen::Ref<const state_array> &state)
{
  printf("Position: x=%f, y=%f; Steering angle: %f; Velocity: %f; Yaw: %f \n", state(0), state(1), state(2), state(3), state(4));
}

void SingletrackDynamics::printState(float *state)
{
  printf("Position: x=%f, y=%f; Steering angle: %f; Velocity: %f; Yaw: %f \n", state[0], state[1], state[2], state[3], state[4]);
}

Dynamics<SingletrackDynamics, SingletrackDynamicsParams>::state_array
SingletrackDynamics::stateFromMap(const std::map<std::string, float> &map)
{
  state_array s;
  s(S_INDEX(X_POS)) = map.at("X_POS");
  s(S_INDEX(Y_POS)) = map.at("Y_POS");
  s(S_INDEX(STEER)) = map.at("STEER");
  s(S_INDEX(VEL)) = map.at("VEL");
  s(S_INDEX(YAW)) = map.at("YAW");

  return s;
}