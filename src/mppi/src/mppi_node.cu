#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <mppi/controllers/MPPI/mppi_controller.cuh>
#include <mppi/cost_functions/quadratic_cost/quadratic_cost.cuh>
#include <mppi/feedback_controllers/DDP/ddp.cuh>
#include <singletrack_dynamics.cuh>
#include <singletrack_cost.cuh>
#include <plant.hpp>

#include <interfaces/msg/distance_map.hpp>

using DriveMsg = ackermann_msgs::msg::AckermannDriveStamped;
using DistanceFieldMsg = interfaces::msg::DistanceMap;
using OdomMsg = nav_msgs::msg::Odometry;

using DYN_T = SingletrackDynamics;
using COST_T = SingletrackCost;

const int NUM_TIMESTEPS = 100;
const int NUM_ROLLOUTS = 1024;
const int DYN_BLOCK_X = 64;
const int DYN_BLOCK_Y = DYN_T::STATE_DIM;

using FB_T = DDPFeedback<DYN_T, NUM_TIMESTEPS>;
using SAMPLING_T = mppi::sampling_distributions::GaussianDistribution<DYN_T::DYN_PARAMS_T>;
using CONTROLLER_T = VanillaMPPIController<DYN_T, COST_T, FB_T, NUM_TIMESTEPS, NUM_ROLLOUTS, SAMPLING_T>;
using CONTROLLER_PARAMS_T = CONTROLLER_T::TEMPLATED_PARAMS;
using PLANT_T = SingletrackPlant<CONTROLLER_T>;

__global__ void myKernel(float* output, cudaTextureObject_t texObj, float x , float y, float resolution)
{
  // Calculate texture coordinates
  float offset = resolution / 2.0f;

  x = (x + 1.240737762184536f + offset) / resolution;
  y = (y + 0.9999977108091116f + offset) / resolution;

  *output = tex2D<float>(texObj, x, y);
}

class LocalTrajectoryPublisher : public rclcpp::Node
{
public:
  LocalTrajectoryPublisher() : Node("mppi_node")
  {
    auto qos_distance_map = rclcpp::QoS(rclcpp::KeepLast(1));
    qos_distance_map.transient_local();
    qos_distance_map.reliable();

    drive_pub_ = this->create_publisher<DriveMsg>("/drive", 10);
    distance_map_sub_ = this->create_subscription<DistanceFieldMsg>("/distance_map", 10, [this](const DistanceFieldMsg::SharedPtr msg) {
      this->distanceMapCallback(msg);
    });
    odom_sub_ = this->create_subscription<OdomMsg>("/odom", qos_distance_map, [this](const OdomMsg::SharedPtr msg) {
      this->odomCallback(msg);
    });
  }

private:
  rclcpp::Publisher<DriveMsg>::SharedPtr drive_pub_;
  rclcpp::Subscription<DistanceFieldMsg>::SharedPtr distance_map_sub_;
  rclcpp::Subscription<OdomMsg>::SharedPtr odom_sub_;

  void distanceMapCallback(const DistanceFieldMsg::SharedPtr msg)
  {
    std::cout << "Received distance map of size: " << msg->data.size() << std::endl;

    float dt = 0.02;
    DYN_T dynamics;                    // set up dynamics
    COST_T cost;                       // set up cost
    cost.createDistanceField(msg->data.data(), msg->resolution, msg->width, msg->height, msg->origin_x, msg->origin_y);
    FB_T fb_controller(&dynamics, dt); // set up feedback controller
    // set up sampling distribution
    SAMPLING_T sampler;
    auto sampler_params = sampler.getParams();
    std::fill(sampler_params.std_dev, sampler_params.std_dev + DYN_T::CONTROL_DIM, 1.0);
    sampler.setParams(sampler_params);

    // set up MPPI Controller
    CONTROLLER_PARAMS_T controller_params;
    controller_params.dt_ = dt;
    controller_params.lambda_ = 1.0;
    controller_params.dynamics_rollout_dim_ = dim3(DYN_BLOCK_X, DYN_BLOCK_Y, 1);
    controller_params.cost_rollout_dim_ = dim3(96, 1, 1);
    std::shared_ptr<CONTROLLER_T> controller =
        std::make_shared<CONTROLLER_T>(&dynamics, &cost, &fb_controller, &sampler, controller_params);

    // Create plant
    PLANT_T plant(controller, (1.0 / dt), 1);

    std::atomic<bool> alive(true);
    for (int t = 0; t < 2000; t++)
    {
      plant.updateState(plant.current_state_, (t + 1) * dt);
      plant.runControlIteration(&alive);

      // std::cout << "State: at time " << (t + 1) * dt << "s: " << plant.current_state_.transpose() << std::endl;

      ackermann_msgs::msg::AckermannDriveStamped msg;
      msg.drive.speed = plant.current_state_(3);
      msg.drive.steering_angle = plant.current_state_(4);
      drive_pub_->publish(msg);

      std::this_thread::sleep_for(std::chrono::milliseconds(20)); // wait 0.02s
    }

    std::cout << "Avg Optimization time: " << plant.getAvgOptimizationTime() << " ms" << std::endl;
    std::cout << "Last Optimization time: " << plant.getLastOptimizationTime() << " ms" << std::endl;
    std::cout << "Avg Loop time: " << plant.getAvgLoopTime() << " ms" << std::endl;
    std::cout << "Avg Optimization Hz: " << 1.0 / (plant.getAvgOptimizationTime() * 1e-3) << " Hz" << std::endl;

    auto control_sequence = controller->getControlSeq();
    std::cout << "State: \n"
              << plant.current_state_.transpose() << std::endl;
    std::cout << "Control Sequence:\n"
              << control_sequence << std::endl;
    // Allocate result of transformation in device memory    
    // float* output;
    // cudaMalloc(&output, sizeof(float));
    // cudaFree(output);
  }

  void odomCallback(const OdomMsg::SharedPtr msg)
  {
    std::cout << "Received odometry: position (" << msg->pose.pose.position.x << ", " << msg->pose.pose.position.y << "), orientation ("
              << msg->pose.pose.orientation.x << ", " << msg->pose.pose.orientation.y << ", "
              << msg->pose.pose.orientation.z << ", " << msg->pose.pose.orientation.w << ")" << std::endl;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalTrajectoryPublisher>());
  rclcpp::shutdown();
  return 0;
}
