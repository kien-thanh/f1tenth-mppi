#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


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
using PathMsg = nav_msgs::msg::Path;

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

class LocalTrajectoryPublisher : public rclcpp::Node
{
public:
  LocalTrajectoryPublisher() : Node("mppi_node"), tfBuffer_(this->get_clock()), tfListener_(tfBuffer_)
  {
    auto qos_distance_map = rclcpp::QoS(rclcpp::KeepLast(1));
    qos_distance_map.transient_local();
    qos_distance_map.reliable();

    drive_pub_ = this->create_publisher<DriveMsg>("/drive", 10);

    auto qos_trajectory = rclcpp::QoS(rclcpp::KeepLast(1));
    qos_trajectory.durability_volatile();
    qos_trajectory.best_effort();
    local_trajectory_pub_ = this->create_publisher<PathMsg>("/local_trajectory", qos_trajectory);

    distance_map_sub_ = this->create_subscription<DistanceFieldMsg>("/distance_map", qos_distance_map, [this](const DistanceFieldMsg::SharedPtr msg) {
      this->distanceMapCallback(msg);
    });

    odom_sub_ = this->create_subscription<OdomMsg>("/ego_racecar/odom", 10, [this](const OdomMsg::SharedPtr msg) {
      // this->odomCallback(msg);
    });
  }

private:
  rclcpp::Publisher<DriveMsg>::SharedPtr drive_pub_;
  rclcpp::Subscription<DistanceFieldMsg>::SharedPtr distance_map_sub_;
  rclcpp::Subscription<OdomMsg>::SharedPtr odom_sub_;
  rclcpp::Publisher<PathMsg>::SharedPtr local_trajectory_pub_;

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  DYN_T dynamics_;
  COST_T cost_;
  SAMPLING_T sampler_;
  CONTROLLER_PARAMS_T controller_params_;
  std::unique_ptr<FB_T> fb_controller_;
  std::unique_ptr<PLANT_T> plant_;
  std::shared_ptr<CONTROLLER_T> controller_;

  bool received_distance_map_ = false;

  void distanceMapCallback(const DistanceFieldMsg::SharedPtr msg)
  {
    std::cout << "Received distance map of size: " << msg->data.size() << std::endl;

    float dt = 0.02;
    cost_.createDistanceField(msg->data.data(), msg->resolution, msg->width, msg->height, msg->origin_x, msg->origin_y);

    fb_controller_ = std::make_unique<FB_T>(&dynamics_, dt); // set up feedback controller

    auto sampler_params = sampler_.getParams();
    std::fill(sampler_params.std_dev, sampler_params.std_dev + DYN_T::CONTROL_DIM, 1.0);
    sampler_.setParams(sampler_params);

    // set up MPPI Controller
    controller_params_.dt_ = dt;
    controller_params_.lambda_ = 1.0;
    controller_params_.dynamics_rollout_dim_ = dim3(DYN_BLOCK_X, DYN_BLOCK_Y, 1);
    controller_params_.cost_rollout_dim_ = dim3(96, 1, 1);
    controller_ = std::make_shared<CONTROLLER_T>(&dynamics_, &cost_, fb_controller_.get(), &sampler_, controller_params_);

    // Create plant
    plant_ = std::make_unique<PLANT_T>(controller_, (1.0 / dt), 1);

    received_distance_map_ = true;

    std::atomic<bool> alive(true);
    for (int t = 0; t < 2000; t++)
    {
      plant_->updateState(plant_->current_state_, (t + 1) * dt);
      plant_->runControlIteration(&alive);
      ackermann_msgs::msg::AckermannDriveStamped msg;
      msg.drive.speed = plant_->current_state_(3);
      msg.drive.steering_angle = plant_->current_state_(4);
      drive_pub_->publish(msg);

      std::this_thread::sleep_for(std::chrono::milliseconds(20)); // wait 0.02s

      publishLocalTrajectory();
    }

    std::cout << "Avg Optimization time: " << plant_->getAvgOptimizationTime() << " ms" << std::endl;
    std::cout << "Last Optimization time: " << plant_->getLastOptimizationTime() << " ms" << std::endl;
    std::cout << "Avg Loop time: " << plant_->getAvgLoopTime() << " ms" << std::endl;
    std::cout << "Avg Optimization Hz: " << 1.0 / (plant_->getAvgOptimizationTime() * 1e-3) << " Hz" << std::endl;

    auto control_sequence = controller_->getControlSeq();
    std::cout << "State: \n"
              << plant_->current_state_.transpose() << std::endl;
    std::cout << "Control Sequence:\n"
              << control_sequence << std::endl;
    // Allocate result of transformation in device memory    
    // float* output;
    // cudaMalloc(&output, sizeof(float));
    // cudaFree(output);
  }

  // void odomCallback(const OdomMsg::SharedPtr msg)
  // {
  //   if (!received_distance_map_)
  //   {
  //     return;
  //   }

  //   geometry_msgs::msg::TransformStamped transformStamped;
  //   try {
  //     transformStamped = tfBuffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
  //   }
  //   catch (tf2::TransformException &ex) {
  //     RCLCPP_WARN(this->get_logger(), "Could not transform base_link to map: %s", ex.what());
  //     return;
  //   }

  //   std::atomic<bool> alive(true);

  //   Eigen::Matrix<float, 5, 1> current_state;

  //   current_state(0) = transformStamped.transform.translation.x;
  //   current_state(1) = transformStamped.transform.translation.y;

  //   if (plant_->current_state_(2) > 0.47)
  //   {
  //     current_state(2) = 0.47;
  //   }
  //   else if (plant_->current_state_(2) < -0.47)
  //   {
  //     current_state(2) = -0.47;
  //   }
  //   else
  //   {
  //     current_state(2) = plant_->current_state_(2);
  //   }

  //   double vx = msg->twist.twist.linear.x;
  //   double vy = msg->twist.twist.linear.y;
  //   current_state(3) = std::sqrt(vx*vx + vy*vy);

  //   tf2::Quaternion q;
  //   tf2::fromMsg(msg->pose.pose.orientation, q); // convert geometry_msgs -> tf2
  //   double roll, pitch, yaw;
  //   tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  //   current_state(4) = yaw;

  //   plant_->updateState(current_state, msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
  //   plant_->runControlIteration(&alive);

  //   ackermann_msgs::msg::AckermannDriveStamped driveMsg;
  //   driveMsg.drive.speed = plant_->current_state_(3);
  //   driveMsg.drive.steering_angle = plant_->current_state_(4);
  //   drive_pub_->publish(driveMsg);

  // }
  void publishLocalTrajectory()
  {
    Eigen::Matrix<float, 5, NUM_TIMESTEPS> state_traj = plant_->getStateTraj();
    PathMsg path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "map";
    for (int i = 0; i < state_traj.cols(); i++)
    {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header = path_msg.header;
      pose_stamped.pose.position.x = state_traj(0, i);
      pose_stamped.pose.position.y = state_traj(1, i);
      pose_stamped.pose.position.z = 0.0;

      path_msg.poses.push_back(pose_stamped);
    }

    local_trajectory_pub_->publish(path_msg);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalTrajectoryPublisher>());
  rclcpp::shutdown();
  return 0;
}
