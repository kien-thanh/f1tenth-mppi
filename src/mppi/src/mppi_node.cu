#include <cstdio>
#include <algorithm>

#include "scan_processor.cuh"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <mppi/controllers/MPPI/mppi_controller.cuh>
#include <mppi/feedback_controllers/DDP/ddp.cuh>
#include <singletrack_dynamics.cuh>
#include <singletrack_cost.cuh>
#include <plant.hpp>

#include <interfaces/msg/distance_map.hpp>

using DriveMsg = ackermann_msgs::msg::AckermannDriveStamped;
using DistanceFieldMsg = interfaces::msg::DistanceMap;
using OccupancyGridMsg = nav_msgs::msg::OccupancyGrid;
using OdomMsg = nav_msgs::msg::Odometry;
using PathMsg = nav_msgs::msg::Path;
using LaserScanMsg = sensor_msgs::msg::LaserScan;
using PoseMsg = geometry_msgs::msg::PoseWithCovarianceStamped;

using DYN_T = SingletrackDynamics;
using COST_T = SingletrackCost;

const int NUM_TIMESTEPS = 64;
const int NUM_ROLLOUTS = 2048;
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
    // Initialize parameters
    this->declare_parameter("c_distance", 10.0f);      // cost coefficient for distance to centerline
    this->declare_parameter("c_velocity", 10.0f);      // cost coefficient for velocity tracking
    this->declare_parameter("desired_velocity", 2.0f); // desired velocity
    this->declare_parameter("c_costmap", 1.0f);       // cost coefficient for costmap
    this->declare_parameter("c_angle", 5.0f);         // cost coefficient for angle alignment
    this->declare_parameter("sim", false);             // whether to run in simulation mode
    this->declare_parameter("v_max", 3.0f);            // maximum velocity
    this->declare_parameter("delay_constant", 1.0f);   // constant delay term
    this->declare_parameter("delay_factor", 0.0f);     // delay factor
    this->declare_parameter("c_latacc", 1.0f);        // cost coefficient for lateral acceleration

    scan_processor_ = std::make_unique<ScanProcessor>(this, cost_, tfBuffer_);

    // Initialize publishers
    drive_pub_ = this->create_publisher<DriveMsg>(
        "/drive",
        10);

    local_trajectory_pub_ = this->create_publisher<PathMsg>(
        "/local_trajectory",
        rclcpp::QoS(1).durability_volatile().best_effort());

    debug_drive_unclamped_pub_ = this->create_publisher<DriveMsg>(
        "/debug/drive",
        10);

    debug_state_pub_ = this->create_publisher<PoseMsg>(
        "/debug/mppi_state",
        10);

    // Initialize subscribers
    distance_map_sub_ = this->create_subscription<DistanceFieldMsg>(
        "/distance_map",
        rclcpp::QoS(1).transient_local().reliable(),
        [this](const DistanceFieldMsg::SharedPtr msg)
        {
          this->distanceMapCallback(msg);
        });

    angle_grid_sub_ = this->create_subscription<DistanceFieldMsg>(
        "/angle_grid",
        rclcpp::QoS(1).transient_local().reliable(),
        [this](const DistanceFieldMsg::SharedPtr msg)
        {
          this->angleGridCallback(msg);
        });

    costmap_sub_ = this->create_subscription<OccupancyGridMsg>(
        "/global_costmap/costmap",
        rclcpp::QoS(1).transient_local().reliable(),
        [this](const OccupancyGridMsg::SharedPtr msg)
        {
          this->costmapCallback(msg);
        });

    if (this->get_parameter("sim").as_bool())
    {
      odom_sub_ = this->create_subscription<OdomMsg>(
          "/ego_racecar/odom",
          10,
          [this](const OdomMsg::SharedPtr msg)
          {
            this->odomCallback(msg);
          });
    }
    else
    {
      pose_sub_ = this->create_subscription<PoseMsg>(
          "/amcl_pose",
          10,
          [this](const PoseMsg::SharedPtr msg)
          {
            this->poseCallback(msg);
          });

      real_odom_sub_ = this->create_subscription<OdomMsg>(
          "/odom",
          10,
          [this](const OdomMsg::SharedPtr msg)
          {
            this->realOdomCallback(msg);
          });
    }
    
    steering_angle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/steering_angle",
        10,
        [this](const std_msgs::msg::Float32::SharedPtr msg)
        {
          this->steeringAngleCallback(msg);
        });

    mppi_init_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]()
        {
          if (!initializedMPPI_ && receivedDistanceMap_ && receivedAngleGrid_ && receivedCostmap_ && scan_processor_->receivedFirstScan())
          {
            RCLCPP_INFO(this->get_logger(), "Initializing MPPI...");
            initMPPI();
            mppi_init_timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "MPPI Initialized.");
          }
        });
  }

  ~LocalTrajectoryPublisher()
  {
    std::cout << "Avg Optimization time: " << plant_->getAvgOptimizationTime() << " ms" << std::endl;
    std::cout << "Last Optimization time: " << plant_->getLastOptimizationTime() << " ms" << std::endl;
    std::cout << "Avg Loop time: " << plant_->getAvgLoopTime() << " ms" << std::endl;
    std::cout << "Avg Optimization Hz: " << 1.0 / (plant_->getAvgOptimizationTime() * 1e-3) << " Hz" << std::endl;
  }

private:
  rclcpp::Publisher<DriveMsg>::SharedPtr drive_pub_;
  rclcpp::Subscription<DistanceFieldMsg>::SharedPtr distance_map_sub_;
  rclcpp::Subscription<DistanceFieldMsg>::SharedPtr angle_grid_sub_;
  rclcpp::Subscription<OccupancyGridMsg>::SharedPtr costmap_sub_;
  rclcpp::Subscription<OdomMsg>::SharedPtr real_odom_sub_;
  rclcpp::Subscription<OdomMsg>::SharedPtr odom_sub_;
  rclcpp::Subscription<PoseMsg>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steering_angle_sub_;
  rclcpp::Publisher<PathMsg>::SharedPtr local_trajectory_pub_;
  rclcpp::Publisher<DriveMsg>::SharedPtr debug_drive_unclamped_pub_;
  rclcpp::Publisher<PoseMsg>::SharedPtr debug_state_pub_;

  rclcpp::TimerBase::SharedPtr mppi_init_timer_;

  std::unique_ptr<ScanProcessor> scan_processor_;

  float current_velocity_ = 0.0f;
  float current_steering_angle_ = 0.0f;

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  DYN_T dynamics_;
  COST_T cost_;
  SAMPLING_T sampler_;
  CONTROLLER_PARAMS_T controller_params_;
  std::unique_ptr<FB_T> fb_controller_;
  std::unique_ptr<PLANT_T> plant_;
  std::shared_ptr<CONTROLLER_T> controller_;

  bool receivedDistanceMap_ = false;
  bool receivedAngleGrid_ = false;
  bool receivedCostmap_ = false;
  bool initializedMPPI_ = false;

  void distanceMapCallback(const DistanceFieldMsg::SharedPtr msg)
  {
    std::cout << "Received distance map of size: " << msg->data.size() << std::endl;

    cost_.createDistanceField(msg->data.data(), msg->resolution, msg->width, msg->height, msg->origin_x, msg->origin_y);

    receivedDistanceMap_ = true;
  }

  void angleGridCallback(const DistanceFieldMsg::SharedPtr msg)
  {
    std::cout << "Received angle grid of size: " << msg->data.size() << std::endl;

    cost_.createAngleGrid(msg->data.data(), msg->resolution, msg->width, msg->height, msg->origin_x, msg->origin_y);

    receivedAngleGrid_ = true;
  }

  void costmapCallback(const OccupancyGridMsg::SharedPtr msg)
  {
    if (initializedMPPI_)
    {
      return;
    }

    // Wait 2 seconds
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Convert occupancy data from int8 to float
    std::vector<float> occupancy_data;
    occupancy_data.reserve(msg->data.size());
    for (auto v : msg->data)
    {
      occupancy_data.push_back(static_cast<float>(v));
    }

    printf("Received occupancy grid of size: %zu\n", msg->data.size());

    cost_.createCostmap(
        occupancy_data.data(),
        msg->info.resolution,
        msg->info.width,
        msg->info.height,
        msg->info.origin.position.x,
        msg->info.origin.position.y);

    receivedCostmap_ = true;
  }

  void odomCallback(const OdomMsg::SharedPtr msg)
  {
    if (!initializedMPPI_)
    {
      return;
    }

    geometry_msgs::msg::TransformStamped transformStamped;
    try
    {
      transformStamped = tfBuffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "Could not transform base_link to map: %s", ex.what());
      return;
    }

    std::atomic<bool> alive(true);

    Eigen::Matrix<float, 5, 1> current_state;

    current_state(0) = transformStamped.transform.translation.x;
    current_state(1) = transformStamped.transform.translation.y;

    if (plant_->current_state_(2) > 0.47)
    {
      current_state(2) = 0.47;
    }
    else if (plant_->current_state_(2) < -0.47)
    {
      current_state(2) = -0.47;
    }
    else
    {
      current_state(2) = plant_->current_state_(2);
    }

    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    current_state(3) = std::sqrt(vx * vx + vy * vy);

    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q); // convert geometry_msgs -> tf2
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    current_state(4) = yaw;

    plant_->updateState(current_state, msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
    plant_->runControlIteration(&alive);

    auto x = plant_->getStateTraj()(0, 0);
    auto y = plant_->getStateTraj()(1, 0);
    auto steering_angle = plant_->getStateTraj()(2, 3);
    auto velocity = plant_->getStateTraj()(3, 3);
    auto yaw_after = plant_->getStateTraj()(4, 0);
    publishDebugState(x, y, yaw_after);

    publishDrive(velocity, steering_angle);

    publishLocalTrajectory();
  }

  void realOdomCallback(const OdomMsg::SharedPtr msg)
  {
    float vx = msg->twist.twist.linear.x;
    float vy = msg->twist.twist.linear.y;
    current_velocity_ = std::sqrt(vx * vx + vy * vy);
  }

  void steeringAngleCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    float servo_position = msg->data;
    current_steering_angle_ = (servo_position - 0.511f) / -0.851f; // steering angle = servo value (0 to 1) / gain
  }

  void poseCallback(const PoseMsg::SharedPtr msg)
  {
    if (!initializedMPPI_)
    {
      return;
    }

    rclcpp::Duration delay = now() - msg->header.stamp;

    std::atomic<bool> alive(true);

    Eigen::Matrix<float, 5, 1> current_state;

    float x = msg->pose.pose.position.x;
    float y = msg->pose.pose.position.y;
    float v = current_velocity_;

    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q); // convert geometry_msgs -> tf2
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // forward projection
    float delay_constant = this->get_parameter("delay_constant").as_double();
    float delay_factor = this->get_parameter("delay_factor").as_double();
    float delay_total = delay_constant + delay_factor * delay.seconds();

    x += v * std::cos(yaw) * delay_total;
    y += v * std::sin(yaw) * delay_total;

    current_state(0) = x;
    current_state(1) = y;
    current_state(2) = plant_->getStateTraj()(2, 1);
    current_state(3) = v;
    current_state(4) = yaw;

    plant_->updateState(current_state, now().seconds());
    plant_->runControlIteration(&alive);
    x = plant_->getStateTraj()(0, 0);
    y = plant_->getStateTraj()(1, 0);
    auto steering_angle = plant_->getStateTraj()(2, 3);
    auto velocity = plant_->getStateTraj()(3, 3);
    auto yaw_after = plant_->getStateTraj()(4, 0);

    publishDebugState(x, y, yaw_after);

    publishDrive(velocity, steering_angle);

    publishLocalTrajectory();
  }

  void initMPPI()
  {
    float dt = 0.02;

    // Set cost parameters
    SingletrackCostParams cost_params;
    cost_params.c_distance = this->get_parameter("c_distance").as_double();
    cost_params.c_velocity = this->get_parameter("c_velocity").as_double();
    cost_params.desired_velocity = this->get_parameter("desired_velocity").as_double();
    cost_params.c_costmap = this->get_parameter("c_costmap").as_double();
    cost_params.c_angle = this->get_parameter("c_angle").as_double();
    cost_params.c_latacc = this->get_parameter("c_latacc").as_double();

    RCLCPP_INFO(this->get_logger(), "Setting cost parameters: c_distance=%.2f, c_velocity=%.2f, desired_velocity=%.2f, c_costmap=%.2f, c_angle=%.2f, c_latacc=%.2f",
                cost_params.c_distance, cost_params.c_velocity, cost_params.desired_velocity, cost_params.c_costmap, cost_params.c_angle, cost_params.c_latacc);

    cost_.setParams(cost_params);

    fb_controller_ = std::make_unique<FB_T>(&dynamics_, dt); // set up feedback controller

    auto sampler_params = sampler_.getParams();
    std::fill(sampler_params.std_dev, sampler_params.std_dev + DYN_T::CONTROL_DIM, 1.0);
    sampler_.setParams(sampler_params);

    // set up MPPI Controller
    controller_params_.dt_ = dt;
    controller_params_.lambda_ = 1.0;
    controller_params_.dynamics_rollout_dim_ = dim3(DYN_BLOCK_X, DYN_BLOCK_Y, 1);
    controller_params_.cost_rollout_dim_ = dim3(64, 1, 1);
    controller_ = std::make_shared<CONTROLLER_T>(&dynamics_, &cost_, fb_controller_.get(), &sampler_, controller_params_);

    // Create plant
    plant_ = std::make_unique<PLANT_T>(controller_, (1.0 / dt), 1);

    initializedMPPI_ = true;
  }

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
      pose_stamped.pose.position.z = state_traj(3, i);

      path_msg.poses.push_back(pose_stamped);
    }

    local_trajectory_pub_->publish(path_msg);
  }

  void publishDrive(float velocity, float steering_angle)
  {
    const float v_max = this->get_parameter("v_max").as_double();
    const float steering_angle_min = -0.44f;
    const float steering_angle_max = 0.44f;

    float velocity_limited = std::clamp(velocity, 0.0f, v_max);
    float steering_angle_limited = std::clamp(steering_angle, steering_angle_min, steering_angle_max);

    // Publish clamped drive messages for actual drive
    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = now();
    drive_msg.drive.speed = velocity_limited;
    drive_msg.drive.steering_angle = steering_angle_limited;
    drive_pub_->publish(drive_msg);

    // Publish unclamped drive messages for debugging
    ackermann_msgs::msg::AckermannDriveStamped debug_drive_msg;
    debug_drive_msg.header.stamp = now();
    debug_drive_msg.drive.speed = velocity;
    debug_drive_msg.drive.steering_angle = steering_angle;
    debug_drive_unclamped_pub_->publish(debug_drive_msg);
  }

  void publishDebugState(float x, float y, float yaw)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped state_msg;
    state_msg.header.stamp = now();
    state_msg.header.frame_id = "map";
    state_msg.pose.pose.position.x = x;
    state_msg.pose.pose.position.y = y;
    state_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    state_msg.pose.pose.orientation = tf2::toMsg(q);

    debug_state_pub_->publish(state_msg);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<LocalTrajectoryPublisher>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
