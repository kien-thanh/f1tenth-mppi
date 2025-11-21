#include <cstdio>

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
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <mppi/controllers/MPPI/mppi_controller.cuh>
#include <mppi/cost_functions/quadratic_cost/quadratic_cost.cuh>
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
    this->declare_parameter("sim", false);             // whether to run in simulation mode

    // Initialize publishers
    auto qos_texture = rclcpp::QoS(rclcpp::KeepLast(1));
    qos_texture.transient_local();
    qos_texture.reliable();
    drive_pub_ = this->create_publisher<DriveMsg>("/drive", 10);

    auto qos_trajectory = rclcpp::QoS(rclcpp::KeepLast(1));
    qos_trajectory.durability_volatile();
    qos_trajectory.best_effort();
    local_trajectory_pub_ = this->create_publisher<PathMsg>("/local_trajectory", qos_trajectory);

    // Initialize subscribers
    distance_map_sub_ = this->create_subscription<DistanceFieldMsg>("/distance_map", qos_texture, [this](const DistanceFieldMsg::SharedPtr msg)
                                                                    { this->distanceMapCallback(msg); });

    costmap_sub_ = this->create_subscription<OccupancyGridMsg>("/global_costmap/costmap", qos_texture, [this](const OccupancyGridMsg::SharedPtr msg)
                                                               { this->costmapCallback(msg); });

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, [this](const sensor_msgs::msg::LaserScan::SharedPtr msg)
                                                                       { this->scanCallback(msg); });

    if (this->get_parameter("sim").as_bool())
    {
      odom_sub_ = this->create_subscription<OdomMsg>("/odom", 10, [this](const OdomMsg::SharedPtr msg)
                                                     { this->odomCallback(msg); });
    }
    else
    {
      pose_sub_ = this->create_subscription<PoseMsg>("/amcl_pose", 10, [this](const PoseMsg::SharedPtr msg)
                                                     { this->poseCallback(msg); });
    }
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
  rclcpp::Subscription<OccupancyGridMsg>::SharedPtr costmap_sub_;
  rclcpp::Subscription<LaserScanMsg>::SharedPtr scan_sub_;
  rclcpp::Subscription<OdomMsg>::SharedPtr odom_sub_;
  rclcpp::Subscription<PoseMsg>::SharedPtr pose_sub_;
  rclcpp::Publisher<PathMsg>::SharedPtr local_trajectory_pub_;

  double current_velocity_ = 0.0;

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
  bool receivedCostmap_ = false;
  bool initializedMPPI_ = false;
  bool receivedFirstScan_ = false;

  void distanceMapCallback(const DistanceFieldMsg::SharedPtr msg)
  {
    std::cout << "Received distance map of size: " << msg->data.size() << std::endl;

    cost_.createDistanceField(msg->data.data(), msg->resolution, msg->width, msg->height, msg->origin_x, msg->origin_y);

    receivedDistanceMap_ = true;

    if (receivedCostmap_ && !initializedMPPI_)
    {
      initMPPI();
    }
  }

  void costmapCallback(const OccupancyGridMsg::SharedPtr msg)
  {
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

    if (receivedDistanceMap_ && !initializedMPPI_)
    {
      initMPPI();
    }
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

    ackermann_msgs::msg::AckermannDriveStamped driveMsg;
    driveMsg.drive.speed = plant_->current_state_(3);
    driveMsg.drive.steering_angle = plant_->current_state_(2);
    drive_pub_->publish(driveMsg);

    publishLocalTrajectory();
  }

  void poseCallback(const PoseMsg::SharedPtr msg)
  {
    if (!initializedMPPI_)
    {
      return;
    }

    std::atomic<bool> alive(true);

    Eigen::Matrix<float, 5, 1> current_state;

    current_state(0) = msg->pose.pose.position.x;
    current_state(1) = msg->pose.pose.position.y;

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

    current_state(3) = current_velocity_;

    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q); // convert geometry_msgs -> tf2
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    current_state(4) = yaw;

    plant_->updateState(current_state, msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
    plant_->runControlIteration(&alive);

    ackermann_msgs::msg::AckermannDriveStamped driveMsg;
    driveMsg.drive.speed = plant_->current_state_(3);
    driveMsg.drive.steering_angle = plant_->current_state_(2);
    drive_pub_->publish(driveMsg);

    publishLocalTrajectory();
  }

  void scanCallback(const LaserScanMsg::SharedPtr msg)
  {
    // if (!initializedMPPI_)
    // {
    //   return;
    // }

    static laser_geometry::LaserProjection projector;
    sensor_msgs::msg::PointCloud2 cloud;

    geometry_msgs::msg::TransformStamped transformStamped;
    try {
      transformStamped = tfBuffer_.lookupTransform("map", "laser", tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "Could not transform laser to map: %s", ex.what());
      return;
    }

    projector.projectLaser(*msg, cloud);

    tf2::doTransform(cloud, cloud, transformStamped);

    std::vector<float2> h_points;

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y)
    {
      float2 point;
      point.x = *iter_x;
      point.y = *iter_y;
      h_points.push_back(point);
    }

    if (!receivedFirstScan_)
    {
      cost_.createScanMap(0.05f, 130, 113, 0.387, -4.08);
      receivedFirstScan_ = true;
    }

    cost_.updateScanMap(h_points);
  }

  void initMPPI()
  {
    float dt = 0.02;

    // Set cost parameters
    SingletrackCostParams cost_params;
    cost_params.c_distance = this->get_parameter("c_distance").as_double();
    cost_params.c_velocity = this->get_parameter("c_velocity").as_double();
    cost_params.desired_velocity = this->get_parameter("desired_velocity").as_double();
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
      pose_stamped.pose.position.z = 0.0;

      path_msg.poses.push_back(pose_stamped);
    }

    local_trajectory_pub_->publish(path_msg);
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
