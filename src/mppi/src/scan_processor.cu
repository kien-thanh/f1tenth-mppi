#include "scan_processor.cuh"

ScanProcessor::ScanProcessor(rclcpp::Node *const node, SingletrackCost &cost, tf2_ros::Buffer &tfBuffer) : node_(node), cost_(cost), tfBuffer_(tfBuffer)
{
  node_->declare_parameter("real_laser_frame", "laser");
  node_->declare_parameter("sim_laser_frame", "ego_racecar/laser");

  laser_frame_ = node_->get_parameter("sim").as_bool()
                     ? node_->get_parameter("sim_laser_frame").as_string()
                     : node_->get_parameter("real_laser_frame").as_string();

  RCLCPP_INFO(node_->get_logger(), "ScanProcessor initialized with laser frame: %s", laser_frame_.c_str());

  scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",
      10,
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg)
      {
        scanCallback(msg);
      });

  map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map",
      rclcpp::QoS(1).transient_local(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
      {
        map_width_ = msg->info.width;
        map_height_ = msg->info.height;
        map_resolution_ = msg->info.resolution;
        map_origin_x_ = msg->info.origin.position.x;
        map_origin_y_ = msg->info.origin.position.y;

        map_received_ = true;
      });

  scan_processing_time_pub_ = node_->create_publisher<std_msgs::msg::Float32>(
    "debug/scan_processing/time_ms",
    rclcpp::QoS(1).best_effort());

  scan_map_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "debug/scan_processing/scan_map",
    rclcpp::QoS(1).best_effort());
}

void ScanProcessor::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  auto start = std::chrono::steady_clock::now();

  if (!map_received_)
  {
    RCLCPP_WARN(node_->get_logger(), "Map not received yet, skipping scan processing.");
    return;
  }

  if (!receivedFirstScan_)
  {
    RCLCPP_INFO(node_->get_logger(), "Received first scan, allocating space for scan points.");
    scan_points_in_map_frame_.reserve(msg->ranges.size());
  }

  sensor_msgs::msg::PointCloud2 cloud;

  geometry_msgs::msg::TransformStamped transformStamped;
  try
  {
    transformStamped = tfBuffer_.lookupTransform("map", laser_frame_, tf2::TimePointZero);
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN(node_->get_logger(), "Could not transform laser to map: %s", ex.what());
    return;
  }

  projector_.projectLaser(*msg, cloud);

  tf2::doTransform(cloud, cloud, transformStamped);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");

  scan_points_in_map_frame_.clear();
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y)
  {
    float2 point;
    point.x = *iter_x;
    point.y = *iter_y;
    scan_points_in_map_frame_.push_back(point);
  }

  if (!receivedFirstScan_)
  {
    cost_.createScanMap(map_resolution_, map_width_, map_height_, map_origin_x_, map_origin_y_);
    receivedFirstScan_ = true;
  }

  cost_.updateScanMap(scan_points_in_map_frame_);

  if (debug_)
  {
    publishProcessedScan();
  }

  float duration_ms = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - start).count();

  std_msgs::msg::Float32 msg_out;
  msg_out.data = duration_ms;
  scan_processing_time_pub_->publish(msg_out);
}



void ScanProcessor::publishProcessedScan()
{
  std::vector<unsigned char> scan_map_data;
  cost_.getScanMapData(scan_map_data, map_width_, map_height_);

  nav_msgs::msg::OccupancyGrid grid;
  grid.header.stamp = node_->now();
  grid.header.frame_id = "map";
  grid.info.resolution = map_resolution_;
  grid.info.width = map_width_;
  grid.info.height = map_height_;
  grid.info.origin.position.x = map_origin_x_;
  grid.info.origin.position.y = map_origin_y_;
  grid.info.origin.position.z = 0.0f;
  grid.info.origin.orientation.w = 1.0f;

  grid.data.resize(scan_map_data.size());
  for (size_t i = 0; i < scan_map_data.size(); i++)
  {
    grid.data[i] = static_cast<int8_t>(scan_map_data[i]);
  }

  scan_map_pub_->publish(grid);
}