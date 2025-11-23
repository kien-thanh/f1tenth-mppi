#pragma once

#include <singletrack_cost.cuh>

#include <rclcpp/rclcpp.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class ScanProcessor
{
public:
    explicit ScanProcessor(rclcpp::Node *const node, SingletrackCost &cost, tf2_ros::Buffer &tfBuffer);

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    void publishProcessedScan();

    void publishScanProcessingTime(float time_ms);

private:
    bool debug_ = true;

    rclcpp::Node *const node_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr scan_processing_time_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr scan_map_pub_;

    unsigned char map_width_;
    unsigned char map_height_;
    float map_resolution_;
    float map_origin_x_;
    float map_origin_y_;

    // Use flag to ensure we only process scans after receiving the map
    bool map_received_ = false;

    SingletrackCost &cost_;

    tf2_ros::Buffer &tfBuffer_;

    laser_geometry::LaserProjection projector_;
    std::string laser_frame_;

    // Use flag to create scan map only once
    bool receivedFirstScan_ = false;
};
