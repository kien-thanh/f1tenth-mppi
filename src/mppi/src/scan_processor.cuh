#pragma once

#include <singletrack_cost.cuh>

#include <laser_geometry/laser_geometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

class ScanProcessor
{
public:
    explicit ScanProcessor(SingletrackCost &cost, tf2_ros::Buffer &tfBuffer) : cost_(cost), tfBuffer_(tfBuffer) {};

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

private:
    SingletrackCost &cost_;
    tf2_ros::Buffer &tfBuffer_;
    bool receivedFirstScan_ = false;
    laser_geometry::LaserProjection projector_;
};
