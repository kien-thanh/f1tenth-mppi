#include "scan_processor.cuh"

void ScanProcessor::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    sensor_msgs::msg::PointCloud2 cloud;

    geometry_msgs::msg::TransformStamped transformStamped;
    try {
      transformStamped = tfBuffer_.lookupTransform("map", "laser", tf2::TimePointZero);
      // RCLCPP_INFO(this->get_logger(), "Transform found from laser to map");
    }
    catch (tf2::TransformException &ex)
    {
      std::cout << "Could not transform laser to map: " << ex.what() << std::endl;
      return;
    }

    projector_.projectLaser(*msg, cloud);

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