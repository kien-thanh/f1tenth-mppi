#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "vehicle_msgs/msg/vehicle_state.hpp"

using std::placeholders::_1;

class PathAugment : public rclcpp::Node
{
  public:
    PathAugment() : Node("path_augment")
    {
      unsmoothed_plan_sub_ = this->create_subscription<nav_msgs::msg::Path>("unsmoothed_plan", 10, std::bind(&PathAugment::unsmoothed_plan_received_callback_, this, _1));
      global_plan_pub_ = this->create_publisher<vehicle_msgs::msg::VehicleState>("global_plan", 10);
    }

  private:
    void unsmoothed_plan_received_callback_(const nav_msgs::msg::Path::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard a path with %zu poses", msg->poses.size());

      vehicle_msgs::msg::VehicleState plan;
      plan.header = msg->header;

      size_t n = msg->poses.size();
      plan.x.resize(n);
      plan.y.resize(n);
      plan.theta.resize(n);
      plan.v.resize(n, 2.0);     // Initialize all velocities to 2.0
      plan.delta.resize(n, 0.0); // Initialize all steering angles to 0

      for (size_t i = 0; i < n; ++i) {
        plan.x[i] = msg->poses[i].pose.position.x;
        plan.y[i] = msg->poses[i].pose.position.y;

        // Convert quaternion to yaw angle (theta) for 2D case
        plan.theta[i] = 2.0 * std::atan2(msg->poses[i].pose.orientation.z, msg->poses[i].pose.orientation.w);
      }

      global_plan_pub_->publish(plan);
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr unsmoothed_plan_sub_;
    rclcpp::Publisher<vehicle_msgs::msg::VehicleState>::SharedPtr global_plan_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathAugment>());
  rclcpp::shutdown();
  return 0;
}