#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"

using std::placeholders::_1;

class PathAugment : public rclcpp::Node
{
  public:
    PathAugment() : Node("path_augment")
    {
      subscription_ = this->create_subscription<nav_msgs::msg::Path>("plan", 10, std::bind(&PathAugment::topic_callback, this, _1));
    }

  private:
    void topic_callback(const nav_msgs::msg::Path::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard a path with %zu poses", msg->poses.size());
    }
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathAugment>());
  rclcpp::shutdown();
  return 0;
}