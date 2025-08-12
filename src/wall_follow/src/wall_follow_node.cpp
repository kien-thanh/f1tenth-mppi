#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"
#include <string>
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "wall_follow/msg/distance.hpp"
#include "wall_follow/msg/pid.hpp"
#include "wall_follow/msg/steering.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "control_toolbox/pid.hpp"

class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node("wall_follow_node"), pid_()
    {
        drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&WallFollow::scan_callback, this, std::placeholders::_1));
        distance_pub_ = create_publisher<std_msgs::msg::Float64>("/distance", 10);
        steering_pub_ = create_publisher<std_msgs::msg::Float64>("/steering", 10);

        kp_ = declare_parameter("kp", 1.0);
        ki_ = declare_parameter("ki", 0.0);
        kd_ = declare_parameter("kd", 0.0);

        desired_distance_ = declare_parameter("desired_distance", 0.5);

        pid_.initPid(kp_, ki_, kd_, 1.0, -1.0, false);
        last_t = this->now();

        param_cb_handle_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &params) {
            for (const auto &param : params) {
                if (param.get_name() == "kp") {
                    kp_ = param.get_value<double>();
                } else if (param.get_name() == "ki") {
                    ki_ = param.get_value<double>();
                } else if (param.get_name() == "kd") {
                    kd_ = param.get_value<double>();
                } else if (param.get_name() == "desired_distance") {
                    desired_distance_ = param.get_value<double>();
                }
            }
            pid_.setGains({kp_, ki_, kd_, 1.0, -1.0, false});
            RCLCPP_INFO(get_logger(), "PID updated: Kp=%.3f Ki=%.3f Kd=%.3f", kp_, ki_, kd_);
            
            return rcl_interfaces::msg::SetParametersResult().set__successful(true);
        });
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_pub_;

    OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    double kp_;
    double kd_;
    double ki_;
    double desired_distance_;

    control_toolbox::Pid pid_;

    rclcpp::Time last_t;

    double get_range(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, double angle)
    {
        /*
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
        */

        float min_angle = scan_msg->angle_min;
        float angle_increment = scan_msg->angle_increment;

        int index = static_cast<int>((angle - min_angle) / angle_increment);

        float range = scan_msg->ranges[index];

        return range;
    }

    double get_error(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, double dist)
    {
        /*
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        */

        auto a = get_range(scan_msg, deg2rad(45.0));
        auto b = get_range(scan_msg, deg2rad(90.0));

        auto alpha = atan((a * cos(deg2rad(45.0)) - b) / (a * sin(deg2rad(45.0))));

        auto D_t = b * cos(alpha);

        auto D_t1 = D_t + 0.5 * sin(alpha);

        auto distance_msg = std_msgs::msg::Float64();
        distance_msg.data = D_t;
        distance_pub_->publish(distance_msg);

        return dist - D_t1;
    }

    float deg2rad(float angle)
    {
        return angle * M_PI / 180.0;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /*
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        */

        rclcpp::Time t = this->now();
        auto dt = t - last_t;
        last_t = t;

        auto error = get_error(scan_msg, 0.5);

        auto steering_angle = pid_.computeCommand(error, dt.nanoseconds());

        auto steering_msg = std_msgs::msg::Float64();
        steering_msg.data = steering_angle;
        steering_pub_->publish(steering_msg);
        
        if (steering_angle > 0.47) {
            steering_angle = 0.47; // Limit steering angle to 45 degrees
        } else if (steering_angle < -0.47) {
            steering_angle = -0.47; // Limit steering angle to -45 degrees
        }


        double velocity = 0.5;

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = this->now();
        drive_msg.drive.speed = velocity;
        drive_msg.drive.steering_angle = -steering_angle;
        drive_pub_->publish(drive_msg);

    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}