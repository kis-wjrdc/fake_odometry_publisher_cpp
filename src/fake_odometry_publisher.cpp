#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

class FakeOdometryPublisher : public rclcpp::Node
{
public:
    FakeOdometryPublisher() : Node("fake_odometry_publisher_cpp")
    {
        // Initialize last_time_ to current ROS 2 time
        last_time_ = this->now();

        // Publisher for odometry messages
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        // Transform broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Timer for periodic publishing
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&FakeOdometryPublisher::publish, this));

        // Initialize variables
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
        vx_ = 0.1; // 0.1 m/s
        vy_ = 0.0;
        vtheta_ = 0.1; // 0.1 rad/s
    }

private:
    void publish()
    {
        // Get current time
        auto current_time = this->now();

        // Compute time difference
        double dt = (current_time - last_time_).seconds();

        // Compute odometry
        double delta_x = (vx_ * cos(theta_) - vy_ * sin(theta_)) * dt;
        double delta_y = (vx_ * sin(theta_) + vy_ * cos(theta_)) * dt;
        double delta_theta = vtheta_ * dt;

        // Update pose
        x_ += delta_x;
        y_ += delta_y;
        theta_ += delta_theta;

        // Publish the transform
        geometry_msgs::msg::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = x_;
        odom_trans.transform.translation.y = y_;
        odom_trans.transform.translation.z = 0.0;
        tf2::Quaternion quat;
        quat.setRPY(0, 0, theta_);
        odom_trans.transform.rotation.x = quat.x();
        odom_trans.transform.rotation.y = quat.y();
        odom_trans.transform.rotation.z = quat.z();
        odom_trans.transform.rotation.w = quat.w();
        tf_broadcaster_->sendTransform(odom_trans);

        // Publish the odometry message
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_trans.transform.rotation;
        odom.twist.twist.linear.x = vx_;
        odom.twist.twist.linear.y = vy_;
        odom.twist.twist.angular.z = vtheta_;
        odom_pub_->publish(odom);

        // Update last_time_
        last_time_ = current_time;
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_time_;

    double x_, y_, theta_;
    double vx_, vy_, vtheta_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FakeOdometryPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

