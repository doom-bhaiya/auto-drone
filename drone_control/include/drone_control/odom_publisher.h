#ifndef ODOM_TF_BROADCASTER_HPP_
#define ODOM_TF_BROADCASTER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class OdomTFBroadcaster : public rclcpp::Node
{
public:
  OdomTFBroadcaster();

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

#endif  // ODOM_TF_BROADCASTER_HPP_