#ifndef DRONE_MOTION__GOAL_PUBLISHER_HPP_
#define DRONE_MOTION__GOAL_PUBLISHER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"

class GoalPublisher : public rclcpp::Node
{
public:
  GoalPublisher();

private:
  void timer_callback();

  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // DRONE_MOTION__GOAL_PUBLISHER_HPP_
