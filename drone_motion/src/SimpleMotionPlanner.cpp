#include "drone_motion/SimpleMotionPlanner.h"

GoalPublisher::GoalPublisher() : Node("goal_publisher")
{
  publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/Goal", 10);

  float start_x = 0;
  float start_y = 0;
  float start_z = 1;

  float end_x = 3;
  float end_y = 4;
  float end_z = 5;



  for (int i = 0; i < 1000; ++i)
  {

    float t = static_cast<float>(i) / 1000.0f;

    geometry_msgs::msg::Point msg;
    msg.x = start_x + (end_x - start_x) * t;
    msg.y = start_y + (end_y - start_y) * t;
    msg.z = start_z + (end_z - start_z) * t * t;

    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Publishing Goal: [%.2f, %.2f, %.2f]", msg.x, msg.y, msg.z);
    
    rclcpp::sleep_for(std::chrono::milliseconds(50));  // brief delay to ensure publication
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoalPublisher>();
  rclcpp::shutdown();
  return 0;
}
