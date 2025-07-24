#include "drone_control/odom_publisher.h"

OdomTFBroadcaster::OdomTFBroadcaster()
: Node("odom_tf_broadcaster")
{
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/DronePose", 10,
    std::bind(&OdomTFBroadcaster::odom_callback, this, std::placeholders::_1));
}

void OdomTFBroadcaster::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  geometry_msgs::msg::TransformStamped tf_msg;

  tf_msg.header.stamp = msg->header.stamp;
  tf_msg.header.frame_id = msg->header.frame_id;       // e.g., "odom"
  tf_msg.child_frame_id = msg->child_frame_id;         // e.g., "base_link"

  tf_msg.transform.translation.x = msg->pose.pose.position.x;
  tf_msg.transform.translation.y = msg->pose.pose.position.y;
  tf_msg.transform.translation.z = msg->pose.pose.position.z;

  tf_msg.transform.rotation = msg->pose.pose.orientation;

  tf_broadcaster_->sendTransform(tf_msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomTFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
