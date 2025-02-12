#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <drone_msgs/PropellerVelocity.h>

const std::string ROBOT_NAME = "my_robot";  // Replace with your actual drone name
const double TARGET_ALTITUDE = 5.0;

// PID gains
const double KP = 1000.0;  
const double KI = 2;
const double KD = 50.0;

double prev_error = 0.0;
double integral = 0.0;
ros::Time last_time;

ros::Publisher propeller_pub;

void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    auto it = std::find(msg->name.begin(), msg->name.end(), ROBOT_NAME);

    if (it != msg->name.end()) {
        int index = std::distance(msg->name.begin(), it);
        double z_position = msg->pose[index].position.z;

        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();
        last_time = current_time;

        if (dt <= 0) return;  // Prevent division by zero

        // PID control calculations
        double error = TARGET_ALTITUDE - z_position;
        integral += error * dt;
        double derivative = (error - prev_error) / dt;
        double thrust = (KP * error) + (KI * integral) + (KD * derivative);
        prev_error = error;

        thrust = std::max(0.0, std::min(thrust, 3000.0));  // Limit thrust

        drone_msgs::PropellerVelocity prop_msg;
        prop_msg.prop1 = thrust;
        prop_msg.prop2 = thrust;
        prop_msg.prop3 = thrust;
        prop_msg.prop4 = thrust;

        propeller_pub.publish(prop_msg);

        ROS_INFO("Altitude: %.2f, Thrust: %.2f, Error: %.2f", z_position, thrust, error);
    } else {
        ROS_WARN("Drone not found in /gazebo/model_states");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "hover_pid_controller");
    ros::NodeHandle nh;

    propeller_pub = nh.advertise<drone_msgs::PropellerVelocity>("/cmd_vel_three", 10);
    ros::Subscriber sub = nh.subscribe("/gazebo/model_states", 10, modelStatesCallback);

    last_time = ros::Time::now();

    ros::spin();
    return 0;
}
