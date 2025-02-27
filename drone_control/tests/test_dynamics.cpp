#include <ros/ros.h>
#include <gtest/gtest.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <drone_msgs/PropellerVelocity.h>

#include <utils.h>

class DroneConfigTest : public ::testing::Test {
public:
Eigen::Vector4d speeds;

protected:

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher propeller_pub_;
    std::vector<double> config_matrix_;
    bool message_received_;
    std::string robot_name_;

    Drone drone;
    ros::Time last_time;


    Eigen::Matrix<double, 4, 3>  weights;

    bool passed;

    float threshold = 10000;



    DroneConfigTest() : nh_(), config_matrix_(12, 0.0), message_received_(false), robot_name_("my_robot") {
        sub_ = nh_.subscribe("/gazebo/model_states", 1, &DroneConfigTest::modelStatesCallback, this);
        propeller_pub_ = nh_.advertise<drone_msgs::PropellerVelocity>("/cmd_vel_three", 1);
        ros::Time last_time = ros::Time::now();
        weights << 1, 1, 1,
                   10, 10, 10,
                   10, 10, 10, 
                   10, 10, 10;
        speeds << 1000, 1000, 1000, 1000;
    }

    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        ROS_INFO_STREAM("Callback triggered");
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == robot_name_) {

                ros::Time current_time = ros::Time::now();
                ros::Duration dt_ms = current_time - last_time;
                double dt = dt_ms.toSec();

                const geometry_msgs::Pose& pose = msg->pose[i];
                const geometry_msgs::Twist& twist = msg->twist[i];

                tf::Quaternion quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
                tf::Matrix3x3 mat(quat);
                double roll, pitch, yaw;
                mat.getRPY(roll, pitch, yaw);

                Eigen::Matrix<double, 4, 3> new_state;

                new_state << pose.position.x, pose.position.y, pose.position.z,           
                         twist.linear.x, twist.linear.y, twist.linear.z,             
                         roll, pitch, yaw,                                                      
                         twist.angular.x, twist.angular.y, twist.angular.z;

                drone.prop_to_motion(speeds, dt);

                Eigen::Matrix<double, 4, 3> difference = new_state - drone.state;
                difference = difference.cwiseProduct(weights);  
            
                float total_error = difference.cwiseAbs().sum();


                if (total_error < threshold){
                    passed = true;
                }
                message_received_ = true;
                break;

            }
        }
    }

    void publishPropellerSpeeds() {
        passed = false;
        drone_msgs::PropellerVelocity msg;

        msg.prop1 = speeds(0);
        msg.prop2 = speeds(1);
        msg.prop3 = speeds(2);
        msg.prop4 = speeds(3);

        propeller_pub_.publish(msg);
        ros::Time last_time = ros::Time::now();
    }


};

TEST_F(DroneConfigTest, TestModelStatesSubscription) {
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Time start = ros::Time::now();
    while (!message_received_ && (ros::Time::now() - start).toSec() < 5.0) {
        ROS_INFO_STREAM("Waiting for message...");
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }
    
    ASSERT_TRUE(message_received_) << "Message not received within timeout";
    ROS_INFO_STREAM("Test passed, message received");
}

TEST_F(DroneConfigTest, UpwardAcceleration) {

    ros::AsyncSpinner spinner(1);
    spinner.start();

    Eigen::Vector4d speeds;
    speeds << 2000000, 2000000, 2000000, 2000000;
    publishPropellerSpeeds();

    ros::Duration(1.0).sleep();
    ASSERT_TRUE(passed) << "Error is greater";
    ROS_INFO_STREAM("Test passed, message received");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "drone_config_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
