#include <ros/ros.h>
#include <gtest/gtest.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <drone_msgs/PropellerVelocity.h>
#include <Eigen/Dense>
#include <utils.h>
#include <std_srvs/Empty.h>

void pauseSimulation(ros::NodeHandle& nh) {
    ros::ServiceClient pause_client = nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    std_srvs::Empty srv;
    if (pause_client.call(srv)) {
        ROS_INFO("Simulation paused.");
    } else {
        ROS_ERROR("Failed to pause simulation.");
    }
}

void unpauseSimulation(ros::NodeHandle& nh) {
    ros::ServiceClient unpause_client = nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
    std_srvs::Empty srv;
    if (unpause_client.call(srv)) {
        ROS_INFO("Simulation unpaused.");
    } else {
        ROS_ERROR("Failed to unpause simulation.");
    }
}

void resetWorld(ros::NodeHandle& nh) {
    ros::ServiceClient reset_client = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
    std_srvs::Empty srv;
    if (reset_client.call(srv)) {
        ROS_INFO("World reset successful.");
    } else {
        ROS_ERROR("Failed to reset world.");
    }
}

class DroneConfigTest : public ::testing::Test {
protected:
    ros::NodeHandle nh_;
    ros::Publisher propeller_pub_;
    Drone drone_;
    std::string robot_name_;
    Eigen::Matrix<double, 4, 3> weights_;
    Eigen::Vector4d speeds_;
    const double threshold_ = 10;

    DroneConfigTest() : nh_(), drone_(nh_), robot_name_("my_robot") {
        propeller_pub_ = nh_.advertise<drone_msgs::PropellerVelocity>("/cmd_vel_three", 1);
        weights_ << 1, 1, 1,
                    2, 2, 2,
                    5, 5, 5,
                    5, 5, 5;
        drone_.state(0, 2) = 0.1;
    }

    void SetUp() override {
        ROS_INFO_STREAM("Waiting for initial model states message to confirm Gazebo is running...");
        unpauseSimulation(nh_);
        auto msg = ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/gazebo/model_states", nh_, ros::Duration(5.0));
        pauseSimulation(nh_);
        ASSERT_TRUE(msg) << "Timed out waiting for initial /gazebo/model_states message.";
        ROS_INFO_STREAM("Initial model states confirmed.");
    }

    void publishPropellerSpeeds() {
        drone_msgs::PropellerVelocity msg;
        msg.prop1 = speeds_(0);
        msg.prop2 = speeds_(1);
        msg.prop3 = speeds_(2);
        msg.prop4 = speeds_(3);

        ROS_INFO_STREAM("Publishing propeller speeds: " << speeds_.transpose());
        propeller_pub_.publish(msg);
    }

    bool getModelState(Eigen::Matrix<double, 4, 3>& state) {
        auto msg = ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/gazebo/model_states", nh_, ros::Duration(5.0));
        if (!msg) {
            ROS_ERROR("Failed to get /gazebo/model_states message!");
            return false;
        }

        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == robot_name_) {
                const geometry_msgs::Pose& pose = msg->pose[i];
                const geometry_msgs::Twist& twist = msg->twist[i];

                tf::Quaternion quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
                double roll, pitch, yaw;
                tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

                state << pose.position.x, pose.position.y, pose.position.z,
                         twist.linear.x, twist.linear.y, twist.linear.z,
                         roll, pitch, yaw,
                         twist.angular.x, twist.angular.y, twist.angular.z;

                return true;
            }
        }

        ROS_ERROR("Robot name '%s' not found in model states!", robot_name_.c_str());
        return false;
    }

    double runPropellerTest(const Eigen::Vector4d& prop_speeds, double duration_seconds) {

        resetWorld(nh_);
        unpauseSimulation(nh_);
    
        speeds_ = prop_speeds;
        publishPropellerSpeeds();
        publishPropellerSpeeds();
    
        ros::Time reference = ros::Time::now();
        ros::Time last_time = reference;
    
        while ((ros::Time::now() - reference).toSec() < duration_seconds) {
            ros::Time current_time = ros::Time::now();
            double dt = (current_time - last_time).toSec();
            last_time = current_time;
    
            drone_.prop_to_motion(speeds_, dt); 
        }
    

        Eigen::Matrix<double, 4, 3> state;
        if (!getModelState(state)) {
            ROS_ERROR_STREAM("Failed to get model state after command.");
            return std::numeric_limits<double>::max();  
        }
    
        Eigen::Matrix<double, 4, 3> error = (state - drone_.state).cwiseProduct(weights_);
        double total_error = error.cwiseAbs().sum();
    
        ROS_INFO_STREAM("State: \n" << state);
        ROS_INFO_STREAM("drone_.state: \n" << drone_.state);
        ROS_INFO_STREAM("Total error: " << total_error);

        pauseSimulation(nh_);
    
        return total_error;
    }
};

TEST_F(DroneConfigTest, Y_tilt) {
    ROS_INFO_STREAM("Starting UpwardAcceleration test");

    Eigen::Vector4d speeds(1600, 1900, 1900, 1600);
    double error = runPropellerTest(speeds, 2);

    ASSERT_LT(error, threshold_) << "Model state did not match expected upward motion.";
    ROS_INFO_STREAM("UpwardAcceleration test passed.");
}

TEST_F(DroneConfigTest, X_tilt) {
    ROS_INFO_STREAM("Starting UpwardAcceleration test");

    Eigen::Vector4d speeds(1900, 1900, 1600, 1600);
    double error = runPropellerTest(speeds, 2);

    ASSERT_LT(error, threshold_) << "Model state did not match expected upward motion.";
    ROS_INFO_STREAM("UpwardAcceleration test passed.");
}

TEST_F(DroneConfigTest, Rotation) {
    ROS_INFO_STREAM("Starting UpwardAcceleration test");

    Eigen::Vector4d speeds(1600, 1900, 1600, 1900);
    double error = runPropellerTest(speeds, 2);

    ASSERT_LT(error, threshold_) << "Model state did not match expected upward motion.";
    ROS_INFO_STREAM("UpwardAcceleration test passed.");
}

TEST_F(DroneConfigTest, UpwardAcceleration) {
    ROS_INFO_STREAM("Starting UpwardAcceleration test");

    Eigen::Vector4d speeds(1800, 1800, 1800, 1800);
    double error = runPropellerTest(speeds, 1);

    ASSERT_LT(error, threshold_) << "Model state did not match expected upward motion.";
    ROS_INFO_STREAM("UpwardAcceleration test passed.");
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "drone_dynamics_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
