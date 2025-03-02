#include <gtest/gtest.h>
#include <ros/ros.h>
#include "utils.h"
#include <random>

// Helper function to generate random numbers within a range
double getRandom(double min, double max) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(min, max);
    return dis(gen);
}

TEST(DroneInverseDynamicsTest, InverseDynamicsAccuracy) {
    ros::NodeHandle nh;
    Drone drone(nh);
    
    // Generate random test inputs
    double random_linear_acc_z = getRandom(0, 2.0);  // Linear acceleration only in z
    Eigen::Vector3d desired_linear_acc(0.0, 0.0, random_linear_acc_z);
    Eigen::Vector3d desired_angular_acc(getRandom(-1.0, 1.0), getRandom(-1.0, 1.0), getRandom(-1.0, 1.0));
    
    // Compute the propeller speeds
    Eigen::Vector4d prop_speeds = drone.inverse_dynamics(desired_linear_acc, desired_angular_acc);

    // Get initial state
    Eigen::Matrix<double, 4, 3> initial_state = drone.state;
    
    float dt = 0.1;

    drone.prop_to_motion(prop_speeds, dt);


    // Get final state
    Eigen::Matrix<double, 4, 3> final_state = drone.state;
    
    // Compute actual acceleration based on state change
    Eigen::Vector3d actual_linear_acc = (final_state.row(1) - initial_state.row(1)).transpose() / (dt);
    Eigen::Vector3d actual_angular_acc = (final_state.row(3) - initial_state.row(3)).transpose() / (dt);
    
    // Check the difference
    double lin_acc_error = (actual_linear_acc - desired_linear_acc).norm();
    double ang_acc_error = (actual_angular_acc - desired_angular_acc).norm();

    ROS_INFO_STREAM("actual_linear_acc : " << lin_acc_error);
    ROS_INFO_STREAM("actual_angular_acc : " << ang_acc_error);
    
    EXPECT_LT(lin_acc_error, 0.1) << "Linear acceleration error too high: " << lin_acc_error;
    EXPECT_LT(ang_acc_error, 0.1) << "Angular acceleration error too high: " << ang_acc_error;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gtest_inverse_dynamics");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
