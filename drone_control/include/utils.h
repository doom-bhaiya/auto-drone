#ifndef UTILS
#define UTILS

#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <ros/ros.h>



class Drone{
    private:
            float mass;
            float g;
            float length_of_prop;
            Eigen::Matrix3d Inertial_matrix;
            double kf;
            double km;
            ros::NodeHandle nh_;
    
    public:
    
        Eigen::Matrix<double, 4, 3> state;
        Drone(ros::NodeHandle& nh);
    
    private:
    
        Eigen::Matrix3d getRotationMatrix() const;
    
        Eigen::Vector3d get_linear_Acceleration(double u1) const;
        Eigen::Vector3d get_angular_Acceleration(double u2, double u3, double u4) const;
    
        void update_translational(Eigen::Vector3d linear_Acceleration, float dt);
        void update_rotational(Eigen::Vector3d angular_Acceleration, float dt);
    
    public:
    
        void prop_to_motion(Eigen::Vector4d prop_speeds, float dt);
        Eigen::Vector4d inverse_dynamics(
            const Eigen::Vector3d& desired_linear_acceleration,
            const Eigen::Vector3d& desired_angular_acceleration);
    };
    
    #endif