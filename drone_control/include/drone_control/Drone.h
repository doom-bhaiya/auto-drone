#ifndef DRONE_H
#define DRONE_H

#include <iostream>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "nlohmann/json.hpp"
using json = nlohmann::json;

#include "spdlog/spdlog.h"

#define GRAVITY 9.8

class Drone{
public:

    Drone(std::string file_path);
    void set_params();
    double calculateThrust(double errAltitude);
    double calculateAngulaMoment(double errLateral);
    Eigen::Vector4f calculatePropSpeeds(double& u1, double& u2, double& u3, double& u4);
    void setOrientation(Eigen::Vector3d euler);


private:

    std::string file_path;
    std::string controllerName_;

    std::shared_ptr<spdlog::logger> logger_;
    
    double kf_;
    double km_;
    double mass_;
    double propLength_;
    double MomentOfInertia_;

    Eigen::Vector3d curOrientation;
    Eigen::Matrix4f PropSpeedToForce; 



};


#endif
