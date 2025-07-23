#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <algorithm>
#include <iostream>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rclcpp/rclcpp.hpp"

#include "nlohmann/json.hpp"
using json = nlohmann::json;

#include "spdlog/spdlog.h"

class PIDController {
public:

    PIDController(std::string file_path, std::string controller_name);
    void setParams();
    double compute(double setpoint, double measured_value, double dt);
    void reset();
    void resetKi();

private:

    std::string file_path_;
    std::string controllerName_;

    std::shared_ptr<spdlog::logger> logger_;

    double kp_;
    double ki_;
    double kd_;

    double currentErr;
    double prevErr;
    double integralErr;

    bool PositiveIntegral;
};


#endif 
