#include <ros/ros.h>
#include <fstream>
#include <sstream>
#include <unordered_map>

bool loadParamsFromFile(const std::string& file_path, ros::NodeHandle& nh) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open params file: %s", file_path.c_str());
        return false;
    }

    std::unordered_map<std::string, double> params;
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string key;
        double value;
        if (std::getline(ss, key, '=') && (ss >> value)) {
            params[key] = value;
        }
    }

    // Set each parameter to ROS param server
    for (const auto& pair : params) {
        nh.setParam(pair.first, pair.second);
        ROS_INFO("Set param: %s = %f", pair.first.c_str(), pair.second);
    }

    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "param_loader_node");
    ros::NodeHandle nh("~");  // Use private namespace for cleaner params

    std::string param_file_path;
    if (!nh.getParam("param_file_path", param_file_path)) {
        ROS_ERROR("param_file_path not set! Please specify in launch file.");
        return 1;
    }

    if (!loadParamsFromFile(param_file_path, nh)) {
        return 1;
    }

    ROS_INFO("Parameter loading complete.");
    ros::spin();  // Or just return if you want it to exit immediately.
    return 0;
}