#ifndef UTILS
#define UTILS

#include <Eigen/Dense>

class Drone{
    private:
            float mass;
            float g;
            float length_of_prop;
            Eigen::Matrix3d Inertial_matrix;
            float kf;
            float km;
    
    public:
    
        Eigen::Matrix<double, 4, 3> state;
        Drone();
    
    private:
    
        Eigen::Matrix3d getRotationMatrix() const;
    
        Eigen::Vector3d get_linear_Acceleration(float u1) const;
        Eigen::Vector3d get_angular_Acceleration(float u2, float u3, float u4) const;
    
        void update_translational(Eigen::Vector3d linear_Acceleration, float dt);
        void update_rotational(Eigen::Vector3d angular_Acceleration, float dt);
    
    public:
    
        void prop_to_motion(Eigen::Vector4d prop_speeds, float dt);
    };
    
    #endif