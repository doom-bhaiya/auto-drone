#include <utils.h>
#include <cmath>

Drone::Drone(ros::NodeHandle& nh) : nh_(nh) {

    Inertial_matrix << 1, 0, 0,
                       0, 1, 0,
                       0, 0, 1;

    state <<    0, 0, 0,
                0, 0, 0,
                0, 0, 0,
                0, 0, 0;

    g = 9.8;

    nh.getParam("/param_loader/k1", kf);
    nh.getParam("/param_loader/k2", km);
    nh.getParam("/param_loader/m", mass);
    nh.getParam("/param_loader/length_of_prop", length_of_prop);
}

// Helper function to wrap angles to [-pi, pi)
double wrapAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle <= -M_PI) angle += 2.0 * M_PI;
    return angle;
}

Eigen::Matrix3d Drone::getRotationMatrix() const {
    double theta = state(2, 0); // Roll
    double phi   = state(2, 1); // Pitch
    double omega = state(2, 2); // Yaw

    Eigen::Matrix3d R_x;
    R_x << 1, 0, 0,
           0, cos(theta), -sin(theta),
           0, sin(theta), cos(theta);

    Eigen::Matrix3d R_y;
    R_y << cos(phi), 0, sin(phi),
           0, 1, 0,
          -sin(phi), 0, cos(phi);

    Eigen::Matrix3d R_z;
    R_z << cos(omega), -sin(omega), 0,
           sin(omega), cos(omega), 0,
           0, 0, 1;

    return R_z * R_y * R_x;
}

Eigen::Vector3d Drone::get_linear_Acceleration(double u1) const {
    Eigen::Vector3d linear_Acceleration;
    Eigen::Vector3d g_vector(0, 0, -g * mass);
    Eigen::Vector3d prop_force(0, 0, u1);

    Eigen::Matrix3d rot_mat = getRotationMatrix();
    linear_Acceleration = (g_vector + rot_mat * prop_force) / mass;
    return linear_Acceleration;
}

Eigen::Vector3d Drone::get_angular_Acceleration(double u2, double u3, double u4) const {
    Eigen::Vector3d moment(u2, u3, u4);
    Eigen::Vector3d body_rate(state(3, 0), state(3, 1), state(3, 2));

    return Inertial_matrix.inverse() * (moment - body_rate.cross(Inertial_matrix * body_rate));
}

void Drone::update_translational(Eigen::Vector3d linear_Acceleration, float dt) {
    state.row(0) += state.row(1) * dt;           // Position update
    state.row(1) += linear_Acceleration * dt;    // Linear velocity update
}

void Drone::update_rotational(Eigen::Vector3d angular_Acceleration, float dt) {
    Eigen::Matrix3d rot_mat = getRotationMatrix();

    state.row(2) += (rot_mat * state.row(3).transpose()) * dt;  // Orientation update
    state.row(3) += angular_Acceleration * dt;                   // Angular velocity update

    for (int i = 0; i < 3; ++i) {
        state(2, i) = wrapAngle(state(2, i));
    }
}

void Drone::prop_to_motion(Eigen::Vector4d prop_speeds, float dt) {
    double f1 = kf * prop_speeds(0) * prop_speeds(0);
    double f2 = kf * prop_speeds(1) * prop_speeds(1);
    double f3 = kf * prop_speeds(2) * prop_speeds(2);
    double f4 = kf * prop_speeds(3) * prop_speeds(3);

    double m1 = km * prop_speeds(0) * prop_speeds(0);
    double m2 = km * prop_speeds(1) * prop_speeds(1);
    double m3 = km * prop_speeds(2) * prop_speeds(2);
    double m4 = km * prop_speeds(3) * prop_speeds(3);

    double u1 = f1 + f2 + f3 + f4;
    double u2 = length_of_prop * (f3 - f1);
    double u3 = length_of_prop * (f4 - f2);
    double u4 = m1 - m2 + m3 - m4;

    Eigen::Vector3d linear_acceleration = get_linear_Acceleration(u1);
    Eigen::Vector3d angular_acceleration = get_angular_Acceleration(u2, u3, u4);

    update_translational(linear_acceleration, dt);
    update_rotational(angular_acceleration, dt);
}

Eigen::Vector4d Drone::inverse_dynamics(
    const Eigen::Vector3d& desired_linear_acceleration,
    const Eigen::Vector3d& desired_angular_acceleration) {

    double u1 = mass * (g + desired_linear_acceleration(2));

    Eigen::Vector3d body_rate(state(3, 0), state(3, 1), state(3, 2));

    Eigen::Vector3d required_moments = Inertial_matrix * desired_angular_acceleration
                                       + body_rate.cross(Inertial_matrix * body_rate);

    double u2 = required_moments(0);
    double u3 = required_moments(1);
    double u4 = required_moments(2);

    Eigen::Vector4d desired_u;
    desired_u << u1, u2, u3, u4;

    Eigen::Matrix4d A;
    A <<  kf,  kf,  kf,  kf,
         -length_of_prop * kf, 0,  length_of_prop * kf,  0,
         0,  -length_of_prop * kf,  0, length_of_prop * kf,
         km,  -km, km,  -km;

    Eigen::Vector4d forces = A.inverse() * desired_u;

    Eigen::Vector4d prop_speeds;
    ROS_INFO_STREAM(desired_u);
    ROS_INFO_STREAM(forces);
    for (int i = 0; i < 4; ++i) {
        // if (forces(i) < 0) forces(i) = 0;
        prop_speeds(i) = std::sqrt(forces(i) / kf);
    }
    ROS_INFO_STREAM(prop_speeds);

    return prop_speeds;
}