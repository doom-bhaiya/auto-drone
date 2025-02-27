#include <utils.h>

Drone::Drone(){

    Inertial_matrix << 1, 0, 0,
                        0, 1, 0,
                        0, 0, 1;

    state <<    0, 0, 0,
                0, 0, 0,
                0, 0, 0,
                0, 0, 0;

    mass = 1;
    g = 9.81;
    length_of_prop = 1;
    kf = 0.000001;
    km = 0.0000001;

}

Eigen::Matrix3d Drone::getRotationMatrix() const {

    double theta = state(2, 0);
    double phi = state(2, 1);
    double omega = state(2, 2);

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

Eigen::Vector3d Drone::get_linear_Acceleration(float u1) const {

    Eigen::Vector3d linear_Acceleration;

    Eigen::Vector3d  g_vector;
    Eigen::Vector3d  prop_force;
    Eigen::Matrix3d rot_mat;

    g_vector << 0, 0, -1 * g * mass;
    prop_force << 0, 0, u1;
    rot_mat << getRotationMatrix();

    linear_Acceleration << g_vector + rot_mat * prop_force;
    return linear_Acceleration;
}

Eigen::Vector3d Drone::get_angular_Acceleration(float u2, float u3, float u4) const {

    Eigen::Vector3d angular_acceleration;

    Eigen::Vector3d moment;
    Eigen::Vector3d body_rate;

    moment << u2, u3, u4;
    body_rate << state(3, 0), state(3, 1), state(3, 2);

    angular_acceleration << Inertial_matrix.inverse() * (moment - body_rate.cross(Inertial_matrix * body_rate));
    return angular_acceleration;
}

void Drone::update_translational(Eigen::Vector3d linear_Acceleration, float dt){

    state.row(0) += state.row(1) * dt;
    state.row(1) += linear_Acceleration * dt;
}

void Drone::update_rotational(Eigen::Vector3d angular_Acceleration, float dt){
    
    Eigen::Matrix3d rot_mat;
    rot_mat = getRotationMatrix();


    state.row(2) += (rot_mat * state.row(3).transpose()) * dt;
    state.row(3) += angular_Acceleration * dt;
}

void Drone::prop_to_motion(Eigen::Vector4d prop_speeds, float dt){

    float f1 = kf * prop_speeds(0) * prop_speeds(0);
    float f2 = kf * prop_speeds(1) * prop_speeds(1);
    float f3 = kf * prop_speeds(2) * prop_speeds(2);
    float f4 = kf * prop_speeds(3) * prop_speeds(3);

    float m1 = km * prop_speeds(0) * prop_speeds(0);
    float m2 = km * prop_speeds(1) * prop_speeds(1);
    float m3 = km * prop_speeds(2) * prop_speeds(2);
    float m4 = km * prop_speeds(3) * prop_speeds(3);

    float u1 = f1 + f2 + f3 + f4;
    float u2 = length_of_prop * (f3 - f1);
    float u3 = length_of_prop * (f4 - f2);
    float u4 = m1 - m2 + m3 - m4;

    Eigen::Vector3d linear_acceleration = get_linear_Acceleration(u1);
    Eigen::Vector3d angular_acceleration = get_angular_Acceleration(u2, u3, u4);

    update_translational(linear_acceleration, dt);
    update_rotational(angular_acceleration, dt);
}

