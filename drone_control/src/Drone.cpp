#include "drone_control/Drone.h"


Drone::Drone(std::string config_file_path){

     logger_ = spdlog::get("DronePoseSubscriber_logger");
     if (!logger_) std::cerr << "Things work" << std::endl;

    file_path = config_file_path;
    set_params();

    PropSpeedToForce << kf_, kf_, kf_, kf_,
                        -kf_ * propLength_, 0, kf_ * propLength_, 0,
                        0, - kf_ * propLength_, 0, kf_ * propLength_,
                        km_, -km_, km_, -km_;


}

void Drone::set_params(){

    std::ifstream file(file_path);
    if (!file)
    {
        std::cerr << "Could not open config.json\n";
    }
    else
    {
        json config;
        file >> config;

        kf_ = config["k1"];
        km_ = config["k2"];
        mass_ = config["m"];
        // MomentOfInertia_ = config["Ip"];
        MomentOfInertia_ = 1;
        propLength_ = config["length_of_prop"];
    }
}

double Drone::calculateThrust(double errAltitude){

    if (errAltitude < - GRAVITY / 2) errAltitude = - GRAVITY / 2;

    float thrust = ((errAltitude + GRAVITY) * mass_) / (cos(curOrientation(0)) * cos(curOrientation(1)));

    if (thrust < 0) thrust = 0;
    logger_->trace("Thrust calculated : {}", thrust);
    return thrust;
    
}

double Drone::calculateAngulaMoment(double errLateral){

    float lateralMoment = errLateral * MomentOfInertia_;
    logger_->trace( "Lateral Moment calculated : {}", lateralMoment);
    return lateralMoment;
    
}

Eigen::Vector4f Drone::calculatePropSpeeds(double& u1, double& u2, double& u3, double& u4){

    Eigen::Vector4f u;

    if (u2){
        if (abs(u2) > abs(u1) / 30) u2 = u2 / abs(u2) * u1 / 40;
    }

    if (u3){
        if (abs(u3) > abs(u1) / 30) u3 = u3 / abs(u3) * u1 / 40;
    }

    if (u4){
        if (abs(u4) > abs(u1) / 30) u4 = u4 / abs(u4) * u1 / 40;
    }

    // if (u4){
        
    // }
    u << u1, u2, u3, u4;

    Eigen::Matrix4f Ainv;
Ainv << 
    1.0/(4.0*kf_),     -1.0/(2.0*kf_*propLength_),      0,                 1.0/(4.0*km_),
    1.0/(4.0*kf_),      0,                   -1.0/(2.0*kf_*propLength_),  -1.0/(4.0*km_),
    1.0/(4.0*kf_),      1.0/(2.0*kf_*propLength_),      0,                 1.0/(4.0*km_),
    1.0/(4.0*kf_),      0,                    1.0/(2.0*kf_*propLength_),  -1.0/(4.0*km_);


    Eigen::Vector4f PropSpeeds = Ainv *  u;
    PropSpeeds = PropSpeeds.array().sqrt();
    if (PropSpeeds.array().isNaN().any()){
        std::cout << " U : " << u.transpose() << std::endl;
        std::cout << "PropSpeeds: " << PropSpeeds.transpose() << std::endl;
    }
    return PropSpeeds;
}


void Drone::setOrientation(Eigen::Vector3d euler){
    curOrientation << euler;
}