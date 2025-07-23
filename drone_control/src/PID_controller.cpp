#include "drone_control/PID_controller.h"

PIDController::PIDController(std::string config_file_path, std::string name){
    file_path_ = config_file_path;
    controllerName_ = name;

    logger_ = spdlog::get("DronePoseSubscriber_logger");
    logger_->info("Initializing PID Controller: {}", controllerName_);
    logger_->info("File path for PID params : {}", file_path_);
    setParams();
    reset();

}

void PIDController::setParams(){
    

    std::ifstream paramsFile(file_path_);
    if (!paramsFile)
    {
        std::cerr << "Could not open config.json\n";
    }
    else
    {
        json config;
        paramsFile >> config;

        kp_ = config[controllerName_]["kp"];
        ki_ = config[controllerName_]["ki"];
        kd_ = config[controllerName_]["kd"];
    }

    PositiveIntegral = true;

    reset();
}

double PIDController::compute(double setpoint, double measured_value, double dt) {

    // dt = std::max(dt, 1e-6); 

    currentErr = setpoint - measured_value;
    integralErr += currentErr * dt;
    double derivativeErr = (currentErr - prevErr) / dt;

    logger_->trace("Output of P of {} PID : {}", controllerName_, kp_ * currentErr);
    logger_->trace("Output of I of {} PID : {}", controllerName_, ki_ * integralErr);
    logger_->trace("Output of D of {} PID : {}", controllerName_, kd_ * derivativeErr);


    if (currentErr > 0) PositiveIntegral = true; 
    else if (currentErr < 0) PositiveIntegral = false;       

    if (PositiveIntegral && integralErr < 0) {
        integralErr = 0; 
        logger_->info("Resetting Integral of PID : {} PID", controllerName_);
    } else if (!PositiveIntegral && integralErr > 0) {
        integralErr = 0; 
        logger_->info("Resetting Integral of PID : {} PID", controllerName_);
    }

    double output = kp_ * currentErr + ki_ * integralErr + kd_ * derivativeErr;
    logger_->trace("Output of {} PID : {}", controllerName_, output);

    prevErr = currentErr;



    return output;
}

void PIDController::reset(){

    logger_->info("Resetting PID : {} PID", controllerName_);

    currentErr = 0;
    prevErr = 0;
    integralErr = 0;

}