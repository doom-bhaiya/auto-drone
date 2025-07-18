#include "drone_control/PID_controller.h"

PIDController::PIDController() {}

PIDController::PIDController(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd), currentErr(0), prevErr(0), integralErr(0) {}

void PIDController::setParams(double kp, double ki, double kd){
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;

    PositiveIntegral = true;

    reset();
}

double PIDController::compute(double setpoint, double measured_value, double dt) {

    dt = std::max(dt, 1e-6); 

    currentErr = setpoint - measured_value;
    integralErr += currentErr * dt;
    double derivativeErr = (currentErr - prevErr) / dt;

    if (currentErr > 0) {
        PositiveIntegral = true; 
    } else if (currentErr < 0) {
        PositiveIntegral = false; 
    }

    if (PositiveIntegral && integralErr < 0) {
        integralErr = 0; 
    } else if (!PositiveIntegral && integralErr > 0) {
        integralErr = 0; 
    }

    double output = kp_ * currentErr + ki_ * integralErr + kd_ * derivativeErr;

    prevErr = currentErr;

    return output;
}

void PIDController::reset(){

    currentErr = 0;
    prevErr = 0;
    integralErr = 0;

}