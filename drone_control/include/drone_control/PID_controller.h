#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <algorithm>

class PIDController {
public:
    PIDController();
    PIDController(double kp, double ki, double kd);
    void setParams(double kp, double ki, double kd);
    double compute(double setpoint, double measured_value, double dt);
    void reset();
    void resetKi();

private:
    double kp_;
    double ki_;
    double kd_;

    double currentErr;
    double prevErr;
    double integralErr;

    bool PositiveIntegral;
};


#endif 
