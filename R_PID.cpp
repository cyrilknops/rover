#include "Arduino.h"
#include <math.h>
#include "R_PID.h"

R_PID::R_PID(double Kp, double Kd, double Ki, double maxi, double mini )
{
    _Kp = Kp;
    _Kd = Kd;
    _Ki = Ki;
    _maxi = maxi;
    _mini = mini;
}

R_PID::~R_PID() 
{
}

double R_PID::calculate( double setpoint, double pv )
{
    _proportional = ((int)pv - setpoint);
    
    _derivative = _proportional - _last_proportional;
    _integral = _integral+_proportional;

    _last_proportional = _proportional;
    // use the tutorial to set initial values of Kp, Ki, and Kd

    _power_difference = _proportional*_Kp + _integral*_Ki + _derivative*_Kd;
    if(_power_difference > _maxi)
     _power_difference = _maxi;
    if(_power_difference < _mini)
     _power_difference = _mini;

     return _power_difference;
       
}
