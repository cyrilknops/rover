class R_PID
{
    public:
        // Kp -  proportional gain
        // Ki -  Integral gain
        // Kd -  derivative gain
        // dt -  loop interval time
        // max - maximum value of manipulated variable
        // min - minimum value of manipulated variable
        R_PID(double Kp, double Kd, double Ki, double maxi, double mini );
        // Returns the manipulated variable given a setpoint and current process value
        double calculate( double setpoint, double pv );
        ~R_PID();

    private:
        double _Kp;
        double _Kd;
        double _Ki;
        double _maxi;
        double _mini;
        int _derivative;
        int _proportional;
        int _integral;
        int _last_proportional;
        int _power_difference;
};
