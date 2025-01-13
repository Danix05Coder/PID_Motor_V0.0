#ifndef PID_H
#define PID_H

#include "Arduino.h"

class PID {
public:
    PID(double kp, double ki, double kd, unsigned long sampling_time, double max_out, double error_tolerance);

    double compute  (double _input, double set_point);
    double compute_2(double _input, double set_point);  
    bool isStable() const;                           

    void setKp(double kp);
    void setKi(double ki);
    void setKd(double kd);
    void setSamplingTime(unsigned long sampling_time);
    void setMaxOut(double max_out);
    void setErrorTolerance(double error_tolerance);
    void Print();
    void Print(bool p_error , bool p_errorIntegral, bool p_errorDerivative, bool p_input, bool p_output);
    void processCommand(String command);

private:
  
    double kp;
    double ki;
    double kd;
    unsigned long sampling_time;
    double max_out;          
    double error_tolerance;  

    double error; 
    double errorIntegral;
    double errorDerivative;
    unsigned long last_time;                                              
    double        prev_error;   
    double input;
    double output;
    bool   stable;

    bool last_Print_error;         
    bool last_Print_errorIntegral;    
    bool last_Print_errorDerivative;  
    bool last_Print_input;            
    bool last_Print_output;

    int    numberParameter(String command, int paramIndex);
};

#endif
