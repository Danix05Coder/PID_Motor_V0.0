#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H

#define DEFAULT_PWM_CHANNEL    0         
#define DEFAULT_PWM_FREQ       5000      
#define DEFAULT_PWM_RESOLUTION 8  
#define DEFAULT_SAMPLING_TIME  100000

#include <Arduino.h>
#include "Encoder.h"

class MotorEncoder {
public:
    MotorEncoder(byte dir, byte en, byte encoderPinA, byte encoderPinB, float pul_Rv);
    void  init();
    #ifdef ESP32
    void  configureESP32PWM   (byte channel, int frequency, int resolution);
    void  configureESP32PWM_2 (byte channel, int frequency, int resolution);
    #endif
    void  setEncoderFilter_T(unsigned long period_filter);
    void  setSamplingTime(unsigned long Sam_Time);
    long  getPulses();
    long  getEncoderPeriod();
    float getDegrees();
    float getSpeed();
    void  turn  (int vel);
    void  turn_2(int vel); 
    void  Stop  ();
    void  Stop_2();
    void  Print(bool print_pulses , bool print_period, bool print_degrees, bool print_Speed);
    void  Print();
    void  resetEncoderData(); 
    void  processMotorCommand(String command);

private:
    byte dirPin;
    byte enPin;
    Encoder encoder; 
    float pul_rv;
    unsigned long samplingTime;
    byte pwm_channel;
    bool view;
    String stringParameter(String command, int paramIndex);
    int    numberParameter(String command, int paramIndex);
    
    long          lastPulseCount=0; 
    float         speedR_S=0;       
    unsigned long lastUpdate=0;    
    
    bool lastPrintPulses;
    bool lastPrintPeriod;
    bool lastPrintDegrees;
    bool lastPrintSpeed;
};

#endif
