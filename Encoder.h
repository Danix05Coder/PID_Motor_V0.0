#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

#define N_Encoders 8

#define DEFAULT_FILTER 1100  //460

class Encoder {
public: 
    byte A;     
    byte B;  
    volatile int pulses;
    volatile unsigned long T;
    unsigned long T_filter;

    Encoder();
    Encoder(byte a, byte b, float p_r);

    byte getInstanceCount();
    byte getId() const;
    
    void init(); 

    void setPulses(int p);
    int  getPulses() const;

    void setPeriod(unsigned long T);
    unsigned long getPeriod() const;

    void setFilter(unsigned long filter);
    unsigned long getFilter() const;

    void  setDegrees(float _degrees);
    float getDegrees() const;

private:
    static byte instanceCount;   
    byte id;
    float pul_rv;   
};

#endif
