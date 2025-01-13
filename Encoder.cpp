#include "Encoder.h"

Encoder* encoders[N_Encoders];  // Inicializa todos los encoders

byte Encoder::instanceCount = 0;

#define new_interrupts(i)                                                                             \
static void pulses##i() {                                                                             \
    bool sA = digitalRead(encoders[i]->A);                                                            \
    bool sB = digitalRead(encoders[i]->B);                                                            \
    unsigned long currentTime = micros();                                                             \
    static unsigned long T_pas[N_Encoders] = {0};                                                     \
    encoders[i]->T = currentTime - T_pas[i];                                                          \
    if (encoders[i]->T >= encoders[i]->T_filter) {                                                    \
        encoders[i]->pulses += (sA == sB) ? -1 : 1;                                                   \
        T_pas[i] = currentTime;                                                                       \
    }                                                                                                 \
}


new_interrupts(0)
new_interrupts(1)
new_interrupts(2)
new_interrupts(3)
new_interrupts(4)
new_interrupts(5)
new_interrupts(6)
new_interrupts(7)

void (*interruptFunctions[N_Encoders])() = {
    pulses0, pulses1, pulses2, pulses3,pulses4, pulses5, pulses6, pulses7
};


Encoder::Encoder(): pulses(0), T(0), T_filter(0), A(0), B(0), pul_rv(0){}

Encoder::Encoder(byte a, byte b, float p_r) :  A(a), B(b), pul_rv(p_r), T_filter(DEFAULT_FILTER), pulses(0), T(0) {
    id = instanceCount;
    encoders[id] = this;
    instanceCount++; 
}

void Encoder::init(){
   pinMode(A, INPUT_PULLUP);
   pinMode(B, INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(A), interruptFunctions[id], RISING);
}

byte Encoder::getInstanceCount() {
   return instanceCount;
}

byte Encoder::getId() const{
   return id; 
}
 
void Encoder::setPulses(int p) {
   pulses = p;
}

int Encoder::getPulses() const {
   return pulses;
}

void Encoder::setPeriod(unsigned long T) {
   this->T = T;
}

unsigned long Encoder::getPeriod() const {
   return T;
}

void Encoder::setFilter(unsigned long filter) {
   T_filter = filter;
}

unsigned long Encoder::getFilter() const {
   return T_filter;
}

void  Encoder::setDegrees(float _degrees){
   pulses = _degrees/pul_rv;
}

float Encoder::getDegrees() const{
   return pulses*pul_rv;
}
