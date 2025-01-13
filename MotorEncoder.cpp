#include "MotorEncoder.h"

MotorEncoder::MotorEncoder(byte dir, byte en, byte encoderPinA, byte encoderPinB, float pul_Rv )
    : dirPin(dir), enPin(en), encoder(encoderPinA, encoderPinB, pul_Rv ) {
    samplingTime     =  DEFAULT_SAMPLING_TIME;
    lastPrintPulses  = false;
    lastPrintPeriod  = false;
    lastPrintDegrees = false;
    lastPrintSpeed   = false;
}


#ifdef ESP32
void MotorEncoder::configureESP32PWM (byte channel, int frequency, int resolution) {
    pwm_channel = channel;
    ledcSetup(channel, frequency, resolution);
    ledcAttachPin(enPin, channel);
}

void MotorEncoder::configureESP32PWM_2 (byte channel, int frequency, int resolution){
    pwm_channel = channel;
    ledcSetup(channel, frequency, resolution);
    ledcAttachPin(dirPin, channel);
}
#endif

void MotorEncoder::init(){
    pinMode(dirPin, OUTPUT);
    pinMode(enPin,  OUTPUT);
    digitalWrite(enPin , LOW);
    digitalWrite(dirPin, LOW);
    encoder.init();
}

void MotorEncoder::setEncoderFilter_T(unsigned long period_filter) {
    encoder.setFilter(period_filter);
}

void MotorEncoder::setSamplingTime(unsigned long Sam_Time) {
    samplingTime = Sam_Time;
}

long MotorEncoder::getPulses() {
    return encoder.getPulses();
}

long MotorEncoder::getEncoderPeriod() {
    return encoder.getPeriod();
}

float MotorEncoder::getDegrees() {
    return encoder.getDegrees();
}

float MotorEncoder::getSpeed() {
    
  unsigned long        currentTime    = micros();
  
  if ((currentTime - lastUpdate) >= samplingTime) {
    long pulseDifference = encoder.getPulses() - lastPulseCount;
    
    speedR_S = (pulseDifference == 0) 
                       ? 0 
                       : ( (pulseDifference)*159829.51 ) / (currentTime - lastUpdate);
    
   
    lastPulseCount = encoder.getPulses();
    lastUpdate = currentTime;
    
  }

  return speedR_S;  
}

void MotorEncoder::turn  (int vel) {
    if (vel > 0) {
        digitalWrite(dirPin, HIGH);
    } else {
        digitalWrite(dirPin, LOW);
        vel = -vel;
    }
    
#ifdef ESP32
    ledcWrite(pwm_channel, vel);
#else
    analogWrite(enPin, vel);
#endif
}
/*
void MotorEncoder::turn_2(int vel){

    byte _vel=0;
    digitalWrite(enPin,HIGH);
    
    if (vel > 0) {
 
       #ifdef ESP32
          _vel = map(vel, 0, 255, 128, 255); 
          ledcWrite(pwm_channel, _vel);
       #else
          analogWrite(dirPin, _vel);
       #endif
           
    }else {
       vel=-vel;
       
       #ifdef ESP32
          _vel = map(vel, 0, 255, 128, 0); 
          ledcWrite(pwm_channel, _vel);
       #else
          analogWrite(dirPin, _vel);
       #endif 
    }
}
*/

void MotorEncoder::turn_2(int vel){

    byte _vel=0;
    digitalWrite(enPin,HIGH);
    
    if (vel > 0) {
 
       #ifdef ESP32
          _vel = map(vel, 0, 255, 128, 0); 
          ledcWrite(pwm_channel, _vel);
       #else
          analogWrite(dirPin, _vel);
       #endif
           
    }else {
       vel=-vel;
       
       #ifdef ESP32
          _vel = map(vel, 0, 255, 128, 255); 
          ledcWrite(pwm_channel, _vel);
       #else
          analogWrite(dirPin, _vel);
       #endif 
    }
}


void MotorEncoder::Stop() {
    digitalWrite(dirPin, LOW);
    analogWrite(enPin, 0);
}

void MotorEncoder::Stop_2() {
  
    #ifdef ESP32
        ledcWrite(pwm_channel, 128);
     #else
        analogWrite(dirPin, 128);
     #endif 
    
}

void MotorEncoder::Print(bool printPulses, bool printPeriod, bool printDegrees, bool printSpeed) {
   

    lastPrintPulses  = printPulses;
    lastPrintPeriod  = printPeriod;
    lastPrintDegrees = printDegrees;
    lastPrintSpeed   = printSpeed;

    if(lastPrintPulses || lastPrintPeriod || lastPrintDegrees ||  lastPrintSpeed ){
        Serial.print(encoder.getId());
        if (lastPrintPulses)  { Serial.print(" Pulses: ");  Serial.print(encoder.getPulses());  }
        if (lastPrintPeriod)  { Serial.print(" Period: ");  Serial.print(encoder.getPeriod());  }
        if (lastPrintDegrees) { Serial.print(" Degrees: "); Serial.print(encoder.getDegrees()); }
        if (lastPrintSpeed)   { Serial.print(" Speed: ");   Serial.print(this->getSpeed());     }
        Serial.println();
    }
}

void MotorEncoder::Print() {
     this->Print(lastPrintPulses, lastPrintPeriod, lastPrintDegrees, lastPrintSpeed);
}

void MotorEncoder::resetEncoderData() {
    encoder.setPulses(0);
    encoder.setPeriod(0); // o lo que se necesite para reiniciar T
}

void MotorEncoder::processMotorCommand(String command) {
    
    if (command.length() == 0) {
        Serial.println("Invalid command: Command is empty.");
        return;
    }

    else if (command.startsWith("filter_")) {
        unsigned long period = numberParameter(command, 1);
        if (period > 0) {
            this->setEncoderFilter_T(period);
            Serial.print("Encoder filter set to: ");
            Serial.println(period);
        } else {
            Serial.println("Invalid command format for filter period.");
        }
    }
    else if (command.startsWith("sampling_")) {
        unsigned long samplingTime = numberParameter(command, 1);
        if (samplingTime > 0) {
            this->setSamplingTime(samplingTime);
            Serial.print("Sampling time set to: ");
            Serial.println(samplingTime);
        } else {
            Serial.println("Invalid command format for sampling time.");
        }
    }
    else if (command.startsWith("PWM")) {
        int channel    =  numberParameter(command, 1);
        int frequency  =  numberParameter(command, 2);
        int resolution =  numberParameter(command, 3);
        if (channel >= 0 && frequency > 0  && resolution > 0 ) {
            this->configureESP32PWM(channel, frequency, resolution);
            Serial.print("PWM configured: Channel ");
            Serial.print(channel);
            Serial.print(", Frequency ");
            Serial.print(frequency);
            Serial.print(", Resolution ");
            Serial.println(resolution);
        } else {
            Serial.println("Invalid parameters for PWM command.");
        }
    }
    else if (command.startsWith("turn_")) {
        int _speed = numberParameter(command, 1);
        if(-2147483648 ==_speed) {
           Serial.println("Invalid command format for speed."); 
        }else {
           this->turn_2(_speed);
           Serial.print("Motor turning at speed: ");
           Serial.println(_speed);
        }
    }
    else if (command.equals("Stop")) {
        this->Stop_2();
        Serial.println("Motor stopped.");
    }
    else if (command.equals("Print_C") ) {
        Serial.println("......");
        Serial.println("*****************");
        Serial.println("*****************");
        Serial.println(".........Comandos......");
        Serial.println("filter_period (ej: filter_100)");
        Serial.println("sampling_time (ej: sampling_1000)");
        Serial.println("PWM           (ej: PWM_channel_frequency_resolution ");
        Serial.println("turn_vel      (ej: turn_255 o turn_-255)");
        Serial.println("Stop          (ej: Stop ");
        Serial.println("Print         (ej: Print, Print_Degrees, Print_Degrees_Pulses, _C )");
        Serial.println("*****************");
        Serial.println("*****************");
        this->Print(false,false,false,false); 
    }
    else if (command.indexOf("Print")  != -1 && command.indexOf("Print_") == -1 ){
        Serial.println("......");
        this->Print();
    }
    else if (command.indexOf("Print_")  != -1){  
        Serial.println("......");
        
        bool printPulses  = false;
        bool printPeriod  = false;
        bool printDegrees = false;
        bool printSpeed   = false;
        
        if (command.indexOf("Pulses")  != -1) printPulses  = true;
        if (command.indexOf("Period")  != -1) printPeriod  = true;
        if (command.indexOf("Degrees") != -1) printDegrees = true;
        if (command.indexOf("Speed")   != -1) printSpeed   = true;
        
        if (printPulses ||printPeriod || printDegrees || printSpeed)this->Print(printPulses,printPeriod,printDegrees, printSpeed); 
        else Serial.println("Invalid parameters for Print command.");
    }
    else {
        Serial.print("Invalid command: ");
        Serial.println(command);
    }
}

int MotorEncoder::numberParameter(String command, int paramIndex) {
    int startIndex = 0;
    for (int i = 0; i < paramIndex; i++) {
        startIndex = command.indexOf('_', startIndex) + 1;
        if (startIndex == 0) return -2147483648; 
    }
    int endIndex = command.indexOf('_', startIndex);
    if (endIndex == -1) endIndex = command.length();
    return command.substring(startIndex, endIndex).toInt();
}
