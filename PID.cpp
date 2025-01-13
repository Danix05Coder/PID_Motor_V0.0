#include "PID.h"

PID::PID(double kp, double ki, double kd, unsigned long sampling_time, double max_out, double error_tolerance)
    : kp(kp), ki(ki), kd(kd), sampling_time(sampling_time), max_out(max_out), error_tolerance(error_tolerance), 
      error(0), output(0), input(0), errorIntegral(0), errorDerivative(0), stable(false) {

    last_Print_error = false;
    last_Print_errorIntegral = false;
    last_Print_errorDerivative = false;
    last_Print_input = false;
    last_Print_output = false;

        
}
      
double PID::compute(double _input, double set_point) {
                          
    long   current_time    = 0;
    long   timeDifference  = 0;
    
    input = _input;
    current_time = micros();
    timeDifference =  current_time - last_time;

    if ( timeDifference >= sampling_time) {

        error      =  set_point - input;

        errorIntegral = (prev_error == error) ? errorIntegral+(error + prev_error)*(timeDifference/2)*(1e-6) : 0 ;  //limita la integral

        //errorIntegral  += (error + prev_error)*(timeDifference/2)*(1e-6);
        
        errorDerivative = (error - prev_error)/ (timeDifference*1e-6);

        output = (kp * error) + (ki * errorIntegral) + (kd * errorDerivative);

        output = fmax(fmin(output, max_out), -max_out);

        stable = (abs(error) <= error_tolerance) && (errorDerivative == 0);

        if (stable) {
            errorIntegral   = 0;
            errorDerivative = 0;
            output=0;
        }

        prev_error = error;
        last_time = current_time;
    }

    return output;
}

double PID::compute_2(double _input, double set_point) {
                          
    long   current_time    = 0;
    long   timeDifference  = 0;
    
    input = _input;
    current_time = micros();
    timeDifference =  current_time - last_time;

    if ( timeDifference >= sampling_time) {

        error      =  set_point - input;

        //errorIntegral = (prev_error == error) ? errorIntegral+(error + prev_error)*(timeDifference/2)*(1e-6) : 0 ;  //limita la integral

        errorIntegral  += (error + prev_error)*(timeDifference/2)*(1e-6);
        
        errorDerivative = (error - prev_error)/ (timeDifference*1e-6);

        output = (kp * error) + (ki * errorIntegral) + (kd * errorDerivative);

        output = fmax(fmin(output, max_out), -max_out);

        stable = (abs(error) <= error_tolerance) && (errorDerivative == 0);

        if (stable) {
            errorIntegral   = 0;
            errorDerivative = 0;
            output=0;
        }

        prev_error = error;
        last_time = current_time;
    }

    return output;
}

bool PID::isStable() const {
    return stable;
}

void PID::setKp(double kp){
     this->kp=kp; 
}
void PID::setKi(double ki){
     this->ki=ki;
}
void PID::setKd(double kd){
     this->kd=kd;
}
void PID::setSamplingTime(unsigned long sampling_time){
     this->sampling_time=sampling_time;
}
void PID::setMaxOut(double max_out){
     this->max_out=max_out;
}
void PID::setErrorTolerance(double error_tolerance){
     this->error_tolerance=error_tolerance;
}

void PID::Print(){
    this->Print(last_Print_error,last_Print_errorIntegral,last_Print_errorDerivative,last_Print_input,last_Print_output );
}
void PID::Print(bool p_error , bool p_errorIntegral, bool p_errorDerivative, bool p_input, bool p_output){
    
    last_Print_error = p_error;
    last_Print_errorIntegral = p_errorIntegral;
    last_Print_errorDerivative = p_errorDerivative;
    last_Print_input = p_input;
    last_Print_output = p_output;

    
    if( last_Print_error || last_Print_errorIntegral ||  last_Print_errorDerivative || last_Print_input || last_Print_output  ){
        if(p_error){           Serial.print("   error: ");       Serial.print(error);            }
        if(p_errorIntegral){   Serial.print("  ∫error: ");       Serial.print(errorIntegral);    }
        if(p_errorDerivative){ Serial.print("  d(error)/dt: ");  Serial.print(errorDerivative);  }
        if(p_input ){          Serial.print("  input: ");        Serial.print(input);            }
        if(p_output){          Serial.print("  output: ");       Serial.print(output);           }
        Serial.println();
    }
}

void PID::processCommand(String command){
    
    if (command.length() == 0) {
        Serial.println("Invalid command: Command is empty.");
        return;
    }

    else if (command.startsWith("set_kp_")) {
        double _kp = (double)numberParameter(command, 2);
        if (_kp >= 0) {
            kp=_kp;
            Serial.print("Kp set to: ");
            Serial.println(_kp);
        } else {
            Serial.println("Invalid command format for Kp.");
        }
    }
    else if (command.startsWith("set_ki_")) {
        double _ki = (double)numberParameter(command, 2);
        if (ki >= 0) {
            this->setKi(_ki);
            Serial.print("Ki set to: ");
            Serial.println(ki);
        } else {
            Serial.println("Invalid command format for Ki.");
        }
    }
    else if (command.startsWith("set_kd_")) {
        double _kd = (double)numberParameter(command, 2);
        if (kd >= 0) {
            this->setKd(_kd);
            Serial.print("Kd set to: ");
            Serial.println(kd);
        } else {
            Serial.println("Invalid command format for Kd.");
        }
    }
    else if (command.startsWith("set_sampling_")) {
        unsigned long samplingTime = numberParameter(command, 2);
        if (samplingTime > 0) {
            this->setSamplingTime(samplingTime);
            Serial.print("Sampling time set to: ");
            Serial.println(samplingTime);
        } else {
            Serial.println("Invalid command format for sampling time.");
        }
    }
    else if (command.startsWith("set_max_out_")) {
        double maxOut = numberParameter(command, 3);
        if (maxOut >= 0) {
            this->setMaxOut(maxOut);
            Serial.print("Max output set to: ");
            Serial.println(maxOut);
        } else {
            Serial.println("Invalid command format for max output.");
        }
    }
    else if (command.startsWith("set_error_tolerance_")) {
        double errorTolerance = numberParameter(command, 3);
        if (errorTolerance >= 0) {
            this->setErrorTolerance(errorTolerance);
            Serial.print("Error tolerance set to: ");
            Serial.println(errorTolerance);
        } else {
            Serial.println("Invalid command format for error tolerance.");
        }
    }
    else if (command.equals("Print_PID")) {
        Serial.println("......");
        this->Print(true, true, true, true, true);
    }
    else if (command.indexOf("Print_C") != -1 ) {
        Serial.println("......");
        this->Print(false, false, false, false, false); // Print all by default
    }
    else if (command.indexOf("Print_") != -1) {
        Serial.println("......");

        bool pError = false;
        bool pErrorIntegral = false;
        bool pErrorDerivative = false;
        bool pInput = false;
        bool pOutput = false;

        if (command.indexOf("Error") != -1)           pError = true;
        if (command.indexOf("ErrorIntegral") != -1)   pErrorIntegral = true;
        if (command.indexOf("ErrorDerivative") != -1) pErrorDerivative = true;
        if (command.indexOf("Input") != -1)  pInput = true;
        if (command.indexOf("Output") != -1) pOutput = true;

        if (pError || pErrorIntegral || pErrorDerivative || pInput || pOutput) {
            this->Print(pError, pErrorIntegral, pErrorDerivative, pInput, pOutput);
        } else {
            Serial.println("Invalid parameters for Print command.");
        }
    }
    else {
        Serial.print("Invalid command: ");
        Serial.println(command);
    }
}

int PID::numberParameter(String command, int paramIndex) {
    int startIndex = 0;
    for (int i = 0; i < paramIndex; i++) {
        startIndex = command.indexOf('_', startIndex) + 1;
        if (startIndex == 0) return -2147483648;
    }
    int endIndex = command.indexOf('_', startIndex);
    if (endIndex == -1) endIndex = command.length();
    return command.substring(startIndex, endIndex).toInt();
}
