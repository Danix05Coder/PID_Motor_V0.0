#include "Encoder.h"
#include "PID.h"
#include "MotorEncoder.h"

#define ENCODER_PPR    0.5143

#define encoderPinA_L 18
#define encoderPinB_L 17
#define dirPin_L      12
#define enPin_L       14

#define encoderPinA_R 10 
#define encoderPinB_R 11
#define dirPin_R      13
#define enPin_R       14

MotorEncoder motorL (dirPin_L, enPin_L, encoderPinA_L, encoderPinB_L, ENCODER_PPR);
MotorEncoder motorR (dirPin_R, enPin_R, encoderPinA_R, encoderPinB_R, ENCODER_PPR);

//////PID//////

#define Kp_L       5     
#define Ki_L       200    
#define Kd_L       0     
#define St_L       1000
#define MAX_OUT_L  150
#define T_error_L  2
#define Filter_L   500

#define Kp_R       5    
#define Ki_R       200    
#define Kd_R       0     
#define St_R       1000
#define MAX_OUT_R  150
#define T_error_R  2
#define Filter_R   500

#define Kp_V       1    
#define Ki_V       2.6    
#define Kd_V       0     
#define St_V       1000
#define MAX_OUT_V  50
#define T_error_V  1


PID pidMotorL      ( Kp_L , Ki_L , Kd_L , St_L ,MAX_OUT_L, T_error_L );
PID pidMotorR      ( Kp_R , Ki_R , Kd_R , St_R ,MAX_OUT_R ,T_error_R );
PID pidMotor_Vel   ( Kp_V , Ki_V , Kd_V , St_V ,MAX_OUT_V ,T_error_V );

double input_L=0, output_L=0,set_point_L=0; 
double input_R=0, output_R=0,set_point_R=0; 
double input_V=0, output_V=0,set_point_V=0;

int L_vel_pwm = 0;

//////////////////////

void setup(){
  
  Serial.begin(115200);
  
  motorL.init();
  motorR.init();
  
  motorL.configureESP32PWM_2(0, 50000, 8);
  motorR.configureESP32PWM_2(1, 50000, 8);

  motorL.setEncoderFilter_T(Filter_L);
  motorR.setEncoderFilter_T(Filter_R);
  
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
  
  motorL.Stop_2();
  motorR.Stop_2();
}

void loop() {
/*
    set_point_L =   120;
    
    L_vel_pwm  = 1.1* set_point_L + 14.7;
    
    input_L  = motorL.getSpeed();
    
    output_L = pidMotorL.compute(input_L, set_point_L);
    
    motorL.turn_2(  output_L);
    
    
    Serial.print(" Input_L: "   );
    Serial.print( input_L       ); 
    Serial.print(" setpoint_L: ");
    Serial.print( set_point_L   ); 
    Serial.print(" output_L:   ");
    Serial.print( output_L);
  
    Serial.println();
*/
   set_point_L =   184;// 45°-92  90°-184 180°-368
   set_point_R =   184;
   set_point_V =   0;
   
   input_L  = motorL.getDegrees();
   input_R  = motorR.getDegrees();
   input_V  = abs(input_L) - abs(input_R);
  
    
   output_L = pidMotorL.compute       (input_L, set_point_L);
   output_R = pidMotorR.compute       (input_R, set_point_R);
   output_V = pidMotor_Vel.compute_2  (input_V, set_point_V);
    
   motorL.turn_2(  output_L  +  output_V );
   motorR.turn_2(  output_R  -  output_V );
    
   Serial.print("Diferencia_Grados L-R: ");
   Serial.print(input_V);
   Serial.println();
   
}
