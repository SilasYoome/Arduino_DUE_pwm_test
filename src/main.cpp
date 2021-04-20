#include <Arduino.h>
#include "pwm_lib.h"
#include "tc_lib.h"

#define ANGLES 10
uint32_t angles[ANGLES]= // degrees
{
  0,
  51, 
  102,
  153,
  204,
  255,
  204,
  153,
  102,
  51
};
uint32_t angle=0;
uint32_t last_angle;

//using namespace arduino_due::pwm_lib;
#define PINA PWMH1_PA19
#define PINB PWMH1_PC5
arduino_due::pwm_lib::servo<arduino_due::pwm_lib::pwm_pin::PINA> servo_pwm_pinA;
arduino_due::pwm_lib::servo<arduino_due::pwm_lib::pwm_pin::PINB> servo_pwm_pinB;
void setup(){
  servo_pwm_pinA.start(1670,0,1669,0,255,angles[angle]);
  servo_pwm_pinB.start(1670,0,1669,0,255,angles[angle]);

  last_angle=millis();
  Serial.println("********************************************************");
  Serial.print("angle "); Serial.print(angle); 
  Serial.print(": "); Serial.print(angles[angle]); 
  Serial.println(" dgs.");
  Serial.println("********************************************************");
}

void loop(){
  delay(5000);
  if(millis()-last_angle>5000){
    angle=(angle+1)&0x07;
    servo_pwm_pinA.set_angle(angles[angle]);
    servo_pwm_pinB.set_angle(angles[angle]);
    last_angle = millis();
    Serial.println("********************************************************");
    Serial.print("angle "); Serial.print(angle); 
    Serial.print(": "); Serial.print(angles[angle]); 
    Serial.println(" dgs.");
    Serial.println("********************************************************");
  }
}