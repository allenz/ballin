
#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();  

// Connect a stepper motor with 360 steps per revolution (1 degree)
Adafruit_StepperMotor *platform = AFMS.getStepper(514, 1);
Adafruit_StepperMotor *launcher = AFMS.getStepper(514, 2);
Adafruit_StepperMotor *motor;

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Ballin!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  
  platform->setSpeed(10);  // 10 rpm   
  launcher->setSpeed(10);
}

void loop() {
  //platform to find hoop

// if (input == 0){ 
//    platform->step(1, FORWARD, DOUBLE);
//    Serial.println(Step!);
//  else if (input ==1){
//    platform->step(1, FORWARD, DOUBLE);
//    Serial.println(Stop!);  
//  }


  //Launcher angle set
  while(Serial.available() ==0);
  int input = Serial.parseInt();
  int angle = 0; 
  
  if(angle < 1000){
    motor = platform;
  }
  else{
    motor = launcher;
    angle = angle - 2000;
  }
   for(int x = 0; x < input; x++){
    motor->step(1,FORWARD, SINGLE);
       delay(1000);
       angle = angle+1;
       Serial.println(angle);  
   }
       delay(1000);
}
