#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Motor shield uses the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();  

// Stepper motor has 514 steps per revolution
Adafruit_StepperMotor *platform = AFMS.getStepper(514, 1);
Adafruit_StepperMotor *launcher = AFMS.getStepper(514, 2);
Adafruit_StepperMotor *motor;

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Ballin!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  
  platform->setSpeed(20);  // 10 rpm   
  launcher->setSpeed(20);
}

void loop() {
  //platform to find hoop
  
  //Launcher angle set
  while(Serial.available() ==0);
  int angle = Serial.parseInt();
  if(angle < 1000){
    motor = platform;
  } else{
    motor = launcher;
    angle = angle - 1000;
  }
  motor->step(angle, FORWARD, DOUBLE);
  delay(1000); //1000 = 1 second - Delay for X minutes 
  motor->step(angle, BACKWARD, DOUBLE);
  delay(1000);
}
