#include <Arduino.h>
#include <Wire.h>
#include <Zumo32U4.h>
#include <PololuOLED.h>
int speed = 20;

Zumo32U4OLED display;
Zumo32U4IMU imu;
Zumo32U4ButtonA buttonA;
Zumo32U4Buzzer buzzer;
Zumo32U4Encoders endcoders;
Zumo32U4Motors motors;
Zumo32U4ProximitySensors proximitySensor; 
Zumo32U4LineSensors lineSenors;
Zumo32U4IRPulses irpulses;
//Zumo32U4FastGPIO fastGio;



void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

void forward(){
  motors.setSpeeds(speed, speed);
}

void backward(){
  motors.setSpeeds(-speed, -speed);
}¨

// rest the distance counter 
void resetEncoders(){
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
}

void distance(){

}

// too avoid af object
void avoid(){

}

// there check if there theft
void checkTheft(){

}

// is ways save object there in the way for robot 
void saveObject(){

}

// 
void pose(int x,int y){

}

 