#include <Arduino.h>
#include <Wire.h>
#include <Zumo32U4.h>
#include <PololuOLED.h>

int speed = 20;

Zumo32U4LCD display;
Zumo32U4IMU imu;
Zumo32U4ButtonA buttonA;
Zumo32U4Buzzer buzzer;
Zumo32U4Encoders encoders;
Zumo32U4Motors motors;
Zumo32U4ProximitySensors proximitySensor;
Zumo32U4LineSensors lineSenors;
// Zumo32U4FastGPIO fastGio;

// variables for gyro
int16_t gyroOffset;
uint32_t turnAngle = 0;
int16_t turnRate;
uint16_t gyroLastUpdate = 0;

float wheelCirc = 122.52;

// the postion of the robot 
int robotposx = 0;
int robotposy = 0;
int robotangle = 20;

int checkposx = 0;
int checkposy = 0;

// this are the postions the robot need to check// lave om på talene senere
const int checkmax = 3;
int check[checkmax][2] = {{20,47},{40,38},{65,10}};

// rest the distance counter
void resetEncoders()
{
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
}

// Stop the movement
void stop()
{
  motors.setSpeeds(0, 0);
}

// Encoder functions
float getDistance()
{
  int countsL = encoders.getCountsLeft();
  int countsR = encoders.getCountsRight();

  float distanceL = countsL / 900.0 * wheelCirc;
  float distanceR = countsR / 900.0 * wheelCirc;

  return (distanceL + distanceR) / 2;
}

// Go forward a distance with a specified speed
void forward(int dist = 0, int speed = 0)
{
  resetEncoders();
  while (getDistance() <= dist*10)
  {
    /*
    If right is ahead, diff is negative. If right is behind, diff is positive.
    */
    int diff = encoders.getCountsLeft() - encoders.getCountsRight();
    //Serial.println(diff);
    int compSpeed = speed + diff * 0.5;
    motors.setSpeeds(speed, compSpeed);
  }
  stop();

  // convert distance and the angle of robot to x and y coordinates
  // istedet for angle skal der bruges "getTurnAngleInDegrees()"
  robotposx = robotposx + dist*10*cos(robotangle/(180/PI));// ikke færdig
  robotposy = robotposy + dist*10*sin(robotangle/(180/PI));
    
    // just to check 
    display.clear();                                                   // Clears the OLED display.
    display.gotoXY(0, 0);                                              // Sets the position on the OLED, where the message should be printed.
    display.print(robotposx/10);   // 20                                  // Prints "Obstacle".
    display.gotoXY(0, 1);                                              // Sets the position on the OLED, where the next line should be printed.
    display.print(robotposy/10);  // 46
}

// Go backwards a distance with a specified speed
void backward(int dist = 0, int speed = 0)
{
  resetEncoders();
  while (getDistance() >= -dist*10)
  {
    motors.setSpeeds(-speed, -speed);
  }
  stop();
   // convert distance and the angle of robot to x and y coordinates
  robotposx = robotposx - dist*10*cos(robotangle/(180/PI));// ikke færdig
  robotposy = robotposy - dist*10*sin(robotangle/(180/PI));
}

// Avoid collision with a object by going around it, return true when done.
void avoid()
{
  backward(100, 50);
  // turn
  forward(50, 50);
  // turn
  forward(200, 50);
}

// Turn around and create scan of area, turn around again and compare.
void checkTheft()
{
}

// is ways save object there in the way for robot, ambiguus for the moment.
void saveObject()
{
}

// Used to go to a specific location specified in x and y coordinates
void pose(int x, int y)
{
}

// Saves the curent position updates global var with current position in x and y coordinates // not
void savePos()
{
  checkposx = robotposx;
  checkposy = robotposy;
}

// It move the robot to given positions 
void MoveToPos(int x = 0, int y = 0){
  // varibel for the vektor 
  int newposx = 0;
  int newposy = 0;
  // Dette er bare en float
  int angle = 0;
  int dist = 0;

  // check if the positions need too add on or minus with
  if(x>robotposx){
    newposx = x - robotposx;
  }
  else {
     newposx = robotposx - x;
  }
  
  if(y>newposy){
    newposy = y - robotposy;
  }
  else{
    newposy = robotposy - y;
  }
  
  newposx = robotposx + newposx;
  newposy = robotposy + newposy;

 // angle get round up it float return get convert to int
  angle = atan(newposy/newposx)*(180/PI); // makes angle from the vektor 
  dist = sqrt(pow(newposx,2)+pow(newposy,2)); // find length of the vektor 

  if (angle>robotangle) {
    angle = angle - robotangle;
  }
  else{
    angle = robotangle - angle;
  }
  
  // Her we put the angle and distance robot too travel 
  // turn  
  forward(dist,200);
  

}

// this code just check if method "MoveToPos" Works 
void test() {

  for(int i = 0; i<checkmax; i++){
    MoveToPos(check[0][i],check[1][i]);

  }

}

// Robot turns around itself with a random angle.
void turnRandomAng()
{

}

// Robot detects if any object is infront of it, returns true if a object is present.
void detectObject()
{
}

/** \brief Robot turns right or left with a specified radius, angle and speed.
 *
 * \param dir 0 = Right 1 = Left 
 **/
void turnArc(bool dir = 0, int radius = 100, int speed = 100)
{
}

// This should be called to set the starting point for measuring
// a turn.  After calling this, turnAngle will be 0.
void turnSensorReset()
{
  gyroLastUpdate = micros();
  turnAngle = 0;
}

/*
Gyro setup and convenience functions
*/
void turnSensorSetup()
{
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();

  display.clear();
  display.print(F("Gyro cal"));

  // Turn on the yellow LED in case the LCD is not available.
  ledYellow(1);

  // Delay to give the user time to remove their finger.
  delay(500);

  // Calibrate the gyro.
  int32_t total = 0;
  for (uint16_t i = 0; i < 1024; i++)
  {
    // Wait for new data to be available, then read it.
    while (!imu.gyroDataReady())
    {
    }
    imu.readGyro();

    // Add the Z axis reading to the total.
    total += imu.g.z;
  }
  ledYellow(0);
  gyroOffset = total / 1024;

  // Display the angle (in degrees from -180 to 180) until the
  // user presses A.
  display.clear();
  turnSensorReset();
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  display.init();
  turnSensorSetup();
  encoders.init();
  proximitySensor.initFrontSensor();
  lineSenors.initThreeSensors();
}

unsigned long previusTime = 0;

void loop()
{
  // put your main code here, to run repeatedly:
  //MoveToPos(20,47);
  
  forward(51,200);
  delay(2000);

  // if(millis() - previusTime > 52){

  // tjek foran
  // tjek linesor
  // forward

  // previusTime = millis();
  // }

  // if(millis() - previusTime1 > 317){

  // skærm

  // previusTime1 = millis();
  // }
}