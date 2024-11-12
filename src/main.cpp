#include <Arduino.h>
#include <Wire.h>
#include <Zumo32U4.h>
#include <PololuOLED.h>

int speed = 200;
#define thieveThreshold 1.5

Zumo32U4LCD display;
Zumo32U4IMU imu;
Zumo32U4ButtonA buttonA;
Zumo32U4Buzzer buzzer;
Zumo32U4Encoders encoders;
Zumo32U4Motors motors;
Zumo32U4ProximitySensors proximitySensor;
Zumo32U4LineSensors lineSenors;
// Zumo32U4FastGPIO fastGio;
#define NUM_SENSORS 3
// variables for gyro
int16_t gyroOffset;
uint32_t turnAngle = 0;
int16_t turnRate;
uint16_t gyroLastUpdate = 0;
uint16_t lineSensorValues[NUM_SENSORS];

float wheelCirc = 122.52;

// the postion of the robot 
int robotposx = 0;
int robotposy = 0;
int robotangle = 20;

int checkposx = 0;
int checkposy = 0;

// this are the postions the robot need to check// lave om på talene senere
const int check[3][2] = {{20,47},{40,38},{65,10}};

/** \brief Function takes an angle from 0 to 360 and offsets it by an amount,
 * the function handels wrapping of the value back to 0
 * \param value The current absolute angle of the robot
 * \param offset The amount to offset the angle
 * \retval Offsat angle wrapped back to 0.
 **/
int offsetAngValue(int value, int offset)
{
  // Add offset and wrap within 0-359 range
  int newValue = (value + offset) % 360;
  // If result is negative, wrap it to 359
  if (newValue < 0)
  {
    newValue += 360;
  }
  return newValue;
}

/** \brief Resets the encoder counts.
 **/
void resetEncoders()
{
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
}

/** \brief Stops the robot movement
 **/
void stop()
{
  motors.setSpeeds(0, 0);
}

/**
 * \retval Returns the accumulated distance that the robot has traveled based on the encoder counts.
 */
float getDistance()
{
  long int countsL = encoders.getCountsLeft();
  long int countsR = encoders.getCountsRight();

  float distanceL = countsL / 900.0 * wheelCirc;
  float distanceR = countsR / 900.0 * wheelCirc;

  return (distanceL + distanceR) / 2;
}

/**
 * \brief Go forward a distance with a specified speed
 * \param dist Specify the distance that the robot should, move only allows posetive integers.
 * \param speed Specify the speed at which the robot should move, only allows posetive integers.
 */
void forward(uint16_t dist = 0, uint16_t speed = 0)
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
  // i stedet for angle skal der bruges "getTurnAngleInDegrees()"
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
void avoid(){
  detectObject();
  if (detectObject() == true){
    backward(30,100);
    stop();
    delay(50);
    
    turnByAngle(90);
    stop();
    delay(50);
    
    forward(30,100);
    stop();
    delay(50);
    
    turnByAngle(270);
    stop();
    delay(50);
    
    forward(70,100);
    stop();
    delay(50);
    
    turnByAngle(270);
    stop();
    delay(50);
    
    forward(30,100);
    stop();
    delay(50);

    turnByAngle(90);
    stop();
    delay(50);

    forward(30,100);
    stop();
    delay(50);
  }
  else{
    forward(50,100);
  }
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

  for(uint16_t i = 0; i<2; i++){
    MoveToPos(check[0][i],check[1][i]);
  }

}

// Robot turns around itself with a random angle.
void turnRandomAng()
{

}

// Robot detects if any object is infront of it, returns true if a object is present.
bool detectObject(){
  proximitySensor.read();
  int leftReading = proximitySensor.countsFrontWithLeftLeds();
  int rightReading = proximitySensor.countsFrontWithRightLeds();

  int threshold = 5;                                                   // The threshold, when the robot is too close to an object.
  
  if (leftReading >= threshold || rightReading >= threshold){          // If the sensor readings is more or equal to the threshold value, then the robot should print "Obstacle ahead!" to the OLED.
    display.clear();                                                   // Clears the OLED display.
    display.gotoXY(0, 0);                                              // Sets the position on the OLED, where the message should be printed.
    display.print(F("Obstacle"));                                      // Prints "Obstacle".
    display.gotoXY(0, 1);                                              // Sets the position on the OLED, where the next line should be printed.
    display.print(F("ahead!"));                                        // Prints "ahead!".
  } 
  else{
    display.clear();
    return false;
  }
  delay(100);
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

/* Read the gyro and update the angle.  This should be called as
 frequently as possible while using the gyro to do turns. */
void turnSensorUpdate()
{
  // Read the measurements from the gyro.
  imu.readGyro();
  turnRate = imu.g.z - gyroOffset;

  // Figure out how much time has passed since the last update (dt)
  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;

  // Multiply dt by turnRate in order to get an estimation of how
  // much the robot has turned since the last update.
  // (angular change = angular velocity * time)
  int32_t d = (int32_t)turnRate * dt;

  // The units of d are gyro digits times microseconds.  We need
  // to convert those to the units of turnAngle, where 2^29 units
  // represents 45 degrees.  The conversion from gyro digits to
  // degrees per second (dps) is determined by the sensitivity of
  // the gyro: 0.07 degrees per second per digit.
  //
  // (0.07 dps/digit) * (1/1000000 s/us) * (2^29/45 unit/degree)
  // = 14680064/17578125 unit/(digit*us)
  turnAngle += (int64_t)d * 14680064 / 17578125;
}

uint32_t getTurnAngleInDegrees()
{
  turnSensorUpdate();
  // do some math and pointer magic to turn angle in seconds to angle in degree
  return (((uint32_t)turnAngle >> 16) * 360) >> 16;
}

void turnByAngle(int newconstant = 0){
  
  int currentconstant = 0;
  int etellerandet = getTurnAngleInDegrees();

   if( currentconstant >=  newconstant)
    {
      currentconstant -= newconstant;
        while (newconstant != etellerandet)
        {
          motors.setSpeeds(0, -100);
          etellerandet = getTurnAngleInDegrees();
        }
    }
   else if( currentconstant <  newconstant)
    {
      currentconstant += newconstant;
        while (newconstant != etellerandet)
        {
          motors.setSpeeds(0, 100);
          etellerandet = getTurnAngleInDegrees();
        }
        motors.setSpeeds(0, 0);
    }
    
    currentconstant = newconstant;
    
}

// Turns the robot a random angle
void turnRandom(){
  int randomNumber = random(10, 359);
  turnByAngle(randomNumber);
  stop();
  delay(1000);
}

/**
 * \brief Rotates twice while recording values from the proximity sensors.
 * \return Returns true if a thieve is detected. Returns false if no thieve is detected
 */
bool checkTheft()
{
  // Initialise variables
  int base[15], test[15];
  uint32_t angle;
  uint8_t numTurns = 0;
  float error = 0;
  bool stepNoted = false;
  bool turnNoted = false;

  // Store the initial angle, used for offset.
  uint32_t startAngle = getTurnAngleInDegrees();

  // Begin rotation
  motors.setSpeeds(-120, 120);

  // Record a value for each 24 degrees (15 measurements) save it in base list.
  // Next repeat the measurement but save the result in test list.

  while (numTurns <= 1)
  {
    // Update the offsat angle and print it.
    angle = offsetAngValue(getTurnAngleInDegrees(), startAngle);
    Serial.println("Vinkel: " + String(angle));

    // Count number of full rotations
    if (angle == 355 && turnNoted == false)
    {
      turnNoted = true;
      numTurns++;
    }
    else if (angle == 0)
    {
      turnNoted = false;
    }

    // Store reading from proximity sensor.
    if (angle % 24 == 0 && stepNoted == false)
    {
      // Reads values for the front proximity sensor, if there is an object in range of the left or right IR light.
      proximitySensor.read();
      //Add the two readings for an avg.
      int proxReading = (proximitySensor.countsFrontWithLeftLeds() + proximitySensor.countsFrontWithRightLeds());
      stepNoted = true;
      int index = angle / 24;
      //Store in correct array
      if (numTurns == 0)
      {
        base[index] = proxReading;
      }
      else if (numTurns == 1)
      {
        test[index] = proxReading;
      }
    }
    else
    {
      //Robot is not at an angle where a measurement should be taken.
      stepNoted = false;
    }
  }

//After all measurements have been taken, stop the motion
  stop();

//Print all the recorded values
  Serial.println("Base list:");
  for (int j = 0; j < 15; j++)
  {
    Serial.println(String(j) + ": " + String(int(base[j])));
  }
  Serial.println();

  Serial.println("Test list:");
  for (int j = 0; j < 15; j++)
  {
    Serial.println(String(j) + ": " + String(int(test[j])));
  }
  Serial.println();

  // Compare the two lists and calculate a mean deviation, store in error var. and print it
  for (int i = 0; i < 15; i++)
  {
    error += abs(base[i] - test[i]);
  }
  error = error / 15;
  Serial.println("Error: " + String(error));

  // Compare measurements with allowed error and return based on result
  if (error > thieveThreshold)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void Linesensor()
{
  // detects when the distance to an object is readable
  if (lineSensorValues[0] < 1000 && lineSensorValues[1] < 1000 && lineSensorValues[2] < 1000)
  {
    motors.setSpeeds(speed, speed);
    if (getDistance() > 150)
    {
      motors.setSpeeds(speed, speed);
      resetEncoders();
    }
  }
  // decides which way the robot will turn
  // right
  else if (getDistance() < 50)
  {
    int randnumber = random(300, 500);
    motors.setSpeeds(-200, 200);
    delay(randnumber);
    stop();
  }
  // left
  else if (getDistance() < 100)
  {
    int randnumber = random(300, 500);
    motors.setSpeeds(200, -200);
    delay(randnumber);
    stop();
  }
  // random
  else if (getDistance() < 150)
  {
    int randNumber = random(300, 500);
    long dir = random(1, 3);
    if (dir == 1)
      motors.setSpeeds(200, -200);
    else
      motors.setSpeeds(-200, 200);
    delay(randNumber);
    motors.setSpeeds(0, 0);
  }
  else
  {
    stop();
    resetEncoders();
  }
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  display.init();
  turnSensorSetup();
  encoders.init();
  proximitySensor.initFrontSensor();
  // proximitySensor.setBrightnessLevels(proxBrightnesses,6);
  lineSenors.initThreeSensors();
}

// unsigned long previusTime = 0;

void loop()
{
  // put your main code here, to run repeatedly:
  // forward(1000, 300);
  // backward(200, 400);
  Serial.println(checkTheft());
  delay(30000);
}

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