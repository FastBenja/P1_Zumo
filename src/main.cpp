#include <Arduino.h>
#include <Wire.h>
#include <Zumo32U4.h>
#include <PololuOLED.h>
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_HMC5883_U.h>
// #include <SoftWire.h>
#include <SoftI2C-master/src/SoftI2C.h>
#include <HMC5883L.h>

int speed = 100;
#define thieveThreshold 3 // Was 1.6
#define turnThreshold 2
#define lineThreshold 1000
#define objThreshold 5
#define turnThreshold 2

// I2C addresses
#define MPU6050_ADDR 0x68  // Address of MPU6050
#define HMC5883L_ADDR 0x1E // Address of HMC5883L

// MPU6050 registers
#define MPU6050_REG_PWR_MGMT_1 0x6B //For waking up the MPU
#define MPU6050_REG_INT_PIN_CFG 0x37 //For bypassing the i2c bus

// HMC5883L registers
#define HMC5883L_REG_CONFIG_A 0x00 //Config
#define HMC5883L_REG_MODE 0x02 //Config
#define HMC5883L_REG_DATA_X_MSB 0x03 //Data

//These are the postions the robot need to check - lave om på talene senere
const int check[3][2] = {{20, 47}, {40, 38}, {65, 10}};

//Calculated transformation matrix ans offset for magnetometer correction
float transformationMatrix[2][2] = {{-0.00724577, -0.00242076}, {0.00220427, -0.00659776}};
float magOffsetX = -98.54545454, magOffsetY = -338.4919786;

//Charging station location in X,Y coordinates
const int charger[2] = {0, 0};

// Values for storing the raw mag readings.
int16_t mx, my, mz; 

Zumo32U4OLED display;
Zumo32U4IMU imu;
Zumo32U4ButtonA buttonA;
Zumo32U4Buzzer buzzer;
Zumo32U4Encoders encoders;
Zumo32U4Motors motors;
Zumo32U4ProximitySensors proximitySensor;
Zumo32U4LineSensors lineSenors;
SoftI2C SoftWire(4, 20); // Instantiate a softI2C object on Zumo pins 4 and 20

// variables for gyro
int16_t gyroOffset = 0;
uint32_t turnAngle = 0;
int16_t turnRate = 0;
uint16_t gyroLastUpdate = 0;
unsigned int lineSensorValues[3];

float wheelCirc = 122.52 / 10;
float cpr = 909.7;

// Postion of the robot
int robotposx = 0;
int robotposy = 0;
int robotangle = 0;

int checkposx = 0;
int checkposy = 0;

int avoidDist = 15;
float checkDist = 0;

int i = 0;

// Function definitions:
void ALARM(uint32_t);
int offsetAngValue(int, int);
void resetEncoders();
void stop();
float getDistance();
void backwards(int, int);
void pose(int, int);
void savePos();
void turnSensorUpdate();
bool detectObject();
uint32_t getTurnAngleInDegrees();
void turnByAngleNew(int);
void forward(uint16_t, uint16_t);
void MoveToPos(int, int);
void turnSensorReset();
void turnSensorSetup();
bool checkTheft();
bool checkSurroundings();
void avoid();
void newAvoid();
bool linesensor();
void forward2(uint16_t, uint16_t);
void navigateRandom();
void turnByMag(int);
void getMag();
float calculateMagHeading();
void writeRegister(uint8_t, uint8_t, uint8_t);
uint8_t readRegister(uint8_t, uint8_t);
void readHMC5883LData(int16_t *, int16_t *, int16_t *);
void randomMovement();

void setup()
{
  delay(2000);

  Serial.begin(9600);

  display.init();

  turnSensorSetup();

  encoders.init();

  proximitySensor.initFrontSensor();
  uint16_t customBrightnessLevels[] = {1, 5, 10, 15, 30, 45};
  proximitySensor.setBrightnessLevels(customBrightnessLevels, 6);

  lineSenors.initThreeSensors();

  SoftWire.begin();
  // Initialize MPU6050
  writeRegister(MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1, 0x00); // Wake up MPU6050

  // Enable I2C bypass mode
  writeRegister(MPU6050_ADDR, MPU6050_REG_INT_PIN_CFG, 0x02);

  // Initialize HMC5883L
  writeRegister(HMC5883L_ADDR, HMC5883L_REG_CONFIG_A, 0x70); // 8-average, 15 Hz default
  writeRegister(HMC5883L_ADDR, HMC5883L_REG_MODE, 0x00);     // Continuous measurement mode

  imu.init();
  Wire.begin();
  imu.enableDefault();
  imu.configureForTurnSensing();
  uint32_t seed = imu.m.x ^ micros(); // Kombiner IMU-data med tid
  randomSeed(seed);
}

void loop()
{
  while (i < 1){
    i++;
    MoveToPos(40,40);
  }
  delay(500);
  navigateRandom();
  delay(50);
}

void newAvoid()
{
  turnByAngleNew(20);
  forward(25, 150);
  turnByAngleNew(310);
  forward(25, 150);
  turnByAngleNew(20);
  checkDist = 42.64;
}

/**
 * \brief SOUND THE ALARM !!!!
 * \param time Duration in ms that the alarm should sound (Best if devisible by 300)
 * otherwise the duration will be prolonged until the time in ms is divisible by 300
 */

void ALARM(uint32_t time = 3000)
{
  uint32_t startTime = millis();
  Serial.println("ALARM! Robot Position:");
  Serial.println("x: " + String(robotposx + getDistance() * cos(robotangle / (180 / PI))));
  Serial.println("y: " + String(robotposy + getDistance() * sin(robotangle / (180 / PI))));
  while (millis() - startTime < time)
  {
    Serial.println("ALARM!");
    buzzer.playFrequency(5000, 300, 10);
    ledRed(1);
    ledYellow(1);
    ledGreen(1);
    delay(200);
    buzzer.playFrequency(6000, 300, 10);
    ledRed(0);
    ledYellow(0);
    ledGreen(0);
    delay(200);
  }
}

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

// retval Returns the accumulated distance that the robot has traveled based on the encoder counts.

float getDistance()
{
  // get distance in cm
  long int countsL = encoders.getCountsLeft();
  long int countsR = encoders.getCountsRight();

  float distanceL = countsL / cpr * wheelCirc;
  float distanceR = countsR / cpr * wheelCirc;

  return (distanceL + distanceR) / 2;
}

// Go backwards a distance with a specified speed
void backward(int dist = 0, int speed = 0)
{
  resetEncoders();
  while (getDistance() >= -dist)
  {
    // If right is ahead, diff is negative. If right is behind, diff is positive.
    int diff = encoders.getCountsRight() - encoders.getCountsLeft();
    // Serial.println(diff);
    int compSpeed = speed + diff * 0.5;
    motors.setSpeeds(-speed, -compSpeed);
  }
  /*
  stop();

  // convert distance and the angle of robot to x and y coordinates
  robotposx = robotposx - dist * cos(robotangle / (180 / PI)); // ikke færdig
  robotposy = robotposy - dist * sin(robotangle / (180 / PI));
  */
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

/* this code just check if method "MoveToPos" Works
void test()
{

  for (uint16_t i = 0; i < 2; i++)
  {
    MoveToPos(check[0][i], check[1][i]);
  }
}

/* Read the gyro and update the angle.  This should be called as
 frequently as possible while using the gyro to do turns.*/
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

// This should be called to set the starting point for measuring
// a turn.  After calling this, turnAngle will be 0.
void turnSensorReset()
{
  gyroLastUpdate = micros();
  turnAngle = 0;
}

uint32_t getTurnAngleInDegrees()
{
  turnSensorUpdate();
  // do some math and pointer magic to turn angle in seconds to angle in degree
  return (((uint32_t)turnAngle >> 16) * 360) >> 16;
}

/**
 * \brief Takes an angle and turns the robot to face that angle relative to the starting position.
 * \param angleToTurn Specify the angle to turn, only positive integers between 0 and 359
 */
void turnByAngleNew(int angleToTurn = 0)
{
  // Return if parm is outside allowed spectrum
  if (angleToTurn < 0 || angleToTurn > 359)
  {
    return;
  }

  // Init and reset
  turnSensorReset();
  int ang = 0;

  // While not in correct orientation
  while (angleToTurn != ang)
  {
    ang = getTurnAngleInDegrees(); // Get angle

    if (angleToTurn < 180) // Decide on direction
    {
      if ((angleToTurn - ang) > 40) // Decrease speed
      {
        motors.setSpeeds(-200, 200); // Drive left
      }
      else
      {
        motors.setSpeeds(-100, 100);
      }
    }
    else
    {
      if ((ang - angleToTurn) > 40) // Decrease speed
      {
        motors.setSpeeds(200, -200); // Drive right
      }
      else
      {
        motors.setSpeeds(100, -100);
      }
    }
  }
  robotangle = offsetAngValue(robotangle, ang);
  motors.setSpeeds(0, 0);                                  // Stop
  Serial.println("Turned to: " + String(ang) + "Degrees"); // Print resulting angle
}

/**
 * \brief Rotates twice while recording values from the proximity sensors.
 * \return Returns true if a thieve is detected. Returns false if no thieve is detected
 */
bool checkTheft() // kordnate
{

  // Initialise variables
  int base[9], test[9];
  uint32_t angle = 0;
  uint8_t numTurns = 0;
  float error = 0;
  bool stepNoted = false;
  bool halfway = false;

  // Store the initial angle, used for offset.
  turnSensorReset();
  // uint32_t startAngle = getTurnAngleInDegrees();

  // Begin rotation
  motors.setSpeeds(-150, 150);

  // Record a value for each 40 degrees (9 measurements) save it in base list.
  // Next repeat the measurement but save the result in test list.

  while (numTurns < 2)
  {
    // Update the offsat angle and print it.
    // angle = offsetAngValue(getTurnAngleInDegrees(), startAngle);
    angle = getTurnAngleInDegrees();
    // Clears the OLED display.
    display.clear();
    display.gotoXY(0, 0); // Sets the position on the OLED, where the message should be printed.
    display.print(angle);
    // Serial.println("Vinkel: " + String(angle) + " numTurns: " + String(numTurns) + " Turn angle in degrees: " + String(getTurnAngleInDegrees()) + " StartAngle: " + startAngle);

    // Count number of full rotations
    if (angle == 180)
    {
      halfway = true;
    }
    if (halfway && angle == 0)
    {
      numTurns++;
    }
    if (angle == 0)
    {
      halfway = false;
    }

    // Store reading from proximity sensor.
    if (angle % 40 == 0 && stepNoted == false)
    {
      // Reads values for the front proximity sensor, if there is an object in range of the left or right IR light.
      proximitySensor.read();
      // Add the two readings for an avg.
      int proxReading = (proximitySensor.countsFrontWithLeftLeds() + proximitySensor.countsFrontWithRightLeds());
      stepNoted = true;
      int index = angle / 40;
      // Store in correct array
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
      // Robot is not at an angle where a measurement should be taken.
      stepNoted = false;
    }
  }
  // After all measurements have been taken, stop the motion
  stop();

  // Print all the recorded values
  Serial.println("Base list:");
  for (int j = 0; j < 9; j++)
  {
    Serial.println(String(j) + ": " + String(int(base[j])));
  }
  Serial.println();

  Serial.println("Test list:");
  for (int j = 0; j < 9; j++)
  {
    Serial.println(String(j) + ": " + String(int(test[j])));
  }
  Serial.println();

  // Compare the two lists and calculate a mean deviation, store in error var. and print it
  for (int i = 0; i < 9; i++)
  {
    error += abs(base[i] - test[i]);
  }
  error = error / 9;
  Serial.println("Error: " + String(error));
  display.clear();      // Clears the OLED display.
  display.gotoXY(0, 0); // Sets the position on the OLED, where the message should be printed.
  display.print(String(error, 3));

  // Compare measurements with allowed error and return based on result
  if (error > thieveThreshold)
  {
    ALARM();
    buttonA.waitForButton();
    return true;
  }
  return false;
}

/**
 * \brief Go forward a distance with a specified speed
 * \param dist Specify the distance that the robot should, move only allows posetive integers.
 * \param speed Specify the speed at which the robot should move, only allows posetive integers.
 */
void forward(uint16_t dist = 0, uint16_t speed = 0)
{
  int diff = 0;
  float leftEncCount = 0;
  float rightEncCount = 0;
  resetEncoders();
  checkDist = 0;
  while (getDistance() + checkDist < dist) // ændre til ikke lig dist
  {

    bool check = false;
    check = checkSurroundings();
    leftEncCount = encoders.getCountsLeft();
    rightEncCount = encoders.getCountsRight();

    if (check)
    {
      diff = leftEncCount - rightEncCount;
      float distanceL = leftEncCount / 900.0 * (wheelCirc / 10);
      float distanceR = rightEncCount / 900.0 * (wheelCirc / 10);

      checkDist = checkDist + (distanceL + distanceR) / 2;
    }
    else
    {

      diff = leftEncCount - rightEncCount;
    }

    // rest encode so surround don't effect distance
    /*
    If right is ahead, diff is negative. If right is behind, diff is positive.
    */

    // Serial.println(diff);
    int compSpeed = speed + diff * 5;
    motors.setSpeeds(speed, compSpeed);
    display.clear();      // Clears the OLED display.
    display.gotoXY(0, 0); // Sets the position on the OLED, where the message should be printed.
    display.print(speed);
    display.gotoXY(0, 1);
    display.print(compSpeed);
  }
  stop();

  // convert distance and the angle of robot to x and y coordinates
  // i stedet for angle skal der bruges "getTurnAngleInDegrees()"
  robotposx = robotposx + dist * cos(robotangle / (180 / PI)); // ikke færdig
  robotposy = robotposy + dist * sin(robotangle / (180 / PI));

  // just to check
  /*
  display.clear();          // Clears the OLED display.
  display.gotoXY(0, 0);     // Sets the position on the OLED, where the message should be printed.
  display.print(robotposx); // 20                                  // Prints "Obstacle".
  display.gotoXY(0, 1);     // Sets the position on the OLED, where the next line should be printed.
  display.print(robotposy); // 46
  */
}

void forward2(uint16_t dist = 0, uint16_t speed = 0)
{
  int diff = 0;
  float leftEncCount = 0;
  float rightEncCount = 0;
  resetEncoders();
  checkDist = 0;
  while (getDistance() + checkDist < dist)
  {
    bool check = false;
    leftEncCount = encoders.getCountsLeft();
    rightEncCount = encoders.getCountsRight();

    // display.clear();      // Clears the OLED display.

    diff = encoders.getCountsLeft() - encoders.getCountsRight();

    // rest encde so surround don't effect distance
    /*
    If right is ahead, diff is negative. If right is behind, diff is positive.
    */

    // Serial.println(diff);
    int compSpeed = speed + diff * 5;
    motors.setSpeeds(speed, compSpeed);
  }
  stop();

  // convert distance and the angle of robot to x and y coordinates
  // i stedet for angle skal der bruges "getTurnAngleInDegrees()"
  robotposx = robotposx + dist * cos(robotangle / (180 / PI)); // ikke færdig
  robotposy = robotposy + dist * sin(robotangle / (180 / PI));

  // just to check
  /*
  display.clear();          // Clears the OLED display.
  display.gotoXY(0, 0);     // Sets the position on the OLED, where the message should be printed.
  display.print(robotposx); // 20                                  // Prints "Obstacle".
  display.gotoXY(0, 1);     // Sets the position on the OLED, where the next line should be printed.
  display.print(robotposy); // 46
  */
}

// Avoid collision with a object by going around it, return true when done.
void avoid()
{
  checkDist += getDistance();
  backward(3, 100);
  stop();
  delay(50);

  turnByAngleNew(90);
  stop();
  delay(50);

  forward(10, 100);
  stop();
  delay(50);

  turnByAngleNew(270);
  stop();
  delay(50);

  forward(avoidDist, 100);
  stop();
  delay(50);

  turnByAngleNew(270);
  stop();
  delay(50);

  forward(10, 100);
  stop();
  delay(50);

  turnByAngleNew(90);
  stop();
  delay(50);

  forward(3, 100);
  stop();
  delay(50);
  checkDist += avoidDist;
  resetEncoders();
}

// Robot detects if any object is infront of it, returns true if a object is present.
bool detectObject()
{
  proximitySensor.read();
  int leftReading = proximitySensor.countsFrontWithLeftLeds();
  int rightReading = proximitySensor.countsFrontWithRightLeds();

  if (leftReading >= objThreshold || rightReading >= objThreshold)
  {

    display.clear();      // Clears the OLED display.
    display.gotoXY(0, 0); // Sets the position on the OLED, where the message should be printed.
    display.print("theft");
    if (!checkTheft())
    {
      // avoid();
      display.clear();      // Clears the OLED display.
      display.gotoXY(0, 0); // Sets the position on the OLED, where the message should be printed.
      display.print("avoid");

      newAvoid();

      /// Husk !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
    }
    /*
                                  // If the sensor readings is more or equal to the threshold value, then the robot should print "Obstacle ahead!" to the OLED.
    display.clear();              // Clears the OLED display.
    display.gotoXY(0, 0);         // Sets the position on the OLED, where the message should be printed.
    display.print(F("Obstacle")); // Prints "Obstacle".
    display.gotoXY(0, 1);         // Sets the position on the OLED, where the next line should be printed.
    display.print(F("ahead!"));   // Prints "ahead!".
    return true;
     */

    resetEncoders();
    return true;
  }
  return false;
}

// Robot detects if any object is infront of it, returns true if a object is present.

bool linesensor()
{
  // Read line sensor values
  lineSenors.read(lineSensorValues);

  // detects when the distance to an object is readable
  if (lineSensorValues[0] > lineThreshold || lineSensorValues[1] > lineThreshold)
  {
    backward(5, 150);
    int rand = random(181, 350);
    turnByAngleNew(rand);
    resetEncoders();
    return true;
  }
  else if (lineSensorValues[2] > lineThreshold)
  {
    backward(5, 150);
    int rand = random(10, 179);
    turnByAngleNew(rand);
    resetEncoders();
    return true;
  }
  return false;

  /* if (getDistance() > 150)
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
 */
}

bool checkSurroundings()
{
  bool check = false;
  check = detectObject();
  display.clear();      // Clears the OLED display.
  display.gotoXY(0, 0); // Sets the position on the OLED, where the message should be printed.
  display.print("Object");

  check = linesensor();
  display.clear();      // Clears the OLED display.
  display.gotoXY(0, 0); // Sets the position on the OLED, where the message should be printed.
  display.print("forward");
  return check;
}

// It move the robot to given positions
void MoveToPos(int x = 0, int y = 0)
{
  display.clear();      // Clears the OLED display.
  display.gotoXY(0, 0); // Sets the position on the OLED, where the message should be printed.
  display.print("x: " + String(robotposx));

  display.gotoXY(0, 1); // Sets the position on the OLED, where the message should be printed.
  display.print("y: " + String(robotposy));

  delay(1000);

  // varibel for the vektor
  int newposx = 0;
  int newposy = 0;
  // Dette er bare en float
  int angle = 0;
  int dist = 0;

  // check if the positions need too add on or minus with
  /*if (x > robotposx)
  {
    newposx = x - robotposx;
  }
  else
  {
    newposx = robotposx - x;
  }

  if (y > newposy)
  {
    newposy = y - robotposy;
  }
  else
  {
    newposy = robotposy - y;
  }

  newposx = robotposx + newposx;
  newposy = robotposy + newposy; */

  newposx = x - robotposx;
  newposy = y - robotposy;

  display.clear();      // Clears the OLED display.
  display.gotoXY(0, 0); // Sets the position on the OLED, where the message should be printed.
  display.print("newx: " + String(newposx));

  display.gotoXY(0, 1); // Sets the position on the OLED, where the message should be printed.
  display.print("newy: " + String(newposy));
  delay(1500);

  // angle get round up it float return get convert to int
  angle = abs(robotangle - (atan2(newposy, newposx) * (180 / PI))); // makes angle from the vektor
  dist = sqrt(pow(newposx, 2) + pow(newposy, 2));                   // find length of the vektor
  if (angle < 0)
  {
    angle = 360 + angle;
  }

  display.clear();      // Clears the OLED display.
  display.gotoXY(0, 0); // Sets the position on the OLED, where the message should be printed.
  display.print("V: " + String(angle));

  display.gotoXY(0, 1); // Sets the position on the OLED, where the message should be printed.
  display.print("D: " + String(dist));

  delay(1000);

  Serial.println("newposx: ");
  Serial.println(newposx);

  Serial.println("newposy: ");
  Serial.println(newposy);

  Serial.println("angle: ");
  Serial.println(angle);
  /* den virk med dette men burde ikke bruges if (angle > robotangle)
  {
    angle = angle - robotangle;

  else
  {
    angle = robotangle - angle;
   */

  // Her we put the angle and distance robot too travel
  // turn
  turnByAngleNew(angle);
  forward(dist, 200);
}

/** \brief Robot turns right or left with a specified radius, angle and speed.
 *
 * \param dir 0 = Right 1 = Left
 */
void turnArc(bool dir = 0, int radius = 100, int speed = 100)
{
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

// Turns the robot a random angle
void turnRandom()
{
  int randomNumber = random(10, 359);
  turnByAngleNew(randomNumber);
  Serial.print(randomNumber);
  stop();
  delay(1000);
}

void navigateRandom()
{
  int randDist = random(10, 50);
  int randSpeed = random(100, 200);
  turnRandom();
  forward(randDist, randSpeed);
  delay(50);
}

void randomMovement(){
    for (int i = 0; i <6; i++)
    {
      display.clear();
      delay(500);
      int caseNumber = random(1,3);
      int distRandom = random(10,50);
      int speedRandom = random(25, 200);
      int turnR = (10, 359);
      display.gotoXY(0,0);
      display.print(caseNumber);
      delay(1000);
      switch (caseNumber)
      {
      case(1):
      display.clear();
      display.gotoXY(0,0);
      display.print(distRandom);
      display.gotoXY(0,1);
      display.print(speedRandom);
        forward2(distRandom, speed);
      break;
      case(2):
      display.clear();
      display.gotoXY(0,0);
      display.print(turnR);
        turnByAngleNew(turnR);
        delay(1500);
      break;
      }
    }
}

void turnByMag(int angle)
{
  float initHead = calculateMagHeading();
  turnByAngleNew(angle);
  float obtainedHead = calculateMagHeading();
  float magDiff = obtainedHead - initHead;
  int f;
  if (magDiff > angle)
  {
    f = 360 - (magDiff - angle);
  }
  if (angle > magDiff)
  {
    f = angle - magDiff;
  }

  Serial.print("Heading Error: " + String(f));
  if (abs(magDiff) - angle > turnThreshold)
  {
    turnByMag(f);
  }
}

void getMag()
{
  // Reads the magnetometer and stores the raw data in the global variables.
  readHMC5883LData(&mx, &my, &mz);
}

float calculateMagHeading()
{
  getMag();

  // Step 1: Offset correction
  float centeredX = mx - magOffsetX;
  float centeredY = my - magOffsetY;

  // Step 2: Apply the transformation matrix
  float calibratedX = transformationMatrix[0][0] * centeredX + transformationMatrix[0][1] * centeredY;
  float calibratedY = transformationMatrix[1][0] * centeredX + transformationMatrix[1][1] * centeredY;

  // Serial.println('$' + String(imu.m.x) + ' ' + String(imu.m.y) + ' ' + String(imu.m.z) + ' ' + String(calibratedX) + ' ' + String(calibratedY) + ';');

  // Calculate the heading in radians
  float heading = atan2(calibratedX, calibratedY);

  // Convert radians to degrees
  float headingDegrees = heading * (180.0 / M_PI);

  // Normalize to 0-360 degrees
  if (headingDegrees < 0)
  {
    headingDegrees += 360.0;
  }

  return headingDegrees;
}

void writeRegister(uint8_t deviceAddress, uint8_t regAddress, uint8_t value)
{
  SoftWire.beginTransmission(deviceAddress);
  SoftWire.write(regAddress);
  SoftWire.write(value);
  SoftWire.endTransmission();
}

uint8_t readRegister(uint8_t deviceAddress, uint8_t regAddress)
{
  SoftWire.beginTransmission(deviceAddress);
  SoftWire.write(regAddress);
  SoftWire.endTransmission(false);

  SoftWire.requestFrom(deviceAddress, (uint8_t)1);
  return SoftWire.read();
}

void readHMC5883LData(int16_t *x, int16_t *y, int16_t *z)
{
  SoftWire.beginTransmission(HMC5883L_ADDR);
  SoftWire.write(HMC5883L_REG_DATA_X_MSB);
  SoftWire.endTransmission(false);

  SoftWire.requestFrom(HMC5883L_ADDR, (int)6);
  *x = (SoftWire.read() << 8) | SoftWire.read();
  *z = (SoftWire.read() << 8) | SoftWire.read();
  *y = (SoftWire.read() << 8) | SoftWire.read();
}