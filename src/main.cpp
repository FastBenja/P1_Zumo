// Include libareis
#include <Arduino.h>                    //For standard arduino functionallity
#include <Zumo32U4.h>                   //For easily interfacing with the Zumo platform
#include <Wire.h>                       //Dependency for Zumo32U4 used to communicate with IMU
#include <SoftI2C-master/src/SoftI2C.h> //For bitbanging a I2C bus
#include <PID_v1.h>                     //For closed loop control

int speed = 100;           // Target Zumo speed
#define thieveThreshold 3  // Maximum difference in measurements before detecting a thieve
#define lineThreshold 1000 // Threshold for detecting a line
#define objThreshold 5     // Threshold for detecting a object

// I2C addresses
#define MPU6050_ADDR 0x68  // Address of MPU6050
#define HMC5883L_ADDR 0x1E // Address of HMC5883L

// MPU6050 registers
#define MPU6050_REG_PWR_MGMT_1 0x6B  // For waking up the MPU
#define MPU6050_REG_INT_PIN_CFG 0x37 // For bypassing the I2C bus

// HMC5883L registers
#define HMC5883L_REG_CONFIG_A 0x00   // Config
#define HMC5883L_REG_MODE 0x02       // Config
#define HMC5883L_REG_DATA_X_MSB 0x03 // Data first byte

// These are the postions the robot need to check
const int check[3][2] = {{20, 47}, {40, 38}, {65, 10}};

// Calculated transformation matrix ans offset for magnetometer correction
float transformationMatrix[2][2] = {{-0.00724577, -0.00242076}, {0.00220427, -0.00659776}};
float magOffsetX = -98.54545454, magOffsetY = -338.4919786;

// Charging station location in X,Y coordinates
const int charger[2] = {0, 0};

// Values for storing the raw mag readings.
int16_t mx, my, mz;

// Variables for PID
double pidSetpoint, pidInput, pidOutput;

// Parameters for PID
double Kp = 1, Ki = 2, Kd = 0.25;

// variables for gyro
int16_t gyroOffset = 0;
uint32_t turnAngle = 0;
int16_t turnRate = 0;
uint16_t gyroLastUpdate = 0;

// Variable to sore linesensor measurements
unsigned int lineSensorValues[3];

// Variables for wheel dimensions and encoder counts pr revolution
float wheelCirc = 122.52 / 10;
float cpr = 909.7;

// Variables to store the postion of the robot
int robotposx = 0;
int robotposy = 0;
int robotangle = 0;

// Variables to store a robot position
int checkposx = 0;
int checkposy = 0;

// Variable to set the size of the object that the robot should go around
int avoidDist = 15;

// Variable to store how long the robot have traveled in a specific move
float checkDist = 0;

// Instantiation of objects
Zumo32U4OLED display;                                               // Display
Zumo32U4IMU imu;                                                    // IMU (Internal Measurement Unit)
Zumo32U4ButtonA buttonA;                                            // Button A
Zumo32U4Buzzer buzzer;                                              // Buzzer
Zumo32U4Encoders encoders;                                          // Encoders
Zumo32U4Motors motors;                                              // Motors
Zumo32U4ProximitySensors proximitySensor;                           // Proximity sensors
Zumo32U4LineSensors lineSenors;                                     // Linesensors
SoftI2C SoftWire(4, 20);                                            // SoftI2C object on Zumo pins 4 and 20
PID myPID(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, DIRECT); // PID for closed loop mag turning

// Function definitions:
void ALARM(uint32_t);
float offsetAngValue(float, float);
void resetEncoders();
void stop();
float getDistance();
void backwards(int, int);
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
void newAvoid();
bool linesensor();
void forward2(uint16_t, uint16_t);
void navigateRandom();
void turnByMag(float, float);
void getMag();
float getMagHeading();
void writeRegister(uint8_t, uint8_t, uint8_t);
uint8_t readRegister(uint8_t, uint8_t);
void readHMC5883LData(int16_t *, int16_t *, int16_t *);
void randomMovement();
float calculateAngleError(float, float);
void turnRandom();

void setup()
{
  Serial.begin(9600);                                             // Initialize the serial monitor
  display.init();                                                 // Initialize the display
  turnSensorSetup();                                              // Initialize and setup the turn-sensor
  encoders.init();                                                // Initialize encoders
  proximitySensor.initFrontSensor();                              // initialize proximity-sensors
  uint16_t customBrightnessLevels[] = {1, 5, 10, 15, 30, 45};     // Define custom brightnesses for proximity-sensor emmitters
  proximitySensor.setBrightnessLevels(customBrightnessLevels, 6); // Set custom brightnesses for proximity-sensor emmitters
  lineSenors.initThreeSensors();                                  // Initialize linesensors
  SoftWire.begin();                                               // Initialize bit-banging

  // Initialize MPU6050
  writeRegister(MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1, 0x00); // Wake up MPU6050

  // Enable I2C bypass mode
  writeRegister(MPU6050_ADDR, MPU6050_REG_INT_PIN_CFG, 0x02); // Bypass I2C ON

  // Initialize HMC5883L
  writeRegister(HMC5883L_ADDR, HMC5883L_REG_CONFIG_A, 0x70); // Average of 8 measurements, 15 Hz default
  writeRegister(HMC5883L_ADDR, HMC5883L_REG_MODE, 0x00);     // Continuous measurement mode

  Wire.begin();                       // Initialize default I2C bus
  imu.init();                         // Initialize IMU
  imu.enableDefault();                // Enable IMU as default
  imu.configureForTurnSensing();      // Setup IMU for turn-sensing
  uint32_t seed = imu.m.x ^ micros(); // Seed random algorithm

  // Setup PID
  myPID.SetOutputLimits(-400, 400); // Set output limits to motor input range.
  myPID.SetMode(AUTOMATIC);         // Configure for AUTO
}

void loop()
{
  ledRed(LOW);
  turnByMag(230, 0.25);
  delay(500);
  turnByMag(80, 0.25);
  delay(500);
  turnByMag(290, 0.25);
  delay(500);
  turnByMag(40, 0.25);
  delay(500);
  turnByMag(150, 0.25);
  ledRed(HIGH);
  buttonA.waitForPress();
}

/**
 * \brief Execuetes a triangular avoid move
 */
void newAvoid()
{
  turnByMag(20, 0.25);
  forward(25, 150);
  turnByMag(310, 0.25);
  forward(25, 150);
  turnByMag(20, 0.25);
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
float offsetAngValue(float value, float offset)
{
  // Add offset and wrap within 0-359 range
  float newValue = fmod(value + offset, 360);
  // If result is negative, wrap it to 359
  if (newValue < 0)
  {
    newValue += 360;
  }
  return newValue;
}

/** \brief Function takes an angle from 0 to 360 and calculates the shortest angle from the current to the setpoint angle.
 * \param currentAngle The current absolute angle of the robot
 * \param setpointAngle The desired angle the robot should obtain
 * \retval The shortest angle from the current to the setpoint
 **/
float calculateAngleError(float currentAngle, float setpointAngle)
{
  float error = setpointAngle - currentAngle;

  // Wrap the error to the range -180 to 180
  if (error > 180)
  {
    error -= 360;
  }
  else if (error < -180)
  {
    error += 360;
  }

  return error;
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
 * \brief
 * \retval Returns the accumulated distance that the robot has traveled in cm, based on the encoder counts.
 */
float getDistance()
{
  // get distance in cm
  long int countsL = encoders.getCountsLeft();
  long int countsR = encoders.getCountsRight();

  float distanceL = countsL / cpr * wheelCirc;
  float distanceR = countsR / cpr * wheelCirc;

  return (distanceL + distanceR) / 2;
}

/**
 * \brief Go backwards a distance with a specified speed
 * \param dist Specify the length to go
 * \param speed Specify the movement speed
 */
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
  stop();

  // convert distance and the angle of robot to x and y coordinates
  robotposx = robotposx - dist * cos(robotangle / (180 / PI)); // ikke færdig
  robotposy = robotposy - dist * sin(robotangle / (180 / PI));
}

/**
 * \brief Saves the curent position updates global var with current position in x and y coordinates // not
 */
void savePos()
{
  checkposx = robotposx;
  checkposy = robotposy;
}

/**
 * \brief Read the gyro and update the angle.  This should be called as
 * frequently as possible while using the gyro to do turns.
 */
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

/**
 * \brief This should be called to set the starting point for measuring a turn.  After calling this, turnAngle will be 0.
 */
void turnSensorReset()
{
  gyroLastUpdate = micros();
  turnAngle = 0;
}

/**
 * \retval Returns the estimated turn angle in degrees.
 */
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
        motors.setSpeeds(-100, 100); // Drive left slow
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
        motors.setSpeeds(100, -100); // DRive right slow
      }
    }
  }
  robotangle = offsetAngValue(robotangle, ang);                      // Save new rob angle
  motors.setSpeeds(0, 0);                                            // Stop
  Serial.println("Turned by angle to: " + String(ang) + " Degrees"); // Print resulting angle
}

/**
 * \brief Rotates twice while recording values from the proximity sensors.
 * \return Returns true if a thieve is detected. Returns false if no thieve is detected
 */
bool checkTheft()
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
    if (angle == 180) // When the robot is at a half turn
    {
      halfway = true; // Set the halfway mark
    }
    if (halfway && angle == 0) // When robot have done a full rotation
    {
      numTurns++; // Increment the number of full turns
    }
    if (angle == 0) // When robot at the 0 position
    {
      halfway = false; // Reset the halfway mark
    }

    // Store reading from proximity sensor.
    // If the angle of the robot is devisible by 40 with a remainder of 0, take a measurement
    // Since 360/40 is 9, 9 measurements will be taken during each rotation
    if (angle % 40 == 0 && stepNoted == false)
    {
      // Reads values for the front proximity sensor, if there is an object in range of the left or right IR light.
      proximitySensor.read();
      // Add the two readings for an avg.
      int proxReading = (proximitySensor.countsFrontWithLeftLeds() + proximitySensor.countsFrontWithRightLeds());
      // Set the stepNoted mark to make sure that only one measurement will be recorded for each step.
      stepNoted = true;
      // Calculate the measurement index number, the execution pointer will only be here if angle%40=0 therefor angle/40 will always be an integer.
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
    buttonA.waitForButton(); // If a thieve is detected, wait for user (operator) to take action.
    return true;
  }
  return false;
}

/**
 * \brief Go forward a distance with a specified speed while continuously monitoring surroundings
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

    // If right is ahead, diff is negative. If right is behind, diff is positive.
    int compSpeed = speed + diff * 5;
    motors.setSpeeds(speed, compSpeed);

    // Print the left and right speed to the screen
    display.clear();
    display.gotoXY(0, 0);
    display.print(speed);
    display.gotoXY(0, 1);
    display.print(compSpeed);
  }
  stop();

  // Convert distance and the angle of robot to x and y coordinates
  robotposx = robotposx + dist * cos(robotangle / (180 / PI));
  robotposy = robotposy + dist * sin(robotangle / (180 / PI));

  // Debugging
  /*
  display.clear();          // Clears the OLED display.
  display.gotoXY(0, 0);     // Sets the position on the OLED, where the message should be printed.
  display.print(robotposx); // 20                                  // Prints "Obstacle".
  display.gotoXY(0, 1);     // Sets the position on the OLED, where the next line should be printed.
  display.print(robotposy); // 46
  */
}

/**
 * \brief Go forward a distance with a specified speed without monitoring the surroundings
 * \param dist Specify the distance that the robot should, move only allows posetive integers.
 * \param speed Specify the speed at which the robot should move, only allows posetive integers.
 */
void forward2(uint16_t dist = 0, uint16_t speed = 0)
{
  int diff = 0;
  resetEncoders();
  checkDist = 0;
  while (getDistance() + checkDist < dist)
  {
    diff = encoders.getCountsLeft() - encoders.getCountsRight();

    // If right is ahead, diff is negative. If right is behind, diff is positive.

    // Serial.println(diff);
    int compSpeed = speed + diff * 5;
    motors.setSpeeds(speed, compSpeed);
  }
  stop();

  // Convert distance and the angle of robot to x and y coordinates
  robotposx = robotposx + dist * cos(robotangle / (180 / PI));
  robotposy = robotposy + dist * sin(robotangle / (180 / PI));
}

/**
 * \brief Robot detects if any object is infront of it.
 * \returns Returns true if a object is present.
 */
bool detectObject()
{
  proximitySensor.read();
  int leftReading = proximitySensor.countsFrontWithLeftLeds();
  int rightReading = proximitySensor.countsFrontWithRightLeds();

  if (leftReading >= objThreshold || rightReading >= objThreshold) // If left or right reading is above the preset threshold.
  {

    display.clear();      // Clears the OLED display.
    display.gotoXY(0, 0); // Sets the position on the OLED, where the message should be printed.
    display.print("theft");
    if (!checkTheft()) // If it is not a thieve
    {
      display.clear();      // Clears the OLED display.
      display.gotoXY(0, 0); // Sets the position on the OLED, where the message should be printed.
      display.print("avoid");

      newAvoid(); // Do obstacle avoidance
    }
    resetEncoders();
    return true;
  }
  return false;
}

/**
 * \brief Should be called very often to check if the robot is on top of a line.
 * If a line is detected the robot will reverse and obtain a new direction from that point.
 * \returns Returns true if a line was detected and the new direction has been obtained
 */
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
}

/**
 * \brief A function that handels all obstacles that the robot might encounter, should be called as often as possible.
 * \returns Returns true if the robot has interacted with any obstacle
 */
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

/**
 * \brief Executie a move of the robot to the given position
 * \param x the x-coordinate of the ending position
 * \param y the y-coordinate of the ending position
 */
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

  newposx = x - robotposx;
  newposy = y - robotposy;

  display.clear();      // Clears the OLED display.
  display.gotoXY(0, 0); // Sets the position on the OLED, where the message should be printed.
  display.print("new-x: " + String(newposx));

  display.gotoXY(0, 1); // Sets the position on the OLED, where the message should be printed.
  display.print("new-y: " + String(newposy));
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

  // Here we put the angle and distance for the robot to travel
  turnByAngleNew(angle);
  forward(dist, 200);
}

/**
 * \brief Gyro setup and convenience functions
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

/**
 * \brief Turns the robot a random angle
 */
void turnRandom()
{
  int randomNumber = random(10, 359);
  turnByAngleNew(randomNumber);
  Serial.print(randomNumber);
  stop();
  delay(1000);
}

/**
 * \brief Turns the robot a random angle and starts a move with a random speed and a random length
 */
void navigateRandom()
{
  int randDist = random(10, 50);
  int randSpeed = random(100, 200);
  turnRandom();
  forward(randDist, randSpeed);
  delay(50);
}

/**
 * \brief Executies a precision turn based on the magnetometer reading
 * \param angle Specify the angle to turn by, relative to the robot current position.
 * \param maxError Specify the precision level of the turn, the closed loop turning will stop after being within the limit for 2 seconds
 *  
 */
void turnByMag(float angle, float maxError)
{
  // Get the initial heading
  float initHead = getMagHeading();
  Serial.println("InitHead: " + String(initHead));

  // Calculate the target (final) heading
  float finalHeading = initHead + angle;

  // Normalize the target heading to 0–359.999
  if (finalHeading >= 360.0)
  {
    finalHeading -= 360.0;
  }
  else if (finalHeading < 0.0)
  {
    finalHeading += 360.0;
  }
  Serial.println("Target/FinalHead: " + String(finalHeading));

  // Allow some time for the robot to begin turning
  delay(3000);

  // Try resetting the PID
  myPID.SetMode(MANUAL);
  myPID.SetMode(AUTOMATIC);

  bool inOkBand = false;
  unsigned long startMark;
  // Enter the loop to correct heading
  while (true)
  {
    // Read the current heading
    float obtainedHead = getMagHeading();

    // Calculate the error using the wrapped error function
    float error = calculateAngleError(obtainedHead, finalHeading);

    // Use the error as input for the PID controller
    pidInput = error;
    pidSetpoint = 0; // PID should drive the error to 0
    myPID.Compute();

    // Apply the PID output to the motors
    motors.setSpeeds(pidOutput, -pidOutput);

    Serial.println("Current Error: " + String(error) + " Current pidOut: " + String(pidOutput));

    if (abs(error) <= maxError && !inOkBand)
    {
      startMark = millis();
      inOkBand = true;
    }
    else
    {
      inOkBand = false;
    }

    // If the error is within the allowed range, stop the motors and exit
    if (abs(error) <= maxError && startMark + 2000 > millis())
    {
      motors.setSpeeds(0, 0);
      break;
    }
  }

  // Print the final heading for debugging
  float endHead = getMagHeading();
  Serial.println("EndHead: " + String(endHead));
}

/**
 * \brief Makes the most recent magnetometer data available in mx, my and mz
 */
void getMag()
{
  // Reads the magnetometer and stores the raw data in the global variables.
  readHMC5883LData(&mx, &my, &mz);
}

/**
 * \brief Updates and calculate the absolute heading of the robot based on the magnetometer
 * \returns Return the absolute magnetic heading of the robot, not compensated for the magnetic declination of the location the robot is working in.
 */
float getMagHeading()
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
/**
 * \brief Write a value to a address of an i2c device
 */
void writeRegister(uint8_t deviceAddress, uint8_t regAddress, uint8_t value)
{
  SoftWire.beginTransmission(deviceAddress);
  SoftWire.write(regAddress);
  SoftWire.write(value);
  SoftWire.endTransmission();
}

/**
 * \brief Read the value of an address of an i2c device
 */
uint8_t readRegister(uint8_t deviceAddress, uint8_t regAddress)
{
  SoftWire.beginTransmission(deviceAddress);
  SoftWire.write(regAddress);
  SoftWire.endTransmission(false);

  SoftWire.requestFrom(deviceAddress, (uint8_t)1);
  return SoftWire.read();
}

/**
 * \brief Read raw data of magnetometer and store the readings in x, y and z vars.
 */
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