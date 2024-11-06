#include <Arduino.h>
#include <Wire.h>
#include <Zumo32U4.h>
#include <PololuOLED.h>

int speed = 20;

Zumo32U4OLED display;
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
  while (getDistance() <= dist)
  {
    int diff = encoders.getCountsLeft() - encoders.getCountsRight();
    Serial.println(diff);
    if (diff == 0)
    {
      motors.setSpeeds(speed, speed);
    }
    else
    {
      motors.setSpeeds(speed, speed/diff);
    }
  }
  stop();
}

// Go backwards a distance with a specified speed
void backward(int dist = 0, int speed = 0)
{
  resetEncoders();
  while (getDistance() >= -dist)
  {
    motors.setSpeeds(-speed, -speed);
  }
  stop();
}

// Avoid collision with a object
void avoid()
{
  backward(100, 50);
  // turn
  forward(50, 50);
  // turn
  forward(200, 50);
}

// there check if there theft
void checkTheft()
{
}

// is ways save object there in the way for robot
void saveObject()
{
}

/*???*/
void pose(int x, int y)
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

void loop()
{
  // put your main code here, to run repeatedly:
  forward(100, 400);
  backward(200, 400);
}