#include <M5Stack.h>
#include <Arduino.h>
#include <QTRSensors.h>
#include "FastLED.h"
#include "NewPing.h"
#include "utility/MPU9250.h"
#include "Drive.h"

// Define led constants
#define NEOPIXEL_PIN 1
#define NUM_LEDS 7

// Define ultrasonic sensor constants
#define TRIGGER_PIN  26
#define ECHO_PIN     13
#define MAX_DISTANCE 200

// Define IMU variables
MPU9250 IMU;

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// Define QTR sensor variables
QTRSensors qtr;
const uint8_t qtrSensorCount = 3;
uint16_t lineSensorValues[qtrSensorCount];

// Define leds
CRGB leds[NUM_LEDS];

// Function prototypes
boolean performChecks();
boolean arrayCompare(int *a, int *b, int len_a, int len_b);

void setup() {

  M5.begin();

  // Initialize motor control
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  // Initialize the LEDS
  FastLED.addLeds<WS2811,NEOPIXEL_PIN,RGB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.clear();

  // Initialize the line sensor
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){2, 5, 15}, qtrSensorCount);
  delay(1000);

  // Initialize, test and calibrate MPU sensor
  Wire.begin();

  IMU.MPU9250SelfTest(IMU.SelfTest);
  IMU.calibrateMPU9250(IMU.gyroBias, IMU.accelBias);
  IMU.initMPU9250();
  IMU.initAK8963(IMU.magCalibration);


  M5.Lcd.clear(BLACK);

  boolean systemChecks = performChecks();

  if(systemChecks) {
    // Play success sound here

    // Fancy green leds
    for(int i = 0; i < 7; i++) {
      fill_solid(leds, NUM_LEDS, CRGB::Black);
      leds[i] = CRGB::Green;
      FastLED.show();
      delay(50);
    }

    // Turn off leds
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();

    // Give a little time to see the test results
    delay(750);

    // Erase test output message
    M5.Lcd.clear(BLACK);

  } else {
    // Play fail sound here

    FastLED.setBrightness(50);
    for(;;) {  // Infinite loop

      // Flash red lights
      fill_solid(leds, NUM_LEDS, CRGB::Red);
      FastLED.show();
      delay(250);

      fill_solid(leds, NUM_LEDS, CRGB::Black);
      FastLED.show();
      delay(250);

    }
  }
}

void loop() {

  // Read line sensor values
  qtr.read(lineSensorValues);

  // Display sonar pings
  // M5.Lcd.clear(BLACK);
  M5.Lcd.setCursor(10, 10);
  M5.Lcd.printf("%03d", sonar.ping_cm());

  delay(50);


}

/**
 * Checks all systems of the robot and prints the test results to the screen of
 * the robot.
 *
 * NOTE: All functions of the robot must be initialized before calling this function
 * This function checks the following functions:
 *   QTR Sensor
 *   Ultrasonic Sensor
 *   Accelerometer
 *   Gyroscope
 *
 * Magnetometer is not tested (assumed if accel and gyro is functional, mag is too)
 *
 * Sample output error message:

     Sensor tests:
     QTR - [SUCCESS] - (60, 49, 54)
     Ultrasonic - [FAIL] - (0.0000000000)
     ...

 * @return true if all checks good, false if not
 */
boolean performChecks() {
  // Define test variables
  boolean allGood = true;

  boolean qtrStatus = true;
  uint16_t qtrValues[qtrSensorCount];

  boolean sonicStatus = true;
  int sonicValue;

  boolean accelStatus = true;
  int accelValues1[3];
  int accelValues2[3];

  boolean gyroStatus = true;
  int gyroValues1[3];
  int gyroValues2[3];

  // OPTIONAL: Uncomment to test leds with a rainbow gradient
  // for(int i = 0; i < 7; i++) {
  //   for (int colorIndex = 0; colorIndex < 255; colorIndex += 4) {
  //     leds[i] = ColorFromPalette( RainbowColors_p, colorIndex);
  //     FastLED.show();
  //   }
  //
  //   leds[i] = CRGB::Black;
  // }

  // Get line sensor values
  qtr.read(qtrValues);

  // Line sensor return 2500 when not plugged in
  if (qtrValues[0] == 2500 || qtrValues[1] == 2500 || qtrValues[2] == 2500) {
    allGood = false;
    qtrStatus = false;
  }

  // Ultrasonic sensor tests
  sonicValue = sonar.ping_cm();
  if(sonicValue == 0) {
    allGood = false;
    sonicStatus = false;
  }


  // Accel and gyro tests
  // Note: if the sensor is not connected, it will return random, but constant values

  IMU.readAccelData(IMU.accelCount);
  IMU.readGyroData(IMU.gyroCount);


  // memcpy(accelValues1, IMU.accelCount, sizeof(IMU.accelCount));

  // Save accel values
  accelValues1[0] = IMU.accelCount[0];
  accelValues1[1] = IMU.accelCount[1];
  accelValues1[2] = IMU.accelCount[2];

  // Save gyro values
  gyroValues1[0] = IMU.gyroCount[0];
  gyroValues1[1] = IMU.gyroCount[1];
  gyroValues1[2] = IMU.gyroCount[2];

  delay(250);

  IMU.readAccelData(IMU.accelCount);
  IMU.readGyroData(IMU.gyroCount);


  accelValues2[0] = IMU.accelCount[0];
  accelValues2[1] = IMU.accelCount[1];
  accelValues2[2] = IMU.accelCount[2];

  gyroValues2[0] = IMU.gyroCount[0];
  gyroValues2[1] = IMU.gyroCount[1];
  gyroValues2[2] = IMU.gyroCount[2];

  if (arrayCompare(accelValues1, accelValues2, 3, 3)) {
    allGood = false;
    accelStatus = false;
  }

  if (arrayCompare(gyroValues1, gyroValues2, 3, 3)) {
    allGood = false;
    gyroStatus = false;
  }

  if (arrayCompare(accelValues1, gyroValues1, 3, 3) || arrayCompare(accelValues2, gyroValues2, 3, 3)) {
    allGood = false;
    gyroStatus = false;
    accelStatus = false;
  }

  // Test motors
  sendDrive(25, 25, 13000);
  delay(250);
  sendDrive(-25, -25, 13000);
  delay(300);

  // Display test results
  M5.Lcd.println("Sensor tests:");

  // Print line sensor results
  M5.Lcd.print("QTR - [");
  if(qtrStatus) {
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.print("SUCCESS");

  } else {
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.print("FAIL");
  }

  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.printf("] - (%d, %d, %d)\n", qtrValues[0], qtrValues[1], qtrValues[2]);

  // Print ultrasonic results
  M5.Lcd.print("Ultrasonic - [");
  if(sonicStatus) {
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.print("SUCCESS");

  } else {
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.print("FAIL");
  }

  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.printf("] - (%d)\n", sonicValue);

  // Print accel sensor results
  M5.Lcd.print("Accel - [");
  if(accelStatus) {
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.print("SUCCESS");

  } else {
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.print("FAIL");
  }

  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.printf("] - (%d, %d, %d) (%d, %d, %d)\n", accelValues1[0], accelValues1[1], accelValues1[2], accelValues2[0], accelValues2[1], accelValues2[2]);

  // Print gyro sensor results
  M5.Lcd.print("Gyro - [");
  if(gyroStatus) {
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.print("SUCCESS");

  } else {
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.print("FAIL");
  }

  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.printf("] - (%d, %d, %d) (%d, %d, %d)\n", gyroValues1[0], gyroValues1[1], gyroValues1[2], gyroValues2[0], gyroValues2[1], gyroValues2[2]);


  return allGood;
}

boolean arrayCompare(int *a, int *b, int len_a, int len_b){
     int n;
     // if their lengths are different, return false
     if (len_a != len_b) return false;
     // test each element to be the same. if not, return false
     for (n=0;n<len_a;n++) if (a[n]!=b[n]) return false;
     return true;
}
