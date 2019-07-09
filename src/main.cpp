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

// Define ultrasonic sensor constants and variables
#define TRIGGER_PIN  26
#define ECHO_PIN     13
#define MAX_DISTANCE 200

// Define movement constants

/////////////////////////////////////////
// TEMPORARY VALUES: ADJUSTMENT NEEDED //
/////////////////////////////////////////

#define ACCEPTABLE_DRIFT    100
#define CORRECTION_DRIFT    100
#define TURN_BACKUP_AMOUNT  3200
#define TURN_AMOUNT_90      1000
#define TURN_AMOUNT_180     1000
#define TURN_SPEED          8000
#define FORWARD_SPEED       8000
#define LINE_THRESHOLD      400
#define FIND_LINE_DELAY     500
#define MAX_PATROL_LENGTH   5
#define STATE_CHANGE_DELAY  250


NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// Define IMU variables
MPU9250 IMU;

// Define QTR sensor variables
QTRSensors qtr;
const uint8_t qtrSensorCount = 3;
uint16_t lineSensorValues[qtrSensorCount];

// Define leds
CRGB leds[NUM_LEDS];

// State machine variables

/*
  States:
    0 - setup
    1 - mapping
 */
int state = 0;

/*
  States:
    0 - wait
    1 - straight and follow line
    2 - backing up
    3 - turning (left)
    4 - 180deg turn then go forward
    5 - Go forward until line is found
    6 - make 180deg turn to ensure end of patrol has not been reached

 */
int movementState = 1;
int movementStartMillis = 0;
int movementWaitMillis = 0;
int movementDuration;
bool movementInitTaken = false;

// Patrol path storage structure
typedef struct {
  // The time (in millis) it takes for the robot to complete this line segment
  int length;

  // The distance measured at the start of the line segment
  int startingDistance;

  // The distance measured and the end of the line segment
  int endingDistance;

  /*
    The next turn for the robot:
    0 - left
    1 - right
   */
  int nextTurn;
} Path;

// An array to store the patrol infomation
// NOTE: May need to increase patrol path limit
Path patrol[MAX_PATROL_LENGTH];
int patrolIndex = 0;

// Function prototypes
boolean performChecks();
boolean arrayCompare(int *a, int *b, int len_a, int len_b);
void waitForButtonPress(String);
void throwError(String);
void printPatrol();

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

    boolean wait = true;

    M5.Lcd.setCursor(35, 220);
    M5.Lcd.println("Press 'A' to continue with program launch");

    FastLED.setBrightness(50);

    while (wait) {  // Wait until button press
      // Flash red lights
      if(millis() % 500 > 250) {
        fill_solid(leds, NUM_LEDS, CRGB::Red);
      } else {
        fill_solid(leds, NUM_LEDS, CRGB::Black);
      }

      FastLED.show();

      M5.update();
      if(M5.BtnA.wasReleased()) {
        wait = false;
      }
    }

    M5.Lcd.clear(BLACK);

    delay(100);

  }

  M5.Lcd.setCursor(35, 20);
  M5.Lcd.println("Line sensor calibration: 'A' to begin");

  M5.update();
  boolean wait = true;
  while(wait) {
    M5.update();
    wait = !M5.BtnA.wasReleased();
  }

  M5.Lcd.clear(BLACK);
  M5.Lcd.setCursor(35, 20);
  M5.Lcd.println("Calibrating now...");
  float percent = 0;
  for (uint8_t i = 0; i < 100; i++) {
    percent += 100.0 / 100.0;
    M5.Lcd.setCursor(35, 40);
    M5.Lcd.printf("%d%%", (int) percent);


    qtr.calibrate();
    // delay(20);
  }

  M5.Lcd.clear(BLUE);
  delay(250);
  M5.Lcd.clear(BLACK);

  waitForButtonPress("Press A to continue");

  state = 1;

}

void loop() {

  ////////////////////////
  // READ SENSOR VALUES //
  ////////////////////////
  M5.update();

  // Read line sensor values
  qtr.read(lineSensorValues);
  int linePosition = qtr.readLineBlack(lineSensorValues);
  int lineTotal = lineSensorValues[0] + lineSensorValues[1] + lineSensorValues[2];

  // Read ultrasonic sensor values
  int distanceCm = sonar.ping_cm();


  ///////////////////
  // MAPPING STATE //
  ///////////////////

  if (state == 1) {

    if (millis() < movementWaitMillis) {
      // Stop and wait until the delay time is over
      sendDrive(0, 0);

    } else {

      // Go straight and correct direction
      if (movementState == 1) {
        // If initial measurements are not taken, take them
        if(!movementInitTaken) {
          if(patrolIndex >= MAX_PATROL_LENGTH) {
            throwError("ERROR: Max patrol length reached");
          }

          movementInitTaken = true;
          movementStartMillis = millis();
          patrol[patrolIndex].startingDistance = distanceCm;
          patrolIndex++;
        }

        // If line has stopped, switch to state 2
        if(lineTotal < LINE_THRESHOLD) {
          // NOTE: MAY NEED TO STOP TO TAKE FINAL DISTANCE READING...
          patrol[patrolIndex - 1].endingDistance = distanceCm;
          patrol[patrolIndex - 1].length = millis() - movementStartMillis;

          movementState = 2;
          movementDuration = TURN_BACKUP_AMOUNT;
          movementInitTaken = false;
          movementWaitMillis = millis() + STATE_CHANGE_DELAY;


        } else // next statement

        // If on track, go straight
        if (linePosition > 1000 - ACCEPTABLE_DRIFT && linePosition < 1000 + ACCEPTABLE_DRIFT) {
          sendDrive(FORWARD_SPEED, FORWARD_SPEED);
        } else if (linePosition > 1000) { // Turn left
          sendDrive(FORWARD_SPEED - CORRECTION_DRIFT, FORWARD_SPEED);
        } else { // Turn right
          sendDrive(FORWARD_SPEED, FORWARD_SPEED - CORRECTION_DRIFT);
        }
      }

      // Back up until the time specified
      if (movementState == 2) {

        // Save starting millis time
        if (!movementInitTaken) {
          movementInitTaken = true;
          movementStartMillis = millis();
        }

        sendDrive(-FORWARD_SPEED, -FORWARD_SPEED);

        // If backup complete, switch to turning mode
        if (millis() > movementStartMillis + movementDuration) {
          movementState = 3;
          movementDuration = TURN_AMOUNT_90;
          movementInitTaken = false;
          movementWaitMillis = millis() + STATE_CHANGE_DELAY;

        }
      }

      // 90 deg turn right to check intersection directions
      if (movementState == 3) {

        // Save starting millis time
        if (!movementInitTaken) {
          movementInitTaken = true;
          movementStartMillis = millis();
        }

        sendDrive(TURN_SPEED, -TURN_SPEED);

        // If turn complete, check for lines
        if (millis() > movementStartMillis + movementDuration) {

          /*
          If a line is detected, the robot must make a 180 turn to continue on its
          path. (Because the line sensor is behind the robots pivot point. So it
          is behind the line.)
           */

          if(lineTotal > LINE_THRESHOLD) {
            // The next turn is left, save in patrol
            patrol[patrolIndex - 1].nextTurn = 0;

            movementState = 4;
            movementDuration = TURN_AMOUNT_180;
            movementInitTaken = false;
            movementWaitMillis = millis() + STATE_CHANGE_DELAY;

          } else {
            /*
              If here:
              need to make 180deg turn, ensure that a line is detected
              then 180deg turn back and go forward.

              This is necessary to ensure that the end of the patrol has not been
              reached.
             */

            movementState = 6;
            movementDuration = TURN_AMOUNT_180;
            movementInitTaken = false;
            movementWaitMillis = millis() + STATE_CHANGE_DELAY;


            // Continue on to next line

            // The next turn is right, save in patrol
            patrol[patrolIndex - 1].nextTurn = 1;

            movementState = 5;
            movementDuration = FIND_LINE_DELAY;
            movementInitTaken = false;
            movementWaitMillis = millis() + STATE_CHANGE_DELAY;

          }
        }
      }

      // 180 deg turn, then continue on next line
      if (movementState == 4) {
        // Save starting millis time
        if (!movementInitTaken) {
          movementInitTaken = true;
          movementStartMillis = millis();
        }

        sendDrive(TURN_SPEED, -TURN_SPEED);

        // If turn complete, continue forward onto next line
        if (millis() > movementStartMillis + movementDuration) {

          movementState = 1;
          movementInitTaken = false;
          movementWaitMillis = millis() + STATE_CHANGE_DELAY;

        }
      }

      // Move foward a brief amount to find the next line
      if (movementState == 5) {
        // Save starting millis time
        if (!movementInitTaken) {
          movementInitTaken = true;
          movementStartMillis = millis();
        }

        sendDrive(FORWARD_SPEED, FORWARD_SPEED);

        // If forward complete, switch to next line following mode
        if (millis() > movementStartMillis + movementDuration) {

          // Check the line has been found in time
          if (lineTotal > LINE_THRESHOLD) {
            movementState = 1;
            movementInitTaken = false;
            movementWaitMillis = millis() + STATE_CHANGE_DELAY;

          } else {
            // Throw error: LINE Lost
            // TODO: Or end of patrol reached.
            throwError("ERROR: Line lost. Program abort");
          }
        }
      }

      // Make 180deg turn, check line is detected, then turn back and continue on mapping
      if (movementState == 6) {
        // Save starting millis time
        if (!movementInitTaken) {
          movementInitTaken = true;
          movementStartMillis = millis();
        }


        sendDrive(TURN_SPEED, -TURN_SPEED);

        // If turn complete, continue forward onto next line
        if (millis() > movementStartMillis + movementDuration) {

          // Check the line has been found at end of turn
          if (lineTotal > LINE_THRESHOLD) {
            movementState = 4;
            movementDuration = TURN_AMOUNT_180;
            movementInitTaken = false;
            movementWaitMillis = millis() + STATE_CHANGE_DELAY;

          } else {
            // Throw error: LINE Lost
            // TODO: Or end of patrol reached.
            throwError("ERROR: Patrol end reached, no futher instructions.");
          }

        }
      }
    }
  }

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

  M5.Lcd.clear(BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.println("Testing...");

  M5.Lcd.print("QTR... ");


  // Get line sensor values
  qtr.read(qtrValues);

  // Line sensor return 2500 when not plugged in
  if (qtrValues[0] == 2500 || qtrValues[1] == 2500 || qtrValues[2] == 2500) {
    allGood = false;
    qtrStatus = false;
  }

  M5.Lcd.println("done");
  M5.Lcd.print("Ultrasonic... ");


  // Ultrasonic sensor tests
  sonicValue = sonar.ping_cm();
  if(sonicValue == 0) {
    allGood = false;
    sonicStatus = false;
  }

  M5.Lcd.println("done");
  M5.Lcd.print("IMU... ");


  // Accel and gyro tests
  // Note: if the sensor is not connected, it will return random, but constant values

  IMU.readAccelData(IMU.accelCount);
  IMU.readGyroData(IMU.gyroCount);

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

  M5.Lcd.println("done");
  M5.Lcd.print("Motors... ");

  // Test motors - GRBL
  // sendDrive(25, 25, 13000);
  // delay(250);
  // sendDrive(-25, -25, 13000);
  // delay(300);

  // Test motors - New nano firmware
  sendDrive(5000, 5000);
  delay(250);
  sendDrive(-5000, -5000);
  delay(250);
  sendDrive(0, 0);


  M5.Lcd.clear(BLACK);
  M5.Lcd.setCursor(0, 0);

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

void waitForButtonPress(String message) {
  M5.Lcd.clear(BLACK);
  M5.Lcd.setCursor(35, 20);
  M5.Lcd.println(message);

  boolean wait = true;
  while(wait) {
    M5.update();
    wait = !M5.BtnA.wasReleased();
  }
  M5.Lcd.clear(BLACK);
}

void throwError(String message) {
  M5.Lcd.clear(BLACK);
  M5.Lcd.setTextColor(RED, BLACK);

  M5.Lcd.setCursor(0, 0);
  M5.Lcd.println(message);

  while(true) {
    if(millis() % 500 > 250) {
      fill_solid(leds, NUM_LEDS, CRGB::Red);
    } else {
      fill_solid(leds, NUM_LEDS, CRGB::Black);
    }

    FastLED.show();
  }


}

/*
  Prints the patrol information to the lcd
    DISTANCE  START Distance  END Distance
    00000000  000             000
 */
void printPatrol() {
  M5.Lcd.clear(BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.println("DISTANCE  START Distance  END Distance");
  for(int i = 0; i < MAX_PATROL_LENGTH; i++) {
    M5.Lcd.printf("%08d  %04d             %04d\n\r", patrol[i].length, patrol[i].startingDistance, patrol[i].endingDistance);
  }
}
