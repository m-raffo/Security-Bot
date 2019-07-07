#include <M5Stack.h>
#include <Arduino.h>
#include <QTRSensors.h>
#include "FastLED.h"
#include "NewPing.h"

// Define led constants
#define NEOPIXEL_PIN 1
#define NUM_LEDS 7

// Define ultrasonic sensor constants
#define TRIGGER_PIN  26
#define ECHO_PIN     13
#define MAX_DISTANCE 200

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// Define QTR sensor variables
QTRSensors qtr;
const uint8_t qtrSensorCount = 3;
uint16_t lineSensorValues[qtrSensorCount];

// Define leds
CRGB leds[NUM_LEDS];

// Function prototypes
boolean performChecks();

void setup() {

  M5.begin();

  // Initialize the LEDS
  FastLED.addLeds<WS2811,NEOPIXEL_PIN,RGB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.clear();


  // Initialize the line sensor
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){2, 5, 15}, qtrSensorCount);
  delay(1000);

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

    FastLED.setBrightness(25);
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
 *
 * Sample output error message:

     Sensor tests:
     QTR - [SUCCESS] - (60, 49, 54)
     Ultrasonic - [FAIL] - (0.0000000000)

 * @return true if all checks good, false if not
 */
boolean performChecks() {
  // Define test variables
  boolean allGood = true;

  boolean qtrStatus = true;
  uint16_t qtrValues[qtrSensorCount];

  boolean sonicStatus = true;
  int sonicValue;

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


  return allGood;
}
