#include <M5Stack.h>

#define LEFT90            65
#define RIGHT90           65
#define STANDARD_FOWARD   15000
#define T360              263
#define T180              131

void sendDrive(int left, int right, int speed) {
  Serial2.printf("$J = G21 G91 X%d Y%d F%d", left, right, speed);
  Serial2.print("\r\n\r\n");
}

void right90(int speed) {
  sendDrive(RIGHT90, -RIGHT90, speed);
}

void right90(){
  sendDrive(RIGHT90, -RIGHT90, 8000);
}

void right180(){
  sendDrive(T180, -T180, 8000);
}

void right360() {
  sendDrive(T360, -T360, 8000);
}

void left90(int speed) {
  sendDrive(-LEFT90, LEFT90, speed);
}

void left90(){
  sendDrive(-LEFT90, LEFT90, 8000);
}

void forward(int amount) {
  sendDrive(amount, amount, STANDARD_FOWARD);
}

void forward(int amount, int speed) {
  sendDrive(amount, amount, speed);
}

void forwardInch(float inches) {
  // Note: 100 units = one rotation = 7 inches
  // 1 inch = 14.28 units

  int amount = inches * 14.28;
  sendDrive(amount, amount, STANDARD_FOWARD);
}

void forwardInch(float inches, int speed) {
  int amount = inches * 14.28;
  sendDrive(amount, amount, speed);
}

int getUnitsInch(float inches) {
  return inches * 14.28;
}

void forwardCm(float cm) {
  // 1cm = 5.62 units
  int amount = cm * 5.62;
  sendDrive(amount, amount, STANDARD_FOWARD);
}

void forwardCm(float cm, int speed) {
  int amount = cm * 5.62;
  sendDrive(amount, amount, speed);
}

int getUnitsCm(float cm) {
  return cm * 5.62;
}

int millisTogoUnits(int units, int speed) {
  return (60 * (float)units) / (float)speed * 1000;
}

void turnAngle(float angle, float radius, int speed) {
  bool turnLeft = false;
  if(angle < 0) {
    turnLeft = true;
    angle *= -1;
  }
  angle = angle * 1.15;
  // NOTE: CURRENTLY ACCEPTS radius IN CM

  // Turning right, right wheel moves less
  // 135 mm between wheels
  // right wheel distance from center of turn =
  //          raduis - (135 / 2)
  // left wheel distance from center of turn =
  //          radius + (135 / 2)

  // Calculate radius of each wheel turn
  float rightWheel = (float)radius - (13.5 / (float)2);
  float leftWheel = (float)radius + (13.5 / (float)2);

  // Calculate circumference of each wheel turn C=2(pi)r
  rightWheel = 2 * rightWheel * 3.1415;
  leftWheel = 2 * leftWheel * 3.1415;

  // Calculate how much each wheel should turn
  rightWheel = rightWheel * ((float)angle / 360);
  leftWheel = leftWheel * ((float)angle / 360);

  // Calculate how many units the wheel needs to move
  rightWheel = getUnitsCm(rightWheel);
  leftWheel = getUnitsCm(leftWheel);

  if(!turnLeft) {
    sendDrive(leftWheel, rightWheel, speed);

  } else {

    sendDrive(rightWheel, leftWheel, speed);
  }

}

void turnAngle(float angle, float radius) {
  turnAngle(angle, radius, STANDARD_FOWARD);
}
