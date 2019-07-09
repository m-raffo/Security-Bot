#include <M5Stack.h>

void sendDrive(int left, int right) {
  // Serial2.printf("$J = G21 G91 X%d Y%d F%d", left, right, speed);
  // Serial2.print("\r\n\r\n");

  Serial2.printf("SS %6d %6d\r", left, right);

}

void sendMove(int left, int right, int time) {
  // Serial2.printf("$J = G21 G91 X%d Y%d F%d", left, right, speed);
  // Serial2.print("\r\n\r\n");

  Serial2.printf("SS %6d %6d\r", left, right);
  delay(time);
  Serial2.printf("SS %6d %6d\r", 0, 0);

}
