# Security-Bot
A security bot network for [M5Stack](https://m5stack.com/) and [TEC Bot 7.0](https://idea7.cc/project/tec-bot-7-0/)

This system will allow multiple robots to patrol an area on paths set by black lines on the floor. The robots will sound an alarm if they detect any changes in their environment. Bluetooth connectivity will allow multiple robots to sound alarms simultaneously.

## Milestones:
### Overview
1. ~~Robot follows line~~
1. User programs path
2. Robot records distance to walls
3. Sound alarm if distance changes
4. Audio and visual alarm
5. Communicate between robots during patrol via bluetooth
6. Alarm is triggered if the robot is picked up

### ~~1. Robot follows line~~
1. ~~Activate line sensor ([QTR-HD-05RC](https://www.pololu.com/product/4105))~~
2. ~~Read values from sensor~~
3. ~~Follow straight line~~
4. ~~Detect right angle turns left or right~~
5. ~~Detect end of path and turn around~~
5. ~~Record approximate distance of each segment in the path~~

### 1. User programs path
1. User programs path of any length
2. Program distance and direction of turns
3. Robot follows given
4. Robot returns backward on path

### 2. Robot records distance to walls
1. Activate distance sensor ([HC-SRO4](https://www.sparkfun.com/products/13959))
2. Read values from sensor
3. Read distance at each turn on the recorded path and save values
4. Read distance continually during patrol

### 3. Sound alarm if distance changes
1. Check distance at every turn
2. Compare to saved distance
3. Continually check distance (Note: distance may be out of range at some points along the path)
4. Sound alarm if distance is out of an error range

### 4. Audio and visual alarm
1. Multi-colored leds flash
2. Alarm sound plays

### 5. Communication between robots via wifi
1. Establish a wifi connection between two robots
2. Send alarm signal if alarm is tripped
4. ~~Send "all-clear" signal at regular intervals~~
5. ~~Sound alarm if "all-clear" is not received~~
6. Send alarm both ways (server to client and client to server)

### 6. Alarm is triggered if the robot is picked up (NOT COMPLETED YET)
1. Activate IMU sensor ([MPU-9250/6500](https://www.amazon.com/HiLetgo-Gyroscope-Acceleration-Accelerator-Magnetometer/dp/B01I1J0Z7Y))
2. Read acceleration and orientation data
3. Determine acceptable ranges of acceleration and orientation for normal operation
4. Sound alarm if ranges are exceeded

