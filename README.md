
Based off of [isouriadaki's](https://github.com/isouriadakis/Arduino_ICM20948_DMP_Full-Function) fork of [ZaneL's ](https://github.com/ZaneL/Teensy-ICM-20948) library

Encapsulates the c library into a cpp class with static references removed.

 - Allows more than one imu on the same MCU
   
  
 - Added accessor methods for dealing with bias values.
   
   
 - Added a (not ICM / DMP) tap detector (custom tap count).
   
  - Examples added for tap detection and bias saving/loading from an
   ESP32 MCU.
