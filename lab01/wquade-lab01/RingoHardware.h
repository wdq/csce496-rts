/*

Ringo Robot:  RingoHardware  Rev01.02  12/2015

Significant portions of this code written by
Dustin Soodak for Plum Geek LLC. Some portions
contributed by Kevin King.
Portions from other open source projects where noted.
This code is licensed under:
Creative Commons Attribution-ShareAlike 2.0 Generic (CC BY-SA 2.0)
https://creativecommons.org/licenses/by-sa/2.0/

Visit http://www.plumgeek.com for Ringo information.
Visit http://www.arduino.cc to learn about the Arduino.

*/

#ifndef RINGO_HARDWARE_H
#define RINGO_HARDWARE_H

#include "Arduino.h"
#include "Navigation.h"


// ***************************************************
// Pin defines
// ***************************************************

#define Accel_Interrupt 2 //used by both Gyro and Accel chips.
#define Accel_Interrupt_Num 0 //pin2 is interrupt 0 on arduino uno board
#define Source_Select 4 


// ***************************************************
// end Pin defines
// ***************************************************

// ***************************************************
// General hardware
// ***************************************************
#define SERIAL_SPEED 57600
extern void HardwareBegin(void);
//Note: "while(!Serial.available());  delus=Serial.parseInt();" to wait for numerical input from user
// ***************************************************
// end General hardware
// ***************************************************

// ***************************************************
// Simple Timer
// ***************************************************
  //Ultra-simple stop watch functions using the built-in arduino millis() function.
  extern int32_t GetTime(void);
  extern void RestartTimer(void);
  extern void StopTimer(void);
// ***************************************************
// end Simple Timer
// ***************************************************

// ***************************************************
// end Ringo Hardware
// ***************************************************


#endif
