/*

Ringo Robot:  RingoHardware  Rev06.01  12/2015

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

#include "RingoHardware.h"
#include "Navigation.h"

// ***************************************************
// Ringo Hardware
// ***************************************************


void HardwareBegin(void){//Ver. 1.0, Dustin Soodak
  I2CBegin();                               // startup I2C
  GyroWriteRegister(GYR_CTRL_REG3,0x10);    // turn off gyro open drain (drains current via int line from accel if not set)
  GyroWriteRegister(GYR_CTRL_REG1,0x0f);    // place gyro in power down mode (saves 6mA of current)
                                            // call NavigationBegin(); to wake up and configure Gyro and Accelerometer 
  Serial.begin(SERIAL_SPEED);//Serial.begin(9600);
  GetGyroCalibrationMultiplier();  
}



// ***************************************************
// end Ringo Hardware
// ***************************************************


// ***************************************************
// Timer
// ***************************************************

int32_t Timer_InitTime=0,Timer_StoppedTime=0;
char Timer_Running=0;
int32_t GetTime(void){//Ver. 1.0, Dustin Soodak
  if(Timer_Running){
    return millis()-(uint32_t)Timer_InitTime;
  }
  else
    return Timer_StoppedTime;
}
void RestartTimer(void){//Ver. 1.0, Dustin Soodak
  Timer_InitTime=millis();
  Timer_Running=1;
}
void StopTimer(void){//Ver. 1.0, Dustin Soodak
  if(Timer_Running){
    Timer_StoppedTime=millis()-(uint32_t)Timer_InitTime;
    Timer_Running=0;
  }
}

// ***************************************************
// end Timer
// ***************************************************


