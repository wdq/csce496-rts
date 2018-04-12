/*

Ringo Robot:  Navigation  Rev01.02  12/2015

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

#include "Navigation.h"

//Ver. 1.0, Dustin Soodak

// ***************************************************
// Ringo I2C
// ***************************************************

#include <RingoWire.h> 
//put "RingoWireLibrary\RingoWire" into "C:\Program Files\Arduino\libraries"

//Accelerometer:
//http://cache.freescale.com/files/sensors/doc/data_sheet/MMA8451Q.pdf
//p.18
//
//Gyroscope:
//http://www.st.com/web/en/resource/technical/document/datasheet/DM00036465.pdf
//p.23-24
//
//Gyro: "Use same format for read/write single/multiple bytes"
//Accel: "The MMA8451Q automatically increments the received register address commands after a write command is received"


void I2CBegin(void){//Ver. 1.0, Dustin Soodak
  Wire.begin();
}

uint8_t I2CReadRegs(uint8_t Device, uint8_t Reg, uint8_t *RxBuffer, uint8_t Length){//Ver. 1.0, Dustin Soodak
  uint8_t i;
  Wire.beginTransmission(Device); // transmit to device (note: actually just starts filling of buffer)
  Wire.write(Reg);
  Wire.endTransmission(0);//send data without stop at end
  Wire.requestFrom(Device, Length);
  i=0;
  while(Wire.available()){
    RxBuffer[i]=Wire.read(); 
    i++;
  }
  return i;  
}

uint8_t I2CReadReg(uint8_t Device, uint8_t Reg){//Ver. 1.0, Dustin Soodak
  uint8_t dat=0;
  I2CReadRegs(Device, Reg, &dat, 1);
  return dat;
}

void I2CWriteRegs(uint8_t Device, uint8_t Reg, uint8_t *TxBuffer, uint8_t Length){//Ver. 1.0, Dustin Soodak
  char i;
  Wire.beginTransmission(Device); // transmit to device (note: actually just starts filling of buffer)
  Wire.write(Reg);              // sends one byte 
  for(i=0;i<Length;i++){
    Wire.write(TxBuffer[i]); 
  }
  Wire.endTransmission();    //send data with stop at end 
}

void I2CWriteReg(uint8_t Device, uint8_t Reg, uint8_t TxData){//Ver. 1.0, Dustin Soodak
  I2CWriteRegs( Device, Reg, &TxData, 1);
}

// ***************************************************
// end Ringo I2C
// ***************************************************




// ***************************************************
// Navigation
// ***************************************************

float GyroscopeCalibrationMultiplier=GYROSCOPE_CALIBRATION_MULTIPLIER_DEFAULT;

char IsStationary=1;//used in NavigationXY()
int NonStationaryValue=0;
char NonStationaryAxis=0;
char IsStationaryCount=0;


char XYMode=0;//added to tell GetAccelerationX(), etc. which navigation function was called: SimpleNavigation() or NavigationXY().

char GyroFifoOverflow=0;
int GyroZeroes[3]={0,0,0};
char AccelFifoOverflow=0;
int AccelZeroes[3]={0,0,0};
char NavigationOutOfRange=0;

char PauseNavigationInterrupt=1;
char NavigationOn=0;
int32_t count;

int nav_data[3];//made global so compiler doesn't optimize it out of code
int nav_accel[3];
int nav_data3[3];

signed char GyroEdgeDetection=0;
int GyroEdgeDetectionLevel=100;
char GyroEdgeDetectionRepeats=10;
char GyroEdgeDetected;


void ZeroGyroEdgeDetection(void){//Ver. 1.0, Dustin Soodak
  //works but already too late by the time edge is detected
  GyroEdgeDetection=0;
}

void UpdateGyroEdgeDetection(int GyroYAxisRawZeroed){//Ver. 1.0, Dustin Soodak
  //works but already too late by the time edge is detected
  if(!GyroEdgeDetected){
    if(GyroYAxisRawZeroed>10){
      if(GyroEdgeDetection>=0) 
        GyroEdgeDetection=-1;
      else
        GyroEdgeDetection--;
      if(GyroEdgeDetection<-GyroEdgeDetectionLevel){
        GyroEdgeDetection=0;//so ready for next time when GyroEgeDetected set to 0
        GyroEdgeDetected=2;//bit 1: left
      }
    }
    else if(GyroYAxisRawZeroed<-10){
      if(GyroEdgeDetection<=0) 
        GyroEdgeDetection=1;
      else
        GyroEdgeDetection++;
      if(GyroEdgeDetection>GyroEdgeDetectionLevel){
        GyroEdgeDetection=0;//so ready for next time when GyroEgeDetected set to 0
        GyroEdgeDetected=1;//bit 0: right
      }
    }
  }//end if(!GyroEdgeDetected)
}//end UpdateGyroEdgeDetection

void SimpleGyroNavigation(void){//Ver. 1.0, Dustin Soodak
  char n;
  //int nav_data[3];//Compiler error:  when nav_data[] local, it was not actually reserving space for this since it isn't explicitly set (is set by pointers)
  char i,j;
  ConvertNavigationCoordinates(0);

  if(!PauseNavigationInterrupt){
    
    //Get Gyroscope data
    n=GyroBufferSize();
    //if(n>0)
    //digitalWrite(Light_Bus_BTN1,0);
    for(i=0;i<n;i++){
      GyroGetAxes(nav_data);
      //digitalWrite(Light_Bus_BTN1,0);
      for(j=1;j<3;j++){//just y & z axis      
        GyroVelocity[j]=((int32_t)nav_data[j])-GyroZeroes[j];
        GyroPosition[j]+=(GyroVelocity[j]);
      }    
      //UpdateGyroEdgeDetection(GyroVelocity[1]);//works but already too late by the time edge is detected

    }//end for(i=0;i<n;i++)
    //get current rotational velocity for x & y axes
    for(j=0;j<2;j++){
      GyroVelocity[j]=((int32_t)nav_data[j])-GyroZeroes[j];
    }  
    if(n>=31)
      GyroFifoOverflow=1;    
  } 
}
int32_t AverGyroVelocity;

int AccelPositionXOffset=0,AccelPositionYOffset;
//Can remove this feature by commenting out body of
//this function(replace with "XYMode=NewXYMode;")
//and removing AccelPositionYOffset from GetPositionY().
void ConvertNavigationCoordinates(char NewXYMode){//Ver. 1.0, Dustin Soodak
  float Theta,conversion;
  if(XYMode==(!NewXYMode)){
    Theta=((float)90-GetDegrees())*3.14159/180;//(((float)GyroPosition[2])*((0.0000355*2000))*3.14159/180);
    conversion=(-((float)32768)*380*380/(2*9800));//since GetPositionX() in 
    //XYMode is ((float)AccelPosition[0])*(-(float)(2*9800)/32768/380/380)+AccelPositionXOffset;
    //GetPositionY() is same in XYMode but is ((float)AccelPosition[1])*(-(float)(2*9800)/32768/400) 
    //when using SimpleNagivation()
    AccelPositionXOffset=cos(Theta)*ACCEL_DIST;
    AccelPositionYOffset=sin(Theta)*ACCEL_DIST;
    if(NewXYMode){
      AccelPosition[1]=((float)AccelPosition[1])*(((float)380)*380/400);//scale to XYMode
      AccelPosition[1]+=((float)AccelPositionYOffset)*conversion;//add offset    
    }
    else{
      AccelPosition[1]-=((float)AccelPositionYOffset)*conversion;//get rid of offset
      AccelPosition[1]=((float)AccelPosition[1])*(((float)400)/380/380);//scale to Simple mode
    }
    XYMode=NewXYMode;
  }//if XY mode changed
  
}


#ifdef INT_VER
const int SinFunctionTable[]={0,286,572,857,1143,1428,1713,1997,2280,2563,2845,3126,3406,3686,3964,4240,4516,4790,5063,5334,5604,5872,6138,6402,6664,6924,7182,7438,7692,7943,8192,8438,8682,8923,9162,9397,9630,9860,10087,10311,10531,10749,10963,11174,11381,11585,11786,11982,12176,12365,12551,12733,12911,13085,13255,13421,13583,13741,13894,14044,14189,14330,14466,14598,14726,14849,14968,15082,15191,15296,15396,15491,15582,15668,15749,15826,15897,15964,16026,16083,16135,16182,16225,16262,16294,16322,16344,16362,16374,16382,16384};
int SineFunction(int degr){//Ver. 1.0, Dustin Soodak
  int sign=1;
  if(degr<0){
    sign=-1;
    degr=-degr;
  }
  if(degr>360)
    degr%=360;
  if(degr<=90)
    return sign*SinFunctionTable[degr];
  else if(degr<=180)
    return sign*SinFunctionTable[180-degr];
  else if(degr<=270)
    return -sign*SinFunctionTable[degr-180];
  else
    return -sign*SinFunctionTable[270-degr];
}
int CosineFunction(int degr){//Ver. 1.0, Dustin Soodak
   if(degr<0)
     degr=-degr;
   if(degr>360)
     degr%=360;
   if(degr<=90)
     return SinFunctionTable[90-degr];
   else if(degr<=180)
     return -SinFunctionTable[degr-90];
   else if(degr<=270)
     return -SinFunctionTable[270-degr];
   else
     return SinFunctionTable[360-degr];
}
#endif //end #ifdef INT_VER

extern int GyroVelocityZPrev;//(defined below)
//int32_t accelxraw,accelyraw;
//int CosDegr,SinDegr;
int32_t N_accelx,N_accely;//made global so compiler doesn't "optimize" them out.



// calibrates out stationary drift in sensors. MUST BE RUN WHEN RINGO IS PERFECTLY STILL!!
void CalibrateNavigationSensors(void){//Ver. 1.0, Dustin Soodak
  char i,j,n,prev;
  int32_t totals[3];
  //int nav_data[3];//Compiler error:  when nav_data[] local, it was not actually reserving space for this since it isn't explicitly set (is set by pointers)
  prev=PauseNavigationInterrupt;
  PauseNavigationInterrupt=1;//PauseNavigationInterrupt=1;//noInterrupts();
  //Gyro average to find zero value (assuming not currently moving)
  for(i=0;i<3;i++)
    totals[i]=0;
  i=1000;
  while(GyroBufferSize()<20 && i)i--; 
  for(i=0;i<20;i++){
    GyroGetAxes(nav_data);
    for(j=0;j<3;j++){
      totals[j]+=nav_data[j];
    }    
  }
  for(i=0;i<3;i++){
    GyroZeroes[i]=totals[i]/20;
  }
  //Accel average to find zero value (assuming not currently moving)
  for(i=0;i<3;i++)
    totals[i]=0;
  i=1000;
  while(AccelBufferSize()<20 && i)i--; 
  for(i=0;i<20;i++){
    AccelGetAxes(nav_accel);
    for(j=0;j<3;j++){
      totals[j]+=nav_accel[j];
    }    
  }
  for(i=0;i<3;i++){
    AccelZeroes[i]=totals[i]/20;
    AccelVelocity[i]=0;//assumes it isn't moving when this function called
  }  
  //Clear buffers
  n=AccelBufferSize();
  for(i=0;i<n;i++){
    AccelGetAxes(nav_data);
  }
  n=GyroBufferSize();
  for(i=0;i<n;i++){
    GyroGetAxes(nav_data);
  }
  AccelFifoOverflow=0; 
  PauseNavigationInterrupt=prev;//interrupts();//PauseNavigationInterrupt=0;
  for(i=0;i<3;i++){
    AccelVelocity[i]=0;
  }   
  
  //Zero sensors with arcsin:
  //9800*sin(GyroXRaw*((0.0000355*2000/380)*3.14159/180))=AccelYRaw*(-(float)(2*9800)/32768
  //9800*sin(GyroPosition[0]*((0.0000355*2000/380)*3.14159/180))=AccelZeroes[1]*(-(float)(2*9800)/32768
  //GyroPosition[0]=-arcsin(AccelZeroes[1]*((float)(2)/32768))/((0.0000355*2000/380)*3.14159/180)
  //Zero sensors with arctan:
  //atan2(x,y) (instead of y,x) so theta of 90 degrees -> 0 degrees.
  //GyroPosition[0]=-atan2(AccelZeroes[1],AccelZeroes[2])/((0.0000355*2000/380)*3.14159/180);
  //Calculate new:
  //Theta=GyroPosition[0]*((0.0000355*2000/380)*3.14159/180);
  //AccelAcceleration[1]-=sin(Theta)*16384;//raw to mm/s^2 is ((float)(2*9800)/32768);
  //Since sin(theta)=theta for small theta, just need one float and no trig:
  //AccelAcceleration[1]-=GyroPosition[0]*(((0.0000355*2000/380)*3.14159/180)*16384)

  //For gyroX correction to accelY:
  GyroPosition[0]=-asin(AccelZeroes[1]*((float)(2)/32768))/((0.0000355*2000/380)*3.14159/180);
  //Serial.print(AccelZeroes[1],DEC);Serial.print(" ");Serial.println(AccelZeroes[2]);
  //Serial.println(atan2(AccelZeroes[1],AccelZeroes[2]));
  //Serial.println(atan2(AccelZeroes[1],AccelZeroes[2])*180/3.14159);
  
  //9800*sin(GyroPosition[0]*((0.0000355*2000/380)*3.14159/180))
  
  //Serial.print("init gyro raw after zeroed ");
  //Serial.println(GyroPosition[0],DEC);
  //Serial.print(" to degr ");
  //Serial.println(GyroRawToDegrees(GyroPosition[0]),DEC);
  
  IsStationary=1;IsStationaryCount=0;//IsMovingCount=0;
  GyroVelocityZPrev=0;
  NavigationOutOfRange=0;
  //ZeroGyroEdgeDetection();//works, but already too late by the time an edge is detected 
  //The following initializes accel in case gyro
  //reading but not accel reading so accel has an initial
  //default value for NavigationXY()
  nav_accel[0]=AccelZeroes[0];
  nav_accel[1]=AccelZeroes[1];
  //Though already set above, put here explicitly in case
  //other code changed for an unrelated reason.
  
}//end CalibrateNavigationSensors()

//re-sets Ringo's X, Y, and Heading coordinates to zeros
void ZeroNavigation(void){//Ver. 1.0, Dustin Soodak
  char i;
   //Zero total changes in position, velocity, and orientation
  for(i=0;i<3;i++){
    AccelPosition[i]=0;
    AccelVelocity[i]=0;
    if(i>0)//for part in CalibrateNavigationSensors() which 
      GyroPosition[i]=0;
  }
  XYMode=0;
  AccelDistance=0;  
  IsStationary=1;IsStationaryCount=0;//IsMovingCount=0;
  AccelFifoOverflow=0; 
  GyroFifoOverflow=0; 
  NavigationOutOfRange=0;
  AccelPositionYOffset=30;
  AccelPositionXOffset=0;
}


void PauseNavigation(void){//Ver. 1.0, Dustin Soodak
  char i;
  PauseNavigationInterrupt=1; 
  for(i=0;i<3;i++){
    AccelVelocity[i]=0;
  }
}

void ResumeNavigation(void){//Ver. 1.0, Dustin Soodak
  char i,n;
  //clear gyro buffer and save last values
  n=GyroBufferSize();
  for(i=0;i<n;i++){
    GyroGetAxes(GyroVelocity);
  }
  //subtract gyro zeroes
  for(i=0;i<3;i++){
   GyroVelocity[i]-=GyroZeroes[i];
  }
  //clear accel buffer and save last values
  n=AccelBufferSize();
  for(i=0;i<n;i++){
   AccelGetAxes(AccelAcceleration);
  }
  //subtract accel zeroes
  for(i=0;i<3;i++){
   AccelAcceleration[i]-=AccelZeroes[i];
  }
  for(i=0;i<2;i++){
   AccelVelocity[i]=0;
  }
  PauseNavigationInterrupt=0;
  IsStationary=1;IsStationaryCount=0;//IsMovingCount=0;
  GyroVelocityZPrev=0;
}

char NavigationPaused(void){//Ver. 1.0, Dustin Soodak
  return PauseNavigationInterrupt;
}

void NavigationBegin(void){ //Ver. 1.0, Dustin Soodak   
  char i;
  I2CBegin();//needs to be called before writing Gryro or Accel registers
  PauseNavigationInterrupt=1;
  //
  //GyroWriteRegister(GYR_CTRL_REG3,0x34);//open drain (bit4), watermark interrupt on DRDY/INT2 (bit2). bit5 is INT1 high, but seems to work for INT2 as well
  GyroWriteRegister(GYR_CTRL_REG3,0x10);  //0x10 eliminates excess current drain from Accel via the shared interrupt line
  GyroWriteRegister(GYR_FIFO_CTRL_REG,0x54);//watermark FIFO threshold of 20 and fifo stream mode (bits7:6)
  GyroWriteRegister(GYR_CTRL_REG1,0x0f);//all axes on(0:2) and auto-power off disabled
  GyroWriteRegister(GYR_CTRL_REG5,0x40);//fifo enable
  //
  AccelWriteRegister(ACC_CTRL_REG1,0x0);// 0x2a inactive but can set registers (bit1)
  AccelWriteRegister(ACC_CTRL_REG3,0x3);// 0x2c interrupt is OD(bit0), active high(bit1)
  AccelWriteRegister(ACC_CTRL_REG4,0x40);// 0x2d fifo interrupt enable
  //AccelWriteRegister(ACC_CTRL_REG4,0x00);
  AccelWriteRegister(ACC_CTRL_REG5,0x40);// 0x2e fifo interrupt on INT1 pin
  AccelWriteRegister(ACC_F_SETUP,0x14);// 0x9 watermark at 20 measurements
  AccelWriteRegister(ACC_CTRL_REG1,0x9);// 0x2a 400 Hz and active (bit1)
  AccelWriteRegister(ACC_F_SETUP,0x54);// 0x09 circular buffer mode (now at 0x54=01010100)
  //
  
  GyroSetFrequency(380);
  GyroSetRange(2000);

  delay(200);                         // let nav sensors buffer fill some before calibrating
  CalibrateNavigationSensors();
  ZeroNavigation();
  
  pinMode(Accel_Interrupt, INPUT_PULLUP);
  
  NavigationOn=1;
  PauseNavigationInterrupt=0;  
  
}//end void NavigationBegin(void)


int PresentHeading(void){//Ver. 1.0, Kevin King
  return -GyroRawToDegrees(GyroPosition[2]);// z axis
}
int GetDegrees(void){//Ver. 1.0, Dustin Soodak
  return -GyroRawToDegrees(GyroPosition[2]);// z axis 
}
int GetDegreesX(void){//Ver. 1.0, Dustin Soodak
  return GyroRawToDegrees(GyroPosition[0]);// x axis
}
int GetDegreesPerSecond(void){//Ver. 1.0, Dustin Soodak
  return -GyroRawToDegreesPerSec(GyroVelocity[2]);// z axis
}
int GetDegreesPerSecondX(void){//Ver. 1.0, Dustin Soodak
  return GyroRawToDegreesPerSec(GyroVelocity[0]);
}
int GetDegreesPerSecondY(void){//Ver. 1.0, Dustin Soodak
  return GyroRawToDegreesPerSec(GyroVelocity[1]);
}
int GetDegreesToStop(void){//Ver. 1.0, Dustin Soodak
  return -GyroDegreesToStopFromRaw(GyroVelocity[2]);
}

int GetAccelerationX(void){//Ver. 1.0, Dustin Soodak
  return ((float)AccelAcceleration[0])*(-(float)(2*9800)/32768);
}
int GetAccelerationY(void){//Ver. 1.0, Dustin Soodak
  return ((float)AccelAcceleration[1])*(-(float)(2*9800)/32768);
}
int GetAccelerationYUnZeroed(void){//Ver. 1.0, Dustin Soodak
  //for front/back tilt analysis
  return ((float)AccelAcceleration[1]+AccelZeroes[1])*(-(float)(2*9800)/32768);
}
int GetAccelerationZ(void){//Ver. 1.0, Dustin Soodak
  return ((float)AccelAcceleration[2])*(-(float)(2*9800)/32768);
}
//The following 4 functions have different cases depending on if 
//NavigationXY() or one of the simpler functions are used.
int GetVelocityX(void){//Ver. 1.0, Dustin Soodak
  if(XYMode)
    return ((float)AccelVelocity[0])*(-(float)(2*9800)/32768/380);
  else
    return 0;
}
int GetVelocityY(void){//Ver. 1.0, Dustin Soodak
  if(XYMode)
    return ((float)AccelVelocity[1])*(-(float)(2*9800)/32768/380);  
  else
    return ((float)AccelVelocity[1])*(-(float)(2*9800)/32768/400);  
}
int GetPositionX(void){//Ver. 1.0, Dustin Soodak
  //SimpleNavigation() and SimpleGyroNavigation() leave X unchanged, so decided
  //not to have an XYMode==0 option.
  //if(XYMode)
    return ((float)AccelPosition[0])*(-(float)(2*9800)/32768/380/380)-AccelPositionXOffset;
  //else
    //return ((float)AccelPosition[0])*(-(float)(2*9800)/32768/380/380);
}
int GetPositionY(void){//Ver. 1.0, Dustin Soodak
  if(XYMode)  //put these sections back to use auto offset
    return ((float)AccelPosition[1])*(-(float)(2*9800)/32768/380/380)-AccelPositionYOffset;
  else
    return ((float)AccelPosition[1])*(-(float)(2*9800)/32768/400);//note: already an extra "/400" in handler to keep in range (400 for 400hz)

}

// ***************************************************
// end Navigation
// ***************************************************

// ***************************************************
// Accelerometer
// ***************************************************

//Accelerometer code based on code by Jim Lindblom
//https://github.com/sparkfun/MMA8452_Accelerometer/tree/master/Firmware

#define AccelAddr 0x1C

int32_t AccelPosition[3]={0,0,0};//running total
int32_t AccelDistance=0;
int32_t AccelVelocity[3]={0,0,0};//running total
int AccelAcceleration[3]={0,0,0};

uint8_t AccelReadRegisters(uint8_t Reg, uint8_t *RxBuffer, uint8_t Length){//Ver. 1.0, Dustin Soodak
  return I2CReadRegs(AccelAddr,0x80|Reg,RxBuffer,Length);//"In order to read multiple bytes, it is necessary to assert the most significant bit of the subaddress field"
}
uint8_t AccelReadRegister(uint8_t Reg){//Ver. 1.0, Dustin Soodak
  return I2CReadReg(AccelAddr,Reg);
}
void AccelWriteRegisters(uint8_t Reg, uint8_t *TxBuffer, uint8_t Length){//Ver. 1.0, Dustin Soodak
   I2CWriteRegs(AccelAddr, Reg, TxBuffer, Length);
}
void AccelWriteRegister(uint8_t Reg, uint8_t TxData){//Ver. 1.0, Dustin Soodak
  I2CWriteReg(AccelAddr, Reg, TxData);
}
//Put in standby to change register settings
void AccelStandby(void){//Ver. 1.0, Dustin Soodak
  byte c = AccelReadRegister(ACC_CTRL_REG1);
  AccelWriteRegister(ACC_CTRL_REG1, c & ~(0x01));
}
//Needs to be in this mode to output data
void AccelActive(void){//Ver. 1.0, Dustin Soodak
  byte c = AccelReadRegister(ACC_CTRL_REG1);
  AccelWriteRegister(ACC_CTRL_REG1, c | 0x01);
}

uint8_t AccelBufferSize(void){//Ver. 1.0, Dustin Soodak
  return AccelReadRegister(ACC_F_STATUS)&0x3F;
}
int AccelGetAxis(char Axis){//Ver. 1.0, Dustin Soodak
  //Axis=0,1,2 (see p.22)
  uint8_t ar[2];
  int16_t val;//OUT_X_MSB, etc.
  if(Axis>2)
    Axis=2;
  AccelReadRegisters((ACC_OUT_X_MSB+2*Axis),ar,2);
  val=(ar[0]>>2)+(((uint8_t)(ar[1]&0x7F))<<6)+(((uint8_t)(ar[1]&0x80))<<8);
  return val;
}
void AccelGetAxes(int *Axes){//Ver. 1.0, Dustin Soodak
  //Axis=0,1,2 (see p.22) [int is 16 bit in adruino]
  uint8_t ar[6];
  int i;
  AccelReadRegisters((ACC_OUT_X_MSB),ar,6);
  for(i=0;i<6;i+=2){
     //Axes[i>>1]=(int)(ar[i]>>2)+(((uint8_t)(ar[i+1]&0x7F))<<6)+(((uint8_t)(ar[i+1]&0x80))<<8);
     Axes[i>>1]=(((signed short)ar[i])<<8)+ar[i+1];//((((unsigned int)ar[i])&0x10)<<8)|((((unsigned int)ar[i])&0x7F)<<6);
  }
}

//Serial.print(" v= ");Serial.print(-(GetVelocityRaw(1))*(2*9800)/(32768)/400,DEC);  // debugging code
//Serial.print(" p= ");Serial.println(-(GetPositionRaw(1))*(2*9800)/(32768)/400/* /400 in handler instead*/,DEC);  // debugging code

// ***************************************************
// end Accelerometer
// ***************************************************

// ***************************************************
// Gyro
// ***************************************************

//Note: referenced code from "L3G4200D 3-axis gyro example code" by Jim Lindblom at
//https://www.sparkfun.com/products/10612

int32_t GyroPosition[3]={0,0,0};//running total
int GyroVelocity[3]={0,0,0};
int GyroAccelerationZ=0;//rate of change in angular speed (degrees pre second)
int GyroVelocityZPrev=0;//used in NavigationXY() to calculate GyroAccelerationZ

#define GyroAddr 0x6B
#define GR_250dps 0x00
#define GR_500dps 0x10
#define GR_2000dps 0x20
int GyroRange=250;//default value for chip
#define GF_95Hz  0b00000000  //wg 20 0f
#define GF_190Hz 0b01000000  //wg 20 4f
#define GF_380Hz 0b10000000  //wg 20 8f
#define GF_760Hz 0b11000000  //wg 20 cf
int GyroFrequency=95;
float GyroRawToDegreesMult=1;
float GyroDegreesToRawMult=1;
float GyroRawToDegreesPerSecMult=1;
float GyroDegreesPerSecToRawMult=1;
float GyroRawToSkidMult=0;

uint8_t GyroReadRegisters(uint8_t Reg, uint8_t *RxBuffer, uint8_t Length){//Ver. 1.0, Dustin Soodak
  return I2CReadRegs(GyroAddr,Reg,RxBuffer,Length);
}
uint8_t GyroReadRegister(uint8_t Reg){//Ver. 1.0, Dustin Soodak
  return I2CReadReg(GyroAddr,Reg);
}
void GyroWriteRegisters(uint8_t Reg, uint8_t *TxBuffer, uint8_t Length){//Ver. 1.0, Dustin Soodak
   I2CWriteRegs(GyroAddr, Reg, TxBuffer, Length);
}
void GyroWriteRegister(uint8_t Reg, uint8_t TxData){//Ver. 1.0, Dustin Soodak
  I2CWriteReg(GyroAddr, Reg, TxData);
}

uint8_t GyroBufferSize(void){//Ver. 1.0, Dustin Soodak
  return GyroReadRegister(GYR_FIFO_SRC_REG)&0x1F;
}

int16_t GyroGetAxis(char Axis){//Ver. 1.0, Dustin Soodak
  //Axis=0,1,2 (p.36)
  int16_t val;//OUT_X_L=0x28, OUT_X_H=0x29 (y and z follow)
  if(Axis>2)
    Axis=2;
  GyroReadRegisters((GYR_OUT_X_L+2*Axis)|0x80,(uint8_t*)val,2);//"In order to read multiple bytes, it is necessary to assert the most significant bit of the subaddressfield."
  return val;
}
void GyroGetAxes(int *Axes){//Ver. 1.0, Dustin Soodak
//Axis=0,1,2
  //OUT_X_L=0x28, OUT_X_H=0x29 (y and z follow)
  GyroReadRegisters((GYR_OUT_X_L)|0x80,(uint8_t*)Axes,6);//"In order to read multiple bytes, it is necessary to assert the most significant bit of the subaddressfield."
}

//gyro:
//range	calc dps/dig	"typ" in datasheet	to get to "typ"		
//250	0.007629395	0.00875           	1.14687993         
//500	0.015258789	0.0175           	1.146880005
//2000	0.061035156	0.070            	1.146880005

void UpdateGyroConversionVars(void){//Ver. 1.0, Dustin Soodak
  GyroRawToDegreesPerSecMult=((float)GyroRange)*0.0000355/GyroscopeCalibrationMultiplier;// 1/2^15=1/32768=0.000030517578125      // 1/2^15*1.14688=exactly .000035
  GyroRawToDegreesMult=GyroRawToDegreesPerSecMult/GyroFrequency;
  GyroDegreesPerSecToRawMult=((float)28169)/GyroRange*GyroscopeCalibrationMultiplier; //2^15=32768                             //1/0.000035=28571.42857
  GyroDegreesToRawMult=GyroDegreesPerSecToRawMult*GyroFrequency;    
  GyroRawToSkidMult=GyroRawToDegreesPerSecMult*0.1029;//used in GyroDegreesToStopFromRaw()
}

void GetGyroCalibrationMultiplier(void){//Ver. 1.0, Kevin King   //Gets gyro cal mult from EEPROM if present
}

void GyroSetRange(int Range){//Ver. 1.0, Dustin Soodak
  char RangeByte;
  if(Range==2000)
    RangeByte=GR_2000dps;
  else if(Range==500)
    RangeByte=GR_500dps;
  else
    RangeByte=GR_250dps;
  GyroWriteRegister(GYR_CTRL_REG4,RangeByte);
  //FS=250dps: 8.75 mdps/digit
  //   500:    17.5
  //   2000:   70 
  GyroRange=Range;
  UpdateGyroConversionVars();    
}

int GyroGetRangeFromChip(void){//Ver. 1.0, Dustin Soodak
  char d=GyroReadRegister(GYR_CTRL_REG4)&0x30;
  if(d==GR_250dps)
    return 250;
  else if(d==GR_500dps)
    return 500;
  else if(d==GR_2000dps)
    return 2000;
  else 
    return 250;
}

int GyroGetFrequencyFromChip(void){//Ver. 1.0, Dustin Soodak
  char d=GyroReadRegister(GYR_CTRL_REG1)&0xC0;
  if(d==GF_95Hz)
    return 95;
  else if(d==GF_190Hz)
    return 190;
  else if(d==GF_380Hz)
    return 380;
  else if(d==GF_760Hz)
    return 760;  
  else
    return 95;
}

//GyroSetFrequencyByte(GyroReadRegister(GYR_CTRL_REG1)&0xC0);
//GyroSetRangeByte(GyroReadRegister(GYR_CTRL_REG4)&0x30); 

void GyroSetFrequency(int Frequency){//Ver. 1.0, Dustin Soodak
  char FrequencyByte;
  char r=GyroReadRegister(GYR_CTRL_REG1);  
  if(Frequency==190)
    FrequencyByte=GF_190Hz;
  else if(Frequency==380)
    FrequencyByte=GF_380Hz;
  else if(Frequency==760)
    FrequencyByte=GF_760Hz;
  else if(Frequency==95)
    FrequencyByte=GF_95Hz;
  GyroWriteRegister(GYR_CTRL_REG1,FrequencyByte|(r&~0b11000000));
  GyroFrequency=Frequency;
  UpdateGyroConversionVars();
}


int32_t GyroDegreesToRaw(int Degrees){//Ver. 1.0, Dustin Soodak
  int32_t raw=1;
  return GyroDegreesToRawMult*Degrees;
  /*if(GyroRangeByte==GR_250dps){
    raw=-Degrees*100000/875;
    //Serial.print("_250_");
  }
  else if(GyroRangeByte==GR_500dps){
    //Serial.print("_500_");
    raw=-Degrees*10000/175;
  }
  else if(GyroRangeByte==GR_2000dps){
    raw=-Degrees*1000/70;
    //Serial.print("_2000_");
  }
  if(GyroFrequencyByte==GF_95Hz)
    return raw*95;
  else if(GyroFrequencyByte==GF_190Hz)
    return raw*190;
  else if(GyroFrequencyByte==GF_380Hz)
    return raw*380;
  else if(GyroFrequencyByte==GF_760Hz){
    //Serial.print("_760_");
    return raw*760;
  }
  else
    return 0;
  */
}
int GyroDegreesPerSecToRaw(int Degrees){//Ver. 1.0, Dustin Soodak
  return Degrees*GyroDegreesPerSecToRawMult;  
}
int GyroRawToDegrees(int32_t Raw){//Ver. 1.0, Dustin Soodak
  int32_t deg=1;
  return Raw*GyroRawToDegreesMult;
  /*
  if(GyroRangeByte==GR_250dps)
    deg=-Raw*875/100000;
  else if(GyroRangeByte==GR_500dps)
    deg=-Raw*175/10000;
  else if(GyroRangeByte==GR_2000dps){
    deg=-Raw*70/1000;
  }
  if(GyroFrequencyByte==GF_95Hz)
    return deg/95;
  else if(GyroFrequencyByte==GF_190Hz)
    return deg/190;
  else if(GyroFrequencyByte==GF_380Hz)
    return deg/380;
  else if(GyroFrequencyByte==GF_760Hz)
    return deg/760;
  else
    return 0;
  */
}

int GyroRawToDegreesPerSec(int Raw){//Ver. 1.0, Dustin Soodak
  return Raw*GyroRawToDegreesPerSecMult;  
}

int GyroDegreesToStopFromRaw(int DegreesPerSecondRaw){//Ver. 1.0, Dustin Soodak
 int mx=((float)DegreesPerSecondRaw)*GyroRawToSkidMult;
 if(-24<=mx && mx<=24)
   return 0;
 else if(mx>24)
   return mx-24;
 else //mx<24
   return mx+24;
}

int GyroDegreesToStop(int DegreesPerSecond){//Ver. 1.0, Dustin Soodak
  int mx=((float)DegreesPerSecond)*0.1029;
  if(-24<=mx && mx<=24)
   return 0;
 else if(mx>24)
   return mx-24;
 else //mx<24
   return mx+24; 
}

// ***************************************************
// end Gyro
// ***************************************************

// ***************************************************
// Computation
// ***************************************************


int VectorToDegrees(int x,int y){//Ver. 1.0, Dustin Soodak
  return 90-(int)(atan2(y,x)*180/3.14159265359);
}

// ***************************************************
// end Computation
// ***************************************************










