// Will eventually get the ringo stuff out of their library

/*
float MyGyroRawToDegreesMult=1;
int MyGyroRange=250;

void MyNavigationBegin(void){
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
  
}

void MyUpdateGyroConversionVars(void){
  MyGyroRawToDegreesPerSecMult=((float)MyGyroRange)*0.0000355/GyroscopeCalibrationMultiplier;// 1/2^15=1/32768=0.000030517578125      // 1/2^15*1.14688=exactly .000035
  MyGyroRawToDegreesMult=GyroRawToDegreesPerSecMult/GyroFrequency;
  MyGyroDegreesPerSecToRawMult=((float)28169)/MyGyroRange*GyroscopeCalibrationMultiplier; //2^15=32768                             //1/0.000035=28571.42857
  MyGyroDegreesToRawMult=GyroDegreesPerSecToRawMult*GyroFrequency;    
  MyGyroRawToSkidMult=GyroRawToDegreesPerSecMult*0.1029;//used in GyroDegreesToStopFromRaw()
}

int MyGyroRawToDegrees(int32_t Raw){
  int32_t deg=1;
  return Raw*MyGyroRawToDegreesMult;
}

int MyGetDegrees() {
  return -MyGyroRawToDegrees(GyroPosition[2]);// z axis 
}

*/
