char GyroFifoOverflow = 0;
int GyroZeroes[3]={0,0,0};
int32_t GyroPosition[3];//running total
int GyroVelocity[3];//raw values go here
int GyroAccelerationZ;//derivative of GyroVelocity[2] (z-axis)
int nav_data[3];

void GyroGetAxes(int *Axes){//Ver. 1.0, Dustin Soodak
//Axis=0,1,2
  //OUT_X_L=0x28, OUT_X_H=0x29 (y and z follow)
  I2C_read(&gyro, (0x28 | 0x80), (uint8_t*)Axes, 6);
  //GyroReadRegisters((GYR_OUT_X_L)|0x80,(uint8_t*)Axes,6);//"In order to read multiple bytes, it is necessary to assert the most significant bit of the subaddressfield."
}

void SimpleGyroNavigation(void) {
  char n;
  char i,j;
  //ConvertNavigationCoordinates(0);
  
  n=i2cRead8(0x2F)&0x1F; // GYR_FIFO_SRC_REG
  for(i=0;i<n;i++){
    GyroGetAxes(nav_data);
    for(j=1;j<3;j++){//just y & z axis      
      GyroVelocity[j]=((int32_t)nav_data[j])-GyroZeroes[j];
      GyroPosition[j]+=(GyroVelocity[j]);
    }    
  }//end for(i=0;i<n;i++)

  //get current rotational velocity for x & y axes
  for(j=0;j<2;j++){
    GyroVelocity[j]=((int32_t)nav_data[j])-GyroZeroes[j];
  }  
  if(n>=31)
    GyroFifoOverflow=1;  
      
}

int GyroRawToDegrees(int32_t Raw){//Ver. 1.0, Dustin Soodak
  int32_t deg=1;
  deg=-Raw*875/100000;
  return deg/95; // GF_95Hz
  //return deg/190; // GF_190Hz
  //return deg/380; // GF_380Hz
  //return deg/760; // GF_760Hz
  
}

int GetDegrees(void){//Ver. 1.0, Dustin Soodak
  return -GyroRawToDegrees(GyroPosition[2]);// z axis 
}

