#include "ci2c.h"

// ***************************************************
// Begin Gyro registers
// ***************************************************

#define    L3GD20_REGISTER_WHO_AM_I             0x0F   // 11010100   r
#define   L3GD20_REGISTER_CTRL_REG1            0x20   // 00000111   rw
#define   L3GD20_REGISTER_CTRL_REG2            0x21   // 00000000   rw
#define   L3GD20_REGISTER_CTRL_REG3            0x22   // 00000000   rw
#define   L3GD20_REGISTER_CTRL_REG4            0x23   // 00000000   rw
#define   L3GD20_REGISTER_CTRL_REG5            0x24   // 00000000   rw
#define   L3GD20_REGISTER_REFERENCE            0x25   // 00000000   rw
#define   L3GD20_REGISTER_OUT_TEMP             0x26   //            r
#define   L3GD20_REGISTER_STATUS_REG           0x27   //            r
#define   L3GD20_REGISTER_OUT_X_L              0x28   //            r
#define   L3GD20_REGISTER_OUT_X_H              0x29   //            r
#define   L3GD20_REGISTER_OUT_Y_L              0x2A   //            r
#define   L3GD20_REGISTER_OUT_Y_H              0x2B   //            r
#define   L3GD20_REGISTER_OUT_Z_L              0x2C   //            r
#define   L3GD20_REGISTER_OUT_Z_H              0x2D   //            r
#define   L3GD20_REGISTER_FIFO_CTRL_REG        0x2E   // 00000000   rw
#define   L3GD20_REGISTER_FIFO_SRC_REG         0x2F   //            r
#define   L3GD20_REGISTER_INT1_CFG             0x30   // 00000000   rw
#define   L3GD20_REGISTER_INT1_SRC             0x31   //            r
#define   L3GD20_REGISTER_TSH_XH               0x32   // 00000000   rw
#define   L3GD20_REGISTER_TSH_XL               0x33   // 00000000   rw
#define   L3GD20_REGISTER_TSH_YH               0x34   // 00000000   rw
#define   L3GD20_REGISTER_TSH_YL               0x35   // 00000000   rw
#define   L3GD20_REGISTER_TSH_ZH               0x36   // 00000000   rw
#define   L3GD20_REGISTER_TSH_ZL               0x37   // 00000000   rw
#define   L3GD20_REGISTER_INT1_DURATION        0x38   // 00000000   rw

#define L3GD20_SENSITIVITY_250DPS  (0.00875F)      // Roughly 22/256 for fixed point match

#define   GYRO_ADDRESS  0x6B

// ***************************************************
// end Gyro registers
// ***************************************************

// Some global variables for gyroscope data
float gyroAngle = 0;
int16_t gyroDegrees = 0;
uint8_t gyroLoopCount = 0;

I2C_SLAVE gyro; // Gyroscope I2C object

// Read a byte over I2C, helper function
uint8_t i2cRead8(uint8_t reg) {
  uint8_t out[1];
  I2C_read(&gyro, reg, &out[0], 1); // read 1 byte at reg into out
  return out[0]; 
}

// Write a byte over I2C, helper function
void i2cWrite8(uint8_t reg, uint8_t value) {
  uint8_t in[1];
  in[0] = value;
  I2C_write(&gyro, reg, &in[0], 1); // write 1 byte from in into reg
}

void setup() {
  pinMode(13, OUTPUT);
  I2C_init(I2C_FM); // init I2C bus

  // Initialize the gyro
  //while(xSemaphoreTake(gyroSemaphore, (TickType_t) 10) == pdFALSE) {} // Wait on semaphore
  I2C_slave_init(&gyro, GYRO_ADDRESS, I2C_8B_REG);
  uint8_t id = i2cRead8(L3GD20_REGISTER_WHO_AM_I); // get ID, for testing
  i2cWrite8(L3GD20_REGISTER_CTRL_REG1, 0x0F); // Normal mode, enable all three channels
  i2cWrite8(L3GD20_REGISTER_CTRL_REG4, 0x00); // Make sure it's 250 dps resolution 
  //xSemaphoreGive(gyroSemaphore); // Release the semaphore   
}

void loop() {

  digitalWrite(13, 1);
  
    uint8_t xhi, xlo, ylo, yhi, zlo, zhi;
    uint8_t out[6];
    float gyroValue = 0;
    //while(xSemaphoreTake(gyroSemaphore, (TickType_t) 10) == pdFALSE) {} // Wait on semaphore
    I2C_read(&gyro, (L3GD20_REGISTER_OUT_X_L | 0x80), &out[0], 6); // read six bytes for position data
    zlo = out[4];
    zhi = out[5];
    gyroValue = (int16_t)(zlo | (zhi << 8)) * L3GD20_SENSITIVITY_250DPS; // scale the value
    gyroAngle += gyroValue / 100.0; // integrate the value
    gyroLoopCount++;
    if(gyroLoopCount > 16) { // Handle drift, not perfect, but better than nothing
      gyroLoopCount = 0;
      gyroAngle -= 0.01;        
    }

  digitalWrite(13, 0);

}
