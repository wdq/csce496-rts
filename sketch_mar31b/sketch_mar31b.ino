#include <Adafruit_NeoPixel.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <projdefs.h>
//#include "libraries/myi2c/myi2c.h"
#include "libraries/mygyro/mygyro.h"
#include "ci2c.h"

// ***************************************************
// Pin defines
// ***************************************************

#define Accel_Interrupt 2 //used by both Gyro and Accel chips.
#define Accel_Interrupt_Num 0 //pin2 is interrupt 0 on arduino uno board
#define MotorDirection_Right 1
#define MotorDirection_Left 0
#define MotorDrive_Left 6
#define MotorDrive_Right 5
//
#define Chirp 9 //tone(pin, frequency) and noTone(),  or tone(pin, frequency, duration). also look at toneAC library
#define Edge_Lights 8 //turn on IR_FRNT_LEFT_BTM and IR_FRNT_RGHT_BTM
#define _38kHz_Rx 3
#define LightSense_Rear 3 //AD3
#define Source_Select 4 
#define LightSense_Left 2 //AD2 //Source_Select LOW=AMB_FRNT_LEFT, HIGH=EDGE_FRNT_LEFT
#define LightSense_Right 1 //AD1 //Source_Select LOW=AMB_FRNT_RIGHT, HIGH=EDGE_FRNT_RIGHT
#define MotorCapBattVolts 0 //AD0 //Source_Select LOW=motor capacitor, HIGH=battery
//
#define IR_Enable_Front 13
#define IR_Enable_RearLeft 12
#define IR_Enable_RearRight 11
#define IR_Send 10
//
#define Light_Bus_BTN1 7 //for 6 neo pixel RGB

// ***************************************************
// end Pin defines
// ***************************************************


// Create an available semaphore for using LED hardware
SemaphoreHandle_t ledSemaphore;


// Create an available semaphore for using motor hardware
SemaphoreHandle_t motorSemaphore;

// Create an available semaphore for using I2C hardware
SemaphoreHandle_t i2cSemaphore;


// ***************************************************
// Pixels
// ***************************************************

#define NUM_PIXELS 6

#define EYE_LEFT 5
#define EYE_RIGHT 4 
#define BODY_TOP 3
#define BODY_BOTTOM 2
#define TAIL_TOP 0
#define TAIL_BOTTOM 1

// These functions use the Adafruit NeePixel Libraray https://github.com/adafruit/Adafruit_NeoPixel

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_PIXELS, Light_Bus_BTN1, NEO_GRB + NEO_KHZ800);

// Is reentrant, hardware access guarded by semaphore
void SetPixelRGB(int Pixel, int Red, int Green, int Blue){
  while(xSemaphoreTake(ledSemaphore, (TickType_t) 10) == pdFALSE) {} // Wait on semaphore
  pixels.setPixelColor(Pixel, pixels.Color(Red,Green,Blue));
  xSemaphoreGive(ledSemaphore); // Release the semaphore 
  
}

// Is reentrant, hardware access guarded by semaphore
void RefreshPixels(){
  while(xSemaphoreTake(ledSemaphore, (TickType_t) 10) == pdFALSE) {} // Wait on semaphore
  pixels.show();  
  xSemaphoreGive(ledSemaphore); // Release the semaphore 
}
// ***************************************************
// end Pixels
// ***************************************************

// Motor control function from Ringo.
// Is reentrant, hardware access guarded by semaphore
void Motors(int16_t LeftMotor, int16_t RightMotor) {
  while(xSemaphoreTake(motorSemaphore, (TickType_t) 10) == pdFALSE) {} // Wait on semaphore
  // Had some logic to bound the speeds to the min/max.
  // I removed this since not needed since I know the bounds.
  // Also got rid of the use of the global motor speed variables.
  if(LeftMotor<0){
    digitalWrite(MotorDirection_Left,0);
  } else {
    digitalWrite(MotorDirection_Left,1);
  }
  if(RightMotor<0){
    digitalWrite(MotorDirection_Right,0);
  } else {
    digitalWrite(MotorDirection_Right,1);
  }
    
  analogWrite(MotorDrive_Left,abs(LeftMotor));
  analogWrite(MotorDrive_Right,abs(RightMotor));
  xSemaphoreGive(motorSemaphore); // Release the semaphore 
}

// Hardware begin from Ringo.
// Still needs stuff to inititalize the gyro.
// Is reentrant (disables interrupts, fine since in setup)
void HardwareBegin() {
  cli();
  // Motors begin
  pinMode(MotorDirection_Left, OUTPUT);
  pinMode(MotorDirection_Right, OUTPUT);
  analogWrite(MotorDrive_Left,0);
  analogWrite(MotorDrive_Right,0); 

  // Todo: can nuke stuff I don't need like the IR stuff
  pinMode(Edge_Lights,OUTPUT);
  digitalWrite(Edge_Lights,0);
  pinMode(Source_Select,OUTPUT);
  digitalWrite(Source_Select,0);
  
  pinMode(_38kHz_Rx,INPUT_PULLUP);
  pinMode(IR_Send,OUTPUT);
  digitalWrite(IR_Send,0);
  pinMode(IR_Enable_Front,OUTPUT);
  digitalWrite(IR_Enable_Front,1);
  pinMode(IR_Enable_RearLeft,OUTPUT);
  digitalWrite(IR_Enable_RearLeft,1);
  pinMode(IR_Enable_RearRight,OUTPUT);
  digitalWrite(IR_Enable_RearRight,1);

  // Use lights
  pinMode(Light_Bus_BTN1,OUTPUT);
  digitalWrite(Light_Bus_BTN1, LOW);  

  sei();
}

I2C_SLAVE gyro;


#define   GYRO_ADDRESS  0x6B


uint8_t i2cRead8(uint8_t reg) {
  uint8_t out[1];
  I2C_read(&gyro, reg, &out[0], 1); // read 1 byte at reg into out
  return out[0]; 
}

void i2cWrite8(uint8_t reg, uint8_t value) {
  uint8_t in[1];
  in[0] = value;
  I2C_write(&gyro, reg, &in[0], 1); // write 1 byte from in into reg
}

float gyroX;
float gyroY;
float gyroZ;

#define L3GD20_SENSITIVITY_250DPS  (0.00875F)      // Roughly 22/256 for fixed point match

void GyroRead() {
  uint8_t xhi, xlo, ylo, yhi, zlo, zhi;
  uint8_t out[6];
  I2C_read(&gyro, (L3GD20_REGISTER_OUT_X_L | 0x80), &out[0], 6); // read six bytes
  xlo = out[0];
  xhi = out[1];
  ylo = out[2];
  yhi = out[3];
  zlo = out[4];
  zhi = out[5];

  gyroX = (int16_t)(xlo | (xhi << 8)) * L3GD20_SENSITIVITY_250DPS;
  gyroY = (int16_t)(ylo | (yhi << 8)) * L3GD20_SENSITIVITY_250DPS;
  gyroZ = (int16_t)(zlo | (zhi << 8)) * L3GD20_SENSITIVITY_250DPS;

  
}

void setup() {
  Serial.begin(9600);
  delay(500);
  Serial.println("Starting...");
  vSemaphoreCreateBinary(ledSemaphore);
  vSemaphoreCreateBinary(motorSemaphore);
  vSemaphoreCreateBinary(i2cSemaphore);
  HardwareBegin();
  //Motors(100, -100);
  //SetPixelRGB(1, 0, 255, 0);
  //SetPixelRGB(2, 0, 255, 0);
  //SetPixelRGB(3, 0, 0, 255);
  //SetPixelRGB(4, 255, 255, 0);
  //SetPixelRGB(5, 255, 0, 255);
  //SetPixelRGB(6, 255, 0, 0);
  //RefreshPixels();


  /*i2c_init(); // init I2C bus

  uint8_t startStatus = i2c_start(GYRO_ADDRESS); // connect to gyro
  Serial.print("Start: ");
  Serial.println(startStatus);

  //i2c_transmit();
  uint8_t id = i2cRead8(L3GD20_REGISTER_WHO_AM_I);
  Serial.print("ID: ");
  Serial.println(id);*/

  I2C_init(I2C_FM);
  I2C_slave_init(&gyro, GYRO_ADDRESS, I2C_8B_REG);
  uint8_t id = i2cRead8(L3GD20_REGISTER_WHO_AM_I);
  Serial.print("ID: ");
  Serial.println(id);
  i2cWrite8(L3GD20_REGISTER_CTRL_REG1, 0x0F); // Normal mode, enable all three channels
  i2cWrite8(L3GD20_REGISTER_CTRL_REG4, 0x00); // Make sure it's 250 dps resolution
  
  
  
  
}

float gyroValue = 0;
float gyroAngle = 0;

void loop() {
  uint8_t xhi, xlo, ylo, yhi, zlo, zhi;
  uint8_t out[6];
  I2C_read(&gyro, (L3GD20_REGISTER_OUT_X_L | 0x80), &out[0], 6); // read six bytes
  zlo = out[4];
  zhi = out[5];
  gyroValue = (int16_t)(zlo | (zhi << 8)) * L3GD20_SENSITIVITY_250DPS;
  gyroAngle += gyroValue / 100.0;
  Serial.println(gyroAngle);  

  //GyroRead();
  //Serial.print("X: "); Serial.print((int)gyroX);   Serial.print(" ");
  //Serial.print("Y: "); Serial.print((int)gyroY);   Serial.print(" ");
  //Serial.print("Z: "); Serial.println((int)gyroZ); Serial.print(" ");
  delay(10);

}
