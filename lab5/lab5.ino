/////////////////////////////////////////////////////////////////////////////
////                       Includes + Definitions                        ////
/////////////////////////////////////////////////////////////////////////////

#include <Adafruit_NeoPixel.h>
#include "ci2c.h"

#define TURN_ANGLE    88 // 87 sometimes seems to work better, other times 88
#define CYCLES_SINCE_CORRECTION_THRESHOLD  100

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

/////////////////////////////////////////////////////////////////////////////
////                        Data Structures                              ////
/////////////////////////////////////////////////////////////////////////////

// Struct to hold PID controller parameters.
struct PID {
  int8_t kp;          // Proportional gain
  int8_t ki;          // Integral gain
  int8_t kd;          // Derivative gain
  int8_t integral;    // Growing integral value
  int8_t error;       // Previous error value for derivative
  int8_t dt;          // Run/sampling interval time/rate
  int8_t minimum;     // Minimum allowed output
  int8_t maximum;     // Maximum allowed output
};

typedef struct {
  int16_t angle; // degrees of desired heading
  uint8_t distance; // desired distance in centimeters
  bool isTurn;
} directionData;

/////////////////////////////////////////////////////////////////////////////
////                       Global Variables                              ////
/////////////////////////////////////////////////////////////////////////////

// Some global variables
bool isTurning = false; // These first two are used to keep track of if we're going straight or turning, defaults to straight.
bool isDrivingStraight = false;

bool isAvoidingObstacle = false; // Follow the avoidance code if this is true.
bool isObstacle = false; // Keep track of if there is an obstacle in the way
uint8_t cyclesSinceCorrectionLeft = 0;
uint8_t cyclesSinceCorrectionRight = 0;
uint8_t cyclesSinceCorrectionStraight = 0;

// This is used in the control task to do maneuvers
int directionIndex = 0;
int16_t directionDataAngle;
uint8_t directionDataDistance;
directionData directions[8];

// These are for the drive straight task.
uint8_t straightLoopCounter = 0;
bool straightPush = true;

// Some global variables for gyroscope data
float gyroAngle = 0;
int16_t gyroDegrees = 0;
uint8_t gyroLoopCount = 0;

I2C_SLAVE gyro; // Gyroscope I2C object

/////////////////////////////////////////////////////////////////////////////
////                       Helper Functions                              ////
/////////////////////////////////////////////////////////////////////////////

// Check for obstacle based on PID controller corrections not moving, return true if obstacle, otherwise false.
bool checkForObstacle() {
  if(isAvoidingObstacle) {
    return false; // Already handling it, so no obstacle.
  }
  if(isObstacle) {
    return true;
  }
  if(cyclesSinceCorrectionStraight >= CYCLES_SINCE_CORRECTION_THRESHOLD || 
        cyclesSinceCorrectionLeft >= CYCLES_SINCE_CORRECTION_THRESHOLD ||
        cyclesSinceCorrectionRight >= CYCLES_SINCE_CORRECTION_THRESHOLD) {
    cyclesSinceCorrectionStraight = 0;
    cyclesSinceCorrectionLeft = 0;
    cyclesSinceCorrectionRight = 0;
    return true; // Obstacle    
  }
  return false; // No obstacle
}

// Setup ringo stuff
void ringoSetup() {
  MyHardwareBegin();        //initialize Ringo's brain to work with his circuitry   
  //PlayStartChirp();       //Play startup chirp and blink eyes
  //NavigationBegin();
  ////SimpleGyroNavigation(); 
  //SwitchMyMotorsToSerial(); //Call "SwitchMyMotorsToSerial()" before using Serial.print functions as motors & serial share a line
  //RestartTimer();  
  //NavigationBegin();  
}

// Reset millis back to 0 (so I can use smaller data types in the loop)
extern volatile unsigned long timer0_millis;
void resetMillis(){
  uint8_t oldSREG = SREG;
  cli();
  timer0_millis = 0;
  SREG = oldSREG;
}


// Get the degrees from the gyro
int16_t GetDegrees() {
  //while(xSemaphoreTake(gyroSemaphore, (TickType_t) 10) == pdFALSE) {} // Wait on semaphore
  return (int16_t)(gyroAngle * 9.0); // This scale factor gets it to the angle in degrees in integer
  //xSemaphoreGive(gyroSemaphore); // Release the semaphore 
}

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

/////////////////////////////////////////////////////////////////////////////
////                               Tasks                                 ////
/////////////////////////////////////////////////////////////////////////////

// task code
// This task WCET is rounded up to 5ms
// This task runs every 30ms
void TaskTurn() {
    if(isTurning) { /* begin if turning 90 degrees */
      MySetPixelRGB( 4, 255, 0, 0); // Set the lights to red
      MySetPixelRGB( 5, 255, 0, 0);
      MyRefreshPixels();
      PID pid = (PID){.kp=3, .ki=0, .kd=100, .integral=0, .error=0, .dt=30, .minimum=-90, .maximum=90}; // setup the PID controller
      MyMotors(0,0); // Make sure the motors have stopped before doing anything (todo: maybe a small delay?)
      int16_t setHeading = directionDataAngle;
      
      //SimpleGyroNavigation(); // Pull sensors
      int16_t currentHeading = GetDegrees();
      if(abs(abs(setHeading) - abs(currentHeading)) == 0) { // If we have reached set point, stop.
        MyMotors(0,0);
        isTurning = false; // Change modes
        MySetPixelRGB( 0, 0, 255, 0);
        MySetPixelRGB( 4, 0, 0, 0);
        MySetPixelRGB( 5, 0, 0, 0);
        MyRefreshPixels();
        return; // Leave
      }
      int16_t output = CalculatePID(setHeading, currentHeading, &pid); // Calculate the PID control value
      // I added a 12 offset to the PID output value, otherwise finishing the turn wouldn't happen since the motors would run at a speed that's too slow to turn.
      if(output > 0) { // Need to move right
        output = output + 12;
      } else if(output < 0) { // Need to move left
        output = output - 12;
      }
      MyMotors((int)output,-(int)output); // Drive motors with PID output value
  
    } /* end if turning 90 degrees */
    return;
}

// task code
// This task WCET is rounded up to 9ms
// This task runs every 30ms
void TaskDriveStraight() {
   // Task setup here (like set a pin mode)
   // Task loop here
    if(isDrivingStraight) { /* begin if driving straight */
      MySetPixelRGB( 4, 0, 0, 255); // set the lights to green
      MySetPixelRGB( 5, 0, 0, 255);
      MyRefreshPixels();
      PID pid = (PID){.kp=50, .ki=0, .kd=0, .integral=0, .error=0, .dt=30, .minimum=-100, .maximum=100}; // setup the PID controller      
      MyMotors(0,0); // Make sure the motors have stopped before doing anything (todo: maybe a small delay?)
      int16_t setHeading = directionDataAngle;
      
      //SimpleGyroNavigation();  // Pull sensors
      int16_t currentHeading = GetDegrees();
      int16_t output = CalculatePID(setHeading, currentHeading, &pid); // Get control output
      int16_t headingDiff = currentHeading - setHeading; // Figure out if we need to move left or right, and control motors based on that
      // Runs the motors for 20ms at the control output value to drive towards the set point.
      if(straightPush) {
        straightPush = false;
        straightLoopCounter++; // Keep track of the number of straight driving runs, and change modes back to a turn after 80
        if(headingDiff == 0) {
          cyclesSinceCorrectionStraight = 0;
          cyclesSinceCorrectionLeft++;
          cyclesSinceCorrectionRight++;
          MySetPixelRGB( 3, 0, 0, 255);
          MyRefreshPixels();
        }
        MyMotors(100, 100);  // Drive the motor straight for 30ms to progress forward. The control part above will correct any errors
      } else {
        straightPush = true;
        if(headingDiff > 0) { // Left
          MyMotors(0,(int)abs(output));
          cyclesSinceCorrectionStraight++;
          cyclesSinceCorrectionLeft = 0;
          cyclesSinceCorrectionRight++;
          MySetPixelRGB( 3, 255, 0, 0);
          MyRefreshPixels();   
          //vTaskDelay(30 / portTICK_PERIOD_MS);    
        } else if(headingDiff < 0) { // Right
          MyMotors((int)abs(output), 0); 
          cyclesSinceCorrectionStraight++;
          cyclesSinceCorrectionLeft++;
          cyclesSinceCorrectionRight = 0;
          MySetPixelRGB( 3, 0, 255, 0);
          MyRefreshPixels();  
          //vTaskDelay(30 / portTICK_PERIOD_MS);
        } else {
          MyMotors(100, 100);
          cyclesSinceCorrectionStraight = 0;
          cyclesSinceCorrectionLeft++;
          cyclesSinceCorrectionRight++;
          MySetPixelRGB( 3, 0, 0, 255);
          MyRefreshPixels();
        }
      }

      isObstacle = checkForObstacle();

      // This can be used to control how long the sides of the square/rectangle are, at least sort of, it isn't quite perfect.
      // Originally I did a fixed run time before changing modes (in a third task), but had some issues with inconsistency from it sometimes being
      // stopped when it was turning right or left to correct the straight line driving, this guarantees that it always stops at the same spot, 
      // and doesn't require that I disable interrupts or anything. 
      MySetPixelRGB( 4, 0, straightLoopCounter, 0);
      if(straightLoopCounter == directionDataDistance) {
        straightLoopCounter = 0;
        MyMotors(0,0);
        MySetPixelRGB( 3, 0, 0, 0);
        MySetPixelRGB( 4, 0, 0, 0);
        MySetPixelRGB( 5, 0, 0, 0);
        MyRefreshPixels();
        isDrivingStraight = false; // Change modes
      }
    } /* end if driving straight */
    return;
}

// task code
// This task WCET is rounded up to 1ms
// This task runs every 30ms
void TaskControl() {
   // Task setup here (like set a pin mode)
   // Task loop here
   
    if(isDrivingStraight || isTurning) { // Wait for the straight or turn task to do its thing
      return; // Nothing to do if the other tasks are doing their thing  
    }

    if(!isObstacle && !isAvoidingObstacle) { // If no obstacle, go straight (to reach goal)
      //Serial.println("Straight");
      directionDataAngle = 0;
      directionDataDistance = 50;
      isTurning = false;
      isDrivingStraight = true;
    } else if(isObstacle && !isAvoidingObstacle) { // Try to go around obstacle.
      //Serial.println("Setup avoidance");
      //SimpleGyroNavigation();  // Pull sensors
      int16_t currentHeading = GetDegrees();      
      // todo: might want to back up too
      //MyMotors(-100, -100);
      //vTaskDelay(250 / portTICK_PERIOD_MS);
      MyMotors(0, 0);
      directions[0] = (directionData){.angle=currentHeading+90, .distance=0, .isTurn=true}; // turn 90 degrees
      directions[1] = (directionData){.angle=currentHeading+90, .distance=50, .isTurn=false}; // straight 50
      directions[2] = (directionData){.angle=currentHeading, .distance=0, .isTurn=true}; // turn -90
      directions[3] = (directionData){.angle=currentHeading, .distance=50, .isTurn=false}; // straight 50
      directions[4] = (directionData){.angle=currentHeading-90, .distance=0, .isTurn=true}; // turn -90
      directions[5] = (directionData){.angle=currentHeading-90, .distance=50, .isTurn=false}; // straight 50
      directions[6] = (directionData){.angle=currentHeading, .distance=0, .isTurn=true}; // turn 90 degrees
      directions[7] = (directionData){.angle=0, .distance=0, .isTurn=false}; // stop      
      directionIndex = 0;
      isAvoidingObstacle = true;
      MyMotors(-100, -100); // Back up
    } 
    
    if(isAvoidingObstacle) {
      //Serial.println("Avoiding");
      // Get the next direction
      MyMotors(0, 0);
      directionDataAngle = directions[directionIndex].angle;
      directionDataDistance = directions[directionIndex].distance;
      isTurning = directions[directionIndex].isTurn;
      isDrivingStraight = !isTurning;
      if(directionDataAngle == 0 && directionDataDistance == 0) { // stop condition
        isAvoidingObstacle = false;
        isObstacle = false;
        isTurning = false;
        isDrivingStraight = false;
      }
      directionIndex++;
      MySetPixelRGB( 0, 0, 0, 0);
      MyRefreshPixels();
    }
    return;
}

// Task code
// This task's WCET was rounded up to 1ms.
// This task runs every 10ms
// Sort of acts like a nonperiodic task because it takes up free time in the frames.
void TaskGyro() {
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
}

/////////////////////////////////////////////////////////////////////////////
////                               Setup                                 ////
/////////////////////////////////////////////////////////////////////////////

void setup() {
  delay(2000); // Delay so that my hand can move away before gyro calibrates
  ringoSetup(); // Setup ringo stuff
  I2C_init(I2C_FM); // init I2C bus

  // Initialize the gyro
  //while(xSemaphoreTake(gyroSemaphore, (TickType_t) 10) == pdFALSE) {} // Wait on semaphore
  I2C_slave_init(&gyro, GYRO_ADDRESS, I2C_8B_REG);
  uint8_t id = i2cRead8(L3GD20_REGISTER_WHO_AM_I); // get ID, for testing
  i2cWrite8(L3GD20_REGISTER_CTRL_REG1, 0x0F); // Normal mode, enable all three channels
  i2cWrite8(L3GD20_REGISTER_CTRL_REG4, 0x00); // Make sure it's 250 dps resolution 
  //xSemaphoreGive(gyroSemaphore); // Release the semaphore     
}

/////////////////////////////////////////////////////////////////////////////
////                               Loop                                  ////
/////////////////////////////////////////////////////////////////////////////

/* 
 *  This is where the cyclic executive scheduler switches tasks. 
 *  I adjusted the timing of my tasks to play nicely with each other.
 *  Each task runs once per short 30ms hyperperiod, allowing each to run at fixed 30ms intervals.
 *  The frame size is 10ms, which allows the drive straight task to run.
 *  Other tasks like the control one take less than 1ms so there is a lot of idle time in that frame.
 *  
 *  I rounded up my WCET a ton in hopes that none of them will ever run over. 
 *  This allows me to have an optimistic scheduler that doesn't interrupt the tasks.
 */

void loop() {
  resetMillis();
  int startMillis = millis();
  int frameMillisLength = 10; // 10ms frames
  int hyperPeriodMillisLength = 30; // 30ms hyperperiod
  bool taskTurnDone = false;
  bool taskDriveStraightDone = false;
  bool taskControlDone = false;

  while(1) { // Execute a hyperperiod
    if(millis() >= (startMillis + hyperPeriodMillisLength)) { // Hyperperiod (>=30ms) is over, restart loop
      //Serial.println("Done");
      break;      
    }
    if(millis() >= (startMillis + (0 * frameMillisLength)) && !taskControlDone) { // Run control task at 0ms
      TaskGyro();
      TaskControl();
      //Serial.println("Control");
      taskControlDone = true;     
      
    } else if(millis() >= (startMillis + (1 * frameMillisLength)) && !taskDriveStraightDone) { // Run straight task at 10ms
      TaskGyro();
      TaskDriveStraight();
      //Serial.println("Straight");
      taskDriveStraightDone = true;
      
    } else if(millis() >= (startMillis + (2 * frameMillisLength)) && !taskTurnDone) { // Run turn task at 20ms
      TaskGyro();
      TaskTurn();
      //Serial.println("Turn");
      taskTurnDone = true;      
    } else {
      //Serial.println(millis());
      //delay(1000);
    }
  }
}

