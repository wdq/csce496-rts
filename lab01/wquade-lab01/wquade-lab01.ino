#include <Adafruit_NeoPixel.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <projdefs.h>

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

#define TURN_ANGLE    88 // 87 sometimes seems to work better, other times 88
#define CYCLES_SINCE_CORRECTION_THRESHOLD  100

// Semaphores
SemaphoreHandle_t ledSemaphore; // Create an available semaphore for using LED hardware
SemaphoreHandle_t motorSemaphore; // Create an available semaphore for using motor hardware

// Some global variables
bool isTurning = false; // These first two are used to keep track of if we're going straight or turning, defaults to straight.
bool isDrivingStraight = false;

bool isAvoidingObstacle = false; // Follow the avoidance code if this is true.
bool isObstacle = false; // Keep track of if there is an obstacle in the way
uint8_t cyclesSinceCorrectionLeft = 0;
uint8_t cyclesSinceCorrectionRight = 0;
uint8_t cyclesSinceCorrectionStraight = 0;

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

// This is used in the control task to do maneuvers
int directionIndex = 0;
int16_t directionDataAngle;
uint8_t directionDataDistance;
directionData directions[8];

// These are for the drive straight task.
uint8_t straightLoopCounter = 0;
bool straightPush = true;

// Define tasks
void TaskTurn(void *pvParameters); // Turn
void TaskDriveStraight(void *pvParameters); // Go straight
void TaskControl(void *pvParameters);   // Control the robot by triggering the other tasks.

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
  //RestartTimer();  
  //NavigationBegin();  
}

// Setup the tasks
void taskSetup() {
  // Create tasks   
       
  xTaskCreate(
    TaskTurn, // task function
    (const portCHAR *)"Turn", // task name string
    95, // stack size
    NULL, // nothing
    1, // Priority, 0 is lowest
    NULL); // nothing
    
  xTaskCreate(
    TaskDriveStraight, // task function
    (const portCHAR *)"DriveStraight", // task name string
    110, // stack size
    NULL, // nothing
    0, // Priority, 0 is lowest
    NULL); // nothing   
    
  xTaskCreate(
    TaskControl, // task function
    (const portCHAR *)"Control", // task name string
    145, // stack size
    NULL, // nothing
    2, // Priority, 0 is lowest
    NULL); // nothing                   
}

// First run code
void setup(){
  // Initialize semaphores
  vSemaphoreCreateBinary(ledSemaphore);
  vSemaphoreCreateBinary(motorSemaphore);
      
  delay(2000); // Delay so that my hand can move away before gyro calibrates
  ringoSetup(); // Setup ringo stuff
  
  taskSetup(); // Setup the tasks
}

// Don't do anything here since the tasks do the work
void loop(){}

// task code
void TaskTurn(void *pvParameters) {
  (void) pvParameters;
   // Task setup here (like set a pin mode)
   // Task loop here
   while(1) { /* begin task loop */
    if(isTurning) { /* begin if turning 90 degrees */
      MySetPixelRGB( 4, 255, 0, 0); // Set the lights to red
      MySetPixelRGB( 5, 255, 0, 0);
      MyRefreshPixels();
      PID pid = (PID){.kp=3, .ki=0, .kd=100, .integral=0, .error=0, .dt=30, .minimum=-90, .maximum=90}; // setup the PID controller
      MyMotors(0,0); // Make sure the motors have stopped before doing anything (todo: maybe a small delay?)
      int16_t setHeading = directionDataAngle;
      
      MySimpleGyroNavigation(); // Pull sensors
      int16_t currentHeading = MyGetDegrees();
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
    vTaskDelay(25 / portTICK_PERIOD_MS); // Schedule to run every 25ms (this runs when the task is idle, not turning)
   } /* end task loop */
}

// task code
void TaskDriveStraight(void *pvParameters) {
  (void) pvParameters;
   // Task setup here (like set a pin mode)
   // Task loop here
   uint8_t straightLoopCounter = 0;
   while(1) { /* begin task loop */
    if(isDrivingStraight) { /* begin if driving straight */
      MySetPixelRGB( 4, 0, 0, 255); // set the lights to green
      MySetPixelRGB( 5, 0, 0, 255);
      MyRefreshPixels();
      PID pid = (PID){.kp=50, .ki=0, .kd=0, .integral=0, .error=0, .dt=30, .minimum=-100, .maximum=100}; // setup the PID controller      
      MyMotors(0,0); // Make sure the motors have stopped before doing anything (todo: maybe a small delay?)
      int16_t setHeading = directionDataAngle;
      
      MySimpleGyroNavigation();  // Pull sensors
      int16_t currentHeading = MyGetDegrees();
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
    vTaskDelay(30 / portTICK_PERIOD_MS); // Schedule to run every 30ms (this runs when the task is idle, not going straight)
   } /* end task loop */
}

// task code
void TaskControl(void *pvParameters) {
  (void) pvParameters;
   // Task setup here (like set a pin mode)
   // Task loop here
   int directionIndex = 0;
   while(1) { /* begin task loop */

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
      MySimpleGyroNavigation();  // Pull sensors
      int16_t currentHeading = MyGetDegrees();      
      // todo: might want to back up too
      //Motors(-100, -100);
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


    
    vTaskDelay(20 / portTICK_PERIOD_MS); // Schedule to run every 20ms
   }
}


