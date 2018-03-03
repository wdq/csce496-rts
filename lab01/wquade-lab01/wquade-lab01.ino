#include <Arduino_FreeRTOS.h>
#include "RingoHardware.h"

#define TURN_ANGLE    88 // 87 sometimes seems to work better, other times 88

// Some global variables
bool isTurning = false; // These first two are used to keep track of if we're going straight or turning, defaults to straight.
bool isDrivingStraight = false;
bool directionIsRight = false; // This is used to switch between making right turns, and making left turns.
uint8_t turnCount = 0; // Keep track of the turn number to get angle to set

// Struct to hold PID controller parameters.
struct PID {
  double kp;          // Proportional gain
  double ki;          // Integral gain
  double kd;          // Derivative gain
  double integral;    // Growing integral value
  double error;       // Previous error value for derivative
  double dt;          // Run/sampling interval time/rate
  double minimum;     // Minimum allowed output
  double maximum;     // Maximum allowed output
};

typedef struct {
  int16_t angle; // degrees of desired heading
  int distance; // desired distance in centimeters
} directionData;

int16_t directionDataAngle;
int directionDataDistance;
directionData directions[20];



// Define tasks
void TaskTurn(void *pvParameters); // Turn
void TaskDriveStraight(void *pvParameters); // Go straight
void TaskControl(void *pvParameters);   // Control the robot by triggering the other tasks.

// Setup ringo stuff
void ringoSetup() {
  HardwareBegin();        //initialize Ringo's brain to work with his circuitry   
  //PlayStartChirp();       //Play startup chirp and blink eyes
  //NavigationBegin();
  //SimpleGyroNavigation(); 
  SwitchMotorsToSerial(); //Call "SwitchMotorsToSerial()" before using Serial.print functions as motors & serial share a line
  RestartTimer();  
  NavigationBegin();  
}

// Setup the tasks
void taskSetup() {
  // Create tasks        
  xTaskCreate(
    TaskTurn, // task function
    (const portCHAR *)"Turn", // task name string
    128, // stack size
    NULL, // nothing
    0, // Priority, 0 is lowest
    NULL); // nothing
    
  xTaskCreate(
    TaskDriveStraight, // task function
    (const portCHAR *)"DriveStraight", // task name string
    128, // stack size
    NULL, // nothing
    1, // Priority, 0 is lowest
    NULL); // nothing   
    
  xTaskCreate(
    TaskControl, // task function
    (const portCHAR *)"Control", // task name string
    128, // stack size
    NULL, // nothing
    2, // Priority, 0 is lowest
    NULL); // nothing              
}


// First run code
void setup(){
  // Equilateral triangle
  /*
  directions[0] = (directionData){.angle=0, .distance=40}; // straight 40
  directions[1] = (directionData){.angle=60, .distance=0}; // turn 60 degrees
  directions[2] = (directionData){.angle=0, .distance=40}; // straight 40
  directions[3] = (directionData){.angle=60, .distance=0}; // turn 60 degrees
  directions[4] = (directionData){.angle=0, .distance=40}; // straight 40
  directions[5] = (directionData){.angle=0, .distance=0}; // stop
  */

  // Square
  
  directions[0] = (directionData){.angle=0, .distance=80}; // straight 80
  directions[1] = (directionData){.angle=90, .distance=0}; // turn 90 degrees
  directions[2] = (directionData){.angle=0, .distance=80}; // straight 80
  directions[3] = (directionData){.angle=90, .distance=0}; // turn 90 degrees
  directions[4] = (directionData){.angle=0, .distance=80}; // straight 80
  directions[5] = (directionData){.angle=90, .distance=0}; // turn 90 degrees
  directions[6] = (directionData){.angle=0, .distance=80}; // straight 90  
  directions[7] = (directionData){.angle=90, .distance=0}; // turn 90 degrees  
  directions[8] = (directionData){.angle=0, .distance=0}; // stop
  
  
  delay(2000); // Delay so that my hand can move away before gyro calibrates
  ringoSetup(); // Setup ringo stuff
  taskSetup(); // Setup the tasks
  Serial.begin(9600); // For debugging
  Serial.println("Setup");  

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
      SetPixelRGB( 4, 255, 0, 0); // Set the lights to red
      SetPixelRGB( 5, 255, 0, 0);
      RefreshPixels();
      PID pid = (PID){.kp=3.2, .ki=0, .kd=100, .integral=0, .error=0, .dt=25, .minimum=-90, .maximum=90}; // setup the PID controller
      Motors(0,0); // Make sure the motors have stopped before doing anything (todo: maybe a small delay?)
      ZeroNavigation();
      SimpleGyroNavigation(); // Pull sensors
      int16_t setHeading = directionDataAngle;
      while(isTurning) { /* begin turning 90 degrees loop */
        SimpleGyroNavigation(); // Pull sensors
        int16_t currentHeading = GetDegrees();
        if(abs(abs(setHeading) - abs(currentHeading)) == 0) { // If we have reached set point, stop.
          Motors(0,0);
          isTurning = false; // Change modes
          SetPixelRGB( 4, 0, 0, 0);
          SetPixelRGB( 5, 0, 0, 0);
          RefreshPixels();
          break; // Leave
        }
        double output = CalculatePID(setHeading, currentHeading, &pid); // Calculate the PID control value
        // I added a 12 offset to the PID output value, otherwise finishing the turn wouldn't happen since the motors would run at a speed that's too slow to turn.
        if(output > 0) { // Need to move right
          output = output + 12;
        } else if(output < 0) { // Need to move left
          output = output - 12;
        }
        Motors((int)output,-(int)output); // Drive motors with PID output value
  
        vTaskDelay(25 / portTICK_PERIOD_MS); // Drive the motors at the control value for 25ms, before running the control loop again
      } /* end turning 90 degrees loop */
    } /* end if turning 90 degrees */
    vTaskDelay(25 / portTICK_PERIOD_MS); // Schedule to run every 25ms (this runs when the task is idle, not turning)
   } /* end task loop */
}

// task code
void TaskDriveStraight(void *pvParameters) {
  (void) pvParameters;
   // Task setup here (like set a pin mode)
   // Task loop here
   uint16_t straightLoopCounter = 0;
   while(1) { /* begin task loop */
    if(isDrivingStraight) { /* begin if driving straight */
      SetPixelRGB( 4, 0, 0, 255); // set the lights to green
      SetPixelRGB( 5, 0, 0, 255);
      RefreshPixels();
      PID pid = (PID){.kp=50, .ki=0, .kd=0, .integral=0, .error=0, .dt=50, .minimum=-100, .maximum=100}; // setup the PID controller      
      Motors(0,0); // Make sure the motors have stopped before doing anything (todo: maybe a small delay?)
      ZeroNavigation();
      SimpleGyroNavigation(); // Pull sensors
      int16_t startingHeading = GetDegrees();
      int16_t setHeading = directionDataAngle;
      while(isDrivingStraight) { /* begin driving straight loop */
        SimpleGyroNavigation();  // Pull sensors
        int16_t currentHeading = GetDegrees();
        double output = CalculatePID(setHeading, currentHeading, &pid); // Get control output
        int16_t headingDiff = currentHeading - setHeading; // Figure out if we need to move left or right, and control motors based on that
        // Runs the motors for 20ms at the control output value to drive towards the set point.
        if(headingDiff > 0) { // Left
          Motors(0,(int)abs(output));      
          vTaskDelay(20 / portTICK_PERIOD_MS);    
        } else if(headingDiff < 0) { // Right
          Motors((int)abs(output), 0); 
          vTaskDelay(20 / portTICK_PERIOD_MS);
        }

        Motors(100, 100);  // Drive the motor straight for 30ms to progress forward. The control part above will correct any errors
        straightLoopCounter++; // Keep track of the number of straight driving runs, and change modes back to a turn after 80
        // This can be used to control how long the sides of the square/rectangle are, at least sort of, it isn't quite perfect.
        // Originally I did a fixed run time before changing modes (in a third task), but had some issues with inconsistency from it sometimes being
        // stopped when it was turning right or left to correct the straight line driving, this guarantees that it always stops at the same spot, 
        // and doesn't require that I disable interrupts or anything. 
        if(straightLoopCounter == directionDataDistance) {
          straightLoopCounter = 0;
          isDrivingStraight = false; // Change modes
          Motors(0,0);
          SetPixelRGB( 4, 0, 0, 0);
          SetPixelRGB( 5, 0, 0, 0);
          RefreshPixels();
        }
        
  
        vTaskDelay(30 / portTICK_PERIOD_MS); // Drive the motors straight for 30ms
      } /* end driving straight loop */
    } /* end if driving straight */
    vTaskDelay(50 / portTICK_PERIOD_MS); // Schedule to run every 50ms (this runs when the task is idle, not going straight)
   } /* end task loop */
}

// task code
void TaskControl(void *pvParameters) {
  (void) pvParameters;
   // Task setup here (like set a pin mode)
   // Task loop here
   int directionIndex = 0;
   while(1) { /* begin task loop */
    //SimpleGyroNavigation(); 
    //int16_t currentHeading = GetDegrees();
    //Serial.println(currentHeading);
    //vTaskDelay(100 / portTICK_PERIOD_MS); // Schedule to run every 100ms

    while(isDrivingStraight || isTurning) { // Wait for the straight or turn task to do its thing
      vTaskDelay(250 / portTICK_PERIOD_MS); // Schedule to run every 250ms      
    }

    // Get the next direction
    directionDataAngle = directions[directionIndex].angle;
    directionDataDistance = directions[directionIndex].distance;
    if(directionDataAngle == 0 && directionDataDistance == 0) { // stop condition
      while(1) {
        // just wait, since the task is done
        vTaskDelay(250 / portTICK_PERIOD_MS);
      }
    }
    directionIndex++;

    // Update booleans to get the tasks to run.
    if(directionDataDistance == 0) { // no distance, so must be a turn
      isTurning = true;
      isDrivingStraight = false;
    } else if(directionDataAngle == 0) { // no angle, so must be straight
      isTurning = false;
      isDrivingStraight = true;      
    }

    // Enable the correct thing

    
    vTaskDelay(250 / portTICK_PERIOD_MS); // Schedule to run every 250ms, may need to lower this if there is a noticable pause in between switching the other tasks
   }
}


