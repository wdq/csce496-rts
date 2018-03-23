/////////////////////////////////////////////////////////////////////////////
////                       Includes + Definitions                        ////
/////////////////////////////////////////////////////////////////////////////

#include "RingoHardware.h"

#define TURN_ANGLE    88 // 87 sometimes seems to work better, other times 88
#define CYCLES_SINCE_CORRECTION_THRESHOLD  100

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

int16_t directionDataAngle;
uint8_t directionDataDistance;
directionData directions[8];

// These are for the drive straight task.
uint8_t straightLoopCounter = 0;
bool straightPush = true;

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
  HardwareBegin();        //initialize Ringo's brain to work with his circuitry   
  //PlayStartChirp();       //Play startup chirp and blink eyes
  //NavigationBegin();
  //SimpleGyroNavigation(); 
  SwitchMotorsToSerial(); //Call "SwitchMotorsToSerial()" before using Serial.print functions as motors & serial share a line
  RestartTimer();  
  NavigationBegin();  
}

// Reset millis back to 0 (so I can use smaller data types in the loop)
extern volatile unsigned long timer0_millis;
void resetMillis(){
  uint8_t oldSREG = SREG;
  cli();
  timer0_millis = 0;
  SREG = oldSREG;
}

/////////////////////////////////////////////////////////////////////////////
////                               Tasks                                 ////
/////////////////////////////////////////////////////////////////////////////

// task code
// This task WCET is rounded up to 5ms
// This task runs every 30ms
void TaskTurn() {
    if(isTurning) { /* begin if turning 90 degrees */
      SetPixelRGB( 4, 255, 0, 0); // Set the lights to red
      SetPixelRGB( 5, 255, 0, 0);
      RefreshPixels();
      PID pid = (PID){.kp=3, .ki=0, .kd=100, .integral=0, .error=0, .dt=30, .minimum=-90, .maximum=90}; // setup the PID controller
      Motors(0,0); // Make sure the motors have stopped before doing anything (todo: maybe a small delay?)
      int16_t setHeading = directionDataAngle;
      
      SimpleGyroNavigation(); // Pull sensors
      int16_t currentHeading = GetDegrees();
      if(abs(abs(setHeading) - abs(currentHeading)) == 0) { // If we have reached set point, stop.
        Motors(0,0);
        isTurning = false; // Change modes
        SetPixelRGB( 4, 0, 0, 0);
        SetPixelRGB( 5, 0, 0, 0);
        RefreshPixels();
        return; // Leave
      }
      int16_t output = CalculatePID(setHeading, currentHeading, &pid); // Calculate the PID control value
      // I added a 12 offset to the PID output value, otherwise finishing the turn wouldn't happen since the motors would run at a speed that's too slow to turn.
      if(output > 0) { // Need to move right
        output = output + 12;
      } else if(output < 0) { // Need to move left
        output = output - 12;
      }
      Motors((int)output,-(int)output); // Drive motors with PID output value
  
    } /* end if turning 90 degrees */
    return;
}

// task code
// This task WCET is rounded up to 10ms
// This task runs every 30ms
void TaskDriveStraight() {
   // Task setup here (like set a pin mode)
   // Task loop here
    if(isDrivingStraight) { /* begin if driving straight */
      SetPixelRGB( 4, 0, 0, 255); // set the lights to green
      SetPixelRGB( 5, 0, 0, 255);
      RefreshPixels();
      PID pid = (PID){.kp=50, .ki=0, .kd=0, .integral=0, .error=0, .dt=30, .minimum=-100, .maximum=100}; // setup the PID controller      
      Motors(0,0); // Make sure the motors have stopped before doing anything (todo: maybe a small delay?)
      int16_t setHeading = directionDataAngle;
      
      SimpleGyroNavigation();  // Pull sensors
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
          SetPixelRGB( 3, 0, 0, 255);
          RefreshPixels();
        }
        Motors(100, 100);  // Drive the motor straight for 30ms to progress forward. The control part above will correct any errors
      } else {
        straightPush = true;
        if(headingDiff > 0) { // Left
          Motors(0,(int)abs(output));
          cyclesSinceCorrectionStraight++;
          cyclesSinceCorrectionLeft = 0;
          cyclesSinceCorrectionRight++;
          SetPixelRGB( 3, 255, 0, 0);
          RefreshPixels();   
          //vTaskDelay(30 / portTICK_PERIOD_MS);    
        } else if(headingDiff < 0) { // Right
          Motors((int)abs(output), 0); 
          cyclesSinceCorrectionStraight++;
          cyclesSinceCorrectionLeft++;
          cyclesSinceCorrectionRight = 0;
          SetPixelRGB( 3, 0, 255, 0);
          RefreshPixels();  
          //vTaskDelay(30 / portTICK_PERIOD_MS);
        } else {
          cyclesSinceCorrectionStraight = 0;
          cyclesSinceCorrectionLeft++;
          cyclesSinceCorrectionRight++;
          SetPixelRGB( 3, 0, 0, 255);
          RefreshPixels();
        }
      }

      isObstacle = checkForObstacle();

      // This can be used to control how long the sides of the square/rectangle are, at least sort of, it isn't quite perfect.
      // Originally I did a fixed run time before changing modes (in a third task), but had some issues with inconsistency from it sometimes being
      // stopped when it was turning right or left to correct the straight line driving, this guarantees that it always stops at the same spot, 
      // and doesn't require that I disable interrupts or anything. 
      SetPixelRGB( 4, 0, straightLoopCounter, 0);
      if(straightLoopCounter == directionDataDistance) {
        straightLoopCounter = 0;
        Motors(0,0);
        SetPixelRGB( 3, 0, 0, 0);
        SetPixelRGB( 4, 0, 0, 0);
        SetPixelRGB( 5, 0, 0, 0);
        RefreshPixels();
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
   int directionIndex = 0;
   
    if(isDrivingStraight || isTurning) { // Wait for the straight or turn task to do its thing
      return; // Nothing to do if the other tasks are doing their thing  
    }

    if(!isObstacle && !isAvoidingObstacle) { // If no obstacle, go straight (to reach goal)
      //Serial.println("Straight");
      directionDataAngle = 0;
      directionDataDistance = 50;
      isTurning = false;
      isDrivingStraight = true;
    } else if(!isAvoidingObstacle) { // Try to go around obstacle.
      //Serial.println("Setup avoidance");
      SimpleGyroNavigation();  // Pull sensors
      int16_t currentHeading = GetDegrees();      
      // todo: might want to back up too
      //Motors(-100, -100);
      //vTaskDelay(250 / portTICK_PERIOD_MS);
      Motors(0, 0);
      directions[0] = (directionData){.angle=currentHeading+90, .distance=0, .isTurn=true}; // turn 90 degrees
      directions[1] = (directionData){.angle=currentHeading+90, .distance=25, .isTurn=false}; // straight 25
      directions[2] = (directionData){.angle=currentHeading, .distance=0, .isTurn=true}; // turn -90
      directions[3] = (directionData){.angle=currentHeading, .distance=50, .isTurn=false}; // straight 50
      directions[4] = (directionData){.angle=currentHeading-90, .distance=0, .isTurn=true}; // turn -90
      directions[5] = (directionData){.angle=currentHeading-90, .distance=25, .isTurn=false}; // straight 25
      directions[6] = (directionData){.angle=currentHeading, .distance=0, .isTurn=true}; // turn 90 degrees
      directions[7] = (directionData){.angle=0, .distance=0, .isTurn=false}; // stop      
      directionIndex = 0;
      isAvoidingObstacle = true;
      Motors(-100, -100); // Back up
    } else {
      if(isAvoidingObstacle) {
        //Serial.println("Avoiding");
        // Get the next direction
        Motors(0, 0);
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
      }
    }
    return;
}

/////////////////////////////////////////////////////////////////////////////
////                               Setup                                 ////
/////////////////////////////////////////////////////////////////////////////

void setup() {
  delay(2000); // Delay so that my hand can move away before gyro calibrates
  ringoSetup(); // Setup ringo stuff
  Serial.begin(9600); // For debugging
  Serial.println("Setup");  
  Serial.println("Starting tasks");
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
      TaskControl();
      //Serial.println("Control");
      taskControlDone = true;     
      
    } else if(millis() >= (startMillis + (1 * frameMillisLength)) && !taskDriveStraightDone) { // Run straight task at 10ms
      TaskDriveStraight();
      //Serial.println("Straight");
      taskDriveStraightDone = true;
      
    } else if(millis() >= (startMillis + (2 * frameMillisLength)) && !taskTurnDone) { // Run turn task at 20ms
      TaskTurn();
      //Serial.println("Turn");
      taskTurnDone = true;      
    } else {
      //Serial.println(millis());
      //delay(1000);
    }
  }
}

