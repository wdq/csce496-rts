
/* 

Ringo Robot
Ringo_Base_Sketch_Rev06
Version 6.1 12/2015

This is a basic sketch that can be used as a starting point
for various functionality of the Ringo robot.

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


#define TURN_ANGLE    88 // 87 sometimes seems to work better, other times 88
#define CYCLES_SINCE_CORRECTION_THRESHOLD  100

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

int16_t directionDataAngle;
uint8_t directionDataDistance;
directionData directions[8];

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

void setup(){
  HardwareBegin();        //initialize Ringo's brain to work with his circuitry
  //PlayStartChirp();       //Play startup chirp and blink eyes
  SwitchMotorsToSerial(); //Call "SwitchMotorsToSerial()" before using Serial.print functions as motors & serial share a line
  RestartTimer();
  NavigationBegin();
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  digitalWrite(13, 0);
      directionDataAngle = 90;
      directionDataDistance = 0;  
}


void TaskTurnWCET() {
      volatile bool test1 = true;
      volatile bool test2 = true;
      if(test1) {
        test1 = false;
      }
      if(test2) {
        test1 = true;
      }
      SetPixelRGB( 4, 255, 0, 0); // Set the lights to red
      SetPixelRGB( 5, 255, 0, 0);
      RefreshPixels();
      PID pid = (PID){.kp=3, .ki=0, .kd=100, .integral=0, .error=0, .dt=25, .minimum=-90, .maximum=90}; // setup the PID controller
      Motors(0,0); // Make sure the motors have stopped before doing anything (todo: maybe a small delay?)
      //ZeroNavigation();
      SimpleGyroNavigation(); // Pull sensors
      int16_t setHeading = directionDataAngle;

        SimpleGyroNavigation(); // Pull sensors
        int16_t currentHeading = GetDegrees();
                  volatile int testNumber = 0;
        if(abs(abs(setHeading) - abs(currentHeading)) == 0) { // If we have reached set point, stop.
          volatile int testNumber2 = 0;
          testNumber++;

        }
                  testNumber--;
        
          Motors(0,testNumber);
          isTurning = false; // Change modes
          SetPixelRGB( 4, 0, 0, 0);
          SetPixelRGB( 5, 0, 0, 0);
          RefreshPixels();
          //break; // Leave
        //}
        volatile int16_t output = CalculatePID(setHeading, currentHeading, &pid); // Calculate the PID control value
        // I added a 12 offset to the PID output value, otherwise finishing the turn wouldn't happen since the motors would run at a speed that's too slow to turn.
        if(output > 0) { // Need to move right
          output = output + 12;
        } else if(output < 0) { // Need to move left
          output = output - 12;
        }
        Motors((int)output,-(int)output); // Drive motors with PID output value      
}

void TaskStraightWCET() {
      volatile uint8_t straightLoopCounter = 0;
      volatile bool test1 = true;
      volatile bool test2 = true;
      if(test1) {
        test1 = false;
      }
      if(test2) {
        test1 = true;
      }
      SetPixelRGB( 4, 0, 0, 255); // set the lights to green
      SetPixelRGB( 5, 0, 0, 255);
      RefreshPixels();
      PID pid = (PID){.kp=50, .ki=0, .kd=0, .integral=0, .error=0, .dt=50, .minimum=-100, .maximum=100}; // setup the PID controller      
      Motors(0,0); // Make sure the motors have stopped before doing anything (todo: maybe a small delay?)
      //ZeroNavigation();
      SimpleGyroNavigation(); // Pull sensors
      int16_t setHeading = directionDataAngle;
        SimpleGyroNavigation();  // Pull sensors
        int16_t currentHeading = GetDegrees();
        int16_t output = CalculatePID(setHeading, currentHeading, &pid); // Get control output
        int16_t headingDiff = currentHeading - setHeading; // Figure out if we need to move left or right, and control motors based on that
        // Runs the motors for 20ms at the control output value to drive towards the set point.
        if(headingDiff > 10000) { // Left
          Motors((int)abs(output), 0); 
          cyclesSinceCorrectionStraight++;
          cyclesSinceCorrectionLeft++;
          cyclesSinceCorrectionRight = 0;
          SetPixelRGB( 3, 0, 255, 0);
          RefreshPixels();  
          //delay(30);
        } else if(headingDiff < -10000) { // Right
          Motors((int)abs(output), 0); 
          cyclesSinceCorrectionStraight++;
          cyclesSinceCorrectionLeft++;
          cyclesSinceCorrectionRight = 0;
          SetPixelRGB( 3, 0, 255, 0);
          RefreshPixels();
          //delay(30);  
        } else {
          Motors(0,(int)abs(output));
          cyclesSinceCorrectionStraight++;
          cyclesSinceCorrectionLeft = 0;
          cyclesSinceCorrectionRight++;
          SetPixelRGB( 3, 255, 0, 0);
          RefreshPixels();       
          cyclesSinceCorrectionStraight = 0;
          cyclesSinceCorrectionLeft++;
          cyclesSinceCorrectionRight++;
          SetPixelRGB( 3, 0, 0, 255);
          RefreshPixels();
          //delay(30);
        }
        isObstacle = checkForObstacle();

        Motors(100, 100);  // Drive the motor straight for 30ms to progress forward. The control part above will correct any errors
        straightLoopCounter++; // Keep track of the number of straight driving runs, and change modes back to a turn after 80
        // This can be used to control how long the sides of the square/rectangle are, at least sort of, it isn't quite perfect.
        // Originally I did a fixed run time before changing modes (in a third task), but had some issues with inconsistency from it sometimes being
        // stopped when it was turning right or left to correct the straight line driving, this guarantees that it always stops at the same spot, 
        // and doesn't require that I disable interrupts or anything. 
        SetPixelRGB( 4, 0, straightLoopCounter, 0);
        uint8_t testnumber = 0;
        if(testnumber == 0) { // Need to move right
          straightLoopCounter = 10;
          //cyclesSinceCorrectionStraight = 0;
          //cyclesSinceCorrectionLeft = 0;
          //cyclesSinceCorrectionRight = 0;
          Motors(0,0);
          SetPixelRGB( 3, 0, 0, 0);
          SetPixelRGB( 4, 0, 0, 0);
          SetPixelRGB( 5, 0, 0, 0);
          RefreshPixels();
          isDrivingStraight = false; // Change modes
        }
        
}

void TaskControlWCET() {
   volatile int directionIndex = 0;
      volatile bool test1 = true;
      volatile bool test2 = true;
      if(test1) {
        test1 = false;
      }
      if(test2) {
        test1 = true;
      }
            if(test1 && test2) {
        test1 = true;
      }
   
    //if(!isObstacle && !isAvoidingObstacle) { // If no obstacle, go straight (to reach goal)
      //Serial.println("Straight");
      directionDataAngle = 0;
      directionDataDistance = 50;
      isTurning = false;
      isDrivingStraight = true;
   // } else if(!isAvoidingObstacle) { // Try to go around obstacle.
      //Serial.println("Setup avoidance");
      SimpleGyroNavigation();  // Pull sensors
      int16_t currentHeading = GetDegrees();      
      // todo: might want to back up too
      Motors(-100, -100);
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
    //}

    //if(isAvoidingObstacle) {
      //Serial.println("Avoiding");
      // Get the next direction
      directionDataAngle = directions[directionIndex].angle;
      directionDataDistance = directions[directionIndex].distance;
      isTurning = directions[directionIndex].isTurn;
      isDrivingStraight = !isTurning;
      if((directionDataAngle == 0 && directionDataDistance == 0) || test1) { // stop condition
        isAvoidingObstacle = false;
        isObstacle = false;
        isTurning = false;
        isDrivingStraight = false;
      }
      directionIndex++;
    //}
}

// To find WCET:
// Run the right task in the loop.
// The tasks were modified to hopefully go through their worst case.
// I've added useless code to it so that it will still go through the same sorts of conditionals and things.
// Hopefully the compiler isn't removing too much stuff.
// Connect an oscilloscope to the pin 13 (SCK/IR_ENABLE_FRONT) on the programming header. 
// Then measure the width of the high time of the square wave.
// Let it run for a while, and grab the highest width you see, then add 25%. 
void loop(){ 
  digitalWrite(13, 1);
  //delay(1);
  // Turn:
  // 2.5ms was the longest I saw.
  // 3.125ms is 2.5ms + 25%.
  // I'll round that up to 3.5ms.
  //TaskTurnWCET();

  // Straight:
  // 40ms was the longest I saw.
  // 50ms is 40ms + 25%.
  // I'll stick with 50ms.
  // This one could be cut down if I redesign the task so the delays aren't part of it.
  //TaskStraightWCET();
  // If I were to move the delays away from inside the task this would be:
  // 4.45ms time high.
  // 5.5625 is +25%.
  // So round that up to 6ms.

    // Control:
  //TaskControlWCET();
  // 375us was the longest I saw.
  // 468.75us is 375us + 25%.
  // I'll stick with 500us (0.5ms).
  // This is with the delays refactored.
  digitalWrite(13, 0);
  //delay(1);


}




