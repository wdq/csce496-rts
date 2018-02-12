#include <Arduino_FreeRTOS.h>
#include "RingoHardware.h"

// Define tasks
void TaskTurn90Degrees(void *pvParameters); // Turn 90 degrees
void TaskDriveStraight(void *pvParameters); // Go straight
void TaskDriveSquare(void *pvParameters);   // Drive in a square

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
    TaskTurn90Degrees, // task function
    (const portCHAR *)"Turn90Degrees", // task name string
    128, // stack size
    NULL, // nothing
    4, // Priority, 0 is lowest
    NULL); // nothing
    
  xTaskCreate(
    TaskDriveStraight, // task function
    (const portCHAR *)"DriveStraight", // task name string
    128, // stack size
    NULL, // nothing
    5, // Priority, 0 is lowest
    NULL); // nothing   
    
  xTaskCreate(
    TaskDriveSquare, // task function
    (const portCHAR *)"DriveSquare", // task name string
    128, // stack size
    NULL, // nothing
    2, // Priority, 0 is lowest
    NULL); // nothing              
}


// First run code
void setup(){
  ringoSetup(); // Setup ringo stuff
  taskSetup(); // Setup the tasks
  Serial.begin(9600); // For debugging
  Serial.println("Setup");

}

// Don't do anything here since the tasks do the work
void loop(){}

bool isTurning90Degrees = false;
bool isDrivingStraight = true;
bool directionIsRight = true;

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

// Function to calculate PID output given set point, current value (process variable), and PID controller parameters.
// todo: look into extending this to support combinations of P, I, D control instead of only all three together.
// Based on this: https://gist.github.com/bradley219/5373998
double CalculatePID(double setPoint, double processVariable, struct PID *pid) {
  // Error
  double error = setPoint - processVariable;

  // Proportional
  double proportionalOutput = pid->kp * error;

  // Integral
  pid->integral += error * pid->dt;
  double integralOutput = pid->ki * pid->integral;

  // Derivative
  double derivative = (error - pid->error) / pid->dt;
  double derivativeOutput = pid->kd * derivative;

  // Output
  double output = proportionalOutput + integralOutput + derivativeOutput;  

  // Limit to minimum and maximum
  if(output > pid->maximum) {
    output = pid->maximum;
  } else if(output < pid->minimum) {
    output = pid->minimum;
  }

  // Save error for derivative
  pid->error = error;

  return output;
}

// task code
void TaskTurn90Degrees(void *pvParameters) {
  (void) pvParameters;
   // Task setup here (like set a pin mode)
   
   // Task loop here
   while(1) {
    if(isTurning90Degrees) {
      SetPixelRGB( 4, 255, 0, 0);
      SetPixelRGB( 5, 255, 0, 0);
      RefreshPixels();
      PID pid;
      pid.kp = 3.2;
      pid.ki = 0;
      pid.kd = 100;
      pid.integral = 0;
      pid.error = 0;
      pid.dt = 25;
      pid.minimum = -90;
      pid.maximum = 90;
      Motors(0,0);
      SimpleGyroNavigation(); 
      int16_t startingHeading = GetDegrees();
      int16_t setHeading = 0;
      if(directionIsRight) {
        setHeading = startingHeading + 90;
      } else {
        setHeading = startingHeading - 90;
      }
      while(isTurning90Degrees) {
        SimpleGyroNavigation(); 
        int16_t currentHeading = GetDegrees();
        if(abs(abs(setHeading) - abs(currentHeading)) == 0) {
          Motors(0,0);
          isTurning90Degrees = false;
          isDrivingStraight = true;
          SetPixelRGB( 4, 0, 0, 0);
          SetPixelRGB( 5, 0, 0, 0);
          RefreshPixels();
          break;
        }
        double output = CalculatePID(setHeading, currentHeading, &pid);
        if(output > 0) {
          output = output + 12;
        } else if(output < 0) {
          output = output - 12;
        }
        Motors((int)output,-(int)output);
  
        vTaskDelay(25 / portTICK_PERIOD_MS); // Schedule to run every 250ms
      }
    }
    vTaskDelay(25 / portTICK_PERIOD_MS); // Schedule to run every 250ms
   }
}

// task code
void TaskDriveStraight(void *pvParameters) {
  (void) pvParameters;
   // Task setup here (like set a pin mode)
   
   // Task loop here
   uint16_t straightLoopCounter = 0;
   while(1) {
    if(isDrivingStraight) {
      SetPixelRGB( 4, 0, 0, 255);
      SetPixelRGB( 5, 0, 0, 255);
      RefreshPixels();
      PID pid;
      pid.kp = 50;
      pid.ki = 0;
      pid.kd = 0;
      pid.integral = 0;
      pid.error = 0;
      pid.dt = 50;
      pid.minimum = -100;
      pid.maximum = 100;
      Motors(0,0);
      SimpleGyroNavigation(); 
      ZeroNavigation();
      int16_t startingHeading = GetDegrees();
      int16_t setHeading = startingHeading;
      while(isDrivingStraight) {
        SimpleGyroNavigation(); 
        int16_t currentHeading = GetDegrees();
        double output = CalculatePID(setHeading, currentHeading, &pid);
        if(currentHeading > 0) {
          Motors(0,(int)abs(output));      
          vTaskDelay(20 / portTICK_PERIOD_MS);    
        } else if(currentHeading < 0) {
          Motors((int)abs(output), 0); 
          vTaskDelay(20 / portTICK_PERIOD_MS);
        }
        //Serial.println(output);
        Motors(100, 100); 
        straightLoopCounter++;
        if(straightLoopCounter == 80) {
          straightLoopCounter = 0;
          isDrivingStraight = false;
          isTurning90Degrees = true;
          Motors(0,0);
          SetPixelRGB( 4, 0, 0, 0);
          SetPixelRGB( 5, 0, 0, 0);
          RefreshPixels();
        }
        
  
        vTaskDelay(30 / portTICK_PERIOD_MS); // Schedule to run every 250ms
      }
    }
    vTaskDelay(50 / portTICK_PERIOD_MS); // Schedule to run every 250ms
   }
}

// task code
void TaskDriveSquare(void *pvParameters) {
  (void) pvParameters;
   // Task setup here (like set a pin mode)
   //isDrivingStraight = true;
   // Task loop here
   while(1) {
    //isDrivingStraight = true;
            //SimpleGyroNavigation(); 
        //int16_t currentHeading = GetDegrees();
        //Serial.println(currentHeading);
    //vTaskDelay(100 / portTICK_PERIOD_MS); // Schedule to run every 250ms
    vTaskDelay(1500 / portTICK_PERIOD_MS); // Schedule to run every 250ms
   }
    /*isDrivingStraight = false;
    isTurning90Degrees = true;
    while(isTurning90Degrees) {
      vTaskDelay(250 / portTICK_PERIOD_MS);
      }
   } */
}


