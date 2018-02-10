#include <Arduino_FreeRTOS.h>
#include "RingoHardware.h"

// Define tasks
void TaskSenseGyroscope(void *pvParamters); // Periodic sensor task

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
    TaskSenseGyroscope, // task function
    (const portCHAR *)"SenseGyroscope", // task name string
    100, // stack size
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
double CalculatePID(double setPoint, double processVariable, struct PID &pid) {
  // Error
  double error = setPoint - processVariable;

  // Proportional
  double proportionalOutput = pid.kp * error;

  // Integral
  pid.integral += error * pid.dt;
  double integralOutput = pid.ki * pid.integral;

  // Derivative
  double derivative = (error - pid.error) / pid.dt;
  double derivativeOutput = pid.kd * derivative;

  // Output
  double output = proportionalOutput + integralOutput + derivativeOutput;  

  // Limit to minimum and maximum
  if(output > pid.maximum) {
    output = pid.maximum;
  } else if(output < pid.minimum) {
    output = pid.minimum;
  }

  // Save error for derivative
  pid.error = error;

  return output;
}

// Sense gyroscope task code
void TaskSenseGyroscope(void *pvParameters) {
  (void) pvParameters;
  /*
   * Periodic sensor task
   * It will check the gyroscope position at a regular interval, and update a global variable with the position
   */

   // Task setup here (like set a pin mode)
    NavigationBegin();//initialize and start navigation
    RestartTimer();  
   // Task loop here
   while(1) {
    SimpleGyroNavigation(); 
    //gyroDirection =  PresentHeading();
    //Serial.println(gyroDirection);
    vTaskDelay(250 / portTICK_PERIOD_MS); // Schedule to run every 250ms
   }
}


