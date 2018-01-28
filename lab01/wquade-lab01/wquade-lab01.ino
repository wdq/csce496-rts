#include <Arduino_FreeRTOS.h>
#include "RingoHardware.h"

// Define tasks
void TaskLab1(void *pvParameters);

// Setup ringo stuff
void ringoSetup() {
  HardwareBegin();        //initialize Ringo's brain to work with his circuitry
  PlayStartChirp();       //Play startup chirp and blink eyes
  SwitchMotorsToSerial(); //Call "SwitchMotorsToSerial()" before using Serial.print functions as motors & serial share a line
  RestartTimer();  
}

// Setup the tasks
void taskSetup() {
  // Create lab 1 task
  xTaskCreate(
    TaskLab1, // task function
    (const portCHAR *)"Lab1", // task name string
    128, // stack size
    NULL, // nothing
    2, // Priority, 0 is lowest
    NULL); // nothing 
}

// First run code
void setup(){
  Serial.begin(9600); // For debugging
  ringoSetup(); // Setup ringo stuff
  taskSetup(); // Setup the tasks

}

// Don't do anything here since the tasks do the work
void loop(){}

// Lab 1 task code
void TaskLab1(void *pvParameters) {
  (void) pvParameters;
  /*
   * A test task for lab 1.
   * It just makes a sound while turning on its lights, and repeats it every 5 seconds.
   */

   // Task setup here (like set a pin mode)

   // Task loop here
   while(1) {
    PlayChirp(1000, 25); // Make a sound
    RandomEyes(); // Turn on some lights
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait a bit (1s)
    OffChirp(); // Turn off the sound
    OffEyes(); // Turn off the lights
    vTaskDelay(5000 / portTICK_PERIOD_MS); // Wait a bit longer (5s)
   }
}



