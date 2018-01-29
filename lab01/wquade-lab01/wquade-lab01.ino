#include <Arduino_FreeRTOS.h>
#include "RingoHardware.h"

/* 
 *  CSCE 496 lab 2:
 *  
 *  Need tasks to show four aspects: visual, auditory, motor, and sensor.
 *  Need two periodic tasks, -> Task that is executed at regular interval.
 *  one aperiodic task, -> Irregular task, no upper bound, like related to event or specific situation.
 *  and one sporadic task. -> Irregular intervals, but with upper bound, like user invoked task (button press)
 *  
 *  Visual -> Lights
 *  Auditory -> Beep
 *  Motor -> Move
 *  Sensor -> Gyroscope
 *  
 *  Sporadic -> Button press triggers beep, bounded by the maximum press rate a human can do. (auditory)
 *  Periodic -> Motors change direction at regular interval. (motor)
 *  Periodic -> Sense gyroscope at a regular interval (sensor)
 *  Aperodic -> When gyroscope is within a certain range change LED color, sort of bounded, but also sort of not (visual)
 *              It is associated with specific event. The robot could get stuck and it could run continuously (unbounded)
 */

int gyroDirection = 0;
uint8_t redValue = 0;
uint8_t greenValue = 0;
uint8_t blueValue = 0;

// Define tasks
void TaskBeep(void *pvParameters); // Sporadic auditory task, when the button is pressed
void TaskChangeDirection(void *pvParameters); // Periodic motor task
void TaskSenseGyroscope(void *pvParamters); // Periodic sensor task
void TaskChangeLed(void *pvParameters); // Aperodic visual task

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
    TaskChangeDirection, // task function
    (const portCHAR *)"ChangeDirection", // task name string
    100, // stack size
    NULL, // nothing
    3, // Priority, 0 is lowest
    NULL); // nothing
      
  xTaskCreate(
    TaskSenseGyroscope, // task function
    (const portCHAR *)"SenseGyroscope", // task name string
    100, // stack size
    NULL, // nothing
    2, // Priority, 0 is lowest
    NULL); // nothing 
    
  xTaskCreate(
    TaskBeep, // task function
    (const portCHAR *)"Beep", // task name string
    100, // stack size
    NULL, // nothing
    1, // Priority, 0 is lowest
    NULL); // nothing 
    
  xTaskCreate(
    TaskChangeLed, // task function
    (const portCHAR *)"ChangeLed", // task name string
    100, // stack size
    NULL, // nothing
    0, // Priority, 0 is lowest, aperodic, so it needs to be lowest priority so it doesn't starve others
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

// Beep task code
void TaskBeep(void *pvParameters) {
  (void) pvParameters;
  /*
   * Sporadic auditory task, when the button is pressed
   * It just yields while waiting for it to detect that the button was pressed
   */

   // Task setup here (like set a pin mode)
   
   // Task loop here
   while(1) {
    // The button and the LED's using the same pins really complicates things.
    // I ended up storing and resstoring the RGB values so the lights appear to still be working.
    SwitchPixelsToButton();
    volatile int buttonStatus = digitalRead(7);
    SetPixelRGB( 4, redValue, greenValue, blueValue);
    SetPixelRGB( 5, redValue, greenValue, blueValue);    
    SwitchButtonToPixels();
    
    if(buttonStatus == 0) {
      PlayChirp(1000, 20); // Make a sound 
      vTaskDelay(500 / portTICK_PERIOD_MS); // Give it half a second to play
      OffChirp(); // Turn off the sound  
    }

    vTaskDelay(250 / portTICK_PERIOD_MS); // Upper bound since it's sporadic, if this isn't here the lights never change
   }
}

// Change direction task code
void TaskChangeDirection(void *pvParameters) {
  (void) pvParameters;
  /*
   * Periodic motor task
   * It will change motor direction every second
   */

   // Task setup here (like set a pin mode)

   // Task loop here
   while(1) {
    Motors(32, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait a second
    Motors(0, 42);
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait a second
   }
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
    gyroDirection =  PresentHeading();
    //Serial.println(gyroDirection);
    vTaskDelay(250 / portTICK_PERIOD_MS); // Schedule to run every 250ms
   }
}

// Change LED task code
void TaskChangeLed(void *pvParameters) {
  (void) pvParameters;
  /*
   * Aperodic visual task
   * It will change the LED color based on the gyroscope value. If the device moves fast it runs very often, otherwise not as much
   */

   // Task setup here (like set a pin mode)

   // Task loop here
   while(1) {
    if(gyroDirection >= 0) {
      redValue = 0;
      greenValue = 220;
      blueValue = 0;
      SetPixelRGB( 4, redValue, greenValue, blueValue);
      SetPixelRGB( 5, redValue, greenValue, blueValue);
    } else {
      redValue = 220;
      greenValue = 0;
      blueValue = 0;      
      SetPixelRGB( 4, redValue, greenValue, blueValue);
      SetPixelRGB( 5, redValue, greenValue, blueValue);
    }
   }
}


