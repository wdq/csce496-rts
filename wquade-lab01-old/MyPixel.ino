

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

Adafruit_NeoPixel myPixels = Adafruit_NeoPixel(NUM_PIXELS, Light_Bus_BTN1, NEO_GRB + NEO_KHZ800);

// Is reentrant, hardware access guarded by semaphore
void MySetPixelRGB(int Pixel, int Red, int Green, int Blue){
  //while(xSemaphoreTake(ledSemaphore, (TickType_t) 10) == pdFALSE) {} // Wait on semaphore
  myPixels.setPixelColor(Pixel, myPixels.Color(Red,Green,Blue));
  //xSemaphoreGive(ledSemaphore); // Release the semaphore 
  
}

// Is reentrant, hardware access guarded by semaphore
void MyRefreshPixels(){
  //while(xSemaphoreTake(ledSemaphore, (TickType_t) 10) == pdFALSE) {} // Wait on semaphore
  myPixels.show();  
  //xSemaphoreGive(ledSemaphore); // Release the semaphore 
}
// ***************************************************
// end Pixels
// ***************************************************
