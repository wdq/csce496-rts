

// Motor control function from Ringo.
// Is reentrant, hardware access guarded by semaphore
void MyMotors(int16_t LeftMotor, int16_t RightMotor) {
  //while(xSemaphoreTake(motorSemaphore, (TickType_t) 10) == pdFALSE) {} // Wait on semaphore
  // Had some logic to bound the speeds to the min/max.
  // I removed this since not needed since I know the bounds.
  // Also got rid of the use of the global motor speed variables.
  if(LeftMotor<0){
    digitalWrite(MotorDirection_Left,0);
  } else {
    digitalWrite(MotorDirection_Left,1);
  }
  if(RightMotor<0){
    digitalWrite(MotorDirection_Right,0);
  } else {
    digitalWrite(MotorDirection_Right,1);
  }
    
  analogWrite(MotorDrive_Left,abs(LeftMotor));
  analogWrite(MotorDrive_Right,abs(RightMotor));
  //xSemaphoreGive(motorSemaphore); // Release the semaphore 
}
