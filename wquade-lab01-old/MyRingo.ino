

// Hardware begin from Ringo.
// Still needs stuff to inititalize the gyro.
// Is reentrant (disables interrupts, fine since in setup)
void MyHardwareBegin() {
  cli();
  // Motors begin
  pinMode(MotorDirection_Left, OUTPUT);
  pinMode(MotorDirection_Right, OUTPUT);
  analogWrite(MotorDrive_Left,0);
  analogWrite(MotorDrive_Right,0); 

  // Todo: can nuke stuff I don't need like the IR stuff
  pinMode(Edge_Lights,OUTPUT);
  digitalWrite(Edge_Lights,0);
  pinMode(Source_Select,OUTPUT);
  digitalWrite(Source_Select,0);
  
  pinMode(_38kHz_Rx,INPUT_PULLUP);
  pinMode(IR_Send,OUTPUT);
  digitalWrite(IR_Send,0);
  pinMode(IR_Enable_Front,OUTPUT);
  digitalWrite(IR_Enable_Front,1);
  pinMode(IR_Enable_RearLeft,OUTPUT);
  digitalWrite(IR_Enable_RearLeft,1);
  pinMode(IR_Enable_RearRight,OUTPUT);
  digitalWrite(IR_Enable_RearRight,1);

  // Use lights
  pinMode(Light_Bus_BTN1,OUTPUT);
  digitalWrite(Light_Bus_BTN1, LOW);  

  sei();
}




