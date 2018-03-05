// I broke the PID controller function into another file since the main file was getting a little big.

// Function to calculate PID output given set point, current value (process variable), and PID controller parameters.
// Based on this: https://gist.github.com/bradley219/5373998
int8_t CalculatePID(int16_t setPoint, int16_t processVariable, struct PID *pid) {
  // Error
  int16_t error = setPoint - processVariable;

  // Proportional
  int16_t proportionalOutput = pid->kp * error;

  // Integral
  pid->integral += error * pid->dt;
  int16_t integralOutput = pid->ki * pid->integral;

  // Derivative
  int16_t derivative = (error - pid->error) / pid->dt;
  int16_t derivativeOutput = pid->kd * derivative;

  // Output
  int16_t output = proportionalOutput + integralOutput + derivativeOutput;  

  // Limit to minimum and maximum
  if(output > pid->maximum) {
    output = pid->maximum;
  } else if(output < pid->minimum) {
    output = pid->minimum;
  }

  // Save error for derivative
  pid->error = error;

  return (int8_t)output;
}

