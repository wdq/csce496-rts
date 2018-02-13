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

