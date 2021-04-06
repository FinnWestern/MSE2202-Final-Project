//#define OUTPUT_ON                          // uncomment to turn on output debugging information

#define servoPin 15                    // select the digital pin used for RC servo motor
#define servoChannel 6                // select the channel used for the RC servo motor

const long minDutyCycle = 1375;            // duty cycle for 0 degrees
const long maxDutyCycle = 8100;            // duty cycle for 180 degrees



void setupServo() {
  ledcAttachPin(servoPin, servoChannel);     // Assign servo pin to servo channel
  ledcSetup(servoChannel, 50, 16);           // setup for channel for 50 Hz, 16-bit resolution
}

void setServo(int angle) {
  long dutyCycle = map(angle, 0, 180, minDutyCycle, maxDutyCycle);  // convert to duty cycle
  ledcWrite(servoChannel, dutyCycle);  // set the desired servo position
}
