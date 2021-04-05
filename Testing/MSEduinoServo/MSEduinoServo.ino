/*
  Demonstrates set up and control of an RC servo on the ESP32 using a potentiometer (variable resistor)

  Language: Arduino (ESP32)
  Author: Michael Naish
  Date: 21/01/24

  Rev 1 - Initial verison

*/

#define OUTPUT_ON                          // uncomment to turn on output debugging information

const int potPin = 32;                       // select the analog pin used for the potentiometer
const int servoPin = 2;                     // select the digital pin used for RC servo motor
const int servoChannel = 5;                  // select the channel used for the RC servo motor

int val;                                     // input value from the analog pin
int servoPos;                                // desired servo angle

void setup() {
#ifdef OUTPUT_ON
  Serial.begin(9600);
#endif

  ledcAttachPin(servoPin, servoChannel);     // Assign servo pin to servo channel
  ledcSetup(servoChannel, 50, 16);           // setup for channel for 50 Hz, 16-bit resolution
}

void loop() {
  val = analogRead(potPin);                  // reads the value of the potentiometer (value between 0 and 4096)
  servoPos = map(val, 0, 4096, 0, 180);      // scale it into servo range 0 to 180 degrees
  ledcWrite(servoChannel, degreesToDutyCycle(servoPos));  // set the desired servo position
}

// converts servo position in degrees into the required duty cycle for an RC servo motor control signal assuming 16-bit resolution
long degreesToDutyCycle(int deg) {
  const long minDutyCycle = 1375;            // duty cycle for 0 degrees
  const long maxDutyCycle = 8100;            // duty cycle for 180 degrees

  long dutyCycle = map(deg, 0, 180, minDutyCycle, maxDutyCycle);  // convert to duty cycle

#ifdef OUTPUT_ON
  Serial.print(" Degrees: ");
  Serial.print(servoPos);
  Serial.print(", Duty Cycle Val: ");
  Serial.print(dutyCycle);
  Serial.print(" = ");
  float percent = dutyCycle * 0.0015259;     // dutyCycle / 65535 * 100
  Serial.print(percent);
  Serial.println("%");
#endif

  return dutyCycle;
}
