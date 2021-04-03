/*
  Demonstrates set up and control of a stepper motor using the DRV8825 stepper motor driver

  Assumes use of MSEduino:
  R1 (connect jumper to JP2) controls step rate
  PB1 (connect jumper JP13) to controls direction
  Connect SDA (JP11) to direction pin of DRV8825
  Connect SCLK (JP11) to step pin of DRV8825

  Note that this code does not implement any acceleration or deceleration. The stepper may "lock up" if the
  direction is changed when the motor is spinning quickly. 

  Language: Arduino (ESP32)
  Author: Michael Naish
  Date: 21/02/08

  Rev 1 - Initial verison

*/

//#define OUTPUT_ON                           // uncomment to turn on output debugging information

const int potPin = 32;                        // select the analog pin used for the potentiometer (R1)
const int buttonPin = 27;                     // select digital pin for built-in pushbutton PB1 (JP13 required)

boolean btDir = false;                         // step direction

const long clDebounceDelay = 50;              // button debounce delay in milliseconds
unsigned long ulLastDebounceTime;             // start time for debounce, in milliseconds
int iButtonState;                             // button state
int iLastButtonState = HIGH;                  // 

const int winchA = 2;
const int winchB = 15;
int winchSpeed = 0;

void setup() {
  
  //setup PWM for motors
  ledcAttachPin(winchA, 1); // assign Motors pins to channels
  ledcAttachPin(winchB, 2);

  // Initialize channels 
  // channels 0-15, resolution 1-16 bits, freq limits depend on resolution
  // ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);
  ledcSetup(1, 20000, 8); // 20mS PWM, 8-bit resolution
  ledcSetup(2, 20000, 8);
  winchSpeed = 0;
  
#ifdef OUTPUT_ON
  Serial.begin(9600);
#endif
  
  pinMode(buttonPin, INPUT_PULLUP);            // Assign digital input for pushbutton and turn on internal pullup
  pinMode(potPin, INPUT);
}

void loop() {
   int iButtonValue = digitalRead(buttonPin);  // read value of push button 1
   if (iButtonValue != iLastButtonState) {     // if value has changed
      ulLastDebounceTime = millis();           // reset the debouncing timer
   }

  if ((millis() - ulLastDebounceTime) > clDebounceDelay) {
     if (iButtonValue != iButtonState) {       // if the button state has changed
     iButtonState = iButtonValue;              // update current button state

      // only toggle direction if the new button state is LOW
      if (iButtonState == LOW) {
        btDir = !btDir;
      }
    }
  }
  iLastButtonState = iButtonValue;             // store button state    

  winchSpeed = map(analogRead(potPin), 0, 4096, 130, 255);

  if(btDir)
  {
    ledcWrite(2,0);
    ledcWrite(1,winchSpeed);
    Serial.println("Forwards");
  }
  else
  {
    ledcWrite(2,winchSpeed);
    ledcWrite(1,0);
  }

#ifdef OUTPUT_ON
    Serial.print("Dir: ");
    Serial.print(btDir);
    Serial.print(", rate: ");
    Serial.print(stepRate);
    Serial.print(", count: ");
    Serial.println(stepCount);
#endif
  }  
