
//MSE 2202
//Western Engineering base code
//2020 05 13 E J Porter


/*
  esp32                                           MSE-DuinoV2
  pins         description                        Brd Jumpers /Labels                                                                  User (Fill in chart with user PIN usage)
  1             3v3                               PWR 3V3                                                                              3V3
  2             gnd                               GND                                                                                  GND
  3             GPIO15/AD2_3/T3/SD_CMD/           D15 (has connections in both 5V and 3V areas)
  4             GPIO2/AD2_2/T2/SD_D0              D2(has connections in both 5V and 3V areas)  /INDICATORLED ( On ESP32 board )        Heartbeat LED
  5             GPIO4/AD2_0/T0/SD_D1              D4(has connections in both 5V and 3V areas)                                          Left Motor, Channel A
  6             GPIO16/RX2                        Slide Switch S1b                                                                     IR Receiver
  7             GPIO17/TX2                        Slide Switch S2b                                                                     Left Encoder, Channel A
  8             GPIO5                             D5 (has connections in both 5V and 3V areas)                                         Left Encoder, Channel B
  9             GPIO18                            D18 (has connections in both 5V and 3V areas)                                        Left Motor, Channel B
  10            GPIO19/CTS0                       D19 (has connections in both 5V and 3V areas)                                        Right Motor, Channel A
  11            GPIO21                            D21/I2C_SDA
  12            GPIO3/RX0                         RX0
  13            GPIO1//TX0                        TX0
  14            GPIO22/RTS1                       D22/I2C_CLK
  15            GPIO23                            D23 (has connections in both 5V and 3V areas)
  16            EN                                JP4 (Labeled - RST) for reseting ESP32
  17            GPI36/VP/AD1_0                    AD0
  18            GPI39/VN/AD1_3/                   AD3
  19            GPI34/AD1_6/                      AD6
  20            GPI35/AD1_7                       Potentiometer R2 / AD7
  21            GPIO32/AD1_4/T9                   Potentiometer R1 / AD4                                                               Pot 1 (R1)
  22            GPIO33/AD1_5/T8                   IMon/D33  monitor board current
  23            GPIO25/AD2_8/DAC1                 SK6812 Smart LEDs / D25                                                              Smart LEDs
  24            GPIO26/A2_9/DAC2                  Push Button PB2                                                                      Limit switch
  25            GPIO27/AD2_7/T7                   Push Button PB1                                                                      PB1
  26            GPOP14/AD2_6/T6/SD_CLK            Slide Switch S2a                                                                     Right Encoder, Channel A
  27            GPIO12/AD2_5/T5/SD_D2/            D12(has connections in both 5V and 3V areas)                                         Right Motor, Channel B
  28            GPIO13/AD2_4/T4/SD_D3/            Slide Switch S1a                                                                     Right Encoder, Channel B
  29            GND                               GND                                                                                  GND
  30            VIN                               PWR 5V t 7V                                                                          PWR 5V to 7V
*/


//Pin assignments
const int ciPB1 = 27;
const int ciPot1 = A4;    //GPIO 32  - when JP2 has jumper installed Analog pin AD4 is connected to Poteniometer R1
const int rightSwitch = 26;
const int leftSwitch = 16;
//const int ciIRDetector = 16;
const int ciMotorLeftA = 4;
const int ciMotorLeftB = 18;
const int ciMotorRightA = 19;
const int ciMotorRightB = 12;
const int ciEncoderLeftA = 17;
const int ciEncoderLeftB = 5;
const int ciEncoderRightA = 14;
const int ciEncoderRightB = 13;
//const int ciSmartLED = 25;

volatile uint32_t vui32test1;
volatile uint32_t vui32test2;

#include "0_Core_Zero.h"

#include <esp_task_wdt.h>

#include <Adafruit_NeoPixel.h>
#include <Math.h>
#include "Motion.h";
#include "MyWEBserver.h"
#include "BreakPoint.h"
#include "WDT.h";
#include "Ultrasonic.h"
#include "Servo.h"

#define motorStartIndex 0      //choose which step to start on
#define ropeDistance 19.5      //distance from box to rope in cm____________________________________

void loopWEBServerButtonresponce(void);

const int CR1_ciMainTimer =  1000;
const int CR1_ciMotorRunTime = 1000;
const int CR1_ciMotorPauseTime = 1000;
const long CR1_clDebounceDelay = 50;
const long CR1_clReadTimeout = 220;

const uint8_t ci8RightTurn = 23;            //turn ticks_____________________________
const uint8_t ci8LeftTurn = 26;

unsigned char CR1_ucMainTimerCaseCore1;
uint8_t CR1_ui8LimitSwitch;

uint8_t CR1_ui8IRDatum;
uint8_t CR1_ui8WheelSpeed;
uint8_t CR1_ui8LeftWheelSpeed;
uint8_t CR1_ui8RightWheelSpeed;
float bias;
int int_rightWheelSpeed;
int int_leftWheelSpeed;

uint32_t CR1_u32Now;
uint32_t CR1_u32Last;
uint32_t CR1_u32Temp;
uint32_t CR1_u32Avg;

unsigned long CR1_ulLastDebounceTime;
unsigned long CR1_ulLastByteTime;

unsigned long CR1_ulMainTimerPrevious;
unsigned long CR1_ulMainTimerNow;

unsigned long CR1_ulMotorTimerPrevious;
unsigned long CR1_ulMotorTimerNow;
unsigned char ucMotorStateIndex = motorStartIndex;

uint8_t winchState = 0;
uint8_t winchSpeed = 230;

boolean btRun = false;
boolean btToggle = true;
boolean adjustSpeed = false;    //key to if statement which averages speed to kep robot straight
boolean doorCheck = false;      //key to checking distance from door with ultrasonic
boolean whiskerCheck = false;   //key to checking when robot makes contact with door with whisker switches
boolean checkRopeCatch = false;
boolean boxEdge = false;
boolean checkAscent = false;
boolean checkDescent = false;

boolean checkDistance = false;    //key to check distance
unsigned long distanceCheckTime = 100;    //time in between distance checks
unsigned long lastDistanceCheckTime = 0;

unsigned long servoTime = 3000;
unsigned long startServoTime = 0;

boolean wiggle = false;
boolean reachedWiggle = false;
unsigned long noLatchTime = 6000;   //time to wait before backing up to grab rope_________________________
unsigned long winchStartTime = 0;
unsigned long hangTime = 3000;
unsigned long startHang = 0;

int iButtonState;
int iLastButtonState = HIGH;

// Declare our SK6812 SMART LED object:
//Adafruit_NeoPixel SmartLEDs(2, 25, NEO_GRB + NEO_KHZ400);
// Argument 1 = Number of LEDs (pixels) in use
// Argument 2 = ESP32 pin number
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

void setup() {
  Serial.begin(115200);
  //Serial2.begin(2400, SERIAL_8N1, ciIRDetector);  // IRDetector on RX2 receiving 8-bit words at 2400 baud

  Core_ZEROInit();

  WDT_EnableFastWatchDogCore1();
  WDT_ResetCore1();
  WDT_vfFastWDTWarningCore1[0] = 0;
  WDT_vfFastWDTWarningCore1[1] = 0;
  WDT_vfFastWDTWarningCore1[2] = 0;
  WDT_vfFastWDTWarningCore1[3] = 0;
  WDT_ResetCore1();
  WDT_vfFastWDTWarningCore1[4] = 0;
  WDT_vfFastWDTWarningCore1[5] = 0;
  WDT_vfFastWDTWarningCore1[6] = 0;
  WDT_vfFastWDTWarningCore1[7] = 0;
  WDT_ResetCore1();
  WDT_vfFastWDTWarningCore1[8] = 0;
  WDT_vfFastWDTWarningCore1[9] = 0;
  WDT_ResetCore1();

  setupMotion();
  setupUltrasonic();
  setupServo();
  attachInterrupt(echoPin, finishPulse, FALLING);
  pinMode(ciPB1, INPUT_PULLUP);
  pinMode(rightSwitch, INPUT_PULLUP);
  pinMode(leftSwitch, INPUT_PULLUP);
  /*
     SmartLEDs.begin();                          // Initialize Smart LEDs object (required)
     SmartLEDs.clear();                          // Set all pixel colours to off
     SmartLEDs.show();                           // Send the updated pixel colours to the hardware
  */
}

void loop()
{
  //WSVR_BreakPoint(1);

  //average the encoder tick times
  ENC_Averaging();

  int iButtonValue = digitalRead(ciPB1);       // read value of push button 1
  if (iButtonValue != iLastButtonState) {      // if value has changed
    CR1_ulLastDebounceTime = millis();        // reset the debouncing timer
  }

  if ((millis() - CR1_ulLastDebounceTime) > CR1_clDebounceDelay) {
    if (iButtonValue != iButtonState) {        // if the button state has changed
      iButtonState = iButtonValue;               // update current button state

      // only toggle the run condition if the new button state is LOW
      if (iButtonState == LOW)
      {
        ENC_ClearLeftOdometer();
        ENC_ClearRightOdometer();
        btRun = !btRun;
        Serial.println(btRun);
        // if stopping, reset motor states and stop motors
        if (!btRun)
        {
          setServo(90);
          ENC_stopMotors();
          ucMotorStateIndex = motorStartIndex;
          ucMotorState = 0;
          winchState = 0;
          move(0);
        }

      }
    }
  }
  iLastButtonState = iButtonValue;             // store button state
  /*
    if(!digitalRead(rightSwitch))
    {
    btRun = 0; //if limit switch is pressed stop bot
    ucMotorStateIndex = motorStartIndex;
    ucMotorState = 0;
    move(0);
    }
  
  if (Serial2.available() > 0) {               // check for incoming data
    CR1_ui8IRDatum = Serial2.read();          // read the incoming byte
    // Serial.println(iIncomingByte, HEX);        // uncomment to output received character
    CR1_ulLastByteTime = millis();            // capture time last byte was received
  }
  else
  {
    // check to see if elapsed time exceeds allowable timeout
    if (millis() - CR1_ulLastByteTime > CR1_clReadTimeout) {
      CR1_ui8IRDatum = 0;                     // if so, clear incoming byte
    }
  }
*/
  if(checkDistance) {
    if (millis() - lastDistanceCheckTime >= distanceCheckTime) {
      lastDistanceCheckTime = millis();
      getDistance();
      Serial.println(us_Distance);
    }
  }

  if(boxEdge){    //in main loop for better sensitivity
    if(us_Distance > 20){
      boxEdge = false;
      ENC_stopMotors();
      move(0);
      ucMotorState = 0;
      ucMotorStateIndex++;
    }
  }

  CR1_ulMainTimerNow = micros();
  if (CR1_ulMainTimerNow - CR1_ulMainTimerPrevious >= CR1_ciMainTimer)
  {
    WDT_ResetCore1();
    WDT_ucCaseIndexCore0 = CR0_ucMainTimerCaseCore0;

    CR1_ulMainTimerPrevious = CR1_ulMainTimerNow;

    switch (CR1_ucMainTimerCaseCore1) //full switch run through is 1mS
    {
      //###############################################################################
      case 0:
        {

          if (btRun)
          {
            if (!ENC_ISMotorRunning()) {
              CR1_ulMotorTimerNow = millis();
              if (CR1_ulMotorTimerNow - CR1_ulMotorTimerPrevious >= CR1_ciMotorPauseTime)
              {
                switch (ucMotorStateIndex)
                {
                  case 0:
                    {
                      setServo(90);
                      move(0);
                      adjustSpeed = true;
                      checkDistance = true;
                      us_Distance = 0;
                      boxEdge = true;
                      ucMotorState = 1;   //forward
                      ENC_runMotors();

                      break;
                    }
                  case 1:
                    {
                      move(0);
                      checkDistance = false;
                      ENC_SetDistance(40, 40);
                      ucMotorState = 1;   //forward
                      ucMotorStateIndex++;

                      break;
                    }
                  case 2:
                    {
                      move(0);
                      ENC_SetDistance(-(ci8LeftTurn), ci8LeftTurn);
                      ucMotorState = 2;  //left
                      ucMotorStateIndex++;
                      break;
                    }
                  case 3:
                    {
                      move(0);
                      ENC_SetDistance(50, 50);
                      ucMotorState = 1;  //forward
                      ucMotorStateIndex++;
                      break;
                    }
                  case 4:
                    {
                      move(0);
                      boxEdge = true;
                      checkDistance = true;
                      us_Distance = 0;
                      ucMotorState = 1;   //forward
                      ENC_runMotors();

                      break;
                    }
                  case 5:
                    {
                      move(0);
                      ENC_SetDistance(3*ropeDistance, 3*ropeDistance);
                      checkDistance = false;
                      ucMotorState = 1;   //forward
                      ucMotorStateIndex++;

                      break;
                    }
                  case 6:
                    {
                      move(0);
                      ENC_SetDistance(-(ci8LeftTurn), ci8LeftTurn);
                      ucMotorState = 2;  //left
                      ucMotorStateIndex++;

                      break;
                    }
                  case 7:
                    {
                      move(0);
                      setServo(0);
                      checkDistance = true;
                      ucMotorState = 1;  //forward
                      ENC_runMotors();
                      doorCheck = true; 

                      break;
                    }
                  case 8:
                    {
                      move(0);    //rotate servo, start winch, square with door
                      setServo(0);
                      us_Distance = 0;
                      checkDistance = true;
                      checkRopeCatch = true;
                      winchState = 1;
                      ucMotorState = 0;
                      ENC_runMotors();
                      wiggle = true;
                      winchStartTime = millis();

                      break;
                    }
                  case 9:
                    {
                      move(0);    
                      checkAscent = true;
                      wiggle = false;
                      setServo(180);
                      startServoTime = millis();
                      winchState = 1;
                      ENC_runMotors();
                      ucMotorState = 0;
                      
                      break;
                    }
                  case 10:
                    {
                      move(0);  
                      checkDistance = true; 
                      setServo(180);
                      ucMotorState = 0;
                      winchState = 0;
                      ENC_runMotors();
                      checkDescent = true;
                      startHang = millis();

                      break;
                    }
                  case 11:
                    {
                      move(0);   
                      ucMotorState = 0;
                      winchState = 0;
                      checkDistance = false;

                      break;
                    }
                }
              } else {
                ucMotorState = 0;
                move(0);
              }
            } else {
              CR1_ulMotorTimerPrevious = millis();
            }

            if(checkDescent){
              if(millis() - startHang >= hangTime){
                winchState = 2;
                if(us_Distance < 20){
                  checkDescent = false;
                  winchState = 0;
                  ucMotorStateIndex++;
                  ENC_stopMotors();
                }
              }
            }

            if(wiggle){
              if(millis() - winchStartTime >= noLatchTime){
                if(us_Distance < 20){
                  CR1_ui8WheelSpeed = 130;
                  ucMotorState = 4;
                }else{
                  wiggle = false;
                  move(0);
                  ucMotorState = 0;           
                }
              }
            }

            if(checkAscent){                                                  // checks height when winching
              if(us_Distance > 60 && (millis() - startServoTime) >= servoTime){
                checkAscent = false;
                ENC_stopMotors();
                ucMotorStateIndex++;
              }
            }

            if(checkRopeCatch){   //move winch servo
              if(us_Distance > 50){
                checkRopeCatch = false;
                ENC_stopMotors();
                ucMotorStateIndex++;
                wiggle = false;
              }
            }

            if (doorCheck) {
              if (us_Distance < 15) {
                whiskerCheck = true;    //go to check with whiskers
                doorCheck = false;
              }
            }
            
            if (whiskerCheck) {
              if (!digitalRead(rightSwitch) && !digitalRead(leftSwitch)) {
                move(0);
                ucMotorState = 0;
                ENC_stopMotors();
                ucMotorStateIndex++;
                whiskerCheck = false;
              }
              else if (!digitalRead(rightSwitch)) {
                ucMotorState = 6;  //right pivot
              }
              else if (!digitalRead(leftSwitch)) {
                ucMotorState = 5;  //left pivot
              }
            }


            //adjust speed to remain straight
            if (adjustSpeed) {
              bias = ENC_SpeedBias();
              //Serial.print(bias);
              //Serial.print(" Speed: ");
              //Serial.print(CR1_ui8WheelSpeed);
              int_leftWheelSpeed = CR1_ui8WheelSpeed * bias; //average speeds
              int_rightWheelSpeed = CR1_ui8WheelSpeed / bias;
              //Serial.print(" Right before: ");
              //Serial.print(CR1_ui8RightWheelSpeed);
              //Serial.print(" Left before: ");
              //Serial.print(CR1_ui8LeftWheelSpeed);

              if (int_rightWheelSpeed > 255) {
                int_rightWheelSpeed = 255;
              } else if (int_rightWheelSpeed < 130) {
                int_rightWheelSpeed = 130;
              }
              if (int_leftWheelSpeed > 255) {
                int_leftWheelSpeed = 255;
              } else if (int_leftWheelSpeed < 130) {
                int_leftWheelSpeed = 130;
              }
              CR1_ui8RightWheelSpeed = int_rightWheelSpeed;
              CR1_ui8LeftWheelSpeed = int_leftWheelSpeed;

            } else {
              CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
              CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
            }
            //Serial.print(" Left: ");
            //Serial.print(CR1_ui8LeftWheelSpeed);
            //Serial.print(" Right: ");
            //Serial.println(CR1_ui8RightWheelSpeed);
          }

          CR1_ucMainTimerCaseCore1 = 1;

          break;
        }
      //###############################################################################
      case 1:
        {
          //read pot 1 for motor speeds
          CR1_ui8WheelSpeed = map(analogRead(ciPot1), 0, 4096, 130, 255);  // adjust to range that will produce motion
          

          CR1_ucMainTimerCaseCore1 = 2;
          break;
        }
      //###############################################################################
      case 2:
        {
          // asm volatile("esync; rsr %0,ccount":"=a" (vui32test1)); // @ 240mHz clock each tick is ~4nS

          //   asm volatile("esync; rsr %0,ccount":"=a" (vui32test2)); // @ 240mHz clock each tick is ~4nS

          CR1_ucMainTimerCaseCore1 = 3;
          break;
        }
      //###############################################################################
      case 3:
        {
          //move bot X number of odometer ticks
          if (ENC_ISMotorRunning())
          {
            MoveTo(ucMotorState, CR1_ui8LeftWheelSpeed, CR1_ui8RightWheelSpeed);
          }

          CR1_ucMainTimerCaseCore1 = 4;
          break;
        }
      //###############################################################################
      case 4:
        {
          moveWinch(winchState, winchSpeed);
          CR1_ucMainTimerCaseCore1 = 5;
          break;
        }
      //###############################################################################
      case 5:
        {


          CR1_ucMainTimerCaseCore1 = 6;
          break;
        }
      //###############################################################################
      case 6:
        {


          CR1_ucMainTimerCaseCore1 = 7;
          break;
        }
      //###############################################################################
      case 7:
        { /*
            if (CR1_ui8IRDatum == 0x55) {                // if proper character is seen
              SmartLEDs.setPixelColor(0,0,25,0);         // make LED1 green with 10% intensity
            }
            else if (CR1_ui8IRDatum == 0x41) {           // if "hit" character is seen
              SmartLEDs.setPixelColor(0,25,0,25);        // make LED1 purple with 10% intensity
            }
            else {                                       // otherwise
              SmartLEDs.setPixelColor(0,25,0,0);         // make LED1 red with 10% intensity
            }
            SmartLEDs.show();                            // send updated colour to LEDs
          */
          CR1_ucMainTimerCaseCore1 = 8;
          break;
        }
      //###############################################################################
      case 8:
        {

          CR1_ucMainTimerCaseCore1 = 9;
          break;
        }
      //###############################################################################
      case 9:
        {

          CR1_ucMainTimerCaseCore1 = 0;
          break;
        }

    }
  }

}
