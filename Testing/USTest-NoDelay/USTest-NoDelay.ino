// ---------------------------------------------------------------- //
// Arduino Ultrasoninc Sensor HC-SR04
// Re-writed by Arbi Abdul Jabbaar
// Using Arduino IDE 1.8.7
// Using HC-SR04 Module
// Tested on 17 September 2019
// ---------------------------------------------------------------- //

#define echoPin 23 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 25 //attach pin D3 Arduino to pin Trig of HC-SR04

// defines variables
long duration; // variable for the duration of sound wave travel in microseconds
unsigned long sendTime;
unsigned long receiveTime;
unsigned long lastCheckTime = 0;
unsigned long checkTime = 500;    //how often to check distance
boolean received = false;
int distance; // variable for the distance measurement

void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
  Serial.println("Ultrasonic Sensor HC-SR04 Test"); // print some text in Serial Monitor
  Serial.println("with Arduino UNO R3");

  attachInterrupt(echoPin, receivePulse, RISING);
}
void loop() {
  // Clears the trigPin condition
  if(millis() - lastCheckTime >= checkTime){
    lastCheckTime = millis();
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
  
    sendTime = micros();
    // Reads the echoPin, returns the sound wave travel time in microseconds
    // Calculating the distance
    if(received){
      received = false;
      distance = duration / 29 / 2; // Speed of sound wave divided by 2 (go and back)
      // Displays the distance on the Serial Monitor
      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println(" cm");
  }
  }
}

void receivePulse(){
  receiveTime = micros();
  received = true;
}

