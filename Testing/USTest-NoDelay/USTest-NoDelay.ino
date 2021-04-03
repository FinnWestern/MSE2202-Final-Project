#define echoPin 2 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 4 //attach pin D3 Arduino to pin Trig of HC-SR04

// defines variables
long duration; // variable for the duration of sound wave travel in microseconds
unsigned long sendTime;
unsigned long lastCheckTime = 0;
unsigned long checkTime = 1000;    //how often to check distance
boolean received = false;
double distance; // variable for the distance measurement

void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
  attachInterrupt(echoPin, receivePulse, FALLING);
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
  }
  if(received){
    received = false;
    distance = ((double)duration / 29 / 2); // Speed of sound wave divided by 2 (go and back)
    // Displays the distance on the Serial Monitor
    Serial.print("Distance: ");
    Serial.print(duration);
    Serial.println(" cm");
  }
}

void receivePulse(){
  duration = micros() - sendTime;
  received = true;
}

