#define echoPin 21 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 22 //attach pin D3 Arduino to pin Trig of HC-SR04

// defines variables
long us_Duration; // variable for the duration of sound wave travel in microseconds
unsigned long us_SendTime;
boolean us_Received = false;
double us_Distance = 0; // variable for the distance measurement

void finishPulse(){
  us_Duration = micros() - us_SendTime;
  us_Received = true;
  us_Distance = ((double)us_Duration / 29 / 2) - 8.24;
}

void setupUltrasonic() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void getDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    us_SendTime = micros();
}

//Have to subtract the time it takes for the 8, 40kHz pulses to be sent (takes 8.24cm)
//Sensor accurate within 0.42cm

