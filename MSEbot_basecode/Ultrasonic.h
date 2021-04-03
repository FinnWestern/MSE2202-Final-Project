//functions specific for climber program

int checkDistance(int trig, int echo){	//check distance read by ultrasonic sensor in cm
  long duration; // variable for the duration of sound wave travel
  int distance; // variable for the distance measurement
  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)

  return distance;    //distance in cm
}

//may need to put interrupts in here and put winch functions in motion.h
