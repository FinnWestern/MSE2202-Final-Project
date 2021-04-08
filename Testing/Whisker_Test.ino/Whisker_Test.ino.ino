const int rightSwitch = 26;
const int leftSwitch = 16;

void setup() {
  // put your setup code here, to run once:
  pinMode(rightSwitch, INPUT_PULLUP);
  pinMode(leftSwitch, INPUT_PULLUP);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (!digitalRead(rightSwitch) && !digitalRead(leftSwitch)) {
    Serial.println("Both");
  }
  else if (!digitalRead(rightSwitch)) {
    Serial.println("Right");
  }
  else if (!digitalRead(leftSwitch)) {
    Serial.println("Left");
  }
}
