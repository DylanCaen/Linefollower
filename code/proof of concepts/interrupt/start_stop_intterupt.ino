int ledPin = 8;
int interruptPin = 2;
int state = LOW;
long lastInterrupt;

void setup() {
  lastInterrupt = millis();
  pinMode(ledPin, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, CHANGE);
}

void loop() {
  digitalWrite(ledPin, state);
}

void blink() {
  if (millis() - lastInterrupt > 200)
  {
     state = !state;
    lastInterrupt = millis();
  }
}
