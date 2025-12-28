// === LM358 IR Sensor (Analog + Potentiometer + Indicator LED) ===

int sensorPin = A0;     // LM358 output pin
int ledPin = 13;        // Built-in LED or external LED
int value = 0;

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  Serial.println("LM358 Analog IR Sensor with Potentiometer");
}

void loop() {
  value = analogRead(sensorPin);   // Read analog value

  Serial.print("Analog Value: ");
  Serial.print(value);

  if (value > 700) {
    Serial.println("  -> Object very close");
    digitalWrite(ledPin, HIGH);  // Turn on LED
  } else if (value > 400) {
    Serial.println("  -> Object nearby");
    digitalWrite(ledPin, HIGH);  // Weak reflection
  } else {
    Serial.println("  -> No object");
    digitalWrite(ledPin, LOW);   // Turn off LED
  }

  delay(200);
}