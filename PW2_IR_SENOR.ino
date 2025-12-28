// === IR Sensor using LM358 ===
// Reads both Analog and Digital outputs

int analogPin = A0;     // Analog output from U1
int digitalPin = 2;     // Digital output from U2
int digitalState;        // Variable to store digital reading
int analogValue;         // Variable to store analog reading

void setup() {
  Serial.begin(9600);     // Start serial communication
  pinMode(digitalPin, INPUT);  // Digital pin as input
  Serial.println("IR Sensor Test Started");
}

void loop() {
  // Read values from sensor
  digitalState = digitalRead(digitalPin);
  analogValue = analogRead(analogPin);

  // Print both values
  Serial.print("Analog Value: ");
  Serial.print(analogValue);
  Serial.print("\t Digital Value: ");
  Serial.print(digitalState);

  // Interpret detection result
  if (digitalState == HIGH) {
    Serial.println(" --> Object Detected");
  } else {
    Serial.println(" --> No Object");
  }

  delay(300); // Small delay for readability
}
