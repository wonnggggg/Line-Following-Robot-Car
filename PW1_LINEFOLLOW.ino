#include <LiquidCrystal.h>

// LCD pins
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// === IR SENSOR PINS ===
int analogLeft  = A1;
int analogRight  = A2;

// Low speed settings
const int MOTOR_DELAY = 15; // Higher value = slower speed
unsigned long lastMotorTime = 0;
bool motorToggle = false;

// Threshold (LOW = black)
int thresholdVal = 650;

// === MOTOR DRIVER PINS ===
const int ENA = 11;
const int IN1 = 13;
const int IN2 = 2;
const int ENB = 3;
const int IN3 = 12;
const int IN4 = 1;

// Timer
unsigned long startTime = 0;

void setup() {
  lcd.begin(16, 2);
  lcd.print("Line Follower");
  delay(1500);
  lcd.clear();

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  startTime = millis();
}

void loop() {
  // Read sensors
  int aLeft  = analogRead(analogLeft);
  int aRight = analogRead(analogRight);

  // BLACK (low value) detection
  bool leftWhite  = (aLeft  > thresholdVal);
  bool rightWhite = (aRight > thresholdVal);

  unsigned long currentTime = (millis() - startTime) / 1000;

  // Simple speed reduction by toggling motors
  if (millis() - lastMotorTime > MOTOR_DELAY) {
    lastMotorTime = millis();
    motorToggle = !motorToggle;
  }

  // === LINE FOLLOWING LOGIC ===
  if (leftWhite && rightWhite) {
    moveForward(motorToggle);
    lcdStatus(currentTime);
  }
  else if (leftWhite && !rightWhite) {
    stop();
    turnRight(true); // Full speed for turns
    lcdStatus(currentTime);
  }
  else if (!leftWhite && rightWhite) {
    stop();
    turnLeft(true); // Full speed for turns
    lcdStatus(currentTime);
  }
  else {
    stop();
    lcdStatus(currentTime);
  }
}

void lcdStatus(int timeSec) {
  lcd.setCursor(0, 1);
  lcd.print("Time:");
  lcd.print(timeSec);
  lcd.print(" sec   ");
}

// === MOTOR FUNCTIONS ===
void moveForward(bool enable) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA,130);
  analogWrite(ENB, 130);
}

void turnLeft(bool enable) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 230);
  analogWrite(ENA, -130);
}

void turnRight(bool enable) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 230);
  analogWrite(ENB, -130);
}

void stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
}
