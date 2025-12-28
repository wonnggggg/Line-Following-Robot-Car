#include <LiquidCrystal.h>

// LCD pins
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// === IR SENSOR PINS ===
int analogLeft  = A1;
int analogRight  = A2;

// === ENCODER PINS ===
const int encoderRightPin = A3;
const int encoderLeftPin = A4;

// === MOTOR DRIVER PINS ===
const int ENA = 11;
const int IN1 = 13;
const int IN2 = 2;
const int ENB = 3;
const int IN3 = 12;
const int IN4 = 1;

// Motor speed settings
const int MOTOR_DELAY = 15;
unsigned long lastMotorTime = 0;
bool motorToggle = false;

// Threshold (LOW = black)
int thresholdVal = 650;

// === ENCODER VARIABLES ===
volatile unsigned long rightCount = 0;
volatile unsigned long leftCount = 0;
#define CM_PER_COUNT 0.5  // (pi*66.1/10)/20

// Timer and state variables
unsigned long startTime = 0;
int stopCounter = 0;

// === INTERRUPT SETUP FOR ENCODERS ===
ISR(PCINT1_vect) {
  uint8_t portState = PINC;

  // Check A3 (PC3) for right encoder
  int rightState = (portState >> 3) & 1;
  // Check A4 (PC4) for left encoder
  int leftState = (portState >> 4) & 1;

  static int lastRightState = 0;
  static int lastLeftState = 0;

  if (rightState != lastRightState) {
    rightCount++;
  }
  if (leftState != lastLeftState) {
    leftCount++;
  }

  lastRightState = rightState;
  lastLeftState = leftState;
}

void setup() {
  lcd.begin(16, 2);
  lcd.print("Line Follower");
  delay(1500);
  lcd.clear();

  // Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Sensor pins
  pinMode(analogLeft, INPUT);
  pinMode(analogRight, INPUT);
  pinMode(encoderRightPin, INPUT);
  pinMode(encoderLeftPin, INPUT);

  // Enable Pin Change Interrupts for Encoders (A3 and A4)
  PCICR |= (1 << PCIE1);                     // Enable PCINT[14:8] group (Port C)
  PCMSK1 |= (1 << PCINT11) | (1 << PCINT12); // Enable PC3 (A3) and PC4 (A4)

  startTime = millis();
}

void loop() {
  // Read line sensors
  int aLeft  = analogRead(analogLeft);
  int aRight = analogRead(analogRight);

  // BLACK (low value) detection
  bool leftWhite  = (aLeft  > thresholdVal);
  bool rightWhite = (aRight > thresholdVal);

  unsigned long currentTime = (millis() - startTime) / 1000;

  // Calculate distance from encoders
  float avgCounts = (leftCount + rightCount) / 2.0;
  float distance = avgCounts * CM_PER_COUNT;

  // === STOP CONDITION: Both sensors detect black ===
  if (!leftWhite && !rightWhite) {
    stop();
    
    // Display final time and distance
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("FINISH! ");
    lcd.print(currentTime);
    lcd.print("s");
    
    lcd.setCursor(0, 1);
    lcd.print("Dist: ");
    lcd.print(distance);
    lcd.print("cm");
    
    stopCounter++;
    
    // Permanent stop after 5 consecutive detections
    if (stopCounter > 5) {
      while (1) {
        stop(); // Infinite stop loop
      }
    }
  } else {
    stopCounter = 0;
    
    // === LINE FOLLOWING LOGIC ===
    if (leftWhite && rightWhite) {
      moveForward();
    }
    else if (leftWhite && !rightWhite) {
      turnRight();
    }
    else if (!leftWhite && rightWhite) {
      turnLeft();
    }
  }

  // Update LCD display
  lcdDisplay(currentTime, aLeft, aRight, distance);
}

void lcdDisplay(int timeSec, int leftVal, int rightVal, float distance) {
  
  //Time and distance
  lcd.setCursor(0, 0);
  lcd.print("Time:");
  lcd.print(timeSec);
  lcd.print("s:");
  lcd.setCursor(0, 1);
  lcd.print("Dist:");
  lcd.print(distance, 1);
  lcd.print("cm");
}

// === MOTOR FUNCTIONS ===
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 100);
  analogWrite(ENB, 100);
}

void turnLeft() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 200);
  analogWrite(ENB, -100);
}

void turnRight() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 200);
  analogWrite(ENB, -100);
}

void stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
}