#include <Wire.h>
#include <LiquidCrystal.h>  // --- ADDED ---

// --- ADDED - Motor Control Pins ---
#define MOTOR_A_EN 3
#define MOTOR_A_IN1 13
#define MOTOR_A_IN2 12
#define MOTOR_B_EN 11
#define MOTOR_B_IN1 2
#define MOTOR_B_IN2 1

// LCD pins
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

double Kp = 50;
double Ki = 0;
double Kd = 0.8;

double yaw_setpoint = 0;
double pid_error = 0;
double pid_integral = 0;
double pid_derivative = 0;
double pid_previous_error = 0;
double pid_output = 0;
int pitchState = 0;

int baseSpeed = 255;
int motorSpeedA = 0;
int motorSpeedB = 0;

const int MPU = 0x68;
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw = 0.0;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;

float elapsedTime, currentTime, previousTime;
unsigned long lastLCDUpdate = 0;
bool stopOperation = false;
unsigned long lastMillis = 0;
int c = 0;

// *** NEW: store highest ramp angle ***
float maxRampAngle = 0;

void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Yaw:");
  lcd.setCursor(0, 1);
  lcd.print("PID:");

  pinMode(MOTOR_A_EN, OUTPUT);
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_EN, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);

  setMotorSpeed('A', 0);
  setMotorSpeed('B', 0);

  calculate_IMU_error();
  delay(20);
}

void loop() {
  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;

  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

  GyroZ -= GyroErrorZ;
  GyroY -= GyroErrorY;

  yaw += GyroZ * elapsedTime;
  pitch += GyroY * elapsedTime;

  // *** NEW: track highest ramp angle while climbing ***
  if (pitchState == 0 || pitchState == 1) {
    if (pitch > maxRampAngle) {
      maxRampAngle = pitch;
    }
  }

  pid_error = yaw_setpoint - yaw;
  pid_integral += pid_error * elapsedTime;
  pid_integral = constrain(pid_integral, -100, 100);
  pid_derivative = (pid_error - pid_previous_error) / elapsedTime;

  pid_output = (Kp * pid_error) + (Ki * pid_integral) + (Kd * pid_derivative);
  pid_previous_error = pid_error;

  if (!stopOperation) {
    motorSpeedB = baseSpeed - pid_output;
    motorSpeedA = baseSpeed + pid_output;
    motorSpeedA = constrain(motorSpeedA, -255, 255);
    motorSpeedB = constrain(motorSpeedB, -255, 255);
    setMotorSpeed('A', motorSpeedA);
    setMotorSpeed('B', motorSpeedB);
  } else {
    setMotorSpeed('A', 0);
    setMotorSpeed('B', 0);
  }

  // === Ramp State Machine ===
  if (pitchState == 0) {
    if (pitch >= 20) {
      pitchState = 1;
      delay(100);
      stopOperation = true;
      lastMillis = millis();
    }
  } else if (pitchState == 1) {
    if (millis() - lastMillis >= 5000) {
      stopOperation = false;
      pitchState = 2;
    }
  } else if (pitchState == 2) {
    if (pitch <= 5) {
      pitchState = 3;
      delay(200);
      stopOperation = true;
      lastMillis = millis();
    }
  } else if (pitchState == 3) {
    if (millis() - lastMillis >= 4000) {
      stopOperation = false;
      pitchState = 4;
      yaw_setpoint = 360;
    }
  } else if (pitchState == 4) {
    if (abs(pid_error) <= 1) {
      setMotorSpeed('A', 0);
      setMotorSpeed('B', 0);
      stopOperation = true;
      pitchState = 5;
      lastMillis = millis();
      pitch = 0;
    }
  } else if (pitchState == 5) {
    if (millis() - lastMillis >= 1000) {
      stopOperation = false;
      pitchState = 6;
    }
  } else if (pitchState == 6) {
    if (pitch <= -20) {
      pitchState = 7;
    }
  } else if (pitchState == 7) {
    if (pitch >= -5) {
      pitchState = 8;
      stopOperation = true;
    }
  }

  // ==========================
  // LCD SECTION (MODIFIED)
  // ==========================
  if (currentTime - lastLCDUpdate > 100) {

    // When task finishes → show highest ramp angle
    if (pitchState == 8) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("MAX ANGLE:");
      lcd.setCursor(0, 1);
      lcd.print(maxRampAngle);
    } 
    else {
      // Normal live display
      lcd.setCursor(0, 0);
      lcd.print("Yaw:");
      lcd.setCursor(4, 0);
      lcd.print(yaw);

      lcd.setCursor(0, 1);
      lcd.print("Pitch:");
      lcd.setCursor(6, 1);
      lcd.print(pitch);
    }

    lastLCDUpdate = currentTime;
  }
}

void setMotorSpeed(char motor, int speed) {
  int enPin, in1Pin, in2Pin;

  if (motor == 'A') {
    enPin = MOTOR_A_EN;
    in1Pin = MOTOR_A_IN1;
    in2Pin = MOTOR_A_IN2;
  } else {
    enPin = MOTOR_B_EN;
    in1Pin = MOTOR_B_IN1;
    in2Pin = MOTOR_B_IN2;
  }

  if (speed > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    analogWrite(enPin, constrain(abs(speed), 100, 255));
  } else if (speed < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    analogWrite(enPin, constrain(abs(speed), 100, 255));
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(enPin, 0);
  }
}

void calculate_IMU_error() {
  c = 0;
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
    AccErrorX += (atan((AccY) / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI);
    AccErrorY += (atan(-1 * (AccX) / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI);
    c++;
  }
  AccErrorX /= 200;
  AccErrorY /= 200;

  c = 0;
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    GyroErrorX += GyroX / 131.0;
    GyroErrorY += GyroY / 131.0;
    GyroErrorZ += GyroZ / 131.0;
    c++;
  }
  GyroErrorX /= 200;
  GyroErrorY /= 200;
  GyroErrorZ /= 200;
}

