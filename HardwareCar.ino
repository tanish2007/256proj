// --- Pin Definitions ---
#include <Arduino.h>
#include <Wire.h>
#include "SSD1306Wire.h"

// 4 Ultrasonic Sensors (HC-SR04)
// Sensor 1 (Front): TRIG=5, ECHO=16
const int trig1 = 5;
const int echo1 = 16;

// Sensor 2 (Left): TRIG=4, ECHO=2
const int trig2 = 4;
const int echo2 = 18;

// Sensor 3 (Right): TRIG=32, ECHO=26
const int trig3 = 32;
const int echo3 = 26;

// Sensor 4 (Back): TRIG=19, ECHO=23
const int trig4 = 19;
const int echo4 = 23;

// Motor Driver Pins (L298N)
const int enA = 15;
const int in1 = 13;
const int in2 = 12;

const int enB = 33;
const int in3 = 14;
const int in4 = 27;

// Buttons (active LOW with internal pullups)
const int rightButtonPin = 0;  // B0 -> right turn
const int stopButtonPin  = 35; // B1 -> stop toggle
const int leftButtonPin  = 34; // B2 -> left turn

// --- Constants ---
const int OBSTACLE_DISTANCE_CM = 20;

// --- OLED Display ---
SSD1306Wire display(0x3c, 21, 22);

// --- State ---
bool stopped = false;
int lastStopReading = HIGH;
int stableStopState = HIGH;
unsigned long lastStopDebounceMs = 0;

enum TurnDir { TURN_NONE, TURN_LEFT, TURN_RIGHT };
TurnDir activeTurn = TURN_NONE;

const unsigned long DEBOUNCE_MS = 50;

void setup() {
  Serial.begin(115200);

  // Sensor 1
  pinMode(trig1, OUTPUT);
  pinMode(echo1, INPUT);
  // Sensor 2
  pinMode(trig2, OUTPUT);
  pinMode(echo2, INPUT);
  // Sensor 3
  pinMode(trig3, OUTPUT);
  pinMode(echo3, INPUT);
  // Sensor 4
  pinMode(trig4, OUTPUT);
  pinMode(echo4, INPUT);

  // Motors
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Buttons
  pinMode(stopButtonPin,  INPUT_PULLUP);
  pinMode(rightButtonPin, INPUT_PULLUP);
  pinMode(leftButtonPin,  INPUT_PULLUP);

  analogWrite(enA, 255);
  analogWrite(enB, 255);

  // OLED
  display.init();
  display.flipScreenVertically();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.clear();
  display.drawString(0, 0, "Booting...");
  display.display();
}

void loop() {
  checkStopButton();
  checkTurnButtons();

  int dFront = readDistance(trig1, echo1);
  int dLeft  = readDistance(trig2, echo2);
  int dRight = readDistance(trig3, echo3);
  int dBack  = readDistance(trig4, echo4);

  Serial.print("F:"); Serial.print(dFront);
  Serial.print(" L:"); Serial.print(dLeft);
  Serial.print(" R:"); Serial.print(dRight);
  Serial.print(" B:"); Serial.println(dBack);

  showStatus(dFront, dLeft, dRight, dBack);

  // Priority: stop > held turn button > obstacle stop > forward
  if (stopped) {
    stopCar();
  } else if (activeTurn == TURN_RIGHT) {
    turnRight();
  } else if (activeTurn == TURN_LEFT) {
    turnLeft();
  } else if (dFront > 0 && dFront <= OBSTACLE_DISTANCE_CM) {
    stopCar();
  } else {
    moveForward();
  }

  delay(50);
}

void checkStopButton() {
  int reading = digitalRead(stopButtonPin);
  if (reading != lastStopReading) lastStopDebounceMs = millis();

  if ((millis() - lastStopDebounceMs) > DEBOUNCE_MS) {
    if (reading != stableStopState) {
      stableStopState = reading;
      if (stableStopState == LOW) {
        stopped = !stopped;
        Serial.print("Stop toggled: ");
        Serial.println(stopped ? "STOPPED" : "RUNNING");
      }
    }
  }
  lastStopReading = reading;
}

void checkTurnButtons() {
  bool rightHeld = (digitalRead(rightButtonPin) == LOW);
  bool leftHeld  = (digitalRead(leftButtonPin)  == LOW);

  if (rightHeld)      activeTurn = TURN_RIGHT;
  else if (leftHeld)  activeTurn = TURN_LEFT;
  else                activeTurn = TURN_NONE;
}

// Show all 4 sensor distances on the OLED
void showStatus(int front, int left, int right, int back) {
  display.clear();

  // Line 1: state
  display.setFont(ArialMT_Plain_10);
  const char* state = "RUNNING";
  if (stopped)                       state = "STOPPED";
  else if (activeTurn == TURN_RIGHT) state = "TURN RIGHT";
  else if (activeTurn == TURN_LEFT)  state = "TURN LEFT";
  display.drawString(0, 0, state);

  // Line 2: front and back
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 14, "F:" + String(front));
  display.drawString(64, 14, "B:" + String(back));

  // Line 3: left and right
  display.drawString(0, 34, "L:" + String(left));
  display.drawString(64, 34, "R:" + String(right));

  display.display();
}

int readDistance(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH, 30000);
  int distance = duration * 0.0343 / 2;
  return distance;
}

void moveForward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void turnRight() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void turnLeft() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void stopCar() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
