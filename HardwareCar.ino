// --- Pin Definitions ---
// Change these values to match your physical wiring on the CICS 256 board.
#include <Arduino.h>
#include <Wire.h>
#include "SSD1306Wire.h"

// Ultrasonic Sensor (HC-SR04)
const int trigPin = 5;  // Printed on board as TRIG-5
const int echoPin = 16; // Printed on board as ECHO-16

// Motor Driver Pins (L298N / L293D)
const int enA = 15; // Enable Pin for Left Motor
const int in1 = 13; // Direction Pin 1
const int in2 = 12; // Direction Pin 2

const int enB = 33; // Enable Pin for Right Motor
const int in3 = 14; // Direction Pin 3
const int in4 = 27; // Direction Pin 4

// Buttons (active LOW with internal pullups)
// B0 = right turn, B1 = stop toggle, B2 = left turn.
// Update these GPIO numbers to match whichever pins B0/B1/B2 are wired to on your CICS 256.
const int rightButtonPin = 0;  // B0 -> right turn
const int stopButtonPin  = 35; // B1 -> stop toggle
const int leftButtonPin  = 34; // B2 -> left turn

// --- Constants ---
const int OBSTACLE_DISTANCE_CM = 20;    // Stop when object is within this distance

// --- OLED Display ---
// CICS 256 onboard SSD1306 OLED. Default I2C address 0x3c, SDA=21, SCL=22.
SSD1306Wire display(0x3c, 21, 22);

// --- State ---
bool stopped = false;

// Stop button debounce
int lastStopReading = HIGH;
int stableStopState = HIGH;
unsigned long lastStopDebounceMs = 0;

// Active turn direction (based on whether a turn button is currently held)
enum TurnDir { TURN_NONE, TURN_LEFT, TURN_RIGHT };
TurnDir activeTurn = TURN_NONE;

const unsigned long DEBOUNCE_MS = 50;

void setup() {
  Serial.begin(115200);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(stopButtonPin,  INPUT_PULLUP);
  pinMode(rightButtonPin, INPUT_PULLUP);
  pinMode(leftButtonPin,  INPUT_PULLUP);

  analogWrite(enA, 255);
  analogWrite(enB, 255);

  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.clear();
  display.drawString(0, 0, "Booting...");
  display.display();
}

void loop() {
  checkStopButton();
  checkTurnButtons();

  int distance = readDistance();
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  showStatus(distance);

  // Priority: stop > held turn button > obstacle stop > forward
  if (stopped) {
    stopCar();
  } else if (activeTurn == TURN_RIGHT) {
    turnRight();
  } else if (activeTurn == TURN_LEFT) {
    turnLeft();
  } else if (distance > 0 && distance <= OBSTACLE_DISTANCE_CM) {
    stopCar();
  } else {
    moveForward();
  }

  delay(50);
}

// Toggle the stopped flag on a clean press of B1.
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

// Hold B0 -> right turn; hold B2 -> left turn. Released = no turn.
void checkTurnButtons() {
  bool rightHeld = (digitalRead(rightButtonPin) == LOW);
  bool leftHeld  = (digitalRead(leftButtonPin)  == LOW);

  if (rightHeld)      activeTurn = TURN_RIGHT;
  else if (leftHeld)  activeTurn = TURN_LEFT;
  else                activeTurn = TURN_NONE;
}

void showStatus(int distance) {
  display.clear();
  display.setFont(ArialMT_Plain_16);

  const char* state = "RUNNING";
  if (stopped)                          state = "STOPPED";
  else if (activeTurn == TURN_RIGHT)    state = "RIGHT";
  else if (activeTurn == TURN_LEFT)     state = "LEFT";

  display.drawString(0, 0, state);

  display.setFont(ArialMT_Plain_24);
  String line = "Dist: " + String(distance) + " cm";
  display.drawString(0, 24, line);
  display.display();
}

int readDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  int distance = duration * 0.0343 / 2;
  return distance;
}

void moveForward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

// Right turn: stop right motor, left motor forward.
void turnRight() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

// Left turn: stop left motor, right motor forward.
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
