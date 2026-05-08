#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include "SSD1306Wire.h"
#include "q_table.h"   // Q_TABLE_INIT — pre-trained values from qsim.py

// --- Wireless log monitor (open http://192.168.4.1 in your browser) ---
const char* AP_SSID = "QBot";
const char* AP_PASS = "qbot1234";   // 8+ chars required
WebServer webServer(80);
SSD1306Wire display(0x3c, 21, 22);

const int LOG_BUFFER_SIZE = 60;
String logBuffer[LOG_BUFFER_SIZE];
int logHead = 0;
int logCount = 0;

void logLine(const String& s) {
  Serial.println(s);
  logBuffer[logHead] = s;
  logHead = (logHead + 1) % LOG_BUFFER_SIZE;
  if (logCount < LOG_BUFFER_SIZE) logCount++;
}

void handleLogs() {
  String out = "";
  int start = (logCount < LOG_BUFFER_SIZE) ? 0 : logHead;
  for (int i = 0; i < logCount; i++) {
    int idx = (start + i) % LOG_BUFFER_SIZE;
    out += logBuffer[idx] + "\n";
  }
  webServer.send(200, "text/plain", out);
}

// --- PIN DEFINITIONS (verified with ObstacleAvoid.ino) ---
// Sensor 1 (Front)
const int trig1 = 4;
const int echo1 = 18;
// Sensor 2 (Left)
const int trig2 = 19;
const int echo2 = 23;
// Sensor 3 (Right)
const int trig3 = 32;
const int echo3 = 26;
// Sensor 4 (Back)
const int trig4 = 5;
const int echo4 = 16;

// Motor Driver Pins (L298N)
const int enA = 15;
const int in1 = 13;
const int in2 = 12;
const int enB = 33;
const int in3 = 14;
const int in4 = 27;

// --- Q-LEARNING PARAMETERS ---
const int NUM_STATES = 16;  // 2^4 sensors (F, L, R, B)
const int NUM_ACTIONS = 4;  // 0:Fwd, 1:Bwd, 2:Left, 3:Right
float q_table[NUM_STATES][NUM_ACTIONS];

// Pure simulation policy — no on-bot learning, always use best action
const float epsilon = 0.0;  // 0 = always pick best action from sim-trained table

// State bit layout: bit0=Front, bit1=Left, bit2=Right, bit3=Back
const int BIT_FRONT = 0;
const int BIT_LEFT  = 1;
const int BIT_RIGHT = 2;
const int BIT_BACK  = 3;

const int NEAR_CM     = 40;   // Q-table state threshold
const int SAFETY_CM   = 55;   // hard override — always stop if front < this
const int BACKUP_CM   = 15;   // only back up if closer than this (nearly touching)
const int MAX_DIST_CM = 200;

int currentState = 0;
int prevAction   = -1;  // track previous action to avoid unnecessary stops


void setup() {
  Serial.begin(115200);

  // WiFi access point — connect your laptop to "QBot" then open http://192.168.4.1
  WiFi.softAP(AP_SSID, AP_PASS);
  IPAddress ip = WiFi.softAPIP();
  Serial.print("WiFi AP started. SSID="); Serial.print(AP_SSID);
  Serial.print(" PASS="); Serial.print(AP_PASS);
  Serial.print(" -> open http://"); Serial.println(ip);
  webServer.on("/logs", handleLogs);
  webServer.begin();

  // Sensors
  pinMode(trig1, OUTPUT); pinMode(echo1, INPUT);
  pinMode(trig2, OUTPUT); pinMode(echo2, INPUT);
  pinMode(trig3, OUTPUT); pinMode(echo3, INPUT);
  pinMode(trig4, OUTPUT); pinMode(echo4, INPUT);

  // Motors
  pinMode(enA, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  // Tune these two values until the car drives straight.
  // If it curves RIGHT -> right motor (B) is slower, lower LEFT_SPEED.
  // If it curves LEFT  -> left motor (A) is slower, lower RIGHT_SPEED.
  analogWrite(enA, 170);   // left motor speed — reduced to limit momentum
  analogWrite(enB, 170);   // right motor speed

  // OLED
  display.init();
  display.flipScreenVertically();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "Loading Q-table...");
  display.display();

  // Always load from sim-trained table — no flash, no on-bot learning
  memcpy(q_table, Q_TABLE_INIT, sizeof(q_table));
  logLine("Loaded sim-trained Q-table. Running in pure policy mode.");
}

void loop() {
  webServer.handleClient();

  // --- Read all sensors first ---
  int dF = readDistance(trig1, echo1); delay(8);
  int dL = readDistance(trig2, echo2); delay(8);
  int dR = readDistance(trig3, echo3); delay(8);
  int dB = readDistance(trig4, echo4);

  String mode;

  // --- SAFETY SHIELD: front sensor overrides everything ---
  // If wall is close ahead, always stop and turn — Q-table cannot override this
  if (dF < SAFETY_CM) {
    stop_motors();
    delay(100);

    // Only back up if nearly touching — otherwise tank turn in place
    if (dF < BACKUP_CM) {
      moveBackward();
      delay(400);
      stop_motors();
      delay(100);
    }

    // Turn toward whichever side has more space
    if (dR >= dL) {
      tankTurnRight();
    } else {
      tankTurnLeft();
    }
    delay(750);
    stop_motors();
    delay(100);

    mode = "SAFETY";
    prevAction = -1;

  } else {
    // --- Q-TABLE: front is clear, let Q-table guide behavior ---
    currentState = get_encoded_state();
    int action = get_best_action(currentState);

    if (action == 0) {
      // Q-table says forward — go
      if (prevAction != 0 && prevAction != -1) { stop_motors(); delay(60); }
      moveForward();
      prevAction = 0;
      delay(180);
      mode = "FWD";

    } else {
      // Q-table says don't go forward — turn toward more space
      stop_motors();
      delay(100);

      if (dB < NEAR_CM) {
        moveForward();
        delay(400);
        mode = "ESCAPE";
      } else if (dR >= dL) {
        tankTurnRight();
        delay(750);
        mode = "Q-RIGHT";
      } else {
        tankTurnLeft();
        delay(750);
        mode = "Q-LEFT";
      }

      stop_motors();
      delay(100);
      prevAction = action;
    }
  }

  // WiFi log
  String line = "F:" + String(dF) +
                " L:" + String(dL) +
                " R:" + String(dR) +
                " B:" + String(dB) +
                " | " + mode;
  logLine(line);

  // Display live sensor distances + current mode
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, mode);
  display.setFont(ArialMT_Plain_16);
  display.drawString(0,  14, "F:" + String(dF));
  display.drawString(64, 14, "B:" + String(dB));
  display.drawString(0,  34, "L:" + String(dL));
  display.drawString(64, 34, "R:" + String(dR));
  display.display();
}

// --- HELPERS ---

int readDistance(int trig, int echo) {
  int r[3];
  for (int i = 0; i < 3; i++) {
    digitalWrite(trig, LOW);  delayMicroseconds(2);
    digitalWrite(trig, HIGH); delayMicroseconds(10);
    digitalWrite(trig, LOW);
    long dur = pulseIn(echo, HIGH, 25000);
    int cm = (dur == 0) ? 999 : (int)(dur * 0.0343 / 2);
    r[i] = (cm < 2) ? 999 : cm;
    delay(5);
  }
  // Return median of 3
  if (r[0] > r[1]) { int t = r[0]; r[0] = r[1]; r[1] = t; }
  if (r[1] > r[2]) { int t = r[1]; r[1] = r[2]; r[2] = t; }
  if (r[0] > r[1]) { int t = r[0]; r[0] = r[1]; r[1] = t; }
  return r[1];
}

int get_encoded_state() {
  int dF = readDistance(trig1, echo1); delay(8);
  int dL = readDistance(trig2, echo2); delay(8);
  int dR = readDistance(trig3, echo3); delay(8);
  int dB = readDistance(trig4, echo4);

  int state = 0;
  if (dF > 0 && dF < NEAR_CM) state |= (1 << BIT_FRONT);
  if (dL > 0 && dL < NEAR_CM) state |= (1 << BIT_LEFT);
  if (dR > 0 && dR < NEAR_CM) state |= (1 << BIT_RIGHT);
  if (dB > 0 && dB < NEAR_CM) state |= (1 << BIT_BACK);
  return state;
}

int get_best_action(int state) {
  int best = 0;
  for (int i = 1; i < NUM_ACTIONS; i++) {
    if (q_table[state][i] > q_table[state][best]) best = i;
  }
  return best;
}

// --- MOTOR CONTROL ---

void execute_action(int action) {
  switch (action) {
    case 0: moveForward();  break;
    case 1: moveBackward(); break;
    case 2: turnLeft();     break;
    case 3: turnRight();    break;
  }
}

void moveForward() {
  digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);  // left forward
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);   // right forward (wired backward, flipped)
}

void moveBackward() {
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);   // left backward
  digitalWrite(in3, LOW);  digitalWrite(in4, HIGH);  // right backward (flipped)
}

void turnLeft() {
  tankTurnLeft();
}

void turnRight() {
  tankTurnRight();
}

void tankTurnLeft() {
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);   // left backward
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);   // right forward (flipped)
}

void tankTurnRight() {
  digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);  // left forward
  digitalWrite(in3, LOW);  digitalWrite(in4, HIGH);  // right backward (flipped)
}

void stop_motors() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
