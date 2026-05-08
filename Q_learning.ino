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

// Two-threshold hysteresis prevents state flipping when a wall sits exactly
// at the detection boundary. Bit goes high at THRESH_CLOSE, stays high until
// the reading exceeds THRESH_CLEAR (4 cm dead band).
const int THRESH_CLOSE = 38;
const int THRESH_CLEAR = 42;
const int MAX_DIST_CM  = 200;

int currentState = 0;
int prevAction   = -1;

unsigned long lastFreeMs  = 0;
const unsigned long STUCK_MS = 10000;  // 10 seconds without reaching state 0 = stuck

// --- ASYNC SENSOR READING (interrupt-driven, all 4 fire in parallel) ---
// Each ISR records the micros() timestamp on rising edge (start) and falling
// edge (end). The main loop fires all 4 triggers together, then waits up to
// ECHO_TIMEOUT_US for every echo to complete.

#define NUM_SENSORS 4
const int TRIG_PINS[NUM_SENSORS] = { trig1, trig2, trig3, trig4 };
const int ECHO_PINS[NUM_SENSORS] = { echo1, echo2, echo3, echo4 };

volatile unsigned long echoStart[NUM_SENSORS];
volatile unsigned long echoDur[NUM_SENSORS];
volatile bool          echoReady[NUM_SENSORS];

// One ISR per echo pin — must be in IRAM and as small as possible
void IRAM_ATTR isrEcho0() {
  if (digitalRead(echo1)) { echoStart[0] = micros(); }
  else { echoDur[0] = micros() - echoStart[0]; echoReady[0] = true; }
}
void IRAM_ATTR isrEcho1() {
  if (digitalRead(echo2)) { echoStart[1] = micros(); }
  else { echoDur[1] = micros() - echoStart[1]; echoReady[1] = true; }
}
void IRAM_ATTR isrEcho2() {
  if (digitalRead(echo3)) { echoStart[2] = micros(); }
  else { echoDur[2] = micros() - echoStart[2]; echoReady[2] = true; }
}
void IRAM_ATTR isrEcho3() {
  if (digitalRead(echo4)) { echoStart[3] = micros(); }
  else { echoDur[3] = micros() - echoStart[3]; echoReady[3] = true; }
}

void (*ISR_TABLE[NUM_SENSORS])() = { isrEcho0, isrEcho1, isrEcho2, isrEcho3 };

// Maximum echo wait: sound travels 4 m in ~23 ms; 30 ms covers up to ~5 m
const unsigned long ECHO_TIMEOUT_US = 30000UL;

void setupSensorInterrupts() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    echoStart[i] = 0;
    echoDur[i]   = 0;
    echoReady[i] = false;
    attachInterrupt(digitalPinToInterrupt(ECHO_PINS[i]), ISR_TABLE[i], CHANGE);
  }
}

// Fires all 4 triggers simultaneously, waits for echoes, returns cm distances.
// Returns 999 for any sensor that times out (no echo / out of range).
void readAllSensors(int* dF, int* dL, int* dR, int* dB) {
  // Reset ready flags
  noInterrupts();
  for (int i = 0; i < NUM_SENSORS; i++) echoReady[i] = false;
  interrupts();

  // Fire all 4 triggers together (10 µs pulse each)
  for (int i = 0; i < NUM_SENSORS; i++) {
    digitalWrite(TRIG_PINS[i], LOW);
  }
  delayMicroseconds(2);
  for (int i = 0; i < NUM_SENSORS; i++) {
    digitalWrite(TRIG_PINS[i], HIGH);
  }
  delayMicroseconds(10);
  for (int i = 0; i < NUM_SENSORS; i++) {
    digitalWrite(TRIG_PINS[i], LOW);
  }

  // Wait until all echoes have returned or timeout
  unsigned long deadline = micros() + ECHO_TIMEOUT_US;
  while (micros() < deadline) {
    bool allDone = true;
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (!echoReady[i]) { allDone = false; break; }
    }
    if (allDone) break;
  }

  // Convert durations to cm; treat timeout (echoReady still false) as 999
  int raw[NUM_SENSORS];
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!echoReady[i]) {
      raw[i] = 999;
    } else {
      int cm = (int)(echoDur[i] * 0.0343f / 2.0f);
      raw[i] = (cm < 2 || cm > MAX_DIST_CM) ? 999 : cm;
    }
  }

  *dF = raw[0];
  *dL = raw[1];
  *dR = raw[2];
  *dB = raw[3];
}

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

  setupSensorInterrupts();

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
  lastFreeMs = millis();
  logLine("Loaded sim-trained Q-table. Running in pure policy mode.");
}

void loop() {
  webServer.handleClient();

  // Read all 4 sensors in parallel (~30 ms total vs ~400 ms sequential)
  int dF, dL, dR, dB;
  readAllSensors(&dF, &dL, &dR, &dB);

  // Build state and look up Q-table — purely RL, no hardcoded rules
  currentState = encodeState(dF, dL, dR, dB);

  // Reset free timer whenever bot is in open space
  if (currentState == 0) lastFreeMs = millis();

  // Stuck detection — if not in open space for 10s, force escape
  if (millis() - lastFreeMs > STUCK_MS) {
    logLine("STUCK — escaping");
    stop_motors(); delay(100);
    moveBackward(); delay(700);
    stop_motors(); delay(100);
    tankTurnLeft(); delay(1000);  // turn opposite to Q-table default
    stop_motors(); delay(100);
    lastFreeMs = millis();  // reset so it doesn't immediately re-trigger
    prevAction = -1;
    return;  // skip rest of loop, re-read sensors fresh next iteration
  }

  int action = get_best_action(currentState);

  // Brief stop only when changing direction to avoid motor jerk
  if (action != prevAction && prevAction != -1) {
    stop_motors();
    delay(80);
  }

  execute_action(action);
  prevAction = action;

  // Action durations — turns need more time to actually rotate meaningfully
  if      (action == 0) delay(200);  // forward
  else if (action == 1) delay(400);  // backward
  else                  delay(600);  // left or right turn

  // Stop after turns/backward so next sensor read is clean
  if (action != 0) {
    stop_motors();
    delay(80);
  }

  // WiFi log
  const char* actionNames[] = { "FWD", "BWD", "LEFT", "RIGHT" };
  String line = "S:" + String(currentState) +
                " A:" + String(actionNames[action]) +
                " F:" + String(dF) +
                " L:" + String(dL) +
                " R:" + String(dR) +
                " B:" + String(dB);
  logLine(line);

  // OLED display
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "S:" + String(currentState) + " " + actionNames[action]);
  display.setFont(ArialMT_Plain_16);
  display.drawString(0,  14, "F:" + String(dF));
  display.drawString(64, 14, "B:" + String(dB));
  display.drawString(0,  34, "L:" + String(dL));
  display.drawString(64, 34, "R:" + String(dR));
  display.display();
}

// --- HELPERS ---

int encodeState(int dF, int dL, int dR, int dB) {
  static bool nearF = false, nearL = false, nearR = false, nearB = false;

  // Each axis: latch true when reading drops below THRESH_CLOSE,
  // release only when it rises above THRESH_CLEAR.
  if      (dF > 0 && dF < THRESH_CLOSE) nearF = true;
  else if (dF > THRESH_CLEAR)           nearF = false;

  if      (dL > 0 && dL < THRESH_CLOSE) nearL = true;
  else if (dL > THRESH_CLEAR)           nearL = false;

  if      (dR > 0 && dR < THRESH_CLOSE) nearR = true;
  else if (dR > THRESH_CLEAR)           nearR = false;

  if      (dB > 0 && dB < THRESH_CLOSE) nearB = true;
  else if (dB > THRESH_CLEAR)           nearB = false;

  int state = 0;
  if (nearF) state |= (1 << BIT_FRONT);
  if (nearL) state |= (1 << BIT_LEFT);
  if (nearR) state |= (1 << BIT_RIGHT);
  if (nearB) state |= (1 << BIT_BACK);
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
