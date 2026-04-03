#include <NewPing.h>

// --- PIN DEFINITIONS ---
// Sensors: Front-Left, Front-Right, Left, Right, Back
const int TRIG_PINS[5] = {2, 4, 6, 8, 10};
const int ECHO_PINS[5] = {3, 5, 7, 9, 11};
const int MAX_DIST = 200; // Max distance in cm

NewPing sensors[5] = {
  NewPing(TRIG_PINS[0], ECHO_PINS[0], MAX_DIST),
  NewPing(TRIG_PINS[1], ECHO_PINS[1], MAX_DIST),
  NewPing(TRIG_PINS[2], ECHO_PINS[2], MAX_DIST),
  NewPing(TRIG_PINS[3], ECHO_PINS[3], MAX_DIST),
  NewPing(TRIG_PINS[4], ECHO_PINS[4], MAX_DIST)
};

// --- Q-LEARNING PARAMETERS ---
const int NUM_STATES = 32; // 2^5 sensors
const int NUM_ACTIONS = 4; // 0:Fwd, 1:Bwd, 2:Left, 3:Right
float q_table[NUM_STATES][NUM_ACTIONS];

float alpha = 0.1;   // Learning Rate
float gamma = 0.9;   // Discount Factor
float epsilon = 0.2; // Exploration Rate (20% chance to try random move)

int currentState = 0;

void setup() {
  Serial.begin(9600);
  // Initialize Q-table with zeros
  for(int i=0; i<NUM_STATES; i++) {
    for(int j=0; j<NUM_ACTIONS; j++) q_table[i][j] = 0.0;
  }
}

void loop() {
  // 1. Observe current state
  currentState = get_encoded_state();

  // 2. Choose action (Epsilon-Greedy)
  int action;
  if ((random(0, 100) / 100.0) < epsilon) {
    action = random(0, NUM_ACTIONS); // Explore
  } else {
    action = get_best_action(currentState); // Exploit
  }

  // 3. Perform action
  execute_action(action);
  delay(200); // Give motors time to move
  stop_motors();

  // 4. Observe new state and reward
  int nextState = get_encoded_state();
  float reward = calculate_reward(action);

  // 5. Update Q-Table (Bellman Equation)
  // Q(s,a) = Q(s,a) + alpha * (reward + gamma * max(Q(s',a')) - Q(s,a))
  float maxNextQ = q_table[nextState][get_best_action(nextState)];
  q_table[currentState][action] += alpha * (reward + (gamma * maxNextQ) - q_table[currentState][action]);

  Serial.print("State: "); Serial.print(currentState);
  Serial.print(" | Action: "); Serial.print(action);
  Serial.print(" | Reward: "); Serial.println(reward);
}

// --- HELPER FUNCTIONS ---

int get_encoded_state() {
  int state = 0;
  for (int i = 0; i < 5; i++) {
    int dist = sensors[i].ping_cm();
    // If distance is between 1 and 15cm, mark as "Near" (1)
    int binaryDist = (dist > 0 && dist < 15) ? 1 : 0;
    state |= (binaryDist << i); // Encode 5 sensors into a 5-bit number
  }
  return state;
}

int get_best_action(int state) {
  int best = 0;
  for (int i = 1; i < NUM_ACTIONS; i++) {
    if (q_table[state][i] > q_table[state][best]) best = i;
  }
  return best;
}

float calculate_reward(int action) {
  int s = get_encoded_state();
  bool frontNear = (s & 1) || (s & 2); // FL or FR near
  bool backNear = (s & 16);           // Back near
  
  if (action == 0 && frontNear) return -100; // Hit something in front
  if (action == 1 && backNear) return -100;  // Hit something in back
  if (action == 0 && !frontNear) return 10;   // Moving forward safely
  return -1; // Small penalty for just turning or idling
}

void execute_action(int action) {
  switch(action) {
    case 0: /* Code to Drive Forward */ break;
    case 1: /* Code to Drive Backward */ break;
    case 2: /* Code to Spin Left */ break;
    case 3: /* Code to Spin Right */ break;
  }
}

void stop_motors() { /* Code to stop all motors */ }