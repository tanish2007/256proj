// --- Pin Definitions ---
// Change these values to match your physical wiring on the CICS 256 board.
#include <Arduino.h>
// Ultrasonic Sensor (HC-SR04)
// On your board, it looks like TRIG is meant to be 5, and ECHO might be 11 or 18.
// Please check the small text on the board next to the Ultrasonic header!
const int trigPin = 5;  // Printed on board as TRIG-5
const int echoPin = 18; // Printed on board, please verify this number!

// Motor Driver Pins (L298N / L293D)
// Using general-purpose pins available on your black pin header
const int enA = 13; // Enable Pin for Left Motor 
const int in1 = 12; // Direction Pin 1
const int in2 = 14; // Direction Pin 2

const int enB = 27; // Enable Pin for Right Motor
const int in3 = 33; // Direction Pin 3
const int in4 = 32; // Direction Pin 4

// --- Constants ---
const int OBSTACLE_DISTANCE_CM = 25; // Distance at which the car should react

void setup() {
  // Initialize Serial Monitor for debugging
  Serial.begin(9600);

  // Set up Ultrasonic pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Set up Motor pins as outputs
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Start with motors at full speed (0-255)
  // If you are not using PWM pins for enable, you can remove these lines
  // and wire the L298N/L293D enable pins directly to 5V.
  analogWrite(enA, 255);
  analogWrite(enB, 255);
}

void loop() {
  // 1. Read the current distance to any obstacle
  int distance = readDistance();
  
  // Print distance to Serial Monitor for debugging
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // 2. Decision Logic based on distance
  if (distance > 0 && distance <= OBSTACLE_DISTANCE_CM) {
    // Obstacle detected! Take evasive action.
    turn(); 
    
    // Continue the turn for a short duration to clear the obstacle.
    // You may need to tune this delay depending on your motor's speed and car weight.
    delay(400); 
  } else {
    // Path is clear! Keep moving forward.
    moveForward();
  }
  
  // Small delay for sensor stability before the next reading
  delay(50);
}

/**
 * Reusable function to accurately read distance from the HC-SR04
 * Returns distance in centimeters.
 */
int readDistance() {
  // Ensure the trigger pin is clear before sending a pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Send a 10 microsecond HIGH pulse to trigger the sensor
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echo pin. pulseIn() returns the duration of the pulse in microseconds.
  // This represents the time it took for the sound wave to travel to the object and back.
  long duration = pulseIn(echoPin, HIGH);

  // Calculate distance in cm. 
  // Speed of sound is approx 343 m/s or 0.0343 cm/microsecond.
  // We divide by 2 because the wave travels out and back.
  int distance = duration * 0.0343 / 2;

  return distance;
}

/**
 * Moves the car straight forward.
 * Both left and right motors are set to move in the forward direction.
 */
void moveForward() {
  // Set Left Motor forward
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  // Set Right Motor forward
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

/**
 * Turns the car.
 * As requested, this performs a differential turn WITHOUT reversing.
 * It stops the right motor completely and keeps the left motor moving forward.
 * This effectively executes a right turn.
 */
void turn() {
  // Keep Left Motor moving forward
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  // Stop Right Motor completely
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

/**
 * Stops the car completely.
 * Useful if you need an emergency stop or want to halt entirely.
 */
void stopCar() {
  // Stop Left Motor
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

  // Stop Right Motor
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
