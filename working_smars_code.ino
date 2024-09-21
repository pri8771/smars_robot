#include <AFMotor.h>
#include <NewPing.h>  // Include NewPing library for ultrasonic sensor

// SMARS Demo with IR sensor for edge detection and Ultrasonic sensor for obstacle avoidance

// Motor definitions
AF_DCMotor R_motor(2);  // defines Right motor connector
AF_DCMotor L_motor(1);  // defines Left motor connector

// Ultrasonic sensor definitions
#define TRIG_PIN  A5  // Trig pazin for ultrasonic sensor
#define ECHO_PIN  A4 // Echo pin for ultrasonic sensor

#define IR_SENSOR_PIN A0  // IR sensor for edge detection
#define IR_THRESHOLD 800  // Example threshold value; adjust based on testing



#define MAX_DISTANCE 200  // Maximum distance we want to measure (in centimeters)
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);


void setup() {
  Serial.begin(9600);  // sets up Serial library at 9600 bps
  L_motor.setSpeed(140);  // sets L motor speed
  R_motor.setSpeed(140);  // sets R motor speed
  pinMode(IR_SENSOR_PIN, INPUT);  // Set IR sensor pin as input
  
  // Initialize motors in the stopped state
  R_motor.run(RELEASE);
  L_motor.run(RELEASE);
}

void loop() {
  // Check for obstacles using the ultrasonic sensor
  if (detectObstacle()) {
    avoidObstacle();  // If an obstacle is detected, avoid it
  }
  
  // Check for edges using the IR sensor
  if (detectEdge()) {
    avoidEdge();  // If an edge is detected, avoid it
  } else {
    // If no obstacle or edge is detected, move forward
    moveForward();
  }
}

// Function to detect obstacles using the ultrasonic sensor
bool detectObstacle() {
  int distance = sonar.ping_cm();  // Get the distance to the nearest object in centimeters
  if (distance > 0 && distance < 15) {  // If distance is less than 15cm, consider it as an obstacle
  Serial.println("Object Detected");
    return true;
  } else {
    return false;
  }
}

// // Function to detect the edge of the table using the IR sensor
// bool detectEdge() {
//   int sensorValue = digitalRead(IR_SENSOR_PIN);
//   Serial.print("IR Sensor Value: ");  // Print a label
//   Serial.println(sensorValue);  // Print the sensor value to the Serial Monitor
//   if (sensorValue == LOW) {  // LOW indicates no surface below (edge of table)
//   Serial.println("Edge detected!");  // Print message if edge is detected
//     Serial.println("Edge detected!");  // Print message if edge is detected
//     return true;
//   } else {
//     Serial.println("Surface detected.");  // Print message if no edge is detected
//     return false;
//   }
// }

bool detectEdge() {
  int sensorValue = analogRead(IR_SENSOR_PIN);  // Read the analog value from the IR sensor
  //Serial.print("IR Sensor Value: ");  // Print a label
  //Serial.println(sensorValue);  // Print the sensor value to the Serial Monitor

  // Check if the sensor value is below the threshold (indicating an edge)
  if (sensorValue < IR_THRESHOLD) {
    //Serial.println("Edge detected!");  // Print message if edge is detected
    return true;  // Return true to indicate an edge
  } else {
    //Serial.println("Surface detected.");  // Print message if no edge is detected
    return false;  // Return false to indicate no edge
  }
}




// Function to avoid obstacles
void avoidObstacle() {
  stopMoving();  // Stop the robot
  delay(500);  // Wait for half a second
  moveBackward();  // Move backward to avoid the obstacle
  delay(1000);  // Move back for 1 second
  turnLeft();  // Turn left to change direction
  delay(500);  // Turn for half a second
  stopMoving();  // Stop after turning
}

// Function to avoid falling off the edge of the table
void avoidEdge() {
  stopMoving();  // Stop the robot
  delay(500);  // Wait for half a second
  moveBackward();  // Move backward away from the edge
  delay(1000);  // Move back for 1 second
  turnLeft();  // Turn left to move away from the edge
  delay(500);  // Turn for half a second
  stopMoving();  // Stop after turning
}

// Function to move the robot forward
void moveForward() {
  R_motor.run(FORWARD);
  L_motor.run(FORWARD);
}

// Function to move the robot backward
void moveBackward() {
  R_motor.run(BACKWARD);
  L_motor.run(BACKWARD);
}

// Function to turn the robot left
void turnLeft() {
  R_motor.run(FORWARD);
  L_motor.run(BACKWARD);
}

// Function to stop the robot
void stopMoving() {
  R_motor.run(RELEASE);
  L_motor.run(RELEASE);
}