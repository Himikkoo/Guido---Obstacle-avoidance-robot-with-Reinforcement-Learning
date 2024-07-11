#include <Servo.h>

// Ultrasonic Sensor
const int trigPin = 2;
const int echoPin = 6;

// Motor A
int enA = 9;
int in1 = 8;
int in2 = 7;

// Motor B
int enB = 3;
int in3 = 4;
int in4 = 5;

// Servo Motor
Servo servoMotor;
int servoPin = 10;

// Variables for distance
long duration;
int distanceLeft, distanceCenter, distanceRight;

void setup() {
  // Set all the motor control pins to outputs
  // Motor A
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  // Motor B
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Ultrasonic
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);  // Sets the echoPin as an Input

  // Servo
  servoMotor.attach(servoPin);
  Serial.begin(9600); // Starts the serial communication

  Serial.println("Setup complete");
}

void forward() {
  Serial.println("Moving forward");
  // Turn on motor A
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, 200);

  // Turn on motor B
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, 200);
}

void reverse() {
  Serial.println("Reversing");
  // Turn on motor A
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, 200);

  // Turn on motor B
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, 200);
}

void stopMotors() {
  Serial.println("Stopping motors");
  // Stop both motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

int measureDistance() {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH, 30000); // Timeout after 30 ms
  // Calculating the distance
  if (duration == 0) {
    // Indicates no echo received (timeout)
    Serial.println("No echo received (timeout)");
    return -1;
  } else {
    int distance = duration * 0.034 / 2;
    Serial.print("Distance: ");
    Serial.println(distance);
    return distance;
  }
}

void loop() {
  // Sweep servo to the left, center, and right positions
  servoMotor.write(0); // Look left
  delay(500);
  distanceLeft = measureDistance();

  servoMotor.write(90); // Look center
  delay(500);
  distanceCenter = measureDistance();

  servoMotor.write(180); // Look right
  delay(500);
  distanceRight = measureDistance();

  // Make decision based on the distance measurements
  if (distanceCenter > 0 && distanceCenter < 100) {
    // Obstacle detected in front, check left and right
    if (distanceLeft > distanceRight) {
      // More space on the left, turn left
      Serial.println("Obstacle in front, turning left");
      stopMotors();
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      delay(1000); // Delay for turning
    } else {
      // More space on the right, turn right
      Serial.println("Obstacle in front, turning right");
      stopMotors();
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      delay(1000); // Delay for turning
    }
  } else {
    // No obstacle in front, move forward
    Serial.println("No obstacle in front, moving forward");
    forward();
  }

  delay(500);
  stopMotors(); // Stop the motors before the next iteration
}
