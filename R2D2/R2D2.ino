#include "protothreads.h"
#include <Servo.h>

pt ptEyelid;
pt ptHuskylens;

Servo myservo;  // create Servo object to control a servo
// twelve Servo objects can be created on most boards

int pos = 0;  // variable to store the servo position

void setup() {
  Serial.begin(115200);
  PT_INIT(&ptEyelid);
  PT_INIT(&ptHuskylens);
  myservo.attach(9);  // attaches the servo on pin 9 to the Servo object
  // huskyAlgorithm();   // Huskylens - Vision.ino
  // motorSetup();       // Motor Driver - 
  // groveDLSsetup();    // Light Sensor - 
  // touchSetup();       // Touch Sensor - 
  // eyelidSetup();      // Servo / Iris - Eyelid.ino
}

void loop() {
  PT_SCHEDULE(eyelidThread(&ptEyelid));
  PT_SCHEDULE(huskyRead(&ptHuskylens));
}






const int MOTOR_LEFT_FWD  = 3;
const int MOTOR_LEFT_BWD  = 4;
const int MOTOR_RIGHT_FWD = 5;
const int MOTOR_RIGHT_BWD = 6;
const int MOTOR_LEFT_SPEED  = 2;
const int MOTOR_RIGHT_SPEED = 7;

const int SPEED_FULL    = 200;
const int SPEED_TURN    = 160;
const int SPEED_APPROACH = 120;
const int SPEED_REVERSE  = 140;

const int FRAME_CENTER_X = 160;
const int FRAME_CENTER_Y = 120;
const int CENTER_DEADZONE = 30;

const float CLOSE_DISTANCE = 15.0;
const float CAPTURE_DISTANCE = 8.0;
const float GOAL_FAR_DISTANCE = 50.0;

typedef enum soccer_strategy_state {
  strategy_idle,
  strategy_find_ball,
  strategy_chase_ball,
  strategy_align_goal,
  strategy_drive_to_goal,
  strategy_shoot,
  strategy_retreat
};

soccer_strategy_state strategy_state = strategy_idle;
pt pt_strategy;
unsigned long strategy_timer = 0;
unsigned long search_start_time = 0;
const unsigned long MAX_SEARCH_TIME = 3000;

// Sets up all motor pins as outputs and stops the motors
void motorSetup() {
  pinMode(MOTOR_LEFT_FWD, OUTPUT);
  pinMode(MOTOR_LEFT_BWD, OUTPUT);
  // Right motor direction pins
  pinMode(MOTOR_RIGHT_FWD, OUTPUT);
  pinMode(MOTOR_RIGHT_BWD, OUTPUT);
  // PWM speed control pins
  pinMode(MOTOR_LEFT_SPEED, OUTPUT);
  pinMode(MOTOR_RIGHT_SPEED, OUTPUT);
  // Make sure motors are off at startup
  stopMotors();
}

// Both motors forward at the given speed
void driveForward(int speed) {
  digitalWrite(MOTOR_LEFT_FWD, HIGH);
  digitalWrite(MOTOR_LEFT_BWD, LOW);
  // Right motor also forward
  digitalWrite(MOTOR_RIGHT_FWD, HIGH);
  digitalWrite(MOTOR_RIGHT_BWD, LOW);
  // Set speed for both motors
  analogWrite(MOTOR_LEFT_SPEED, speed);
  analogWrite(MOTOR_RIGHT_SPEED, speed);
}

// Both motors backward at the given speed
void driveBackward(int speed) {
  digitalWrite(MOTOR_LEFT_FWD, LOW);
  digitalWrite(MOTOR_LEFT_BWD, HIGH);
  // Right motor also backward
  digitalWrite(MOTOR_RIGHT_FWD, LOW);
  digitalWrite(MOTOR_RIGHT_BWD, HIGH);
  // Set speed for both motors
  analogWrite(MOTOR_LEFT_SPEED, speed);
  analogWrite(MOTOR_RIGHT_SPEED, speed);
}

// Left motor backward, right motor forward to spin left
void turnLeft(int speed) {
  digitalWrite(MOTOR_LEFT_FWD, LOW);
  digitalWrite(MOTOR_LEFT_BWD, HIGH);
  // Right motor goes forward to create the spin
  digitalWrite(MOTOR_RIGHT_FWD, HIGH);
  digitalWrite(MOTOR_RIGHT_BWD, LOW);
  // Same speed on both for a centered pivot
  analogWrite(MOTOR_LEFT_SPEED, speed);
  analogWrite(MOTOR_RIGHT_SPEED, speed);
}

// Left motor forward, right motor backward to spin right
void turnRight(int speed) {
  digitalWrite(MOTOR_LEFT_FWD, HIGH);
  digitalWrite(MOTOR_LEFT_BWD, LOW);
  // Right motor reverses to swing right side back
  digitalWrite(MOTOR_RIGHT_FWD, LOW);
  digitalWrite(MOTOR_RIGHT_BWD, HIGH);
  // Same speed on both for a centered pivot
  analogWrite(MOTOR_LEFT_SPEED, speed);
  analogWrite(MOTOR_RIGHT_SPEED, speed);
}

// Steers toward a target by slowing one wheel
// x_offset is negative for left, positive for right
void steerToward(int x_offset, int speed) {
  // Scale the offset into a speed reduction for the inner wheel
  int reduction = map(abs(x_offset), 0, FRAME_CENTER_X, 0, speed);
  // Clamp so we never go below zero
  int slow_side = max(0, speed - reduction);

  // Both motors go forward
  digitalWrite(MOTOR_LEFT_FWD, HIGH);
  digitalWrite(MOTOR_LEFT_BWD, LOW);
  // Right motor also forward
  digitalWrite(MOTOR_RIGHT_FWD, HIGH);
  digitalWrite(MOTOR_RIGHT_BWD, LOW);

  if (x_offset < 0) {
    // Target is left, so slow the left motor to curve left
    analogWrite(MOTOR_LEFT_SPEED, slow_side);
    analogWrite(MOTOR_RIGHT_SPEED, speed);
  } else {
    // Target is right, so slow the right motor to curve right
    analogWrite(MOTOR_LEFT_SPEED, speed);
    analogWrite(MOTOR_RIGHT_SPEED, slow_side);
  }
}
