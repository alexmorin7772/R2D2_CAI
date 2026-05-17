#include <Arduino.h>
#include "protothreads.h"
#include <SoftwareSerial.h>
#include "HUSKYLENS.h"
#include <Wire.h>
#include <Digital_Light_TSL2561.h>
#include <Servo.h>
#include "pt-sem.h"
#include "defines.h"
#include "MyObjRec.h"
#include "touch.h"
#include <SparkFun_TB6612.h>


extern pt_sem sem_ball, sem_goal, sem_line, sem_ipc, sem_motor;
extern pt ptBrain;
extern int goal_lux;


pt readPos, adcDisp;
pt solKick;
pt ptAlex_test;
pt ptActuator;

//Create all protothread structs
pt ptEyelid;
pt ptHuskylens;
pt pt_ball_distance;
pt pt_goal_distance;
pt pt_line_tracking;
pt pt_lens_adjustment;

//Huskylens variables
HUSKYLENS huskylens;

// struct location {
//   int x, y;
// };

//ball distance variables
//float ball_size_10 = 120.0;
//float ball_current_distance;
float ball_current_size;
float ball_turn_angle;
int ball_leftmost_x;
int ball_rightmost_x;
location ball_location;
//float ball_current_size;
bool ball_success;

//goal distance variablesball_location
// float goal_size_30 = 100.0;
//float goal_current_distance;
// float goal_height;
// bool goal_success;
// location goal_location;

const double BALL_DIAMETER = 4.27;


//object recognition state machine
// typedef enum object_recognition_state {
//   start,
//   initialization,
//   test_for_object,
//   check_time,
//   evaluate_distance,
//   finish
// };

//Initialize functions
int ball_distance(struct pt* pt);
int eyelidThread(struct pt* pt);
int goal_distance(struct pt* pt);
// int threadBrain(struct pt *pt);
// int threadActuator(struct pt *pt);

//state machine variables
int servo_position = 0;
//other variables
// unsigned long max_milliseconds = 400; //0.4 second

Servo myservo;  // create Servo object to control a servo

int pos = 0;  // variable to store the servo position

// typedef enum lens_adjustment_state {
//   lens_start,
//   read_lux,
//   calculate_angle,
//   move_servo,
//   lens_finish
// };

// int lux = 9999;
// int goal_lux = 500;
// int angle = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  TSL2561.init();
  PT_INIT(&ptEyelid);
  PT_INIT(&ptHuskylens);
  PT_INIT(&pt_ball_distance);
  PT_INIT(&pt_goal_distance);
  PT_INIT(&pt_line_tracking);
  PT_INIT(&pt_lens_adjustment);
  PT_INIT(&ptBrain);
  PT_INIT(&ptActuator);
  PT_SEM_INIT(&sem_ball, 1);
  PT_SEM_INIT(&sem_goal, 1);
  PT_SEM_INIT(&sem_line, 1);
  PT_SEM_INIT(&sem_ipc, 1);
  PT_SEM_INIT(&sem_motor, 1);
  myservo.attach(9);  // attaches the servo on pin 9 to the Servo object
  while (!huskylens.begin(Wire)) Serial.println("Begin failed!");
  // huskyAlgorithm();   // Huskylens - Vision.ino
  // motorSetup();       // Motor Driver - 
  // groveDLSsetup();    // Light Sensor - 
  // touchSetup();       // Touch Sensor - 
  // eyelidSetup();      // Servo / Iris - Eyelid.ino
}

void loop() {
  //PT_SCHEDULE(eyelidThread(&ptEyelid));
  //PT_SCHEDULE(huskyRead(&ptHuskylens));
  PT_SCHEDULE(ball_distance(&pt_ball_distance));
  PT_SCHEDULE(goal_distance(&pt_goal_distance));
  PT_SCHEDULE(lens_adjustment(&pt_lens_adjustment, goal_lux));
  PT_SCHEDULE(threadADCRead(&readPos));
  PT_SCHEDULE(threadDisplay(&adcDisp));
  PT_SCHEDULE(threadKick(&solKick));
  PT_SCHEDULE(threadMain(&ptAlex_test));
  // PT_SCHEDULE(threadBrain(&ptBrain));
  // PT_SCHEDULE(threadActuator(&ptActuator));
}


const int MOTOR_LEFT_FWD  = 3;        //Change pin if necessary
const int MOTOR_LEFT_BWD  = 4;        //Change pin if necessary
const int MOTOR_RIGHT_FWD = 5;        //Change pin if necessary
const int MOTOR_RIGHT_BWD = 6;        //Change pin if necessary
const int MOTOR_LEFT_SPEED  = 2;      //Change pin if necessary
const int MOTOR_RIGHT_SPEED = 7;      //Change pin if necessary

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

// typedef enum soccer_strategy_state {
//   strategy_idle,
//   strategy_find_ball,
//   strategy_chase_ball,
//   strategy_align_goal,
//   strategy_drive_to_goal,
//   strategy_shoot,
//   strategy_retreat
// };

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
  //stopMotors();
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

