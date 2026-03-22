#include "protothreads.h"
#include "SoftwareSerial.h"
#include "HUSKYLENS.h"
#include <Wire.h>
#include <Digital_Light_TSL2561.h>
#include <Servo.h>

/*
The following are all defined constants
#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define EULER 2.718281828459045235360287471352

#define SERIAL  0x0
#define DISPLAY 0x1

#define LSBFIRST 0
#define MSBFIRST 1

#define CHANGE 1
#define FALLING 2
#define RISING 3
*/

//Create all protothread structs
pt ptEyelid;
pt ptHuskylens;
pt pt_ball_distance;
pt pt_goal_distance;
pt pt_lens_adjustment;

//Huskylens variables
HUSKYLENS huskylens;
SoftwareSerial serial(10, 11);

struct location {
  int x, y;
};

//ball distance variables
float ball_size_10 = 120.0;
float ball_current_distance;
float ball_current_size;
bool ball_success;
location ball_location;

//goal distance variables
float goal_size_30 = 100.0;
float goal_current_distance;
float goal_height;
bool goal_success;
location goal_location;

const double BALL_DIAMETER = 4.27;
//ball diameter in cm
/*
Coordinate system
(0, 0)                 (320, 0)

           (160, 120)

(0, 240)               (320, 240)
*/

//object recognition state machine
typedef enum object_recognition_state {
  start,
  initialization,
  test_for_object,
  check_time,
  evaluate_distance,
  finish
};

//do NOT write to any of these variables except for is_most_recent
typedef struct object_recognition_results {
  bool is_most_recent = false; //this will be changed to true once the Huskylens writes to it
  //once someone reads it, it should be set to false so the same information isn't used again
  float object_distance;
  float object_size;
  location object_location;
  //stores the x and y coordinates of the ball (do object_location.x or object_location.y to get the x and y values)
};

//Initialize functions
int ball_distance(struct pt* pt);
int eyelidThread(struct pt* pt);
int goal_distance(struct pt* pt);

//state machine variables
int servo_position = 0;

//other variables
unsigned long max_milliseconds = 400; //0.4 second

Servo myservo;  // create Servo object to control a servo

int pos = 0;  // variable to store the servo position

typedef enum lens_adjustment_state {
  lens_start,
  read_lux,
  calculate_angle,
  move_servo,
  lens_finish
};

int lux = 9999;
int actual_lux = 9999;
int goal_lux = 500;
int angle = 0;

const int OUT1 = -1;
const int OUT2 = -1;
//the OUT1 and OUT2 pins go to analog pins and we read them to figure out whether to start or stop
//remember to change the OUT1 and OUT2 pins when we decide on it

bool started = false;
//boolean for the start/stop signal

const float ANALOG_RANGE = 3.3;
const float ANALOG_MAX = 1023.0;
//range of analog voltages and the maximum possible analog reading
const float ON = 3.3;
const float OFF = 0.0;
//voltage for on and off signals

void setup() {
  Serial.begin(115200);
  serial.begin(9600);
  Wire.begin();
  TSL2561.init();
  PT_INIT(&ptEyelid);
  PT_INIT(&ptHuskylens);
  PT_INIT(&pt_ball_distance);
  PT_INIT(&pt_goal_distance);
  PT_INIT(&pt_lens_adjustment);
  myservo.attach(9);  // attaches the servo on pin 9 to the Servo object
  while (!huskylens.begin(serial)) Serial.println("Begin failed!");
  // huskyAlgorithm();   // Huskylens - Vision.ino
  // motorSetup();       // Motor Driver - 
  // groveDLSsetup();    // Light Sensor - 
  // touchSetup();       // Touch Sensor - 
  // eyelidSetup();      // Servo / Iris - Eyelid.ino
}

void loop() {
  //PT_SCHEDULE(eyelidThread(&ptEyelid));
  //PT_SCHEDULE(huskyRead(&ptHuskylens));
  //PT_SCHEDULE(ball_distance(&pt_ball_distance));
  //PT_SCHEDULE(goal_distance(&pt_goal_distance));
  PT_SCHEDULE(lens_adjustment(&pt_lens_adjustment, goal_lux));
}

pt pt_strategy;
motorSetup();
PT_INIT(&pt_strategy);
PT_SCHEDULE(soccer_strategy(&pt_strategy));

//const int MOTOR_LEFT_FWD  = 3;
//const int MOTOR_LEFT_BWD  = 4;
//const int MOTOR_RIGHT_FWD = 5;
//const int MOTOR_RIGHT_BWD = 6;
//const int MOTOR_LEFT_SPEED  = 2;
//const int MOTOR_RIGHT_SPEED = 7;

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
