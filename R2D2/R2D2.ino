#include "protothreads.h"
#include "SoftwareSerial.h"
#include "HUSKYLENS.h"
#include <Wire.h>
#include <Digital_Light_TSL2561.h>
#include <Servo.h>

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
  PT_SCHEDULE(ball_distance(&pt_ball_distance));
  PT_SCHEDULE(goal_distance(&pt_goal_distance));
  PT_SCHEDULE(lens_adjustment(&pt_lens_adjustment, goal_lux));
}
