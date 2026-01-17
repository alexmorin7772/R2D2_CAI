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

//ball distance variables
float ball_size_10 = 120.0;
float ball_current_distance;
float ball_current_size;
bool ball_success;

//goal distance variables
float goal_size_30 = 100.0;
float goal_current_distance;
float goal_height;
bool goal_success;

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
  while (!huskylens.begin(serial)) {
    Serial.println("Begin failed!");
    delay(100);
  }
  // huskyAlgorithm();   // Huskylens - Vision.ino
  // motorSetup();       // Motor Driver - 
  // groveDLSsetup();    // Light Sensor - 
  // touchSetup();       // Touch Sensor - 
  // eyelidSetup();      // Servo / Iris - Eyelid.ino
}

void loop() {
  //PT_SCHEDULE(eyelidThread(&ptEyelid));
  //PT_SCHEDULE(huskyRead(&ptHuskylens));
  PT_SCHEDULE(ball_distance(&pt_ball_distance, max_milliseconds));
  PT_SCHEDULE(goal_distance(&pt_goal_distance, max_milliseconds));
  PT_SCHEDULE(lens_adjustment(&pt_lens_adjustment, goal_lux));
}
