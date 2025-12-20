#include "protothreads.h"
#include "SoftwareSerial.h"
#include "HUSKYLENS.h"
#include <Servo.h>

//Create all protothread structs
pt ptEyelid;
pt ptHuskylens;
pt pt_ball_distance;
pt pt_goal_distance;

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

//Initialize functions
int ball_distance(struct pt* pt);
int eyelidThread(struct pt* pt);
int goal_distance(struct pt* pt);

Servo myservo;  // create Servo object to control a servo
// twelve Servo objects can be created on most boards

int pos = 0;  // variable to store the servo position

void setup() {
  Serial.begin(115200);
  serial.begin(9600);
  PT_INIT(&ptEyelid);
  PT_INIT(&ptHuskylens);
  PT_INIT(&pt_ball_distance);
  PT_INIT(&pt_goal_distance);
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
  PT_SCHEDULE(eyelidThread(&ptEyelid));
  //PT_SCHEDULE(huskyRead(&ptHuskylens));
  PT_SCHEDULE(ball_distance(&pt_ball_distance));
  PT_SCHEDULE(goal_distance(&pt_goal_distance));
}
