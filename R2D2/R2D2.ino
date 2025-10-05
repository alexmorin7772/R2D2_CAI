#include "protothreads.h"
#include <Servo.h>

pt ptEyelid;
pt ptHuskylens;

Servo myservo;  // create Servo object to control a servo
// twelve Servo objects can be created on most boards

int pos = 0;  // variable to store the servo position

void setup() {
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