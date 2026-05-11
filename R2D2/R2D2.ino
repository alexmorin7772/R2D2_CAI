#include "protothreads.h"
#include "SoftwareSerial.h"
#include "HUSKYLENS.h"
#include <Wire.h>
#include <Digital_Light_TSL2561.h>
#include <Servo.h>
#include "pt-sem.h"
#include "touch.h"
/*
"R2D2.ino" by team R2D2_CAI in the git repsitory "R2D2_CAI"
To create a "robot" that can "play soccer"
Can Comment/Uncomment lines using #ifdef and #endif
*/

// #define CAMERA_THREADS // For CAMERA_THREADS that are disabled: Uncomment to re-enable

extern volatile pt solKick; // Touch.ino declarations
extern volatile pt ptAlex_test; // Touch.ino declarations
extern volatile pt readPos, adcDisp; // Touch.ino declarations

uint32_t ipc_comms = 0;

bool started = false;
//boolean for the start/stop signal

#define SOLENOID_PIN 12

void setup() {
  Serial.begin(115200);
  // while (!Serial && (millis() < 5000)); // Wait 5 seconds
  Wire.begin();
  Serial.println("Serial Start");
  
  // setupVision();
  //   Serial.println("Vision Start");
  // setupLightSensor();
  //   Serial.println("Light Start");
  touchSetup();
    Serial.println("Touch Start");
  
  // Touch.ino declarations moved to touch.h

  // huskyAlgorithm();   // Huskylens - Vision.ino
  // motorSetup();       // Motor Driver - 
  // groveDLSsetup();    // Light Sensor - 
  // touchSetup();       // Touch Sensor - Touch.ino
  // eyelidSetup();      // Servo / Iris - Eyelid.ino
}

void loop() {
  //PT_SCHEDULE(eyelidThread(&ptEyelid));
  //PT_SCHEDULE(huskyRead(&ptHuskylens));
  #ifdef CAMERA_THREADS
    PT_SCHEDULE(ball_distance(&pt_ball_distance));
    PT_SCHEDULE(goal_distance(&pt_goal_distance));
    PT_SCHEDULE(lens_adjustment(&pt_lens_adjustment));
  #endif
  PT_SCHEDULE(threadADCRead(&readPos));
  PT_SCHEDULE(threadDisplay(&adcDisp));
  PT_SCHEDULE(threadKick(&solKick));
  PT_SCHEDULE(threadMain(&ptAlex_test));
}