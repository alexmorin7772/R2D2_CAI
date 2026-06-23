#include <Arduino.h>
#include "protothreads.h"
#include "HUSKYLENS.h"
#include <SoftwareSerial.h>
#include <Wire.h>
//#include <Digital_Light_TSL2561.h>
#include <Servo.h>
#include "Digital_Light_TSL2561.h"
#include "defines.h"
#include "MyObjRec.h"
#include "math.h"

extern volatile pt solKick;
extern volatile pt ptAlex_test;
extern volatile pt readPos, adcDisp;


lens_adjustment_state lens_state = lens_start;

extern Servo myservo;
extern int lux;
extern int angle;

int lens_adjustment(struct pt* pt, int goal_lux) {
  PT_BEGIN(pt);
  for(;;) {
    if (lens_state == lens_start) {
      lens_state = read_lux;
    } else if (lens_state == read_lux) {
      lux = TSL2561.readVisibleLux();
      lens_state = calculate_angle;
    } else if (lens_state == calculate_angle) {
      if (goal_lux >= lux) angle = 180;
      else angle = 180 - 2 * acos(sqrt(static_cast<float>(goal_lux) / static_cast<float>(lux)));
      lens_state = move_servo;
    } else if (lens_state == move_servo) {
      myservo.write(angle);
      lens_state = lens_finish;
    } else if (lens_state == lens_finish) {
      lens_state = lens_start;
      PT_SLEEP(pt, 1000);
    } else {
      lens_state = lens_start;
    }
  }
  PT_END(pt);
}