#include <Arduino.h>
#include "HUSKYLENS.h"
#include "protothreads.h"
#include "defines.h"
#include "MyObjRec.h"
extern HUSKYLENS huskylens;


float ball_size_10 = 120.0;
extern float ball_current_size;



float ball_current_distance;
extern location ball_location;

object_recognition_state ball_state = start;
unsigned long ball_start_millis;

int ball_distance(struct pt* pt) {
  PT_BEGIN(pt);
  for (;;) {
    //Serial.print("Ball state: ");
    //Serial.println(ball_state);
    if (ball_state == start) {
      ball_start_millis = millis();
      ball_state = initialization;
      //have to go to initialization
    } else if (ball_state == initialization) {
      huskylens.writeAlgorithm(ALGORITHM_OBJECT_TRACKING);
      ball_state = test_for_object;
      //initialize everything and test for ball
    } else if (ball_state == test_for_object) {
      //failure leads to checking the time
      if (!huskylens.request()) ball_state = check_time;
      else if (!huskylens.isLearned()) ball_state = check_time;
      else if (!huskylens.available()) ball_state = check_time;
      else ball_state = evaluate_distance;
      //otherwise find the distance
    } else if (ball_state == check_time) {
      if ((millis() - ball_start_millis) > max_milliseconds) {
        //went over time limit, so go to finish
        ball_state = finish;
      } else ball_state = test_for_object;
    } else if (ball_state == evaluate_distance) {
      HUSKYLENSResult result = huskylens.read();
      if (result.command != COMMAND_RETURN_BLOCK) ball_state = initialization; //if initialization somehow went wrong
      else {
        ball_current_size = (static_cast<float>(result.width) + static_cast<float>(result.height)) / 2.0;
        ball_current_distance = (ball_size_10 * 10) / ball_current_size;
        ball_location = {result.xCenter, result.yCenter};
        Serial.print(F("Ball distance: "));
        Serial.println(ball_current_distance);
        Serial.print(F("Location: ("));
        Serial.print(ball_location.x);
        Serial.print(F(", "));
        Serial.print(ball_location.y);
        Serial.println(F(")"));
        ball_state = finish; //successfully found the distance, so go to finish
      }
    } else if (ball_state == finish) {
      ball_state = start;
      PT_SLEEP(pt, 1000);
      //recalculate after 1 second
    } else {
      //ball_state doesn't match anything
      ball_state = initialization;
    }
  }
  PT_END(pt);
}
