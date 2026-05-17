#include <Arduino.h>
#include "HUSKYLENS.h"
#include "protothreads.h"
#include "MyObjRec.h"
#include "defines.h"
#include "Servo.h"

extern HUSKYLENS huskylens;
extern float ball_current_distance;
extern float ball_current_size;
extern location ball_location;
extern float ball_turn_angle;
extern int ball_leftmost_x;
extern int ball_rightmost_x;

location goal_location;

float goal_current_distance;
float goal_height;
float goal_size_30 = 100.0;
bool goal_success;

object_recognition_state goal_state = start;
unsigned long goal_start_millis;

int goal_distance(struct pt* pt) {
  PT_BEGIN(pt);
  for (;;) {
    //Serial.print("Goal state: ");
    //Serial.println(goal_state);
    if (goal_state == start) {
      goal_start_millis = millis();
      goal_state = initialization;
      //have to go to initialization
    } else if (goal_state == initialization) {
      huskylens.writeAlgorithm(ALGORITHM_COLOR_RECOGNITION);
      goal_state = test_for_object;
      //initialize everything and test for goal
    } else if (goal_state == test_for_object) {
      //failure leads to checking the time
      if (!huskylens.request()) goal_state = check_time;
      else if (!huskylens.isLearned()) goal_state = check_time;
      else if (!huskylens.available()) goal_state = check_time;
      else goal_state = evaluate_distance;
      //otherwise find the distance
    } else if (goal_state == check_time) {
      if (millis() - goal_start_millis > max_milliseconds) {
        //went over time limit, so go to finish
        goal_state = finish;
      } else goal_state = test_for_object;
    } else if (goal_state == evaluate_distance) {
      HUSKYLENSResult result = huskylens.read();
      if (result.command != COMMAND_RETURN_BLOCK) goal_state = initialization; //if initialization somehow went wrong
      else {
        goal_height = static_cast<float>(result.height);
        goal_current_distance = (goal_size_30 * 30) / goal_height;
        goal_location = {result.xCenter, result.yCenter};
        Serial.print("Goal distance: ");
        Serial.println(goal_current_distance);
        Serial.print("Location: (");
        Serial.print(goal_location.x);
        Serial.print(", ");
        Serial.print(goal_location.y);
        Serial.println(")");
        goal_state = finish; //successfully found the distance, so go to finish
      }
    } else if (goal_state == finish) {
      goal_state = start;
      PT_SLEEP(pt, 1000);
      //recalculate after 1 second
    } else {
      //goal_state doesn't match anything
      goal_state = initialization;
    }
  }
  PT_END(pt);
}

void update_ball_results(object_recognition_results& ball_results) {
  ball_results.object_distance = ball_current_distance;
  ball_results.object_size = ball_current_size;
  ball_results.object_location = ball_location;
  ball_results.object_turn_angle = ball_turn_angle;
  ball_results.leftmost_x = ball_leftmost_x;
  ball_results.rightmost_x = ball_rightmost_x;
  ball_results.is_most_recent = true;
  ball_results.object_found = true;
}
