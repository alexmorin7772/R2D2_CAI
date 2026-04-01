object_recognition_state goal_state = start;
object_recognition_results goal_results;
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
      Serial.println("Goal started!");
      //failure leads to checking the time
      if (!huskylens.request()) goal_state = check_time;
      else if (!huskylens.isLearned()) goal_state = check_time;
      else if (!huskylens.available()) goal_state = check_time;
      else goal_state = evaluate_distance;
      //otherwise find the distance
    } else if (goal_state == check_time) {
      if (millis() - goal_start_millis > MAX_MILLISECONDS) {
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
        goal_results.object_distance = goal_current_distance;
        goal_results.object_size = goal_height;
        goal_results.object_location = goal_location;
        goal_results.is_most_recent = true;
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
