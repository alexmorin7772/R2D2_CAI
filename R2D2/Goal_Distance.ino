/*int goal_distance(struct pt* pt) {
  PT_BEGIN(pt);
  for (;;) {
    huskylens.writeAlgorithm(ALGORITHM_COLOR_RECOGNITION);
    if (!huskylens.request()) {
      Serial.println(F("Request ERROR!"));
      PT_SLEEP(pt, 10);
      goal_success = false;
      continue;
      //Wait 10 ms and try again
    } else if (!huskylens.isLearned()) {
      Serial.println(F("Learn ERROR!"));
      PT_SLEEP(pt, 10);
      goal_success = false;
      continue;
      //Wait 10 ms and try again
    } else if (!huskylens.available()) {
      Serial.println(F("Availability ERROR!"));
      PT_SLEEP(pt, 10);
      goal_success = false;
      continue;
      //Wait 10 ms and try again
    } else {
      HUSKYLENSResult result = huskylens.read();
      if (result.command != COMMAND_RETURN_BLOCK) {
        huskylens.writeAlgorithm(ALGORITHM_COLOR_RECOGNITION);
        PT_SLEEP(pt, 10);
        goal_success = false;
        continue;
        //Wait 10 ms and try again
      }
      goal_height = static_cast<float>(result.height);
      goal_current_distance = (goal_size_30 * 30) / goal_height;
      Serial.println(goal_current_distance);
    }
    PT_SLEEP(pt, 1000);
    goal_success = true;
    //Recalculate after 1 second
  }
  PT_END(pt);
}*/
