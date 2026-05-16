object_recognition_state goal_state = start;
//object_recognition_results goal_results;
unsigned long goal_start_millis;

int goal_distance(struct pt* pt) {
  PT_BEGIN(pt);
  for (;;) {
   if (goal_state == start) {
      goal_state = test_for_object;
      //once we start, test if the object is there
    } else if (goal_state == test_for_object) {
     if (!huskylens.request() or !huskylens.countBlocks(GOAL_ID)) goal_state = fail;
      else goal_state = evaluate_distance;
      //otherwise find the distance
    } else if (goal_state == evaluate_distance) {
      HUSKYLENSResult result = huskylens.getBlock(GOAL_ID, 0); //the 0 means the first block with that ID
      goal_height = static_cast<float>(result.height);
      goal_current_distance = (GOAL_SIZE_30 * 30) / goal_height;
      goal_location = {result.xCenter, result.yCenter};
      goal_turn_angle = angle_finder(result.xCenter);
      goal_leftmost_x = result.xCenter - result.width / 2;
      goal_rightmost_x = result.xCenter + result.width / 2;
      Serial.print(F("Goal distance: "));
      Serial.println(goal_current_distance);
      Serial.print(F("Location: ("));
      Serial.print(goal_location.x);
      Serial.print(F(", "));
      Serial.print(goal_location.y);
      Serial.println(F(")"));
      Serial.print(F("Goal angle: "));
      Serial.println(goal_turn_angle * RAD_TO_DEG);
      //take control of the results struct
      PT_SEM_WAIT(pt, &sem_goal);
      update_goal_results(goal_results);
      //change the confidence bit to 1 (confident)
      ipc_comms |= GOAL_CONFIDENCE;
      //allow other threads to read the results struct
      PT_SEM_SIGNAL(pt, &sem_goal);
      goal_state = finish; //successfully found the distance, so go to finish
   } else if (goal_state == finish) {
      //debug prints for the semaphore and shared flags
      Serial.print(F("Goal semaphore value: "));
      Serial.println(sem_goal.count);
      Serial.print(F("Goal confidence: "));
      Serial.println(static_cast<bool>(ipc_comms & GOAL_CONFIDENCE));
      goal_state = start;
      PT_SLEEP(pt, 1);
      //recalculate after 1 millisecond
    } else {
      PT_SEM_WAIT(pt, &sem_goal);
      //change the confidence bit to 0 (not confident)
      ipc_comms &= ~GOAL_CONFIDENCE;
      goal_results.is_most_recent = false;
      goal_results.object_found = false;
      PT_SEM_SIGNAL(pt, &sem_goal);
      //goal_state doesn't match anything
      goal_state = start;
    }
  }
  PT_END(pt);
}

void update_goal_results(object_recognition_results& goal_results) {
  goal_results.object_distance = goal_current_distance;
  goal_results.object_size = goal_height;
  goal_results.object_location = goal_location;
  goal_results.object_turn_angle = goal_turn_angle;
  goal_results.leftmost_x = goal_leftmost_x;
  goal_results.rightmost_x = goal_rightmost_x;
  goal_results.is_most_recent = true;
  goal_results.object_found = true;
}
