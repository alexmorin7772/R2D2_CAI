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
      //Serial.println("Goal started!");
      //failure leads to checking the time
      if (!huskylens.request()) goal_state = check_time;
      else if (!huskylens.isLearned()) goal_state = check_time;
      else if (!huskylens.available()) goal_state = check_time;
      else goal_state = evaluate_distance;
      //otherwise find the distance
    } else if (goal_state == check_time) {
      if (millis() - goal_start_millis > MAX_MILLISECONDS) {
        PT_SEM_WAIT(pt, &sem_goal);
        //change the confidence bit to 0 (not confident)
        ipc_comms &= ~GOAL_CONFIDENCE;
        goal_results.is_most_recent = false;
        goal_results.object_found = false;
        PT_SEM_SIGNAL(pt, &sem_goal);
        //went over time limit, so go to finish
        goal_state = finish;
      } else goal_state = test_for_object;
    } else if (goal_state == evaluate_distance) {
      HUSKYLENSResult result = huskylens.read();
      if (result.command != COMMAND_RETURN_BLOCK) goal_state = initialization; //if initialization somehow went wrong
      else {
        goal_height = static_cast<float>(result.height);
        goal_current_distance = (GOAL_SIZE_30 * 30) / goal_height;
        goal_location = {result.xCenter, result.yCenter};
        goal_turn_angle = angle_finder(result.xCenter);
        goal_leftmost_x = result.xCenter - result.width / 2;
        goal_rightmost_x = result.xCenter + result.width / 2;
        Serial.print("Goal distance: ");
        Serial.println(goal_current_distance);
        Serial.print("Location: (");
        Serial.print(goal_location.x);
        Serial.print(", ");
        Serial.print(goal_location.y);
        Serial.println(")");
        Serial.print("Goal angle: ");
        Serial.println(goal_turn_angle * RAD_TO_DEG);
        //take control of the results struct
        PT_SEM_WAIT(pt, &sem_goal);
        update_goal_results(goal_results);
        //change the confidence bit to 1 (confident)
        ipc_comms |= GOAL_CONFIDENCE;
        //allow other threads to read the results struct
        PT_SEM_SIGNAL(pt, &sem_goal);
        goal_state = finish; //successfully found the distance, so go to finish
      }
    } else if (goal_state == finish) {
      //debug prints for the semaphore and shared flags
      Serial.print("Goal semaphore value: ");
      Serial.println(sem_goal.count);
      Serial.print("Goal confidence: ");
      Serial.println(static_cast<bool>(ipc_comms & GOAL_CONFIDENCE));
      goal_state = start;
      PT_SLEEP(pt, 1000);
      //recalculate after 1 second
    } else {
      PT_SEM_WAIT(pt, &sem_goal);
      //change the confidence bit to 0 (not confident)
      ipc_comms &= ~GOAL_CONFIDENCE;
      goal_results.is_most_recent = false;
      goal_results.object_found = false;
      PT_SEM_SIGNAL(pt, &sem_goal);
      //goal_state doesn't match anything
      goal_state = initialization;
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
