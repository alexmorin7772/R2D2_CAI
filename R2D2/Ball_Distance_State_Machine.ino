object_recognition_state ball_state = start;
object_recognition_results ball_results;
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
      huskylens.writeAlgorithm(ALGORITHM_COLOR_RECOGNITION);
      ball_state = test_for_object;
      //initialize everything and test for ball
    } else if (ball_state == test_for_object) {
      //Serial.println("Ball started!");
      //failure leads to checking the time
      if (!huskylens.request() or !huskylens.countBlocks(BALL_ID)) ball_state = check_time;
      else ball_state = evaluate_distance;
      //otherwise find the distance
    } else if (ball_state == check_time) {
      if (millis() - ball_start_millis > MAX_MILLISECONDS) {
        PT_SEM_WAIT(pt, &sem_ball);
        //change the confidence bit to 0 (not confident)
        ipc_comms &= ~BALL_CONFIDENCE;
        ball_results.is_most_recent = false;
        ball_results.object_found = false;
        PT_SEM_SIGNAL(pt, &sem_ball);
        //went over time limit, so go to finish
        ball_state = finish;
      } else ball_state = test_for_object;
    } else if (ball_state == evaluate_distance) {
      HUSKYLENSResult result = huskylens.getBlock(BALL_ID, 0); //the 0 means the first block with that ID
      ball_current_size = (static_cast<float>(result.width) + static_cast<float>(result.height)) / 2.0;
      ball_current_distance = (BALL_SIZE_10 * 10) / ball_current_size;
      ball_location = {result.xCenter, result.yCenter};
      ball_turn_angle = angle_finder(result.xCenter);
      ball_leftmost_x = result.xCenter - result.width / 2;
      ball_rightmost_x = result.xCenter + result.width / 2;
      Serial.print("Ball distance: ");
      Serial.println(ball_current_distance);
      Serial.print("Location: (");
      Serial.print(ball_location.x);
      Serial.print(", ");
      Serial.print(ball_location.y);
      Serial.println(")");
      Serial.print("Ball angle: ");
      Serial.println(ball_turn_angle * RAD_TO_DEG);
      //take control of the results struct
      PT_SEM_WAIT(pt, &sem_ball);
      update_ball_results(ball_results);
      //change the confidence bit to 1 (confident)
      ipc_comms |= BALL_CONFIDENCE;
      //allow other threads to read the results struct
      PT_SEM_SIGNAL(pt, &sem_ball);
      ball_state = finish; //successfully found the distance, so go to finish
    } else if (ball_state == finish) {
      //debug prints for the semaphore and shared flags
      Serial.print("Ball semaphore value: ");
      Serial.println(sem_ball.count);
      Serial.print("Ball confidence: ");
      Serial.println(ipc_comms & BALL_CONFIDENCE);
      ball_state = start;
      PT_SLEEP(pt, 1000);
      //recalculate after 1 second
    } else {
      PT_SEM_WAIT(pt, &sem_ball);
      //change the confidence bit to 0 (not confident)
      ipc_comms &= ~BALL_CONFIDENCE;
      ball_results.is_most_recent = false;
      ball_results.object_found = false;
      PT_SEM_SIGNAL(pt, &sem_ball);
      //ball_state doesn't match anything
      ball_state = initialization;
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
