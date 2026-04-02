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
      huskylens.writeAlgorithm(ALGORITHM_OBJECT_TRACKING);
      ball_state = test_for_object;
      //initialize everything and test for ball
    } else if (ball_state == test_for_object) {
      //Serial.println("Ball started!");
      //failure leads to checking the time
      if (!huskylens.request()) ball_state = check_time;
      else if (!huskylens.isLearned()) ball_state = check_time;
      else if (!huskylens.available()) ball_state = check_time;
      else ball_state = evaluate_distance;
      //otherwise find the distance
    } else if (ball_state == check_time) {
      if (millis() - ball_start_millis > MAX_MILLISECONDS) {
        //change the confidence bit to 0 (not confident)
        ipc_comms &= ~0b1;
        //set the count to 0 so no other threads can view
        sem_ball.count = 0;
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
        ball_results.object_distance = ball_current_distance;
        ball_results.object_size = ball_current_size;
        ball_results.object_location = ball_location;
        ball_results.is_most_recent = true;
        Serial.print("Ball distance: ");
        Serial.println(ball_current_distance);
        Serial.print("Location: (");
        Serial.print(ball_location.x);
        Serial.print(", ");
        Serial.print(ball_location.y);
        Serial.println(")");
        //change the confidence bit to 1 (confident)
        ipc_comms |= 0b1;
        //allow other threads to read the results struct
        PT_SEM_SIGNAL(pt, &sem_ball);
        ball_state = finish; //successfully found the distance, so go to finish
      }
    } else if (ball_state == finish) {
      //debug prints for the semaphore and shared flags
      Serial.print("Ball semaphore value: ");
      Serial.println(sem_ball.count);
      Serial.print("Ball confidence: ");
      Serial.println(ipc_comms & 1);
      ball_state = start;
      PT_SLEEP(pt, 1000);
      //recalculate after 1 second
    } else {
      //change the confidence bit to 0 (not confident)
      ipc_comms &= ~0b1;
      //set the count to 0 so no other threads can view
      sem_ball.count = 0;
      //ball_state doesn't match anything
      ball_state = initialization;
    }
  }
  PT_END(pt);
}
