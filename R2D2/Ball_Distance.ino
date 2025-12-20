int ball_distance(struct pt* pt) {
  PT_BEGIN(pt);
  for (;;) {
    huskylens.writeAlgorithm(ALGORITHM_OBJECT_TRACKING);
    if (!huskylens.request()) {
      Serial.println("Request ERROR!");
      PT_SLEEP(pt, 10);
      ball_success = false;
      continue;
      //Wait 10 ms and try again
    } else if (!huskylens.isLearned()) {
      Serial.println("Learn ERROR!");
      PT_SLEEP(pt, 10);
      ball_success = false;
      continue;
      //Wait 10 ms and try again
    } else if (!huskylens.available()) {
      Serial.println("Availability ERROR!");
      PT_SLEEP(pt, 10);
      ball_success = false;
      continue;
      //Wait 10 ms and try again
    } else {
      HUSKYLENSResult result = huskylens.read();
      if (result.command != COMMAND_RETURN_BLOCK) {
        huskylens.writeAlgorithm(ALGORITHM_OBJECT_TRACKING);
        PT_SLEEP(pt, 10);
        ball_success = false;
        continue;
        //Wait 10 ms and try again
      }
      ball_current_size = (static_cast<float>(result.width) + static_cast<float>(result.height)) / 2.0;
      ball_current_distance = (ball_size_10 * 10) / ball_current_size;
      Serial.println(ball_current_distance);
    }
    ball_success = true;
    PT_SLEEP(pt, 1000);
    //Recalculate after 100 ms
  }
  PT_END(pt);
}
