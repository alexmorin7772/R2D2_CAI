line_tracking_state line_state = line_start;
line_tracking_results line_results;
unsigned long line_start_millis;

int line_tracking(struct pt* pt) {
  PT_BEGIN(pt);
  for (;;) {
    //Serial.print("Line state: ");
    //Serial.println(line_state);
    if (line_state == line_start) {
      line_start_millis = millis();
      line_state = line_initialization;
      //have to go to initialization
    } else if (line_state == line_initialization) {
      huskylens.writeAlgorithm(ALGORITHM_LINE_TRACKING);
      line_state = test_for_line;
      //initialize everything and test for line
    } else if (line_state == test_for_line) {
      //Serial.println("Line started!");
      //failure leads to checking the time
      if (!huskylens.request()) line_state = line_check_time;
      else if (!huskylens.isLearned()) line_state = line_check_time;
      else if (!huskylens.available()) line_state = line_check_time;
      else line_state = process_line;
      //otherwise find the distance
    } else if (line_state == line_check_time) {
      if (millis() - line_start_millis > MAX_MILLISECONDS) {
        PT_SEM_WAIT(pt, &sem_line);
        //change the confidence bit to 0 (not confident)
        ipc_comms &= ~LINE_CONFIDENCE;
        line_results.is_most_recent = false;
        PT_SEM_SIGNAL(pt, &sem_line);
        //went over time limit, so go to finish
        line_state = line_finish;
      } else line_state = test_for_line;
    } else if (line_state == process_line) {
      HUSKYLENSResult result = huskylens.read();
      if (result.command != COMMAND_RETURN_ARROW) line_state = line_initialization; //if initialization somehow went wrong
      else {
        line_origin = {result.xOrigin, result.yOrigin};
        line_target = {result.xTarget, result.yTarget};
        Serial.print(F("Line origin: ("));
        Serial.print(line_origin.x);
        Serial.print(", ");
        Serial.print(line_origin.y);
        Serial.println(")");
        Serial.print(F("Line target: ("));
        Serial.print(line_target.x);
        Serial.print(", ");
        Serial.print(line_target.y);
        Serial.println(")");
        //take control of the results struct
        PT_SEM_WAIT(pt, &sem_line);
        line_results.origin = line_origin;
        line_results.target = line_target;
        line_results.is_most_recent = true;
        //change the confidence bit to 1 (confident)
        ipc_comms |= LINE_CONFIDENCE;
        //allow other threads to read the results struct
        PT_SEM_SIGNAL(pt, &sem_line);
        line_state = line_finish; //successfully found the distance, so go to finish
      }
    } else if (line_state == line_finish) {
      //debug prints for the semaphore and shared flags
      Serial.print(F("Line semaphore value: "));
      Serial.println(sem_line.count);
      Serial.print(F("Line confidence: "));
      Serial.println(ipc_comms & LINE_CONFIDENCE);
      line_state = line_start;
      PT_SLEEP(pt, 1000);
      //recalculate after 1 second
    } else {
      PT_SEM_WAIT(pt, &sem_line);
      //change the confidence bit to 0 (not confident)
      ipc_comms &= ~LINE_CONFIDENCE;
      line_results.is_most_recent = false;
      PT_SEM_SIGNAL(pt, &sem_line);
      //line_state doesn't match anything
      line_state = line_initialization;
    }
  }
  PT_END(pt);
}
