int eyelidThread(struct pt* pt) {
  PT_BEGIN(pt);
  for(;;) {
    for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      PT_SLEEP(pt, 15);                       // waits 15 ms for the servo to reach the position
    }
    for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      PT_SLEEP(pt, 15);                       // waits 15 ms for the servo to reach the position
    }
  }
  PT_END(pt);
}