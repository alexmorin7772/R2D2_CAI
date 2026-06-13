lens_adjustment_state lens_state = lens_start;

/*
Please note that cos^2(angle) means cos(angle) * cos(angle), NOT cos(cos(angle))
Formula for amount of light let through: lux * cos^2(angle)
Formula for actual light let through: lux / cos^2(angle)
Formula for angle: arccos(sqrt(wanted_lux / lux))

The angle that I have is currently relative to the servo
An angle of 180 means that it's as open as possible
An angle of 0 means that it's completely closed

To convert to the angle that's used in Malus' law, do (180 - angle) / 2
*/

int lens_adjustment(struct pt* pt) {
  PT_BEGIN(pt);
  for(;;) {
    //Serial.print("Lens adjustment state: ");
    //Serial.println(lens_state);
    if (lens_state == lens_start) {
      lens_state = read_lux;
    } else if (lens_state == read_lux) {
      lux = TSL2561.readVisibleLux();
      //Serial.print(F("Current lux: "));
      //Serial.println(lux);
      actual_lux = lux / (cos((180 - angle) / 2.0 * DEG_TO_RAD) * cos((180 - angle) / 2.0 * DEG_TO_RAD));
      //Serial.print(F("Actual lux: "));
      //Serial.println(actual_lux);
      lens_state = calculate_angle;
    } else if (lens_state == calculate_angle) {
      if (GOAL_LUX >= actual_lux) angle = 180;
      else angle = 180 - 2 * arccos(sqrt(static_cast<float>(GOAL_LUX) / static_cast<float>(actual_lux)));
      //Serial.print(F("Angle: "));
      //Serial.println(angle);
      lens_state = move_servo;
    } else if (lens_state == move_servo) {
      myservo.write(angle);
      lens_state = lens_finish;
    } else if (lens_state == lens_finish) {
      lens_state = lens_start;
      PT_SLEEP(pt, 1000);
    } else {
      lens_state = lens_start;
    }
  }
  PT_END(pt);
}