bool is_on(float voltage) {
  if (abs(ON - voltage) <= abs(OFF - voltage)) return true;
  return false;
  //if the voltage is closer to on than off, return true
}

void update_start_stop() {
  float input1 = analogRead(OUT1) * (ANALOG_RANGE / ANALOG_MAX);
  float input2 = analogRead(OUT2) * (ANALOG_RANGE / ANALOG_MAX);
  float average_input = (input1 + input2) / 2;
  //get the average of the two inputs to use
  if (is_on(average_input)) started = true;
  else started = false;
}
