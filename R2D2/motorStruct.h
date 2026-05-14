// Motor Data Payload
struct motorStruct {
  float targetX;
  float targetAngle;
  runMode_t opState; //enum state, so it will be a number
};

motorStruct motorData;
