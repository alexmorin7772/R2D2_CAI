/*
//do NOT write to any of these variables except for is_most_recent
struct object_recognition_results {
  location object_location;
  //stores the x and y coordinates of the object (do object_location.x or object_location.y to get the x and y values)
  float object_distance;
  float object_size;
  float object_turn_angle;
  int leftmost_x, rightmost_x;
  bool is_most_recent = false; //this will be changed to true once the Huskylens writes to it
  //once someone reads it, it should be set to false so the same information isn't used again
  bool object_found = false;
};

extern object_recognition_results goal_results;
extern object_recognition_results ball_results;
*/