typedef enum OBJECT_RECOGNITION_STATE {
  start,
  initialization,
  test_for_object,
  check_time,
  evaluate_distance,
  finish
} object_recognition_state;

typedef enum SOCCER_STRATEGY_STATE {
   strategy_idle,
   strategy_find_ball,
   strategy_chase_ball,
   strategy_align_goal,
   strategy_drive_to_goal,
   strategy_shoot,
   strategy_retreat
 } soccer_strategy_state;


typedef struct LOCATION {
  int x, y;
} location;

typedef struct OBJECT_RECOGNITION_RESULT {
  location object_location;
  //stores the x and y coordinates of the object (do object_location.x or object_location.y to get the x and y values)
  float object_distance;
  float object_size;
  float object_turn_angle;
  int leftmost_x, rightmost_x;
  bool is_most_recent = false; //this will be changed to true once the Huskylens writes to it
  //once someone reads it, it should be set to false so the same information isn't used again
  bool object_found = false;
} object_recognition_results;

// Movement commands the brain can issue.
typedef enum MOVEMENT_COMMAND {
  CMD_STOP,
  CMD_SEARCH,
  CMD_CHASE,
  CMD_ALIGN,
  CMD_SHOOT
} movement_command;


typedef enum LENS_ADJUSTMENT_STATE {
  lens_start,
  read_lux,
  calculate_angle,
  move_servo,
  lens_finish
} lens_adjustment_state;


typedef enum KICK_COMMAND {
  KICK_IDLE,
  KICK_READY,
  KICK_NOW_MAX,
  KICK_NOW_MED
} kick_command;

// Brain fills this struct; solenoid code reads power/command.
typedef struct KICK_INSTRUCTION {
  kick_command command;
  int          power;
  float        ball_dist;
  bool         ball_confident;
  bool         goal_centered;
} kick_instruction;


typedef enum LINE_TRACKING_STATE {
  line_start,
  line_initialization,
  test_for_line,
  line_check_time,
  process_line,
  line_finish
} line_tracking_state;
