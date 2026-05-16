#include <SparkFun_TB6612.h>
#include "defines.h"

#define MASK_IPC_Brain_Read_Confirmation   0x00040000
#define MASK_IPC_Touch_To_Brain            0x00080000
#define MASK_IPC_Touch_Read_Confirmation   0x00400000
#define MASK_IPC_Brain_To_Touch            0x00800000

#define MASK_IPC_Motor_Read_Confirm     0x00000100
#define MASK_IPC_Brain_To_Motor 0x00001000
#define MASK_MOTOR_MOVING 0x00000200
#define MASK_OVERRIDE_SEEN 0x00000400
#define MASK_OVERRIDE_FLAG 0x00002000
#define MASK_SPEED 0x00004000

//numbers to multiply speed by

#define CENTER_X 160
#define GOAL_OFFSET_X 40

#define BALL_ANGLE_OFFSET 0.09 //0.09 radians is about 5 degrees
#define MAX_GOAL_DISTANCE 20 //maximum distance the goal can be away for us to kick

//pt_sem sem_ipc_comms, sem_motor;

//PT_SEM_INIT(&sem_ipc_comms, 1);
//make sure this goes into void setup


//initialize motors
//Motor motor1 = Motor(AIN1, AIN2, PWMA, OFFSET_A, STBY);
//Motor motor2 = Motor(BIN1, BIN2, PWMB, OFFSET_B, STBY);
/*
//possible actions the motors could be doing
enum motor_action {
  drive_forward,
  drive_backward, //not sure if we will need this or not
  turn_left,
  turn_right,
  idle
};

//data the motor uses
struct motor_data {
  float object_distance;
  float turn_angle;
  motor_action action;
};*/

extern volatile boolean bPresent; //this was just defined so the code would compile without needing Alex's code
//be sure to delete once we merge

bool between_posts = false;
bool centered_on_goal = false;
bool aligned_on_goal = false;
bool aligned_on_ball = false;

enum motor_state {
  kick,
  search,
  align_on_ball,
  align_on_goal,
  go_to_ball,
  go_to_goal,
};

motor_state current_motor_state = search; //set default to search for ball
motor_data current_motor_data = {-1, -1, idle}; //start by being idle

int update_motor_state(struct pt *pt) {
  PT_BEGIN(pt);
  for(;;) {
    Serial.println(current_motor_state);
    between_posts = (goal_results.object_found) && (goal_results.leftmost_x <= CENTER_X) && (goal_results.rightmost_x >= CENTER_X);
    centered_on_goal = (goal_results.object_found) && (abs(goal_results.object_location.x - CENTER_X) <= GOAL_OFFSET_X);
    aligned_on_goal = between_posts;
    //while knowing if we are centered on the goal might be useful, I think being between the posts is more important
    aligned_on_ball = abs(ball_results.object_turn_angle) <= BALL_ANGLE_OFFSET;

    if (!ball_results.object_found && !bPresent) {
      //we don't have/know where the ball is, so we search for it
      current_motor_state = search;
      Serial.println(F("search"));
    } else if (!ball_results.object_found && bPresent && !goal_results.object_found) {
      //we have the ball but have to search for the goal
      current_motor_state = search;
      Serial.println(F("search"));
    } else if (!ball_results.object_found && bPresent && goal_results.object_found && !aligned_on_goal) {
      //we have the ball and see the goal but are not aligned
      current_motor_state = align_on_goal;
      Serial.println(F("align_on_goal"));
    } else if (!ball_results.object_found && bPresent && goal_results.object_found && aligned_on_goal && goal_results.object_distance < MAX_GOAL_DISTANCE) {
      //we have the ball, are aligned on the goal, and are close enough
      current_motor_state = kick;
      Serial.println(F("kick"));
    } else if (!ball_results.object_found && bPresent && goal_results.object_found && aligned_on_goal) {
      //we have and ball and are aligned on the goal
      current_motor_state = go_to_goal;
      //Serial.println(F("go_to_goal"));
    } else if (ball_results.object_found && !bPresent && !aligned_on_ball) {
      //we see the ball but are not aligned
      current_motor_state = align_on_ball;
      //Serial.println(F("align_on_ball"));
    } else if (ball_results.object_found && !bPresent && aligned_on_ball) {
      //we see the ball and are aligned
      current_motor_state = go_to_ball;
      Serial.println(F("go_to_ball"));
    } else {
      Serial.println(F("**************"));
    }
    PT_SLEEP(pt, 1);
  }
  PT_END(pt);
}

int update_motor_data(struct pt *pt) {
  PT_BEGIN(pt);
  for(;;) {
    if (current_motor_state == kick) {
      //activate solenoid here
    } else if (current_motor_state == search) {
      PT_SEM_WAIT(pt, &sem_motor);
      current_motor_data.action = turn_left;
      current_motor_data.turn_angle = 68;
      PT_SEM_SIGNAL(pt, &sem_motor);
    } else if (current_motor_state == align_on_ball) {
      PT_SEM_WAIT(pt, &sem_motor);
      if (ball_results.object_turn_angle < 0) current_motor_data.action = turn_left;
      else current_motor_data.action = turn_right;
      current_motor_data.turn_angle = abs(ball_results.object_turn_angle * RAD_TO_DEG);
      PT_SEM_SIGNAL(pt, &sem_motor);
    } else if (current_motor_state == align_on_goal) {
      PT_SEM_WAIT(pt, &sem_motor);
      if (goal_results.object_turn_angle < 0) current_motor_data.action = turn_left;
      else current_motor_data.action = turn_right;
      current_motor_data.turn_angle = abs(goal_results.object_turn_angle * RAD_TO_DEG);
      PT_SEM_SIGNAL(pt, &sem_motor);
    } else if (current_motor_state == go_to_ball) {
      PT_SEM_WAIT(pt, &sem_motor);
      current_motor_data.action = drive_forward;
      PT_SEM_SIGNAL(pt, &sem_motor);
    } else if (current_motor_state == go_to_goal) {
      PT_SEM_WAIT(pt, &sem_motor);
      current_motor_data.action = drive_forward;
      PT_SEM_SIGNAL(pt, &sem_motor);
    } 
    PT_SLEEP(pt, 1);
  }
  PT_END(pt);
}
