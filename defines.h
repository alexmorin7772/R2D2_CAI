#pragma once

/*
typedef enum motorModes {
    rForward = 0,
    rBackward,
    rRotateL,
    rRotateR,
    rDone
} runMode_t;

// States for simulating for Main protothread's logic/brain execution.
typedef enum BRAIN_STATES {
  msKick = 0,   // State to do one-time initialization or full restart of the state machine
  msBallSearch,  //Waits for signal from bit flags and gets data from structs
  msGoalSearch,
  msGoalAlign,
  msBallAlign,
  msGoalApproach,
  msBallApproach
} Brain_States;
*/
typedef enum motor_action {
  drive_forward,
  drive_backward, //not sure if we will need this or not
  turn_left,
  turn_right,
  idle
} runMode_t;

//data the motor uses
struct motor_data {
  float object_distance;
  float turn_angle;
  motor_action action;
};