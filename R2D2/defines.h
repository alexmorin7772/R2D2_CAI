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
