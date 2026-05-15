#include "defines.h"
#include "motorStruct.h"

#define AIN1 2
#define BIN1 7
#define AIN2 4
#define BIN2 8
#define PWMA 5
#define PWMB 6
#define STBY 9

// these constants are used to allow you to make your motor configuration 
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;

// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);


float degA;
const int CENTER_X = 160; 
const int OFFSET_X = 40; //random numbers
bool bBetweenPosts = false;
bool bCenteredOnGoal = false;
bool bGoalAligned = false;
float temp_ball_distance;
float temp_goal_distance;

extern volatile boolean bKick;
extern volatile boolean bPresent;
extern volatile boolean bBeastMode;

runMode_t runMode;
static Brain_States brainState;
static Brain_States prevState;

// States for simulating for Main protothread's logic/brain execution.
/*
typedef enum Brain_States {
  msKick = 0,   // State to do one-time initialization or full restart of the state machine
  msSearch,  //Waits for signal from bit flags and gets data from structs
  msGoalAlign,
  msBallAlign,
  msGoalApproach,
  msBallApproach
};
Brain_States 
*/
#define MASK_IPC_Brain_Read_Confirmation   0x00040000 // same as 0b 0000 0000 0000 0100 0000 0000 0000 0000 - Same as Touch confirming that it read from Brain
#define MASK_IPC_Touch_To_Brain            0x00080000 // same as 0b 0000 0000 0000 1000 0000 0000 0000 0000

#define MASK_IPC_Touch_Read_Confirmation   0x00400000 // same as 0b 0000 0000 0100 0000 0000 0000 0000 0000 - Same as Brain confirming that it read from Touch
#define MASK_IPC_Brain_To_Touch            0x00800000 // same as 0b 0000 0000 1000 0000 0000 0000 0000 0000

// Bitmasks (Example based on your provided hex values)
#define MASK_IPC_Motor_Read_Confirm     0x00000100 //0000 0000 0000 0000 0000 0001 0000 0000
#define MASK_IPC_Brain_To_Motor 0x00001000 //0000 0000 0000 0000 0001 0000 0000 0000

#define MASK_MOTOR_MOVING 0x00000200

#define MASK_OVERRIDE_SEEN 0x00000400
#define MASK_OVERRIDE_FLAG 0x00002000

#define MASK_SPEED 0x00004000 //1 = fast, 0 = slow

int threadBrain(struct pt *pt) {
    PT_BEGIN(pt);
    for(;;) {
        bBetweenPosts = ((goal_results.leftmost_x < CENTER_X) && (goal_results.rightmost_x > CENTER_X));
        bCenteredOnGoal = (abs(goal_results.object_location.x - CENTER_X) <= OFFSET_X);
        bGoalAligned = bBetweenPosts || bCenteredOnGoal;
        if (ball_results.object_found) {
            Serial.println(F("WOW I CAN SEE THE BALL ********"));
        }

        if ((!ball_results.object_found) && (bPresent) && (goal_results.object_distance < 10) && (bGoalAligned)) {
            if (prevState != brainState) {
                Serial.println(F("msKick"));
            }
            prevState = brainState;
            brainState = msKick;
        }
        else if ((!ball_results.object_found) && (!bPresent)) {
            if (prevState != brainState) {
                Serial.println(F("msBallSearch"));
            }
            prevState = brainState;
            brainState = msBallSearch;
        } 
        else if ((!ball_results.object_found) && (bPresent) && (goal_results.object_found)) {
            if (prevState != brainState) {
                Serial.println(F("msAlign"));
            }
            prevState = brainState;
            brainState = msGoalAlign;
        }
        else if ((!ball_results.object_found) && (bPresent) && (!goal_results.object_found)) {
            if (prevState != brainState) {
                Serial.println(F("msGoalSearch"));
            }
            prevState = brainState;
            brainState = msGoalSearch;
        }
        //no ball spotted after doing a full revolution?
        else if ((ball_results.object_found) && (ball_results.object_location.x != CENTER_X)) {
            if (prevState != brainState) {
                Serial.println(F("msBallAlign"));
            }
            prevState = brainState;
            brainState = msBallAlign;
        }
        else if ((ball_results.object_found) && (ball_results.object_location.x == CENTER_X)) {
            if (prevState != brainState) {
                Serial.println(F("msBallApproach"));
            }
            prevState = brainState;
            brainState = msBallApproach;
        }
        else if ((!ball_results.object_found) && (bPresent) && (goal_results.object_found) && (!bGoalAligned)) {
            if (prevState != brainState) {
                Serial.println(F("msGoalAlign"));
            }
            prevState = brainState;
            brainState = msGoalAlign;
        }
        else if ((!ball_results.object_found) && (bPresent) && (goal_results.object_found) && (bGoalAligned)) {
            if (prevState != brainState) {
                Serial.println(F("msGoalApproach"));
            }
            prevState = brainState;
            brainState = msGoalApproach;
        }
        else {
            Serial.println("...");
        }
        PT_SLEEP(pt, 1);  // Sleep after each decision cycle
    }
    PT_END(pt);
}

//Actuator Thread: State Execution
int threadActuator(struct pt *pt) {
    PT_BEGIN(pt);
    for(;;) {
        if (brainState == msKick) {
            Serial.println(F("kicking..."));
            //activate solenoid
        } 
        else if (brainState == msBallSearch) {
            PT_SEM_WAIT(pt, &sem_motor);
            motorData.opState = rRotateL;
            motorData.targetAngle = 68.0; //camera can only see 68 ig oof
            PT_SEM_SIGNAL(pt, &sem_motor);
        } 
        else if (brainState == msGoalSearch) {
            PT_SEM_WAIT(pt, &sem_motor);
            motorData.opState = rRotateL;
            motorData.targetAngle = 68.0; //need to do goal/ball difference to tell camera to prioritize checking of one
            PT_SEM_SIGNAL(pt, &sem_motor);
        }
        else if (brainState == msBallAlign) {
            PT_SEM_WAIT(pt, &sem_motor);
            degA = ball_results.object_turn_angle * RAD_TO_DEG; //agree on this (done 5/9)
            if (degA < 0){
                motorData.opState = rRotateL;
            } else {
                motorData.opState = rRotateR;
            }
            motorData.targetAngle = abs(degA); 
            PT_SEM_SIGNAL(pt, &sem_motor);
        } 
        else if(brainState == msGoalAlign) {
            PT_SEM_WAIT(pt, &sem_motor);
            degA = goal_results.object_turn_angle * RAD_TO_DEG; //agree on this (done 5/9)
            if (degA < 0){
                motorData.opState = rRotateL;
            } else {
                motorData.opState = rRotateR;
            }
            motorData.targetAngle = abs(degA); 
            PT_SEM_SIGNAL(pt, &sem_motor);
        }
        else if (brainState == msBallApproach) {
            PT_SEM_WAIT(pt, &sem_motor);
            motorData.opState = rForward;
            temp_ball_distance = ball_results.object_distance * 10.0;
            motorData.targetX = temp_ball_distance;
            PT_SEM_SIGNAL(pt, &sem_motor);
        }
        else if (brainState == msGoalApproach) {
            PT_SEM_WAIT(pt, &sem_motor);
            motorData.opState = rForward;
            temp_goal_distance = goal_results.object_distance * 10.0;
            motorData.targetX = temp_goal_distance;
            PT_SEM_SIGNAL(pt, &sem_motor);
        }
        PT_SLEEP(pt, 1);
    }
    PT_END(pt);
}


