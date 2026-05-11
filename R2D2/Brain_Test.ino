#include "protothreads.h"
#include "SoftwareSerial.h"
#include "HUSKYLENS.h"
#include <Wire.h>
#include <Digital_Light_TSL2561.h>
#include <Servo.h>
#include <SparkFun_TB6612.h>
#include "pt-sem.h"

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

// Inter-Process Communication (IPC) variables
volatile uint32_t ipc_comms = 0; 
static struct pt_sem sem_ipc;      // Protects ipc_comms

// Motor Data Payload
struct MotorData {
  float targetX;
  float targetAngle;
  Modes opState; //enum state, so it will be a number
} motorData;
static struct pt_sem semMotor;    // Protects motor data payload

struct object_recognition_results {
  location object_location;
  //stores the x and y coordinates of the object (do object_location.x or object_location.y to get the x and y values)
  float object_distance;
  float object_size;
  float object_turn_angle;
  bool is_most_recent = false; //this will be changed to true once the Huskylens writes to it
  //once someone reads it, it should be set to false so the same information isn't used again
  bool object_found = false;
};


Modes runMode;
float degA;
const int CENTER_X = 160; 
const int OFFSET_X = 40; //random numbers
bool bBetweenPosts = false;
bool bCenteredOnGoal = false;
bool bGoalAligned = false;



pt ptBrain;
pt ptActuator;

enum Modes {
    rForward = 0,
    rBackward,
    rRotateL,
    rRotateR,
    rPivotFwd,
    rPivotRev,
    rTurnLeft,
    rTurnRight,
    rDone
};

// States for simulating for Main protothread's logic/brain execution.
enum brainState {
  msKick = 0,   // State to do one-time initialization or full restart of the state machine
  msSearch,  //Waits for signal from bit flags and gets data from structs
  msAlign,
  msApproach,
  msReverse,
  msScore
} brainState;

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
        bBetweenPosts = (goal_results.leftmost_x < CENTER_X) && (goal_results.rightmost_x > CENTER_X);
        bCenteredOnGoal = (abs(goal_results.object_location.x - CENTER_X) <= OFFSET_X);
        bGoalAligned = bBetweenPosts || bCenteredOnGoal;

        if (!ball_results.object_found) && (bPresent) && (goal_results.object_distance < 10) && (bGoalAligned) {
            brainState = msKick;
        }
        else if ((!ball_results.object_found) && (!bPresent)) {
            brainState = msSearch;
        } 
        else if ((!ball_results.object_found) && (bPresent) && (goal_results.object_found)) {
            brainState = msGoalAlign
        }
        else if ((!ball_results.object_found) && (bPresent) && (!goal_results.object_found)) {
            brainState = msSearch;
        }
        //no ball spotted after doing a full revolution?
        else if ((ball_results.object_found) && (ball_results.object_location.x != CENTER_X)) {
            brainState = msBallAlign;
        }
        else if ((ball_results.object_found) && (ball_results.object_location.x == CENTER_X)) {
            brainState = msBallApproach;
        }
        else if ((!ball_results.object_found) && (bPresent) && (goal_results.object_found) && (!bGoalAligned) {
            brainState = msGoalAlign;
        }
        else if ((!ball_results.object_found) && (bPresent) && (goal_results.object_found) && (bGoalAligned) {}
            brainState = msGoalApproach;

        PT_SLEEP(pt, 1);  // Sleep after each decision cycle
    }
    PT_END(pt);
}

//Actuator Thread: State Execution
int threadActuator(struct pt *pt) {
    PT_BEGIN(pt);
    for(;;) {
        if (brainState == msKick) {
            //activate solenoid
        } 
        else if (brainState == msSearch) {
            PT_SEM_WAIT(pt, &semMotor);
            MotorData.opState = rRotateL;
            MotorData.targetAngle = 68.0; //camera can only see 68 ig oof
            PT_SEM_SIGNAL(pt, &semMotor);
        } 
        else if (brainState == msBallAlign) {
            PT_SEM_WAIT(pt, &semMotor) {
            degA = ball_results.object_turn_angle * RAD_TO_DEG; //agree on this (done 5/9)
            if (degA < 0){
                MotorData.opState = rRotateL;
            } else {
                MotorData.opState = rRotateR;
            }
            MotorData.targetAngle = abs(degA); 
            PT_SEM_SIGNAL(pt, &semMotor);
        } 
        else if(brainState == msGoalAlign) {
            PT_SEM_WAIT(pt, &semMotor) {
            degA = goal_results.object_turn_angle * RAD_TO_DEG; //agree on this (done 5/9)
            if (degA < 0){
                MotorData.opState = rRotateL;
            } else {
                MotorData.opState = rRotateR;
            }
            MotorData.targetAngle = abs(degA); 
            PT_SEM_SIGNAL(pt, &semMotor);
        }
        else if (brainState == msBallApproach) {
            PT_SEM_WAIT(pt, &semMotor);
            MotorData.opState = rForward;
            ball_distance = ball_results.object_distance * 10;
            MotorData.targetX = ball_distance;
            PT_SEM_SIGNAL(pt, &semMotor);
        }
        else if (brainState == msGoalApproach) {
            PT_SEM_WAIT(pt, &semMotor);
            MotorData.opState = rForward;
            goal_distance = goal_results.object_distance * 10;
            MotorData.targetX = goal_distance;
            PT_SEM_SIGNAL(pt, &semMotor);
        }
        PT_SLEEP(pt, 1);
    }
    PT_END(pt);
}

void setup() {
  Serial.begin(115200);
  PT_INIT(&ptBrain);
  PT_INIT(&ptKick);
  PT_SEM_INIT(&sem_ipc, 1);
  PT_SEM_INIT(&semMotor, 1);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  PT_SCHEDULE(threadBrain(&ptBrain));
  PT_SCHEDULE(threadKick(&ptKick));
}
