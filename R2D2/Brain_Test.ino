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
static struct pt_sem semIPC;      // Protects ipc_comms

// Motor Data Payload
struct MotorData {
  float targetX;
  float targetAngle;
  Modes opState; //enum state, so it will be a number
} motorData;
static struct pt_sem semMotor;    // Protects motor data payload

struct object_recognition_results {
  bool is_most_recent = false; //this will be changed to true once the Huskylens writes to it
  //once someone reads it, it should be set to false so the same information isn't used again
  float object_distance;
  float object_size;
  float object_turn_angle;
  location object_location;
  //stores the x and y coordinates of the object (do object_location.x or object_location.y to get the x and y values)
};

// Handshake Flags
volatile bool bMotor = false;     // Local flag to trigger slave thread
//static struct pt_sem semWorker;    // Protects bmotor and slave data
volatile bool bRunOnce = false;

//global variables
static uint32_t COPY_SPEED;
float tempX;
float tempA;
Modes runMode;
static uint32_t TmrStart;
static uint32_t TmrDur;
static uint32_t StartTime;

pt ptBrain;
pt ptKick;

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
  msInit = 0,   // State to do one-time initialization or full restart of the state machine
  msSearch,  //Waits for signal from bit flags and gets data from structs
  msAlign,
  msApproach,
  msKick,
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

static int threadBrain(struct pt* pt) {
  PT_BEGIN(pt);
  
  brainState = msSearch;

  for(;;) {
    // 1. Wait for fresh data from the camera
    PT_WAIT_UNTIL(pt, cameraData.is_most_recent == true);
    
    // 2. Logic: Decide what to do based on camera values
    PT_SEM_WAIT(pt, &semMotor);
    
    if (cameraData.object_size <= 0) {
      brainState = msSearch;
    } 
    else if (abs(cameraData.object_location.x - CENTER_X) > X_DEADZONE) {
      brainState = msAlign;
    } 
    else if (cameraData.object_distance > KICK_DISTANCE) {
      brainState = msApproach;
    } 
    else {
      brainState = msKick;
    }

    // 3. Execution: Command the Motors or the Kicker
    // Check if the motor is already doing something (unless we need to override)
    bool isMoving = (ipc_comms & MASK_MOTOR_MOVING);

    switch (brainState) {
      case msSearch:
        if (!isMoving) {
          motorData.opState = rRotateR;
          motorData.targetAngle = cameraData.object_turn_angle; //random numbers
          ipc_comms |= MASK_IPC_Brain_To_Motor;
        }
        break;

      case msAlign:
        // random OVERRIDE case to test
        motorData.targetAngle = cameraData.object_turn_angle; //random numbers
        motorData.opState = (cameraData.object_location.x < CENTER_X) ? rRotateL : rRotateR;
        ipc_comms |= MASK_OVERRIDE_FLAG; 
        break;

      case msApproach:
        if (!isMoving) {
          motorData.opState = rForward;
          motorData.targetX = cameraData.object_distance;
          ipc_comms |= MASK_IPC_Brain_To_Motor;
        }
        break;

      case msKick:
        // Stop the motors and trigger the kick flag
        brake(motor1, motor2);
        bKick_Start = true; // Signals the threadKick below
        break;
    }

    cameraData.is_most_recent = false; // "Consume" the frame
    PT_SEM_SIGNAL(pt, &semMotor);

    PT_SLEEP(pt, 1);
  }
  PT_END(pt);
}

// --- Kicker Thread (based off of alex's thread) ---
static int threadKick(struct pt* pt) {
  PT_BEGIN(pt);

  for(;;) {
    // Wait until the Brain sets bKick_Start to true
    PT_WAIT_UNTIL(pt, bKick_Start);
    
    Serial.println(">>> KICKING <<<");
    PORTB |= (1 << 4); // Solenoid on

    StartTime = millis();
    PT_WAIT_UNTIL(pt, (millis() - StartTime) >= 200); //random numbers kinda
    
    PORTB &= ~(1 << 4); // Solenoid off
    bKick_Start = false; // Reset flag
    
    StartTime = millis();
    PT_WAIT_UNTIL(pt, (millis() - StartTime) >= 500); //safety cooldowning?
  }

  PT_END(pt);
}

void setup() {
  Serial.begin(115200);
  PT_INIT(&ptBrain);
  PT_INIT(&ptKick);
  PT_SEM_INIT(&semIPC, 1);
  PT_SEM_INIT(&semMotor, 1);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  PT_SCHEDULE(threadBrain(&ptBrain));
  PT_SCHEDULE(threadKick(&ptKick));
}