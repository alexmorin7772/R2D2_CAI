#include "protothreads.h"
#include "pt-sem.h"
#include "modes.h"
#include <SparkFun_TB6612.h>

// Pins for all inputs, keep in mind the PWM defines must be on PWM pins
// the default pins listed are the ones used on the Redbot (ROB-12097) with
// the exception of STBY which the Redbot controls with a physical switch
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

// Handshake Flags
volatile bool bMotor = false;     // Local flag to trigger slave thread
static struct pt_sem semWorker;    // Protects bmotor and slave data

//global variables
static uint32_t COPY_SPEED;
float tempX;
float tempA;
Modes runMode;
uint32_t TmrStart;
uint32_t TmrDur;

pt ptManager;
pt ptSlave;

// Bitmasks (Example based on your provided hex values)
#define MASK_IPC_Motor_Read_Confirm     0x00000100 //0000 0000 0000 0000 0000 0001 0000 0000
#define MASK_IPC_Logic_To_Motor 0x00001000 //0000 0000 0000 0000 0001 0000 0000 0000
#define MASK_MOTOR_MOVING 0x00000200
#define MASK_OVERRIDE_SEEN 0x0000400
#define MASK_OVERRIDE_FLAG 0x00002000
#define MASK_SPEED 0x00004000 //1 = fast, 0 = slow

// States for simulating for Main protothread's logic/brain execution.
enum eMState {
  msInit = 0,   // State to do one-time initialization or full restart of the state machine
  msFlagCheck,  // Triggers/waits touch sensor read and execute a solenoid kick if needed
  msReadValue,  // Triggers/Waits for the ADC read value to execute
} mainState;

static int threadMotorManager(struct pt *pt) {
  PT_BEGIN(pt);
  for(;;) {
    if (mainState == msInit) {
      mainState = msFlagCheck;
    }
    else if (mainState ==msFlagCheck) {
      // 1) Check bitflags for new task signal (Logic -> Motor)
      PT_SEM_WAIT(pt, &semIPC); //set local flags for things that i am looking for
       //copy the logic/motor to local flag
      static uint32_t COPY_OVERRIDE = (ipc_comms & MASK_OVERRIDE_FLAG);  
      COPY_SPEED = (ipc_comms & MASK_SPEED);
      ipc_comms &= ~MASK_OVERRIDE_FLAG; //check override all the time, only check newdata when finished with task
      ipc_comms |= MASK_OVERRIDE_SEEN;
      PT_SEM_SIGNAL(pt, &semIPC); //Give control back
      if (COPY_OVERRIDE) { //Now check to see
        // 2) Check struct to get data
        PT_SEM_WAIT(pt, &semMotor);
        tempX = motorData.targetX;
        tempA = motorData.targetAngle;
        runMode = motorData.opState;
        PT_SEM_SIGNAL(pt, &semMotor);

        // Trigger the Slave Thread
        PT_SEM_WAIT(pt, &semWorker);
        // Pass data to slave (re-using motor data or a slave-specific struct)
        if (!bMotor) {
          PT_SEM_WAIT(pt, &semIPC);
          static uint32_t COPY_NEWDATA = (ipc_comms & MASK_IPC_Logic_To_Motor);
          ipc_comms &= ~MASK_IPC_Logic_To_Motor;
          ipc_comms |= MASK_IPC_Motor_Read_Confirm;
          ipc_comms |= MASK_MOTOR_MOVING;
          PT_SEM_SIGNAL(pt, &semIPC);
          bMotor = true;
          mainState = msInit;
        } else {
          mainState = msInit;
        }
        PT_SEM_SIGNAL(pt, &semWorker);
        // Wait until Slave completes the task (Calculation + Spawn)
        PT_WAIT_WHILE(pt, !bMotor);
      } 
    }
    PT_SLEEP(pt, 10);
  }
  PT_END(pt);
}

static int threadMotorSlave(struct pt *pt) {
  static struct pt ptTimerSpawn; // Structure for the child timer
  static uint32_t moveTime;
  static float fastspeedconst = 500; //purely hypothetical. need to decide
  static float slowspeedconst = 250;
  static float fastspeedfactor = 1000/340;
  static float slowspeedfactor = 500/340;
  static float speedconst;


  PT_BEGIN(pt);

  for(;;) {
    // Wait for the Manager to signal "true"
    PT_WAIT_WHILE(pt, bMotor);

    // 4) Calculate time needed to move x distance or x angle
    // Logic: Time = (Distance * factor) or (Angle * factor)
    if (COPY_SPEED) {
      if (tempX) {
        moveTime = (uint32_t)(tempX * fastspeedfactor);
      } else {
        moveTime = (uint32_t)(tempA * fastspeedfactor); //do calcs after deciding speeds
      }
    } else {
      if (tempX) {
        moveTime = (uint32_t)(tempX * slowspeedfactor);
      } else {
        moveTime = (uint32_t)(tempA * slowspeedfactor); //do calcs after deciding speeds
      }
    }

    // 5) Start pt_spawn to time the action
    TmrStart = millis();
    TmrDur = moveTime;
    
    if (COPY_SPEED) {
      speedconst = fastspeedconst;
    } else {
      speedconst = slowspeedconst;
    }

    if (runMode == rForward) {
      forward(motor1, motor2, speedconst);
    } else if (runMode == rBackward){
      back(motor1, motor2, speedconst);
    } else if (runMode == rRotateL){
      left(motor1, motor2, 2*speedconst);
    } else if (runMode == rRotateR){
      right(motor1, motor2, 2*speedconst);
    }
    
    PT_SPAWN(pt, &ptTimerSpawn, threadTimer_MotorWait(&ptTimerSpawn));
    
    brake(motor1, motor2);
    //Tell ipc comms movement stopped

    PT_SEM_WAIT(pt, &semWorker);
    bMotor = false;
    PT_SEM_SIGNAL(pt, &semWorker);
    
    PT_YIELD(pt);
  }

  PT_END(pt);
}

int threadTimer_MotorWait(struct pt *move)
{
  // mark the beginnning of the kick wait thread
  PT_BEGIN(move);

  PT_WAIT_WHILE(move, getTicksDuration(TmrStart, millis()) >= TmrDur); // One tick is 1 milliseconds

  PT_EXIT(move);

  PT_END(move);
}

unsigned long getTicksDuration(unsigned long prevTicks, unsigned long currTicks) {
  if (prevTicks > currTicks) {                         // System time overflows (through 0)
    currTicks += (unsigned long)(-1) - prevTicks + 1;  // Shift current time forward relative to zero reference
    prevTicks = 0;
  }

  return (currTicks - prevTicks);  // Now just return the difference
}

void setup() {
  Serial.begin(115200);
  Serial.begin(9600);
  PT_INIT(&ptManager);
  PT_INIT(&ptSlave);
}

void loop() {
  PT_SCHEDULE(threadMotorSlave(&ptSlave));
  PT_SCHEDULE(threadMotorManager(&ptManager));
}