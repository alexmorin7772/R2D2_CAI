#include "protothreads.h"
#include "pt-sem.h"
#include <SparkFun_TB6612.h>
#include "Utils.h"

// Pins for all inputs, keep in mind the PWM defines must be on PWM pins
// the default pins listed are the ones used on the Redbot (ROB-12097) with
// the exception of STBY which the Redbot controls with a physical switch
#define AIN1 11
#define BIN1 7
#define AIN2 12
#define BIN2 8
#define PWMA 5
#define PWMB 6
#define STBY 13

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

extern motor_state current_motor_state; //set default to search for ball
extern motor_data current_motor_data; //start by being idle

//static struct pt_sem sem_motor;    // Protects motor data payload
runMode_t runMode;
// Handshake Flags
volatile bool bMotor = false;     // Local flag to trigger slave thread
//static struct pt_sem semWorker;    // Protects bmotor and slave data
volatile bool bRunOnce = false;

//global variables
static uint32_t COPY_SPEED;
float tempX;
float tempA;

static uint32_t TmrStart;
static uint32_t TmrDur;
static uint32_t StartTime;

//pt ptTestBrain;

// Bitmasks (Example based on your provided hex values)
#define MASK_IPC_Motor_Read_Confirm     0x00000100 //0000 0000 0000 0000 0000 0001 0000 0000
#define MASK_IPC_Brain_To_Motor 0x00001000 //0000 0000 0000 0000 0001 0000 0000 0000
#define MASK_MOTOR_MOVING 0x00000200
#define MASK_OVERRIDE_SEEN 0x00000400
#define MASK_OVERRIDE_FLAG 0x00002000
#define MASK_SPEED 0x00004000 //1 = fast, 0 = slow

// States for simulating for Main protothread's logic/brain execution.
enum eMState {
  msInit = 0,   // State to do one-time initialization or full restart of the state machine
  msFlagCheck,  //Waits for signal from bit flags and gets data from struct
} mainState;

static int threadMotorManager(struct pt *pt) {
  PT_BEGIN(pt);
  static uint32_t COPY_OVERRIDE;
  static uint32_t COPY_NEWDATA;
  for(;;) {
    if (mainState == msInit) {
      mainState = msFlagCheck;
    }
    else if (mainState == msFlagCheck) {
      // 1) Check bitflags for new task signal (Logic -> Motor)
      PT_SEM_WAIT(pt, &sem_ipc); //set local flags for things that i am looking for
       //copy the logic/motor to local flag
      //Serial.println("checking for override");
      COPY_OVERRIDE = (ipc_comms & MASK_OVERRIDE_FLAG);  
      //Serial.println(COPY_OVERRIDE, HEX);
      //Serial.println(ipc_comms, HEX);
      COPY_SPEED = (ipc_comms & MASK_SPEED);
      if (COPY_OVERRIDE) {
        //Serial.println("clearing override");
        ipc_comms &= ~MASK_OVERRIDE_FLAG; //check override all the time, only check newdata when finished with task
        ipc_comms |= MASK_OVERRIDE_SEEN;
      }
      PT_SEM_SIGNAL(pt, &sem_ipc); //Give control back
      if (COPY_OVERRIDE) { //Now check to see
        Serial.println(F("override flag found: executing command"));
        // 2) Check struct to get data
        PT_SEM_WAIT(pt, &sem_motor);
        tempX = current_motor_data.object_distance;
        tempA = current_motor_data.turn_angle;
        runMode = current_motor_data.action;
        PT_SEM_SIGNAL(pt, &sem_motor);
        // Trigger the Slave Thread
        bMotor = true;
        Serial.println(F("worker starts"));
        // Pass data to slave (re-using motor data or a slave-specific struct)
      }
      PT_WAIT_WHILE(pt, bMotor);
      //Serial.println("not bmotor");
      PT_SEM_WAIT(pt, &sem_ipc);
      Serial.println(F("checking for normal flag"));
      COPY_NEWDATA = (ipc_comms & MASK_IPC_Brain_To_Motor);
      if (COPY_NEWDATA) {
        //Serial.println("clearing brain->motor");
        Serial.println(F("normal flag found: executing command"));
        ipc_comms &= ~MASK_IPC_Brain_To_Motor;
        ipc_comms |= MASK_IPC_Motor_Read_Confirm;
        ipc_comms |= MASK_MOTOR_MOVING;
      }
      PT_SEM_SIGNAL(pt, &sem_ipc);
      if (COPY_NEWDATA) {
        PT_SEM_WAIT(pt, &sem_motor);
        tempX = current_motor_data.object_distance;
        tempA = current_motor_data.turn_angle;
        runMode = current_motor_data.action;
        PT_SEM_SIGNAL(pt, &sem_motor);
        bMotor = true;
      }
      mainState = msInit;
    } else {
        mainState = msInit;
    }
      // Wait until Slave completes the task (Calculation + Spawn)
    PT_WAIT_UNTIL(pt, !bMotor); 
    StartTime = millis();
    PT_SLEEP(pt, 100);
  }
  PT_END(pt);
}

static int threadMotorWorker(struct pt *pt) {
  static struct pt ptTimerSpawn; // Structure for the child timer
  static uint32_t moveTime;
  static float fastspeedconst = 500; //purely hypothetical. need to decide
  static float slowspeedconst = 250;
  static float fastspeedfactor;
  static float slowspeedfactor;

  static float speedconst;


  PT_BEGIN(pt);

  for(;;) {

    if ((current_motor_data.action == turn_left) || (current_motor_data.action == turn_right)) {
      fastspeedfactor = 2;
      slowspeedfactor = 4;
    } else {
      fastspeedfactor = 500/340;
      slowspeedfactor = 1000/340;
    }
    // Wait for the Manager to signal "true"

    PT_WAIT_UNTIL(pt, bMotor);
    ipc_comms &= ~MASK_OVERRIDE_SEEN;
    ipc_comms &= ~MASK_IPC_Motor_Read_Confirm;

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

    if (COPY_SPEED) {
      speedconst = fastspeedconst;
    } else {
      speedconst = slowspeedconst;
    }

    PT_SEM_WAIT(pt, &sem_ipc);
    if (runMode == drive_forward) {
      forward(motor1, motor2, speedconst / 2);
      Serial.println(F("I AM MOVING FORWARD"));
      ipc_comms |= MASK_MOTOR_MOVING;
    } else if (runMode == drive_backward){
      back(motor1, motor2, speedconst / 2);
      Serial.println(F("I AM MOVING BACKWARD"));
      ipc_comms |= MASK_MOTOR_MOVING;
    } else if (runMode == turn_left){
      right(motor1, motor2, speedconst / 2);
      Serial.println(F("I AM MOVING LEFT"));
      ipc_comms |= MASK_MOTOR_MOVING;
    } else if (runMode == turn_right){
      left(motor1, motor2, speedconst / 2);
      Serial.println(F("I AM MOVING RIGHT"));
      ipc_comms |= MASK_MOTOR_MOVING;
    } else if (runMode == idle){
      brake(motor1, motor2);
      Serial.println(F("I AM BRAKING"));
    }

    TmrStart = millis();
    TmrDur = (20 * moveTime) + 442;
    
    PT_SEM_SIGNAL(pt, &sem_ipc);

    Serial.print(F("Going to move for "));
    Serial.print(TmrDur);
    Serial.println(F(" milliseconds."));

    PT_SPAWN(pt, &ptTimerSpawn, threadTimer_MotorWait(&ptTimerSpawn));
                                   
    PT_SEM_WAIT(pt, &sem_ipc);
    ipc_comms &= ~MASK_MOTOR_MOVING;
    PT_SEM_SIGNAL(pt, &sem_ipc);

    //Tell ipc comms movement stopped
    bMotor = false;
    //PT_YIELD(pt);
    PT_SLEEP(pt, 1);
  }

  PT_END(pt);
}

int threadTimer_MotorWait(struct pt *move)
{
  // mark the beginnning of the kick wait thread
  PT_BEGIN(move);

  PT_WAIT_UNTIL(move, (getTicksDuration(TmrStart, millis()) >= TmrDur) || ((ipc_comms & MASK_OVERRIDE_FLAG) && !no_overrides)); // One tick is 1 milliseconds
  //brain needs to send payload then send override so that i am not executing old data
  Serial.print(F("Finished moving after "));
  Serial.print(millis() - TmrStart);
  Serial.println(F(" milliseconds."));
  PT_EXIT(move);

  PT_END(move);
}
