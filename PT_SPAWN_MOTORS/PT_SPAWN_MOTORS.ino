#include "protothreads.h"
#include "pt-sem.h"

// Inter-Process Communication (IPC) variables
volatile uint32_t ipc_comms = 0; 
static struct pt_sem semIPC;      // Protects ipc_comms

// Motor Data Payload
struct MotorData {
  float targetX;
  float targetAngle;
} motorData;
static struct pt_sem semMotor;    // Protects motor data payload

// Handshake Flags
volatile bool bMotor = false;     // Local flag to trigger slave thread
static struct pt_sem semWorker;    // Protects bmotor and slave data

// Bitmasks (Example based on your provided hex values)
#define MASK_IPC_Logic_To_Motor     //b block thingy
#define MASK_IPC_Motor_Read_Confirm 

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
      LOCAL_NEWCOMM = (ipc_comms & MASK_IPC_Logic_To_Motor); //copy the logic/motor to local flag
      ipc_comms &= ~MASK_IPC_Logic_To_Motor;  
      ipc_comms |= MASK_IPC_Motor_Read_Confirm;
      PT_SEM_SIGNAL(pt, &semIPC); //Give control back
      if (LOCAL_NEWCOMM) { //Now check to see
        // 2) Check struct to get data
        PT_SEM_WAIT(pt, &semMotor);
        float tempX = motorData.targetX;
        float tempA = motorData.targetAngle;
        PT_SEM_SIGNAL(pt, &semMotor);

        // Trigger the Slave Thread
        PT_SEM_WAIT(pt, &semWorker);
        // Pass data to slave (re-using motor data or a slave-specific struct)
        if (!bMotor) {
          bMotor = true;
          mainState = msInit;
        } else {
          mainState = //go somewhere else
        }
        PT_SEM_SIGNAL(pt, &semWorker);
        // Wait until Slave completes the task (Calculation + Spawn)
        PT_WAIT_UNTIL(pt, bmotor == false);
        Serial.println("Manager: Task complete reported by Slave.");
      } 
    }
    PT_SLEEP(pt, 10);
  }
  PT_END(pt);
}

static int threadMotorSlave(struct pt *pt) {
  static struct pt ptTimerSpawn; // Structure for the child timer
  static uint32_t moveTime;

  PT_BEGIN(pt);

  for(;;) {
    // Wait for the Manager to signal "true"
    PT_WAIT_UNTIL(pt, bMotor == true);

    // 4) Calculate time needed to move x distance or x angle
    // Logic: Time = (Distance * factor) or (Angle * factor)
    if (motorData.targetX) {
      moveTime = (uint32_t)((motorData.targetX * );
    } else {
      moveTime = (uint32_t)((motorData.targetAngle * );
    }

    // 5) Start pt_spawn to time the action
    TmrStart = millis();
    TmrDur = moveTime;
    //Would i put the actual movement code here?
    PT_SPAWN(pt, &ptTimerSpawn, threadTimer_MotorWait(&ptTimerSpawn));

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

  PT_WAIT_UNTIL(move, getTicksDuration(TmrStart, millis()) >= TmrDur); // One tick is 1 milliseconds

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