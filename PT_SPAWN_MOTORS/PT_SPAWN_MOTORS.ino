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

pt ptManager;
pt ptWorker;
pt ptTestBrain;

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
    else if (mainState ==msFlagCheck) {
      // 1) Check bitflags for new task signal (Logic -> Motor)
      PT_SEM_WAIT(pt, &semIPC); //set local flags for things that i am looking for
       //copy the logic/motor to local flag
      Serial.println("checking for override");
      COPY_OVERRIDE = (ipc_comms & MASK_OVERRIDE_FLAG);  
      Serial.println(COPY_OVERRIDE, HEX);
      Serial.println(ipc_comms, HEX);
      COPY_SPEED = (ipc_comms & MASK_SPEED);
      if (COPY_OVERRIDE) {
        Serial.println("clearing override");
        ipc_comms &= ~MASK_OVERRIDE_FLAG; //check override all the time, only check newdata when finished with task
        ipc_comms |= MASK_OVERRIDE_SEEN;
      }
      PT_SEM_SIGNAL(pt, &semIPC); //Give control back
      if (COPY_OVERRIDE) { //Now check to see
        Serial.println("override flag");
        // 2) Check struct to get data
        PT_SEM_WAIT(pt, &semMotor);
        tempX = motorData.targetX;
        tempA = motorData.targetAngle;
        runMode = motorData.opState;
        PT_SEM_SIGNAL(pt, &semMotor);
        // Trigger the Slave Thread
        bMotor = true;
        Serial.println("worker starts");
        // Pass data to slave (re-using motor data or a slave-specific struct)
      }
      PT_WAIT_WHILE(pt, bMotor);
      Serial.println("not bmotor");
      PT_SEM_WAIT(pt, &semIPC);
      Serial.println("checking for normal flag");
      COPY_NEWDATA = (ipc_comms & MASK_IPC_Brain_To_Motor);
      if (COPY_NEWDATA) {
        Serial.println("clearing brain->motor");
        ipc_comms &= ~MASK_IPC_Brain_To_Motor;
        ipc_comms |= MASK_IPC_Motor_Read_Confirm;
        ipc_comms |= MASK_MOTOR_MOVING;
      }
      PT_SEM_SIGNAL(pt, &semIPC);
      if (COPY_NEWDATA) {
        PT_SEM_WAIT(pt, &semMotor);
        tempX = motorData.targetX;
        tempA = motorData.targetAngle;
        runMode = motorData.opState;
        PT_SEM_SIGNAL(pt, &semMotor);
        bMotor = true;
      }
      mainState = msInit;
    } else {
        mainState = msInit;
    }
      // Wait until Slave completes the task (Calculation + Spawn)
    PT_WAIT_UNTIL(pt, !bMotor); 
    StartTime = millis();
    PT_SLEEP(pt, 1);
  }
  PT_END(pt);
}

static int threadMotorWorker(struct pt *pt) {
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
    digitalWrite(LED_BUILTIN, HIGH);

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

    TmrStart = millis();
    TmrDur = moveTime;

    Serial.println("begin moving");

    if (runMode == rForward) {
      forward(motor1, motor2, speedconst);
    } else if (runMode == rBackward){
      back(motor1, motor2, speedconst);
    } else if (runMode == rRotateL){
      left(motor1, motor2, 2*speedconst);
    } else if (runMode == rRotateR){
      right(motor1, motor2, 2*speedconst);
    }

    Serial.println("moving");
    PT_SPAWN(pt, &ptTimerSpawn, threadTimer_MotorWait(&ptTimerSpawn));

    digitalWrite(LED_BUILTIN, LOW);

    brake(motor1, motor2);
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

  PT_WAIT_UNTIL(move, getTicksDuration(TmrStart, millis()) >= TmrDur || (ipc_comms & MASK_OVERRIDE_FLAG)); // One tick is 1 milliseconds
  
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

static int threadTestInjector(struct pt *pt) {
  PT_BEGIN(pt);

  mainState = msInit;

  for(;;) {
    if (Serial.available()) {
      char cmd = Serial.read();
      Serial.println("brain takes control");
      PT_SEM_WAIT(pt, &semMotor);
      PT_SEM_WAIT(pt, &semIPC);
      if (cmd == 'f') { // Test Forward
        motorData.targetX = 1020.0;
        motorData.opState = rForward;
        ipc_comms |= MASK_IPC_Brain_To_Motor;
        Serial.println("FORWARD command");
      } else if (cmd == 'r') { // Test Rotate
        motorData.targetAngle = 90.0;
        motorData.opState = rRotateL;
        ipc_comms |= MASK_OVERRIDE_FLAG; // You require this to trigger the Manager
        Serial.println(ipc_comms, HEX);
        Serial.println("ROTATE command");
        Serial.println(">>> Injecting OVERRIDE");
      } else if (cmd == 'b') { //Test Backwards
        motorData.targetX = 340.0;
        motorData.opState = rBackward;
        ipc_comms |= MASK_IPC_Brain_To_Motor;
        Serial.println("BACKWARD command");
      }
      Serial.println("brain gives up control");
      PT_SEM_SIGNAL(pt, &semIPC);
      PT_SEM_SIGNAL(pt, &semMotor);
    }
    PT_SLEEP(pt, 10);
  }
  PT_END(pt);
}

void setup() {
  Serial.begin(115200);
  PT_INIT(&ptManager);
  PT_INIT(&ptWorker);
  PT_INIT(&ptTestBrain);
  PT_SEM_INIT(&semIPC, 1);
  PT_SEM_INIT(&semMotor, 1);
  //PT_SEM_INIT(&semWorker, 1);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  PT_SCHEDULE(threadMotorWorker(&ptWorker));
  PT_SCHEDULE(threadMotorManager(&ptManager));
  PT_SCHEDULE(threadTestInjector(&ptTestBrain));
}