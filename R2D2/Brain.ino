#include <Arduino.h>
#include "protothreads.h"
#include <SoftwareSerial.h>
#include "HUSKYLENS.h"
#include <Wire.h>
//#include <Digital_Light_TSL2561.h>
#include <Servo.h>
#include "MyObjRec.h"
#include "defines.h"
#include "touch.h"
#include "pt-sem.h"
// Brain.ino — reads camera and touch data via ipc_comms,
// decides what the robot should do, writes motor_flags for
// whatever motor code gets added later.
// No motor pins or speeds defined here — that's the motor
// person's job. Brain only outputs decisions.

// ----- MOTOR COMMAND BITFLAGS -----
// Brain writes these; motor code reads them to know what to do.
// These are Brain decisions, not hardware definitions.
#define MOTOR_STOP   0x01
#define MOTOR_SEARCH 0x02
#define MOTOR_CHASE  0x04
#define MOTOR_ALIGN  0x08
#define MOTOR_SHOOT  0x10

// Brain's movement output — motor code reads each cycle.
uint8_t motor_flags      = MOTOR_STOP;
extern int command_x_offset;  // pixels from center (negative = left, positive = right)

// ----- KICK SETTINGS -----
#define KICK_DISTANCE  0.1   // cm — 1mm, max kick
#define KICK_RANGE     2.0   // cm — medium kick if goal aligned
#define KICK_COOLDOWN  500   // ms between kicks
#define KICK_POWER_MAX 255
#define KICK_POWER_MED 180


kick_instruction kick_order = {KICK_IDLE, 0, 999.0, false, false};

//extern pt ptBrain;
unsigned long last_kick_time = 0;

// From Touch.ino — Brain writes bKick, reads bPresent.
extern volatile boolean bKick;
extern volatile boolean bPresent;
extern uint32_t ipc_comms;
extern object_recognition_results goal_results;
extern pt_sem sem_ball;
extern object_recognition_results ball_results;

// check_goal_aligned() — true if goal confidence bit (bit 1)
// is set and goal x is within 30px of frame center (160).

bool check_goal_aligned() {
  // ipc_comms bit 1 set by Goal_Distance_State_Machine.ino
  if (!(ipc_comms & 0b10)) return false;
  return (abs(goal_results.object_location.x - 160) < 30);
}


// signal_kick() — tells threadKick (Touch.ino) to fire solenoid.
// Uses MASK_IPC_Logic_To_Touch + bKick so threadDisplay picks
// it up and sets bKick_Start for threadKick.

void signal_kick() {
  bKick      = true;
  ipc_comms |= MASK_IPC_Brain_To_Touch;
  ipc_comms |= MASK_IPC_Touch_Read_Confirmation;
  ipc_comms &= ~MASK_IPC_Touch_To_Brain;
}


object_recognition_results ball_results;
object_recognition_results goal_results;


uint32_t ipc_comms = 0;

extern location goal_location;

// Shared output variables — movement code reads these.
movement_command current_command = CMD_STOP;
int command_x_offset = 0;

// Protothread for the brain loop.
pt ptBrain;
pt_sem sem_ball, sem_goal, sem_line, sem_ipc, sem_motor;



// brain_loop() — runs every 50 ms to check camera
// and decide what the movement system should do next.

int brain_loop(struct pt* pt) {
  PT_BEGIN(pt);

  for (;;) {
    // Wait until the ball state machine signals new data.
    PT_SEM_WAIT(pt, &sem_ball);

    // Check the ball confidence bit in the shared flags.
    if (!(ipc_comms & 0b1)) {
      // No ball detected — tell motors to stop.
      current_command = CMD_STOP;
      command_x_offset = 0;

    } else {
      // Ball is visible — calculate how far off-center it is.
      command_x_offset = ball_results.object_location.x - 160;

      if (ball_results.object_distance > 8.0) {
        // Ball is far — tell motors to chase it.
        current_command = CMD_CHASE;

      } else if (ipc_comms & 0b10) {
        // Ball is close AND we see the goal.
        command_x_offset = goal_results.object_location.x - 160;

        if (abs(command_x_offset) <= 30) {
          // Goal is centered — tell motors to shoot.
          current_command = CMD_SHOOT;
        } else {
          // Goal is off to one side — tell motors to align.
          current_command = CMD_ALIGN;
        }

      } else {
        // Ball is close but no goal visible — search for goal.
        current_command = CMD_SEARCH;
        command_x_offset = 0;
      }
    }

    // Refresh every 50 ms for responsive play.
    PT_SEM_SIGNAL(pt, &sem_ball);
    PT_SLEEP(pt, 50);
  }

  PT_END(pt);
}
