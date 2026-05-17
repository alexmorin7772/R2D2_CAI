// #include "protothreads.h"
// #include "SoftwareSerial.h"
// #include "HUSKYLENS.h"
// #include <Wire.h>
// #include <Digital_Light_TSL2561.h>
// #include <Servo.h>
// // Brain.ino — Reads sensor data every 2 seconds, decides what
// // movement command to send, and calls the motor functions.



// // Brain.ino — Reads sensor data every 2 seconds, decides what
// // movement command to send, and calls the motor functions.


// // All possible commands the brain can give to the motors
// typedef enum brain_command {
//   CMD_STOP,
//   CMD_FORWARD,
//   CMD_BACKWARD,
//   CMD_TURN_LEFT,
//   CMD_TURN_RIGHT,
//   CMD_STEER_LEFT,
//   CMD_STEER_RIGHT,
//   CMD_SHOOT
// };

// // Packages a command with its speed and steering info
// struct motion_command {
//   brain_command command;
//   int speed;
//   int steer_offset;  // pixels from center, negative=left, positive=right
// };

// // Current command starts as STOP, updated every 2 seconds
// motion_command current_command = {CMD_STOP, 0, 0};

// // Protothread so the brain runs alongside other tasks
// pt pt_brain;

// // Holds a snapshot of ALL sensor readings at one moment
// struct sensor_snapshot {
//   bool   ball_visible;
//   float  ball_distance;
//   int    ball_x;
//   int    ball_y;
//   bool   goal_visible;
//   float  goal_distance;
//   int    goal_x;
//   int    goal_y;
//   bool   game_started;
// };

// sensor_snapshot latest_snapshot = {false, 0, 0, 0, false, 0, 0, 0, false};


// // read_all_sensors() — grabs latest ball, goal, and game state
// // data into one snapshot for the brain to reason about.

// void read_all_sensors(sensor_snapshot* snap) {
//   // Read ball data from Ball_Distance_State_Machine.ino
//   if (ball_results.is_most_recent) {
//     snap->ball_visible  = true;
//     snap->ball_distance = ball_results.object_distance;
//     snap->ball_x        = ball_results.object_location.x;
//     snap->ball_y        = ball_results.object_location.y;
//     ball_results.is_most_recent = false;
//   } else {
//     snap->ball_visible = false;
//   }

//   // Read goal data from Goal_Distance_State_Machine.ino
//   if (goal_results.is_most_recent) {
//     snap->goal_visible  = true;
//     snap->goal_distance = goal_results.object_distance;
//     snap->goal_x        = goal_results.object_location.x;
//     snap->goal_y        = goal_results.object_location.y;
//     goal_results.is_most_recent = false;
//   } else {
//     snap->goal_visible = false;
//   }

//   // Read start/stop signal from Communication.ino
//   update_start_stop();
//   snap->game_started = started;
// }


// // decide_motion() — the brain logic. Looks at sensor data and
// // picks the right movement: search, chase, aim, or shoot.

// motion_command decide_motion(sensor_snapshot* snap) {
//   motion_command cmd;
//   cmd.steer_offset = 0;

//   // Game hasn't started — stay still
//   if (!snap->game_started) {
//     cmd.command = CMD_STOP;
//     cmd.speed = 0;
//     return cmd;
//   }

//   int ball_x_offset = snap->ball_x - FRAME_CENTER_X;
//   int goal_x_offset = snap->goal_x - FRAME_CENTER_X;

//   // Can't see the ball — spin to search for it
//   if (!snap->ball_visible) {
//     cmd.command = CMD_TURN_RIGHT;
//     cmd.speed = SPEED_TURN;
//     return cmd;
//   }

//   // Ball is far — drive toward it at full speed
//   if (snap->ball_distance > CLOSE_DISTANCE) {
//     if (ball_x_offset < -CENTER_DEADZONE) {
//       cmd.command = CMD_STEER_LEFT;
//       cmd.speed = SPEED_FULL;
//       cmd.steer_offset = ball_x_offset;
//     } else if (ball_x_offset > CENTER_DEADZONE) {
//       cmd.command = CMD_STEER_RIGHT;
//       cmd.speed = SPEED_FULL;
//       cmd.steer_offset = ball_x_offset;
//     } else {
//       cmd.command = CMD_FORWARD;
//       cmd.speed = SPEED_FULL;
//     }
//     return cmd;
//   }

//   // Ball is close — slow approach for better control
//   if (snap->ball_distance > CAPTURE_DISTANCE) {
//     if (ball_x_offset < -CENTER_DEADZONE) {
//       cmd.command = CMD_STEER_LEFT;
//       cmd.speed = SPEED_APPROACH;
//       cmd.steer_offset = ball_x_offset;
//     } else if (ball_x_offset > CENTER_DEADZONE) {
//       cmd.command = CMD_STEER_RIGHT;
//       cmd.speed = SPEED_APPROACH;
//       cmd.steer_offset = ball_x_offset;
//     } else {
//       cmd.command = CMD_FORWARD;
//       cmd.speed = SPEED_APPROACH;
//     }
//     return cmd;
//   }

//   // Ball captured but can't see goal — spin slowly to find it
//   if (!snap->goal_visible) {
//     cmd.command = CMD_TURN_RIGHT;
//     cmd.speed = SPEED_APPROACH;
//     return cmd;
//   }

//   // Goal is close — shoot!
//   if (snap->goal_distance <= CLOSE_DISTANCE) {
//     cmd.command = CMD_SHOOT;
//     cmd.speed = 255;
//     return cmd;
//   }

//   // Goal is far — push ball toward it
//   if (goal_x_offset < -CENTER_DEADZONE) {
//     cmd.command = CMD_STEER_LEFT;
//     cmd.speed = SPEED_FULL;
//     cmd.steer_offset = goal_x_offset;
//   } else if (goal_x_offset > CENTER_DEADZONE) {
//     cmd.command = CMD_STEER_RIGHT;
//     cmd.speed = SPEED_FULL;
//     cmd.steer_offset = goal_x_offset;
//   } else {
//     cmd.command = CMD_FORWARD;
//     cmd.speed = SPEED_FULL;
//   }
//   return cmd;
// }


// // execute_motion() — translates a brain command into actual
// // motor function calls (driveForward, turnLeft, etc.)

// void execute_motion(motion_command* cmd) {
//   switch (cmd->command) {
//     case CMD_STOP:       stopMotors();                        break;
//     case CMD_FORWARD:    driveForward(cmd->speed);            break;
//     case CMD_BACKWARD:   driveBackward(cmd->speed);           break;
//     case CMD_TURN_LEFT:  turnLeft(cmd->speed);                break;
//     case CMD_TURN_RIGHT: turnRight(cmd->speed);               break;
//     case CMD_STEER_LEFT:
//     case CMD_STEER_RIGHT:
//       steerToward(cmd->steer_offset, cmd->speed);             break;
//     case CMD_SHOOT:      driveForward(255);                   break;
//   }
// }


// // brain_loop() — main protothread. Every 2 seconds:
// //   1. Reads sensors  2. Decides action  3. Moves motors

// int brain_loop(struct pt* pt) {
//   PT_BEGIN(pt);

//   for (;;) {
//     read_all_sensors(&latest_snapshot);
//     current_command = decide_motion(&latest_snapshot);
//     execute_motion(&current_command);

//     // Debug output to Serial Monitor
//     Serial.print("Brain | Ball: ");
//     if (latest_snapshot.ball_visible) {
//       Serial.print("dist=");
//       Serial.print(latest_snapshot.ball_distance);
//       Serial.print(" x=");
//       Serial.print(latest_snapshot.ball_x);
//     } else {
//       Serial.print("NOT VISIBLE");
//     }
//     Serial.print(" | Goal: ");
//     if (latest_snapshot.goal_visible) {
//       Serial.print("dist=");
//       Serial.print(latest_snapshot.goal_distance);
//       Serial.print(" x=");
//       Serial.print(latest_snapshot.goal_x);
//     } else {
//       Serial.print("NOT VISIBLE");
//     }
//     Serial.print(" | Cmd: ");
//     Serial.println(current_command.command);

//     // Wait 2 seconds before next decision cycle
//     PT_SLEEP(pt, 2000);
//   }

//   PT_END(pt);
// }
