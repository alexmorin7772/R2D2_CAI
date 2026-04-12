// Byte B Motor Control Masks
#define MASK_MOTOR_MOVE             0x00000100  //Status: 1 = Motors moving, 0 = Idle.
#define MASK_MOTOR_STALL            0x00000200  //Power is being applied but the robot isn't moving (common if you're stuck against a wall or another robot).

#define MASK_M_DIRECTION            0x00000400 //1 for forward, 0 for backwards
#define MASK_MOTOR_DONE             0x00000800 //Motors finished the move (Forward/Turn complete).

#define MASK_MOTOR_READ             0x00001000 //Motor confirms it received a new order from Logic.
#define MASK_OVERRIDE_SEEN          0x00002000 //Motor confirms it saw the override flag and will switch.

#define MASK_TARGET_REACHED         0x00004000 //flag set to 1 right after it has finished a command to move a certain distance/angle

// --- CHECKERS (Used by Logic/Camera to see what Motors are doing) ---

boolean Is_Motor_Moving() {
  return (ipc_comms & MASK_MOTOR_MOVE);
}

boolean Is_Motor_Stalled() {
  return (ipc_comms & MASK_MOTOR_STALL);
}

boolean Is_Motor_Done() {
  return (ipc_comms & MASK_MOTOR_DONE);
}

// --- POSTERS (Used by your Motor Thread to update status) ---

void Post_Motor_Status(boolean moving, boolean forward) {
  if (moving) ipc_comms |= MASK_MOTOR_MOVE;
  else        ipc_comms &= ~MASK_MOTOR_MOVE;

  if (forward) ipc_comms |= MASK_M_DIRECTION;
  else         ipc_comms &= ~MASK_M_DIRECTION;
}

void Post_Motor_Stall() {
  ipc_comms |= MASK_MOTOR_STALL;
}

void Post_Motor_Done() {
  ipc_comms |= MASK_MOTOR_DONE;
  ipc_comms &= ~MASK_MOTOR_MOVE; // If we are done, we aren't moving
}

// --- HANDSHAKING (Confirmations) ---

void Confirm_Command_Read() {
  ipc_comms |= MASK_MOTOR_READ;
}

void Confirm_Override_Seen() {
  ipc_comms |= MASK_OVERRIDE_SEEN;
}

void Clear_Motor_Confirmations() {
  // Clears both READ and OVERRIDE bits at once
  ipc_comms &= ~(MASK_MOTOR_READ | MASK_OVERRIDE_SEEN);
}
