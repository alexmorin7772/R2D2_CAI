/******************************************************************************
TestRun.ino
TB6612FNG H-Bridge Motor Driver Example code
Michelle @ SparkFun Electronics
8/20/16
https://github.com/sparkfun/SparkFun_TB6612FNG_Arduino_Library

Uses 2 motors to show examples of the functions in the library.  This causes
a robot to do a little 'jig'.  Each movement has an equal and opposite movement
so assuming your motors are balanced the bot should end up at the same place it
started.

Resources:
TB6612 SparkFun Library

Development environment specifics:
Developed on Arduino 1.6.4
Developed with ROB-9457
******************************************************************************/

int Motor(struct pt* drive) {
    PT_BEGIN(drive);
    static unsigned long curr = 0;
    static unsigned long prev = 0;   
    for(;;) {
        // CRITICAL CHECK: If a new Override arrives while we are moving, break this move!
        if (ipc_comms & MASK_OVERRIDE_SEEN) {
            brake(motor1, motor2);
            runMode = rDone; // Force exit to restart the loop and get new data
            break; 
        } else if (runMode == rForward) { //need to test how far robot runs at 200, 250, 500 speed for 1 second
            //at 200, runs ? mm in 1000 ms
            //at 250, runs 340 mm in 1000 ms
            forward(motor1, motor2, 250);
            unsigned long waittime = (commandValue*1000/340);
            Serial.println("forward");
            curr = millis();
            PT_WAIT_UNTIL(drive, curr-prev>=waittime);
            brake(motor1, motor2);
            prev = curr;
            runMode = rDone;
        } else if (runMode == rBackward) { //fix for backwards too
            forward(motor1, motor2, -200);
            unsigned long waittime = (commandValue*1000/340);
            Serial.println("backward");
            curr = millis();
            PT_WAIT_UNTIL(drive, curr-prev>=waittime);
            brake(motor1, motor2);
            prev = curr;
            runMode = rDone;
        } else if (runMode == rRotateL) { 
        //200 speed, 1000 ms = around 100 degrees, so 1 degree every 10 ms
        //250 speed, 1000 ms = around 125 degrees, so 1 degree every 8 ms
        //400 speed, 1000 ms = around 200 degress, so 1 degree every 5 ms
        //500 speed, 1000 ms = around 250 degrees, so 1 degree every 4 ms
            unsigned long waittime = (commandValue*5);
            left(motor1, motor2, 400);
            Serial.println("rotateL");
            curr = millis();
            PT_WAIT_UNTIL(drive, curr-prev>=waittime);
            brake(motor1, motor2);
            prev = curr;
            runMode = rDone;
        } else if (runMode == rRotateR) {
            unsigned long waittime = (commandValue*5);
            right(motor1, motor2, 400);
            Serial.println("rotateR"); 
            curr = millis();
            PT_WAIT_UNTIL(drive, curr-prev>=waittime);
            brake(motor1, motor2);
            prev = curr;
            runMode = rDone;
        } else if (runMode == rPivotFwd) {
            unsigned long waittime = (commandValue*5);
            pivotfwd(motor1, motor2, 400, false);
            Serial.println("pivotfwd");
            curr = millis();
            PT_WAIT_UNTIL(drive, curr-prev>=waittime);
            brake(motor1, motor2);
            prev = curr;
            runMode = rDone;
        } else if (runMode == rPivotRev) {
            pivotrev(motor1, motor2, 400, false);
            unsigned long waittime = (commandValue*5);
            Serial.println("pivotrev");
            curr = millis();
            PT_WAIT_UNTIL(drive, curr-prev>=waittime);
            brake(motor1, motor2);
            prev = curr;
            runMode = rDone;
        } else if(runMode == rTurnLeft) {
            turnleft(motor1, motor2, 400.0);
            unsigned long waittime = (commandValue*5);
            Serial.println("turnleft");
            curr = millis();
            PT_WAIT_UNTIL(drive, curr-prev>=waittime);
            brake(motor1, motor2);
            prev = curr;
            runMode = rDone;
        } else if(runMode == rTurnRight) {
            turnright(motor1, motor2, 400.0);
            unsigned long waittime = (commandValue*5);
            Serial.println("turnright");
            curr = millis();
            PT_WAIT_UNTIL(drive, curr-prev>=waittime);
            brake(motor1, motor2);
            prev = curr;
            runMode = rDone;
        } else if (runMode == rDone) {
            Serial.println("NOTRUNNING");
            PT_SEM_WAIT(drive, &sem_cmd_available);
            if (ipc_comms & MASK_OVERRIDE_SEEN) {
                Serial.println(">>> MOTOR: OVERRIDE DETECTED <<<");
                // (Logic for clearing queue is already in Terminal)
            }
            runMode = newMode; 
            curr = millis();
            prev = millis();
            ipc_comms |= MASK_MOTOR_MOVE;  // Set Bit: I am moving
            ipc_comms &= ~MASK_MOTOR_DONE; // Clear Bit: I am not finished yet
        }   
        PT_SLEEP(drive,100);
    }
    PT_END(drive)
}

