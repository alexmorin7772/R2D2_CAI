int BitFlag(struct pt* pt) {
    PT_BEGIN(pt);

    for(;;) {
        // --- 1. RECEIVE & BITWISE UPDATE ---
        if (Serial.available() > 0) {
            String raw = Serial.readStringUntil('\n');
            Command cmd = parseString(raw);

            // Update Bitwise Flags based on Command Type
            if (cmd.type == "BALL_FOUND") {
                ipc_comms |= MASK_BALL_CONF;     // Bitwise OR: Set A1 to 1
                PT_SEM_SIGNAL(pt, &sem_ball);    // Signal: Wake up anyone waiting for ball
            } 
            else if (cmd.type == "BALL_LOST") {
                ipc_comms &= ~MASK_BALL_CONF;    // Bitwise AND + NOT: Flip A1 to 0
            }

            if (cmd.type == "GOAL_FOUND") {
                ipc_comms |= MASK_GOAL_CONF;     // Bitwise OR: Set A2 to 1
                PT_SEM_SIGNAL(pt, &sem_goal);    // Signal: Wake up anyone waiting for goal
            }

            // --- 2. MOTOR COMMAND LOGIC ---
            if (cmd.isOverride) {
                ipc_comms |= MASK_KICK_AL;       // Flag Byte C that we are in Override
                newMode = cmd.mode;
                commandValue = cmd.value;
                startRun = true;
                prev = millis();
                queueCount = 0;                  // Flush LIFO queue for priority
            } else {
                if (queueCount < 5) {
                    commandQueue[queueCount++] = cmd; // Push to stack
                }
            }
        }

        // --- 3. PROCESS LIFO QUEUE (Newest First) ---
        if (runMode == rDone && queueCount > 0) {
            int latestIndex = queueCount - 1; // Grab the very last item added

            newMode = commandQueue[latestIndex].mode;
            commandValue = commandQueue[latestIndex].value;
            startRun = true;
            prev = millis();

            queueCount--; // "Delete" the newest signal
        }

        // --- 4. TRANSMIT (Send feedback back to friend) ---
        // We check specific bits to see if we need to send a report
        if (ipc_comms & MASK_KICK_FAIL) {
            Serial.println("(status, kick_failure)"); 
            ipc_comms &= ~MASK_KICK_FAIL; // Clear the bit after reporting
        }
        
        if (ipc_comms & MASK_SENS_ERR) {
            Serial.println("(error, system_check_required)");
            ipc_comms &= ~MASK_SENS_ERR;
        }

        PT_YIELD(pt); 
    }

    PT_END(pt);
}
