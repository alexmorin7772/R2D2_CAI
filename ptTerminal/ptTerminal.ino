struct pt_sem sem_cmd_available;
// Initialize in setup() with PT_SEM_INIT(&sem_cmd_available, 0);
int Terminal(struct pt* pt) {
    PT_BEGIN(pt);

    for(;;) {
        // --- 1. RECEIVE DATA ---
        if (Serial.available() > 0) {
            String raw = Serial.readStringUntil('\n');
            Command cmd = parseString(raw);

            if (cmd.isOverride) {
                // EMERGENCY: Take over active slot immediately
                activeCommand = cmd;
                queueCount = 0;       // Clear stale tasks
                PT_SEM_SIGNAL(pt, &sem_cmd_available); // Wake up Motor NOW
            } 
            else {
                if (queueCount < 5) {
                    commandQueue[queueCount++] = cmd;
                    // We don't signal yet; we wait for the motor to be idle
                }
            }
        }

        // Instead of the Motor looking at the queue, the Terminal "pushes" 
        // the task to the motor when the motor is ready (rDone).
        if (runMode == rDone && queueCount > 0) {
            // LIFO: Grab the newest
            int latestIndex = queueCount - 1;
            activeCommand = commandQueue[latestIndex];
            
            queueCount--; 
            
            // SIGNAL: Tell the Motor thread "The activeCommand is ready for you"
            PT_SEM_SIGNAL(pt, &sem_cmd_available);
        }

        PT_YIELD(pt); 
    }

    PT_END(pt);
}