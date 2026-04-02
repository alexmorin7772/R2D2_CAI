int Terminal(struct pt* pt) {
    PT_BEGIN(pt);
    for(;;) {
        // 1. CHECK FOR NEW INCOMING DATA
        if (Serial.available() > 0) {
            String raw = Serial.readStringUntil('\n');
            // parseString(raw) fills a temp Command 'cmd'
            Command cmd = parseString(raw);
            if (cmd.isOverride) {
                // EMERGENCY FLAG: Immediate execution
                newMode = cmd.mode;
                commandValue = cmd.value;
                startRun = true;
                prev = millis(); // Reset the motor timer
                queueCount = 0;       // Clear pending minor tasks
            } else {
                // Add to queue if there's space
                if (queueCount < 5) {
                    commandQueue[queueCount++] = cmd;
                }
            }
        }

        // 2. PROCESS QUEUE (if Motor is currently idle)
        if (runMode == rDone && queueCount > 0) {
            // Target the LATEST addition (the "top" of the stack)
            int latestIndex = queueCount - 1;

            newMode = commandQueue[latestIndex].mode;
            commandValue = commandQueue[latestIndex].value;
            startRun = true;
            prev = millis();

            // "Delete" the signal simply by reducing the count
            // Since we took the last one, no need to shift other elements!
            queueCount--; 
        }

        PT_YIELD(pt); // Keep the terminal responsive
    }

    PT_END(pt);
}
