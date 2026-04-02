int LightSensor(struct pt* sense) {
    PT_BEGIN(sense);
    for(;;) {
        Serial.print("The Light value is: ");
        Serial.println(TSL2561.readVisibleLux());
        if (TSL2561.readVisibleLux() < 20) {
            startRun  = true;
            idle = false;
        } else {
            startRun = false;
        }
        PT_SLEEP(sense,1000);
    }
    PT_END(sense);
}