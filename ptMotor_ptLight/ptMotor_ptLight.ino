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

// This is the library for the TB6612 that contains the class Motor and all the
// functions
#include <SparkFun_TB6612.h>
#include <Digital_Light_TSL2561.h>
#include "protothreads.h"
#include <Wire.h>

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

bool startRun = false;
bool idle = true;

enum Modes {
    rForward = 0,
    rBackward,
    rRotateL,
    rRotateR,
    rPivotFwd,
    rPivotRev,
    rTurnLeft,
    rTurnRight,
    rDone
};
Modes runMode = rDone;

pt ptLight;
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

pt ptBlink; // can probably get rid of this
int BlinkLED(struct pt* led) {
    PT_BEGIN(led);
    for(;;) {
        if (idle) {
            digitalWrite(13, HIGH);  
            PT_SLEEP(led,500);   
            digitalWrite(13, LOW); 
            PT_SLEEP(led,500);
        }
        PT_SLEEP(led,1000);
    }
    PT_END(led);
}
/*
pt ptTerminal; //eventually needs to send pacakges both ways
int Terminal(struct pt* package) { //use lifo struct/stack (last in first out)
    PT_BEGIN(package);
    for(;;) {
        //constant check for the most important/override flag first
            //then, switch variables -> state = list[0], val = list[1], status = list[3]/override flag, in each motor state constantly chekcing if override
            //once signal seen delete it from struct
        //if completed past action/no flag, go through struct (ordered in most recent to least recent + most important)
            //switch variables
            //delete signal
        PT_SLEEP(package, 1) //<- time value not determined yet
    }
    PT_END(package)
}
*/
struct Command {
    int mode;           
    float value;        
    bool isOverride;    
};

pt ptTerminal;
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

pt ptMotor;
int Motor(struct pt* drive) {
    PT_BEGIN(drive);
    static unsigned long curr = 0;
    static unsigned long prev = 0;
    for(;;) {
        if (runMode == rForward) { //need to test how far robot runs at 200, 250, 500 speed for 1 second
        //at 200, runs ? mm in 1000 ms
        //at 250, runs 340 mm in 1000 ms
            forward(motor1, motor2, 250);
            unsigned long waittime = (commandValue*1000/340);
            Serial.println("forward");
            curr = millis();
            if (curr - prev > waittime) {
                brake(motor1, motor2);
                prev = curr;
                runMode = rDone;
                }
        } else if (runMode == rBackward) { //fix for backwards too
            forward(motor1, motor2, -200);
            unsigned long waittime = (commandValue*1000/340);
            Serial.println("backward");
            curr = millis();
            if (curr - prev > waittime) {
                brake(motor1, motor2);
                prev = curr;
                runMode = rDone;
            }
        } else if (runMode == rRotateL) { 
        //200 speed, 1000 ms = around 100 degrees, so 1 degree every 10 ms
        //250 speed, 1000 ms = around 125 degrees, so 1 degree every 8 ms
        //400 speed, 1000 ms = around 200 degress, so 1 degree every 5 ms
        //500 speed, 1000 ms = around 250 degrees, so 1 degree every 4 ms
            unsigned long waittime = (commandValue*5);
            left(motor1, motor2, 400);
            Serial.println("rotateL");
            curr = millis();
            if (curr - prev > waittime) { // <-- waitime personalized for each thing based on angle
                brake(motor1, motor2);
                prev = curr;
                runMode = rDone;
            }
        } else if (runMode == rRotateR) {
            unsigned long waittime = (commandValue*5);
            right(motor1, motor2, 400);
            Serial.println("rotateR"); 
            curr = millis();
            if (curr - prev > waittime) {
                brake(motor1, motor2);
                prev = curr;
                runMode = rDone;
            }
        } else if (runMode == rPivotFwd) {
            unsigned long waittime = (commandValue*5);
            pivotfwd(motor1, motor2, 400, false);
            Serial.println("pivotfwd");
            curr = millis();
            if (curr - prev > waittime) {
                brake(motor1, motor2);
                prev = curr;
                runMode = rDone;
            }
        } else if (runMode == rPivotRev) {
            pivotrev(motor1, motor2, 400, false);
            unsigned long waittime = (commandValue*5);
            Serial.println("pivotrev");
            curr = millis();
            if (curr - prev > waittime) {
                brake(motor1, motor2);
                prev = curr;
                runMode = rDone;
            }
        } else if(runMode == rTurnLeft) {
            turnleft(motor1, motor2, 400.0);
            unsigned long waittime = (commandValue*5);
            Serial.println("turnleft");
            curr = millis();
            if (curr-prev > waittime) {
                brake(motor1, motor2);
                prev = curr;
                runMode = rDone;
            }
        } else if(runMode == rTurnRight) {
            turnright(motor1, motor2, 400.0);
            unsigned long waittime = (commandValue*5);
            Serial.println("turnright");
            curr = millis();
            if (curr-prev > waittime) {
                brake(motor1, motor2);
                prev = curr;
                runMode = rDone;
            }
        } else if (runMode == rDone) {
            Serial.println("NOTRUNNING");
            if (startRun) {
                runMode = newMode;
                curr = millis();
                prev = millis();
            }
        }   
        PT_SLEEP(drive,100);
    }
    PT_END(drive)
}

void setup() {
   PT_INIT(&ptLight);
   PT_INIT(&ptMotor);
   PT_INIT(&ptBlink);
   Serial.begin(115200);
   Wire.begin();
   TSL2561.init();
}

void loop() {
    PT_SCHEDULE(LightSensor(&ptLight));
    PT_SCHEDULE(Motor(&ptMotor));
    PT_SCHEDULE(BlinkLED(&ptBlink));
}
