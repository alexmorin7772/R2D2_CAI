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
bool straight = true;
bool single1 = false;
bool single2 = true;
bool rotate = false;
bool pivot = true;
bool idle = true;

enum Modes {
    rStraight = 0,
    rSingle1,
    rSingle2,
    rRotate,
    rPivot,
    rDone
};
Modes runMode = rDone;

pt ptLight;
int LightSensor(struct pt* sense) {
    PT_BEGIN(sense);
    for(;;) {
        Serial.print("The Light value is: ");
        Serial.println(TSL2561.readVisibleLux());
        if (TSL2561.readVisibleLux() < 50) {
            startRun  = true;
            idle = false;
        } else {
            startRun = false;
        }
        PT_SLEEP(sense,1000);
    }
    PT_END(sense);
}

pt ptBlink;
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

pt ptMotor;
int Motor(struct pt* drive) {
    PT_BEGIN(drive);
    static unsigned long prev = 0;
    static unsigned long curr = 0;
    const long waittime = 500;
    for(;;) {
        if (runMode == rStraight) {
            forward(motor1, motor2, 150);
            back(motor1, motor2, -150);
            brake(motor1, motor2);
            Serial.println("straight");
            curr = millis();
            if (curr - prev > waittime) {
                prev = curr;
                if (startRun == false) {
                    runMode = rDone;
                } else {
                    runMode = rSingle1;
                }
            }
        } else if (runMode == rSingle1) {
            motor1.drive(255);
            motor1.drive(-255);
            motor1.brake();
            Serial.println("single1");
            curr = millis();
            if (curr - prev > waittime) {
                prev = curr;
                if (startRun == false) {
                    runMode = rDone;
                } else {
                    runMode = rSingle2;
                }
            }
        } else if (runMode == rSingle2) {
            motor2.drive(255);
            motor2.drive(-255);
            motor2.brake();
            Serial.println("single2");
            curr = millis();
            if (curr - prev > waittime) {
                prev = curr;
                if (startRun == false) {
                    runMode = rDone;
                } else {
                    runMode = rRotate;
                }
            }
        } else if (runMode == rRotate) {
            left(motor1, motor2, 500);
            right(motor1, motor2, 500);
            brake(motor1, motor2);
            Serial.println("rotate");
            curr = millis();
            if (curr - prev > waittime) {
                prev = curr;
                if (startRun == false) {
                    runMode = rDone;
                } else {
                    runMode = rPivot;
                }
            }
        } else if (runMode == rPivot) {
            pivotfwd(motor1, motor2, 100, true);
            Serial.println("pivot");
            curr = millis();
            if (curr - prev > waittime) {
                prev = curr;
                if (startRun == false) {
                    runMode = rDone;
                } else {
                    runMode = rStraight;
                }
            }
        } else if (runMode == rDone) {
            Serial.println("done");
            idle = true;
            Serial.println("NOTRUNNING");
            if (startRun) {
                idle = false;
                runMode = rStraight;
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
