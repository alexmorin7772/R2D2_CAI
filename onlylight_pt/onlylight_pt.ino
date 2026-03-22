/*
    Digital_Light_Sensor.ino
    A library for TSL2561

    Copyright (c) 2012 seeed technology inc.
    Website    : www.seeed.cc
    Author     : zhangkun
    Create Time:
    Change Log :

    The MIT License (MIT)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include <Wire.h>
#include <Digital_Light_TSL2561.h>
#include "protothreads.h"

bool isTooHigh = false;
int lightval = 0;

pt ptLight;
int LightSensor(struct pt* sense) {
    PT_BEGIN(sense);
    for(;;) {
        isTooHigh = false;
        Serial.print("The Light value is: ");
        Serial.println(TSL2561.readVisibleLux());
        PT_SLEEP(sense,1000)
        Serial.print("The Infrared value is: ");
        Serial.println(TSL2561.readIRLuminosity());  //read Infrared channel value only, not convert to lux.
        PT_SLEEP(sense,1000)
        Serial.print("The Full Spectrum value is: ");
        Serial.println(TSL2561.readFSpecLuminosity());///read Full Spectrum channel value only,  not convert to lux.
        PT_SLEEP(sense,1000);
        if (TSL2561.readVisibleLux() > 150) {
            lightval = TSL2561.readVisibleLux();
            isTooHigh = true;
        } else {
            isTooHigh = false;
        if (isTooHigh == true) {
            Serial.println("OVER: ");
        } else {
            Serial.println("CONTINUE: ");
        }
        }
    }
    PT_END(sense);
}

void setup() {
    PT_INIT(&ptLight);
    Wire.begin();
    Serial.begin(9600);
    TSL2561.init();
}

void loop() {
    PT_SCHEDULE(LightSensor(&ptLight));
}


