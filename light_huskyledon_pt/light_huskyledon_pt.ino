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
#include "HUSKYLENS.h"
#include "SoftwareSerial.h"
#include "protothreads.h"
HUSKYLENS huskylens;
SoftwareSerial mySerial(10, 11); // RX, TX
//HUSKYLENS green line >> Pin 10; blue line >> Pin 11
void printResult(HUSKYLENSResult result);
bool isTooHigh = false;
bool isFoundArrow = false;
int lightval = 0;
pt ptHusky;
int HuskyLens(struct pt* husky) {
    PT_BEGIN(husky);
    for(;;) {
        while (!huskylens.available()) {
            isFoundArrow = false;
            if (!huskylens.request()) Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
            else if(!huskylens.isLearned()) Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
            else if(!huskylens.available()) Serial.println(F("No block or arrow appears on the screen!"));
            else
            {
                Serial.println(F("###########"));
                while (huskylens.available())
                {
                    HUSKYLENSResult result = huskylens.read();
                    printResult(result);
                }    
            }
            PT_SLEEP(husky, 1000);
        }
    }
    PT_END(husky);
}
pt ptLight;
int LightSensor(struct pt* sense) {
    PT_BEGIN(sense);
    for(;;) {
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
        }
        if (isTooHigh == true) {
            Serial.println("LIGHTON");
            PT_SLEEP(sense,1000);
        } else {
            Serial.println("LIGHTOFF");
            PT_SLEEP(sense,1000);
        }

    }
    PT_END(sense);
}

void setup() {
    PT_INIT(&ptHusky);
    PT_INIT(&ptLight);
    Wire.begin();
    Serial.begin(115200);
    mySerial.begin(9600);
    TSL2561.init();
    while (!huskylens.begin(mySerial))
    {
        Serial.println(F("Begin failed!"));
        Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>Serial 9600)"));
        Serial.println(F("2.Please recheck the connection."));
        delay(100);
    }
}

void loop() {
    PT_SCHEDULE(LightSensor(&ptLight));
    PT_SCHEDULE(HuskyLens(&ptHusky));
}

void printResult(HUSKYLENSResult result){
    if (result.command == COMMAND_RETURN_BLOCK){
        Serial.println(String()+F("Block:xCenter=")+result.xCenter+F(",yCenter=")+result.yCenter+F(",width=")+result.width+F(",height=")+result.height+F(",ID=")+result.ID);
    }
    else if (result.command == COMMAND_RETURN_ARROW){
        Serial.println(String()+F("Arrow:xOrigin=")+result.xOrigin+F(",yOrigin=")+result.yOrigin+F(",xTarget=")+result.xTarget+F(",yTarget=")+result.yTarget+F(",ID=")+result.ID);
        isFoundArrow = true;
    }
    else{
        Serial.println("Object unknown!");
    }
}


