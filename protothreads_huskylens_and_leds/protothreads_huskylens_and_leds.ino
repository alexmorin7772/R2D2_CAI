/***************************************************
 HUSKYLENS An Easy-to-use AI Machine Vision Sensor
 <https://www.dfrobot.com/product-1922.html>
 
 ***************************************************
 This example shows the basic function of library for HUSKYLENS via Serial.
 
 Created 2020-03-13
 By [Angelo qiao](Angelo.qiao@dfrobot.com)
 
 GNU Lesser General Public License.
 See <http://www.gnu.org/licenses/> for details.
 All above must be included in any redistribution
 ****************************************************/

/***********Notice and Trouble shooting***************
 1.Connection and Diagram can be found here
 <https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336#target_23>
 2.This code is tested on Arduino Uno, Leonardo, Mega boards.
 ****************************************************/

#include "HUSKYLENS.h"
#include "SoftwareSerial.h"
#include "protothreads.h"

HUSKYLENS huskylens;
SoftwareSerial mySerial(10, 11); // RX, TX
//HUSKYLENS green line >> Pin 10; blue line >> Pin 11
void printResult(HUSKYLENSResult result);
pt ptHusky;

pt ptBlinky;
int blinkyThread(struct pt* blink) {
  PT_BEGIN(blink);

  // Loop forever
  for(;;) {
    digitalWrite(7, HIGH);   // turn the LED on (HIGH is the voltage level)
    PT_SLEEP(blink, 1000);
    digitalWrite(7, LOW);    // turn the LED off by making the voltage LOW
    PT_SLEEP(blink, 1000);
  }

  PT_END(blink);
}
pt ptBlinkr;
int blinkrThread(struct pt* blink) {
  PT_BEGIN(blink);

  // Loop forever
  for(;;) {
    digitalWrite(5, HIGH);   // turn the LED on (HIGH is the voltage level)
    PT_SLEEP(blink, 500);
    digitalWrite(5, LOW);    // turn the LED off by making the voltage LOW
    PT_SLEEP(blink, 500);
  }

  PT_END(blink);
}


void setup() {
    PT_INIT(&ptHusky);
    PT_INIT(&ptBlinkr);
    PT_INIT(&ptBlinky);
    pinMode(5, OUTPUT);
    pinMode(7, OUTPUT);
    Serial.begin(115200);
    mySerial.begin(9600);
    while (!huskylens.begin(mySerial))
    {
        Serial.println(F("Begin failed!"));
        Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>Serial 9600)"));
        Serial.println(F("2.Please recheck the connection."));
        delay(100);
    }
}
int HuskyLens(struct pt* husky) {
    PT_BEGIN(husky);
    for(;;) {
        while (!huskylens.available()) {
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
            PT_SLEEP(husky, 1);
        }
    }
    PT_END(husky);
}

void loop() {
  PT_SCHEDULE(HuskyLens(&ptHusky));
  PT_SCHEDULE(blinkyThread(&ptBlinky));
  PT_SCHEDULE(blinkrThread(&ptBlinkr));
}

void printResult(HUSKYLENSResult result){
    if (result.command == COMMAND_RETURN_BLOCK){
        Serial.println(String()+F("Block:xCenter=")+result.xCenter+F(",yCenter=")+result.yCenter+F(",width=")+result.width+F(",height=")+result.height+F(",ID=")+result.ID);
    }
    else if (result.command == COMMAND_RETURN_ARROW){
        Serial.println(String()+F("Arrow:xOrigin=")+result.xOrigin+F(",yOrigin=")+result.yOrigin+F(",xTarget=")+result.xTarget+F(",yTarget=")+result.yTarget+F(",ID=")+result.ID);
    }
    else{
        Serial.println("Object unknown!");
    }
}