/***************************************************
 HUSKYLENS An Easy-to-use AI Machine Vision Sensor
 <https://www.dfrobot.com/product-1922.html>
 
 ***************************************************
 This example shows the basic function of library for HUSKYLENS via I2c.
 
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
#include <Wire.h>
#include <Digital_Light_TSL2561.h>
#include "protothreads.h"

HUSKYLENS huskylens;
//HUSKYLENS green line >> SDA; blue line >> SCL

void printResult(HUSKYLENSResult result);

void setup() {
    PT_INIT(&ptHusky);
    PT_INIT(&ptLight);
    Serial.begin(115200);
    Wire.begin();
    TSL2561.init();
    while (!huskylens.begin(Wire))
    {
        Serial.println(F("Begin failed!"));
        Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>I2C)"));
        Serial.println(F("2.Please recheck the connection."));
        delay(100);
    }
}
void loop() {
    PT_SCHEDULE(LightSensor(&ptLight));
    PT_SCHEDULE(HuskyLens(&ptHusky));
}

bool isTooHigh = false;
int lightval = 0;
pt ptHusky;
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
        int vari = TSL2561.readVisibleLux();
        Serial.print(vari);
        if (TSL2561.readVisibleLux() > 150) {
            lightval = TSL2561.readVisibleLux();
            isTooHigh = true;
        } else {
            isTooHigh = false;
        }
    }
    PT_END(sense);
}
//pseudocode
//pt ptBlock;
//int Block(struct pt* block) {
    //PT_BEGIN(block);
    //for(;;) {
        //if (isTooHigh) {
            //percentage = (lightval-n)/n;
            //if (percentage >= 100) {
                //rotate = 90
            //}
            //else {
                //rotate = int(90 * percentage/100)
            //}
        //}
        //rotate the polarized film at the same angle as the var rotate so that it gets less light
    //}
    //PT_END(block);
//}

void printResult(HUSKYLENSResult result){
    if (result.command == COMMAND_RETURN_BLOCK){
        Serial.println(String()+F("Block:xCenter=")+result.xCenter+F(",yCenter=")+result.yCenter+F(",width=")+result.width+F(",height=")+result.height+F(",ID=")+result.ID);
        if (result.ID == 1){
            Serial.println("Ball Spotted");
            delay(100);
        }
    }
    else if (result.command == COMMAND_RETURN_ARROW){
        Serial.println(String()+F("Arrow:xOrigin=")+result.xOrigin+F(",yOrigin=")+result.yOrigin+F(",xTarget=")+result.xTarget+F(",yTarget=")+result.yTarget+F(",ID=")+result.ID);
        float x = abs(result.xOrigin - result.xTarget);
        float y = abs(result.yOrigin - result.yTarget);
        float radangle = atan2(y, x);
        float degangle = radangle * (180.0 / PI);
        Serial.println(degangle);
    }
    else{
        Serial.println("Object unknown!");
    }
}

