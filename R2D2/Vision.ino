#include "HUSKYLENS.h"
#include "SoftwareSerial.h"
#include "protothreads.h"

HUSKYLENS huskylens; 

SoftwareSerial mySerial(2, 3); // RX, TX
//HUSKYLENS green line = TX >> Pin 2 = RX; blue line = RX >> Pin 3 = TX
void printResult(HUSKYLENSResult result);

void huskyAlgorithm(void);
void huskyAlgorithm() {
  huskylens.writeAlgorithm(ALGORITHM_LINE_TRACKING);
}

int huskyRead(struct pt* husky) {
  PT_BEGIN(husky);
  for(;;) {
    if (!huskylens.request()) Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
      //else if(!huskylens.isLearned()) Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
      //else if(!huskylens.available()) Serial.println(F("No block or arrow appears on the screen!"));
      else {
       //Serial.println(F("###########"));
        while (huskylens.available()) {
          HUSKYLENSResult result = huskylens.read();
          printResult(result);
        }
      }
      PT_SLEEP(husky, 1);
    }
  PT_END(husky);
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