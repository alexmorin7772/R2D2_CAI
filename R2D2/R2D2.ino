#include "protothreads.h"

pt ptEyelid;
pt ptHuskylens;

void setup() {
  PT_INIT(&ptEyelid);
  PT_INIT(&ptHuskylens);
}

void loop() {
  PT_SCHEDULE(eyelidThread(&ptEyelid));
  PT_SCHEDULE(huskyRead(&ptHuskylens));
}