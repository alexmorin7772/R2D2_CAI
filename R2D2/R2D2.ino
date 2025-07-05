#include "protothreads.h"

pt ptEyelid;

void setup() {
  PT_INIT(&ptEyelid);
}

void loop() {
  PT_SCHEDULE(eyelidThread(&ptEyelid));
}