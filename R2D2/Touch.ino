/*
Touch.ino, by Alex
Code for a Touch Sensor for the R2D2_CAI Repository (git).

Function:
A) Reading ADC values from a Touch Sensor (specifically utilizing FSP02CE from Ohmite's FSP series (Force-Sensing Potentiometer) as of current)
Features:
A) Protect the read variable readSensor A_pos with a semaphore so that only exclusive access by the thread can update it
(Overall includes viable examples of semaphores for reference)
B) bitwise flag operations outlined in header file "touch.h", using "ipc_comms" as the value to compare with bit masks
C) Using an extra function "getTicksDuration" for millis() time calculations outined by sourcefile "Utils.h"
*/

#include "touch.h"
#include "Utils.h"

struct pt_sem semTouch;

static volatile float V_datapoints[4];

static volatile boolean bKick = false;
static volatile boolean bKick_Start = false;
static volatile boolean bPresent = false;
static volatile boolean bBeastMode = false;

static volatile unsigned long prevTime1 = 0;
static volatile unsigned long prevTime2 = 0;
static volatile unsigned long prevTime3 = 0;
static volatile unsigned long prevTime4 = 0;

static volatile float V_value;
static volatile float V_average;
static volatile boolean bKick_Again = false;

// ADC complete interrupt service routine
ISR(ADC_vect) {
  A_pos = ADC; // Read the ADC value
  adcStarted = false;  // Set the flag to indicate a reading is ready
}

boolean Check_Available_Brain_Event (void) {
  // The following returns true for new events only - (i.e. One that has not yet been confirmed-read by Touch)
  if ( (ipc_comms & MASK_IPC_Brain_To_Touch) && !(ipc_comms & MASK_IPC_Touch_Read_Confirmation) ) { // Touch uses Read Confirmation to conclude the last event
    return true;
  }
  return false;
}

boolean Check_Available_Touch_Event (void) {
  // The following returns true for new events only - (i.e. One that has not yet been confirmed-read by Brain)
  if ( (ipc_comms & MASK_IPC_Touch_To_Brain) && !(ipc_comms & MASK_IPC_Brain_Read_Confirmation) ) { // Main uses Read Confirmation to conclude the last event
    return true;
  }
  return false;
}

void Post_Touch_Event_to_Brain () {
  ipc_comms |= MASK_IPC_Touch_To_Brain;
}

void Post_Brain_Event_to_Touch () {
  ipc_comms |= MASK_IPC_Brain_To_Touch;
}

void readSensorPos(void) {
  digitalWrite(V1, HIGH);
  digitalWrite(V2, LOW);
  digitalWrite(V_ref_neg, LOW);
  digitalWrite(V_wiper, LOW);
  
  delayMicroseconds(3);

  sei(); // Enable global interrupts
  
  // Start a new ADC conversion
  adcStarted = true; // Set the flag
  ADCSRA |= bit(ADSC); // Start ADC conversion
  // A_pos = analogRead(V_wiper);

  // digitalWrite(V1, LOW);
  // digitalWrite(V2, HIGH);
}

static volatile enum {
  RS_INIT = 0,
  RS_ADC,
  RS_WAIT
} eReadState = RS_INIT;

static volatile int i = 0;

static int threadADCRead(struct pt* pos)
{
  PT_BEGIN(pos);

  if (eReadState == RS_INIT) {
    prevTime4 = millis();
    eReadState = RS_ADC;
  }
  else if (eReadState == RS_ADC) {
    if (!adcStarted) { // Check if a reading is ready after sampling and interrupt
      // read the value from the sensor:
      // PT_SEM_WAIT(pos, &semTouch); // This forces a sleep until the semaphore is signaled by another thread or already signaled
      readSensorPos();  // This enables the sensor sampler and interrupt trigger, setting the adcStarted to true
      PT_WAIT_UNTIL(pos, adcStarted == false); // Forces a sleep until adcStarted returns to false (by the ISR interrupt)
      // Post_Touch_Event_to_Brain();
      // PT_SEM_SIGNAL(pos, &semTouch); // Releases the next thread th     at was waiting for the semaphore to be signaled
      
      if (i < 4) {
        V_datapoints[i++] = (5.0/1024)*29*A_pos;
      }
      else {
        V_datapoints[0] = V_datapoints[1];
        V_datapoints[1] = V_datapoints[2];
        V_datapoints[2] = V_datapoints[3];
        V_datapoints[3] = (5.0/1024)*29*A_pos;

        eReadState = RS_WAIT;
      }
    }

    PT_YIELD(pos);
  }
  else if (eReadState == RS_WAIT) {
    if (getTicksDuration(prevTime4, millis()) >= 1) {
      prevTime4 = millis();
      eReadState = RS_ADC;
    }
  }

  PT_END(pos);
}

static volatile enum {
  DS_INIT = 0,
  DS_CHECK,
  DS_PRINT,
  DS_PRESENT,
  DS_ABSENT,
  DS_WAIT
} eDispState = DS_INIT;

static int threadDisplay(struct pt* disp)
{
  PT_BEGIN(disp);

  // for (;;)
  // {
  if (eDispState == DS_INIT) {
    prevTime1 = millis();
    eDispState = DS_CHECK;
  }
  else if (eDispState == DS_CHECK) {
    // PORTB |= (1 << 4);
    PT_SEM_WAIT(disp, &semTouch);
    if (Check_Available_Brain_Event()) {
      ipc_comms |= MASK_IPC_Touch_Read_Confirmation; // Touch confirms that it read from brain (turns the bit from 0 to 1)
      // ipc_comms &= ~MASK_IPC_Brain_To_Touch; // Touch resets the brain event to open for new ones (turns the bit back from 1 to 0 again)

      if ( (bKick == true) && (bKick_Start == false) ) {  // Kick when not already active - (including solenoid reset time)
        bKick = false;
        // digitalWrite(LED_BUILTIN, LOW); // Kick execution starts (LOW)
        bKick_Start = true;
      }
      
      if (bBeastMode == true) {
        bKick_Again = true;
      }
    }
    PT_SEM_SIGNAL(disp, &semTouch);

    PT_YIELD(disp);

    eDispState = DS_PRINT;
  }
  else if (eDispState == DS_PRINT) {
    Serial.print(5);
    Serial.print(",");
    Serial.print(0);
    Serial.print(",");
    
    V_value = (5.0/1024)*29*A_pos;
    V_average = (V_datapoints[0] + V_datapoints[1] + V_datapoints[2] + V_datapoints[3]) / 4.0; // Used array to find average between 4 shifting Volt values

    if (V_average >= 0.01) {
      eDispState = DS_PRESENT;
    } 
    else {
      eDispState = DS_ABSENT;
    }
  }
  else if (eDispState == DS_PRESENT) {
    Serial.print(V_value); // removed offset from the original equation: Serial.println((5.0/1024)*20*A_pos+offset);
    Serial.print(",");
    Serial.print(4.0);
    Serial.print(",");
    Serial.println(V_average);

    PT_SEM_WAIT(disp, &semTouch);
    // Check if event is already sent or "posted", but also check that confirmation is already set to true, then turn off confirmation.
    // Before a new (or updated) event can be posted to Brain, it must first clear the confirmation flag from Brain
    if ( (ipc_comms & MASK_IPC_Touch_To_Brain) && (ipc_comms & MASK_IPC_Brain_Read_Confirmation) ) { // The last event was consumed; solution to above comment
      ipc_comms &= ~MASK_IPC_Brain_Read_Confirmation;
    }
    Post_Touch_Event_to_Brain();
    bPresent = true;
    // digitalWrite(LED_BUILTIN, HIGH); // Touch sending (HGH)
    PORTB |= (1 << 5);
    PT_SEM_SIGNAL(disp,&semTouch);

    PT_YIELD(disp);

    if (bKick_Again) {
      bKick_Start = true;
      bKick_Again = false;
    }

    eDispState = DS_WAIT;
  }
  else if (eDispState == DS_ABSENT) {
    Serial.print(0.0);
    Serial.print(",");
    Serial.print(5.0);
    Serial.print(",");
    Serial.println(V_average);
    PT_SEM_WAIT(disp, &semTouch);
    // Before a new (or updated) event can be posted to Brain, it must first clear the confirmation flag from Brain
    if ( (ipc_comms & MASK_IPC_Touch_To_Brain) && (ipc_comms & MASK_IPC_Brain_Read_Confirmation) ) { // The last event was consumed; solution to above comment
      ipc_comms &= ~MASK_IPC_Brain_Read_Confirmation;
    }
    bPresent = false;
    
    Post_Touch_Event_to_Brain();
    
    PT_SEM_SIGNAL(disp,&semTouch);

    PT_YIELD(disp);
    
    eDispState = DS_WAIT;
  }
  else if (eDispState == DS_WAIT) {
    // PT_WAIT_WHILE(disp, getTicksDuration(prevTime1, millis()) < 10);
    // PT_WAIT_UNTIL(disp, (millis() - prevTime1) >= 10);

    PT_YIELD(disp);

    if (getTicksDuration(prevTime1, millis()) >= 10) {
      eDispState = DS_CHECK;
      // prevTime1 = millis();
      // PORTB &= ~(1 << 4);
    }
  }
  // } // forever

  PT_END(disp);
}

static int threadKick(struct pt* sol) // Worker Thread
{
  PT_BEGIN(sol);

  // for (;;)
  // {
    PT_WAIT_UNTIL(sol, bKick_Start);
    // digitalWrite(SOLENOID_PIN, HIGH);
    PORTB |= (1 << 4);

    prevTime2 = millis();
    // PT_WAIT_WHILE(sol, getTicksDuration(prevTime2, millis()) < 200);
    PT_WAIT_UNTIL(sol, (millis() - prevTime2) >= 200);
    
    PORTB &= ~(1 << 4);
    // digitalWrite(SOLENOID_PIN, LOW);
    bKick_Start = false;
  // }

  PT_END(sol);
}

static int threadMain(struct pt* brain)
{
  PT_BEGIN(brain);
  
  digitalWrite(LED_BUILTIN, LOW); // Initial state
  
  // for(;;)
  // {
    PT_SEM_WAIT(brain, &semTouch);
    if ( Check_Available_Touch_Event() ) {  // New event found!
      ipc_comms |= MASK_IPC_Brain_Read_Confirmation;  // Confirm the event
      // digitalWrite(LED_BUILTIN, LOW); // Brain received (LOW)
      PORTB &= ~(1 << 5);

      if (bPresent) {
        bKick = true;
        // PORTB &= ~(1 << 5);

        // Before a new (or updated) event can be posted to Touch, it must first clear the confirmation flag from Touch
        if ( (ipc_comms & MASK_IPC_Brain_To_Touch) && (ipc_comms & MASK_IPC_Touch_Read_Confirmation) ) { // The last event was consumed; solution to above comment
          ipc_comms &= ~MASK_IPC_Touch_Read_Confirmation;
        }
        Post_Brain_Event_to_Touch();
      }
    }
    PT_SEM_SIGNAL(brain, &semTouch);

    prevTime3 = millis();
    // PT_WAIT_WHILE(brain, getTicksDuration(prevTime3, millis()) < 1);
    PT_WAIT_UNTIL(brain, (millis() - prevTime3) >= 1);
  // }
  
  PT_END(brain);
}