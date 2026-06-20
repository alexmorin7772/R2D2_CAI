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
#include <util/atomic.h>

static volatile pt solKick;
static volatile pt readPos, adcDisp;

extern uint32_t ipc_comms;

int offset = 0;

// Touch.ino declaration code
struct pt_sem semTouch;
// Define a global variable to store the ADC reading
volatile int A_pos = 0;
// Flag to indicate if a reading is available
volatile bool adcStarted = false;

static volatile float V_datapoints[4];

extern volatile bool bKick;
extern volatile bool bKick_Start;
extern volatile bool bPresent;
extern volatile bool bBeastMode;

extern volatile unsigned long prevTime1;
extern volatile unsigned long prevTime2;
extern volatile unsigned long prevTime3;
extern volatile unsigned long prevTime4;

static volatile unsigned long prevTouchTime = 0;

static volatile float V_value;
static volatile float V_average;
static volatile boolean bKick_Again = false;

void touchSetup(void) {
  PT_INIT(&readPos); // Touch.ino
  PT_INIT(&adcDisp); // Touch.ino
  PT_SEM_INIT(&semTouch, 1); // Touch.ino
  PT_INIT(&solKick); // Touch.ino
  // Touch.ino setup for code pinmode & ADC
  pinMode(V1, OUTPUT);
  pinMode(V2, OUTPUT);
  //pinMode(V_ref_neg, INPUT);
  pinMode(V_wiper, OUTPUT);
  pinMode(SOLENOID_PIN, OUTPUT);
  digitalWrite(SOLENOID_PIN, LOW);  // Initialize de-energized
  //pinMode(LED_BUILTIN, OUTPUT); // Touch.ino for debug purposes, open to change!
  //digitalWrite(LED_BUILTIN, LOW); // Using LED_BUILTIN from pinmode

  //pinMode(V_LOW, OUTPUT);
  //digitalWrite(V_LOW, LOW);

  // below are the 4 pinmodes for the impeller
  pinMode(FAN_AIN1, OUTPUT);
  pinMode(FAN_PWM, OUTPUT);
  pinMode(FAN_STBY, OUTPUT);
  //pinMode(LED_BUILTIN, OUTPUT);

  // Making sure the impeller is off at the start
  digitalWrite(FAN_STBY, LOW);
  digitalWrite(FAN_AIN1, HIGH);
  analogWrite(FAN_PWM, 0);

  cli(); // Disable global interrupts until required

  // Configure ADC settings
  ADCSRA = bit(ADEN);  // Enable ADC
  ADCSRA |= bit(ADPS0) | bit(ADPS1) | bit(ADPS2); // Set ADC clock prescaler
  ADMUX = bit(REFS0) | 1; // Set voltage reference and select ADC channel
  ADCSRA |= bit(ADIE); // Enable ADC interrupt
  // **************************************

  sei(); // Enable global interrupts
  
  // Start a new ADC conversion
  //adcStarted = true; // Set the flag
  //ADCSRA |= bit(ADSC); // Start ADC conversion
}

void RunTouchKickScheduler(void) {
  PT_SCHEDULE(threadADCRead(&readPos));
  PT_SCHEDULE(threadDisplay(&adcDisp));
  PT_SCHEDULE(threadKick(&solKick));
}

// ADC complete interrupt service routine
ISR(ADC_vect) {
  A_pos = ADC; // Read the ADC value
  adcStarted = false;  // Set the flag to indicate a reading is ready
}


// Important bitflag operations
boolean Check_Available_Brain_Event (void) {
  // The following returns true for new events only - (i.e. One that has not yet been confirmed-read by Touch)
  if ( (ipc_comms & MASK_IPC_Brain_To_Touch) && !(ipc_comms & MASK_IPC_Touch_Read_Confirmation) ) { // Touch uses Read Confirmation to conclude the last event
    return true;
  }
  return false;
}


// Important bitflag operations
void Post_Touch_Event_to_Brain () {
  ipc_comms |= MASK_IPC_Touch_To_Brain;
}



void readSensorPos(void) {
  digitalWrite(V1, HIGH);
  digitalWrite(V2, LOW);
  //digitalWrite(V_ref_neg, LOW);
  digitalWrite(V_wiper, LOW);
  
  delayMicroseconds(3);

  sei(); // Enable global interrupts
  
  // Start a new ADC conversion
  adcStarted = true; // Set the flag
  ADCSRA |= bit(ADSC); // Start ADC conversion
}

static volatile int i = 0;

static int threadADCRead(struct pt* pos)
{
  PT_BEGIN(pos);

  eReadState_t eReadState = RS_INIT;

  for (;;) {

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
          V_datapoints[i++] = (5.0/1024)*30*A_pos;
        }
        else {
          V_datapoints[0] = V_datapoints[1];
          V_datapoints[1] = V_datapoints[2];
          V_datapoints[2] = V_datapoints[3];
          V_datapoints[3] = (5.0/1024)*30*A_pos;

          eReadState = RS_WAIT;
        }
      }

      PT_YIELD(pos);
    }
    else if (eReadState == RS_WAIT) {
      if (getTicksDuration(prevTime4, millis()) >= 10) {
        prevTime4 = millis();
        eReadState = RS_ADC;
      }
    }
  } // forever

  PT_END(pos);
}

static int threadDisplay(struct pt* disp)
{
  PT_BEGIN(disp);

  static eDispState_t eDispState = DS_INIT;

  for (;;)
  {
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
      /*
      Serial.print(5);
      Serial.print(",");
      Serial.print(0);
      Serial.print(",");
      */
      V_value = (5.0/1024)*30*A_pos;
      V_average = (V_datapoints[0] + V_datapoints[1] + V_datapoints[2] + V_datapoints[3]) / 4.0; // Used array to find average between 4 shifting Volt values

      if (V_average >= 0.001) {
        eDispState = DS_PRESENT;
        
        prevTouchTime = millis();

        digitalWrite(FAN_STBY, HIGH);
        digitalWrite(FAN_AIN1, LOW);
        analogWrite(FAN_PWM, 255);
      } 
      else {
        eDispState = DS_ABSENT;
      }
    }
    else if (eDispState == DS_PRESENT) {
      /*
      Serial.print(V_value); // removed offset from the original equation: Serial.println((5.0/1024)*20*A_pos+offset);
      Serial.print(",");
      Serial.print(4.0);
      Serial.print(",");
      Serial.println(V_average);
      */
      PT_SEM_WAIT(disp, &semTouch);
      // Check if event is already sent or "posted", but also check that confirmation is already set to true, then turn off confirmation.
      // Before a new (or updated) event can be posted to Brain, it must first clear the confirmation flag from Brain
      if ( (ipc_comms & MASK_IPC_Touch_To_Brain) && (ipc_comms & MASK_IPC_Brain_Read_Confirmation) ) { // The last event was consumed; solution to above comment
        ipc_comms &= ~MASK_IPC_Brain_Read_Confirmation;
      }
      Post_Touch_Event_to_Brain();
      bPresent = true;
      // digitalWrite(LED_BUILTIN, HIGH); // Touch sending (HGH)
      //PORTB |= (1 << 5);
      PT_SEM_SIGNAL(disp,&semTouch);

      PT_YIELD(disp);

      if (bKick_Again) {
        bKick_Start = true;
        bKick_Again = false;
      }

      eDispState = DS_WAIT;
    }
    else if (eDispState == DS_ABSENT) {
      /*
      Serial.print(0.0);
      Serial.print(",");
      Serial.print(5.0);
      Serial.print(",");
      Serial.println(V_average);
      */
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

      if (getTicksDuration(prevTouchTime, millis()) >= 5000) {
        digitalWrite(FAN_STBY, LOW);
        digitalWrite(FAN_AIN1, HIGH);
        analogWrite(FAN_PWM, 0);
      }

    }
  } // forever

  PT_END(disp);
}

static int threadKick(struct pt* sol) // Worker Thread
{
  PT_BEGIN(sol);

  for (;;)
  {
    PT_WAIT_UNTIL(sol, bKick_Start);
    // digitalWrite(SOLENOID_PIN, HIGH);
    //PORTB |= (1 << 4);

    prevTime2 = millis();
    // PT_WAIT_WHILE(sol, getTicksDuration(prevTime2, millis()) < 200);
    PT_WAIT_UNTIL(sol, (millis() - prevTime2) >= 200);
    
    //PORTB &= ~(1 << 4);
    // digitalWrite(SOLENOID_PIN, LOW);
    bKick_Start = false;
  }

  PT_END(sol);
}
