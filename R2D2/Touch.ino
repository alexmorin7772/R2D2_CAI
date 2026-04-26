/*
Touch.ino, by Alex
Reading ADC values from the Touch Sensor
This code will protect the readSensor A_pos with a semaphore so that only exclusive access by the thread can update it.
Overall includes viable examples of semaphores for reference
*/

#include "touch.h"

struct pt_sem semTouch;

volatile float V_datapoints[4];

volatile boolean bKick = false;
volatile boolean bKick_Start = false;
volatile boolean bPresent = false;
volatile boolean bBeastMode = false;

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

int threadADCRead(struct pt* pos)
{
  static int i = 0;
  
  PT_BEGIN(pos);

  for (;;)
  {
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
      }
    }

    PT_SLEEP(pos, 1);
  } // forever

  PT_END(pos);
}

int threadDisplay(struct pt* disp)
{
  static float V_value;
  static float V_average;
  static boolean bKick_Again = false;

  PT_BEGIN(disp);

  for (;;)
  { 
    PT_SEM_WAIT(disp, &semTouch);
    if (Check_Available_Brain_Event()) {
      ipc_comms |= MASK_IPC_Touch_Read_Confirmation; // Touch confirms that it read from brain (turns the bit from 0 to 1)
      // ipc_comms &= ~MASK_IPC_Brain_To_Touch; // Touch resets the brain event to open for new ones (turns the bit back from 1 to 0 again)

      if ( (bKick == true) && (bKick_Start == false) ) {  // Kick when not already active - (including solenoid reset time)
        bKick = false;
        // digitalWrite(LED_BUILTIN, LOW); // Kick execution starts (LOW)
        bKick_Start = true;
        // PORTB &= ~(1 << 5);
      }
      
      if (bBeastMode == true) {
        bKick_Again = true;
      }
    }
    PT_SEM_SIGNAL(disp, &semTouch);

    // PT_SLEEP(disp, 2);  // Sleep here for 1 ms to let worker thread execute the start of the solenoid kick

    Serial.print(5.0);
    Serial.print(",");

    // if(A_pos > 1)
    //   offset = 2;
    // else
    //   offset = 0;
    
    V_value = (5.0/1024)*29*A_pos;
    V_average = (V_datapoints[0] + V_datapoints[1] + V_datapoints[2] + V_datapoints[3]) / 4.0; // Used array to find average between 4 shifting Volt values

    if (V_average >= 0.01) {
      Serial.print(V_value); // removed offset from the original equation: Serial.println((5.0/1024)*20*A_pos+offset);
      Serial.print(",");
      Serial.print(4.0);
      // (ipc_comms  MASK_IPC_TOUCH_TO_BRAIN)
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

      if (bKick_Again) {
        bKick_Start = true;
        bKick_Again = false;
      }
    } 
    else {
      Serial.print(0.0);
      Serial.print(",");
      Serial.print(5.0);
      PT_SEM_WAIT(disp, &semTouch);
      // Before a new (or updated) event can be posted to Brain, it must first clear the confirmation flag from Brain
      if ( (ipc_comms & MASK_IPC_Touch_To_Brain) && (ipc_comms & MASK_IPC_Brain_Read_Confirmation) ) { // The last event was consumed; solution to above comment
        ipc_comms &= ~MASK_IPC_Brain_Read_Confirmation;
      }
      Post_Touch_Event_to_Brain();
      // ipc_comms &= ~MASK_IPC_Touch_To_Brain; // Turned off touch event to not re-trigger "Check_Available_Touch_Event" again
      bPresent = false;
      PT_SEM_SIGNAL(disp,&semTouch);
    }
    
    Serial.print(",");
    Serial.println(V_average);

    PT_SLEEP(disp, 10);
  } // forever

  PT_END(disp);
}

int threadKick(struct pt* sol) // Worker Thread
{
  PT_BEGIN(sol);

  for (;;)
  {
    PT_WAIT_UNTIL(sol, bKick_Start);
    // digitalWrite(SOLENOID_PIN, HIGH);
    PORTB |= (1 << 4);
    
    PT_SLEEP(sol, 200);
    
    PORTB &= ~(1 << 4);
    // digitalWrite(SOLENOID_PIN, LOW);
    bKick_Start = false;
  }

  PT_END(sol);
}

int threadMain(struct pt* brain) {
  PT_BEGIN(brain);
  
  digitalWrite(LED_BUILTIN, LOW); // Initial state
  
  for(;;){
    
    PT_SEM_WAIT(brain, &semTouch);
    
    if ( Check_Available_Touch_Event() ) {  // New event found!
      ipc_comms |= MASK_IPC_Brain_Read_Confirmation;  // Confirm the event
      // digitalWrite(LED_BUILTIN, LOW); // Brain received (LOW)
      // PORTB &= ~(1 << 5);

      if (bPresent) {
        bKick = true;
        PORTB &= ~(1 << 5);

        // Before a new (or updated) event can be posted to Touch, it must first clear the confirmation flag from Touch
        if ( (ipc_comms & MASK_IPC_Brain_To_Touch) && (ipc_comms & MASK_IPC_Touch_Read_Confirmation) ) { // The last event was consumed; solution to above comment
          ipc_comms &= ~MASK_IPC_Touch_Read_Confirmation;
        }
        Post_Brain_Event_to_Touch();
      }
    }
    
    PT_SEM_SIGNAL(brain, &semTouch);

    PT_SLEEP(brain, 1);
  }
  
  PT_END(brain);
}