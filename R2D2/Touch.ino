/*
Touch.ino, by Alex
Reading ADC values from the Touch Sensor
This code will protect the readSensor A_pos with a semaphore so that only exclusive access by the thread can update it.
Overall includes viable examples of semaphores for reference
*/

#include "touch.h"

#define SOLENOID_PIN A6

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

boolean Check_Available_Logic_Event (void) {
  if (ipc_comms & MASK_IPC_Logic_To_Touch) {
    return true;
  }
  return false;
}

void Post_Touch_Event_to_Logic () {
  ipc_comms |= MASK_IPC_Touch_To_Logic;
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
      PT_SEM_WAIT(pos, &semTouch); // This forces a sleep until the semaphore is signaled by another thread or already signaled
      readSensorPos();  // This enables the sensor sampler and interrupt trigger, setting the adcStarted to true
      PT_WAIT_UNTIL(pos, adcStarted == false); // Forces a sleep until adcStarted returns to false (by the ISR interrupt)
      ipc_comms |= MASK_IPC_Touch_To_Logic;
      PT_SEM_SIGNAL(pos, &semTouch); // Releases the next thread th     at was waiting for the semaphore to be signaled
      
      if (i > 4) {
        V_datapoints[i++] = (5.0/1024)*20*A_pos;
      }
      else {
        V_datapoints[0] = V_datapoints[1];
        V_datapoints[1] = V_datapoints[2];
        V_datapoints[2] = V_datapoints[3];
        V_datapoints[3] = (5.0/1024)*20*A_pos;
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
    // turn the ledPin on
    digitalWrite(LED_BUILTIN, HIGH); // Using LED_BUILTIN from pinmode
    // stop the program for <sensorValue> milliseconds:
    // PT_SLEEP(disp, 100);
    
    PT_SEM_WAIT(disp, &semTouch);
    if (Check_Available_Logic_Event()) {
      if (bKick == true) {
        bKick_Start = true;
      }
      
      if (bBeastMode == true) {
        bKick_Again = true;
      }

      ipc_comms |= MASK_IPC_Logic_Read_Confirmation; // Touch confirms that it read from logic (turns the bit from 0 to 1)
      ipc_comms &= ~MASK_IPC_Logic_To_Touch; // Touch resets the logic event to open for new ones (turns the bit back from 1 to 0 again)
    }
    PT_SEM_SIGNAL(disp, &semTouch);

    Serial.print(5.0);
    Serial.print(",");

    // if(A_pos > 1)
    //   offset = 2;
    // else
    //   offset = 0;
    
    V_value = (5.0/1024)*20*A_pos;
    V_average = (V_datapoints[0] + V_datapoints[1] + V_datapoints[2] + V_datapoints[3]) / 4; // Used array to find average between 4 shifting Volt values

    if (V_average >= 0.5) {
      Serial.print(V_value); // removed offset from the original equation: Serial.println((5.0/1024)*20*A_pos+offset);
      Serial.print(",");
      Serial.print(4.0);
      // (ipc_comms  MASK_TOUCH_TO_LOGIC)
      PT_SEM_WAIT(disp, &semTouch);
      Post_Touch_Event_to_Logic();
      bPresent = true;
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
      Post_Touch_Event_to_Logic();
      bPresent = false;
      PT_SEM_SIGNAL(disp,&semTouch);
    }
    
    Serial.print(",");
    Serial.println(V_average);
    
    // turn the ledPin off:
    digitalWrite(LED_BUILTIN, LOW); // Using LED_BUILTIN from pinmode

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
    digitalWrite(SOLENOID_PIN, HIGH);
    
    PT_SLEEP(sol, 200);

    digitalWrite(SOLENOID_PIN, LOW);
    bKick_Start = false;
  }
  PT_END(sol);
}
