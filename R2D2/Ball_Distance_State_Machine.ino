/*
The following are all defined constants
#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define EULER 2.718281828459045235360287471352

#define SERIAL  0x0
#define DISPLAY 0x1

#define LSBFIRST 0
#define MSBFIRST 1

#define CHANGE 1
#define FALLING 2
#define RISING 3
*/

//Define all macros
#define BALL_DIAMETER 4.27
//ball diameter in cm
#define OUT1 -1
#define OUT2 -1
//the OUT1 and OUT2 pins refer to analog pins and we read them to figure out whether to start or stop
//remember to change the OUT1 and OUT2 pins when we decide on it
#define ANALOG_RANGE 3.3
#define ANALOG_MAX 1023.0
//range of analog voltages and the maximum possible analog reading
#define ON 3.3
#define OFF 0.0
//voltage for on and off signals
#define GOAL_LUX 100
//how much lux we want the huskylens to receive
#define MAX_MILLISECONDS 400 //0.4 second
//how many milliseconds a huskylens function can run for

/*
Coordinate system
(0, 0)                 (320, 0)

           (160, 120)

(0, 240)               (320, 240)
*/

pt ptEyelid;
pt ptHuskylens;
pt pt_ball_distance;
pt pt_goal_distance;
pt pt_lens_adjustment;

//variables to test semaphores and reading from a shared 32-bit variable
pt_sem sem_ball, sem_goal;

struct location {
  int x, y;
};

//Huskylens variables
HUSKYLENS huskylens;
SoftwareSerial serial(10, 11);

//ball distance variables
float ball_size_10 = 120.0;
float ball_current_distance;
float ball_current_size;
bool ball_success;
location ball_location;

//object recognition state machine
typedef enum object_recognition_state {
  start,
  initialization,
  test_for_object,
  check_time,
  evaluate_distance,
  finish
};

//do NOT write to any of these variables except for is_most_recent
typedef struct object_recognition_results {
  bool is_most_recent = false; //this will be changed to true once the Huskylens writes to it
  //once someone reads it, it should be set to false so the same information isn't used again
  float object_distance;
  float object_size;
  location object_location;
  //stores the x and y coordinates of the ball (do object_location.x or object_location.y to get the x and y values)
};

//Initialize functions
int ball_distance(struct pt* pt);
int eyelidThread(struct pt* pt);
int goal_distance(struct pt* pt);

object_recognition_state ball_state = start;
object_recognition_results ball_results;
unsigned long ball_start_millis;

void setupVision(void) {
  serial.begin(9600);
  
  PT_INIT(&ptEyelid);
  PT_INIT(&ptHuskylens);
  PT_INIT(&pt_ball_distance);
  PT_INIT(&pt_goal_distance);
  PT_INIT(&pt_lens_adjustment);
  PT_SEM_INIT(&sem_ball, 0);
  PT_SEM_INIT(&sem_goal, 0);
  
  #ifdef CAMERA_THREADS
    while (!huskylens.begin(Wire)) Serial.println("Begin failed!");
  #endif
}

int ball_distance(struct pt* pt) {
  PT_BEGIN(pt);
  for (;;) {
    //Serial.print("Ball state: ");
    //Serial.println(ball_state);
    if (ball_state == start) {
      ball_start_millis = millis();
      ball_state = initialization;
      //have to go to initialization
    } else if (ball_state == initialization) {
      huskylens.writeAlgorithm(ALGORITHM_OBJECT_TRACKING);
      ball_state = test_for_object;
      //initialize everything and test for ball
    } else if (ball_state == test_for_object) {
      //Serial.println("Ball started!");
      //failure leads to checking the time
      if (!huskylens.request()) ball_state = check_time;
      else if (!huskylens.isLearned()) ball_state = check_time;
      else if (!huskylens.available()) ball_state = check_time;
      else ball_state = evaluate_distance;
      //otherwise find the distance
    } else if (ball_state == check_time) {
      if (millis() - ball_start_millis > MAX_MILLISECONDS) {
        //change the confidence bit to 0 (not confident)
        ipc_comms &= ~0b1;
        //set the count to 0 so no other threads can view
        sem_ball.count = 0;
        //went over time limit, so go to finish
        ball_state = finish;
      } else ball_state = test_for_object;
    } else if (ball_state == evaluate_distance) {
      HUSKYLENSResult result = huskylens.read();
      if (result.command != COMMAND_RETURN_BLOCK) ball_state = initialization; //if initialization somehow went wrong
      else {
        ball_current_size = (static_cast<float>(result.width) + static_cast<float>(result.height)) / 2.0;
        ball_current_distance = (ball_size_10 * 10) / ball_current_size;
        ball_location = {result.xCenter, result.yCenter};
        ball_results.object_distance = ball_current_distance;
        ball_results.object_size = ball_current_size;
        ball_results.object_location = ball_location;
        ball_results.is_most_recent = true;
        Serial.print("Ball distance: ");
        Serial.println(ball_current_distance);
        Serial.print("Location: (");
        Serial.print(ball_location.x);
        Serial.print(", ");
        Serial.print(ball_location.y);
        Serial.println(")");
        //change the confidence bit to 1 (confident)
        ipc_comms |= 0b1;
        //allow other threads to read the results struct
        PT_SEM_SIGNAL(pt, &sem_ball);
        ball_state = finish; //successfully found the distance, so go to finish
      }
    } else if (ball_state == finish) {
      //debug prints for the semaphore and shared flags
      Serial.print("Ball semaphore value: ");
      Serial.println(sem_ball.count);
      Serial.print("Ball confidence: ");
      Serial.println(ipc_comms & 1);
      ball_state = start;
      PT_SLEEP(pt, 1000);
      //recalculate after 1 second
    } else {
      //change the confidence bit to 0 (not confident)
      ipc_comms &= ~0b1;
      //set the count to 0 so no other threads can view
      sem_ball.count = 0;
      //ball_state doesn't match anything
      ball_state = initialization;
    }
  }
  PT_END(pt);
}
