#include "protothreads.h"
#include "pt-sem.h"
//#include "SoftwareSerial.h"
#include "HUSKYLENS.h"
#include <Wire.h>
#include <Digital_Light_TSL2561.h>
#include <Servo.h>
#include <SparkFun_TB6612.h>
#include "touch.h"
/*
"R2D2.ino" by team R2D2_CAI in the git repsitory "R2D2_CAI"
To create a "robot" that can "play soccer"
Can Comment/Uncomment lines using #ifdef and #endif
*/

//Define all macros
#define BALL_DIAMETER 4.27
//ball diameter in cm
#define BALL_SIZE_10 120.0
#define GOAL_SIZE_30 100.0
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
#define MAX_MILLISECONDS 200 //0.2 second
//how many milliseconds a huskylens function can run for
#define BALL_CONFIDENCE 0b1
#define GOAL_CONFIDENCE 0b10
#define LINE_CONFIDENCE 0b100
#define FOCAL_LENGTH 234 //focal length of camera
#define BALL_ID 1
#define GOAL_ID 2
//huskylens IDs for the ball and the goal

//Create all protothread structs
//pt ptHuskylens;
pt pt_ball_distance;
pt pt_goal_distance;
pt pt_line_tracking;
pt pt_lens_adjustment;
pt pt_update_motor_state;
pt pt_update_motor_data;
pt ptManager;
pt ptWorker;
pt pt_touch_and_brain;

extern volatile pt solKick;
extern volatile pt ptAlex_test;
extern volatile pt readPos, adcDisp;


//Huskylens variables
HUSKYLENS huskylens;
//SoftwareSerial serial(10, 11);

struct location {
  int x, y;
};

//ball distance variables
float ball_current_distance;
float ball_current_size;
float ball_turn_angle;
int ball_leftmost_x;
int ball_rightmost_x;
location ball_location;

//goal distance variables
float goal_current_distance;
float goal_height;
float goal_turn_angle;
int goal_leftmost_x;
int goal_rightmost_x;
location goal_location;

//line tracking variables
location line_origin, line_target;

/*
Coordinate system
(0, 0)                 (320, 0)

           (160, 120)    

(0, 240)               (320, 240)
*/

volatile bool bKick = false;
volatile bool bKick_Start = false;
volatile bool bPresent = false;
volatile bool bBeastMode = false;

volatile unsigned long prevTime1 = 0;
volatile unsigned long prevTime2 = 0;
volatile unsigned long prevTime3 = 0;
volatile unsigned long prevTime4 = 0;

bool started = false;
//boolean for the start/stop signal

#define SOLENOID_PIN A3

//object recognition state machine
enum object_recognition_state {
  start,
  test_for_object,
  evaluate_distance,
  fail,
  finish
};

enum line_tracking_state {
  line_start,
  line_initialization,
  test_for_line,
  line_check_time,
  process_line,
  line_finish
};

//do NOT write to any of these variables except for is_most_recent
struct object_recognition_results {
  location object_location;
  //stores the x and y coordinates of the object (do object_location.x or object_location.y to get the x and y values)
  float object_distance;
  float object_size;
  float object_turn_angle;
  int leftmost_x, rightmost_x;
  bool is_most_recent = false; //this will be changed to true once the Huskylens writes to it
  //once someone reads it, it should be set to false so the same information isn't used again
  bool object_found = false;
};
object_recognition_results ball_results;
object_recognition_results goal_results;

struct line_tracking_results {
  location origin, target;
  bool is_most_recent = false;
};

//variables to test semaphores and reading from a shared 32-bit variable
pt_sem sem_ball, sem_goal, sem_line, sem_ipc, sem_motor;
extern pt_sem semTouch;
uint32_t ipc_comms = 0;

//state machine variables
int servo_position = 0;

Servo myservo;  // create Servo object to control a servo

int pos = 0;  // variable to store the servo position

enum lens_adjustment_state {
  lens_start,
  read_lux,
  calculate_angle,
  move_servo,
  lens_finish
};

int lux = 0;
int actual_lux = 0;
int angle = 0;

void setup() {
  Serial.begin(115200);
  //serial.being(9600);
  Wire.begin();
  TSL2561.init();
  touchSetup();
  //PT_INIT(&ptHuskylens);
  PT_INIT(&pt_ball_distance);
  PT_INIT(&pt_goal_distance);
  PT_INIT(&pt_line_tracking);
  PT_INIT(&pt_lens_adjustment);
  PT_INIT(&pt_update_motor_state);
  PT_INIT(&pt_update_motor_data);
  PT_INIT(&ptManager);
  PT_INIT(&ptWorker);
  PT_INIT(&pt_touch_and_brain);
  PT_SEM_INIT(&sem_ball, 1);
  PT_SEM_INIT(&sem_goal, 1);
  PT_SEM_INIT(&sem_line, 1);
  PT_SEM_INIT(&sem_ipc, 1);
  PT_SEM_INIT(&sem_motor, 1);
  //myservo.attach(9);  // attaches the servo on pin 9 to the Servo object
  while (!huskylens.begin(Wire)) Serial.println(F("Begin failed!"));
  huskylens.writeAlgorithm(ALGORITHM_COLOR_RECOGNITION);

  // huskyAlgorithm();   // Huskylens - Vision.ino
  // motorSetup();       // Motor Driver - 
  // groveDLSsetup();    // Light Sensor - 
  // touchSetup();       // Touch Sensor - Touch.ino
  // eyelidSetup();      // Servo / Iris - Eyelid.ino
}

void loop() {
  //PT_SCHEDULE(huskyRead(&ptHuskylens));
  PT_SCHEDULE(ball_distance(&pt_ball_distance));
  PT_SCHEDULE(goal_distance(&pt_goal_distance));
  //PT_SCHEDULE(line_tracking(&pt_line_tracking));
  //do not uncomment as line tracking could mess with other Huskylens algorithms
  //PT_SCHEDULE(lens_adjustment(&pt_lens_adjustment));
  PT_SCHEDULE(update_motor_state(&pt_update_motor_state));
  PT_SCHEDULE(update_motor_data(&pt_update_motor_data));
  //PT_SCHEDULE(threadADCRead(&readPos));
  //PT_SCHEDULE(threadDisplay(&adcDisp));
  //PT_SCHEDULE(threadKick(&solKick));
  PT_SCHEDULE(threadMotorWorker(&ptWorker));
  PT_SCHEDULE(threadMotorManager(&ptManager));
  //PT_SCHEDULE(check_touch_ipc(&pt_touch_and_brain));
}
