#include "protothreads.h"
#include "pt-sem.h"
#include "SoftwareSerial.h"
#include "HUSKYLENS.h"
#include <Wire.h>
#include <Digital_Light_TSL2561.h>
#include <Servo.h>

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
#define MAX_MILLISECONDS 200 //0.2 second
//how many milliseconds a huskylens function can run for
#define BALL_CONFIDENCE 0b1
#define GOAL_CONFIDENCE 0b10
#define LINE_CONFIDENCE 0b100
#define FOCAL_LENGTH 234 //focal length of camera

//Create all protothread structs
pt ptEyelid;
pt ptHuskylens;
pt pt_ball_distance;
pt pt_goal_distance;
pt pt_line_tracking;
pt pt_lens_adjustment;

//Huskylens variables
HUSKYLENS huskylens;
SoftwareSerial serial(10, 11);

struct location {
  int x, y;
};

//ball distance variables
float ball_size_10 = 120.0;
float ball_current_distance;
float ball_current_size;
float ball_turn_angle;
location ball_location;

//goal distance variables
float goal_size_30 = 100.0;
float goal_current_distance;
float goal_height;
float goal_turn_angle;
location goal_location;

//line tracking variables
location line_origin, line_target;

/*
Coordinate system
(0, 0)                 (320, 0)

           (160, 120)    

(0, 240)               (320, 240)
*/

//object recognition state machine
enum object_recognition_state {
  start,
  initialization,
  test_for_object,
  check_time,
  evaluate_distance,
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

struct line_tracking_results {
  location origin, target;
  bool is_most_recent = false;
};

//variables to test semaphores and reading from a shared 32-bit variable
pt_sem sem_ball, sem_goal, sem_line;
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

bool started = false;
//boolean for the start/stop signal

void setup() {
  Serial.begin(115200);
  serial.begin(9600);
  Wire.begin();
  TSL2561.init();
  PT_INIT(&ptEyelid);
  PT_INIT(&ptHuskylens);
  PT_INIT(&pt_ball_distance);
  PT_INIT(&pt_goal_distance);
  PT_INIT(&pt_line_tracking);
  PT_INIT(&pt_lens_adjustment);
  PT_SEM_INIT(&sem_ball, 1);
  PT_SEM_INIT(&sem_goal, 1);
  PT_SEM_INIT(&sem_line, 1);
  myservo.attach(9);  // attaches the servo on pin 9 to the Servo object
  while (!huskylens.begin(Wire)) Serial.println("Begin failed!");
  // huskyAlgorithm();   // Huskylens - Vision.ino
  // motorSetup();       // Motor Driver - 
  // groveDLSsetup();    // Light Sensor - 
  // touchSetup();       // Touch Sensor - 
  // eyelidSetup();      // Servo / Iris - Eyelid.ino
}

void loop() {
  //PT_SCHEDULE(eyelidThread(&ptEyelid));
  //PT_SCHEDULE(huskyRead(&ptHuskylens));
  PT_SCHEDULE(ball_distance(&pt_ball_distance));
  PT_SCHEDULE(goal_distance(&pt_goal_distance));
  PT_SCHEDULE(line_tracking(&pt_line_tracking));
  PT_SCHEDULE(lens_adjustment(&pt_lens_adjustment));
}
