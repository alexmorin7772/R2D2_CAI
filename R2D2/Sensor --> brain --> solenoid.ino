#define SOLENOID_PIN 8
pinMode(SOLENOID_PIN, OUTPUT);
analogWrite(SOLENOID_PIN, 0);

// Brain.ino — Reads ball/goal sensor data using semaphores,
// decides when the solenoid should kick, and sends the command.
// The solenoid person writes the physical kick code separately.


// Kick threshold: 0.1 cm = 1 mm
// If the ball is this close or closer, fire the solenoid at max
#define KICK_DISTANCE 0.1

// How hard to kick (0-255). 255 = maximum effort
#define KICK_POWER_MAX 255

// Medium kick for when ball is close but not touching
#define KICK_POWER_MED 180

// Close enough to consider kicking (in cm)
#define KICK_RANGE 2.0

// Cooldown between kicks so the solenoid doesn't burn out (ms)
#define KICK_COOLDOWN 500

// Pin that controls the solenoid — change to your actual pin
#define SOLENOID_PIN 8

// How long the solenoid stays fired (ms)
#define KICK_DURATION 80

// What the brain tells the solenoid to do
typedef enum kick_command {
  KICK_IDLE,       // do nothing
  KICK_READY,      // ball is in range, prepare
  KICK_NOW_MAX,    // ball is 1mm away — fire at full power
  KICK_NOW_MED     // ball is close but not touching — medium kick
};

// This struct is what the brain outputs for the solenoid code to read
struct kick_instruction {
  kick_command command;    // what to do
  int power;               // PWM value 0-255
  float ball_dist;         // how far the ball is (for debugging)
  bool ball_confident;     // true if sensor data is trustworthy
  bool goal_centered;      // true if goal is lined up
};

// The solenoid code reads this variable to know what to do
kick_instruction kick_order = {KICK_IDLE, 0, 999.0, false, false};

// Protothread for the brain
pt pt_brain;

// Track when we last kicked to enforce cooldown
unsigned long last_kick_time = 0;


// check_goal_aligned() — returns true if the goal is visible,
// confident, and roughly centered in the camera frame.

bool check_goal_aligned() {
  // Check goal confidence bit (bit 1 of ipc_comms)
  if (!(ipc_comms & 0b10)) return false;

  // Goal x position: 160 is center, allow 30px deadzone
  int goal_offset = goal_results.object_location.x - 160;
  return (abs(goal_offset) < 30);
}


// brain_loop() — every 2 seconds, reads sensors via semaphores,
// checks distances, and writes a kick_instruction for the
// solenoid code to act on.

int brain_loop(struct pt* pt) {
  PT_BEGIN(pt);
  for (;;) {

    // Wait until ball state machine signals fresh data
    PT_SEM_WAIT(pt, &sem_ball);

    // Check ball confidence (bit 0 of ipc_comms)
    kick_order.ball_confident = (ipc_comms & 0b1);

    if (kick_order.ball_confident) {
      // Read the ball distance from the sensor results
      kick_order.ball_dist = ball_results.object_distance;

      // Check if goal is lined up
      kick_order.goal_centered = check_goal_aligned();

      // Ball is 1mm or less — MAXIMUM KICK
      if (kick_order.ball_dist <= KICK_DISTANCE) {
        // Only kick if cooldown has passed
        if (millis() - last_kick_time > KICK_COOLDOWN) {
          kick_order.command = KICK_NOW_MAX;
          kick_order.power = KICK_POWER_MAX;

          // Fire the solenoid at full power
          digitalWrite(SOLENOID_PIN, KICK_POWER_MAX);
          PT_wait(KICK_DURATION);
          digitalWrite(SOLENOID_PIN, 0);

          last_kick_time = millis();

          Serial.println("BRAIN: KICK MAX — ball at 1mm!");
        }
      }
      // Ball is within 2cm — medium kick if goal is aligned
      else if (kick_order.ball_dist <= KICK_RANGE && kick_order.goal_centered) {
        if (millis() - last_kick_time > KICK_COOLDOWN) {
          kick_order.command = KICK_NOW_MED;
          kick_order.power = KICK_POWER_MED;

          // Fire solenoid at medium power
          digitalWrite(SOLENOID_PIN, KICK_POWER_MED);
          PT_wait(KICK_DURATION);
          digitalWrite(SOLENOID_PIN, 0);

          last_kick_time = millis();

          Serial.println("BRAIN: KICK MED — ball close + goal aligned");
        }
      }
      // Ball is in view but not close enough to kick
      else {
        kick_order.command = KICK_READY;
        kick_order.power = 0;
      }
    }
    // Sensor data not confident — don't kick
    else {
      kick_order.command = KICK_IDLE;
      kick_order.power = 0;
      kick_order.ball_dist = 999.0;
    }

    // Debug output
    Serial.print("BRAIN | dist=");
    Serial.print(kick_order.ball_dist);
    Serial.print(" conf=");
    Serial.print(kick_order.ball_confident);
    Serial.print(" goal=");
    Serial.print(kick_order.goal_centered);
    Serial.print(" cmd=");
    Serial.println(kick_order.command);

    // Wait 2 seconds before next cycle
    PT_SLEEP(pt, 2000);
  }

  PT_END(pt);
}
