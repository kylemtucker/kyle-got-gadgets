#include <Stepper.h>
#include <Encoder.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <L298N.h>
#include <spa.h> // Solar Positioning Algorithm package

// PHYSICAL UMBRELLA CONSTANTS:
// The umbrella is to be seeded with an "origin" pose of:
// Pitch: 0 Degrees (Perpendicular with ground plane). Positive direction: Towards NORTH (Max angle of +/- (sqrt(2) / 2) pi rads)
// Yaw: 0 Degrees (0 degrees to be aligned with true north). Positive direction: COUNTER CLOCKWISE (Max angle of +/- 1 pi rads)

// STATES:
#define SEED 0   // Manually seed pose
#define GPS 1    // GPS Data collection
#define CALC 2   // Pose calculation
#define MOVE 3   // Motor actuation
#define SLEEP 4  // Sleep
#define ERR 5

//
// PIN DEFINITIONS:
//

// "Begin" button to launch program
#define START 2

// Pitch manual motor control. PM = Pitch Manual
#define PM_POS 7 
#define PM_NEG 8

// Yaw manual motor control. YM = Yaw Manual
#define YM_POS 5
#define YM_NEG 6

// GPS SERIAL PINS
#define GPS_RX 0 
#define GPS_TX 1

// Motor control output pins. YC, PC => Yaw Control, Pitch Control
#define PC_PWM 9
#define PC_1 10
#define PC_2 11

#define YC_PULSE 3
#define YC_DIR 4

// Motor encoder input pins. PE = Pitch Encoder
#define PE_1 12
#define PE_2 13

void setup() {
  
  // CALLIBRATION CONSTANTS
  
  // For your editing pleasure.
  int PITCH_PWM = 50; // PWM Size
  int PITCH_THRESH = 5; // 200 Encoder steps per revolution = +/- 2.5% error.
  int YAW_THRESH = 5; // 200 steps per revolution = +/- 2.5% error.
  int PITCH_STEPS_PER_ROT = 500;
  int YAW_STEPS_PER_ROT = 200;

  //
  // CONTROL CONSTATNTS
  //

  // Delays for controlling speed of stepper, delaying PWM commands to DC motor.
  int SLEEP_INTERVAL = 300000; // Check for pose update every 5 minutes.
  int PITCH_DELAY = 50;  // How frequently should we check our pitch encoder location?
  int YAW_DELAY = 500; // 1 step every 500ms
  int PITCH_THRESH = 5; // Acceptability threshold (heursitic)
  int YAW_THRESH = 5;   // Acceptability threshold (heuristic)

  
  // Conversion constatnts for synthesizing radians and encoder steps.
  const float RAD_PER_PITCH = 1; 
  const float RAD_PER_YAW = PI / 100;

  // Timeouts, in ms
  long GPS_TIMEOUT = 15000;

  // GLOBAL VARS
  
  // Keep track of pitch motor information
  int PITCH_ORIG = 0; // Origin
  int PITCH_CURR = 0; // Curr location
  int PITCH_TARGET = 0; // Target location (encoder steps)
  float PITCH_TARGET_RAD = 0; // Target location (radians)
  int PITCH_DIR = 1; // One for positive direction, 0 for negative direction.


  // Keep track of yaw motor information
  int YAW_ORIG = 0;
  int YAW_CURR = 0;
  int YAW_TARGET = 0;
  float YAW_TARGET_RAD = 0;
  int YAW_DIR = 1; // 1 = CCW, 0 = CW

  // Initialize starting state.
  int STATE = SEED;

  //
  // MOTOR INITIALIZATION
  //
  
  // Manual control inputs
  pinMode(PM_POS, INPUT);
  pinMode(PM_NEG, INPUT);
  
  pinMode(YM_POS, INPUT);
  pinMode(YM_NEG, INPUT);


  // Worm motor output (DC)
  L298N pitchMotor(PC_PWM, PC_1, PC_2);
  pitchMotor.setSpeed(PITCH_PWM);

  // May be replaced with stepper lib stuff
  pinMode(YC_PULSE, OUTPUT);
  pinMode(YC_DIR, OUTPUT);
  
  // ENCODERS
  Encoder pitchEnc(PE_1, PE_2);

  // Init tinyGPS object
  TinyGPSPlus tinyGPS;
  bool FIRST_GPS = false; // Track whether or not we have valid GPS data this launch.

  // GPS data necessary for sun calculations.
  long LAST_GPS_READ = 0;
  float LATITUDE = 0;
  float LONGITUDE = 0;
  float TIME = 0;
  int DAY = 0;
  

  // Init serial read across TX, RX pins
  SoftwareSerial ssGPS(GPS_TX, GPS_RX);

  // Set the two serial ports to be the same, redefine SerialMonitor port.
  #define gpsPort ssGPS
  #define SerialMonitor Serial

  // Solar Positioning Algorithm
  spa_data spa;
  

  // OTHER HARDWARE

  // Start/reset button input
  pinMode(START, INPUT);


  // Init serial communications
  SerialMonitor.begin(9600);
  gpsPort.begin(9600);


}

// Main operational loop for FSM/umbrella control
void loop() {

  // State machine control, choose which branch to traverse depending on STATE
  if (STATE == SEED) {
    seed();
  }
  
  else if (STATE == GPS) {
    kill_switch();
    gps();
  }
  
  else if (STATE == CALC) {
    kill_switch();
    calc();
  }
  
  else if (STATE == MOVE) {
    kill_switch();
    move_pose();
  }
  
  else if (STATE == SLEEP) {
    kill_switch();
    sleep(SLEEP_INTERVAL);
  }
  
  else if (STATE == ERR) {
    error();
  }
  
}



int readGPS(); {
  // Read most current GPS information. 

  // Start time for timeout detection
  long start = millis();
  
  while (gpsPort.available()) {
    tinyGPS.encode(gpsPort.read()); 
    
    if (millis() - start > GPS_TIMEOUT) {
      // Timed out
      if (FIRST_GPS) {
        // No GPS data returned, no previous data to predict with. 
        return 2;
      }
      // No new GPS data, but have old data to work with
      return 1;
    }
  }

  // New GPS data, is valid.
  FIRST_GPS = false;
  LAST_GPS_READ = millis();
  return 0;
}

// Seed the pose of our umbrella. Read inputs from manual motor controls, as well as start button.
// Loops until start button pressed, outputting motor control to specified motors as instructed.
void seed() {

  // Read start button to begin!
  if (digitalRead(START) == HIGH) {
    STATE = 1;
    PITCH_ORIG = PITCH_CURR;
    YAW_ORIG = YAW_CURR;
  }

  // Wait until button released
  while (digitalRead(START) == HIGH);
  delay(20);

  // Move pitch in pos dir.
  while (digitalRead(PM_POS) == HIGH) {
    // Drive pitch motor in positive direction
    pitchMotor.forward();
    PITCH_CURR = pitchEnc.read();
    delay(PITCH_DELAY);
  }
  pitchMotor.stop();

  // Move pitch in neg dir.
  while (digitalRead(PM_NEG) == HIGH) {
    // Drive pitch motor in negative direction
    pitchMotor.backward();
    PITCH_CURR = pitchEnc.read();
    delay(PITCH_DELAY);
  }
  pitchMotor.stop(); 

  // Step yaw in pos dir
  while (digitalRead(YM_POS) == HIGH) {
    // Drive yaw motor in positive direction (CCW)
    step_yaw(1);
    YAW_CURR++:
  }

  // Step yaw in neg dir
  while (digitalRead(YM_NEG) == HIGH) {
    // Drive yaw motor in negative direction (CW)
    step_yaw(0);
    YAW_CURR--;
  }
  
}

void gps() {
  
  // Read GPS information via serial port
  int gps_status = readGPS();

  // No GPS data, must transition to error state.
  if (gps_status == 2) {
    STATE = ERR;
  }

  // Update data in SPA handler, move state to CALC stage.
  else {
    STATE = CALC;
    
    LATITUDE = tinyGPS.location.lat();
    LONGITUDE = tinyGPS.location.lng();
    DATE = tinyGPS.date.value();
    
    spa.year = tinyGPS.date.year();
    spa.month = tinyGPS.date.month();
    spa.day = tinyGPS.date.day();
    spa.hour = tinyGPS.time.hour();
    spa.minute = tinyGPS.time.minute();
    spa.latitude = LATITUDE;
    spa.longitude = LONGITUDE;
    spa.elevation = tinyGPS.altitude.meters();
    spa.timezone = get_timezone_est(LONGITUDE);
  }
}

void calc() {
  // Calculate locations for which we must align our yaw and tilt motors.

  // Do solar calculations
  int spa_status = spa_calculate(&spa);

  // Check that we have valid data
  if (spa_status == 0) {

    // Set target for pitch, yaw to point directly at sun
    PITCH_TARGET_RAD = (PI / 2) - spa.azimuth; // Height from horizon
    YAW_TARGET_RAD = spa.zenith; //  Rotation from 0 deg north

    // Set targets for motors in terms of encoder steps or stepper steps
    set_targets(); 

    // Transition to move state
    STATE = MOVE;
  }
  else {
    
    // Transition to error state
    STATE = ERR;
  }
  
}

void move_pose_with_reset() {

  // Store old target
  int yawTemp = YAW_TARGET;
  int pitchTemp = PITCH_TARGET;

  // Set curr target to origin
  YAW_TARGET = YAW_ORIGIN;
  PITCH_TARGET = PITCH_ORIGIN;

  // Reset to origin
  move_pose();

  // Restore target pose
  YAW_TARGET = yawTemp;
  PITCH_TEMP = pitchTemp;

  // Move to target
  move_pose();
}

void move_pose() {
  // Operates off YAW_TARGET and PITCH_TARGET, which must be set and normalized by origin values before 
  
  // Begin with yaw actuation. Determine positive or negative direction before rotating.
  
  if (YAW_TARGET - YAW_CURRENT > 0) {
    YAW_DIRECTION = 1;
  } else {
    YAW_DIRECTION = 0;
  }

  // Determine positive or negative actuation for tilt.
  
  if (PITCH_TARGET - PITCH_CURRENT > 0) {
    PITCH_DIRECTION = 1;
  } else {
    PITCH_DIRECTION = 0;
  }

  // Step yaw in proper direction until "close enough"
  while (!close_enough_yaw()) {
    step_yaw(YAW_DIRECTION);
  }

  // 'Step' pitch in proper direction until "close enough"
  while (!close_enough_pitch()) {
    if (PITCH_DIRECTION == 0) {
      pitchMotor.forward();
    } else {
      pitchMotor.backward();
    }
    delay(PITCH_DELAY);
    pitchMotor.stop();
  }
  STATE = SLEEP;
}

void sleep(int duration) {

  
  delay(duration);

  STATE = GPS;
  
  // Optimization: Here, we could potentially conduct calculations for the future pose, given our current time and GPS coordinates.
  // Pre-emptively find future pose to lessen the computational load when attempting to find a new pose.
}

void error() {
  // Wait for reset signal.
  kill_motors();
  if (digitalRead(START) == HIGH) {
    STATE = SEED;
  }
  while (digitalRead(START) == HIGH);
  delay(20);
}

//
// HELPER FUNCTIONS
//


// Singular step of Yaw motor in DIRECTION
void step_yaw(int dir) {
  digitalWrite(YC_DIR, dir);
  digitalWrite(YC_PULSE, HIGH);
  delay(YAW_DELAY);
  digitalWrite(YC_PULSE, LOW); 
}


// Stop all actuation.
void kill_motors() {
  pitchMotor.stop();
  digitalWrite(YC_PULSE, LOW);
  digitalWrite(YC_PULSE, LOW);
}

void kill_switch() {
  if (digitalRead(START) == HIGH) {
    STATE = ERR;
  }
  while (digitalRead(START) == HIGH);
  delay(20);
}


// CONVERSION HELPERS

void set_targets() {
  YAW_TARGET = norm_yaw(rad_to_yaw(YAW_TARGET_RAD)) % YAW_STEPS_PER_ROT;
  PITCH_TARGET = norm_pitch(rad_to_pitch(PITCH_TARGET_RAD)) % PITCH_STEPS_PER_ROT;
}

// Convert pitch encoder number to radians
float pitch_to_rad(int pitch) {
  return RAD_PER_PITCH * pitch;
}

// Convert radians to pitch encoder number
int rad_to_pitch(float rad) {
  return (int) (rad / RAD_PER_PITCH);
}

// Convert yaw encoder number to radians
float yaw_to_rad(int yaw) {
  return RAD_PER_YAW * yaw;
}

// Convert radians to yaw encoder number
int rad_to_yaw(float rad) {
  return (int) (rad / RAD_PER_YAW);
}

int norm_pitch(int pitch) {
  return pitch + PITCH_ORIG;
}

int norm_yaw(int yaw) {
  return yaw + YAW_ORIG;
}

// Close enough comparison for yaw
bool close_enough_yaw() {
  return (abs(YAW_ORIG - YAW_CURR) < YAW_THRESH);
}

// Close enough comparison for pitch
bool close_enough_pitch() {
  return (abs(PITCH_ORIG - PITCH_CURR) < PITCH_THRESH);
}

// SOLAR CALCULATION HELPERS

double get_timezone_est(float longitude) {
  double num = ((int) longitude) % 15;
  if (num > 18) {
    return 24 - num;
  }
  return num;
}
