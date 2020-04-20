#include <Stepper.h>
#include <Encoder.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <L298N.h>

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
#define GPS_RX = 0 
#define GPS_TX = 1

// Motor control output pins
// All of these pins are PWM enabled if necessary. YC, PC => Yaw Control, Pitch Control
#define PC_PWM = 9
#define PC_1 = 10
#define PC_2 = 11

#define YC_PULSE = 3
#define YC_DIR = 4

// Motor encoder input pins. PE = Pitch Encoder
#define PE_1 = 12
#define PE_2 = 13

void setup() {
  // put your setup code here, to run once:
  // Initialize buttons for start, manual activation for nodes
  // Initialize control booleans, encoder data, etc

  //
  // CALLIBRATION CONSTANTS
  //
  // For your editing pleasure.
  int PITCH_PWM = 50; // PWM Size
  int PITCH_THRESH = 5; // 200 Encoder steps per revolution = +/- 2.5% error.
  int YAW_THRESH = 5; // 200 steps per revolution = +/- 2.5% error.

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
  

  // OTHER HARDWARE

  // Start/reset button input
  pinMode(START, INPUT);


  // Init serial communications
  SerialMonitor.begin(9600);
  gpsPort.begin(9600);


}

// Main operational loop for FSM/umbrella control
void loop() {

  // Read all pins into 
  if (STATE == SEED) {
    seed();
  }
  else if (STATE == GPS) {
    gps();
  }
  else if (STATE == CALC) {
    calc();
  }
  else if (STATE == MOVE) {
    move_pose();
  }
  else if (STATE == SLEEP) {
    sleep(SLEEP_INTERVAL);
  }
}



int readGPS(); {
  // Read most current GPS information.
  
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


// Return declanation angle in radians based on current day of year. (01/01 = 1, 12/31 = 365)
float declination(int day) {
  return 23.45 * (PI / 180) * sin(2 * PI * ((284 + day) / 36.25));
}

float hour_angle(int day) {
  float longitude = ((int) tinyGPS.location.lng()) % 15;
  float LT = 60 // in minutes???
  float B = 360 * (day - 81) / 365
  float ET = (9.87 * sin(2 * B)) - (7.54 * cos(B)) - (1.5 * cos(B))
  float ST = LT + (ET / 60) + ((4 / 60) * (longitude));
  return 15 * (12 - ST);
}

// Seed the pose of our umbrella. Read inputs from manual motor controls, as well as start button.
// Loops until start button pressed, outputting motor control to specified motors as instructed.
void seed() {

  if (digitalRead(START) == true) {
    STATE = 1;
    PITCH_ORIG = PITCH_CURR;
    YAW_ORIG = YAW_CURR;
  }
  
  while (digitalRead(START) == true);
  delay(20);
  
  while (digitalRead(PM_POS) == HIGH) {
    // Drive pitch motor in positive direction
    pitchMotor.forward();
    PITCH_CURR = pitchEnc.read();
    delay(5);
  }
  pitchMotor.stop();
  
  while (digitalRead(PM_NEG) == HIGH) {
    // Drive pitch motor in negative direction
    pitchMotor.backward();
    PITCH_CURR = pitchEnc.read();
    delay(5);
  }
  pitchMotor.stop(); 
   
  while (digitalRead(YM_POS) == HIGH) {
    // Drive yaw motor in positive direction (CCW)
    step_yaw(1);
    YAW_CURR++:
  }
  
  while (digitalRead(YM_NEG) == HIGH) {
    // Drive yaw motor in negative direction (CW)
    step_yaw(0);
    YAW_CURR--;
  }
  
}

void gps() {
  int gps_status = readGPS();

  if (gps_status == 2) {
    STATE = SLEEP;
  }
  else {
    STATE = CALC;
    LATITUDE = tinyGPS.location.lat();
    LONGITUDE = tinyGPS.location.lng();
    TIME = tinyGPS.time.value();
    DAY = tinyGPS.date.value();
  }
}

void calc() {
  // DO_CALCULATIONS
  
  // CHECK VALIDITY OF DETERMINED POSE (outside of tilt max/min?)
  // SET UP DESIRED VALUES IN BOTH RAD AND ENCODER BASES
  
}

void move_pose_reset() {
  reset_pitch();
  move_pose
}

void move_pose() {
  // Begin with yaw actuation
  
  if (YAW_TARGET - YAW_CURRENT > 0) {
    YAW_DIRECTION = 1;
  } else {
    YAW_DIRECTION = 0;
  }
  
  if (PITCH_TARGET - PITCH_CURRENT > 0) {
    PITCH_DIRECTION = 1;
  } else {
    PITCH_DIRECTION = 0;
  }
  
  while (!close_enough_yaw()) {
    step_yaw(YAW_DIRECTION);
  }

  while (!close_enough_pitch()) {
    if (PITCH_DIRECTION == 0) {
      pitchMotor.forward();
    } else {
      pitchMotor.backward();
    }
    delay(PITCH_DELAY);
    pitchMotor.stop();
  }
}

void sleep(int duration) {
  
  delay(duration);
  
  // Optimization: Here, we could potentially conduct calculations for the future pose, given our current time and GPS coordinates.
  // Pre-emptively find future pose to lessen the computational load when attempting to find a new pose.
}

//
// HELPER FUNCTIONS
//

// Singular step of Yaw motor in DIRECTION
void step_yaw(int dir) {
  digitalWrite(YC_DIR, dir);
  digitalWrite(YC_PULSE, HIGH);
  delay(YAW_DELAY);
  digitalWrite(YC_PULSE,LOW); 
}


void kill_motors() {
  pitchMotor.stop()
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
  return pitch - PITCH_ORIG;
}

int norm_yaw(int yaw) {
  return yaw - YAW_ORIG;
}

bool close_enough_yaw() {
  return (abs(YAW_ORIG - YAW_CURR) < YAW_THRESH);
}

bool close_enough_pitch() {
  return (abs(PITCH_ORIG - PITCH_CURR) < PITCH_THRESH);
}
