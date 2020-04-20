#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// Borrowed lots from : https://learn.sparkfun.com/tutorials/gps-logger-shield-hookup-guide/example-sketch-tinygps-serial-streaming


void setup() {
  // put your setup code here, to run once:
  // Initialize buttons for start, manual activation for nodes
  // Initialize control booleans, encoder data, etc

 
  // ESSENTIAL CONSTATNTS
  int PITCH_ORIG = 0;
  int PITCH_TARGET = 0;
  
  int YAW_ORIG = 0;
  int YAW_TARGET = 0;

  // Initialize state to 0
  int STATE = 0;
  
  // STATES: 
  // 0 = Seed pose
  // 1 = Read GPS and Time
  // 2 = Calculate Pose
  // 3 = Move Umbrella
  // 4 = Sleep mode
  // 5 = Error mode. Kills umbrella.
  

  // PIN DEFINITIONS
  // "Begin" button to launch program
  const int START = A0;

  // Pitch manual motor control.
  const int P_POS = 12; 
  const int P_NEG = 13;

  // Yaw manual motor control.
  const int Y_POS = 4;
  const int Y_NEG = 7;

  // GPS SERIAL PINS
  const int GPS_RX = 0;
  const int GPS_TX = 1;

  // Motor control output pins
  // All of these pins are PWM enabled if necessary.
  const int PC_1 = 10; // pc = pitch_control
  const int PC_2 = 11;
  
  const int YC_1 = 5; // yc = yaw_control
  const int YC_2 = 6;

  // Motor encoder input pins
  const int PE_1 = 8; // pe = pitch_encoder
  const int PE_2 = 9;

  const int YE_1 = 2; // ye = yaw_encoder
  const int YE_2 = 3;


  // MOTOR PIN INITIALIZATION
  // Manual control inputs
  pinMode(P_POS, INPUT);
  pinMode(P_NEG, INPUT);

  pinMode(Y_POS, INPUT);
  pinMode(Y_NEG, INPUT);

  // Motor output
  pinMode(PC_1, OUTPUT);
  pinMode(PC_2, OUTPUT);

  pinMode(YC_1, OUTPUT);
  pinMode(YC_2, OUTPUT);


  // ENCODERS
  // TODO: Update for use with valid encoder library for ease of use.
  pinMode(PE_1, INPUT);
  pinMode(PE_2, INPUT);

  pinMode(YE_1, INPUT);
  pinMode(YE_2, INPUT);


  // GPS INIT
  // Init tinyGPS object
  TinyGPSPlus tinyGPS;

  // Init serial read across TX, RX pins
  SoftwareSerial ssGPS(GPS_TX, GPS_RX);

  // Set the two serial ports to be the same, redefine SerialMonitor port.
  #define gpsPort ssGPS
  #define SerialMonitor Serial
  

  // OTHER HARDWARE
  // Start button input
  pinMode(START, INPUT);


  SerialMonitor.begin(9600);
  gpsPort.begin(9600);
  

}

void readGPS(); {
  // Read most current GPS information.
  // TODO: Implement timeout
  
  while (gpsPort.available()) {
    tinyGPS.encode(gpsPort.read());
  }
}

// Stolen code xd
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    // If data has come in from the GPS module
    while (gpsPort.available())
      tinyGPS.encode(gpsPort.read());
  } while (millis() - start < ms);
}


void loop() {
  // put your main code here, to run repeatedly:
  if (state == 0) {
    seed();
  }
  else if (state == 1) {
    gps();
  }
  else if (state == 2) {
    calc();
  }
  else if (state == 3) {
    mov_pose();
  }
  else if (state == 4) {
    sleep();
  }
}

void seed() {
  
}

void gps() {
  readGPS();
  
}

void calc() {
  
}

void mov_pose() {
  
}

void sleep() {
  
}
