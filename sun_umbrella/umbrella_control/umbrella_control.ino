// Import necessary packages here
// We will be defining umbrella position through two different variables:
float rot_theta = 0; // IN PI RADIANS [-1, 1]
float ang_theta = 0; // IN PI RADIANS [-1, 1]
float desired_rot = 0;
float desired_ang = 0;
float pi = 3.14159265;
// When the umbrella is 100% vertical and the designated "front" is 
// facing north, rot_theta and ang_theta = 0
// 

int init_

void setup() {
  // put your setup code here, to run once:
  // Use manual buttons to reset location of umbrella on startup
  // Wait for button press signal to proceed, signifying a return to the home position.
  

}

void loop() {
  // put your main code here, to run repeatedly:
  // check for data over serial port. 
  // if desired_rot, desired_ang change, then move umbrella


}

void worm_control(int direction) {
  if (direction > 0) {
    // CCW
  }
  else {
    // CW
  }
  
}

void nema_control(int direction) {
  if (direction > 0) {
    // CCW
  }
  else {
    // CW
  }
}

void angle_umbrella() {
  if (desired_ang > ang_theta) {
    while (desired_ang > ang_theta) {
      // Turn worm gear in positive angular direction.
    }
  }
  else {
     while (desired_ang > ang_theta) {
       // Turn work gear in negative angular direction.
     }
  }
}

void rotate_umbrella() {
  if (ang_theta != 0) {
    desired_ang = 0;
    angle_umbrella();
  }
  if (desired_rot > rot_theta) {
    while (desired_ang > rot_theta) {
      // activate nema stepper for 1 degree a bunch of times. in positive direction (CCW)
    }
  }
  else {
    while (desired_rot < rot_theta) {
      // activate nema stepper in negative direction (CW)
    }
  }
}
