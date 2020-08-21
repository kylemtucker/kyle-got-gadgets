
// Only two pins that allow interrupts across all arduino boards
#define LEFT_SIGNAL 2
#define RIGHT_SIGNAL 3

#define LEFT_CONTROL 4
#define RIGHT_CONTROL 5

#define DELAY_MS 750

// NECESSARY GLOBALS
// Define button states. 0 is open, 1 is closed (pressed)
int leftState = 0;
int rightState = 0;
int blinkState = 0; // State of blinker, are we blinking on (1)? or off (0)?
int lastState = 0; // (1 is left, 2 is right, 3 is both, 0 is off)

unsigned long lastMillis = 0;

void setup() {
  // put your setup code here, to run once:

  // Initialization routine
  pinMode(LEFT_SIGNAL, INPUT);
  pinMode(RIGHT_SIGNAL, INPUT);
  
  // Initialize button interrupt routines. Done this way because I wanted to learn how to use interrupts!
  // TODO: Implement button safeguards to avoid dual-activation mid-press!
  attachInterrupt(digitalPinToInterrupt(LEFT_SIGNAL), LEFT_ISR_RISING, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_SIGNAL), RIGHT_ISR_RISING, CHANGE);

  // attachInterrupt(digitalPinToInterrupt(LEFT_SIGNAL), LEFT_ISR_FALLING, FALLING);
  // attachInterrupt(digitalPinToInterrupt(RIGHT_SIGNAL), RIGHT_ISR_FALLING, FALLING);

  pinMode(LEFT_CONTROL, OUTPUT);
  pinMode(RIGHT_CONTROL, OUTPUT);
  
}

void loop() {
  unsigned long now = millis();
  if (now - lastMillis > DELAY_MS) {
    lastMillis = now;
    blinkState = !blinkState;
  }
  if (leftState && blinkState) {
    left_on();
  }
  if (rightState && blinkState) {
    right_on();
  }
  if (!blinkState) {
    both_off();
  }
}

void LEFT_ISR_RISING() {
  leftState = !leftState;
}

void RIGHT_ISR_RISING() {
  rightState = !rightState;
}

void LEFT_ISR_FALLING() {
  leftState = !leftState;
}

void RIGHT_ISR_FALLING() {
  rightState = !rightState;
}

void both_on() {
  left_on();
  right_on();
}

void right_on() {
  digitalWrite(RIGHT_CONTROL, HIGH);
}

void left_on() {
  digitalWrite(LEFT_CONTROL, HIGH);
}

void both_off() {
  left_off();
  right_off();
}

void left_off() {
  digitalWrite(LEFT_CONTROL, LOW);
}

void right_off() {
  digitalWrite(RIGHT_CONTROL, LOW);
}
