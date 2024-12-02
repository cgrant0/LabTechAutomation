#include <Servo.h>

// Horizontal motor pin definitions
const int H_MOTOR_STEP_PIN = NULL;
const int H_MOTOR_DIR_PIN = NULL;
const int H_MOTOR_EN_PIN = NULL;
#define H_FORWARD true
#define H_BACKWARD false 
bool h_calibrated = false;

// Vertical motor pin definitions
const int V_MOTOR_STEP_PIN = NULL;
const int V_MOTOR_DIR_PIN = NULL;
const int V_MOTOR_EN_PIN = NULL;
#define V_UP true // NOTE: may need to update these
#define V_DOWN false
bool v_calibrated = false;

int currStep = 192000;

// Servo moto pin definitions
const int SERVO_PIN = NULL;
const int UL_DELAY = NULL; // experimentally determined value it takes servo to change volume by 0.1uL
Servo volumeServo;
float volume = 98.4; // random starting value for now, could set to real pipette value
const int VOLUME_INCREASE_SIGNAL = 100;
const int VOLUME_DECREASE_SIGNAL = 80;
const int VOLUME_CONSTANT_SIGNAL = 88;


// Dispense Linear actuator pin definitions
const int AIN1_PIN = NULL;
const int BIN1_PIN = NULL;
const int AIN2_PIN = NULL;
const int BIN2_PIN = NULL;
const int PWMA_PIN = NULL;
const int PWMB_PIN = NULL;
const int STBY_PIN = NULL;

// Optical limit switch pins definition
const int H_LIMIT_PIN = NULL;
const int V_LIMIT_PIN = NULL;
const int LIMIT_DELAY = 100;

void setup() {
  Serial.begin(9600);

  // Servo pin setup
  volumeServo.attach(SERVO_PIN);
  volumeServo.attach(VOLUME_CONSTANT); // found writing 88 should be no motion

  // Horizontal motor pin setup
  pinMode(H_MOTOR_STEP_PIN, OUTPUT); 
  pinMode(H_MOTOR_DIR_PIN, OUTPUT);
  pinMode(H_MOTOR_EN_PIN, OUTPUT);
  digitalWrite(H_MOTOR_EN_PIN, LOW);

  // Vertical motor pin setup
  pinMode(V_MOTOR_STEP_PIN, OUTPUT); 
  pinMode(V_MOTOR_DIR_PIN, OUTPUT);
  pinMode(V_MOTOR_EN_PIN, OUTPUT);
  digitalWrite(V_MOTOR_EN_PIN, LOW);

  // Linear actuator pin setup
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  // calibrate horizontal movement
  if (!calibrateHorizontal(87)) println("Hopefully this bad boy didn't break");
  else h_calibrated = true;

  if (!calibrateVertical(20)) println("Pipette case might be broken");
  else v_calibrated = true;
}

// 1 rotation is 8mm 
// distance is in mm
//speed is represents the delay 
// dir, true for cw (forward), false for ccw (backward)
void moveHorizontal(bool dir, int speed, int distance){
  int steps = 0;
  int signalValue;
  steps = distance * 3200; // right now distance is how many rotations will need to fix.
  bool move = false;
  if(dir) {
    signalValue = HIGH;
    //digitalWrite(HorizontalDirPin,HIGH); //clockwise (forward movement)
    if(steps + currStep <= 275200) {
      move = true;
      currStep = currStep + steps;
    }
  } else {
    signalValue = LOW;
    //digitalWrite(HorizontalDirPin,LOW); //Counter-clockwise (backward movement)
    if(currStep - steps >= 0) {
      move = true;
      currStep = currStep - steps;
    }
  }

  if(move) {
    digitalWrite(H_MOTOR_DIR_PIN, signalValue);
    for(int x = 0; x < steps; x++) { //amount of steps you want to move  
      digitalWrite(H_MOTOR_STEP_PIN, HIGH); 
      delayMicroseconds(120); 
      digitalWrite(H_MOTOR_STEP_PIN, LOW); 
      delayMicroseconds(1); 
    }
  }
}

// 1 rotation is 8mm 
// distance is in mm
//speed is represents the delay 
// dir, true for cw (forward), false for ccw (backward)
void moveVertical(bool dir, int speed, int distance){
  int steps = 0;
  int signalValue;
  steps = distance * 3200; // right now distance is how many rotations will need to fix.
  bool move = false;
  if(dir) {
    signalValue = LOW; // high goes down, low goes up
    if(steps + currStep <= 275200) {
      move = true;
      currStep = currStep + steps;
    }
  } else {
    signalValue = HIGH;
    //digitalWrite(HorizontalDirPin,LOW); //Counter-clockwise (backward movement)
    if(currStep - steps >= 0) {
      move = true;
      currStep = currStep - steps;
    }
  }

  if(move) {
    digitalWrite(V_MOTOR_DIR_PIN, signalValue);
    for(int x = 0; x < steps; x++) { //amount of steps you want to move  
      digitalWrite(V_MOTOR_STEP_PIN, HIGH); 
      delayMicroseconds(120); 
      digitalWrite(V_MOTOR_STEP_PIN, LOW); 
      delayMicroseconds(1);
    }
  }
}

bool readHorizontalLimit() {
  return (analogRead(H_LIMIT_PIN) * 3.3/1023) == 1;
}

bool readVerticalLimit() {
  return (analogRead(V_LIMIT_PIN) * 3.3/1023) == 1;
}

// Basic idea to set up horizontal movement, move until limit switch is hit
// timeout is max rotations before calling it for safety ig
  // could set timeout to 87, ideally limit switch should trigger before then
bool calibrateHorizontal(int timeout) {
  int rotations = 0;
  while (!readHorizontalLimit()) {
    moveHorizontal(H_BACKWARD, 120, 1); // PUNEET: idk what distance translates to, maybe just move it the lowest amount
    delay(100);

    if (++rotations >= timeout) return false;
  }

  // move until just out of limit switch
  while (readHorizontalLimit()) {
    moveHorizontal(H_FORWARD, 120, 1); // PUNEET: move like one step forward, again idk what this means lowkey
    delay(100);
  }
  
  return true;
}

// PUNEET: if we use vertical limit switch may need to update directions of calibration to accomodate position of switch.
  // ie - if switch is at the top, move V_UP first, then V_DOWN. 
bool calibrateVertical(int timeout) {
  int rotations = 0;
  while (!readVerticalLimit()) {
    moveVertical(V_DOWN, 120, 1); // PUNEET: idk what distance translates to, maybe just move it the lowest amount
    delay(100);

    if (++rotations >= timeout) return false; // failed
  }

  // move until just out of limit switch
  while (readVerticalLimit()) {
    moveVertical(V_UP, 120, 1); // PUNEET: move like one step forward, again idk what this means lowkey
    delay(100);
  }
  
  return true; // wooo success
}

// Changes volume to newVolume using servo
void changeVolume(float newVolume) {
  if (volume == newVolume) return;

  float dist = volume - newVolume;
  bool dir = dist < 0 ? true : false;

  if (dir) dist = abs(dist);

  int rotationTime = (dist * 10.0) / UL_DELAY;

  if (dir) volumeServo.write(VOLUME_INCREASE_SIGNAL);
  else volumeServo.write(VOLUME_DECREASE_SIGNAL);
  delayMicroseconds(rotationTime);
  volumeServo.write(VOLUME_CONSTANT_SIGNAL);

  volume = newVolume;
}

void eject() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(PWMA, HIGH); // Enable motor A
  delay(1386);              // Run UP for 1.386 seconds

  // Stop motor
  digitalWrite(PWMA, LOW); // Turn off motor
  delay(1000);             // Wait 1 second

  // Set motor direction for DOWN
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(PWMA, HIGH); // Enable motor A
  delay(1386);              // Run DOWN for 5 seconds
  
  // Stop motor
  digitalWrite(PWMA, LOW); // Turn off motor
  delay(1000);             // Wait 1 second
}

void loop() {
  // put your main code here, to run repeatedly:
  if (!horizontalCalibrated || !verticalCalibrated) return; // don't run if calibration failed

  // Write as global pos from reset state
  int troughOnePos = NULL;
  int troughVerticalDist = NULL;
  int wellPlateVerticalDist = NULL;
  int wellPlateRowOnePos = NULL;
  int wellPlateRowTwoPos = NULL;
  int disposalTroughPos = NULL;
  
  // Move to trough one
  moveHorizontal(H_FORWARD, 120, troughOnePos);
  // Move down and up
  moveVertical(V_DOWN, 120, troughVerticalDist);
  delay(1000); // wait 1s to emulate liquid intake
  // NOTE: add button press here if desired (intake)
  moveVertical(V_UP, 120, troughVerticalDist);

  // Move to well plate
  moveHorizontal(H_FORWARD, 120, wellPlateRowOnePos - troughOnePos); // Global pos adjustment - ie; well plate is 20cm from reset pos, trough is 5cm from reset pos
  // Move down and up
  moveVertical(V_DOWN, 120, wellPlateVerticalDist);
  delay(1000); // wait 1s to emulate liquid dispense
  // NOTE: add button press here if desired (dispense)
  moveVertical(V_UP, 120, wellPlateVerticalDist);
  
  // Move back to trough one
  moveHorizontal(H_BACKWARD, 120, wellPlateRowOnePos - troughOnePos);
  // Move down and up
  moveVertical(V_DOWN, 120, troughVerticalDist);
  delay(1000); // wait 1s to emulate liquid intake
  // NOTE: add button press here if desired (intake)
  moveVertical(V_UP, 120, troughVerticalDist);

  // Move to different row in well plate
  moveHorizontal(H_FORWARD, 120, wellPlateRowTwoPos - troughOnePos);
  // Move down and up
  moveVertical(V_DOWN, 120, wellPlateVerticalDist);
  delay(1000); // wait 1s to emulate liquid dispense
  // NOTE: add button press here if desired (dispense)
  moveVertical(V_UP, 120, wellPlateVerticalDist);

  // Move to disposal bin
  moveHorizontal(H_FORWARD, 120, disposalTroughPos - wellPlateRowTwoPos);
  // drop pipette tips
  eject();

  // Return to home
  if (calibrateHorizontal()) println("Successful run.");

  return;
}
