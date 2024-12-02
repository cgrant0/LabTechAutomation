#include <Servo.h>
#define AIN1 10
#define BIN1 8
#define AIN2 11
#define BIN2 7
#define PWMA 12
#define PWMB 6
#define STBY 9

// Horizontal motor pin definitions
const int H_MOTOR_STEP_PIN = 3;
const int H_MOTOR_DIR_PIN = 2;
//int H_direction;
//bool h_calibrated = false;

// Vertical motor pin definitions

const int V_MOTOR_STEP_PIN = 4;
const int V_MOTOR_DIR_PIN = 5;
//int V_direction;

//bool v_calibrated = false;

// Servo moto pin definitions

const int SERVO_PIN = 13;
const int UL_DELAY = 1; // experimentally determined value it takes servo to change volume by 0.1uL
Servo volumeServo;
float volume = 98.4; // random starting value for now, could set to real pipette value
const int VOLUME_INCREASE_SIGNAL = 100;
const int VOLUME_DECREASE_SIGNAL = 80;
const int VOLUME_CONSTANT_SIGNAL = 88;


// Dispense Linear actuator pin definitions
/*
const int AIN1_PIN = NULL;
const int BIN1_PIN = NULL;
const int AIN2_PIN = NULL;
const int BIN2_PIN = NULL;
const int PWMA_PIN = NULL;
const int PWMB_PIN = NULL;
const int STBY_PIN = NULL;
*/
// Optical limit switch pins definition
/*
const int H_LIMIT_PIN = NULL;
const int V_LIMIT_PIN = NULL;
const int LIMIT_DELAY = 100;
*/
void setup() {
  Serial.begin(9600);

  // Servo pin setup
  
  volumeServo.attach(SERVO_PIN);
  //volumeServo.write(VOLUME_CONSTANT_SIGNAL); // found writing 88 should be no motion

  // Horizontal motor pin setup
  pinMode(H_MOTOR_STEP_PIN, OUTPUT); 
  pinMode(H_MOTOR_DIR_PIN, OUTPUT);
  digitalWrite(H_MOTOR_STEP_PIN,LOW);
  digitalWrite(H_MOTOR_DIR_PIN,LOW);

  // Vertical motor pin setup
  pinMode(V_MOTOR_STEP_PIN, OUTPUT); 
  pinMode(V_MOTOR_DIR_PIN, OUTPUT);
  digitalWrite(V_MOTOR_STEP_PIN,LOW);
  digitalWrite(V_MOTOR_DIR_PIN,LOW);
  
  // Linear actuator pin setup
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  /*
  // calibrate horizontal movement
  if (!calibrateHorizontal(87)) Serial.println("Hopefully this bad boy didn't break");
  else h_calibrated = true;

  if (!calibrateVertical(20)) Serial.println("Pipette case might be broken");
  else v_calibrated = true;
  */
}

// 1 rotation is 8mm 
// distance is in mm
//speed is represents the delay 
// dir, true for cw (forward), false for ccw (backward)
long moveHorizontal(bool dir, int speed, int distance, long currStep){
  long steps = 0;
  long NewCurrStep = 0;
  int H_direction;
  Serial.println("First Entry");
  Serial.println(NewCurrStep);
  Serial.println("distance");
  Serial.println(distance);
  steps = distance * long(3200); // right now distance is how many rotations will need to fix.
  Serial.println("Steps");
  Serial.println(steps);
  bool move = false;
  if(dir) {
    H_direction = HIGH;
    //digitalWrite(HorizontalDirPin,HIGH); //clockwise (forward movement)
    Serial.println("Forward Entry");
    Serial.println(NewCurrStep);
    if((steps + currStep) <= 275200) {
      move = true;
      Serial.println("Steps");
      Serial.println(steps);
      Serial.println(NewCurrStep);
      NewCurrStep = currStep + steps;
      Serial.println("Forward Second Entry");
      Serial.println(NewCurrStep);
       
    }
  }
  else {
    H_direction = LOW;
    Serial.println("Backward Entry");
    Serial.println(NewCurrStep);
    //digitalWrite(HorizontalDirPin,LOW); //Counter-clockwise (backward movement)
    if((currStep - steps) >= 0) {
      move = true;
      NewCurrStep = currStep - steps;
      Serial.println("Backward Second Entry");
      Serial.println(NewCurrStep);
    }
  }

  if(move) {
    Serial.println("MOVE Entry");
    for(long x = 0; x < steps; x++) { //amount of steps you want to move 
      digitalWrite(H_MOTOR_DIR_PIN, H_direction);
      digitalWrite(H_MOTOR_STEP_PIN, HIGH); 
      delayMicroseconds(100); 
      digitalWrite(H_MOTOR_STEP_PIN, LOW); 
      delayMicroseconds(1); 
    }
  }
  return NewCurrStep;
}

// 1 rotation is mm 
// distance is in mm
//speed is represents the delay 
// dir, true for DOWN, false for UP
int moveVertical(bool dir, int speed, int rotations, int currVertRotation){
  int NewCurrRotation = 0;
  bool move = false;
  int V_direction;
  if(dir) { //GO DOWN 
    V_direction = HIGH;
    if((rotations + currVertRotation) <= 18) {
      move = true;
      NewCurrRotation = currVertRotation + rotations;
    }
  }
  else { // GO UP
    V_direction = LOW;
    if((currVertRotation - rotations) >= 0) {
      move = true;
      NewCurrRotation = currVertRotation - rotations;
    }
  }
  if(move){
    for(int i = 0; i < rotations; i++){
      digitalWrite(V_MOTOR_DIR_PIN,V_direction); // Enables the motor to move in a particular direction
      for(int x = 0; x < 3200; x++) {
          digitalWrite(V_MOTOR_STEP_PIN,HIGH); 
          delayMicroseconds(100); 
          digitalWrite(V_MOTOR_STEP_PIN,LOW); 
          delayMicroseconds(1); 
      }
      delay(1);
    }
  }
  return NewCurrRotation;
}

/*
bool readHorizontalLimit() {
  return (analogRead(H_LIMIT_PIN) * 3.3/1023) == 1;
}

bool readVerticalLimit() {
  return (analogRead(V_LIMIT_PIN) * 3.3/1023) == 1;
}
*/
// Basic idea to set up horizontal movement, move until limit switch is hit
// timeout is max rotations before calling it for safety ig
  // could set timeout to 87, ideally limit switch should trigger before then
  /*
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
  /*
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
*/
// Changes volume to newVolume using servo
/*
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
*/
/*
void eject() {
  digitalWrite(AIN1_PIN, LOW);
  digitalWrite(PWMA_PIN, HIGH); // Enable motor A
  delay(1386);              // Run UP for 1.386 seconds

  // Stop motor
  digitalWrite(PWMA_PIN, LOW); // Turn off motor
  delay(1000);             // Wait 1 second

  // Set motor direction for DOWN
  digitalWrite(AIN1_PIN, LOW);
  digitalWrite(AIN2_PIN, HIGH);
  digitalWrite(PWMA_PIN, HIGH); // Enable motor A
  delay(1386);              // Run DOWN for 5 seconds
  
  // Stop motor
  digitalWrite(PWMA_PIN, LOW); // Turn off motor
  delay(1000);             // Wait 1 second
}
*/

void runServoMotor(){
  volumeServo.write(VOLUME_INCREASE_SIGNAL); //ccw
  delayMicroseconds(3000);
 volumeServo.write(VOLUME_CONSTANT_SIGNAL);
}

void intake_down(){
  Serial.println("Mosfet LOW: Moving DOWN");

  // Set motor direction for DOWN
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(PWMA, HIGH); // Enable motor A
  delay(1120);              // Run DOWN for 1.120 seconds
  return;
  
  // Stop motor
  digitalWrite(PWMA, LOW); // Turn off motor
  delay(1000);             // Wait 1 second
}

void intake_up(){
  Serial.println("Mosfet HIGH: Moving UP");

  //delay(10000);

  // Set motor direction for UP
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(PWMA, HIGH); // Enable motor A
  delay(1120);              // Run UP for 1.120 seconds

  // Stop motor
  digitalWrite(PWMA, LOW); // Turn off motor
  delay(1000);             // Wait 1 second
}

void loop() {
  delay(4000);
  long currHoriStep = 0;
  int currVertRotation = 0;
  //move forward to trough
  //currHoriStep = moveHorizontal(true, 120, 20, currHoriStep);
  currHoriStep = moveHorizontal(true, 120, 75, currHoriStep);
  delay(100);

  ///////CHANGE THE SERVO////////////
  runServoMotor();
  delay(100);
  
  //move down
  currVertRotation = moveVertical(true, 120, 8, currVertRotation);
  delay(1000);

  ///////INTAKE BUTTON ACTUATOR//////////
  intake_down();
  intake_up();

  //move up
  currVertRotation = moveVertical(false, 120, 8, currVertRotation);
  delay(100);
  //move half way back to start (ie going to a well plate)
  //currHoriStep = moveHorizontal(false, 120, 10, currHoriStep);
  currHoriStep = moveHorizontal(false, 120, 37, currHoriStep);
  delay(100);
  //move down
  currVertRotation = moveVertical(true, 120, 8, currVertRotation);
  delay(1000);

  ///////INTAKE BUTTON ACTUATOR//////////
  intake_down();
  intake_up();

  //move up
  currVertRotation = moveVertical(false, 120, 8, currVertRotation);
  delay(100);

  //move back to starting position (pos:0)
  //currHoriStep = moveHorizontal(false, 120, 10, currHoriStep);
  currHoriStep = moveHorizontal(false, 120, 38, currHoriStep);
  delay(100);
  /*
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
*/
  while(true){
    delay(100000);
  }
  return;
}
