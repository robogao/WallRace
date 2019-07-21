/* Pololu Zumo + Protractor
This code is for the Pololu Zumo + Protractor Sensor to compete in the Atlanta Hobby Robot Club's "Wallrace" competition

Future Improvements:
 1. Use Zumo's Gyro and/or Compass during turns.
 2. Use Zumo's Accelerometer to detect a box hit.
 3. Use PID for wall following, currently its just "P"
 4. Play a catchy tune throughout the race. Too bad "I am the Walrus" is not catchy when played on a simple buzzer. Consider "Chariots of Fire" theme instead.

A series of actions is executed in sequence.
   STATES:
   0     = stop
   1     = follow wall
   2     = turn corner
   3     = follow wall
   4     = hit box
   5     = reverse
   6     = u-turn
   
   STATE TRANSITION EVENTS:
   0->1  = button pressed and released
   1->2  = arrived at corner: the wall is seen in front of the robot
   2->3  = finished turning corner: the wall is no longer in front of the robot
   3->4  = arrived at box: the wall is seen in front of the robot
   4->5  = not getting any closer to the box: intensity of reflected IR is not increasing
   5->6  = timeout: an amount of time has elapsed
   6->1  = finished u-turn: the wall is seen on the opposite side and nothing is seen in front of the robot
   X->0  = button is pressed: button can be pressed at any time and the robot will go to State 0 and stop
*/

// ###########  Zumo 75:1 HP Motors  ############
// The following values were tested and work well with Pololu 75:1 gear ratio HP Micro-Metal Gear Motors on the Zumo Shield V1.2 kit.
// FOLLOW WALL
  int followWallSpeed = 350;
  // Avoid premature turns. Don't turn the corner until we've been wall following for at least this much time.
  unsigned int followWallMinDuration = 1500; // Increase duration if speed is reduced
  unsigned int followWallMaxDuration = 2800; // Increase duration if speed is reduced
// CORNER TURN
  int cornerTurnSpeed = 400;
  // Begin 90d corner turn if the protractor sees the wall in this range
  int cornerTurnBeginMinAngle = 45; // Decrease if Zumo wall-follows around the corner. Increase if Zumo mistakes the wall for a corner.
  int cornerTurnBeginMaxAngle = 135;
  // Corner Turn is finished if protractor does NOT see anything straight ahead (40-90 deg) AND protractor sees Wall on the correct side (0-40 deg) 
  int cornerTurnEndMinWallAngle = 0;
  int cornerTurnEndMaxWallAngle = 40; // Decrease if Zumo turns too much. Increase if Zumo turns too little.
  int cornerTurnEndMinOpenAngle = 40; // Decrease if Zumo turns too much. Increase if Zumo turns too little.
  int cornerTurnEndMaxOpenAngle = 90;
  // Limit duration of corner turn
  unsigned int cornerTurnMinDuration = 200; // Increase duration if speed is reduced
  unsigned int cornerTurnMaxDuration = 800; // Increase duration if speed is reduced
// HIT BOX
  int hitBoxSpeed = 350; // Speed at which Zumo drives towards the box
  // Stop Wall Following and Begin to Hit Box if the protractor sees an object in this range
  int hitBoxBeginMinAngle = 40; // Decrease if Zumo wall-follows around the box. Increase if Zumo mistakes the wall for a corner.
  int hitBoxBeginMaxAngle = 135;
  // Zumo will turn towards the box. If no object is seen within this angle range, just drive straight ahead. Prevents noise or other robots from distracting Zumo.
  int hitBoxMinAngle = 60; // Increase if Zumo hits the wall or the corner instead of hitting the box head-on. 
  int hitBoxMaxAngle = 135; // Decrease if Zumo heads toward the competing robot instead of heading for the box.
  // Limit Duration of driving towards the box
  unsigned int hitBoxMinDuration = 1000; // Increase duration if speed is reduced
  unsigned int hitBoxMaxDuration = 3000; // Increase duration if speed is reduced
// REVERSE after hitting box
  int reverseSpeed = 350;
  unsigned int reverseDuration = 200; // Increase duration if speed is reduced
// U TURN
  int uTurnSpeed = 400;
  // Limit Duration
  unsigned int uTurnMinDuration = 150; // Increase duration if speed is reduced
  unsigned int uTurnMaxDuration = 800; // Increase duration if speed is reduced
  // U Turn is finished if protractor does NOT see anything straight ahead (45-120 deg) AND protractor sees Wall on the opposite side (120-180 deg) 
  int uTurnEndMinOpenAngle = 45;
  int uTurnEndMaxOpenAngle = 120; // Decrease if Zumo turns too much. Increase if Zumo turns too little.
  int uTurnEndMinWallAngle = 120; // Decrease if Zumo turns too much. Increase if Zumo turns too little.
  int uTurnEndMaxWallAngle = 180;
// ################  END Zumo 75:1 HP Motor Parameters  #############

//// ###########  Zumo 30:1 HP Motors  ############
//// The following values were tested and work well with Pololu 30:1 gear ratio HPCB Micro-Metal Gear Motors on the Zumo Shield V1.2 kit.
//// FOLLOW WALL
//  int followWallSpeed = 300;
//  // Avoid premature turns. Don't turn the corner until we've been wall following for at least this much time.
//  unsigned int followWallMinDuration = 900;
//  unsigned int followWallMaxDuration = 2800;
//// CORNER TURN
//  int cornerTurnSpeed = 350;
//  // Begin 90d corner turn if the protractor sees the wall in this range
//  int cornerTurnBeginMinAngle = 45;
//  int cornerTurnBeginMaxAngle = 135;
//  // Corner Turn is finished if protractor does NOT see anything straight ahead (40-90 deg) AND protractor sees Wall on the correct side (0-40 deg) 
//  int cornerTurnEndMinWallAngle = 0;
//  int cornerTurnEndMaxWallAngle = 45;
//  int cornerTurnEndMinOpenAngle = 45;
//  int cornerTurnEndMaxOpenAngle = 90;
//  // Limit duration of corner turn
//  unsigned int cornerTurnMinDuration = 150;
//  unsigned int cornerTurnMaxDuration = 350;
//// HIT BOX
//  int hitBoxSpeed = 300; // Speed at which Zumo drives towards the box
//  // Stop Wall Following and Begin to Hit Box if the protractor sees an object in this range
//  int hitBoxBeginMinAngle = 40;
//  int hitBoxBeginMaxAngle = 135;
//  // Zumo will turn towards the box. If no object is seen within this angle range, just drive straight ahead. Prevents noise or other robots from distracting Zumo.
//  int hitBoxMinAngle = 50;
//  int hitBoxMaxAngle = 135;
//  // Limit Duration of driving towards the box
//  unsigned int hitBoxMinDuration = 1000;
//  unsigned int hitBoxMaxDuration = 2500;
//// REVERSE after hitting box
//  int reverseSpeed = 250;
//  unsigned int reverseDuration = 200;
//// U TURN
//  int uTurnSpeed = 350;
//  // Limit Duration
//  unsigned int uTurnMinDuration = 150;
//  unsigned int uTurnMaxDuration = 350;
//  // U Turn is finished if protractor does NOT see anything straight ahead (45-120 deg) AND protractor sees Wall on the opposite side (120-180 deg) 
//  int uTurnEndMinOpenAngle = 45;
//  int uTurnEndMaxOpenAngle = 120; 
//  int uTurnEndMinWallAngle = 120;
//  int uTurnEndMaxWallAngle = 180;
//// ################  END Zumo 30:1 HP Motor Parameters  #############

#include <ZumoBuzzer.h>
#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <avr/pgmspace.h>
#include <Wire.h>
#include <Protractor.h>

Protractor protractor;
Pushbutton button(ZUMO_BUTTON); // pushbutton on pin 12
ZumoMotors motors;
ZumoBuzzer buzzer;
const char sound_effect[] PROGMEM = "O4 T100 V15 L4 MS g12>c12>e12>G6>E12 ML>G2"; // "charge" melody

#define LED 13
#define RIGHT 1
#define LEFT -1
int volume = 15; // 10 to 15, 0 is off.
int wall_side = 0; // -1 = Left, 1 = Right, 0 = No Wall
int init_wall_side = 0;
int init_angle = 0;
int init_vis = 0;
int state = 0;
unsigned long stateStartTime = millis();

void setup(){  
  // Initiate the Wire library and join the I2C bus as a master
  Wire.begin();
  Serial.begin(9600);
  while(!Serial);

  pinMode(LED, HIGH);
  buzzer.playMode(PLAY_AUTOMATIC);  
  
  // Initiate Protractor
  protractor.begin(Wire,69);
  delay(10); // Give the Protractor a little time to boot up and self-test
  if(protractor.read(0)) {
    Serial.println(F("Protractor Connected"));
    buzzer.playNote(NOTE_C(5), 60, volume); delay(80); // Play Happy Tone
    buzzer.playNote(NOTE_D(5), 60, volume); delay(80);
    buzzer.playNote(NOTE_E(5), 60, volume); delay(80);
  }else{
    Serial.println(F("Protractor not Connected"));
    buzzer.playNote(NOTE_E(4), 60, volume); delay(80); // Play Sad Tone
    buzzer.playNote(NOTE_D(4), 60, volume); delay(80);
    buzzer.playNote(NOTE_C(4), 60, volume); delay(80);
  }

  // uncomment if necessary to correct motor directions
  //motors.flipLeftMotor(true);
  //motors.flipRightMotor(true);
}

void loop(){
  if (button.isPressed()) { // Always check if we should stop
    StopAndReset();
    button.waitForRelease();
  }
  
  // CHECK FOR TRANSITIONS AND SET NEW STATES
  switch(state){
    case 0: // Button is Released
      Serial.println(F("0: Wait for Button to be pressed and released"));
      if(ButtonIsReleased()){ // ButtonIsReleased() is blocking, it will wait forever until the button is pressed and released.
        SetState(1);
      }
      break;
    case 1: // Angle to Wall > 45 deg
      Serial.println(F("1: Check if Corner Turn should begin"));
      if( (TimeElapsed(followWallMinDuration) && ObjectIsInAngleRange(cornerTurnBeginMinAngle,cornerTurnBeginMaxAngle)) || TimeElapsed(followWallMaxDuration)){
        SetState(2);
      }
      break;
    case 2: // Angle to Wall < 30 deg
      Serial.println(F("2: Check if Corner Turn is finished"));
      if( (TimeElapsed(cornerTurnMinDuration) && ObjectIsInAngleRange(cornerTurnEndMinWallAngle,cornerTurnEndMaxWallAngle) && NoObjectIsInAngleRange(cornerTurnEndMinOpenAngle,cornerTurnEndMaxOpenAngle)) || TimeElapsed(cornerTurnMaxDuration)){
        SetState(3);
      }
      break;
    case 3: // Angle to Wall > 45 deg
      Serial.println(F("3: Check if Box is visible"));
      if( (TimeElapsed(followWallMinDuration) && ObjectIsInAngleRange(hitBoxBeginMinAngle,hitBoxBeginMaxAngle)) || TimeElapsed(followWallMaxDuration)){
        SetState(4);
      }
      break;
    case 4: // Visibility not Changing
      Serial.println(F("4: Check if Box is not getting any closer"));
      if( (TimeElapsed(hitBoxMinDuration) && VisibilityNotIncreasing()) || TimeElapsed(hitBoxMaxDuration)){
        SetState(5);
      }
      break;
    case 5: // 1/4 second elapsed (drive reverse)
      Serial.println(F("5: Check if Reverse is finished"));
      if(TimeElapsed(reverseDuration)){
        SetState(6);
      }
      break;
    case 6: // Angle to Wall > 135 deg, reverse wall_side
      Serial.println(F("6: Check if U Turn is finished"));
      if( (TimeElapsed(uTurnMinDuration) && ObjectIsInAngleRange(uTurnEndMinWallAngle,uTurnEndMaxWallAngle) && NoObjectIsInAngleRange(uTurnEndMinOpenAngle,uTurnEndMaxOpenAngle)) || TimeElapsed(uTurnMaxDuration)){
        SetState(1);
        wall_side *= -1; // reverse the wall side
      }
      break;
  }

  // RUN STATES
  switch(state){
    case 0: // Wait for Button Press
      Serial.println(F("0: Stop and Reset"));
      StopAndReset(); // do nothing
      break;
    case 1: // Follow Wall
      Serial.println(F("1: Follow Wall"));
      FollowWall(followWallSpeed);
      break;
    case 2: // Pivot Turn away from Wall
      Serial.println(F("2: Turn Corner"));
      PivotFromWall(cornerTurnSpeed);
      break;
    case 3: // Follow Wall
      Serial.println(F("1: Follow Wall"));
      FollowWall(followWallSpeed);
      break;
    case 4: // Drive Towards Object
      Serial.println(F("4: Hit the Box"));
      DriveTowardsObject(hitBoxSpeed,hitBoxMinAngle,hitBoxMaxAngle);
      break;
    case 5: // Reverse, Start Time
      Serial.println(F("5: Reverse"));
      ReverseFromObject(reverseSpeed);
      break;
    case 6: // Turn towards Wall Side
      Serial.println(F("6: U Turn"));
      TurnTowardsWall(uTurnSpeed);
      break;
  }
}

bool SetStartingPosition(){
  Serial.println(F("Set Starting Position"));
  protractor.read();
  if(protractor.objectCount() > 0){
    init_angle = protractor.objectAngle();
    init_vis = protractor.objectVisibility();
    Serial.print(F(" Initial Angle = ")); Serial.print(init_angle); Serial.print(F(" Initial Vis = ")); Serial.print(init_vis);
    if(init_angle <= 60){
      init_wall_side = LEFT;
      wall_side = init_wall_side;
      Serial.println(F(" Wall Side Left"));
      return true;
    }else if(init_angle >= 120){
      init_wall_side = RIGHT;
      wall_side = init_wall_side;
      Serial.println(F(" Wall Side Right"));
      return true;
    }else{
      Serial.println(F(" ERROR: WALL VISIBLE IN FRONT ON STARTUP"));
      return false;
    }
  }else{
    Serial.println(F(" ERROR: NO WALL VISIBLE ON STARTUP"));
    return false;
  }
}

void SetState(int newState){
  state = newState;
  stateStartTime = millis();
  buzzer.playNote(NOTE_G(4), 50, volume);
  Serial.println(F(" New State Confirmed!"));
}

// ##### PRIMITIVES #####
void StopAndReset(){
  motors.setSpeeds(0, 0);
  state = 0;
  wall_side = 0;
  init_angle = 0;
  init_vis = 0;
}

void FollowWall(int speed){
  // Drives forwards, turning left or right as needed so that the visibility of the wall is maintained.
  protractor.read(); 
  int error = 256*(init_vis - protractor.objectVisibility(0))/init_vis;
  int correction = (long)wall_side*speed*error/256;
  int leftMotorSpeed = speed + correction;
  int rightMotorSpeed = speed - correction;
  motors.setSpeeds(leftMotorSpeed,rightMotorSpeed);
//  Serial.print(F("    Error = ")); Serial.print(error); Serial.print(F(" Wall Side = ")); Serial.print(wall_side); Serial.print(F(" Correction = ")); Serial.print(correction);
//  Serial.print(F(" Left = ")); Serial.print(leftMotorSpeed); Serial.print(F(" Right = ")); Serial.println(rightMotorSpeed);
  delay(15);
}

void PivotFromWall(int speed){
  // Outside wheel keeps moving at speed, inside wheel stops
  int leftMotorSpeed  = (wall_side == LEFT)  ? speed : 0;
  int rightMotorSpeed = (wall_side == RIGHT) ? speed : 0;
  motors.setSpeeds(leftMotorSpeed,rightMotorSpeed);
}

void DriveTowardsObject(int speed, int minAngle, int maxAngle){
  // Drives towards object whose PrAngle is closest to 90 (straight ahead)
  // Figure out which object is closest to 90 and how far off it is
  int error = 90; // 90 is far right, -90 is far left, 0 is directly ahead
//  protractor.read();
  if(protractor.objectCount() == 0 || NoObjectIsInAngleRange(minAngle,maxAngle)){
    error = -wall_side*10; // No object seen in range, drive slightly towards the wall.  -1 = Left.  -90 is far left
    digitalWrite(LED,HIGH);
  }else{
    for(int i = 0; i < protractor.objectCount(); i++){
      if( abs(90 - protractor.objectAngle(i)) < abs(error) ) error = 90 - protractor.objectAngle(i);
    }
    digitalWrite(LED,LOW);
  }
  error = 256*error/90; // map error from (-90 to 90) to (-256 to 256)
  int leftMotorSpeed  = speed - (long)speed*error/256;
  int rightMotorSpeed = speed + (long)speed*error/256;
  motors.setSpeeds(leftMotorSpeed,rightMotorSpeed);
  delay(15);
}

void ReverseFromObject(int speed){
  // drives in reverse away from object whose PrAngle is closest to 90
  // Figure out which object is closest to 90 and how far off it is
  int error = 90; // 90 is far right, -90 is far left, 0 is directly ahead
  protractor.read();
  if(protractor.objectCount() == 0) error = 0; // No object seen, so just drive straight ahead
  else{
    for(int i = 0; i < protractor.objectCount(); i++){
      if( abs(90 - protractor.objectAngle(i)) < abs(error) ) error = 90 - protractor.objectAngle(i);
    }
  }
  speed = abs(speed); // sanitize input
  error = 256*error/90; // map error from (-90 to 90) to (-256 to 256)
  int leftMotorSpeed  = -speed - (long)speed*error/256;
  int rightMotorSpeed = -speed + (long)speed*error/256; // positive error means object is on the right side, so right wheel needs to reverse faster
  motors.setSpeeds(leftMotorSpeed,rightMotorSpeed);
  delay(15);
}

void TurnTowardsWall(int speed){
  // turns in direction of wall_side, wheel speed is +/- speed/2
  int leftMotorSpeed  = (wall_side == LEFT)  ? -speed : speed;
  int rightMotorSpeed = (wall_side == RIGHT) ? -speed : speed;
  motors.setSpeeds(leftMotorSpeed,rightMotorSpeed);
}

// ##### TRANSITIONS #####
bool TimeElapsed(unsigned long ms){
  if(millis() - stateStartTime >= ms)
    return true;
  else
    return false;
}

bool ButtonIsReleased(){
  digitalWrite(LED, HIGH); // LED on means Zumo is ready and waiting to start
  button.waitForButton();
  digitalWrite(LED, LOW); // LED off means Zumo is ready to go
  
  // play audible countdown
  for (int i = 0; i < 3; i++)
  {
    delay(1000);
    buzzer.playNote(NOTE_G(3), 50, 12);
  }
  
  if(SetStartingPosition()){
    delay(1000);
    buzzer.playFromProgramSpace(sound_effect);
    delay(1000);
    
    motors.setSpeeds(50,50);
    return true;
  }
  else{
    buzzer.playNote(NOTE_E(4), 60, volume); delay(80);
    buzzer.playNote(NOTE_D(4), 60, volume); delay(80);
    buzzer.playNote(NOTE_C(4), 60, volume); delay(80);
    return false;
  }
}

bool ObjectIsInAngleRange(int minAngle, int maxAngle){
  // Return TRUE if any object exists at an angle minAngle <= x <= maxAngle
  // Input angles are relative to the wall_side. minAngle is closest to the wall and maxAngle is further from the wall
  // Example: ObjectIsInAngleRange(0,90) will return true if there is any object on the wall_side of the robot
  protractor.read();
  bool count = 0; // Counts the number of objects within the range
  for(int i = 0; i < min(protractor.objectCount(),2); i++){
    int wallAngle = (wall_side == RIGHT) ? 180 - protractor.objectAngle(i) : protractor.objectAngle(i); // Flip angle if wall_side is right
    if(wallAngle >= minAngle && wallAngle <= maxAngle) count++;
  }
  if(count) return true;
  else      return false;
}

bool NoObjectIsInAngleRange(int minAngle, int maxAngle){
  // Return TRUE if no object exists at an angle minAngle < x < maxAngle
  // Input angles are relative to the wall_side. minAngle is closest to the wall and maxAngle is further from the wall
  // Example: ObjectIsInAngleRange(0,90) will return true if there is any object on the wall_side of the robot
  protractor.read();
  bool count = 0; // Counts the number of objects within the range
  for(int i = 0; i < protractor.objectCount(); i++){
    int wallAngle = (wall_side == RIGHT) ? 180 - protractor.objectAngle(i) : protractor.objectAngle(i); // Flip angle if wall_side is right
    if(wallAngle > minAngle && wallAngle < maxAngle) count++;
  }
  if(count == 0) return true;
  else           return false;
}

bool VisibilityNotIncreasing(){
  const int numReadings = 10; // how many historical readings to log
  static int visibilities[numReadings]; 
  static int higherVis = numReadings-1; // Count of how many times a more recent visibility was higher than the prior visibility, max value is numReadings-1
  static long lastTime = 0;
  static int lastState = 0;
  if(state != lastState || millis() - lastTime > 1000) // re-initialize readings with each state change or long delay between function calls
    for(int i = 0; i < numReadings; i++)
      visibilities[i] = numReadings-i; // initialize visibilities as {...,5,4,3,2,1} so it appears the object is becoming more visible with each reading
  
  if(millis() - lastTime > 20){ // Only run the check every 20ms so that numReadings doesn't need to be ridiculously large
    // read the protractor, determine index of object that is most directly in front
    protractor.read();
    int offCenter = 90; // 90 is far right, -90 is far left, 0 is directly ahead
    int objectInFront = 0; // store the index of the objectAngle which is closest to 90 (straight ahead)
    for(int i = 0; i < protractor.objectCount(); i++){ // loop through all visible objects and determine which is least offCenter
      if( abs(90 - protractor.objectAngle(i)) < abs(offCenter) ){
        offCenter = 90 - protractor.objectAngle(i);
        objectInFront = i;
      }
    }
    for(int i = numReadings; i > 0; i--) visibilities[i] = visibilities[i-1]; // shift history of visibilities
    visibilities[0] = protractor.objectVisibility(objectInFront); // record most recent visibility
    higherVis = 0;
    for(int i = 0; i < numReadings-1; i++) if(visibilities[i] > visibilities[i+1]) higherVis++; // count how often a more recent reading was more visible than the prior reading
    lastTime = millis();
    lastState = state;
  }
  if(higherVis <= numReadings*0.4) return true; // most of readings indicate visibility is not increasing each time
  else                           return false;
}


