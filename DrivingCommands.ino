/*
 * The following are functions used to direct the motion of the robot
 * while it is driving. This applies to 90 and 180 degree turns as well
 * as functions to adjust course during line following
 */
enum DriveCommands {tL, tR, gL, gR, sL, sR, fwd, rev, halt} lastDrive = fwd;
//-------------------------------------defined motions
//turn the robot to the right
void turnRight() {
  lastDrive = tR;
  if (driveFlag) {
    leftForward(255);
    rightReverse(255);
  }
}


//turns the robot 90 degrees to the left, and
//returns true when it's done. As this is called from path following
//state machines, their state changes as soon as it finishes, so the turns
//will properly terminate
boolean turnLeft90(int dir) {
  if (abs(lastEncTicks) < abs(rightEncoder.read()) - inchesToTicks(5)) {
    if (!digitalRead(leftLine)) { //if the leftLine sensor sees white
      turnLeft();
      return false;
    } else { //when the left line sensor sees a line
      stopDrive();
      direct = (Compass) changeDirectionLeft(dir); //update the direction variable
      return true;
    }
  }
  else {
    sharpLeft();  //drive the robot forward and slightly to the left
    return false;
  }
}
//turns the robot 90 degrees to the right, and
//returns true when it's done. As this is called from path following
//state machines, their state changes as soon as it finishes, so the turns
//will properly terminate
boolean turnRight90(int dir) {
  if (abs(lastEncTicks) < abs(leftEncoder.read()) - inchesToTicks(5)) {
    if (!digitalRead(rightLine)) { //if the right Line sensor sees white
      turnRight();
      return false;
    } else { //when the right line sensor sees a line
      stopDrive();
      direct = (Compass) changeDirectionRight(dir); //update the direction variable
      return true;
    }
  }
  else
  {
    sharpRight();  //drive the robot forward and slightly to the right
    return false;
  }
}


//turn the robot 90 degrees to its left and re-center it on the line
//perpendicular to the one it previously followed
void turnLeft() {
  lastDrive = tL;
  if (driveFlag) {
    leftReverse(255);
    rightForward(255);
  }
}


//turn the robot 180 degrees (to the left) and find the line again
void aboutFace(int inches) {

  if (abs(lastEncTicks) < abs(rightEncoder.read()) - inchesToTicks(inches)) {
    if (!digitalRead(rightLine)) {
      turnRight();
    } else {
      stopDrive();
      countInter = 0;        //reset countInter, this will allow finding the center line
      resetEncoders();
      toNextState();
    }
  }
  else {
    reverse(fullPower);
  }
}

//sets the motors to even power in the same direction
//to drive the robot forwards
void driveForward(int power) {
  lastDrive = fwd;
  if (driveFlag) {
    leftForward(power);
    rightForward(power);
  }
}

//sets the motor to an even power, backwards
void reverse(int power) {
  lastDrive = rev;
  if (driveFlag) {
    leftReverse(power);
    rightReverse(power);
  }
}

//sets both motors to their stop value
void stopDrive() {
  lastDrive = halt;
  leftStop();
  rightStop();
}

//-------------------------------------Line following functions
//writes the motors depending on the given state
//to be placed inside a loop
//may need to be changed to a polling implementation due to limited external interrupt pins
//either that or add an uno to cover some of the interrupt functions
/*
 *   The states are determined by reading the right and left sensor
 *   then scaling the readings to create discrete state values for each case:
 *   Right | Left | Right Scale | Left Scale| State Value | State Name
 *   0       0        0               0          0             forward
 *   0       1        0               4          4             go Left
 *   1       0        1               0          1             go Right
 *   1       1        1               4          5             at Intersect
 *
 *   the commented out sections of the ISRS use the enum State to determine the
 *   state of the robot. It was replaced by the easier to read integerState implementation
 */
void followLine() {
  integerState = digitalRead(rightLine) + digitalRead(leftLine) * 4;
  switch (integerState) {
    case 5:  doLastDrive();  break;        //go forwards at the given power
    case 4:  guideLeft();        break;        //turn the robot slightly to the left
    case 1:  guideRight();       break;        //turn the robot slightly to the right
    case 0:  driveForward(255);  break;        //robot is at an intersection, either stop or count it?
    default:    break;
  }
}

//calls whichever drive command was last called
void doLastDrive() {
  switch (lastDrive) {
    case tL: guideLeft(); break;
    case tR: guideRight(); break;
    case gL: guideLeft(); break;
    case gR: guideRight(); break;
    case sL: guideLeft(); break;
    case sR: guideRight(); break;
    case fwd: driveForward(fullPower); break;
    case rev: reverse(fullPower); break;
    case halt: stopDrive(); break;
    default: stopDrive();
  }
}

//sets the right motor to a higher power than the
//left motor, to cause the robot to turn to the left
void guideLeft() {
  lastDrive = gL;
  if (driveFlag) {
    leftForward(50);
    rightForward(255);
  }
}

//sets the left motor to a higher power than the
//right, to cause the robot to turn towards the right
void guideRight() {
  lastDrive = gR;
  if (driveFlag) {
    leftForward(255);
    rightForward(50);
  }
}

//sets the left soft reverse, right hard forward for a sharp left turn
void sharpLeft() {
  lastDrive = sL;
  if (driveFlag) {
    leftReverse(150);
    rightForward(255);
  }
}

//sets the right soft reverse, left hard forward for a sharp right turn
void sharpRight() {
  lastDrive = sR;
  if (driveFlag) {
    leftForward(255);
    rightReverse(150);
  }
}
//-------------------------------------basic driving functions
//writes the left motor to forwards at a given power
void leftForward(int power) {
  analogWrite(leftMotorPin1, constrain(power, 0, 255));
  analogWrite(leftMotorPin2, 0);
}

//writes the left motor to reverse at a given power
void leftReverse(int power) {
  analogWrite(leftMotorPin1, 0);
  analogWrite(leftMotorPin2, constrain(power, 0, 255));
}

//stops the left motor
void leftStop() {
  analogWrite(leftMotorPin1, 0);
  analogWrite(leftMotorPin2, 0);
}

//stops right motor
void rightStop() {
  analogWrite(rightMotorPin1, 0);
  analogWrite(rightMotorPin2, 0);
}

//writes the right motor to forwards at a given power
void rightForward(int power) {
  analogWrite(rightMotorPin1, constrain(power, 0, 255));
  analogWrite(rightMotorPin2, 0);
}

//writes the right motor to reverse at a given power
void rightReverse(int power) {
  analogWrite(rightMotorPin1, 0);
  analogWrite(rightMotorPin2, constrain(power, 0, 255));
}





