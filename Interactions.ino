/*
 * These functions control the robots interaction with objects on the field
 * write these functions without while loops, use if statements and rely on loop to cycle
 * through fast enough to be effective
 */





//uses the encoder value from the grip motor to determine
//whether or not the rod has been entirely absorbed by the gripper
boolean fullyInOut() {
  //return abs(gripEncoder.read()) > rodLengthTicks;
  return sendHB;
}

//sets grip to intake mode
void removeRod() {
  analogWrite(gripMotorPin2, 255);
  analogWrite(gripMotorPin1, 0);

}

//sets grip to output mode
void dispenseRod() {
  analogWrite(gripMotorPin2, 0);
  analogWrite(gripMotorPin1, 255);
}

//stops motor on the grip
void stopGrip() {
  analogWrite(gripMotorPin2, 0);
  analogWrite(gripMotorPin1, 0);
}

//stops the motion of the arm
void stopArm(){
  analogWrite(armMotorPin1, 0);
  analogWrite(armMotorPin2, 0);
}

//this is used from the pickup stages of both ends. It will change
//the stage of the pickup state to extracting the rod, and stop the drive motors
//after the limit switch is triggered
void approachReactor() {
  if (digitalRead(frontLimit)) {
    followLine();
  } else {
    stopDrive();
    stagePU = pickUpRod_s;
  }
}

//inches to ticks converts the given number of inches to 
//the number of encoder ticks the wheel encoders will read
int inchesToTicks(int inch){
  return inch * 1231; //the wheel encoder read 1231 ticks for every inch traveled (roughly 1/8 rotation)
}


