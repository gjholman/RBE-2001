/*
 * functions specific to interactions with the reactor tubes
 */





//add description
void removeFromReactor () {
  switch (intakeState) {
    case lowerArm:                          //state the puts the arm in the lowered position
      if (isSettled(armDownValue)) {                 //when the arm is in the right position, the state changes
        hbCount = 0;                //marks the time
        intakeState = intakeRod;
        stopArm();
        clearPIDHistory();                 //clear past values from PID history
      }
      else
      {
        if (analogRead(armPot) < armDownValue + 50) {       //starts spinning the rod intake wheels on the descent
          //removeRod();
        }
        controlArmPID(armDownValue);
      }
      break;
    case intakeRod:
      if (hbCount > 2) { //stops the intake after intake time has passed
        stopGrip();
        lowLevel = true;
        writeStrLCD(" Low Radiation! ",1,0);         //print a low level alert
        intakeState = raiseArm;             //changes state if it is done intaking
      }
      else {
        removeRod();                        //otherwise it keeps intaking
      }
      break;
    case raiseArm:
      controlArmPID(armUpValue);                    //sets the arm to its raised position
      if(isSettled(armUpValue)){
        stagePU = turnAroundP_s;
        stopArm();
        resetEncoders();
      }
      break;
    default:
      stopGrip();                            //if something goes wrong, stop the intake
  }
}

//TODO add desctiption
void replaceRod() {
  switch (intakeState) {
    case lowerArm:                          //state the puts the arm in the lowered position
      if (isSettled(armDownValue)) {                 //when the arm is in the right position, the state changes
        hbCount = 0;                //marks the time
        intakeState = intakeRod;
        stopArm();
        clearPIDHistory();                 //clear past values from PID history
      }
      else
      {
        controlArmPID(armDownValue);
      }
      break;
    case intakeRod:
      if (hbCount > 2) { //stops the intake after intake time has passed
        stopGrip();
        highLevel = false;
        clearTopLine();
        intakeState = raiseArm;             //changes state if it is done intaking
      }
      else {
        dispenseRod();                        //otherwise it keeps intaking
      }
      break;
    case raiseArm:
      if (analogRead(armPot) < armDownValue + 50) {       //starts spinning the rod intake wheels on the descent
        dispenseRod();
      }
      else
      {
        stopGrip();
      }
      controlArmPID(armUpValue);                    //sets the arm to its raised position
      if (isSettled(armUpValue)) {                 //when the arm is in the right position, the state changes
        hbCount = 0;                //marks the time
        intakeState = intakeRod;
        stopArm();
        resetEncoders();
        clearPIDHistory();                 //clear past values from PID history
        stageDN = turnAroundN_s;
      }
      break;
    default:
      stopGrip();                            //if something goes wrong, stop the intake
  }
}
