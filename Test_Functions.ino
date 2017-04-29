/*
 * These are for testing various modular aspects of the code
 * call them from loop one at a time to test.
 *
 * Functions to test:
 *  - pickup
 * DONE    - approachReactor
 * DONE    - removeFromReactor
 * DONE    - aboutFace *********************************important to test carefully for each state
 * DONE - depositSpent
 * DONE    - pickStorage
 * DONE    - goToStorage
 * DONE       - goToIntersect (from north and south)
 * DONE       - turnToStorage (same)
 * DONE          - turnRight90 -- revisit later
 * DONE          - turnLeft90 -- revisit later
 * DONE        - approachS *******************************important to test carefully for each state
 * DONE    - insert
 * DONE    - relaseRod
 * done? - getNew
 * DONE    - pickSupply
 * DONE    - goToSupply
 * DONE      - returnToCenter
 * DONE       - pathTravel
 * DONE          - turnDestination**************************will take 16 tests
 * DONE          - turnRight90 -- revisit later
 * DONE          - turnLeft90 -- revisit later
 * DONE          - goToDestIntersection*********************will take 16 tests
 * DONE          - approachS
 * DONE    - extractRod
 *   - depositNew
 * DONE     - turnHome
 * DONE     - driveToReactor - try using if(approachS()){ nextState()}
 * done?        - replaceRod
 *   -switchField
 *      - resetAll
 */

void testRodReplace() {
  runRodReplace();
  writeIntLCD(activeStorage, 0, 0);
  writeIntLCD(activeSupply, 2, 0);
  printDestination(4, 0);
  writeIntLCD(stageDS, 7, 0);
}

void testDepositSpent() {
  depositSpent();
  switch (stageDS) {
    case pickStorage_s: //while in the pickStorage case, do nothing
      break;
    case goTo_s:  //drive the robot to the active storage tube
      printGoToStorageInfo();
      break;
    case insert_s: //insert the rod into the storage tube, turn off low level radiation alert
      printStage(stageDS);
      writeStrLCD("TIn?: ", 7, 1);
      writeIntLCD(lastTime < millis() - intakeTime, 13, 1);
      writeStrLCD("FIn?: ", 0, 0);
      writeIntLCD(bitRead(storageData, activeStorage - 1), 7, 0);
      writeStrLCD("Cnt: ", 10, 0);
      writeIntLCD(countInter, 15, 0);
      break;
    case releaseRod_s: //ensure rod is released (extra turn or two of the gripper), if it isn't go back to insert mode
      printStage(stageDS);
      break;
    case turnAroundS_s: //turn robot around, progress to next state
      printStage(stageDS);
      writeStrLCD("Cnt: ", 10, 0);
      writeIntLCD(countInter, 15, 0);
      break;
  }
}
//tests that the robot can navigate from the reactor tube to an open storage tube (3)
void testGoToStorage() {
  printGoToStorageInfo();
  activeStorage = 3;
  destinationCount = 3;
  goToStorage();
}

//use this to test the PID control for the fourbar
void testPIDControl(int setValue) {
  Serial.print("Pot Value: ");
  Serial.print(analogRead(armPot));
  Serial.print(" Settled? ");
  Serial.print(isSettled(setValue));
  Serial.print(" Output Value: ");
  Serial.print(out);
  controlArmPID(setValue);
  writeStrLCD("Settled? ", 0, 0);
  writeIntLCD(isSettled(setValue), 10, 0);
  writeIntLCD(analogRead(armPot), 0, 1);
}

//use this to test the line follow for intersection counting as well as
void testLineFollow() {
  Serial.println(countInter);
  followLine();

}

//test the basic drive functions (turns, forward/reverse, guides)
void testDrive() {
  driveFlag = true;
  turnRight();
  delay(1000);
  stopDrive();
  turnLeft();
  delay(1000);
  stopDrive();
  delay(1000);
  driveForward(255);
  delay(1000);
  stopDrive();
  reverse(255);
  delay(1000);
  stopDrive();
}

//tests the intake and stop states of the grip motor
void testGrip() {
  removeRod();
  Serial.println("intake");
  delay(1000);
  dispenseRod();
  Serial.println("dispense");
  delay(1000);
}

//use this to test sending bluetooth messages
void testBluetooth() {
  if (sendHB) {
    sendHB = false;
    sendHBMessage();
  }
  //  delay(20);
  //  sendNewRodAlert();
}

//tests the follow line function, printing the current state (on/off the line)
void testFollowLine() {
  writeStrLCD("Following", 0, 0);
  followLine();
  printLineStates();
}

//tests that the robot correctly counts the
//number of intersections it sees
void testCountIntersections() {
  followLine();
  pollIntersections();
  writeIntLCD(countInter, 12, 0);
  printLineStates();
}

//prints encoder values to the serial monitor
void testEncoder() {
  removeRod();
  driveForward(fullPower);
  Serial.print(leftEncoder.read());
  Serial.print("   ");
  Serial.print(gripEncoder.read());
  Serial.print("   ");
  Serial.println(rightEncoder.read());
}

//test the larger level function remove from reactor
void testRemoveFromReactor() {
  removeFromReactor();
  writeStrLCD("Settled? ", 0, 0);
  writeIntLCD(isSettled(armDownValue), 10, 0);
  writeIntLCD(analogRead(armPot), 0, 1);
  writeIntLCD(out, 5, 1);
  writeIntLCD(hbCount, 9,1);
}

//test the larger level function remove from reactor
void testReplaceRod() {
  replaceRod();
  writeStrLCD("Settled? ", 0, 0);
  writeIntLCD(isSettled(armDownValue), 10, 0);
  writeIntLCD(analogRead(armPot), 0, 1);
  writeIntLCD(out, 5, 1);
}

//tests that the robot stops when it gets to the reactor and
//transitions to the pickup stage
void testApproachReactor() {
  approachReactor();
  writeStrLCD("Stage: ", 0, 1);
  writeIntLCD(stagePU, 7, 1);
}

//tests about face for each top level state
//by resetting the state each iteration, skipping the other operations of the state
//this function also checks that toNextState operates properly
void testAboutFace() {
  rightEncoder.write(0); //reset the value of the right encoder
  aboutFace(2); //turn around
  writeIntLCD(state, 0, 1); //print the integer value of the state to the lcd, should cycle 0-4
}

//tests the value written to activeStorage and destinationCount
//for both the north end and the south ends
void testPickStorage() {
  writeStrLCD("NorthVal: ", 0, 0);
  writeStrLCD("SouthVal: ", 0, 1);
  northEnd = true; //test the calculations for the north end next
  pickStorage();
  writeIntLCD(activeStorage, 11, 0);
  writeIntLCD(destinationCount, 13, 0);

  northEnd = false; //test the calculations for the south end
  pickStorage();
  writeIntLCD(activeStorage, 11, 1);
  writeIntLCD(destinationCount, 13, 1);
}

//tests the value written to activeSupply and destinationCount
//pickSupply does not depend on whether the robot is on the north or south end of the field
void testPickSupply() {
  pickStorage();
  pickSupply();
  writeStrLCD("Active: ", 0, 0);
  writeStrLCD("DestCount: ", 0, 1);
  writeIntLCD(activeStorage, 9, 0);
  writeIntLCD(activeSupply, 11, 0);
  if (destinationCount < 0) {
    writeIntLCD(destinationCount, 13, 1);
  }
  else {
    writeStrLCD(" ", 13, 1);
    writeIntLCD(destinationCount, 14, 1);
  }
}

//test the left turn of the robot
void testLeftTurning () {
  writeStrLCD("Left", 0, 0);
  writeStrLCD("Turn:", 5, 0);
  writeStrLCD("Dir:", 0, 1);
  writeIntLCD(direct, 5, 1);
  //lastEncTicks=leftEncoder.read();
  if (!turnLeft90(direct)) {
    writeStrLCD("Yes", 11, 0);
  }
  else {
    writeStrLCD("No", 11, 0);
  }
  Serial.print("Last Enc: ");
  Serial.print(lastEncTicks);
  Serial.print("   Current Enc: ");
  Serial.print(rightEncoder.read());
  Serial.print("   Difference ");
  Serial.println(rightEncoder.read() - inchesToTicks(2));
}

//test the right turn of the robot
void testRightTurning () {
  writeStrLCD("Right Turn:", 0, 0);
  writeStrLCD("Dir:", 0, 1);
  writeIntLCD(direct, 5, 1);
  lastEncTicks = 0;
  if (!turnRight90(direct)) {
    writeStrLCD("Y", 13, 0);
  }
  else {
    writeStrLCD("N", 13, 0);
  }
  Serial.print("Last Enc: ");
  Serial.print(lastEncTicks);
  Serial.print("   Current Enc: ");
  Serial.print(abs(leftEncoder.read()));
  Serial.print("   Difference ");
  Serial.println(abs(leftEncoder.read()) - inchesToTicks(2));
}

//turnToStorage should turn the robot right when coming from the north (facing south)
//should turn the robot left when coming from the south (facing north)
void testTurnToStorage() {
  if (northEnd) {
    writeStrLCD("Turn Right", 0, 0);
  }
  else {
    writeStrLCD("Turn Left ", 0, 0);
  }
  turnToStorage();
}

//tests that the robot correcty counts the perscribed number of intersections and
//stops upon reaching the end
void testGoToIntersect() {
  activeStorage = 4;
  destinationCount = 4;
  writeIntLCD(countInter, 0, 0);
  goToIntersect();
  writeIntLCD(destinationCount, 2, 0);
  writeIntLCD(stNav, 4, 0);
}

//makes sure the state is set to the correct stage upon completion
void testInsert() {
  writeIntLCD(stageDS, 0, 0);
  writeIntLCD(bitRead(storageData, activeStorage - 1), 3, 0);
  writeIntLCD(activeStorage, 6, 0);
  Serial.println(stageDS);
  insert();
}

//test that the rod is wholly extracted
void testExtractRod() {
  pickSupply();
  writeIntLCD(stageGN, 0, 0);
  Serial.println(stageGN);
  extractRod();
}

//prints the boolean result of approachS, make sure it matches the physical action (true = done, false = not done)
void testApproachS() {
  writeStrLCD("Front: ", 0, 0);
  writeIntLCD(digitalRead(frontLimit), 7, 0);
  boolean approached = approachS();
  writeIntLCD(approached, 0, 1);
  if (approached) {
    writeStrLCD("Done", 2, 1);
  }
}

//tests that the robot registers when it hits the centerline
void testReturnToCenter() {
  if (returnToCenter()) {
    spNav = turnNorthSouth;
  }
  writeStrLCD("ICnt: ", 0, 0);
  writeIntLCD(countInter, 7, 0);
  writeStrLCD("spNav: ", 0, 1);
  writeIntLCD(spNav, 8, 1);
}

//tests that the robot turns right when destinationCoun > 0
//turns left when destinationCount < 0
//goes straight if destinationCount = 0
void testTurnDestination() {
  destinationCount = -1;
  writeStrLCD("DCnt: ", 0, 0);
  writeIntLCD(destinationCount, 7, 0);
  if (destinationCount > 0) {
    writeStrLCD("Turn Right ", 0, 1);
  } else if (destinationCount < 0) {
    writeStrLCD("Turn Left  ", 0, 1);
  } else if (destinationCount == 0) {
    writeStrLCD("Go Straight", 0, 1);
  }
  if (turnDestination(destinationCount)) {
    writeStrLCD("Done", 9, 0);
  }
  //  writeStrLCD("Turn: ", 9,0);
  //  writeIntLCD(turnDestination(destinationCount),15,0);
}

//tests that the robot drives to the correct intersection, based on the pickSupply algorithm
void testGoToDestIntersection() {
  activeStorage = 1;
  activeSupply = 1;
  destinationCount = 5 - activeStorage - activeSupply;
  goToDestIntersection();
  writeStrLCD("AStr: ", 0, 1);
  writeIntLCD(activeStorage, 6 , 1);
  writeStrLCD("ASupp: ", 7, 1);
  writeIntLCD(activeSupply, 13, 1);
  writeStrLCD("DCnt: ", 0, 0);
  writeIntLCD(destinationCount, 7, 0);
  writeStrLCD("Cnt: ", 10, 0);
  writeIntLCD(countInter, 15, 0);
}


void testPathTravel() {
  pathTravel();
  writeStrLCD("DCnt: ", 0, 0);
  writeIntLCD(destinationCount, 7, 0);
  writeStrLCD("cntI: ", 9, 0);
  writeIntLCD(countInter, 15, 0);
  writeStrLCD("spNav: ", 0, 1);
  writeIntLCD(spNav, 9, 1);
  writeIntLCD(leftEncoder.read(), 11, 1);
}

//test 0 case, >0 case, <0 case
//for = 0, robot should drive straight across the board
void testGoToSupply() {
  goToSupply();
  writeStrLCD("DCnt: ", 0, 0);
  writeIntLCD(destinationCount, 7, 0);
  writeStrLCD("cntI: ", 9, 0);
  writeIntLCD(countInter, 15, 0);
  writeStrLCD("spNav: ", 0, 1);
  writeIntLCD(spNav, 9, 1);
  writeIntLCD(leftEncoder.read(), 11, 1);
}

void testGetNew() {
  getNew();
  printStage(stageGN);
  writeStrLCD("DCnt: ", 0, 0);
  writeIntLCD(destinationCount, 7, 0);
  writeStrLCD("cntI: ", 9, 0);
  writeIntLCD(countInter, 15, 0);
  writeStrLCD("spNav: ", 7, 1);
  writeIntLCD(spNav, 15, 1);
}

//makes sure the robot will turn north when north is active, and south when south is active
void testTurnHome() {
  writeIntLCD(northEnd, 0, 0);
  turnHome();
}

//makes sure the robot ignores intersections,
//stops when it reaches the reactor
//iteractes the stage to drop_off (value of 3)
void testDriveToReactor() {
  printStage(stageDN);
  writeStrLCD("FrontLim:", 0, 0);
  writeIntLCD(digitalRead(frontLimit), 10, 1);
  driveToReactor();
}


void testSwitchField() {
  switchField();
  printStage(state);
  writeIntLCD(countInter, 1, 0);
  writeIntLCD(completedCycles, 0, 0);
}

void testRunRodReplace() {
  runRodReplace();
  printStage(state);
  switch (state) {
    case pickUpSpent_s:
      writeIntLCD(isSettled(armDownValue), 6, 1);
      writeIntLCD(isSettled(armUpValue), 8, 1);
      writeIntLCD(analogRead(armPot), 10, 1);
      break; //run the pick-up stuff in the north branch
    case depositSpent_s: depositSpent();
      writeStrLCD("C:", 6, 1);
      writeIntLCD(countInter, 8, 1);
      writeStrLCD(" D:", 9, 1);
      writeIntLCD(destinationCount, 12, 1);
      writeIntLCD(stageDS, 13, 1);
      writeIntLCD(stNav, 15,1);
      break;  //show countInter, destinationCount, and stage of deposit Spend on bottom row, should see Low Radiation on the top
    case getNew_s: getNew();
      writeStrLCD("C:", 6, 1);
      writeIntLCD(countInter, 8, 1);
      writeStrLCD(" D:", 9, 1);
      writeIntLCD(destinationCount, 12, 1);
      writeIntLCD(stageGN, 14, 1);
      break; //show countInter, destinationCount, and stage of get new on bottom row, should NOT see a radiatio alert in the top row
    case depositNew_s: depositNew();
      writeIntLCD(isSettled(armDownValue), 6, 1);
      writeIntLCD(isSettled(armUpValue), 8, 1);
      writeIntLCD(analogRead(armPot), 10, 1);
      break; //show the settled value of the arm (bottom then top), and the value from the potentiometer
    case switchEnds_s: switchField();    
      writeStrLCD("C:", 6, 1);
      writeIntLCD(countInter, 8, 1);
    break; //switch ends of the field
  }
  //  writeStrLCD("D:", 7,1);
  //  writeIntLCD(destinationCount, 10, 1);
  //  writeStrLCD("C:", 12, 1);
  //  writeIntLCD(countInter, 14, 1);
}

void testSharpLeftAndRight(){
  sharpLeft();
  delay(1000);
  sharpRight();
  delay(1000);
}

