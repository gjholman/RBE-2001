/*
 * These are the functions for the robot when
 * operating in the north end of the field
 * The necessary definitions are included below
 */


//robot picks up the reactor tube in the north end of the field
void pickUp() {
  switch (stagePU) {
    case driveToReactor_s: //stuffs
      approachReactor();
      break;
    case pickUpRod_s:
      removeFromReactor();
      break;
    case turnAroundP_s:
      aboutFace(10);  //give the robot how many inches to reverse before turning around
      intakeState = lowerArm; //reset the insertion/extraction state for the deposit new state
      break;

  }
}

//TODO
void depositSpent() {
  switch (stageDS) {
    case pickStorage_s: //select the active storage tube, set destinationCount, reset countInter
      pickStorage();
      stageDS = goTo_s;
      break;
    case goTo_s:  //drive the robot to the active storage tube
      goToStorage();
      break;
    case insert_s: //insert the rod into the storage tube, turn off low level radiation alert
      insert();
      break;
    case releaseRod_s: //ensure rod is released (extra turn or two of the gripper), if it isn't go back to insert mode
      releaseRod();
      break;
    case turnAroundS_s: //turn robot around, progress to next state
      aboutFace(1);
      break;
  }
}

//robot will drive from the current active storage tube
//to an available supply tube, extract a new rod, and turn around
void getNew() {
  switch (stageGN) {
    case pickSupply_s: //select the active supply, set destinationCount, reset countInter
      pickSupply();
      stageGN = goToSupply_s;
      break;
    case goToSupply_s: //drive to the active supply
      goToSupply();
      break;
    case remove_s:  //remove the rod, turn on high radiation alerts
      extractRod();
      break;
    case turnAroundG_s: //turn around, progress to next state
      aboutFace(3);
      break;
  }
}

void depositNew() {
  switch (stageDN) {
    case depart_s: //robot returns to center line
      if(returnToCenter()){
        stageDN = turnHome_s;
      }
      break;
    case turnHome_s: //turns towards the empty reactor tube
      turnHome();
      break;
    case goToReactor_s: //drives until it reaches the tube
      driveToReactor();
      break;
    case dropOff_s: //insert rod into the reactor tube
      replaceRod();
      break;
    case turnAroundN_s: //turn robot around, progress to next state
      aboutFace(7);
      break;
  }
}

//drives to the other end of the field
//upon reeching the destination, resets all states and accumulators
//then begins the cycle again. If two cycles have been completed,
//it prints Done! to the lcd
void switchField() {
  if (completedCycles < 1) {
    pollIntersections();
    if (countInter == 5) {
      northEnd = !northEnd;
      stopDrive();
      resetAll();
      completedCycles++;
      state = pickUpSpent_s;
    } else {
      followLine();
    }
  } else {
    stopDrive();
    lcd.clear();
    writeStrLCD("Done", 7, 0);
    writeStrLCD("You're Welcome", 1, 1);
  }

}

//returns the direction 90 degrees to the left
int changeDirectionLeft(int dir) {
  switch (dir) {
    case 0: return west; break;
    case 1: return east; break;
    case 2: return north; break;
    case 3: return south; break;
  }
}

//returns the direction 90 degrees to the right
int changeDirectionRight(int dir) {
  switch (dir) {
    case 0: return east; break;
    case 1: return west; break;
    case 2: return south; break;
    case 3: return north; break;
  }
}

//uses two boolean values to count the intersections the robot has passed
void pollIntersections() {
  pollThirdLine();
  if (!lastThird && currentThird) {
    countInter++;
  }
}

//sets the boolean value last to the same value as current
//then sets the current value to the current high/low reading
//of the third line sensor
void pollThirdLine() {
  lastThird = currentThird;
  currentThird = analogRead(thirdLine) > 700;
}

//approach drives the robot along a line until the front limit switch is pressed
//it returns true when the limit switch is pressed
//this means that the robot has finished approaching
boolean approachS() {
  boolean limitSwitch = digitalRead(frontLimit);
  if (limitSwitch) { //if the front switch is not pressed
    followLine();
    return false;
  } else {
    stopDrive();
    resetEncoders();
    return true;
  }
}

//resets all states and accumulators to their values for the
//beginning of a rod cycle
void resetAll() {
  state = pickUpSpent_s;              //the robot begins by picking up a spend rod
  stagePU = driveToReactor_s;         //robot starts at drive to reactor
  stageDS = pickStorage_s;            //robot starts by pick which available storage to use
  stageGN = pickSupply_s;             //robot starts by picking a supply tube
  stageDN = depart_s;                 //robot starts by returning to the center
  stNav = countInters; 
  spNav = returnToMiddle;
  intakeState = lowerArm; 
  countInter = 0;
  destinationCount = 0;
}

//resets all encoder values to zero to aid turning logic
//must call this from the state BEFORE the turn, not in the turn state
void resetEncoders(){
  lastEncTicks = 0;
  leftEncoder.write(0);
  rightEncoder.write(0);
}

