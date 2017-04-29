/*
   logic: assume the robot is facing away from the reactor tube, with the arm raised.
          - which end of the field is it on? (north or south)
          - does it need to turn around first? (facing north/south, answer yes/no)
          which storage tube is it going to?
          how many intersections away is that storage tube?
          is that storage tube on its left or right? (which way is west?)

          start driving until correct number of intersections have been counted
          turn 90 degrees west (will be right when facing south, left when facing north)
          go forward along the line until the limit switch is pressed

                  north tube (B)
                      |
                      |
          Storage     |     Supply
              1 ------------- 4
                      |
              2 ------------- 3
                      |
              3 ------------- 2
                      |
              4 ------------- 1
                      |
                      |
                      |
                 south tube (A)
                  if (countInter == destinationCount) { //if the robot reached the desired intersection
    if(approachS()){
        stageDS = insert_s;
    }
  }
  else {
    followLine();
    pollIntersections();
  }
*/

//TOTEST
//drives the robot to the designated active storage tube
void goToStorage() {
  switch (stNav) {
    case countInters: goToIntersect(); break;
    case turnWest: turnToStorage(); break;
    case approachStorage:
      if (approachS()) {
        stageDS = insert_s;
        hbCount = 0;
      } break;
  }

}
//TODO
//polls for intersects, drives the robot to the intersection
//that corresponds to the active storage tube
void goToIntersect() {
  if (countInter == destinationCount) {  //if the robot is at the destination
    stopDrive();                         // stop
    stNav = turnWest;                    //go to next state
    resetEncoders();
  } else {
    followLine();
    pollIntersections();
  }
}

//turns the robot right or left, depending on which end
//it is operating on
void turnToStorage() {
  if (northEnd) { //coming from the north, west is to the right
    if (turnRight90(direct)) {
      stNav = approachStorage;
    }
  }
  else { //coming from the north, west is to the left
    if (turnLeft90(direct)) {
      stNav = approachStorage;
    }
  }
}

//given the recieved data from the field controller,
//picks which storage unit the robot should deposit
//its rod in, sets activeStorage 1-4
//from the north end, preference is given to the first storage tube
//from the south end, preference is given to the fourth storage tube
//destination count stores the number of intersections the robot has to count
//to get from where it is to the active storage or supply rod
void pickStorage() {
  if (northEnd) {
    if (!bitRead(storageData, 0)) {
      activeStorage = 1;
    } if (!bitRead(storageData, 1)) {
      activeStorage = 2;
    } if (!bitRead(storageData, 2)) {
      activeStorage = 3;
    } if (!bitRead(storageData, 3)) {
      activeStorage = 4;
    }
    destinationCount = 5 - activeStorage;
  }
  else
  {
    if (!bitRead(storageData, 3)) {
      activeStorage = 4;
    } if (!bitRead(storageData, 2)) {
      activeStorage = 3;
    } if (!bitRead(storageData, 1)) {
      activeStorage = 2;
    } if (!bitRead(storageData, 0)) {
      activeStorage = 1;
    }
    destinationCount = activeStorage;
  }
  countInter = 0;
  stageDS = goTo_s;
}

//insert inserts the rod into the storage tube, checking to see that
//the field computer registers the rod, and that the encoder has counted
//enough of a rotation to actually have inserted the rod
void insert() {
  if (bitRead(storageData, activeStorage - 1) || hbCount > 2) { //if the field computer registers the rod and the robot has been inserting long enough code to potentially add back -> && 
    lowLevel = false;
    clearTopLine();
    stopGrip();
    resetEncoders();
    stageDS = releaseRod_s;
  } else {
    dispenseRod();
  }
}

//releaseRod ensures that the robot does not accidentally pull the rod
//back out of the storage tube as it backs away. The grip motor is set to output while the robot reverses slightly.
//if the field computer loses track of the tube, the robot reverts to input mode
void releaseRod() {
  if (abs(rightEncoder.read()) < inchesToTicks(3)) {  //if the robot hasn't backed all the way up yet
    if (!bitRead(storageData, activeStorage - 1)) { //if the field computer doesn't sense the rod anymore
      lowLevel = true;                                  //turn the low level radiation flag back on
      dispenseRod();                               //revert to insert mode
    } else {                                        //field computer still has rod registered
      reverse(fullPower);                               //back up
      dispenseRod();                                    //keep the grip motor in output mode
    }
  } else {                                       //robot has backed up
    stopDrive();                                       //stop driving and dispensing
    stopGrip();
    stageDS = turnAroundS_s;                           //go to next stage
  }
}



