/*
   based on the activeStorage, this function will pick a path
   to a chosen supply tube. Each time, it will need to return to the middle
   line and turn towards the required supply tube.
   destinatin value calculation: destination = 5 - activeStorage - active supply

   active storage (current location)  |  destination values
                                      | supply 4 | supply 3 | supply 2 | supply 1 |
                            storage 1 |     0    |    1     |    2     |    3     |
                            storage 2 |     -1   |    0     |    1     |    2     |
                            storage 3 |     -2   |    -1    |    0     |    1     |
                            storage 4 |     -3   |    -2    |    -1    |    0     |

   for each storae supply: return to middle
                           turn left/turn right
                           count intersections
                            - figure out algorithm for determining path
                           turn left if turned right, turn right if turned left
                           approach

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


*/


//drives the robot from one of the four storage tubes (designated as active storage
//to the active supply tube
void goToSupply() {
  if (destinationCount == 0) {
    if (approachS()) {
      stageGN = remove_s;
    }
  }
  else {
    pathTravel();
  }
}

//picks which state of the travel the robot is in
//return to middle - drive the robot back to the center line then progress state, assume the robot is already facing the center line
//turnNorthSouth - turns to the north or south depending on destinationCount >, < 0
//countLines - drives the robot to the intersection of the active supply tube
//turnEast - turns the robot towards the supply tube
//approachSupply - drives the robot up to the supply tube
void pathTravel() {
  switch (spNav) {
    case returnToMiddle:
      if (returnToCenter()) {
        spNav = turnNorthSouth; 
        resetEncoders(); break;
      } break; //return to the middle, the state is then iterated by 1 to turnNorthSouth
    case turnNorthSouth:
      if (turnDestination(destinationCount)) {
        spNav = countLines; break;
      }  break;  //turn right or left based on the given destination value
    case countLines: goToDestIntersection(); 
    break;
    case turnEast:
      if (turnDestination(0 - destinationCount)) {
        spNav = approachSupply;
      } 
      break;       //turn the opposite way from turnNorthSouth
    case approachSupply: //approaches the supply tube
      if (approachS()) {
        stageGN = remove_s;
      } 
      break;
  }
}
//turns the robot right or left based on the input
//destination value
boolean turnDestination(int dest) {
  boolean isDone = false;
  if (dest > 0) {   
      isDone = turnRight90(direct);
      countInter = 0;
  } else if (dest < 0) {
      isDone = turnLeft90(direct);
      countInter = 0;
  } else {
    isDone = false;
    countInter = 0;
  }
  return isDone;
}

//changes the spNav state to the next in the sequence
void nextNav() {
  switch (spNav) {
      switch (spNav) {
        case turnNorthSouth: spNav = countLines; break;
        case countLines: spNav = turnEast; break;
        case turnEast: spNav = approachSupply; break;
        case approachSupply: spNav = turnNorthSouth; break;
      }
  }
}

//drives robot to the correct number of intersections
void goToDestIntersection() {
  if (countInter == abs(destinationCount)) { //if the robot has reached the desired intersection
    stopDrive();
    resetEncoders();
    spNav = turnEast;
  } else {
    pollIntersections();
    followLine();
  }
}

//given the recieved data from the field controller,
//picks which supply unit the robot should deposit
//its rod in, sets activeSupply 1-4
//from the north end, preference is given to the first supply tube
//from the south end, preference is given to the fourth supply tube
void pickSupply() {
  if (northEnd) {
    if (bitRead(supplyData, 0)) {
      activeSupply = 1;
    }
    if (bitRead(supplyData, 1)) {
      activeSupply = 2;
    }
    if (bitRead(supplyData, 2)) {
      activeSupply = 3;
    }
    if (bitRead(supplyData, 3)) {
      activeSupply = 4;
    }
  } else {
    if (bitRead(supplyData, 3)) {
      activeSupply = 4;
    }
    if (bitRead(supplyData, 2)) {
      activeSupply = 3;
    }
    if (bitRead(supplyData, 1)) {
      activeSupply = 2;
    }
    if (bitRead(supplyData, 0)) {
      activeSupply = 1;
    }
  }
  destinationCount =  activeStorage - activeSupply;
  countInter = 0;
}

//remove extracts a new rod from the supply tube and
//sets the high radiation flag to true
void extractRod() {
  if (bitRead(supplyData, activeSupply - 1)) { //if the active supply is still active or the encoder hasn't reached rod length
    //controlArmPID(extractValue);  //set the arm to extraction height
    driveForward(40); //gently press the robot against the post for a more consistant grip
    removeRod();
  } else { //rod has been extracted
    stopGrip();
    highLevel = true;
    writeStrLCD("!High Radiation!",0,0);       //print a high level alert
    stageGN = turnAroundG_s;
  }
}

