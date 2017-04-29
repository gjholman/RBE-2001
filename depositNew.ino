/*
 * from any supply tube, the robot must:
 *  - return to the centerline
 *  - turn towards the active reactor tube (north or south)
 *  - drive there
 *  - insert the new fuel rod
 *  - turn around
 *  available states:depart_s, turnHome_s, goToReactor_s, dropOff_s, turnAroundN_s
 */

//return to centerline drives the robot back to the center
//assumes countInter starts at 0
boolean returnToCenter() {
  pollIntersections();
  if (countInter == 1) {
    return true;
    countInter = 0;
  } else {
    followLine();
    return false;
  }
}

//turns the robot towards the empty reactor tube
void turnHome() {
  if (northEnd) {              //if robot is operation on the north end
    if (turnRight90(direct)) {    //turn right
      stageDN = goToReactor_s;    //go to drive state when done turning
    }
  } else {                    //if robot is operating on the south end
    if (turnLeft90(direct)) {  //turn left
      stageDN = goToReactor_s;    //go to drive state when done turning
    }
  }
}

//drives the robot until it hits the reactor tube
void driveToReactor() {
    if(approachS()){    //if the robot has reached the reactor
      stageDN = dropOff_s;  //go to the drop off phase
    }
}


