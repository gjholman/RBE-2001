//this is the top level state machine for the rod replacing as a whole
void runRodReplace() {
  switch (state) {
    case pickUpSpent_s: pickUp();        break; //run the pick-up stuff in the north branch
    case depositSpent_s: depositSpent(); break; //run the depsoit spent rods from the north
    case getNew_s: getNew();             break; //run the get new rods branch
    case depositNew_s: depositNew();     break; //deposit the rod in the north end of the field
    case switchEnds_s: switchField();    break; //switch ends of the field
  }
}

//handles the sending/recieving of bluetooth messages
void handleBluetooth() {
  if (readMessagesFlag) {
    readMessagesFlag = false;
    readMessages();                              //read supply/storage date over bluetooth
  }
  if (lowLevel && sendHB) { //if there is an exposed spend rod
    sendSpentRodAlert();                       //send a low level radiation warning
    //writeStrLCD(" Low Radiation! ", 1, 0);       //print a low level alert
  }
  if (highLevel && sendHB) {                             //if there is an exposed new rod
    sendNewRodAlert();                         //send a high level radiation alert
   // writeStrLCD("!High Radiation!", 0, 0);     //print a high level alert
  }
  if (sendHB) {                                //if it is time to send a HB message
    sendHB = false;                            //invert the hb flag
    sendHBMessage();                           //send the message
  }
}

//toNextState changes the overall state to the next one in the progression
//except for in the switchEnds state, it is only called from aboutFace,
//which is used to complete the final stage of any other state, the turn around
void toNextState() {
  switch (state) {
    case pickUpSpent_s: state = depositSpent_s; break;
    case depositSpent_s: state = getNew_s; break;
    case getNew_s: state = depositNew_s; break;
    case depositNew_s: state = switchEnds_s; break;
    case switchEnds_s: state = pickUpSpent_s; break;   //cycle back to the pickupstage
  }
}


