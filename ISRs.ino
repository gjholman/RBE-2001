/*
 * these are the interrupt service routines for all the 
 * message sends, starts/stops from limit switches and 
 * push buttons
 */

/*
 * sends the heart beat message every time the timer runs down
 * if the lowLevel flag is set to true, sends a low level radiation alert
 * if the highLevel flag is set to true, sends a high level radiation alert
 */
 void flagSendHB(){
  if(sendCount > 5){
    readMessagesFlag = true;
  }
  if(sendCount == 10){
  sendHB = true;
  hbCount ++;
  sendCount = 0;
  } else {
    sendCount++;
  }
 }





