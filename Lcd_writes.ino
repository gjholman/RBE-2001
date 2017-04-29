/*
 * use these to write integers and strings to the LCD as needed
 */

//writes the given string to the lcd in the given location
void writeStrLCD(String s, int x, int y){
  lcd.setCursor(x,y);
  lcd.print(s);
}

//writes the given integer to the lcd in the given location
void writeIntLCD(int s, int x, int y){
  lcd.setCursor(x,y);
  lcd.print(s);
}

//for use with the lineFollowProgram. This will print the current
//state (on/off the line) to the lcd and the serial monitor
void printLineStates(){
  switch (integerState) {
    case 0:  Serial.println("Forward");           //drive the robot forwards
             writeStrLCD("Forward    ", 0, 1); break;
    case 4:  Serial.println("goLeft");
             writeStrLCD("goLeft     ", 0, 1); break;             //turn the robot slightly to the left      
    case 1:  Serial.println("goRight");
             writeStrLCD("goRight    ", 0, 1); break;             //turn the robot slightly to the right     
    case 5:  Serial.println("atIntersect");       //robot is at an intersection, either stop or count it?
             writeStrLCD("atIntersect", 0, 1); break;
    default: Serial.println("No state");     break;
  }
}

//prints the destination value of the robot cleanly
void printDestination(int x, int y){
  if (destinationCount < 0) {
    writeIntLCD(destinationCount, x, y);
  }
  else {
    writeStrLCD(" ", 13, 1);
    writeIntLCD(destinationCount, x+1, y);
  }
}

//pritns the stage in the navigation, the value of the front limit switch and the number of inersections counted
void printGoToStorageInfo(){
  printStage(stageDS);
  writeStrLCD("AStor: ", 7,1);
  writeIntLCD(activeStorage, 13,1);
  writeStrLCD("DCnt: ",0,0);
  writeIntLCD(destinationCount, 7,0);
  writeStrLCD("Cnt: ", 10,0);  
  writeIntLCD(countInter, 15,0);
}

//prints the stage of the robot in the bottom left corner
void printStage(int stage){
  writeStrLCD("Stg: ",0,1);
  writeIntLCD(stage,5,1);
}

//empties the top line of the screen
void clearTopLine(){
  writeStrLCD("                ",0,0);
}

//empties the bottom line of the screen
void clearBottomLine(){
  writeStrLCD("                ",0,0);
}

