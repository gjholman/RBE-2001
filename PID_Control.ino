//PID attempt, Lab 4 Prelab
//Everett Harding
/*
 * PID algorithm:
 *  Output = gainP * currentError + gainI * (sum pastError) + gainD * (diff currentError pastError)
 *  
 *  Uses int because the pot value will always be read as an integer
 */


const int storeSize = 15; //change to edit how many past errors are stored
const int potSetValue = 400; //change this to set desired angle
const int settleThreshHold = 6; //settling error
double gainP = 4.5;
double gainI = 0.5;
double gainD = 0.01;
double history[storeSize]; //holds the last 10 values of the potentiometer, 0 is least recent, 9 is most recent
int out;
//controls the position of the arm based on the set value
void controlArmPID(int setValue){
  out = output(setValue); //only call output() once per loop
  if(analogRead(armPot) > setValue){
  analogWrite(armMotorPin1, out); //when turning motor counter clockwise
  analogWrite(armMotorPin2, 0);
  //writeStrLCD(" CW",12,0);
  Serial.println(" CW");
  } else {
  analogWrite(armMotorPin2, out); //when turning motor clockwise
  analogWrite(armMotorPin1, 0);
  //writeStrLCD("CCW",12,0);
  Serial.println(" CCW");
  }
}

//is settled checks that the values stored in history are roughly the same
//level of error, which would indicate that the arm has reached a stable settled point
boolean isSettled(int setValue){
    return (analogRead(armPot) >= (setValue - settleThreshHold)) && (analogRead(armPot) <= (setValue + settleThreshHold));
}

//calculates the difference betweent the goal set value
//and the current value
int calcError(int setValue){
  return abs(analogRead(armPot) - setValue);
}

//adds the total of all past error values stored
int sumPastError(int setValue){
  updateHistory(setValue);
  int total = 0;
  for(int i = 0; i < storeSize; i++){
    total += history[i];
  }
  return total;
}

//adds the current error to the history array
//"deletes" the oldest value
void updateHistory(int setValue){
  double current = calcError(setValue);
  for(int i =0; i < storeSize; i++){
    if(i == storeSize - 1){ //reached last element in history
      history[i] = current;
    }else{
      history[i] = history[i+1]; //set move each stored error value one down the list
    }
  }
}

//sets all values in the history array to 0
void clearPIDHistory(){
  for(int i = 0; i < storeSize; i++)
  history[i] = 0;
}

//calculats the difference between the current reading and the previous reading
int calcDifference(){
  return analogRead(armPot) - history[storeSize - 1];
}

/*
 * begins at motor stop and adds calculated change 
 * output will be 90 when all gains are 0
 */
int output(int setValue){
  return constrain((gainP * calcError(setValue)) + (gainI * sumPastError(setValue)) + (gainD * calcDifference()), 0, 255);
}


