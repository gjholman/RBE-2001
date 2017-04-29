/* Everett Harding - group 6
   This is the whole program for the RBE 2001 Final project
   Its goal is to remove a spend reactor rod from a vertically oriented tube
   then drive (following a line) to a horizontally oriented storage tube and
   re-insert the rod there. Next, it will drive to an available supply tube
   and take the rod from there to the original reactor tube and insert it.
   After doing so, it will then repeat the process for a rod at the opposite
   end of the field

   Still to be done:
    -add clear LCD to insertion functions for final presentation
*/

// necessary include files
#include <Arduino.h>
#include <BluetoothClient.h>
#include <BluetoothMaster.h>
#include <ReactorProtocol.h>
#include <TimerOne.h>
#include <Encoder.h>
#include <LiquidCrystal.h>

//-------------------------------sensor port definitions (may need to re-evaluate if calls for too many interrupts)
#define frontLimit 27
#define armPot 0
#define leftLine 25       //line sensors
#define rightLine 26
#define thirdLine 4
#define goSwitch 36       //go button sensor
#define lMEncoderPin1 20   //motor encoder pins
#define lMEncoderPin2 23
#define rMEncoderPin1 21
#define rMEncoderPin2 24
#define gEncoderPin1  3  //gripper motor
#define gEncoderPin2  22
z
//-------------------------------output object pin definitions
#define leftMotorPin1 5   //motor pins (2 per motor to run h-bridge)
#define leftMotorPin2 4
#define rightMotorPin1 9
#define rightMotorPin2 8
#define armMotorPin1 10
#define armMotorPin2 11
#define gripMotorPin1 6
#define gripMotorPin2 7
#define timingPin 28      //used to test latency of loop


//-------------------------------constants
#define motorStopPower 0
#define fullPower 255
#define armDownValue 650
#define armUpValue 815
#define extractValue 838
const int intakeTime = 1800;          //time to extract or insert a rod
const boolean forwards = true;
const boolean backwards = false;
//-------------------------------objects to be used
Encoder leftEncoder(lMEncoderPin1, lMEncoderPin2);
Encoder rightEncoder(rMEncoderPin1, rMEncoderPin2);
Encoder gripEncoder(gEncoderPin1, gEncoderPin2);
LiquidCrystal lcd(40, 42, 44, 46, 48, 52);

//--------------------------------Bluetooth Declarations (taken from sample code)
#define thisROBOT 6                            // define our team number
ReactorProtocol pcol(byte(thisROBOT));         // instantiate the protocol object and set the robot/team source address
BluetoothClient bt;                            // instantiate a Bluetooth client object
BluetoothMaster btmaster;                      // instantiate a master object

byte pkt[10];                                  // allocate memory for the bytes in the packet
int sz;                                        // holds the size of the message (in bytes)
byte type;                                     // hold the message type id
byte data[3];                                  // holds any data associated with a message

//--------------------------------state variable enumerations
//-----------------general enum-------
 enum Compass{north, south, east, west}direct = north;                                           //used to keep track of robot orientation on the field
 enum State{pickUpSpent_s, depositSpent_s, getNew_s, depositNew_s, switchEnds_s}state = pickUpSpent_s;  //used to determine which stage of replacing a rod the robot is in
//-----------------typedefs for 2nd-level operations
 enum StagePU{driveToReactor_s, pickUpRod_s, turnAroundP_s}stagePU = driveToReactor_s;                           //stages for pickUp
 enum StageDS{pickStorage_s, goTo_s, insert_s, releaseRod_s, turnAroundS_s}stageDS = pickStorage_s;              //stages for depositSpent
 enum StageGN{pickSupply_s, goToSupply_s, remove_s, turnAroundG_s}stageGN = pickSupply_s;                        //stages for getNew
 enum StageDN{depart_s, turnHome_s, goToReactor_s, dropOff_s, turnAroundN_s}stageDN = depart_s ;                 //stages for depositNew
 enum StorageNav{countInters, turnWest, approachStorage}stNav = countInters;                                     //stages for pathFollowing storage navigatsion
 enum SupplyNav{returnToMiddle, turnNorthSouth, countLines, turnEast, approachSupply}spNav = returnToMiddle;     //stages for the pathFollowing supply navigation
 enum reactorInterationStates {lowerArm, intakeRod, raiseArm} intakeState = lowerArm;                            //stages for the reactor interactions

 
//-----------------volatile variables
volatile boolean go = false;                   // used to make the robot wait until bluetooth connection is established before beginning its run
volatile boolean sendHB = true;                // flags that it is time to send the HB message
volatile boolean lowLevel = false;             // flags if there is a low level of radiation (exposed spend rod)
volatile boolean highLevel = false;            // flags if there is a high level of radiation (exposed new rod)
volatile boolean driveFlag = true;             // determines whether the robot has recieved stop/move messages
volatile boolean goToNext = false;             // used to determine whether the robot has complete the tast of its current state
volatile boolean readMessagesFlag = true;      // tell the robot to read messages
volatile int sendCount = 0;                    // counts in tenths of a second
volatile int integerState = 5;                 // 0 is intersection, 1 is goRight, 4 is goLeft, 5 is forward
volatile int countInter = 0;                   // used to count number of intersections passed
volatile int destinationCount = 0;             // holds the number of intersections to count
volatile int hbCount = 0;               // use to judge time by the heartBeat
//-----------------variables------

int power = motorStopPower;                    //motor power begins at stop value
boolean northEnd = true;                       //true = operating on the north reactor tube. When false, it means the robot is operating on the southern reactor tube
boolean lastThird = true;                      //stores the last polled value of the third line sensor
boolean currentThird = true;                   //stores the value of the current reading of the third line sensor
int completedCycles = 0;                            //counts the number of times the robot has completed a RodCycle
int activeStorage;                             //chosen storage to deposit a spent rod
int activeSupply;                              //chosen supply to pick up a new rod
int lastTime;                                  //used for the occasional dead reconing (extraction/insertion of rods)
long lastEncTicks;                              //used to determine difference in encoder values
byte storageData;                              // holds the bitmask for the storage tubes
byte supplyData;                               // and the supply tubes



//--------------------------------Start setup code
void setup() {
  Serial.begin(115200);                    //initialize computer serial
  Serial1.begin(115200);                   //initialize bluetooth serial
  lcd.begin(16, 2);                        //initialize the lcd screen

  pinMode(frontLimit, INPUT_PULLUP);//-------set up all input pins
  pinMode(armPot, INPUT);
  pinMode(leftLine, INPUT_PULLUP);
  pinMode(rightLine, INPUT_PULLUP);
  pinMode(thirdLine, INPUT_PULLUP);
  pinMode(goSwitch, INPUT_PULLUP);
  pinMode(lMEncoderPin1, INPUT);            //encoders use external pullups
  pinMode(lMEncoderPin2, INPUT);
  pinMode(rMEncoderPin1, INPUT);
  pinMode(rMEncoderPin2, INPUT);
  pinMode(gEncoderPin1, INPUT);
  pinMode(gEncoderPin2, INPUT);

  pinMode(leftMotorPin1, OUTPUT); //---------set up all output pins, intial output of LOW
  digitalWrite(leftMotorPin1, LOW);
  pinMode(leftMotorPin2, OUTPUT);
  digitalWrite(leftMotorPin2, LOW);
  pinMode(rightMotorPin1, OUTPUT);
  digitalWrite(rightMotorPin1, LOW);
  pinMode(rightMotorPin2, OUTPUT);
  digitalWrite(rightMotorPin2, LOW);
  pinMode(armMotorPin1, OUTPUT);
  digitalWrite(armMotorPin1, LOW);
  pinMode(armMotorPin2, OUTPUT);
  digitalWrite(armMotorPin2, LOW);
  pinMode(gripMotorPin1, OUTPUT);
  analogWrite(gripMotorPin1, 0);
  pinMode(gripMotorPin2, OUTPUT);
  analogWrite(gripMotorPin2, 0);
  pinMode(timingPin, OUTPUT);

  //attachInterrupt(digitalPinToInterrupt(frontLimit), flagStop, CHANGE);    //possible use of third line sensor to count intersections
  Timer1.initialize(100000);                                                 //initialize heartbeat timer for one tenth of second
  Timer1.attachInterrupt(flagSendHB);                                        //set it to send an hb message when it finishes
}

void loop() {
  digitalWrite(timingPin, HIGH);
  handleBluetooth();
  if (!digitalRead(goSwitch)) {
    go = true;
    //pickStorage();
    activeStorage = 1;
    activeSupply = 4;
    destinationCount = 3;
    lcd.clear();
    writeStrLCD("Go!",7,0);
    delay(500);  //give the user time to read go
    lcd.clear();  //wipe lcd for other diagnostic writes
    lastTime = millis();
  }
  if (go) {
    //runRodReplace();            //run the full rod replacing program
    //testDrive();
    //testSharpLeftAndRight();
    //testGrip();
    //testPIDControl(armUpValue);
    //testBluetooth();
    //testFollowLine();
    //testCountIntersections();
    //testRemoveFromReactor();
    //testReplaceRod();
    //testEncoder();
    //testApproachReactor();
    //testAboutFace();
    //testPickStorage();
    //testPickSupply();
    //testLeftTurning();
    //testRightTurning();
    //testTurnToStorage();
    //testGoToIntersect();
    //testInsert();
    //testExtractRod();
    //testApproachS();
    //testGoToStorage();
    //testDepositSpent();
    //testReturnToCenter();
    //testTurnDestination();
    //testGoToDestIntersection();
    //testPathTravel();
    //testGoToSupply();
    //testGetNew();
    //testTurnHome();
    //testDriveToReactor();
    //testSwitchField();
    testRunRodReplace(); //start South
  }
  else {
    writeStrLCD("I'm Waiting...", 0,0);
  }
  digitalWrite(timingPin, LOW);
}
