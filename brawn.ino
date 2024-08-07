using namespace std;
//general
#include<string.h>
#include<stdio.h>
#include<stdlib.h>
//display
#include <Wire.h>
#include "Waveshare_LCD1602_RGB.h"
//servo control
#include <Servo.h>
//CAN bus
#include <SPI.h>          //Library for using SPI Communication 
#include <mcp2515.h>      //Library for using CAN Communication  autowp-mcp2515 by autowp . V1.0.3
//timers
#include <SoftTimers.h>

//boot?
int boot = 0;
int NUM_MOTORS = 12;

//========= Serial communication ===========
//incoming command string
String cmd;
//reply string
String reply;
//last sent message
String lastMsg;
//new message to send
String newMsg;
//incoming command string array
char *cmdList [12];
//========== Motor attributes ===============
//going in order LF, RF, LB, RB, Hip-Shoulder-Knee (e.g. LFHip = theoMotorVals[0])
double theoMotorVals[12];
double currMotorVals[12];
double theoMotorTorques[12];
double currMotorTorques[12];
double upperBounds[12];
double lowerBounds[12];
unsigned int canIDs[12];
unsigned int errorMsgs[12];
//========== CAN communications =============
struct can_frame canMsg;
struct can_frame recievedCANMsg;
MCP2515 mcp2515(10);
unsigned char WAIT_TIME = 15; //ms
unsigned char buf[8];
//================ Display ==================
Waveshare_LCD1602_RGB lcd(16,2);  //16 characters and 2 lines of show
String disp1;
String disp2;
const char* outputDisp;
int r,g,b,t=0;
//========== Timer =========================
SoftTimerProfilerMillis profiler; //mircosecond profiler

void setup() {
  //CAN BUS initialization
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS,MCP_16MHZ); //set the CAN bus speed to 1 Mbps and the frequency of the crystal oscillator to 16 MHz
  mcp2515.setNormalMode();
  //start serial comms
  Serial.begin(9600);
  // initialize display
  lcd.init();
  //version info, etc
  disp1 = "DogOS 2.2";
  //status update
  disp2 = "Booting Brawn...";
  display();
  //initialize motor IDs
  initialize();

  //timer
  profiler.reset(); //resets timer value
}

void loop() {
  profiler.start();
  //display stuff
  r = (abs(sin(3.14*t/180)))*255;
  g = (abs(sin(3.14*(t + 60)/180)))*255;
  b = (abs(sin(3.14*(t + 120)/180)))*255;
  t = t + 3;
  lcd.setRGB(r,g,b);

  //checks if serial input, updates commands
  if(Serial.available()!=0){
    //sends current motor positions to Pi first, if first boot
    if(boot == 0){
      Serial.println(updatePi('p'));
      disp2 = "success!";
      boot = 1;
    }

    //command string 
    cmd = "";
    //gathers new input
    cmd = Serial.readString();  //read received data

    //dev commands
    if(cmd == "std"){
      stand();
    }
    if(cmd == "ld"){
      legsDown();
    }
    if(cmd == "lu"){
      legsUp();
    }
    if(cmd == "lh"){
      levelHips();
    }
    if(cmd == "lo"){
      legsOut();
    }
    if(cmd == "li"){
      legsIn();
    }
    if(cmd == "zero"){
      zeroMotors();
    }
    if(cmd == "error") {
      readMotorErrors();
      for(int i = 0; i < NUM_MOTORS; i++) {
        Serial.print(errorMsgs[i], HEX);
        Serial.print("  ");
      }
      Serial.print("\n"); 
    }
    if (cmd == "time") {
      profiler.end();
      profiler.print("Loop Time");
    }
    if (cmd == "pos") {
      Serial.println(updatePi('p'));
    }


    //if not any of the preprogrammed commands, do the standard analysis:
    //format: rFHip, rfHipSpeed, rFShoulder, rFShoulderSpeed, rFKnee, rFKneeSpeed, lFHip, lFHipSpeed, lFShoulder, lFShoulderSpeed, lFKnee, lFKneeSpeed, rBHip, rBHipSpeed, rBShoulder, rBShoulderSpeed, rBKnee, rBKneeSpeed, lBHip, lBHipSpeed, lBShoulder, lBShoulderSpeed, lBKnee, lBKneeSpeed
    //Then, cmd[0] = rFHip position to move to, cmd[1] = rfHip speed to move by
    //cmd[0,2,4,6,8,10,12,14,16,18,20,22] are movement position commands <--- possibly ignore
    //cmd[1,3,5,7,9,11,13,15,17,19,21,23] are speeds for those movement commands <--- possibly ignore
    //else{
      //TODO: split into string array, maybe look below if that works?
    //}
    //TODO: remove after finishing splitting code
    
    else if (cmd == "dev"){
      //char buffer[12] = cmd;
      //char *token1 = strtok(buffer, ",");
      //theoMotorVals[1] = atoi(strtok(buffer, ",");
      char testcmd = cmd.c_str();
      int i = 0;
      i = testcmd - '0';
      Serial.println("runs");
      Serial.println(i);
      move(0x141, i, 400);
      cmd = "";
      testcmd = (char) 0;
    }

    else{
      char arr[cmd.length() + 1]; 
	    strcpy(arr, cmd.c_str());
      char temp[3];
      /*
      for(int i = 0; i<12; i++){
        //TODO: append digits here
        std::string fs(fc);
        float f=std::stof(fs);
        theoMotorVals[i] = f;
      }
      */
      //do string splitting here
    }
    disp2 = "";
  }
  readMotorPositions();
  disp2 = cmd;
  display();
  //Serial.println(updatePi('p'));
  profiler.stop();

}

//============================================
//movement functions
void stand(){
  levelHips();
  double a1 = (upperBounds[1]+lowerBounds[1])/2;
  double a2 = (upperBounds[4]+lowerBounds[4])/2;
  move(canIDs[1], a1, 100);
  move(canIDs[4], a2, 100);
}
void levelHips(){
  move(canIDs[0], 25, 400);
  move(canIDs[3], 60, 400);
}
void legsOut(){
  move(canIDs[0], lowerBounds[0], 100);
  move(canIDs[3], lowerBounds[3], 100);
}
void legsIn(){
  move(canIDs[0], upperBounds[0], 100);
  move(canIDs[3], upperBounds[3], 100);
}
void legsUp(){
  levelHips();
  move(canIDs[1], upperBounds[1], 100);
  move(canIDs[4], upperBounds[4], 100);
}
void legsDown(){
  levelHips();
  move(canIDs[1], lowerBounds[1], 100);
  move(canIDs[4], lowerBounds[4], 100);
}

//checks the bounds of the desired motor, 
void testBounds(int idIndex, char specifyExtent){
  if(specifyExtent == 'u'){
    move(canIDs[idIndex], upperBounds[idIndex], 500);
  }
  if(specifyExtent == 'l'){
    move(canIDs[idIndex], lowerBounds[idIndex], 500);
  }
}

//============================================
//general functions
void initialize(){
  disp2 = "Initializing CAN...";
  display();
  //define IDs and ranges
  defineMotors();
  //get current positions
  readMotorPositions();
}

void defineMotors(){
  //========= CAUTION: VERIFY ALL VALUES HERE BEFORE TESTING =================//
  //TODO: populate all values, test them, then move!
  //=== motor 1
  canIDs[0] = 0x141;
  upperBounds[0] = 45;
  lowerBounds[0] = 5;
  //=== motor 2
  canIDs[1] = 0x142;
  upperBounds[1] = 105;
  lowerBounds[1] = 5;
  //=== motor 3
  canIDs[2] = 0x143; 
  upperBounds[2] = 110;
  lowerBounds[2] = 0;
  //=== motor 4
  canIDs[3] = 0x144;
  upperBounds[3] = 40;
  lowerBounds[3] = 80;
  //=== motor 5
  canIDs[4] = 0x145;
  upperBounds[4] = 5;
  lowerBounds[4] = 105;
  //=== motor 6
  canIDs[5] = 0x146;
  upperBounds[5] = 0;
  lowerBounds[5] = 110;
  //=== motor 7
  canIDs[6] = 0x147;
  //upperBounds[6] = ;
  //lowerBounds[6] = ;
  //=== motor 8
  canIDs[7] = 0x148;
  //upperBounds[7] = ;
  //lowerBounds[7] = ;
  //=== motor 9
  canIDs[8] = 0x149;
  upperBounds[8] = 110;
  lowerBounds[8] = 0;
  //=== motor 10
  canIDs[9] = 0x140+10; //not sure if this is how to do it
  //upperBounds[9] = ;
  //lowerBounds[9] = ;
  //=== motor 11
  canIDs[10] = 0x140+11; //not sure if this is how to do it
  //upperBounds[10] = ;
  //lowerBounds[10] = ;
  //=== motor 12
  canIDs[11] = 0x140+12; //not sure if this is how to do it
  upperBounds[11] = 0;
  lowerBounds[11] = 110;
}

void setZeroMotor(int idIndex){
  canMsg.can_id  = canIDs[idIndex];  
  canMsg.can_dlc = 0x08;
  canMsg.data[0] = 0x64;
  canMsg.data[1] = 0x00;
  canMsg.data[2] = 0x00;
  canMsg.data[3] = 0x00;
  canMsg.data[4] = 0x00;
  canMsg.data[5] = 0x00;
  canMsg.data[6] = 0x00;
  canMsg.data[7] = 0x00;
  mcp2515.sendMessage(&canMsg);

  int len = WAIT_TIME + 15;
  while ((mcp2515.readMessage(&canMsg) != MCP2515::ERROR_OK)){
    /* check received response CAN */
    delay(1);
    len--;
    if ((len <= 0)) 
    {
      Serial.print("Cannot zero: ");
      Serial.print(idIndex);
      Serial.print("\n");
      return;
    }
  }
  Serial.print("\nZERO MOTOR: ");
  Serial.print(idIndex);
  Serial.print("\n");

}

void setTorque(int idIndex, int16_t torque) {
  canMsg.can_id  = canIDs[idIndex];
  canMsg.can_dlc = 0x08;
  canMsg.data[0] = 0xA1;
  canMsg.data[1] = 0x00; 
  canMsg.data[2] = 0x00;
  canMsg.data[3] = 0x00;
  canMsg.data[4] = (uint8_t) torque;
  canMsg.data[5] = (uint8_t) (torque >> 8);
  canMsg.data[6] = 0x00;
  canMsg.data[7] = 0x00;
  mcp2515.sendMessage(&canMsg);

  int len = WAIT_TIME;
  while ((mcp2515.readMessage(&canMsg) != MCP2515::ERROR_OK)){
    /* check received response CAN */
    delay(1);
    len--;
    if (len <= 0) break;
  }

}
void zeroMotor(int idIndex, int16_t torque) {
  readMotorPositions();
  double lastAngle = currMotorVals[idIndex];
  //set the torque for the motor
  canMsg.can_id  = canIDs[idIndex];
  canMsg.can_dlc = 0x08;
  canMsg.data[0] = 0xA1;
  canMsg.data[1] = 0x00; 
  canMsg.data[2] = 0x00;
  canMsg.data[3] = 0x00;
  canMsg.data[4] = (uint8_t) torque;
  canMsg.data[5] = (uint8_t) (torque >> 8);
  canMsg.data[6] = 0x00;
  canMsg.data[7] = 0x00;
  mcp2515.sendMessage(&canMsg);

  // wait for position to move
  delay(25);

  bool first = true;
  double currAngle = lastAngle;
  while (fabs(currAngle - lastAngle) > 3 || first) {
    if (first) first = false;

    lastAngle = currAngle;

    canMsg.can_id  = canIDs[idIndex];           // set CAN id
    canMsg.can_dlc = 0x08;
    canMsg.data[0] = 0x92;
    canMsg.data[1] = 0x00; 
    canMsg.data[2] = 0x00;
    canMsg.data[3] = 0x00;
    canMsg.data[4] = 0x00;
    canMsg.data[5] = 0x00;
    canMsg.data[6] = 0x00;
    canMsg.data[7] = 0x00;

    mcp2515.sendMessage(&canMsg);
    //receive message
    int len = WAIT_TIME;
    while ((mcp2515.readMessage(&canMsg) != MCP2515::ERROR_OK)){
      /* check received response CAN */
      delay(1);
      len--;
      if ((len <= 0)) 
      {
        break;
      }
    }
    //creating a 32bit integer value from the contents of the motor CAN reply message 
    int32_t position = (canMsg.data[7] << 24) | (canMsg.data[6] << 16) | (canMsg.data[5] << 8) | (canMsg.data[4]);
    // Convert the position to degrees, considering the 0.01°/LSB unit
    currAngle =  position * 0.01;
    
    delay(10);
  }

  setZeroMotor(idIndex);

}

void zeroMotors(){
  readMotorPositions();
  int16_t zeroMotorTorques[12] = {25, 25, -55, -25, -25, 55, 25, 25, -25, -18, -18, 25};
  // zero the knees -- 2, 5, 8, 11
  int knees[4] = {2, 5, 8, 11};
  for (int i = 0; i < 4; i++) {
    int index = knees[i];
    zeroMotor(index, zeroMotorTorques[index]);
  }

  delay(2000);
  int hips[4] = {0, 3, 6, 9};
  for (int i = 0; i < 4; i++) {
    int index = hips[i];
    zeroMotor(index, zeroMotorTorques[index]);
  }

  delay(2000);
  int shoulders[4] = {1, 4, 7, 10};
  for (int i = 0; i < 4; i++) {
    int index = shoulders[i];
    zeroMotor(index, zeroMotorTorques[index]);
  }
}

void readMotorErrors() {
  for(int i=0; i<12; i++){
    canMsg.can_id  = canIDs[i];           // set CAN id
    canMsg.can_dlc = 0x08;
    canMsg.data[0] = 0x9A;
    canMsg.data[1] = 0x00; 
    canMsg.data[2] = 0x00;
    canMsg.data[3] = 0x00;
    canMsg.data[4] = 0x00;
    canMsg.data[5] = 0x00;
    canMsg.data[6] = 0x00;
    canMsg.data[7] = 0x00;
    mcp2515.sendMessage(&canMsg);
    //receive message
    int len = WAIT_TIME;
    uint16_t error = 0;
    while ((mcp2515.readMessage(&canMsg) != MCP2515::ERROR_OK)){
      /* check received response CAN */
      delay(1);
      len--;
      if (len <= 0) 
      {
        error = 0x0200;
        break;
      }
    }
    //creating a 32bit integer value from the contents of the motor CAN reply message 
    if (error == 0) {
      error = canMsg.data[6] || canMsg.data[7] << 8;
    }
    
    errorMsgs[i] = error;
    
  }
}

void readMotorPositions(){
  for(int i=0; i<12; i++){
    canMsg.can_id  = canIDs[i];           // set CAN id
    canMsg.can_dlc = 0x08;
    canMsg.data[0] = 0x92;
    canMsg.data[1] = 0x00; 
    canMsg.data[2] = 0x00;
    canMsg.data[3] = 0x00;
    canMsg.data[4] = 0x00;
    canMsg.data[5] = 0x00;
    canMsg.data[6] = 0x00;
    canMsg.data[7] = 0x00;
    mcp2515.sendMessage(&canMsg);
    //receive message
    int len = WAIT_TIME;
    bool toUpdate = true;
    while ((mcp2515.readMessage(&canMsg) != MCP2515::ERROR_OK)){
      /* check received response CAN */
      delay(1);
      len--;
      if ((len <= 0)) 
      {
        /** Serial.print("\nNO RESPONSE FROM: ");
        Serial.print(i);
        Serial.print("\n"); **/
        toUpdate = false;
        break;
      }
    }

    // set the ID to the received ID
    int id = canMsg.can_id - 0x241;
    //check message was received
    if (toUpdate && id == i) {
      //creating a 32bit integer value from the contents of the motor CAN reply message 
      int32_t position = (canMsg.data[7] << 24) | (canMsg.data[6] << 16) | (canMsg.data[5] << 8) | (canMsg.data[4]);
      // Convert the position to degrees, considering the 0.01°/LSB unit
      currMotorVals[i] =  position * 0.01;
    }
    
  }
}

void move(int i, uint32_t moveTo, int moveVel){
  int moveActual = moveTo*100;   //adjusts for 36000 resolution
  canMsg.can_id  = canIDs[i];           // set CAN id, stored in canIDs[], passed through in moveMotors()
  canMsg.can_dlc = 0x08;
  canMsg.data[0] = 0xA4;
  canMsg.data[1] = 0x00;
  canMsg.data[2] = (uint8_t)(moveVel);
  canMsg.data[3] = (uint8_t)(moveVel >> 8);
  canMsg.data[4] = (uint8_t)(moveActual);
  canMsg.data[5] = (uint8_t)(moveActual >> 8);
  canMsg.data[6] = (uint8_t)(moveActual >> 16);
  canMsg.data[7] = (uint8_t)(moveActual >> 24);
  mcp2515.sendMessage(&canMsg);

  int len = WAIT_TIME;
  while ((mcp2515.readMessage(&canMsg) != MCP2515::ERROR_OK)){
  /* check received response CAN */
  delay(1);
  len--;
  if ((len <= 0)) 
  {
    Serial.print("\nNO RESPONSE FROM: ");
    Serial.print(i);
  }
}


}

String updatePi(char mode){
  String msg;
  //position response
  if(mode == 'p'){
    for (int i = 0; i<12; i++){
    msg = msg+String(currMotorVals[i], 2); //populates with current motor positions
    msg = msg + " "; //adds spacing 
    }
  }
  //torque response
  if(mode == 't'){
    for (int i = 0; i<12; i++){
    msg = msg+String(currMotorTorques[i], 2); //populates with current motor torques
    msg = msg + " "; //adds spacing 
    }
  }
  //CAN id response
  if(mode == 'i'){
    for (int i = 0; i<12; i++){
    msg = msg+String(currMotorTorques[i], 2); 
    msg = msg + " "; //adds spacing 
    }
  }
  return msg;
}

void clearDisp(){
  String clear = "                ";
  //lcd.setCursor(0,0);
  //lcd.send_string(clear.c_str());
  lcd.setCursor(0,1);
  lcd.send_string(clear.c_str());
}

void display(){
  clearDisp();
  //printing first line
  lcd.setCursor(0,0);
  lcd.send_string(disp1.c_str());

  //printing second line
  lcd.setCursor(0,1);
  lcd.send_string(disp2.c_str());
}
