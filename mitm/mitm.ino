/*
*****grbl "Man in the Middle" Jog Controller Pendant*****

WARNING:  I suck at c and c++.  It is a miracle this even works.  Use at your own risk!

  Port to stm32f103 by Jesse Schoch

// Modes notes


Jog Modes

mode 1 is jog queue mode.  In this mode the encoders will be set to a value and await a "go" button press.
mode 2 is realtime jog mode.  this mode will jog and wait for "ok" status from the planner. subsequent encoder turns will queue the next jog.  A stop of encoder movement inside some debounce window will issue a jog stop.

setup/probe modes

mode 3 is setup modes.  this mode is used to set virtual stops

Cutting modes


mode 4 is facing mode.  encoder turns will start a "pass", "stepover" or "plunge/ramp"
mode 5 will be slotting mode
mode 6 "yo yo" step mode.  Back and forth on one axis and stepover on the other.
  likely do first move is yo yo move, 2nd is stepover.
  could add diameter of tool and auto calculate stepover based on sfm or something.

*/

// TODO: in pass mode use wheels  to adjust feeds 

#include "HardwareTimer.h"
#include "mitm.h"
#include <string>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <neotimer.h>
#include "gstate.h"
#include "pos.h"
#include <Bounce2.h>
#include "grbl.h"

// Vars

bool newResp = false;
bool newPush = false;
bool newMsg = false;
bool serial_dbg = true;
bool halted = false;
bool disp_tick = false;

// wait for buffer to clear
bool okWait = false;

// flag for passive mode;
bool pass = false;

// buffer control

// CMD_B is the counter for outstanding commands
uint8_t CMD_B = 0;
const uint8_t CMD_MAX = 9;

// serial vars
// globals for parsing Serial2 (grbl) command responses
const byte numChars = 255;
char cmd[numChars];   // an array to store the received data
static byte ndx = 0;
char endMarker = '\n';
char rc;

// btns

uint8_t btn1 = PB5;
uint8_t btn2 = PB4;
uint8_t btn3 = PB3;
uint8_t btn4 = PA15;
uint8_t btn5 = PB12;


const int NUM_BUTTONS = 5;
const uint8_t BUTTON_PINS[NUM_BUTTONS] = {btn1,btn2,btn3,btn4,btn5};
Bounce * buttons = new Bounce[NUM_BUTTONS];

std::string current_mode = "Startup";

uint8_t stepCnt = 1;

// inc_mode moves one stepSize.  inc_mode false activates acceleration mode.
// acceleration mode attempts to queue moves and to stop motion when the velocity of the wheel is zero.
bool inc_mode = true;

// update timer thing

unsigned long lastUpdate = 0;

// _gs holds x,y,z coordinates

Gstate _gs;

// holds last position, not used to compare right now

PosSet oldPos = {Pos(),Pos(),false};

// timer for updating screen

Neotimer mytimer = Neotimer(100);

// timer for getting status

Neotimer lasttimer = Neotimer(100);


// display stuff
#define SSD1306_128_64
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

// Serial rates


unsigned long rate1 = 115200;
unsigned long rate2 = 115200;

// TODO:  consider extended ascii, will it be ignored by the sender?
const char* prefixor = "P <<<";

bool idle = true;
bool waiting = false;


// TODO: consider per axis feeds and adjustments with eeprom save
int feedXY = 1000;
int feedZ = 1000;
int feed = 1000;
int rapid = 2000;
int rapidXY = 2000;
float stepSize = 0.01;

//  STM32 encoder stuff

//
// PA8/PA9 timer_1
// PA0/PA1 timer_2
// PB6/PB7 timer_3
//


int PPR = 2;

HardwareTimer timer_1(1);

Axis Xaxis("X",timer_1);


void func_timer_1(){
   if(timer_1.getDirection()){
      Xaxis.forward = true;
      Xaxis.decrPos();
  }else{
      Xaxis.forward = false;
      Xaxis.incrPos();
  }    
}

// uses internal timer 2 on PA0 and PA1
HardwareTimer timer_2(2);

Axis Zaxis("Z",timer_2);

void func_timer_2(){
   if(timer_2.getDirection()){
      Zaxis.forward = true;
      Zaxis.decrPos();
  }else{
      Xaxis.forward = false;
      Zaxis.incrPos();
  }    
}



// use internal timer 3 on PA 6/7

HardwareTimer timer_3(3);

volatile long ypos = 0;
volatile long yold_pos = 2;
Axis Yaxis("Y", timer_3);

void func_timer_3(){
   if(timer_3.getDirection()){
      Yaxis.forward = true;
      Yaxis.decrPos();
  }else{
      Yaxis.forward = false;
      Yaxis.incrPos();
  }    
}

// uses internal timer 4 on PB6 and PB7
//  not used

// velocity

long x_newposition;
long x_oldposition = 0;
unsigned long x_newtime;
unsigned long x_oldtime = 0;
long x_vel;


/////////////////////////////////////////// Functions //////////////////////////////////////////////////


// encoder velocity

void calculate_velocity(){
  if(!pass){
    Xaxis.velocity();
    Yaxis.velocity();
    Zaxis.velocity();
  }
}

// configures hardwaretimers
void config_timer(int channel, HardwareTimer this_timer, void (*func)()){
  this_timer.pause(); //stop... 

  // setMode(channel, mode)  mode 0 was crashing but it was not supposed to be used!?
  this_timer.setMode(channel, TIMER_ENCODER); //set mode, the channel is not used when in this mode.
  this_timer.setPrescaleFactor(1); //normal for encoder to have the lowest or no prescaler. 
  this_timer.setOverflow(PPR);    //use this to match the number of pulse per revolution of the encoder. Most industrial use 1024 single channel steps. 
  this_timer.setCount(0);          //reset the counter. 
  this_timer.setEdgeCounting(TIMER_SMCR_SMS_ENCODER3); //or TIMER_SMCR_SMS_ENCODER1 or TIMER_SMCR_SMS_ENCODER2. This uses both channels to count and ascertain direction. 
  this_timer.attachInterrupt(0, func); //channel doesn't mean much here either.  
  this_timer.resume();
}

// reads a full message ending with \n
void readLine(){
  while (Serial2.available() > 0 && newMsg == false) {
        rc = Serial2.read();

        if (rc != endMarker) {
            cmd[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            cmd[ndx] = '\0'; // terminate the string
            ndx = 0;
  
            newMsg = true;
        }
    }
}

void dbg(char* s){
  if(serial_dbg){
    Serial.print(s);
  }
}
void dbgln(char* s){
  if(serial_dbg){
    Serial.println(s);
  }
}

// process an 'ok' or 'error'
void processResp(){
  // look for error
  if(strcasestr(cmd, "error") != NULL){
     dbg("Error found");
      dbgln(cmd);
     }
   // must be an 'ok'
   else if(strcasestr(cmd,"ok") != NULL){
      if(CMD_B == 0){
        CMD_B = 0; 
        okWait = false;
      }else{
        CMD_B--;
      }
   }
   else{
     dbgln("Nothing matched");
     dbgln(cmd); 
     //halt("Unknown Cmd");
   }

  if(serial_dbg){
    Serial.print("-----> ");
    Serial.print(cmd);
    Serial.print("\n");
  }
}

// process msg starting with [ or <
void processPush(){
  if(serial_dbg){
    Serial.print(cmd);
    Serial.print("\n");
  }
  
}

// this should halt everything and wait to confirm the jog has been canceled.
void cancelJog(){
  okWait = true;
  // TODO; update with header def
  Serial2.flush();
  Serial2.write(0x85);
  Serial2.flush();
  CMD_B = 1;
}

void updateStepSize(){
  if(stepCnt <= 1){
    stepCnt = 1;
    stepSize = 0.01;
  }else{
    if(stepCnt == 2){
      stepSize = 0.1;
    }
    else if(stepCnt == 3){
      stepSize = 0.5;
    }
    else if(stepCnt == 4){
      stepSize = 1.0;
    }
    else if(stepCnt == 5){
      stepSize = 5.0;
    }
    else if(stepCnt == 6){
      stepSize = 10.0;
    }
    else{
      stepSize = 10.0;
      stepCnt = 1;
    }
  }
}


// checks the buffer
bool bufferFull(){
  if(CMD_B < CMD_MAX){
    return false;
  }else{
    // TODO:  debug to print out buffered commands
    return true;
  }
}

void checkJogInputs(){
  //  mock for encoder handling
  checkBtns();
  if(buttons[3].rose()){
    batchJog("$J=G91 ", Xaxis);
    batchJog("$J=G91 ", Xaxis);
    batchJog("$J=G91 ", Xaxis);
    batchJog("$J=G91 ", Xaxis);
    batchJog("$J=G91 ", Xaxis);
    batchJog("$J=G91 ", Xaxis);
    batchJog("$J=G91 ", Xaxis);
    _gs.d_status = "batch";
  }
}

void checkBtns(){
  for (int i = 0; i < NUM_BUTTONS; i++)  {
    // Update the Bounce instance :
    buttons[i].update();
    }
}

void checkCtrlInputs(){
  // check buttons
  checkBtns();

  // checks buttons and any other non-jog inputs
  if(buttons[0].read()){
    current_mode = "Jog Mode";
    pass = false;
  }else{
    current_mode = "Passive";
    pass = true;
  }
  
  if(buttons[1].rose()){
    stepCnt++;
  }
  

  if(buttons[2].rose()){
      inc_mode = !inc_mode;
  }

  /*
  if(buttons[3].rose()){
    //  TODO: using this button to mock a jog command, revert to $X
     //  send unlock
     Serial2.println("$X");
    //
  }
  */
  
  if(buttons[4].rose()){
     //  TODO:  add stop jog command here
     cancelJog();
  }
  updateStepSize();
}

// process messages from grbl
void parseMsg(){
  if(newMsg){
     // match push
     if(cmd[0] == '[' || cmd[0] == '<'){
      processPush();
     }
     // match resp
     else {
      processResp();
     }

     newMsg = false;
  }else{
    // blink or something
  }
}

// Process any inputs to jog and ensure the commands are completing 
void loopJog(){
  parseMsg();

  // Ensure no new commands are issued while waiting for buffer to clear.
  if(okWait && CMD_B > 0){
    // TODO:  update status msg here.
    _gs.d_status = "S:okWait";
  }
  //
  else{
    if(bufferFull()){
      _gs.d_status = "buf full"; 
    }else{
      checkJogInputs();
      _gs.d_status = "check jog";
    }
  }
}

// watch for position updates and update gui
void loopPass(){
  if(newMsg){
    Serial.print(cmd);
    Serial.print("\n");
    newMsg = false;
  }
  
}

void halt(char * msg){
  halted = true;
  // TODO: update display;
}

void batchJog(const char* start, Axis axis){
  doJog(start,axis);
  CMD_B++;
}

void waitJog(const char* start, Axis axis){
  okWait = true;
  doJog(start,axis);
  CMD_B++;
}

void doJog(const char* start, Axis axis){

  Serial2.print(start);
  Serial2.print(axis.axis_name);
  if(!axis.forward){
    Serial2.print("-");
  }
  Serial2.print((stepSize));
  Serial2.print("F");
  Serial2.println(feed);
  if(serial_dbg){
    Serial.print(CMD_B);
    Serial.println(" Jog");
  }
}

/////////////////////////////////////// DISPLAY /////////////////////////////////////////////////////

void drawCMD(std::string &c){
  display.setTextColor(WHITE, BLACK);
  display.setCursor(0,1);
  display.print(disp_tick);
  disp_tick = !disp_tick;
  display.print("-");
  display.print(CMD_B);
  display.print(c.c_str());
  display.print(" okWait: ");
  display.print(okWait);
  display.setCursor(0,0);
}

void drawStatus(std::string &c){
  display.setTextColor(WHITE, BLACK);
  display.setCursor(90,45);
  display.print(c.c_str());
}

void drawPOS(float x, float y, float z){
  display.setTextColor(WHITE, BLACK);
  display.setCursor(0,15);
  if(Xaxis.running){
    display.print("-");
  }
  display.print("X: ");
  display.println(oldPos.mpos.x);
  display.print("Y: ");
  display.println(oldPos.mpos.y);
  display.print("Z: ");
  display.println(oldPos.mpos.z);
}

void drawMode(){

  display.setTextColor(WHITE,BLACK);
  display.setCursor(0,50);
  if(inc_mode){
    display.print("i");
  }else{
    display.print("a");
  }
  display.print(current_mode.c_str());
}

void drawStep(){
  display.setTextColor(WHITE,BLACK);
  display.setCursor(85,25);
  display.print("S:");
  display.print(stepSize);
  
}

void drawDisplay(){
  if(mytimer.repeat()){
    display.clearDisplay();
    //display.print(".");
    drawCMD(_gs.d_cmd);
    drawStatus(_gs.d_status);
    drawPOS(0.0,0.0,0.0);
    drawMode();
    drawStep();
    display.display();
  }
}





//////////////////////////////////////// SETUP and LOOP //////////////////////////////////////////////



void setup() {
  // Timer pins
  pinMode(PC13,OUTPUT);
  pinMode(PA0, INPUT_PULLUP);
  pinMode(PA1, INPUT_PULLUP);
  pinMode(PA6, INPUT_PULLUP);
  pinMode(PA7, INPUT_PULLUP);
  //pinMode(PB6, INPUT_PULLUP);
  //pinMode(PB7, INPUT_PULLUP);
  pinMode(PB0, INPUT_PULLUP);
  pinMode(PB1, INPUT_PULLUP);

  pinMode(PA8, INPUT_PULLUP);
  pinMode(PA9, INPUT_PULLUP);

  // buttons

  for (int i = 0; i < NUM_BUTTONS; i++) {
    buttons[i].attach( BUTTON_PINS[i] , INPUT_PULLUP  );       //setup the bounce instance for the current button
    buttons[i].interval(50);              // interval in ms
  }


  // Serial

  Serial.begin(rate1);


  // Display stuff

  display.begin(SSD1306_SWITCHCAPVCC, 0x3c); 
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("Setup: ");
  display.display();

  display.println("Serial connected... connecting to grbl...");
  // set the data rate for the SoftwareSerial port

  // Using serial pins PA2/PA3
  Serial2.begin(rate1);

  display.println("Ready to configure timer");

  config_timer(1, timer_1,func_timer_1);
  config_timer(1, timer_2,func_timer_2);
  config_timer(1, timer_3,func_timer_3);
  display.display();

  // setup axis objects

  Yaxis.begin("Y", 1000, 0.01f);
  Zaxis.begin("Z", 1000, 0.01f);
  Xaxis.begin("X", 1000, 0.01f);


 
  display.println("Timer configured");
  display.println("Running setup");

  // TODO: get config when we can parse it
  //Serial2.println("$$");

  // TODO: here is set MM mode, how to deal with these preferences?
  Serial2.println("G21");
  display.display();

  if (serial_dbg){
    Serial.println("setup done");
  }

}

void loop() {
  
  // get stuff from serial and assemble messages from grbl.
  readLine();

  checkCtrlInputs();

  if (Serial.available()) {
    Serial2.write(Serial.read());
  }
  
  // passive mode passes messages through from GUI.
  // passive mode ignores commands but updates position data 
  if(pass){
    loopPass();
  }else{
    // Jog mode tracks commands and responses
    loopJog();
  }

  // 

  drawDisplay();

  // pos updater

  if(lasttimer.repeat() ){
    calculate_velocity();
    
    // TODO:  should only ask for status if the sender isn't polling
    if((millis() - lastUpdate) > 1000 || !pass || !okWait){
      Serial2.println("?");
      CMD_B++;
      lastUpdate = millis();
    }
    
    digitalWrite(PC13, (!digitalRead(PC13)));
    
  }

  // TODO: consider key combo to unhalt
  while(halted){
    dbgln("HALT");
    delay(1000);
  }
}
