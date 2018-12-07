/*
*****grbl "Man in the Middle" Jog Controller Pendant*****

WARNING:  I suck at c and c++.  It is a miracle this even works.  Use at your own risk!

  Port to stm32f103 by Jesse Schoch

// Modes notes


#Jog Modes

mode 1 is jog queue mode.  In this mode the encoders will be set to a value and await a "go" button press.
mode 2 is realtime jog mode.  

##Incremental Mode 

Single stepSize increments.  Wheels will not take additional input until the target position has been reached and the grbl state is idle

##Acceleration Mode

this mode will issue jog commands while the encoder wheel keeps turning and the jog buffer isn't full.  The stepSize will increase with wheel velocity

#setup/probe modes

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
#include "myenums.h"
//#include "./gbbb.h"
#include <grbl_chat.h>

// Vars


const char* stateNames[] = {
  "Unknown",
    "Idle",
    "Run",
    "Jog",
    "Hold:0",
    "Hold:1",
    "Alarm",
    "Check",
    "Door:0",
    "Door:1",
    "Door:2",
    "Door:3",
    "Tool"
};

//grbl_data_t *grbl_data;

uint8_t mstate = Setup;
uint8_t old_mstate = Setup;
uint8_t last_mstate = Setup;
uint8_t old_grbl_state = Unknown;

bool newResp = false;
bool newPush = false;
bool newMsg = false;
bool serial_dbg = true;
bool serial_dbg2 = true;
bool halted = false;
bool disp_tick = false;
uint8_t screen_tick = 0;

const char* halt_msg;

// wait for buffer to clear
bool okWait = false;
bool jogWait = false;

// flag for passive mode;
bool pass = false;

// buffer control

// CMD_B is the counter for outstanding commands
uint8_t CMD_B = 0;
const uint8_t CMD_MAX = 5;

// serial vars
// globals for parsing Serial2 (grbl) command responses
const unsigned int numChars = 255;
char cmd[numChars];   // an array to store the received data
//static byte ndx = 0;
static unsigned int ndx = 0;
char endMarker = '\n';
char rc;
char readch;

// btns

uint8_t btn1 = PB15;
uint8_t btn2 = PB4;
uint8_t btn3 = PA4;
uint8_t btn4 = PA15;
uint8_t btn5 = PB12;


const int NUM_BUTTONS = 5;
const uint8_t BUTTON_PINS[NUM_BUTTONS] = {btn1,btn2,btn3,btn4,btn5};
Bounce * buttons = new Bounce[NUM_BUTTONS];

std::string current_mode = "Startup";

uint8_t stepCnt = 3;

// inc_mode moves one stepSize.  inc_mode false activates acceleration mode.
// acceleration mode attempts to queue moves and to stop motion when the velocity of the wheel is zero.
bool inc_mode = true;

// update timer thing

unsigned long lastUpdate = 0;

// _gs holds x,y,z coordinates

Gstate _gs;

// holds last position, not used to compare right now

PosSet oldPos = {Pos(),Pos(),false};

Neotimer btntimer = Neotimer(35);

// timer for updating screen

Neotimer displaytimer = Neotimer(75);

Neotimer incjogstarttimeout = Neotimer(500);

Neotimer mytimer = Neotimer(75);


// timer for getting updates via "?"

Neotimer updatetimer = Neotimer(25);


// timer for getting status

Neotimer lasttimer = Neotimer(50);


// display stuff

/* no need to change i2c2 now
// change to i2c2

//#include <libmaple/i2c.h>

//i2c_master_enable(i2c1,I2C_REMAP);

//HardWire Wire(1,I2C_REMAP);
*/

/*
TwoWire Wire2(PB11, PB10);

#define Wire Wire2
*/

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
float stepSize = 10.01;

//  STM32 encoder stuff

//
// PA8/PA9 timer_1
// PA0/PA1 timer_2
// PB6/PB7 timer_3
//


int PPR = 4;

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
      Zaxis.forward = false;
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
  if(!pass && !inc_mode){
    Xaxis.velocity();
    Yaxis.velocity();
    Zaxis.velocity();
  }
}

// configures hardwaretimers
void config_timer(int channel, HardwareTimer this_timer, void (*func)()){
  this_timer.pause(); //stop... 

  // setMode(channel, mode)  mode 0 was crashing but it was not supposed to be used!?
  this_timer.setMode(channel, TIMER_ENCODER); //set mode, the channel is 1..4 but the docs say not to require it
  this_timer.setPrescaleFactor(1); //normal for encoder to have the lowest or no prescaler. 
  this_timer.setOverflow(PPR);    //use this to match the number of pulse per revolution of the encoder. Most industrial use 1024 single channel steps. 
  this_timer.setCount(0);          //reset the counter. 
  this_timer.setEdgeCounting(TIMER_SMCR_SMS_ENCODER3); //or TIMER_SMCR_SMS_ENCODER1 or TIMER_SMCR_SMS_ENCODER2. This uses both channels to count and ascertain direction. 
  //this_timer.setEdgeCounting(TIMER_SMCR_SMS_ENCODER2);
  this_timer.refresh();
  this_timer.attachInterrupt(TIMER_UPDATE_INTERRUPT, func); //first arg ends up as a timer_interrupt_id, see timer.h
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
  
            //newMsg = true;


            parseData(cmd);
            //grbl_data_t *grbl_data = getData();
            //mstate = 
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



void inc_check_encoders(){
  inc_check_axis(Yaxis);
  inc_check_axis(Xaxis);
  inc_check_axis(Zaxis);
}

void inc_reset_pos(){
  Yaxis.resetPos();
  Zaxis.resetPos();
  Xaxis.resetPos();
}

void inc_check_axis(Axis &axis){
  if(axis.moved() && mstate == IncModeWait )
    {
    waitJog(axis);
    old_mstate = mstate;
    mstate = IncJogStart;
    incjogstarttimeout.start();
    axis.resetPos();
  }
}

void checkBtns(){
  for (int i = 0; i < NUM_BUTTONS; i++)  {
    // Update the Bounce instance :
    buttons[i].update();
    }

   if(buttons[PASS_TOGGLE].read()){
    pass = false;
    if (inc_mode){
      current_mode = "Step Jog ";
      }else{
      current_mode = "Speed Jog";
    }

  }else{
    pass = true;
  }
  
  if(buttons[STEP_INC].rose()){
    stepCnt++;
    updateStepSize();
  }


  if(buttons[INC_TOGGLE].rose()){
      inc_mode = !inc_mode;
      if(inc_mode){

      }else{
        // reset states
        CMD_B = 0;
        okWait = 0;
      }
  }

  if(buttons[CLEAR_ALARM].rose()){
      //TODO: using this button to mock a jog command, revert to $X
    //  send unlock
     Serial2.println("$X");

  }

  if(buttons[JOG_CANCEL].rose()){
     //  TODO:  add stop jog command here
     //cancelJog();
  }
}

/*
void checkCtrlInputs(){
  // check buttons
  checkBtns();

  // checks buttons and any other non-jog inputs
  if(buttons[PASS_TOGGLE].read()){
    pass = false;
    if (inc_mode){
      current_mode = "Step Jog ";
      }else{
      current_mode = "Speed Jog"; 
    }
    
  }else{
    pass = true;
  }
  
  if(buttons[STEP_INC].rose()){
    stepCnt++;
  }
  

  if(buttons[INC_TOGGLE].rose()){
      inc_mode = !inc_mode;
      if(inc_mode){
        
      }else{
        // reset states
        CMD_B = 0; 
        okWait = 0;
      }
  }

  if(buttons[CLEAR_ALARM].rose()){
      //TODO: using this button to mock a jog command, revert to $X
    //  send unlock
     Serial2.println("$X");
    
  }
  
  if(buttons[JOG_CANCEL].rose()){
     //  TODO:  add stop jog command here
     //cancelJog();
  }
  updateStepSize();
}

*/

// process messages from grbl
void parseMsg(){
  if(newMsg){
     if(cmd[0] == 'o' && cmd[1] == 'k'){ // == "ok"
        //processResp();
     } else{
      parseData(cmd);
      }
     newMsg = false;
     if(serial_dbg){
       Serial.print(cmd);
       Serial.println("");
       } 
     //checkJogIdle();
  }else{
    // blink or something
  }
}



void halt(char * msg){
  halted = true;
  mstate = Halt;
  // TODO: update display;
}

void batchJog(Axis axis){
  batchJog("$J=G91 ",axis);
}

void batchJog(const char* start, Axis axis){
  if(!bufferFull()){
    doJog(start,axis);
    CMD_B++;
  }
}
  

// jogs axis one stepSize
void waitJog(Axis axis){
  waitJog("$J=G91 ", axis);
}

void requestUpdate(){
  Serial2.print("?");
}

void waitJog(const char* start, Axis axis){
  if(!bufferFull() && !jogWait){
    doJog(start,axis);
    CMD_B++;
    requestUpdate();
  }
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


  // 
  if(serial_dbg2){
    Serial.print(axis.getOldPos());
    Serial.print(",");
    Serial.println(axis.getPos());
  }
  if(serial_dbg){
    Serial.print(okWait);
    Serial.print(",");
    Serial.print(jogWait);
    Serial.print(":");
    Serial.print(axis.getOldPos());
    Serial.print(",");
    Serial.print(axis.getPos());
    Serial.print(",");
    Serial.print(CMD_B);
    Serial.println(" Jog");
  }
}

/////////////////////////////////////// DISPLAY /////////////////////////////////////////////////////

void drawCMD(std::string &c){
  display.setTextColor(WHITE, BLACK);
  display.setCursor(51,1);

  //display.print(disp_tick);
  //disp_tick = !disp_tick;

  display.print("-");
  display.print(CMD_B);
  display.print(c.c_str());
  display.print(" : ");

  grbl_data_t *grbl_data = getData();
  //display.print(gD->grbl.state);
  display.print(stateNames[grbl_data->grbl.state]);
  //Serial.println(stateNames[grbl_data->grbl.state]);

  //display.print(" okWait: ");
  //display.print(okWait);
  display.setCursor(0,0);
}

int tick_line(){
  screen_tick++;
  if(screen_tick >= 128){
    screen_tick = 0;
  }
  return screen_tick;
}

// draws _gs.d_cmd
void drawStatus(std::string &c){
  display.drawFastHLine(tick_line(), 50,30, WHITE);
  display.setTextColor(WHITE, BLACK);
  display.setCursor(0,42);
  display.print(c.c_str());
  //display.print();
}

void drawPOS(){
  grbl_data_t *grbl_data = getData();
  display.setTextColor(WHITE, BLACK);
  display.setCursor(0,16);
  if(Xaxis.running){
    display.print("-");
  }
  display.print("X: ");
  //display.println(oldPos.mpos.x);
  display.println(grbl_data->position[X_AXIS]);
  display.print("Y: ");
  //display.println(oldPos.mpos.y);
  display.println(grbl_data->position[Y_AXIS]);
  display.print("Z: ");
  //display.println(oldPos.mpos.z);
  display.println(grbl_data->position[Z_AXIS]);
}

void drawMode(){

  display.setTextColor(WHITE,BLACK);
  display.setCursor(0,0);
  display.print(mstate);
  display.print("#");
  display.print(current_mode.c_str());
  //display.print(" | ");
  //display.print(oldPos.lastState);
}

void drawStep(){

  // prints the jog step size on the right
  display.setTextColor(WHITE,BLACK);
  display.setCursor(85,25);
  display.print("S:");
  display.print(stepSize);


  /// Print waiting status and the buffer size right of the position
  ///  < okWait > : < buffer size > 
  display.setCursor(85,33);
  display.print(okWait);
  
  display.print(":");
  
  display.print(CMD_B);
  display.print(":");

  display.print(jogWait);
  
  
}

void drawDisplay(){
  if(displaytimer.repeat()){
    display.clearDisplay();
    drawCMD(_gs.d_cmd);
    drawStatus(_gs.d_status);
    drawPOS();
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

  //grbl_data_t * grlb_data = getData(); 
  //delay(15000);
  mstate = SetupDone;
} // end setup


/////////////////////////////////////////////////////// Loop

void loop() {
  
  // get stuff from serial and assemble messages from grbl.
  readLine();

  if (Serial.available()) {
    Serial2.write(Serial.read());
  }
  
  drawDisplay();

  if(updatetimer.repeat() && mstate != AccelModeRun){
    requestUpdate(); 
  }




  if(btntimer.repeat()){
    checkBtns();
  }
  grbl_data_t *grbl_data = getData();
  switch(mstate){
      case SetupDone:
        if(pass){
          mstate = Passive;
        }else{
          mstate = IncModeWait;
        }  
        break;
      case Passive:
        if(pass == false){
          old_mstate = mstate;
          mstate = IncModeWait;
        }
        break;
      case IncModeWait:
        if(pass){
          old_mstate = mstate;
          mstate = Passive;
        }else{
          // check encoders, if they move they will transition to IncJogStart
          inc_check_encoders();
        }
        break;
      /* 
      case AccelModeWait:
        break;
      case AccelModeTurning: 
        break;
      case AccelModeStop:
        break;
      */
      case IncJogStart: 
        if(grbl_data->grbl.state == Jog){
          old_mstate = mstate;
          mstate = IncJogRun;
        }else{
          _gs.d_status = "Waiting for Jog Start";
        }
        if (incjogstarttimeout.done()){
          dbgln("IncJogStart timedout!");
          old_mstate = mstate;
          mstate = IncModeWait;
        }
        break;
      case IncJogRun:
        if(grbl_data->grbl.state == Idle){
          old_mstate = mstate;
          mstate = IncModeWait;
          CMD_B--;
          // capture current position on encoders.  this is used to detect a move.
          inc_reset_pos();
        }else{
          _gs.d_status = "Waiting for Jog Stop";
        }
        break;
      case IncJogEnd:
        break;
      case Halt:
        break;

  
      default: 
        halt_msg = "Unknown state";
        halted = true;
        break;
    }

    if(grbl_data->grbl.state != old_grbl_state){
      Serial.print("New State: ");
      Serial.println(grbl_data->grbl.state);
      old_grbl_state = grbl_data->grbl.state;
    }

    if (last_mstate != mstate){
      if (serial_dbg){
        Serial.print(" New MState: ");
        Serial.print(old_mstate);
        Serial.print(",");
        Serial.println(mstate);
      }
      last_mstate = mstate;
    }
}
