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

#Cutting modes


mode 4 is facing mode.  encoder turns will start a "pass", "stepover" or "plunge/ramp"
mode 5 will be slotting mode
mode 6 "yo yo" step mode.  Back and forth on one axis and stepover on the other.
  likely do first move is yo yo move, 2nd is stepover.
  could add diameter of tool and auto calculate stepover based on sfm or something.

*/

// TODO: in pass mode use wheels  to adjust feeds 

#undef min
#undef max

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
#include <grbl_chat.h>
#include <EwmaT.h>

EwmaT <int> feedFilter(50, 50);
EwmaT <int> stepFilter(10,100);

// Vars


//grbl_data_t grbl_data;

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
bool gotAccelOk = false;
uint8_t screen_tick = 0;

const char* halt_msg;
char jog_string[20];


//float mpg_factor = 0.007;
float mpg_factor = 0.01;


// wait for buffer to clear
bool noOk = true;


// flag for passive mode;
bool pass = true;

// buffer control

// this is the grbl buffer max, for grblHAL it is 35, grbl is smaller
const uint8_t GBUFF_MAX = 35;
const uint8_t CMD_MAX = 20;

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

uint8_t btn1 = PB15; // PASS_TOGGLE
uint8_t btn2 = PB4; // STEP_INC
uint8_t btn3 = PA5; // INC_TOGGLE
uint8_t btn4 = PA15;// CLEAR_ALARM
uint8_t btn5 = PB12; // JOG_CANCEL


const int NUM_BUTTONS = 5;
const uint8_t BUTTON_PINS[NUM_BUTTONS] = {btn1,btn2,btn3,btn4,btn5};
Bounce * buttons = new Bounce[NUM_BUTTONS];

// this sets the stepSize using a button to step through the options
uint8_t stepCnt = 3;

// inc_mode moves one stepSize.  inc_mode false activates acceleration mode.
// acceleration mode attempts to queue moves and to stop motion when the velocity of the wheel is zero.
bool inc_mode = false;

// update timer thing

unsigned long lastUpdate = 0;

// _gs holds x,y,z coordinates

Gstate _gs;

// holds last position, not used to compare right now

PosSet oldPos = {Pos(),Pos(),false};

Neotimer btntimer = Neotimer(100);

// timer for updating screen

Neotimer displaytimer = Neotimer(150);

// timer for waiting for grbl to go back to IDLE state after CMD_JOG_CANCEL

Neotimer canceltimer = Neotimer(1200);

// ensures we recover from missing a jog state transition

//Neotimer incjogstarttimeout = Neotimer(50);

// allow for regular timing of velocity
//Neotimer veltimer = Neotimer(6);


// limit batch jogs by time
// lower makes less jerky
Neotimer acceltimer = Neotimer(10);


// timer for getting updates via "?"

Neotimer updatetimer = Neotimer(50);

// state check timer
Neotimer statetimer = Neotimer(100);


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


// store hundreths of a mm, this is converted back to mm in doJog
float defaultStepSize = 0.2;
float stepSize = 0.2;

//  STM32 encoder stuff

//
// PA8/PA9 timer_1   Xaxis
// PA0/PA1 timer_2   Zaxis
// PA6/PA7 timer_3   Yaxis
// PB6/PB7 TIMER_4 BUT
// PB6/PB7 also SDA!
//


int PPR = 1;

HardwareTimer timer_1(1);

Axis Xaxis("X",X_AXIS,timer_1);


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

Axis Zaxis("Z",Z_AXIS,timer_2);

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

Axis Yaxis("Y", Y_AXIS,timer_3);

void func_timer_3(){
   if(timer_3.getDirection()){
      Yaxis.forward = true;
      Yaxis.decrPos();
  }else{
      Yaxis.forward = false;
      Yaxis.incrPos();
  }    
  Yaxis.velocity();
}

// uses internal timer 4 on PB6 and PB7
//  not used



/////////////////////////////////////////// Functions //////////////////////////////////////////////////



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

        // TODO need to mute this when connected to the sender
        if(serial_dbg){

          Serial.write(rc);
          //Serial.flush();

        }
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
            parseData(cmd);
            if(mstate == Passive){
              Serial.print(cmd);
              Serial.print("\n");
            }
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


/*  TODO: do I need this?
void updateStepSize(){
  if(stepCnt <= 1){
    stepCnt = 1;
    stepSize = defaultStepSize;
  }else{
    if(stepCnt == 2){
      stepSize = stepSize * 2;
    }
    else if(stepCnt == 3){
      stepSize = stepSize * 2;
    }
    else if(stepCnt == 4){
      stepSize = stepSize * 2;
    }
    else if(stepCnt == 5){
      stepSize = stepSize * 2;
    }
    else if(stepCnt == 6){
      stepSize = stepSize * 2;
    }
    else{
      stepSize = 10;
      stepCnt = 1;
    }
  }
}
*/


// checks the buffer
bool bufferFull(){
  grbl_data_t *grbl_data = getData();
  if((GBUFF_MAX - grbl_data->buffer) <= CMD_MAX){
    //buffer_full = true;
    return false;
  }else{
    // TODO:  debug to print out buffered commands
    //buffer_full = false;
    return true;
  }
}

void sendCMD(char* s){
  Serial2.println(s);
  if(serial_dbg){
    Serial.println(s);
  }
  waitForOk();
}

void waitForOk(){
  _gs.d_status = "ok?";
  noOk = true;
  while(noOk){
    readLine();
    if(cmd[0] == 'o' && cmd[1] == 'k'){
      noOk = false;
    }else if(!strncmp(cmd,"error:", 6)){
      old_mstate = mstate;
      mstate = Halt;
      doJogCancelAll();
      noOk = false;
    }
  }
}



void inc_check_encoders(){
  inc_check_axis(Yaxis);
  inc_check_axis(Xaxis);
  inc_check_axis(Zaxis);
}



void reset_pos(){
  Yaxis.resetPos();
  Zaxis.resetPos();
  Xaxis.resetPos();
}

void inc_check_axis(Axis &axis){
  if(axis.moved() && (mstate == IncModeWait || mstate == IncJogRun))
    {
    int distance = abs( axis.old_pos - axis.pos );
    Serial.print("D: ");
    Serial.print(distance);
    Serial.print(" ");
    Serial.print(axis.rate);
    //axis.vel = (1/axis.rate) * 100000; 
    axis.vel = ((float)1/(float)axis.rate) * 100000UL;
    Serial.print(" - ");
    Serial.println(axis.vel);
    //Serial.println(vel);
    stepSize = 0.02 * distance;
    axis.setRunning();
    waitJog(axis);
    old_mstate = mstate;
    mstate = IncJogRun;
    axis.resetPos();
  }else if(axis.moved()){
    axis.resetPos();
  }
}

void accel_check_encoders(){
  accel_check_axis(Yaxis);
  accel_check_axis(Xaxis);
  accel_check_axis(Zaxis);
  calculate_velocity(); 
}

void calculate_velocity(){
  //if(veltimer.repeat()){
    doVelChecks(Xaxis);
    doVelChecks(Yaxis);
    doVelChecks(Zaxis);
  //}
}

void doJogCancelAll(){
  doJogCancel();
  Yaxis.notRunning();
  Zaxis.notRunning();
  Xaxis.notRunning();
}

void doJogCancel(){
  if(serial_dbg)
      {
      //Serial.print(axis.vel);
      Serial.println(" JOG_CANCEL");
    }
    Serial2.write(CMD_JOG_CANCEL);
    Serial2.flush();
}

void doVelChecks(Axis &axis){
  if(axis.vel < 1 && axis.running && mstate == AccelModeRun){
    old_mstate = mstate;
    mstate = AccelModeCancel;
    doJogCancel();
    _gs.d_status = "STOP!";
    axis.resetPos();
    axis.notRunning();
  }
  
}



void accel_check_axis(Axis &axis){
  
  doVelChecks(axis);
  if(axis.moved() && (mstate == AccelModeWait || mstate == AccelModeRun) && !bufferFull()  && acceltimer.repeat())
    {

    // create curve for step size
    float tmpStepSize = (axis.vel * axis.vel * axis.vel) * 0.00000005;
  
    if(tmpStepSize < 0.000001){
      axis.resetPos();
      return;
    }else{
      if(tmpStepSize < 2.0 ){
        stepSize = tmpStepSize;
      }else if(tmpStepSize < 0.01){
        stepSize = 0.01;
      }else{
        stepSize = 2;
      }
    }
    
    old_mstate = mstate;
    mstate = AccelModeRun;
    axis.setRunning();
    batchJog(axis);
    if(serial_dbg){
      Serial.print("StepSize: ");
      Serial.println(stepSize);
    }  
    axis.resetPos();
  }
  // ensure pos updated if we moved but the buffer was full or we were in the wrong mode
  else if(axis.moved()){
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
   }else{
     pass = true;
  }
  
  if(buttons[STEP_INC].rose()){
    stepCnt++;
    //updateStepSize();
  }


  if(buttons[INC_TOGGLE].rose()){
    inc_mode = !inc_mode;
  }

  if(buttons[CLEAR_ALARM].rose()){
      //TODO: using this button to mock a jog command, revert to $X
    //  send unlock
    if(serial_dbg)
      Serial.print("CLEAR_ALARM $X");
     Serial2.println("$X\n");

  }

  if(buttons[JOG_CANCEL].rose()){
     //  TODO:  add stop jog command here
     //cancelJog();
  }
}


void halt(char * msg){
  halted = true;
  mstate = Halt;
  // TODO: update display;
}


// queue up the jog commands and cancel when you the MPG acceleration goes to 0
void batchJog(Axis &axis){
  //batchJog("$J=G91 ",axis);
  batchJog("$J=G91 ",axis);
}

void batchJog(const char* start, Axis &axis){
  if(!bufferFull()){
    doJog(start,axis);
  }else{
    if(serial_dbg){
      Serial.println("Buffer full");
    }
  }
}
  

// jogs axis one stepSize, keep going unless velocity is 0.
// could wait for OK on each jog or just look at the buffer... not sure which is better
void waitJog(Axis &axis){
  waitJog("$J=G91 ", axis);
  //waitJog("G1 ",axis);
}

void requestUpdate(){
  Serial2.print("?");
}

void waitJog(const char* start, Axis &axis){
  if(!bufferFull()){
    doJog(start,axis);
  }
}


//  this should execute the jog for any jog type and make available the jog command sent for debugging
void doJog(const char* start, Axis &axis){

  float jog_pos = stepSize;

  // calculates the feed rate based on velocity
  int tmpFeed = feedFilter.filter((feed * mpg_factor) * axis.vel);
  axis.feed = tmpFeed;
  if(!axis.forward){
    jog_pos = jog_pos * -1;
  }


  if(jog_pos != 0 && axis.feed != 0){
    sprintf(jog_string, "%s%s%.2fF%d",start,axis.axis_name,jog_pos,axis.feed);
    sendCMD(jog_string);
  }
}

void smartJog(Axis &axis){
  /* terjeio's stuff
  
  // gets a snapshot of the current mpg position
  pos = MPG_GetPosition();

  // get distance between mpg position and the projected move position
  delta_z = (float)(pos->z.position - axis[Z_AXIS].mpg_position) * axis[Z_AXIS].mpg_factor / 400.0f;

  // update mpg_position to snapshot position
  axis[Z_AXIS].mpg_position = pos->z.position;

  // absDistance is a bool, maybe setting absolute mode moves? 
  if(grbl_data->absDistance)
    // mpg_base is reset in MPG_ResetPosition when grbl_dat->changed.mpg is true
    axis[Z_AXIS].mpg_base += delta_z;

  // velocity *50 seems to be used to set the feedrate
  velocity = pos->z.velocity;
  sprintf(append(buffer), "Z%.3f", grbl_data->absDistance ? axis[Z_AXIS].mpg_base - grbl_data->offset[Z_AXIS] : delta_z);


  formula:
  
  dt is estimated jog time
  v current jog rate
  N is the number of planner blocks N=15

  dt > v^2 / (2 * a * (N-1))

  T = dt * N  ; computes latency

  max_jog_rate is rate in mm/s/s

  a is the max acceleration along the jog vector
  estimated_jog_time = current_jog_rate / (2 * max_jog_rate_accel *10)

  jog_distance = current_jog_rate * estimated_jog_time


  given max_jog_rate = 1000mm/s and current_jog_rate = 100mm/s

  estimated_jog_time = 
  */

   
}

/////////////////////////////////////// DISPLAY /////////////////////////////////////////////////////

void drawCMD(std::string &c){
  display.setTextColor(WHITE, BLACK);
  display.setCursor(51,1);
  display.print(c.c_str());
  display.print(" : ");

  grbl_data_t *grbl_data = getData();
  display.print(shortNames[grbl_data->grbl.state]);
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
  display.drawFastHLine(tick_line(), 63,30, WHITE);
  display.setTextColor(WHITE, BLACK);
  display.setCursor(0,80);
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

  // Y stuff

  if(Yaxis.running){
    display.print("Y: ");
  }else{
    display.print("y: ");
  }
  //display.println(oldPos.mpos.y);
  display.print(grbl_data->position[Y_AXIS]);
  display.print(" ");
  display.print(Yaxis.vel);
  display.print(" ");
  display.println(Yaxis.feed);  

  // z:
  display.print("Z: ");
  //display.println(oldPos.mpos.z);
  display.println(grbl_data->position[Z_AXIS]);
}

void drawMode(){
  grbl_data_t *grbl_data = getData();
  display.setTextColor(WHITE,BLACK);
  display.setCursor(0,0);
  display.print(shortNames[mstate]);
  display.print("#");
  display.print(stateNames[grbl_data->grbl.state]);
}

void drawStep(){

  // prints the jog step size on the right
  display.setTextColor(WHITE,BLACK);
  display.setCursor(85,10);
  display.print("S:");
  display.print(stepSize);
}
void drawBuff(){
  grbl_data_t *grbl_data = getData();
  display.setTextColor(WHITE,BLACK);
  display.setCursor(0,54);
  display.print("BB:");
  display.print(grbl_data->buffer);
  display.print(" RX: ");
  display.print(grbl_data->buffer_rx);
}

void drawHalt(){
  display.setTextColor(WHITE,BLACK);
  display.setCursor(0,0);
  display.print("HALT:");
  display.print(cmd);
}

void drawDisplay(){
  if(displaytimer.repeat() && mstate != Halt){
    if(mstate == Halt){
      drawHalt();
    }else{
      display.clearDisplay();
      drawCMD(_gs.d_cmd);
      drawStatus(_gs.d_status);
      drawBuff();
      drawPOS();
      drawMode();
      drawStep();
      display.display();
    }
  }
}





//////////////////////////////////////// SETUP and LOOP //////////////////////////////////////////////



void setup() {
  // Timer pins
  pinMode(PC13,OUTPUT);
  pinMode(PA0, INPUT_PULLUP);
  pinMode(PA1, INPUT_PULLUP);

  // SDA SCL
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
  Serial.println("setup done");
} // end setup


/////////////////////////////////////////////////////// Loop

void loop() {
  
  // get stuff from serial and assemble messages from grbl.
  readLine();

  /*  TODO: what is this?  should only workin passive mode right?
  if (Serial.available()) {
    Serial2.write(Serial.read());
  }
  */
  
  drawDisplay();

  if(updatetimer.repeat() && mstate != Passive){
    requestUpdate(); 
  }




  if(btntimer.repeat()){
    checkBtns();
  }
  grbl_data_t *grbl_data = getData();
  calculate_velocity();

  if(statetimer.repeat()){

    switch(mstate){
      case SetupDone:
        if(pass){
          mstate = Passive;
        }else{
          //mstate = IncModeWait;
          mstate = AccelModeWait;
        }  
        break;
      case Passive:
        if(pass == false){
          old_mstate = mstate;
          mstate = IncModeWait;
          //mstate = AccelModeWait;
        }
        break;
      case IncModeWait:
        if(pass){
          old_mstate = mstate;
          mstate = Passive;
          break;
        }
        if(!inc_mode){
          old_mstate = mstate;
          mstate =  AccelModeWait;
        }else{
          inc_check_encoders();
        }
        break;
       
      case AccelModeWait:
        accel_check_encoders();

        if(inc_mode){
          old_mstate = mstate;
          mstate = IncModeWait;
        }
        if(pass ){
          old_mstate = mstate;
          mstate = Passive;
        } 
        else{
          // ?
        }
        break;

      //
      //  Executing a Jog.  Jogs may be added to the queue
      //
      case AccelModeRun: 
        accel_check_encoders();

        // issue jog, wait for ok, reset encoder position
        // TODO: need an Ok ack state to ensure no other commands get issued
        if(gotAccelOk){
          gotAccelOk = false;
          reset_pos();
        }
        

        // Jog done, transition state
        if(grbl_data->grbl.state == Idle){
          old_mstate = mstate;
          mstate = AccelModeWait;
          // capture current position on encoders.  this is used to detect a move.
          reset_pos();
        }else{
          // check to see if a running axis's encoder reversed direction
          _gs.d_status = "Waiting for Jog Stop";
        }
        break;

      //
      // transition state
      //

      case AccelModeStop:
        break;
      case AccelModeCancel:
        if(canceltimer.repeat()){
          if(serial_dbg)
            Serial.println("G4P0");

          Serial2.println("G4P0");
        }
        if(grbl_data->grbl.state == Idle){
          _gs.d_status = "Wait for a jog";
          mstate = AccelModeWait;
        }
        break;
      case IncJogStart: 
        if(grbl_data->grbl.state == Jog){
          old_mstate = mstate;
          mstate = IncJogRun;
        }else{
          _gs.d_status = "Waiting for Jog Start";
        }

        // if we are not using ok and parsing grbl buffer do we need this?
        /*
        if (incjogstarttimeout.done()){
          dbgln("IncJogStart timedout!");
          old_mstate = mstate;
          mstate = IncModeWait;
        }
        */
        break;
      case IncJogRun:
        if((Zaxis.running && Zaxis.vel < 10) || (Yaxis.running && Yaxis.vel < 10) || (Xaxis.running && Xaxis.vel < 10)){
          doJogCancelAll();
          old_mstate = mstate;
          mstate = IncJogEnd;
        }
        if(grbl_data->grbl.state == Idle){
          old_mstate = mstate;
          mstate = IncModeWait;
          // capture current position on encoders.  this is used to detect a move.
          //reset_pos();
        }else{
          inc_check_encoders();
          _gs.d_status = "Waiting for Jog Stop";
        }
        break;
      case IncJogEnd:
        if(grbl_data->grbl.state == Idle){
          old_mstate = mstate;
          mstate = IncModeWait;
        }else{
          if(!bufferFull()){
            Serial2.println("G4P0");
          }
        }
        break;
      case Halt:
        break;

  
      default: 
        halt_msg = "Unknown state";
        halted = true;
        break;
      }// end state case
    if(grbl_data->grbl.state != old_grbl_state){
      if (serial_dbg){
        Serial.print("New State: ");
        Serial.println(grbl_data->grbl.state);
        old_grbl_state = grbl_data->grbl.state;
      }
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
  }// end statetimer check
}
