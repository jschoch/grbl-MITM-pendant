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
      CMD_B--; 
      if(CMD_B == 0){  
        okWait = false;  
      }
      if(CMD_B < 0){
        halt("lost track of commands!"); 
      }
   }
   else{
     dbgln("Nothing matched");
     dbgln(cmd); 
     halt("Unknown Cmd");
   }
}

// process msg starting with [ or <
void processPush(){
  
}

// this should halt everything and wait to confirm the jog has been canceled.
void cancelJog(){
  okWait = true;
  // TODO; update with header def
  Serial2.flush();
  Serial2.write(0x85);
  Serial2.flush();
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
  // 
}

void checkCtrlInputs(){
  // checks buttons and any other non-jog inputs
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
     }// match push

     // try "ok"

     // fail and update GUI.
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
  }
  //
  else{
    if(bufferFull()){
      
    }else{
      checkJogInputs();
    }
  }
}

// watch for position updates and update gui
void loopPass(){
  
}

void halt(char * msg){
  halted = true;
  // TODO: update display;
}



void setup() {
  // put your setup code here, to run once:

}

void loop() {
  
  // get stuff from serial and assemble messages from grbl.
  readLine();
  
  // passive mode passes messages through from GUI.
  // passive mode ignores commands but updates position data 
  if(pass){
    loopPass();
  }else{
    // Jog mode tracks commands and responses
    loopJog();
  }

  // TODO: consider key combo to unhalt
  while(halted){
    dbgln("HALT");
    delay(1000);
  }
}
