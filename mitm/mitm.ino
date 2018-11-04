/*
*****grbl "Man in the Middle" Jog Controller Pendant*****

  Port to stm32f103 by Jesse Schoch

*/

#include "HardwareTimer.h"
#include "mitm.h"
//#include <cstring>
#include <string>

//
// PA8/PA9 timer_1
// PA0/PA1 timer_2
// PB6/PB7 timer_3
//

//double rate1 = 115200;
unsigned long rate1 = 115200;
unsigned long rate2 = 115200;

String prefixor = "P <<<";

bool idle = true;
bool waiting = false;

int feedXY = 500;
int rapidXY = 1000;
float stepSize = 0.01;

//  STM32 encoder stuff

int PPR = 2;

HardwareTimer timer_1(1);

Axis Xaxis("X",X, 0L, 0L);


void func_timer_1(){
   if(timer_1.getDirection()){
      Xaxis.decrPos();
  }else{
      Xaxis.incrPos();
  }    
}

// uses internal timer 2 on PA0 and PA1
HardwareTimer timer_2(2);

Axis Zaxis("Z",Z,0L,0L);

void func_timer_2(){
   if(timer_2.getDirection()){
      Zaxis.decrPos();
  }else{
      Zaxis.incrPos();
  }    
}


// uses internal timer 4 on PB6 and PB7
//  not used


// use internal timer 3 on PA 6/7

HardwareTimer timer_3(3);

volatile long ypos = 0;
volatile long yold_pos = 2;
//Axis Yaxis("Y",Y,ypos, yold_pos);
Axis Yaxis;

void func_timer_3(){
   if(timer_3.getDirection()){
      Yaxis.forward = true;
      Yaxis.decrPos();
  }else{
      Yaxis.forward = false;
      Yaxis.incrPos();
  }    
}

void config_timer(HardwareTimer this_timer, void (*func)()){
  this_timer.pause(); //stop... 

  // setMode(channel, mode)  mode 0 was crashing but it was not supposed to be used!?
  this_timer.setMode(1, TIMER_ENCODER); //set mode, the channel is not used when in this mode.
  this_timer.setPrescaleFactor(1); //normal for encoder to have the lowest or no prescaler. 
  this_timer.setOverflow(PPR);    //use this to match the number of pulse per revolution of the encoder. Most industrial use 1024 single channel steps. 
  this_timer.setCount(0);          //reset the counter. 
  this_timer.setEdgeCounting(TIMER_SMCR_SMS_ENCODER3); //or TIMER_SMCR_SMS_ENCODER1 or TIMER_SMCR_SMS_ENCODER2. This uses both channels to count and ascertain direction. 
  this_timer.attachInterrupt(0, func); //channel doesn't mean much here either.  
  this_timer.resume();
}


// Modes

/*

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


void setup() {
  
  //configure timers 
  
  //config_timer(timer_1,func_timer_1);
  //config_timer(timer_2,func_timer_2);
  //config_timer(timer_3,func_timer_3);

  pinMode(PC13,OUTPUT);
  pinMode(PA0, INPUT_PULLUP);
  pinMode(PA1, INPUT_PULLUP);
  pinMode(PA6, INPUT_PULLUP);
  pinMode(PA7, INPUT_PULLUP);


  //config_timer(timer_2,func_timer_2);

  Serial.begin(rate1);

  // Open serial communications and wait for port to open:
  while (!Serial) {
    
    ; // wait for serial port to connect. Needed for native USB port only
    digitalWrite(PC13,LOW);
  }
  delay(500);
  digitalWrite(PC13,HIGH);

  Serial.println("Serial connected... connecting to grbl...");
  // set the data rate for the SoftwareSerial port

  // Using serial pins PA2/PA3
  Serial2.begin(rate1);
  while( !Serial2){
    digitalWrite(PC13,LOW);
  }
  digitalWrite(PC13,HIGH);
  Serial.println("Ready to configure timer");

  config_timer(timer_2,func_timer_2);
  config_timer(timer_3,func_timer_3);

  // setup axis objects

  Yaxis.begin("Y");
  //Zaxis.begin();
  //Xaxis.begin();


 
  Serial.println("Timer configured");
  Serial.println("Running setup");
  Serial2.println("$$");

  // TODO: here is set MM mode, how to deal with these preferences?
  Serial2.println("G21");
}


void runG(String start, Axis axis, int steps){

  // TODO: should i check the state first?

  Serial2.print(start);
  Serial.print(prefixor);
  Serial.print(start);
  runG(axis,steps);
  // finish 
  runG();
}

void runG(String s){

  Serial2.print(s);
  Serial.print(s);
  
}

void runG(std::string s){

  Serial2.print(s.c_str());
  Serial.print(s.c_str());
}


void runG(Axis axis, int steps){
  float d = steps * stepSize;
  //String distance = String(d);
  std::string gcode(axis.axis_name);
   
  // negative distance for reverse
  if(!axis.forward){
    gcode.append("-");
  }
  char buffer[13];
  gcode.append(dtostrf(d,6, 6, buffer));

  // TODO: add axis specific feeds
  gcode.append(" F1000");

  
  runG(gcode);

}

void runG(long distance){
  runG(String(distance));
}

void runG(){
  // TODO:  should i just have runGln which prints eol
  Serial.println("");
  Serial2.println("");
}

// TODO: distance should be float modified by step size
// shoudl be able to queue some number of moves based on formula described on grbl jog docs
// in other modes incremental distance works

void jogAxis(Axis axis, int steps){
  if (!waiting){
    waiting = true;
    runG("$J=G91 ",axis,steps);
  }else{
  
    // maybe blink here or something.  print "w" seems to mess up line parsing.
    //Serial.print("w");
  }
}


void cutAxis(int axis){
  //  how should this work, should it get the speed or use the current set speed?
}


void check_input(){
  check_axis(Yaxis);
  check_axis(Xaxis);
  check_axis(Zaxis);
  }

void check_axis(Axis &axis){
  if(axis.moved()){
    jogAxis(axis, 1);
    bool ya = axis.resetPos();
  }
}



void check_pos(){
  // issue ? command and parse reult
}

void hold(){
  // issue ! command to enter feed hold
}

void jobCancel(){
  // need motion cancel hardware button to quickly issue Jog Cancel command
}

void feedOverride(){
  // feed override hard/soft keys
}

void draw(){
  // draw on the display
}

void setMode(){
  // Set current mode
}

String getLine(){
  String inData;
  bool foundeol = false;
  while (!foundeol){
        while(Serial2.available() > 0){
          char recieved = Serial2.read();
          inData += recieved; 
  
          // Process message when new line character is recieved
          if (recieved == '\n')
          {
              //inData += '\n';
              foundeol = true;
              break;
          }
        } // available while
    }
  Serial.print("  \t...:");
  Serial.print(inData);
  return inData;
}

bool isError(String cmd){
  return false;
}

void handleError(String cmd){

  // TODO:  how do you ensure you are not parsing error codes from a job?

  Serial.println("Error: " + cmd);
  /*  this doesn't work switch doesn't like String, you need to parse out the int in the error number
  switch (cmd){
    case "error:8":
      Serial.prinln("machine in alarm, cannot jog!  Press Alarm reset if safe");
      break;

    case "error:2":
      Serial.println("no feed set.  Set a feedrate!");

    case "error:15":
      Serial.println("Jog would exceed machine boundary.  Update soft limits or don't jog beyond the machine");

    default: 
      Serial.println(" unknown msg!");
      Serial.print(cmd);
      Serial.println("DISASTER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

  }
  */
}

void checkOk(String cmd){
  //if(std::strcmp(cmd,"ok\n") == 0){


  // this startsWith seems shitty
  if (cmd.startsWith( "ok")){
      Serial.println("OK!");
      waiting = false;
  }
  else if(cmd.startsWith("error:")){
    handleError(cmd);
  }
  else{
    Serial.println("HORROR!");
  }
}

void parseCmd(String cmd){
  checkOk(cmd);
}

long cntr = 0L;



void loop() { // run over and over
  if (Serial2.available()) {
    //Serial.println(Serial2.read(), BIN);
    String cmd = getLine();
    if(waiting){
      parseCmd(cmd);
      // do something with cmd here
    }
    Serial.print(cmd);
  }
  if (Serial.available()) {
    Serial2.write(Serial.read());
  }
  
  if (idle){
    check_input();
  }
}


