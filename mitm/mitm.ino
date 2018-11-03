/*
*****grbl "Man in the Middle" Jog Controller Pendant*****

  Port to stm32f103 by Jesse Schoch

*/

#include "HardwareTimer.h"
#include "mitm.h"
#include <cstring>

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

long timer_1_ints = 1L;


void func_timer_1(){
   if(timer_1.getDirection()){
      timer_1_ints--;    
  }else{
      timer_1_ints++;
  }    
}

// uses internal timer 2 on PA0 and PA1
HardwareTimer timer_2(2);

long timer_2_ints = 1L;
long old_timer_2_ints = 1L;


void func_timer_2(){
   if(timer_2.getDirection()){
      timer_2_ints--;    
  }else{
      timer_2_ints++;
  }    
}


// uses internal timer 4 on PB6 and PB7

//  not used


// use internal timer 3 on PA 6/7
// somewhat confusing to call this timer3 when it is internal timer 4....
HardwareTimer timer_3(3);

long timer_3_ints = 1L;

long old_timer_3_ints = 1L;

void func_timer_3(){
   if(timer_3.getDirection()){
      timer_3_ints--;    
  }else{
      timer_3_ints++;
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
  Serial.println("Timer configured");
  Serial.println("Running setup");
  Serial2.println("$$");

  // TODO: here is set MM mode, how to deal with these preferences?
  Serial2.println("G21");
}


void runG(String start, int axis, int steps){
  // TODO: should i check the state first?

  Serial2.print(start);
  Serial.print(prefixor);
  Serial.print(start);
  runG(axis,steps);
  //String feed = " F" + feedXY;
  //runG(feed);
  runG(" F1000 ");
  runG();
}

void runG(String s){

  Serial2.print(s);
  Serial.print(s);
  
}


void runG(int axis, int steps){
  float d = steps * stepSize;
  String distance = String(d);
  switch (axis){
    case AA:
      runG(" A "+ distance);
      break;
    case AX:
      // TODO shoudl use axis specific steps
      runG(" X "+ distance);
      break;
    case AY:
      runG(" Y "+ distance);
      break;
    case AZ: 
      runG(" Z "+ distance);
    default: 
      break;
  }

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
void jogAxis(int axis, int steps){
  if (!waiting){
    waiting = true;
    runG("$J=G91",axis,steps);
  }else{
    Serial.print("w");
  }
}


void cutAxis(int axis){
  //  how should this work, should it get the speed or use the current set speed?
}

void check_x(){
  if (old_timer_2_ints != timer_2_ints){
    Serial.print("XI: ");
    Serial.println(timer_2_ints);
    old_timer_2_ints = timer_2_ints;
  }

  if (old_timer_3_ints != timer_3_ints){
    Serial.print(timer_3.getCount());
    Serial.print(" YI: ");
    Serial.println(timer_3_ints);
    old_timer_3_ints = timer_3_ints;
    //jogAxis(AY,timer_3_ints);
    jogAxis(AY,2);
    old_timer_3_ints = timer_3_ints;
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

void checkOk(String cmd){
  //if(std::strcmp(cmd,"ok\n") == 0){

  // fuck there are way too many ways to do simple shit in c++
  //if (cmd == 'ok\n'){

  // this startsWith seems shitty
  if (cmd.startsWith( "ok")){
      //std::cout << "string was 'ok'";
      Serial.write("OK!");
      waiting = false;
  }else{
    //std::cout << "string was not 'ok'";   
    Serial.write("HORROR!");
  }
}

void parseCmd(String cmd){
  if (isError(cmd)){
    
  }
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
    // working
    check_x();
  }
}


  /*
  if(cntr > 200000){
    cntr = 0;
    int a = digitalRead(PA6);
    int b = digitalRead(PA7);
    Serial.print("T3: ");
    Serial.print(timer_3_ints);
    Serial.print(" A: ");
    Serial.print(a);
    Serial.print(" B: ");
    Serial.println(b);

  }else{
    cntr++;
  }
  */
