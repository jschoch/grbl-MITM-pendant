/*
*****grbl "Man in the Middle" Jog Controller Pendant*****

  Port to stm32f103 by Jesse Schoch

*/

#include "HardwareTimer.h"
#include "mitm.h"

//
// PA8/PA9 timer_1
// PA0/PA1 timer_2
// PB6/PB7 timer_3
//

//double rate1 = 115200;
unsigned long rate1 = 115200;
unsigned long rate2 = 115200;

String prefixor = "P <<<";

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

mode 1 is jog queue mode.  In this mode the encoders will be set to a value and await a "go" button press.
mode 2 is realtime jog mode.  this mode will jog and wait for "ok" status from the planner. subsequent encoder turns will queue the next jog.  A stop of encoder movement inside some debounce window will issue a jog stop.
mode 3 is setup modes.  this mode is used to set virtual stops
mode 4 is facing mode.  encoder turns will start a "pass", "stepover" or "plunge/ramp"
mode 5 will be slotting mode

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
  Serial2.println("$$");
}


void runG(String start, int axis, long distance){
  Serial2.print(start);
  Serial.print(prefixor);
  Serial.print(start);
  runG(axis);
  runG(distance);
}

void runG(String s){
  /*Serial2.print("G91 G0  X-5");
  Serial.print("P >>> G91 G0  X-5");
  */

  Serial2.print(s);
  Serial.print(s);
  
}


void runG(int axis){
  switch (axis){
    case AA:
      runG(" A ");
      break;
    case AX:
      runG(" X ");
      break;
    case AY:
      runG(" Y ");
      break;
    case AZ: 
      runG(" Z ");
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


void jogAxis(int axis, long distance){
  runG("$J=G91");
  runG(axis);
  runG(distance);
  // TODO: add feed rate var
  runG("F1000");
  
  runG();
  // TODO:  should I look for a response?
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
    jogAxis(AY,timer_3_ints);
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

long cntr = 0L;

void loop() { // run over and over
  if (Serial2.available()) {
    //Serial.println(Serial2.read(), BIN);
    Serial.write(Serial2.read());
  }
  if (Serial.available()) {
    Serial2.write(Serial.read());
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


  check_x();
}
