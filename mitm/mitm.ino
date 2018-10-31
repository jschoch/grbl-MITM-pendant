/*
*****grbl "Man in the Middle" Jog Controller Pendant*****

  Uses interrupt-based rotary encoder code by Simon Merrett, based on insight from Oleg Mazurov, Nick Gammon, rt, Steve Spence
  Based on SoftwareSerialExample sketch by Tom Igoe
  Receives from the hardware serial, sends to software serial.
  Receives from software serial, sends to hardware serial.

  The circuit:
   RX is digital pin  (connect to TX of other device)
   TX is digital pin  (connect to RX of other device)


  Port to stm32f103 by Jesse Schoch

*/

#include "HardwareTimer.h"
#include "HardwareSerial.h"

//
// PA8/PA9 timer_1
// PA0/PA1 timer_2
// PB6/PB7 timer_3
//

static int pinA = PA8; // Our first hardware interrupt pin is digital pin 2
static int pinB = PA9; // Our second hardware interrupt pin is digital pin 3
volatile byte aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
volatile byte bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
volatile byte encoderPos = 0; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
volatile byte oldEncPos = 0; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
volatile byte reading = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent

//double rate1 = 115200;
unsigned long rate1 = 115200;
unsigned long rate2 = 115200;

void setup() {
  // Open serial communications and wait for port to open:
  pinMode(PC13,OUTPUT);
  Serial.begin(rate1);
  while (!Serial) {
    
    ; // wait for serial port to connect. Needed for native USB port only
    digitalWrite(PC13,LOW);
  }
  delay(500);
  digitalWrite(PC13,HIGH);
  //pinMode(pinA, INPUT_PULLUP); // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  //pinMode(pinB, INPUT_PULLUP); // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  //attachInterrupt(0, PinA, RISING); // set an interrupt on PinA, looking for a rising edge signal and executing the "PinA" Interrupt Service Routine (below)
  //attachInterrupt(1, PinB, RISING); // set an interrupt on PinB, looking for a rising edge signal and executing the "PinB" Interrupt Service Routine (below)

  Serial.println("Pendant connected");
  // set the data rate for the SoftwareSerial port

  // Using serial pins PA2/PA3
  Serial2.begin(rate1);
  // mySerial.println("Hello, world?");
}

//  STM32 encoder stuff

int PPR = 1;

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


void func_timer_2(){
   if(timer_2.getDirection()){
      timer_2_ints--;    
  }else{
      timer_2_ints++;
  }    
}


// uses internal timer 4 on PB6 and PB7
// somewhat confusing to call this timer3 when it is internal timer 4....
HardwareTimer timer_3(4);

long timer_3_ints = 1L;


void func_timer_3(){
   if(timer_3.getDirection()){
      timer_3_ints--;    
  }else{
      timer_3_ints++;
  }    
}

void config_timer(HardwareTimer this_timer, void (*func)()){
  this_timer.setMode(0, TIMER_ENCODER); //set mode, the channel is not used when in this mode. 
  this_timer.pause(); //stop... 
  this_timer.setPrescaleFactor(1); //normal for encoder to have the lowest or no prescaler. 
  this_timer.setOverflow(PPR);    //use this to match the number of pulse per revolution of the encoder. Most industrial use 1024 single channel steps. 
  this_timer.setCount(0);          //reset the counter. 
  this_timer.setEdgeCounting(TIMER_SMCR_SMS_ENCODER3); //or TIMER_SMCR_SMS_ENCODER1 or TIMER_SMCR_SMS_ENCODER2. This uses both channels to count and ascertain direction. 
  this_timer.attachInterrupt(0, func); //channel doesn't mean much here either.  
  this_timer.resume();
}


void PinA() {
  /*
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; // read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00001100 && aFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos --; //decrement the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00000100) bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
  sei(); //restart interrupts
  */
}


void PinB() {
  /*
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; //read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00001100 && bFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos ++; //increment the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00001000) aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
  sei(); //restart interrupts
  */
}

void loop() { // run over and over
  if (Serial2.available()) {
    //Serial.println(Serial2.read(), BIN);
    Serial.write(Serial2.read());
  }
  if (Serial.available()) {
    Serial2.write(Serial.read());
  }
  if (encoderPos != oldEncPos) {
    if (oldEncPos > encoderPos) {
      //mySerial.println("$J=G91 X-10");
      Serial2.println("G91 G0  X-5");
      Serial.println("P >>> G91 G0  X-5");
      oldEncPos = encoderPos;
    }
    else if (oldEncPos < encoderPos) {
      //mySerial.println("$J=G91 X10");
      Serial2.println("G91 G0  X5");
      Serial.println("P >>> G91 G0  X5");
      oldEncPos = encoderPos;
    }
  }
}
