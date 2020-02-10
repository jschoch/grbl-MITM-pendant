#include "HardwareTimer.h"
#include "Arduino.h"
#include <EwmaT.h>

EwmaT <int> adcFilter(50, 50);

class Axis {
    
    int id;
    HardwareTimer timer;
  public:
    const char *axis_name;
    volatile long old_pos;
    volatile long pos;
    volatile bool forward;
    long vel_new_pos;
    long vel_old_pos;
    //volatile long vel;
    long vel;
    volatile unsigned long rate;
    long oldvel;
    long current_vel;
    bool running;
    volatile unsigned long newtime;
    volatile unsigned long oldtime;
    int feed;
    float step;
    float factor;
    int axis_num;

    //Axis(const char *axis_name="UNDEF", HardwareTimer &t)
    Axis(const char *axis_name, int axis_num, HardwareTimer &t)
        //: axis_name(axis_name), id(id), pos(1), old_pos(1)
        : axis_name(axis_name), axis_num(axis_num),timer(t)
    {
        // constructor takes care of initialization
    }

    void begin(const char* in_name, int feed, float step) {
        axis_name = in_name;
        old_pos=1L;
        oldvel = 0;
        pos=1L;
        rate = 0;
        running = false;
        forward = true;
        oldtime = millis();
        feed = 100;
        
    }

    bool moved() {
        return (pos != old_pos);
    }

    long getPos(){
      return pos;
    }

    long getOldPos(){
      return old_pos;
    }

    void setPos(long nextpos) {
        pos = nextpos;
    }

    void decrPos() {
        pos--;
    }

    void incrPos() {
        pos++;
    }

    void blastPos(){
      old_pos = 0L;
      pos = 0L;
    }

    void resetPos(){
      old_pos = pos;
    }

    void setRunning(){
      running = true;
    }
    
    void notRunning(){
      running = false;
    }
    void velocity(){
      newtime = micros();
      //newtime = millis();
      //vel = (1/(newtime - oldtime)) * 100000 ;
      rate = newtime - oldtime;
      oldtime = newtime;
    }

};
