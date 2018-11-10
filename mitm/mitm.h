#include "HardwareTimer.h"

class Axis {
    
    int id;
    HardwareTimer timer;
  public:
    const char *axis_name;
    volatile long old_pos;
    volatile long pos;
    volatile bool forward;
    int feed;
    float step;

    //Axis(const char *axis_name="UNDEF", HardwareTimer &t)
    Axis(const char *axis_name, HardwareTimer &t)
        //: axis_name(axis_name), id(id), pos(1), old_pos(1)
        : axis_name(axis_name), timer(t)
    {
        // constructor takes care of initialization
    }

    void begin(const char* in_name, int feed, float step) {
        axis_name = in_name;
        old_pos=1L;
        pos=1L;
        forward = true;
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

};
