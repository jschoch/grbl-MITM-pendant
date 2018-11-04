enum aaxis {
  X,Y,Z,F
};

const char* ax_names = "XYZF";

class Axis {
    
    int id;
    
  public:
    const char *axis_name;
    volatile long old_pos;
    volatile long pos;
    volatile bool forward;

    Axis(const char *axis_name="UNDEF", int id=-1, long pos=10L, long old_pos=10L) 
        //: axis_name(axis_name), id(id), pos(1), old_pos(1)
        : axis_name(axis_name), id(id), pos(pos), old_pos(old_pos)
    {
        // constructor takes care of initialization
    }

    void begin(const char* in_name) {
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

    bool resetPos(){
      old_pos = pos;
      return (old_pos == pos);
    }

};
