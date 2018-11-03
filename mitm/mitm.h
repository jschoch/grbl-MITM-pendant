enum axis {
  X,Y,Z,F
};

const char* ax_names = "XYZF";

class Axis {
    
    int id;
    
  public:
    const char *axis_name;
    long old_pos;
    long pos;

    Axis(const char *axis_name="UNDEF", int id=-1) 
        : axis_name(axis_name), id(id), pos(0), old_pos(1)
    {
        // constructor takes care of initialization
    }

    void begin() {
        old_pos=pos=0;
    }

    bool moved() {
        return (pos == old_pos);
    }

    long getPos(){
      return pos;
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

    void resetPos(){
      old_pos = pos;
    }

};
