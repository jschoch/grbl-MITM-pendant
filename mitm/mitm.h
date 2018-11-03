enum axis {
  X,
  Y,
  Z,
  F
};

const char* ax_names = "XYZF";

class Axis{

  public:
    void begin(long *pos, const char* axis_name, int id);
    long *pos;
    long old_pos;
    int id;
    const char* axis_name = "UNDEF";
    bool moved();
    long getPos(){
      return *pos;
    };
    void resetPos(){
      old_pos = *pos;
    }
};

void Axis::begin(long *inpos, const char* inname, int inid){  
  *pos = *inpos;
  axis_name = inname;
  id = inid;
  old_pos = 1L;  
}

bool Axis::moved(){
  if(*pos == old_pos){
    return false;
  }else{
    return true;
  }
  
}
