// parses and deals with the machine position for both work and machine position.

#include <string>
#include <vector>
#include "gstate.h"

extern unsigned long lastUpdate;

class Pos {
    public: 
    float x;
    float y; 
    float z;
    
    Pos(float x, float y, float z) : x(x), y(y), z(z){}
    Pos() : x(0.0), y(0.0), z(0.0) {}
};

typedef struct {
    Pos mpos;
    Pos wco;
    bool changed;
    int f1;
    int f2;
    char *lastState;
    } PosSet;
    
PosSet pp = {Pos(), Pos(), false};
Pos wpos;
Pos mpos;

static std::vector<char*> status;

float px,py,pz;
int pf1, pf2;
char mystate[255];
int r = 0;

void updatePos(char* cmd, PosSet &posSet){

  // TODO: why can't i have it modify directly?
  //static int r = sscanf(cmd,"<Idle|MPos:%f,%f,%f",&x,posSet.mpos.y,&z);
  //int r = sscanf(cmd,"<Idle|MPos:%f,%f,%f",&x,&y,&z);
  r = sscanf(cmd, "<%[a-zA-Z]|MPos:%f,%f,%f|FS:%d,%d",mystate,&px,&py,&pz,&pf1,&pf2);
  
  // TODO: what if these are actually 0.0?
  if(r == 6){
    lastUpdate = millis();
    posSet.mpos.x = px;
    posSet.mpos.y = py;
    posSet.mpos.z = pz;
    posSet.f1 = pf1;
    posSet.f2 = pf2;
    posSet.lastState = mystate;

  }
  /*
  if (r == 3){
    posSet.mpos.x = x;
  }
  if (y != 0){
    posSet.mpos.y = y;
  }
  if (z != 0){
    posSet.mpos.z = z;
  }
  */
}

