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

