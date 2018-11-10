#include <string>
#include <vector>
#include "gstate.h"

/*  this is not memory friendly
std::vector<char*> split(char* s, char const *sep,int max){
    char *end, *r, *tok;
    
    
    std::vector<char*> strings;
    r = end = strdup(s);
    int cnt = 0;
    while ((tok = strsep(&end, sep)) != NULL) {
        //printf("%s <-- \n", tok);
        //std::cout << "DBG: " << tok << " -" << sep << "- ";
        //strings[cnt] = tok;
        strings.push_back(tok);
        cnt++;
        
    }
    //std::cout << "\n";
    //strings[cnt++] = "";
    return strings;
}
*/

class Pos {
    public: 
    float x;
    float y; 
    float z;
    
    Pos(float x, float y, float z) : x(x), y(y), z(z){}
    Pos() : x(0.0), y(0.0), z(0.0) {}
};


/* not mem friendly
static std::vector<char*> coords;

void parsePos(char* s, Pos &pos){
    coords = split(s, ",",8);
    
    if(coords.size() == 3){
        // terrible
        //pos = Pos((float)atof(coords[0]),(float)atof(coords[1]),(float)atof(coords[2]));
        pos.x = (float)atof(coords[0]);
        pos.y = (float)atof(coords[1]);
        pos.z = (float)atof(coords[2]);
    }else{
        //std::cout << "HORROR\n"; 
        _gs.d_status = "POS HORROR";
        //pos = Pos(0.0f,0.0f,0.0f);
    }
    
}
*/

typedef struct {
    Pos mpos;
    Pos wco;
    bool changed;
    } PosSet;
    
PosSet pp = {Pos(), Pos(), false};
Pos wpos;
Pos mpos;

static std::vector<char*> status;

void updatePos(char* cmd, PosSet &posSet){
  float x,y,z;

  // TODO: why can't i have it modify directly?
  //static int r = sscanf(cmd,"<Idle|MPos:%f,%f,%f",&x,posSet.mpos.y,&z);
  int r = sscanf(cmd,"<Idle|MPos:%f,%f,%f",&x,&y,&z);
  
  // TODO: what if these are actually 0.0?
  if (x != 0){
    posSet.mpos.x = x;
  }
  if (y != 0){
    posSet.mpos.y = y;
  }
  if (z != 0){
    posSet.mpos.z = z;
  }
}

/* not mem friendly
PosSet parseStatus(std::vector<char*> &statusBlock){
    
    for (unsigned int i = 1; i < statusBlock.size();i++){
        status = split(statusBlock[i],":",8);
        if(status[1] != NULL && ((strcmp(status[0], "MPos") == 0) )){
            //std::cout << "Pos found:" << status[0] << ":" << status[1] << " \n";
            parsePos(status[1],mpos);
            //std::cout << "mpos parsed: " << pos.x << "\n";
            pp.mpos = mpos;
            pp.changed = true;
        }
         if(status[1] != NULL && (strcmp(status[0], "WCO") == 0)){
            //std::cout << "Pos found:" << status[0] << ":" << status[1] << " \n";
            parsePos(status[1],wpos);
            //std::cout << "wco parsed: " << pos.x << "\n";
            pp.wco = wpos;
            pp.changed = true;
        }
        
    }
    return pp;
}
*/
