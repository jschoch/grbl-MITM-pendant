#include <string>
#include <vector>
#include "gstate.h"


std::vector<char*> split(String s, char const *sep,int max){
    char *end, *r, *tok;
    
    
    //char *strings[max];
    std::vector<char*> strings;
    r = end = strdup(s.c_str());
    //assert(end != NULL);
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

class Pos {
    public: 
    float x;
    float y; 
    float z;
    
    Pos(float x, float y, float z) : x(x), y(y), z(z){}
    Pos() : x(0.0), y(0.0), z(0.0) {}
};

Pos parsePos(char* s){
    std::vector<char*> coords = split(s, ",",8);
    
    if(coords.size() == 3){
        // terrible
        Pos pos = Pos((float)atof(coords[0]),(float)atof(coords[1]),(float)atof(coords[2]));
        return pos;
    }else{
        //std::cout << "HORROR\n"; 
        _gs.d_status = "HORROR";
        Pos pos = Pos(0.0f,0.0f,0.0f);
        return pos;
    }
    
}

typedef struct {
    Pos mpos;
    Pos wco;
    bool changed;
    } PosSet;
    
PosSet parseStatus(std::vector<char*> statusBlock){
    PosSet p = {Pos(), Pos(), false};
    
    for (unsigned int i = 1; i < statusBlock.size();i++){
        std::vector<char*> status = split(statusBlock[i],":",8);
        if(status[1] != NULL && ((strcmp(status[0], "MPos") == 0) )){
            //std::cout << "Pos found:" << status[0] << ":" << status[1] << " \n";
            Pos pos = parsePos(status[1]);
            //std::cout << "mpos parsed: " << pos.x << "\n";
            p.mpos = pos;
            p.changed = true;
        }
         if(status[1] != NULL && (strcmp(status[0], "WCO") == 0)){
            //std::cout << "Pos found:" << status[0] << ":" << status[1] << " \n";
            Pos pos = parsePos(status[1]);
            //std::cout << "wco parsed: " << pos.x << "\n";
            p.wco = pos;
            p.changed = true;
        }
        
        for (unsigned int y = 0; y < status.size(); y++){
            std::vector<char*> bar = split(status[y],",",8);
        }
    }
    return p;
}
