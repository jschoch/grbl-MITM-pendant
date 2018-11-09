#ifndef GSTATE_H
#define GSTATE_H

#include <string>
#include <Arduino.h>


/*class Gstate {

};

*/

class Gstate {

    public: 
      std::string d_cmd;
      std::string d_status;
      Gstate() : d_cmd(""), d_status("DONGS") {};
      void begin(){
        d_cmd = "";
        d_status = "DONGS";
      }
  
};

extern Gstate _gs;

#endif
