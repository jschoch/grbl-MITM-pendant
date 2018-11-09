#include <vector>
#include <cstring>



std::vector<char*> split(char *s, char const *sep,int max){
    char *end, *r, *tok;
    
    
    //char *strings[max];
    std::vector<char*> strings;
    r = end = strdup(s);
    //assert(end != NULL);
    int cnt = 0;
    while ((tok = strsep(&end, sep)) != NULL) {
        //printf("%s <-- \n", tok);
        std::cout << tok << " -" << sep << "- ";
        //strings[cnt] = tok;
        strings.push_back(tok);
        cnt++;
        
    }
    std::cout << "\n";
    //strings[cnt++] = "";
    return strings;
}

Status parseStatus(String line){
  if(line.charAt(0) == "<"){
     
    // playing with parsing here.
    // http://cpp.sh/6tpb

  }else{
    return Status s(false);
  }
}


class Status {
  public:
    bool error;
    state std::string;
    Status(bool b) : error(true)

}
