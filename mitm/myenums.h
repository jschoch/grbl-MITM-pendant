

enum buttons_enum {
  PASS_TOGGLE,
  STEP_INC,
  INC_TOGGLE,
  CLEAR_ALARM,
  JOG_CANCEL 
};

enum ctrl_states{
  Setup,
  SetupDone,
  Passive,
  IncModeWait,
  AccelModeWait,
  AccelModeStart,
  AccelModeRun,
  AccelModeStop,
  AccelModeCancel,
  IncJogStart,
  IncJogStartTimeout,
  IncJogRun,
  IncJogEnd,
  Halt
  
};

const char* stateNames[] = {
  "Unknown",
    "Idle",
    "Run",
    "Jog",
    "Hold:0",
    "Hold:1",
    "Alarm",
    "Check",
    "Door:0",
    "Door:1",
    "Door:2",
    "Door:3",
    "Tool"
};

const char* cStateNames[] = {
  "Setup",
  "SetupDone",
  "Passive",
  "IncModeWait",
  "AccelModeWait",
  "AccelModeStart",
  "AccelModeRun",
  "AccelModeStop",
  "AccelModeCancel",
  "IncJogStart",
  "IncJogStartTimeout",
  "IncJogRun",
  "IncJogEnd",
  "Halt"
};

