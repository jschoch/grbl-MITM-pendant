

enum buttons_enum {
  PASS_TOGGLE, // PB15
  STEP_INC, // PB4
  INC_TOGGLE, // PA4
  CLEAR_ALARM, // PA15
  JOG_CANCEL // PB12
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

const char* shortNames[] = {
  "S",
  "SD",
  "Pas",
  "IMW",
  "AMW",
  "AMS",
  "AMR",
  "AMH",
  "AMC",
  "IJS",
  "IJST",
  "IJR",
  "IJE",
  "Halt"
};

