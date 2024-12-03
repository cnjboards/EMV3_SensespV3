#include <Arduino.h>

// unique chip id
extern int32_t chipId;

// used for N2K Messages
extern double AltRPM;
extern double EngRPM;
extern double OilPres;
extern double AltVolts;
extern double HouseVolts;
extern double FuelLevel;
extern double B1A3;
extern double B2A0;
extern double B2A1;
extern double B2A2;
extern double B2A3;
extern bool chkEng;
extern bool digIn2;
extern double engineCoolantTemp;
extern double engineBlockTemp;
extern double engineRoomTemp;
extern double engineExhaustTemp;
extern String screenSelect;

// push button states
extern int shortButtonStateLatched;
extern int longButtonStateLatched;
