// if N2K enabled
#include "EngineMonitorBuildFlags.h"
#include "EngineMonitorHardware.h"
#include "Globals.h"
#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <N2kMessages.h>
#include <N2KDeviceList.h>
#include "EngineMonitorN2K.h"

// Forward declarations
void SendN2kEngine( void );
void setupN2K( void );
void SendN2kTemperature( void );

// Set the information for other bus devices, which messages we support
const unsigned long TransmitMessages[] PROGMEM = { 127488L, // Engine Rapid / RPM
                                                   127489L, // Engine Dynamic, Oil Pressure and temperature
                                                   127505L, // Fluid Level
                                                   127508L, // Battery Status
                                                   130316L, // Temperature values (or alternatively 130311L or 130312L but they are depricated)                                                   
                                                   0
                                                 };

// Define schedulers for messages. Define schedulers here disabled. Schedulers will be enabled
// on OnN2kOpen so they will be synchronized with system.
// We use own scheduler for each message so that each can have different offset and period.
// Setup periods according PGN definition (see comments on IsDefaultSingleFrameMessage and
// IsDefaultFastPacketMessage) and message first start offsets. Use a bit different offset for
// each message so they will not be sent at same time.
tN2kSyncScheduler TemperatureScheduler(false,1000,2000);
tN2kSyncScheduler EngineRapidScheduler(false,750,500);
tN2kSyncScheduler EngineDynamicScheduler(false,500,1200);
tN2kSyncScheduler BatteryStatusScheduler(false,1000,1200);
tN2kSyncScheduler FuelTankLevelScheduler(false,500,500);
tN2kDeviceList *locN2KDeviceList;
uint8_t n2kConnected = 0;

// *****************************************************************************
// Call back for NMEA2000 open. This will be called, when library starts bus communication.
// See NMEA2000.SetOnOpen(OnN2kOpen); on setup()
void OnN2kOpen() {
  // Start schedulers now.
  TemperatureScheduler.UpdateNextTime();
  BatteryStatusScheduler.UpdateNextTime();
  EngineRapidScheduler.UpdateNextTime();
  EngineDynamicScheduler.UpdateNextTime();
  FuelTankLevelScheduler.UpdateNextTime();
} // OnN2kOpen

// setup for N2k
void setupN2K() {
  
  // Set Product information
  // Tweaked to display properly on Raymarine...
  NMEA2000.SetProductInformation("EM V3", // Manufacturer's Model serial code
                                 100, // Manufacturer's product code
                                 "SN00000001",  // Manufacturer's Model ID
                                 "1.0.0.01 (2024-01-15)",  // Manufacturer's Software version code
                                 #ifndef LITE_V3
                                 "EM V3" // Manufacturer's Model version
                                 #else
                                 "EM V3 Lite" // Manufacturer's Model version
                                 #endif
                                 );
  // Set device information
  NMEA2000.SetDeviceInformation(chipId, // Unique number. Use e.g. Serial number.
                                132, // Device function=Analog -> NMEA2000. See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                25, // Device class=Sensor Communication Interface. See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2040 // Just choosen free from code list on https://web.archive.org/web/20190529161431/http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                               );

  // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode/*N2km_NodeOnly*/,22);
  locN2KDeviceList = new tN2kDeviceList(&NMEA2000);
  NMEA2000.EnableForward(false); // Disable all msg forwarding to USB (=Serial)
  // Here we tell library, which PGNs we transmit
  NMEA2000.ExtendTransmitMessages(TransmitMessages);
  // Define OnOpen call back. This will be called, when CAN is open and system starts address claiming.
  NMEA2000.SetOnOpen(OnN2kOpen);

  // Reserve enough buffer for sending all messages. This does not work on small memory devices like Uno or Mega
  NMEA2000 . SetN2kCANMsgBufSize ( 32 );
  NMEA2000 . SetN2kCANReceiveFrameBufSize ( 500 );
  NMEA2000 . SetN2kCANSendFrameBufSize ( 500 );

  NMEA2000.Open();
} // end setupN2K

// *****************************************************************************
double ReadCabinTemp() {
  int tmptmp=random(0,300);
  return CToKelvin((float)(tmptmp)/10); // Read here the true temperature e.g. from analog input
}

// *****************************************************************************
double ReadWaterTemp() {
  int tmptmp=random(100,900);
  return CToKelvin((float)(tmptmp)/10); // Read here the true temperature e.g. from analog input
}

// *****************************************************************************
void SendN2kEngine() {
  tN2kMsg N2kMsg;
  tN2kEngineDiscreteStatus1 Status1=0;
  tN2kEngineDiscreteStatus2 Status2=0;
  if ( EngineRapidScheduler.IsTime() ) {
    EngineRapidScheduler.UpdateNextTime();
    SetN2kEngineParamRapid ( N2kMsg , n2kInstance, EngRPM, N2kDoubleNA, N2kInt8NA );
    NMEA2000.SendMsg(N2kMsg);
  } // endif

  if ( EngineDynamicScheduler.IsTime() ) {
    EngineDynamicScheduler.UpdateNextTime();
    
    // if this bit is set then check engine alarm pops up on the screen
    // this is double reversed, in PCB h/w and on the engine.
    // low at the terminal is high in the processor
    // low at the terminal is alarm state.
    Status1.Bits.CheckEngine = chkEng;
    Status2.Bits.MaintenanceNeeded = 0;

    // use engine temp as oil temp for now
    SetN2kEngineDynamicParam ( N2kMsg , n2kInstance, OilPres, engineBlockTemp, engineCoolantTemp, AltVolts,
                               N2kDoubleNA , N2kDoubleNA , N2kDoubleNA , N2kDoubleNA , N2kInt8NA , N2kInt8NA , Status1 , Status2 );

    NMEA2000.SendMsg(N2kMsg);
  } // endif

} // end sendn2kenginerapid

// *****************************************************************************
void SendN2kBattery() {
  tN2kMsg N2kMsg;
  if ( BatteryStatusScheduler.IsTime() ) {
    BatteryStatusScheduler.UpdateNextTime();
    SetN2kDCBatStatus(N2kMsg, n2kInstance, AltVolts, 0/*BatteryCurrent=N2kDoubleNA*/,
                     0/*BatteryTemperature=N2kDoubleNA*/, 0xff);
    NMEA2000.SendMsg(N2kMsg);
  } // endif

} // end SendN2kBattery

// *****************************************************************************
void SendN2kTemperature() {
  tN2kMsg N2kMsg;

  if ( TemperatureScheduler.IsTime() ) {
    TemperatureScheduler.UpdateNextTime();
    //SetN2kTemperature(N2kMsg, 1, 1, N2kts_MainCabinTemperature, ReadCabinTemp());
    SetN2kTemperatureExt(N2kMsg, 255, n2kInstance, N2kts_EngineRoomTemperature, engineRoomTemp, N2kDoubleNA);
    NMEA2000.SendMsg(N2kMsg);
    delay(50); // probably not neccessary
    SetN2kTemperatureExt(N2kMsg, 255, n2kInstance, N2kts_ExhaustGasTemperature, engineExhaustTemp, N2kDoubleNA);
    NMEA2000.SendMsg(N2kMsg);
  } // endif

} // SendN2kTemperature

// *****************************************************************************
void SendN2kTankLevel() {
  tN2kMsg N2kMsg;

  if ( FuelTankLevelScheduler.IsTime() ) {
    // fuel level
    FuelTankLevelScheduler.UpdateNextTime();
    SetN2kFluidLevel(N2kMsg, n2kInstance, N2kft_Fuel, FuelLevel, 75.7);
    NMEA2000.SendMsg(N2kMsg);
  } // end if
} // end send2ktanklevel

// does all N2K send/recieve processing. Send message cadence is controlled with timers.
// This helper is called on a regular timer, Send messages are senbt when thier 
// specifiec timer is done.
void doN2Kprocessing(){
  SendN2kTemperature();
  SendN2kEngine();
  SendN2kBattery();
  SendN2kTankLevel();
  NMEA2000.ParseMessages();
  // record the number of devices on n2k bus
  n2kConnected = locN2KDeviceList->Count();
} // end doN2kprocessing

