
// Signal K application template file.
//
// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries.
#include "EngineMonitorBuildFlags.h"
#include <Arduino.h>
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/digital_output.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/sensors/constant_sensor.h"
#include "sensesp/signalk/signalk_listener.h"
#include "sensesp/signalk/signalk_value_listener.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include <sensesp/system/valueconsumer.h>
#include "sensesp_onewire/onewire_temperature.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/transforms/frequency.h"
#include "sensesp/transforms/analogvoltage.h"
#include "sensesp/transforms/hysteresis.h"
#include "sensesp/transforms/enable.h"
#include "sensesp_app_builder.h"

// specific to this design
#include "EngineMonitorHardware.h"
#include "EngineMonitorN2K.h"
#include "EngineMonitortft320x240.h"
#include <Adafruit_ADS1X15.h>
#include <lvgl.h>
#include <ESP.h>

using namespace sensesp;
using namespace sensesp::onewire;

//reactesp::ReactESP app;

// ADS1115 Module Instantiation
// ADS1115 @ 0x48 I2C
Adafruit_ADS1115 adsB1;

// volts for adsB1
float read_adsB1_ch0_callback() { return (adsB1 . computeVolts ( adsB1 . readADC_SingleEnded ( 0 ) ));}
float read_adsB1_ch1_callback() { return (adsB1 . computeVolts ( adsB1 . readADC_SingleEnded ( 1 ) ));}
// invert since level voltage decreses as level increases. 
float read_adsB1_ch2_callback() { return (2.2 - (adsB1 . computeVolts ( adsB1 . readADC_SingleEnded ( 2 ) )));}
//float read_adsB1_ch2_callback() { return ((adsB1 . computeVolts ( adsB1 . readADC_SingleEnded ( 2 ) )));}

float read_adsB1_ch3_callback() { return (adsB1 . computeVolts ( adsB1 . readADC_SingleEnded ( 3 ) ));}

#ifndef LITE_V3
// ADS1115 @ 0x49 I2C
Adafruit_ADS1115 adsB2;

// volts for adsB2
float read_adsB2_ch0_callback() { return (adsB2 . computeVolts ( adsB2 . readADC_SingleEnded ( 0 ) ));}
float read_adsB2_ch1_callback() { return (adsB2 . computeVolts ( adsB2 . readADC_SingleEnded ( 1 ) ));}
float read_adsB2_ch2_callback() { return (adsB2 . computeVolts ( adsB2 . readADC_SingleEnded ( 2 ) ));}
float read_adsB2_ch3_callback() { return (adsB2 . computeVolts ( adsB2 . readADC_SingleEnded ( 3 ) ));}
#endif

// forward declarations
void setupADS1115();
void checkButton(void);

extern void do_lvgl_init(uint32_t );
extern void processDisplay(void);

DallasTemperatureSensors* dts;
int numberOfDevices = 3;
bool signalKConnected = false;

// unique chip id
int32_t chipId=0;

// used for N2K Messages
double AltRPM = 0;
double EngRPM = 0;
double OilPres = 0;
double AltVolts = 0;
double HouseVolts = 0;
double FuelLevel = 0;
double B1A3 = 0;
double B2A0 = 0;
double B2A1 = 0;
double B2A2 = 0;
double B2A3 = 0;
bool chkEng = 0;
bool digIn2 = 0;
double engineCoolantTemp = 0;
double engineBlockTemp = 0;
double engineRoomTemp = 0;
double engineExhaustTemp = 0;
String screenSelect;

// variables to handle the dial push button debounce etc
int buttonState;            // the current reading from the input pin
int buttonStateLong;            // the current reading from the input pin
int lastButtonState = LOW;  // the previous reading from the input pin
int shortButtonStateLatched = LOW;  // latch if button is pressed for short interval
int longButtonStateLatched = LOW;  // latch if pressed for long interval
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled 
unsigned long shortDebounceDelay = 80;    // the short debounce time
unsigned long longDebounceDelay = 1000;    // the long debounce time
unsigned long latchClearDelay = 500;    // the time to allow latched states to be consumed befor autonomous reset


// The setup function performs one-time application initialization.
void setup() {

  // seupt outputs at start
  pinMode(Out1, OUTPUT_OPEN_DRAIN); // output 1
  digitalWrite(Out1, true);

  #ifndef LITE_V3
  pinMode(Out2, OUTPUT_OPEN_DRAIN); // output 1
  digitalWrite(Out2, true);
  #endif
  // used to create a unique id
  uint8_t chipid [ 6 ];

  // setup the logging infra
  SetupLogging();

  // derive a unique chip id from the burned in MAC address
  esp_efuse_mac_get_default ( chipid );
  for ( int i = 0 ; i < 6 ; i++ )
    chipId += ( chipid [ i ] << ( 7 * i ));
  
  // dump chip id at startup
  debugI("Poweron reset - Chip ID: %x" , chipId);
  debugI("Flash Size: %d bytes" , ESP.getFlashChipSize());
  
  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    #ifdef LITE_V3
                    // Set a custom hostname for the app, use "lite" for small version
                    ->set_hostname("EM V3 Lite")
                    #else
                    // Set a custom hostname for the app.
                    ->set_hostname("EM V3")
                    #endif
                    // Optionally, hard-code the WiFi and Signal K server
                    // ->set_wifi("ssid","password")
                    // settings. This is normally not needed.
                    //->set_sk_server("ip address", port#)
                    ->enable_ota("1Qwerty!")
                    ->set_button_pin(0)
                    ->get_app();

// for debounce init
pinMode ( In2 , INPUT_PULLDOWN );
lastButtonState = digitalRead(In2);
buttonState = lastButtonState;
buttonStateLong = lastButtonState;

// constant for use as screen control via webui
const char *kscreenSelectSKPath = "/misc/screen/select/sk_path";

auto screenSelector = new StringConstantSensor("1", 1, "/screen/select");
ConfigItem(screenSelector)
      ->set_title("Screen Selector");

// init to 1 always
screenSelector->set("1");

screenSelector 
  ->connect_to(new SKOutputString(
            "/misc/screen/select/sk_path",         // Signal K path
            kscreenSelectSKPath,        
                                                    // web UI and for storing the
                                                    // configuration
            new SKMetadata("Screen #",                     // Define output units
                          "Screen Number as string")))  // Value description
  ->connect_to(
          new LambdaConsumer<String>([](String AltScreenSelect) {
            screenSelect = AltScreenSelect;
          }));
    
  //************* ads1115 I2C section **************************************************
  // setup ads1115 on I2C bus, ads1115 is 16bit a/d. Raw inputs should not exceed 3.3V
  // Bank 1 (B1) has resitor divider networks on each input to allow for a maximum input voltage
  // of 18V. Vchip = (3.3K/(14.7K + 3.3K)) * Vin. @18V Vin = 3.3V Vchip
  // Bank 2 (B2) (non Lite only) has resitor divider networks to allow for a maximum input voltage
  // of 5V. Vchip = (3.3K/(1.7K + 3.3K)) * Vin. @5V Vin = 3.3V Vchip
  setupADS1115();

  // Create a new sensor for adsB1_A0
  const char *kAlternatorVoltageLinearConfigPath = "/propulsion/alternator voltage/calibrate";
  const char *kAlternatorVoltageSKPath = "/propulsion/alternator voltage/sk_path";
  const float AltVmultiplier = 5.64; // resitor divider ratio, measured
  const float AltVoffset = 0;

  auto analog_input_adsB1_0 = new RepeatSensor<float>(
      500, read_adsB1_ch0_callback);

  auto analog_input_adsB1_0_linear = new Linear(AltVmultiplier, AltVoffset, kAlternatorVoltageLinearConfigPath);
  ConfigItem(analog_input_adsB1_0_linear)
        ->set_title("Alternator Voltage Scale");

  analog_input_adsB1_0
      ->connect_to(analog_input_adsB1_0_linear)
      
      ->connect_to(new SKOutputFloat(
          "/propulsion/alternator voltage/sk_path",         // Signal K path
          kAlternatorVoltageSKPath,        
                                                  // web UI and for storing the
                                                  // configuration
          new SKMetadata("Volts",                     // Define output units
                        "Alternator Voltage")))  // Value description

      // connect for n2k purposes
      ->connect_to(
        new LambdaConsumer<float>([](float AltVoltsValue) {
          AltVolts = AltVoltsValue;
        }));                     

      #ifdef SERIALDEBUG
      // debug - add an observer that prints out the current value of the analog input
      // every time it changes.
      analog_input_adsB1_0->attach([analog_input_adsB1_0]() {
        debugD("Analog input adsB1_0 value: %f Raw Volts", analog_input_adsB1_0->get());
      });
      #endif

  // Create a new sensor for adsB1 channel 1
  const char *kHouseVoltageLinearConfigPath = "/electrical/house voltage/calibrate";
  const char *kHouseVoltageSKPath = "/electrical/house voltage/sk_path";
  const float HouseVmultiplier = 5.64; // resitor divider ratio, measured
  const float HouseVoffset = 0;

  auto analog_input_adsB1_1 = new RepeatSensor<float>(
      500, read_adsB1_ch1_callback);

  auto analog_input_adsB1_1_linear = new Linear(HouseVmultiplier, HouseVoffset, kHouseVoltageLinearConfigPath);
  ConfigItem(analog_input_adsB1_1_linear)
        ->set_title("House Voltage Scale");

  analog_input_adsB1_1    
      ->connect_to(analog_input_adsB1_1_linear)
  
      ->connect_to(new SKOutputFloat(
          "/electrical/house voltage/voltage",         // Signal K path
          kHouseVoltageSKPath,        
                                                  // web UI and for storing the
                                                  // configuration
          new SKMetadata("Volts",                     // Define output units
                        "House Battery Voltage")))  // Value description
      // connect for n2k purposes
      ->connect_to(
        new LambdaConsumer<float>([](float HouseVoltsValue) {
          HouseVolts = HouseVoltsValue;
        }));                     

      // debug - add an observer that prints out the current value of the analog input
      // every time it changes.
      analog_input_adsB1_1->attach([analog_input_adsB1_1]() {
        debugD("Analog input adsB1_1 value: %f Raw Volts", analog_input_adsB1_1->get());
      });


  // Create a new sensor for adsB1 channel 2
  const char *kFuelLevelLinearConfigPath = "/vessel/tank/fuel level/calibrate/volts";
  const char *kFuelLevelLinearConfigPath1 = "/vessel/tank/fuel level/calibrate/level";
  const char *kFuelLevelSKPath = "/vessel/tank/fuel level/sk_path";
  const float FuelLevelmultiplier = 5.64; // convert analog inout to raw volts
  const float FuelLeveloffset = 0;
  auto analog_input_adsB1_2 = new RepeatSensor<float>(
      500, read_adsB1_ch2_callback);

  auto analog_input_adsB1_2_linear = new Linear(FuelLevelmultiplier, FuelLeveloffset, kFuelLevelLinearConfigPath);
  ConfigItem(analog_input_adsB1_2_linear)
        ->set_title("Fuel Tank Level Scale");

  auto analog_input_adsB1_2_linear2 = new Linear(140.0, -93.0, kFuelLevelLinearConfigPath1);
  ConfigItem(analog_input_adsB1_2_linear2)
        ->set_title("Fuel Tank Voltage Scale");

  analog_input_adsB1_2
      ->connect_to(analog_input_adsB1_2_linear) // convert to raw volts
      ->connect_to(analog_input_adsB1_2_linear2) // convert to level, 4.5V = 100%, 8.5V = 0%
      ->connect_to(new SKOutputFloat(
          "/vessel/tank/fuel/level",         // Signal K path
          kFuelLevelSKPath,        
                                                  // web UI and for storing the
                                                  // configuration
          new SKMetadata("Percent",                     // Define output units
                        "Propulsion Fuel Level")))  // Value description
      // connect for n2k purposes
      ->connect_to(
        new LambdaConsumer<float>([](float FuelLevelValue) {
          FuelLevel = FuelLevelValue;
        }));                     

      // debug - add an observer that prints out the current value of the analog input
      // every time it changes.
      analog_input_adsB1_2->attach([analog_input_adsB1_2]() {
        debugD("Analog input adsB1_2 value: %f Raw Volts", analog_input_adsB1_2->get());
      });

  // Create a new sensor for adsB1 channel 3
  const char *kAnalogInputB1A3LinearConfigPath = "/sensors/analog_input_adsB1_3/calibrate";
  const char *kAnalogInputB1A3SKPath = "/sensors/analog_input_adsB1_3/voltage/sk_path";
  const float AnalogInputB1A3multiplier = 5.64; // just read in volts for now
  const float AnalogInputB1A3offset = 0;

  auto analog_input_adsB1_3 = new RepeatSensor<float>(
      500, read_adsB1_ch3_callback);

  auto analog_input_adsB1_3_linear = new Linear(AnalogInputB1A3multiplier, AnalogInputB1A3offset, kAnalogInputB1A3LinearConfigPath);
  ConfigItem(analog_input_adsB1_3_linear)
        ->set_title("Analog input B1_3 Scale");

  // Connect the analog input to Signal K output. This will publish the
  // analog input value to the Signal K server every time it changes.
  analog_input_adsB1_3
      ->connect_to(analog_input_adsB1_3_linear)
  
      ->connect_to(new SKOutputFloat(
          "/sensors/analog_input_adsB1_3/voltage",         // Signal K path
          kAnalogInputB1A3SKPath,        
                                                  // web UI and for storing the
                                                  // configuration
          new SKMetadata("Volts",                     // Define output units
                        "Analog Input Volts")))  // Value description
      // connect for n2k purposes
      ->connect_to(
        new LambdaConsumer<float>([](float B1A3Value) {
          B1A3 = B1A3Value;
        }));                     

      // debug - add an observer that prints out the current value of the analog input
      // every time it changes.
      analog_input_adsB1_3->attach([analog_input_adsB1_3]() {
          debugD("Analog input adsB1_3 value: %f Raw Volts", analog_input_adsB1_3->get());
       });


// add second bank for non Lite version
#ifndef LITE_V3
  // Create a new sensor for adsB2 channel 0
  const char *kAnalogInputB2A0LinearConfigPath = "/sensors/analog_input_adsB2_0/calibrate";
  const char *kAnalogInputB2A0SKPath = "/sensors/analog_input_adsB2_0/voltage/sk_path";
  const float AnalogInputB2A0multiplier = 1.515; // just read in volts for now
  const float AnalogInputB2A0offset = 0;

  auto analog_input_adsB2_0 = new RepeatSensor<float>(
      500, read_adsB2_ch0_callback);

  auto analog_input_adsB2_0_linear = new Linear(AnalogInputB2A0multiplier, AnalogInputB2A0offset, kAnalogInputB2A0LinearConfigPath);
  ConfigItem(analog_input_adsB2_0_linear)
        ->set_title("Analog input B2_0 Scale");

  // Connect the analog input to Signal K output. This will publish the
  // analog input value to the Signal K server every time it changes.
  analog_input_adsB2_0
      ->connect_to(analog_input_adsB2_0_linear)
  
      ->connect_to(new SKOutputFloat(
          "/sensors/analog_input_adsB2_0/voltage",         // Signal K path
          kAnalogInputB2A0SKPath,        
                                                  // web UI and for storing the
                                                  // configuration
          new SKMetadata("Volts",                     // Define output units
                        "Analog Input Volts")))  // Value description
      // connect for n2k purposes
      ->connect_to(
        new LambdaConsumer<float>([](float B2A0Value) {
          B2A0 = B2A0Value;
        }));                     

      // debug - add an observer that prints out the current value of the analog input
      // every time it changes.
      analog_input_adsB2_0->attach([analog_input_adsB2_0]() {
        debugD("Analog input adsB2_0 value: %f Raw Volts", analog_input_adsB2_0->get());
      });

  // Create a new sensor for adsB2 channel 1
  const char *kAnalogInputB2A1LinearConfigPath = "/sensors/analog_input_adsB2_1/calibrate";
  const char *kAnalogInputB2A1SKPath = "/sensors/analog_input_adsB2_1/voltage/sk_path";
  const float AnalogInputB2A1multiplier = 1.515; // just read in volts for now
  const float AnalogInputB2A1offset = 0;

  auto analog_input_adsB2_1 = new RepeatSensor<float>(
      500, read_adsB2_ch1_callback);

  auto analog_input_adsB2_1_linear = new Linear(AnalogInputB2A1multiplier, AnalogInputB2A1offset, kAnalogInputB2A1LinearConfigPath);
  ConfigItem(analog_input_adsB2_1_linear)
        ->set_title("Analog input B2_1 Scale");

  // Connect the analog input to Signal K output. This will publish the
  // analog input value to the Signal K server every time it changes.
  analog_input_adsB2_1
      ->connect_to(analog_input_adsB2_1_linear)
  
      ->connect_to(new SKOutputFloat(
          "/sensors/analog_input_adsB2_1/voltage",         // Signal K path
          kAnalogInputB2A1SKPath,        
                                                  // web UI and for storing the
                                                  // configuration
          new SKMetadata("Volts",                     // Define output units
                        "Analog Input Volts")))  // Value description
      // connect for n2k purposes
      ->connect_to(
        new LambdaConsumer<float>([](float B2A1Value) {
          B2A1 = B2A1Value;
        }));                     

      // debug - add an observer that prints out the current value of the analog input
      // every time it changes.
      analog_input_adsB2_1->attach([analog_input_adsB2_1]() {
        debugD("Analog input adsB2_1 value: %f Raw Volts", analog_input_adsB2_1->get());
      });

  // Create a new sensor for adsB2 channel 2
  const char *kAnalogInputB2A2LinearConfigPath = "/sensors/analog_input_adsB2_2/calibrate";
  const char *kAnalogInputB2A2SKPath = "/sensors/analog_input_adsB2_2/voltage/sk_path";
  const float AnalogInputB2A2multiplier = 1.515; // just read in volts for now
  const float AnalogInputB2A2offset = 0;

  auto analog_input_adsB2_2 = new RepeatSensor<float>(
      500, read_adsB2_ch2_callback);

  auto analog_input_adsB2_2_linear = new Linear(AnalogInputB2A2multiplier, AnalogInputB2A2offset, kAnalogInputB2A2LinearConfigPath);
  ConfigItem(analog_input_adsB2_2_linear)
        ->set_title("Analog input B2_2 Scale");

  // Connect the analog input to Signal K output. This will publish the
  // analog input value to the Signal K server every time it changes.
  analog_input_adsB2_2
      ->connect_to(analog_input_adsB2_2_linear)
  
      ->connect_to(new SKOutputFloat(
          "/sensors/analog_input_adsB2_2/voltage",         // Signal K path
          kAnalogInputB2A2SKPath,        
                                                  // web UI and for storing the
                                                  // configuration
          new SKMetadata("Volts",                     // Define output units
                        "Analog Input Volts")))  // Value description
      // connect for n2k purposes
      ->connect_to(
        new LambdaConsumer<float>([](float B2A2Value) {
          B2A2 = B2A2Value;
        }));                     

      // debug - add an observer that prints out the current value of the analog input
      // every time it changes.
      analog_input_adsB2_2->attach([analog_input_adsB2_2]() {
        debugD("Analog input adsB2_2 value: %f Raw Volts", analog_input_adsB2_2->get());
      });

  // Create a new sensor for adsB2 channel 3
  const char *kAnalogInputB2A3LinearConfigPath = "/sensors/analog_input_adsB2_3/calibrate";
  const char *kAnalogInputB2A3SKPath = "/sensors/analog_input_adsB2_3/voltage/sk_path";
  const float AnalogInputB2A3multiplier = 1.515; // just read in volts for now
  const float AnalogInputB2A3offset = 0;

  auto analog_input_adsB2_3 = new RepeatSensor<float>(
      500, read_adsB2_ch3_callback);

  auto analog_input_adsB2_3_linear = new Linear(AnalogInputB2A3multiplier, AnalogInputB2A3offset, kAnalogInputB2A3LinearConfigPath);
  ConfigItem(analog_input_adsB2_3_linear)
        ->set_title("Analog input B2_3 Scale");

  // Connect the analog input to Signal K output. This will publish the
  // analog input value to the Signal K server every time it changes.
  analog_input_adsB2_3
      ->connect_to(analog_input_adsB2_3_linear)
  
      ->connect_to(new SKOutputFloat(
          "/sensors/analog_input_adsB2_3/voltage",         // Signal K path
          kAnalogInputB2A3SKPath,        
                                                  // web UI and for storing the
                                                  // configuration
          new SKMetadata("Volts",                     // Define output units
                        "Analog Input Volts")))  // Value description
      // connect for n2k purposes
      ->connect_to(
        new LambdaConsumer<float>([](float B2A3Value) {
          B2A2 = B2A3Value;
        }));                     

      // debug - add an observer that prints out the current value of the analog input
      // every time it changes.
      analog_input_adsB2_3->attach([analog_input_adsB2_3]() {
        debugD("Analog input adsB2_3 value: %f Raw Volts", analog_input_adsB2_3->get());
      });
#endif
//*****************end ads115 I2C

  // **************** engine rpm
  //const char *kEngineRPMInputConfig = "/Engine RPM/DigialInput";
  const char *kEngineRPMCalibrate = "/propulsion/engine RPM/calibrate";
  const char *kEngineRPMSKPath = "/propulsion/engine RPM/sk_path";
  const float multiplier = ((1.0/numberPoles)*60.0);
  const float engmultiplier = multiplier * (1.63/pulleyRatio);
  const unsigned int rpm_read_delay = 500;

  // setup input pin for rpm pulse
  pinMode ( Engine_RPM_Pin , INPUT_PULLUP );
  //auto* rpmSensor = new DigitalInputCounter(Engine_RPM_Pin, INPUT_PULLUP, RISING, rpm_read_delay, kEngineRPMInputConfig);
  auto rpmSensor = new DigitalInputCounter(Engine_RPM_Pin, INPUT_PULLUP, RISING, rpm_read_delay);

  auto rpmSensorFreq =new Frequency(engmultiplier, kEngineRPMCalibrate);
  ConfigItem(rpmSensorFreq)
        ->set_title("Engine RPM Frequency Scale");

  // alternator rpm converted to engine rpm converted to hz for sk
  rpmSensor
      ->connect_to(rpmSensorFreq)  // connect the output of sensor))

      ->connect_to(new SKOutputFloat(
        "/propulsion/engine RPM", 
        kEngineRPMSKPath, 
        new SKMetadata("RPM",                     // Define output units
                     "Engine RPM")))
      
      // this is how we get values out of sensesp :-)
      // connect for n2k purposes
      ->connect_to(
        new LambdaConsumer<float>([](float engRpmValue) {
          EngRPM = engRpmValue;
        }));

    // This is a relay that will activate when RPM is high enough: ie) enable a charger at higher rpm
    // Out 1 is for the relay
    const char *kEngineRPMRelayHystresisConfig = "/propulsion/engine RPM relay/config";
    auto engineRpmHystresis = new Hysteresis<float, bool>(1400, 1500, true, false, kEngineRPMRelayHystresisConfig);
    ConfigItem(engineRpmHystresis)
          ->set_title("RPM Relay Setpoint");

    pinMode(Out1, OUTPUT_OPEN_DRAIN); // output 1
    rpmSensor 
        ->connect_to(new Frequency(
          engmultiplier, kEngineRPMCalibrate))  // connect the output of sensor))
        ->connect_to(engineRpmHystresis)
        ->connect_to(new DigitalOutput(Out1));
// *************** end engine rpm

// Oil Pressure Sender Config //
// transducer has .5V - 4.5V for 0-100psi (689476 Pa).
// Analog input is 3.3V raw with a resister divider of 7.5K and 11K.
// analog input returns 0raw=0V and 4095raw=3.3V @ the pin.
// thru resister divider, .5V xducer ~= .2V at chip = .2*4095/3.3 ~= 248 (125 actual)
// 4.5V xducer ~= 1.82V at chip = 1.82*4095/3.3 ~= 2258
const char *kOilPressureADCConfigPath = "/propulsion/Engine Oil Pressure/ADC/scale";
const char *kOilPressureLinearConfigPath = "/propulsion/oil pressure/calibrate";
const char *kOilPressureSKPath = "/propulsion/oil pressure/sk_path";
const float OilPmultiplier = 228;
const float OilPoffset = -82080;

// Oil Pressure ESP32 Analog Input
auto analog_input = new AnalogInput(OIL_PRESSURE, 500, kOilPressureADCConfigPath, 4096);

auto analog_input_linear = new Linear(OilPmultiplier, OilPoffset, kOilPressureLinearConfigPath);
ConfigItem(analog_input_linear)
      ->set_title("Engine Oil Pressure Scale");

analog_input
      // scale using linear transform
      ->connect_to(analog_input_linear)
      // send to SK, value is in Pa
      ->connect_to(new SKOutputFloat("/propulsion/oil pressure", kOilPressureSKPath))
      // for N2K use, value is in Pa
      ->connect_to(
        new LambdaConsumer<float>([](float engOilPresValue) {
          OilPres = engOilPresValue;
          // if negative then make zero
          if (OilPres <= 0.0) OilPres = 0; // for some reason n2k does not like negative oil pressure??
        }));


// *********** digital inputs
// use pull down so a 1 is the input on - non inverted
// Use In1 for High Temp/Low Oil, when on (+12V) we are in alarm
const uint8_t DigInput1 = In1;
pinMode ( DigInput1 , INPUT_PULLDOWN );

// sampling interval for digital inputs
const unsigned int kDigitalInput1Interval = 250;
const char *kCheckEngineEnableSKPath = "/propulsion/check engine/enable";
const char *kCheckEngineSKPath = "/propulsion/check engine/sk_path";
// Digital input 1  - connect to high temp/low oil alarm
// when input low, no alarm
auto digitalInput1 = new RepeatSensor<bool>(
    kDigitalInput1Interval,
    [DigInput1]() { return !digitalRead(DigInput1); });
 
auto digitalInput1Enable = new Enable<bool>(true, kCheckEngineEnableSKPath);
ConfigItem(digitalInput1Enable)
      ->set_title("Check Engine Alarm Enable");

// INput 1 send out to SK
digitalInput1
  
  // bool to enable/disable the check engine
  ->connect_to(digitalInput1Enable)
  
  ->connect_to(new SKOutputBool(
      "/propulsion/check engine",          // Signal K path
      kCheckEngineSKPath,         // configuration path
      new SKMetadata("",                       // No units for boolean values
                    "Digital Input High Temp/Low Oil Pres")  // Value description
    ))
  
  ->connect_to(
    new LambdaConsumer<bool>([](bool checkEngineValue) {
      chkEng = checkEngineValue;
    }));


  // Digital input 2, sample periodically
  const unsigned int kDigitalInput2Interval = 250;
  const uint8_t DigInput2 = In2;
  pinMode ( DigInput2 , INPUT_PULLDOWN );
  const char *kDigitalInput2SKPath = "/sensors/digital in2/sk_path";
  auto digitalInput2 = new RepeatSensor<bool>(
      kDigitalInput2Interval,
      [DigInput2]() { return digitalRead(DigInput2); });

  // Input 2 send out to SK
  digitalInput2->connect_to(new SKOutputBool(
      "/sensors/digital in2",          // Signal K path
      kDigitalInput2SKPath,         // configuration path
      new SKMetadata("",                       // No units for boolean values
                     "Digital input 2 value")  // Value description
      ))

    ->connect_to(
      new LambdaConsumer<bool>([](bool digInput2Value) {
        digIn2 = digInput2Value;
      }));

// ******** end digital input

//************* start onewire temperature sensors
/*
     Tell SensESP where the sensor is connected to the board
     ESP32 pins are specified as just the X in GPIOX
  */
  uint8_t OneWirePin = ONE_WIRE_BUS;
  // Define how often SensESP should read the sensor(s) ms
  uint OneWireReadDelay = 1250;

  // start onewire
  dts = new DallasTemperatureSensors(OneWirePin);

  // Below are temperatures sampled and sent to Signal K server
  // To find valid Signal K Paths that fits your need you look at this link:
  // https://signalk.org/specification/1.4.0/doc/vesselsBranch.html


  // Measure engine temperature
  const char *kOWEngineTempConfig = "/propulsion/engine temperature/config";
  const char *kOWEngineTempLinearConfig = "/propulsion/engine temperature/linear";
  const char *kOWEngineTempSKPath = "/propulsion/engine temperature/sk_path";

  auto engineTemp =
      new OneWireTemperature(dts, OneWireReadDelay, kOWEngineTempConfig);
  ConfigItem(engineTemp)
      ->set_title("Engine Temperature Onewire cfg");

  auto engineTempLinear = new Linear(1.0, 0.0, kOWEngineTempLinearConfig);
  ConfigItem(engineTempLinear)
      ->set_title("Engine Temperature Scale");

  engineTemp
      ->connect_to(engineTempLinear)
      ->connect_to(new SKOutputFloat("/propulsion/engine temperature",
                                     kOWEngineTempSKPath))

      ->connect_to(
      new LambdaConsumer<float>([](float engineBlockTempValue) {
        engineBlockTemp = engineBlockTempValue;
      }));

  // Measure engine exhaust temperature
  const char *kOWExhaustTempConfig = "/propulsion/exhaust temperature/config";
  const char *kOWExhaustTempLinearConfig = "/propulsion/exhaust temperature/linear";
  const char *kOWExhaustTempSKPath = "/propulsion/exhaust temperature/sk_path";

  auto exhaustTemp =
      new OneWireTemperature(dts, OneWireReadDelay, kOWExhaustTempConfig);

  ConfigItem(exhaustTemp)
      ->set_title("Engine Exhaust Temperature Onewire cfg");

  auto exhaustTempLinear = new Linear(1.0, 0.0, kOWExhaustTempLinearConfig);
  ConfigItem(exhaustTempLinear)
      ->set_title("Engine Exhaust Temperature Onewire cfg");

  exhaustTemp
    ->connect_to(exhaustTempLinear)
    ->connect_to(new SKOutputFloat("/propulsion/exhaust temperature",
                                     kOWExhaustTempSKPath))

    ->connect_to(
            new LambdaConsumer<float>([](float engineExhaustTempValue) {
              engineExhaustTemp = engineExhaustTempValue;
            }));

  // Engine Compartment Temp....sensor floating in the comaprtment
  const char *kOWEngineRoomTempConfig = "/sensors/engine room temperature/config";
  const char *kOWEngineRoomTempLinearConfig = "/sensors/engine room temperature/linear";
  const char *kOWEngineRoomTempSKPath = "/sensors/engine room temperature/sk_path";

  auto engineRoom =
      new OneWireTemperature(dts, OneWireReadDelay, kOWEngineRoomTempConfig);

  ConfigItem(engineRoom)
      ->set_title("Engine Room Temperature Onewire cfg");
  
  auto engineRoomLinear = new Linear(1.0, 0.0, kOWEngineRoomTempLinearConfig);

  ConfigItem(engineRoomLinear)
      ->set_title("Engine Room Temperature Scale");

  engineRoom
      ->connect_to(engineRoomLinear)
      ->connect_to(new SKOutputFloat("/sensors/engine room temperature",
                                     kOWEngineRoomTempSKPath))

      ->connect_to(
          new LambdaConsumer<float>([](float engineRoomTempValue) {
            engineRoomTemp = engineRoomTempValue;
          }));

  // Measure engine coolant temperature
  const char *kOWCoolantTempConfig = "/propulsion/coolant temperature/config";
  const char *kOWCoolantTempLinearConfig = "/propulsion/coolant temperature/linear";
  const char *kOWCoolantTempSKPath = "/propulsion/coolant temperature/sk_path";

  auto coolantTemp =
      new OneWireTemperature(dts, OneWireReadDelay, kOWCoolantTempConfig);

  ConfigItem(coolantTemp)
      ->set_title("Engine Coolant Temperature Onewire cfg");

  auto coolantTempLinear = new Linear(1.0, 0.0, kOWCoolantTempLinearConfig);
  ConfigItem(coolantTempLinear)
      ->set_title("Engine Coolant Temperature Scale");

  coolantTemp
      ->connect_to(coolantTempLinear)
      ->connect_to(new SKOutputFloat("/propulsion/coolant temperature",
                                     kOWCoolantTempSKPath))

      ->connect_to(
      new LambdaConsumer<float>([](float engineCoolantTempValue) {
        engineCoolantTemp = engineCoolantTempValue;
      }));

  // Measure engine coolant temperature
  const char *kOWPCBTempConfig = "/sensors/PCB temperature/config";
  const char *kOWPCBTempLinearConfig = "/sensors/PCB temperature/linear";
  const char *kOWPCBTempSKPath = "/sensors/PCB temperature/sk_path";

  auto PCBTemp =
      new OneWireTemperature(dts, OneWireReadDelay, kOWPCBTempConfig);
  
  ConfigItem(PCBTemp)
      ->set_title("Engine Monitor Board Temperature Onewire cfg");
  
  auto PCBTempLinear = new Linear(1.0, 0.0, kOWPCBTempLinearConfig);
  ConfigItem(PCBTempLinear)
      ->set_title("Engine Monitor Board Temperature Scale");

  PCBTemp->connect_to(PCBTempLinear)
      ->connect_to(new SKOutputFloat("/sensors/PCB temperature",
                                     kOWPCBTempSKPath));

// ************* end onewire temp sensors

  // now start n2k
  setupN2K();

  // No need to parse the messages at every single loop iteration; 1 ms will do
  SensESPBaseApp::get_event_loop()->onRepeat(100, []() { doN2Kprocessing(); });

  #if defined (INCLUDE_TFT)
    // setup tft
    #ifndef LITE_V3
      do_lvgl_init(1); // 1 for regular, 3 for lite
    #else
      do_lvgl_init(3); // 1 for regular, 3 for lite
    #endif

    // setup an onrepeat to update the display regularly
    SensESPBaseApp::get_event_loop()->onRepeat(100, []() { processDisplay(); });
  #endif

} // end setup

void loop() { 					 
  // changed for V3
  static auto event_loop = SensESPBaseApp::get_event_loop();
  event_loop->tick();

  // see is pb is pushed
  checkButton();
} // end loop

// Initialize the ADS1115 module(s)
void setupADS1115 ( void )
{
  // ads1115 stuff
  uint16_t locBank1Present;
  uint16_t locBank2Present;

  //#if defined ( SERIALDEBUG )
  //  Serial . println ( "Getting single-ended readings from AIN0..3" );
  //  Serial . println ( "ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)" );
  //#endif

  // Set the ADC gain to 1. By design we expect input range @ the ADS1115 from 0-3.3V 
  // based on the onboard resistor values.
  //
  // Values at terminals: Bank 1 will divide 0-18V into 0-3.3V
  // Values at terminals: Bank 2 will divide 0-5V into 0-3.3V at the ADC
  //
  //                                                                   ADS1115
  //                                                                   -------
  adsB1 .setGain ( GAIN_ONE );        // 1x gain   +/- 4.096V  1 bit = 0.125mV
  if (!  adsB1  .  begin  ( ADS1115_BANK1_ADDR )) {
    debugE("Failed to initialize ADS @0x48]");
    //Serial . println ( "Failed to initialize ADS @0x48." );
    locBank1Present = 0; // flag to indicate if bank 1 is available
  } else {
  //#if defined ( SERIALDEBUG )
    debugI("Initialized ADS @0x48 - Bank 1");
    //Serial . println ( "Initialized ADS @0x48 - Bank 1" );
  //#endif
    locBank1Present = 1; // flag to indicate if bank 1 is available
  } // end if

#ifndef LITE_V3
  adsB2 .setGain ( GAIN_ONE );        // 1x gain   +/- 4.096V  1 bit = 0.125mV
  if ( ! adsB2 . begin ( ADS1115_BANK2_ADDR )) {
    debugE("Failed to initialize ADS @0x49]");
    //Serial . println ( "Failed to initialize ADS @0x49." );
    locBank2Present = 0; // flag to indicate if bank 1 is available
  } else {
  //#if defined ( SERIALDEBUG )
    debugI("Initialized ADS @0x49 - Bank 2");
  //#endif
    locBank2Present = 1; // flag to indicate if bank 1 is available
} // end if
#endif

//#if defined ( SERIALDEBUG )
debugI("ADS1115 module setup complet\n");
//  Serial . println ( "ADS1115 module setup complete." );
//#endif

} // end setupADS1115

// this debounces the dial PB and will latch both a short and long press
void checkButton(void){
  // read the state of the switch into a local variable:
  int reading = (int)digIn2; /* digitalRead(BUTTON_PIN);
 */
  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  } // end if

  if ((millis() - lastDebounceTime) > shortDebounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;

      // if we timeout on debounce then latch short button pressed
      if (buttonState == HIGH) {
        shortButtonStateLatched = HIGH;
        debugV("Button pressed, short debounce");
      } // end if
    } // end if
  } // end if

  if ((millis() - lastDebounceTime) > longDebounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonStateLong) {
      buttonStateLong = reading;

      // if we timeout on debounce then latch short button pressed
      if (buttonState == HIGH) {
        longButtonStateLatched = HIGH;
        debugV("Button pressed, long debounce");
      } // end if
    } // end if
  } // end if

  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = reading;
} // end checkButton
