// this file contains all the hardware definitions for the EngineMonitor board v3.0

// Can bus IO pins - connected to  SN65HVD230 transciever device, using internal es32 can engine
#define ESP32_CAN_TX_PIN GPIO_NUM_5 // If you use ESP32 and do not have TX on default IO 16, uncomment this and and modify definition to match your CAN TX pin.
#define ESP32_CAN_RX_PIN GPIO_NUM_4 // If you use ESP32 and do not have RX on default IO 4, uncomment this and and modify definition to match your CAN TX pin.

// Onewire Data bus pin
#define ONE_WIRE_BUS 17 // GPIO17

// Interrupt input used to count pulse from the engine alternator
#define Engine_RPM_Pin 27 // GPIO 27

// Predefined digital IO on the board.
// Digital outputs, formC relay contacts
#define Out1 26 // Relay Output1
#define Out2 19 // Relay Output2

// Digital inputs, opto isolated, configured for 12V input
#define In1 14 // Digital Input1
#define In2 13 // Digital Input2

// input used for factory reset, soft reset
#define FACTORY_RESET 15

// SPI bus for project, shared with TFT display and Touch controller, 
// See file:
// C:\Users\dmarlin\Documents\PlatformIO\Projects\PlatformIO_EngMonitTest\.pio\libdeps\mhetesp32minikit\TFT_eSPI\User_Setup.h
// Pin definitions shown here for reference only.
//#define TFT_MISO 35
//#define TFT_MOSI 23
//#define TFT_SCLK 18
//#define TFT_CS   16  // Chip select for display
//#define TFT_DC   25  // Data Command control pin
//#define TFT_RST   2  // Reset pin (could connect to RST pin)
//#define TOUCH_CS 33  // Chip select for Touch controller

// Analog Input used for Oil Pressure sensor, direct into ESP32 IO
// Transducer output is 0-5V, need to reduce to 0-3.3V using resister divider
#define OIL_PRESSURE 34 // use GPIO34 for oil pressure

// I2C bus used to talk to ADS1115 (x2) and QWIIC expansion connector
// We will use the default for esp32 and Arduino (defined elsewhere ??)
// SDA - GPIO21
// SCL - GPIO22
// I2C Bus addresses
#define ADS1115_BANK1_ADDR 0x48
#define ADS1115_BANK2_ADDR 0x49

// Analog input using ADS1115
#define HOUSE_BATTERY_ADC 0
#define STARTER_BATTERY_ADC 2

// RPM defines. Generator RPM is measured on alternator wire "W"
// #define pulleyRatio 1.78 // Translates Alternator RPM to Engine RPM, (Crank Dia./Alt Dia.)
#define pulleyRatio 2.39 // determined empirically using the tach
#define numberPoles 6.0

// This is the HW timer interval which controls all the realtime hard processing
// of the engine monitor.
#define TIMER_INTERVAL_MS       500
