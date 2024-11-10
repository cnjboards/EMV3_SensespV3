#pragma once

// This contains all the code for running the "optional" tft display
#define MAXVALUES 4 // number of parameters supported

#define MAXSCREENINDEX 10 // max number of screens
#define TFTNUMVALUES_SCREEN 10 // # of values on each screen
#define HDRSPACE 20
#define USERSPACEXSTART 0
#define USERSPACEYSTART (HDRSPACE+1)
#define WIFIBARHIEGHT 4
#define WIFIBAR3WIDTH 18
#define WIFIBAR2WIDTH 12
#define WIFIBAR1WIDTH 6
#define WIFIBARXSTART (WIFIBAR3WIDTH + 5)

// Gauge Stuff
// these define the label for the axis from 0 to full scale
#define ZERO_VALUE "0"                   
#define QTR_VALUE "62"
#define HALF_VALUE "124"
#define THREEQTR_VALUE "190"
#define FULL_VALUE "250"

// indexes of various tft display values
#define tftRssiIndex 1
#define tftOilPressureIndex 2
#define tftRpmIndex 3
#define tftHbatIndex 4
#define tftSbatIndex 5
#define tftBrdTmpIndex 6
#define tftEngTmpIndex 7
#define tftEngCTmpIndex 8
#define tftSpareIndex 9

#define MAXWIFILISTENTRYLENGTH 48 // max size of string used for wifi 32 chars for name plus some extra
#define MAXWIFILISTENTRIES 20 // how many wifi networks we should handle

// forward declaration
extern void setupTFT(void);
extern void clearScreen(void);
extern void mainDisplay(void);
extern void do_lvgl_init(void);
extern void lv_example_event_1(void);

//typedef enum {
//  NONE,
//  NETWORK_SEARCHING,
//  NETWORK_CONNECTED_POPUP,
//  NETWORK_CONNECTED,
//  NETWORK_CONNECT_FAILED
//} Network_Status_t;

