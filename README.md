# EMV3_SensespV3
<img src="https://github.com/user-attachments/assets/8523d30d-3f78-414c-9345-2c325a261c58" width="450" height="350">


This repository contains example code for an Marine Engine Monitor based on SensEsp V3 and CNJBoards EMV3 hardware. EMV3 is based on a ESP32-WROOM-32 module so the software could easily be adapted to most ESP32 based designs. Schematics of the EMV3 are provided for reference. 

This code base has examples of a SensEspV3 application with:
- high speed digital input for RPM measurement
- ADS1115 analog processing
- OneWire processing
- NMEA2000 processing
- Constants used for alarm processing
- lvgl using a cheap yellow display (320 x 240)

I have tested this with a Raymarine Axiom MFD and it integrates well with the builtin engine display. The N2K portion uses standard NMEA2000 PGN's so it should work with any device that supports the following PGN's: PGN127488-Engine Rapid / RPM, PGN127489 - Engine Dynamic, Oil Pressure and temperature, PGN127505L-Fluid Level, PGN127508L-Battery Status, PGN130316L-Temperature values (or alternatively 130311L or 130312L but they are depricated

This software is a work in progress.

# Program Build:
Once this repository has been downloaded AND PlatformIO has updated the all the external libraries 
you will need to make some manual changes prior to building the code. All of the paths below are relative to the project root directory.

Step 1: Add lv_conf.h 

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;In the directory "src\extras" copy the file "lv_conf.h.extra" into the directory ".pio\libdeps\esp32dev" and rename it to "lv_conf.h". 


Step 2: Replace User_Setup.h

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;In the directory "src\extras" copy the file "User_Setup.h.extra" into the directory ".pio\libdeps\esp32dev\TFT_eSPI". There is already a file called "User_Setup.h" in this directory which needs to be deleted. Now rename "User_Setup.h.custom" to "User_Setup.h"


Step 3: Modify analog_reader.h

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;In the directory ".pio\libdeps\esp32dev\SensESP\src\sensesp\sensors" edit the file called "analog_reader.h" Replace the text "ADC_ATTEN_DB_12" with the text "ADC_ATTEN_DB_11". For my environment this was required for Sensesp to build. 


Note: If either the "LVGL", "TFT_eSPI" or "Sensesp" libraries change, you must redo the appropriate step above. 

# Program Use

This program is based on SensEspV3. A good reference for the SensEsp platform can be found here: https://signalk.org/SensESP/pages/getting_started/

Once the program has been downloaded, it should come up in AP (Access Point) mode. The screen will remain blank untill configured.
To configure the EMV3, connect you pc to the wifi define as "EM_V3_Lite".

# Update

We have moved the code to SensEsp V3.1.1. Also I added the following:

- Engine running logic. You can set the threshold for the Engine running logic based on rpm. This is used for alarm processing. I decided I want to generate alarms only when the engine is running. This only applies to low oil pressure and hi temperature alarms for now.
- Check Engine alarm, Low Oil Pressure alarm and Hi Engine Temperature alarm. The check engine alarm is derived from a digital input connected to the "engine light" on old school engine panels. You can set the thresholds for low oil and hi temp from the Web Ui. To disable, just make the oil pressure alarm setpoint very low and/or the engine temperature alarm very hi. The oil and temperature alarms are only generated when the engine is running.

Note: This application is still under active development and is provided as-is.
