# EMV3_SensespV2
<img src="https://github.com/user-attachments/assets/8523d30d-3f78-414c-9345-2c325a261c58" width="450" height="350">


This repository contains example code for an Engine Monitor based on SensEsp V3.0.0. This is meant as a demonstration of the EMV3 hardware only.
It is not necessarily a finished project but provided as an example to help you for your own engine monitor project. 

# Program Build:
Once this repository has been downloaded AND PlatformIO has updated the all the external libraries 
you will need to make some manual changes prior to building the code. All of the paths below are relative to the project root directory.

Step 1: Add lv_conf.h 

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;In the directory "src\extras" copy the file "lv_conf.h.custom" into the directory ".pio\libdeps\esp32dev" and rename it to "lv_conf.h". 


Step 2: Replace User_Setup.h

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;In the directory "src\extras" copy the file "User_Setup.h.custom" into the directory ".pio\libdeps\esp32dev\TFT_eSPI". There is already a file called "User_Setup.h" in this directory which needs to be deleted. Now rename "User_Setup.h.custom" to "User_Setup.h"


Step 3: Modify analog_reader.h

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;In the directory ".pio\libdeps\esp32dev\SensESP\src\sensesp\sensors" edit the file called "analog_reader.h" Replace the text "ADC_ATTEN_DB_12" with the text "ADC_ATTEN_DB_11". For my environment this was required for Sensesp to build. 


Note: If either the "LVGL", "TFT_eSPI" or "Sensesp" libraries change, you must redo the appropriate step above. 

# Program Use

This program is based on Senesp V3.0.0. A good reference for the Sensesp platform can be found here: https://signalk.org/SensESP/pages/getting_started/

Once the program has been downloaded, it should come up in AP (Access Point) mode. The screen will remain blank untill configured.
To configure the EMV3, connect you pc to the wifi define as "EM_V3_Lite" where X is a 8 digit unique hex value.

Note: This application is still under active development and is provided as-is.
