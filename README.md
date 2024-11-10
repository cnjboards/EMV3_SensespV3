# EMV3_SensespV2
<img src="https://github.com/user-attachments/assets/8523d30d-3f78-414c-9345-2c325a261c58" width="450" height="350">


This repository contains example code for an Engine Monitor based on SensEsp V2.7.2. This is meant as a demonstration of the EMV3 hardware only.
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

This program is based on Senesp V2.7.2. A good reference for the Sensesp platform can be found here: https://signalk.org/SensESP/pages/getting_started/

Once the program has been downloaded, it should come up in AP (Access Point) mode. The screen will remain blank untill configured.
To configure the EMV3, connect you pc to the wifi define as "Configure EM_V3_Lite-X" where X is a 8 digit unique hex value. It should loo as follows:

![image-1](https://github.com/user-attachments/assets/f36207d7-4143-4799-9c55-b2eca204a633)

Use the default password "thisisfine"

The wifi configuration screen should come up in your browser automatically. If not then point a browser at 192.168.1.1.
You should see a screen like this:

![image](https://github.com/user-attachments/assets/59518381-cc77-460b-b139-a15bf02dd238)

Now you can use the setup menu to connect the EMV3 to a Wifi network OR it can also run as a standalone AP. For more information please refer to the
Sensesp documentation found here: https://signalk.org/SensESP/pages/user_interface/
