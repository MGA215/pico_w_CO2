# Board for CO2 sensors measurement

This project is the firmware source code for my bachelors thesis. The firmware was developed for Raspberry Pi Pico WH and is capable of measuring up to 8 CO2 sensors from the list below, acquiring and sending their data to a database and showing some useful info on a display. Some parts of the source code are not public such as used protocols and credentials.

Table of supported CO2 modules
| **Sensor** | **Company**|
|------------|------------|
| EE895      | E+E        |
| CDM7162    | Figaro     |
| SunRise    | SenseAir   |
| SunLight   | SenseAir   |
| SCD30      | Sensirion  |
| SCD41      | Sensirion  |
| CozIR-LP3  | GSS        |
| CM1107N    | Cubic      |



## How to copmile

If you don't have the protocols, use the FULL_BUILD flag in the main [CMakeLists.txt](https://github.com/MGA215/pico_w_CO2/blob/master/CMakeLists.txt) to change its value to 0, otherwise keep it on value 1. After that just compile with [Pico SDK](https://github.com/raspberrypi/pico-sdk) libraries using cmake and upload to Raspberry Pi Pico. Be aware that without configuring the EEPROM first you will not be able to change any of the inputs or internal configuration.

## How to use

Just connect the sensors to the inputs according to board's configuration and on the display you can see the measured values.

## Display

The on-board display can show some useful info. The display itself has 5 buttons labeled A to E:

| **Button** |**Function**|
|------------|------------|
| A          | Back       |
| B          | Down       |
| C          | Left       |
| D          | Right      |
| E          | Enter      |

By pressing Enter on the main page you can go to SENSORS page, where you can see sensors measured values. Change between the sensors using the Down button. By pressing Enter again you can see sensors configuration. Again, you can press Down to see more configuration items. Press Back to get back to the higher level.\
If you choose the DEBUG item in the main menu, you can change the debug info level being output via UART by presssing the Left and Right buttons. By entering this item you can choose the maximum output severity level of each individual submodule within the firmware.\
The next option GLOBAL CONFIG contains global configuration info such as device IP address and others.\
Item STATUS contains runtime info about loops, FW version and message errors.\
Item CHANNELS is used to see the internal message sending configuration. It can support up to 16 outputs and contains information about measured sensor and its measured value (pressure, CO2, humidity or temperature).\
The last item in the list can adjust brightness of the display.




