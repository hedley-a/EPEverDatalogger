# ESP8266 based datalogger for EPEver Solar CC
- Thanks to  https://github.com/chickey/RS485-WiFi-EPEver


# Features:
Hardware Changes:
 *  Microchip 12F683 to time reset from sleep and reset on external input
 *  Logging: saving homeopathic amounts of data, 4 bytes hourly - 8 bytes daily  
 *  Maybe EEPROM? space for 512 days of daily data. Write endurance 10,000 days
 *  RTC memory enough for last two days worth of hourly data..maybe pop some into EEPROM
Software Changes:
 *    WiFi-GUI as AP - no LAN so direct GUI connection to mobile device - implemented 20/12/23
 *    Powersave: ESP runs once to log load and solar power: reset every 40 seconds by 12F683
 *    Epever RTC used hourly to save watt/hour figures to FRAM
 *    If load state toggled - ESP+wifi+GUI powered for 10 mins (use Epever RTC to time this?)
 *    Managed by GUI banner with timeout - reset by GUI pushbutton?
 *    Midday Load OFF - every day - 

![grafik](https://github.com/hedley-a/EPEverDatalogger/assets/30367667/bc1038a8-85d1-49f0-b62f-6d7b42dab50c.png)

![grafik](https://github.com/hedley-a/EPEverDatalogger/assets/30367667/a847ab49-01aa-47ac-b780-41ef428c1dc6.png)


#
- [![LICENSE](https://licensebuttons.net/l/by-nc-nd/4.0/88x31.png)](https://creativecommons.org/licenses/by-nc-nd/4.0/)
