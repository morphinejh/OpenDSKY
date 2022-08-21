# CustomDSKY - Adapted for Nano Every

## Work In Progress
Make sure to get the release version for verified working functionality. Anything else may not function as you expect - make sure to read the change logs.

## Why the 'Nano Every' vs the 'Nano'
One big driving factor for me is that the Nano Every is cheaper than the Nano at this time. Other big advantages are the dedicated Serial port for USB vs I/O - this allows talking to the GPS simultaneously with the host computer,allowing for remote commands and monitoring. The Nano Every also has more programm space and more memory to add new features.

[![Link to YouTube video demo of CustomDSKY functions](https://i.ytimg.com/vi/2ct89vLxmSU/sddefault.jpg)](https://www.youtube.com/watch?v=2ct89vLxmSU)

## Available Functions

Quick Reference for verb/noun functions. See code for all available functions.

| Verb | Noun | Function |
|:-------------:|:-------------:| -----|
| 06 | 32 | Display Time from Perigee || 16 | 17 | ReadIMU Gyro |
| 16 | 18 | ReadIMU Accel |
| 16 | 19 | Read Temp Date & Time |
| 16 | 31 | Read AGC Power-on Time* |
| 16 | 35 | Monitor/Stop Timer to event |
| 16 | 36 | Read Time from RTC* |
| 16 | 37 | Read Date from RTC* |
| 16 | 38 | Read Time from GPS* |
| 16 | 39 | Read Date from GPS* |
| 16 | 43 | GPS POS & ALT |
| 16 | 44 | or V82 Monitor Orbital Parameters |
| 16 | 68 | Apollo 11 Decent & Landing |
| 16 | 87 | READ IMU WITH RANDOM 1202 ALARM |
| 16 | 98 | Play audio track, or |
| 25 | 34 | or V25N35 set/Start Timer to/from event |
| 25 | 36 | Set The Time from Keypad |
| 25 | 37 | Set The Date from Keypad |
| 26 | 36 | Set Time on RTC from GPS |
| 26 | 37 | Set Date on RTC from GPS |
| 37 | 00 | Enter P00 Idle Mode |
| 37 | 01 | Apollo 11 Launch |
| 37 | 06 | AGC Standby - "Apollo 13" simulation |

**Verb 16 can typically be replaced with Verb 06 for reading a value and not contiuously monitoring it.*

## Required Libraries
> Adafruit NeoPixel Library: https://github.com/adafruit/Adafruit_NeoPixel

> RTCLib: https://github.com/adafruit/RTClib

> Led Control: https://github.com/wayoda/LedControl

> NeoGPS: https://github.com/morphinejh/NeoGPS (adapted for Nano Every)

 
