/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// CustomDSKY - Further customized implementation of the OpenDSKY updated from code for
//    the Apollo Education Experience Project - for 'Nano Every' compatibility.
//
// Copyright (c) 2022 by Jason Hill
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// CustomDSKY - Customized implementation of the OpenDSKY interface written exclusively
//    for the Apollo Education Experience Project
//
//    https://www.apolloexperience.com
//
// Copyright (c) 2019-2021 by Bill Walker - bwalker@apolloexperience.com
//
// Based in part on original code by S&T Geotronics, Ron Diamond, Scott Pavlovec, and others
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// This program is free software: you can redistribute it and/or modify it under the terms of
// the GNU General Public License, version 3, as published by the Free Software Foundation
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with this program.
// If not, see <https://www.gnu.org/licenses/gpl-3.0.txt>.
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// PLEASE NOTE:  The following lines MUST be uncommented in the NeoGPS configuration files:
//  - #define GPS_FIX_LOCATION_DMS in file GPSfix_cfg.h
//  - #define NMEAGPS_PARSING_SCRATCHPAD in file NMEAGPS_cfg.h
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Includes
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//   Required libraries
//    - Adafruit NeoPixel
//    - Adafruit RTClib
//    - LEDcontrol (by wayoda / Eberhard Fahle)
//    - NeoGPS (by SlashDevin)
/////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <RTClib.h>

#ifdef SECONDS_PER_DAY
  #undef SECONDS_PER_DAY
#endif

#include <LedControl.h>
#include <NMEAGPS.h>
#include <Streamers.h>
#include <GPSport.h>

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Defines
/////////////////////////////////////////////////////////////////////////////////////////////////////////

#define PIXEL_PIN      6
#define RELAY_PIN      7
#define NUMPIXELS      18
#define PUSHBTN_WAIT   150
#define MP3PIN         9
#define EEPROMADDR     10
/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Externals
/////////////////////////////////////////////////////////////////////////////////////////////////////////

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
LedControl lc=LedControl(12,10,11,4);
RTC_DS1307 rtc;


/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Variables and Constants
/////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////
// GPS-related items
//////////////////////////////////////////

bool  gpsFix     = 0;
int   lat        = 0;
int   latDegrees = 0;
int   latMinutes = 0;
int   latSeconds = 0;
int   lon        = 0;
int   lonDegrees = 0;
int   lonMinutes = 0;
int   lonSeconds = 0;
int   alt        = 0;
int   spd        = 0;
int   hdg        = 0;
float range      = 0;
int   rangeFeet  = 0;
int   bearing    = 0;

int32_t  wpLatitude  = 0;
int32_t  wpLongitude = 0;
uint8_t  GPSyear     = 0;
uint8_t  GPSmonth    = 0;
uint8_t  GPSdate     = 0;
uint8_t  GPShour     = 0;
uint8_t  GPSmin      = 0;
uint8_t  GPSsec      = 0;

static NMEAGPS  gps;
static gps_fix  fix;


//////////////////////////////////////////
// IMU-repated items
//////////////////////////////////////////

const int MPU_addr = 0x69;       // I2C address of the MPU-6050
const int tCal     = -1600;      // temperature calibration constant
long  imuGyroX     = 0;
long  imuGyroY     = 0;
long  imuGyroZ     = 0;
long  imuAccelX    = 0;
long  imuAccelY    = 0;
long  imuAccelZ    = 0;
long  imuTemp      = 0;
long  temp_C;
long  temp_F;


//////////////////////////////////////////
// DSKY display-related and keyboard-related items
//////////////////////////////////////////

long Register[4];
byte digitVal[4][7];
 
byte keyVal = 20;
byte oldKey = 20;
bool fresh  = 0;

bool navActive = 0;
bool error     = 0;
byte action    = 0;

byte verb     = 0;
byte verbNew[2];
byte verbOld[2];
byte noun     = 0;
byte nounNew[2];
byte nounOld[2];
byte prog     = 0;
byte progNew[2];
byte progOld[2];
byte count    = 0;
byte mode     = 0;
byte oldMode  = 0;
bool toggle   = 0;
byte togCount = 0;
bool newAct   = 0;

bool     VNtoggle = 0;
byte     VNflash  = 0;
uint32_t VNmillis = 0;

uint32_t compActMillis = 0;
uint32_t cmdHeadMillis = 0;

bool     GPStoggle = 0;
uint32_t GPSmillis = 0;

bool setupFlag    = 0;
byte refreshInt   = 15;
byte refreshCount = 0;


//////////////////////////////////////////
// Time and timer-related items
//////////////////////////////////////////

unsigned long previousMillis = 0;

int oldSecond = 0;

DateTime now;
DateTime simNow;

TimeSpan elapsedCurr(0);
int      elapsedSec = 0;
int      elapsedMin = 0;
int      elapsedHr  = 0;

TimeSpan AGCcurrent(0);
long     AGCstart   = 0;
TimeSpan METcurrent(0);
long     METstart   = 0;
bool     METrunning = 0;

TimeSpan N34current(0);
long     N34start   = 0;
int      N34currhr  = 0;
int      N34currmin = 0;
int      N34currsec = 0;
bool     N34running = 0;

TimeSpan N35current(0);
long     N35end     = 0;
uint16_t N35days    = 0;
int      N35currhr  = 0;
int      N35currmin = 0;
int      N35currsec = 0;
bool     N35running = 0;


//////////////////////////////////////////
// Data for lunar landing simulation
//////////////////////////////////////////

const int  timePoint[]  = {   0,   9,  26, 61, 66, 70, 75, 86, 91, 95,105,116,126,136,143,150,159,164,173,195,218,220,999};
const int  alt_Point[]  = {3500,3000,2000,800,700,600,540,400,350,330,300,270,250,220,200,160,120,100, 75, 40,  8,  0,  0};
const int  vel_Point[]  = { 750, 700, 500,230,210,190,150, 90, 40, 35, 35, 15, 25, 35, 45, 65, 45, 35, 25, 25, 15,  0,  0};
const byte maxPtr       = 22;
const int  totalSeconds = 252;


//////////////////////////////////////////
// Data for Saturn V launch simulation
//////////////////////////////////////////

const int  ltimePoint[]  = {0,35,40,45, 55, 64,  94, 100, 117, 124, 154, 169, 184, 195, 196, 214, 244,  274,  304,  334,  364,  394,  424,  454,  484,  494,  514,  544,  574,  586,  587,  604,  634,  664,  694,  724,  737,  743,  999};
const int  lvel_Point[]  = {0, 0,11,33,137,496,1192,1340,2671,3123,5226,6492,8073,9068,9100,9222,9763,10424,11191,12063,13041,14133,15351,16713,18244,18725,19709,21016,22439,22690,22699,23178,23703,24252,24824,25420,25562,25567,25567};
const int  lvvelpoint[]  = {0, 0,11,33,137,414, 857, 972,1323,1542,2321,2794,3095,3307,3307,2904,2437, 1958, 1517, 1115,  937,  716,  533,  394,  308,  288,  267,  284,  266,  279,  279,  224,  129,   56,    9,  -11,    0,    0,    0};
const int  lalt_Point[]  = {0, 0, 0, 1,  2,  7,  35,  43,  72,  94, 189, 238, 324, 357, 360, 478, 610,  718,  804,  869,  915,  933,  948,  951,  969,  973,  985,  999, 1007, 1014, 1019, 1025, 1031, 1036, 1037, 1037, 1036, 1036, 1036};
const byte lmaxPtr       = 38;
const int  ltotalSeconds = 780;
const int  earthOrbitPer = 5286;
uint32_t   launchTime    = 0;
uint32_t   perigeeTime   = 0;
bool       v37n01Running = 0;
bool       v37n01Done    = 0;
TimeSpan   perigeeCurrent(0);


//////////////////////////////////////////
// Data for P06 power-on simulation
//////////////////////////////////////////

const int act22LampSeq[] = {17, 4, 16, 6, 15, 8, 13};
bool      prog06Ack      = 0;


//////////////////////////////////////////
// Misc scratchpad items for simulations
//////////////////////////////////////////

long timeDiff = 0;
long timeCalc = 0;
int  altDiff  = 0;
int  currAlt  = 0;
long velDiff  = 0;
long currVel  = 0;
int  vvelDiff = 0;
int  currVvel = 0;

int elapsedSeconds = 0;


//////////////////////////////////////////
// Audio-related items
//////////////////////////////////////////

byte audioTrack     = 1;
byte audioIndexAdj  = 0;
byte newTrack       = 0;
byte myTrack        = 0;

const byte numTracks      = 17;
const byte aud_silence    = 1;
const byte aud_alarm      = 2;
const byte aud_jfkbelieve = 3;
const byte aud_jfkchoose  = 4;
const byte aud_a8genesis  = 5;
const byte aud_a11eagle   = 6;
const byte aud_a13problem = 7;
const byte aud_a11landing = 8;


/////////////////////////////////////////////////////////////////////////////////////////////////////////
// setup - Initializes the DSKY
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {

  if (setupFlag == 0) {                      // Have we done this yet?

    pinMode(A0, INPUT);
    pinMode(A1, INPUT);
    pinMode(A2, INPUT);
    pinMode(A3, INPUT);
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, HIGH);  
    randomSeed(analogRead(A7));
    pixels.begin();

    for(int index = 0; index < 4; index++){
      lc.shutdown(index,false); 
      lc.setIntensity(index,15);
      lc.clearDisplay(index); 
    }

    Wire.begin();
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);                          // PWR_MGMT_1 register
    Wire.write(0);                             // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
    updateIMU();

    rtc.begin();    
    now      = rtc.now();
    AGCstart = now.secondstime();

    startUp();                

    cmdHeaders(300, false);

    lampit(0,0,0, 3);
    lampit(100,100,100, 16);

    #ifndef NMEAGPS_RECOGNIZE_ALL
      #error You must define NMEAGPS_RECOGNIZE_ALL in NMEAGPS_cfg.h!
    #endif

    #ifdef NMEAGPS_INTERRUPT_PROCESSING
      #error You must *NOT* define NMEAGPS_INTERRUPT_PROCESSING in NMEAGPS_cfg.h!
    #endif

    #if !defined( NMEAGPS_PARSE_GGA ) & !defined( NMEAGPS_PARSE_GLL ) & \
        !defined( NMEAGPS_PARSE_GSA ) & !defined( NMEAGPS_PARSE_GSV ) & \
        !defined( NMEAGPS_PARSE_RMC ) & !defined( NMEAGPS_PARSE_VTG ) & \
        !defined( NMEAGPS_PARSE_ZDA ) & !defined( NMEAGPS_PARSE_GST )

    #else
      if (gps.merging == NMEAGPS::NO_MERGING) {
      }
    #endif

    gpsPort.begin(9600);

    setupFlag = 1;
  } 
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// loop - polls GPSloop() and continuously checks for newly-selected activities
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {

  // checks for various stand-alone PROG codes

  // operational codes

  // audio playback codes - all reset PROG to 11 after audio starts
  if (prog == 61){ audioPlayback( aud_jfkbelieve); prog=11; }       // Plays JFK's "I believe" clip from message to congress
  if (prog == 62){ audioPlayback( aud_jfkchoose);  prog=11; }       // Plays JFK's "We choose" clip from Rice U speech
  if (prog == 68){ audioPlayback( aud_a8genesis);  prog=11; }       // Plays "Genesis" recitation from Apollo 8
  if (prog == 69){ audioPlayback( aud_a11eagle);   prog=11; }       // Plays "Eagle has landed" clip from Apollo 11
  if (prog == 70){ audioPlayback( aud_a13problem); prog=11; }       // Plays "We have a problem" clip from Apollo 13

  // checks for various modes - mostly internal functions but some can be selected
  if (mode == 0) { mode0(); }                  // read a key
  if (mode == 1) { mode1(); }                  // read a verb
  if (mode == 2) { mode2(); }                  // read a noun
  if (mode == 3) { mode3(); }                  // read a program
  if (mode == 4) { mode4(); }                  // V35 Lamp Test
  if (mode == 5) { mode5(); }                  // V36 Fresh Start or V69 Force Restart

  // toggles the toggle flag after 4 times thru the loop
  // used to determine duration of flashing lamps

  if (togCount == 4) {

    togCount = 0;

    toggle = !toggle;

  }

  togCount++;

  if (action == 3){
    togCount = 4;
  }

  // checks for various action and calls the appropriate functions
  if(action == 1)   { action1();  }            // V16N17 ReadIMU Gyro
  if(action == 2)   { action2();  compTime();} // V16N36 Read Time from RTC
  if(action == 3)   { action3();  }            // V16N43 GPS POS & ALT
  if(action == 4)   { action4();  }            // V16N87 READ IMU WITH RANDOM 1202 ALARM
  if(action == 5)   { action5();  }            // V25N36 Set The Time from Keypad
  if(action == 6)   { action6();  }            // V25N37 Set The Date from Keypad
  if(action == 8)   { action8();  }            // V16N18 ReadIMU Accel
  if(action == 9)   { action9();  }            // V16N19 Read Temp Date & Time
  if(action == 10)  { action10(); }            // V16N68 Apollo 11 Decent & Landing
  if(action == 11)  { action11(); }            // V26N36 Set Time on RTC from GPS
  if(action == 12)  { action12(); }            // V26N37 Set Date on RTC from GPS
  if(action == 13)  { action13(); }            // V16N37 Read Date from RTC
  if(action == 14)  { action14(); }            // V16N38 Read Time from GPS
  if(action == 15)  { action15(); }            // V16N39 Read Date from GPS
  if(action == 16)  { action16(); }            // V37N00 Enter P00 Idle Mode
  if(action == 17)  { action17(); }            // V16N31 Read AGC Power-on Time, or
                                               // V16N34 Monitor/Stop Timer from event, or
                                               // V16N65 Read Mission Elapsed Time (MET)
  if(action == 18)  { action18(); }            // V16N35 Monitor/Stop Timer to event
  if(action == 19)  { action19(); }            // V25N34 or V25N35 set/Start Timer to/from event
  if(action == 20)  { action20(); }            // V16N98 Play audio track, or
                                               // V21N98 select track, or
                                               // V22N98 adjust audio index
  if(action == 21)  { action21(); }            // V37N01 Apollo 11 Launch
  if(action == 22)  { action22(); }            // V37N06 AGC Standby - "Apollo 13" simulation

  // special actions that can be called normally or from within simulations
  if(action == 101) { action101();}            // V16N44 or V82 Monitor Orbital Parameters
  if(action == 102) { action102();}            // V06N32 Display Time from Perigee, or
                                               // V16N32 Monitor Time from Perigee

  GPSloop();

}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Mode Routines
/////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// mode0 - reads keyboard during time of no activity
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void mode0() {

  if (newAct == 1) {

    validateAct();

  } else {

    if(error == 1){
      flasher();
    }

    keyVal = readkb();
    processkey0();

  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// mode1 - Inputs and processes a VERB
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void mode1() {

  input_cmd( 2 );
  processkey1();

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// mode2 - Inputs and processes a NOUN
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void mode2() {

  input_cmd( 0 );
  processkey2();

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// mode3 - Inputs and processes a PROG
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void mode3() {

  if (action > 100) {

    action = 0;
    mode = 0;

  } else {

    input_cmd( 1 );
    processkey3();

  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// input_cmd - inputs key for a command, flashes the command header, and checks for errors
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void input_cmd(int cmdPosn) {

  if ( cmdHeadMillis > millis() )  cmdHeadMillis = millis();

  flashkr();
  if( error == 1 ){
    flasher();
  }

  lampit(0,0,0, cmdPosn);
  if ( millis() - cmdHeadMillis > 500 ) { 
    cmdHeadMillis = millis();                   // reset the timer
    lampit(0,150,0, cmdPosn); 
  }

  keyVal = readkb();

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// mode4 - Performs V35 Lamp Test
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void mode4() {

  cmdHeaders(200,true);

  lampit(0,150,0, 3);

  for (int index = 4; index < 18; index++) {
    if(index < 11){
      lampit(100,100,0, index);
    }
    if(index <= 12){
      lampit(100,100,100, 23-index);
    }
    delay(50);
  }

  for (int index = 0; index < 4; index++) {
    for (int indexb = 0; indexb < 6; indexb++){
      setDigits(index,indexb,8);
      delay(25);
    }
  }

  delay(1000);

  for (int count = 0; count < 4; count++) {
    VNtoggle = 1; flashVerbNoun();
    toggle = 1; flashkr();
    toggle = 1; flasher();
    delay(500);
    VNtoggle = 0; flashVerbNoun();
    toggle = 0; flashkr();
    toggle = 0; flasher();
    delay(500);
  }

  erase_display();

  delay(500);

  restore_display();

  for (int index = 3; index < 18; index++) {
    if(index != 16){
      lampit(0,0,0, index);
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// mode5 - Performs V36 Fresh Start or V69 Force Restart
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void mode5() {

  lampit(0,0,0, 16);

  for(int index = 0; index < 4; index++){

    lampit(0,0,0,index);
    lc.shutdown(index,false); 
    lc.clearDisplay(index);

  }

  if (verb == 69) {

    now        = rtc.now();
    AGCstart   = now.secondstime();
    N34currhr  = 0;
    N34currmin = 0;
    N34currsec = 0;
    N34start   = 0;
    N34running = 0;
    N35currhr  = 0;
    N35currmin = 0;
    N35currsec = 0;
    N35end     = 0;
    N35running = 0;

  }

  verb = 0;
  verbNew[0]=0;
  verbNew[1]=0;
  verbOld[0]=0;
  verbOld[1]=0;

  noun = 0;
  nounNew[0]=0;
  nounNew[1]=0;
  nounOld[0]=0;
  nounOld[1]=0;

  prog = 0;
  progNew[0]=0;
  progNew[1]=0;
  progOld[0]=0;
  progOld[1]=0;

  startUp();

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// modeSub0 - reads keyboard during execution of another function
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void modeSub0() {

  if (newAct == 1) {

    validateSubAct();

  } else {

    if(error == 1){
      flasher();
    }

    keyVal = readkb();
    processkey0();

  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// validateSubAct - validates activities available only during execution of another function
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void validateSubAct() {

  if(verb == 82) { action =  101;newAct = 0; }                                   // Monitor Orbital Params
  else if((verb ==  6) && (noun == 32)) { action = 102;newAct = 0; }             // Monitor Time from Perigee
  else { newAct = 0;action = 0; }

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Action Routines
/////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// action1 - Reads and displays gyro data from IMU
//  - V06N17 - runs only once then resets
//  - V16N17 - runs continuously until a different command is entered
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void action1() {

  refreshCount = refreshInt+1;
  readimuGyro();

  if(verb == 6) {   // prevent repeat action if V06 display single point of data
    action = 0;
  }

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// action2 - Reads and displays time data from RTC
//  - V06N36 - runs only once then resets
//  - V16N36 - runs continuously until a different command is entered
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void action2() {

  DateTime now = rtc.now();

  compTime();

  if(oldSecond < now.second()) {

    oldSecond = now.second();
    previousMillis = millis();

  } 

  setShowRegisters( now.hour(), now.minute(), (now.second()*100) + ((millis()-previousMillis)/10)%100 );      // allow space for tenths and hundredths

  if(verb == 6) {   // prevent repeat action if V06 display single point of data
    action = 0;
  }

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// action3 - Display latitude and longitude from GPS
//  - V16N43
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void action3() {

  if (gpsFix == 1) {

    compAct();
    lampit(0,0,0, 16);
    lampit(0,0,0, 8);

  } else {

    lampit(100,100,100, 16);

  }

  if (GPStoggle) {

    setShowRegisters(latDegrees,latMinutes,latSeconds);
    setDigits(0, 3, 5);

  } else {

    setShowRegisters(lonDegrees,lonMinutes,lonSeconds);
    setDigits(0, 3, 6);

  }

  setDigits(0, 2, 2);
  if (VNflash <5) { flashVerbNoun(); }

  if ((millis() - VNmillis) > 250) {

    VNmillis = millis();
    VNtoggle = !VNtoggle;
    VNflash++;

  }

  if ((millis() - GPSmillis) > 5000) {

    GPSmillis = millis();
    GPStoggle = !GPStoggle;
    VNflash   = 0;
    VNtoggle  = 0;

  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// action4 - Monitors IMU translational speed with random 1202 errors
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void action4() {

  imu_1202(); 

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// action5 - Sets time for RTC using manual keypad entry, then runs V16N36
//  - V25N36
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void action5() {

  DateTime now = rtc.now();
  int NYR = now.year();
  int NMO = now.month();
  int NDY = now.day();
  int NHR = now.hour();
  int NMI = now.minute();
  int NSE = now.second();

  NHR = editregister( NHR, NMI, NSE, 1, 0, 23 );

  NMI = editregister( NHR, NMI, NSE, 2, 0, 59 );

  NSE = editregister( NHR, NMI, NSE, 3, 0, 59 );

  rtc.adjust(DateTime(NYR,NMO,NDY,NHR,NMI,NSE));

  action = 2; 
  setVerb( 16, 1, 6);

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// action6 - Sets date for RTC using manual keypad entry, then runs V16N19
//  - V25N37
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void action6() {

  DateTime now = rtc.now();
  int NYR = now.year();
  int NMO = now.month();
  int NDY = now.day();
  int NHR = now.hour();
  int NMI = now.minute();
  int NSE = now.second();

  NMO = editregister( NMO, NDY, NYR, 1, 1, 12 );

  NDY = editregister( NMO, NDY, NYR, 2, 1, 31 );

  NYR = editregister( NMO, NDY, NYR, 3, 1900, 2199 );

  rtc.adjust(DateTime(NYR,NMO,NDY,NHR,NMI,NSE)); 

  action = 9; 
  setVerb( 16, 1, 6);
  setNoun( 19, 1, 9);

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// action8 - Reads and displays acceleration data from IMU
//  - V06N18 - runs only once then resets
//  - V16N18 - runs continuously until a different command is entered
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void action8() { 

  refreshCount = refreshInt+1;
  readimuAccel();

  if(verb == 6) {   // prevent repeat action if V06 display single point of data
    action = 0;
  }

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// action9 - Reads and displays date and time from RTC, and temperature from IMU
//  - V16N19
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void action9() {

  tempDateTime();

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// action10 - Runs simulation of Apollo 11 Descent and Landing coordinated with ground loop audio
//  - V16N68
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void action10() {

  audioPlayback(aud_a11landing);  
  lunarDecentSim();

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// action11 - Reads time from GPS and uses it to set time on RTC, then runs V16n36
//  - V26N36
//  - NOTE:  Time will be UTC
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void action11() {

  DateTime now = rtc.now();

  int NYR = now.year();
  int NMO = now.month();
  int NDY = now.day();

  GPSloop();

  int NHR = GPShour;
  int NMI = GPSmin;
  int NSE = GPSsec;

  rtc.adjust(DateTime(NYR,NMO,NDY,NHR,NMI,NSE));

  action = 2; 
  setVerb( 16, 1, 6);

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// action12 - Reads date from GPS and uses it to set date on RTC, then runs V16n19
//  - V26N37
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void action12(){

  DateTime now = rtc.now();

  int NHR = now.hour();
  int NMI = now.minute();
  int NSE = now.second();

  GPSloop();

  int NYR = GPSyear;
  int NMO = GPSmonth;
  int NDY = GPSdate;

  rtc.adjust(DateTime(NYR,NMO,NDY,NHR,NMI,NSE));

  action = 9; 
  setVerb( 16, 1, 6);
  setNoun( 19, 1, 9);

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// action13 - Reads and displays date data from RTC
//  - V06N37 - runs only once then resets
//  - V16N37 - runs continuously until a different command is entered
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void action13() {

  DateTime now = rtc.now();

  Register[1] = (now.year());
  Register[2] = (now.month());
  Register[3] = (now.day());
  set_Digits();  

  if(verb == 6) {   // prevent repeat action if V06 display single point of data
    action = 0;
  }

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// action14 - Reads and displays time data from GPS
//  - V06N38 - runs only once then resets
//  - V16N38 - runs continuously until a different command is entered
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void action14() {

  compTime();

  if(oldSecond < GPSsec) {
    oldSecond = GPSsec;
    previousMillis - millis();
  } 

  setShowRegisters(GPShour,GPSmin,GPSsec * 100 + ((millis()-previousMillis)/10)%100);

  if(verb == 6) {   // prevent repeat action if V06 display single point of data
    action = 0;
  }

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// action15 - Reads and displays date data from GPS
//  - V06N39 - runs only once then resets
//  - V16N39 - runs continuously until a different command is entered
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void action15() {

  setShowRegisters(GPSyear,GPSmonth,GPSdate);

  if(verb == 6) {   // prevent repeat action if V06 display single point of data
    action = 0;
  }

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// action16 - Stop all commands and enter idle mode
//  - V37N00 - sets PROG 00 or P00, also known as "Winnie the P00"
//  - Note:  timers V16N34 and V16N35 continue running
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void action16() {

  erase_display();
  mode   = 0;
  action = 0;
  prog   = 0;
  setDigits(0, 2, 0);
  setDigits(0, 3, 0);
  setVerb(verb,(verb/10),(verb%10));
  setNoun(0,0,0);

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// action17 - Displays Count-Up Timers
//  - V06N31 / V16N31 - Time since DSKY power-up (or V69)
//  - V16N34 - Time from event
//  - V06N65 / V16N65 - MET - Mission Elapsed Time (started by V37N01)
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void action17() {

  switch (noun) {

    case 31:
      elapsedTimeDisplay( AGCstart );
      break;

    case 65:
      if ( METrunning ) {
         elapsedTimeDisplay( METstart );
      } else {
         setShowRegisters(0,0,0);  // allow space for tenths and hundredths
      }
      break;

    case 34:
      if (N34running) {
        elapsedTimeDisplay( N34start );

        N34currsec = elapsedSec;
        N34currmin = elapsedMin;
        N34currhr  = elapsedHr;

        keyVal = readkb();
        if(keyVal == 17) { N34running = 0; keyVal = 20; }

      } else {

        while((keyVal == 15)||(keyVal == 17)){ keyVal = readkb();}

        if ((N34currhr!=0)||(N34currmin!=0)||(N34currsec!=0)) {

          while(keyVal != 17) {

            keyVal = readkb();

            if(keyVal != oldKey) {

              oldKey = keyVal;

              if (keyVal == 17) {

                N34currhr  = 0;
                N34currmin = 0;
                N34currsec = 0;
                N34start   = 0;

              }
            }

            setShowRegisters(N34currhr,N34currmin,N34currsec * 100);  // allow space for tenths and hundredths

          }
        } else { setShowRegisters(0,0,0); }
      }

      break;

  }

  if(verb == 6) {   // prevent repeat action if V06 display single point of data
    action = 0;
  }

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void elapsedTimeDisplay( long displayTime ) {

    compTime();
    now = rtc.now();

    elapsedCurr = TimeSpan(now.secondstime() - displayTime);
    elapsedSec  = elapsedCurr.seconds();
    elapsedMin  = elapsedCurr.minutes();
    elapsedHr   = elapsedCurr.hours() + elapsedCurr.days()*24;

    setShowRegisters(elapsedHr,elapsedMin,elapsedSec * 100);      // allow space for tenths and hundredths

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// action18 - Displays/Stops the Count-Down Timer
//  - V16N35
//  - also called by V21N35 upon starting the timer
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void action18() {

  now = rtc.now();

  if (now.secondstime() >= N35end) { N35running = 0; }

  if(N35running) {

    compTime();

    N35current = TimeSpan(N35end - now.secondstime());
    N35currsec = N35current.seconds();
    N35currmin = N35current.minutes();
    N35currhr  = N35current.hours()+N35current.days()*24;

    setShowRegisters(N35currhr,N35currmin,N35currsec * 100);      // allow space for tenths and hundredths

    keyVal = readkb();

    if(keyVal == 17) { N35running = 0; keyVal = 20; }

  } else {

    N35days = 0;
    N35currhr  = 0;
    N35currmin = 0;
    N35currsec = 0;
    N35end   = 0;

    setShowRegisters(N35currhr,N35currmin,N35currsec);

  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// action19 - Sets/Starts the Count-Up and Count-Down Timers
//  - V25N34 - Sets count-up timer
//  - V25N35 - Sets count-down timer
//  - once timer is started, calls V16N34 or V16N35 depending on NOUN selected
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void action19() {

  if (noun == 34) {

    N34currhr  = 0;
    N34currmin = 0;
    N34currsec = 0;
    N34start   = 0;

    setShowRegisters(N34currhr,N34currmin,N34currsec * 100);  // allow space for tenths and hundredths

    while(keyVal == 15){ keyVal = readkb(); }

    while(keyVal != 15){ keyVal = readkb(); }

    now = rtc.now();
    N34start = now.secondstime();
    N34running = 1;
    action = 17;

  } else {

    N35days    = 0;
    N35currhr  = 0;
    N35currmin = 0;
    N35currsec = 0;
    N35end     = 0;

    N35currhr  = editregister( N35currhr, N35currmin, N35currsec, 1, 0, 199 );
    N35currmin = editregister( N35currhr, N35currmin, N35currsec, 2, 0, 59 );
    N35currsec = editregister( N35currhr, N35currmin, N35currsec, 3, 0, 59 );

    while(keyVal == 15){ keyVal = readkb(); }

    while(keyVal != 15){ keyVal = readkb(); }

    now = rtc.now();

    N35days = N35currhr/24;
    N35currhr %= 24;
    N35current = TimeSpan( N35days, N35currhr, N35currmin, N35currsec);

    N35end += N35current.totalseconds()+now.secondstime();
    N35running = 1;
    action = 18;

  }

  setVerb(16,1,6);

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// action20 - 
//  - V21N98 - Enter audio track to play
//  - V22N98 - Enter adjustment to audio playback index
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void action20() {

  if (myTrack <= 2) {myTrack = 3;}             // skip the utility tracks

  setShowRegisters(myTrack, audioIndexAdj, 0);

  while(keyVal == 15){ keyVal = readkb(); }

  while(keyVal != 15){

    keyVal = readkb();

    if (verb == 16) {

      if (keyVal == 15) {

        audioPlayback( myTrack );
        action = 0;

      }

    } else {

      if ((keyVal==12)||(keyVal==13)) {

        if (verb == 21) {

          myTrack = editregister( myTrack, audioIndexAdj, 0, 1, 3, numTracks );   // don't count the utility tracks

        } else {

          audioIndexAdj = editregister( myTrack, audioIndexAdj, 0, 2, 0, numTracks-1 );

        }
      }
    }
  }

  setVerb(16,1,6);

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// action21 - Runs simulation of Apollo 11 Launch coordinated with Flight Director Loop audio
//  - V16N68
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void action21() {

  audioPlayback(9);
  saturnLaunchSim();

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// saturnLaunchSim - Simulation of Apollo 11 Saturn V Launch
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void saturnLaunchSim(){

  lampit(0,0,0,6);
  lampit(100,100,100,16);

  int oldSecond = 0;
  int flashSec  = 0;

  byte arrayPtr = 0;

  VNflash  = 0;
  VNtoggle = 0;
  VNmillis = 0;

  int tcounter2 = 0;

  v37n01Running = 1;
  v37n01Done    = 0;
  METrunning    = 0;

  action = 0;
  mode   = 0;

  setVerb(16,1,6);
  setNoun(36,3,6);
  setProg(01,0,1);

  while(elapsedSeconds < ltotalSeconds){

    if (elapsedSeconds <= 34) {

      setShowRegisters(0,0,(elapsedSeconds - 34)*100);
      compTime();

    }

    ////////////////////////
    //    Set P02         //
    ////////////////////////
    if (elapsedSeconds > 17 && elapsedSeconds < 21) {

      lampit(  0,  0,0,16);
      lampit(100,100,0, 6);

      setProg(02,0,2);

      if (elapsedSeconds < 20) {

        setVerb(37,3,7);
        setNoun(02,0,2);

        if (VNflash <7) { flashVerbNoun(); }

        if ((millis() - VNmillis) > 250) {

          VNmillis = millis();
          VNtoggle = !VNtoggle;
          VNflash++;

        }

      } else {

        VNtoggle = 0;
        VNflash = 0;
        flashVerbNoun();
        setVerb(16,1,6);
        setNoun(36,3,6);

      }
    }

    ////////////////////////
    //    Set P11 V06N62  //
    ////////////////////////
    if ((elapsedSeconds > 34) && (elapsedSeconds < 37)) {

      if ( !METrunning ) {

        now = rtc.now();
        METstart = now.secondstime();
        METrunning = 1;

      }

      setVerb(06,0,6);
      setNoun(62,6,2);
      setProg(11,1,1);

      if (elapsedSeconds < 36) {

        if (VNflash <7) { flashVerbNoun(); }

        if ((millis() - VNmillis) > 250) {

          VNmillis = millis();
          VNtoggle = !VNtoggle;
          VNflash++;

        }

      } else {

        VNtoggle = 0;
        VNflash = 0;
        flashVerbNoun();
        launchTime  = simNow.secondstime() - 2;
        perigeeTime = launchTime + ltotalSeconds - 34 + ( earthOrbitPer / 2 );
        v37n01Done  = 1;

      }
    }

    /////////////////////////
    //  Main body of loop  //
    /////////////////////////

    if((elapsedSeconds > 34) && (elapsedSeconds < ltotalSeconds)) {

      timeDiff = ltimePoint[arrayPtr+1] - ltimePoint[arrayPtr];
      velDiff  = lvel_Point[arrayPtr+1] - lvel_Point[arrayPtr];
      vvelDiff = lvvelpoint[arrayPtr+1] - lvvelpoint[arrayPtr];
      altDiff  = lalt_Point[arrayPtr+1] - lalt_Point[arrayPtr];

      timeCalc = (elapsedSeconds - ltimePoint[arrayPtr]);

      currVel  = lvel_Point[arrayPtr] + (velDiff  * timeCalc / timeDiff);
      currVvel = lvvelpoint[arrayPtr] + (vvelDiff * timeCalc / timeDiff);
      currAlt  = lalt_Point[arrayPtr] + (altDiff  * timeCalc / timeDiff);

      if (mode == 0) { modeSub0(); }      // read a key
      if (mode == 1) { mode1(); }         // read a verb
      if (mode == 2) { mode2(); }         // read a noun
      if (mode == 3) { mode3(); }         // read a program

      if (tcounter2 >= 2) {

        if (action == 101) {

          setVerb(16,1,6);
          setNoun(44,4,4);
          action101();

        } else if (action == 102) {

          action102();

        } else {

          if ( mode == 0 ) {

            lampit(0,0,0,14);
            setVerb(06,0,6);
            setNoun(62,6,2);

          }

          setShowRegisters(currVel,currVvel,currAlt);

        }

        tcounter2 = 0; // reset the timer

      }

      compAct();

    }

    if (togCount == 18) {

      togCount = 0;
      toggle = !toggle;

    }

    togCount++;

    simNow = rtc.now();

    if(oldSecond != simNow.second()) {

      oldSecond = simNow.second();
      elapsedSeconds++;
      tcounter2++;
      if((elapsedSeconds >= ltimePoint[arrayPtr+1]) && (arrayPtr < lmaxPtr)) {arrayPtr++;}

    }
  }    // while

  v37n01Running = 0;
  v37n01Done    = 1;

  lampit(0,0,0,6);

  action = 2;
  mode = 0;
  setVerb( 16, 1, 6);
  setNoun( 36, 3, 6);

  prog = 0;
  lc.setRow(0,2,0);
  lc.setRow(0,3,0);

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// action22 - simulates shutdown and restart sequences from movie "Apollo 13"
//  - V37N06 (was P06)
//  - accurate to movie, but may not be accurate to real DSKY
//  - press and hold PRO on V50N25 to put computer in "standby"
//  - press PRO to "restart" computer
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void action22() {

  setVerb( 50, 5, 0 );
  setNoun( 25, 2, 5 );
  setProg( 06, 0, 6 );
  setShowRegisters( 62, 0, 0 );

  prog06Ack = 0;
  keyVal    = 20;

  while( !prog06Ack ) {

    flashVerbNoun();

    if ( (millis() - VNmillis) > 250 ) {

      VNmillis = millis();
      VNtoggle = !VNtoggle;

    }

    keyVal = readkb();

    if ( keyVal != 14 ) {                                // ignore anything but PRO

      previousMillis = millis();

    } else {

      if ( (millis() - previousMillis) >= 1500 ) {       // has PRO been pressed long enough?

        prog06Ack = !prog06Ack;

      }
    }
  }

  // shutdown request confirmed

  VNtoggle = 0;
  flashVerbNoun();

  lampit(0,0,0, 16);
  lampit(100,100,100,15);
  delay(500);

  for(int index = 0; index < 4; index++){

    lampit(0,0,0,index);
    lc.shutdown(index,false); 
    lc.clearDisplay(index); 

  }

  verb = 0;verbNew[0]=0;verbNew[1]=0;verbOld[0]=0;verbOld[1]=0;
  noun = 0;nounNew[0]=0;nounNew[1]=0;nounOld[0]=0;nounOld[1]=0;
  prog = 0;progNew[0]=0;progNew[1]=0;progOld[0]=0;progOld[1]=0;

  while(keyVal != 20){ keyVal = readkb(); }                        // clear the key buffer

  while(keyVal != 14){ keyVal = readkb(); }                        // look for PRO key

  lampit(0,0,0,15);
  delay(1000);

  for (int index = 0; index < 7; index++){

    lampit(100,100,100*abs((index % 2)-1),act22LampSeq[index]);
    delay(600);

  }

  delay(2000);

  for (int index = 4; index < 18; index++){ lampit(0,0,0,index); }

  flashRstrt();

  for (int index = 0; index < 4; index++){

    lampit(0,0,0, index);
    delay(200);
    lampit(0,150,0, index);

  }

  delay(200);

  setVerb(16,1,6);
  delay(100);
  setProg(46,4,6);
  setNoun(20,2,0);

  delay(200);

  setShowRegisters(-100000,180,0);

  keyVal = 20;
  mode = 0;
  validateAct(); 

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// action101 - Simulation of orbital parameters
//  - V82    - Monitor from within launch sim
//  - V16N44 - Monitor from outside launch sim
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void action101() {

  if ( v37n01Running ) {

    long freeFall = (-3599 * long(elapsedSeconds)/long(ltotalSeconds)*long(elapsedSeconds)/long(ltotalSeconds)*long(elapsedSeconds)/long(ltotalSeconds));
    setShowRegisters( currAlt, ( lalt_Point[ lmaxPtr-8 ]-lvel_Point[ lmaxPtr ]+currVel ), ((freeFall/60)*1000) - 100 + (freeFall%60) );

  } else if ( v37n01Done ) {

    setShowRegisters( lalt_Point[ lmaxPtr ], lalt_Point[ lmaxPtr-8 ], -59159);

  } else {

    setShowRegisters(0,0,100);

  }

  lc.setRow(3,3,0b00000000);
  compAct();

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// action102 - Simulation of time from perigee
//  - V06N32 - Monitor within launch sim, then display outside sim
//  - V16N32 - Monitor outside of launch sim
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void action102() {

  if (v37n01Done) {

    now = rtc.now();

    while ( perigeeTime <= now.secondstime() ) {
      perigeeTime += earthOrbitPer;
    }

    perigeeCurrent = TimeSpan(perigeeTime - now.secondstime());
    setShowRegisters(perigeeCurrent.hours(),perigeeCurrent.minutes(),-perigeeCurrent.seconds() * 100);      // allow space for tenths and hundredths

    if( (verb == 6) && (!v37n01Running) ) {                     // prevent repeat action if V06 display single point of data
      action = 0;
    }

  } else {

    setShowRegisters(0,0,0);

  }

  compAct();

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// lunarDescentSim - Simulation of Apollo 11 Lunar Descent
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void lunarDecentSim() {

  lampit(0,0,0,  9);
  lampit(0,0,0, 10);
  lampit(0,0,0, 16);

  int elapsedSecs = 0;
  int oldSecond = 0;
  int flashsec = 0;
  int end_sec = 20;

  DateTime p64now = rtc.now();

  int tcounter2 = 0;

  byte arrayptr = 0;
  int timediff = 0;
  long timecalc = 0;
  int altdiff = 0;
  int curralt = 0;
  long veldiff = 0;
  long currvel = 0;

  int errcode = 1201;

  toggle = 0;

  while(elapsedSecs < totalSeconds){


    ////////////////////////
    //  1201/1202 Alarm   //
    ////////////////////////

    if((elapsedSecs > 8 && elapsedSecs < 20)||(elapsedSecs > 48 && elapsedSecs < 61)){

      toggle = 1;
      lampit(100,100,0,6);  

      if((elapsedSecs == 9)||(elapsedSecs == 49)) {

        for(int t=1;t<4;t++) {

          for(int d=0;d<6;d++) {

            lc.setRow(t,d,0b00000000);

          }
        }
      }

      delay(1500);

      if(elapsedSecs > 48) {

        errcode = 1202;
        end_sec  = 61;

      }

      Register[1]= errcode;
      Register[2]= errcode;
      set_Digits();

      for(int t=1;t<4;t++) {

        lc.setRow(t,0,0b00000000);
        lc.setRow(t,1,0b00000000); 

        if(t == 3){

          for(int d=0;d<6;d++) {

            lc.setRow(t,d,0b00000000);

          }
        }
      }

      delay(4000);
      toggle = 0;
      elapsedSecs = end_sec;
      flashRstrt();

    }
    ////////////////////////
    //     END alarms     //
    ////////////////////////

    ////////////////////////
    //    Set P65 or P66  //
    ////////////////////////

    if ((elapsedSecs > 76 && elapsedSecs < 81) || (elapsedSecs > 90 && elapsedSecs < 95)) {

      if (elapsedSecs < 81) {

        setProg(65,6,5);
        end_sec = 80;

      } else {

        setProg(66,6,6);
        end_sec = 94;

      }
        
      if (elapsedSecs != end_sec) {

        setDigits(0,0,0);
        setDigits(0,5,0);

        if (flashsec != elapsedSecs) {

          VNtoggle = 1; flashVerbNoun();
          delay(400);
          VNtoggle = 0; flashVerbNoun();
          flashsec = elapsedSecs;

        }

      } else {

        setDigits(0,0,1);
        setDigits(0,5,8);

      }
    }

    ////////////////////////
    //  ALT & VEL LIGHTS  //
    ////////////////////////

    if(elapsedSecs > 129 && elapsedSecs < 200){

      lampit(100,100,0,9);
      lampit(100,100,0,10);

    }
    ////////////////////////
    //   END ALT & VEL    //
    ////////////////////////

    if(toggle == 0 && elapsedSecs < (totalSeconds - 31)) {

      lampit(0,0,0,6);

      tcounter2++;

      if (tcounter2 >= 90) {

        timediff = timePoint[arrayptr+1] - timePoint[arrayptr];
        altdiff  = alt_Point[arrayptr]   - alt_Point[arrayptr+1];
        veldiff  = vel_Point[arrayptr]   - vel_Point[arrayptr+1];

        timecalc = elapsedSecs*100    - timePoint[arrayptr]*100 + ((millis() - previousMillis)/10)%100 ;
        curralt  = alt_Point[arrayptr]   - (altdiff*timecalc/(timediff*100));
        currvel  = -(vel_Point[arrayptr] - (veldiff*timecalc/(timediff*100)));
      
        setShowRegisters(elapsedSecs + 532,currvel,curralt);
        
        tcounter2 = 0;               // reset the refresh timer

      }
    }

    compAct();

    p64now = rtc.now();

    if(oldSecond != p64now.second()) {

      oldSecond = p64now.second();
      elapsedSecs++;
      if(( elapsedSecs>=timePoint[arrayptr+1] ) && ( arrayptr < maxPtr )) { arrayptr++; }
      previousMillis = millis();

    } 

  }

  lampit(0,0,0,9);
  lampit(0,0,0,10);

  action = 2;
  mode = 0;
  setVerb( 16, 1, 6);
  setNoun( 36, 3, 6);

  prog = 0;
  lc.setRow(0,2,0);
  lc.setRow(0,3,0);

} //lunardescentsim 


/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Keypad Entry Processing Routines
/////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// readkb - Read an individual key
/////////////////////////////////////////////////////////////////////////////////////////////////////////

int readkb() {

  int value_row1 = analogRead(A0);
  int value_row2 = analogRead(A1);
  int value_row3 = analogRead(A2);

  if ((value_row1 > 930)&&(value_row2 > 930)&&(value_row3 > 930)) {return 20 ;} // no key

  else if (value_row1 < 225) return 10 ; // Verb
  else if (value_row1 < 370) return 12 ; // +
  else if (value_row1 < 510) return 7 ;
  else if (value_row1 < 650) return 8 ;
  else if (value_row1 < 790) return 9 ;
  else if (value_row1 < 930) return 18 ;  // Clear
 
  else if (value_row2 < 200) return 11 ;  // Noun
  else if (value_row2 < 330) return 13 ;  // -
  else if (value_row2 < 455) return 4 ;
  else if (value_row2 < 577) return 5 ;
  else if (value_row2 < 700) return 6 ;
  else if (value_row2 < 823) return 14 ;  // Program
  else if (value_row2 < 930) return 15 ;  // Enter
 
  else if (value_row3 < 225) return 0 ; 
  else if (value_row3 < 370) return 1 ;
  else if (value_row3 < 510) return 2 ;
  else if (value_row3 < 650) return 3 ;
  else if (value_row3 < 790) return 16 ; // Key Rel
  else if (value_row3 < 930) return 17 ; // Reset

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// processkey0 - Process an individual key
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void processkey0() {

  if(keyVal != oldKey) {

    fresh = 1;
    oldKey = keyVal;

  }

  if((keyVal == 10) && (fresh == 1)){

    mode = 1;
    fresh = 0;
    byte keeper = verb;

    for(int index = 0; keeper >= 10; keeper = (keeper - 10)) {

      index ++;
      verbOld[0] = index;

    }

    for(int index = 0;keeper >= 1; keeper = (keeper - 1)) {

      index ++;
      verbOld[1] = index;

    }
  }             //verb

  if((keyVal == 11) && (fresh == 1)){

    mode = 2;
    fresh = 0;
    byte keeper = noun;

    for(int index = 0; keeper >= 10; keeper = (keeper - 10)) {

      index ++;
      nounOld[0] = index;

    }

    for(int index = 0;keeper >= 1; keeper = (keeper - 1)) {

      index ++;
      nounOld[1] = index;

    }
  }              //noun

  if((keyVal == 14) && (fresh == 1)) {

    mode = 3;
    fresh = 0;

  }              //program

  if((keyVal == 17) && (fresh == 1)) {

    error = 0;
    fresh = 0;

  }              //resrt reeor
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// processkey1 - Process a VERB entry
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void processkey1() {

  lampit(0,150,0, 2);

  if(keyVal == oldKey){

    fresh = 0;

  } else { 

    fresh = 1;
    oldKey = keyVal; 

    if((error == 1) && (keyVal == 17) && (fresh == 1)){

      error = 0;
      lampit(0,0,0, 13);
      fresh = 0;

    } //resrt reeor

    if((keyVal == 15 || keyVal == 11) && (fresh == 1)) {

      fresh = 0;
      verb = ((verbNew[0] * 10) + (verbNew[1]));

      if ( (v37n01Running && (verb != 6) && (verb != 82)) ||
          (!v37n01Running && (verb != 6) && (verb != 16) && (verb != 21) && (verb != 22) && (verb != 25) && (verb != 26) && (verb != 35) && (verb != 36) && (verb != 37) && ( verb != 69) && ( verb != 82) && ( verb != 0)) ) {
 
        error = 1;
        verb = ((verbOld[0] * 10) + verbOld[1]);

      } else {

        restore_disp2( 2 );

      }
    }  //verb

    if((keyVal == 16) && (fresh == 1)){

      mode = oldMode;
      lampit(0,0,0, 14);
      count = 0; 
      fresh = 0;

      if (verb == 0) {

        lc.setRow(0,0,0);
        lc.setRow(0,1,0);

      } else{

        setDigits(0, 0,verbOld[0]);
        setDigits(0, 1,verbOld[1]);

      }
    }  // Key Rel 

    if((keyVal == 11) && (fresh == 1)){

      mode = 2;count = 0;
      fresh = 0;

    }  //noun

    if((keyVal == 14) && (fresh == 1)){

      mode = 3;count = 0;
      fresh = 0;

    } //program

    if((keyVal < 10)&&(count < 2)) {

      verbNew[count] = keyVal;
      setDigits(0, count, keyVal);
      count++;fresh = 0;

    }

    if((keyVal == 15) && (verb == 0) && (fresh == 0) && (count == 0)){

      lc.setRow(0,0,0);
      lc.setRow(0,1,0);

    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// processkey2 - Process a NOUN entry
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void processkey2() {

  lampit(0,150,0, 0);

  if(keyVal == oldKey){

    fresh = 0;

  } else { 

    fresh = 1; 
    oldKey = keyVal; 

    if((error == 1) && (keyVal == 17) && (fresh == 1)){

      error = 0;
      lampit(0,0,0, 13); 
      fresh = 0;

    } //resrt reeor

    if((keyVal == 15 || keyVal == 10) && (fresh == 1)) {

      fresh = 0;
      noun = ((nounNew[0] * 10) + (nounNew[1]));
      fresh = 0;

      if((noun != 1)&&(noun != 6)&&(noun != 17)&&(noun != 18)&&(noun != 19)&&(noun != 31)&&(noun != 32)&&(noun != 33)&&(noun != 34)&&(noun != 35)&&(noun != 36)&&(noun != 37)&&(noun != 38)&&(noun != 39)&&(noun != 43)&&(noun != 44)&&(noun != 65)&&(noun != 68)&&(noun != 87)&&(noun != 98)&&(noun != 0)) {

        error = 1;
        noun = ((nounOld[0] * 10) + nounOld[1]);

      } else {

        restore_disp2( 0 );

      }
    }

    if((keyVal == 16) && (fresh == 1)){

      mode = oldMode;
      lampit(0,0,0, 14);
      count = 0;
      fresh = 0;

      if (noun == 0) {

        lc.setRow(0,4,0);
        lc.setRow(0,5,0);

      } else{

        setDigits(0, 4,nounOld[0]);
        setDigits(0, 5,nounOld[1]);

      }
    } 

    if((keyVal == 10) && (fresh == 1)){mode = 1;count = 0; fresh = 0;}//verb

    if((keyVal == 14) && (fresh == 1)){mode = 3;count = 0; fresh = 0;}//program

    if((keyVal < 10)&&(count < 2)) {nounNew[count] = keyVal;setDigits(0, (count + 4), keyVal);count++;}

    if(verb == 16 && noun == 87) {setProg(11,1,1);}

    if(verb == 16 && noun == 68) {setProg(64,6,4);} 

    if((keyVal == 15) && (noun == 0) && (fresh == 0) && (count == 0)){

      lc.setRow(0,4,0);
      lc.setRow(0,5,0);

    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// processkey3 - Process a PROG entry
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void processkey3() {

  lampit(0,150,0, 1);

  if(keyVal == oldKey){

    fresh = 0;

  } else { 

    fresh = 1; 
    oldKey = keyVal; 

    if((error == 1) && (keyVal == 17) && (fresh == 1)){

      error = 0;
      lampit(0,0,0, 13);
      fresh = 0;

    } //resrt reeor

    if((keyVal == 15) && (fresh == 1)) {

      prog = ((progNew[0] * 10) + (progNew[1]));
      fresh = 0;

      if((prog != 11)&&(prog != 16)&&(prog != 21)&&(prog != 35)&&(prog != 61)&&(prog != 62)&&(prog != 68)&&(prog != 69)&&(prog != 70)&&(prog != 0)) {
 
       error = 1;

      } else {

        progOld[0] = progNew[0];
        progOld[1] = progNew[1];
        restore_disp2( 1 );

      }
    }
  
    if((keyVal == 16) && (fresh == 1)){

      mode = oldMode;
      lampit(0,0,0, 14);
      count = 0;
      fresh = 0;

      if (prog == 0) {

        lc.setRow(0,2,0);
        lc.setRow(0,3,0);
      } else{

        setDigits(0, 2,progOld[0]);
        setDigits(0, 3,progOld[1]);

      }
    }  //prog 

    if((keyVal == 11) && (fresh == 1)){

      mode = 2;count = 0;
      fresh = 0;

    }  //noun

    if((keyVal == 10) && (fresh == 1)){

      mode = 1;count = 0;
      fresh = 0;

    }  //verb

    if((keyVal < 10)&&(count < 2)) {

      progNew[count] = keyVal;
      setDigits(0, (count + 2), keyVal);
      count++;

    }

    if((keyVal == 15) && (prog == 0) && (fresh == 0) && (count == 0)){

      lc.setRow(0,2,0);
      lc.setRow(0,3,0);

    }
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Display Routines
/////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// restore_display - Restores Verb/Noun/Prog displays to previous states
/////////////////////////////////////////////////////////////////////////////////////////////////////////


void restore_display() {

  verbNew[0] = verbOld[0];
  verbNew[1] = verbOld[1];
  verb = ((verbNew[0] * 10) + verbNew[1]);

  if (verb == 0) {

    lc.setRow(0,0,0);lc.setRow(0,1,0);

  } else{

    setDigits(0, 0,verbNew[0]);
    setDigits(0, 1,verbNew[1]);

  }

  if (prog == 0) {

    lc.setRow(0,2,0);lc.setRow(0,3,0);

  } else{

    setDigits(0, 2,progNew[0]);
    setDigits(0, 3,progNew[1]);

  }

  if (noun == 0) {

    lc.setRow(0,4,0);
    lc.setRow(0,5,0);

  } else{

    setDigits(0, 4,nounNew[0]);
    setDigits(0, 5,nounNew[1]);

  }

  keyVal = 20;
  mode = 0;
  validateAct(); 

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// restore_disp2 - Clears error lamps and illuminates selected command lamp (VERB, NOUN, or PROG)
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void restore_disp2( byte cmdPosn ) {

  lampit(0,0,0, 13);

  mode = 0;

  lampit(0,0,0, 14);
  lampit(0,150,0, cmdPosn);

  count = 0;
  fresh = 0;
  error = 0;

  newAct = 1;

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// erase_display - Turns off all lamps and digits
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void erase_display() {

  for (int index = 0; index < 4; index++) {

    lc.clearDisplay(index);

  }

  for (int index = 3; index < 11; index++) {

    lampit(0,0,0, index);

  }

  for (int index = 11; index < 18; index++) {

    if(index != 16){

      lampit(0,0,0, index);

    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// lampit - Sets the color and intensity of a specified NeoPixel LED
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void lampit(byte r, byte g, byte b , int lamp) {

  pixels.setPixelColor(lamp, pixels.Color(r,g,b));   // Set it the way we like it.
  pixels.show();                                     // This sends the updated pixel color to the hardware.

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// setDigits - Sets values of a single digit
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void setDigits(byte maxim, byte digit, byte value) {

  lc.setDigit(maxim,digit,value,false);

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// set_Digits - sets digits for Rows 1, 2, and 3 from values in Register array
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void set_Digits() {

  for (int indexa = 0; indexa < 4; indexa ++){

    for (int index = 0; index < 7; index++) {

      digitVal[indexa][index]=0;   
   
    } 
  }

  for (int indexa = 1; indexa < 4; indexa ++){

    if (Register[indexa] < 0) {

      Register[indexa] = (Register[indexa] - (Register[indexa] + Register[indexa]));
      digitVal[indexa][0] = 1;

    } else {

      digitVal[indexa][0] = 0;

    }

    for(int index = 0; Register[indexa] >= 100000; Register[indexa] = (Register[indexa] - 100000)) {index++;}
    for(int index = 0; Register[indexa] >= 10000; Register[indexa] = (Register[indexa] - 10000)) {index ++;  digitVal[indexa][1] = index; }
    for(int index = 0; Register[indexa] >= 1000; Register[indexa] = (Register[indexa] - 1000)) { index ++; digitVal[indexa][2] = index; }
    for(int index = 0; Register[indexa] >= 100; Register[indexa] = (Register[indexa] - 100)) { index ++; digitVal[indexa][3] = index; }
    for(int index = 0; Register[indexa] >= 10; Register[indexa] = (Register[indexa] - 10)) { index ++; digitVal[indexa][4] = index; }
    for(int index = 0; Register[indexa] >= 1; Register[indexa] = (Register[indexa] - 1)) { index ++; digitVal[indexa][5] = index; }
 
 }

  for(int index = 1; index < 4; index ++){

    for(int i=0;i<6;i++) {

      if (i == 0){

        if (digitVal[(index)][i] == 1) {

          lc.setRow(index,i,B00100100);

        } else {

          lc.setRow(index,i,B01110100);

        }

      } else {

          lc.setDigit(index,i,digitVal[index][i],false);

      }
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// setShowRegisters - stores three values in the Register array then displays them
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void setShowRegisters(long r1, long r2, long r3) {

    Register[1] = r1;
    Register[2] = r2;
    Register[3] = r3;
    set_Digits();

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// cmdHeaders - Illuminates the command header lamps VERB, NOUN, and PROG
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void cmdHeaders(int delaytime, bool blankfirst) {

  for (int index = 0; index < 3; index++){

    if (blankfirst) {

      lampit(0,0,0, index);

    }

    delay(delaytime);
    lampit(0,150,0, index);

  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// flashVerbNoun - Flashes the verb and noun digits based on the toggle setting
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void flashVerbNoun() {

  if(VNtoggle == 0) {

    lc.setIntensity(0,15);

  } else {

    lc.setIntensity(0,0);

  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// compTime - Flashes the COMP ACT lamp once per second
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void compTime() {

  if (millis() - compActMillis > 1000) { 

    compActMillis = millis();                // reset the timer
    lampit(0,150,0, 3);

  }

  if (millis() - compActMillis > 75) { lampit(0,0,0, 3); }

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// compAct - Flashes the COMP ACT lamp and UPLINK ACT lamp randomly
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void compAct() {

  int randNumb = random(10, 30);

  if ((randNumb == 15) || (randNumb == 20) || randNumb == 4 || randNumb == 17) {lampit(0,150,0,3);}
  else {lampit(0,0,0,3);}

  if (randNumb == 9 || randNumb == 17) {lampit(90,90,90,17);}
  else {lampit(0,0,0,17);}

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// flashkr - Flashes KEY REL lamp based on the toggle setting
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void flashkr() {

  if(toggle == 0) {

    lampit(100,100,100, 14);

  } else {

    lampit(0,0,0, 14);

  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// flasher - Flashes OPR ERR lamp based on the toggle setting
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void flasher() {

  if(toggle == 0) {

    lampit(100,100,100, 13);

  } else {

    lampit(0,0,0, 13);

  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// flashRstrt - Flashes RESTART lamp 4 times
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void flashRstrt() {

  for (int count = 0; count <4; count++){

    lampit(100,100,0, 7);
    delay(500);
    lampit(0,0,0, 7);
    delay(500);

  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
// misc routines 
/////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// startUp - Simulates the start-up sequence for the DSKY
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void startUp() {

  flashRstrt();

  cmdHeaders(200, true);

  for (int index = 4; index < 18; index++) {

    if(index <= 12){lampit(100,100,100, 23-index);}

    delay(50);

    if(index < 11){lampit(100,100,0, index);}

    delay(50);

  }

  for (int index = 0; index < 4; index++) {

    for (int indexb = 0; indexb < 6; indexb++){

      setDigits(index,indexb,8);
      delay(25);

    }
  }

  delay(1000);

  erase_display();

  restore_display();

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// validateAct - Validates actions for Verb/Noun selections
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void validateAct(){

  if(verb == 35) {mode = 4; newAct = 0; }                                      // Lamp Test
  else if( verb == 82) {action = 101; newAct = 0; verb = 16; noun = 44; }      // Monitor Orbital Parameters
  else if((verb == 36) || (verb == 69)) {mode = 5; newAct = 0;}                // Restart
  else if((verb ==  6) && (noun == 17)) {action =  1;newAct = 0;}              // Display single IMU Gyro capture
  else if((verb ==  6) && (noun == 18)) {action =  8;newAct = 0;}              // Display single IMU Accel capture
  else if((verb ==  6) && (noun == 31)) {action = 17;newAct = 0;}              // Display single AGC power-on time capture
  else if((verb ==  6) && (noun == 32)) {action =102;newAct = 0;}              // Display time to perigee
  else if((verb ==  6) && (noun == 36)) {action =  2;newAct = 0;}              // Display single RTC Time capture
  else if((verb ==  6) && (noun == 37)) {action = 13;newAct = 0;}              // Display single RTC Date capture
  else if((verb ==  6) && (noun == 38)) {action = 14;newAct = 0;}              // Display single GPS Time capture
  else if((verb ==  6) && (noun == 39)) {action = 15;newAct = 0;}              // Display single GPS Date capture
  else if((verb ==  6) && (noun == 65)) {action = 17;newAct = 0;}              // Display single MET Time Capture 
  else if((verb == 16) && (noun == 17)) {action =  1;newAct = 0;}              // Monitor IMU Gyro
  else if((verb == 16) && (noun == 18)) {action =  8;newAct = 0;}              // Monitor IMU Accel
  else if((verb == 16) && (noun == 19)) {action =  9;newAct = 0;}              // Monitor Date/Time/Temp
  else if((verb == 16) && (noun == 31)) {action = 17;newAct = 0;}              // Monitor AGC/DSKY power-on time 
  else if((verb == 16) && (noun == 32)) {action =102;newAct = 0;}              // Monitor time to perigee
  else if((verb == 16) && (noun == 34)) {action = 17;newAct = 0;}              // Monitor/Stop Timer from event 
  else if((verb == 16) && (noun == 35)) {action = 18;newAct = 0;}              // Monitor/Stop Timer to event 
  else if((verb == 16) && (noun == 36)) {action =  2;newAct = 0;}              // Monitor RTC Time 
  else if((verb == 16) && (noun == 37)) {action = 13;newAct = 0;}              // Monitor RTC Date 
  else if((verb == 16) && (noun == 38)) {action = 14;newAct = 0;}              // Monitor GPS Time 
  else if((verb == 16) && (noun == 39)) {action = 15;newAct = 0;}              // Monitor GPS Date 
  else if((verb == 16) && (noun == 43)) {action =  3;newAct = 0;}              // Monitor current GPS
  else if((verb == 16) && (noun == 44)) {action =101;newAct = 0;}              // Monitor Orbital Parameters
  else if((verb == 16) && (noun == 65)) {action = 17;newAct = 0;}              // Monitor Mission Elapsed Time (MET) 
  else if((verb == 16) && (noun == 68)) {action = 10;newAct = 0;prog == 64;}   // Apollo 11 Decent & Landing
  else if((verb == 16) && (noun == 87)) {action =  4;newAct = 0;}              // Display IMU With 1202
  else if((verb == 16) && (noun == 98)) {action = 20;newAct = 0;}              // Playback audio track
  else if((verb == 21) && (noun == 98)) {action = 20;newAct = 0;}              // enter audio track to play
  else if((verb == 22) && (noun == 98)) {action = 20;newAct = 0;}              // enter adjustment to audio track index
  else if((verb == 25) && (noun == 34)) {action = 19;newAct = 0;}              // Set/start Timer from event 
  else if((verb == 25) && (noun == 35)) {action = 19;newAct = 0;}              // Set/start Timer to event 
  else if((verb == 25) && (noun == 36)) {action =  5;newAct = 0;}              // set RTC time from keypad
  else if((verb == 25) && (noun == 37)) {action =  6;newAct = 0;}              // set RTC date from keypad
  else if((verb == 26) && (noun == 36)) {action = 11;newAct = 0;}              // set RTC time from GPS
  else if((verb == 26) && (noun == 37)) {action = 12;newAct = 0;}              // set RTC date from GPS
  else if((verb == 37) && (noun ==  0)) {action = 16;newAct = 0;}              // enter idle mode - P00
  else if((verb == 37) && (noun ==  1)) {action = 21;newAct = 0;}              // Apollo 11 Launch
  else if((verb == 37) && (noun ==  6)) {action = 22;newAct = 0;}              // AGC Shutdown
  else if(prog == 11) {action = 8;newAct = 0;verb = 16;noun = 33;prog=0;}      // default action
  else{newAct = 0;action = 0;}

  if (action > 0 || mode == 4) {
    VNtoggle = 0;
    flashVerbNoun();
  }

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// editregister - displays 3 values and allows editing of one selected value, returning it on completion
/////////////////////////////////////////////////////////////////////////////////////////////////////////

int editregister( int r1, int r2, int r3, int editReg, int minVal, int maxVal ) {

  int editVal;
  byte blankCounter=0;
  unsigned long blankTime=micros();

  Register[1] = r1;
  Register[2] = r2;
  Register[3] = r3;

  editVal = Register[editReg];

  while(keyVal == 15){ keyVal = readkb();}

  while(keyVal != 15){

    keyVal = readkb();

    if(keyVal != oldKey) {

      oldKey = keyVal;

      if(keyVal == 12) { editVal++; }
      if(keyVal == 13) { editVal--; } 

      if( editVal > maxVal) { editVal = minVal; }
      if( editVal < minVal) { editVal = maxVal; }

    }

    Register[1] = r1; Register[2] =  r2; Register[3] = r3;
    Register[editReg] = editVal;

    if(blankCounter==0)
      lc.clearDisplay(editReg);
    
    if( (micros()- blankTime) > 25000){
      blankCounter++;
      blankTime=micros();
      
      switch(blankCounter){
        case 2://~50ms display is off
          set_Digits();
          break;
        case 8://~200ms before display is blanked again
          blankCounter=0;
          break;
      }

      if(blankCounter>8)
        blankCounter=0;
    }

  }

  return editVal;

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// setVerb - sets VERB and verbOld to a selected value and displays the digits under VERB
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void setVerb( byte newVerb, byte verb10, byte verb01 ) {

  setDigits(0, 0, verb10);
  setDigits(0, 1, verb01);
  verbOld[0] = verb10; 
  verbOld[1] = verb01; 
  verb = newVerb; 

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// setNoun - sets NOUN and nounOld to a selected value and displays the digits under NOUN
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void setNoun( byte newNoun, byte noun10, byte noun01 ) {

  setDigits(0, 4, noun10);
  setDigits(0, 5, noun01);
  nounOld[0] = noun10; 
  nounOld[1] = noun01; 
  noun = newNoun; 

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// setProg - sets PROG and progOld to a selected value and displays the digits under PROG
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void setProg( byte newProg, byte prog10, byte prog01 ) {

  setDigits(0, 2, prog10);
  setDigits(0, 3, prog01);
  progOld[0] = prog10; 
  progOld[1] = prog01; 
  prog = newProg; 

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// updateGPS - Checks to see if GPS has a valid fix, and if so reads the GPS data
/////////////////////////////////////////////////////////////////////////////////////////////////////////

static void updateGPS( const gps_fix & fix ){

  if (fix.valid.location) {

    gpsFix = 1;

    lat        = (fix.latitude() * 100);
    latDegrees = fix.latitudeDMS.degrees;
    latMinutes = fix.latitudeDMS.minutes;
    latSeconds = fix.latitudeDMS.seconds_whole * 100 + fix.latitudeDMS.seconds_frac/10;
    if (lat < 0) { latDegrees = 0-latDegrees; }

    lon        = (fix.longitude() * 100);
    lonDegrees = fix.longitudeDMS.degrees;
    lonMinutes = fix.longitudeDMS.minutes;
    lonSeconds = fix.longitudeDMS.seconds_whole * 100 + fix.longitudeDMS.seconds_frac/10;
    if (lon < 0) { lonDegrees = 0-lonDegrees; }

    alt        = fix.altitude_ft();
    spd        = floor(fix.speed_mph());
    hdg        = fix.heading();

    GPSyear  = fix.dateTime.year;
    GPSmonth = fix.dateTime.month;
    GPSdate  = fix.dateTime.date;
    GPShour  = fix.dateTime.hours;
    GPSmin   = fix.dateTime.minutes;
    GPSsec   = fix.dateTime.seconds;

  } else {

    gpsFix = 0;                               // No valid location data yet!

  }

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// GPSloop - calls updateGPS only if the GPS is available
/////////////////////////////////////////////////////////////////////////////////////////////////////////

static void GPSloop() {

  while (gps.available( gpsPort ))
  updateGPS( gps.read() );
  delay(20);

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// updateIMU - reads the gyro, acceleration, and temperature from IMU
/////////////////////////////////////////////////////////////////////////////////////////////////////////

static void updateIMU() {

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);                           // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  #ifdef ARDUINO_AVR_NANO_EVERY
    Wire.requestFrom((uint8_t)MPU_addr,(size_t)14,true);   // request a total of 14 registers  
  #else
    Wire.requestFrom(MPU_addr,14,true);         // request a total of 14 registers
  #endif
  
  imuAccelX = Wire.read()<<8|Wire.read();     // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  imuAccelY = Wire.read()<<8|Wire.read();     // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  imuAccelZ = Wire.read()<<8|Wire.read();     // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  imuTemp  = Wire.read()<<8|Wire.read();      // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)

  imuGyroX = Wire.read()<<8|Wire.read();      // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  imuGyroY = Wire.read()<<8|Wire.read();      // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  imuGyroZ = Wire.read()<<8|Wire.read();      // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// readimuGyro - Reads and displays the IMU's rotational speed on three axes
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void readimuGyro() {

  lampit(0,0,0, 16);                          // turn off NO ATT lamp
  compAct();

  refreshCount++;

  if (refreshCount > refreshInt) { 

    refreshCount=0;                           // reset the timer

    updateIMU();

    Register[1] = imuGyroX;
    Register[2] = imuGyroY;
    Register[3] = imuGyroZ;

    set_Digits();

  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// readimuAccel - Reads and displays the IMU's translational speed on three axes
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void readimuAccel() {

  lampit(0,0,0, 16);                          // Extinguish NO ATT
  compAct();

  refreshCount++;

  if (refreshCount > refreshInt) { 

    refreshCount=0;                           // reset the timer

    updateIMU();

    Register[1] = imuAccelX;
    Register[2] = imuAccelY;
    Register[3] = imuAccelZ;

    set_Digits();

  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// tempDateTime - displays date and time from RTC, and calculates temperature from IMU
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void tempDateTime(){

  compAct();

  refreshCount++;

  if (refreshCount > refreshInt) { 

    refreshCount=0;                           // reset the timer

    updateIMU();

    temp_C = (imuTemp + tCal) * 100 / 340 + 3653;        // equation for temp in degrees C (x100 to preserve precision)
    temp_F = (temp_C * 9/5 + 3200) / 100;                // convert to degrees F

    DateTime now = rtc.now();
    byte MM[2];
    byte DD[2];
    byte HH[2];
    byte mm[2];
    MM[0] = now.month() / 10;
    MM[1] = now.month() % 10;
    DD[0] = now.day() / 10;
    DD[1] = now.day() % 10;
    HH[0] = now.hour() / 10;
    HH[1] = now.hour() % 10;
    mm[0] = now.minute() / 10;
    mm[1] = now.minute() % 10;


    setDigits(1,1,MM[0]);
    setDigits(1,2,MM[1]);
    lc.setRow(1,3,0b00000000);
    setDigits(1,4,(DD[0]));
    setDigits(1,5,DD[1]);

    setDigits(2,1,0);
    setDigits(2,2,HH[0]);
    setDigits(2,3,HH[1]);
    setDigits(2,4,(mm[0]));
    setDigits(2,5,mm[1]);

    setDigits(3,0,0);
    setDigits(3,1,0);
    setDigits(3,2,0);
    setDigits(3,3,0);
    setDigits(3,4,(temp_F / 10));
    setDigits(3,5,(temp_F % 10));

  }        
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// imu_1202 - monitors acceleration data from IMU and generates random 1202 errors with audio alarm
//  - press CLR to reset error
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void imu_1202(){

  lampit(0,0,0, 16);                          // Extinguish NO ATT
  compAct();

  int randNumb = random(10, 700); 

  if (randNumb == 121 || randNumb == 677) {

    audioPlayback(aud_alarm);  
       
    keyVal = readkb();
    Register[1]= 1202;
    Register[2]= 1202;
    set_Digits(); 

    while(keyVal != 18){

      lampit(100,100,0,6);
      keyVal = readkb();

      for(int i=1;i<4;i++) {

        lc.setRow(i,0,0b00000000);
        lc.setRow(i,1,0b00000000); 

        if(i == 3){

          for(int d=0;d<6;d++) {
            lc.setRow(i,d,0b00000000);
          }
        }
      }

      if(keyVal != oldKey) {

        for(int i=0;i<7;i++) {
          lc.setRow(1,i,0b00000000);
          lc.setRow(2,i,0b00000000); 
          lc.setRow(3,i,0b00000000); 
          lc.setRow(4,i,0b00000000); 
        }

        oldKey = keyVal;
            
      }
    }

    audioPlayback(aud_silence);  

  } else {

    lampit(0,0,0,6);                          // turn off PROG light

    readimuAccel();

  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// audioPlayback - plays selected audio clip thru MP3 player
//  - plays back on an indexed basis - can get off if Arduino is soft reset without power cycling
//  - audioIndexAdj value helps with the playback index being off
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void audioPlayback(byte playTrack) {

  newTrack = playTrack + audioIndexAdj;

  if (newTrack > numTracks) { newTrack -= numTracks; }

  if (audioTrack > numTracks) { audioTrack = 1; }

  while (audioTrack != newTrack) {
    delay(PUSHBTN_WAIT);
    pinMode(MP3PIN, INPUT);
    delay(PUSHBTN_WAIT);
    
    pinMode(MP3PIN, OUTPUT);
    digitalWrite(MP3PIN,LOW);
    delay(PUSHBTN_WAIT);
    pinMode(MP3PIN, INPUT);
    delay(PUSHBTN_WAIT);
    
    audioTrack++;

    if (audioTrack > numTracks) {

      audioTrack = 1;

    }

  }

}
