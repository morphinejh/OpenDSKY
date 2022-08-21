/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// CustomDSKY Header
//    This file will override part of NeoGPS. Please refer to: NeoGPS/src/GPSport_h.h
//
// Copyright (c) 2022 by Jason Hill
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////

#define GPSport_h

#ifdef ARDUINO_AVR_NANO_EVERY

    #define gpsPort Serial1
    #define GPS_PORT_NAME "Serial1"
    #define DEBUG_PORT Serial
    
#else

    #define gpsPort Serial
    #define GPS_PORT_NAME "Serial"
    #define DEBUG_PORT Serial             
    
#endif
