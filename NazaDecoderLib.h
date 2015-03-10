/*
  DJI Naza (v1, v1 Lite, V2) data decoder library
  (c) Pawelsky 20141130
  Not for commercial use

  Refer to naza_decoder_wiring.jpg diagram for proper connection

  The RC PWM input code taken from https://www.instructables.com/id/RC-Quadrotor-Helicopter/step12/Arduino-Demo-PWM-Input/
*/

#ifndef __NAZA_DECODER_LIB_H__
#define __NAZA_DECODER_LIB_H__

    
#include <stdlib.h>         
#include <string.h>
#include <math.h>
#include <stdio.h>

// Uncomment the line below if you want to disable attitude (pitch/roll) sensing. This may be useful when you experience a conflict with another library that also uses the interrupt
// or on a board that does not support the interrupt used. Note that for Teensy boards attitude sensing will be disabled by default.
//#define ATTITUDE_SENSING_DISABLED

#define NAZA_MESSAGE_NONE    0x00
#define NAZA_MESSAGE_GPS     0x10
#define NAZA_MESSAGE_COMPASS 0x20

#if defined(__MK20DX128__) || defined(__MK20DX256__)
  #define ATTITUDE_SENSING_DISABLED
#endif

class NazaDecoderLib
{
  public:
    typedef enum { NO_FIX = 0, FIX_2D = 2, FIX_3D = 3, FIX_DGPS = 4 } fixType_t;

    NazaDecoderLib();

    unsigned char decode(int input);
    double getLat();
    double getLon();
    double getGpsAlt();
    double getSpeed();
    fixType_t getFixType();
    unsigned char getNumSat();
    double getHeadingNc();
    double getCog();
    double getGpsVsi();
    double getHdop();
    double getVdop();
    unsigned char getYear();
    unsigned char getMonth();
    unsigned char getDay();
    unsigned char getHour(); // Note that for time between 16:00 and 23:59 the hour returned from GPS module is actually 00:00 - 7:59.
    unsigned char getMinute();
    unsigned char getSecond();

  private:
    int payload[58];
    int seq;
    int cnt;
    int msgId;
    int msgLen;
    unsigned char cs1; // checksum #1
    unsigned char cs2; // checksum #2
    unsigned char magXMin;
    unsigned char magXMax;
    unsigned char magYMin;
    unsigned char magYMax;

    double lon;     // longitude in degree decimal
    double lat;     // latitude in degree decimal
    double gpsAlt;  // altitude in m (from GPS)
    double spd;     // speed in m/s
    fixType_t fix;   // fix type
    unsigned char sat;     // number of satellites
    double headingNc;// heading (not tilt compensated) in degrees
    double cog;     // course over ground
    double gpsVsi;  // vertical speed indicator (from GPS) in m/s (a.k.a. climb speed)
    double hdop;    // horizontal dilution of precision
    double vdop;    // vertical dilution of precision
    unsigned char year;
    unsigned char month;
    unsigned char day;
    unsigned char hour;
    unsigned char minute;
    unsigned char second;

    unsigned char  decodeLong(unsigned char idx, unsigned char mask);
    unsigned char  decodeShort(unsigned char idx, unsigned char mask);
    void     updateCS(int input);
};

extern NazaDecoderLib NazaDecoder;

#endif // __NAZA_DECODER_LIB_H__
