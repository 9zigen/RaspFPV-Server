#include "NazaDecoderLib.h"

NazaDecoderLib NazaDecoder;

NazaDecoderLib::NazaDecoderLib()
{
  seq = 0;
  cnt = 0;
  msgId = 0;
  msgLen = 0;
  cs1 = 0;
  cs2 = 0;
}

int NazaDecoderLib::decodeLong(unsigned char idx, unsigned char mask)
{
  union { unsigned int l; unsigned char b[4]; } val;
  for(int i = 0; i < 4; i++) val.b[i] = payload[idx + i] ^ mask;
  return val.l;
}

short int NazaDecoderLib::decodeShort(unsigned char idx, unsigned char mask)
{
  union { unsigned short int s; unsigned char b[2]; } val;
  for(int i = 0; i < 2; i++) val.b[i] = payload[idx + i] ^ mask;
  return val.s;
}

void NazaDecoderLib::updateCS(int input)
{
  cs1 += input;
  cs2 += cs1;
}

double NazaDecoderLib::getLat() { return lat; }
double NazaDecoderLib::getLon() { return lon; }
double NazaDecoderLib::getGpsAlt() { return gpsAlt; }
double NazaDecoderLib::getSpeed() { return spd; }
NazaDecoderLib::fixType_t  NazaDecoderLib::getFixType() { return fix; }
unsigned char NazaDecoderLib::getNumSat() { return sat; }
double NazaDecoderLib::getHeadingNc() { return headingNc; }
double NazaDecoderLib::getCog() { return cog; }
double NazaDecoderLib::getGpsVsi() { return gpsVsi; }
double NazaDecoderLib::getHdop() { return hdop; }
double NazaDecoderLib::getVdop() { return vdop; }
unsigned char NazaDecoderLib::getYear() { return year; }
unsigned char NazaDecoderLib::getMonth() { return month; }
unsigned char NazaDecoderLib::getDay() { return day; }
unsigned char NazaDecoderLib::getHour() { return hour; }
unsigned char NazaDecoderLib::getMinute() { return minute; }
unsigned char NazaDecoderLib::getSecond() { return second; }

unsigned char NazaDecoderLib::decode(int input)
{ 
  if((seq == 0) && (input == 0x55)) { seq++; }                                                            // header (part 1 - 0x55)
  else if((seq == 1) && (input == 0xAA)) { cs1 = 0; cs2 = 0; seq++; }                                     // header (part 2 - 0xAA) 
  else if(seq == 2) { msgId = input; updateCS(input); seq++; }                                            // message id
  else if((seq == 3) && (((msgId == 0x10) && (input == 0x3A)) ||                                          // message payload lenght (should match message id)
                         ((msgId == 0x20) && (input == 0x06)))) { msgLen = input; cnt = 0; updateCS(input); seq++; }
  else if(seq == 4) { payload[cnt++] = input; updateCS(input); if(cnt >= msgLen) { seq++; } }             // store payload in buffer
  else if((seq == 5) && (input == cs1)) { seq++; }                                                        // verify checksum #1
  else if((seq == 6) && (input == cs2)) { seq++; }                                                        // verify checksum #2
  else seq = 0;

  if(seq == 7) // all data in buffer
  {
    seq = 0;
    // Decode GPS data
    if(msgId == NAZA_MESSAGE_GPS)
    {
      unsigned char mask = payload[55];
      unsigned int time = decodeLong(0, mask);
      second = time & 0b00111111; time >>= 6;
      minute = time & 0b00111111; time >>= 6;
      hour = time & 0b00001111; time >>= 4;
      day = time & 0b00011111; time >>= 5; if(hour > 7) day++;
      month = time & 0b00001111; time >>= 4;
      year = time & 0b01111111;
      lon = (double)decodeLong(4, mask) / 10000000;
      lat = (double)decodeLong(8, mask) / 10000000;
      gpsAlt = (double)decodeLong(12, mask) / 1000;
      double nVel = (double)decodeLong(28, mask) / 100; 
      double eVel = (double)decodeLong(32, mask) / 100;
      spd = sqrt(nVel * nVel + eVel * eVel);
      cog = atan2(eVel, nVel) * 180.0 / M_PI;
      if(cog < 0) cog += 360.0;
      gpsVsi = -(double)decodeLong(36, mask) / 100;
      vdop = (double)decodeShort(42, mask) / 100;
      double ndop = (double)decodeShort(44, mask) / 100; 
      double edop = (double)decodeShort(46, mask) / 100;
      hdop = sqrt(ndop * ndop + edop * edop);
      sat  = payload[48];
      unsigned char fixType = payload[50] ^ mask;
      unsigned char fixFlags = payload[52] ^ mask;
      switch(fixType)
      {
        case 2 : fix = FIX_2D; break;
        case 3 : fix = FIX_3D; break;
        default: fix = NO_FIX; break;
      }
      if((fix != NO_FIX) && (fixFlags & 0x02)) fix = FIX_DGPS;
    }
    // Decode compass data (not tilt compensated)
    else if (msgId == NAZA_MESSAGE_COMPASS)
    {
      unsigned char mask = payload[4];
      mask = (((mask ^ (mask >> 4)) & 0x0F) | ((mask << 3) & 0xF0)) ^ (((mask & 0x01) << 3) | ((mask & 0x01) << 7)); 
      short int x = decodeShort(0, mask);
      short int y = decodeShort(2, mask);
      if(x > magXMax) magXMax = x;
      if(x < magXMin) magXMin = x;
      if(y > magYMax) magYMax = y;
      if(y < magYMin) magYMin = y;
      headingNc = -atan2(y - ((magYMax + magYMin) / 2), x - ((magXMax + magXMin) / 2)) * 180.0 / M_PI;
      if(headingNc < 0) headingNc += 360.0; 
    }
    return msgId;
  }
  else
  {
    return NAZA_MESSAGE_NONE;
  }
}