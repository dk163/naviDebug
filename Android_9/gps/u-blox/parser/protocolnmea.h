/******************************************************************************
 *
 * Copyright (C) u-blox AG
 * u-blox AG, Thalwil, Switzerland
 *
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose without fee is hereby granted, provided that this entire notice is
 * included in all copies of any software which is or includes a copy or
 * modification of this software and in all copies of the supporting
 * documentation for such software.
 *
 * THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
 * WARRANTY. IN PARTICULAR, NEITHER THE AUTHOR NOR U-BLOX MAKES ANY
 * REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY OF
 * THIS SOFTWARE OR ITS FITNESS FOR ANY PARTICULAR PURPOSE.
 *
 ******************************************************************************
 *
 * Project: libParser
 * Purpose: Library providing functions to parse u-blox GNSS receiver messages.
 *
 ******************************************************************************
 * $Id$
 * $HeadURL$
 *****************************************************************************/

#ifndef __PROTOCOLNMEA_H__
#define __PROTOCOLNMEA_H__

#include "protocol.h"

class CProtocolNMEA : public CProtocol
{
public:
  enum
  {
    NMEA_CHAR_SYNC = 36 /* '$' */,
    NMEA_MAX_SIZE = 120 /* NMEA standard specifies 82, but receivers can emit more */,
    PUBX_MAX_SIZE = 512
  };

  CProtocolNMEA();
  virtual int Parse(const unsigned char *pBuffer, int iSize);
  virtual void Process(const unsigned char *pBuffer, int iSize, CDatabase *pDatabase);
  virtual PROTOCOL_t GetType(void) { return NMEA; }

protected:
  static bool IsValidNmeaGnssType(char t);
  static void ProcessGBS(const unsigned char *pBuffer, int iSize, CDatabase *pDatabase);
  static void ProcessGGA(const unsigned char *pBuffer, int iSize, CDatabase *pDatabase);
  static void ProcessGLL(const unsigned char *pBuffer, int iSize, CDatabase *pDatabase);
  static void ProcessGNS(const unsigned char *pBuffer, int iSize, CDatabase *pDatabase);
  static void ProcessGRS(const unsigned char *pBuffer, int iSize, CDatabase *pDatabase);
  static void ProcessGSA(const unsigned char *pBuffer, int iSize, CDatabase *pDatabase);
  static void ProcessGST(const unsigned char *pBuffer, int iSize, CDatabase *pDatabase);
  static void ProcessGSV(const unsigned char *pBuffer, int iSize, CDatabase *pDatabase);
  static void ProcessRMC(const unsigned char *pBuffer, int iSize, CDatabase *pDatabase);
  static void ProcessVTG(const unsigned char *pBuffer, int iSize, CDatabase *pDatabase);
  static void ProcessZDA(const unsigned char *pBuffer, int iSize, CDatabase *pDatabase);

  static void SetLatLon(const unsigned char *pBuffer, int iSize, CDatabase *pDatabase, int ix);
  static void SetStatus(const unsigned char *pBuffer, int iSize, CDatabase *pDatabase, int ix);
  static void
  SetModeIndicator(const unsigned char *pBuffer, int iSize, CDatabase *pDatabase, int ix);
  static void CheckSetTime(const unsigned char *pBuffer, int iSize, CDatabase *pDatabase, int ix);

  static int GetItemCount(const unsigned char *pBuffer, int iSize);
  static const char *GetItem(int iIndex, const unsigned char *pBuffer, int iSize);
  static char *FindPos(int iIndex, char *pStart, const char *pEnd);
  static bool GetItem(int iIndex, const unsigned char *pBuffer, int iSize, double &dValue);
  static bool
  GetItem(int iIndex, const unsigned char *pBuffer, int iSize, int &iValue, int iBase = 10);
  static bool GetItem(int iIndex, const unsigned char *pBuffer, int iSize, char &ch);
  static bool MatchChar(const char *string, char ch, int &i);
  static double Limit360(double);
  static double CalcAngle(double d);
  static bool CalcLon(char ch, double &d);
  static bool CalcLat(char ch, double &d);
  static bool CalcTime(double d, int &h, int &m, double &s);
  static bool CalcDate(int d, int &day, int &mon, int &yr);
};

#endif //__PROTOCOLNMEA_H__
