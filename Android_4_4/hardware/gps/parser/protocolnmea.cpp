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
 * $Id: protocolnmea.cpp 107929 2015-12-10 13:26:48Z fabio.robbiani $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/PRODUCTS/libParser/tags/libParser_v1.04/protocolnmea.cpp $
 *****************************************************************************/

#include "protocolnmea.h"
#include "parserbuffer.h"
#include "gpsconst.h"
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

///////////////////////////////////////////////////////////////////////////////

// filter out unresonable values
const double stdDevLimit	= 1000000.0;
const double dopLimit		= 99.9;

///////////////////////////////////////////////////////////////////////////////

CProtocolNMEA::CProtocolNMEA()
{
}

int CProtocolNMEA::Parse(const unsigned char* pBuffer, int iSize)
{
    // Start
    if (iSize == 0)
    {
        return CParserBuffer::WAIT;
    }

    if (pBuffer[0] != NMEA_CHAR_SYNC)
    {
        return CParserBuffer::NOT_FOUND;
    }

    if (iSize == 1)
    {
        return CParserBuffer::WAIT;
    }

    // Only default NMEA messages 'G' and PUBX messages 'P' are supported
    int iMaxSize;
    if (pBuffer[1] == 'G')
    {
        if (iSize == 2)
        {
            return CParserBuffer::WAIT;
        }

        if(!IsValidNmeaGnssType(pBuffer[2]))
        {
            return CParserBuffer::NOT_FOUND;
        }

        iMaxSize = NMEA_MAX_SIZE;
    }
    else if (pBuffer[1] == 'P')
    {
        iMaxSize = PUBX_MAX_SIZE;
    }
    else
    {
        return CParserBuffer::NOT_FOUND;
    }

    // NMEA prefix looks good, now look at the payload
    for (int i = 1; (i < iSize); i ++)
    {
        if (i == iMaxSize)
        {
            // Message still unknown, and max possible NMEA message size reached
            return CParserBuffer::NOT_FOUND;
        }
        else if (pBuffer[i] == '\n')
        {
            // The NMEA message is terminated with a cr lf sequence,
            // since we are tolerant we allow a lf only also
            // if it has a checksum, we have to test it
            int iAsterix = (pBuffer[i-1] == '\r') ? i - 4: i - 3;
            if ((iAsterix > 0) && (pBuffer[iAsterix] == '*'))
            {
                // the checksum consists of two hex digits (usually in upper case, but we are tolerant)
                unsigned char highNibble = pBuffer[iAsterix + 1];
                unsigned char lowNibble  = pBuffer[iAsterix + 2];
                highNibble = (highNibble >= 'A' && highNibble <= 'F') ? highNibble - 'A' + 10 :
                             (highNibble >= 'a' && highNibble <= 'f') ? highNibble - 'a' + 10 :
                             (highNibble >= '0' && highNibble <= '9') ? highNibble - '0' : 0xFF ;
                lowNibble  = (lowNibble  >= 'A' && lowNibble  <= 'F') ? lowNibble  - 'A' + 10 :
                             (lowNibble  >= 'a' && lowNibble  <= 'f') ? lowNibble  - 'a' + 10 :
                             (lowNibble  >= '0' && lowNibble  <= '9') ? lowNibble  - '0' : 0xFF ;
                if (lowNibble <= 0xF && highNibble <= 0xF)
                {
                    int calcCRC = 0;
                    for (int j = 1; j < iAsterix; j++)
                    {
                        calcCRC ^= pBuffer[j];
                    }
                    // Get the checksum of NMEA message
                    int msgCRC = (highNibble << 4) | lowNibble;

                    // Compare checksum
                    if (msgCRC != calcCRC)
                    {
                        // Checksum doesn't match
                        return CParserBuffer::NOT_FOUND;
                    }
                }
                else
                {
                    return CParserBuffer::NOT_FOUND;
                }
            }
            return i + 1; // Return NMEA message size
        }
        else if ((!isprint(pBuffer[i])) && (!isspace(pBuffer[i])))
        {
            // A message should contain printable characters only !
            // we are tolerant and tolerate spaces also
            return CParserBuffer::NOT_FOUND;
        }
    }
    return CParserBuffer::WAIT;
}

void CProtocolNMEA::Process(unsigned char* pBuffer, int iSize, CDatabase* pDatabase)
{
    if (IsValidNmeaGnssType(pBuffer[2]))
    {
        if (strncmp((char*) &pBuffer[3], "GBS", 3) == 0)		pDatabase->Process(CDatabase::MSG_NMEA_GBS, pBuffer, iSize, ProcessGBS);
        else if (strncmp((char*) &pBuffer[3], "GGA", 3) == 0)	pDatabase->Process(CDatabase::MSG_NMEA_GGA, pBuffer, iSize, ProcessGGA);
        else if (strncmp((char*) &pBuffer[3], "GLL", 3) == 0)	pDatabase->Process(CDatabase::MSG_NMEA_GLL, pBuffer, iSize, ProcessGLL);
        else if (strncmp((char*) &pBuffer[3], "GNS", 3) == 0)	pDatabase->Process(CDatabase::MSG_NMEA_GNS, pBuffer, iSize, ProcessGNS);
        else if (strncmp((char*) &pBuffer[3], "GRS", 3) == 0)	pDatabase->Process(CDatabase::MSG_NMEA_GRS, pBuffer, iSize, ProcessGRS);
        else if (strncmp((char*) &pBuffer[3], "GSA", 3) == 0)	pDatabase->Process(CDatabase::MSG_NMEA_GSA, pBuffer, iSize, ProcessGSA);
        else if (strncmp((char*) &pBuffer[3], "GST", 3) == 0)	pDatabase->Process(CDatabase::MSG_NMEA_GST, pBuffer, iSize, ProcessGST);
        else if (strncmp((char*) &pBuffer[3], "RMC", 3) == 0)	pDatabase->Process(CDatabase::MSG_NMEA_RMC, pBuffer, iSize, ProcessRMC);
        else if (strncmp((char*) &pBuffer[3], "VTG", 3) == 0)	pDatabase->Process(CDatabase::MSG_NMEA_VTG, pBuffer, iSize, ProcessVTG);
        else if (strncmp((char*) &pBuffer[3], "ZDA", 3) == 0)	pDatabase->Process(CDatabase::MSG_NMEA_ZDA, pBuffer, iSize, ProcessZDA);
    }
    pDatabase->AddNmeaMessage(pBuffer, iSize);
}

bool CProtocolNMEA::IsValidNmeaGnssType(char t)
{
    return ( t == 'P' /* GPS, SBAS, QZSS */
          || t == 'L' /* GLO */
          || t == 'A' /* GAL */
          || t == 'B' /* BDS */
          || t == 'N' /* GNSS */ );
}

void CProtocolNMEA::ProcessGBS(unsigned char* pBuffer, int iSize, CDatabase* pDatabase)
{
    // Time
    CheckSetTime(pBuffer, iSize, pDatabase, 1);
}

void CProtocolNMEA::ProcessGGA(unsigned char* pBuffer, int iSize, CDatabase* pDatabase)
{
    double d;
    int i;
    char ch;
    // Time
    CheckSetTime(pBuffer, iSize, pDatabase, 1);
    // Position
    SetLatLon(pBuffer, iSize, pDatabase, 2 /* .. 5 */);
    // NMEA: GGA quality indicator
    // '0' = Fix not available or invalid
    // '1' = GPS SPS Mode, fix valid
    // '2' = Diffetential GPS, SPS Mode, fix valid
    // '3' = GPS PPS Mode, fix valid
    // '4' = Real Time Kinematic, System used RTK with fixed integers
    // '5' = Float RTK, Satellite system used in RTK mode, floating integers
    // '6' = Estimated (dead reckoning) Mode
    // '7' = Manual Input Mode
    // '8' = Simulator Mode
    if (GetItem(6, pBuffer, iSize, i) && ((i >= 0) && (i <= 8)))
    {
        pDatabase->Set(CDatabase::DATA_FIX_QUALITY, i); // todo map >=3 to 0..2
    }

    // used SVS - Don't use as GGA output is bugged - GSA used instead
    if (GetItem(7, pBuffer, iSize, i) && (i >= 0))
    {

    }

    // horizontal DOP
    if (GetItem(8, pBuffer, iSize, d) && (d < dopLimit))
    {
        pDatabase->Set(CDatabase::DATA_HORIZONAL_DILUTION_OF_PRECISION, d);
    }

    // altitude
    if (GetItem(9, pBuffer, iSize, d) && GetItem(10, pBuffer, iSize, ch) && (toupper(ch) == 'M'))
    {
        pDatabase->Set(CDatabase::DATA_ALTITUDE_SEALEVEL_METERS, d);
        //pDatabase->Set(CDatabase::DATA_ALTITUDE_ANTENNA_SEALEVEL_METERS, d); // todo
    }

    // geodial separation
    if (GetItem(11, pBuffer, iSize, d) && GetItem(12, pBuffer, iSize, ch) && (toupper(ch) == 'M'))
    {
        pDatabase->Set(CDatabase::DATA_GEOIDAL_SEPARATION, d);
    }

    // dgps age
    if (GetItem(13, pBuffer, iSize, i) && (i >= 0))
    {
        pDatabase->Set(CDatabase::DATA_DGPS_DATA_AGE, i);
    }

    // dgps station id
    if (GetItem(14, pBuffer, iSize, i) && (i >= 0) && (i <= 1023))
    {
        pDatabase->Set(CDatabase::DATA_DIFFERENTIAL_REFERENCE_STATION_ID, i);
    }
}

void CProtocolNMEA::ProcessGLL(unsigned char* pBuffer, int iSize, CDatabase* pDatabase)
{
    // Time
    CheckSetTime(pBuffer, iSize, pDatabase, 5);
    // Position
    SetLatLon(pBuffer, iSize, pDatabase, 1 /* .. 4 */ );
    // Status
    SetStatus(pBuffer, iSize, pDatabase, 6);
    // Mode Indicator
    SetModeIndicator(pBuffer, iSize, pDatabase, 7);
}

void CProtocolNMEA::ProcessGNS(unsigned char* pBuffer, int iSize, CDatabase* pDatabase)
{
    double d;
    int i;
    // Time
    CheckSetTime(pBuffer, iSize, pDatabase, 1);
    // Position
    SetLatLon(pBuffer, iSize, pDatabase, 2 /* .. 5 */);
    // used SVS
    if (GetItem(7, pBuffer, iSize, i) && (i >= 0))
    {
        // Don't use - Need to accumulate number of SVs used in GSA message
    }
    // horizontal DOP
    if (GetItem(8, pBuffer, iSize, d) && (d < dopLimit))
        pDatabase->Set(CDatabase::DATA_HORIZONAL_DILUTION_OF_PRECISION, d);
    // altitude
    if (GetItem(9, pBuffer, iSize, d))
        pDatabase->Set(CDatabase::DATA_ALTITUDE_SEALEVEL_METERS, d);
    // geodial separation
    if (GetItem(10, pBuffer, iSize, d))
        pDatabase->Set(CDatabase::DATA_GEOIDAL_SEPARATION, d);
    // dgps age
    if (GetItem(11, pBuffer, iSize, i) && (i >= 0))
        pDatabase->Set(CDatabase::DATA_DGPS_DATA_AGE, i);
    // dgps station id
    if (GetItem(12, pBuffer, iSize, i) && (i >= 0) && (i <= 1023))
        pDatabase->Set(CDatabase::DATA_DIFFERENTIAL_REFERENCE_STATION_ID, i);
}

void CProtocolNMEA::ProcessGRS(unsigned char* pBuffer, int iSize, CDatabase* pDatabase)
{
    // Time
    CheckSetTime(pBuffer, iSize, pDatabase, 1);
    // Mode
    // Residuals
}


void CProtocolNMEA::ProcessGSA(unsigned char* pBuffer, int iSize, CDatabase* pDatabase)
{
    // In a multi GNSS environment, we can expect multiple GSA messages per epoch
    double d;
    int i;
    char ch;
    if (GetItem(1, pBuffer, iSize, ch) && MatchChar("MA", ch, i))
        pDatabase->Set(CDatabase::DATA_GPS_OPERATION_MODE, i);
    // GSA navigation mode
    // '1' = Fix not available
    // '2' = 2D/DR
    // '3' = 3D
    if (GetItem(2, pBuffer, iSize, i) && (i >= 1) && (i <= 3))
    {
        pDatabase->Set(CDatabase::DATA_FIX_TYPE, i - 1 /*M$ why add 1*/);
    }

    // DOP
    if (GetItem(15, pBuffer, iSize, d) && (d < dopLimit))
    {
        pDatabase->Set(CDatabase::DATA_POSITION_DILUTION_OF_PRECISION, d);
    }
    if (GetItem(16, pBuffer, iSize, d) && (d < dopLimit))
    {
        pDatabase->Set(CDatabase::DATA_HORIZONAL_DILUTION_OF_PRECISION, d);
    }
    if (GetItem(17, pBuffer, iSize, d) && (d < dopLimit))
    {
        pDatabase->Set(CDatabase::DATA_VERTICAL_DILUTION_OF_PRECISION, d);
    }
}

void CProtocolNMEA::ProcessGST(unsigned char* pBuffer, int iSize, CDatabase* pDatabase)
{
    double d, e;
    // Time
    CheckSetTime(pBuffer, iSize, pDatabase, 1);
    // RMS
    // Std Dev Maj
    // Std Dev Min
    // Orient
    // Std Dev Lat / Lon
    if (GetItem(6, pBuffer, iSize, d) && (d < stdDevLimit) &&
        GetItem(7, pBuffer, iSize, e) && (d < stdDevLimit))
    {
        d = sqrt(d*d + e*e);
        pDatabase->Set(CDatabase::DATA_ERROR_RADIUS_METERS, d);
    }
    // Std Dev Alt
    if (GetItem(8, pBuffer, iSize, d) && (d < stdDevLimit))
    {
        pDatabase->Set(CDatabase::DATA_ALTITUDE_SEALEVEL_ERROR_METERS, d);
        pDatabase->Set(CDatabase::DATA_ALTITUDE_ELLIPSOID_ERROR_METERS, d);
    }
}

void CProtocolNMEA::ProcessRMC(unsigned char* pBuffer, int iSize, CDatabase* pDatabase)
{
    char ch;
    double d, second = 0.0;
    int year = 0, month = 0, day = 0, hour = 0, minute = 0, i = 0;
    // Time / Date
    CheckSetTime(pBuffer, iSize, pDatabase, 1);
    bool bDateOk = GetItem(9, pBuffer, iSize, i) && CalcDate(i, day, month, year);
    if (bDateOk)
    {
        if (year < 80)
            year += 2000;
        else
            year += 1900;
    }

    if (bDateOk)
    {
        pDatabase->Set(CDatabase::DATA_UTC_DATE_YEAR,  year);
        pDatabase->Set(CDatabase::DATA_UTC_DATE_MONTH, month);
        pDatabase->Set(CDatabase::DATA_UTC_DATE_DAY,   day);
    }
    // RMC/GLL status
    SetStatus(pBuffer, iSize, pDatabase, 2);
    // Lat / Lon
    SetLatLon(pBuffer, iSize, pDatabase, 3 /* .. 6 */);
    // SOG
    if (GetItem(7, pBuffer, iSize, d) && (d >= 0.0))
    {
        pDatabase->Set(CDatabase::DATA_SPEED_KNOTS,  d);
    }

    // COG
    if (GetItem(8, pBuffer, iSize, d) && (d >= -180.0) && (d <= 360.0))
    {
        d = (d < 0.0) ? d + 360.0 : d;
        pDatabase->Set(CDatabase::DATA_TRUE_HEADING_DEGREES,  d);
    }
    // COG Mag
    if (GetItem(10, pBuffer, iSize, d) && GetItem(11, pBuffer, iSize, ch) && (d >= 0.0) && (d <= 180.0))
    {
        ch = (char) toupper(ch); // be tolerant
        // (E)ast subtracts from true course
        // (W)est adds to true course
        if (ch == 'W')
        {
            pDatabase->Set(CDatabase::DATA_MAGNETIC_VARIATION, d);
        }
        else if (ch == 'E')
        {
            pDatabase->Set(CDatabase::DATA_MAGNETIC_VARIATION, -d);
        }
    }
    // Mode Indicator
    SetModeIndicator(pBuffer, iSize, pDatabase, 12);
}

void CProtocolNMEA::ProcessVTG(unsigned char* pBuffer, int iSize, CDatabase* pDatabase)
{
    double d;
    char ch;
    // COG (true)
    if (GetItem(1, pBuffer, iSize, d) && (d >= -180.0) && (d <= 360.0) &&
        GetItem(2, pBuffer, iSize, ch) && (toupper(ch) == 'T'))
    {
        pDatabase->Set(CDatabase::DATA_TRUE_HEADING_DEGREES, CDatabase::Degrees360(d));
    }

    // COG (magnetic)
    if (GetItem(3, pBuffer, iSize, d) && (d >= -180.0) && (d <= 360.0) &&
        GetItem(4, pBuffer, iSize, ch) && (toupper(ch) == 'M'))
    {
        pDatabase->Set(CDatabase::DATA_MAGNETIC_HEADING_DEGREES, CDatabase::Degrees360(d));
    }

    // SOG (knots)
    if (GetItem(5, pBuffer, iSize, d) && (d >= 0.0) &&
        GetItem(6, pBuffer, iSize, ch) && (toupper(ch) == 'N'))
    {
        pDatabase->Set(CDatabase::DATA_SPEED_KNOTS, d);
    }
    // SOG (km/hr)
    else if (GetItem(7, pBuffer, iSize, d) && (d >= 0.0) &&
             GetItem(8, pBuffer, iSize, ch) && (toupper(ch) == 'K'))
    {
        pDatabase->Set(CDatabase::DATA_SPEED_KNOTS, d * 3600.0 / METERS_PER_NAUTICAL_MILE);
    }
    // Mode Indicator
    SetModeIndicator(pBuffer, iSize, pDatabase, 9);
}

void CProtocolNMEA::ProcessZDA(unsigned char* pBuffer, int iSize, CDatabase* pDatabase)
{
    double d, second = 0.0;
    int year = 0, month = 0, day = 0, hour = 0, minute = 0;
    CheckSetTime(pBuffer, iSize, pDatabase, 1);
    bool bDateOk = GetItem(2, pBuffer, iSize, day) &&
                   GetItem(3, pBuffer, iSize, month) &&
                   GetItem(4, pBuffer, iSize, year);

    if (bDateOk)
    {
        pDatabase->Set(CDatabase::DATA_UTC_DATE_YEAR,  year);
        pDatabase->Set(CDatabase::DATA_UTC_DATE_MONTH, month);
        pDatabase->Set(CDatabase::DATA_UTC_DATE_DAY,   day);
    }
}

// HELPER for NMEA decoding

void CProtocolNMEA::SetLatLon(unsigned char* pBuffer, int iSize, CDatabase* pDatabase, int ix)
{
    double d;
    char ch;
    if (GetItem(ix, pBuffer, iSize, d)   && GetItem(ix + 1, pBuffer, iSize, ch) && CalcLat(ch, d))
    {
        pDatabase->Set(CDatabase::DATA_LATITUDE_DEGREES, d);
    }
    // Position Longitude
    if (GetItem(ix + 2, pBuffer, iSize, d) && GetItem(ix + 3, pBuffer, iSize, ch) && CalcLon(ch, d))
    {
        pDatabase->Set(CDatabase::DATA_LONGITUDE_DEGREES, d);
    }
}

void CProtocolNMEA::SetStatus(unsigned char* pBuffer, int iSize, CDatabase* pDatabase, int ix)
{
    int i;
    char ch;
    // NMEA: RMC/GLL status
    // 'A' = Data valid
    // 'V' = Navigation receiver warning (inValid)
    if (GetItem(ix, pBuffer, iSize, ch) && MatchChar("AV", ch, i))
    {
        pDatabase->Set(CDatabase::DATA_GPS_STATUS, i + 1 /* M$ why add 1 */);
    }
}

void CProtocolNMEA::SetModeIndicator(unsigned char* pBuffer, int iSize, CDatabase* pDatabase, int ix)
{
    int i;
    char ch;
    // Mode Indicator
    // NMEA: GLL/RMC/VTG positioning mode indicator (NMEA ver >= 2.3)
    // 'N' = Data not valid
    // 'E' = Estimated (dead reckoning) mode
    // 'D' = Differential mode
    // 'A' = Autonomous mode
    // 'M' = Manual input mode
    // 'S' = Simulator mode
    if (GetItem(ix, pBuffer, iSize, ch) && MatchChar("ADEMSN", ch, i))
    {
        pDatabase->Set(CDatabase::DATA_GPS_SELECTION_MODE, i);
    }
}

void CProtocolNMEA::CheckSetTime(unsigned char* pBuffer, int iSize, CDatabase* pDatabase, int ix)
{
    double d = 0.0;
    double second = 0.0;
    int hour = 0;
    int minute = 0;

    bool bTimeOk = GetItem(ix, pBuffer, iSize, d) && CalcTime(d, hour, minute, second);
    if (bTimeOk)
    {
        // Update time
        pDatabase->Set(CDatabase::DATA_UTC_TIME_HOUR,   hour);
        pDatabase->Set(CDatabase::DATA_UTC_TIME_MINUTE, minute);
        pDatabase->Set(CDatabase::DATA_UTC_TIME_SECOND, second);
    }
}

int CProtocolNMEA::GetItemCount(unsigned char* pBuffer, int iSize)
{
    char* pEnd = (char*) &pBuffer[iSize];
    // find the start
    int iCount = 0;
    for (char* pPos = (char*) pBuffer; (pPos < pEnd); pPos ++)
    {
        if (*pPos == ',')
            iCount ++;
    }
    return iCount;
}


//! Find the first character of an NMEA field
/*! Return a pointer to the first character of the field iIndex in the NMEA string starting
    with pStart and ending with pEnd-1
  \param iIndex     : Index of the field in the NMEA string of whcih the position should be found
  \param pBuffer    : Pointer to the NMEA string which should be parsed
  \param pEnd       : Pointer to the character after the last character that belongs to the NMEA string
  \return           : On success, the pointer to the first character of the NMEA field. If not found, NULL
 */
char* CProtocolNMEA::FindPos(int iIndex, char* pStart, const char* pEnd)
{
    // find the start
    for (; (pStart < pEnd) && (iIndex > 0); pStart ++)
    {
        if (*pStart == ',')
            iIndex --;
    }
    // found and check bounds
    if ((iIndex == 0) && (pStart < pEnd) &&
        (*pStart != ',') && (*pStart != '*') && (*pStart != '\r') && (*pStart != '\n'))
        return pStart;
    else
        return NULL;
}

//! Convert a NMEA field to double
/*! Get an NMEA field at position iIndex in pBuffer and return the result as double
  \param iIndex     : Index of the field in the NMEA string that should be extracted
  \param pBuffer    : Pointer to the NMEA string which should be parsed
  \param iSize      : The number of characters in the NMEA string
  \param dValue     : If successful, the resulting double will be saved in this variable
  \return           : true if the field was non-emtpy and could be converted to the correct type
 */
bool CProtocolNMEA::GetItem(int iIndex, unsigned char* pBuffer, int iSize, double& dValue)
{
    char* pEnd = (char*) &pBuffer[iSize];
    char* pPos = FindPos(iIndex, (char*)pBuffer, pEnd);
    // find the start
    if (!pPos || (pEnd <= pPos))
    {
        return false;
    }

    char* pTemp;
    // M$ specific - because the strtod function uses a strlen we make sure that
    // the string is zero terminated, this ensures correct behaviour of the function
    char chEnd = pEnd[-1];
    pEnd[-1] = '\0';
    dValue = strtod(pPos, &pTemp);
    // restore the last character
    pEnd[-1] = chEnd;

    return (pTemp > pPos);
}

//! Convert a NMEA field to integer
/*! Get an NMEA field at position iIndex in pBuffer and return the result as integer
  \param iIndex     : Index of the field in the NMEA string that should be extracted
  \param pBuffer    : Pointer to the NMEA string which should be parsed
  \param iSize      : The number of characters in the NMEA string
  \param iValue     : If successful, the resulting integer will be saved in this variable
  \param iBase      : The base that is used for the integer conversion (0 will distinguish
                      between 8, 10, 16 depending on the prefix automatically)
  \return           : true if the field was non-emtpy and could be converted to the correct type
 */
bool CProtocolNMEA::GetItem(int iIndex, unsigned char* pBuffer, int iSize, int& iValue, int iBase /*=10*/)
{
    char* pEnd = (char*) &pBuffer[iSize];
    char* pPos = FindPos(iIndex, (char*)pBuffer, pEnd);
    // find the start
    if (!pPos)
    {
        return false;
    }
    char* pTemp;
    iValue = (int) strtol(pPos, &pTemp, iBase);
    return (pTemp > pPos);
}

//! Convert a NMEA field to single char
/*! Get an NMEA field at position iIndex in pBuffer and return the first character
  \param iIndex     : Index of the field in the NMEA string that should be extracted
  \param pBuffer    : Pointer to the NMEA string which should be parsed
  \param iSize      : The number of characters in the NMEA string
  \param chValue    : If successful, the resulting double will be saved in this variable
  \return           : true if the field was non-emtpy and could be converted to the correct type
 */
bool CProtocolNMEA::GetItem(int iIndex, unsigned char* pBuffer, int iSize, char& chValue)
{
    char* pEnd = (char*) &pBuffer[iSize];
    char* pPos = FindPos(iIndex, (char*)pBuffer, pEnd);
    // find the start
    if (!pPos)
    {
        return false;
    }
    // skip leading spaces
    while ((pPos < pEnd) && isspace(*pPos))
    {
        pPos++;
    }
    // check bound
    if ((pPos < pEnd) &&
        (*pPos != ',') && (*pPos != '*') && (*pPos != '\r') && (*pPos != '\n'))
    {
        chValue = *pPos;
        return true;
    }
    return false;
}

// special helpers for converting items in nmea formats

bool CProtocolNMEA::MatchChar(const char* string, char ch, int& i)
{
    if (ch)
    {
        const char* p;
        ch = (char) toupper(ch); // be tolerant
        p = strchr(string, ch);
        if (p)
        {
            i = (int)(p - string); // calc index
            return true;
        }
    }
    return false;
}

double CProtocolNMEA::CalcAngle(double d)
{
    // converts an angle in the format dddmm.mmmm to radians
    int t;
    t = (int) (d / 100.0);
    return (((d - (100.0 * t)) / 60.0) + t);
}

bool CProtocolNMEA::CalcLon(char ch, double& d)
{
    // be tolerant
    ch = (char) toupper(ch);
    // convert lon in nmea format to degrees
    if (ch == 'E')
        d = CalcAngle(d);
    else if (ch == 'W')
        d = - CalcAngle(d);
    else
        return false;
    if (d > 180.0)
        d -= 360.0;
    return (d >= -180.0) && (d <= 180.0);
}

bool CProtocolNMEA::CalcLat(char ch, double& d)
{
    // be tolerant
    ch = (char) toupper(ch);
    // convert lat in nmead format to degrees
    if (ch == 'N')
        d = CalcAngle(d);
    else if (ch == 'S')
        d = - CalcAngle(d);
    else
        return false;
    if (d > 180.0)
        d -= 360.0;
    return (d >= -180.0) && (d <= 180.0);
}

bool CProtocolNMEA::CalcTime(double d, int& h, int& m, double& s)
{
    // extracts hh, mm and ss.ssss from a value in the format hhmmss.ssss
    if ((d >= 1000000.0) || (d < 0.0))
        return false;
    int t = (int)(d / 100);
    m = t % 100;
    h = (t - m) / 100;
    s = d - 100.0 * t;
    return (h >= 0) && (h <= 23) && (m >= 0) && (m < 60) && (s >= 0.0) && (s < 60.0);
}

bool CProtocolNMEA::CalcDate(int d, int& day, int& mon, int& yr)
{
    // extracts yy, mm and dd from a value in the format ddmmyy
    if ((d > 999999) || (d < 0))
        return false;
    yr = d % 100;
    d /= 100;
    mon = d % 100;
    d /= 100;
    day = d % 100;
    return (day >= 1) && (day <= 31) && (mon >= 1) && (mon <= 12) && (yr >= 0) && (yr <= 99);
}
