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
 * $Id: database.cpp 108172 2015-12-14 13:03:19Z fabio.robbiani $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/PRODUCTS/libParser/tags/libParser_v1.04/database.cpp $
 *****************************************************************************/
/*!
  \file
  \brief Contains the databse used for storing the parsed values

  The database allows the parser modules to store the parsed values in it.
*/

#include "database.h"
#include "gpsconst.h"
#include <math.h>
#include <malloc.h>
#include <string.h>
#include <assert.h>

const int CDatabase::GNSSID_UNKN = -1;
const int CDatabase::GNSSID_GPS  =  0;
const int CDatabase::GNSSID_SBAS =  1;
const int CDatabase::GNSSID_GAL  =  2;
const int CDatabase::GNSSID_BDS  =  3;
const int CDatabase::GNSSID_QZSS =  5;
const int CDatabase::GNSSID_GLO  =  6;

///////////////////////////////////////////////////////////////////////////////

// Debugging Stuff / Tools to dump the Database
#define DUMP_X(ix, txt, fmt, fmtbad) 	\
{										\
    double d; 							\
if (pVar[ix].Get(d))				\
    Printf(txt fmt, d);				\
    else 								\
    Printf(txt fmtbad);				\
}

#define DUMP_TS(ix)														\
{																		\
    TIMESTAMP ts;														\
if (pVar[ix].Get(ts))												\
    {																	\
    unsigned long s = ts.lMicroseconds / 1000000;					\
    unsigned long us = ts.lMicroseconds - s * 1000000;				\
    Printf("time %02u:%02u:%02lu.%06lu date %02u.%02u.%04u ", \
    ts.wHour, ts.wMinute, s, us, \
    ts.wDay, ts.wMonth, ts.wYear);						\
    }																	\
    else																\
    Printf("time ??:??:??.?????? date ??.??.???? ");				\
}

///////////////////////////////////////////////////////////////////////////////

CDatabase::CDatabase(void)
{
    m_pNmeaMsgListNext = NULL;
    m_pNmeaMsgListOutput = NULL;
    m_epochBegin=MSG_NMEA_RMC;
    m_epochEnd=MSG_NUM;
    Reset();
}

CDatabase::~CDatabase(void)
{
    free(m_pNmeaMsgListNext);
    free(m_pNmeaMsgListOutput);
}

//! Define the first message expected at the start of an epoch
/*! The first message of an epoch can be defined with this function. Default is MSG_NMEA_RMC
  \param begin : The message which will be expected on the start of an epoch
*/
void CDatabase::setBeginEpoch(MSG_t begin)
{
    if(begin!=MSG_NUM)
    {
        m_epochBegin=begin;
    }
}

//! Define the last message expected at the end of an epoch
/*! The first message of an epoch can be defined with this function. Default is MSG_NMEA_RMC.
    Be aware that if a message is received that maches 'end', the epoch is considered
    as finished. Never select a message as the 'end' message which could appear more
    than once within an epoch. The default value for this is MSG_NUM, which indicates
    that the commit of the database will happen on receiving the first message of the new
    epoch.
  \param end : The message which will be expected on the start of an epoch
*/
void CDatabase::setEndEpoch(MSG_t end)
{
    m_epochEnd=end;
}

//! Process the message received with the function provided by the parser
/*! The message of type 'type' found in 'pBuffer' will be parsed by 'func' if the epoch
    is considered as having started. The function will also indicate an epoch as having
    started or ended based on 'type'. If this function is called to parse a message
    that doesn't start an epoch and the epoch is not yet indicated as having
    started, nothing will be done.
  \param type    : Type of the message that must be parsed
  \param pBuffer : Buffer in which the message of type 'type' hsould be located
  \param iSize   : The size of the Buffer 'pBuffer'
  \param func    : The function which is responsible to parse the message
  \return        : -1 on invalid arguments, 0 if the parsing function was
                   called for parsing, 1 if the message arrived out of epoch
                   and nothing was done
*/
int CDatabase::Process(MSG_t type, unsigned char * pBuffer, int iSize, PARSE_FUNC_t func)
{
    if(func==NULL || pBuffer==NULL || iSize<=0)
    {
        return -1;
    }
    int result=1;
    this->beforeProcessing(type);
    if(m_epochBegin)
    {
        result=0;
        func(pBuffer, iSize, this);
        this->afterProcessing(type);
    }
    return result;
}

//! Makes sure an epoch is indicated as having started
/*! Sets the indication for an epoch having started to positive if the current
    message matches the m_epochBegin message type. If this is called without
    the prior epoch having been ended officially, this function will do this and
    start the new one (default operation)
  \param message    : The message that has arrived and for which it has to be
                      checked if it modifies the epoch indicator.
*/
void CDatabase::beforeProcessing(MSG_t message)
{
    if(m_epochBegin==message)
    {
        // This must be the begin of a new epoch
        if(m_receivingEpoch)
        {
            // If the old epoch has not ended as it
            // is supposed, do it now
            afterProcessing(m_epochEnd);
        }
        m_receivingEpoch=true; // It is now allowed to set data
    }
}

//! Makes sure an epoch is indicated as having ended
/*! Sets the indication for an epoch having started to negative if the current
    message matches the m_epochEnd message type.
    \param message    : The message that has arrived and for which it has to be
                        checked if it modifies the epoch indicator.
*/
void CDatabase::afterProcessing(MSG_t message)
{
    if(m_epochEnd==message)
    {
        Commit(true);
        m_receivingEpoch=false;
    }
}

CDatabase::STATE_t CDatabase::Commit(bool timeBasedEpoch)
{
    timeBasedEpoch;

    // Populate any properties that can calculated from known properties
    CompletePosition();
    CompleteVelocity();
    CompleteAltitude();
    CompleteHeading();
    CompleteTimestamp();

    LockOutputDatabase();
    memcpy(m_varO, m_varN, sizeof(m_varN));
    free(m_pNmeaMsgListOutput);
    m_pNmeaMsgListOutput = m_pNmeaMsgListNext;

    // set time commit time stamp
    TIMESTAMP ts;
    if (GetCurrentTimestamp(ts))
    {
        m_varO[DATA_COMMIT_TIMESTAMP].Set(ts);
    }

    // fill in an accuarcy if it is not available
    if (m_varO[DATA_ERROR_RADIUS_METERS].IsEmpty() &&
        m_varO[DATA_LONGITUDE_DEGREES].IsSet()     &&
        m_varO[DATA_LATITUDE_DEGREES].IsSet())
    {
        int i;
        double d;
        if (m_varO[DATA_UBX_POSITION_ECEF_ACCURACY].Get(d))
        {
            m_varO[DATA_ERROR_RADIUS_METERS].Set(d);
        }
        else if (m_varO[DATA_SATELLITES_USED_COUNT].Get(i))
        {
            // fake the error radius in case the error radius is not set
            // this should be filled with something more resonable here
            m_varO[DATA_ERROR_RADIUS_METERS].Set((i >= 6) ? 5 :
                                                (i >= 4) ?  10 :
                                                           100 );
        }
    }

    // compute the last known good fix
    if (m_varO[DATA_LONGITUDE_DEGREES].IsSet() && m_varO[DATA_LATITUDE_DEGREES].IsSet() &&
        m_varO[DATA_ERROR_RADIUS_METERS].IsSet())				//  && m_varO[DATA_LOCAL_TIMESTAMP].IsSet())
    {
        // mandatory fields
        m_varO[DATA_LASTGOOD_LONGITUDE_DEGREES] = m_varO[DATA_LONGITUDE_DEGREES];
        m_varO[DATA_LASTGOOD_LATITUDE_DEGREES] = m_varO[DATA_LATITUDE_DEGREES];
        m_varO[DATA_LASTGOOD_ERROR_RADIUS_METERS] = m_varO[DATA_ERROR_RADIUS_METERS];
        m_varO[DATA_LASTGOOD_TIMESTAMP] = m_varO[DATA_COMMIT_TIMESTAMP];				// m_varO[DATA_LOCAL_TIMESTAMP];
        // optional fields
        m_varO[DATA_LASTGOOD_ALTITUDE_ELLIPSOID_METERS] = m_varO[DATA_ALTITUDE_ELLIPSOID_METERS];
        m_varO[DATA_LASTGOOD_ALTITUDE_ELLIPSOID_ERROR_METERS] = m_varO[DATA_ALTITUDE_ELLIPSOID_ERROR_METERS];
        // set state to ready
        m_vasS = STATE_READY;
    }
    else if (m_vasS == STATE_READY)
    {
        // fallback to reacq
        m_vasS = STATE_REACQ;
    }
    UnlockOutputDatabase();
    // Clear 'Next' epoch data
    m_pNmeaMsgListNext = NULL;
    memset(m_varN, 0, sizeof(m_varN));

    return m_vasS;
}

void CDatabase::Reset(void)
{
    m_receivingEpoch=false;
    free(m_pNmeaMsgListNext);
    free(m_pNmeaMsgListOutput);
    m_pNmeaMsgListNext = NULL;
    m_pNmeaMsgListOutput = NULL;
    memset(m_varN, 0, sizeof(m_varN));
    memset(m_varO, 0, sizeof(m_varO));
    m_vasS = STATE_NO_DATA;
}


bool CDatabase::CheckTime(int hour, int minute, double seconds) const
{
    bool timeMatches =  (m_varN[DATA_UTC_TIME_HOUR].Check(hour))   &&
                        (m_varN[DATA_UTC_TIME_MINUTE].Check(minute)) &&
                        (m_varN[DATA_UTC_TIME_SECOND].Check(seconds));
    return timeMatches;
}

bool CDatabase::CheckDate(int year, int month, int day) const
{
    bool dateMatches = (m_varN[DATA_UTC_DATE_YEAR].Check(year))  &&
                       (m_varN[DATA_UTC_DATE_MONTH].Check(month)) &&
                       (m_varN[DATA_UTC_DATE_DAY].Check(day));
    return dateMatches;
}

double CDatabase::Degrees360(double val)
{
    return (val <    0.0) ? val + 360.0 :
           (val >= 360.0) ? val - 360.0 : val;
}

bool CDatabase::ConvertUbxSvidToNmea41e(int svid, NMEA_t *nmeaOut)
{
    if( !nmeaOut )
    {
        return false;
    }

    int gnssId = GNSSID_UNKN;

    if( svid >= 1 && svid <= 32 /* GPS */ )
    {
        // NMEA has reserved 32 svs for GPS,
        // which are not remapped
        gnssId = GNSSID_GPS;
    }
    else if( svid >= 120 && svid <= 151 /* SBAS Block 1 */ )
    {
        // NMEA has reserved 32 svs for SBAS
        // The svids 120-151 are mapped to 33-64.
        gnssId = GNSSID_SBAS;
        svid = (svid - 120 + 33);
    }
    else if( svid >= 152 && svid <= 158 /* SBAS Block 2 */ )
    {
        // The SBAS satellite numbers 152 to 158
        // are not remapped
        gnssId = GNSSID_SBAS;
    }
    else if( svid >= 211 && svid <= 246 /* Galileo */ )
    {
        // Remap Galileo
        // in the range 1-36
        svid = (svid - 211 + 1);
        gnssId = GNSSID_GAL;
    }
    else if( svid >= 159 && svid <= 163 /* BeiDou Block 1 */ )
    {
        // Remap the lower part of the BeiDou
        // numbering to 1-5
        gnssId = GNSSID_BDS;
        svid = (svid - 159 + 1);
    }
    else if( svid >= 33 && svid <= 64 /* BeiDou Block 2 */ )
    {
        // Remap the lower part of the BeiDou
        // numbering to 6-37
        gnssId = GNSSID_BDS;
        svid = (svid - 33 + 6);
    }
    else if( svid >= 193 && svid <= 197 /* QZSS */)
    {
        // NMEA has reserved 5 SVs for QZSS,
        // in the range 193-197
        gnssId = GNSSID_QZSS;
    }
    else if( svid >= 65 && svid <= 96 /* GLONASS */)
    {
        // NMEA has reserved 32 SVs for GLONASS,
        // in the range 65-96
        gnssId = GNSSID_GLO;
    }
    else if( svid == 255 ) /* Unknown GLONASS */
    {
        // The svid of glonass is unknown, set it to -1
        gnssId = GNSSID_GLO;
        svid = -1;
    }

    bool result = false;

    // Assign values if everything went right
    if( gnssId != GNSSID_UNKN )
    {
        nmeaOut->gnssid = gnssId;
        nmeaOut->svid   = svid;
        result          = true;
    }

    return result;
}

bool CDatabase::ConvertNmeaToNmea41e(char t, int svid, NMEA_t *nmeaOut)
{
    if( !nmeaOut )
    {
        return false;
    }

    int gnssId = GNSSID_UNKN;

    switch (t)
    {
        case 'P': /* GPS, SBAS, QZSS */
        {

            if( svid >=   1 && svid <=  32 /* GPS */ )
            {
                gnssId = GNSSID_GPS;
            }
            else if( (svid >=  33 && svid <=  64 /* SBAS block 1 */)
                  || (svid >= 152 && svid <= 158 /* SBAS block 2 */) )
            {
                gnssId = GNSSID_SBAS;
            }
            else if( svid >= 193 && svid <= 197 /* QZSS */ )
            {
                gnssId = GNSSID_QZSS;
            }

            break;
        }
        case 'A': /* GAL */
        {
            if( svid >=   1 && svid <=  36 /* Galileo */ )
            {
                gnssId = GNSSID_GAL;
            }
            else if( svid >= 301 && svid <= 336 /* Galileo NMEA 2.X-4.0 extended */ )
            {
                gnssId = GNSSID_GAL;
                svid  -= 300;
            }

            break;
        }
        case 'B': /* BDS */
        {
            if( svid >=   1 && svid <=  37 /* BeiDou NMEA 4.1+ */ )
            {
                gnssId = GNSSID_BDS;
            }
            else if( svid >= 401 && svid <= 437 /* BeiDou NMEA 2.X-4.0 extended */ )
            {
                gnssId = GNSSID_BDS;
                svid -= 400;
            }

            break;
        }
        case 'L': /* GLO */
        {
            if( svid >=  65 && svid <=  96 /* GLONASS */ )
            {
                gnssId = GNSSID_GLO;
            }
            else if( svid <= 0 || svid == 255 /* Uknown GLONASS */ )
            {
                gnssId = GNSSID_GLO;
                svid = -1;
            }

            break;
        }
    }

    bool result = false;

    // Assign values if everything went right
    if( gnssId != GNSSID_UNKN )
    {
        nmeaOut->gnssid = gnssId;
        nmeaOut->svid   = svid;
        result          = true;
    }

    return result;
}

#define TIMEDIFFERENCE_UTC2GPS	315964800.0 // which is (double)mkgmtime(1980,1,6,0,0,0,0)

double CDatabase::CalcLeapSeconds(double dUtcSec) const
{
    const struct
    {
        double dTime;
        unsigned short wYear;
        unsigned char byMonth;
        unsigned char byDay;
        double dLeapSecs;
    } LUT[] = {
        { 1341100800, 2012, 7, 1, 16 },
        { 1230764400, 2009, 1, 1, 15 },
        { 1136073600, 2006, 1, 1, 14 },
        {  915148800, 1999, 1, 1, 13 },
        {  867715200, 1997, 7, 1, 12 },
        {  820454400, 1996, 1, 1, 11 },
        {  773020800, 1994, 7, 1, 10 },
        {  741484800, 1993, 7, 1,  9 },
        {  709948800, 1992, 7, 1,  8 },
        {  662688000, 1991, 1, 1,  7 },
        {  631152000, 1990, 1, 1,  6 },
        {  567993600, 1988, 1, 1,  5 },
        {  489024000, 1985, 7, 1,  4 },
        {  425865600, 1983, 7, 1,  3 },
        {  394329600, 1982, 7, 1,  2 },
        {  362793600, 1981, 7, 1,  1 },
        {  315532800, 1980, 1, 1,  0 }
    };
    unsigned int i;
    for (i=0;i<sizeof(LUT)/sizeof(*LUT);i++)
    {
        if (dUtcSec > LUT[i].dTime)
        {
            return LUT[i].dLeapSecs;
        }
    }
    return 0;
}

void CDatabase::CompleteTimestamp(void)
{
    TIMESTAMP st;
    double seconds;
    memset(&st, 0, sizeof(st));		// clear first
    if (m_varN[DATA_UTC_DATE_YEAR].Get(st.wYear)		&& m_varN[DATA_UTC_DATE_MONTH].Get(st.wMonth) &&
        m_varN[DATA_UTC_DATE_DAY].Get(st.wDay)			&& m_varN[DATA_UTC_TIME_HOUR].Get(st.wHour) &&
        m_varN[DATA_UTC_TIME_MINUTE].Get(st.wMinute)	&& m_varN[DATA_UTC_TIME_SECOND].Get(seconds))
    {
        st.lMicroseconds = (unsigned long)(seconds * 1e6);
        m_varN[DATA_UTC_TIMESTAMP].Set(st);
    }
}

void CDatabase::CompletePosition(void)
{
    double X, Y;
    if (m_varN[DATA_UBX_POSITION_ECEF_X].Get(X) &&
        m_varN[DATA_UBX_POSITION_ECEF_Y].Get(Y))
    {
        double Z;
        double dLon = atan2(Y,X);
        // Set Longitude if needed
        if (m_varN[DATA_LONGITUDE_DEGREES].IsEmpty())
        {
            m_varN[DATA_LONGITUDE_DEGREES].Set(dLon * DEGREES_PER_RADIAN);
        }

        if (m_varN[DATA_UBX_POSITION_ECEF_Z].Get(Z))
        {
            double p = sqrt(X * X + Y * Y);
            double T = atan2(Z * A, p * B);
            double sinT = sin(T);
            double cosT = cos(T);
            double dLat = atan2(Z + E2SQR * B * sinT * sinT * sinT, p - E1SQR * A * cosT * cosT * cosT);
            // Set Latitude if needed
            if (m_varN[DATA_LATITUDE_DEGREES].IsEmpty())
            {
                m_varN[DATA_LATITUDE_DEGREES].Set(dLat * DEGREES_PER_RADIAN);
            }
            // Set Altitude if needed
            if (m_varN[DATA_ALTITUDE_ELLIPSOID_METERS].IsEmpty())
            {
                // handle the poles
                double dAlt;
                if (p == 0.0)
                {
                    dAlt = fabs(Z) - B;
                }
                else
                {
                    double sinF = sin(dLat);
                    double cosF = cos(dLat);
                    double N =  A*A / sqrt(A*A * cosF*cosF + B*B * sinF*sinF);
                    dAlt = p / cosF - N;
                }
                m_varN[DATA_ALTITUDE_ELLIPSOID_METERS].Set(dAlt);
            }
        }
    }
}

void CDatabase::CompleteVelocity(void)
{
    double X,Y,Long;
    if (m_varN[DATA_UBX_VELOCITY_ECEF_VX].Get(X) &&
        m_varN[DATA_UBX_VELOCITY_ECEF_VY].Get(Y) &&
        m_varN[DATA_LONGITUDE_DEGREES].Get(Long))
    {
        Long *= RADIANS_PER_DEGREE;
        double sinL = sin(Long);
        double cosL = cos(Long);
        double ve = - X * sinL + Y * cosL;
        double Z,Lat;
        if (m_varN[DATA_UBX_VELOCITY_ECEF_VZ].Get(Z) &&
            m_varN[DATA_LATITUDE_DEGREES].Get(Lat))
        {
            Lat *= RADIANS_PER_DEGREE;
            double sinF = sin(Lat);
            double cosF = cos(Lat);
            double vn = - X * sinF * cosL - Y * sinF * sinL + Z * cosF;
            //double vd = - X * cosF * cosL - Y * cosF * sinL - Z * sinF;
            double speed2 = vn*vn + ve*ve;
            if (m_varN[DATA_SPEED_KNOTS].IsEmpty())
            {
                double speed = sqrt(speed2);
                m_varN[DATA_SPEED_KNOTS].Set(speed / METERS_PER_NAUTICAL_MILE);
            }

            if (m_varN[DATA_TRUE_HEADING_DEGREES].IsEmpty())
            {
                // No heading... try to calculate it.
                if (speed2 > (1.0*1.0))
                {
                    double cog = atan2(ve, vn);
                    m_varN[DATA_TRUE_HEADING_DEGREES].Set(cog * DEGREES_PER_RADIAN);
                }
                else
                {
                    // Can not calculate, so set to zero
                    m_varN[DATA_TRUE_HEADING_DEGREES].Set(0.0);
                }
            }
        }
    }
}

void CDatabase::CompleteAltitude(void)
{
    // complete altitudes and geodial separation if one of the fields is missing
    double sep = 0.0, ell = 0.0, msl = 0.0;
    bool bSep = m_varN[DATA_GEOIDAL_SEPARATION].Get(sep);
    bool bMsl = m_varN[DATA_ALTITUDE_SEALEVEL_METERS].Get(msl);
    bool bEll = m_varN[DATA_ALTITUDE_ELLIPSOID_METERS].Get(ell);

    if (!bEll && bMsl && bSep)
    {
        m_varN[DATA_ALTITUDE_ELLIPSOID_METERS].Set(sep + msl);
    }
    else if (!bMsl && bSep && bEll)
    {
        m_varN[DATA_ALTITUDE_SEALEVEL_METERS].Set(ell - sep);
    }
    else if (!bSep && bEll && bMsl)
    {
        m_varN[DATA_GEOIDAL_SEPARATION].Set(ell - msl);
    }
}

void CDatabase::CompleteHeading(void)
{
    // complete headings and mag separation if one of the fields is missing
    double var = 0.0, th = 0.0, mh = 0.0;
    bool bVar = m_varN[DATA_MAGNETIC_VARIATION].Get(var);
    bool bMh = m_varN[DATA_MAGNETIC_HEADING_DEGREES].Get(mh);
    bool bTh = m_varN[DATA_TRUE_HEADING_DEGREES].Get(th);

    if (!bVar && bTh && bMh)
    {
        m_varN[DATA_MAGNETIC_VARIATION].Set(Degrees360(mh - th));
    }
    else if (!bMh && bTh && bVar)
    {
        m_varN[DATA_MAGNETIC_HEADING_DEGREES].Set(Degrees360(th + var));
    }
    else if (!bTh && bMh && bVar)
    {
        m_varN[DATA_TRUE_HEADING_DEGREES].Set(Degrees360(mh - var));
    }
}

void CDatabase::AddNmeaMessage(const unsigned char* pBuffer, int iSize)
{
    size_t length = (size_t) iSize;
    char* pNewNmea = NULL;
    if (m_pNmeaMsgListNext)
    {
        length += strlen(m_pNmeaMsgListNext);
        pNewNmea = (char*) realloc(m_pNmeaMsgListNext, length + 1);
    }
    else
    {
        pNewNmea = (char*) malloc(length + 1);
        if (pNewNmea)
        {
            pNewNmea[0] = 0;
        }
    }

    if (pNewNmea != NULL )
    {
        m_pNmeaMsgListNext = pNewNmea;
        strncat(m_pNmeaMsgListNext, (const char*) pBuffer, iSize);
        m_pNmeaMsgListNext[length] = 0;

        assert(strlen(m_pNmeaMsgListNext) == length);
    }
}

void CDatabase::Dump(const CVar* pVar)
{
    int i;
    //Printf("pc "); DUMP_TS(DATA_LOCAL_TIMESTAMP); DUMP_TS(DATA_LOCALX_TIMESTAMP); Printf("\n");
    DUMP_TS(DATA_UTC_TIMESTAMP);
    DUMP_X(DATA_UTC_TIME_HOUR,						"time ",	"%02.0f",	"??");
    DUMP_X(DATA_UTC_TIME_MINUTE,					":",		"%02.0f",	"??");
    DUMP_X(DATA_UTC_TIME_SECOND,					":",		"%06.3f ",	"??.??? ");
    DUMP_X(DATA_UTC_DATE_DAY,						"date ",	"%02.0f",	"??");
    DUMP_X(DATA_UTC_DATE_MONTH,						".",		"%02.0f",	"??");
    DUMP_X(DATA_UTC_DATE_YEAR,						".",		"%04.0f ",	"???? ");
    DUMP_X(DATA_UBX_TTAG,							"ttag ",	"%11.3f",	"      ?.???");
    Printf("\n");
    DUMP_X(DATA_LATITUDE_DEGREES,					"lat ",		"%11.6f ",	"   ?.?????? ");
    DUMP_X(DATA_LONGITUDE_DEGREES,					"lon ",		"%10.6f ",	 "  ?.?????? ");
    DUMP_X(DATA_ERROR_RADIUS_METERS,				"err ",		"%7.2f ",	"   ?.?? ");
    Printf("\n");
    DUMP_X(DATA_ALTITUDE_ELLIPSOID_METERS,			"alt ",		"%7.2f ",	"   ?.?? ");
    DUMP_X(DATA_ALTITUDE_ELLIPSOID_ERROR_METERS,	"err ",		"%7.2f ",	"   ?.?? ");
    DUMP_X(DATA_ALTITUDE_SEALEVEL_METERS,			"msl ",		"%7.2f ",	"   ?.?? ");
    DUMP_X(DATA_ALTITUDE_SEALEVEL_ERROR_METERS,		"err ",		"%7.2f ",	"   ?.?? ");
    DUMP_X(DATA_GEOIDAL_SEPARATION,					"sep ",		"%6.2f ",	"  ?.?? ");
    Printf("\n");
    DUMP_X(DATA_SPEED_KNOTS,						"knots ",	"%7.2f ",	"   ?.?? ");
    DUMP_X(DATA_TRUE_HEADING_DEGREES,				"head ",	"%06.2f ",	"???.?? ");
    DUMP_X(DATA_MAGNETIC_HEADING_DEGREES,			"headmag ",	"%06.2f ",	"???.?? ");
    DUMP_X(DATA_MAGNETIC_VARIATION,					"magvar ",	"%06.2f ",	"???.?? ");
    Printf("\n");
    DUMP_X(DATA_POSITION_DILUTION_OF_PRECISION,		"pdop ",	"%7.2f ",	"   ?.?? ");
    DUMP_X(DATA_HORIZONAL_DILUTION_OF_PRECISION,	"hdop ",	"%7.2f ",	"   ?.?? ");
    DUMP_X(DATA_VERTICAL_DILUTION_OF_PRECISION,		"vdop",		"%7.2f ",	"   ?.?? ");
    Printf("\nfix: ");
    DUMP_X(DATA_FIX_QUALITY,						"qual ",	"%1.0f ",	"? ");
    DUMP_X(DATA_FIX_TYPE,							"type ",	"%1.0f ",	"? ");
    Printf("gps: ");
    DUMP_X(DATA_GPS_OPERATION_MODE,					"op ",		"%1.0f ",	"? ");
    DUMP_X(DATA_GPS_SELECTION_MODE,					"sel ",		"%1.0f ",	"? ");
    DUMP_X(DATA_GPS_STATUS,							"stat ",	"%1.0f ",	"? ");
    Printf("dgps: ");
    DUMP_X(DATA_DIFFERENTIAL_REFERENCE_STATION_ID,	"station ",	"%04.0f ",	"   ? ");
    DUMP_X(DATA_DGPS_DATA_AGE,						"age ",		"%4.0f ",	"   ? ");

    Printf("\n");
    DUMP_X(DATA_SATELLITES_USED_COUNT,				"used ",	"%2.0f ",	" ? ");
    Printf("prn ");
    Printf("\n");
    DUMP_X(DATA_SATELLITES_IN_VIEW,					"in view ",	"%2.0f ",	" ? ");
    Printf("\nprn ");
    for (i = 0; i < CDatabase::MAX_SATELLITES_IN_VIEW; i ++)
        DUMP_X(DATA_SATELLITES_IN_VIEW_SVID_(i),		"",		"%5.0f ",	"      ");
    Printf("\norb ");
    for (i = 0; i < CDatabase::MAX_SATELLITES_IN_VIEW; i ++)
        DUMP_X(DATA_UBX_SATELLITES_IN_VIEW_NAV_STA_(i),	"",		"%5.0f ",	"      ");
    Printf("\naz  ");
    for (i = 0; i < CDatabase::MAX_SATELLITES_IN_VIEW; i ++)
        DUMP_X(DATA_SATELLITES_IN_VIEW_AZIMUTH_(i),		"",		"%5.1f ",	"      ");
    Printf("\nel  ");
    for (i = 0; i < CDatabase::MAX_SATELLITES_IN_VIEW; i ++)
        DUMP_X(DATA_SATELLITES_IN_VIEW_ELEVATION_(i),	"",		"%5.1f ",	"      ");
    Printf("\ncno ");
    for (i = 0; i < CDatabase::MAX_SATELLITES_IN_VIEW; i ++)
        DUMP_X(DATA_SATELLITES_IN_VIEW_STN_RATIO_(i),	"",		"%5.1f ",	"      ");
    Printf("\n");

    if (pVar == m_varO)
    {
        Printf("commit "); DUMP_TS(DATA_COMMIT_TIMESTAMP); Printf("\n");
        Printf("last good "); DUMP_TS(DATA_LASTGOOD_TIMESTAMP); Printf("\n");
        DUMP_X(DATA_LASTGOOD_LATITUDE_DEGREES,					"lat ",		"%11.6f ",	"   ?.?????? ");
        DUMP_X(DATA_LASTGOOD_LONGITUDE_DEGREES,					"lon ",		"%10.6f ",	 "  ?.?????? ");
        DUMP_X(DATA_LASTGOOD_ERROR_RADIUS_METERS,				"err ",		"%7.2f ",	"   ?.?? ");
        Printf("\n");
        DUMP_X(DATA_LASTGOOD_ALTITUDE_ELLIPSOID_METERS,			"alt ",		"%7.2f ",	"   ?.?? ");
        DUMP_X(DATA_LASTGOOD_ALTITUDE_ELLIPSOID_ERROR_METERS,	"err ",		"%7.2f ",	"   ?.?? ");
    }
    // done
    Printf("\n");
}

