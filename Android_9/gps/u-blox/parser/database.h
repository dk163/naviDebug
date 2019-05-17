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
/*!
  \file
  \brief Contains the  header of the databse used for storing the parsed values

  The database allows the parser modules to store the parsed values in it.
*/

#ifndef __DATABASE_H__
#define __DATABASE_H__

#include "datavar.h"

class CDatabase
{
public:
  CDatabase(void);
  virtual ~CDatabase(void);

public:
  typedef void (*PARSE_FUNC_t)(const unsigned char *pBuffer, int iSize, CDatabase *pDatabase);

  enum
  {
    MAX_SATELLITES_IN_VIEW = 64 // Estimation of max. visible
                                // satellites at any given location
                                // at any given time
  };

  typedef enum STATE_e
  {
    STATE_NO_DATA,
    STATE_REACQ,
    STATE_READY
  } STATE_t;

  typedef enum DATA_e
  {
    // these fields are taken from sensor api definitions
    // DATA_ADDRESS1
    // DATA_ADDRESS2
    // DATA_ALTITUDE_ANTENNA_SEALEVEL_METERS ???
    DATA_ALTITUDE_ELLIPSOID_ERROR_METERS,
    DATA_ALTITUDE_ELLIPSOID_METERS,
    DATA_ALTITUDE_SEALEVEL_ERROR_METERS,
    DATA_ALTITUDE_SEALEVEL_METERS,
    // DATA_CITY
    // DATA_COUNTRY_REGION
    DATA_DGPS_DATA_AGE,
    DATA_DIFFERENTIAL_REFERENCE_STATION_ID,
    DATA_ERROR_RADIUS_METERS,
    DATA_FIX_QUALITY,
    DATA_FIX_TYPE,
    DATA_GEOIDAL_SEPARATION,
    DATA_GPS_OPERATION_MODE,
    DATA_GPS_SELECTION_MODE,
    DATA_GPS_STATUS,
    DATA_HORIZONAL_DILUTION_OF_PRECISION,
    DATA_LATITUDE_DEGREES,
    DATA_LONGITUDE_DEGREES,
    DATA_MAGNETIC_HEADING_DEGREES,
    DATA_MAGNETIC_VARIATION,
    // DATA_NMEA_SENTENCE
    DATA_POSITION_DILUTION_OF_PRECISION,
    // DATA_POSTALCODE
    DATA_SATELLITES_IN_VIEW,
    DATA_SATELLITES_IN_VIEW_AZIMUTH,
#define DATA_SATELLITES_IN_VIEW_AZIMUTH_(ix)                                                       \
  (CDatabase::DATA_t)(CDatabase::DATA_SATELLITES_IN_VIEW_AZIMUTH + ix)
    DATA_SATELLITES_IN_VIEW_AZIMUTH_X = MAX_SATELLITES_IN_VIEW + DATA_SATELLITES_IN_VIEW_AZIMUTH,
    DATA_SATELLITES_IN_VIEW_ELEVATION,
#define DATA_SATELLITES_IN_VIEW_ELEVATION_(ix)                                                     \
  (CDatabase::DATA_t)(CDatabase::DATA_SATELLITES_IN_VIEW_ELEVATION + ix)
    DATA_SATELLITES_IN_VIEW_ELEVATION_X =
      MAX_SATELLITES_IN_VIEW + DATA_SATELLITES_IN_VIEW_ELEVATION,
    DATA_SATELLITES_IN_VIEW_GNSSID,
#define DATA_SATELLITES_IN_VIEW_GNSSID_(ix)                                                        \
  (CDatabase::DATA_t)(CDatabase::DATA_SATELLITES_IN_VIEW_GNSSID + ix)
    DATA_SATELLITES_IN_VIEW_GNSSID_X = MAX_SATELLITES_IN_VIEW + DATA_SATELLITES_IN_VIEW_GNSSID,
    DATA_SATELLITES_IN_VIEW_SVID,
#define DATA_SATELLITES_IN_VIEW_SVID_(ix)                                                          \
  (CDatabase::DATA_t)(CDatabase::DATA_SATELLITES_IN_VIEW_SVID + ix)
    DATA_SATELLITES_IN_VIEW_SVID_X = MAX_SATELLITES_IN_VIEW + DATA_SATELLITES_IN_VIEW_SVID,
    DATA_SATELLITES_IN_VIEW_STN_RATIO,
#define DATA_SATELLITES_IN_VIEW_STN_RATIO_(ix)                                                     \
  (CDatabase::DATA_t)(CDatabase::DATA_SATELLITES_IN_VIEW_STN_RATIO + ix)
    DATA_SATELLITES_IN_VIEW_STN_RATIO_X =
      MAX_SATELLITES_IN_VIEW + DATA_SATELLITES_IN_VIEW_STN_RATIO,
    DATA_SATELLITES_USED_COUNT,
    DATA_SPEED_KNOTS,
    // DATA_STATE_PROVINCE
    DATA_TRUE_HEADING_DEGREES,
    DATA_VERTICAL_DILUTION_OF_PRECISION,
    DATA_NMEA_EPOCH,

    // internal fields
    DATA_UTC_DATE_YEAR,
    DATA_UTC_DATE_MONTH,
    DATA_UTC_DATE_DAY,
    DATA_UTC_TIME_HOUR,
    DATA_UTC_TIME_MINUTE,
    DATA_UTC_TIME_SECOND,
    DATA_UTC_TIMESTAMP,
    //		DATA_LOCAL_TIMESTAMP,
    //		DATA_LOCALX_TIMESTAMP,

    // proprietary ubx specific fields
    DATA_UBX_TTAG,
    DATA_UBX_GPSTIME_TOW,
    DATA_UBX_GPSTIME_WEEK,
    DATA_UBX_GPSFIXOK,
    DATA_UBX_GPSFIX,
    DATA_UBX_DGPS,
    DATA_UBX_POSITION_ECEF_X,
    DATA_UBX_POSITION_ECEF_Y,
    DATA_UBX_POSITION_ECEF_Z,
    DATA_UBX_POSITION_ECEF_ACCURACY,
    DATA_UBX_VELOCITY_ECEF_VX,
    DATA_UBX_VELOCITY_ECEF_VY,
    DATA_UBX_VELOCITY_ECEF_VZ,
    DATA_UBX_VELOCITY_ECEF_ACCURACY,
    DATA_UBX_SATELLITES_IN_VIEW_NAV_STA,
#define DATA_UBX_SATELLITES_IN_VIEW_NAV_STA_(ix)                                                   \
  (CDatabase::DATA_t)(CDatabase::DATA_UBX_SATELLITES_IN_VIEW_NAV_STA + ix)
    DATA_UBX_SATELLITES_IN_VIEW_NAV_STA_X =
      MAX_SATELLITES_IN_VIEW + DATA_UBX_SATELLITES_IN_VIEW_NAV_STA,

#ifdef SUPL_ENABLED
    // SUPL MS-ASSIST properties
    DATA_UBX_GNSS_TOW,
    DATA_UBX_GNSS_DOP_CENTER,
    DATA_UBX_SATELLITES_IN_MEAS_COUNT,

    DATA_UBX_SATELLITES_IN_MEAS,
#define DATA_UBX_SATELLITES_IN_MEAS_(ix)                                                           \
  (CDatabase::DATA_t)(CDatabase::DATA_UBX_SATELLITES_IN_MEAS + ix)
    DATA_UBX_SATELLITES_IN_MEAS_X = MAX_SATELLITES_IN_VIEW + DATA_UBX_SATELLITES_IN_MEAS,

    DATA_UBX_SATELLITES_IN_MEAS_CNO,
#define DATA_UBX_SATELLITES_IN_MEAS_CNO_(ix)                                                       \
  (CDatabase::DATA_t)(CDatabase::DATA_UBX_SATELLITES_IN_MEAS_CNO + ix)
    DATA_UBX_SATELLITES_IN_MEAS_CNO_X = MAX_SATELLITES_IN_VIEW + DATA_UBX_SATELLITES_IN_MEAS_CNO,

    DATA_UBX_SATELLITES_IN_MEAS_PRRMS,
#define DATA_UBX_SATELLITES_IN_MEAS_PRRMS_(ix)                                                     \
  (CDatabase::DATA_t)(CDatabase::DATA_UBX_SATELLITES_IN_MEAS_PRRMS + ix)
    DATA_UBX_SATELLITES_IN_MEAS_PRRMS_X =
      MAX_SATELLITES_IN_VIEW + DATA_UBX_SATELLITES_IN_MEAS_PRRMS,

    DATA_UBX_SATELLITES_IN_MEAS_MULTIPATH_IND,
#define DATA_UBX_SATELLITES_IN_MEAS_MULTIPATH_IND_(ix)                                             \
  (CDatabase::DATA_t)(CDatabase::DATA_UBX_SATELLITES_IN_MEAS_MULTIPATH_IND + ix)
    DATA_UBX_SATELLITES_IN_MEAS_MULTIPATH_IND_X =
      MAX_SATELLITES_IN_VIEW + DATA_UBX_SATELLITES_IN_MEAS_MULTIPATH_IND,

    DATA_UBX_SATELLITES_IN_MEAS_REL_CODEPHASE,
#define DATA_UBX_SATELLITES_IN_MEAS_REL_CODEPHASE_(ix)                                             \
  (CDatabase::DATA_t)(CDatabase::DATA_UBX_SATELLITES_IN_MEAS_REL_CODEPHASE + ix)
    DATA_UBX_SATELLITES_IN_MEAS_REL_CODEPHASE_X =
      MAX_SATELLITES_IN_VIEW + DATA_UBX_SATELLITES_IN_MEAS_REL_CODEPHASE,

    DATA_UBX_SATELLITES_IN_MEAS_DOPPLER,
#define DATA_UBX_SATELLITES_IN_MEAS_DOPPLER_(ix)                                                   \
  (CDatabase::DATA_t)(CDatabase::DATA_UBX_SATELLITES_IN_MEAS_DOPPLER + ix)
    DATA_UBX_SATELLITES_IN_MEAS_DOPPLER_X =
      MAX_SATELLITES_IN_VIEW + DATA_UBX_SATELLITES_IN_MEAS_DOPPLER,
#endif

    // ttff
    DATA_UBX_TTFF,

    // Automatic Gain Control
    DATA_UBX_AGC,

    // Accuracies
    DATA_UBX_ACCURACY_HORIZONTAL_M,
    DATA_UBX_ACCURACY_VERTICAL_M,
    DATA_UBX_ACCURACY_SPEED_MPS,
    DATA_UBX_ACCURACY_HEADING_DEG,

    ///////////////////////////////////////////////////////////////////////////////////
    // the following fields can not be set by the parser, they have to be
    // calculated //
    ///////////////////////////////////////////////////////////////////////////////////
    DATA_PARSE,
    _DATA_PARSE_X = DATA_PARSE - 1,

    // commit timestamp
    DATA_COMMIT_TIMESTAMP,
    // last good fields
    DATA_LASTGOOD_LATITUDE_DEGREES,
    DATA_LASTGOOD_LONGITUDE_DEGREES,
    DATA_LASTGOOD_ERROR_RADIUS_METERS,
    DATA_LASTGOOD_ALTITUDE_ELLIPSOID_ERROR_METERS,
    DATA_LASTGOOD_ALTITUDE_ELLIPSOID_METERS,
    DATA_LASTGOOD_TIMESTAMP,

    // total number of fields
    DATA_NUM
  } DATA_t;

  typedef enum
  {
    MSG_NMEA_GBS,
    MSG_NMEA_GGA,
    MSG_NMEA_GLL,
    MSG_NMEA_GNS,
    MSG_NMEA_GRS,
    MSG_NMEA_GSA,
    MSG_NMEA_GST,
    MSG_NMEA_GSV,
    MSG_NMEA_RMC,
    MSG_NMEA_VTG,
    MSG_NMEA_ZDA,
    MSG_UBX_NAVSOL,
    MSG_UBX_NAVPVT,
    MSG_UBX_NAV_SVINFO,
    MSG_UBX_RXM_MEAS,
    MSG_UBX_NAV_STATUS,
    MSG_UBX_NAV_PVT,
    MSG_UBX_MON_HW,
    MSG_UBX_NAV_SAT,
    // number of fields
    MSG_NUM
  } MSG_t;

public:
  typedef enum
  {
    GNSSID_UNKN = -1,
    GNSSID_GPS = 0,
    GNSSID_SBAS = 1,
    GNSSID_GAL = 2,
    GNSSID_BDS = 3,
    GNSSID_QZSS = 5,
    GNSSID_GLO = 6,
  } GNSSID_t;

  typedef enum
  {
    HAS_EPH = 1,
    HAS_ALM = 2,
    IS_USED = 4
  } NAV_STATUS_t;

  typedef struct
  {
    int gnssid;
    int svid;
  } NMEA_t;

  virtual int Process(MSG_t type, const unsigned char *pBuffer, int iSize, PARSE_FUNC_t func);
  virtual CDatabase::STATE_t Commit(bool timeBasedEpoch);

  void Reset(void);
  bool CheckTime(int hour, int minute, double seconds) const;
  bool CheckDate(int year, int month, int day) const;
  void setBeginEpoch(MSG_t begin);
  void setEndEpoch(MSG_t end);

  static double Degrees360(double val);
  static bool ConvertUbxSvidToNmea41e(int svid, NMEA_t *nmeaOut);
  static bool ConvertNmeaToNmea41e(char t, int svid, NMEA_t *nmeaOut);

  //! Sets a field in the database to a specific value
  /*! The field 'data' in the databse will be set to the value 'v' if
      'settable' part of the databse.
    \param data : Name of the field that has to be set in the databse
    \param v    : Value that has to be assigned to the field 'data'
  */
  template <typename T> void Set(DATA_t data, T v)
  {
    if (data < DATA_PARSE)
    {
#if defined(_DEBUG) && 1
      if (!m_varN[data].Check(v))
      {
        double d;
        m_varN[data].Get(d);
        // printf("Set(%d, %f) -> Overwrite %f\n", v, d);
      }
#endif
      m_varN[data].Set(v);
    }
  }

  template <typename T> T GetNext(DATA_t data)
  {
    T v = 0;
    m_varN[data].Get(v);

    return v;
  }

  template <typename T> bool GetNext(DATA_t data, T &v)
  {
    bool result = m_varN[data].Get(v);
    return result;
  }

  template <typename T> bool GetOutput(DATA_t data, T &v) { return m_varO[data].Get(v); }

  bool GetOutput(DATA_t data, TIMESTAMP &rTS) const { return m_varO[data].Get(rTS); }

  CVar *GetOutputArrayBase(DATA_t data) { return &m_varO[data]; }

  template <typename T> bool Check(DATA_t data, T v) const
  {
    if (data < DATA_PARSE)
    {
      return m_varN[data].Check(v);
    }
    return true;
  }

  const char *GetNmeaOutputList(void) { return m_pNmeaMsgListOutput; }

  void AddNmeaMessage(const unsigned char *pBuffer, int iSize);

protected:
  double CalcLeapSeconds(double dUtcSec) const;
  void CompleteTimestamp(void);
  void CompletePosition(void);
  void CompleteVelocity(void);
  void CompleteAltitude(void);
  void CompleteHeading(void);
  void beforeProcessing(MSG_t message);
  void afterProcessing(MSG_t message);

  virtual bool GetCurrentTimestamp(TIMESTAMP &ft) = 0;
  virtual void LockOutputDatabase(void) = 0;
  virtual void UnlockOutputDatabase(void) = 0;

  virtual int Printf(const char * /*pFmt*/, ...) { return 0; }
  void Dump(const CVar *pVar);

protected:
  CVar m_varN[DATA_PARSE]{};          // Database image for 'Next' epoch
  CVar m_varO[DATA_NUM]{};            // Database image for current 'Output' reports
  MSG_t m_epochBegin{ MSG_NMEA_RMC }; // Defines the first message of an epoch
                                      // that has to arrive to allow writing to
                                      // the database
  MSG_t m_epochEnd{ MSG_NUM };        // Defines the last message of an epoch that has to
                                      // arrive to allow committing the database
  STATE_t m_vasS{ STATE_NO_DATA };    // Status of GNSS reception
  bool m_receivingEpoch{ false };     // Indicates if we are in the process of
                                      // receiving an epoch (database is writable)

  char *m_pNmeaMsgListNext{ nullptr };
  char *m_pNmeaMsgListOutput{ nullptr };
};

#endif //__DATABASE_H__
