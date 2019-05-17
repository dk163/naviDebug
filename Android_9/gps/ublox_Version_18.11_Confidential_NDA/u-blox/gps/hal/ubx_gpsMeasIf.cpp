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
 * Project: Android GNSS Driver
 *
 *****************************************************************************/

#include "ubx_gpsMeasIf.h"
#include "protocolubx.h"
#include "ubx_localDb.h"
#include "ubx_messageDef.h"
#include "ubx_units.h"
#include <cmath>
#include <string>
#include <ubx_androidHelper.h>
#include <vector>

////////////////////////////////////////////////////////////////////////////////
using namespace ublox::Units;
namespace
{
  static const U8 NS_IN_S = 1e9;      //!< Nanoseconds in one second
  static const U8 R8_SOL = 299792458; //!< Speed of light (R8) [m/s]

  //! Number of constellation (see RAW_CONSTELLATION enum)
  static const U8 NUM_GNSSID = 7;

  //! Number of (new) sigId used (see 'Signal Identifiers' in protocol spec)
  static const U8 NUM_SIGID = 11;

  //!Carrier phase uncertainty factor
  static const double CARRIER_PHASE_UNCERTAINTY_FACTOR = 0.004;

  //! Frequency table indexed by Constellation type and new UBX sigId
  struct FrequencyTable
  {
    U8 table[NUM_GNSSID][NUM_SIGID]{};

    // define [] operator to access table
    const U8 *operator[](U8 idx) const { return table[idx]; }

    // initialize table
    FrequencyTable()
    {
      // initialize GPS sigIds
      U8 gps = static_cast<U8>(Android::RAW_CONSTELLATION::GPS);
      table[gps][0] = (U8)(Android::CARRIER_FREQ::GPS_L1);
      table[gps][3] = (U8)(Android::CARRIER_FREQ::GPS_L2);
      table[gps][4] = (U8)(Android::CARRIER_FREQ::GPS_L2);


      // initialize GALILEO sigIds
      U8 gal = static_cast<U8>(Android::RAW_CONSTELLATION::GALILEO);
      table[gal][0] = (U8)(Android::CARRIER_FREQ::GAL_E1);
      table[gal][1] = (U8)(Android::CARRIER_FREQ::GAL_E1);
      table[gal][5] = (U8)(Android::CARRIER_FREQ::GAL_E5B);
      table[gal][6] = (U8)(Android::CARRIER_FREQ::GAL_E5B);

      // initialize BEIDOU sigIds
      U8 bds = static_cast<U8>(Android::RAW_CONSTELLATION::BEIDOU);
      table[bds][0] = (U8)(Android::CARRIER_FREQ::BDS_B1);
      table[bds][1] = (U8)(Android::CARRIER_FREQ::BDS_B1);
      table[bds][2] = (U8)(Android::CARRIER_FREQ::BDS_B2);
      table[bds][3] = (U8)(Android::CARRIER_FREQ::BDS_B2);


      // initialize QZSS sigIds
      U8 qzss = static_cast<U8>(Android::RAW_CONSTELLATION::QZSS);
      table[qzss][0] = (U8)(Android::CARRIER_FREQ::QZSS_L1);

      // initialize GLONASS sigIds
      U8 glo = static_cast<U8>(Android::RAW_CONSTELLATION::GLONASS);
      table[glo][0] = (U8)(Android::CARRIER_FREQ::GLO_L1_BASE);
      table[glo][2] = (U8)(Android::CARRIER_FREQ::GLO_L2_BASE);
    };
  };

  static const FrequencyTable FREQUENCY_FROM_SIGID{};

  static const U8 MOSCOW_OFFSET = 3LL * 1.0_hours; //! Offset for GLO from UTC to Moscow time
  static const X1 RECSTAT_LEAPS_FLAG = 0x01;
  static const X1 RECSTAT_CLKRESET_FLAG = 0x02;

  /** Return gps time in seconds */
  uint64_t getGpsTime(const UBX_RXM_RAWX_SUMMARY &summary)
  {
    return static_cast<uint64_t>((summary.rcvTow + summary.week * 1.0_weeks) * NS_IN_S);
  }

  bool hasNoMeasurements(const UBX_RXM_RAWX &rawMessage) { return rawMessage.summary.numMeas <= 0; }

  R8 adjustToGlonassSigToW(R8 sigtow_s, const UBX_RXM_RAWX_SUMMARY &summary)
  {
    /* For GLO: Received Glonass time of day, at the measurement time in
     * nanoseconds.*/
    sigtow_s += (MOSCOW_OFFSET - summary.leaps);
    /*  For Glonass we return Glonass time of day, bound check here */
    sigtow_s = fmod(sigtow_s, 1.0_days);
    return sigtow_s;
  }

  R8 adjustToSbasSigToW(R8 sigtow_s)
  {
    /* For Sbas: Received sigtow seconds part */
    return fmod(sigtow_s, 1.0);
  }

  R8 adjustToGpsSigToW(R8 sigtow_s)
  {
    /* Check week rollovers */
    if (sigtow_s > 1.0_weeks)
    {
      sigtow_s -= 1.0_weeks;
    }
    else if (sigtow_s < 0)
    {
      sigtow_s += 1.0_weeks;
    }
    return sigtow_s;
  }

  R8 getSigToW(const UBX_RXM_RAWX_INDIVIDUAL &measurement, const UBX_RXM_RAWX_SUMMARY &summary)
  {
    R8 sigtow_s = (summary.rcvTow - measurement.prMes / R8_SOL);
    switch (static_cast<Android::RAW_CONSTELLATION>(measurement.gnssId))
    {
    case Android::RAW_CONSTELLATION::GLONASS:
      sigtow_s = adjustToGlonassSigToW(sigtow_s, summary);
      break;
    case Android::RAW_CONSTELLATION::SBAS:
      sigtow_s = adjustToSbasSigToW(sigtow_s);
      break;
    case Android::RAW_CONSTELLATION::QZSS:
    case Android::RAW_CONSTELLATION::GPS:
    case Android::RAW_CONSTELLATION::GALILEO:
    case Android::RAW_CONSTELLATION::BEIDOU:
    case Android::RAW_CONSTELLATION::IMES:
      sigtow_s = adjustToGpsSigToW(sigtow_s);
      break;
    }
    return sigtow_s;
  }

  R8 getCarrierFrequencyOldSigId(const UBX_RXM_RAWX_INDIVIDUAL &measurement)
  {
    switch (static_cast<Android::RAW_CONSTELLATION>(measurement.gnssId))
    {
    case Android::RAW_CONSTELLATION::GPS:
      return static_cast<R8>(Android::CARRIER_FREQ::GPS_L1);
    case Android::RAW_CONSTELLATION::SBAS:
      return static_cast<R8>(Android::CARRIER_FREQ::SBAS_L1);
    case Android::RAW_CONSTELLATION::GALILEO:
      return static_cast<R8>(Android::CARRIER_FREQ::GAL_E1);
    case Android::RAW_CONSTELLATION::BEIDOU:
      return static_cast<R8>(Android::CARRIER_FREQ::BDS_B1);
    case Android::RAW_CONSTELLATION::IMES:
      return static_cast<R8>(Android::CARRIER_FREQ::IMES_L1);
    case Android::RAW_CONSTELLATION::QZSS:
      return static_cast<R8>(Android::CARRIER_FREQ::QZSS_L1);
    case Android::RAW_CONSTELLATION::GLONASS:
      return Android::glonassFreqL1(measurement.freqId);
    }
  }

  R8 getCarrierFrequencyNewSigId(const UBX_RXM_RAWX_INDIVIDUAL &measurement)
  {
    // check if gnssId and sigId are valid
    if (measurement.gnssId >= NUM_GNSSID || measurement.sigId >= NUM_SIGID)
      return 0;

    // GLONASS specific case handling
    Android::RAW_CONSTELLATION gnssId = static_cast<Android::RAW_CONSTELLATION>(measurement.gnssId);
    if (gnssId == Android::RAW_CONSTELLATION::GLONASS)
    {
      switch (measurement.sigId)
      {
      case 0:
      case 1:
      case 7:
        return Android::glonassFreqL1(measurement.freqId);
      case 2:
      case 3:
        return Android::glonassFreqL2(measurement.freqId);
      case 4:
      case 5:
        // for L3 values, frequency is static and specified in the table
        break;
      default:
        // unknown sigId for GLONASS
        return 0;
      }
    }

    // get frequency from table
    return static_cast<R8>(FREQUENCY_FROM_SIGID[measurement.gnssId][measurement.sigId]);
  }

  GnssMeasurementState getGpsState()
  {
    return GNSS_MEASUREMENT_STATE_CODE_LOCK | GNSS_MEASUREMENT_STATE_BIT_SYNC |
           GNSS_MEASUREMENT_STATE_SUBFRAME_SYNC | GNSS_MEASUREMENT_STATE_TOW_DECODED;
  }

  GnssMeasurementState getGlonassState()
  {
    return GNSS_MEASUREMENT_STATE_CODE_LOCK | GNSS_MEASUREMENT_STATE_CODE_LOCK |
           GNSS_MEASUREMENT_STATE_SYMBOL_SYNC | GNSS_MEASUREMENT_STATE_BIT_SYNC |
           GNSS_MEASUREMENT_STATE_GLO_STRING_SYNC | GNSS_MEASUREMENT_STATE_GLO_TOD_DECODED;
  }

  GnssMeasurementState getBeidouState()
  {
    return GNSS_MEASUREMENT_STATE_CODE_LOCK | GNSS_MEASUREMENT_STATE_BDS_D2_BIT_SYNC |
           GNSS_MEASUREMENT_STATE_BIT_SYNC | GNSS_MEASUREMENT_STATE_BDS_D2_SUBFRAME_SYNC |
           GNSS_MEASUREMENT_STATE_TOW_DECODED;
  }

  GnssMeasurementState getGalileoState()
  {
    return GNSS_MEASUREMENT_STATE_GAL_E1BC_CODE_LOCK |
           GNSS_MEASUREMENT_STATE_GAL_E1C_2ND_CODE_LOCK | GNSS_MEASUREMENT_STATE_GAL_E1B_PAGE_SYNC |
           GNSS_MEASUREMENT_STATE_TOW_DECODED;
  }

  GnssMeasurementState getSbasState()
  {
    return GNSS_MEASUREMENT_STATE_CODE_LOCK | GNSS_MEASUREMENT_STATE_SYMBOL_SYNC |
           GNSS_MEASUREMENT_STATE_SBAS_SYNC;
  }

  GnssMeasurementState getMeasurementState(GnssConstellationType &constellation)
  {
    /** We only output RAWX once everything is locked.
  Thus we simply return the maximum accuracy achievable by locked state for each
  GNSS system as described in gps.h
     **/
    // TODO ask christoph if this is correct
    switch (constellation)
    {
    case GNSS_CONSTELLATION_GPS:
    case GNSS_CONSTELLATION_QZSS:
      return getGpsState();
      break;
    case GNSS_CONSTELLATION_GLONASS:
      return getGlonassState();
      break;
    case GNSS_CONSTELLATION_BEIDOU:
      return getBeidouState();
      break;
    case GNSS_CONSTELLATION_GALILEO:
      return getGalileoState();
      break;
    case GNSS_CONSTELLATION_SBAS:
      return getSbasState();
      break;
    default:
      return 0;
      break;
    }
  }

  double getAgcLevelDb()
  {
    // agcCnt is in the range 0 to 8191
    int agcCnt = CAndroidDatabase::getInstance()->getAgc();
    if (agcCnt < 0 || agcCnt > 8191)
      return -1;

    // get a percentage from agcCnt
    double percentage = (double)agcCnt / 8191.;

    // cannot calculate a value for 0 (yields -inf dB)
    if (percentage == 0)
      percentage = 0.01;

    // convert to dB with nominal value = 0.3 (0.3 -> 0 dB)
    double agcLevelDb = 70 * (percentage - 0.3);

    return agcLevelDb;
  }

  bool isUnknownGlonassSatellite(const UBX_RXM_RAWX_INDIVIDUAL &measurement)
  {
    return (static_cast<Android::RAW_CONSTELLATION>(measurement.gnssId) ==
            Android::RAW_CONSTELLATION::GLONASS) &&
           Android::unknownGlonassSatellite(measurement.svId);
  }
} // anonymous namespace

static CGpsMeasIf s_my_gps_meas_interface;

const GpsMeasurementInterface CGpsMeasIf::s_gps_meas_interface = {
  .size = sizeof(GpsMeasurementInterface),
  .init = CGpsMeasIf::init,
  .close = CGpsMeasIf::close,
};
uint32_t CGpsMeasIf::m_clockResetCount = 0;

CGpsMeasIf::CGpsMeasIf() {}

int CGpsMeasIf::init(GpsMeasurementCallbacks *callbacks)
{
  if (haveGpsCallback() || haveGnssCallback())
  {
    UBX_LOG(LCAT_ERROR, "already initialized");
    return GPS_MEASUREMENT_ERROR_ALREADY_INIT;
  }
  UBX_LOG(LCAT_VERBOSE, "");

  initializeCallbacks(*callbacks);
  resetClkResetCount();
  CProtocolUBX::insertHandler(UBXID_RXM_RAWX, [](const Message &message) {
    getInstance()->navMeasurementUpdate(static_cast<const UBX_RXM_RAWX &>(message));
  });
  CProtocolUBX::insertHandler(UBXID_NAV_CLOCK, [](const Message &message) {
    getInstance()->setClockDrift(static_cast<const GPS_UBX_NAV_CLOCK_t &>(message).clkD);
    getInstance()->setClockUncertainty(static_cast<const GPS_UBX_NAV_CLOCK_t &>(message).fAcc);
  });

  if (noCallbacksDefined())
  {
    return GPS_MEASUREMENT_ERROR_GENERIC;
    UBX_LOG(LCAT_VERBOSE, "CGpsMeasIf initialization error!");
  }
  UBX_LOG(LCAT_VERBOSE, "CGpsMeasIf Initialized Successfully!");
  return GPS_MEASUREMENT_OPERATION_SUCCESS;
}

void CGpsMeasIf::close()
{
  clearCallbacks();
  resetClkResetCount();
}

const GpsMeasurementInterface *CGpsMeasIf::getIf(void) { return &(s_gps_meas_interface); }

CGpsMeasIf *CGpsMeasIf::getInstance(void) { return &(s_my_gps_meas_interface); }

void CGpsMeasIf::navMeasurementUpdate(const UBX_RXM_RAWX &rawMessage)
{
  GnssData gnssData{};

  if (!haveGnssCallback())
  {
    UBX_LOG(LCAT_ERROR, "GnssMeasurementInterface callback is not initialized");
    return;
  }

  if (!fillGnssData(gnssData, rawMessage))
  {
    UBX_LOG(LCAT_ERROR, "Could not fill in GnssData");
    return;
  }

  UBX_LOG(LCAT_VERBOSE, "updating GnssMeasurementInterface");

#if (PLATFORM_SDK_VERSION >= 26 /* >=8.0 */)
  GnssDataExt gnssDataExt{};

  // copy old struct to new struct
  gnssDataExt.gnss_data = gnssData;

  // fill new data. For now only one field: AGC. It is not part of the RAWX
  // message but stored in the parser database so rawMessage is not needed.
  if (!fillGnssDataExt(gnssDataExt))
  {
    UBX_LOG(LCAT_ERROR, "Could not fill in GnssDataExt");
    return;
  }

  s_my_gps_meas_interface.m_gps_meas_callbacks.gnss_measurement_callback(&gnssDataExt.gnss_data);

#else /* <8.0 */

  s_my_gps_meas_interface.m_gps_meas_callbacks.gnss_measurement_callback(&gnssData);

#endif
}

GnssMeasurementFlags CGpsMeasIf::getFlags(const UBX_RXM_RAWX_INDIVIDUAL & /*  measurement*/)
{
  return static_cast<GnssMeasurementFlags>(
    GNSS_MEASUREMENT_HAS_CARRIER_CYCLES | GNSS_MEASUREMENT_HAS_SNR |
    GNSS_MEASUREMENT_HAS_CARRIER_FREQUENCY | GNSS_MEASUREMENT_HAS_CARRIER_PHASE);
}

GnssMultipathIndicator
CGpsMeasIf::getMultipathIndicator(const UBX_RXM_RAWX_INDIVIDUAL & /* measurement */)
{
  // we currently do not support multipath indicator
  return static_cast<GnssMultipathIndicator>(GNSS_MULTIPATH_INDICATOR_UNKNOWN);
}

/**
*/
/*! \brief Tries to fill in GnssData
 *
 *  Detailed description
 *
 * \param gnssData GnssData to fill
 * \param rawMessage RawMessage to get data form
 * \return true if data has been sucessfully filled, false otherwise
 */
bool CGpsMeasIf::fillGnssData(GnssData &gnssData, const UBX_RXM_RAWX &rawMessage)
{
  // Android can't handle Measurement message with empty messages
  if (hasNoMeasurements(rawMessage))
  {
    return false;
  }
  gnssData.size = sizeof(GnssData);
  gnssData.measurement_count = fillMeasurementData(gnssData.measurements, rawMessage);
  // static_cast<size_t>(rawMessage.summary.numMeas) < GNSS_MAX_MEASUREMENT ?
  // static_cast<size_t>(rawMessage.summary.numMeas) : GNSS_MAX_MEASUREMENT;

  if (!internalClockBiasInitialized())
  {
    initializeInternalClock(rawMessage.summary);
  }

  if (!fillClockData(gnssData.clock, rawMessage) || (gnssData.measurement_count <= 0))
  {
    return false;
  }

  return true;
}

#if (PLATFORM_SDK_VERSION >= 26 /* >=8.0 */)
/*! \brief Tries to fill in GnssDataExt
 *
 *  Fill in additional fields only used in the new HIDL interface. For now the
 *  only field is AGC. This field is part of the MON-HW message and is stored
 *  in the parser database.
 *
 *  Note: the GnssData struct must already be filled.
 *
 * \param gnssDataExt GnssDataExt to fill
 * \return true if data has been sucessfully filled, false otherwise
 */
bool CGpsMeasIf::fillGnssDataExt(GnssDataExt &gnssDataExt)
{
  double agcLevelDb = getAgcLevelDb();
  if (agcLevelDb < 0)
    return false;

  // there is only one AGC value, so we use it for each measurement
  for (size_t i = 0; i < gnssDataExt.gnss_data.measurement_count; ++i)
  {
    gnssDataExt.gnss_data.measurements[i].flags |= GNSS_MEASUREMENT_HAS_AUTOMATIC_GAIN_CONTROL;
    gnssDataExt.measurements[i].agc_level_db = agcLevelDb;
  }

  return true;
}
#endif

/*! \brief fill GnssClock with RAWX data
 *
 *  Fill all clock data from RAWX message in to GnssClock structure
 *
 * \param clock Clock structure to fill data in to
 * \param rawMessage rawMessage to get data from
 * \return true if success, false otherwise
 */
bool CGpsMeasIf::fillClockData(GnssClock &clock, const UBX_RXM_RAWX &rawMessage)
{

  clock.size = sizeof(GnssClock);
  if (rawMessage.summary.recStat & RECSTAT_LEAPS_FLAG)
  {
    clock.flags |= GNSS_CLOCK_HAS_LEAP_SECOND;
  }
  clock.flags = clock.flags | GNSS_CLOCK_HAS_BIAS | GNSS_CLOCK_HAS_BIAS_UNCERTAINTY |
                GNSS_CLOCK_HAS_FULL_BIAS | GNSS_CLOCK_HAS_TIME_UNCERTAINTY | GNSS_CLOCK_HAS_DRIFT |
                GNSS_CLOCK_HAS_DRIFT_UNCERTAINTY;
  clock.leap_second = static_cast<int16_t>(rawMessage.summary.leaps);
  clock.time_ns = computeGnssClockTime(getGpsTime(rawMessage.summary));
  clock.time_uncertainty_ns = 0;
  clock.full_bias_ns = computeGnssClockBias(getGpsTime(rawMessage.summary));
  clock.bias_ns = 0;
  clock.bias_uncertainty_ns = 20000; /* Hardcoded value, according to csch */
  clock.drift_nsps = clockDrift_;
  clock.drift_uncertainty_nsps = clockUncertainty_;
  clock.hw_clock_discontinuity_count = computeClkResetCount(rawMessage.summary);
  return true;
}

/*! \brief fill Measurements with RAWX data
 *
 *  Fill all measurements from RAWX message in to GnssMeasurement structure,
 *  up to GNSS_MAX_MEASUREMENT measurements
 *
 * \param measurements Measurement to fill data in to
 * \param rawMessage rawMessage to get data from
 * \return true if success, false otherwise
 */
unsigned int CGpsMeasIf::fillMeasurementData(GnssMeasurement *measurements,
                                             const UBX_RXM_RAWX &rawMessage)
{
  unsigned int index = 0;

  // in version 0 of RXM_RAWX messages, frequencies are static for each
  // constellation, whereas in version >1, frequency depends on the sigId field
  auto getCarrierFrequency =
    rawMessage.summary.version == 0 ? getCarrierFrequencyOldSigId : getCarrierFrequencyNewSigId;

  for (const auto &measurement : rawMessage.individual)
  {
    if ((!Android::validGnssSystem(measurement.gnssId)) || isUnknownGlonassSatellite(measurement))
    {
      UBX_LOG(LCAT_VERBOSE,
              "Dropping not supported measurement from GNSS system %u, satellite: %u",
              measurement.gnssId,
              measurement.svId);
      continue;
    }

    if (index >= GNSS_MAX_MEASUREMENT)
      break;

    R8 carrierFreq = getCarrierFrequency(measurement);
    R8 wavelength = carrierFreq != 0 ? 1.0 * R8_SOL / carrierFreq : 0;

    GnssMeasurement &m = measurements[index];

    m.size = sizeof(GnssMeasurement);
    m.flags = m.flags | getFlags(measurement);
    m.svid = measurement.svId;
    m.constellation = Android::getGnssId(measurement.gnssId);
    m.time_offset_ns = 0;
    m.state = getMeasurementState(m.constellation);
    m.received_sv_time_in_ns = getSigToW(measurement, rawMessage.summary) * NS_IN_S;
    m.received_sv_time_uncertainty_in_ns = ldexpf(2.0, measurement.prStdev) * (1.0 * 1e7 / R8_SOL);
    m.c_n0_dbhz = static_cast<double>(measurement.cno);
    m.pseudorange_rate_mps = -1.0 * measurement.doMes * wavelength;
    m.pseudorange_rate_uncertainty_mps = ldexpf(0.002f * wavelength, measurement.doStdev);
    m.carrier_frequency_hz = carrierFreq;
    m.carrier_cycles = static_cast<int64_t>(measurement.cpMes);

    // ADR is only set if carrier phase is valid (see below)
    m.accumulated_delta_range_state = GNSS_ADR_STATE_UNKNOWN;

    // carrier phase
    double whole;
    double frac = std::modf(measurement.cpMes, &whole);
    m.carrier_phase = frac;

    // only fill in the carrier phase uncertainty if valid
    if (measurement.cpStdev != 0x0F)
    {
      m.carrier_phase_uncertainty =
        static_cast<double>(measurement.cpStdev * CARRIER_PHASE_UNCERTAINTY_FACTOR);
      m.flags |= static_cast<GnssMeasurementFlags>(GNSS_MEASUREMENT_HAS_CARRIER_PHASE_UNCERTAINTY);

      // ADR also needs carrierFreq to be valid
      if (carrierFreq != 0)
      {
        m.accumulated_delta_range_state = GNSS_ADR_STATE_VALID;
        m.accumulated_delta_range_m = -1.0 * wavelength * m.carrier_phase;
        m.accumulated_delta_range_uncertainty_m = -1.0 * wavelength * m.carrier_phase_uncertainty;
      }
    }

    m.multipath_indicator = getMultipathIndicator(measurement);
    m.snr_db = static_cast<double>(measurement.cno);

    index++;
  }
  return index;
}

bool CGpsMeasIf::haveGpsCallback()
{
  return s_my_gps_meas_interface.m_gps_meas_callbacks.measurement_callback != nullptr;
}

bool CGpsMeasIf::haveGnssCallback()
{
  return s_my_gps_meas_interface.m_gps_meas_callbacks.gnss_measurement_callback != nullptr;
}

bool CGpsMeasIf::noCallbacksDefined() { return !haveGpsCallback() && !haveGnssCallback(); }
void CGpsMeasIf::initializeCallbacks(const GpsMeasurementCallbacks &callbacks)
{
  s_my_gps_meas_interface.m_gps_meas_callbacks = callbacks;
}

void CGpsMeasIf::clearCallbacks()
{
  s_my_gps_meas_interface.m_gps_meas_callbacks.measurement_callback = nullptr;
  s_my_gps_meas_interface.m_gps_meas_callbacks.gnss_measurement_callback = nullptr;
}

uint32_t CGpsMeasIf::computeClkResetCount(const UBX_RXM_RAWX_SUMMARY &summary)
{
  if (summary.recStat & RECSTAT_CLKRESET_FLAG)
    m_clockResetCount++;
  return m_clockResetCount;
}

void CGpsMeasIf::resetClkResetCount() { m_clockResetCount = 0; }
void CGpsMeasIf::setClockDrift(const double drift) { clockDrift_ = drift; }

void CGpsMeasIf::setClockUncertainty(double uncertainty) { clockUncertainty_ = uncertainty; }

void CGpsMeasIf::setInternalClockBias(uint64_t bias) { gpsInternalClockBias_ = bias; }
uint64_t CGpsMeasIf::getInternalClockBias() const { return gpsInternalClockBias_; }

bool CGpsMeasIf::internalClockBiasInitialized() const { return gpsInternalClockBias_ != 0; }

void CGpsMeasIf::initializeInternalClock(const UBX_RXM_RAWX_SUMMARY &rawMessageSummary)
{
  setInternalClockBias(getGpsTime(rawMessageSummary));
}

uint64_t CGpsMeasIf::computeGnssClockTime(uint64_t timeNow) const
{
  return timeNow - getInternalClockBias();
}

uint64_t CGpsMeasIf::computeGnssClockBias(uint64_t /*timeNow*/) const
{
  return -getInternalClockBias();
}
