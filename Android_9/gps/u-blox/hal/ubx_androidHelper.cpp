
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
 ******************************************************************************
******************************************************************************
 * $Id: $
 * $comments: $
 *****************************************************************************/
#include "ubx_androidHelper.h"
#include "std_types.h"
#include <algorithm>
#include <hardware/gps.h>

namespace
{
  static const unsigned int DUMMY_GLONASS_SVID = 255;
} // anonymous namespace

namespace Android
{

  I2 getGnssId(const unsigned int rawId)
  {
    return getGnssId(static_cast<RAW_CONSTELLATION>(rawId));
  }

  I2 getGnssId(const RAW_CONSTELLATION rawConstellation)
  {
    switch (rawConstellation)
    {
    case RAW_CONSTELLATION::GPS:
      return GNSS_CONSTELLATION_GPS;
      break;
    case RAW_CONSTELLATION::BEIDOU:
      return GNSS_CONSTELLATION_BEIDOU;
      break;
    case RAW_CONSTELLATION::GLONASS:
      return GNSS_CONSTELLATION_GLONASS;
      break;
    case RAW_CONSTELLATION::GALILEO:
      return GNSS_CONSTELLATION_GALILEO;
      break;
    case RAW_CONSTELLATION::SBAS:
      return GNSS_CONSTELLATION_SBAS;
      break;
    default:
      return GNSS_CONSTELLATION_UNKNOWN;
    }
  }

  I2 getGnssMessageType(const unsigned int rawId)
  {
    return getGnssMessageType(static_cast<RAW_CONSTELLATION>(rawId));
  }
  I2 getGnssMessageType(const RAW_CONSTELLATION rawConstellation)
  {
    switch (rawConstellation)
    {
    case RAW_CONSTELLATION::GPS:
      return GNSS_NAVIGATION_MESSAGE_TYPE_GPS_L1CA;
      break;
    case RAW_CONSTELLATION::BEIDOU:
      return GNSS_NAVIGATION_MESSAGE_TYPE_BDS_D1;
      break;
    case RAW_CONSTELLATION::GLONASS:
      return GNSS_NAVIGATION_MESSAGE_TYPE_GLO_L1CA;
      break;
    case RAW_CONSTELLATION::GALILEO:
      return GNSS_NAVIGATION_MESSAGE_TYPE_GAL_I;
      break;
    case RAW_CONSTELLATION::SBAS:
    case RAW_CONSTELLATION::IMES:
    case RAW_CONSTELLATION::QZSS:
    default:
      return GNSS_NAVIGATION_MESSAGE_TYPE_UNKNOWN;
      break;
    }
  }

  /**
    \brief check if this is a supported GNSS type
    As of now (Android 8.1) only GPS, GLO, BDS and GAL are supported.
    This function will probably have to be changed as AOSP incorporates new
    systems.
    \param RAWX message
    \return true if supported, false otherwise
  */
  bool validGnssSystem(const unsigned int gnssId)
  {
    Android::RAW_CONSTELLATION constellationId = static_cast<Android::RAW_CONSTELLATION>(gnssId);
    return std::find(std::begin(Android::SupportedGnssSystems),
                     std::end(Android::SupportedGnssSystems),
                     constellationId) != std::end(Android::SupportedGnssSystems);
  }

  /**
    \brief Check if this is an unknown Glonass sattellite
    \param sattellite ID (should be from a Glonass system)
    \return true if the number (svId) from this Glonass satellite is still unknown
  */
  bool unknownGlonassSatellite(unsigned int svId)
  {
    return static_cast<U1>(svId) == DUMMY_GLONASS_SVID;
  }

  /**
    \brief Compute GLONASS carrier frequency
    \param Frequency base (should be GLO_L1_BASE or GLO_L2_BASE)
    \param Frequency offset (should be GLO_L1_OFFS or GLO_L2_OFFS)
    \param Frequency Id (between 0 and 13 included)
    \return Computed GLONASS carrier frequency
  */
  U8 glonassFreq(CARRIER_FREQ base, CARRIER_FREQ offset, U8 freqId)
  {
    return static_cast<U8>(base) + (freqId - 7) * static_cast<U8>(offset);
  }

  /**
    \brief Compute GLONASS carrier frequency for L1 base
    \param Frequency Id (between 0 and 13 included)
    \return Computed GLONASS carrier frequency
  */
  U8 glonassFreqL1(U8 freqId)
  {
    return glonassFreq(CARRIER_FREQ::GLO_L1_BASE, CARRIER_FREQ::GLO_L1_OFFS, freqId);
  }

  /**
    \brief Compute GLONASS carrier frequency for L2 base
    \param Frequency Id (between 0 and 13 included)
    \return Computed GLONASS carrier frequency
  */
  U8 glonassFreqL2(U8 freqId)
  {
    return glonassFreq(CARRIER_FREQ::GLO_L2_BASE, CARRIER_FREQ::GLO_L2_OFFS, freqId);
  }

} // namespace Android
