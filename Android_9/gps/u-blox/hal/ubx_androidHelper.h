
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
#pragma once

#ifndef ANDROID_BUILD
#error "Including Android only file with none android build"
#endif

#include "std_types.h"
#include <array>
#include <string>

namespace Android
{
  enum class RAW_CONSTELLATION
  {
    GPS = 0,
    SBAS = 1,
    GALILEO = 2,
    BEIDOU = 3,
    IMES = 4,
    QZSS = 5,
    GLONASS = 6

  };

  // supported Gnss-systems as of Android 8.1 - GNSS interface 1.0
  static const std::array<RAW_CONSTELLATION, 4> SupportedGnssSystems{
    { RAW_CONSTELLATION::GPS,
      RAW_CONSTELLATION::GLONASS,
      RAW_CONSTELLATION::BEIDOU,
      RAW_CONSTELLATION::GALILEO }
  };

  // carrier frequencies
  enum class CARRIER_FREQ
  {
    // GPS
    GPS_L1 = 1575420000, //!< GPS L1 carrier frequency
    GPS_L2 = 1227600000, //!< GPS L2 carrier frequency
    GPS_L5 = 1176450000, //!< GPS L5 carrier frequency

    // SBAS
    SBAS_L1 = GPS_L1, //!< SBAS L1 carrier frequency
    SBAS_L5 = GPS_L5, //!< SBAS L5 carrier frequency

    // GAL
    GAL_E1 = 1575420000,  //!< Galileo E1 carrier frequency
    GAL_E5 = 1191795000,  //!< Galileo E5 AltBOC carrier frequency
    GAL_E5A = 1176450000, //!< Galileo E5a carrier frequency
    GAL_E5B = 1207140000, //!< Galileo E5b carrier frequency
    GAL_E6 = 1278750000,  //!< Galileo E6 carrier frequency

    // BDS
    BDS_B1 = 1561098000, //!< BeiDou B1 carrier frequency
    BDS_B2 = 1207140000, //!< BeiDou B2 carrier frequency
    BDS_B3 = 1268520000, //!< BeiDou B3 carrier frequency

    // IMES
    IMES_L1 = GPS_L1, //!< IMES L1 carrier frequency

    // QZSS
    QZSS_L1 = GPS_L1, //!< QZSS L1 carrier frequency
    QZSS_L5 = GPS_L5, //!< QZSS L5 carrier frequency

    // GLO
    GLO_L1_BASE = 1602000000, //!< GLO L1 base carrier frequency
    GLO_L1_OFFS = 562500,     //!< GLO L1 slot offset, carrier channel spacing
    GLO_L2_BASE = 1246000000, //!< GLO L2 base carrier frequency
    GLO_L2_OFFS = 437500,     //!< GLO L2 slot offset, carrier channel spacing
    GLO_L3 = 1201000000,      //!< GLO L3 carrier frequency
  };

  static const std::string UBLOX_CONFIG_FILE =
    "/system/etc/u-blox.conf"; //!< Default path to the u-blox config file, will try to replace 'system' with 'vendor'
  static const std::string AIDING_DATA_FILE =
    "/data/persistance.agnss"; //!< Default path to the persistence file
  static const std::string SERPORT_DEFAULT = "/dev/ttyACM0"; //!< Default receiver interface

  I2 getGnssId(const unsigned int rawId);
  I2 getGnssId(const RAW_CONSTELLATION rawConstellation);
  I2 getGnssMessageType(const unsigned int rawId);
  I2 getGnssMessageType(const RAW_CONSTELLATION rawId);
  bool validGnssSystem(const unsigned int gnssId);
  bool unknownGlonassSatellite(unsigned int svId);
  U8 glonassFreq(CARRIER_FREQ base, CARRIER_FREQ offset, U8 freqId);
  U8 glonassFreqL1(U8 freqId);
  U8 glonassFreqL2(U8 freqId);

} // namespace Android
