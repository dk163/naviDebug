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

#include "ubx_gnssConfIf.h"
#include "ubx_cfg.h"
#include "ubxgpsstate.h"
#include <ctype.h>
#include <memory>
#include <string.h>
////////////////////////////////////////////////////////////////////////////////

static CGnssConfIf s_my_gnss_configuration_interface;

const GnssConfigurationInterface CGnssConfIf::s_gnss_configuration_interface = {
  .size = sizeof(GnssConfigurationInterface),
  .configuration_update = CGnssConfIf::configuration_update,
};

CGnssConfIf::CGnssConfIf() {}

const GnssConfigurationInterface *CGnssConfIf::getIf(void)
{
  UBX_LOG(LCAT_VERBOSE, "Get called from JNI through getExtention()!");
  return &(s_gnss_configuration_interface);
}

CGnssConfIf *CGnssConfIf::getInstance(void) { return &(s_my_gnss_configuration_interface); }

void CGnssConfIf::configuration_update(const char *config_data, int32_t length)
{
  UBX_LOG(LCAT_VERBOSE, "New configuration requested");

  // Handle the configuration contents
  CCfg configuration;
  configuration.load(config_data, length);
  configuration.print();
  CUbxGpsState *gpsState = CUbxGpsState::getInstance();
  if (gpsState != nullptr)
  {
    // TODO is it correct that we re-init the gpsState?
    // This will (at the moment) hard reset the device!
    gpsState->updateConfig(configuration);
  }
}
