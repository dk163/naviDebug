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

#include "ubx_gpsNavMesIf.h"
#include "protocolubx.h"
#include "ubx_log.h"
#include "ubx_messageDef.h"
#include <cstdint>
#include <string>
#include <ubx_androidHelper.h>

////////////////////////////////////////////////////////////////////////////////

static CGpsNavMesIf s_my_gps_nav_mes_interface;

const GpsNavigationMessageInterface CGpsNavMesIf::s_gps_nav_mes_interface = {
  .size = sizeof(GpsNavigationMessageInterface),
  .init = CGpsNavMesIf::init,
  .close = CGpsNavMesIf::close,
};

CGpsNavMesIf::CGpsNavMesIf() {}

int CGpsNavMesIf::init(GpsNavigationMessageCallbacks *callbacks)
{
  if (haveGpsCallback() || haveGnssCallback())
  {
    UBX_LOG(LCAT_ERROR, "already initialized");
    return GPS_NAVIGATION_MESSAGE_ERROR_ALREADY_INIT;
  }
  UBX_LOG(LCAT_VERBOSE, "");

  initializeCallbacks(*callbacks);
  CProtocolUBX::insertHandler(UBXID_RXM_SFRBX, [](const Message &message) {
    getInstance()->navMessageUpdate(static_cast<const UBX_RXM_SFRBX &>(message));
  });
  if (noCallbacksDefined())
  {
    return GPS_NAVIGATION_MESSAGE_ERROR_GENERIC;
  }
  UBX_LOG(LCAT_VERBOSE, "CGpsNavMesIf Initialized Successfully!");
  return GPS_NAVIGATION_MESSAGE_OPERATION_SUCCESS;
}

void CGpsNavMesIf::close() { clearCallbacks(); }

const GpsNavigationMessageInterface *CGpsNavMesIf::getIf(void)
{
  // UBX_LOG(LCAT_VERBOSE, "Get called from JNI through getExtention()!");
  return &(s_gps_nav_mes_interface);
}

CGpsNavMesIf *CGpsNavMesIf::getInstance(void) { return &(s_my_gps_nav_mes_interface); }

void CGpsNavMesIf::navMessageUpdate(const UBX_RXM_SFRBX &message)
{
  GnssNavigationMessage navigationMessage{};

  if (!haveGnssCallback())
  {
    UBX_LOG(LCAT_ERROR, "GnssMessageInterface callback is not initialized");
    return;
  }

  if (!Android::validGnssSystem(message.summary.gnssId))
  {
    UBX_LOG(LCAT_VERBOSE,
            "Dropping NavigationMessage with not supported GNSS-System %u",
            message.summary.gnssId);
    return;
  }

  if (!fillNavigationMessage(navigationMessage, message))
  {
    UBX_LOG(LCAT_ERROR, "Could not fill in NavigiationMessageData");
    return;
  }

  UBX_LOG(LCAT_VERBOSE, "updating GnssMessageInterface");
  s_my_gps_nav_mes_interface.m_gps_nav_mes_callbacks.gnss_navigation_message_callback(
    &navigationMessage);
}

bool CGpsNavMesIf::fillNavigationMessage(GnssNavigationMessage &navigationMessage,
                                         const UBX_RXM_SFRBX &message)
{
  if (Android::unknownGlonassSatellite(message.summary.svId))
  {
    return false;
  }
  navigationMessage.size = sizeof(GnssNavigationMessage);
  navigationMessage.svid = message.summary.svId;
  navigationMessage.type = Android::getGnssMessageType(message.summary.gnssId);
  navigationMessage.status = NAV_MESSAGE_STATUS_PARITY_PASSED;
  /* navigationMessage.message_id = ; //NOT MANDATORY */
  /* navigationMessage.submessage_id = ; //NOT MANDATORY */
  navigationMessage.data_length = message.data_wd.size() * sizeof(UBX_RXM_SFRBX_DATA_WD);
  navigationMessage.data =
    const_cast<uint8_t *>(reinterpret_cast<const uint8_t *>(message.data_wd.data()));

  return true;
}

bool CGpsNavMesIf::haveGpsCallback()
{
  return s_my_gps_nav_mes_interface.m_gps_nav_mes_callbacks.navigation_message_callback != nullptr;
}

bool CGpsNavMesIf::haveGnssCallback()
{
  return s_my_gps_nav_mes_interface.m_gps_nav_mes_callbacks.gnss_navigation_message_callback !=
         nullptr;
}

bool CGpsNavMesIf::noCallbacksDefined() { return !haveGpsCallback() && !haveGnssCallback(); }
void CGpsNavMesIf::initializeCallbacks(const GpsNavigationMessageCallbacks &callbacks)
{
  s_my_gps_nav_mes_interface.m_gps_nav_mes_callbacks = callbacks;
}

void CGpsNavMesIf::clearCallbacks()
{
  s_my_gps_nav_mes_interface.m_gps_nav_mes_callbacks.navigation_message_callback = nullptr;
  s_my_gps_nav_mes_interface.m_gps_nav_mes_callbacks.gnss_navigation_message_callback = nullptr;
}
