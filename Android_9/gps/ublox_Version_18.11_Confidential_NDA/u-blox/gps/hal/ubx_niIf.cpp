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
 * $Id$
 * $HeadURL$
 *****************************************************************************/

#include "ubx_niIf.h"
#include "suplSMmanager.h"

#include <mutex>

using namespace ublox::log::stringtables;

static CNiIf s_myIf;

const GpsNiInterface CNiIf::s_interface = {
  .size = sizeof(GpsNiInterface), .init = init, .respond = respond,
};

CNiIf::CNiIf() {}

CNiIf *CNiIf::getInstance() { return &s_myIf; }

void CNiIf::init(GpsNiCallbacks *callbacks)
{
  if (s_myIf.m_ready)
    UBX_LOG(LCAT_ERROR, "already initialized");
  UBX_LOG(LCAT_VERBOSE, "");
  s_myIf.m_callbacks = *callbacks;
  s_myIf.m_ready = true;
  // lint -e{818} remove  Pointer parameter 'callbacks' (line 53) could be
  // declared as pointing to const
}

void CNiIf::respond(int notif_id, GpsUserResponseType user_response)
{
  UBX_LOG(LCAT_VERBOSE,
          "id=%d respond=%d(%s)",
          notif_id,
          user_response,
          _LOOKUPSTR((unsigned int)user_response, GpsUserResponseType));

  s_myIf.m_cmd = SUPL_AUTH_DENIED;
  switch (user_response)
  {
  case GPS_NI_RESPONSE_ACCEPT:
    s_myIf.m_cmd = SUPL_AUTH_GRANT;
    break;

  case GPS_NI_RESPONSE_DENY:
    s_myIf.m_cmd = SUPL_AUTH_DENIED;
    break;

  case GPS_NI_RESPONSE_NORESP:
    break;

  default:
    break;
  }

  // Unlock the mutex
  s_myIf.m_mutex.unlock();
}

void CNiIf::timoutThread(void *pThreadData)
{
  UBX_LOG(LCAT_VERBOSE, "");
  ((void)(pThreadData));
}

void CNiIf::request(GpsNiNotification *pNotification)
{
  if (!s_myIf.m_ready)
  {
    UBX_LOG(LCAT_ERROR, "class not initialized");
    return;
  }
  UBX_LOG(LCAT_VERBOSE, "");

  s_myIf.m_callbacks.notify_cb(pNotification);
}

void CNiIf::lock() { m_mutex.lock(); }

SuplCmd_t CNiIf::getSuplState() const { return m_cmd; }
