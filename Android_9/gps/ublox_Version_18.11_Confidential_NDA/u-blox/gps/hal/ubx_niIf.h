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

#pragma once

#include "suplSMmanager.h"
#include "ubx_niIf.h"
#include <mutex>

class CNiIf
{
public:
  CNiIf();
  static const void *getIf(void) { return &s_interface; }
  static CNiIf *getInstance(void);
  /** lock internal semaphore, improvement over exposed mutex */
  void lock();
  /** get SuplState */
  SuplCmd_t getSuplState() const;

  // callbacks
  static void request(GpsNiNotification *pNotification);

private:
  // interface
  static void init(GpsNiCallbacks *callbacks);
  static void respond(int notif_id, GpsUserResponseType user_response);

  // variables
  static const GpsNiInterface s_interface;
  GpsNiCallbacks m_callbacks{};
  bool m_ready{false};
  SuplCmd_t m_cmd{SUPL_AUTH_DENIED};
  std::mutex m_mutex{};

  // impelementation
  static void timoutThread(void *pThreadData);
};
