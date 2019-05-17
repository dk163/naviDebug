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

#pragma once

#include "std_inc.h"
#include "ubx_messageDef.h"

class CGpsNavMesIf
{
public:
  CGpsNavMesIf();

  static const GpsNavigationMessageInterface *getIf(void);
  static CGpsNavMesIf *getInstance(void);
  void navMessageUpdate(const UBX_RXM_SFRBX &message);

private:
  static int init(GpsNavigationMessageCallbacks *callbacks);
  static void close();
  bool fillNavigationMessage(GnssNavigationMessage &navigationMessage,
                             const UBX_RXM_SFRBX &message);
  static bool haveGpsCallback();
  static bool haveGnssCallback();
  static bool noCallbacksDefined();
  static void initializeCallbacks(const GpsNavigationMessageCallbacks &callbacks);
  static void clearCallbacks();

  // interface
  static const GpsNavigationMessageInterface s_gps_nav_mes_interface;
  GpsNavigationMessageCallbacks m_gps_nav_mes_callbacks{};
};