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

#include <arpa/inet.h>

#include "std_inc.h"
#include <mutex>

///////////////////////////////////////////////////////////////////////////////

#define SETID_MAX_SIZE 30 //!< Size of Phone ID strings

///////////////////////////////////////////////////////////////////////////////

class CRilIf
{
public:
  CRilIf();
  virtual ~CRilIf();
  static const void *getIf(void) { return &s_interface; }
  static CRilIf *getInstance(void);

  AGpsRefLocation getRefLocation(void) const { return m_refLoc; };
  char *getIsmi(void) { return (char *)m_setidImsi; };
  char *getMsisdn(void) { return (char *)m_setidMsisdn; };
  struct in_addr getClientIP(void);
  void setClientIp(const char *pIpAddress);
  void requestCellInfo(void);
  bool isSimPresent(void) const;
  bool isConnected(void);
  bool isAvailable(void);

  std::timed_mutex m_mutexFramework{}; //!< Mutex allowing calls to framework
                                     //!'request' functions to
  //!< to synchronise with framework 'set' callback from framework

  char m_setidMsisdn[SETID_MAX_SIZE]{""}; //!< MSISDN ID info from framework
  char m_setidImsi[SETID_MAX_SIZE]{""};   //!< IMSI ID info from framework

private:
  // Interface
  static void init(AGpsRilCallbacks *callbacks);
  static void setRefLocation(const AGpsRefLocation *agps_reflocation, size_t sz_struct);
  static void setSetId(AGpsSetIDType type, const char *setid);
  static void niMessage(uint8_t *msg, size_t len);
  static void updateNetworkState(int connected, int type, int roaming, const char *extra_info);
#if (PLATFORM_SDK_VERSION >= 14 /* =4.0 */)
  static void updateNetworkAvailability(int available, const char *apn);
#endif
  // Helpers
  void setAvailability(bool avail);
  void setNetworkType(bool avail);
  static in_addr readIpAddress(const char *pDeviceName);
  void requestSetId(uint32_t flags) const;
  void requestRefLoc(uint32_t flags) const;

  // Variables
  static const AGpsRilInterface s_interface; //!< Jump table of functions
                                             //! implementing the RIL interface
  //! to the driver
  AGpsRilCallbacks m_callbacks{}; //!< Jump table of RIL API functions provided by framework
  bool m_ready{false};                   //!< RIL interface initialised flag

  AGpsRefLocation m_refLoc{}; //!< Reference location info from framework
  int m_networkType{};          //!< Connected network type - Told to us by framework
  int m_available{};            //!< Allowed to use network flag
  std::mutex m_mutexNetwork{};
  //!< getting m_available and
  //!< m_networkType thread save
};
