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
/*!
  \file
  \brief  RIL interface implementation

*/

#include "ubx_rilIf.h"
#include <cassert>
#include <cmath>
#include <cstring>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#ifdef RAW_MEAS_ENABLED
#include <ifaddrs.h>
#include <netdb.h>
#include <sys/types.h>
#endif

#include "ubxgpsstate.h"

#ifdef SUPL_ENABLED
#include "suplSMmanager.h"

#endif

///////////////////////////////////////////////////////////////////////////////
// Definitions & Types
static const int RIL_NO_NETWORK = -1;
static const std::chrono::seconds REQUEST_REF_LOC_TIMEOUT{
  2
}; //!< Timeout (in seconds) to wait on request location ref mutex

using namespace ublox::log::stringtables;
///////////////////////////////////////////////////////////////////////////////
// Static data
static CRilIf s_myIf; //!< Private instance of the CRilIf class - 'singleton'

const AGpsRilInterface CRilIf::s_interface = //!< RIL interface jump table
  {
    .size = sizeof(AGpsRilInterface),
    .init = CRilIf::init,
    .set_ref_location = CRilIf::setRefLocation,
    .set_set_id = CRilIf::setSetId,
    .ni_message = CRilIf::niMessage,
    .update_network_state = CRilIf::updateNetworkState,
#if (PLATFORM_SDK_VERSION >= 14 /* =4.0 */)
    .update_network_availability = CRilIf::updateNetworkAvailability,
#endif
  };

///////////////////////////////////////////////////////////////////////////////
//! Constructor for CRilIf class
CRilIf::CRilIf()
{
  m_networkType = RIL_NO_NETWORK;
  //lock the mutex in the beginning, until setRefLocation is called and unlocks it
  m_mutexFramework.lock();
#if (PLATFORM_SDK_VERSION >= 14 /* =4.0 */)
  m_available = 0;
#else
  m_available = 1; // Always true for Android < 4.0
#endif
}

///////////////////////////////////////////////////////////////////////////////
//! Destructor for CRilIf class
CRilIf::~CRilIf() {}

///////////////////////////////////////////////////////////////////////////////
//! Retrieve singleton class instance
/*!
  \return	: Pointer to singleton CRilIf class instance
*/
CRilIf *CRilIf::getInstance() { return &s_myIf; }

///////////////////////////////////////////////////////////////////////////////
//! RIL interface 'init' function implementation
/*! Framework calls this function on initialisation of the gps driver
  \param callbacks	: Pointer to jump table implementing RIM API into
  framework
*/
void CRilIf::init(AGpsRilCallbacks *callbacks)
{
  if (s_myIf.m_ready)
    UBX_LOG(LCAT_ERROR, "already initialized");
  UBX_LOG(LCAT_VERBOSE, "");
  s_myIf.m_callbacks = *callbacks;
  s_myIf.m_ready = true;
#ifdef NETWORK_ALWAYS_CONNECTED
  UBX_LOG(LCAT_WARNING,
          "This driver will always act as if the network is "
          "connected! (NETWORK_ALWAYS_CONNECTED set during "
          "compilation!)");
#endif
#ifdef NETWORK_ALWAYS_AVAILABLE
  UBX_LOG(LCAT_WARNING,
          "This driver will always act as if the network is "
          "available! (NETWORK_ALWAYS_AVAILABLE set during "
          "compilation!)");
#endif
  // lint -e{818} remove Pointer parameter 'callbacks' (line 94) could be
  // declared as pointing to const
}

///////////////////////////////////////////////////////////////////////////////
//! RIL interface 'set_ref_location' implementation
/*! Framework calls this function when it has been instructed to give the gps
    driver reference location information. Usually this when the driver has
  called
    the function 'request_refloc' on the AGPS framework interface.
  \param agps_reflocation	: Pointer to a framework defined agps reference
                              location data structure
  \param sz_struct			: Size of the passed structure
*/
void CRilIf::setRefLocation(const AGpsRefLocation *agps_reflocation, size_t sz_struct)
{
  if (agps_reflocation->type == AGPS_REG_LOCATION_TYPE_MAC)
    UBX_LOG(LCAT_VERBOSE,
            "size=%d type=%d(%s) -> mac=%02X-%02X-%02X-%02X-%02X-%02X",
            sz_struct,
            agps_reflocation->type,
            _LOOKUPSTR(agps_reflocation->type, AGpsRefLocation),
            agps_reflocation->u.mac.mac[0],
            agps_reflocation->u.mac.mac[1],
            agps_reflocation->u.mac.mac[2],
            agps_reflocation->u.mac.mac[3],
            agps_reflocation->u.mac.mac[4],
            agps_reflocation->u.mac.mac[5]);
  else if ((agps_reflocation->type == AGPS_REF_LOCATION_TYPE_GSM_CELLID) ||
           (agps_reflocation->type == AGPS_REF_LOCATION_TYPE_UMTS_CELLID))
    UBX_LOG(LCAT_VERBOSE,
            "size=%d type=%d(%s) -> type=%d mcc=%d mnc=%d lac=%d cid=%d",
            sz_struct,
            agps_reflocation->type,
            _LOOKUPSTR(agps_reflocation->type, AGpsRefLocation),
            agps_reflocation->u.cellID.type, // random, not filled by framework (random)
            agps_reflocation->u.cellID.mcc,
            agps_reflocation->u.cellID.mnc,
            agps_reflocation->u.cellID.lac, // in 3g lac is discarded
            agps_reflocation->u.cellID.cid);
  else
    UBX_LOG(LCAT_VERBOSE,
            "size=%d type=%d(%s)",
            sz_struct,
            agps_reflocation->type,
            _LOOKUPSTR(agps_reflocation->type, AGpsRefLocation));

  s_myIf.m_refLoc = *agps_reflocation; // make a copy

  // unlock the mutex
  s_myIf.m_mutexFramework.unlock();
}

///////////////////////////////////////////////////////////////////////////////
//! RIL interface 'set_set_id' implementation
/*! Framework calls this function when it has been instructed to give the gps
    driver ID information. Usually this when the driver has called
    the function 'request_setid' on the AGPS framework interface.
  \param type	: Identifier type
  \param setid	: Pointer to test string identifier
*/
void CRilIf::setSetId(AGpsSetIDType type, const char *setid)
{
  if (!setid)
    setid = "";
  UBX_LOG(LCAT_VERBOSE, "type=%d(%s) setid='%s'", type, _LOOKUPSTR(type, AGpsSetIDType), setid);
  if (type == AGPS_SETID_TYPE_NONE)
  {
    // Do nothing
  }
  else
  {
    size_t len = strlen(setid);
    if (len > SETID_MAX_SIZE - 1)
    {
      UBX_LOG(LCAT_ERROR, "Supplied setid too big '%s' (%i)", setid, len);
    }
    else if (type == AGPS_SETID_TYPE_IMSI)
    {
      strncpy(s_myIf.m_setidImsi, setid, SETID_MAX_SIZE);
    }
    else if (type == AGPS_SETID_TYPE_MSISDN)
    {
      strncpy(s_myIf.m_setidMsisdn, setid, SETID_MAX_SIZE);
    }
    else
    {
      UBX_LOG(LCAT_ERROR, "Unknown setid type %d '%s'", type, setid);
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
//! RIL interface 'ni_message' implementation
/*! Framework calls this function when nit wants to pass a Network Initiated
    message to the Gps driver. The driver processes this as a Supl initiation.
  \param 	: Pointer to received message buffer
  \param 	: Size of buffer
*/
void CRilIf::niMessage(uint8_t *msg, size_t len)
{
  assert(pthread_self() != g_gpsDrvMainThread); // On an arbitory thread
  UBX_LOG(LCAT_VERBOSE, "msg len %i:", len);
  if (!msg)
  {
    return;
  }
#ifdef SUPL_ENABLED
  // Assumption is that the msg from the network is a SUPL message
  suplHandleNetworkInitiatedAction((char *)msg, (int)len);
#endif
  // lint -e{818} remove Pointer parameter 'msg' (line 187) could be declared as
  // pointing to const
}

///////////////////////////////////////////////////////////////////////////////
//! RIL interface 'updateNetworkState' implementation
/*! The framework calls this function when it wants to report the networking
    state of the platform
  \param connected	: 1 if connected to a network, 0 if not.
  \param type		: Type of network.
  \param roaming	: 1 if on a roaming network, 0 if not.
  \param extra_info	: Pointer to extra info buffer.
*/
void CRilIf::updateNetworkState(int connected, int type, int roaming, const char *extra_info)
{
  UBX_LOG(LCAT_VERBOSE,
          "connected=%d type=%d(%s) roaming=%d extra_info='%s'",
          connected,
          type,
          _LOOKUPSTR((unsigned int)type, AgpsRilNetworkType),
          roaming,
          extra_info);

  s_myIf.setNetworkType(connected ? type : RIL_NO_NETWORK);
}

#if (PLATFORM_SDK_VERSION >= 14 /* =4.0 */)
///////////////////////////////////////////////////////////////////////////////
//! RIL interface 'update_network_state' implementation
/*! Framework call this function when it wants to report additional information
    regarding the networking state of the platform
  \param available	: 0 if network connection can not be used, 1 if it can
  \param apn		: Pointer to string containing Access Point Name
*/
void CRilIf::updateNetworkAvailability(int available, const char *apn)
{
  UBX_LOG(LCAT_VERBOSE, "available=%d apn='%s'", available, apn);
  s_myIf.setAvailability(available);
}
#endif

///////////////////////////////////////////////////////////////////////////////
//! Sets the availability value in a thread-safe way
/*!
  \param avail        : The new availability value
*/
void CRilIf::setAvailability(bool avail)
{
  std::lock_guard<std::mutex> guard(m_mutexNetwork);
  m_available = avail;
}

///////////////////////////////////////////////////////////////////////////////
//! Sets the network type value in a thread-safe way
/*!
  \param type         : New network type
*/
void CRilIf::setNetworkType(bool type)
{
  std::lock_guard<std::mutex> guard(m_mutexNetwork);
  m_networkType = type;
}

///////////////////////////////////////////////////////////////////////////////
//! Requests ID information from the framework
/*! Handles requests to call the framework to request ID information
    Implements 'phone faking' to set pretend ID parameters directly and avoids
    calling the framework which won't yield anything useful.
  \param setid	: Type of ID to request from framework
*/
void CRilIf::requestSetId(uint32_t flags) const
{
  if (!m_ready)
  {
    UBX_LOG(LCAT_ERROR, "class not initialized");
    return;
  }
  UBX_LOG(LCAT_VERBOSE, "flags=%d(%s)", flags, _LOOKUPSTRX(flags, AgpsRilRequestSetId));

#ifdef SUPL_ENABLED
  if (CUbxGpsState::getInstance()->getFakePhone())
  {
    // Phone fake - Sets up ID parameters without calling framework
    UBX_LOG(LCAT_VERBOSE, "Faking phone ID");
    const char *setid = "";
    AGpsSetIDType type = AGPS_SETID_TYPE_NONE;
    if ((flags & AGPS_RIL_REQUEST_SETID_IMSI) == AGPS_RIL_REQUEST_SETID_IMSI)
    {
      type = AGPS_SETID_TYPE_IMSI;
#if 0
			setid = "2280215121973630";
#else
      setid = "460001831429979";
#endif
    }
    else if ((flags & AGPS_RIL_REQUEST_SETID_MSISDN) == AGPS_RIL_REQUEST_SETID_MSISDN)
    {
      type = AGPS_SETID_TYPE_MSISDN;
      setid = "380561234567";
    }
    setSetId(type, setid);
  }
  else
#endif
    m_callbacks.request_setid(flags);
}

///////////////////////////////////////////////////////////////////////////////
//! Request location reference information from the framework
/*! Handles requests to call the framework to request reference location
  information
    Implements 'phone faking' to set pretend ref location  parameters directly
  and avoids
    calling the framework which won't yield anything useful.
  \param flags	: Type of location information to request from the framework
*/
void CRilIf::requestRefLoc(uint32_t flags) const
{
  if (!m_ready)
  {
    UBX_LOG(LCAT_ERROR, "class not initialized");
    return;
  }
  UBX_LOG(LCAT_VERBOSE, "flags=%d(%s)", flags, _LOOKUPSTRX(flags, AgpsRilRequestRefLoc));

#ifdef SUPL_ENABLED
  if (CUbxGpsState::getInstance()->getFakePhone())
  {
    // Phone fake - Sets up reference location parameters without calling
    // framework
    AGpsRefLocation refLoc{};
    if ((flags & AGPS_RIL_REQUEST_REFLOC_CELLID) == AGPS_RIL_REQUEST_REFLOC_CELLID)
    {
#if 0
			refLoc.type = AGPS_REF_LOCATION_TYPE_GSM_CELLID;
			refLoc.u.cellID.type = 0/*random*/;
			refLoc.u.cellID.mcc = 230;
			refLoc.u.cellID.mnc = 120;
			refLoc.u.cellID.lac = 99;
			refLoc.u.cellID.cid = 123;
#elif 0
      // spreadtrum phone / shanghai
      refLoc.type = AGPS_REF_LOCATION_TYPE_UMTS_CELLID;
      refLoc.u.cellID.type = 0 /*random*/;
      refLoc.u.cellID.mcc = 460;
      refLoc.u.cellID.mnc = 0;
      refLoc.u.cellID.lac = 0;
      refLoc.u.cellID.cid = 545;
#else
      // switzerland
      refLoc.type = AGPS_REF_LOCATION_TYPE_GSM_CELLID;
      refLoc.u.cellID.type = 0 /*random*/;
      refLoc.u.cellID.mcc = 228;
      refLoc.u.cellID.mnc = 1;
      refLoc.u.cellID.lac = 1010;
      refLoc.u.cellID.cid = 20777;

// Reigate (2G) - UK
//			refLoc.type = AGPS_REF_LOCATION_TYPE_GSM_CELLID;
//			refLoc.u.cellID.type = 0/*random*/;
//			refLoc.u.cellID.mcc = 234;
//			refLoc.u.cellID.mnc = 30;
//			refLoc.u.cellID.lac = 682;
//			refLoc.u.cellID.cid = 3612;

// // Reigate alternative (2G) - UK
// refLoc.type = AGPS_REF_LOCATION_TYPE_GSM_CELLID;
// refLoc.u.cellID.type = 0/*random*/;
// refLoc.u.cellID.mcc = 234;
// refLoc.u.cellID.mnc = 15;
// refLoc.u.cellID.lac = 142;
// refLoc.u.cellID.cid = 24946;

// Reigate alternative (2G) - UK
//			refLoc.type = AGPS_REF_LOCATION_TYPE_GSM_CELLID;
//			refLoc.u.cellID.type = 0/*random*/;
//			refLoc.u.cellID.mcc = 234;
//			refLoc.u.cellID.mnc = 33;
//			refLoc.u.cellID.lac = 118;
//			refLoc.u.cellID.cid = 18297;

// Reigate (3G) - UK
//			refLoc.type = AGPS_REF_LOCATION_TYPE_UMTS_CELLID;
//			refLoc.u.cellID.type = 0/*random*/;
//			refLoc.u.cellID.mcc = 234;
//			refLoc.u.cellID.mnc = 30;
//			refLoc.u.cellID.lac = 0;
//			refLoc.u.cellID.cid = 9243701;
#endif
    }
    else if ((flags & AGPS_RIL_REQUEST_REFLOC_MAC) == AGPS_RIL_REQUEST_REFLOC_MAC)
    {
      // refLoc.type = AGPS_REG_LOCATION_TYPE_MAC;
      // refLoc.u.mac.mac[0] =
      // refLoc.u.mac.mac[1] =
      // refLoc.u.mac.mac[2] =
      // refLoc.u.mac.mac[3] =
      // refLoc.u.mac.mac[4] =
      // refLoc.u.mac.mac[6] =
    }
    setRefLocation(&refLoc, sizeof(refLoc));
  }
  else
#endif
    m_callbacks.request_refloc(flags);
}

///////////////////////////////////////////////////////////////////////////////
//! Retrieves the IP address of this platform
/*! depending on the type of data network platform is connected to, retrieves
    the IP address of the corresponding networking adaptor
  \return	: IP address of this platform
*/
in_addr CRilIf::getClientIP(void)
{
  struct in_addr addr
  {
  };

  int type{};
  {
    std::lock_guard<std::mutex> guard(m_mutexNetwork);
    type = m_networkType;
  }

  switch (type)
  {
  case RIL_NO_NETWORK:
    addr = readIpAddress("eth0");
    break;

  case AGPS_RIL_NETWORK_TYPE_WIFI:
    addr = readIpAddress("wlan0");
    break;

  default:
    break;
  }

  UBX_LOG(LCAT_VERBOSE, "%s (Type = %i)", inet_ntoa(addr), type);

  return addr;
}

///////////////////////////////////////////////////////////////////////////////
//! Retrieves the IP address assigned to a newtorking device
/*!
  \param pDeviceName	: Pointer to the name of the network device
  \return				: IP address of device
*/
in_addr CRilIf::readIpAddress(const char *pDeviceName)
{
  int fd;
  struct ifreq ifr
  {
  };

  fd = socket(AF_INET, SOCK_DGRAM, 0);

  /* I want to get an IPv4 IP address */
  ifr.ifr_addr.sa_family = AF_INET;

  /* I want IP address for device */
  strncpy(ifr.ifr_name, pDeviceName, IFNAMSIZ - 1);
  ioctl(fd, SIOCGIFADDR, &ifr);
  close(fd);

  return ((struct sockaddr_in *)(void *)&ifr.ifr_addr)->sin_addr;
}

///////////////////////////////////////////////////////////////////////////////
//! Collect cell information
/*! Initiates calls to framework to retrieve all cell information

*/
void CRilIf::requestCellInfo(void)
{
  requestRefLoc(AGPS_RIL_REQUEST_REFLOC_MAC | AGPS_RIL_REQUEST_REFLOC_CELLID);

  // Be sure the reference location is got...

  m_mutexFramework.try_lock_for(
    REQUEST_REF_LOC_TIMEOUT); // Timeout needed because 'request' may not
                              // cause corresponding 'set' if error occurs

  // First clear existing data as platform my have had SIM removed
  strcpy(m_setidImsi, "");
  strcpy(m_setidMsisdn, "");

  // Now request current cell info
  requestSetId(AGPS_RIL_REQUEST_SETID_IMSI);
  requestSetId(AGPS_RIL_REQUEST_SETID_MSISDN);
}

///////////////////////////////////////////////////////////////////////////////
//! Determine state of connection to data network - thread safe
/*!
  \return	: true if connected to some kind of network, false if not
*/
bool CRilIf::isConnected(void)
{
  bool result = false;
  std::lock_guard<std::mutex> guard(m_mutexNetwork);
#if (PLATFORM_SDK_VERSION < 24)
  // Before Android 7.0, the Android framework would
  // inform the GNSS HAL about the network availability
  // (update_network_availability). This is not the case
  // in Android 7 anymore, so availability needs to be
  // assumed if SDK version is above 24
  if (m_available > 0)
#endif // if (NETWORK_ALWAYS_CONNECTED < 24)
  {
    if (m_networkType != RIL_NO_NETWORK)
    {
      result = true;
    }
  }
  return result;
}

///////////////////////////////////////////////////////////////////////////////
//! Determine if the network type might be used - thread safe
/*!
  \return	: true if the network type flag can be used
*/
bool CRilIf::isAvailable(void)
{
  std::lock_guard<std::mutex> guard(m_mutexNetwork);
  bool result = false;
  result = m_available > 0;
  return result;
}

///////////////////////////////////////////////////////////////////////////////
//! determines if a SIMM card is present
/*!
  \return	: true if a SIMM is present, false if not
*/
bool CRilIf::isSimPresent(void) const
{
  return ((strcmp(m_setidImsi, "") != 0) || (strcmp(m_setidMsisdn, "") != 0));
}

///////////////////////////////////////////////////////////////////////////////
