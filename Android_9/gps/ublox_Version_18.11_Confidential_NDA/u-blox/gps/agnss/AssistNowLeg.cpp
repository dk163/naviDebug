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

/*! \file
    \ref CAssistNowLeg (AssistNow Legacy Online/Offline) implementation.

    \brief

    Implementation of the functions defined by \ref CAssistNowLeg, a class
    derived from \ref CAgnss
*/

#include "AssistNowLeg.h"
#include "func/MemberFunc.h"
#include "helper/helperFunctions.h"
#include <arpa/inet.h>
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <list>
#include <mutex>
#include <netdb.h>
#include <netinet/in.h>
#include <new>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>

const struct UBX_MSG_TYPE CAssistNowLeg::_aid_allowed[] = {
  { 0x0B, 0x01 }, // INI
  { 0x0B, 0x02 }, // Health / UTC / Iono
  { 0x0B, 0x30 }, // ALM
  { 0x0B, 0x31 }, // EPH
  { 0x0B, 0x33 }  // AOP
};

CAssistNowLeg *CAssistNowLeg::_singleton = NULL;
std::mutex CAssistNowLeg::_singleton_mutex{};

/*! Will create an object of this class type.

    \param online_primary_server   : The zero-terminated string defines
                                     the host name of the server from which the
                                     AssistNow Legacy Online data will be
                                     downloaded. The contents of this string
                                     are copied, the string must thus only
                                     exist during the call to this function.
                                     Must not be NULL.
    \param offline_primary_server  : The zero-terminated string defines
                                     the host name of the server from which the
                                     AssistNow Legacy Offline data will be
                                     downloaded. The contents of this string
                                     are copied, the string must thus only
                                     exist during the call to this function.
                                     Must not be NULL.
    \param server_token            : Token to authenticate against the server
    \param cb                      : Structure of \ref CAgnss::CONF_t type
                                     containing the configuration for the base
                                     class.
*/
CAssistNowLeg::CAssistNowLeg(const char *online_primary_server,
                             const char *online_secondary_server,
                             const char *offline_primary_server,
                             const char *offline_secondary_server,
                             const char *server_token,
                             CONF_t *cb)
  : CAgnss(cb, "LEG: "), _ubx_aid_eph_alm((void *)this, CAgnss::writeToRcv, 30),
    _ubx_poll_aid_eph_alm((void *)this, CAgnss::writeToRcv, 30),
    _ubx_aid_ini((void *)this, CAgnss::writeToRcv, 30),
    _ubx_cfg_navx5((void *)this, CAgnss::writeToRcv, 30), _action_state(SUCCESS),
    _action_data_id(0), _transfer_buf(NULL), _transfer_size(0), _sent_online_msg(false),
    _online_msg_counter(0), _num_client_answers(-1), _num_aiding_send_tries(0),
    _num_aiding_recv_tries(0), _num_config_ack_send_tries(0), _alpsrv_data_save_req(false),
    _num_msgs_expected(0), _evt_handler_status(CALL_AGAIN), _event_config(NULL), _flow_config(NULL)
{
  // initialise the libMGA
  mgaInit();

  // Configure the online part
  memset(&_online_config, 0, sizeof(_online_config));
  _online_config.strPrimaryServer =
    online_primary_server ? strdup(online_primary_server) : strdup("");
  _online_config.strSecondaryServer =
    online_secondary_server ? strdup(online_secondary_server) : strdup("");
  _online_config.strServerToken = server_token ? strdup(server_token) : strdup("");
  _online_config.dataTypeFlags = MGA_DATA_EPH | MGA_DATA_ALM | MGA_DATA_AUX;
  // none of these data items valid - no position estimation. Timing Coarse
  _online_config.useFlags = MGA_FLAGS_USE_LEGACY_AIDING;
  _online_config.gnssTypeFlags = MGA_GNSS_GPS;

  // Configure the offline part
  memset(&_offline_config, 0, sizeof(_offline_config));
  _offline_config.strPrimaryServer =
    offline_primary_server ? strdup(offline_primary_server) : strdup("");
  _offline_config.strSecondaryServer =
    offline_secondary_server ? strdup(offline_secondary_server) : strdup("");
  _offline_config.strServerToken = server_token ? strdup(server_token) : strdup("");
  _offline_config.useFlags = MGA_FLAGS_USE_LEGACY_AIDING;
  _offline_config.gnssTypeFlags = MGA_GNSS_GPS;
  // The number of days the future the LEG data should be valid for
  _offline_config.numofdays = 14;

  _fun_lookup[TIME][TRANSFER].push_back(FREF(CAssistNowLeg, this, configureAidAck));
  _fun_lookup[TIME][TRANSFER].push_back(FREF(CAssistNowLeg, this, timeTransfer));
  _fun_lookup[TIME][TRANSFER].push_back(FREF(CAssistNowLeg, this, finishTimeTransfer));

  _fun_lookup[POSITION][TRANSFER].push_back(FREF(CAssistNowLeg, this, configureAidAck));
  _fun_lookup[POSITION][TRANSFER].push_back(FREF(CAssistNowLeg, this, posTransfer));

  _fun_lookup[ONLINE][DOWNLOAD].push_back(FREF(CAssistNowLeg, this, onlineDownload));

  _fun_lookup[ONLINE][TRANSFER].push_back(FREF(CAssistNowLeg, this, configureAidAck));
  _fun_lookup[ONLINE][TRANSFER].push_back(FREF(CAssistNowLeg, this, onlineTransfer));

  _fun_lookup[OFFLINE][DOWNLOAD].push_back(FREF(CAssistNowLeg, this, offlineDownload));

  _fun_lookup[OFFLINE][TRANSFER].push_back(FREF(CAssistNowLeg, this, timeTransfer));
  _fun_lookup[OFFLINE][TRANSFER].push_back(FREF(CAssistNowLeg, this, offlineTransfer));

  _fun_lookup[RECV_AID_STATE][POLL_RECV].push_back(FREF(CAssistNowLeg, this, recvStatePoll));

  _fun_lookup[RECV_AID_STATE][TRANSFER].push_back(FREF(CAssistNowLeg, this, configureAidAck));
  _fun_lookup[RECV_AID_STATE][TRANSFER].push_back(FREF(CAssistNowLeg, this, timeTransfer));
  _fun_lookup[RECV_AID_STATE][TRANSFER].push_back(FREF(CAssistNowLeg, this, recvStateTransfer));
}

/* Destructor freeing the ressources allocated and
   calling \ref teardown as required
*/
// Unfortunately lint does not seem to accept CAgnss::teardown
// as a cleanup function
// lint -e{1578}
// lint -e{1579}
CAssistNowLeg::~CAssistNowLeg()
{
  std::lock_guard<std::mutex> lock(_singleton_mutex);
  teardown();
  free(const_cast<char *>(_online_config.strPrimaryServer));
  free(const_cast<char *>(_online_config.strSecondaryServer));
  free(const_cast<char *>(_online_config.strServerToken));
  free(const_cast<char *>(_offline_config.strPrimaryServer));
  free(const_cast<char *>(_offline_config.strSecondaryServer));
  free(const_cast<char *>(_offline_config.strServerToken));
  free(_transfer_buf);
  _transfer_buf = NULL;
  _singleton = NULL;
}

/*! Creates a singleton of this class or returns the address to the singleton
    if it is already existing. For information on the parameters passed,
    please refer to the constructor documentation
    \ref CAssistNowLeg::CAssistNowLeg

    \param online_primary_server   : Passed on to the constructor if required
                                     otherwise ignored.
    \param offline_primary_server  : Passed on to the constructor if required
                                     otherwise ignored.
    \param server_token            : Passed on to the constructor if required
                                     otherwise ignored.
    \param cb                      : Passed on to the constructor if required
                                     otherwise ignored.
    \return                          The address of the singleton if newly
                                     created, NULL otherwise
*/
CAssistNowLeg *CAssistNowLeg::createInstance(const char *online_primary_server,
                                             const char *online_secondary_server,
                                             const char *offline_primary_server,
                                             const char *offline_secondary_server,
                                             const char *server_token,
                                             CONF_t *cb)
{
  CAssistNowLeg *tmpRef = NULL;
  std::lock_guard<std::mutex> lock(_singleton_mutex);
  if (!_singleton)
  {
    _singleton = new (std::nothrow) CAssistNowLeg(online_primary_server,
                                                  online_secondary_server,
                                                  offline_primary_server,
                                                  offline_secondary_server,
                                                  server_token,
                                                  cb);
    tmpRef = _singleton;
  }

  return tmpRef;
}

/*! Returns the address to the singleton - if it is already existing

    \return                          The address of the singleton if
                                     existing, NULL otherwise
*/
CAssistNowLeg *CAssistNowLeg::getInstance()
{
  CAssistNowLeg *tmpRef = NULL;
  std::lock_guard<std::mutex> lock(_singleton_mutex);
  if (_singleton)
  {
    tmpRef = _singleton;
  }

  return tmpRef;
}

/*! Implements impl_init as required by base. Will initiate the libMga.
    Will check if the data initialised in the constructor is correct.

    \return                          true on success, false otherwise
*/
bool CAssistNowLeg::impl_init()
{
  bool result = false;
  if (_online_config.strPrimaryServer && _online_config.strSecondaryServer &&
      _offline_config.strPrimaryServer && _offline_config.strSecondaryServer)
  {
    _event_config = (MgaEventInterface *)malloc(sizeof(MgaEventInterface));
    _flow_config = (MgaFlowConfiguration *)malloc(sizeof(MgaFlowConfiguration));
    if (_event_config && _flow_config)
    {
      _event_config->evtProgress = evtProgHandler; // progress
      _event_config->evtWriteDevice = sWriteToRcv; // write data to receiver
      result = true;
    }
    else
    {
      impl_deinit();
    }
  }
  return result;
}

/*! Implements impl_deinit as required by base. The ressources allocated for
    the libMga will be freed. Will always return true.

    \return                          true
*/
bool CAssistNowLeg::impl_deinit()
{
  free(_event_config);
  free(_flow_config);
  _event_config = NULL;
  _flow_config = NULL;
  return true;
}

/*! Implements impl_initAction as required by base.
*/
std::list<CFuncMngr> CAssistNowLeg::impl_initAction(SERVICE_t service, ACTION_t action)
{
  std::list<CFuncMngr> result;
  if (isValidServiceAction(service, action))
  {
    result = _fun_lookup[service][action];
    // Clear local data
    _ubx_poll_aid_eph_alm.clearData();
    _ubx_aid_eph_alm.clearData();
    _ubx_aid_ini.clearData();
    _ubx_cfg_navx5.clearData();
    _action_data_id = 0;
    _alpsrv_data_save_req = false;
    _num_client_answers = -1;
    _num_config_ack_send_tries = 0;
    _evt_handler_status = CALL_AGAIN;
    free(_transfer_buf);
    _transfer_buf = NULL;
    _transfer_size = 0;

    // A transfer requires data to be stored already and needs valid
    // time information
    if (action == TRANSFER && (!hasData(service) || !hasValidTime()))
    {
      result.clear();
    }
  }
  return result;
}

bool CAssistNowLeg::impl_isValidServiceAction(SERVICE_t service, ACTION_t action)
{
  bool result = false;
  if (service < _NUM_SERVICES_ && action < _NUM_ACTIONS_)
  {
    result = (_fun_lookup[service][action].size() != 0);
  }
  return result;
}

/*! Implements impl_isValidData as required by base.
*/
bool CAssistNowLeg::impl_isValidData(SERVICE_t service, unsigned char const *buf, size_t size)
{
  bool result = false;
  switch (service)
  {
  case RECV_AID_STATE:
  {
    result = (verifyUbxMsgsBlock(
                buf, size, _aid_allowed, sizeof(_aid_allowed) / sizeof(*_aid_allowed), NULL) > 0);
    break;
  }
  case ONLINE:
  {
    result = (verifyUbxMsgsBlock(
                buf, size, _aid_allowed, sizeof(_aid_allowed) / sizeof(*_aid_allowed), NULL) > 0);
    break;
  }
  case OFFLINE:
  {
    result = verifyOfflineData(buf, size);
    break;
  }
  case TIME:
  case _NUM_SERVICES_:
  default:
  {
    break;
  }
  }
  return result;
}

uint16_t CAssistNowLeg::impl_getDataId() { return _action_data_id; }

/*! Initialises and configures the libMga to be used for transfer
    and download operations.

    \return             'true' on success otherwise 'false'
*/
bool CAssistNowLeg::initLeg()
{
  bool result = false;
  mgaSessionStop();
  // Otherwise there has been an error while stopping the service
  if (_evt_handler_status == CALL_AGAIN)
  {
    assert(_flow_config);
    _flow_config->msgTimeOut = 15000; // timeout 15s
    _flow_config->msgRetryCount = 5;
    _flow_config->mgaFlowControl = MGA_FLOW_SIMPLE;
    if (mgaConfigure(_flow_config, _event_config, this) == MGA_API_OK &&
        mgaSessionStart() == MGA_API_OK)
    {
      result = true;
    }
    else
    {
      print_err("An error occured while trying to "
                "configure the libMga");
    }
  }
  return result;
}

TRI_STATE_FUNC_RESULT_t CAssistNowLeg::onlineDownload(unsigned char const *buf, size_t size)
{
  (void)(buf);
  (void)(size);
  TRI_STATE_FUNC_RESULT_t result = FAIL;
  // lint -esym(438,tmpBuf)
  unsigned char *tmpBuf = NULL;
  int tmpSize = 0;
  if (initLeg() && mgaGetOnlineData(&_online_config, &tmpBuf, &tmpSize) == MGA_API_OK && tmpBuf &&
      tmpSize > 0)
  {
    if (extractLegIniTimeUtcMsg(tmpBuf, tmpSize, true))
    {
      if (saveToDb(ONLINE, tmpBuf, tmpSize, &_action_data_id))
      {
        print_std("Successfully downloaded and stored LEG Online data from the server");
        result = SUCCESS;
      }
    }
    else
    {
      print_err("Could not extract and clear the received time information");
    }

    if (result != SUCCESS)
    {
      print_err("Storing the downloaded data for LEG Online failed! Abort!");
    }
    free(tmpBuf);
    tmpBuf = NULL;
  }
  else
  {
    print_err("Downloading aiding data from the MGA Offline server failed");
  }
  return result;
}

/*! This function will will download assistance data to the CAgnss super
    with \ref saveToDb.

    \param buf        : Ignored.
    \param size       : Ignored.
    \return             SUCCESS if the download completed successfully,
                        FAIL if the download failed,
                        CALL_AGAIN if the download is not complete yet and
                        the function has to be called again at least once
                        to complete
*/
TRI_STATE_FUNC_RESULT_t CAssistNowLeg::offlineDownload(unsigned char const *buf, size_t size)
{
  (void)(buf);
  (void)(size);

  TRI_STATE_FUNC_RESULT_t result = FAIL;
  // The initialisation of tmpBuf is a safety measure
  // lint -esym(438,tmpBuf)
  unsigned char *tmpBuf = NULL;
  int tmpSize = 0;
  if (initLeg() && mgaGetOfflineData(&_offline_config, &tmpBuf, &tmpSize) == MGA_API_OK && tmpBuf &&
      tmpSize > 0)
  {
    if (saveToDb(OFFLINE, tmpBuf, tmpSize, &_action_data_id))
    {
      print_std("Successfully downloaded and stored LEG Offline data from the server");
      result = SUCCESS;
    }
    else
    {
      print_err("Soring the downloaded data for LEG Offline failed! Abort!");
    }
  }
  else
  {
    print_err("Downloading aiding data from the LEG Offline server failed");
  }

  free(tmpBuf);
  return result;
}

bool CAssistNowLeg::insertTransferTimeMsg(unsigned char *msg)
{
  ACCTIME_t accTimeNow;
  GPS_UBX_AID_INI_U5__t Payload;
  if (!msg || !getCurrentTime(&accTimeNow) || !createUbxAidIni(&Payload, &accTimeNow))
  {
    return false;
  }
  bool result = false;

  char nowS[20];
  char accS[20];
  struct tm tmpTmp;
  strftime(nowS, 20, "%Y.%m.%d %H:%M:%S", gmtime_r(&accTimeNow.time.tv_sec, &tmpTmp));
  strftime(accS, 20, "%H:%M:%S", gmtime_r(&accTimeNow.acc.tv_sec, &tmpTmp));

  unsigned char *ubxmsg = NULL;
  ssize_t ubxsize = createUbx(&ubxmsg, 0x0B, 0x01, &Payload, sizeof(Payload));
  if (ubxsize > 0)
  {
    if (ubxsize == 48 + 8)
    {
      memcpy(msg, ubxmsg, ubxsize);
      print_std("Using the following UTC time for aiding the receiver:"
                " %s.%.9d (GPS: %uwn:%ums:%uns) Accuracy: %s.%.9d",
                nowS,
                accTimeNow.time.tv_nsec,
                Payload.wn,
                Payload.tow,
                Payload.towNs,
                accS,
                accTimeNow.acc.tv_nsec);
      result = true;
    }
    free(ubxmsg);
  }
  return result;
}

//! Initiates the transfer to the receiver for the specified service
TRI_STATE_FUNC_RESULT_t CAssistNowLeg::timeTransfer(unsigned char const *buf, size_t size)
{
  return timePosTransfer(buf, size, false);
}

TRI_STATE_FUNC_RESULT_t CAssistNowLeg::posTransfer(unsigned char const *buf, size_t size)
{
  return timePosTransfer(buf, size, true);
}

TRI_STATE_FUNC_RESULT_t
CAssistNowLeg::timePosTransfer(unsigned char const *buf, size_t size, bool forcePosition)
{
  TRI_STATE_FUNC_RESULT_t result = FAIL;

  // Send data if none is stored in this class yet
  if (!_ubx_aid_ini.isReadyToSend())
  {
    ACCTIME_t accTimeNow;
    POS_t pos;
    GPS_UBX_AID_INI_U5__t Payload;

    memset(&accTimeNow, 0, sizeof(accTimeNow));
    memset(&pos, 0, sizeof(pos));
    memset(&Payload, 0, sizeof(Payload));
    bool validPos = getCurrentPosition(&pos);

    // It is required to have information about
    // the current. Having valid Position information
    // is optional as long as forcePosition is false
    if (getCurrentTime(&accTimeNow) && (validPos || !forcePosition) &&
        createUbxAidIni(&Payload, &accTimeNow))
    {
      char nowS[20];
      char accS[20];
      struct tm tmpTmp;
      strftime(nowS, 20, "%Y.%m.%d %H:%M:%S", gmtime_r(&accTimeNow.time.tv_sec, &tmpTmp));
      strftime(accS, 20, "%H:%M:%S", gmtime_r(&accTimeNow.acc.tv_sec, &tmpTmp));
      if (_ubx_aid_ini.setPosTime(&accTimeNow, validPos ? &pos : NULL) && _ubx_aid_ini.sendData())
      {
        print_std("Sent the following UTC time for aiding the receiver:"
                  " %s.%09u (GPS: %uwn:%09ums:%09lins) Accuracy: %s.%.9u",
                  nowS,
                  accTimeNow.time.tv_nsec,
                  Payload.wn,
                  Payload.tow,
                  Payload.towNs,
                  accS,
                  accTimeNow.acc.tv_nsec);

        if (validPos)
        {
          print_std("Sent the following position for aiding the receiver:"
                    " latitude: %f longitude: %f Accuracy: %ucm",
                    ((double)pos.latDeg) * 1e-7,
                    ((double)pos.lonDeg) * 1e-7,
                    pos.posAccCm);
        }
        result = CALL_AGAIN;
      }
    }
  }
  else
  {
    result = _ubx_aid_ini.onNewMsg(buf, size);
    if (result == SUCCESS)
    {
      print_std("Successfully aided the receiver with the current time!");
      _action_data_id = 2;
    }
  }
  return result;
}

//! Sets the _action_data_id to a valid value
TRI_STATE_FUNC_RESULT_t CAssistNowLeg::finishTimeTransfer(unsigned char const *buf, size_t size)
{
  ((void)(buf));
  ((void)(size));
  _action_data_id = 1;
  return SUCCESS; // This is always successful
}

/*! This function extracts the time that can be found in the first message
    of the passed message block and set the current time accordingly.

    \param buf        : Block of ubx messages of which the first one must
                        be a UBX-AID-INI message.
                        Must not be NULL.
    \param size       : The size of buf. Must not be NULL.
    \param clearMsg   : true if the message should be cleared of data
                        and false otherwise

    \return             true on success, false otherwise
*/
bool CAssistNowLeg::extractLegIniTimeUtcMsg(unsigned char *buf, size_t size, bool clearMsg)
{
  const size_t UBX_AID_INI_SIZE = sizeof(GPS_UBX_AID_INI_U5__t) + 8; // Size of data + UBX frame
  if (!buf || size < UBX_AID_INI_SIZE)
    return false;

  bool result = false;
  BUFC_t *tmpMsg = NULL;
  UBX_MSG_TYPE allowed;
  allowed.clsId = 0x0B;
  allowed.msgId = 0x01;
  if (verifyUbxMsgsBlock(buf, UBX_AID_INI_SIZE, &allowed, 1, &tmpMsg) > 0)
  {
    // Inject the server time to the time handler
    unsigned char *iniPay = buf + 6;
    ACCTIME_t accTimeNow;
    GPS_UBX_AID_INI_U5__t tmpPay;
    memcpy(&tmpPay, iniPay, sizeof(tmpPay));
    if (extractUbxAidIni(&tmpPay, &accTimeNow))
    {
      setCurrentTime(&accTimeNow);
      if (clearMsg)
      {
        // Set the time fields to 0 in the INI message
        memset(iniPay + 18, 0, 18);

        // Calculate the new checksum, without the sync chars and
        // the current checksum
        uint16_t chk = calculateChecksum(tmpMsg[0].p + 2, tmpMsg[0].i - 4);
        buf[tmpMsg[0].i - 2] = (unsigned char)(chk >> 8); // Set part A
        buf[tmpMsg[0].i - 1] = (unsigned char)chk;        // Set part B
      }
      result = true;
    }
    free(tmpMsg);
  }
  return result;
}

TRI_STATE_FUNC_RESULT_t CAssistNowLeg::recvStateTransfer(unsigned char const *buf, size_t size)
{
  TRI_STATE_FUNC_RESULT_t result = FAIL;
  if (!_ubx_aid_eph_alm.hasSentData())
  {
    unsigned char *tmpBuf = NULL;
    ssize_t tmpSize = loadFromDb(RECV_AID_STATE, &tmpBuf, &_action_data_id);
    if (tmpSize >= 0)
    {
      if (_ubx_aid_eph_alm.setData(tmpBuf, tmpSize) && _ubx_aid_eph_alm.sendData())
      {
        result = CALL_AGAIN;
      }
      free(tmpBuf);
    }
  }
  else
  {
    result = _ubx_aid_eph_alm.onNewMsg(buf, size);
  }
  return result;
}

/*! This helper function will initiate the polling data from the receiver
    for the specified service

    \param buf        : A message that will be parsed for answers
                        from the receiver (May be NULL)
    \param size       : The size of buf (must be 0 if buf is NULL)
    \return             SUCCESS on success,
                        FAIL of failure
                        CALL_AGAIN if additional calls to this
                        function are required
*/
TRI_STATE_FUNC_RESULT_t CAssistNowLeg::recvStatePoll(unsigned char const *buf, size_t size)
{
  TRI_STATE_FUNC_RESULT_t result = FAIL;
  if (!_ubx_poll_aid_eph_alm.hasSentData())
  {
    print_std("Sending the Poll request for %s", agnssServiceTypeToString(RECV_AID_STATE));
    if (_ubx_poll_aid_eph_alm.sendData())
    {
      print_std("Sent the Poll request for %s", agnssServiceTypeToString(RECV_AID_STATE));
      result = CALL_AGAIN;
    }
    else
    {
      print_err("Failed to send the Poll request for %s", agnssServiceTypeToString(RECV_AID_STATE));
      result = FAIL;
    }
  }
  else
  {
    result = _ubx_poll_aid_eph_alm.onNewMsg(buf, size);
    switch (result)
    {
    case SUCCESS:
    {
      print_std("Received all required data for the poll request for %s",
                agnssServiceTypeToString(RECV_AID_STATE));
      result = FAIL;
      unsigned char *tmpBuf = NULL;
      ssize_t tmpSize = _ubx_poll_aid_eph_alm.getData(&tmpBuf);
      if (tmpSize > 0)
      {
        if (saveToDb(RECV_AID_STATE, tmpBuf, tmpSize, &_action_data_id))
        {
          print_std("Successfully save the data extracted from the poll request for %s",
                    agnssServiceTypeToString(RECV_AID_STATE));
          result = SUCCESS;
        }
        else
        {
          print_err("Could not save the extracted data from the poll request for %s",
                    agnssServiceTypeToString(RECV_AID_STATE));
        }
        free(tmpBuf);
      }
      else
      {
        print_err("Failed to extract the received data from the poll request for %s",
                  agnssServiceTypeToString(RECV_AID_STATE));
      }
      break;
    }
    case FAIL:
    {
      print_err("Failed to receive all required data for the poll request for %s",
                (agnssServiceTypeToString(RECV_AID_STATE)));
      break;
    }
    case CALL_AGAIN:
    default:
    {
      break;
    }
    }
  }
  return result;
}

/*! This function will configure the receiver to send out acknowledgments
    for received legacy aiding data.

    \param buf        : A pointer to a message from the receiver which
                        is checked for containing the acknowledgment
                        of the receiver that acknowledgments have been
                        enabled for for aiding messages. May be NULL.
    \param size       : The size of the data in bytes pointed to by buf
                        Must be 0 if buf is NULL.
    \return             SUCCESS if the configuration completed successfully,
                        FAIL if the configuration failed,
                        CALL_AGAIN if the configuration is not complete yet
                        and the function has to be called again at least once
                        to complete
*/
TRI_STATE_FUNC_RESULT_t CAssistNowLeg::configureAidAck(unsigned char const *buf, size_t size)
{
  TRI_STATE_FUNC_RESULT_t result = FAIL;
  bool sendDataIfRequired = true;
  // If message sent out, check if there has been an answer
  if (_ubx_cfg_navx5.isReadyToSend())
  {
    result = _ubx_cfg_navx5.onNewMsg(buf, size);
    if (result == SUCCESS)
    {
      print_std("Acknowledgment for aiding messages enabled (Try %u/%u)",
                _num_config_ack_send_tries,
                LEG_MAX_CONFIG_ACK_SEND_TRIES);
      _num_config_ack_send_tries = 0;
      sendDataIfRequired = false;
      result = SUCCESS;
    }
    else if (result == FAIL)
    {
      if (_num_config_ack_send_tries >= LEG_MAX_CONFIG_ACK_SEND_TRIES)
      {
        print_err("Did not receive acknowledgement from receiver that enabling "
                  "acknowledgment for aiding messages worked in %u tries. "
                  "Abort.",
                  LEG_MAX_CONFIG_ACK_SEND_TRIES);
        sendDataIfRequired = false;
        _num_config_ack_send_tries = 0;
      }
      else
      {
        print_std("received no acknowledgment for enabling acknowledgments for "
                  "aiding in time (Try %u/%u)",
                  _num_config_ack_send_tries,
                  LEG_MAX_CONFIG_ACK_SEND_TRIES);
      }
    }
  }

  // Check if it is required to send the messsage
  if (!_ubx_cfg_navx5.isReadyToSend() && sendDataIfRequired)
  {
    print_std("Configure receiver to acknowledge aiding (Try %u/%u)",
              _num_config_ack_send_tries + 1,
              LEG_MAX_CONFIG_ACK_SEND_TRIES);
    if (_ubx_cfg_navx5.enableAidingAck(true) && _ubx_cfg_navx5.sendData())
    {
      ++_num_config_ack_send_tries;
      result = CALL_AGAIN;
    }
    else
    {
      print_err("Could not enable acknowledgement for aiding!");
      _num_config_ack_send_tries = 0;
      result = FAIL;
    }
  }

  return result;
}

/*! This static function will be called by libMga to write data to
    the receiver. This function will then actually call the function
    belonging to the object to handle this.

    \param context    : Must contain the 'this' pointer to the actual
                        actual object on which the function of the same name
                        without this parameter will be executed.
    \param buf        : Will be passed to the actual function of the object
    \param size       : Will be passed to the actual function of the object
*/
void CAssistNowLeg::sWriteToRcv(const void *context, unsigned char const *buf, int size)
{
  if (context)
  {
    ((CAssistNowLeg *)const_cast<void *>(context))->writeToRcv(buf, size);
  }
}

/*! This static function will be called by libMga to pass information
    on the current status of to the caller. This function will then
    actually call the function belonging to the object to handle
    these events

    \param pContext   : Must contain the 'this' pointer to the actual
                        actual object on which the function of the same name
                        without this parameter will be executed.
    \param evtType    : Will be passed to the actual function of the object
    \param pEvtInfo   : Will be passed to the actual function of the object
    \param evtInfoSize: Will be passed to the actual function of the object
*/
void CAssistNowLeg::evtProgHandler(MGA_PROGRESS_EVENT_TYPE evtType,
                                   const void *pContext,
                                   const void *pEvtInfo,
                                   UBX_I4 evtInfoSize)
{
  if (pContext)
  {
    ((CAssistNowLeg *)const_cast<void *>(pContext))->evtProgHandler(evtType, pEvtInfo, evtInfoSize);
  }
}

/*! This function will output information on the current status of
    the libMGA and will adjust the transfer status of the object
    depending on that information.

    \param evtType    : The event type triggered by the libMGA
    \param pEvtInfo   : Further information on the event which occured
    \param evtInfoSize: Size of the pEvtInfo object (not used)
*/
void CAssistNowLeg::evtProgHandler(MGA_PROGRESS_EVENT_TYPE evtType,
                                   const void *pEvtInfo,
                                   UBX_I4 evtInfoSize)
{
  ((void)evtInfoSize);
  const int TMP_STR_LEN = 1024;
  char msgTxt[TMP_STR_LEN] = "";

  switch (evtType)
  {
  case MGA_PROGRESS_EVT_START:
  {
    _num_msgs_expected = *((UBX_U4 const *)pEvtInfo);
    print_std("Starting transfer. Expecting %i messages", _num_msgs_expected);
    break;
  }

  case MGA_PROGRESS_EVT_FINISH:
  {
    // pEvtInfo is NULL
    print_std("Successfully finished transfer");
    _evt_handler_status = SUCCESS;
    break;
  }

  case MGA_PROGRESS_EVT_MSG_SENT:
  {
    MgaMsgInfo const *pMsgInfo = (MgaMsgInfo const *)pEvtInfo;

    bool needSvData = false;
    switch (pMsgInfo->pMsg[3])
    {
    case 0x01: // INI
    {
      snprintf(msgTxt, TMP_STR_LEN, "INI");
      break;
    }

    case 0x02: // HUI
    {
      snprintf(msgTxt, TMP_STR_LEN, "HUI");
      break;
    }

    case 0x30: // GPS ALM
    {
      snprintf(msgTxt, TMP_STR_LEN, "ALM");
      needSvData = true;
      break;
    }
    case 0x31: // GPS EPH
    {
      snprintf(msgTxt, TMP_STR_LEN, "EPH");
      needSvData = true;
      break;
    }

    default:
    {
      snprintf(msgTxt, TMP_STR_LEN, "Unknown Message[0x%02x]", pMsgInfo->pMsg[3]);
    }
    }

    if (needSvData) // Add SV Data to the output
    {
      unsigned int sv = pMsgInfo->pMsg[6] | pMsgInfo->pMsg[7] << 8 | pMsgInfo->pMsg[8] << 16 |
                        pMsgInfo->pMsg[9] << 24;
      snprintf(msgTxt + strlen(msgTxt), TMP_STR_LEN - strlen(msgTxt), " for SV %i", sv);
    }

    print_std("Message sent [%i/%i]: %s (size [%i] retries [%i])",
              pMsgInfo->sequenceNumber + 1,
              _num_msgs_expected,
              msgTxt,
              pMsgInfo->msgSize,
              pMsgInfo->retryCount);
    break;
  }

  case MGA_PROGRESS_EVT_TERMINATED:
  {
    EVT_TERMINATION_REASON const *stopReason = (EVT_TERMINATION_REASON const *)pEvtInfo;

    // TERMINATE_HOST_CANCEL is user termination and
    // does not have to be logged or rated as failure
    if (*stopReason != TERMINATE_HOST_CANCEL)
    {
      switch (*stopReason)
      {
      case TERMINATE_RECEIVER_NAK:
      {
        snprintf(msgTxt + strlen(msgTxt),
                 TMP_STR_LEN - strlen(msgTxt),
                 "NAK from receiver. Data received, but receiver had a problem "
                 "processing it. Consider all data invalidated.");
        break;
      }

      case TERMINATE_RECEIVER_NOT_RESPONDING:
      {
        snprintf(msgTxt + strlen(msgTxt),
                 TMP_STR_LEN - strlen(msgTxt),
                 "Too many timeouts. Receiver not responding.");
        break;
      }

      case TERMINATE_PROTOCOL_ERROR:
      {
        snprintf(msgTxt + strlen(msgTxt),
                 TMP_STR_LEN - strlen(msgTxt),
                 "Protocol error. Received an ACK for unexpected data block.");
        break;
      }

      default:
      {
        snprintf(msgTxt + strlen(msgTxt), TMP_STR_LEN - strlen(msgTxt), "Unknown stop reason!");
        break;
      }
      }
      _evt_handler_status = FAIL;
      print_err("Terminated - %s[%i]", msgTxt, *stopReason);
    }
    break;
  }

  case MGA_PROGRESS_EVT_MSG_TRANSFER_FAILED:
  {
    MgaMsgInfo const *pMsgInfo = (MgaMsgInfo const *)pEvtInfo;
    snprintf(msgTxt,
             TMP_STR_LEN,
             "Message [%i], transfer failed. Reason [%i] - ",
             pMsgInfo->sequenceNumber + 1,
             pMsgInfo->mgaFailedReason);

    switch (pMsgInfo->mgaFailedReason)
    {
    case MGA_FAILED_NO_TIME:
      // No UBX-AID-INI not received prior to AID data messages
      snprintf(msgTxt + strlen(msgTxt),
               TMP_STR_LEN - strlen(msgTxt),
               "No UBX-AID-INI message - Terminating...");
      break;

    case MGA_FAILED_VERSION_NOT_SUPPORTED:
      // Don't understand this message version.
      snprintf(
        msgTxt + strlen(msgTxt), TMP_STR_LEN - strlen(msgTxt), "Message version not supported");
      break;

    case MGA_FAILED_SIZE_VERSION_MISMATCH:
      // Message size does not match message version
      snprintf(msgTxt + strlen(msgTxt),
               TMP_STR_LEN - strlen(msgTxt),
               "Message size mismatch with message version");
      break;

    case MGA_FAILED_COULD_NOT_STORE:
      // Message could not be stored in the receiver's database
      snprintf(msgTxt + strlen(msgTxt),
               TMP_STR_LEN - strlen(msgTxt),
               "Message could not be stored. The receiver does not require "
               "this information");
      break;

    case MGA_FAILED_RECEIVER_NOT_READY:
      // Receiver is not ready to use the message
      snprintf(msgTxt + strlen(msgTxt), TMP_STR_LEN - strlen(msgTxt), "Receiver not ready");
      break;

    case MGA_FAILED_MESSAGE_UNKNOWN:
      // Message type unknown
      snprintf(
        msgTxt + strlen(msgTxt), TMP_STR_LEN - strlen(msgTxt), "Message type not recognised");
      break;

    case MGA_FAILED_REASON_TOO_MANY_RETRIES:
      // Terminate. If one message could not be sent in several tries,
      // the chance is very high, that none will be able to be sent
      _evt_handler_status = FAIL;
      snprintf(msgTxt + strlen(msgTxt), TMP_STR_LEN - strlen(msgTxt), "Too many retries");
      break;

    default:
      // Unknown error
      snprintf(msgTxt + strlen(msgTxt), TMP_STR_LEN - strlen(msgTxt), "Unknown error code");
      break;
    }

    print_err("%s", msgTxt);
    break;
  }

  case MGA_PROGRESS_EVT_SERVER_CONNECTING:
  {
    print_std("Connecting to server %s", (char const *)pEvtInfo);
    break;
  }

  case MGA_PROGRESS_EVT_MSG_TRANSFER_COMPLETE:
  {
    MgaMsgInfo const *pMsgInfo = (MgaMsgInfo const *)pEvtInfo;
    print_std("Transfer of Message [%i] completed successfully.", pMsgInfo->sequenceNumber + 1);
    break;
  }

  case MGA_PROGRESS_EVT_UNKNOWN_SERVER:
  {
    print_err("Unknown server \"%s\"!", (char const *)pEvtInfo);
    break;
  }

  case MGA_PROGRESS_EVT_SERVER_CONNECT:
  {
    print_std("Connected to server %s", (char const *)pEvtInfo);
    break;
  }

  case MGA_PROGRESS_EVT_SERVER_CANNOT_CONNECT:
  {
    print_err("Could not connect to server %s!", (char const *)pEvtInfo);
    break;
  }

  case MGA_PROGRESS_EVT_REQUEST_HEADER:
  {
    print_std("Requesting LEG data from service (Request header)");
    break;
  }

  case MGA_PROGRESS_EVT_RETRIEVE_DATA:
  {
    // pEvtInfo is NULL
    print_std("Retrieving LEG data from service");
    break;
  }

  case MGA_PROGRESS_EVT_SERVICE_ERROR:
  {
    EvtInfoServiceError const *pMsgInfo = (EvtInfoServiceError const *)pEvtInfo;

    switch (pMsgInfo->errorType)
    {
    case MGA_SERVICE_ERROR_NOT_HTTP_HEADER:
    {
      snprintf(msgTxt + strlen(msgTxt),
               TMP_STR_LEN - strlen(msgTxt),
               "No HTTP header received from service");
      break;
    }

    case MGA_SERVICE_ERROR_NO_RESPONSE_CODE:
    {
      snprintf(msgTxt + strlen(msgTxt),
               TMP_STR_LEN - strlen(msgTxt),
               "No response code in received HTTP header");
      break;
    }

    case MGA_SERVICE_ERROR_BAD_STATUS:
    {
      snprintf(msgTxt + strlen(msgTxt),
               TMP_STR_LEN - strlen(msgTxt),
               "Bad status: '%s'",
               pMsgInfo->errorMessage);
      break;
    }

    case MGA_SERVICE_ERROR_NO_LENGTH:
    {
      snprintf(msgTxt + strlen(msgTxt),
               TMP_STR_LEN - strlen(msgTxt),
               "No length field in received http header");
      break;
    }

    case MGA_SERVICE_ERROR_ZERO_LENGTH:
    {
      snprintf(msgTxt + strlen(msgTxt),
               TMP_STR_LEN - strlen(msgTxt),
               "Length field is zero in received http header");
      break;
    }

    case MGA_SERVICE_ERROR_NO_CONTENT_TYPE:
    {
      snprintf(msgTxt + strlen(msgTxt),
               TMP_STR_LEN - strlen(msgTxt),
               "No content in received http header");
      break;
    }

    case MGA_SERVICE_ERROR_NOT_UBX_CONTENT:
    {
      snprintf(msgTxt + strlen(msgTxt),
               TMP_STR_LEN - strlen(msgTxt),
               "No ubx content in received http header");
      break;
    }

    case MGA_SERVICE_ERROR_PARTIAL_CONTENT:
    {
      snprintf(msgTxt + strlen(msgTxt),
               TMP_STR_LEN - strlen(msgTxt),
               "Could not download all LEG contents");
      break;
    }

    default:
    {
      snprintf(msgTxt + strlen(msgTxt), TMP_STR_LEN - strlen(msgTxt), "Unknown service error");
      break;
    }
    }

    print_err("Service Error: %s[%i]", msgTxt, pMsgInfo->errorType);
    break;
  }

  case MGA_PROGRESS_EVT_LEGACY_AIDING_STARTUP:
  {
    print_std("Initializing Offline aiding data transfer.");
    break;
  }

  case MGA_PROGRESS_EVT_LEGACY_AIDING_STARTUP_FAILED:
  {
    MgaMsgInfo const *pMsgInfo = (MgaMsgInfo const *)pEvtInfo;
    snprintf(msgTxt + strlen(msgTxt),
             TMP_STR_LEN - strlen(msgTxt),
             "Initialization of Offline data transfer failed [%i] - ",
             pMsgInfo->mgaFailedReason);

    switch (pMsgInfo->mgaFailedReason)
    {
    case MGA_FAILED_REASON_LEGACY_NO_ACK:
      snprintf(msgTxt + strlen(msgTxt), TMP_STR_LEN - strlen(msgTxt), "No Ack");
      break;

    default:
      snprintf(msgTxt + strlen(msgTxt), TMP_STR_LEN - strlen(msgTxt), "Unknown error");
      break;
    }
    _evt_handler_status = FAIL;
    print_err("%s[%i]", msgTxt, pMsgInfo->mgaFailedReason);
    break;
  }

  case MGA_PROGRESS_EVT_LEGACY_AIDING_STARTUP_COMPLETED:
  {
    print_std("Completed the initialization of Offline data transfer");
    break;
  }

  case MGA_PROGRESS_EVT_LEGACY_AIDING_FINALIZE_START:
  {
    print_std("Finalizing Offline data transfer");
    break;
  }

  case MGA_PROGRESS_EVT_LEGACY_AIDING_FINALIZE_FAILED:
  {
    print_err("Finalization of Offline data transfer failed (No Ack)");
    _evt_handler_status = FAIL;
    break;
  }

  case MGA_PROGRESS_EVT_LEGACY_AIDING_FINALIZE_COMPLETED:
  {
    print_err("Finalizing of Offline data transfer done");
    break;
  }

  case MGA_PROGRESS_EVT_LEGACY_AIDING_FLASH_BLOCK_COMPLETE:
  {
    MgaMsgInfo const *pMsgInfo = (MgaMsgInfo const *)pEvtInfo;
    print_std("Offline transfer data block received [%i]", pMsgInfo->sequenceNumber + 1);
    break;
  }

  case MGA_PROGRESS_EVT_LEGACY_AIDING_SERVER_STARTED:
  {
    print_std("Offline server started");
    break;
  }

  case MGA_PROGRESS_EVT_LEGACY_AIDING_SERVER_STOPPED:
  {
    print_std("Offline server stopped");
    break;
  }

  case MGA_PROGRESS_EVT_LEGACY_AIDING_SERVER_REQUEST_RECEIVED:
  {
    print_std("Offline transfer request received from receiver");
    _num_client_answers = -1;
    break;
  }

  case MGA_PROGRESS_EVT_LEGACY_AIDING_SERVER_REQUEST_COMPLETED:
  {
    GPS_UBX_AID_ALPSRV_SRV_t srv;
    memset(&srv, 0, sizeof(srv));
    memcpy(&srv, pEvtInfo, sizeof(srv) < (size_t)evtInfoSize ? sizeof(srv) : (size_t)evtInfoSize);
    print_std("Sent Alp Answer idsize %d type %d ofs %d size %d fileId %d "
              "datasize %d as requested by client",
              srv.idSize,
              srv.type,
              srv.ofs,
              srv.size,
              srv.fileId,
              srv.dataSize);
    break;
  }

  case MGA_PROGRESS_EVT_LEGACY_AIDING_SERVER_UPDATE_RECEIVED:
  {
    GPS_UBX_AID_ALPSRV_CLI_t cli;
    memset(&cli, 0, sizeof(cli));
    memcpy(&cli, pEvtInfo, sizeof(cli) < (size_t)evtInfoSize ? sizeof(cli) : (size_t)evtInfoSize);
    const unsigned int size_in_bytes = cli.size * 2;
    const unsigned int offset_in_bytes = cli.ofs * 2;
    print_std("Received request from receiver to update Alp fileId %d offset "
              "%d size %d",
              cli.fileId,
              offset_in_bytes,
              size_in_bytes);
    break;
  }

  case MGA_PROGRESS_EVT_LEGACY_AIDING_SERVER_UPDATE_COMPLETED:
  {
    print_std("Updated Alp file as requested by receiver.");
    // Indicate that the host based data has been updated
    _alpsrv_data_save_req = true;
    break;
  }

  case MGA_PROGRESS_EVT_LEGACY_AIDING_REQUEST_FAILED_NO_MEMORY:
  {
    print_err("Could not send the expected request to the receiver due to lack "
              "of memory!",
              msgTxt);
    _evt_handler_status = FAIL;
    break;
  }

  case MGA_PROGRESS_EVT_LEGACY_AIDING_REQUEST_FAILED_ID_MISMATCH:
  {
    print_err("The provided data ID does not match the one expected!", msgTxt);
    break;
  }

  default:
  {
    print_err("Unexpected event type! [%i]!", evtType);
    break;
  }
  }
}

/*! This function will try to transfer the online data
    stored in CAgnss to the receiver. To be able to call this function
    either \ref onlineDownload or \ref saveToDb for this service must
    have been successfully called beforehand.

    \param buf        : A message that will be parsed for acknowledgments
                        from the receiver (May be NULL)
    \param size       : The size of buf (must be 0 if buf is NULL)
    \return             SUCCESS if the transfer completed successfully,
                        FAIL if the transfer failed,
                        CALL_AGAIN if the transfer is not complete yet and
                        the function has to be called again at least once
                        to complete
*/
TRI_STATE_FUNC_RESULT_t CAssistNowLeg::onlineTransfer(unsigned char const *buf, size_t size)
{
  TRI_STATE_FUNC_RESULT_t result = FAIL;
  if (!_transfer_buf)
  {
    if (initLegTransfer(true))
    {
      result = CALL_AGAIN;
    }
  }
  else
  {
    result = continueLegTransfer(buf, size);
  }

  return result;
}

/*! This helper function will initiate the transfer to the receiver
    by either determining the current system time and initialising the
    libMGA with it for online or offline data and, in case of an offline
    data transfer, select the data from the current day out of the stored
    messages for transfer. For location and time transfer this will enable
    the acknowledgment of MGA aiding messages. This is a helper function
    called from a other function which manages the state of the current
    transfer.

    \param online     : If true online data will be transferred,
                        If false offline data will be transferred
    \return             true on success, false otherwise
*/
bool CAssistNowLeg::initLegTransfer(bool online)
{
  ACCTIME_t accTimeNow;
  bool result = false;

  SERVICE_t service = OFFLINE;
  if (online)
  {
    service = ONLINE;
  }

  _transfer_size = loadFromDb(service, &_transfer_buf, &_action_data_id);

  if (initLeg())
  {
    MGA_API_RESULT res = MGA_API_OK;
    if (service == ONLINE)
    {
      if (!getCurrentTime(&accTimeNow))
      {
        print_err("Could not load urrent time. Transfer aborted!");
        res = MGA_API_NO_MGA_INI_TIME;
      }
      else
      {
        char nowS[20];
        char accS[20];
        struct tm nowTm;
        struct tm accTm;
        strftime(nowS, 20, "%Y.%m.%d %H:%M:%S", gmtime_r(&accTimeNow.time.tv_sec, &nowTm));
        strftime(accS, 20, "%H:%M:%S", gmtime_r(&accTimeNow.acc.tv_sec, &accTm));

        print_std("Using the follwing UTC time for aiding the receiver:"
                  " %s.%.9d Accuracy: %s.%.9d",
                  nowS,
                  accTimeNow.time.tv_nsec,
                  accS,
                  accTimeNow.acc.tv_nsec);
        MgaTimeAdjust curr_time;
        curr_time.mgaAdjustType = MGA_TIME_ADJUST_ABSOLUTE;
        curr_time.mgaYear = (UBX_U2)nowTm.tm_year + 1900;
        curr_time.mgaMonth = (UBX_U1)nowTm.tm_mon + 1;
        curr_time.mgaDay = (UBX_U1)nowTm.tm_mday;
        curr_time.mgaHour = (UBX_U1)nowTm.tm_hour;
        curr_time.mgaMinute = (UBX_U1)nowTm.tm_min;
        curr_time.mgaSecond = (UBX_U1)nowTm.tm_sec;
        curr_time.mgaAccuracyS = (UBX_U2)accTimeNow.acc.tv_sec;
        curr_time.mgaAccuracyMs = (UBX_U2)accTimeNow.acc.tv_nsec / NSEC_PER_MSEC;
        res = mgaSessionSendOnlineData(_transfer_buf, _transfer_size, &curr_time);
      }
    }
    else
    {
      _num_client_answers = -1;
      res = mgaStartLegacyAiding(_transfer_buf, _transfer_size);
    }

    if (res == MGA_API_OK)
    {
      result = true;
    }
    else
    {
      res = mgaSessionStop();

      switch (res)
      {
      case MGA_API_NO_DATA_TO_SEND:
        print_err("Data from Server does not contain any LEG %s data.",
                  agnssServiceTypeToString(service));
        break;

      case MGA_API_BAD_DATA:
        print_err("Bad data received from MGA %s service.", agnssServiceTypeToString(service));
        break;

      case MGA_API_NO_MGA_INI_TIME:
        print_err("No UBX-AID-INI message in data from LEG %s service.",
                  agnssServiceTypeToString(service));
        break;

      case MGA_API_OUT_OF_MEMORY:
        print_err("Run out of memeory while processing data from LEG %s service.",
                  agnssServiceTypeToString(service));
        break;

      default:
        print_err("Unknown error occured while processing data from LEG %s service: %d",
                  agnssServiceTypeToString(service),
                  res);
        break;
      }
    }
  }
  else
  {
    print_err("Getting Assistance data from lEG %s service failed",
              agnssServiceTypeToString(service));
  }
  return result;
}

/*! Helper function which acts as a wrapper for some libMga functions
    during transfers

    \param buf        : A message that will be parsed for acknowledgments
                        from the receiver (May be NULL)
    \param size       : The size of buf (must be 0 if buf is NULL)
    \return             SUCCESS if the transfer completed successfully,
                        FAIL if the transfer failed,
                        CALL_AGAIN if the transfer is not complete yet and
                        the function has to be called again at least once
                        to complete
*/
TRI_STATE_FUNC_RESULT_t CAssistNowLeg::continueLegTransfer(unsigned char const *buf, size_t size)
{
  MGA_API_RESULT res = MGA_API_OK;
  TRI_STATE_FUNC_RESULT_t result = FAIL;
  if (buf && size)
  {
    res = mgaProcessReceiverMessage(const_cast<UBX_U1 *>(buf), size);
  }

  if (res == MGA_API_OK || res == MGA_API_IGNORED_MSG)
  {
    res = mgaCheckForTimeOuts();
  }

  if (res == MGA_API_OK)
  {
    // This function can never succeed on its own.
    result = _evt_handler_status;
  }
  return result;
}

/*! This function will initially send information on the stored Offline data
    to the receiver and will handle the following request of the receiver
    to update this data, respectively receive certain parts of it.

    \param buf        : A pointer to a message from the receiver which
                        is checked for containing the AssistNow Legacy
                        Offline requests. May be NULL.
    \param size       : The size of the data in bytes pointed to by buf
                        Must be 0 if buf is NULL.
    \return             SUCCESS if the transfer of the header completed
                        successfully,
                        FAIL if transferring header data failed,
                        CALL_AGAIN if there could be an incoming message
                        for more data from the receiver
*/
TRI_STATE_FUNC_RESULT_t CAssistNowLeg::offlineTransfer(unsigned char const *buf, size_t size)
{
  TRI_STATE_FUNC_RESULT_t result = FAIL;
  if (!_transfer_buf)
  {
    if (initLegTransfer(false))
    {
      result = CALL_AGAIN;
    }
  }
  else
  {
    result = continueLegTransfer(buf, size);

    if (result == CALL_AGAIN)
    {
      result = checkAlpSrvState();
    }
  }

  return result;
}

/*! This function checks if too many messages have been received not
    containing any ALP requests/updates from the receiver and stop
    the serving process if this is the case

    \return             SUCCESS If no error occured,
                        FAIL if the action that was requested failed,
                        CALL_AGAIN if there could be an incoming message
                        for more data from the receiver
*/
TRI_STATE_FUNC_RESULT_t CAssistNowLeg::checkAlpSrvState()
{
  TRI_STATE_FUNC_RESULT_t result = CALL_AGAIN;
  ++_num_client_answers;

  // Were too many messages ALPSRV unrelated?
  if (_num_client_answers >= LEG_MAX_ALPSRV_RECV_TRIES)
  {
    // Get updated data from libMga if there has been an update
    if (_alpsrv_data_save_req)
    {
      _alpsrv_data_save_req = false;
      if (saveToDb(OFFLINE, _transfer_buf, _transfer_size, &_action_data_id))
      {
        print_std("Saved the updated ALP data from the receiver");
        result = SUCCESS;
      }
      else
      {
        print_err("Saving the updated ALP data failed!");
      }
    }
    else
    {
      result = SUCCESS; // No answer received
    }
    print_std("No (more?) matching ALPSRV requests received from receiver. "
              "Stopping transfer.");
  }

  // Clean up if necessary
  if (result != CALL_AGAIN)
  {
    mgaStopLegacyAiding();
  }

  return result;
}

/*! This function will verify if the passed data is valid
    AssistNow Legacy Offline data.

    \param buf        : The buffer containing the AssistNow Legacy
                        Offline data that has to be checked. Must not be NULL
    \param size       : The size of the data pointed to by buf in bytes.
                        Must not be 0.
    \return             On success the 'true'. On failure 'false'.
*/
bool CAssistNowLeg::verifyOfflineData(unsigned char const *buf, size_t size)
{
  if (!buf || !size)
  {
    return false;
  }
  _online_msg_counter = 0;
  bool result = false;

  if (size > 0)
  {
    ALP_FILE_HEADER_t head;
    switch (verifyAlpData(buf, size, &head))
    {
    case 0:
    {
      print_std("Alp data with GPS time %d:%06d with a durtion of %.3fdays",
                head.p_wno,
                head.p_tow,
                (head.duration * 10.0) / (60.0 * 24.0));
      result = true;
      break;
    }
    case HELPER_ERR_INVALID_ARGUMENT:
    {
      print_err("Alp invalid argument");
      break;
    }
    case HELPER_ERR_ARG_TOO_SHORT:
    {
      print_err("Alp size %d too small", size);
      break;
    }
    case HELPER_ERR_INVALID_MAGIC:
    {
      print_err("Alp magic bad %08X", head.magic);
      break;
    }
    case HELPER_ERR_ENTRY_TOO_SHORT:
    {
      print_err("Alp size bad %d != %d", head.size * 4, size);
      break;
    }
    default:
    {
      print_err("Alp Unknown error!");
    }
    }
  }

  if (result)
  {
    print_std("The magic of the offline data is correct");
  }
  else
  {
    print_err("The magic of the offline data is incorrect");
  }

  return result;
}
