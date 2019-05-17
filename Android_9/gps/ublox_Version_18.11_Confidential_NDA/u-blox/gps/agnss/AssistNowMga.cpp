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
    \ref CAssistNowMga (AssistNow MGA Online/Offline) implementation.

    \brief

    Implementation of the functions defined by \ref CAssistNowMga, a class
    derived from \ref CAgnss
*/

#include "AssistNowMga.h"
#include "func/MemberFunc.h"
#include "helper/helperFunctions.h"
#include <assert.h>
#include <list>
#include <mutex>
#include <new>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

const struct UBX_MSG_TYPE CAssistNowMga::_recv_state_allowed[] = {
  { 0x13, 0x80 }, // DBD
};

const struct UBX_MSG_TYPE CAssistNowMga::_online_allowed[] = {
  { 0x13, 0x40 }, // INI
  { 0x13, 0x00 }, // GPS
  { 0x13, 0x02 }, // GAL
  { 0x13, 0x03 }, // BDS
  { 0x13, 0x06 }, // GLO
  { 0x13, 0x05 }, // QZSS
};

const struct UBX_MSG_TYPE CAssistNowMga::_offline_allowed[] = {
  { 0x13, 0x20 } // ANO
};

CAssistNowMga *CAssistNowMga::_singleton = NULL;
std::mutex CAssistNowMga::_singleton_mutex{};

/*! Will create an object of this class type.

    \param online_primary_server   : The zero-terminated string defines
                                     the host name of the server from which the
                                     AssistNow Legacy Online data will be tried
                                     to be downloaded at first. The contents of
                                     this string are copied, the string must
                                     thus only exist during the call to this
                                     function. Must not be NULL.
    \param online_secondary_server : The zero-terminated string defines
                                     the host name of the server from which the
                                     AssistNow Legacy Online data will be
                                     tried to be downloaded at a second
                                     attempt, if the first was unsuccessful.
                                     The contents of this string are copied,
                                     the string must thus only exist during
                                     the call to this function.
                                     Must not be NULL.
    \param offline_primary_server  : The zero-terminated string defines
                                     the host name of the server from which the
                                     AssistNow Legacy Offline data will be
                                     tried to be downloaded at first. The
                                     contents of this string are copied, the
                                     string must thus only exist during the
                                     call to this function. Must not be NULL.
    \param offline_secondary_server: The zero-terminated string defines
                                     the host name of the server from which the
                                     AssistNow Legacy Offline data will be
                                     tried to be downloaded at a second
                                     attempt, if the first was unsuccessful.
                                     The contents of this string are copied,
                                     the string must thus only exist during the
                                     call to this function. Must not be NULL.
    \param server_token            : Token to authenticate against the server
    \param cb                      : Structure of \ref CAgnss::CONF_t type
                                     containing the configuration for the base
                                     class.
*/
CAssistNowMga::CAssistNowMga(const char *online_primary_server,
                             const char *online_secondary_server,
                             const char *offline_primary_server,
                             const char *offline_secondary_server,
                             const char *server_token,
                             CONF_t *cb)
  : CAgnss(cb, "MGA: "), _ubx_cfg_navx5((void *)this, CAgnss::writeToRcv, 30),
    _ubx_mga_ini_time((void *)this, CAgnss::writeToRcv, 30),
    _ubx_mga_ini_pos((void *)this, CAgnss::writeToRcv, 30),
    _ubx_mga_dbd((void *)this, CAgnss::writeToRcv, 30),
    _ubx_poll_mga_dbd((void *)this, CAgnss::writeToRcv, 30), _action_data_id(0),
    _num_get_gnss_send_tries(-1), _num_get_gnss_max_send_tries(5), _num_get_gnss_recv_tries(0),
    _num_get_gnss_max_recv_tries(15), _num_config_ack_send_tries(0),
    _num_config_ack_max_send_tries(5), _transfer_buf(NULL), _transfer_size(0), _todays_data(NULL),
    _todays_data_size(0), _num_msgs_expected(0), _evt_handler_status(CALL_AGAIN),
    _event_config(NULL), _flow_config(NULL)
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
  _online_config.useFlags = 0;
  _online_config.gnssTypeFlags = 0;

  // Configure the offline part
  memset(&_offline_config, 0, sizeof(_offline_config));
  _offline_config.strPrimaryServer =
    offline_primary_server ? strdup(offline_primary_server) : strdup("");
  _offline_config.strSecondaryServer =
    offline_secondary_server ? strdup(offline_secondary_server) : strdup("");
  _offline_config.strServerToken = server_token ? strdup(server_token) : strdup("");
  _offline_config.gnssTypeFlags = 0;
  // The number of weeks into the future the MGA data should be valid for
  _offline_config.period = 2;
  // The resolution of the MGA data (days)
  _offline_config.resolution = 1;

  _fun_lookup[TIME][TRANSFER].push_back(FREF(CAssistNowMga, this, configureAidAck));
  _fun_lookup[TIME][TRANSFER].push_back(FREF(CAssistNowMga, this, timeTransfer));
  _fun_lookup[TIME][TRANSFER].push_back(FREF(CAssistNowMga, this, finishTimeTransfer));

  _fun_lookup[POSITION][TRANSFER].push_back(FREF(CAssistNowMga, this, configureAidAck));
  _fun_lookup[POSITION][TRANSFER].push_back(FREF(CAssistNowMga, this, timeTransfer));
  _fun_lookup[POSITION][TRANSFER].push_back(FREF(CAssistNowMga, this, posTransfer));

  _fun_lookup[ONLINE][DOWNLOAD].push_back(FREF(CAssistNowMga, this, getEnabledGnssFromRcv));
  _fun_lookup[ONLINE][DOWNLOAD].push_back(FREF(CAssistNowMga, this, onlineDownload));

  _fun_lookup[ONLINE][TRANSFER].push_back(FREF(CAssistNowMga, this, onlineTransfer));

  _fun_lookup[OFFLINE][DOWNLOAD].push_back(FREF(CAssistNowMga, this, getEnabledGnssFromRcv));
  _fun_lookup[OFFLINE][DOWNLOAD].push_back(FREF(CAssistNowMga, this, offlineDownload));

  _fun_lookup[OFFLINE][TRANSFER].push_back(FREF(CAssistNowMga, this, offlineTransfer));

  _fun_lookup[RECV_AID_STATE][POLL_RECV].push_back(FREF(CAssistNowMga, this, recvStatePoll));

  _fun_lookup[RECV_AID_STATE][TRANSFER].push_back(FREF(CAssistNowMga, this, configureAidAck));
  _fun_lookup[RECV_AID_STATE][TRANSFER].push_back(FREF(CAssistNowMga, this, timeTransfer));
  _fun_lookup[RECV_AID_STATE][TRANSFER].push_back(FREF(CAssistNowMga, this, recvStateTransfer));
}

/* Destructor freeing the ressources allocated and
   calling \ref teardown as required
*/
// Unfortunately lint does not seem to accept CAgnss::teardown
// as a cleanup function
// lint -e{1578}
// lint -e{1579}
CAssistNowMga::~CAssistNowMga()
{
  // teardown() will actually call impl_deinit() if required
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
  free(_todays_data);
  _todays_data = NULL;
  _todays_data_size = 0;
  _singleton = NULL;
}

/*! Creates a singleton of this class or returns the address to the singleton
    if it is already existing. For information on the parameters passed,
    please refer to the constructor documentation

    \param online_primary_server   : Passed on to the constructor if required
                                     otherwise ignored.
    \param online_secondary_server : Passed on to the constructor if required
                                     otherwise ignored.
    \param offline_primary_server  : Passed on to the constructor if required
                                     otherwise ignored.
    \param offline_secondary_server: Passed on to the constructor if required
                                     otherwise ignored.
    \param server_token            : Passed on to the constructor if required
                                     otherwise ignored.
    \param cb                      : Passed on to the constructor if required
                                     otherwise ignored.
    \return                          The address of the singleton if newly
                                     created, NULL otherwise
*/
CAssistNowMga *CAssistNowMga::createInstance(const char *online_primary_server,
                                             const char *online_secondary_server,
                                             const char *offline_primary_server,
                                             const char *offline_secondary_server,
                                             const char *server_token,
                                             CONF_t *cb)
{
  CAssistNowMga *tmpRef = NULL;
  std::lock_guard<std::mutex> lock(_singleton_mutex);
  if (!_singleton)
  {
    _singleton = new (std::nothrow) CAssistNowMga(online_primary_server,
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
CAssistNowMga *CAssistNowMga::getInstance()
{
  CAssistNowMga *tmpRef = NULL;
  std::lock_guard<std::mutex> lock(_singleton_mutex);
  if (_singleton)
  {
    tmpRef = _singleton;
  }

  return tmpRef;
}

/*! Implements impl_init as required by base. Will initiate the libMga.

    \return                          true on success, false otherwise
*/
bool CAssistNowMga::impl_init()
{
  bool result = false;

  if (_online_config.strPrimaryServer && _online_config.strSecondaryServer &&
      _online_config.strServerToken && _offline_config.strPrimaryServer &&
      _offline_config.strSecondaryServer && _offline_config.strServerToken)
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
bool CAssistNowMga::impl_deinit()
{
  free(_event_config);
  free(_flow_config);
  _event_config = NULL;
  _flow_config = NULL;
  return true;
}

//! Implements impl_initAction as required by base.
std::list<CFuncMngr> CAssistNowMga::impl_initAction(SERVICE_t service, ACTION_t action)
{
  std::list<CFuncMngr> result;
  if (isValidServiceAction(service, action))
  {
    result = _fun_lookup[service][action];
    // Clear local data
    _ubx_poll_mga_dbd.clearData();
    _ubx_mga_dbd.clearData();
    _ubx_mga_ini_time.clearData();
    _action_data_id = 0;
    _num_get_gnss_send_tries = -1;
    _evt_handler_status = CALL_AGAIN;
    _offline_config.gnssTypeFlags = 0;
    _online_config.gnssTypeFlags = 0;
    free(_transfer_buf);
    _transfer_buf = nullptr;
    _transfer_size = 0;

    // A transfer requires data to be stored already
    if (action == TRANSFER && !hasData(service))
    {
      result.clear();
    }
  }
  return result;
}

bool CAssistNowMga::impl_isValidServiceAction(SERVICE_t service, ACTION_t action)
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
bool CAssistNowMga::impl_isValidData(SERVICE_t service, unsigned char const *buf, size_t size)
{
  bool result = false;
  switch (service)
  {
  case RECV_AID_STATE:
  {
    result = (verifyUbxMsgsBlock(buf,
                                 size,
                                 _recv_state_allowed,
                                 sizeof(_recv_state_allowed) / sizeof(*_recv_state_allowed),
                                 NULL) > 0);
    break;
  }
  case ONLINE:
  {
    result =
      (verifyUbxMsgsBlock(
         buf, size, _online_allowed, sizeof(_online_allowed) / sizeof(*_online_allowed), NULL) > 0);
    break;
  }
  case OFFLINE:
  {
    result =
      (verifyUbxMsgsBlock(
         buf, size, _offline_allowed, sizeof(_offline_allowed) / sizeof(*_online_allowed), NULL) >
       0);
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

uint16_t CAssistNowMga::impl_getDataId() { return _action_data_id; }

/*! Initialises and configures the libMga to be used for transfer
    and download operations.

    \return             'true' on success otherwise 'false'
*/
bool CAssistNowMga::initMga()
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

/*! This function will first check which GNSSs the receiver supports and
    will download the corresponding data to the CAgnss super with \ref saveToDb.

    \param buf        : A message that will be parsed for supported AGNSS
                        (can be NULL)
    \param size       : The size of buf (must be 0 if buf is NULL)
    \return             SUCCESS if the download completed successfully,
                        FAIL if the download failed,
                        CALL_AGAIN if the download is not complete yet and
                        the function has to be called again at least once
                        to complete
*/
TRI_STATE_FUNC_RESULT_t CAssistNowMga::offlineDownload(unsigned char const *buf, size_t size)
{
  (void)(buf);
  (void)(size);
  TRI_STATE_FUNC_RESULT_t result = FAIL;
  // The initialisation of tmpBuf is a safety measure
  // lint -esym(438,tmpBuf)
  unsigned char *tmpBuf = NULL;
  int tmpSize = 0;
  if (initMga() && mgaGetOfflineData(&_offline_config, &tmpBuf, &tmpSize) == MGA_API_OK && tmpBuf &&
      tmpSize > 0)
  {
    if (saveToDb(OFFLINE, tmpBuf, tmpSize, &_action_data_id))
    {
      print_std("Successfully downloaded and stored MGA Offline data from the server");
      result = SUCCESS;
    }
    else
    {
      print_err("Storing the downloaded data for MGA Offline failed! Abort!");
    }
    free(tmpBuf);
    tmpBuf = NULL;
  }
  else
  {
    print_err("Downloading aiding data from the MGA Online server failed");
  }

  return result;
}

/*! This function will first check which GNSSs the receiver supports and
    will download the corresponding data to the CAgnss super with \ref saveToDb.

    \param buf        : A message that will be parsed for supported AGNSS
                        (can be NULL)
    \param size       : The size of buf (must be 0 if buf is NULL)
    \return             SUCCESS if the download completed successfully,
                        FAIL if the download failed,
                        CALL_AGAIN if the download is not complete yet and
                        the function has to be called again at least once
                        to complete
*/
TRI_STATE_FUNC_RESULT_t CAssistNowMga::onlineDownload(unsigned char const *buf, size_t size)
{
  (void)(buf);
  (void)(size);
  TRI_STATE_FUNC_RESULT_t result = FAIL;
  // lint -esym(438,tmpBuf)
  unsigned char *tmpBuf = NULL;
  int tmpSize = 0;
  if (initMga() && mgaGetOnlineData(&_online_config, &tmpBuf, &tmpSize) == MGA_API_OK && tmpBuf &&
      tmpSize > 0)
  {
    if (extractMgaIniTimeUtcMsg(tmpBuf, tmpSize, true))
    {
      if (saveToDb(ONLINE, tmpBuf, tmpSize, &_action_data_id))
      {
        print_std("Successfully downloaded and stored MGA Online data from the server");
        result = SUCCESS;
      }
    }
    else
    {
      print_err("Could not extract and clear the received time information");
    }

    if (result != SUCCESS)
    {
      print_err("Storing the downloaded data for MGA Online failed! Abort!");
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

/*! This function will try to transfer the offline data
    stored in CAgnss to the receiver. To be able to call this function
    either \ref offlineDownload or \ref saveToDb for this service must
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
TRI_STATE_FUNC_RESULT_t CAssistNowMga::offlineTransfer(unsigned char const *buf, size_t size)
{
  TRI_STATE_FUNC_RESULT_t result = FAIL;
  if (!_transfer_buf)
  {
    if (initMgaTransfer(false))
    {
      result = CALL_AGAIN;
    }
  }
  else
  {
    result = continueMgaTransfer(buf, size);
  }

  return result;
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
TRI_STATE_FUNC_RESULT_t CAssistNowMga::onlineTransfer(unsigned char const *buf, size_t size)
{
  TRI_STATE_FUNC_RESULT_t result = FAIL;
  if (!_transfer_buf)
  {
    if (initMgaTransfer(true))
    {
      result = CALL_AGAIN;
    }
  }
  else
  {
    result = continueMgaTransfer(buf, size);
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
bool CAssistNowMga::initMgaTransfer(bool online)
{
  ACCTIME_t accTimeNow;
  if (!getCurrentTime(&accTimeNow))
  {
    print_err("Could not load urrent time. Transfer aborted!");
    return false;
  }
  bool result = false;

  SERVICE_t service = OFFLINE;
  if (online)
  {
    service = ONLINE;
  }
  _transfer_size = loadFromDb(service, &_transfer_buf, &_action_data_id);
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

  if (initMga())
  {
    MGA_API_RESULT res = MGA_API_OK;
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
    if (service == ONLINE)
    {
      res = mgaSessionSendOnlineData(_transfer_buf, _transfer_size, &curr_time);
    }
    else
    {
      free(_todays_data);
      _todays_data = NULL;
      _todays_data_size = 0;
      res = mgaGetTodaysOfflineData(
        &nowTm, _transfer_buf, _transfer_size, &_todays_data, &_todays_data_size);
      if (res == MGA_API_OK)
      {
        res = mgaSessionSendOfflineData(_todays_data, _todays_data_size, &curr_time, NULL);
        if (res != MGA_API_OK)
        {
          print_err("An error occured while trying to send AssistNow Offline "
                    "data to the receiver. Abort");
        }
      }
      else
      {
        print_err("An error occured while trying to select the current "
                  "AssistNow Offline data. Abort");
      }
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
        print_err("Data from Server does not contain any MGA %s data.",
                  agnssServiceTypeToString(service));
        break;

      case MGA_API_BAD_DATA:
        print_err("Bad data received from MGA %s service.", agnssServiceTypeToString(service));
        break;

      case MGA_API_NO_MGA_INI_TIME:
        print_err("No MGA-INI-TIME message in data from MGA %s service.",
                  agnssServiceTypeToString(service));
        break;

      case MGA_API_OUT_OF_MEMORY:
        print_err("Run out of memeory while processing data from MGA %s service.",
                  agnssServiceTypeToString(service));
        break;

      default:
        print_err("Unknown error occured while processing data from MGA %s service: %d",
                  agnssServiceTypeToString(service),
                  res);
        break;
      }
    }
  }
  else
  {
    print_err("Getting Assistance data from MGA %s service failed",
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
TRI_STATE_FUNC_RESULT_t CAssistNowMga::continueMgaTransfer(unsigned char const *buf, size_t size)
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

/*! Function which will perform the transfer of the previously polled
    receiver state information back to the receiver.

    \param buf        : A message that will be parsed for acknowledgments
                        from the receiver (May be NULL)
    \param size       : The size of buf (must be 0 if buf is NULL)
    \return             SUCCESS if the transfer completed successfully,
                        FAIL if the transfer failed,
                        CALL_AGAIN if the transfer is not complete yet and
                        the function has to be called again at least once
                        to complete
*/
TRI_STATE_FUNC_RESULT_t CAssistNowMga::recvStateTransfer(unsigned char const *buf, size_t size)
{
  TRI_STATE_FUNC_RESULT_t result = FAIL;
  if (!_ubx_mga_dbd.hasSentData())
  {
    unsigned char *tmpBuf = NULL;
    ssize_t tmpSize = loadFromDb(RECV_AID_STATE, &tmpBuf, &_action_data_id);
    if (tmpSize >= 0)
    {
      if (_ubx_mga_dbd.setData(tmpBuf, tmpSize) && _ubx_mga_dbd.sendData())
      {
        result = CALL_AGAIN;
      }
      free(tmpBuf);
    }
  }
  else
  {
    result = _ubx_mga_dbd.onNewMsg(buf, size);
  }
  return result;
}

/*! This function will perform polling data from the receiver
    for the current receiver aiding state

    \param buf        : A message that will be parsed for answers
                        from the receiver (May be NULL)
    \param size       : The size of buf (must be 0 if buf is NULL)
    \return             SUCCESS on success,
                        FAIL of failure
                        CALL_AGAIN if additional calls to this
                        function are required
*/
TRI_STATE_FUNC_RESULT_t CAssistNowMga::recvStatePoll(unsigned char const *buf, size_t size)
{
  TRI_STATE_FUNC_RESULT_t result = FAIL;
  if (!_ubx_poll_mga_dbd.hasSentData())
  {
    if (_ubx_poll_mga_dbd.sendData())
    {
      result = CALL_AGAIN;
    }
  }
  else
  {
    result = _ubx_poll_mga_dbd.onNewMsg(buf, size);
    // If the data has been successfully receiver
    // save it to the DB
    if (result == SUCCESS)
    {
      result = FAIL;
      unsigned char *tmpBuf = NULL;
      ssize_t tmpSize = _ubx_poll_mga_dbd.getData(&tmpBuf);
      if (tmpSize > 0)
      {
        if (saveToDb(RECV_AID_STATE, tmpBuf, tmpSize, &_action_data_id))
        {
          result = SUCCESS;
        }
        free(tmpBuf);
      }
    }
  }
  return result;
}

/*! This function will try to transferring the current time to the receiver.

    \param buf        : A message that will be parsed for acknowledgments
                        from the receiver (May be NULL)
    \param size       : The size of buf (must be 0 if buf is NULL)

    \return             SUCCESS if the transfer completed successfully,
                        FAIL if the transfer failed,
                        CALL_AGAIN if the transfer is not complete yet and
                        the function has to be called again at least once
                        to complete
*/
TRI_STATE_FUNC_RESULT_t CAssistNowMga::timeTransfer(unsigned char const *buf, size_t size)
{
  TRI_STATE_FUNC_RESULT_t result = FAIL;

  // Send data if none is stored in this class yet
  if (!_ubx_mga_ini_time.isReadyToSend())
  {
    ACCTIME_t accTimeNow;
    GPS_UBX_AID_INI_U5__t Payload;

    memset(&accTimeNow, 0, sizeof(accTimeNow));
    memset(&Payload, 0, sizeof(Payload));

    if (getCurrentTime(&accTimeNow) && createUbxAidIni(&Payload, &accTimeNow))
    {
      char nowS[20];
      char accS[20];
      struct tm tmpTmp;
      strftime(nowS, 20, "%Y.%m.%d %H:%M:%S", gmtime_r(&accTimeNow.time.tv_sec, &tmpTmp));
      strftime(accS, 20, "%H:%M:%S", gmtime_r(&accTimeNow.acc.tv_sec, &tmpTmp));
      if (_ubx_mga_ini_time.setTime(&accTimeNow) && _ubx_mga_ini_time.sendData())
      {
        print_std("Sent the follwing UTC time for aiding the receiver:"
                  " %s.%09u (GPS: %uwn:%09ums:%09lins) Accuracy: %s.%.9u",
                  nowS,
                  accTimeNow.time.tv_nsec,
                  Payload.wn,
                  Payload.tow,
                  Payload.towNs,
                  accS,
                  accTimeNow.acc.tv_nsec);
        result = CALL_AGAIN;
      }
    }
  }
  else
  {
    result = _ubx_mga_ini_time.onNewMsg(buf, size);
    if (result == SUCCESS)
    {
      print_std("Successfully aided the receiver with the current time!");
    }
  }

  return result;
}

/*! This function will try to transfer the current position to the receiver.

    \param buf        : A message that will be parsed for acknowledgments
                        from the receiver (May be NULL)
    \param size       : The size of buf (must be 0 if buf is NULL)

    \return             SUCCESS if the transfer completed successfully,
                        FAIL if the transfer failed,
                        CALL_AGAIN if the transfer is not complete yet and
                        the function has to be called again at least once
                        to complete
*/
TRI_STATE_FUNC_RESULT_t CAssistNowMga::posTransfer(unsigned char const *buf, size_t size)
{
  TRI_STATE_FUNC_RESULT_t result = FAIL;

  // Send data if none is stored in this class yet
  if (!_ubx_mga_ini_pos.isReadyToSend())
  {
    POS_t pos;

    if (getCurrentPosition(&pos) && _ubx_mga_ini_pos.setPos(&pos) && _ubx_mga_ini_pos.sendData())
    {
      print_std("Sent the follwing position for aiding the receiver:"
                " latitude: %f longitude: %f Accuracy: %ucm",
                ((double)pos.latDeg) * 1e-7,
                ((double)pos.lonDeg) * 1e-7,
                pos.posAccCm);
      result = CALL_AGAIN;
    }
  }
  else
  {
    result = _ubx_mga_ini_pos.onNewMsg(buf, size);
    if (result == SUCCESS)
    {
      print_std("Successfully aided the receiver with the current position!");
      _action_data_id = 2;
    }
  }

  return result;
}

/*! This function will try set the action id to a valid  transferring the
   current time to the receiver.

    \param buf        : A message that will be parsed for acknowledgments
                        from the receiver (May be NULL)
    \param size       : The size of buf (must be 0 if buf is NULL)

    \return             SUCCESS if the transfer completed successfully,
                        FAIL if the transfer failed,
                        CALL_AGAIN if the transfer is not complete yet and
                        the function has to be called again at least once
                        to complete
*/
TRI_STATE_FUNC_RESULT_t CAssistNowMga::finishTimeTransfer(unsigned char const *buf, size_t size)
{
  ((void)(buf));
  ((void)(size));
  _action_data_id = 1;
  return SUCCESS; // This is always successful
}

/*! This function extracts the time that can be found in the first message
    of the passed message block and set the current time accordingly.

    \param buf        : Block of ubx messages of which the first one must
                        be a MGA-INI-TIME message.
                        Must not be NULL.
    \param size       : The size of buf. Must not be NULL.
    \param clearMsg   : true if the message should be cleared of data
                        and false otherwise

    \return             true on success, false otherwise
*/
bool CAssistNowMga::extractMgaIniTimeUtcMsg(unsigned char *buf, size_t size, bool clearMsg)
{
  const size_t UBX_MGA_INI_TIME_UTC_SIZE = 24 + 8;
  if (!buf || !size)
    return false;

  bool result = false;
  BUFC_t *tmpMsg = NULL;
  UBX_MSG_TYPE allowed;
  allowed.clsId = 0x13;
  allowed.msgId = 0x40;
  if (verifyUbxMsgsBlock(buf, UBX_MGA_INI_TIME_UTC_SIZE, &allowed, 1, &tmpMsg) > 0)
  {
    if (tmpMsg[0].p                                 // Valid pointer
        && tmpMsg[0].i == UBX_MGA_INI_TIME_UTC_SIZE // Valid UBX-MGA-INI-TIME_UTC length
        && tmpMsg[0].p[6] == 0x10)                  // TIME
    {
      // Inject the server time to the time handler
      unsigned char *iniPay = buf + 6;
      ACCTIME_t accTimeNow;
      MGA_INI_TIME_UTC_t tmpPay;
      memcpy(&tmpPay, iniPay, sizeof(tmpPay));
      if (extractMgaIniTimeUtc(&tmpPay, &accTimeNow))
      {
        setCurrentTime(&accTimeNow);
        if (clearMsg)
        {
          // Set the time fields to 0 in the INI message
          memset(iniPay + 4, 0, 7);
          // Set the nanosecond field to 0
          memset(iniPay + 12, 0, 4);

          // Calculate the new checksum, without the sync chars and
          // the current checksum
          uint16_t chk = calculateChecksum(tmpMsg[0].p + 2, tmpMsg[0].i - 4);
          buf[tmpMsg[0].i - 2] = (unsigned char)(chk >> 8); // Set part A
          buf[tmpMsg[0].i - 1] = (unsigned char)chk;        // Set part B
        }
        result = true;
      }
    }
    free(tmpMsg);
  }
  return result;
}

/*! This helper function will send a UBX message to request the enabled
    GNSS from the receiver for a specific service and store the libMga
    compatible flags in the memory location pointed to by 'supported'
    on success.

    \param buf        : A message that will be parsed for supported GNSS (May be
   NULL)
    \param size       : The size of buf (must be 0 if buf is NULL)
    \return             SUCCESS if the information was retrieved from the
   receiver
                        successfully and stored in 'supported',
                        FAIL if the operation failed
                        CALL_AGAIN if the operation is not complete yet and
                        the function has to be called again at least once
                        to complete
*/
TRI_STATE_FUNC_RESULT_t CAssistNowMga::getEnabledGnssFromRcv(unsigned char const *buf, size_t size)
{
  int supported = 0;
  unsigned char ubx_poll_cfggnss[] = { 0xB5, 0x62, 0x06, 0x3E, 0x00, 0x00, 0x44, 0xD2 };

  TRI_STATE_FUNC_RESULT_t result = CALL_AGAIN;

  // Request the currently enabled GNSS
  // until we get an answer from the receiver
  if (_num_get_gnss_send_tries < 0 || _num_get_gnss_recv_tries >= _num_get_gnss_max_recv_tries)
  {
    if (_num_get_gnss_send_tries > _num_get_gnss_max_send_tries)
    {
      print_std("Tried to send the request for the enabled GNSS from the "
                "receiver %i times. Receiver not responding",
                _num_get_gnss_max_send_tries);
      _num_get_gnss_send_tries = -1;
      _num_get_gnss_recv_tries = 0;
      result = FAIL;
    }
    else
    {
      if (_num_get_gnss_recv_tries >= _num_get_gnss_max_recv_tries)
      {
        print_err("Could not get the enabled GNSS from the receiver. Retrying...");
      }
      ++_num_get_gnss_send_tries;
      // This must be the first call to this function
      print_std("Requesting enabled GNSS from the receiver.");

      _num_get_gnss_recv_tries = 0;
      writeToRcv(ubx_poll_cfggnss, 8);
    }
  }
  else
  {
    ++_num_get_gnss_recv_tries;

    if (buf && size)
    {

      // Is it a valid UBX message length or is it non-UBX?
      // Check for a UBX header. (Could still be NMEA/invalid)
      if (size >= 12                               // long enough to be our message
          && (buf[0] == 0xb5) && (buf[1] == 0x62)  // UBX
          && (buf[2] == 0x06) && (buf[3] == 0x3E)) // CFG-GNSS
      {
        // The checksum must be right, thanks to CRcv
        print_std("Received list of enabled GNSS from the receiver.");

        if ((buf[6] == 0)) // Message version 0
        {
          unsigned char const *pData = buf + 6; // start of the actual data

          for (int i = 0; i < pData[3]; ++i) // go through all blocks received
          {
            switch (pData[4 + 8 * i])
            {
            case 0: // GPS
            {
              if (0x01 & pData[8 + 8 * i])
                supported |= MGA_GNSS_GPS;
              break;
            }
            case 2: // Galileo
            {
              if (0x01 & pData[8 + 8 * i])
                supported |= MGA_GNSS_GALILEO;
              break;
            }

            case 3: // BeiDou
            {
              if (0x01 & pData[8 + 8 * i])
                supported |= MGA_GNSS_BEIDOU;
              break;
            }
            case 5: // QZSS
            {
              // This makes only sense in Online mode
              if (0x01 & pData[8 + 8 * i])
                supported |= MGA_GNSS_QZSS;
              break;
            }

            case 6: // GLONASS
            {
              if (0x01 & pData[8 + 8 * i])
                supported |= MGA_GNSS_GLO;
              break;
            }
            default: // Should not happen
            {
              break;
            }
            }
          }
          _offline_config.gnssTypeFlags = (~MGA_GNSS_QZSS) & supported;
          _online_config.gnssTypeFlags = supported;

          // Operation successful. Let's get out of the loop
          result = SUCCESS;
        }
        else
        {
          print_err("Unknown version of the UBX message containing the list of "
                    "supported receivers: %u",
                    buf[6]);
        }
      }
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
void CAssistNowMga::sWriteToRcv(const void *context, unsigned char const *buf, int size)
{
  if (context)
  {
    ((CAssistNowMga *)const_cast<void *>(context))->writeToRcv(buf, size);
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
void CAssistNowMga::evtProgHandler(MGA_PROGRESS_EVENT_TYPE evtType,
                                   const void *pContext,
                                   const void *pEvtInfo,
                                   UBX_I4 evtInfoSize)
{
  if (pContext)
  {
    ((CAssistNowMga *)const_cast<void *>(pContext))->evtProgHandler(evtType, pEvtInfo, evtInfoSize);
  }
}

/*! This function will configure the receiver to send out acknowledgments
    for received MGA aiding data.

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
TRI_STATE_FUNC_RESULT_t CAssistNowMga::configureAidAck(unsigned char const *buf, size_t size)
{
  TRI_STATE_FUNC_RESULT_t result = FAIL;
  bool sendDataIfRequired = true;
  // If message sent out, check if there has been an answer
  if (_ubx_cfg_navx5.hasSentData())
  {
    result = _ubx_cfg_navx5.onNewMsg(buf, size);
    if (result == SUCCESS)
    {
      print_std("Acknowledgment for aiding messages enabled (Try %u/%u)",
                _num_config_ack_send_tries,
                _num_config_ack_max_send_tries);
      _num_config_ack_send_tries = 0;
      sendDataIfRequired = false;
    }
    else if (result == FAIL)
    {
      if (_num_config_ack_send_tries >= _num_config_ack_max_send_tries)
      {
        print_err("Did not receive acknowledgement from receiver that enabling "
                  "acknowledgment for aiding messages worked in %u tries. "
                  "Abort.",
                  _num_config_ack_max_send_tries);
        sendDataIfRequired = false;
        _num_config_ack_send_tries = 0;
      }
      else
      {
        print_std("Received no acknowledgment for enabling acknowledgments for "
                  "aiding in time (Try %u/%u)",
                  _num_config_ack_send_tries,
                  _num_config_ack_max_send_tries);
      }
    }
  }

  // Check if it is required to send the messsage
  if (!_ubx_cfg_navx5.hasSentData() && sendDataIfRequired)
  {
    print_std("Configure receiver to acknowledge aiding (Try %u/%u)",
              _num_config_ack_send_tries + 1,
              _num_config_ack_max_send_tries);
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

/*! This function will output information on the current status of
    the libMGA and will adjust the transfer status of the object
    depending on that information.

    \param evtType    : The event type triggered by the libMGA
    \param pEvtInfo   : Further information on the event which occured
    \param evtInfoSize: Size of the pEvtInfo object (not used)
*/
void CAssistNowMga::evtProgHandler(MGA_PROGRESS_EVENT_TYPE evtType,
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

    UBX_U1 gnssId = (UBX_U1)-1;
    UBX_U1 dataType = (UBX_U1)-1;
    bool needSvData = false;
    bool needDataType = false;

    switch (pMsgInfo->pMsg[3])
    {
    case 0x20: // ANO
    {
      gnssId = pMsgInfo->pMsg[6 + 3];
      needSvData = true;
      snprintf(msgTxt, TMP_STR_LEN, "ANO ");
      break;
    }

    case 0x00: // GPS
    case 0x02: // GAL
    case 0x03: // BDS
    case 0x05: // QZSS
    case 0x06: // GLO
    {
      needDataType = true;
      gnssId = pMsgInfo->pMsg[3];
      dataType = pMsgInfo->pMsg[6 + 0];

      if (dataType == 1 || dataType == 2) // In case of EPH or ALM data
        needSvData = true;

      break;
    }

    case 0x40: // INI
    {
      dataType = pMsgInfo->pMsg[6 + 0];
      UBX_U1 iniType = (UBX_U1)-1;

      iniType = ((dataType & 0xF0) >> 4) * 2 + (dataType & 0x0F);

      snprintf(msgTxt, TMP_STR_LEN, "%s(0x%02x)", mgaIniDataTypeToString(iniType), iniType);
      break;
    }

    default:
    {
      snprintf(msgTxt, TMP_STR_LEN, "Unknown Message[0x%02x]", pMsgInfo->pMsg[3]);
    }
    }

    // To which GNSS belongs this message?
    if (gnssId != (UBX_U1)-1) // This is neither a INI nor an UNKNOWN message
    {
      snprintf(msgTxt + strlen(msgTxt), TMP_STR_LEN - strlen(msgTxt), "%s", gnssIdToString(gnssId));
    }

    // Which datatype is used in the message
    if (needDataType)
    {
      snprintf(msgTxt + strlen(msgTxt),
               TMP_STR_LEN - strlen(msgTxt),
               "-%s",
               aidingDataTypeToString(dataType - 1));
    }

    if (needSvData) // Add SV Data to the output
    {
      UBX_U1 sv = pMsgInfo->pMsg[6 + 2];
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
      // No MGA-INI-TIME not received prior to MGA data messages
      snprintf(msgTxt + strlen(msgTxt),
               TMP_STR_LEN - strlen(msgTxt),
               "No MGA-TIME-INI message - Terminating...");
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
    print_std("Requesting MGA data from service (Request header)");
    break;
  }

  case MGA_PROGRESS_EVT_RETRIEVE_DATA:
  {
    // pEvtInfo is NULL
    print_std("Retrieving MGA data from service");
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
               "Could not download all MGA contents");
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

  default:
  {
    print_err("Unexpected event type! [%i]!", evtType);
    break;
  }
  }
}
