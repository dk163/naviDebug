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
    \ref CAssistNowLeg (AssistNow Legacy Online/Offline) definition.

    \brief
    Definition of \ref CAssistNowLeg, a class derived from \ref CAgnss
*/

#pragma once
#include "Agnss.h"
#include "helper/UbxAidEphAlm.h"
#include "helper/UbxCfgNavX5.h"
#include "helper/UbxPollAidEphAlm.h"
#include "mga/src/libMga.h"
#include <list>
#include <mutex>
#include <unistd.h>
#define LEG_MAX_SEND_TRIES 5            //!< Number of tries sending an
                                        //!< Online message
#define LEG_MAX_RECV_TRIES 15           //!< Number of messages to wait
                                        //!< for an ACK for an Online
                                        //!< message
#define LEG_MAX_CONFIG_ACK_SEND_TRIES 5 //!< Number of tries enabling
                                        //!< aiding ACKs
#define LEG_MAX_ALPSRV_RECV_TRIES 15    //!< Number of messages to wait
                                        //!< for a ALPSRV request
#define LEG_HTTP_TIMEOUT_S 5            //!< Number of seconds until a
                                        //!< HTTP timeout kicks in

/*! \class CAssistNowLeg
    \brief Implements \ref CAgnss and accesses the AssistNow legacy services

    This class implements the interface described by \ref CAgnss.
    It will communicate with u-blox receiver generations <8 and the AssistNow
    Legacy Online and Offline servers of u-blox.
*/
class CAssistNowLeg : public CAgnss
{
private:
  //! Private constructor for the singleton
  CAssistNowLeg(const char *online_primary_server,
                const char *online_secondary_server,
                const char *offline_primary_server,
                const char *offline_secondary_server,
                const char *server_token,
                CONF_t *cb);

public:
  //! Default destructor
  virtual ~CAssistNowLeg();

  //! Creates a singleton of this class type
  static CAssistNowLeg *createInstance(const char *online_primary_server,
                                       const char *online_secondary_server,
                                       const char *offline_primary_server,
                                       const char *offline_secondary_server,
                                       const char *server_token,
                                       CONF_t *cb);

  //! Return the address of the singleton
  static CAssistNowLeg *getInstance();

  //! Return the address of the singleton
  static void destroyInstance();

protected:
  /////////////////////////////////////////////////////////////////
  // Start function declarations as required to implemet by base //
  /////////////////////////////////////////////////////////////////
  virtual bool impl_init();
  virtual bool impl_deinit();
  virtual std::list<CFuncMngr> impl_initAction(SERVICE_t service, ACTION_t action);
  virtual bool impl_isValidServiceAction(SERVICE_t service, ACTION_t action);
  virtual bool impl_isValidData(SERVICE_t service, unsigned char const *buf, size_t size);
  virtual uint16_t impl_getDataId();
  ////////////////////////////////////////////////////////////////
  // Stop function declarations as required to implemet by base //
  ////////////////////////////////////////////////////////////////
private:
  //! Initialises the libMga and starts the session
  bool initLeg();

  //! Downloads online data and saves it
  TRI_STATE_FUNC_RESULT_t onlineDownload(unsigned char const *buf, size_t size);

  //! Downloads offline data and saves it
  TRI_STATE_FUNC_RESULT_t offlineDownload(unsigned char const *buf, size_t size);

  //! Offer the stored Offline data to the receiver and handle receiver requests
  TRI_STATE_FUNC_RESULT_t offlineTransfer(unsigned char const *buf, size_t size);

  //! Transfers online data and
  TRI_STATE_FUNC_RESULT_t onlineTransfer(unsigned char const *buf, size_t size);

  //! Initiates the transfer to the receiver for Online and Offline data
  bool initLegTransfer(bool online);

  //! Continues to transfer the data for the specified service to the receiver
  TRI_STATE_FUNC_RESULT_t continueLegTransfer(unsigned char const *buf, size_t size);
  //! Transfers the current time to the receiver
  TRI_STATE_FUNC_RESULT_t timeTransfer(unsigned char const *buf, size_t size);

  //! Transfers the current position to the receiver
  TRI_STATE_FUNC_RESULT_t posTransfer(unsigned char const *buf, size_t size);

  //! Helper function to transfer time and position
  TRI_STATE_FUNC_RESULT_t
  timePosTransfer(unsigned char const *buf, size_t size, bool forcePosition);

  //! Transfers the current time to the receiver
  TRI_STATE_FUNC_RESULT_t finishTimeTransfer(unsigned char const *buf, size_t size);

  //! The function used by libMGA to write to the receiver
  static void sWriteToRcv(const void *pContext, unsigned char const *buf, int size);

  //! Extract time information from UBX-AID-INI message
  bool extractLegIniTimeUtcMsg(unsigned char *buf, size_t size, bool clearMsg);

  //! The event handler called by \ref evtProgHandler used by libMGA
  static void evtProgHandler(MGA_PROGRESS_EVENT_TYPE evtType,
                             const void *pContext,
                             const void *pEvtInfo,
                             UBX_I4 evtInfoSize);

  //! The event handler called by \ref evtProgHandler used by libMGA
  void evtProgHandler(MGA_PROGRESS_EVENT_TYPE evtType, const void *pEvtInfo, UBX_I4 evtInfoSize);

  //! Insert current time into the msgs that have to be transferred
  bool insertTransferTimeMsg(unsigned char *msg);

  //! Transfer receiver state data to the receiver
  TRI_STATE_FUNC_RESULT_t recvStateTransfer(unsigned char const *buf, size_t size);

  //! Performs polling of the data for the specified service from the receiver
  TRI_STATE_FUNC_RESULT_t recvStatePoll(unsigned char const *buf, size_t size);

  //! Will enable acknowoledgment for legacy transfers to the receiver
  TRI_STATE_FUNC_RESULT_t configureAidAck(unsigned char const *buf, size_t size);

  //! Check ALP Server state
  TRI_STATE_FUNC_RESULT_t checkAlpSrvState();

  //! Verifies if data is valid AssistNow Legacy Offline data
  bool verifyOfflineData(unsigned char const *buf, size_t size);

  CUbxAidEphAlm _ubx_aid_eph_alm;              //!< Send current receiver state
  CUbxPollAidEphAlm _ubx_poll_aid_eph_alm;     //!< Poll current receiver state
  CUbxAidIni _ubx_aid_ini;                     //!< UBX-AID-INI handler
  CUbxCfgNavX5 _ubx_cfg_navx5;                 //!< Configure Acknowledgment
  TRI_STATE_FUNC_RESULT_t _action_state;       //!< Status of the action
  uint16_t _action_data_id;                    //!< Checksum of the data used
                                               //!< for the last finished action
  unsigned char *_transfer_buf;                //!< Buffer that will be used
                                               //!< during the transfer of
                                               //!< online and offline data to
                                               //!< the receiver
  size_t _transfer_size;                       //!< Size of _transfer_buf in bytes
  bool _sent_online_msg;                       //!< Used to determine if a
                                               //!< message has already been
                                               //!< sent during the online
                                               //!< transfer to the receiver
  size_t _online_msg_counter;                  //!< Identifies the current
                                               //!< message that is processed
                                               //!< while transferring the
                                               //!< online messages to the
                                               //!< receiver
  int _num_client_answers;                     //!< Keeps track of the number
                                               //!< of messages of any type
                                               //!< received from the receiver
                                               //!< while waiting for AssistNow
                                               //!< Legacy Offline ACKs
  size_t _num_aiding_send_tries;               //!< Keeps track of the number
                                               //!< of tries while sending
                                               //!< AssistNow Legacy Online
                                               //!< messages
  int _num_aiding_recv_tries;                  //!< Keeps track of the number
                                               //!< of messages of any type
                                               //!< received from the receiver
                                               //!< while waiting for AssistNow
                                               //!< Legacy Online ACKs
  int _num_config_ack_send_tries;              //!< Keeps track of the number
                                               //!< of tries while trying to
                                               //!< configure the receiver
                                               //!< to send aiding ACKs
  bool _alpsrv_data_save_req;                  //!< Save data back to storage
                                               //!< indicator
  int _num_msgs_expected;                      //!< Number of messages the event
                                               //!< handler is transferring
  TRI_STATE_FUNC_RESULT_t _evt_handler_status; //!< Status of the event handler
  MgaEventInterface *_event_config;            //!< libMGA event config
  MgaFlowConfiguration *_flow_config;          //!< libMGA flow config
  MgaOnlineServerConfig _online_config;        //!< libMGA online server config
  MgaOfflineServerConfig _offline_config;      //!< libMGA offline server config
  std::list<CFuncMngr>                         //!< Defines the action points
    _fun_lookup[_NUM_SERVICES_]                //!< for a each Action/Service
               [_NUM_ACTIONS_];                //!< combination

  static CAssistNowLeg *_singleton;   //!< Reference to the singleton
  static std::mutex _singleton_mutex; //!< Mutex to protect singleton

  static const struct UBX_MSG_TYPE _aid_allowed[]; //!< Valid LEG Online UBX msgs
};
