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
 * $Id: AssistNowMga.h 112239 2016-03-09 10:37:08Z fabio.robbiani $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/agnss/AssistNowMga.h $
 *****************************************************************************/
 
/*! \file
    \ref CAssistNowMga (AssistNow MGA Online/Offline) declaration.

    \brief
    Declaration of \ref CAssistNowMga, a class derived from \ref CAgnss
*/

#ifndef __UBX_MGA_H__
#define __UBX_MGA_H__
#include "mga/libMga.h"
#include "Agnss.h"
#include "helper/helperTypes.h"
#include "helper/UbxMgaIniPosLlh.h"
#include "helper/UbxMgaIniTimeUtc.h"
#include "helper/UbxCfgNavX5.h"
#include "helper/UbxMgaDbd.h"
#include "helper/UbxPollMgaDbd.h"

/*! \class CAssistNowMga
    \brief Interfaces the libMGA and implements \ref CAgnss

    This class implements the interface described by \ref
    CAgnss and interfaces the libMGA. It will communicate
    with u-blox receiver generations >=8 and the AssistNow
    MGA Online and Offline servers of u-blox.
*/
class CAssistNowMga : public CAgnss
{
private:
    //! Private constructor for the singleton
    CAssistNowMga(const char *online_primary_server,
         const char *online_secondary_server,
         const char *offline_primary_server,
         const char *offline_secondary_server,
         const char *server_token, 
         CONF_t *cb); // Private because singleton

public:
    //! Default destructor
    ~CAssistNowMga();

    //! Creates a singleton of this class type
    static CAssistNowMga *createInstance(const char *online_primary_server,
                                         const char *online_secondary_server,
                                         const char *offline_primary_server,
                                         const char *offline_secondary_server,
                                         const char *server_token,
                                         CONF_t *cb);

    // Return the address of the singleton
    static CAssistNowMga *getInstance();
protected:
    /////////////////////////////////////////////////////////////////
    // Start function declarations as required to implemet by base //
    /////////////////////////////////////////////////////////////////
    virtual bool impl_init();
    virtual bool impl_deinit();
    virtual CList<CFuncMngr> impl_initAction( SERVICE_t service
                                             , ACTION_t action );
    virtual bool impl_isValidServiceAction( SERVICE_t service
                                          , ACTION_t action );
    virtual bool impl_isValidData( SERVICE_t service
                                 , unsigned char const *buf
                                 , size_t size );
    virtual uint16_t impl_getDataId();
    ////////////////////////////////////////////////////////////////
    // Stop function declarations as required to implemet by base //
    ////////////////////////////////////////////////////////////////
private:
    
    //! Initialises the libMga and starts the session
    bool initMga();

    //! Downloads offline data for the specified service and save it
    TRI_STATE_FUNC_RESULT_t offlineDownload( unsigned char const *buf
                                           , size_t size);

    //! Downloads offline data for the specified service and save it
    TRI_STATE_FUNC_RESULT_t onlineDownload( unsigned char const *buf
                                          , size_t size);

    //! Performs transfer of offline data for the specified to the receiver 
    TRI_STATE_FUNC_RESULT_t offlineTransfer( unsigned char const *buf
                                           , size_t size);

    //! Performs transfer of offline data for the specified to the receiver 
    TRI_STATE_FUNC_RESULT_t onlineTransfer( unsigned char const *buf
                                          , size_t size);

    //! Initiates the transfer to the receiver for Online and Offline data 
    bool initMgaTransfer( bool online );

    //! Continues to transfer the data for the specified service to the receiver 
    TRI_STATE_FUNC_RESULT_t continueMgaTransfer( unsigned char const *buf
                                               , size_t size);

    //! Transfers the receiver state information back to the receiver 
    TRI_STATE_FUNC_RESULT_t recvStateTransfer( unsigned char const *buf
                                             , size_t size );

    //! Poll the receiver state information back from the receiver 
    TRI_STATE_FUNC_RESULT_t recvStatePoll( unsigned char const *buf
                                         , size_t size );

    //! Transfer the current time to the receiver
    TRI_STATE_FUNC_RESULT_t timeTransfer( unsigned char const *buf
                                        , size_t size );

    //! Transfer the current position to the receiver
    TRI_STATE_FUNC_RESULT_t posTransfer( unsigned char const *buf
                                       , size_t size);

    //! Transfer the current time to the receiver
    TRI_STATE_FUNC_RESULT_t finishTimeTransfer( unsigned char const *buf
                                              , size_t size );

    //! Extract time information from MGA-INI-TIME message
    bool extractMgaIniTimeUtcMsg(unsigned char * buf, size_t size, bool clearMsg);

    //! Tries to receive the enabled GNSS from the receiver
    TRI_STATE_FUNC_RESULT_t getEnabledGnssFromRcv( unsigned char const *buf
                                                 , size_t size);

    //! The function used by libMGA to write to the receiver
    static void sWriteToRcv(const void *pContext, unsigned char const *buf,
                            int size);

    //! The event handler called by \ref evtProgHandler used by libMGA
    static void evtProgHandler( MGA_PROGRESS_EVENT_TYPE evtType
                              , const void *pContext
                              , const void *pEvtInfo
                              , UBX_I4 evtInfoSize );

    //! The event handler called by \ref evtProgHandler used by libMGA
    void evtProgHandler( MGA_PROGRESS_EVENT_TYPE evtType
                       , const void *pEvtInfo
                       , UBX_I4 evtInfoSize );

    //! Configure receiver to acknowledge aiding messages
    TRI_STATE_FUNC_RESULT_t configureAidAck( unsigned char const * buf
                                           , size_t size);

    CUbxCfgNavX5 _ubx_cfg_navx5;                //!< Enables aiding
    CUbxMgaIniTimeUtc _ubx_mga_ini_time;        //!< UBX-AID-INI-TIME_UTC handler
    CUbxMgaIniPosLlh _ubx_mga_ini_pos;          //!< UBX-AID-INI-POS_LLH handler
    CUbxMgaDbd _ubx_mga_dbd;                    //!< Get database state
    CUbxPollMgaDbd _ubx_poll_mga_dbd;           //!< Get database state
    uint16_t _action_data_id;                   //!< Checksum of the data used
                                                //!< for the last finished action
    int _num_get_gnss_send_tries;               //!< Used by
                                                //!< \ref getEnabledGnssFromRcv
                                                //!< to keep track of the
                                                //!< request messages sent
    const int _num_get_gnss_max_send_tries;     //!< Maximum number of requests
                                                //!< to the receiver
                                                //!< \ref getEnabledGnssFromRcv
                                                //!< will send
    int _num_get_gnss_recv_tries;               //!< Used by
                                                //!< \ref getEnabledGnssFromRcv
                                                //!< to keep track of the
                                                //!< messages received that
                                                //!< might contain the required
                                                //!< information
    const int _num_get_gnss_max_recv_tries;     //!< Maximum number of answers
                                                //!< \ref getEnabledGnssFromRcv
                                                //!< will wait for before
                                                //!< determining that no valid
                                                //!< answer has been received
    int _num_config_ack_send_tries;             //!< Keeps track of the number
                                                //!< of tries while trying to 
                                                //!< configure the receiver
                                                //!< to send aiding ACKs
    int _num_config_ack_max_send_tries;         //!< Maximum number of tries
                                                //!< to enable acks for aiding
    unsigned char *_transfer_buf;               //!< Buffer that will be used
                                                //!< during the transfer of
                                                //!< online and offline data to
                                                //!< the receiver
    size_t  _transfer_size;                     //!< Size of _transfer_buf in bytes
    unsigned char *_todays_data;                //!< Buffer that will be used
                                                //!< during the transfer of offline
                                                //!< data to the receiver, after
                                                //!< the data from today has been
                                                //!< selected from _transfer_buf
    int _todays_data_size;                      //!< size of _todays_data in bytes 
    int _num_msgs_expected;                     //!< Number of messages the event
                                                //!< handler is transferring
    TRI_STATE_FUNC_RESULT_t _evt_handler_status;//!< Status of the event handler
    MgaEventInterface *_event_config;           //!< libMGA event config
    MgaFlowConfiguration *_flow_config;         //!< libMGA flow config
    MgaOnlineServerConfig _online_config;       //!< libMGA online server config
    MgaOfflineServerConfig _offline_config;     //!< libMGA offline server config
    CList<CFuncMngr>                            //!< Defines the action points
          _fun_lookup[_NUM_SERVICES_]           //!< for a each Action/Service
                     [_NUM_ACTIONS_];           //!< combination

    static CAssistNowMga *_singleton;           //!< Reference to the singleton
    static pthread_mutex_t _singleton_mutex;    //!< Mutex to protect singleton
    static const struct UBX_MSG_TYPE
                        _online_allowed[];      //!< Valid MGA Online UBX msgs
    static const struct UBX_MSG_TYPE
                        _offline_allowed[];     //!< Valid MGA Offline UBX msgs
    static const struct UBX_MSG_TYPE            //!< Valid MGA Receiver state messages
                        _recv_state_allowed[];
};
#endif //ifndef __UBX_MGA_H__
