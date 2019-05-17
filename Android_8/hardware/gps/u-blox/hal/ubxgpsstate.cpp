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
 * $Id: ubxgpsstate.cpp 114179 2016-04-27 12:13:13Z fabio.robbiani $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/hal/ubxgpsstate.cpp $
 *****************************************************************************/

/*!
    \file
    Implementation of \ref CUbxGpsState

    \brief
    This file contains the implementation of \ref CUbxGpsState, which handles
    the state management for the u-blox GNSS receiver.
*/
#include <stdio.h>
#include <stdlib.h>

#include <errno.h>
#include <sys/stat.h>
#include <ctype.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <cutils/properties.h>
#include <string.h>
#include <pthread.h>

#ifndef ANDROID_BUILD
// Needed for Linux build
#include <malloc.h>
#include <string.h>
#define _LIBS_CUTILS_UIO_H
#endif

#include "std_types.h"
#include "ubx_messageDef.h"
#include "ubx_timer.h"
#include "ubx_log.h"
#include "ubx_cfg.h"
#include "ubx_xtraIf.h"
#include "ubx_rilIf.h"
#include "ubxgpsstate.h"
#include "protocolubx.h"
#include "../agnss/AssistNowLeg.h"
#include "../agnss/AssistNowMga.h"
#include "../agnss/helper/helperFunctions.h"
#ifdef SUPL_ENABLED
 #include "ubx_agpsIf.h"
#endif

///////////////////////////////////////////////////////////////////////////////
// Types & Definitions
#define AIDING_DATA_FILE "/data/persistance.agnss" //!< Default path to the persistence file
#define TIME_SOURCE                              0 //!< Default time source: system
#define SERPORT_DEFAULT             "/dev/ttyACM0" //!< Default receiver interface
#define SERPORT_BAUDRATE_DEFAULT              9600 //!< Default receiver interface baudrate
#define SHUTDOWN_TIMEOUT_DEFAULT                10 //!< Default shutdown timeout: 10 seconds
#define AGNSS_OFFLINE_INTERVAL_DEFAULT         720 //!< Default AssistNow Offline download interval: 12 hours
#define AGNSS_ONLINE_INTERVAL_DEFAULT          120 //!< Default AssistNow Online download interval: 2 hours

#ifdef SUPL_ENABLED
#define MSA_RESPONSE_DELAY_DEFAULT              10 //!< Default timeout (in seconds) to response with psedo ranges for MSA session
#define NI_UI_TIMEOUT_DEFAULT                  120 //!< Default timeout (in seconds) to display NI Notify/verify dialog
#define NI_RESPONSE_TIMEOUT                     75 //!< Default timeout (in seconds) to respond to an NI request
#endif

///////////////////////////////////////////////////////////////////////////////
// Static data
static CUbxGpsState s_ubxGpsState;
const size_t CUbxGpsState:: m_maxConsFailedTransf=5;
const uint8_t CUbxGpsState::DB_VERSION_NO=1;
///////////////////////////////////////////////////////////////////////////////
//! Constructor
CUbxGpsState::CUbxGpsState()
{

    memset(&m_Db, 0, sizeof(m_Db));
    m_Db.rateMs = 1000;

    CCfg cfg;

	// Has inner 4G TBox
	char tboxType[PROPERTY_VALUE_MAX];
	property_get("persist.chinatsp.tbox.type", tboxType, "0");
	int type = atoi(tboxType);
	if (1 == type) 
	{
		cfg.load("/system/etc/u-blox-tbox.conf");
	} 
	else 
	{
		cfg.load("/system/etc/u-blox.conf");
	}

    m_pSerialDevice         =strdup( cfg.get("SERIAL_DEVICE",         SERPORT_DEFAULT) );
    m_baudRate                     = cfg.get("BAUDRATE" ,             SERPORT_BAUDRATE_DEFAULT);
    m_baudRateDef                  = cfg.get("BAUDRATE_DEF" ,         SERPORT_BAUDRATE_DEFAULT);
    m_i2cTxReady.enabled    =      ( cfg.get("I2C_TX_READY_ENABLED",  0) > 0);
    m_i2cTxReady.recvPio           = cfg.get("I2C_TX_READY_RECV_PIO", 0);
    m_i2cTxReady.hostGpio          = cfg.get("I2C_TX_READY_HOST_GPIO",0);
    m_stoppingTimeoutMs            = cfg.get("STOP_TIMEOUT",          SHUTDOWN_TIMEOUT_DEFAULT) * 1000;
    m_receiverGeneration           = cfg.get("RECEIVER_GENERATION",   7);
    m_agnssStrategy                = cfg.get("AGNSS_STRATEGY",        0);
    m_agnssPersistence             = cfg.get("AGNSS_PERSISTENCE",     1);
    m_pAgnssPersistenceFile =strdup( cfg.get("AGNSS_PERSISTENCE_FILE",AIDING_DATA_FILE) );
    m_timeSource                   = cfg.get("AGNSS_TIME_SOURCE",     TIME_SOURCE);
    m_agnssOnlineInterval          = cfg.get("AGNSS_ONLINE_INTERVAL", AGNSS_ONLINE_INTERVAL_DEFAULT) * 60 * 1000;
    m_agnssOfflineInterval         = cfg.get("AGNSS_OFFLINE_INTERVAL",AGNSS_OFFLINE_INTERVAL_DEFAULT) * 60 * 1000;
    m_pAgnssOnlineServer1   =strdup( cfg.get("AGNSS_ONLINE_SERVER1",  ""));
    m_pAgnssOnlineServer2   =strdup( cfg.get("AGNSS_ONLINE_SERVER2",  ""));
    m_pAgnssOfflineServer1  =strdup( cfg.get("AGNSS_OFFLINE_SERVER1", ""));
    m_pAgnssOfflineServer2  =strdup( cfg.get("AGNSS_OFFLINE_SERVER2", ""));
    m_pAgnssToken           =strdup( cfg.get("AGNSS_TOKEN",           ""));

    if(m_agnssOnlineInterval == 0)
    {
        m_agnssOnlineInterval=60*1000;
        UBX_LOG(LCAT_WARNING, "Configuration value AGNSS_ONLILNE_INTERVAL too small! Set to the value 1");
    }

    if(m_agnssOfflineInterval == 0)
    {
        m_agnssOfflineInterval=60*1000;
        UBX_LOG(LCAT_WARNING, "Configuration value AGNSS_OFFLILNE_INTERVAL too small! Set to the value 1");
    }

#ifdef SUPL_ENABLED
    m_almanacRequest            = (bool) cfg.get("SUPL_ALMANAC_REQUEST",            false);
    m_utcModelRequest           = (bool) cfg.get("SUPL_UTC_MODEL_REQUEST",          false);
    m_ionosphericModelRequest   = (bool) cfg.get("SUPL_IONOSPHERIC_MODEL_REQUEST",  false);
    m_dgpsCorrectionsRequest    = (bool) cfg.get("SUPL_DGPS_CORRECTIONS_REQUEST",   false);
    m_refLocRequest             = (bool) cfg.get("SUPL_REF_LOC_REQUEST",            false);
    m_refTimeRequest            = (bool) cfg.get("SUPL_REF_TIME_REQUEST",           false);
    m_acquisitionAssistRequest  = (bool) cfg.get("SUPL_AQUISITION_ASSIST_REQUEST",  false);
    m_realTimeIntegrityRequest  = (bool) cfg.get("SUPL_TIME_INTEGRITY_REQUEST",     false);
    m_navigationModelRequest    = (bool) cfg.get("SUPL_NAVIGATIONAL_MODEL_REQUEST", false);
    m_fakePhone                 = (bool) cfg.get("SUPL_FAKE_PHONE_CONNECTION",      false);
    m_niUiTimeout               =        cfg.get("SUPL_NI_UI_TIMEOUT",              NI_UI_TIMEOUT_DEFAULT);
    m_niResponseTimeout         =        cfg.get("SUPL_NI_RESPONSE_TIMEOUT",        NI_RESPONSE_TIMEOUT);
    m_logSuplMessages           = (bool) cfg.get("SUPL_LOG_MESSAGES", false);
    m_cmccLogActive             = (bool) cfg.get("SUPL_CMCC_LOGGING", false);
    m_suplMsgToFile             = (bool) cfg.get("SUPL_MSG_TO_FILE", false);
#endif

    m_pSer = NULL;
#if defined UDP_SERVER_PORT
    m_pUdpServer = NULL;
#endif

    m_timeLastOfflineReq = getMonotonicMsCounter();
    m_timeLastOnlineReq = 0;

    memset(m_transferRequired, false, sizeof(m_transferRequired));
    memset(m_currentChecksum, 0, sizeof(m_currentChecksum));
    m_dbChanged=false;
    m_currState=OFF;
    m_inTransition=false;
    m_pollRecvStateFinished=false;
    m_timeoutStart=0;
    m_consFailedTransf=0;
    pthread_mutex_init(&m_ubxStateMutex, NULL);

    // Prepare u-blox service interface
    CAgnss::CONF_t agnss_config;
    memset(&agnss_config, 0, sizeof(CAgnss::CONF_t));
    agnss_config.func_std_print=CUbxGpsState::printStd;
    agnss_config.func_err_print=CUbxGpsState::printErr;
    agnss_config.func_transfer_finished=CUbxGpsState::finished;
    agnss_config.func_write_to_rcv=CUbxGpsState::writeToRcv;
    agnss_config.time_source=(m_timeSource==1?CAgnss::TIME_INTERNAL:CAgnss::TIME_SYSTEM);
    agnss_config.context=(void *) this;
    m_Db.pAgnssIf=NULL;
    if(getReceiverGeneration()<=7) // Legacy interface
    {
        m_Db.pAgnssIf=CAssistNowLeg::createInstance(m_pAgnssOnlineServer1,
                                                    m_pAgnssOnlineServer2,
                                                    m_pAgnssOfflineServer1,
                                                    m_pAgnssOfflineServer2,
                                                    getAgnssToken(),
                                                    &agnss_config);
    }
    else // MGA interface
    {
        m_Db.pAgnssIf=CAssistNowMga::createInstance(m_pAgnssOnlineServer1,
                                                    m_pAgnssOnlineServer2,
                                                    m_pAgnssOfflineServer1,
                                                    m_pAgnssOfflineServer2,
                                                    getAgnssToken(),
                                                    &agnss_config);
    }
    if(!m_Db.pAgnssIf)
    {
        UBX_LOG(LCAT_ERROR, "Could not create the AGNSS interface!");
    }
    else
    {
        if(!m_Db.pAgnssIf->setup())
        {
            UBX_LOG(LCAT_ERROR, "Could not initialise the AGNSS interface!");
            m_Db.pAgnssIf->teardown();
            delete m_Db.pAgnssIf;
            m_Db.pAgnssIf=NULL;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
//! Destructor
CUbxGpsState::~CUbxGpsState()
{
    // write back the data to the file
    freeBufs(m_Db.aopG, NUM_GPS_SVS);

    if (m_pAgnssToken)          free(m_pAgnssToken);
    if (m_pAgnssOnlineServer1)  free(m_pAgnssOnlineServer1);
    if (m_pAgnssOnlineServer2)  free(m_pAgnssOnlineServer2);
    if (m_pAgnssOfflineServer1) free(m_pAgnssOfflineServer1);
    if (m_pAgnssOfflineServer2) free(m_pAgnssOfflineServer2);
    if (m_pSerialDevice)        free(m_pSerialDevice);
    if (m_pAgnssPersistenceFile)         free(m_pAgnssPersistenceFile);
    m_pSer = NULL;          // Never allocated in this class, so do not need to free
#if defined UDP_SERVER_PORT
    m_pUdpServer = NULL;    // Never allocated in this class, so do not need to free
#endif

    // Delete agnss_if
    delete m_Db.pAgnssIf;
    m_Db.pAgnssIf=NULL;
    pthread_mutex_destroy(&m_ubxStateMutex);
}

//! Return the singleton
CUbxGpsState* CUbxGpsState::getInstance()
{
    return &s_ubxGpsState;
}

/*******************************************************************************
 * Thread safety
 *******************************************************************************/

//! Lock this object
void CUbxGpsState::lock(void)
{
    pthread_mutex_lock(&m_ubxStateMutex);
}

//! Unlock this object
void CUbxGpsState::unlock(void)
{
    pthread_mutex_unlock(&m_ubxStateMutex);
}

//! Init this object
/*! Initiates this object with the information required for operation.

    \param pSer            : A pointer to the interface the receiver is on
                             Must not be NULL
    \return                  true on success, false otherwise
*/
bool CUbxGpsState::init(CSerialPort* pSer)
{
    if(!pSer)
        return false;

    m_pSer = pSer;
    setBaudRate();
    forceTurnOff();

    m_currState=OFF;
    m_inTransition=false;
    return true;
}

//! Initiate a state change of the receiver
/*! Makes the receiver change to the desired state if there is a valid
    transition from the current state to the desired one.

    \param newState        : The desired state
    \return                  true on success, false otherwise
*/
bool CUbxGpsState::turnRecv(RECV_STATE_t newState)
{
    // External calls always require a transition!
    return changeState(newState, true);
}

//! Is the receiver in the state passed as argument?
/*! Checks if the receiver is in the state as passed by argument. Will return
    false if the receiver is in transition between two modes.

    \param mode            : The state against which the current state has
                             to be compared.
    \return                  true if there is a match and the receiver is
                             is not in transition and false otherwise.
*/
bool CUbxGpsState::isRecv(RECV_STATE_t mode)
{
    bool result=false;
    if(mode==m_currState && !m_inTransition)
    {
        result=true;
    }
    return result;
}

//! Is the receiver in transition to the state passed as argument
/*! Checks if the receiver is in transition to the state as passed by argument.

    \param mode            : The state against which the current state has
                             has to be compared.
    \return                  true if there is a match and the receiver
                             is in transition to the passed state
*/
bool CUbxGpsState::isRecvInTransTo(RECV_STATE_t mode)
{
    bool result=false;
    if(mode==m_currState && m_inTransition)
    {
        result=true;
    }
    return result;
}

//! Change the current receiver and transition state to the desired state
/*! This helper function will change the current receiver and tranistion
    state to the desired state passed as an argument, if the desired state
    is valid.

    \param targetState     : The desired receiver state
    \param transition      : The desired transition state
    \return                  true on success, false otherwise
*/
bool CUbxGpsState::changeState(RECV_STATE_t targetState, bool transition)
{
    if(!m_pSer)
        return false;

    bool result=false;
    // Has transition completed since the last check?
    if(m_currState == targetState && m_inTransition != transition && !transition)
    {
        switch(m_currState)
        {
            case OFF: // Receiver is now stopped
            {
                // Stop the current receiver state save action
                if(m_Db.pAgnssIf)
                {
                    m_Db.pAgnssIf->clearActions();
                }
                if(hasDbChanged())
                {
                    saveAiding();
                    setDbChanged(false);
                }

                forceTurnOff();
                break;
            }
            case ON: // Receiver is now on
            default:
            {
                break;
            }
        }
        m_inTransition=false;
        UBX_LOG(LCAT_VERBOSE, "The receiver is now turned %s", targetState==ON?"on":"Off");
        result=true;
    }
    else if(m_currState != targetState && transition) // Has a new state transition started?
    {
        UBX_LOG(LCAT_VERBOSE,"A receiver state change has been detected. Turn receiver %s!", targetState==ON?"on":"Off");
        switch(targetState)
        {
            case OFF: // Turn receiver off now
            {
                prepareTurnOff();
                break;
            }
            case ON: // Turn receiver on now
            default:
            {
                turnOn();
                break;
            }
        }
        m_currState = targetState;
        m_inTransition=true;
        result=true;
    }
    return result;
}


/*******************************************************************************
 * STARTUP / SHUTDOWN
 *******************************************************************************/

//! on Statup Sequence
/*! Exectute the sequence needed to restart the receiver, this includes the
    following steps:
    - powering up the receiver (must be implemented by customer.
      Please refer to \ref powerOn / \ref powerOff)
    - restore the local aiding information from the persistence file
    - enableing the GNSS receiver
    - configure the receiver
    - initiate all the required aiding
*/
void CUbxGpsState::turnOn(void)
{
    // load any previous aiding data
    loadAiding();

    LOGGPS(0x00000000, "#gps start");

    // Switch power on to the gps device
    if(powerOn())
        setBaudRate();

    // turn on the gps, we assume that the gps is turned off
    writeUbxCfgRst(0x09/*gps start*/, 0);

    // Wait 100 msec before sending the rest...
    usleep(100000);

    // Configuration
    // ---------------------------------------

    // set the desired navigation rate
    writeUbxCfgRate();
    usleep(50000);
    // enable additional messages and protocols this gives accuracies
    writeUbxCfgMsg(0x01, 0x03); // enable UBX-NAV-STATUS
    writeUbxCfgMsg(0x01, 0x30); // enable UBX-NAV-SVINFO
    writeUbxCfgMsg(0xF0, 0x00); // enable NMEA-GGA
    writeUbxCfgMsg(0xF0, 0x01); // enable NMEA-GLL
    writeUbxCfgMsg(0xF0, 0x02); // enable NMEA-GSA
    usleep(50000);
    writeUbxCfgMsg(0xF0, 0x03); // enable NMEA-GSV
    writeUbxCfgMsg(0xF0, 0x04); // enable NMEA-RMC
    writeUbxCfgMsg(0xF0, 0x05); // enable NMEA-VTG
    writeUbxCfgMsg(0xF0, 0x06); // enable NMEA-GRS
    usleep(50000);
    writeUbxCfgMsg(0xF0, 0x07); // enable NMEA-GST
    writeUbxCfgMsg(0xF0, 0x08); // enable NMEA-ZDA
    writeUbxCfgMsg(0xF0, 0x09); // enable NMEA-GBS
    writeUbxCfgMsg(0xF0, 0x0D); // enable NMEA-GNS

    usleep(50000);
    if(isAopEnabled())
    {
        writeUbxCfgMsg(0x0B, 0x33);     // enable UBX-AID-AOP message
        sendAopIfRequired();
        writeUbxNavX5AopEnable(); // enable AOP
    }
#ifdef SUPL_ENABLED
    writeUbxCfgMsg(0x02, 0x12);     // enable UBX-RXM-MEAS message for SUPL MS-Assist
#endif

    // Clear all actions that are still
    // active or scheduled for some reason
    if(m_Db.pAgnssIf)
    {
        m_Db.pAgnssIf->clearActions();
    }
    m_consFailedTransf=0;

    // If data is available already
    // retransfer it to the receiver
    if(m_Db.pAgnssIf)
    {
        if(m_Db.pAgnssIf->hasData(CAgnss::RECV_AID_STATE))
        {
            setTransferRequired(CAgnss::RECV_AID_STATE, true);
        }
        else
        {
            setTransferRequired(CAgnss::RECV_AID_STATE, false);
        }
    }
    if(isOfflineEnabled())
    {
        if(m_Db.pAgnssIf->hasData(CAgnss::OFFLINE))
        {
            setTransferRequired(CAgnss::OFFLINE, true);
        }
        else
        {
            setTransferRequired(CAgnss::OFFLINE, false);
        }
    }
    if(isOnlineEnabled())
    {
        if(m_Db.pAgnssIf->hasData(CAgnss::ONLINE))
        {
            setTransferRequired(CAgnss::ONLINE, true);
        }
        else
        {
            setTransferRequired(CAgnss::ONLINE, false);
        }
    }

    // Write current location and time to the receiver
    sendPosAndTime();
}

//! Prepare turning the receiver off
/*! Exectute the sequence needed to turn the receiver off, before actually
    turning it off.
*/
void CUbxGpsState::prepareTurnOff()
{
    // Reset the timeout counter. onNewMsg
    // will force the receiver in to RECV_STATE_STOPPED
    // mode after the timeout has been reached while
    // still being in this mode
    m_pollRecvStateFinished=false;
    m_timeoutStart=getMonotonicMsCounter();
    if(m_Db.pAgnssIf && m_agnssPersistence)
    {
        // Make sure saving the receiver state
        // is executed right away
        if(m_Db.pAgnssIf)
        {
            m_Db.pAgnssIf->clearActions();
        }
        // Get current state of the receiver
        m_Db.pAgnssIf->scheduleAction(CAgnss::RECV_AID_STATE, CAgnss::POLL_RECV);
    }
}

//! Force the receiver to turn off
/*! This function finalises the shut down procedure of
    the receiver and turns it off and then powers it
    off (if implemented by the customer, see \ref
    powerOff).
*/
void CUbxGpsState::forceTurnOff()
{
    // Send stop gps msg
    writeUbxCfgRst(0x08/*gps stop*/, 0);
    usleep(100000);

    // Remove power from receiver
    UBX_LOG(LCAT_VERBOSE, "The receiver output is disabled!");
    powerOff();
    UBX_LOG(LCAT_VERBOSE, "The receiver is powered off!");
    LOGGPS(0x00000001, "#gps stop");
}

/*******************************************************************************
 * TRANSFER REQUIRED HANDLING
 *******************************************************************************/

//! Set or unset the flag which triggers a transfer for the specified service
/*! For the provided service the transfer-required flag will be set to
    the desired value.

    \param service         : The service for which the transfer-required flag
                             must be adjusted
    \param required        : Set to true if a transfer for the provided service
                             is required or set false, if this is not the case
*/
void CUbxGpsState::setTransferRequired( CAgnss::SERVICE_t service
                                      , bool required)
{
    if(service < CAgnss::_NUM_SERVICES_)
        m_transferRequired[service]=required;
}

//! Check if a transfer is possible and required for the provided service
/*! Returns if it is possible and required to transfer data for the
    specified service type.

    \param service         : The service for which it has to be checked
                             if a transfer is possible and required
    \return                  true if a transfer is possible and required,
                             false otherwise
*/
bool CUbxGpsState::isTransferPossibleAndRequired(CAgnss::SERVICE_t service)
{
    bool transferPossible=false;
    if( m_Db.pAgnssIf
     && service < CAgnss::_NUM_SERVICES_ )
    {
        if( ( service==CAgnss::TIME
         &&   m_Db.pAgnssIf->hasValidTime() )
         || ( service!=CAgnss::TIME
         &&   m_Db.pAgnssIf->hasData(service) ) )
        {
            transferPossible=m_transferRequired[service];
        }
    }
    return transferPossible;
}

/*******************************************************************************
 * CURRENT CHECKSUM HANDLING
 *******************************************************************************/

//! Set the current checksum for the specified service
/*! With this function the checksum of the last succesful transfer
    can be set for the specified service

    \param service         : The service for which the checksum
                             must be adjusted
    \param checksum        : The checksum of the last succesful
                             transfer for the provided service
*/
void CUbxGpsState::setCurrentChecksum( CAgnss::SERVICE_t service
                                     , uint16_t checksum )
{
    if(service < CAgnss::_NUM_SERVICES_)
        m_currentChecksum[service]=checksum;
}

//! Verifies if the provided checksum is the current checksum for this service
/*! Checks if the provided checksum matches the checksum of the last succesful
    transfer for the specified service.

    \param service         : Service for which the checksum shall be checked
    \param checksum        : The checksum which should be checked for
                             representing the last succesful transfer.
*/
bool CUbxGpsState::isCurrentChecksum( CAgnss::SERVICE_t service
                                    , uint16_t checksum )
{
    bool result=false;
    if(service < CAgnss::_NUM_SERVICES_)
        result=(m_currentChecksum[service]==checksum);

    return result;
}

/*******************************************************************************
 * DB CHANGED HANDLING
 *******************************************************************************/

//! Set the database-has-changed-flag to the provided value
/*! With this function the database-has-changed-flag can be either set to true
    or false.

    \param changed         : Set to true to indicate the database has changed
                             or false to indicate that the databse has not
                             changed
*/
void CUbxGpsState::setDbChanged(bool changed)
{
    m_dbChanged=changed;
}

//! Get the database-has-changed-flag
/*! With this function the database-has-changed-flag can be checked.

    \return                  Represents the current value of the
                             database-has-changed-flag
*/
bool CUbxGpsState::hasDbChanged()
{
    return m_dbChanged;
}

/*******************************************************************************
 * MEASUREMENT RATE CONFIGURATION
 *******************************************************************************/

//! set the mesurement rate
/*! put the requested measurement rate into our local configuration db,
    the gps receiver can be switched of while we a receiveing such a request

    \param rateMs the mesaurement requested configuration
*/
void CUbxGpsState::putRate(int16_t rateMs)
{
    UBX_LOG(LCAT_VERBOSE, "Put Rate rate=%d", rateMs);
    m_Db.rateMs = rateMs;
    setDbChanged(true);
    if(isRecv(ON))
    {
        writeUbxCfgRate();
    }
}

//! send measurement rate to the receiver
/*! configure the measurement rate of the receiver

    \return         true if sucessfull, false otherwise
*/
bool CUbxGpsState::writeUbxCfgRate()
{
    // Send CFG-RATE command
    GPS_UBX_CFG_RATE_t Payload;
    Payload.measRate = (U2) m_Db.rateMs;
    Payload.navRate = 1;
    Payload.timeRef = 1;
    UBX_LOG(LCAT_VERBOSE, "Send CFG-RATE rate=%d", Payload.measRate);
    return writeUbx(0x06, 0x08, &Payload, sizeof(Payload));
}

//! Initiate Online/Offline data download if required/possible
/*! Will initiate download of Online and Offline data if possible and
    required
*/
void CUbxGpsState::initDownloadIfReq()
{
    // Warning: The receiver might be turned off
    // after / shortly before the action has been
    // scheduled, but before it finishes. In this
    // case CAgnss should make sure the transfer
    // times out
    int64_t now=getMonotonicMsCounter();
    // Is networking available?
    // Warning: The network connection might be
    // turned off after / shortly before the
    // action has been scheduled, but before it
    // finishes. In this case CAgnss should make
    // sure the download times out
    CRilIf *ril_if=CRilIf::getInstance();
    if(ril_if && ril_if->isConnected())
    {
        // Network connection exists
        if(isOfflineEnabled())
        {
            // If the time since the last request for
            // Downloading data has happened more than a
            // specified timeout ago. Because the framework
            // will try to download and inject data at startup
            // the offline part will, contrary to online, not
            // download new data if there has been no download
            // yet at all
            if (m_timeLastOfflineReq && (now - m_timeLastOfflineReq) >= m_agnssOfflineInterval)
            {
                UBX_LOG(LCAT_VERBOSE, "Initiated download of AssistNow Offline data");
                CXtraIf::requestDownload();
                m_timeLastOfflineReq = now;
            }
        }
        if(isOnlineEnabled())
        {
            // If the time since the last request for
            // Downloading data has happened more than a
            // specified timeout ago or not at all yet,
            // fetch new data
            if(!m_timeLastOnlineReq || (now - m_timeLastOnlineReq) >= m_agnssOnlineInterval)
            {
                // such as done in checkAgpsDownload before
                if(m_Db.pAgnssIf->scheduleAction(CAgnss::ONLINE, CAgnss::DOWNLOAD))
                {
                    UBX_LOG(LCAT_VERBOSE, "Initiated download of AssistNow Online data");
                    m_timeLastOnlineReq=now;
                }
                else
                {
                    UBX_LOG(LCAT_WARNING, "Could not initiate download of AssistNow Online data. (Too many requests pending already?)");
                }
            }
        }
    }
}

//! Initiate AGNSS data transfer if it is required/possible
/*! Will initiate data transfer if possible and required
*/
void CUbxGpsState::initTransferIfReq()
{
    // Warning: The receiver might be turned off
    // after / shortly before the action has been
    // scheduled, but before it finishes. In this
    // case CAgnss should make sure the transfer
    // times out
    for(int i=0; i < (int) CAgnss::_NUM_SERVICES_; ++i)
    {
        CAgnss::SERVICE_t service=(CAgnss::SERVICE_t)i;
        if(isTransferPossibleAndRequired(service))
        {
            if(m_Db.pAgnssIf->scheduleAction(service, CAgnss::TRANSFER))
            {
                UBX_LOG(LCAT_VERBOSE, "Initiated transfer of %s data", agnssServiceTypeToString(service));
                setTransferRequired(service, false);
            }
            else
            {
                UBX_LOG(LCAT_WARNING, "Could not initiate transfer of %s. (Too many requests pending already?)",agnssServiceTypeToString(service));
            }
        }
    }
}

//! Set the baud rate for the receiver and the host's serial port
/*! Sets the baud rate of the host to the default baud rate of the
    receiver, configures the receiver to the new baud rate and then
    changes the baud rate of the host to the same.
*/
void CUbxGpsState::setBaudRate()
{
    if (m_pSer == NULL)
    {
        UBX_LOG(LCAT_WARNING, "invalid serial port handler");
        return;
    }
    int baudRate = getBaudRate();
    if (baudRate != getBaudRateDefault())
    {
        // reconfigure the baud rate
        usleep(100000);
        // Set baud rate of the Linux serial port to
        // the default-baud-rate
        m_pSer->setbaudrate(getBaudRateDefault());
        usleep(100000);
        // Configure the receiver to work with the
        // operation-baud-rate
        writeUbxCfgPort(1 /* UART1 */, baudRate);
        usleep(100000);  // 1/10 second
        // Configure the Linux serial port to
        // use the default-baud-rate as well
        m_pSer->setbaudrate(baudRate);
    }
    // Both, receiver and Linux serial port, should
    // now be configured for the same baud rate
}

//! handle new messages from the receiver
/*! This is the main routine which is called whenever a new UBX message was received in the parser.

    \param pMsg     the pointer to the complete message (includes frameing and payload)
    \param iMsg     the size of the complete message (includes frameing and payload)
*/
void CUbxGpsState::onNewMsg(const unsigned char* pMsg, unsigned int iMsg)
{
    // Check if new (state-transition-independent) actions must be initialised
    switch(m_currState)
    {
        case OFF:
        {
            if(m_inTransition)
            {
                // This object should be able to transition
                // to the RECV_STATE_STOPPED mode itself, but if something
                // goes wrong it will be forced into the RECV_STATE_STOPPED
                // mode after a timeout here
                if(m_pollRecvStateFinished)
                {
                    changeState(OFF, false);
                }
                else if((getMonotonicMsCounter()-m_timeoutStart>m_stoppingTimeoutMs))
                {
                    UBX_LOG(LCAT_WARNING, "Time for preparing the shutdown run out. Stopping the receiver...");
                    changeState(OFF, false);
                }
            }
            break;
        }
        case ON:
        default:
        {
            if(m_inTransition) // End transition to ON
            {
                changeState(ON, false);
            }
            else
            {
                // Do what is required to be done if the
                // receiver is turned on
                initDownloadIfReq();
                initTransferIfReq();
            }
            break;
        }
    }

    // In all states, pass the data to the AGNSS interface
    if(m_Db.pAgnssIf)
    {
        m_Db.pAgnssIf->processMsg(pMsg, iMsg);
    }

    // If AOP is available
    if (isAopEnabled() && pMsg && iMsg > 4)
    {
        unsigned char clsId;
        unsigned char msgId;

        clsId = pMsg[2];
        msgId = pMsg[3];

        if ((clsId == 0x0B) && (iMsg >= 9))
        {
            unsigned int svix = pMsg[6] - 1;
            if ((0x33 == msgId) && (iMsg > 9) && (svix < NUM_GPS_SVS))
            {
                // AssistNow Offline parameters
                if (replaceBuf(&m_Db.aopG[svix], pMsg, iMsg))
                {
                    UBX_LOG(LCAT_VERBOSE, "Received AOP from receiver for G%d size %d", svix+1, iMsg);
                    setDbChanged(true);
                }
            }
        }
    }

    if(hasDbChanged())
    {
        saveAiding();
        setDbChanged(false);
    }
}

//! Enable AOP in the receiver
/*! Send a UBX-CFG-NAVX5 message to the receiver to enable the AssistNow Autonomous Feature.

    \return         true if successful, false otherwise
*/
bool CUbxGpsState::writeUbxNavX5AopEnable()
{
    GPS_UBX_CFG_NAV5X_t Payload;
    memset(&Payload, 0, sizeof(Payload));
    Payload.mask1 = 0x4000; // config AOP
    Payload.aopFlags = 0x01; // enable AOP
    Payload.aopOrbMaxError = 0; // use default

    return writeUbx(0x06, 0x23, &Payload, sizeof(Payload));
}

/*******************************************************************************
 * LOCAL AIDING: TIME POSITION
 *******************************************************************************/

//! Helper function to return a reference time
/*! this function has to be used to calculate aging of of the NTP (inject_time)
    return the current refrence time.
*/
int64_t CUbxGpsState::currentRefTimeMs(void)
{
    return getMonotonicMsCounter();
}

//! put time information into the local database.
/*! Put the time information received from a SNTP request into the local database.
    This data is then used by \ref CAgnss to achieve time transfer from the
    network to the device.

    \param timeUtcMs          : The current utc time in milliseconds as define
                                in gps.h
    \param timeRefMs          : the reference time in milliseconds of the SNTP
                                request use together with #currentRefTimeMs to
                                age the timeUtcMs
    \param accMs              : The time accuracy in milliseconds of this
                                information (half the rount trip time).
*/
void CUbxGpsState::putTime(GpsUtcTime timeUtcMs, int64_t timeRefMs, int accMs)
{
    UBX_LOG(LCAT_VERBOSE, "NTP time injected (UTC)");
    if(m_Db.pAgnssIf)
    {
        timeUtcMs+=currentRefTimeMs()-timeRefMs;
        ACCTIME_t accTime;
        accTime.valid=true;
        accTime.leapSeconds=true;
        accTime.time.tv_sec=timeUtcMs/1000;
        accTime.time.tv_nsec=(timeUtcMs%1000)*1000000LL;
        accTime.acc.tv_sec=accMs/1000;
        accTime.acc.tv_nsec=(accMs%1000)*1000000LL;
        m_Db.pAgnssIf->setCurrentTime(&accTime);
    }
}

//! put the position information into the local database
/*! Put the position information received from Android into the local database.
    This data is then used by \ref CAgnss to achieve position aiding from the
    network. The function also recored the time of reception so that it could
    be aged.

    \param latDeg             : the Latitude in Degrees
    \param lonDeg             : the Longitude in Degrees
    \param accM               : the accuracy in meters of this information
*/
void CUbxGpsState::putPos(double latDeg, double lonDeg, float accM)
{
    if(m_Db.pAgnssIf)
    {
        UBX_LOG(LCAT_VERBOSE, "Retrieved the following position as the current from the framework: Lat %.6f Lon %.6f Acc %f", latDeg, lonDeg, (double)accM);
        POS_t pos;
        memset(&pos, 0, sizeof(pos));
        pos.latDeg     = (I4) ((double) latDeg * 1e7);
        pos.lonDeg     = (I4) ((double) lonDeg * 1e7);
        pos.posAccCm   = (U4) ((float ) accM   * 100);
        pos.valid      = true;

        m_Db.pAgnssIf->setCurrentPosition(&pos);
        setDbChanged(true);
    }
}

//! Set the flags for initialising a position and/or time transfer.
/*! If the system possesses an accurate time source it will send
    time information to the receiver. If additionally position
    information is available, it will be sent as well.

    \return                  true on success, false otherwise
*/
bool CUbxGpsState::sendPosAndTime()
{
    bool result=false;
    if(m_Db.pAgnssIf && m_Db.pAgnssIf->hasValidTime())
    {
        result=true;
        // Can position, which includes a time transfer, be used instead?
        if(m_Db.pAgnssIf->hasValidPosition())
        {
            setTransferRequired(CAgnss::POSITION, result);
        }
        else
        {
            setTransferRequired(CAgnss::TIME, result);
        }
    }
    else
    {
        // Position and time can not be transferred, make sure it is not tried
        setTransferRequired(CAgnss::TIME, false);
        setTransferRequired(CAgnss::TIME, false);
    }
    return result;
}

/*******************************************************************************
 * ASSISTNOW OFFLINE / ALMANAC PLUS
 *******************************************************************************/
//! put the Assistnow Offline data into the local database
/*! Put the Assistnow Offline into the local database and send data if required

    \param pData The pointer to the new offline data
    \param iData The size of the new offline data
    \return true if sucessfull, false otherwise
*/
bool CUbxGpsState::putNewOfflineData(const unsigned char* pData, unsigned int iData)
{
    bool result=false;
    if(isOfflineEnabled())
    {
        uint16_t checksum=0;
        if(m_Db.pAgnssIf->saveToDb(CAgnss::OFFLINE, pData, iData, &checksum))
        {
            // If the data is different compared to the last
            // transfer, make sure a new transfer is initiated
            if(!isCurrentChecksum(CAgnss::OFFLINE, checksum))
            {
                UBX_LOG(LCAT_VERBOSE, "New Offline data injected. Scheduling transfer.");
                setTransferRequired(CAgnss::OFFLINE, true);
            }
            else
            {
                UBX_LOG(LCAT_VERBOSE, "Downloaded Offline data identical to already transferred one. Do nothing.");
            }
        }
        else
        {
            UBX_LOG(LCAT_ERROR, "Could not save Offline data to database, because the data is invalid");
        }
    }
    else
    {
        UBX_LOG(LCAT_WARNING, "Offline data is not enabled. Could not inject Offline data");
    }
    return result;
}

/*******************************************************************************
 * UBX PROTOCOL: OTHERS
 *******************************************************************************/

//! Translates a UBX class- and message-id to the name of the message
/*! Translates a UBX class- and message-id to the name of the message. Only
    selecte messages are available for translation.

    \param clsId           : The class id of the UBX message for which
                             the name should be returned
    \param msgId           : The messsage id of the UBX message for which
                             the name should be returned
    \return                  The name of the message for which arguments were
                             provided. If the message is not known, the
                             string "UNKNOWN MESSAGE" will be returned
*/
char const * CUbxGpsState::strUbxCfgMsg(unsigned char clsId, unsigned char msgId)
{
    uint16_t id = clsId << 8 | msgId;

    switch(id)
    {
        case 0x0103: return "UBX-NAV-STATUS";
        case 0x0212: return "UBX-RXM-MEAS";
        case 0x0B33: return "UBX-AID-AOP";
        case 0xF000: return "NMEA-GGA";
        case 0xF001: return "NMEA-GLL";
        case 0xF002: return "NMEA-GSA";
        case 0xF003: return "NMEA-GSV";
        case 0xF004: return "NMEA-RMC";
        case 0xF005: return "NMEA-VTG";
        case 0xF006: return "NMEA-GRS";
        case 0xF007: return "NMEA-GST";
        case 0xF008: return "NMEA-ZDA";
        case 0xF009: return "NMEA-GBS";
        case 0xF00A: return "NMEA-DTM";
        case 0xF00D: return "NMEA-GNS";
        case 0xF040: return "NMEA-GPQ";
        default: return "UNKNOWN MESSAGE";
    }
}

//! Enable UBX or NMEA message on the current port.
/*! write a UBX-CFGiMSG message to the receiver

    \param clsId    the class Id of the message to be enabled
    \param msgId    the message Id of the message to be enabled
    \return         true if sucessfull, false otherwise
*/
bool CUbxGpsState::writeUbxCfgMsg(unsigned char clsId, unsigned char msgId)
{
    GPS_UBX_CFG_MSG_SETCURRENT_t Payload;
    memset(&Payload, 0, sizeof(Payload));
    Payload.classType   = clsId;
    Payload.msgID       = msgId;
    Payload.rate        = 1;
    bool result = writeUbx(0x06, 0x01, &Payload, sizeof(Payload));

    if(result)
    {
        UBX_LOG(LCAT_VERBOSE, "Sent CFG-MSG id=%02X-%02X (enable %s)", clsId, msgId, strUbxCfgMsg(clsId, msgId));
    }
    else
    {
        UBX_LOG(LCAT_WARNING, "Sending CFG-MSG id=%02X-%02X (enabling %s failed!)", clsId, msgId, strUbxCfgMsg(clsId, msgId));
    }

    return result;
}

//! Change the Baudrate of a port.
/*! write a UBX-CFG-PORT message to the receiver

    \param portId   the port Id to be configured (usually this is UART1)
    \param baudRate the baudrate to be configured
    \return         true if sucessfull, false otherwise
*/
bool CUbxGpsState::writeUbxCfgPort(int portId, int baudRate)
{
    GPS_UBX_CFG_PRT_UART_U5_t Payload;
    memset(&Payload, 0, sizeof(Payload));
    Payload.portID          = (unsigned char) portId;
    Payload.mode            = 0x000008D0;
    Payload.baudRate        = (unsigned int) baudRate;
    Payload.inProtoMask     = 0x0007;
    Payload.outProtoMask    = 0x0003;
    UBX_LOG(LCAT_VERBOSE, "Send CFG-Port baudrate=%d", baudRate);
    return writeUbx(0x06, 0x00, &Payload, sizeof(Payload));
}

//! Change the operating mode of the receiver or reset the receiver.
/*! write a UBX-CFG-RST message to the receiver

    \param resetMode    the resetMode to be executed (8=stop,9=start,2=gps reset)
    \param flags        the information to be cleared set to 0 if nothing needs to be cleared.
    \return             true if sucessfull, false otherwise
*/
bool CUbxGpsState::writeUbxCfgRst(int resetMode, GpsAidingData flags)
{
    unsigned short ubxFlags = 0;
    if (flags == GPS_DELETE_ALL)
        ubxFlags = 0xFFFF;
    else
    {
        if (flags & GPS_DELETE_EPHEMERIS)   ubxFlags |= GPS_UBX_CFG_RST_NAVBBRMASK_EPH_MASK;
        if (flags & GPS_DELETE_ALMANAC)     ubxFlags |= GPS_UBX_CFG_RST_NAVBBRMASK_ALM_MASK;
        if (flags & GPS_DELETE_POSITION)    ubxFlags |= GPS_UBX_CFG_RST_NAVBBRMASK_POS_MASK;
        if (flags & GPS_DELETE_TIME)        ubxFlags |= GPS_UBX_CFG_RST_NAVBBRMASK_RTC_MASK;
        if (flags & GPS_DELETE_IONO)        ubxFlags |= GPS_UBX_CFG_RST_NAVBBRMASK_KLOB_MASK;
        if (flags & GPS_DELETE_UTC)         ubxFlags |= GPS_UBX_CFG_RST_NAVBBRMASK_UTC_MASK;
        if (flags & GPS_DELETE_HEALTH)      ubxFlags |= GPS_UBX_CFG_RST_NAVBBRMASK_HEALTH_MASK;
        if (flags & GPS_DELETE_SADATA)      ubxFlags |= (1<<15) /* AOP DATA */;
    }
    // Send CFG-RST command
    GPS_UBX_CFG_RST_t Payload;
    memset(&Payload, 0, sizeof(Payload));
    Payload.resetMode  = (unsigned char) resetMode; // controlled GPS only reset
    Payload.navBbrMask = ubxFlags;
    return writeUbx(0x06, 0x04, &Payload, sizeof(Payload));
}

#ifdef SUPL_ENABLED
void CUbxGpsState::sendEph(const void* pData, int size)
{
    writeUbx(UBXID_AID_EPH >> 8, UBXID_AID_EPH & 0xFF, pData, size);
}

void CUbxGpsState::sendAidingData(const GPS_UBX_AID_INI_U5__t *pAidingData)
{
    writeUbx(UBXID_AID_INI >> 8, UBXID_AID_INI & 0xFF, const_cast<GPS_UBX_AID_INI_U5__t *>(pAidingData), sizeof(GPS_UBX_AID_INI_U5__t));
#if 0
    unsigned char *pMsg = (unsigned char *) pAidingData;
    for (unsigned int i=0; i<sizeof(GPS_UBX_AID_INI_U5__t); i++)
    {
        UBX_LOG(LCAT_VERBOSE, "%.2X", pMsg[i]);
    }
#endif
}

void CUbxGpsState::sendUtcModel(const GPS_UBX_AID_HUI_t *pUtcModel)
{
    writeUbx(UBXID_AID_HUI >> 8, UBXID_AID_HUI & 0xFF, pUtcModel, sizeof(GPS_UBX_AID_HUI_t));
}

#endif

/*******************************************************************************
 * UBX PROTOCOL HELPERS
 *******************************************************************************/

//! calculate the ubx checksum / fletcher-16 on a buffer
/*! calculate the ubx checksum / fletcher-16 on a buffer
    \param crc the checksum seed and result
    \param pData the pointer to the buffer over which to calculate the crc
    \param iData the size of the buffer over which to calculate the crc
*/
void CUbxGpsState::crcUbx(unsigned char crc[2], const unsigned char* pData, int iData)
{
    unsigned int crcA = crc[0];
    unsigned int crcB = crc[1];
    if (iData > 0)
    {
        do
        {
            crcA += *pData++;
            crcB += crcA;
        }
        while (--iData);
    }
    crc[0] = (unsigned char)crcA;
    crc[1] = (unsigned char)crcB;
}

//! Write a UBX message to the receiver
/*! Write a UBX message to the receiver with a specific class/message id and for a certain payload.

    \param classID  the class Id of the message to be sent
    \param msgID    the message Id of the message to be sent
    \param pData0   the pointer to the first part of the payload to be sent (can be NULL)
    \param iData0   the size of the first part of the payload to be sent (can be 0)
    \param pData1   the pointer to the second part of the payload to be sent (can be NULL)
    \param iData1   the size of the second part of the payload to be sent (can be 0)
    \return         true if sucessfull, false otherwise
*/
bool CUbxGpsState::writeUbx(unsigned char classID, unsigned char msgID,
                            const void* pData0, size_t iData0,
                            const void* pData1, size_t iData1)
{
    if (m_pSer == NULL || ((!pData0 && !pData1) && (iData0>0 || iData1>0)))
    {
        return false;
    }

    unsigned char *outBuf=NULL;
    ssize_t outCount=createUbx(&outBuf, classID, msgID, pData0, iData0, pData1, iData1);

    if(outCount<=0)
    {
        return false;
    }

    bool result=false;
    // Send out through the required channels
    if ( m_pSer->writeSerial(outBuf, outCount) == outCount)
    {
        result=true;
#ifdef MSG_UBX_LOG_OUTPUT
        char *pMsg;
        int res=byte_array_to_hex_string(&pMsg, outBuf, outCount);
        if(res==-1)
        {
            UBX_LOG(LCAT_ERROR, "An error occured while trying to log outgoing UBX messages (part 2/2)");
        }
        else
        {
            UBX_LOG(LCAT_VERBOSE, "Sent UBX Msg: %s (size %d)\n", pMsg, res);
            free(pMsg);
        }
#endif //ifdef MSG_UBX_LOG_OUTPUT
#if defined UDP_SERVER_PORT
        if (m_pUdpServer != NULL)
        {
            m_pUdpServer->sendPort(outBuf, outCount);
        }
#endif
    }
    free(outBuf);
    return result;
}

/*******************************************************************************
 * POWER
 *******************************************************************************/

/*! Switch the power on to the receiver.
    \warning Powering the receiver off has to be manually implemented
             in this function if this functionality is required
*/
bool CUbxGpsState::powerOn(void)
{
    // Place any code to handle switching power to the device ON
    // HERE
    // This is optional

    UBX_LOG(LCAT_VERBOSE, "placeholder for power ON function");

    return false;       // Return true if power was successfully turned on
}

/*! Switch the power off to the receiver.
    \warning Powering the receiver off has to be manually implemented
             in this function if this functionality is required
*/
void CUbxGpsState::powerOff(void)
{
    // Place any code to handle switching power to the device OFF
    // HERE
    // This is optional

    UBX_LOG(LCAT_VERBOSE, "placeholder for power OFF function");
}


//! Checks if AOP is enabled
/*! This function will return the state of the AOP feature

    \return                : true if AOP is enabled and false otherwise
*/
bool CUbxGpsState::isAopEnabled()
{
    return  (m_agnssStrategy == AGNSS_AOP_ONLY
            || m_agnssStrategy == AGNSS_AOP_AND_ONLINE);
}

//! Checks if AssistNow Offline is enabled
/*! This function will return the state of the AssistNow Offline feature

    \return                : true if AssistNow Offline is enabled and
                             false otherwise
*/
bool CUbxGpsState::isOfflineEnabled()
{
    return (   m_Db.pAgnssIf
            &&(m_agnssStrategy == AGNSS_OFFLINE_ONLY
            || m_agnssStrategy == AGNSS_OFFLINE_AND_ONLINE));
}

//! Checks if AssistNow Online is enabled
/*! This function will return the state of the AssistNow Online feature

    \return                : true if AssistNow Online is enabled and
                             false otherwise
*/
bool CUbxGpsState::isOnlineEnabled()
{
    return (   m_Db.pAgnssIf
            &&(m_agnssStrategy == AGNSS_ONLINE_ONLY
            || m_agnssStrategy == AGNSS_OFFLINE_AND_ONLINE));
}

//! This static function will log messages with the help of the passed object
/*! Print the provided information to the standard log output of the
    provided object.

    \param context         : An object of type CUbxGpsState cast to a void
                             pointer. The non-static version of \ref printStd
                             of this object will be called
    \param str             : The '\0'-terminated string that will be printed
                             to the standard log output of the provided object
    \return                  The number of printed characters on success
                             and a negative number otherwise
*/
int CUbxGpsState::printStd(void const * context, const char *str)
{
    int result=-1;
    if(context)
        result=((CUbxGpsState*) const_cast<void *>(context))->printStd(str);

    return result;
}

//! Prints to the standard log output
/*! This function will print the provided string to the standard log output
    and is called by the static version of \ref printStd

    \param str             : The '\0'-terminated string that will be printed
                             to the standard log output of the object
*/
int CUbxGpsState::printStd(const char *str)
{
    return UBX_LOG(LCAT_VERBOSE, str);
}

//! This static function will log messages with the help of the passed object
/*! Print the provided information to the error log output of the
    provided object.

    \param context         : An object of type CUbxGpsState cast to a void
                             pointer. The non-static version of \ref printErr
                             of this object will be called
    \param str             : The '\0'-terminated string that will be printed
                             to the error log output of the provided object
    \return                  The number of printed characters on success
                             and a negative number otherwise
*/
int CUbxGpsState::printErr(void const * context, const char *str)
{
    int result=-1;
    if(context)
        result=((CUbxGpsState*) const_cast<void *>(context))->printErr(str);

    return result;
}

//! Prints to the standard log output
/*! This function will print the provided string to the error log output
    and is called by the static version of \ref printErr

    \param str             : The '\0'-terminated string that will be printed
                             to the error log output of the object
*/
int CUbxGpsState::printErr(const char *str)
{
    return UBX_LOG(LCAT_ERROR, str);
}

//! This static function will write to the receiver with the passed object
/*! Write data to the receiver with the object provided

    \param context         : An object of type CUbxGpsState cast to a void
                             pointer. The non-static version of \ref writeToRcv
                             of this object will be called
    \param buf             : Data to be transferred to the receiver
    \param size            : Number of characters in buf
    \return                  On success the number of bytes written to the
                             receiver. Otherwise a negative value
*/
ssize_t CUbxGpsState::writeToRcv( void const * context
                                , unsigned char const *buf
                                , size_t size)
{
    ssize_t result=-1;
    if(context)
        result=((CUbxGpsState*) const_cast<void *>(context))->writeToRcv(buf, size);

    return result;
}

//! Writes to the receiver
/*! Write the provided data to the receiver

    \param buf             : Data to be transferred to the receiver
    \param size            : Number of bytes in buf
    \return                  On success the number of characters written to the
                             receiver. Otherwise a negative value
*/
ssize_t CUbxGpsState::writeToRcv(unsigned char const *buf, size_t size)
{
    if(!m_pSer)
    {
        return -1;
    }
    // This will (usually) be executed by a separate thread.
    // The kernel should handle the locking of the write buffer
    // for the file descriptor - in one single write call!
    return m_pSer->writeSerial(buf, size);
}

//! This static function will be called if an \ref CAgnss action has finished
/*! This static function will be called if an \ref CAgnss action has finished
    and will then pass the data to the object provided

    \param context         : An object of type CUbxGpsState cast to a void
                             pointer. The non-static version of \ref finished
                             of this object will be called
    \param service         : The service for which an action has finished
    \param action          : The action which has finished
    \param success         : If the action has finished successfully
    \param dataId          : The id of the data handled by the action
*/
void CUbxGpsState::finished( void const * context
                           , CAgnss::SERVICE_t service
                           , CAgnss::ACTION_t action
                           , bool success
                           , uint16_t dataId)
{
    if(context)
    {
        ((CUbxGpsState*) const_cast<void *>(context))->lock();
        ((CUbxGpsState*) const_cast<void *>(context))->finished(service, action, success, dataId);
        ((CUbxGpsState*) const_cast<void *>(context))->unlock();
    }
}

//! This function will be called if an \ref CAgnss action has finished
/*! This function will be called by the static version of \ref finished
    if an \ref CAgnss action has finished

    \param service         : The service for which an action has finished
    \param action          : The action which has finished
    \param success         : If the action has finished successfully
    \param dataId          : The id of the data handled by the action
*/
void CUbxGpsState::finished( CAgnss::SERVICE_t service
                           , CAgnss::ACTION_t action
                           , bool success
                           , uint16_t dataId)
{
    if(success)
    {
        switch( action )
        {
            case CAgnss::POLL_RECV:
            {
                if( service == CAgnss::RECV_AID_STATE)
                {
                    m_pollRecvStateFinished=true;
                }
                // NO BREAK!
            }
            case CAgnss::DOWNLOAD:
            {
                UBX_LOG(LCAT_VERBOSE, "%s of %s has been successful!"
                                    , (agnssActionToString(action))
                                    , (agnssServiceTypeToString(service)));

                // If the data is different compared to the last
                // transfer, make sure a new transfer is initiated
                if(!isCurrentChecksum(service, dataId))
                {
                    // Data needs to be saved again
                    if( service==CAgnss::OFFLINE
                     || service==CAgnss::RECV_AID_STATE )
                    {
                        // Save data if required
                        setDbChanged(true);
                    }
                    // Never initiate a transfer after having
                    // obtained the receiver state (the receiver
                    // already has that data)
                    if(service!=CAgnss::RECV_AID_STATE)
                    {
                        UBX_LOG( LCAT_VERBOSE
                               , "New %s data injected by %s"
                                 " (Old data id: %04X New data id: %04X)."
                                 " Scheduling transfer."
                               , agnssServiceTypeToString(service)
                               , agnssActionToString(action)
                               , m_currentChecksum[service]
                               , dataId
                               );
                        setTransferRequired(service, true);
                    }
                }
                else
                {
                    UBX_LOG( LCAT_VERBOSE
                           , "%sed %s data identical to already transferred"
                             " one (%04X). Do nothing."
                           , agnssActionToString(action)
                           , agnssServiceTypeToString(service)
                           , dataId
                           );
                }
                break;
            }
            case CAgnss::TRANSFER:
            {
                UBX_LOG(LCAT_VERBOSE, "Transfer of %s data has been successful!"
                                    , (agnssServiceTypeToString(service)));
                // The transfer is not required anymore
                setCurrentChecksum(service, dataId);
                // Transfer successful, start recounting consecutive fails
                m_consFailedTransf=0;
                break;
            }
            default:
            {
                break;
            }
        }
    }
    else // failed
    {
        UBX_LOG(LCAT_ERROR, "%s of %s data failed!"
                          , (agnssActionToString(action))
                          , (agnssServiceTypeToString(service)));
        if(action==CAgnss::TRANSFER)
        {
            if( m_consFailedTransf<m_maxConsFailedTransf )
            {
                ++m_consFailedTransf;
                UBX_LOG(LCAT_VERBOSE, "Retry transfer. Maximum of consecutive failures while trying to transfer not yet reached. [Try %u/%u]", m_consFailedTransf, m_maxConsFailedTransf);
                setTransferRequired(service, true);
            }
            else
            {
                m_consFailedTransf=0;
                UBX_LOG(LCAT_ERROR, "Maximum consecutive failures while trying to transfer reached (%u). Transfer will not be rescheduled",m_maxConsFailedTransf);
            }
        }
    }
}

//! Send AOP data to the receiver if AOP is enabled
/*! This function will check if AOP is available and if so transfer the
    stored data to the receiver
*/
void CUbxGpsState::sendAopIfRequired()
{
    if (isAopEnabled())
    {
        for (int svix = 0; svix < NUM_GPS_SVS; svix ++)
        {
            BUF_t* p = &m_Db.aopG[svix];
            if ((p->p != NULL) && (p->i>0))
            {
                if ((m_pSer != NULL) && (m_pSer->writeSerial(p->p,p->i) == (int)p->i))
                {
                    UBX_LOG(LCAT_VERBOSE, "Sent AOP G%d size %d", svix+1, p->i);
                }
#if defined UDP_SERVER_PORT
                if (m_pUdpServer != NULL)
                {
                    m_pUdpServer->sendPort(p->p,(int) p->i);
                }
#endif
            }
        }
    }
}


//! Saves aiding data to a temporary file.
/*! Saves the current aiding data to a temporary file, ready to be loaded back into
    the receiver the next time it is started.
*/
void CUbxGpsState::saveAiding(void)
{
    if (m_agnssPersistence)
    {
        UBX_LOG(LCAT_VERBOSE, "Saving aiding data to %s", m_pAgnssPersistenceFile);

        mode_t mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH;
        int fd = open(m_pAgnssPersistenceFile, O_CREAT | O_WRONLY | O_TRUNC,mode);

        if (fd == -1)
        {
            // failed
            UBX_LOG(LCAT_VERBOSE, "Can not save aiding data to %s : %s", m_pAgnssPersistenceFile, strerror(errno));
            return;
        }

        // Save the database
        if (saveDatabase(fd, &m_Db))
        {
            // Success
            close(fd);
            //chmod(m_pAgnssPersistenceFile, S_IRUSR | S_IWUSR);
            UBX_LOG(LCAT_VERBOSE, "Aiding data saved succesfully");
        }
        else
        {
            // Failed
            close(fd);
            remove(m_pAgnssPersistenceFile);
            UBX_LOG(LCAT_VERBOSE, "Aiding data save failed - temp file removed");
        }
    }
    else
    {
        UBX_LOG(LCAT_VERBOSE, "Not saved - DB not changed");
    }
}

//! Loads the temporary stored aiding data
/*! Loads the temporay file containing the last know aiding data, and transfers
    it to the receiver.

    \return true if all aiding data sucessfully loaded, false otherwise
*/
bool CUbxGpsState::loadAiding(void)
{
    if (m_agnssPersistence)
    {
        UBX_LOG(LCAT_VERBOSE, "Loading last aiding data from %s", m_pAgnssPersistenceFile);

        int fd = open(m_pAgnssPersistenceFile, O_RDONLY);

        if (fd == -1)
        {
            // Failed
            UBX_LOG(LCAT_VERBOSE, "Can not load aiding data from %s : %i", m_pAgnssPersistenceFile, errno);
            return false;
        }

        if (loadDatabase(fd, &m_Db))
        {
            UBX_LOG(LCAT_VERBOSE, "Aiding data loaded successfully");
        }
        else
        {
            UBX_LOG(LCAT_VERBOSE, "Failed to load aiding data");
        }

        close(fd);
    }
    else
    {
        UBX_LOG(LCAT_VERBOSE, "Not loaded - DB not cleared");
    }


    return true;
}

//! Write data to a file into a DB_s structure
/*! The DB_s structure is written one field at a time.
    The whole file has the structure according to what
    is described in encodeAgnssf00. The blocks
    stored in this format are written are as follows:
    -----------------------------------------------------------------------
    | DB Version:             [uint8_t]
    | Measurement rate:       [uint16_t]
    | Position:               [STAMPED_POS_t]
    | Reserved0:              EMPTY
    | Reserved1:              EMPTY
    | Reserved2:              EMPTY
    | MGA Recv state:         [UBX messages]
    | LEG Recv state:         [UBX messages]
    | Reserved3:              EMPTY
    | Reserved4:              EMPTY
    | MGA Offline:            [UBX messages]
    | LEG Offline:            [ALP data]
    | Reserved5:              EMPTY
    | Reserved6:              EMPTY
    | Reserved7:              EMPTY
    | LEG AOP:                [UBX messages encoded as Agnssf00]
    -----------------------------------------------------------------------
    If any of these parameters is not used it will
    be written as an empty block (in the index with value 0).

    \param fd       : the file handle of the file to write too
    \param pDb      : a pointer to the structure to write

    \return           true if all fields were written, false otherwise
*/
bool CUbxGpsState::saveDatabase(int fd, struct DB_s* pDb)
{
    unsigned char *raw=NULL;
    ssize_t rawSize=0;
    unsigned char *aopRaw=NULL;
    ssize_t aopRawSize=0;
    unsigned char * offlineData=NULL;
    ssize_t offlineDataSize=0;
    unsigned char * receiverStateData=NULL;
    ssize_t receiverStateDataSize=0;
    unsigned char * positionData=NULL;
    ssize_t positionDataSize=0;

    BUF_t data[DB__FIELDCOUNT__];
    memset(data, 0, sizeof(data));

    uint8_t dbVersion=DB_VERSION_NO;
    data[DB_VERSION].p=(unsigned char *)&dbVersion;
    data[DB_VERSION].i=sizeof(dbVersion);

    UBX_LOG(LCAT_VERBOSE, "Encoded Measurement rate!");
    data[DB_MEAS_RATE].p=(unsigned char *)&(pDb->rateMs);
    data[DB_MEAS_RATE].i=sizeof(&pDb->rateMs);

    if(m_Db.pAgnssIf)
    {
        // Save Position
        positionDataSize=m_Db.pAgnssIf->loadFromDb(CAgnss::POSITION, &positionData);
        if(positionDataSize > 0)
        {
            data[DB_POSITION].p=positionData;
            data[DB_POSITION].i=positionDataSize;
            UBX_LOG(LCAT_VERBOSE, "Encoded Position information!");
        }

        // Save Receiver Aiding State
        receiverStateDataSize=m_Db.pAgnssIf->loadFromDb(CAgnss::RECV_AID_STATE, &receiverStateData);
        if(receiverStateDataSize > 0)
        {
            int db_receiverState=DB_MGA_RECV_STATE;
            if(getReceiverGeneration()>=8)
            {
                UBX_LOG(LCAT_VERBOSE, "Encoded MGA Receiver Aiding State data!");
            }
            else
            {
                db_receiverState=DB_LEG_RECV_STATE;
                UBX_LOG(LCAT_VERBOSE, "Encoded LEG Receiver Aiding State data!");
            }
            data[db_receiverState].p=receiverStateData;
            data[db_receiverState].i=receiverStateDataSize;
        }

        // Save Offline Data
        if(isOfflineEnabled())
        {
            offlineDataSize=pDb->pAgnssIf->loadFromDb(CAgnss::OFFLINE, &offlineData);
            if(offlineDataSize > 0)
            {
                int db_offline=DB_MGA_OFFLINE;
                if(getReceiverGeneration()>=8)
                {
                    UBX_LOG(LCAT_VERBOSE, "Encoded MGA Offline data!");
                }
                else
                {
                    db_offline=DB_LEG_OFFLINE;
                    UBX_LOG(LCAT_VERBOSE, "Encoded LEG Offline data!");
                }
                data[db_offline].p=offlineData;
                data[db_offline].i=offlineDataSize;
            }
        }
    }

    if(isAopEnabled())
    {
        aopRawSize=encodeAgnssf00(&aopRaw, pDb->aopG, NUM_GPS_SVS);
        if(aopRawSize > 0)
        {
            UBX_LOG(LCAT_VERBOSE, "Encoded LEG AOP data!");
            data[DB_LEG_AOP].p=aopRaw;
            data[DB_LEG_AOP].i=aopRawSize;
        }
    }

    bool result=false;
    UBX_LOG(LCAT_VERBOSE, "Create file structure for encoded data!");
    rawSize=encodeAgnssf00(&raw, data, DB__FIELDCOUNT__);
    if(rawSize > 0)
    {
        if(write(fd, raw, rawSize)==rawSize)
        {
            result=true;
        }
    }
    else
    {
        UBX_LOG(LCAT_ERROR, "Creating file structure for encoded data failed! %i", rawSize);
    }

    // Clean up
    free(positionData);
    free(receiverStateData);
    free(offlineData);
    free(aopRaw);
    free(raw);
    return result;
}

//! Reads data from a file into a DB_s structure
/*! The DB_s structure is populated one field at a time.
    To populate each field, a corresponding 'loadBuffer' function is called for handling that field type.

    \param fd the file handle of the file to read
    \param pDb a pointer to the structure to populate
    \return true if all fields were populated, false otherwise
*/
bool CUbxGpsState::loadDatabase(int fd, struct DB_s* pDb)
{
    bool result=false;

    char sys[4]="tst";
    int db_offline=DB_MGA_OFFLINE;
    int db_receiverState=DB_MGA_RECV_STATE;
    if(getReceiverGeneration()>=8)
    {
        strncpy(sys, "MGA", sizeof(sys));
    }
    else
    {
        db_offline=DB_LEG_OFFLINE;
        db_receiverState=DB_LEG_RECV_STATE;
        strncpy(sys, "LEG", sizeof(sys));
    }

    struct stat s;
    if(fstat(fd,  &s))
    {
        UBX_LOG(LCAT_WARNING, "Could not find the persistence file!");
        return false;
    }

    // Don't read more than 1 MiB
    if(s.st_size > 1024*1024)
    {
        UBX_LOG(LCAT_WARNING, "Persistence file too big!");
        return false;
    }

    unsigned char * raw=(unsigned char *)malloc(s.st_size);
    if(!raw)
    {
        UBX_LOG(LCAT_WARNING, "Could not allocate memory to decode the file contents!");
        return false;
    }

    // Read the whole file
    if(read(fd, raw, s.st_size)==s.st_size)
    {
        UBX_LOG(LCAT_VERBOSE, "Interpret file structure for encoded data!");
        BUF_t data[DB__FIELDCOUNT__];
        int8_t bufSize=decodeAgnssf00(raw, s.st_size, data, DB__FIELDCOUNT__);
        if(bufSize<=0)
        {
            UBX_LOG(LCAT_ERROR, "Interpreting file structure for encoded data failed! %i", bufSize);
        }
        else
        {
            if( data[DB_VERSION].i!=sizeof(DB_VERSION_NO)
             || memcmp(&DB_VERSION_NO, data[DB_VERSION].p, sizeof(DB_VERSION_NO)) )
            {
                UBX_LOG(LCAT_ERROR, "Version number of file structure not known!");
            }
            else
            {
                result=true;
                if(data[DB_MEAS_RATE].i==sizeof(&pDb->rateMs))
                {
                    memcpy(&(pDb->rateMs), data[DB_MEAS_RATE].p, data[DB_MEAS_RATE].i);
                    UBX_LOG(LCAT_VERBOSE, "Decoded Measurement rate!");
                }
                else if(data[DB_MEAS_RATE].i)
                {
                    UBX_LOG(LCAT_WARNING, "Could not decode Measurement rate!");
                }
                else
                {
                    UBX_LOG(LCAT_VERBOSE, "No Measurement rate information found in the file!");
                }

                if(m_Db.pAgnssIf)
                {
                    // Load Position data
                    BUF_t * positionData=&(data[DB_POSITION]);

                    if(positionData && positionData->p && positionData->i)
                    {
                        if(pDb->pAgnssIf->saveToDb(CAgnss::POSITION, positionData->p, positionData->i))
                        {
                            UBX_LOG(LCAT_VERBOSE, "Decoded and stored Position information!", sys);
                        }
                        else if(data[DB_POSITION].i)
                        {
                            UBX_LOG(LCAT_WARNING, "Could not decode Position information!");
                        }
                    }
                    else
                    {
                        UBX_LOG(LCAT_VERBOSE, "No position information found in the file!");
                    }

                    // Load Receiver Aiding State
                    BUF_t * receiverStateData=&(data[db_receiverState]);

                    if(receiverStateData && receiverStateData->p && receiverStateData->i)
                    {
                        if(pDb->pAgnssIf->saveToDb(CAgnss::RECV_AID_STATE, receiverStateData->p, receiverStateData->i))
                        {
                            UBX_LOG(LCAT_VERBOSE, "Decoded and stored %s Receiver Aiding State information!", sys);
                        }
                        else
                        {
                            UBX_LOG(LCAT_WARNING, "Could not store %s Receiver Aiding State information!", sys);
                        }
                    }
                    else
                    {
                        UBX_LOG(LCAT_VERBOSE, "No %s Receiver Aiding State information found in the file", sys);
                    }

                    // Load Offline Data
                    if(isOfflineEnabled())
                    {
                        BUF_t * offlineData=NULL;
                        offlineData=&(data[db_offline]);

                        if(offlineData && offlineData->p && offlineData->i)
                        {
                            if(pDb->pAgnssIf->saveToDb(CAgnss::OFFLINE, offlineData->p, offlineData->i))
                            {
                                UBX_LOG(LCAT_VERBOSE, "Decoded and stored %s Offline information!", sys);
                            }
                            else
                            {
                                UBX_LOG(LCAT_WARNING, "Could not store %s Offline information!", sys);
                            }
                        }
                        else
                        {
                            UBX_LOG(LCAT_VERBOSE, "No %s Offline information found in the file", sys);
                        }
                    }
                }

                if(isAopEnabled())
                {
                    if(data[DB_LEG_AOP].p && data[DB_LEG_AOP].i)
                    {
                        int8_t svs=decodeAgnssf00(data[DB_LEG_AOP].p, data[DB_LEG_AOP].i, pDb->aopG, NUM_GPS_SVS);

                        if(svs>0)
                        {
                            UBX_LOG(LCAT_VERBOSE, "Decoded LEG AOP information for %d GPS satellites!", svs);
                        }
                        else
                        {
                            UBX_LOG(LCAT_ERROR, "Could not decode LEG AOP information!");
                        }
                    }
                }
            }
            freeBufs(data, bufSize);
        }
    }
    if(raw)
    {
        free(raw);
    }
    return result;
}
