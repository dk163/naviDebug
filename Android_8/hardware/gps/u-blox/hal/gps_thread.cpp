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
 * $Id: gps_thread.cpp 114242 2016-05-02 14:21:52Z fabio.robbiani $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/hal/gps_thread.cpp $
 *****************************************************************************/

/*!
  \file
  \brief Contains the main thread which coordinates the work

  The main thread defined in this file is started by the HAL module
  interface. The thread coordinates the work that has to be done within
  the whole module. The other functions are either used by this thread
  or by the HAL module interface to communicate with the thread.
*/

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include <sys/time.h>
#include <assert.h>

#include "ubx_log.h"
#include "std_types.h"
#include "std_lang_def.h"
#include "std_macros.h"

#include "ubx_moduleIf.h"
#include "ubx_timer.h"

#include "parserbuffer.h"
#include "protocolubx.h"
#include "protocolnmea.h"
#include "protocolunknown.h"

#include "ubxgpsstate.h"
#include "gps_thread.h"
#include "ubx_localDb.h"

#ifdef TCP_SERVER_PORT
#include "ubx_tcpServer.h"
#endif //ifdef TCP_SERVER_PORT


#ifdef SUPL_ENABLED
#   include "ubx_rilIf.h"
#   include "ubx_agpsIf.h"
#   include "ubx_niIf.h"
#   include "suplSMmanager.h"
#   ifndef ANDROID_BUILD
#       include <openssl/hmac.h>
#       include "openssl/ssl.h"
#       include <openssl/err.h>
#       include <openssl/engine.h>
#       include <openssl/conf.h>
#   endif //ifndef ANDROID_BUILD
#endif //ifdef SUPL_ENABLED

////////////////////////////////////////////////////////////////////////////////
// Definitions & Types
#define MAX_UDP_PACKET_LEN 16384    //!< Dimension of the temporary buffer for reading a complete UDP packet
#define MIN_INTERVAL        200     //!< Minimum interval time (in ms) receiver is capable of

// Debugging
#ifdef SUPL_ENABLED
//#define NI_TEST
#endif //ifdef SUPL_ENABLED

///////////////////////////////////////////////////////////////////////////////
// Local functions
static void requestStartEventHandler(void* pContext);
static void requestStopEventHandler(void* pContext);
static void changeSessionStatus(const ControlThreadInfo* pState);

///////////////////////////////////////////////////////////////////////////////
// Local Data



static CSerialPort s_ser;			//!< Hardware interface class instance(serial port / usb file handle)
#ifdef UDP_SERVER_PORT
static CUdpServer s_udp;			//!< UPD server class instance
#endif //ifdef UDP_SERVER_PORT

static GpsControlEventInterface s_eventHandler =		//!< Gps control event interface implementation
{
	requestStart_cb:	requestStartEventHandler,		//!< 'Start' event handler
	requestStop_cb:		requestStopEventHandler			//!< 'Stop' event handler
};

///////////////////////////////////////////////////////////////////////////////
// Golbal Data
#ifndef UNDEBUG
pthread_t g_gpsDrvMainThread = 0;		// Thread debugging
#endif //ifndef UNDEBUG

///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
// Control support

//! Signals to any waiting threads the last command has completed
/*!
  \param pControlThreadInfo	: Pointer to main thread data
  \param result				: Last command result to make available to interested parties
*/
static void signal_cmd_complete(ControlThreadInfo* pControlThreadInfo, int result)
{
	assert(pControlThreadInfo);
	UBX_LOG(LCAT_DEBUG, "Signal to waiting threads the result of the last command: %i", result);
    pthread_mutex_lock(&pControlThreadInfo->threadCmdCompleteMutex);
    pControlThreadInfo->cmdResult = result;
    pthread_cond_signal(&pControlThreadInfo->threadCmdCompleteCond);
    pthread_mutex_unlock(&pControlThreadInfo->threadCmdCompleteMutex);
	UBX_LOG(LCAT_DEBUG, "Signaling of result completed. Wait for main-thread");

	// Only if the result was positive, the main-thread will signal
	// that it is ready
	if(result==1)
	{
		UBX_LOG(LCAT_DEBUG, "Wait for ubx-main to signalise that everything is ready for normal operation");
		pthread_mutex_lock(&pControlThreadInfo->threadMainReadyMutex);
		while(!pControlThreadInfo->mainReady)
			pthread_cond_wait(&pControlThreadInfo->threadMainReadyCond,
							  &pControlThreadInfo->threadMainReadyMutex);
		pthread_mutex_unlock(&pControlThreadInfo->threadMainReadyMutex);
		UBX_LOG(LCAT_DEBUG, "Initialisation complete. main-thread seems to be ready. Continue.");
	}
	else
	{
		UBX_LOG(LCAT_ERROR, "Signaled error to main-thread. Exit");
	}
}

static void engineStop(ControlThreadInfo* pControlThreadInfo, bool si)
{
	if (si)
	{
		CAndroidDatabase::getInstance()->decPublish();
	}

	pthread_mutex_lock(&pControlThreadInfo->threadDataAccessMutex);
	UBX_LOG(LCAT_VERBOSE, "(Begin) Client count %d", pControlThreadInfo->clientCount);
	
	if (pControlThreadInfo->clientCount > 0)
	{
		pControlThreadInfo->clientCount--;

		if (pControlThreadInfo->clientCount == 0)
		{
			pControlThreadInfo->gpsState = GPS_STOPPING;
			CAndroidDatabase::getInstance()->resetPublish();
		}
	}
	
	if (s_ser.isFdOpen())
	{
		changeSessionStatus(pControlThreadInfo);
	}
	
	UBX_LOG(LCAT_VERBOSE, "(End) Client count %d", pControlThreadInfo->clientCount);
	pthread_mutex_unlock(&pControlThreadInfo->threadDataAccessMutex);

    // Do NOT signal command complete
    // This is send after response from device or timeout
}

static void handle_init(ControlThreadInfo* pControlThreadInfo)
{
	assert(pControlThreadInfo);
	// Signal main 
	UBX_LOG(LCAT_VERBOSE, "(%u): Init state. Signal main-thread to continue", (unsigned int) pthread_self());
    pControlThreadInfo->gpsState = GPS_STOPPED;
	signal_cmd_complete(pControlThreadInfo, 1);
	UBX_LOG(LCAT_VERBOSE, "Signaled main-thread to continue");
}

static void engineStart(ControlThreadInfo* pControlThreadInfo, bool si)
{
	pthread_mutex_lock(&pControlThreadInfo->threadDataAccessMutex);
	UBX_LOG(LCAT_VERBOSE, "(Begin) Client count %d", pControlThreadInfo->clientCount);
	assert(pControlThreadInfo->clientCount >= 0);
	pControlThreadInfo->clientCount++;

	if (pControlThreadInfo->clientCount == 1)
	{
		// Reset database as it will be out of date
		CAndroidDatabase::getInstance()->Reset();
		
		CUbxGpsState* pUbxGps = CUbxGpsState::getInstance();

		pUbxGps->lock();
		pUbxGps->turnRecv(CUbxGpsState::ON);
		pUbxGps->unlock();
		
		// request time from the ntp server
	#if (PLATFORM_SDK_VERSION >= 14 /* =4.0 */)
		CGpsIf::requestUtcTime();
	#endif //if (PLATFORM_SDK_VERSION >= 14)
		
		pControlThreadInfo->gpsState = GPS_STARTED;
		
#ifdef NI_TEST
		CNiIf::request(5000, NotificationType_notificationAndVerficationAllowedNA);
#endif //ifdef NI_TEST
		
		LOGV("%s : SUPL Start Set Initiated Action? %s", __FUNCTION__, (si? "Yes":"No"));
#ifdef SUPL_ENABLED
		if (si)
		{
			GpsPositionMode posMode = CGpsIf::getInstance()->getMode();
			bool suplStarted = false;
			if(suplActiveSessions())
			{
				UBX_LOG(LCAT_VERBOSE, "There are already active Set Initiated SUPL Sessions (%s)!", _LOOKUPSTR(posMode, GpsPositionMode));
			}
			else
			{
				if ((posMode == GPS_POSITION_MODE_MS_BASED) || (posMode == GPS_POSITION_MODE_MS_ASSISTED))
				{
					UBX_LOG(LCAT_VERBOSE, "Start Set Initiated Action for the %s SUPL mode!", _LOOKUPSTR(posMode, GpsPositionMode));
					suplStarted = suplStartSetInitiatedAction();
				}
				else
				{
					UBX_LOG(LCAT_VERBOSE, "Set Initiated Action for this SUPL mode (%s) will not be started!", _LOOKUPSTR(posMode, GpsPositionMode));
				}
			}

			if ((posMode != GPS_POSITION_MODE_MS_ASSISTED) || (!suplStarted))
			{
				CAndroidDatabase::getInstance()->incPublish();
			}
			UBX_LOG(LCAT_VERBOSE, "SUPL Set Initiated Action started (mode: %s)? %s", _LOOKUPSTR(posMode, GpsPositionMode) ,(suplStarted? "Yes":"No"));
		}
#else //ifdef SUPL_ENABLED
		CAndroidDatabase::getInstance()->incPublish();
#endif //else ifdef SUPL_ENABLED

	}
	
	if (s_ser.isFdOpen())
	{
		changeSessionStatus(pControlThreadInfo);
	}
	
	UBX_LOG(LCAT_VERBOSE, "(End) Client count %d", pControlThreadInfo->clientCount);
	pthread_mutex_unlock(&pControlThreadInfo->threadDataAccessMutex);
}

static bool handle_cmd(ControlThreadInfo* pControlThreadInfo, U1 cmd)
{
    bool ret = false;

	switch(cmd)
	{
		case CMD_START_SI:
			engineStart(pControlThreadInfo, true);
			break;
			
		case CMD_STOP_SI:
			engineStop(pControlThreadInfo, true);
			break;
			
		case CMD_START_NI:
			engineStart(pControlThreadInfo, false);
			break;
			
		case CMD_STOP_NI:
			engineStop(pControlThreadInfo, false);
			break;

#ifndef ANDROID_BUILD
		case CMD_EXIT:
			ret = true;
			break;
#endif //ifndef ANDROID_BUILD
		default:
			UBX_LOG(LCAT_ERROR, "Unexpected command %i", cmd);
			break;
	}

    return ret;
}

///////////////////////////////////////////////////////////////////////////////
//! Handle the device shut down process
/*! During the 'stopping' / device shutdown state/phase, initiates the receiver
    shutdown and checks on its progress. If this process has ended, the
    \ref pControlThreadInfo state is adjusted accordingly
  \param pControlThreadInfo	: Pointer to main control thread data
*/
static void checkShutdownState(ControlThreadInfo* pControlThreadInfo)
{
	// Is the receiver supposed to be turning off?
	if (pControlThreadInfo->gpsState == GPS_STOPPING)
	{
		CUbxGpsState* pUbxGps = CUbxGpsState::getInstance();

		pUbxGps->lock();
		// Has the receiver shutdon already?
		if (pUbxGps->isRecv(CUbxGpsState::OFF))
		{
			// Yes! Set the corresponding states here
			pthread_mutex_lock(&pControlThreadInfo->threadDataAccessMutex);
			UBX_LOG(LCAT_VERBOSE, "(Begin)");
			pControlThreadInfo->gpsState = GPS_STOPPED;
			changeSessionStatus(pControlThreadInfo);
			UBX_LOG(LCAT_VERBOSE, "(End)");
			pthread_mutex_unlock(&pControlThreadInfo->threadDataAccessMutex);
		}
		else if (!pUbxGps->isRecvInTransTo(CUbxGpsState::OFF))
		{
			// The receiver is supposed to be turning off, but
			// has not been instructed to do so yet. Do it.
			UBX_LOG(LCAT_VERBOSE, "Command the receiver to turn off!");
			pUbxGps->turnRecv(CUbxGpsState::OFF);
		}
		pUbxGps->unlock();

	}
}

static void changeSessionStatus(const ControlThreadInfo* pState)
{
#ifdef SUPL_ENABLED
	if ((pState->clientCount > 0) &&
		 (pState->clientCount == suplCountSessions(true)))
	{	
		// Only NI sessions. Don't report engine activity
		CGpsIf::gpsStatus(GPS_STATUS_ENGINE_OFF);
		return;
	}
#endif //ifdef SUPL_ENABLED

	switch (pState->gpsState)
	{
		case GPS_STARTED:
			CGpsIf::gpsStatus(GPS_STATUS_SESSION_BEGIN);
			break;

		case GPS_STOPPING:
			CGpsIf::gpsStatus(GPS_STATUS_SESSION_END);
			break;

		case GPS_STOPPED:
			CGpsIf::gpsStatus(GPS_STATUS_ENGINE_OFF);
			break;

		default:
			// Shouldn't happen
			assert(0);
			break;
	}
}

static bool handleCmdInput(ControlThreadInfo* pState, fd_set* pRfds)
{
	bool finish = false;
	
	if(FD_ISSET(pState->cmdPipes[0], pRfds))
	{
		// Command received
		U1 cmd;
		if (read(pState->cmdPipes[0], &cmd, 1) == -1)
		{
			UBX_LOG(LCAT_ERROR, "Cmd pipe read failure (%i)", errno);
		}
		else
		{
			UBX_LOG(LCAT_VERBOSE, "(%u): Cmd received (%i)", (unsigned int) pthread_self(), cmd);
			if (handle_cmd(pState, cmd))
			{
#ifndef ANDROID_BUILD
				UBX_LOG(LCAT_VERBOSE, "Exit thread cmd received");
				finish = true;
#endif //ifndef ANDROID_BUILD
			}
		}
	}
	
	return finish;
}

#ifdef UDP_SERVER_PORT
static void handleUdpInput(fd_set &rfds)
{
	if (s_udp.fdIsSet(rfds))
	{
		char tmpbuf[MAX_UDP_PACKET_LEN];
		int len = s_udp.recvPort(tmpbuf, sizeof(tmpbuf));
//                UBX_LOG(LCAT_VERBOSE, "Received something over UDP! %d", len);
		if (len > 0)
		{
			// UBX or PUBX data - forward to GPS
			if (s_ser.writeSerial(tmpbuf,(unsigned int) len) != len)
			{
				UBX_LOG(LCAT_ERROR, "unable to write %i to a master",len);
			}
		}
	}
}
#endif //ifdef UDP_SERVER_PORT

static bool connectReceiver(const ControlThreadInfo* pState, fd_set& rfds, int& rMaxFd)
{
	bool result=false;
	CUbxGpsState* pUbxGps = CUbxGpsState::getInstance();
	
	UBX_LOG(LCAT_VERBOSE, "Open/Reopen the serial port");
	/* try to open/reopen the serial port for the GPS listener */
	if (!s_ser.openSerial(pUbxGps->getSerialDevice(), pUbxGps->getBaudRateDefault(), 1, pUbxGps->getI2cTxReady()))
	{
		// Device not present (unplugged)
		CGpsIf::gpsStatus(GPS_STATUS_ENGINE_OFF);
		UBX_LOG(LCAT_ERROR, "Failing to reopen the serial port");
	}
	else
	{
		result=true;
		// Serial port opened/reopened - Baud rate needs to be set
		s_ser.fdSet(rfds, rMaxFd);

		pUbxGps = CUbxGpsState::getInstance();
		UBX_LOG(LCAT_VERBOSE, "Init receiver ");
		pUbxGps->lock();
		pUbxGps->init(&s_ser);
		pUbxGps->unlock();
		
		// Report appropriate status
		changeSessionStatus(pState);

		// Restart receiver if required
		if (pState->gpsState == GPS_STARTED)
		{
			UBX_LOG(LCAT_VERBOSE, "Start receiver ");
			// reconfigure the baud rate
			usleep(200000);
			pUbxGps->lock();
			pUbxGps->turnRecv(CUbxGpsState::ON);
			pUbxGps->unlock();
		}
	}
	return result;
}

static bool checkRecvInitReq(const ControlThreadInfo* pState, fd_set& rfds, int& maxFd, int64_t timeSinceMsgMs)
{
	CUbxGpsState *pUbxGps = CUbxGpsState::getInstance();
	bool connectRecv=false;
	int64_t thriceRateMs = 3 * pUbxGps->getRate();
	int64_t validMsgThresholdMs = thriceRateMs < 2000 ? 2000 : thriceRateMs;

#ifdef _EXTEND_MSG_TIMEOUT_FOR_TESTING
	validMsgThresholdMs += 3000;
#endif
	// Serial interface not properly opened?
	if (!s_ser.fdSet(rfds, maxFd))					// Add the gps device
	{
		connectRecv=true;
	}
	else if (pState->gpsState == GPS_STARTED 
	    &&  timeSinceMsgMs > validMsgThresholdMs)
	{
		// we have not received usefull messages within the required interval and need to reinit the receiver
		connectRecv=true;
		UBX_LOG( LCAT_VERBOSE, "No valid messages received in the last %lums . Reinitialising receiver - (Threshold: %lums)"
		       , (unsigned long) timeSinceMsgMs
		       , (unsigned long) validMsgThresholdMs);
	}

	// (Re-)initialise and restart receiver if required
	if(connectRecv)
	{
		sleep(1);
		
		// Serial channel to receiver not open
		connectRecv=connectReceiver(pState, rfds, maxFd);
	}
	return connectRecv;
}
//--------------------------------------------------------------------------------

static void releaseGpsThreadResources(ControlThreadInfo* pControlThreadInfo)
{
    s_ser.closeSerial();
#ifdef UDP_SERVER_PORT
    s_udp.closeUdp();
#endif //ifdef UDP_SERVER_PORT
    if (pControlThreadInfo->cmdPipes[0] != -1)
		close(pControlThreadInfo->cmdPipes[0]);
    pControlThreadInfo->cmdPipes[0] = -1;
    if (pControlThreadInfo->cmdPipes[1] != -1)
		close(pControlThreadInfo->cmdPipes[1]);
    pControlThreadInfo->cmdPipes[1] = -1;
}

///////////////////////////////////////////////////////////////////////////////
//! Main gps driver control thread
/*!
  \param pThreadData	: Pointer to thread data
*/
void ubx_thread(void *pThreadData)
{
    ControlThreadInfo* pState = (ControlThreadInfo*) pThreadData;

	assert(g_gpsDrvMainThread == 0);
#ifndef UNDEBUG	
	g_gpsDrvMainThread = pthread_self();			// For debugging threads
#endif //ifndef UNDEBUG
	int64_t nowMs = getMonotonicMsCounter();//!< the current time in ms
    int64_t timeLastValidUbxMsgMs = nowMs;	//!< time the last valid UBX messages was received in ms
    int64_t timeLastValidNmeaMsgMs = nowMs;	//!< time the last valid UBX messages was received in ms
	int64_t sinceLastUbxMs = 0;             //!< difference between now and the last valid UBX message
	int64_t sinceLastNmeaMs = 0;            //!< difference between now and the last valid NMEA message
	CAndroidDatabase* pDatabase = CAndroidDatabase::getInstance();
	
#ifdef UDP_SERVER_PORT
    int64_t timeoutPtsMs = nowMs;  			//!< for virtual serial status check
#endif //ifdef UDP_SERVER_PORT

	pDatabase->setGpsState(pState);
#ifdef SUPL_ENABLED
	suplInit();
#endif //ifdef SUPL_ENABLED
    CProtocolUBX  protocolUBX;
    CProtocolNMEA protocolNmea;
    CProtocolUnknown protocolUnknown;
    CParserBuffer parser;               // declare after protocols, so destructor called before protocol destructors

    // setup parser buffer
    parser.Register(&protocolUBX);
    parser.Register(&protocolNmea);
    parser.RegisterUnknown(&protocolUnknown);

    UBX_LOG(LCAT_VERBOSE, "(%u): Gps background thread started", (unsigned int) pthread_self());
	CUbxGpsState* pUbxGps = CUbxGpsState::getInstance();

#ifdef SUPL_ENABLED
	if(pUbxGps->getReceiverGeneration()<=7)
	{
		pDatabase->setBeginEpoch(CDatabase::MSG_UBX_RXM_MEAS);
	}
	else
#endif //ifdef SUPL_ENABLED
	{
		pDatabase->setBeginEpoch(CDatabase::MSG_UBX_NAV_STATUS);
	}
	pDatabase->setEndEpoch(CDatabase::MSG_NMEA_GST);

#ifdef SUPL_ENABLED	
	suplRegisterEventCallbacks(&s_eventHandler, pState);
#endif //ifdef SUPL_ENABLED

#ifdef UDP_SERVER_PORT
	pUbxGps->setUdp(&s_udp);
#endif //ifdef UDP_SERVER_PORT

#ifdef TCP_SERVER_PORT
	CTcpServer tcpServer;
	tcpServer.startServer( TCP_SERVER_PORT, 1);
#endif //ifdef TCP_SERVER_PORT
#ifdef UDP_SERVER_PORT
    if (s_udp.openLocalPort(UDP_SERVER_PORT) < 0)
    {
        UBX_LOG(LCAT_ERROR, "unable to open local port");
        UBX_LOG(LCAT_ERROR, "Exiting the thread");
        releaseGpsThreadResources(pState);
        signal_cmd_complete(pState, -1);      // Signal init fail
        return;
    }
#endif //ifdef UDP_SERVER_PORT

    if (pipe(pState->cmdPipes) == -1)
    {
        UBX_LOG(LCAT_ERROR, "Could not create cmd pipes (%i)", errno);
        releaseGpsThreadResources(pState);
        signal_cmd_complete(pState, -1);      // Signal init fail
        return;
    }

    handle_init(pState);    // also turn off the device when the thread starts
                            // and complete (signal init handler function)

	UBX_LOG(LCAT_VERBOSE, "Starting ubx-thread main loop");
    for (;;)
    {
        fd_set rfds;
        int maxFd = 0;
        bool updateTimeLastValid=false;

        /* Initialize the select mask */
        FD_ZERO(&rfds);

		// Does the receiver need to be (re-)initialised or restarted?
		// The time difference passed in ms must be the bigger of the two times.
		// If only UBX or NMEA stop working for some reason (e.g. receiver reboot),
		// the receiver has to be reconfigured accordingly. This requires
		// that periodical NMEA and UBX messages are enabled, otherwise this
		// mechanism might restart the receiver at unexpected times
		//
		// If the receiver was (re-)initialised, make sure that the receiver
		// does not restart for another complete interval by setting
		// the time of the last arrived messages to the current time
		// (set updateTimeLastValid to true)
		updateTimeLastValid=checkRecvInitReq(pState, rfds, maxFd,sinceLastNmeaMs>sinceLastUbxMs?sinceLastNmeaMs:sinceLastUbxMs);

        FD_SET(pState->cmdPipes[0], &rfds);  			// Add cmd queue pipe

        if (pState->cmdPipes[0]+1 > maxFd)
            maxFd = pState->cmdPipes[0]+1;

#ifdef TCP_SERVER_PORT
		tcpServer.fdSet(rfds, maxFd);
#endif //ifdef TCP_SERVER_PORT
#ifdef UDP_SERVER_PORT
        s_udp.fdSet(rfds, maxFd);						// Add UDP port connection
#endif //ifdef UDP_SERVER_PORT

#ifdef SUPL_ENABLED
		suplAddUplListeners(&rfds, &maxFd);				// Add Supl session sockets
#endif //ifdef SUPL_ENABLED
        /* make the select */
		struct timeval tv;		/* and setup the timeout to 1.0 seconds */
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        int res = s_ser.rselect(maxFd, &rfds, &tv);

		// Update the time reference for timeout calculations
        nowMs = getMonotonicMsCounter();

#ifdef UDP_SERVER_PORT
        if ((nowMs - timeoutPtsMs) >= 1000)
        {
            /* Check UDP connections */
            s_udp.checkPort(0);

            /* update pseudoterminal timeout */
            timeoutPtsMs = nowMs;
        }
#endif //ifdef UDP_SERVER_PORT

        if (res > 0)
        {
#ifdef TCP_SERVER_PORT
			if (tcpServer.fdIsSet(rfds))
			{
				ssize_t availableBytes=tcpServer.sizeNextMsg();
				if(availableBytes > 0)
				{
					unsigned char * tmpBuf=(unsigned char*) malloc(availableBytes);
					if(tmpBuf)
					{
						ssize_t len=tcpServer.getNextMsg(tmpBuf, availableBytes);
						if(len > 0)
						{
							if (s_ser.writeSerial(tmpBuf,(unsigned int) len) != len)
							{
								UBX_LOG(LCAT_ERROR, "Unable to write TCP client data of length %i to receiver",len);
							}
						}
						free(tmpBuf);
					}
				}
			}
#endif //ifdef TCP_SERVER_PORT
            if (s_ser.fdIsSet(rfds))
            {
				// There is some input in the serial port
				// fill the parser with new data
                unsigned char *ptr = parser.GetPointer();
                unsigned int space = (unsigned int) parser.GetSpace();
                int iSize = s_ser.readSerial(ptr, space);
				
                if (iSize)
                {
				    int iMsg;
					unsigned char* pMsg;
				    CProtocol* pProtocol;

                    // we read it so update the size
                    parser.Append(iSize);
                    // Parse ...
                    while (parser.Parse(pProtocol, pMsg, iMsg))
                    {
#ifdef UDP_SERVER_PORT
                        /* put the UBX token */
                        s_udp.sendPort(pMsg, iMsg);
#endif //ifdef UDP_SERVER_PORT
							
  						// redirecting
						if (pProtocol == &protocolUBX)
						{
#ifdef MSG_UBX_LOG_OUTPUT
							char *pStr;
							int len=byte_array_to_hex_string(&pStr, (unsigned char const *) pMsg, iMsg);
							if(len==-1)
							{
								UBX_LOG(LCAT_ERROR, "An error occured while trying to log an incoming UBX message");
							}
							else
							{
								UBX_LOG(LCAT_VERBOSE, "MSG UBX RX (size %d): %s", iMsg, pStr);
								free(pStr);
							}
#endif //ifdef MSG_UBX_LOG_OUTPUT	
							timeLastValidUbxMsgMs = nowMs = getMonotonicMsCounter();
						}
                        else if (pProtocol == &protocolNmea)
						{
#if (PLATFORM_SDK_VERSION > 8 /* >2.2 */)
                            if ((CGpsIf::getInstance()->m_callbacks.nmea_cb) &&
                                (getMonotonicMsCounter() >= pDatabase->getNextReportEpoch()))
                            {
 #if 1
                                struct timeval tv1;
                                gettimeofday(&tv1, NULL);
                                GpsUtcTime gpsUtcTime = (GpsUtcTime) tv1.tv_sec * (GpsUtcTime) 1000;
                                gpsUtcTime += (tv1.tv_usec / 1000);
 #else //if 1
                                GpsUtcTime gpsUtcTime = pDatabase->GetGpsUtcTime();
 #endif //else if 1
                                CGpsIf::getInstance()->m_callbacks.nmea_cb(gpsUtcTime, (const char*)pMsg, iMsg);
                            }
#endif // if (PLATFORM_SDK_VERSION > 8)
#ifdef MSG_NMEA_LOG_OUTPUT
                            UBX_LOG(LCAT_VERBOSE, "MSG NMEA %*.*s", iMsg-2, iMsg-2, pMsg);
#endif //ifdef MSG_NMEA_LOG_OUTPUT
							timeLastValidNmeaMsgMs = nowMs = getMonotonicMsCounter();
						}
                        else
						{
#ifdef MSG_UNKNOWN_LOG_OUTPUT
							char *pStr;
							int len=byte_array_to_hex_string(&pStr, (unsigned char const *) pMsg, iMsg);
							if(len==-1)
							{
								UBX_LOG(LCAT_ERROR, "An error occured while trying to log an incoming UNKNOWN message");
							}
							else
							{
								UBX_LOG(LCAT_VERBOSE, "MSG UNKNOWN RX (size %d): %s", iMsg, pStr);
								free(pStr);
							}
#else //MSG_UNKNOWN_LOG_OUTPUT
							UBX_LOG(LCAT_VERBOSE, "MSG UNKNOWN RX (size %d)", iMsg);
#endif //ifdef MSG_UNKNOWN_LOG_OUTPUT
                        }
						// PRINTF("size %5d ", iMsg);
                        // ... and Process
						pUbxGps->lock();
						pUbxGps->onNewMsg(pMsg, (unsigned int) iMsg);
						pUbxGps->unlock();
#ifdef TCP_SERVER_PORT
						tcpServer.onNewMsg(pMsg, iMsg);
#endif //ifdef TCP_SERVER_PORT
                        pProtocol->Process(pMsg, iMsg, pDatabase);
                        parser.Remove(iMsg);
                    }
					
                    parser.Compact();
                }
                else
                {
                    UBX_LOG(LCAT_ERROR, "read error %d", iSize);
                    s_ser.closeSerial();
                }
            }

#ifdef UDP_SERVER_PORT
            /* UDP PORT READ HANDLING */
			handleUdpInput(rfds);
#endif //ifdef UDP_SERVER_PORT

#ifdef SUPL_ENABLED
			suplReadUplSock(&rfds);			// Check and process any incoming SUPL data
#endif //ifdef SUPL_ENABLED
			if (handleCmdInput(pState, &rfds))
			{
				break;		// Will only happen when using test harness
			}
        }
        else
		{
			pUbxGps->lock();
			pUbxGps->onNewMsg(NULL, 0);
			pUbxGps->unlock();
//            UBX_LOG(LCAT_VERBOSE, "No input timeout");
        }

		// If the receiver is on, calculate the time difference to the
		// last arrived NMEA and UBX messages. Otherwise let the
		// time difference at 0
		if(updateTimeLastValid || pState->gpsState != GPS_STARTED)
		{
			timeLastValidUbxMsgMs = timeLastValidNmeaMsgMs = nowMs = getMonotonicMsCounter();
			sinceLastUbxMs=sinceLastNmeaMs=0;
		}
		else
		{
			sinceLastNmeaMs = nowMs - timeLastValidNmeaMsgMs;
			sinceLastUbxMs = nowMs - timeLastValidUbxMsgMs;
		}

#ifdef SUPL_ENABLED
        suplCheckPendingActions();		// Check for any actions on existing SUPL sessions
#endif //ifdef SUPL_ENABLED
        checkShutdownState(pState);
    }

    // Should never get too.

#ifdef TCP_SERVER_PORT
	tcpServer.stopServer();
#endif //ifdef TCP_SERVER_PORT
#ifndef ANDROID_BUILD
	suplDeinit();
	
	ERR_remove_state(0);
	EVP_cleanup();
	CRYPTO_cleanup_all_ex_data();
	ERR_free_strings();
	ENGINE_cleanup();
	CONF_modules_unload(1);
	CONF_modules_free();

    UBX_LOG(LCAT_VERBOSE, "Left main loop");
    releaseGpsThreadResources(pState);
    UBX_LOG(LCAT_VERBOSE, "Thread exiting");
#endif //ifndef ANDROID_BUILD	
    return;
}

///////////////////////////////////////////////////////////////////////////////
//! Injects a time into the receiver
/*!
  \param timeUtcGps		: Utc time
  \param timeReference	: Time reference
  \param uncertainty	: Uncertaincy
*/
void gps_state_inject_time(GpsUtcTime timeUtcGps, int64_t timeReference, int uncertainty)
{
	CUbxGpsState* pUbxGps = CUbxGpsState::getInstance();
	
    pUbxGps->lock();
    pUbxGps->putTime(timeUtcGps, timeReference, uncertainty);
	pUbxGps->sendPosAndTime();
	pUbxGps->unlock();
}

///////////////////////////////////////////////////////////////////////////////
//! Injects a location into the receiver
/*!
  \param latitude	: Latitude part of location
  \param longitude	: Longitude part of location
  \param accuracy	: Accuracy of location
*/
void gps_state_inject_location(double latitude, double longitude, float accuracy)
{
	CUbxGpsState* pUbxGps = CUbxGpsState::getInstance();

    pUbxGps->lock();
	pUbxGps->putPos(latitude, longitude, accuracy);
	pUbxGps->sendPosAndTime();
	pUbxGps->unlock();
}

///////////////////////////////////////////////////////////////////////////////
//! Deletes aiding data in the receiver
/*!
  \param flags	: Flags indicating which data to delete
*/
void gps_state_delete_aiding_data(GpsAidingData flags)
{
	CUbxGpsState* pUbxGps = CUbxGpsState::getInstance();
	
    pUbxGps->lock();
	if(pUbxGps->isRecv(CUbxGpsState::ON))
	{
    	pUbxGps->writeUbxCfgRst(0x02/*reset gps only*/, flags);
	}
	pUbxGps->unlock();
}

///////////////////////////////////////////////////////////////////////////////
//! Sets the interval for the driver to report to the framework
/*!
  \param min_interval	: Reporting interval in ms
*/
void gps_state_set_interval(uint32_t min_interval)
{
    int64_t nextReportEpochMs = 0;     // default - no driver intervention
	int timeInterval = 0;
	
    if (min_interval < MIN_INTERVAL)
    {
        // Below minimun, set to receiver minimum, with no driver intervention
        min_interval = MIN_INTERVAL;
    }
    else if ((min_interval >= MIN_INTERVAL) && (min_interval <= 2000))
    {
        // Receiver can cope with these values.
    }
    else
    {
        // Too fast for receiver, so driver will intervene to extend
        UBX_LOG(LCAT_WARNING, "Interval (%i) too big - Driver will intervene", min_interval);
        timeInterval = (int) min_interval;
        nextReportEpochMs = getMonotonicMsCounter();
        min_interval = 1000;
    }

	CAndroidDatabase* pDatabase = CAndroidDatabase::getInstance();
	pDatabase->setEpochInterval(timeInterval, nextReportEpochMs);
	
	CUbxGpsState* pUbxGps = CUbxGpsState::getInstance();
    pUbxGps->lock();
    pUbxGps->putRate((int) min_interval);
    pUbxGps->unlock();
}

///////////////////////////////////////////////////////////////////////////////
//! Releases any resources in the main thread's control data
/*!
  \param pControlThreadInfo	: Pointer to main thread data
*/
void controlThreadInfoRelease(ControlThreadInfo* pControlThreadInfo)
{
	pthread_mutex_destroy(&pControlThreadInfo->threadCmdCompleteMutex);
	pthread_cond_destroy(&pControlThreadInfo->threadCmdCompleteCond);
	pthread_mutex_destroy(&pControlThreadInfo->threadDataAccessMutex);
	pthread_mutex_destroy(&pControlThreadInfo->threadMainReadyMutex);
	pthread_cond_destroy(&pControlThreadInfo->threadMainReadyCond);
}

///////////////////////////////////////////////////////////////////////////////
//! Sends a command to the main thread
/*!
  \param pControlThreadInfo	: Pointer to main thread data
  \param cmd				: Command to send to main thread
*/
bool controlThreadInfoSendCmd(ControlThreadInfo* pControlThreadInfo, THREAD_CMDS cmd)
{
    U1 c = cmd;
    if (write(pControlThreadInfo->cmdPipes[1], &c, 1) == -1)
    {
        UBX_LOG(LCAT_ERROR, "Could not write to cmd pipe (%i)", errno);
		return false;
    }
	return true;
}

///////////////////////////////////////////////////////////////////////////////
//! Implimentation of a requestStart interface function on GpsControlEventInterface
/*!
  \param pContext	: Pointer to the interface context
*/
static void requestStartEventHandler(void* pContext)
{
	UBX_LOG(LCAT_VERBOSE, "Called");
	controlThreadInfoSendCmd((ControlThreadInfo *) pContext, CMD_START_NI);
}

///////////////////////////////////////////////////////////////////////////////
//! Implimentation of a requestStop interface function on GpsControlEventInterface
/*!
  \param pContext	: Pointer to the interface context
*/
static void requestStopEventHandler(void* pContext)
{
	UBX_LOG(LCAT_VERBOSE, "Called");
	//engineStop((ControlThreadInfo *) pContext);
	controlThreadInfoSendCmd((ControlThreadInfo *) pContext, CMD_STOP_NI);
}


