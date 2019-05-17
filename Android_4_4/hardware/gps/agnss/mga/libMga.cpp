/******************************************************************************
 * Copyright (C) u-blox AG
 * u-blox AG, Thalwil, Switzerland
 *
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose without fee is hereby granted, provided that this entire notice
 * is included in all copies of any software which is or includes a copy
 * or modification of this software and in all copies of the supporting
 * documentation for such software.
 *
 * THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
 * WARRANTY. IN PARTICULAR, NEITHER THE AUTHOR NOR U-BLOX MAKES ANY
 * REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY
 * OF THIS SOFTWARE OR ITS FITNESS FOR ANY PARTICULAR PURPOSE.
 *
 ******************************************************************************
 *
 * Project: libMga
 * Purpose: Library providing functions to help a host application to download
 *          MGA assistance data and pass it on to a u-blox GNSS receiver.
 *
 *****************************************************************************
 * $Id: libMga.cpp 113571 2016-04-01 14:11:03Z jon.bowern $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/PRODUCTS/libMga/tags/1.05/src/libMga.cpp $
 *****************************************************************************/

///////////////////////////////////////////////////////////////////////////////
// includes
#include "libMga.h"

#ifdef WIN32
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <winsock2.h>
#define strncasecmp _strnicmp
#else // WIN32
#  include <sys/types.h>
#  include <sys/socket.h>
#  include <sys/ioctl.h>
#  include <arpa/inet.h>
#  include <netdb.h>
#  include <unistd.h>
#  include <errno.h>
#  include <fcntl.h>
#  include <pthread.h>

typedef int SOCKET;
#  define INVALID_SOCKET        (-1)
#  define SOCKET_ERROR          (-1)
#endif // WIN32

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <assert.h>
#include <time.h>
#include <ctype.h>

///////////////////////////////////////////////////////////////////////////////
// definitions & types
#define UBX_SIG_PREFIX_1        0xB5
#define UBX_SIG_PREFIX_2        0x62
#define UBX_MSG_FRAME_SIZE      8
#define UBX_MSG_PAYLOAD_OFFSET  6

#define UBX_CLASS_MGA           0x13
#define UBX_MGA_ANO             0x20
#define UBX_MGA_ACK             0x60
#define UBX_MGA_BDS             0x03
#define UBX_MGA_GPS             0x00
#define UBX_MGA_GAL             0x02
#define UBX_MGA_QZSS            0x05
#define UBX_MGA_GLO             0x06
#define UBX_MGA_INI             0x40
#define UBX_MGA_FLASH           0x21

#define UBX_CLASS_ACK           0x05
#define UBX_ACK_ACK             0x01
#define UBX_ACK_NAK             0x00

#define UBX_CLASS_CFG           0x06
#define UBX_CFG_MSG             0x01

#define UBX_CLASS_AID           0x0B
#define UBX_AID_INI             0x01
#define UBX_AID_HUI             0x02
#define UBX_AID_ALM             0x30
#define UBX_AID_EPH             0x31
#define UBX_AID_ALP             0x50
#define UBX_AID_ALPSRV          0x32

#define UBX_AID_ALP_ACK_SIZE    9

#define FLASH_DATA_MSG_PAYLOAD  512
#define MS_IN_A_NS              1000000

#define MIN(a,b)                (((a) < (b)) ? (a) : (b))

#define PRIMARY_SERVER_RESPONSE_TIMEOUT      5      // seconds
#define SECONDARY_SERVER_RESPONSE_TIMEOUT   30      // seconds

#ifndef SOCK_NONBLOCK
#  define SOCK_NONBLOCK O_NONBLOCK //!< If SOCK_NONBLOCK is not defined, O_NONBLOCK is used instead
#endif //SOCK_NONBLOCK

typedef struct
{
    UBX_U1 header1;
    UBX_U1 header2;
    UBX_U1 msgClass;
    UBX_U1 msgId;
    UBX_U2 payloadLength;
    UBX_U1 type;
    UBX_U1 typeVersion;
    UBX_U2 sequence;
    UBX_U2 payloadCount;
} FlashDataMsgHeader;

// Legacy aiding message (UBX-AID-ALP) header layout
typedef struct
{
    UBX_U1 header1;
    UBX_U1 header2;
    UBX_U1 msgClass;
    UBX_U1 msgId;
    UBX_U2 payloadLength;
} UbxMsgHeader;

typedef enum
{
    MGA_ACK_MSG_NAK = 0,
    MGA_ACK_MSG_ACK
} MGA_ACK_TYPES;

// Legacy aiding flash data transfer process states
typedef enum
{
    LEGACY_AIDING_IDLE,
    LEGACY_AIDING_STARTING,
    LEGACY_AIDING_MAIN_SEQ,
    LEGACY_AIDING_STOPPING
} LEGACY_AIDING_STATE;

///////////////////////////////////////////////////////////////////////////////
// module variables
static const MgaEventInterface* s_pEvtInterface = NULL;
static const MgaFlowConfiguration* s_pFlowConfig = NULL;
static const void* s_pCallbackContext = NULL;

static MGA_LIB_STATE s_sessionState = MGA_IDLE;

static MgaMsgInfo* s_pMgaMsgList    = NULL;
static UBX_U4 s_mgaBlockCount       = 0;
static UBX_U4 s_ackCount            = 0;
static MgaMsgInfo* s_pLastMsgSent   = NULL;
static UBX_U4 s_messagesSent        = 0;

static MgaMsgInfo* s_pMgaFlashBlockList     = NULL;
static UBX_U4 s_mgaFlashBlockCount          = 0;
static MgaMsgInfo* s_pLastFlashBlockSent    = NULL;
static UBX_U4 s_flashMessagesSent           = 0;
static UBX_U2 s_flashSequence               = 0;

// Specific support for legacy aiding
static bool s_bLegacyAiding             = false;
static LEGACY_AIDING_STATE s_aidState   = LEGACY_AIDING_IDLE;
static time_t s_aidingTimeout           = 0;

static UBX_U2 s_alpfileId                   = 0;
static UBX_U1 *s_pAidingData                = NULL;
static UBX_U4 s_aidingDataSize              = 0;
static bool s_aidingSrvActive               = false;

static long s_serverResponseTimeout = 0;

#ifdef WIN32
static CRITICAL_SECTION s_mgaLock;
#else // WIN32
static pthread_mutex_t s_mgaLock;
#endif // WIN32

///////////////////////////////////////////////////////////////////////////////
// local function declarations
static SOCKET connectServer(const char* strServer, UBX_U2 wPort, int sockType = SOCK_STREAM, int sckProtocol = IPPROTO_TCP);
static int getHttpHeader(SOCKET sock, char* buf, int cnt);
static const char* skipSpaces(const char* pHttpPos);
static const char* nextToken(const char* pText);
static int getData(SOCKET sock, char* p, size_t iSize);
static MGA_API_RESULT getDataFromService(SOCKET iSock, const char* pRequest, UBX_U1** ppData, UBX_I4* piSize);
static MGA_API_RESULT getOnlineDataFromServer(const char* pServer, const MgaOnlineServerConfig* pServerConfig, UBX_U1** ppData, UBX_I4* piSize);
static MGA_API_RESULT getOfflineDataFromServer(const char* pServer, const MgaOfflineServerConfig* pServerConfig, UBX_U1** ppData, UBX_I4* piSize);

static void lock(void);
static void unlock(void);

static MGA_API_RESULT handleAidAckMsg(int ackType);
static MGA_API_RESULT handleMgaAckMsg(const UBX_U1* pPayload);
static MGA_API_RESULT handleFlashAckMsg(const UBX_U1* pPayload);
static MGA_API_RESULT handleAidFlashAckMsg(void);
static MGA_API_RESULT handleAidFlashNakMsg(void);
static MGA_API_RESULT handleAidingResponseMsg(const UBX_U1* pMessageData);

static void legacyAidingCheckMessage(const UBX_U1* pData, UBX_U4 iSize);
static void legacyAidingRequestData(const LegacyAidingRequestHeader *pAidingRequestHeader);
static void legacyAidingUpdateData(const LegacyAidingUpdateDataHeader *pLegacyAidingUpdateHeader);

static void handleLegacyAidingTimeout(void);
static void sendMgaFlashBlock(bool next);
static void sendFlashMainSeqBlock(void);
static UBX_I4 sendNextMgaMessage(void);
static void sendAllMessages(void);
static void resendMessage(MgaMsgInfo* pResendMsg);
static void sendMgaIniTime(const MgaTimeAdjust* pTime);
static void sendCfgMgaAidAcks(bool enable);
static void sendInitialMsgBatch(void);
static void sendFlashStop(void);
static void sendAidingFlashStop(void);
static void sendEmptyFlashBlock(void);
static void initiateMessageTransfer(void);

static MGA_API_RESULT countMgaMsg(const UBX_U1* pMgaData, UBX_I4 iSize, UBX_U4* piCount);
static MgaMsgInfo* buildMsgList(const UBX_U1* pMgaData, unsigned int uNumEntries);
static void sessionStop(MGA_PROGRESS_EVENT_TYPE evtType, const void* pEventInfo, size_t evtInfoSize);
static MgaMsgInfo* findMsgBlock(UBX_U1 msgId, const UBX_U1* pMgaHeader);

static bool validChecksum(const UBX_U1* pPayload, size_t iSize);
static void addChecksum(UBX_U1* pPayload, size_t iSize);
static bool checkForIniMessage(const UBX_U1* pUbxMsg);
static void adjustMgaIniTime(MgaMsgInfo* pMsgInfo, const MgaTimeAdjust* pMgaTime);
static bool isAnoMatch(const UBX_U1* pMgaData, int cy, int cm, int cd);
static void commaToPoint(char* pText);

///////////////////////////////////////////////////////////////////////////////
// libMga API implementation

MGA_API_RESULT mgaInit(void)
{
    assert(s_sessionState == MGA_IDLE);
#ifdef WIN32
    InitializeCriticalSection(&s_mgaLock);
#else // WIN32
    pthread_mutex_init(&s_mgaLock, NULL);
#endif // WIN32
    return MGA_API_OK;
}

MGA_API_RESULT mgaDeinit(void)
{
    assert(s_sessionState == MGA_IDLE);
#ifdef WIN32
    DeleteCriticalSection(&s_mgaLock);
#else // WIN32
    pthread_mutex_destroy(&s_mgaLock);
#endif // WIN32
    return MGA_API_OK;
}

const UBX_CH* mgaGetVersion(void)
{
    return LIBMGA_VERSION;
}

MGA_API_RESULT mgaConfigure(const MgaFlowConfiguration* pFlowConfig,
                            const MgaEventInterface* pEvtInterface,
                            const void* pCallbackContext)
{
    MGA_API_RESULT res = MGA_API_OK;

    lock();

    if (s_sessionState == MGA_IDLE)
    {
        s_pEvtInterface = pEvtInterface;
        s_pFlowConfig = pFlowConfig;
        s_pCallbackContext = pCallbackContext;
    }
    else
        res = MGA_API_ALREADY_RUNNING;

    unlock();

    return res;
}

MGA_API_RESULT mgaSessionStart(void)
{
    MGA_API_RESULT res = MGA_API_OK;

    lock();

    if (s_sessionState == MGA_IDLE)
    {
        assert(s_pMgaMsgList == NULL);
        assert(s_mgaBlockCount == 0);
        assert(s_pLastMsgSent == NULL);
        assert(s_messagesSent == 0);
        assert(s_ackCount == 0);

        assert(s_pMgaFlashBlockList == NULL);
        assert(s_mgaFlashBlockCount == 0);
        assert(s_pLastFlashBlockSent == NULL);
        assert(s_flashMessagesSent == 0);
        assert(s_flashSequence == 0);

        assert(s_aidState == LEGACY_AIDING_IDLE);
        assert(s_aidingTimeout == 0);

        s_sessionState = MGA_ACTIVE_PROCESSING_DATA;
    }
    else
        res = MGA_API_ALREADY_RUNNING;

    unlock();

    return res;
}

MGA_API_RESULT mgaSessionStop(void)
{
    MGA_API_RESULT res = MGA_API_OK;

    lock();

    if (s_sessionState != MGA_IDLE)
    {
        EVT_TERMINATION_REASON stopReason = TERMINATE_HOST_CANCEL;
        sessionStop(MGA_PROGRESS_EVT_TERMINATED, &stopReason, sizeof(stopReason));
    }
    else
        res = MGA_API_ALREADY_IDLE;

    unlock();

    return res;
}

MGA_API_RESULT mgaSessionSendOnlineData(const UBX_U1* pMgaData, UBX_I4 iSize, const MgaTimeAdjust* pMgaTimeAdjust)
{
    if (iSize <= 0)
    {
        return MGA_API_NO_DATA_TO_SEND;
    }

    MGA_API_RESULT res = MGA_API_OK;

    lock();

    if (s_sessionState != MGA_IDLE)
    {
        assert(s_mgaBlockCount == 0);
        assert(s_pMgaMsgList == NULL);

        res = countMgaMsg(pMgaData, iSize, &s_mgaBlockCount);
        if (res == MGA_API_OK)
        {
            if (s_mgaBlockCount > 0)
            {
                s_pMgaMsgList = buildMsgList(pMgaData, s_mgaBlockCount);

                if (s_pMgaMsgList != NULL)
                {
                    if (checkForIniMessage(s_pMgaMsgList[0].pMsg))
                    {
                        if (s_pEvtInterface->evtProgress)
                        {
                            s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_START, s_pCallbackContext, (const void *) &s_mgaBlockCount, (UBX_I4) sizeof(s_mgaBlockCount));
                        }

                        if (pMgaTimeAdjust != NULL)
                        {
                            adjustMgaIniTime(&s_pMgaMsgList[0], pMgaTimeAdjust);
                        }

                        // send initial set of messages to receiver
                        initiateMessageTransfer();

                        res = MGA_API_OK;
                    }
                    else
                    {
                        res = MGA_API_NO_MGA_INI_TIME;
                    }
                }
                else
                {
                    s_mgaBlockCount = 0;
                    res = MGA_API_OUT_OF_MEMORY;
                }
            }
            else
            {
                // nothing to send
                res = MGA_API_NO_DATA_TO_SEND;
            }
        }
    }
    else
    {
        res = MGA_API_ALREADY_IDLE;
    }

    unlock();

    return res;
}

MGA_API_RESULT mgaSessionSendOfflineData(const UBX_U1* pMgaData, UBX_I4 iSize, const MgaTimeAdjust* pTime)
{
    MGA_API_RESULT res = MGA_API_OK;

    lock();

    if (s_sessionState != MGA_IDLE)
    {
        assert(s_mgaBlockCount == 0);
        assert(s_pMgaMsgList == NULL);

        res = countMgaMsg(pMgaData, iSize, &s_mgaBlockCount);

        if (s_mgaBlockCount > 0)
        {
            s_pMgaMsgList = buildMsgList(pMgaData, s_mgaBlockCount);

            if (s_pMgaMsgList != NULL)
            {
                // send initial set of messages to receiver
                if (s_pEvtInterface->evtProgress)
                {
                    s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_START, s_pCallbackContext, (const void *) &s_mgaBlockCount, (UBX_I4) sizeof(s_mgaBlockCount));
                }

                assert(pTime);
                assert(pTime->mgaAdjustType == MGA_TIME_ADJUST_ABSOLUTE);
                sendMgaIniTime(pTime);

                // send initial set of messages to receiver
                initiateMessageTransfer();

                res = MGA_API_OK;
            }
            else
                res = MGA_API_OUT_OF_MEMORY;
        }
    }
    else
        res = MGA_API_ALREADY_IDLE;

    unlock();

    return res;
}

MGA_API_RESULT mgaProcessReceiverMessage(const UBX_U1* pMgaData, UBX_I4 iSize)
{
    MGA_API_RESULT res = MGA_API_IGNORED_MSG;

    lock();

    if (s_sessionState != MGA_IDLE)
    {
        // Look for ACK & NAK
        if ((pMgaData[0] == UBX_SIG_PREFIX_1) &&
            (pMgaData[1] == UBX_SIG_PREFIX_2) &&
            (iSize >= UBX_MSG_FRAME_SIZE))
        {
            // UBX message
            if (s_aidingSrvActive)
            {
                // Legacy aiding server is active
                legacyAidingCheckMessage(pMgaData, iSize);
            }
            else if (iSize == UBX_AID_ALP_ACK_SIZE)
            {
                if (s_bLegacyAiding)
                {
                    res = handleAidingResponseMsg(pMgaData);
                }
            }
            else
            {
                // Look for MGA ack/nak
                switch (pMgaData[2])
                {
                case UBX_CLASS_MGA:  // MGA
                    // MGA message
                    if ((pMgaData[3] == UBX_MGA_ACK) && (iSize == (UBX_MSG_FRAME_SIZE + 8)))
                    {
                        // MGA-ACK
                        if (s_pLastMsgSent != NULL)
                        {
                            res = handleMgaAckMsg(&pMgaData[6]);
                        }
                    }
                    else if ((pMgaData[3] == UBX_MGA_FLASH) && (iSize == (UBX_MSG_FRAME_SIZE + 6)))
                    {
                        // MGA-FLASH-ACK
                        res = handleFlashAckMsg(&pMgaData[6]);
                    }
                    else
                    {
                        // other MGA message - Ignore
                    }
                    break;

                case UBX_CLASS_ACK:  // generic ACK/NAK message
                    if (iSize == (UBX_MSG_FRAME_SIZE + 2))
                    {
                        if ((s_pLastMsgSent != NULL) &&
                            (pMgaData[6] == UBX_CLASS_AID) &&
                            (pMgaData[7] == s_pLastMsgSent->mgaMsg.msgId))
                        {
                            res = handleAidAckMsg(pMgaData[3]);
                        }
                    }
                    break;

                default:
                    break;
                }

            }
        }
    }

    unlock();

    return res;
}

MGA_API_RESULT mgaGetOnlineData(const MgaOnlineServerConfig* pServerConfig, UBX_U1** ppData, UBX_I4* piSize)
{
    s_serverResponseTimeout = PRIMARY_SERVER_RESPONSE_TIMEOUT;

    MGA_API_RESULT res = getOnlineDataFromServer(pServerConfig->strPrimaryServer, pServerConfig, ppData, piSize);

    if (res != MGA_API_OK)
    {
        s_serverResponseTimeout = SECONDARY_SERVER_RESPONSE_TIMEOUT;
        res = getOnlineDataFromServer(pServerConfig->strSecondaryServer, pServerConfig, ppData, piSize);
    }

    return res;
}

MGA_API_RESULT mgaBuildOnlineRequestParams(const MgaOnlineServerConfig* pServerConfig,
                                           UBX_CH* pBuffer,
                                           UBX_I4 iSize)
{
    (void) iSize;

    sprintf(pBuffer, "token=%s", pServerConfig->strServerToken);

    // check which GNSS requested
    if (pServerConfig->gnssTypeFlags)
    {
        strcat(pBuffer,";gnss=");
        if (pServerConfig->gnssTypeFlags & MGA_GNSS_GPS)
            strcat(pBuffer,"gps,");
        if (pServerConfig->gnssTypeFlags & MGA_GNSS_GLO)
            strcat(pBuffer,"glo,");
        if (pServerConfig->gnssTypeFlags & MGA_GNSS_QZSS)
            strcat(pBuffer,"qzss,");
        if (pServerConfig->gnssTypeFlags & MGA_GNSS_BEIDOU)
            strcat(pBuffer,"bds,");
        if (pServerConfig->gnssTypeFlags & MGA_GNSS_GALILEO)
            strcat(pBuffer,"gal,");

        // remove last comma
        pBuffer[strlen(pBuffer) - 1] = '\0';
    }

    // check which data type requested
    if (pServerConfig->dataTypeFlags)
    {
        strcat(pBuffer,";datatype=");
        if (pServerConfig->dataTypeFlags & MGA_DATA_EPH)
            strcat(pBuffer,"eph,");
        if (pServerConfig->dataTypeFlags & MGA_DATA_ALM)
            strcat(pBuffer,"alm,");
        if (pServerConfig->dataTypeFlags & MGA_DATA_AUX)
            strcat(pBuffer,"aux,");
        if (pServerConfig->dataTypeFlags & MGA_DATA_POS)
            strcat(pBuffer,"pos,");

        // remove last comma
        pBuffer[strlen(pBuffer) - 1] = '\0';
    }

    // check if position should be used
    if (pServerConfig->useFlags & MGA_FLAGS_USE_POSITION)
    {
        char* pStart = &pBuffer[strlen(pBuffer)];

        sprintf(pStart,
                ";lat=%f;lon=%f;alt=%f;pacc=%f",
                pServerConfig->dblLatitude,
                pServerConfig->dblLongitude,
                pServerConfig->dblAltitude,
                pServerConfig->dblAccuracy);

        // make sure if commas used, then convert to decimal place
        commaToPoint(pStart);
    }

    // check if ephemeris should be filtered on position
    if (pServerConfig->bFilterOnPos)
    {
        strcat(pBuffer, ";filteronpos");
    }

    // check if latency should be used (for time aiding)
    if (pServerConfig->useFlags & MGA_FLAGS_USE_LATENCY)
    {
        char* pStart = &pBuffer[strlen(pBuffer)];
        sprintf(pStart, ";latency=%f", pServerConfig->latency);
        commaToPoint(pStart);
    }

    // check if time accuracy should be used (for time aiding)
    if (pServerConfig->useFlags & MGA_FLAGS_USE_TIMEACC)
    {
        char* pStart = &pBuffer[strlen(pBuffer)];
        sprintf(pStart, ";tacc=%f", pServerConfig->timeAccuracy);
        commaToPoint(pStart);
    }

    if (pServerConfig->useFlags & MGA_FLAGS_USE_LEGACY_AIDING)
    {
        strcat(pBuffer, ";format=aid");
    }


    assert(((size_t) iSize) > strlen(pBuffer));

    return MGA_API_OK;
}

MGA_API_RESULT mgaGetOfflineData(const MgaOfflineServerConfig* pServerConfig, UBX_U1** ppData, UBX_I4* piSize)
{
    s_serverResponseTimeout = PRIMARY_SERVER_RESPONSE_TIMEOUT;

    MGA_API_RESULT res = getOfflineDataFromServer(pServerConfig->strPrimaryServer, pServerConfig, ppData, piSize);

    if (res != MGA_API_OK)
    {
        s_serverResponseTimeout = SECONDARY_SERVER_RESPONSE_TIMEOUT;
        res = getOfflineDataFromServer(pServerConfig->strSecondaryServer, pServerConfig, ppData, piSize);
    }

    return res;
}

MGA_API_RESULT mgaBuildOfflineRequestParams(const MgaOfflineServerConfig* pServerConfig,
                                            UBX_CH* pBuffer,
                                            UBX_I4 iSize)
{
    (void) iSize;
    sprintf(pBuffer, "token=%s", pServerConfig->strServerToken);

    // check which GNSS requested
    if (pServerConfig->gnssTypeFlags)
    {
        strcat(pBuffer,";gnss=");
        if (pServerConfig->gnssTypeFlags & MGA_GNSS_GPS)
            strcat(pBuffer,"gps,");
        if (pServerConfig->gnssTypeFlags & MGA_GNSS_GLO)
            strcat(pBuffer,"glo,");
        if (pServerConfig->gnssTypeFlags & MGA_GNSS_QZSS)
            strcat(pBuffer,"qzss,");
        if (pServerConfig->gnssTypeFlags & MGA_GNSS_BEIDOU)
            strcat(pBuffer,"bds,");
        if (pServerConfig->gnssTypeFlags & MGA_GNSS_GALILEO)
            strcat(pBuffer,"gal,");

        // remove last comma
        pBuffer[strlen(pBuffer) - 1] = '\0';
    }

    if (pServerConfig->useFlags & MGA_FLAGS_USE_LEGACY_AIDING)
    {
        strcat(pBuffer, ";format=aid");
        // check if number of days should be used
        char numberBuffer[20];
        if (pServerConfig->numofdays > 0)
        {
            strcat(pBuffer, ";days=");
            sprintf(numberBuffer,"%d", pServerConfig->numofdays);
            strcat(pBuffer, numberBuffer);
        }
    }

    // check if period (in weeks) should be used
    char numberBuffer[20];
    if (pServerConfig->period > 0)
    {
        strcat(pBuffer, ";period=");
        sprintf(numberBuffer,"%d", pServerConfig->period);
        strcat(pBuffer, numberBuffer);
    }

    // check if resolution (in days) should be used
    if (pServerConfig->resolution > 0)
    {
        strcat(pBuffer, ";resolution=");
        sprintf(numberBuffer,"%d", pServerConfig->resolution);
        strcat(pBuffer, numberBuffer);
    }


    assert(((size_t) iSize) > strlen(pBuffer));

    return MGA_API_OK;
}


MGA_API_RESULT mgaSessionSendOfflineToFlash(const UBX_U1* pMgaData, UBX_I4 iSize)
{
    MGA_API_RESULT res = MGA_API_OK;

    lock();

    if (s_sessionState != MGA_IDLE)
    {
        s_mgaFlashBlockCount = (UBX_U4)iSize / FLASH_DATA_MSG_PAYLOAD;
        UBX_U2 lastBlockSize = (UBX_U2)iSize % FLASH_DATA_MSG_PAYLOAD;

        if(lastBlockSize > 0)
            s_mgaFlashBlockCount++;

        if (s_mgaFlashBlockCount > 0)
        {
            s_pMgaFlashBlockList = (MgaMsgInfo*) malloc(s_mgaFlashBlockCount * sizeof(MgaMsgInfo));

            if (s_pMgaFlashBlockList != NULL)
            {
                for(UBX_U4 i = 0; i < s_mgaFlashBlockCount; i++)
                {
                    s_pMgaFlashBlockList[i].pMsg = pMgaData;
                    s_pMgaFlashBlockList[i].state = MGA_MSG_WAITING_TO_SEND;
                    s_pMgaFlashBlockList[i].sequenceNumber = (UBX_U2) i;
                    s_pMgaFlashBlockList[i].retryCount = 0;
                    s_pMgaFlashBlockList[i].timeOut = 0;
                    s_pMgaFlashBlockList[i].mgaFailedReason = MGA_FAILED_REASON_CODE_NOT_SET;

                    if ((i == (s_mgaFlashBlockCount - 1)) && (lastBlockSize > 0))
                    {
                        // last block
                        s_pMgaFlashBlockList[i].msgSize = lastBlockSize;
                    }
                    else
                    {
                        s_pMgaFlashBlockList[i].msgSize = FLASH_DATA_MSG_PAYLOAD;
                    }

                    pMgaData += s_pMgaFlashBlockList[i].msgSize;
                }

                if (s_pEvtInterface->evtProgress)
                {
                    s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_START, s_pCallbackContext, (const void *) &s_mgaFlashBlockCount, sizeof(s_mgaFlashBlockCount));
                }

                // send initial set of messages to receiver
                sendMgaFlashBlock(true);
                res = MGA_API_OK;
            }
            else
            {
                s_mgaFlashBlockCount = 0;
                res = MGA_API_OUT_OF_MEMORY;
            }
        }
        else
        {
            // nothing to send
            res = MGA_API_NO_DATA_TO_SEND;
        }
    }
    else
    {
        res = MGA_API_ALREADY_IDLE;
    }

    unlock();

    return res;
}

MGA_API_RESULT mgaSessionSendLegacyOfflineToFlash(const UBX_U1* pAidingData, UBX_U4 iSize)
{
    MGA_API_RESULT res = MGA_API_OK;

    lock();

    if (s_sessionState != MGA_IDLE)
    {
        s_mgaFlashBlockCount = (UBX_U4)iSize / FLASH_DATA_MSG_PAYLOAD;
        UBX_U2 lastBlockSize = (UBX_U2)iSize % FLASH_DATA_MSG_PAYLOAD;

        if(lastBlockSize > 0)
            s_mgaFlashBlockCount++;

        if (s_mgaFlashBlockCount > 0)
        {
            s_pMgaFlashBlockList = (MgaMsgInfo*) malloc(s_mgaFlashBlockCount * sizeof(MgaMsgInfo));

            if (s_pMgaFlashBlockList != NULL)
            {
                for(UBX_U4 i = 0; i < s_mgaFlashBlockCount; i++)
                {
                    s_pMgaFlashBlockList[i].pMsg = pAidingData;
                    s_pMgaFlashBlockList[i].state = MGA_MSG_WAITING_TO_SEND;
                    s_pMgaFlashBlockList[i].sequenceNumber = (UBX_U2) i;
                    s_pMgaFlashBlockList[i].retryCount = 0;
                    s_pMgaFlashBlockList[i].timeOut = 0;
                    s_pMgaFlashBlockList[i].mgaFailedReason = MGA_FAILED_REASON_CODE_NOT_SET;

                    if ((i == (s_mgaFlashBlockCount - 1)) && (lastBlockSize > 0))
                    {
                        // last block
                        s_pMgaFlashBlockList[i].msgSize = lastBlockSize;
                    }
                    else
                    {
                        s_pMgaFlashBlockList[i].msgSize = FLASH_DATA_MSG_PAYLOAD;
                    }

                    pAidingData += s_pMgaFlashBlockList[i].msgSize;
                }
                if (s_pEvtInterface->evtProgress)
                {
                    s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_LEGACY_AIDING_STARTUP, s_pCallbackContext, (const void *) &s_mgaFlashBlockCount, sizeof(s_mgaFlashBlockCount));
                }

                // send initial set of messages to receiver
                s_bLegacyAiding = true;
                s_aidState = LEGACY_AIDING_STARTING;
                sendAidingFlashStop();                  // Quirky 'starting' process for aiding is to send a 'stop'
                res = MGA_API_OK;
            }
            else
            {
                s_mgaFlashBlockCount = 0;
                res = MGA_API_OUT_OF_MEMORY;
            }
        }
        else
        {
            // nothing to send
            res = MGA_API_NO_DATA_TO_SEND;
        }
    }
    else
    {
        res = MGA_API_ALREADY_IDLE;
    }

    unlock();

    return res;
}

MGA_API_RESULT mgaCheckForTimeOuts(void)
{
    lock();

    if ((s_pMgaMsgList == NULL) && (s_pMgaFlashBlockList == NULL))
    {
        // no work to do
        unlock();

        return MGA_API_OK;
    }

    if (s_bLegacyAiding == true)
    {
        handleLegacyAidingTimeout();
    }
    else if (s_pMgaMsgList != NULL)
    {
        assert(s_mgaBlockCount > 0);

        MgaMsgInfo* pMsgInfo = s_pMgaMsgList;

        UBX_U4 i;
        for (i = 0; i < s_mgaBlockCount; i++)
        {
            if (pMsgInfo->state == MGA_MSG_WAITING_FOR_ACK)
            {
                time_t now = time(NULL);

                if (now > pMsgInfo->timeOut)
                {
                    if (pMsgInfo->retryCount < s_pFlowConfig->msgRetryCount)
                    {
                        pMsgInfo->state = MGA_MSG_WAITING_FOR_RESEND;
                        pMsgInfo->retryCount++;
                        resendMessage(pMsgInfo);
                    }
                    else
                    {
                        // too many retries - so message transfer has failed
                        pMsgInfo->state = MGA_MSG_FAILED;
                        pMsgInfo->mgaFailedReason = MGA_FAILED_REASON_TOO_MANY_RETRIES;
                        assert(s_pEvtInterface);
                        assert(s_pEvtInterface->evtWriteDevice);

                        if (s_pEvtInterface->evtProgress)
                        {
                            s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_MSG_TRANSFER_FAILED, s_pCallbackContext, pMsgInfo, sizeof(MgaMsgInfo));
                        }

                        sendNextMgaMessage();

                        // check for last expected message
                        if (s_messagesSent == s_mgaBlockCount)
                        {
                            // stop the session
                            sessionStop(MGA_PROGRESS_EVT_FINISH, NULL, 0);
                        }
                    }
                }
            }
            pMsgInfo++;
        }
    }
    else
    {
        // doing a flash transfer
        assert(s_mgaFlashBlockCount > 0);
        assert(s_pLastFlashBlockSent != NULL);

        if ((s_pLastFlashBlockSent->state == MGA_MSG_WAITING_FOR_ACK) ||
            (s_pLastFlashBlockSent->state  == MGA_MSG_WAITING_FOR_ACK_SECOND_CHANCE))
        {
            time_t now = time(NULL);
            if (now > s_pLastFlashBlockSent->timeOut)
            {
                // Timed out
                if (s_pLastFlashBlockSent->state == MGA_MSG_WAITING_FOR_ACK_SECOND_CHANCE)
                {
                    // resend last block
                    sendMgaFlashBlock(false);
                }
                else
                {
                    // Send nudge byte to receiver
                    s_pLastFlashBlockSent->state = MGA_MSG_WAITING_FOR_ACK_SECOND_CHANCE;
                    UBX_U1 flashDataMsg = 0;
                    s_pEvtInterface->evtWriteDevice(s_pCallbackContext, &flashDataMsg, sizeof(flashDataMsg));
                }
            }
        }
    }

    unlock();

    return MGA_API_OK;
}

MGA_API_RESULT mgaEraseOfflineFlash(void)
{
    MGA_API_RESULT res = MGA_API_OK;

    lock();

    if (s_sessionState != MGA_IDLE)
    {
        sendEmptyFlashBlock();
        sendFlashStop();
    }
    else
        res = MGA_API_ALREADY_IDLE;

    unlock();

    return res;
}

MGA_API_RESULT mgaGetTodaysOfflineData(const tm* pTime, UBX_U1* pOfflineData, UBX_I4 offlineDataSize, UBX_U1** ppTodaysData, UBX_I4* pTodaysDataSize)
{
    assert(ppTodaysData);
    assert(pTodaysDataSize);
    assert(pOfflineData);
    assert(offlineDataSize);

    int curYear = pTime->tm_year + 1900;
    int curMonth = pTime->tm_mon + 1;
    int curDay = pTime->tm_mday;

    *ppTodaysData = NULL;
    *pTodaysDataSize = 0;

    UBX_U4 todaysSize = 0;
    UBX_U4 totalSize = 0;
    UBX_U1* pMgaData = pOfflineData;

    while(totalSize < (UBX_U4)offlineDataSize)
    {
        if ((pMgaData[0] == UBX_SIG_PREFIX_1) && (pMgaData[1] == UBX_SIG_PREFIX_2))
        {
            // UBX message
            UBX_U4 payloadSize = pMgaData[4] + (pMgaData[5] << 8);
            UBX_U4 msgSize = payloadSize + UBX_MSG_FRAME_SIZE;

            if (isAnoMatch(pMgaData, curYear, curMonth, curDay))
            {
                todaysSize += msgSize;
            }
            pMgaData += msgSize;
            totalSize += msgSize;
        }
        else
        {
            assert(0);
            break;
        }
    }

    if (todaysSize == 0)
    {
        return MGA_API_NO_DATA_TO_SEND;
    }

    UBX_U1* pTodaysData = (UBX_U1*) malloc(todaysSize);
    if (pTodaysData)
    {
        *ppTodaysData = pTodaysData;
        *pTodaysDataSize  = (UBX_I4) todaysSize;

        totalSize = 0;
        pMgaData = pOfflineData;

        while(totalSize < (UBX_U4) offlineDataSize)
        {
            if ((pMgaData[0] == UBX_SIG_PREFIX_1) && (pMgaData[1] == UBX_SIG_PREFIX_2))
            {
                // UBX message
                UBX_U4 payloadSize = pMgaData[4] + (pMgaData[5] << 8);
                UBX_U4 msgSize = payloadSize + UBX_MSG_FRAME_SIZE;

                if (isAnoMatch(pMgaData, curYear, curMonth, curDay))
                {
                    memcpy(pTodaysData, pMgaData, msgSize);
                    pTodaysData += msgSize;
                }

                pMgaData += msgSize;
                totalSize += msgSize;
            }
            else
            {
                assert(0);
                break;
            }
        }
    }
    return MGA_API_OK;
}

MGA_API_RESULT mgaStartLegacyAiding(UBX_U1* pAidingData, UBX_I4 iSize)
{
    assert(pAidingData != NULL);
    assert(iSize > 0);

    MGA_API_RESULT res = MGA_API_OK;

    lock();

    // Allocate a new aiding file Id
    time_t t = time(NULL);
    srand((int)t);
    s_alpfileId = (UBX_U2) ((0xffff * rand() / RAND_MAX) + 1);

    assert(s_pAidingData == NULL);
    s_pAidingData = pAidingData;
    s_aidingDataSize = iSize;

    // Activate the AID-ALP message
    UBX_U1 enableAidALP[] = { UBX_SIG_PREFIX_1, UBX_SIG_PREFIX_2,
                              UBX_CLASS_CFG, UBX_CFG_MSG,
                              0x08, 0x00,
                              UBX_CLASS_AID, UBX_AID_ALPSRV,
                              0x01, 0x01, 0x01, 0x01, 0x01, 0x00,
                              0x00, 0x00};

    addChecksum(&enableAidALP[2], sizeof(enableAidALP) - 4);
    assert(validChecksum(&enableAidALP[2], sizeof(enableAidALP) - 4));

    s_pEvtInterface->evtWriteDevice(s_pCallbackContext, enableAidALP, sizeof(enableAidALP));

    // Build the ALP header to inform the receiver there is new ALP data
    UBX_U1 aidingStartMsg[sizeof(UbxMsgHeader) + sizeof(LegacyAidingRequestHeader) + sizeof(LegacyAidingDataHeader) + 2] = { 0 };

    UbxMsgHeader *pMsgHeader = (UbxMsgHeader *) aidingStartMsg;
    pMsgHeader->header1 = UBX_SIG_PREFIX_1;
    pMsgHeader->header2 = UBX_SIG_PREFIX_2;
    pMsgHeader->msgClass = UBX_CLASS_AID;
    pMsgHeader->msgId = UBX_AID_ALPSRV;
    pMsgHeader->payloadLength = sizeof(aidingStartMsg) - UBX_MSG_FRAME_SIZE;

    LegacyAidingRequestHeader *pRqstId = (LegacyAidingRequestHeader*) (aidingStartMsg + sizeof(UbxMsgHeader));
    pRqstId->idSize = sizeof(LegacyAidingRequestHeader);
    pRqstId->type = 1;
    pRqstId->ofs = 0;
    pRqstId->size = sizeof(LegacyAidingDataHeader) / 2;       // Size in words
    pRqstId->fileId = s_alpfileId;
    pRqstId->dataSize = sizeof(LegacyAidingDataHeader);

    memcpy(&pRqstId[1], s_pAidingData, sizeof(LegacyAidingDataHeader));

    addChecksum(&aidingStartMsg[2], sizeof(aidingStartMsg) - 4);
    assert(validChecksum(&aidingStartMsg[2], sizeof(aidingStartMsg) - 4));

    // Send new ALP data message
    s_pEvtInterface->evtWriteDevice(s_pCallbackContext, aidingStartMsg, sizeof(aidingStartMsg));

    s_aidingSrvActive = true;

    s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_LEGACY_AIDING_SERVER_STARTED,
                                s_pCallbackContext,
                                pRqstId,
                                sizeof(LegacyAidingRequestHeader) + sizeof(LegacyAidingDataHeader));
    unlock();

    return res;
}

MGA_API_RESULT mgaStopLegacyAiding(void)
{
    MGA_API_RESULT res = MGA_API_OK;

    lock();

    s_aidingSrvActive = false;

    // Switch off the AID-ALP message
    UBX_U1 disableAidALP[] = { UBX_SIG_PREFIX_1, UBX_SIG_PREFIX_2,
                              UBX_CLASS_CFG, UBX_CFG_MSG,
                              0x03, 0x00,
                              UBX_CLASS_AID, UBX_AID_ALPSRV,
                              0x00,
                              0x00, 0x00};

    addChecksum(&disableAidALP[2], sizeof(disableAidALP) - 4);
    assert(validChecksum(&disableAidALP[2], sizeof(disableAidALP) - 4));

    s_pEvtInterface->evtWriteDevice(s_pCallbackContext, disableAidALP, sizeof(disableAidALP));

    sessionStop(MGA_PROGRESS_EVT_LEGACY_AIDING_SERVER_STOPPED, NULL, 0);

    unlock();

    return res;
}

///////////////////////////////////////////////////////////////////////////////
// private functions

static MGA_API_RESULT getDataFromService(SOCKET iSock, const char* pRequest, UBX_U1** ppData, UBX_I4* piSize)
{
    // send the HTTP get request
    assert(s_pEvtInterface);

    if (s_pEvtInterface->evtProgress)
    {
        s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_REQUEST_HEADER, s_pCallbackContext, NULL, 0);
    }

    MGA_API_RESULT res = MGA_API_OK;
    size_t requestSize = strlen(pRequest);
    send(iSock, pRequest, requestSize, 0);

    // get reply
    char sData[0x2000];
    memset(sData, 0, sizeof(sData));
    getHttpHeader(iSock, sData, sizeof(sData));

    // search for HTTP header
    const char* pHttpTxt = "HTTP/";
    char* pHttpPos = strstr(sData, pHttpTxt);

    if (!pHttpPos)
    {
        // response header format error
        EvtInfoServiceError serviceErrorInfo;
        memset(&serviceErrorInfo, 0, sizeof(serviceErrorInfo));
        serviceErrorInfo.httpRc = 0;
        serviceErrorInfo.errorType = MGA_SERVICE_ERROR_NOT_HTTP_HEADER;

        if (s_pEvtInterface->evtProgress)
        {
            s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_SERVICE_ERROR, s_pCallbackContext, &serviceErrorInfo, sizeof(serviceErrorInfo));
        }

        res = MGA_API_CANNOT_GET_DATA;
    }

    // search for HTTP response code
    const char* pResponseCode = NULL;

    if (res == MGA_API_OK)
    {
        pResponseCode = nextToken(pHttpPos);

        if (!pResponseCode)
        {
            // response header format error
            EvtInfoServiceError serviceErrorInfo;
            memset(&serviceErrorInfo, 0, sizeof(serviceErrorInfo));
            serviceErrorInfo.errorType = MGA_SERVICE_ERROR_NO_RESPONSE_CODE;

            if (s_pEvtInterface->evtProgress)
            {
                s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_SERVICE_ERROR, s_pCallbackContext, &serviceErrorInfo, sizeof(serviceErrorInfo));
            }
            res =  MGA_API_CANNOT_GET_DATA;
        }
    }

    if (res == MGA_API_OK)
    {
        int rc = atoi(pResponseCode);
        if (rc != 200)
        {
            // extract response status text
            const char* pResponseStatus = nextToken(pResponseCode);
            const char* pEnd = strstr(pResponseStatus, "\n");

            EvtInfoServiceError serviceErrorInfo;
            memset(&serviceErrorInfo, 0, sizeof(serviceErrorInfo));
            serviceErrorInfo.errorType = MGA_SERVICE_ERROR_BAD_STATUS;
            serviceErrorInfo.httpRc = (UBX_U4) rc;

            size_t errorTxtSize = pEnd ? (size_t) (pEnd - pResponseStatus) : 0;
            size_t n = MIN(errorTxtSize, sizeof(serviceErrorInfo.errorMessage) - 1);

            strncpy(serviceErrorInfo.errorMessage, pResponseStatus, n);
            assert(strlen(serviceErrorInfo.errorMessage) < sizeof(serviceErrorInfo.errorMessage) - 1);

            if (s_pEvtInterface->evtProgress)
            {
                s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_SERVICE_ERROR, s_pCallbackContext, &serviceErrorInfo, sizeof(serviceErrorInfo));
            }
            res = MGA_API_CANNOT_GET_DATA;
        }
    }

    // search for HTTP content-length
    const char* pLength = NULL;
    const char* pContentLenTxt = "Content-Length: ";
    const size_t contentLenSize = strlen(pContentLenTxt);

    if (res == MGA_API_OK)
    {
        pLength = strstr(sData, pContentLenTxt);

        if (!pLength)
        {
            // no length
            EvtInfoServiceError serviceErrorInfo;
            memset(&serviceErrorInfo, 0, sizeof(serviceErrorInfo));
            serviceErrorInfo.errorType = MGA_SERVICE_ERROR_NO_LENGTH;
            serviceErrorInfo.httpRc = 0;

            if (s_pEvtInterface->evtProgress)
            {
                s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_SERVICE_ERROR, s_pCallbackContext, &serviceErrorInfo, sizeof(serviceErrorInfo));
            }

            res = MGA_API_CANNOT_GET_DATA;
        }
    }

    size_t contentLength = 0;
    if (res == MGA_API_OK)
    {
        assert(pLength);
        pLength += contentLenSize;
        
        contentLength = (size_t) atoi(pLength);

        if (!contentLength)
        {
            // content length is 0
            EvtInfoServiceError serviceErrorInfo;
            memset(&serviceErrorInfo, 0, sizeof(serviceErrorInfo));
            serviceErrorInfo.errorType = MGA_SERVICE_ERROR_ZERO_LENGTH;
            serviceErrorInfo.httpRc = 0;
            strcpy(serviceErrorInfo.errorMessage, "Data length is 0");
            assert(strlen(serviceErrorInfo.errorMessage) < sizeof(serviceErrorInfo.errorMessage) - 1);

            if (s_pEvtInterface->evtProgress)
            {
                s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_SERVICE_ERROR, s_pCallbackContext, &serviceErrorInfo, sizeof(serviceErrorInfo));
            }
            res = MGA_API_CANNOT_GET_DATA;
        }
    }

    const char* pContentTypeTxt = "Content-Type: ";
    const size_t contentTypeSize = strlen(pContentTypeTxt);
    const char* pContentType = strstr(sData, pContentTypeTxt);

    if (!pContentType)
    {
        EvtInfoServiceError serviceErrorInfo;
        memset(&serviceErrorInfo, 0, sizeof(serviceErrorInfo));
        serviceErrorInfo.errorType = MGA_SERVICE_ERROR_NO_CONTENT_TYPE;
        serviceErrorInfo.httpRc = 0;

        if (s_pEvtInterface->evtProgress)
        {
            s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_SERVICE_ERROR, s_pCallbackContext, &serviceErrorInfo, sizeof(serviceErrorInfo));
        }

        res = MGA_API_CANNOT_GET_DATA;
    }

    if (res == MGA_API_OK)
    {
        // check if its a UBX server
        if (strncasecmp(pContentType + contentTypeSize, "application/ubx", 15) != 0)
        {
            EvtInfoServiceError serviceErrorInfo;
            memset(&serviceErrorInfo, 0, sizeof(serviceErrorInfo));
            serviceErrorInfo.errorType = MGA_SERVICE_ERROR_NOT_UBX_CONTENT;
            serviceErrorInfo.httpRc = 0;
            strcpy(serviceErrorInfo.errorMessage, "Content type not UBX");
            assert(strlen(serviceErrorInfo.errorMessage) < sizeof(serviceErrorInfo.errorMessage) - 1);

            if (s_pEvtInterface->evtProgress)
            {
                s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_SERVICE_ERROR, s_pCallbackContext, &serviceErrorInfo, sizeof(serviceErrorInfo));
            }

            res = MGA_API_CANNOT_GET_DATA;
        }
    }

    char* pBuffer = NULL;
    if (res == MGA_API_OK)
    {
        // allocate buffer to receiver data from service
        // this buffer will be passed to the client, who will ultimately free it
        pBuffer = (char*) malloc(contentLength);
        if (pBuffer == NULL)
        {
            res = MGA_API_OUT_OF_MEMORY;
        }
    }

    if (res == MGA_API_OK)
    {
        if (s_pEvtInterface->evtProgress)
        {
            s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_RETRIEVE_DATA, s_pCallbackContext, NULL, 0);
        }
        size_t received = (size_t) getData(iSock, pBuffer, contentLength);
        if (received != contentLength)
        {
            // did not retrieved all data
            EvtInfoServiceError serviceErrorInfo;
            memset(&serviceErrorInfo, 0, sizeof(serviceErrorInfo));
            serviceErrorInfo.errorType = MGA_SERVICE_ERROR_PARTIAL_CONTENT;
            serviceErrorInfo.httpRc = 0;

            if (s_pEvtInterface->evtProgress)
            {
                s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_SERVICE_ERROR, s_pCallbackContext, &serviceErrorInfo, sizeof(serviceErrorInfo));
            }

            // as there is an error, and all the data could not be retrieved, free the buffer and return an error code, so the client does not release the buffer again.
            free(pBuffer);
            res = MGA_API_CANNOT_GET_DATA;
        }
        else
        {
            *ppData = (UBX_U1*) pBuffer;
            *piSize = (UBX_I4) contentLength;
        }
    }
    return res;
}

static SOCKET connectServer(const char* strServer, UBX_U2 wPort, int sockType, int sckProtocol)
{
    // compose server name with port
    size_t iSize = (strlen(strServer) + 6 + 1) * sizeof(char); // len of server string + ':' + largest port number (65535) + null
    char* serverString = (char*) malloc(iSize);
    if (serverString == NULL)
    {
        return INVALID_SOCKET;
    }

    memset(serverString, 0, iSize);
    sprintf(serverString, "%s:%hu", strServer, wPort);

    if (s_pEvtInterface->evtProgress)
    {
        s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_SERVER_CONNECTING, s_pCallbackContext, (const void*)serverString, (UBX_I4) strlen(serverString) + 1);
    }

    if (strlen(strServer) == 0)
    {
        if (s_pEvtInterface->evtProgress)
        {
            s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_UNKNOWN_SERVER, s_pCallbackContext, (const void*)serverString, (UBX_I4) strlen(serverString) + 1);
        }
        free(serverString);
        return INVALID_SOCKET;
    }

    struct sockaddr_in server;
    // create the socket address of the server, it consists of type, IP address and port number
    memset(&server, 0, sizeof (server));
    unsigned long addr = inet_addr(strServer);

    if (addr != INADDR_NONE)  // numeric IP address
    {
        memcpy(&server.sin_addr, &addr, sizeof(addr));
    }
    else
    {
        struct hostent* host_info = gethostbyname(strServer);

        if (host_info != NULL)
        {
            memcpy(&server.sin_addr, host_info->h_addr, (size_t) host_info->h_length);
        }
        else
        {
            if (s_pEvtInterface->evtProgress)
            {
                s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_UNKNOWN_SERVER, s_pCallbackContext, (const void*)serverString, (UBX_I4) strlen(serverString) + 1);
            }
            free(serverString);
            return INVALID_SOCKET;
        }
    }

    server.sin_family = AF_INET;
    server.sin_port   = htons( wPort );

    // create the socket and connect
    SOCKET sock = socket(AF_INET, sockType, sckProtocol);
    if (sock != INVALID_SOCKET)
    {
        bool connFailed = false;
#ifndef WIN32
        // Make the socket non-blocking for the connect call
        // to avoid connect hanging up that can be observed on some platforms
        connFailed = (fcntl(sock, F_SETFL, SOCK_NONBLOCK) != 0);
#endif // WIN32

        // set up the connection to the server
        if (!connFailed)
        {
            connFailed = (connect(sock, (struct sockaddr*)&server, sizeof(server)) == SOCKET_ERROR);
#ifndef WIN32
            if (connFailed && errno == EINPROGRESS)
            {
                // Wait for connection (for 10 seconds)
                fd_set fdset;
                FD_ZERO(&fdset);
                FD_SET(sock, &fdset);
                struct timeval selTimeout;
                selTimeout.tv_sec = 10;
                selTimeout.tv_usec = 0;
                int soerr;
                socklen_t solen = sizeof(soerr);
                // Is the socket writeable (ready) now?
                if (select(sock + 1, NULL, &fdset, NULL, &selTimeout) == 1
                 && !getsockopt(sock, SOL_SOCKET, SO_ERROR, &soerr, &solen)
                 && soerr == 0)
                {
                    // Go into non-blocking mode again
                    const int flags = fcntl(sock, F_GETFL, 0);
                    connFailed = (fcntl(sock, F_SETFL, flags ^ SOCK_NONBLOCK) != 0);
                }
                else
                {
                    // An error occurred. Report, clean up and exit
                    connFailed = true;
                }
            }
#endif // WIN32
        }

        if(connFailed)
        {
            if (s_pEvtInterface->evtProgress)
            {
                s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_SERVER_CANNOT_CONNECT, s_pCallbackContext, (const void*)serverString, (UBX_I4) strlen(serverString) + 1);
            }
        }
        else //success
        {
            if (s_pEvtInterface->evtProgress)
            {
                s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_SERVER_CONNECT, s_pCallbackContext, (const void*)serverString, (UBX_I4) strlen(serverString) + 1);
            }
            free(serverString);
            return sock;
        }

#ifdef WIN32
        closesocket(sock);
#else // WIN32
        close(sock);
#endif // WIN32
    }
    free(serverString);
    return INVALID_SOCKET;
}

static int getHttpHeader(SOCKET sock, char* buf, int cnt)
{
    int c = 0;
    char* p = buf;
    do
    {
        fd_set fdset;
        FD_ZERO(&fdset);
#ifdef WIN32
#  pragma warning(push)
#  pragma warning( disable : 4127 )
#endif // WIN32
        FD_SET(sock, &fdset);
#ifdef WIN32
#  pragma warning(pop)
#endif // WIN32

        struct timeval tv;
        tv.tv_sec  = s_serverResponseTimeout;
        tv.tv_usec = 0;

        if (select(sock+1, &fdset, NULL, NULL, &tv) > 0)
        {
            int b = recv(sock, p, 1, 0);
            if (b <= 0)
            {
                // failed or timeout
                break;
            }
            else if ((b > 0) && (*p != '\r'))
            {
                p ++;
                c ++;
            }
        }
        else
        {
            // no response
            break;
        }
    } while ((c < cnt) && ((c < 2) || (p[-2] != '\n') || (p[-1] != '\n')));
    *p = '\0';

    return c;
}

static void lock(void)
{
#ifdef WIN32
    EnterCriticalSection(&s_mgaLock);
#else // WIN32
    pthread_mutex_lock(&s_mgaLock);
#endif // WIN32
}

static void unlock(void)
{
#ifdef WIN32
    LeaveCriticalSection(&s_mgaLock);
#else // WIN32
    pthread_mutex_unlock(&s_mgaLock);
#endif // WIN32
}

static MGA_API_RESULT handleAidAckMsg(int ackType)
{
    // do not lock here - lock must already be in place
    assert(s_pFlowConfig->mgaFlowControl != MGA_FLOW_SMART);
    assert(s_pLastMsgSent != NULL);

    if (s_pFlowConfig->mgaFlowControl == MGA_FLOW_NONE)
    {
        // no flow control, so ignore ACK/NAKs
        return MGA_API_IGNORED_MSG;
    }

    MGA_API_RESULT res = MGA_API_IGNORED_MSG;
    bool continueAckProcessing = false;

    switch (ackType)
    {
    case UBX_ACK_NAK:  // NAK for last AID message
        s_ackCount++;
        s_pLastMsgSent->state = MGA_MSG_FAILED;
        s_pLastMsgSent->mgaFailedReason = MGA_FAILED_REASON_CODE_NOT_SET;

        if (s_pEvtInterface->evtProgress)
        {
            s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_MSG_TRANSFER_FAILED, s_pCallbackContext, s_pLastMsgSent, sizeof(MgaMsgInfo));
        }

        continueAckProcessing = true;
        break;

    case UBX_ACK_ACK:  // ACK for last AID message
        s_ackCount++;
        s_pLastMsgSent->state = MGA_MSG_RECEIVED;

        if (s_pEvtInterface->evtProgress)
        {
            s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_MSG_TRANSFER_COMPLETE, s_pCallbackContext, s_pLastMsgSent, sizeof(MgaMsgInfo));
        }

        continueAckProcessing = true;
        break;

    default:
        assert(false);
        break;
    }

    if (continueAckProcessing)
    {
        if (s_ackCount == s_mgaBlockCount)
        {
            // last ACK received
            sessionStop(MGA_PROGRESS_EVT_FINISH, NULL, 0);
        }
        else
        {
            // not last ACK, can send another message if there is still some to send
            if (s_messagesSent < s_mgaBlockCount)
            {
                sendNextMgaMessage();
            }
        }
        res = MGA_API_OK;
    }

    return res;
}

static MGA_API_RESULT handleMgaAckMsg(const UBX_U1* pPayload)
{
    // do not lock here - lock must already be in place
    if (s_pFlowConfig->mgaFlowControl == MGA_FLOW_NONE)
    {
        // no flow control, so ignore ACK/NAKs
        return MGA_API_IGNORED_MSG;
    }

    if (s_pLastMsgSent == NULL)
    {
        // no message in flow
        return MGA_API_IGNORED_MSG;
    }

    MGA_API_RESULT res = MGA_API_IGNORED_MSG;
    MGA_ACK_TYPES type = (MGA_ACK_TYPES) pPayload[0];
    UBX_U1 msgId = pPayload[3];
    const UBX_U1* pMgaHeader = &pPayload[4];

    bool continueAckProcessing = false;

    switch(type)
    {
    case MGA_ACK_MSG_NAK:
        {
            // NAK - report NAK & carry on
            MgaMsgInfo* pAckMsg = findMsgBlock(msgId, pMgaHeader);

            if (pAckMsg)
            {
                s_ackCount++;
                pAckMsg->state  = MGA_MSG_FAILED;
                pAckMsg->mgaFailedReason = (MGA_FAILED_REASON) pPayload[2];

                if (s_pEvtInterface->evtProgress)
                {
                    s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_MSG_TRANSFER_FAILED, s_pCallbackContext, pAckMsg, sizeof(MgaMsgInfo));
                }

                continueAckProcessing = true;
            }
        }
        break;

    case MGA_ACK_MSG_ACK:
        {
            // ACK
            MgaMsgInfo* pAckMsg = findMsgBlock(msgId, pMgaHeader);

            if (pAckMsg)
            {
                // ACK is for an outstanding transmitted message
                s_ackCount++;
                pAckMsg->state = MGA_MSG_RECEIVED;

                if (s_pEvtInterface->evtProgress)
                {
                    s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_MSG_TRANSFER_COMPLETE, s_pCallbackContext, pAckMsg, sizeof(MgaMsgInfo));
                }

                continueAckProcessing = true;
            }
        }
        break;

    default:
        // ignored
        break;
    }

    if (continueAckProcessing)
    {
        if (s_ackCount == s_mgaBlockCount)
        {
            // last ACK received
            sessionStop(MGA_PROGRESS_EVT_FINISH, NULL, 0);
        }
        else
        {
            // not last ACK, can send another message if there is still some to send
            if (s_messagesSent < s_mgaBlockCount)
            {
                sendNextMgaMessage();
            }
        }
        res = MGA_API_OK;
    }

    return res;
}

static MGA_API_RESULT handleFlashAckMsg(const UBX_U1* pPayload)
{
    // do not lock here - lock must already be in place
    UBX_U1 type = pPayload[0];
    UBX_U1 typeVersion = pPayload[1];
    UBX_U1 ackType =  pPayload[2];
    UBX_U2 sequence = pPayload[4] + (pPayload[5] << 8);

    (void) typeVersion; // unreferenced

    if (type != 3)
    {
        // not a UBX-MGA-FLASH-ACK message
        return MGA_API_IGNORED_MSG;
    }

    // it is a UBX-MGA-FLASH-ACK message
    MGA_API_RESULT res = MGA_API_OK;

    assert(typeVersion == 0);

    switch(ackType)
    {
    case 0: // ACK
        if (sequence == 0xFFFF)
        {
            // ACK for stop message
            sessionStop(MGA_PROGRESS_EVT_FINISH, NULL, 0);
        }
        else
        {
            if ((s_flashMessagesSent < s_mgaFlashBlockCount) &&
                (s_pLastFlashBlockSent->sequenceNumber == sequence))
            {
                sendMgaFlashBlock(true);
            }
            else
            {
                // ACK not for outstanding flash message
                EVT_TERMINATION_REASON stopReason = TERMINATE_PROTOCOL_ERROR;
                sessionStop(MGA_PROGRESS_EVT_TERMINATED, &stopReason, sizeof(stopReason));
            }
        }
        break;

    case 1:     // NAK - retry
        sendMgaFlashBlock(false);
        break;

    case 2:     // NAK - give up
        {
            // report giving up
            EVT_TERMINATION_REASON stopReason = TERMINATE_RECEIVER_NAK;
            sessionStop(MGA_PROGRESS_EVT_TERMINATED, &stopReason, sizeof(stopReason));
        }
        break;

    default:
        assert(0);
        break;
    }

    return res;
}

static MGA_API_RESULT handleAidFlashAckMsg(void)
{
    // do not lock here - locks must already be in place
    MGA_API_RESULT res = MGA_API_OK;

    switch (s_aidState)
    {
    case LEGACY_AIDING_STARTING:
        s_aidState = LEGACY_AIDING_MAIN_SEQ;

        if (s_pEvtInterface->evtProgress)
        {
            s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_LEGACY_AIDING_STARTUP_COMPLETED,
                                        s_pCallbackContext,
                                        NULL,
                                        MGA_PROGRESS_EVT_LEGACY_AIDING_FINALIZE_START);
        }

        // Send next aiding flash block
        sendFlashMainSeqBlock();
        break;

    case LEGACY_AIDING_MAIN_SEQ:
        assert(s_mgaFlashBlockCount > s_flashMessagesSent);
        // Send next aiding flash block
        sendFlashMainSeqBlock();
        break;

    case LEGACY_AIDING_STOPPING:
        s_aidState = LEGACY_AIDING_IDLE;
        if (s_pEvtInterface->evtProgress)
        {
            s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_LEGACY_AIDING_FINALIZE_COMPLETED,
                                        s_pCallbackContext,
                                        NULL,
                                        0);
        }
        sessionStop(MGA_PROGRESS_EVT_FINISH, NULL, 0);
        break;

    default:
        assert(false);
        break;
    }

    return res;
}

static MGA_API_RESULT handleAidingResponseMsg(const UBX_U1* pMessageData)
{
    MGA_API_RESULT res = MGA_API_IGNORED_MSG;

    // Look for aiding ack/nak
    const UBX_U1 Ack [] = { UBX_SIG_PREFIX_1, UBX_SIG_PREFIX_2, 0x0B, 0x50, 0x01, 0x00, 0x01, 0x5D, 0x7B };
    const UBX_U1 Nak [] = { UBX_SIG_PREFIX_1, UBX_SIG_PREFIX_2, 0x0B, 0x50, 0x01, 0x00, 0x00, 0x5C, 0x7A };

    if (memcmp(pMessageData, Ack, UBX_AID_ALP_ACK_SIZE) == 0)
    {
        res = handleAidFlashAckMsg();
    }
    else if (memcmp(pMessageData, Nak, UBX_AID_ALP_ACK_SIZE) == 0)
    {
        if (s_aidState == LEGACY_AIDING_STARTING)
        {
            // The quirky nature of legacy aiding means that a NAK here needs to be treated as an ACK
            res = handleAidFlashAckMsg();
        }
        else
        {
            res = handleAidFlashNakMsg();
        }
    }

    return res;
}

static MGA_API_RESULT handleAidFlashNakMsg(void)
{
    // do not lock here - locks must already be in place
    EVT_TERMINATION_REASON stopReason = TERMINATE_RECEIVER_NAK;
    sessionStop(MGA_PROGRESS_EVT_TERMINATED, &stopReason, sizeof(stopReason));

    return MGA_API_OK;
}

static void handleLegacyAidingTimeout(void)
{
    // do not lock here - locks must already be in place

    assert(s_bLegacyAiding);
    time_t now = time(NULL);

    switch (s_aidState)
    {
    case LEGACY_AIDING_STARTING:
        if (now > s_aidingTimeout)
        {
            assert(s_pEvtInterface);
            if (s_pEvtInterface->evtProgress)
            {
                UBX_I4 reason  = MGA_FAILED_REASON_LEGACY_NO_ACK;
                s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_LEGACY_AIDING_STARTUP_FAILED, s_pCallbackContext, &reason, sizeof(reason));
            }
            sendAidingFlashStop();
            sessionStop(MGA_PROGRESS_EVT_FINISH, NULL, 0);
        }
        break;

    case LEGACY_AIDING_MAIN_SEQ:
        assert(s_mgaFlashBlockCount > 0);
        assert(s_pLastFlashBlockSent != NULL);
        assert((s_pLastFlashBlockSent->state == MGA_MSG_WAITING_FOR_ACK) ||
               (s_pLastFlashBlockSent->state == MGA_MSG_WAITING_FOR_ACK_SECOND_CHANCE));

        if (now > s_pLastFlashBlockSent->timeOut)
        {
            if (s_pLastFlashBlockSent->state == MGA_MSG_WAITING_FOR_ACK)
            {
                // Send nudge byte to receiver
                s_pLastFlashBlockSent->state = MGA_MSG_WAITING_FOR_ACK_SECOND_CHANCE;
                UBX_U1 flashDataMsg = 0;
                s_pEvtInterface->evtWriteDevice(s_pCallbackContext, &flashDataMsg, sizeof(flashDataMsg));
            }
            else
            {
                s_pLastFlashBlockSent->state = MGA_MSG_FAILED;
                s_pLastFlashBlockSent->mgaFailedReason = MGA_FAILED_REASON_LEGACY_NO_ACK;

                assert(s_pEvtInterface);
                if (s_pEvtInterface->evtProgress)
                {
                    s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_LEGACY_AIDING_FLASH_BLOCK_FAILED, s_pCallbackContext, s_pLastFlashBlockSent, sizeof(MgaMsgInfo));
                }
                sendAidingFlashStop();
                sessionStop(MGA_PROGRESS_EVT_FINISH, NULL, 0);
            }
        }
        break;

    case LEGACY_AIDING_STOPPING:
        if (now > s_aidingTimeout)
        {
            // Give up waiting for 'stop' ack
            if (s_pEvtInterface->evtProgress)
            {
                s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_LEGACY_AIDING_FINALIZE_FAILED, s_pCallbackContext, NULL, 0);
            }
            sessionStop(MGA_PROGRESS_EVT_FINISH, NULL, 0);
        }
        break;

    case LEGACY_AIDING_IDLE:
        // Nothing to check
        break;

    default:
        assert(false);
        break;
    }
}

static void sendMgaFlashBlock(bool next)
{
    // do not lock here - locks must already be in place
    bool terminated = false;

    if (s_pLastFlashBlockSent == NULL)
    {
        // 1st message to send
        assert(next == true);
        assert(s_pMgaFlashBlockList);
        assert(s_flashMessagesSent == 0);
        s_pLastFlashBlockSent = &s_pMgaFlashBlockList[0];
    }
    else
    {
        if (next)
        {
            if (s_flashMessagesSent < s_mgaFlashBlockCount)
            {
                // move on to next block is possible
                // mark last block sent as successful - report success
                s_pLastFlashBlockSent->state = MGA_MSG_RECEIVED;

                if (s_pEvtInterface->evtProgress)
                {
                    s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_MSG_TRANSFER_COMPLETE, s_pCallbackContext, s_pLastFlashBlockSent, sizeof(MgaMsgInfo));
                }

                // move next message
                s_pLastFlashBlockSent++;
                s_flashMessagesSent++;
            }
            else
            {
                assert(0);
                // shouldn't happen
                terminated = true;
            }
        }
        else
        {
            // retry
            s_pLastFlashBlockSent->retryCount++;
            if (s_pLastFlashBlockSent->retryCount > s_pFlowConfig->msgRetryCount)
            {
                // too many retries - give up
                s_pLastFlashBlockSent->state = MGA_MSG_FAILED;

                // report failed block
                if (s_pEvtInterface->evtProgress)
                {
                    s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_MSG_TRANSFER_FAILED, s_pCallbackContext, s_pLastFlashBlockSent, sizeof(MgaMsgInfo));
                }

                terminated = true;
                s_mgaFlashBlockCount = s_flashMessagesSent; // force stop
            }
        }
    }

    if (terminated)
    {
        // report giving up
        EVT_TERMINATION_REASON stopReason = TERMINATE_RECEIVER_NOT_RESPONDING;
        sessionStop(MGA_PROGRESS_EVT_TERMINATED, &stopReason, sizeof(stopReason));
    }
    else if (s_flashMessagesSent >= s_mgaFlashBlockCount)
    {
        // all data messages sent.
        // now send message to tell receiver there is no more data
        sendFlashStop();
    }
    else
    {
        // generate event to send next message
        assert(s_pEvtInterface);
        assert(s_pEvtInterface->evtWriteDevice);

        assert(sizeof(FlashDataMsgHeader) == 12);
        UBX_U1 flashDataMsg[sizeof(FlashDataMsgHeader) + FLASH_DATA_MSG_PAYLOAD + 2];

        FlashDataMsgHeader* pflashDataMsgHeader = (FlashDataMsgHeader*) flashDataMsg;

        pflashDataMsgHeader->header1 = UBX_SIG_PREFIX_1;
        pflashDataMsgHeader->header2 = UBX_SIG_PREFIX_2;
        pflashDataMsgHeader->msgClass = UBX_CLASS_MGA;
        pflashDataMsgHeader->msgId = UBX_MGA_FLASH;
        pflashDataMsgHeader->payloadLength = 6 + s_pLastFlashBlockSent->msgSize;

        // UBX-MGA-FLASH-DATA message
        pflashDataMsgHeader->type = 1;
        pflashDataMsgHeader->typeVersion = 0;
        pflashDataMsgHeader->sequence = s_flashSequence;
        pflashDataMsgHeader->payloadCount = s_pLastFlashBlockSent->msgSize;

        size_t flashMsgTotalSize = sizeof(FlashDataMsgHeader);
        UBX_U1* pFlashDataPayload = flashDataMsg + sizeof(FlashDataMsgHeader);
        memcpy(pFlashDataPayload, s_pLastFlashBlockSent->pMsg, s_pLastFlashBlockSent->msgSize);
        flashMsgTotalSize += s_pLastFlashBlockSent->msgSize;
        assert(flashMsgTotalSize == s_pLastFlashBlockSent->msgSize + sizeof(FlashDataMsgHeader));

        addChecksum(&pflashDataMsgHeader->msgClass, flashMsgTotalSize - 2);
        flashMsgTotalSize += 2;
        assert(validChecksum(&flashDataMsg[2], flashMsgTotalSize - 4));

        s_flashSequence++;
        s_pEvtInterface->evtWriteDevice(s_pCallbackContext, flashDataMsg, (UBX_I4) flashMsgTotalSize);

        s_pLastFlashBlockSent->state = MGA_MSG_WAITING_FOR_ACK;
        s_pLastFlashBlockSent->timeOut = (s_pFlowConfig->msgTimeOut / 1000) + time(NULL);

        if (s_pEvtInterface->evtProgress)
        {
            s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_MSG_SENT, s_pCallbackContext, s_pLastFlashBlockSent, sizeof(MgaMsgInfo));
        }
    }
}

static void sendFlashMainSeqBlock(void)
{
    // do not lock here - locks must already be in place
    bool terminated = false;

    if (s_pLastFlashBlockSent == NULL)
    {
        // 1st message to send
        assert(s_pMgaFlashBlockList);
        assert(s_flashMessagesSent == 0);
        s_pLastFlashBlockSent = &s_pMgaFlashBlockList[0];
    }
    else
    {
        if (s_flashMessagesSent < s_mgaFlashBlockCount)
        {
            // move on to next block is possible
            // mark last block sent as successful - report success
            s_pLastFlashBlockSent->state = MGA_MSG_RECEIVED;

            if (s_pEvtInterface->evtProgress)
            {
                s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_LEGACY_AIDING_FLASH_BLOCK_COMPLETE, s_pCallbackContext, s_pLastFlashBlockSent, sizeof(MgaMsgInfo));
            }

            // move next message
            s_pLastFlashBlockSent++;
            s_flashMessagesSent++;
        }
        else
        {
            assert(0);
            // shouldn't happen
            terminated = true;
        }
    }

    if (terminated)
    {
        // report giving up
        EVT_TERMINATION_REASON stopReason = TERMINATE_RECEIVER_NOT_RESPONDING;
        sessionStop(MGA_PROGRESS_EVT_TERMINATED, &stopReason, sizeof(stopReason));
    }
    else if (s_flashMessagesSent >= s_mgaFlashBlockCount)
    {
        // All data messages sent.
        // Main sequence finished. Next needs to be stop block
        s_aidState = LEGACY_AIDING_STOPPING;
        if (s_pEvtInterface->evtProgress)
        {
            s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_LEGACY_AIDING_FINALIZE_START,
                                        s_pCallbackContext,
                                        NULL,
                                        0);
        }
        sendAidingFlashStop();        // now send message to tell receiver there is no more data
    }
    else
    {
        // Generate event to send next message
        assert(s_pEvtInterface);
        assert(s_pEvtInterface->evtWriteDevice);

        assert(sizeof(UbxMsgHeader) == 6);
        UBX_U1 aidingFlashDataMsg[sizeof(UbxMsgHeader) + FLASH_DATA_MSG_PAYLOAD + 2];

        UbxMsgHeader* pflashDataMsgHeader = (UbxMsgHeader*) aidingFlashDataMsg;

        pflashDataMsgHeader->header1 = UBX_SIG_PREFIX_1;
        pflashDataMsgHeader->header2 = UBX_SIG_PREFIX_2;
        pflashDataMsgHeader->msgClass = UBX_CLASS_AID;
        pflashDataMsgHeader->msgId = UBX_AID_ALP;
        pflashDataMsgHeader->payloadLength = s_pLastFlashBlockSent->msgSize;

        size_t flashMsgTotalSize = sizeof(UbxMsgHeader);
        UBX_U1* pFlashDataPayload = aidingFlashDataMsg + sizeof(UbxMsgHeader);
        memcpy(pFlashDataPayload, s_pLastFlashBlockSent->pMsg, s_pLastFlashBlockSent->msgSize);

        flashMsgTotalSize += s_pLastFlashBlockSent->msgSize;
        assert(flashMsgTotalSize == s_pLastFlashBlockSent->msgSize + sizeof(UbxMsgHeader));
        flashMsgTotalSize += 2; // Add checksum length

        addChecksum(&pflashDataMsgHeader->msgClass, flashMsgTotalSize - 4);
        assert(validChecksum(&aidingFlashDataMsg[2], flashMsgTotalSize - 4));

        s_pEvtInterface->evtWriteDevice(s_pCallbackContext, aidingFlashDataMsg, (UBX_I4) flashMsgTotalSize);

        s_pLastFlashBlockSent->state = MGA_MSG_WAITING_FOR_ACK;
        s_pLastFlashBlockSent->timeOut = (s_pFlowConfig->msgTimeOut / 1000) + time(NULL);

        if (s_pEvtInterface->evtProgress)
        {
            s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_LEGACY_AIDING_FLASH_BLOCK_SENT, s_pCallbackContext, s_pLastFlashBlockSent, sizeof(MgaMsgInfo));
        }
    }
}

static void sendEmptyFlashBlock(void)
{
    UBX_U1 emptyFlashMsg[14] = { UBX_SIG_PREFIX_1, UBX_SIG_PREFIX_2,
                                UBX_CLASS_MGA, UBX_MGA_FLASH,
                                0x06, 0x00,
                                0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
                                0x00,0x00};

    addChecksum(&emptyFlashMsg[2], sizeof(emptyFlashMsg) - 4);
    assert(validChecksum(&emptyFlashMsg[2], sizeof(emptyFlashMsg) - 4));
    s_pEvtInterface->evtWriteDevice(s_pCallbackContext, emptyFlashMsg, sizeof(emptyFlashMsg));
}

static void sendFlashStop(void)
{
    UBX_U1 flashStopMsg[10] = { UBX_SIG_PREFIX_1, UBX_SIG_PREFIX_2,
                                UBX_CLASS_MGA, UBX_MGA_FLASH,
                                0x02, 0x00,
                                0x02, 0x00,
                                0x00, 0x00 };

    addChecksum(&flashStopMsg[2], sizeof(flashStopMsg) - 4);
    assert(validChecksum(&flashStopMsg[2], sizeof(flashStopMsg) - 4));
    s_pEvtInterface->evtWriteDevice(s_pCallbackContext, flashStopMsg, sizeof(flashStopMsg));
}

static void sendAidingFlashStop(void)
{
    UBX_U1 aidingFlashStopMsg[] = { UBX_SIG_PREFIX_1, UBX_SIG_PREFIX_2,
                                    UBX_CLASS_AID, UBX_AID_ALP,
                                    0x01, 0x00,
                                    0xFF,
                                    0x00, 0x00 };

    addChecksum(&aidingFlashStopMsg[2], sizeof(aidingFlashStopMsg) - 4);
    assert(validChecksum(&aidingFlashStopMsg[2], sizeof(aidingFlashStopMsg) - 4));
    s_pEvtInterface->evtWriteDevice(s_pCallbackContext, aidingFlashStopMsg, sizeof(aidingFlashStopMsg));

    s_aidingTimeout = (s_pFlowConfig->msgTimeOut / 1000) + time(NULL);
}

static void sendMgaIniTime(const MgaTimeAdjust* pTime)
{
    UBX_U1 mgaIniTimeMsg[24 + UBX_MSG_FRAME_SIZE] = {
                                UBX_SIG_PREFIX_1, UBX_SIG_PREFIX_2, // header
                                UBX_CLASS_MGA, UBX_MGA_INI,         // UBX-MGA-INI message
                                0x18, 0x00,                         // length (24 bytes)
                                0x10,                               // type
                                0x00,                               // version
                                0x00,                               // ref
                                0x80,                               // leapSecs - really -128
                                0x00, 0x00,                         // year
                                0x00,                               // month
                                0x00,                               // day
                                0x00,                               // hour
                                0x00,                               // minute
                                0x00,                               // second
                                0x00,                               // reserved2
                                0x00, 0x00, 0x00, 0x00,             // ns
                                0x02, 0x00,                         // tAccS
                                0x00, 0x00,                         // reserved3
                                0x00, 0x00, 0x00, 0x00,             // tAccNs
                                0x00, 0x00                          // checksum
    };

    mgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 4] = (UBX_U1) (pTime->mgaYear & 0xFF);
    mgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 5] = (UBX_U1) (pTime->mgaYear >> 8);
    mgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 6] = pTime->mgaMonth;
    mgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 7] = pTime->mgaDay;
    mgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 8] = pTime->mgaHour;
    mgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 9] = pTime->mgaMinute;
    mgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 10] = pTime->mgaSecond;
    mgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 16] = (UBX_U1) (pTime->mgaAccuracyS & 0xFF);
    mgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 17] = (UBX_U1) (pTime->mgaAccuracyS >> 8);

    UBX_U4 timeInNs = ((UBX_U4) pTime->mgaAccuracyMs) * MS_IN_A_NS;
    mgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 20] = (UBX_U1) timeInNs;
    mgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 21] = (UBX_U1) (timeInNs >> 8);
    mgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 22] = (UBX_U1) (timeInNs >> 16);
    mgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 23] = (UBX_U1) (timeInNs >> 24);

    assert(sizeof(mgaIniTimeMsg) == (UBX_MSG_FRAME_SIZE + 24));

    addChecksum(&mgaIniTimeMsg[2], sizeof(mgaIniTimeMsg) - 4);
    assert(validChecksum(&mgaIniTimeMsg[2], sizeof(mgaIniTimeMsg) - 4));
    s_pEvtInterface->evtWriteDevice(s_pCallbackContext, (UBX_U1*) mgaIniTimeMsg, sizeof(mgaIniTimeMsg));
}

static void sendCfgMgaAidAcks(bool enable)
{
    UBX_U1 cfgNavX5Msg[40 + UBX_MSG_FRAME_SIZE] = {
                                UBX_SIG_PREFIX_1, UBX_SIG_PREFIX_2, // header
                                0x06, 0x23,                         // UBX-CFG-NAVX5 message
                                0x28, 0x00,                         // length (40 bytes)
                                0x00, 0x00,                         // version
                                0x00, 0x00,                         // mask1
                                0x00, 0x00, 0x00, 0x00,             // mask2
                                0x00, 0x00,                         // reserved1
                                0x00,                               // minSVs
                                0x00,                               // maxSVs
                                0x00,                               // minCN0
                                0x00,                               // reserved2
                                0x00,                               // iniFix3D
                                0x00, 0x00,                         // reserved3
                                0x00,                               // ackAiding
                                0x00, 0x00,                         // wknRollover
                                0x00, 0x00, 0x00, 0x00, 0x00,0x00,  // reserved4
                                0x00,                               // usePPP
                                0x00,                               // aopCfg
                                0x00, 0x00,                         // reserved5
                                0x00, 0x00,                         // aopOrbMaxErr
                                0x00, 0x00, 0x00, 0x00,             // reserved6
                                0x00, 0x00, 0x00,                   // reserved7
                                0x00,                               // useAdr
                                0x00, 0x00                          // checksum
    };
    UBX_U1* pPayload = &cfgNavX5Msg[6];

    // offset 0 - 1 already set to zero
    pPayload[2] = 0x00;
    pPayload[3] = 0x04;                 // apply assistance acknowledgment settings
    // offset 4 - 16 already set to zero
    pPayload[17] = enable ? 1 : 0;      // issue acknowledgments for assistance message input
    // offset 18 - 36 already set to zero
    // offset 37 & 39 already set to zero - reserved

    assert(sizeof(cfgNavX5Msg) == (UBX_MSG_FRAME_SIZE + 40));

    addChecksum(&cfgNavX5Msg[2], sizeof(cfgNavX5Msg) - 4);
    assert(validChecksum(&cfgNavX5Msg[2], sizeof(cfgNavX5Msg) - 4));
    s_pEvtInterface->evtWriteDevice(s_pCallbackContext, cfgNavX5Msg, sizeof(cfgNavX5Msg));
}

static void sendAllMessages(void)
{
    s_pLastMsgSent = &s_pMgaMsgList[0];
    for(UBX_U4 i = 0; i < s_mgaBlockCount; i++)
    {
        assert(s_pEvtInterface);
        assert(s_pEvtInterface->evtWriteDevice);
        s_pEvtInterface->evtWriteDevice(s_pCallbackContext, s_pLastMsgSent->pMsg, s_pLastMsgSent->msgSize);
        s_pLastMsgSent->state = MGA_MSG_WAITING_FOR_ACK;

        if (s_pEvtInterface->evtProgress)
        {
            s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_MSG_SENT, s_pCallbackContext, s_pLastMsgSent, sizeof(MgaMsgInfo));
        }

        s_pLastMsgSent->state = MGA_MSG_RECEIVED;

        if (s_pEvtInterface->evtProgress)
        {
            s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_MSG_TRANSFER_COMPLETE, s_pCallbackContext, s_pLastMsgSent, sizeof(MgaMsgInfo));
        }

        s_pLastMsgSent++;
        s_messagesSent++;
    }

    // all done
    sessionStop(MGA_PROGRESS_EVT_FINISH, NULL, 0);
}

static UBX_I4 sendNextMgaMessage(void)
{
    // do not lock here - lock must already be in place
    assert(s_pFlowConfig->mgaFlowControl != MGA_FLOW_NONE);

    UBX_I4 msgSize = 0;
    if (s_pLastMsgSent == NULL)
    {
        // 1st message to send
        assert(s_pMgaMsgList);
        assert(s_messagesSent == 0);
        s_pLastMsgSent = &s_pMgaMsgList[0];
    }
    else
    {
        // move next message
        s_pLastMsgSent++;
        s_messagesSent++;
    }

    if (s_messagesSent < s_mgaBlockCount)
    {
        // generate event to send next message
        assert(s_pEvtInterface);
        assert(s_pEvtInterface->evtWriteDevice);
        msgSize = s_pLastMsgSent->msgSize;
        s_pEvtInterface->evtWriteDevice(s_pCallbackContext, s_pLastMsgSent->pMsg, msgSize);
        s_pLastMsgSent->state = MGA_MSG_WAITING_FOR_ACK;
        s_pLastMsgSent->timeOut = (s_pFlowConfig->msgTimeOut / 1000) + time(NULL);

        if (s_pEvtInterface->evtProgress)
        {
            s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_MSG_SENT, s_pCallbackContext, s_pLastMsgSent, sizeof(MgaMsgInfo));
        }
    }

    return msgSize;
}

static void resendMessage(MgaMsgInfo* pResendMsg)
{
    // do not lock here - lock must already be in place
    assert(pResendMsg->retryCount != 0);

    // generate event to resend message
    assert(s_pEvtInterface);
    assert(s_pEvtInterface->evtWriteDevice);
    s_pEvtInterface->evtWriteDevice(s_pCallbackContext, pResendMsg->pMsg, pResendMsg->msgSize);
    pResendMsg->state = MGA_MSG_WAITING_FOR_ACK;
    pResendMsg->timeOut = (s_pFlowConfig->msgTimeOut / 1000) + time(NULL);

    if (s_pEvtInterface->evtProgress)
    {
        s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_MSG_SENT, s_pCallbackContext, pResendMsg, sizeof(MgaMsgInfo));
    }
}

static MGA_API_RESULT countMgaMsg(const UBX_U1* pMgaData, UBX_I4 iSize, UBX_U4* piCount)
{
    assert(pMgaData);
    assert(piCount);
    assert(iSize);

    MGA_API_RESULT res = MGA_API_BAD_DATA;
    UBX_I4 msgCount = 0;
    UBX_I4 totalSize = 0;
    *piCount = 0;

    while(totalSize < iSize)
    {
        if ((pMgaData[0] == UBX_SIG_PREFIX_1) && (pMgaData[1] == UBX_SIG_PREFIX_2))
        {
            // UBX message
            UBX_U4 payloadSize = pMgaData[4] + (pMgaData[5] << 8);
            UBX_U4 msgSize = payloadSize + UBX_MSG_FRAME_SIZE;
            bool bMsgOfInterest = false;

            switch(pMgaData[2])
            {
            case UBX_CLASS_MGA:  // MGA messages
                switch(pMgaData[3])
                {
                case UBX_MGA_GPS:
                case UBX_MGA_GAL:
                case UBX_MGA_BDS:
                case UBX_MGA_QZSS:
                case UBX_MGA_GLO:
                case UBX_MGA_ANO:
                case UBX_MGA_INI:
                    // MGA message of interest
                    bMsgOfInterest = true;
                    break;

                default:
                    // ignore
                    break;
                }
                break;

            case UBX_CLASS_AID:  // AID messages
                switch (pMgaData[3])
                {
                case UBX_AID_INI:
                case UBX_AID_HUI:
                case UBX_AID_ALM:
                case UBX_AID_EPH:
                    // AID message of interest
                    bMsgOfInterest = true;
                    break;

                default:
                    // ignore
                    break;
                }
                break;

            default:
                // ignore
                break;
            }

            if (bMsgOfInterest)
            {
                if (validChecksum(&pMgaData[2], (size_t) (msgSize - 4)))
                {
                    msgCount++;
                }
                else
                {
                    // bad checksum - move on
                }
            }
            pMgaData += msgSize;
            totalSize += (UBX_I4) msgSize;
        }
        else
        {
            // corrupt data - abort
            break;
        }
    }

    if (totalSize == iSize)
    {
        *piCount = (UBX_U4) msgCount;
        res = MGA_API_OK;
    }

    return res;
}

static bool validChecksum(const UBX_U1* pPayload, size_t iSize)
{
    UBX_U1 ChksumA = 0;
    UBX_U1 ChksumB = 0;

    for (size_t i = 0; i < iSize; i++)
    {
        ChksumA = (UBX_U1)(ChksumA + *pPayload);
        pPayload++;
        ChksumB = (UBX_U1)(ChksumB + ChksumA);
    }

    return ((ChksumA == pPayload[0]) && (ChksumB == pPayload[1]));
}

static void addChecksum(UBX_U1* pPayload, size_t iSize)
{
    UBX_U1 ChksumA = 0;
    UBX_U1 ChksumB = 0;

    for (size_t i = 0; i < iSize; i++)
    {
        ChksumA = (UBX_U1)(ChksumA + *pPayload);
        pPayload++;
        ChksumB = (UBX_U1)(ChksumB + ChksumA);
    }

    *pPayload = ChksumA;
    pPayload++;
    *pPayload = ChksumB;
}

static MgaMsgInfo* buildMsgList(const UBX_U1* pMgaData, unsigned int uNumEntries)
{
    // do not lock here - lock must already be in place

    MgaMsgInfo* pMgaMsgList = (MgaMsgInfo*) malloc(sizeof(MgaMsgInfo) * uNumEntries);
    if (pMgaMsgList == NULL)
        return NULL;

    MgaMsgInfo* pCurrentBlock = pMgaMsgList;

    unsigned int i = 0;
    while(i < uNumEntries)
    {
        UBX_U2 payloadSize = pMgaData[4] + (pMgaData[5] << 8);
        UBX_U2 msgSize = payloadSize + UBX_MSG_FRAME_SIZE;
        bool bProcessMsg = false;

        if ((pMgaData[0] == UBX_SIG_PREFIX_1) && (pMgaData[1] == UBX_SIG_PREFIX_2))
        {
            //UBX message
            switch (pMgaData[2])
            {
            case UBX_CLASS_MGA:  // MGA message
                switch (pMgaData[3])
                {
                case UBX_MGA_GPS:
                case UBX_MGA_GAL:
                case UBX_MGA_BDS:
                case UBX_MGA_QZSS:
                case UBX_MGA_GLO:
                case UBX_MGA_ANO:
                case UBX_MGA_INI:
                    bProcessMsg = true;
                    break;

                default:
                    // ignore
                    break;
                }
                break;

            case UBX_CLASS_AID:  // AID message
                switch (pMgaData[3])
                {
                case UBX_AID_INI:
                case UBX_AID_HUI:
                case UBX_AID_ALM:
                case UBX_AID_EPH:
                    bProcessMsg = true;
                    break;

                default:
                    // ignore
                    break;
                }
                break;

            default:
                // ignore
                break;
            }

            if (bProcessMsg)
            {
                assert(pCurrentBlock < &pMgaMsgList[uNumEntries]);
                const UBX_U1* pPayload = &pMgaData[6];

                pCurrentBlock->mgaMsg.msgId = pMgaData[3];
                memcpy(pCurrentBlock->mgaMsg.mgaPayloadStart, pPayload, sizeof(pCurrentBlock->mgaMsg.mgaPayloadStart));
                pCurrentBlock->pMsg = pMgaData;
                pCurrentBlock->msgSize = msgSize;
                pCurrentBlock->state = MGA_MSG_WAITING_TO_SEND;
                pCurrentBlock->timeOut = 0; // set when transfer takes place
                pCurrentBlock->retryCount = 0;
                pCurrentBlock->sequenceNumber = (UBX_U2) i;
                pCurrentBlock->mgaFailedReason = MGA_FAILED_REASON_CODE_NOT_SET;
                pCurrentBlock++;

                i++;
            }
        }
        pMgaData += msgSize;
    }

    assert(pCurrentBlock == &pMgaMsgList[uNumEntries]);
    return pMgaMsgList;
}

static bool checkForIniMessage(const UBX_U1* pUbxMsg)
{
    if ((pUbxMsg[2] == UBX_CLASS_MGA) && (pUbxMsg[3] == UBX_MGA_INI) && (pUbxMsg[6] == 0x10))
    {
        // UBX-MGA-INI-TIME
        return true;
    }
    else if ((pUbxMsg[2] == UBX_CLASS_AID) && (pUbxMsg[3] == UBX_AID_INI))
    {
        // UBX-AID-INI
        return true;
    }

    return false;
}

static void sessionStop(MGA_PROGRESS_EVENT_TYPE evtType, const void* pEventInfo, size_t evtInfoSize)
{
    // do not lock here - lock must already be in place
    assert(s_sessionState != MGA_IDLE);

    if (s_pEvtInterface->evtProgress)
    {
        s_pEvtInterface->evtProgress(evtType, s_pCallbackContext, pEventInfo, (UBX_I4) evtInfoSize);
    }

    // Tidy up and MGA transfer settings
    free(s_pMgaMsgList);
    s_pMgaMsgList = NULL;
    s_mgaBlockCount = 0;
    s_sessionState = MGA_IDLE;
    s_pLastMsgSent = NULL;
    s_messagesSent = 0;
    s_ackCount = 0;

    // Tidy up any flash transfer settings
    free(s_pMgaFlashBlockList);
    s_pMgaFlashBlockList = NULL;
    s_mgaFlashBlockCount= 0;
    s_pLastFlashBlockSent = NULL;
    s_flashMessagesSent = 0;
    s_flashSequence = 0;

    // Tidy up any specific legacy aiding flash transfer settings
    s_bLegacyAiding = false;
    s_aidState = LEGACY_AIDING_IDLE;
    s_aidingTimeout = 0;

    // Tidy up any legacy aiding server settings
    s_pAidingData = NULL;
    s_aidingDataSize = 0;
    s_alpfileId = 0;
}

static const char* skipSpaces(const char* pText)
{
    while((*pText != 0) && isspace(*pText))
    {
        pText++;
    }

    return *pText == 0 ? NULL : pText;
}

static const char* nextToken(const char* pText)
{
    while((*pText != 0) && (!isspace(*pText)))
    {
        pText++;
    }
    return skipSpaces(pText);
}

static int getData(SOCKET sock, char* p, size_t iSize)
{
    size_t c = 0;
    do
    {
        fd_set fdset;
        FD_ZERO(&fdset);

#ifdef WIN32
#  pragma warning(push)
#  pragma warning( disable : 4127 )
#endif // WIN32
        FD_SET(sock, &fdset);
#ifdef WIN32
#  pragma warning(pop)
#endif // WIN32

        struct timeval tv;
        tv.tv_sec  = s_serverResponseTimeout;
        tv.tv_usec = 0;

        if (select(sock+1, &fdset, NULL, NULL, &tv) > 0)
        {
            int b = recv(sock, p, 1, 0);
            if (b <= 0)
            {
                // failed or timeout
                break;
            }
            else if (b > 0)
            {
                p ++;
                c ++;
            }
        }
        else
        {
            // no response
            break;
        }
    } while (c < iSize);

    return (int) c;
}

static void adjustMgaIniTime(MgaMsgInfo* pMsgInfo, const MgaTimeAdjust* pMgaTime)
{
    assert(pMsgInfo);
    assert(pMsgInfo->pMsg[0] == UBX_SIG_PREFIX_1);
    assert(pMsgInfo->pMsg[1] == UBX_SIG_PREFIX_2);
    assert((pMsgInfo->pMsg[2] == UBX_CLASS_MGA) || (pMsgInfo->pMsg[2] == UBX_CLASS_AID));

    UBX_U1* pMgaIniTimeMsg = (UBX_U1*) pMsgInfo->pMsg;

    if (pMsgInfo->pMsg[2] == UBX_CLASS_MGA)
    {
        // MGA data
        assert(pMsgInfo->pMsg[3] == UBX_MGA_INI);
        assert(pMsgInfo->pMsg[6] == 0x10);

        switch (pMgaTime->mgaAdjustType)
        {
        case MGA_TIME_ADJUST_ABSOLUTE:
            pMgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 4] = (UBX_U1) (pMgaTime->mgaYear & 0xFF);
            pMgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 5] = (UBX_U1) (pMgaTime->mgaYear >> 8);
            pMgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 6] = (UBX_U1) pMgaTime->mgaMonth;
            pMgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 7] = (UBX_U1) pMgaTime->mgaDay;
            pMgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 8] = (UBX_U1) pMgaTime->mgaHour;
            pMgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 9] = (UBX_U1) pMgaTime->mgaMinute;
            pMgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 10] = (UBX_U1) pMgaTime->mgaSecond;
            pMgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 16] = (UBX_U1) (pMgaTime->mgaAccuracyS & 0xFF);
            pMgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 17] = (UBX_U1) (pMgaTime->mgaAccuracyS >> 8);
            *((UBX_U4*) (&pMgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 20])) = ((UBX_U4) pMgaTime->mgaAccuracyMs) * MS_IN_A_NS;
            break;

        case MGA_TIME_ADJUST_RELATIVE:
        {
            tm timeAsStruct;
            memset(&timeAsStruct, 0, sizeof(timeAsStruct));

            timeAsStruct.tm_year = (pMgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 4] + (pMgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 5] << 8)) - 1900;
            timeAsStruct.tm_mon = pMgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 6] - 1;
            timeAsStruct.tm_mday = pMgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 7];
            timeAsStruct.tm_hour = pMgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 8];
            timeAsStruct.tm_min = pMgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 9];
            timeAsStruct.tm_sec = pMgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 10];

            time_t t1 = mktime(&timeAsStruct);
            assert(t1 != -1);

            assert(pMgaTime->mgaYear == 0);
            assert(pMgaTime->mgaMonth == 0);
            assert(pMgaTime->mgaDay == 0);

            time_t adjustment = (pMgaTime->mgaHour * 3600) + (pMgaTime->mgaMinute * 60) + pMgaTime->mgaSecond;
            t1 += adjustment;
            t1 -= timezone;     // adjust for time zone as mktime return local time representation
            tm * pAdjustedTime = gmtime(&t1);

            pAdjustedTime->tm_year += 1900;
            pMgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 4] = (UBX_U1) (pAdjustedTime->tm_year & 0xFF);
            pMgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 5] = (UBX_U1) (pAdjustedTime->tm_year >> 8);
            pMgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 6] = (UBX_U1) (pAdjustedTime->tm_mon + 1);
            pMgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 7] = (UBX_U1) pAdjustedTime->tm_mday;
            pMgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 8] = (UBX_U1) pAdjustedTime->tm_hour;
            pMgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 9] = (UBX_U1) pAdjustedTime->tm_min;
            pMgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 10] = (UBX_U1) pAdjustedTime->tm_sec;
        }
        break;

        default:
            assert(0);
            break;
        }

        // recalculate message checksum
        addChecksum(&pMgaIniTimeMsg[2], (UBX_MSG_FRAME_SIZE + 24) - 4);
        assert(validChecksum(&pMgaIniTimeMsg[2], (UBX_MSG_FRAME_SIZE + 24) - 4));
    }
    else
    {
        // Legacy online data
        assert(pMsgInfo->pMsg[3] == UBX_AID_INI);

        switch (pMgaTime->mgaAdjustType)
        {
        case MGA_TIME_ADJUST_ABSOLUTE:
            pMgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 19] = (UBX_U1) (pMgaTime->mgaYear - 2000);
            pMgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 18] = (UBX_U1) pMgaTime->mgaMonth;
            pMgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 23] = (UBX_U1) pMgaTime->mgaDay;
            pMgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 22] = (UBX_U1) pMgaTime->mgaHour;
            pMgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 21] = (UBX_U1) pMgaTime->mgaMinute;
            pMgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 20] = (UBX_U1) pMgaTime->mgaSecond;

            *((UBX_U4*) (&pMgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 28])) = (UBX_U4) (pMgaTime->mgaAccuracyS * 1000) + pMgaTime->mgaAccuracyMs;
            *((UBX_U4*) (&pMgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 32])) = 0;    // ((UBX_U4)) * MS_IN_A_NS;

            *((UBX_U4 *) (&pMgaIniTimeMsg[UBX_MSG_PAYLOAD_OFFSET + 44])) |= 0x400;
            break;

        case MGA_TIME_ADJUST_RELATIVE:
            assert(false);  // Not supported
            break;

        default:
            assert(0);
            break;
        }

        // recalculate message checksum
        addChecksum(&pMgaIniTimeMsg[2], (UBX_MSG_FRAME_SIZE + 48) - 4);
        assert(validChecksum(&pMgaIniTimeMsg[2], (UBX_MSG_FRAME_SIZE + 48) - 4));
    }
}

static MgaMsgInfo* findMsgBlock(UBX_U1 msgId, const UBX_U1* pMgaHeader)
{
    assert(s_pMgaMsgList);

    MgaMsgInfo* pMsgInfo = &s_pMgaMsgList[0];
    for(UBX_U4 i = 0; i < s_mgaBlockCount; i++)
    {
        if ((pMsgInfo->state == MGA_MSG_WAITING_FOR_ACK) &&
            (pMsgInfo->mgaMsg.msgId == msgId) &&
            (memcmp(pMgaHeader, pMsgInfo->mgaMsg.mgaPayloadStart, sizeof(pMsgInfo->mgaMsg.mgaPayloadStart)) == 0))
        {
            // found match
            return pMsgInfo;
        }
        pMsgInfo++;
    }

    return NULL;
}

static void sendInitialMsgBatch(void)
{
    // dispatch maximum amount of messages
    UBX_I4 rxBufferSize = 1000;

    while(rxBufferSize > 0)
    {
        UBX_I4 msgSize = sendNextMgaMessage();
        if (msgSize == 0)
        {
            break;
        }
        rxBufferSize -= msgSize;
    }
}

static void initiateMessageTransfer(void)
{
    switch(s_pFlowConfig->mgaFlowControl)
    {
    case MGA_FLOW_SIMPLE:
        sendCfgMgaAidAcks(true);
        sendNextMgaMessage();
        break;

    case MGA_FLOW_NONE:
        sendAllMessages();
        break;

    case MGA_FLOW_SMART:
        sendCfgMgaAidAcks(true);
        sendInitialMsgBatch();
        break;

    default:
        assert(0);
        break;
    }
}

static bool isAnoMatch(const UBX_U1* pMgaData, int cy, int cm, int cd)
{
    if ((pMgaData[2] == UBX_CLASS_MGA) && (pMgaData[3] == UBX_MGA_ANO))
    {
        // UBX-MGA-ANO
        const UBX_U1* pPayload = &pMgaData[6];
        if (((pPayload[4] + 2000) == cy) && (pPayload[5] == cm) && (pPayload[6] == cd))
        {
            return true;
        }
    }

    return false;
}

static MGA_API_RESULT getOnlineDataFromServer(const char* pServer, const MgaOnlineServerConfig* pServerConfig, UBX_U1** ppData, UBX_I4* piSize)
{
    *ppData = NULL;
    *piSize = 0;

    char* ptr;
    char strServer[80] = {0};
    char host[80] = {0};
    UBX_U2 wPort = 80;  // default port number

    // copy the server name
    strncpy(strServer, pServer, sizeof(strServer)-1);

    // try to get server string
    if( (ptr = strtok(strServer, ":")) != NULL)
        strncpy(host, ptr, sizeof(host)-1);

    // try to get port number
    if( (ptr = strtok(NULL, ":")) != NULL)
        wPort = (UBX_U2)atoi(ptr);

    // connect to server
    SOCKET iSock = connectServer(host, wPort);
    if (iSock == INVALID_SOCKET)
    {
        return MGA_API_CANNOT_CONNECT;
    }

    char requestParams[500];
    mgaBuildOnlineRequestParams(pServerConfig, requestParams, sizeof(requestParams));

    char strHttp[1000];
    sprintf(strHttp,"GET /GetOnlineData.ashx?%s HTTP/1.1\r\n"
                    "Host: %s\r\n"
                    "Accept: */*\r\n"
                    "Connection: Keep-Alive\r\n\r\n",
                    requestParams,
                    pServer);

    assert(strlen(strHttp) < sizeof(strHttp));
    MGA_API_RESULT res = getDataFromService(iSock, strHttp, ppData, piSize);

#ifdef WIN32
    closesocket(iSock);
#else // WIN32
    close(iSock);
#endif // WIN32

    return res;
}

static MGA_API_RESULT getOfflineDataFromServer(const char* pServer, const MgaOfflineServerConfig* pServerConfig, UBX_U1** ppData, UBX_I4* piSize)
{
    *ppData = NULL;
    *piSize = 0;

    char* ptr;
    char strServer[80] = {0};
    char host[80] = {0};
    UBX_U2 wPort = 80;  // default port number

    // copy the server name
    strncpy(strServer, pServer, sizeof(strServer)-1);

    // try to get server string
    if( (ptr = strtok(strServer, ":")) != NULL)
        strncpy(host, ptr, sizeof(host)-1);

    // try to get port number
    if( (ptr = strtok(NULL, ":")) != NULL)
        wPort = (UBX_U2)atoi(ptr);

    // connect to server
    SOCKET iSock = connectServer(host, wPort);
    if (iSock == INVALID_SOCKET)
    {
        return MGA_API_CANNOT_CONNECT;
    }

    char requestParams[500];
    mgaBuildOfflineRequestParams(pServerConfig, requestParams, sizeof(requestParams));

    char strHttp[1000];
    sprintf(strHttp,"GET /GetOfflineData.ashx?%s HTTP/1.1\r\n"
                    "Host: %s\r\n"
                    "Accept: */*\r\n"
                    "Connection: Keep-Alive\r\n\r\n",
                    requestParams,
                    pServer);

    assert(strlen(strHttp) < sizeof(strHttp));
    MGA_API_RESULT res = getDataFromService(iSock, strHttp, ppData, piSize);

#ifdef WIN32
    closesocket(iSock);
#else // WIN32
    close(iSock);
#endif // WIN32

    return res;
}

static void commaToPoint(char* pText)
{
    while(*pText)
    {
        if (*pText == ',')
            *pText = '.';
        pText++;
    }
}

// Internal legacy aiding server support
static void legacyAidingCheckMessage(const UBX_U1* pData, UBX_U4 iSize)
{
    assert(s_aidingSrvActive == true);
    assert(s_pAidingData != NULL);
    assert(s_aidingDataSize > 0);

    const UBX_U1 alpDataRqst [] = { UBX_SIG_PREFIX_1, UBX_SIG_PREFIX_2, UBX_CLASS_AID, UBX_AID_ALPSRV };

    // check if this is an ALP data request
    if ((iSize > 14) && (memcmp(pData, alpDataRqst, sizeof(alpDataRqst)) == 0))
    {
        LegacyAidingRequestHeader *pAidingRequestHeader = (LegacyAidingRequestHeader*) (pData + UBX_MSG_PAYLOAD_OFFSET);
        if ((iSize - UBX_MSG_FRAME_SIZE) >= pAidingRequestHeader->idSize)
        {
            if (pAidingRequestHeader->type != 0xFF)
            {
                // Handle aiding data request
                if (s_pEvtInterface->evtProgress)
                {
                    s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_LEGACY_AIDING_SERVER_REQUEST_RECEIVED,
                                                s_pCallbackContext,
                                                pAidingRequestHeader,
                                                iSize - UBX_MSG_FRAME_SIZE);
                }
                legacyAidingRequestData(pAidingRequestHeader);
            }
            else
            {
                // Update aiding data
                LegacyAidingUpdateDataHeader *pAidingUpdateHeader = (LegacyAidingUpdateDataHeader *) pAidingRequestHeader;
                assert((iSize - UBX_MSG_FRAME_SIZE) == sizeof(LegacyAidingUpdateDataHeader) + (pAidingUpdateHeader->size * 2));

                if (s_pEvtInterface->evtProgress)
                {
                    s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_LEGACY_AIDING_SERVER_UPDATE_RECEIVED,
                                                s_pCallbackContext,
                                                pAidingUpdateHeader,
                                                iSize - UBX_MSG_FRAME_SIZE);
                }
                legacyAidingUpdateData(pAidingUpdateHeader);
            }
        }
    }
}

static void legacyAidingRequestData(const LegacyAidingRequestHeader *pAidingRequestHeader)
{
    assert(s_aidingSrvActive == true);
    assert(s_pAidingData != NULL);
    assert(s_aidingDataSize > 0);
    assert(pAidingRequestHeader != NULL);

    UBX_U4 msgReplySize = 0;
    UBX_U1 *pRqstDataReplyMsg = NULL;
    LegacyAidingRequestHeader *pId = NULL;

    UBX_U4 ofs = pAidingRequestHeader->ofs * 2;
    UBX_U4 dataSize = pAidingRequestHeader->size * 2;

    if ((ofs < s_aidingDataSize) && (dataSize > 0))
    {
        if (ofs + dataSize >= s_aidingDataSize)
        {
            if(ofs >= s_aidingDataSize)
            {
                ofs = s_aidingDataSize - 1;
            }
            // - Just send all we've got starting at the offset
            dataSize = s_aidingDataSize - ofs;
        }

        // Build the aiding request response message
        msgReplySize = UBX_MSG_FRAME_SIZE + pAidingRequestHeader->idSize + dataSize;
        pRqstDataReplyMsg = (UBX_U1 *) malloc(msgReplySize);
        if (pRqstDataReplyMsg)
        {
            pId = (LegacyAidingRequestHeader*) (pRqstDataReplyMsg + UBX_MSG_PAYLOAD_OFFSET);

            // Copy the aiding request header
            assert(pAidingRequestHeader->idSize == sizeof(LegacyAidingRequestHeader));
            memcpy(pId, pAidingRequestHeader, pAidingRequestHeader->idSize);

            // Update the aiding header
            pId->fileId = s_alpfileId;
            pId->dataSize = (UBX_U2) dataSize;

            // Copy the requested aiding data into the response payload
            memcpy(pRqstDataReplyMsg + UBX_MSG_PAYLOAD_OFFSET + pAidingRequestHeader->idSize, &s_pAidingData[ofs], dataSize);

            // Fill in the UBX message header
            UbxMsgHeader *pUbxMsg = (UbxMsgHeader *) pRqstDataReplyMsg;
            pUbxMsg->header1 = UBX_SIG_PREFIX_1;
            pUbxMsg->header2 = UBX_SIG_PREFIX_2;
            pUbxMsg->msgClass = UBX_CLASS_AID;
            pUbxMsg->msgId = UBX_AID_ALPSRV;
            pUbxMsg->payloadLength = (UBX_U2) (pAidingRequestHeader->idSize + dataSize);

            addChecksum(&pRqstDataReplyMsg[2], msgReplySize - 4);
            assert(validChecksum(&pRqstDataReplyMsg[2], msgReplySize - 4));

            // Send the message
            s_pEvtInterface->evtWriteDevice(s_pCallbackContext, pRqstDataReplyMsg, msgReplySize);
        }
        else
        {
            // No memory
            if (s_pEvtInterface->evtProgress)
            {
                s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_LEGACY_AIDING_REQUEST_FAILED_NO_MEMORY,
                    s_pCallbackContext,
                    pAidingRequestHeader,
                    sizeof(LegacyAidingRequestHeader));
            }
        }
    }

    if (s_pEvtInterface->evtProgress)
    {
        s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_LEGACY_AIDING_SERVER_REQUEST_COMPLETED,
            s_pCallbackContext,
            pId,
            pId ? msgReplySize - UBX_MSG_FRAME_SIZE : 0);
    }
    free(pRqstDataReplyMsg);
}

static void legacyAidingUpdateData(const LegacyAidingUpdateDataHeader *pLegacyAidingUpdateHeader)
{
    assert(s_aidingSrvActive == true);
    assert(s_pAidingData != NULL);
    assert(s_aidingDataSize > 0);
    assert(pLegacyAidingUpdateHeader != NULL);

    UBX_U4 ofs = pLegacyAidingUpdateHeader->ofs * 2;
    UBX_U4 dataSize = pLegacyAidingUpdateHeader->size * 2;

    if (pLegacyAidingUpdateHeader->fileId == s_alpfileId)    // Update applies to present data
    {
        if (dataSize > 0)
        {
            // There is some data to update
            if (ofs + dataSize >= s_aidingDataSize)
            {
                if(ofs >= s_aidingDataSize)
                {
                    ofs = s_aidingDataSize - 1;
                }
                // - Just send all we've got starting at the offset
                dataSize = s_aidingDataSize - ofs;
            }

            // Overwrite the original aiding data
            memcpy(&s_pAidingData[ofs], &pLegacyAidingUpdateHeader[1], dataSize);
        }
    }
    else
    {
        // Wrong fileId
        if (s_pEvtInterface->evtProgress)
        {
            s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_LEGACY_AIDING_REQUEST_FAILED_ID_MISMATCH,
                s_pCallbackContext,
                pLegacyAidingUpdateHeader,
                sizeof(LegacyAidingUpdateDataHeader));
        }
    }

    // All done
    if (s_pEvtInterface->evtProgress)
    {
        s_pEvtInterface->evtProgress(MGA_PROGRESS_EVT_LEGACY_AIDING_SERVER_UPDATE_COMPLETED,
            s_pCallbackContext,
            NULL,
            0);
    }
}
