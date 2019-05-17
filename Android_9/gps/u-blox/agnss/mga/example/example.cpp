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
 * Project: libMGA
 * Purpose: Example host application using the libMga library to download
 *          MGA assistance data and pass it on to a u-blox GNSS receiver.
 *
 *****************************************************************************/

#include <stdio.h>      // standard input/output definitions
#include <assert.h>     // defines the C preprocessor macro assert()
#include <stdlib.h>     // numeric conversion functions, memory allocation
#include <string.h>     // string handling
#include <ctype.h>

#include <limits.h>

#include "libMga.h"
#include "parserbuffer.h"
#include "protocolubx.h"
#include "protocolnmea.h"
#include "protocolunknown.h"

#include "arguments.h"  // argument handling

#ifdef _WIN32
#include <winsock.h>
#include <conio.h>
#else
#include <sys/ioctl.h>
#include <termios.h>
#endif

#define TIMEOUT                 1


///////////////////////////////////////////////////////////////////////////////
extern void CleanUpThread();
extern void mySleep(int seconds);
extern bool openSerial(const char* port, unsigned int baudrate);
extern int readSerial(void *pBuffer, unsigned int size);
extern int writeSerial(const void *pBuffer, unsigned int size);
extern void closeSerial();
const char* exename = "";
void startDeviceReadThread(void);

///////////////////////////////////////////////////////////////////////////////

static bool s_keepReadingDevice = false;
static bool s_ActiveMgaTransfer = false;
static bool s_readThreadFinished = false;
static time_t s_nextCheck = 0;
// static char* s_serverToken = NULL;

// global variable for the verbosity
extern unsigned int verbosity;

///////////////////////////////////////////////////////////////////////////////
#ifndef _WIN32
static int _kbhit() {
    static const int STDIN = 0;
    static bool initialized = false;

    if (! initialized) {
        // Use termios to turn off line buffering
        static struct termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}
#endif

static void stopDeviceReadThread(void)
{
    s_keepReadingDevice = false;

    while (!s_readThreadFinished)
    {
        mySleep(TIMEOUT);
    }

    CleanUpThread();
}

static bool waitForFinish(int retries)
{
    int count = (retries > 0) ? retries : 1;
    while ((s_ActiveMgaTransfer) && (count > 0))
    {
        mySleep(TIMEOUT);

        if (retries > 0)
        {
            count--;
        }
    }

    if (!s_ActiveMgaTransfer)
        return true;
    else
        return false;
}


#ifdef _WIN32
void DeviceReadThread(void *)
#else
void *DeviceReadThread(void *)
#endif
{
    s_keepReadingDevice = true;
    s_readThreadFinished = false;

    CProtocolUBX  protocolUBX;
    CProtocolNMEA protocolNmea;
    CProtocolUnknown protocolUnknown;
    CParserBuffer parser;               // declare after protocols, so destructor called before protocol destructor

    // setup parser buffer
    parser.Register(&protocolUBX);
    parser.Register(&protocolNmea);
    parser.RegisterUnknown(&protocolUnknown);

    while (s_keepReadingDevice)
    {
        unsigned char *ptr = parser.GetPointer();
        unsigned int space = (unsigned int)parser.GetSpace();
        int size = readSerial(ptr, space);

        if (size)
        {
            // printf("%i bytes read\n", size);
            int msgSize;
            unsigned char* pMsg;
            CProtocol* pProtocol;

            // we read it so update the size
            parser.Append(size);
            // parse ... only interested breaking the message stream into individual UBX message chunks
            while (parser.Parse(pProtocol, pMsg, msgSize))
            {

                if (pProtocol == &protocolUnknown)
                {
                    printf("%s: MSG UNKNOWN size %d\n", __FUNCTION__, msgSize);
                }


                mgaProcessReceiverMessage(pMsg, msgSize);
                parser.Remove(msgSize);
            }

            parser.Compact();
        }
        else
        {
            // printf("%s: read error %d\n", __FUNCTION__, size);
        }

        time_t now = time(NULL);
        if (now > s_nextCheck)
        {
            mgaCheckForTimeOuts();
            s_nextCheck = now + 1;
        }
    }

    s_readThreadFinished = true;
#ifdef _WIN32
#else
    return NULL;
#endif
}

///////////////////////////////////////////////////////////////////////////////

static void EvtProgressHandler(MGA_PROGRESS_EVENT_TYPE evtType, const void* pContext, const void* pEvtInfo, UBX_I4 evtInfoSize)
{
    // unused variables
    (void)pContext;
    (void)evtInfoSize;

    static int s_max = 0;
    static UBX_U2 s_activeFileId = 0;

    switch (evtType)
    {
    case MGA_PROGRESS_EVT_MSG_SENT:
    {
        MgaMsgInfo* pMsgInfo = (MgaMsgInfo*)pEvtInfo;
        if (verbosity == 1)
        {
            printf("Message sent [%i/%i] size [%i] reties[%i]\n",
                   pMsgInfo->sequenceNumber + 1,
                   s_max,
                   pMsgInfo->msgSize,
                   pMsgInfo->retryCount);
        }
    }
    break;

    case MGA_PROGRESS_EVT_TERMINATED:
    {
        EVT_TERMINATION_REASON* stopReason = (EVT_TERMINATION_REASON*)pEvtInfo;
        printf("Terminated - Reason %i\n", *stopReason);
        s_ActiveMgaTransfer = false;
    }
    break;

    case MGA_PROGRESS_EVT_FINISH:
    {
        // pEvtInfo is NULL
        printf("Finish\n");
        s_ActiveMgaTransfer = false;
    }
    break;

    case MGA_PROGRESS_EVT_START:
    {
        s_max = *((UBX_U4*)pEvtInfo);
        printf("Start. Expecting %i messages\n", s_max);
    }
    break;

    case MGA_PROGRESS_EVT_MSG_TRANSFER_FAILED:
    {
        MgaMsgInfo* pMsgInfo = (MgaMsgInfo*)pEvtInfo;
        printf("Transfer failed - Message [%i]  Reason [%i]\n",
               pMsgInfo->sequenceNumber + 1,
               pMsgInfo->mgaFailedReason);
    }
    break;

    case MGA_PROGRESS_EVT_MSG_TRANSFER_COMPLETE:
    {
        MgaMsgInfo* pMsgInfo = (MgaMsgInfo*)pEvtInfo;
        printf("Transfer complete [%i]\n", pMsgInfo->sequenceNumber + 1);
    }
    break;

    case MGA_PROGRESS_EVT_SERVER_CONNECT:
    {
        printf("Connected to server '%s'\n", (char *)pEvtInfo);
    }
    break;

    case MGA_PROGRESS_EVT_REQUEST_HEADER:
    {
        printf("Requesting header\n");
    }
    break;

    case MGA_PROGRESS_EVT_RETRIEVE_DATA:
    {
        // pEvtInfo is NULL
        printf("Retrieve data\n");
    }
    break;

    case MGA_PROGRESS_EVT_SERVICE_ERROR:
    {
        EvtInfoServiceError* pError = (EvtInfoServiceError*)pEvtInfo;
        printf("Service error - code [%i] '%s'\n", pError->errorType, pError->errorMessage);
    }
    break;

    case MGA_PROGRESS_EVT_SERVER_CONNECTING:
    {
        const char* pServerName = (const char*)pEvtInfo;
        printf("Connecting to server '%s'\n", pServerName);
    }
    break;

    case MGA_PROGRESS_EVT_UNKNOWN_SERVER:
    {
        const char* pServerName = (const char*)pEvtInfo;
        printf("Unknown server '%s'\n", pServerName);
    }
    break;

    case MGA_PROGRESS_EVT_SERVER_CANNOT_CONNECT:
    {
        const char* pServerName = (const char*)pEvtInfo;
        printf("Can not connect to server '%s'\n", pServerName);
    }
    break;

    case MGA_PROGRESS_EVT_LEGACY_AIDING_STARTUP:
        s_max = *((UBX_U4*)pEvtInfo);
        printf("Initializing legacy aiding flash data transfer\n");
        break;

    case MGA_PROGRESS_EVT_LEGACY_AIDING_STARTUP_FAILED:
    {
        UBX_I4 reason = *(UBX_I4*)pEvtInfo;
        printf("Initialization of legacy aiding flash data transfer. Reason [%i]\n", reason);

        switch (reason)
        {
        case MGA_FAILED_REASON_LEGACY_NO_ACK:
            printf("\tNo Ack\n");
            break;

        default:
            assert(false);
            break;
        }
    }
    break;

    case MGA_PROGRESS_EVT_LEGACY_AIDING_STARTUP_COMPLETED:
        printf("Completed the initialization of legacy aiding flash data transfer\n");
        break;

    case MGA_PROGRESS_EVT_LEGACY_AIDING_FLASH_BLOCK_SENT:
    {
        MgaMsgInfo* pInfo = (MgaMsgInfo*)pEvtInfo;

        printf("Legacy aiding flash data block sent [%i/%i] size [%i]\n",
               pInfo->sequenceNumber + 1,
               s_max,
               pInfo->msgSize);
    }
    break;

    case MGA_PROGRESS_EVT_LEGACY_AIDING_FLASH_BLOCK_FAILED:
    {
        MgaMsgInfo* pInfo = (MgaMsgInfo*)pEvtInfo;

        printf("Legacy aiding flash data block [%i], transfer failed. Reason [%i] - ",
               pInfo->sequenceNumber + 1,
               pInfo->mgaFailedReason);
        switch (pInfo->mgaFailedReason)
        {
        case MGA_FAILED_REASON_LEGACY_NO_ACK:
            printf("No Ack\n");
            break;

        default:
            assert(false);
            break;
        }

    }
    break;

    case MGA_PROGRESS_EVT_LEGACY_AIDING_FLASH_BLOCK_COMPLETE:
    {
        MgaMsgInfo* pInfo = (MgaMsgInfo*)pEvtInfo;
        printf("Legacy aiding flash data block received [%i]\n", pInfo->sequenceNumber + 1);
    }
    break;

    case MGA_PROGRESS_EVT_LEGACY_AIDING_FINALIZE_START:
        printf("Finalizing legacy aiding flash data transfer\n");
        break;

    case MGA_PROGRESS_EVT_LEGACY_AIDING_FINALIZE_FAILED:
        printf("Finalization of legacy aiding flash data transfer failed (No Ack)\n");
        break;

    case MGA_PROGRESS_EVT_LEGACY_AIDING_FINALIZE_COMPLETED:
        printf("Finalizing legacy aiding flash data transfer done\n");
        break;

    case MGA_PROGRESS_EVT_LEGACY_AIDING_SERVER_STARTED:
    {
        // pEvtInfo is a pointer to the payload of the UBX-AID-ALPSRV message just sent to inform
        // the receiver the host based aiding server is active.
        // This payload is a LegacyAidingRequestHeader structure followed by a LegacyAidingDataHeader
        // structure.
        // evtInfoSize is the size of these two structures.
        assert(evtInfoSize == (sizeof(LegacyAidingRequestHeader) + sizeof(LegacyAidingDataHeader)));

        LegacyAidingRequestHeader *pAidingRequestHeader = (LegacyAidingRequestHeader *)pEvtInfo;
        LegacyAidingDataHeader *pAidingData = (LegacyAidingDataHeader *)&pAidingRequestHeader[1];

        printf("Legacy aiding server started: Magic: 0x%x, Duration: %i %s\n",
               pAidingData->magic,
               (pAidingData->duration * 600) / 86400,
               ((pAidingData->duration * 600) / 86400) == 1 ? "day" : "days");
        s_activeFileId = pAidingRequestHeader->fileId;
    }
    break;

    case MGA_PROGRESS_EVT_LEGACY_AIDING_SERVER_STOPPED:
    {
        // pEvtInfo is NULL
        // evtInfoSize is 0
        assert(pEvtInfo == NULL);
        assert(evtInfoSize == 0);
        printf("Legacy aiding server stopped\n");
    }
    break;

    case MGA_PROGRESS_EVT_LEGACY_AIDING_SERVER_REQUEST_RECEIVED:
    {
        // pEvtInfo is a pointer to the to the payload of the UBX-AID-ALPSRV message just received.
        // This payload is a LegacyAidingRequestHeader structure.
        // evtInfoSize is the size of this LegacyAidingRequestHeader structure.
        LegacyAidingRequestHeader *pRequestEvtInfo = (LegacyAidingRequestHeader *)pEvtInfo;
        assert(evtInfoSize == sizeof(LegacyAidingRequestHeader));

        printf("Req rcvd: Header size: %i, Type: %x, Offs: %i, Size: %i\n",
               pRequestEvtInfo->idSize,
               pRequestEvtInfo->type,
               pRequestEvtInfo->ofs * 2,
               pRequestEvtInfo->size * 2
               );
    }
    break;

    case MGA_PROGRESS_EVT_LEGACY_AIDING_SERVER_REQUEST_COMPLETED:
    {
        // pEvtInfo is a pointer to the to the payload of the UBX-AID-ALPSRV reply message just sent.
        // This payload is a LegacyAidingRequestHeader structure followed the requested aiding data.
        // evtInfoSize is the size of this LegacyAidingRequestHeader structure plus the requested data.
        // The requested data length can be found in the LegacyAidingRequestHeader 'dataSize' field.
        // If pEvtInfo is NULL, evtInfoSize will be 0. This means no reply message was sent.
        LegacyAidingRequestHeader *pRequestEvtInfo = (LegacyAidingRequestHeader *)pEvtInfo;
        if (pRequestEvtInfo)
        {
            assert(evtInfoSize == (UBX_I4)(sizeof(LegacyAidingRequestHeader) + pRequestEvtInfo->dataSize));

            printf("Repl sent: Header size: %i, Type: %x, Offs: %i, Size: %i\n",
                   pRequestEvtInfo->idSize,
                   pRequestEvtInfo->type,
                   pRequestEvtInfo->ofs * 2,
                   pRequestEvtInfo->size * 2
                   );
        }
        else
        {
            assert(evtInfoSize == 0);
            printf("Reply not sent: Request outside data bounds.\n");
        }
    }
    break;

    case MGA_PROGRESS_EVT_LEGACY_AIDING_SERVER_UPDATE_RECEIVED:
    {
        // pEvtInfo is a pointer to the to the payload of the UBX-AID-ALPSRV update message just sent.
        // This payload is a LegacyAidingUpdateDataHeader structure followed by the new aiding data.
        // evtInfoSize is the size of this LegacyAidingUpdateDataHeader structure plus the new aiding data.
        // The new aiding data length can be calculated by taking the LegacyAidingUpdateDataHeader 'size' field.
        // and multiplying by 2.
        LegacyAidingUpdateDataHeader *pAidingUpdateHeader = (LegacyAidingUpdateDataHeader *)pEvtInfo;
        assert(pAidingUpdateHeader != NULL);
        assert(sizeof(LegacyAidingUpdateDataHeader) == pAidingUpdateHeader->idSize);

        printf("Update received: Offs: %i, Data size: %i\n",
               pAidingUpdateHeader->ofs * 2,
               pAidingUpdateHeader->size * 2);
        printf(" - Don't access aiding data directly\n");
    }
    break;

    case MGA_PROGRESS_EVT_LEGACY_AIDING_SERVER_UPDATE_COMPLETED:
    {
        // pEvtInfo is NULL
        // evtInfoSize is 0
        assert(pEvtInfo == NULL);
        assert(evtInfoSize == 0);

        printf("Update completed: Can now access aiding data directly\n");
    }
    break;

    case MGA_PROGRESS_EVT_LEGACY_AIDING_REQUEST_FAILED_NO_MEMORY:
    {
        // pEvtInfo is a pointer to the to the payload of the UBX-AID-ALPSRV update message just sent.
        // This payload is a LegacyAidingRequestHeader structure.
        // evtInfoSize is the size of this LegacyAidingRequestHeader structure.
        LegacyAidingRequestHeader *pAidingRequestHeader = (LegacyAidingRequestHeader *)pEvtInfo;
        assert(evtInfoSize == sizeof(LegacyAidingRequestHeader));

        printf("Request response not sent: Not enough memory. Offs: %i, Data size: %i\n",
               pAidingRequestHeader->ofs * 2,
               pAidingRequestHeader->dataSize * 2);
    }
    break;

    case MGA_PROGRESS_EVT_LEGACY_AIDING_REQUEST_FAILED_ID_MISMATCH:
    {
        // pEvtInfo is a pointer to the to the payload of the UBX-AID-ALPSRV update message just sent.
        // This payload is a LegacyAidingUpdateDataHeader structure.
        // evtInfoSize is the size of this LegacyAidingUpdateDataHeader structure.
        LegacyAidingUpdateDataHeader *pLegacyAidingheader = (LegacyAidingUpdateDataHeader *)pEvtInfo;
        assert(s_activeFileId != pLegacyAidingheader->fileId);

        printf("Request resoonse not sent: File Ids do not match. Expected: %i, Received: %i\n",
               s_activeFileId,
               pLegacyAidingheader->fileId);
    }
    break;

    default:
        assert(0);
        break;
    }
}


static void EvtWriteDeviceHandler(const void* pContext, const UBX_U1* pData, UBX_I4 iSize)
{
    //printf("Writing %d bytes of data\n", iSize);
    (void)pContext;

    writeSerial(pData, iSize);
}


///////////////////////////////////////////////////////////////////////////////
// MGA online test (MGA and legacy)
static void onlineExample(CL_ARGUMENTS_pt clArgs)
{
    // setup callback for progress and writing to receiver
    MgaEventInterface evtInterface;
    evtInterface.evtProgress = EvtProgressHandler;
    evtInterface.evtWriteDevice = EvtWriteDeviceHandler;

    // setup flow control to the receiver like this
    MgaFlowConfiguration flowConfig;
    memset(&flowConfig, 0, sizeof(flowConfig));
    flowConfig.msgTimeOut = clArgs->timeout;
    flowConfig.msgRetryCount = clArgs->retry;

    // determine the flow control from the command line arguments
    flowConfig.mgaFlowControl = MGA_FLOW_NONE;
    if (strstr(clArgs->flow, "simple"))
        flowConfig.mgaFlowControl = MGA_FLOW_SIMPLE;
    if (strstr(clArgs->flow, "smart"))
        flowConfig.mgaFlowControl = MGA_FLOW_SMART;

    MGA_API_RESULT mgaResult = mgaConfigure(&flowConfig, &evtInterface, NULL);
    assert(mgaResult == MGA_API_OK);

    // setup what Online servers to communicate with, and what parameters to send
    MgaOnlineServerConfig mgaServerConfig;
    memset(&mgaServerConfig, 0, sizeof(mgaServerConfig));

    mgaServerConfig.strPrimaryServer = clArgs->server1;
    mgaServerConfig.strSecondaryServer = clArgs->server2;
    mgaServerConfig.strServerToken = clArgs->token;

    // specify what GNSS systems we need data for, like this
    strstr(clArgs->gnss, "gps") ? mgaServerConfig.gnssTypeFlags |= MGA_GNSS_GPS : 0;
    strstr(clArgs->gnss, "glo") ? mgaServerConfig.gnssTypeFlags |= MGA_GNSS_GLO : 0;
    strstr(clArgs->gnss, "gal") ? mgaServerConfig.gnssTypeFlags |= MGA_GNSS_GALILEO : 0;
    strstr(clArgs->gnss, "bds") ? mgaServerConfig.gnssTypeFlags |= MGA_GNSS_BEIDOU : 0;
    strstr(clArgs->gnss, "qzss") ? mgaServerConfig.gnssTypeFlags |= MGA_GNSS_QZSS : 0;

    // specify what type of online data we need, like this
    strstr(clArgs->dataType, "eph") ? mgaServerConfig.dataTypeFlags |= MGA_DATA_EPH : 0;
    strstr(clArgs->dataType, "alm") ? mgaServerConfig.dataTypeFlags |= MGA_DATA_ALM : 0;
    strstr(clArgs->dataType, "aux") ? mgaServerConfig.dataTypeFlags |= MGA_DATA_AUX : 0;
    strstr(clArgs->dataType, "pos") ? mgaServerConfig.dataTypeFlags |= MGA_DATA_POS : 0;

    // specify position like this
    if (clArgs->usePosition)
    {
        mgaServerConfig.useFlags |= MGA_FLAGS_USE_POSITION;
        mgaServerConfig.dblAccuracy = clArgs->posAccuracy;
        mgaServerConfig.dblAltitude = clArgs->posAltitude;
        mgaServerConfig.dblLatitude = clArgs->posLatitude;
        mgaServerConfig.dblLongitude = clArgs->posLongitude;
    }

    // specify latency like this
    if (clArgs->useLatency)
    {
        mgaServerConfig.useFlags |= MGA_FLAGS_USE_LATENCY;
        mgaServerConfig.latency = clArgs->latency;
    }

    // specify time accuracy like this
    if (clArgs->useTimeAccuracy)
    {
        mgaServerConfig.useFlags |= MGA_FLAGS_USE_TIMEACC;
        mgaServerConfig.timeAccuracy = clArgs->timeAccuracy;
    }

    // check for legacy aiding
    if (strstr(clArgs->format, "aid"))
    {
        mgaServerConfig.useFlags |= MGA_FLAGS_USE_LEGACY_AIDING;

        // reset the flow control
        flowConfig.mgaFlowControl = MGA_FLOW_NONE;
        if (strstr(clArgs->flow, "simple"))
            flowConfig.mgaFlowControl = MGA_FLOW_SIMPLE;
    }

    //check if server certificate should be verified
    if (clArgs->serverVerify)
    {
        mgaServerConfig.bValidateServerCert = true;
    }

    // set the filter on position like this
    mgaServerConfig.bFilterOnPos = true;

    printf("Connecting to u-blox MGA Online Service\n");

    UBX_U1* pMgaData = NULL;
    UBX_I4 iMgaDataSize = 0;
    mgaResult = mgaGetOnlineData(&mgaServerConfig, &pMgaData, &iMgaDataSize);

    printf("Got %d bytes from MGA\n", iMgaDataSize);

    if (mgaResult == MGA_API_OK)
    {
        printf("Retrieved Online data successfully\n");
        mgaResult = mgaSessionStart();
        assert(mgaResult == MGA_API_OK);


        printf("Sending Online data to receiver\n");
        s_ActiveMgaTransfer = true;

        // setup time adjust like this
        time_t now = time(NULL);
        tm* pNowAsTm = gmtime(&now);

        MgaTimeAdjust time;
        time.mgaAdjustType = MGA_TIME_ADJUST_ABSOLUTE;
        time.mgaYear = (UBX_U2)pNowAsTm->tm_year + 1900;
        time.mgaMonth = (UBX_U1)pNowAsTm->tm_mon + 1;
        time.mgaDay = (UBX_U1)pNowAsTm->tm_mday;
        time.mgaHour = (UBX_U1)pNowAsTm->tm_hour;
        time.mgaMinute = (UBX_U1)pNowAsTm->tm_min;
        time.mgaSecond = (UBX_U1)pNowAsTm->tm_sec;
        time.mgaAccuracyS = (UBX_U2)0;
        time.mgaAccuracyMs = (UBX_U2)0;

        mgaResult = mgaSessionSendOnlineData(pMgaData, iMgaDataSize, &time);
        if (mgaResult == MGA_API_OK)
        {
            // now wait until transfer has finished
            waitForFinish(0);
            printf("Online transfer to receiver is complete\n");

        }
        else
        {
            // failed to send, so stop session
            mgaSessionStop();
            switch (mgaResult)
            {
            case MGA_API_NO_DATA_TO_SEND:
                printf("Data form AssistNow Online service does not contain any MGA online data.");
                break;

            case MGA_API_BAD_DATA:
                printf("Bad data received from AssistNow Online service.");
                break;

            case MGA_API_NO_MGA_INI_TIME:
                printf("No MGA-INI-TIME message in data from AssistNow Online service.");
                break;

            case MGA_API_OUT_OF_MEMORY:
                printf("Out of memory.");
                break;

            default:
                assert(0);
                break;
            }
        }

        free(pMgaData);
    }
    else
    {
        // Failed to get online data from service
        switch (mgaResult)
        {
        case MGA_API_CANNOT_CONNECT:
            printf("Failed to connect to AssistNow Online server.\n");
            break;

        case MGA_API_CANNOT_GET_DATA:
            printf("Failed to get online data from service.\n");
            break;

        default:
            assert(0);
            break;
        }
    }

    if (mgaResult != MGA_API_OK)
        exit(EXIT_FAILURE);
}

// MGA offline host based test
static void offlineHostExample(CL_ARGUMENTS_pt clArgs)
{
    // Legacy aiding not supported with legacy offline test
    if (strstr(clArgs->format, "aid"))
    {
        printf("Legacy offline host based test supports only MGA format. Got '%s'.\n", clArgs->format);
        exit(EXIT_FAILURE);
    }

    // setup callback for progress and writing to receiver
    MgaEventInterface evtInterface;
    evtInterface.evtProgress = EvtProgressHandler;
    evtInterface.evtWriteDevice = EvtWriteDeviceHandler;

    // setup flow control to the receiver like this
    MgaFlowConfiguration flowConfig;
    memset(&flowConfig, 0, sizeof(flowConfig));
    flowConfig.msgTimeOut = clArgs->timeout;
    flowConfig.msgRetryCount = clArgs->retry;

    // determine the flow control from the command line arguments
    flowConfig.mgaFlowControl = MGA_FLOW_NONE;
    if (strstr(clArgs->flow, "simple"))
        flowConfig.mgaFlowControl = MGA_FLOW_SIMPLE;
    if (strstr(clArgs->flow, "smart"))
        flowConfig.mgaFlowControl = MGA_FLOW_SMART;

    MGA_API_RESULT mgaResult = mgaConfigure(&flowConfig, &evtInterface, NULL);
    assert(mgaResult == MGA_API_OK);

    // setup what Offline servers to communicate with, and what parameters to send
    MgaOfflineServerConfig mgaServerConfig;
    memset(&mgaServerConfig, 0, sizeof(mgaServerConfig));

    mgaServerConfig.strPrimaryServer = clArgs->server1;
    mgaServerConfig.strSecondaryServer = clArgs->server2;
    mgaServerConfig.strServerToken = clArgs->token;

    // specify what GNSS systems we need data for, like this
    strstr(clArgs->gnss, "gps") ? mgaServerConfig.gnssTypeFlags |= MGA_GNSS_GPS : 0;
    strstr(clArgs->gnss, "glo") ? mgaServerConfig.gnssTypeFlags |= MGA_GNSS_GLO : 0;
    strstr(clArgs->gnss, "gal") ? mgaServerConfig.gnssTypeFlags |= MGA_GNSS_GALILEO : 0;
    strstr(clArgs->gnss, "bds") ? mgaServerConfig.gnssTypeFlags |= MGA_GNSS_BEIDOU : 0;
    strstr(clArgs->gnss, "qzss") ? mgaServerConfig.gnssTypeFlags |= MGA_GNSS_QZSS : 0;

    // specify what type of GNSS we need for almanac, like this
    strstr(clArgs->alm, "gps") ? mgaServerConfig.almFlags |= MGA_GNSS_GPS : 0;
    strstr(clArgs->alm, "glo") ? mgaServerConfig.almFlags |= MGA_GNSS_GLO : 0;
    strstr(clArgs->alm, "gal") ? mgaServerConfig.almFlags |= MGA_GNSS_GALILEO : 0;
    strstr(clArgs->alm, "bds") ? mgaServerConfig.almFlags |= MGA_GNSS_BEIDOU : 0;
    strstr(clArgs->alm, "qzss") ? mgaServerConfig.almFlags |= MGA_GNSS_QZSS : 0;

    // specify more configuration options
    mgaServerConfig.period = clArgs->period;
    mgaServerConfig.resolution = clArgs->resolution;
    mgaServerConfig.numofdays = clArgs->days;

    //check if server certificate should be verified
    if (clArgs->serverVerify)
    {
        mgaServerConfig.bValidateServerCert = true;
    }

    printf("Connecting to u-blox MGA Offline Service\n");

    // get Offline data from server
    UBX_U1* pOfflineData = NULL;
    UBX_I4 offlineDataSize = 0;
    mgaResult = mgaGetOfflineData(&mgaServerConfig, &pOfflineData, &offlineDataSize);

    if (mgaResult == MGA_API_OK)
    {
        // select offline data applicable for today
        time_t now = time(NULL);
        tm* pNowAsTm = gmtime(&now);

        UBX_U1* pTodaysData = NULL;
        UBX_I4 todaysDataSize = 0;
        mgaResult = mgaGetTodaysOfflineData(pNowAsTm, pOfflineData, offlineDataSize, &pTodaysData, &todaysDataSize);

        if (mgaResult == MGA_API_OK)
        {
            s_ActiveMgaTransfer = true;
            mgaResult = mgaSessionStart();
            assert(mgaResult == MGA_API_OK);

            // setup time adjust like this
            time_t now = time(NULL);
            tm* pNowAsTm = gmtime(&now);

            //time adjustment
            MgaTimeAdjust time;
            time.mgaAdjustType = MGA_TIME_ADJUST_ABSOLUTE;
            time.mgaYear = (UBX_U2)pNowAsTm->tm_year + 1900;
            time.mgaMonth = (UBX_U1)pNowAsTm->tm_mon + 1;
            time.mgaDay = (UBX_U1)pNowAsTm->tm_mday;
            time.mgaHour = (UBX_U1)pNowAsTm->tm_hour;
            time.mgaMinute = (UBX_U1)pNowAsTm->tm_min;
            time.mgaSecond = (UBX_U1)pNowAsTm->tm_sec;
            time.mgaAccuracyS = (UBX_U2)0;
            time.mgaAccuracyMs = (UBX_U2)0;

            //position adjustments
            MgaPosAdjust posAdjust;
            posAdjust.mgaAcc = (UBX_U4)(clArgs->posAccuracy * 1e2);
            posAdjust.mgaAlt = (UBX_U4)(clArgs->posAltitude * 1e2);
            posAdjust.mgaLon = (UBX_U4)clArgs->posLongitude;
            posAdjust.mgaLat = (UBX_U4)clArgs->posLatitude;

            // send Offline data to receiver
            mgaResult = mgaSessionSendOfflineData(pTodaysData, todaysDataSize, &time, &posAdjust);

            if (mgaResult == MGA_API_OK)
            {
                // now wait until transfer has finished
                waitForFinish(0);
                printf("Offline Host transfer to receiver is complete\n");

            }
            else
            {
                // failed to send, so stop session
                mgaSessionStop();
            }
            free(pTodaysData);
        }
        else
        {
            printf("Today's offline data not found\n");
        }
        free(pOfflineData);
    }
    else
    {
        // failed to get online data from service
        switch (mgaResult)
        {
        case MGA_API_CANNOT_CONNECT:
            printf("Could not connect to service");
            break;

        case MGA_API_CANNOT_GET_DATA:
            // more detail
            printf("None - Could not get data from service");
            break;

        default:
            assert(0);
            break;
        }
    }

    if (mgaResult != MGA_API_OK)
        exit(EXIT_FAILURE);
}

// MGA offline flash based test
static void offlineFlashExample(CL_ARGUMENTS_pt clArgs)
{
    // legacy aiding not supported with offline test
    if (strstr(clArgs->format, "aid"))
    {
        printf("MGA offline flash based test supports only MGA format. Got '%s'.\n", clArgs->format);
        exit(EXIT_FAILURE);
    }

    // setup callback for progress and writing to receiver
    MgaEventInterface evtInterface;
    evtInterface.evtProgress = EvtProgressHandler;
    evtInterface.evtWriteDevice = EvtWriteDeviceHandler;

    // setup flow control to the receiver like this
    MgaFlowConfiguration flowConfig;
    memset(&flowConfig, 0, sizeof(flowConfig));
    flowConfig.msgTimeOut = clArgs->timeout;
    flowConfig.msgRetryCount = clArgs->retry;

    // determine the flow control from the command line arguments
    flowConfig.mgaFlowControl = MGA_FLOW_NONE;
    if (strstr(clArgs->flow, "simple"))
        flowConfig.mgaFlowControl = MGA_FLOW_SIMPLE;
    if (strstr(clArgs->flow, "smart"))
        flowConfig.mgaFlowControl = MGA_FLOW_SMART;

    MGA_API_RESULT mgaResult = mgaConfigure(&flowConfig, &evtInterface, NULL);
    assert(mgaResult == MGA_API_OK);

    // setup what Offline servers to communicate with, and what parameters to send
    MgaOfflineServerConfig mgaServerConfig;
    memset(&mgaServerConfig, 0, sizeof(mgaServerConfig));

    mgaServerConfig.strPrimaryServer = clArgs->server1;
    mgaServerConfig.strSecondaryServer = clArgs->server2;
    mgaServerConfig.strServerToken = clArgs->token;

    // specify what GNSS systems we need data for, like this
    strstr(clArgs->gnss, "gps") ? mgaServerConfig.gnssTypeFlags |= MGA_GNSS_GPS : 0;
    strstr(clArgs->gnss, "glo") ? mgaServerConfig.gnssTypeFlags |= MGA_GNSS_GLO : 0;
    strstr(clArgs->gnss, "gal") ? mgaServerConfig.gnssTypeFlags |= MGA_GNSS_GALILEO : 0;
    strstr(clArgs->gnss, "bds") ? mgaServerConfig.gnssTypeFlags |= MGA_GNSS_BEIDOU : 0;
    strstr(clArgs->gnss, "qzss") ? mgaServerConfig.gnssTypeFlags |= MGA_GNSS_QZSS : 0;

    // specify more configuration options
    mgaServerConfig.period = clArgs->period;
    mgaServerConfig.resolution = clArgs->resolution;
    mgaServerConfig.numofdays = clArgs->days;

    //check if server certificate should be verified
    if (clArgs->serverVerify)
    {
        mgaServerConfig.bValidateServerCert = true;
    }

    printf("Connecting to u-blox MGA Offline Service\n");

    // get Offline data from server
    UBX_U1* pOfflineData = NULL;
    UBX_I4 offlineDataSize = 0;
    mgaResult = mgaGetOfflineData(&mgaServerConfig, &pOfflineData, &offlineDataSize);

    if (mgaResult == MGA_API_OK)
    {
        s_ActiveMgaTransfer = true;
        mgaResult = mgaSessionStart();
        assert(mgaResult == MGA_API_OK);

        // send offline data to receiver's flash
        mgaResult = mgaSessionSendOfflineToFlash(pOfflineData, offlineDataSize);
        if (mgaResult == MGA_API_OK)
        {
            // now wait until transfer has finished
            waitForFinish(0);
            printf("Offline Flash transfer to receiver is complete\n");
        }
        else
        {
            // failed to send, so stop session
            mgaSessionStop();
        }
        free(pOfflineData);
    }
    else
    {
        printf("Couldn't download data\n");
    }

    if (mgaResult != MGA_API_OK)
        exit(EXIT_FAILURE);
}

// legacy offline host based test
static void legacyOfflineHostBasedTransfer(CL_ARGUMENTS_pt clArgs)
{
    // MGA aiding not supported with legacy offline test
    if (strstr(clArgs->format, "mga"))
    {
        printf("Legacy offline host based test supports only AID format. Got '%s'.\n", clArgs->format);
        exit(EXIT_FAILURE);
    }

    // setup callback for progress and writing to receiver
    MgaEventInterface evtInterface;
    evtInterface.evtProgress = EvtProgressHandler;
    evtInterface.evtWriteDevice = EvtWriteDeviceHandler;

    // setup flow control to the receiver like this
    MgaFlowConfiguration flowConfig;
    memset(&flowConfig, 0, sizeof(flowConfig));
    flowConfig.msgTimeOut = clArgs->timeout;

    MGA_API_RESULT mgaResult = mgaConfigure(&flowConfig, &evtInterface, NULL);
    assert(mgaResult == MGA_API_OK);

    // setup what Offline servers to communicate with, and what parameters to send
    MgaOfflineServerConfig mgaServerConfig;
    memset(&mgaServerConfig, 0, sizeof(mgaServerConfig));

    mgaServerConfig.strPrimaryServer = clArgs->server1;
    mgaServerConfig.strSecondaryServer = clArgs->server2;
    mgaServerConfig.strServerToken = clArgs->token;

    // check for unsupported GNSS types
    if (strstr(clArgs->gnss, "glo") ||
        strstr(clArgs->gnss, "gal") ||
        strstr(clArgs->gnss, "bds") ||
        strstr(clArgs->gnss, "qzss"))
    {
        printf("Legacy offline host based test supports only GPS. Got '%s'.\n", clArgs->gnss);
        exit(EXIT_FAILURE);
    }

    mgaServerConfig.gnssTypeFlags |= MGA_GNSS_GPS;
    mgaServerConfig.numofdays = clArgs->days;
    mgaServerConfig.useFlags |= MGA_FLAGS_USE_LEGACY_AIDING;

    //check if server certificate should be verified
    if (clArgs->serverVerify)
    {
        mgaServerConfig.bValidateServerCert = true;
    }

    printf("Connecting to u-blox MGA Offline Service\n");

    // get Offline data from server
    UBX_U1* pOfflineData = NULL;
    UBX_I4 offlineDataSize = 0;
    mgaResult = mgaGetOfflineData(&mgaServerConfig, &pOfflineData, &offlineDataSize);

    if (mgaResult == MGA_API_OK)
    {
        printf("Downloaded file size %d\n", offlineDataSize);
        mgaResult = mgaSessionStart();
        assert(mgaResult == MGA_API_OK);

        printf("Running host server for %d seconds - press any key to stop early\n", clArgs->legacyServerDuration);
        mgaResult = mgaStartLegacyAiding(pOfflineData, offlineDataSize);
        assert(mgaResult == MGA_API_OK);

        for (int i = 0; i < clArgs->legacyServerDuration; i++)
        {
            if (_kbhit())
                break;
            mySleep(TIMEOUT);
        }

        mgaStopLegacyAiding();
        assert(mgaResult == MGA_API_OK);
    }

    free(pOfflineData);

    if (mgaResult != MGA_API_OK)
        exit(EXIT_FAILURE);
}

// legacy offline host based test
static void legacyOfflineFlashExample(CL_ARGUMENTS_pt clArgs)
{
    // MGA aiding not supported with legacy offline test
    if (strstr(clArgs->format, "mga"))
    {
        printf("Legacy offline flash based test supports only AID format. Got '%s'.\n", clArgs->format);
        exit(EXIT_FAILURE);
    }

    // setup callback for progress and writing to receiver
    MgaEventInterface evtInterface;
    evtInterface.evtProgress = EvtProgressHandler;
    evtInterface.evtWriteDevice = EvtWriteDeviceHandler;

    // setup flow control to the receiver like this
    MgaFlowConfiguration flowConfig;
    memset(&flowConfig, 0, sizeof(flowConfig));
    flowConfig.msgTimeOut = clArgs->timeout;

    MGA_API_RESULT mgaResult = mgaConfigure(&flowConfig, &evtInterface, NULL);
    assert(mgaResult == MGA_API_OK);

    // setup what Offline servers to communicate with, and what parameters to send
    MgaOfflineServerConfig mgaServerConfig;
    memset(&mgaServerConfig, 0, sizeof(mgaServerConfig));

    mgaServerConfig.strPrimaryServer = clArgs->server1;
    mgaServerConfig.strSecondaryServer = clArgs->server2;
    mgaServerConfig.strServerToken = clArgs->token;

    // check for unsupported GNSS types
    if (strstr(clArgs->gnss, "glo") ||
        strstr(clArgs->gnss, "gal") ||
        strstr(clArgs->gnss, "bds") ||
        strstr(clArgs->gnss, "qzss"))
    {
        printf("Legacy offline flash based test supports only GPS. Got '%s'.\n", clArgs->gnss);
        exit(EXIT_FAILURE);
    }

    // specify more configuration options
    mgaServerConfig.gnssTypeFlags |= MGA_GNSS_GPS;
    mgaServerConfig.numofdays = clArgs->days;
    mgaServerConfig.useFlags |= MGA_FLAGS_USE_LEGACY_AIDING;

    //check if server certificate should be verified
    if (clArgs->serverVerify)
    {
        mgaServerConfig.bValidateServerCert = true;
    }

    printf("Connecting to u-blox MGA Offline Service (for legacy data)\n");

    // get Offline data from server
    UBX_U1* pOfflineData = NULL;
    UBX_I4 offlineDataSize = 0;
    mgaResult = mgaGetOfflineData(&mgaServerConfig, &pOfflineData, &offlineDataSize);

    if (mgaResult == MGA_API_OK)
    {
        printf("Downloaded file size %d\n", offlineDataSize);
        s_ActiveMgaTransfer = true;
        mgaResult = mgaSessionStart();
        assert(mgaResult == MGA_API_OK);

        // send legacy offline data to receiver's flash
        mgaResult = mgaSessionSendLegacyOfflineToFlash(pOfflineData, offlineDataSize);

        if (mgaResult == MGA_API_OK)
        {
            // now wait until transfer has finished
            waitForFinish(0);
            printf("Legacy Offline Flash transfer to receiver is complete\n");
        }
        else
        {
            // failed to send, so stop session
            mgaSessionStop();
        }
        free(pOfflineData);
    }
    else
    {
        printf("Couldn't download data\n");
    }

    if (mgaResult != MGA_API_OK)
        exit(EXIT_FAILURE);
}

// main entry point
// will return 0 on success or 1 on failure
int main(int argc, char* argv[])
{
    //register exit function
    atexit(closeSerial);

    // unused variable
    (void)knownArgs;

    // get the arguments
    CL_ARGUMENTS_t clArgs;

    // get the executable name
    exename = argv[0];

    // check for missing arguments
    if (argc == 1)
    {
        // print help
        printUsage();
        exit(EXIT_SUCCESS);
    }

    // parse the arguments
    if (!parseArguments(argc, argv, &clArgs))
    {
        exit(EXIT_FAILURE);
    }

    printf("\n");
    printf("NAME\n");
    printf("    libMGA Example (%s)\n", argv[0]);
    printf("    using libMGA v%s\n", mgaGetVersion());
    printf("\n");


    // print the used configuration
    printConfiguration(&clArgs);

    // Make sure Winsock is initialized
#ifdef _WIN32
    WSADATA  wsaData;
    if (!WSAStartup(MAKEWORD(2, 2), &wsaData) == 0)
    {
        printf("Could not initialize socket library");
        exit(EXIT_FAILURE);
    }
#endif

    // setup to talk to the serial port
    if (!openSerial(clArgs.port, clArgs.baudrate))
    {
        printf("Serial port '%s' could not be opened\n", clArgs.port);
        exit(EXIT_FAILURE);
    }

    MGA_API_RESULT mgaResult = mgaInit();
    if (mgaResult != MGA_API_OK)
    {
        printf("mgaInit failed\n");
        exit(EXIT_FAILURE);
    }

    startDeviceReadThread();

    if (strcmp(clArgs.test, "online") == 0)
    {
        onlineExample(&clArgs);
    }
    else if (strcmp(clArgs.test, "offline") == 0)
    {
        offlineHostExample(&clArgs);
    }
    else if (strcmp(clArgs.test, "flash") == 0)
    {
        offlineFlashExample(&clArgs);
    }
    else if (strcmp(clArgs.test, "legacyhost") == 0)
    {
        legacyOfflineHostBasedTransfer(&clArgs);
    }
    else if (strcmp(clArgs.test, "legacyflash") == 0)
    {
        legacyOfflineFlashExample(&clArgs);
    }
    else
    {
        printf("UNKOWN TEST.\n");
    }


    // stop the reader thread and cleanup
    stopDeviceReadThread();
    mgaDeinit();

    printf("Test finished.\n");
    return(EXIT_SUCCESS);
}


