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
  \brief  SUPL state machine

  Managing and allocation of state machines for SUPL
*/

#include "suplSMmanager.h"
#include "gps_thread.h"
#include "std_types.h"
#include "supl_connection.h"
#include "ubx_agpsIf.h"
#include "ubx_localDb.h"
#include "ubx_log.h"
#include "ubx_niIf.h"
#include "ubx_timer.h"
#include "ubxgpsstate.h"
#include "upldecod.h"
#include "uplsend.h"
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <openssl/hmac.h>

///////////////////////////////////////////////////////////////////////////////
// Types & Definitions

//! states of the SUPL SM
typedef enum
{
  START,      //!< empty start state, after SM alvocated
  NO_NETWORK, //!< Waiting for network to become available
  AUTH,       //!< Waiting for authorization from UI
  WAIT_RES,   //!< Wait for sesponce after a SUPLSTART sent
  RRLP,       //!< RRLP session state
  RRLP_RESP,  //!< RRLP response session state
  END         //!< empty end state, after the SM can be deallocated
} SMState_t;

#define MAX_SM_INSTANCES 5 //!< Maximum number of SUPL stame machine instances
// If this is changed to > 1, then the position publish to framework
// control mechanism needs to be made smarter to allow for multiple
// supl sessions.

#define START_SID_NUMBER 1001                   //!< Start Session Id number
#define MAX_SID_NUMBER (START_SID_NUMBER + 200) //!< Max SID number

#define MAX_UPL_PACKET 5000 //!< Dimension of the max UPL packet coming from the network

#define SUPL_STD_TIMEOUT 10           //!< the default timeout, for the SUPL specs, is 10 seconds
#define SUPL_NOTIFICATION_TIMEOUT 120 //!< Timeout default for the notification from UI, 2 minutes
#define SUPL_POLL_INTERVAL (3600 * 24 * 1000) //!< 1 day - what should this really be? TODO
#define SUPL_NONETWORK_TIMEOUT                                                                     \
  60 //!< 60 seconds timeout if Supl transaction requested but no network

//! Handler for the state machine instancies */
typedef struct suplHandler
{
  int sid;                             //!< Unique session identifier ==  SET ID
  SlpSessionID_t *pSlpId;              //!< must be linked with the SLP ID
  SetSessionID_t *pSetId;              //!< must be linked with the SLP ID
  SMState_t state;                     //!< state of the current SM instance
  SUPL_CONNECTION *supl_connection;    //!< SUPL connection handler
  long long hash;                      //!< Hash of the SUPL INIT
  NotificationType_t notificationType; //!< Notification type of SUPL INIT
  EncodingType_t encodingType;         //!< Notification texts encoding type
  OCTET_STRING_t *pRequestorId;        //!< Pointer to received requestor id text. Used
                                       //!for notify/verify request to framework
  FormatIndicator_t requestorIdType;   //!< Received requestor id text type. Used
                                       //!for notify/verify request to framework
  OCTET_STRING_t *pClientName;         //!< Pointer to received client name text. Used
                                       //!for notify/verify request to framework
  FormatIndicator_t clientNameType;    //!< Received requestor client name type.
                                       //!Used for notify/verify request to
                                       //!framework
  time_t timeout;                      //!< socket file descriptor of the current session
  int rrlpRefNum;                      //!< rrlp reference number for delayed responce
  int nonProxy;                        //!< The server is requesting a non proxy mode
  int reqHorAccuracy;                  //!< Requested Horizontal accuracy
  int reqVerAccuracy;                  //!< Requested Vertical accuracy
  struct suplHandler *pNext;           //!< Pointer to the next handler active in the linked list
  RrlpRespData_t responseType;         //!< Type of pending response
  int msaPosResponseTime;              //!< The time to wait before responsing to server with
                                       //!MSA data
  int QopDelay;                        //!< Delay trieved from Quality of Position 'delay' field in
                                       //!SUPLINIT
  PosMethod_t requestedPosMethod;      //!< The position method to be used thoughout
                                       //!the transaction
  bool networkInitiated;               //!< true if the session is an NI one, false if SI
  bool assistanceRequested;            //!< true is assitance data was requested from the
                                       //!server, false if not
  struct ULP_PDU *pNiMsg;              //!< Pointer NI supl init message
} suplHandler_t;

///////////////////////////////////////////////////////////////////////////////
// Static data
std::vector<std::string> stateInfo = {
  //!< Logging texts for Supl states
  "START",      //
  "NO_NETWORK", //
  "AUTH",       //
  "WAIT_RES",   //
  "RRLP",       //
  "RRLP_RESP",  //
  "END",        //
};

std::vector<std::string> strPosMethodType = {
  //!< Logging texts for position method
  "agpsSETassisted",     //
  "agpsSETbased",        //
  "agpsSETassistedpref", //
  "agpsSETbasedpref",    //
  "autonomousGPS",       //
  "aFLT",                //
  "eCID",                //
  "eOTD",                //
  "oTDOA",               //
  "noPosition",          //
};

std::vector<std::string> strUlpMessageType = {
  //!< Logging texts for Supl message types
  "NOTHING",          //
  "msSUPLINIT",       //
  "msSUPLSTART",      //
  "msSUPLRESPONSE",   //
  "msSUPLPOSINIT",    //
  "msSUPLPOS",        //
  "msSUPLEND",        //
  "PR_msSUPLAUTHREQ", //
  "msSUPLAUTHRESP",   //
};

static suplHandler_t *s_pQueueTail = NULL; //!< Tail of the supl sessions list
static int64_t s_lastSuplSiTime = 0;       //!< Last time a Supl SI session was performed
static GpsControlEventInterface *s_pGpsControlInterface = NULL; //!< Interface to Gps engine control
static void *s_pEventContext = NULL; //!< Pointer to context for Gps engine control interface calls
static std::mutex s_handlerMutex;    //!< Handler list mutex

///////////////////////////////////////////////////////////////////////////////
// Local functions
static int suplSM(suplHandler_t *pHandler, struct ULP_PDU *pMsg, SuplCmd_t cmd);
static suplHandler_t *allocateSM(bool ni);
static bool suplInsertHandler(suplHandler_t *pHandler, const SetSessionID_t *pSetId);
static suplHandler_t *searchSMInst(int sid, bool lock);
static void endSuplSession(suplHandler_t *pHandler);
static void freeSuplHandler(suplHandler_t *pHandler);
static int getNewSid(void);
static void processRawSuplMessage(const char *buffer, int size, suplHandler_t *pReceivedHandler);
static long long calculateHash(const char *buffer, int size);
static int createUplSession(suplHandler_t *pHandler);
static void generateSuplEndMsg(suplHandler_t *pHandler, StatusCode statusCode);
static int sendPositionResponse(suplHandler_t *pHandler, char *pSendBuffer, int size);
static void setNotificationAndResponse(NotificationType_t ans1cNotifyType,
                                       GpsNiNotifyFlags *pNotifyFlags,
                                       GpsUserResponseType *pDefaultResponse);
static GpsNiEncodingType convertAsn1Encoding(EncodingType_t encodingType);
static void copyAndConvertText(char *pDest, int maxLen, const OCTET_STRING_t *pSuplString);
static bool isSuplPossible(void);
static void determineAssistance(suplHandler_t *pHandler, suplPosInitParam_t *pPosInitParams);
static int startSiSession(suplHandler_t *pHandler);
static void startNiSession(suplHandler_t *pHandler);
static bool compareOctets(const OCTET_STRING_t *pOct1, const OCTET_STRING_t *pOct2);
static bool verifySlpSessionId(SlpSessionID_t *pSlpSessionId_1, SlpSessionID_t *pSlpSessionId_2);
static bool verifySetSessionId(const SetSessionID_t *pSetSessionId_1,
                               const SetSessionID_t *pSetSessionId_2);
static void safeFreeHandler(suplHandler_t *pHandler);

///////////////////////////////////////////////////////////////////////////////
//! Register the event interface
/*! Function to register with the Supl session manager the Gps engine control
    event interface.
  \param pEventInterface 	: Pointer to the evenp interface
  \param pContext			: Pointer to the context to use in all calls
  to event the event interface
*/
void suplRegisterEventCallbacks(GpsControlEventInterface *pEventInterface, void *pContext)
{
  s_pGpsControlInterface = pEventInterface;
  s_pEventContext = pContext;
}

///////////////////////////////////////////////////////////////////////////////
//! Function checking if there are any active Supl sessions
/*! Function checking if there are any active Supl sessions
  \return      : true is there are sessions active, false if not
*/
bool suplActiveSessions(void)
{
  assert(pthread_self() == g_gpsDrvMainThread);
  return s_pQueueTail != NULL;
}

///////////////////////////////////////////////////////////////////////////////
//! Counts the number of SI or NI sessions
/*!
  \param ni	: true if NI sessions to be counted, false if SI sessions to be
  counted
  \return   : the number of NI or SI sessions active
*/
int suplCountSessions(bool ni)
{
  int count = 0;
  std::lock_guard<std::mutex> lock(s_handlerMutex);
  suplHandler_t *pHandler = s_pQueueTail;
  while (pHandler != NULL)
  {
    if ((pHandler->networkInitiated && ni) || (!pHandler->networkInitiated && !ni))
    {
      count++;
    }
    pHandler = pHandler->pNext;
  }
  return count;
}

///////////////////////////////////////////////////////////////////////////////
//! Function checking if the SET needs to initiate a SUPL transaction
/*! this function is used to be called regularly to check and if necessary
    initiate a SUPL transaction from the SET
*/
void suplCheckForSiAction(void)
{
  assert(pthread_self() == g_gpsDrvMainThread);

  if (CGpsIf::getInstance()->getMode() != GPS_POSITION_MODE_STANDALONE)
  {
    // Need to check for possible start of a SUPL transaction
    int64_t timeNowMs = getMonotonicMsCounter();
    // UBX_LOG(LCAT_VERBOSE, "%lli %lli", timeNowMs, s_lastSuplSiTime);

    if (timeNowMs > s_lastSuplSiTime)
    {
      UBX_LOG(LCAT_VERBOSE, "SUPL SI");
      suplStartSetInitiatedAction();

      s_lastSuplSiTime = timeNowMs + SUPL_POLL_INTERVAL;
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
//! Function for handling pending SUPL transaction
/*! This function is has to be called regularly for checking timeouts or
   fulfilments of
    pending SUPL transactions
*/
void suplCheckPendingActions(void)
{
  assert(pthread_self() == g_gpsDrvMainThread);
  suplHandler_t *pHandler;
  {
    std::lock_guard<std::mutex> lock(s_handlerMutex);
    pHandler = s_pQueueTail;
  }

  CDatabase *pDatabase = CAndroidDatabase::getInstance();
  assert(pDatabase != NULL);

  /* browse the list of handler */
  while (pHandler != NULL)
  {
    /* save next in the chain */
    suplHandler_t *pNext;
    {
      std::lock_guard<std::mutex> lock(s_handlerMutex);
      pNext = pHandler->pNext;
    }

    /* check for timeout of existing SUPL transactions */
    time_t now = time(NULL);

    if ((pHandler->state == START) && (pHandler->networkInitiated))
    {
      // Get the NI session going
      if (suplSM(pHandler, pHandler->pNiMsg, SUPL_NO_CMD) < 0)
      {
        endSuplSession(pHandler); // error/supl session ended, deallocate handler
      }
    }
    else if ((pHandler->timeout != -1) && (pHandler->timeout <= now))
    {
      // Normal timeout
      UBX_LOG(LCAT_VERBOSE, "SUPL Timer expired");
      /* Clear the timer */
      pHandler->timeout = -1;

      /* send the message to the state machine */
      suplSM(pHandler, NULL, SUPL_TIMEOUT);

      /* After timeout,  session always ended, deallocate handler */
      endSuplSession(pHandler);
    }
    else if (pHandler->state == RRLP_RESP)
    {
      if (pHandler->responseType == RESP_POSITION_DATA)
      {
        /* check if we can fulfil any pending SUPL transactions requiring
         * position information */
        double lat, lon = 0.0, speed;
        int accuracy = pHandler->reqHorAccuracy;
        bool posAvail = (pDatabase->GetOutput(CDatabase::DATA_LATITUDE_DEGREES, lat) &&
                         pDatabase->GetOutput(CDatabase::DATA_LONGITUDE_DEGREES, lon));
        bool speedAvail = pDatabase->GetOutput(CDatabase::DATA_SPEED_KNOTS, speed);

        pDatabase->GetOutput(CDatabase::DATA_ERROR_RADIUS_METERS, accuracy);
        /*UBX_LOG(LCAT_VERBOSE, "Pos response pending - PA %i  SA %i  Acc %i
           RHAcc %i  RVAcc %i",
                       posAvail, speedAvail, accuracy, pHandler->reqHorAccuracy,
           pHandler->reqVerAccuracy);
        */
        if ((posAvail) && (speedAvail) && (accuracy < pHandler->reqHorAccuracy) &&
            (pHandler->reqVerAccuracy == -1 || accuracy < pHandler->reqVerAccuracy))
        {
          UBX_LOG(LCAT_VERBOSE,
                  "Position available with accuracy %d, lat %f, lon %f, speed %f",
                  accuracy,
                  lat,
                  lon,
                  speed);

          /* send the message to the state machine */
          if (suplSM(pHandler, NULL, SUPL_POS_AVAIL) < 0)
          {
            /* error/supl session ended, deallocate handler */
            endSuplSession(pHandler);
          }
        }
      }
      else if (pHandler->responseType == RESP_MSA_DATA)
      {
        /*UBX_LOG(LCAT_VERBOSE, "MSA response still pending - delay %ld - TO
           %ld",
                 pHandler->msaPosResponseTime - now,
                 pHandler->timeout - now);
        */
        if (pHandler->msaPosResponseTime < now)
        {
          UBX_LOG(LCAT_VERBOSE, "MSA request timeout - send what data we have");
          if (suplSM(pHandler, NULL, SUPL_MSA_DATA_AVAIL) < 0)
          {
            /* error/supl session ended, deallocate handler */
            endSuplSession(pHandler);
          }
        }
      }
      else
      {
        // Nothing ready
      }
    }
    else if (pHandler->state == NO_NETWORK)
    {
      // Session pending
      if (CRilIf::getInstance()->isConnected())
      {
        if (suplSM(pHandler, NULL, SUPL_NETWORK_CONNECTED) < 0)
        {
          /* error/supl session ended, deallocate handler */
          endSuplSession(pHandler);
        }
      }
    }

    /* go to the next */
    pHandler = pNext;
  }
}

///////////////////////////////////////////////////////////////////////////////
//! Add Supl sockets to listen on
/*! Function used to add the sockets of any existing SUPL transactions to the
  list
    of sockets 'select' will listen on
  \param pRfds  : pointer to file descriptor array (used by 'select') to added
  SUPL sockets to
  \param pMaxFd : pointer allowing the largest file descriptor in the array to
  be returned
  \return       : 1 if queue empty, 0 if not
*/
int suplAddUplListeners(fd_set *pRfds, int *pMaxFd)
{
  assert(pthread_self() == g_gpsDrvMainThread);
  assert(pRfds);
  assert(pMaxFd);

  /* check if the handler queue is empty! */
  if (s_pQueueTail == NULL)
  {
    return 1;
  }

  suplHandler_t *pHandler = s_pQueueTail;

  /* browse the list of handler */
  while (pHandler != NULL)
  {
    int fd;

    if (pHandler->supl_connection)
    {
      fd = supl_connection_get_fd(pHandler->supl_connection);

      if (fd >= 0)
      {
        FD_SET(fd, pRfds);

        if ((fd + 1) > *pMaxFd)
          *pMaxFd = fd + 1;
      }
    }

    {
      std::lock_guard<std::mutex> lock(s_handlerMutex);
      pHandler = pHandler->pNext;
    }
  }

  return 0;
}

///////////////////////////////////////////////////////////////////////////////
//! Function  for reading a SUPL socket
/*! This function must be used to read any input on a SUPL socket
  \param pRfds : Pointer to file descriptor array
  \return      : 1 if session queue is empty, 0 if not
*/
int suplReadUplSock(fd_set *pRfds)
{
  assert(pthread_self() == g_gpsDrvMainThread);
  assert(pRfds);

  /* check if the handler queue is empty! */
  if (s_pQueueTail == NULL)
  {
    return 1;
  }

  suplHandler_t *pHandler = s_pQueueTail;
  char buffer[MAX_UPL_PACKET];
  int res;

  /* browse the list of handler */
  for (suplHandler_t *pNext; pHandler; pHandler = pNext)
  {
    int fd = -1;

    if (pHandler->supl_connection != NULL)
      fd = supl_connection_get_fd(pHandler->supl_connection);

    /* Store the next pointer */
    {
      std::lock_guard<std::mutex> lock(s_handlerMutex);
      pNext = pHandler->pNext;
    }

    if (fd < 0)
      continue;

    if (!FD_ISSET(fd, pRfds))
      continue;

    UBX_LOG(LCAT_VERBOSE, "Received data over UPL socket %d", fd);

    res = supl_connection_read(pHandler->supl_connection, (unsigned char *)buffer, MAX_UPL_PACKET);

    UBX_LOG(LCAT_VERBOSE, "read result is %d", res);

    if (res > MAX_UPL_PACKET)
    {
      UBX_LOG(LCAT_ERROR, "Unexpected reading size!");
      continue;
    }

    if (res <= 0)
    {
      endSuplSession(pHandler);
      continue;
    }

    processRawSuplMessage(buffer, res, pHandler);
  }

  return 0;
  // lint -e{818} remove Pointer parameter 'pRfds' (line 445) could be declared
  // as pointing to const
}

///////////////////////////////////////////////////////////////////////////////
//! Function to start (initiate) a SUPL transaction
/*! Function to start (initiate) a SUPL transaction
*/
bool suplStartSetInitiatedAction(void)
{
  assert(pthread_self() == g_gpsDrvMainThread);

  CRilIf::getInstance()->requestCellInfo();
  if (!isSuplPossible())
  {
    UBX_LOG(LCAT_VERBOSE, "SI failed - SUPL not possible");
    return false;
  }

  /* get new instance of SET Initiated SM */
  suplHandler_t *pHandler = allocateSM(false);
  if (pHandler == NULL)
  {
    UBX_LOG(LCAT_ERROR, "no more instance set available!!! error");
    return false;
  }

  bool success = true;
  if (suplInsertHandler(pHandler, NULL))
  {

    logAgps.write(0x00000000, "%d # a-gps session starts", pHandler->sid);
    logAgps.write(
      0x00000005, "%d # u-blox, 1.0.0, " __DATE__ " # // version number", pHandler->sid);

    if (suplSM(pHandler, NULL, SUPL_ASK_FOR_AGPS) < 0)
    {
      /* error/supl session ended, deallocate handler */
      endSuplSession(pHandler);
      success = false;
    }
  }
  else
  {
    // Could not insert into handler list - Free basic handler structure
    safeFreeHandler(pHandler);
    success = false;
  }

  return success;
}

///////////////////////////////////////////////////////////////////////////////
//! Network initiated message received (usually from SMS)
/*! If the network has initiated a SUPL transaction (a SUPLINIT message is
  received),
    this function must be called
  \param buffer: Pointer to buffer containing Supl message
  \param size  : Size of the buffer
*/
void suplHandleNetworkInitiatedAction(const char *buffer, int size)
{
  assert(buffer);

  // directly digest the gsm message
  // Make sure we have up to date cell info
  CRilIf::getInstance()->requestCellInfo();
  if (isSuplPossible())
  {
    processRawSuplMessage(buffer, size, NULL);
  }
  else
  {
    UBX_LOG(LCAT_VERBOSE, "NI failed - SUPL not possible");
  }
}

///////////////////////////////////////////////////////////////////////////////
//! Handle injection of authorisation into an existing session
/*!
  \param sid : session ID
  \param cmd : Session command to execute
*/
void suplHandleAuthorization(int sid, int cmd)
{
  if (pthread_self() != g_gpsDrvMainThread)
  {
    UBX_LOG(LCAT_ERROR,
            "Not in the main Thread (%.8X %.8X)",
            (unsigned int)pthread_self(),
            (unsigned int)g_gpsDrvMainThread);
  }
  suplHandler_t *pHandler = searchSMInst(sid, true); // Need to use lock

  if (pHandler)
  {
    if (suplSM(pHandler, NULL, (SuplCmd_t)cmd) < 0)
    {
      /* error/supl session ended - deallocate handler */
      endSuplSession(pHandler);
    }
  }
  else
  {
    UBX_LOG(LCAT_ERROR, "No existing session with sid %i", sid);
  }
}

///////////////////////////////////////////////////////////////////////////////
// Static functions

//! Process a buffer containing a Supl message
/*!
  \param buffer: Pointer to payload buffer containing SUPL message
  \param size  : Size of the buffer
*/
static void processRawSuplMessage(const char *buffer, int size, suplHandler_t *pReceivedHandler)
{
  assert(buffer);

  suplHandler_t *pHandler = NULL;

  /* The decode the incoming message */
  struct ULP_PDU *pMsg = uplDecode(buffer, size);

  if (pMsg == NULL)
  {
    // Decode failed/no message decoded
    UBX_LOG(LCAT_VERBOSE, "Msg not decoded");
    return;
  }
  UBX_LOG(LCAT_VERBOSE,
          "Msg decoded. Msg type %i(%s)",
          pMsg->message.present,
          _LOOKUPSTR(pMsg->message.present, UlpMessageType));

  if (pMsg->message.present == UlpMessage_PR_msSUPLINIT)
  {
    assert(pthread_self() != g_gpsDrvMainThread); // Should be on an arbitary thread
    UBX_LOG(LCAT_VERBOSE, "SLP (NI) initiated session...");
    if (suplCountSessions(true) > 0)
    {
      // There are existing NI sessions - reject this request
      UBX_LOG(LCAT_VERBOSE, "Existing NI session. Rejecting request");
    }
    else
    {
      // Need to prepare handler structure & then insert in to handler list
      // After insertion, main thread can change handler's contents
      pHandler = allocateSM(true);
      if (pHandler == NULL)
      {
        UBX_LOG(LCAT_VERBOSE, "no more instance slp available!!! error");
        UBX_LOG(LCAT_VERBOSE, "Deallocate messege %p", pMsg);
        ASN_STRUCT_FREE(asn_DEF_ULP_PDU, pMsg); /* Deallocate the message decoded */
      }
      else
      {
        // Handler structure allocated - fill in some more
        /* calcolate the hash here... */
        pHandler->hash = calculateHash(buffer, size);
        UBX_LOG(LCAT_VERBOSE,
                "%d NiMsg saved here %p in thread %u",
                __LINE__,
                pMsg,
                (unsigned int)pthread_self());
        pHandler->pNiMsg = pMsg; // Save until NI session is started

        if (suplInsertHandler(pHandler, pMsg->sessionID.setSessionID))
        {
          // Handler contents inserted into handler list and can now be changed
          // by main thread
          // This also means the Supl session has begun from our point of view
          logAgps.write(0x00000000, "%d # a-gps session starts", pHandler->sid);
          logAgps.write(
            0x00000005, "%d # u-blox, 1.0.0, " __DATE__ " # // version number", pHandler->sid);

          if (s_pGpsControlInterface)
          {
            s_pGpsControlInterface->requestStart_cb(s_pEventContext);
          }
        }
        else
        {
          // Could not insert into handler list
          UBX_LOG(LCAT_VERBOSE, "%d Deallocate messege %p", __LINE__, pMsg);
          ASN_STRUCT_FREE(asn_DEF_ULP_PDU, pMsg); // Deallocate the decoded message
          safeFreeHandler(pHandler);
        }
      }
    }
  }
  else
  {
    // Not the SLPINIT - extracting the session Id
    if (pthread_self() != g_gpsDrvMainThread)
    {
      /* unexpected, not from the main thread */
      UBX_LOG(LCAT_WARNING, "Wrong thread");
    }
    else
    {
      int sid = extractSid(pMsg);
      UBX_LOG(LCAT_VERBOSE, "sid %d", sid);
      pHandler = searchSMInst(sid, true);

      if (pHandler == NULL)
      {
        if (pReceivedHandler != NULL)
        {
          /* Received something over established connection but no associated
           * handler */
          if (suplSM(pReceivedHandler, pMsg, SUPL_NO_CMD) < 0)
          {
            /* error/supl session ended, deallocate handler */
            endSuplSession(pReceivedHandler);
          }
        }
        else
        {
          /* No instance associated!!! */
          UBX_LOG(LCAT_VERBOSE, "No session with sid %d ", sid);
        }
      }
      else
      {
        if (suplSM(pHandler, pMsg, SUPL_NO_CMD) < 0)
        {
          /* error/supl session ended, deallocate handler */
          endSuplSession(pHandler);
        }
      }
      /* Deallocate the message decoded */
      UBX_LOG(LCAT_VERBOSE, "Deallocate messege %p", pMsg);
      ASN_STRUCT_FREE(asn_DEF_ULP_PDU, pMsg);
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
//! Process the state machine for a supl session
/*!
  \param pHandler : Pointer to state machine structure for a supl session
  \param pMsg     : Pointer to a structure conatining the decoded Supl message
  \param cmd      : Cmd to the state machine
  \return 0 session is continuing, < 0 session is finished
*/
static int suplSM(suplHandler_t *pHandler, struct ULP_PDU *pMsg, SuplCmd_t cmd)
{
  assert(pHandler);
  assert(pthread_self() == g_gpsDrvMainThread);
  int res = 0; // Success by default

  UBX_LOG(LCAT_VERBOSE,
          "SM entry with Id = %d in state %s (%d) with command %d",
          pHandler->sid,
          stateInfo.at(pHandler->state).c_str(),
          pHandler->state,
          cmd);

  /* State selector */
  switch (pHandler->state)
  {
  case START:
    UBX_LOG(LCAT_VERBOSE, "Handling START");

    if (pMsg == NULL)
    {
      // No incoming msg, so expecting a command from device
      if (cmd == SUPL_ASK_FOR_AGPS)
      {
        // This is Set Initiation (SI)
        logAgps.write(0x10000000, "%d # network connecting...", pHandler->sid);
        if (CRilIf::getInstance()->isConnected())
        {
          res = startSiSession(pHandler);
        }
        else
        {
          // Supl session is pending
          UBX_LOG(LCAT_VERBOSE, "No network. Deferring SI until network present");
          pHandler->state = NO_NETWORK;
          pHandler->timeout = time(NULL) + SUPL_NONETWORK_TIMEOUT;
        }
      }
      else
      {
        UBX_LOG(LCAT_VERBOSE, "Unexpected command %d", cmd);
        pHandler->state = END;
        res = -1;
      }
    }
    else
    {
      /* Message coming from Network (NI hopefully) */
      if (pMsg->message.present == UlpMessage_PR_msSUPLINIT)
      {
        UBX_LOG(LCAT_VERBOSE, "Received SUPLINIT <====");
        logAgps.write(
          0x02000000,
          "%d, 1.0.0, SUPL_INIT # SUPL_MODE : NI_%s",
          pHandler->sid,
          ((pMsg->message.choice.msSUPLINIT.posMethod == PosMethod_agpsSETbased) ||
           (pMsg->message.choice.msSUPLINIT.posMethod == PosMethod_agpsSETbasedpref)) ?
            "MSB" :
            ((pMsg->message.choice.msSUPLINIT.posMethod == PosMethod_agpsSETassisted) ||
             (pMsg->message.choice.msSUPLINIT.posMethod == PosMethod_agpsSETassistedpref)) ?
            "MSA" :
            "? ? ?");
        // This is a Network Initiation (NI)
        UBX_LOG(LCAT_VERBOSE, "method requested is %ld", pMsg->message.choice.msSUPLINIT.posMethod);
        /* check the positioning method */
        if (pMsg->message.choice.msSUPLINIT.posMethod != PosMethod_agpsSETbased &&
            pMsg->message.choice.msSUPLINIT.posMethod != PosMethod_agpsSETassistedpref &&
            pMsg->message.choice.msSUPLINIT.posMethod != PosMethod_agpsSETbasedpref &&
            pMsg->message.choice.msSUPLINIT.posMethod != PosMethod_agpsSETassisted &&
            pMsg->message.choice.msSUPLINIT.posMethod != PosMethod_eCID &&
            pMsg->message.choice.msSUPLINIT.posMethod != PosMethod_noPosition)
        {
          UBX_LOG(LCAT_VERBOSE, "The position method is not supported!");
          // As we haven't yet opened a socket to the server, we have no
          // communications
          // channel, so do nothing. Network will timeout.
          pHandler->state = END;
          res = -1;
          break;
        }

        /* check if non proxy mode! */
        if (pMsg->message.choice.msSUPLINIT.sLPMode == SLPMode_nonProxy)
        {
          UBX_LOG(LCAT_VERBOSE, "Only Proxy mode is supported!");
          // As we haven't yet opened a socket to the server, we have no
          // communications
          // channel, so do nothing. Network will timeout.
          pHandler->state = END;
          res = -1;
          break;
        }

        // Notification
        pHandler->notificationType = NotificationType_noNotificationNoVerification;
        pHandler->encodingType = -1;
        pHandler->pRequestorId = NULL;
        pHandler->requestorIdType = -1;
        pHandler->pClientName = NULL;
        pHandler->clientNameType = -1;

        /* Notification type */
        if (pMsg->message.choice.msSUPLINIT.notification != NULL)
        {
          UBX_LOG(LCAT_VERBOSE,
                  "notification type %ld",
                  pMsg->message.choice.msSUPLINIT.notification->notificationType);
          pHandler->notificationType =
            pMsg->message.choice.msSUPLINIT.notification->notificationType;

          if (pMsg->message.choice.msSUPLINIT.notification->encodingType != NULL)
          {
            pHandler->encodingType = *pMsg->message.choice.msSUPLINIT.notification->encodingType;
          }
          pHandler->pRequestorId = pMsg->message.choice.msSUPLINIT.notification->requestorId;
          if (pMsg->message.choice.msSUPLINIT.notification->requestorIdType != NULL)
          {
            pHandler->requestorIdType =
              *pMsg->message.choice.msSUPLINIT.notification->requestorIdType;
          }

          pHandler->pClientName = pMsg->message.choice.msSUPLINIT.notification->clientName;
          if (pMsg->message.choice.msSUPLINIT.notification->clientNameType != NULL)
          {
            pHandler->clientNameType =
              *pMsg->message.choice.msSUPLINIT.notification->clientNameType;
          }
        }

        /* default no QOP requested */
        pHandler->reqVerAccuracy = -1;
        pHandler->reqHorAccuracy = -1;

        /* check if there is QOP field */
        if (pMsg->message.choice.msSUPLINIT.qoP != NULL)
        {
          /* QOP present */
          if (pMsg->message.choice.msSUPLINIT.qoP->delay != NULL)
          {
            pHandler->QopDelay = 1 << *pMsg->message.choice.msSUPLINIT.qoP->delay;
            UBX_LOG(LCAT_VERBOSE, "Qop delay %d", pHandler->QopDelay);
          }
          else
          {
            /* No delay */
            UBX_LOG(LCAT_VERBOSE, "No Qop delay in SUPLINIT");
          }

          pHandler->reqHorAccuracy =
            (int)(10 * (pow(1.1, (double)pMsg->message.choice.msSUPLINIT.qoP->horacc) - 1));
          UBX_LOG(LCAT_VERBOSE, "Horizontal QOP requested %d", pHandler->reqHorAccuracy);
          if (pMsg->message.choice.msSUPLINIT.qoP->veracc != NULL)
          {
            /* vertical accuracy is present */
            pHandler->reqVerAccuracy = *pMsg->message.choice.msSUPLINIT.qoP->veracc;
            UBX_LOG(LCAT_VERBOSE, "Verical QOP requested %d", pHandler->reqVerAccuracy);
          }
          else
          {
            /* No vertical accuracy */
            UBX_LOG(LCAT_VERBOSE, "No vertical accuracy requested in SUPLINIT");
          }
          /*		OPTIONAL
          long	*veracc;
          long	*maxLocAge;
          long	*delay;
          */
          logAgps.write(0x00000002,
                        "%d, %d, %d, %d, %d # QoP",
                        pHandler->sid,
                        pHandler->reqHorAccuracy,
                        pHandler->reqVerAccuracy,
                        0,
                        1,
                        1); // todo fix this
        }
        else
        {
          // No H or V accuracies
          UBX_LOG(LCAT_VERBOSE, "No QOP requested in SUPLINIT");
        }

        if (pMsg->message.choice.msSUPLINIT.sLPMode == SLPMode_nonProxy)
        {
          /* Not possible, will be given a not possible! - Deferred error
           * reporting */
          pHandler->nonProxy = 1;
        }
        // Copy position method
        pHandler->requestedPosMethod = pMsg->message.choice.msSUPLINIT.posMethod;

        /* copy slpSessionId*/
        pHandler->pSlpId = copySlpId(pMsg->sessionID.slpSessionID);

        // Timeout is set to UI notifcation timeout, which is handled else where
        pHandler->timeout = -1;

        logAgps.write(0x10000000, "%d # network connecting...", pHandler->sid);
        if (CRilIf::getInstance()->isConnected())
        {
          /* check if setid not set (must be empty) */
          if (pMsg->sessionID.setSessionID != NULL || pMsg->sessionID.slpSessionID == NULL)
          {
            /* Create a UPL session */
            if (createUplSession(pHandler) == -1)
            {
              UBX_LOG(LCAT_VERBOSE, "Cannot create the session");
            }
            else
            {
              UBX_LOG(LCAT_VERBOSE, "error in session ID");
              generateSuplEndMsg(pHandler, StatusCode_invalidSessionId);
            }
            pHandler->state = END;
            res = -1;
            break;
          }
          else
          {
            if (pMsg->message.choice.msSUPLINIT.posMethod == PosMethod_noPosition)
            {
              /* Create a UPL session */
              if (createUplSession(pHandler) == -1)
              {
                UBX_LOG(LCAT_VERBOSE, "Cannot create the session");
              }
              else
              {
                generateSuplEndMsg(pHandler, StatusCode_unspecified);
              }
              pHandler->state = END;
              res = -1;
              break;
            }
            else
            {

              // Ask for notification from the UI
              startNiSession(pHandler); // Can recursively call suplSM

              // Do not do any state handling here
              // Also pHandler could be a floating pointer if an error occurred
              // in recursed suplSM called
            }
          }
        }
        else
        {
          // Supl session is pending
          UBX_LOG(LCAT_VERBOSE, "No network. Deferring NI until network present");
          pHandler->state = NO_NETWORK;
          pHandler->timeout = time(NULL) + SUPL_NONETWORK_TIMEOUT;
        }
      }
      else
      {
        UBX_LOG(LCAT_VERBOSE, "Unexpected message from network");
        asn_fprint(stdout, &asn_DEF_ULP_PDU, pMsg);
        // As we haven't yet opened a socket to the server, we have no
        // communications
        // channel, so do nothing. Network will timeout.  TODO - Or do we send a
        // defered error?
        pHandler->state = END;
        res = -1;
      }
    }
    break;

  case NO_NETWORK:
    UBX_LOG(LCAT_VERBOSE, "Handling NO_NETWORK");
    if (cmd == SUPL_NETWORK_CONNECTED)
    {
      if (pHandler->networkInitiated)
      {
        if (pHandler->pNiMsg->sessionID.setSessionID != NULL ||
            pHandler->pNiMsg->sessionID.slpSessionID == NULL)
        {
          /* Create a UPL session */
          if (createUplSession(pHandler) == -1)
          {
            UBX_LOG(LCAT_VERBOSE, "Cannot create the session");
          }
          else
          {
            UBX_LOG(LCAT_VERBOSE, "error in session ID");
            generateSuplEndMsg(pHandler, StatusCode_invalidSessionId);
          }
          pHandler->state = END;
          res = -1;
          break;
        }
        else
        {
          if (pHandler->pNiMsg->message.choice.msSUPLINIT.posMethod == PosMethod_noPosition)
          {
            /* Create a UPL session */
            if (createUplSession(pHandler) == -1)
            {
              UBX_LOG(LCAT_VERBOSE, "Cannot create the session");
            }
            else
            {
              generateSuplEndMsg(pHandler, StatusCode_unspecified);
            }
            pHandler->state = END;
            res = -1;
            break;
          }
          else
          {
            // Ask for notification from the UI
            startNiSession(pHandler); // Can recursively call suplSM

            // Do not do any state handling here
            // Also pHandler could be a floating pointer if an error occurred in
            // recursed suplSM called
          }
        }
      }
      else
      {
        res = startSiSession(pHandler);
      }
    }
    else if (cmd == SUPL_TIMEOUT)
    {
      // Timed out waiting for network connection
      logAgps.write(0x22000001, "%d # SUPL timeout", pHandler->sid);
      res = -1;
    }
    break;

  case WAIT_RES:
    UBX_LOG(LCAT_VERBOSE, "Handling WAIT_RES with message %p", pMsg);
    if (pMsg == NULL)
    {
      // No incoming msg, so expecting a command from device
      if (cmd == SUPL_TIMEOUT)
      {
        // Timed out waiting for server response
        UBX_LOG(LCAT_VERBOSE, "Sending SUPLEND (Timeout) ====>");
        generateSuplEndMsg(pHandler, StatusCode_unspecified);
        logAgps.write(0x22000001, "%d, # SUPL timeout", pHandler->sid);
        res = -1;
      }
    }
    else
    {
      /* check incoming message from network */
      if (pMsg->message.present == UlpMessage_PR_msSUPLEND)
      {
        UBX_LOG(LCAT_VERBOSE, "Received SUPLEND <====");
        /* Next state: END */
        if (pMsg->message.choice.msSUPLEND.statusCode != NULL)
        {
          UBX_LOG(LCAT_VERBOSE, "Reason is :%ld", *(pMsg->message.choice.msSUPLEND.statusCode));
          logAgps.write(0x22000000,
                        "%d, %d, SUPL_END",
                        pHandler->sid,
                        pMsg->message.choice.msSUPLEND.statusCode);
        }
        else
          logAgps.write(0x02000000,
                        "%d, 1.0.0, SUPL_END # SUPL_MODE : %s",
                        pHandler->sid,
                        pHandler->networkInitiated ? "NI" : "SI");
        pHandler->state = END;
        res = -1; // Always needed here as Supl session ended and resources will
                  // need to be freed.
      }
      else if (pMsg->message.present != UlpMessage_PR_msSUPLRESPONSE)
      {
        /* Unexpected message - End session*/
        UBX_LOG(LCAT_VERBOSE, "Unexpected answer - Not SUPLRESPONSE");
        UBX_LOG(LCAT_VERBOSE, "Sending SUPLEND ====>");
        generateSuplEndMsg(pHandler, StatusCode_unexpectedMessage);
        res = -1;
      }
      else
      {
        PosMethod_t posMethod = pMsg->message.choice.msSUPLRESPONSE.posMethod;
        UBX_LOG(LCAT_VERBOSE, "Received SUPLRESPONSE <====");
        logAgps.write(0x02000000,
                      "%d, 1.0.0, SUPL_RESPONSE # SUPL_MODE : SI_%s",
                      pHandler->sid,
                      (posMethod == PosMethod_agpsSETbased) ?
                        "MSB" :
                        (posMethod == PosMethod_agpsSETassisted) ? "MSA" : "? ? ?");

        // Handle SUPLRESPONSE msg
        if (pMsg->message.choice.msSUPLRESPONSE.sLPAddress != NULL)
        {
          /* Non proxy mode unsupported... error */
          UBX_LOG(LCAT_VERBOSE,
                  "Can not handle because non proxy mode "
                  "expected but not supported (%i)",
                  pMsg->message.choice.msSUPLRESPONSE.sLPAddress->present);
          UBX_LOG(LCAT_VERBOSE, "Sending SUPLEND ====>");
          generateSuplEndMsg(pHandler, StatusCode_nonProxyModeNotSupported);
          res = -1;
        }
        else if (!verifySetSessionId(pMsg->sessionID.setSessionID, pHandler->pSetId))
        {
          /* Inconsistency of session ID */
          UBX_LOG(LCAT_VERBOSE,
                  "wrong sid %d inseat of %d",
                  pHandler->sid,
                  (int)pMsg->sessionID.setSessionID->sessionId);
          UBX_LOG(LCAT_VERBOSE, "Sending SUPLEND ====>");
          ASN_STRUCT_FREE(asn_DEF_SetSessionID, pHandler->pSetId);
          pHandler->pSetId = copySetId(pMsg->sessionID.setSessionID);
          pHandler->pSlpId = copySlpId(pMsg->sessionID.slpSessionID);
          generateSuplEndMsg(pHandler, StatusCode_invalidSessionId);
          res = -1;
        }
        else
        {
          UBX_LOG(LCAT_VERBOSE,
                  "Processing SUPLRESPONSE - Pos Method %li(%s)",
                  posMethod,
                  _LOOKUPSTR((U)posMethod, PosMethodType));

          if ((posMethod == PosMethod_agpsSETbased) || (posMethod == PosMethod_agpsSETassisted))
          {
            UBX_LOG(
              LCAT_VERBOSE, "Handling SUPLRESPONSE (%s)", _LOOKUPSTR((U)posMethod, PosMethodType));

            // Handle SUPL_RESPONSE & SetBased
            suplPosInitParam_t par;

            /* copy the slpId from the receiving message */
            pHandler->pSlpId = copySlpId(pMsg->sessionID.slpSessionID);

            par.pSetId = pHandler->pSetId;
            par.pSlpId = pHandler->pSlpId;

            determineAssistance(pHandler, &par);

            /* In that state we are asking for the assistance data, most
               probably we
               do not have the position */
            par.verEn = 0;
            //						par.hash =
            //0x1122334455667788LL;

            // Pass through requested position method
            par.requestedPosMethod = posMethod;

            /* Ok, this is what we expect... */
            UBX_LOG(LCAT_VERBOSE, "Sending SUPLINITPOS ====>");
            sendSuplPosInit(pHandler->supl_connection, &par);
            /* Next state */
            pHandler->state = RRLP;

            /* Set the timeout */
            pHandler->timeout = time(NULL) + SUPL_STD_TIMEOUT;
          }
          else
          {
            /* Only SET based or Assisted possible here... */
            UBX_LOG(LCAT_VERBOSE, "Can not handle. Not MS-Based or MS-Assist");
            UBX_LOG(LCAT_VERBOSE, "Sending SUPLEND ====>");
            generateSuplEndMsg(pHandler, StatusCode_posMethodMismatch);
            res = -1;
          }
        }
      }
    }
    break;

  case AUTH:
    UBX_LOG(LCAT_VERBOSE, "Handling AUTH");
    if (cmd == SUPL_AUTH_GRANT)
    {
      /* Authorization granted!!! */
      UBX_LOG(LCAT_VERBOSE, "Authorization granted");

      /* Create a UPL session */
      if (createUplSession(pHandler) == -1)
      {
        UBX_LOG(LCAT_VERBOSE, "Cannot create the session");
        pHandler->state = END;
        res = -1;
      }
      else
      {
        if (pHandler->nonProxy)
        {
          UBX_LOG(LCAT_VERBOSE, "Proxy mode not supported");
          /* we must send the SUPL_END, and then close the connection */
          UBX_LOG(LCAT_VERBOSE, "Sending SUPLEND ====>");
          generateSuplEndMsg(pHandler, StatusCode_proxyModeNotSupported);
          /* Next state: END */
          pHandler->state = END;
          res = -1;
        }
        else
        {
          suplPosInitParam_t par;

          par.pSetId = pHandler->pSetId;
          par.pSlpId = pHandler->pSlpId;

          /* here need to be verified what can be done depending on the GPS
           * state */
          double lat, lon, speed;
          int accuracy = pHandler->reqHorAccuracy;

          CDatabase *pDatabase = CAndroidDatabase::getInstance();
          pDatabase->GetOutput(CDatabase::DATA_ERROR_RADIUS_METERS, accuracy);

          if ((pDatabase->GetOutput(CDatabase::DATA_LATITUDE_DEGREES, lat) &&
               pDatabase->GetOutput(CDatabase::DATA_LONGITUDE_DEGREES, lon)) &&
              pDatabase->GetOutput(CDatabase::DATA_SPEED_KNOTS, speed) &&
              (pHandler->reqHorAccuracy == -1 || accuracy < pHandler->reqHorAccuracy) &&
              (pHandler->reqVerAccuracy == -1 || accuracy < pHandler->reqVerAccuracy))
          {
            // Don't need assistance as we already have position
            par.posEn = 1;
            par.assEn = 0;
          }
          else
          {
            // Don't have position, request assistance
            par.posEn = 0;
            par.assEn = 1;
            pHandler->assistanceRequested = true;
          }

          /* The hash is needed */
          par.verEn = 1;
          par.hash = pHandler->hash;
          par.requestedPosMethod = pHandler->requestedPosMethod;

          /* we must send the SUPL_POSINIT */
          UBX_LOG(LCAT_VERBOSE, "Sending SUPLPOSINIT ====>");
          sendSuplPosInit(pHandler->supl_connection, &par);

          /* Set the timeout */
          pHandler->timeout = time(NULL) + SUPL_STD_TIMEOUT;

          /* Next state:  RRLP*/
          pHandler->state = RRLP;
        }
      }
    }
    else if (cmd == SUPL_AUTH_DENIED)
    {
      /* Authorization denied!!! */
      UBX_LOG(LCAT_VERBOSE, "Authorization denied");
      /* Create a UPL session */
      if (createUplSession(pHandler) == -1)
      {
        UBX_LOG(LCAT_VERBOSE, "Cannot create the session");
        pHandler->state = END;
        res = -1;
      }
      else
      {
        if (pHandler->notificationType == NotificationType_notificationOnly)
        {
          UBX_LOG(LCAT_VERBOSE, "Impossible! notification only, UI must always return GRANT");
        }

        UBX_LOG(LCAT_VERBOSE, "Authorization denied...");
        UBX_LOG(LCAT_VERBOSE, "Sending SUPLEND ====>");
        generateSuplEndMsg(pHandler, StatusCode_consentDeniedByUser);
        /* Next state: END */
        pHandler->state = END;
        res = -1;
      }
    }
    break;

  case RRLP:
    UBX_LOG(LCAT_VERBOSE, "Handling RRLP");
    if (pMsg == NULL)
    {
      UBX_LOG(LCAT_VERBOSE, "Local cmd %d", cmd);
      /* Only timeout available */
      if (cmd == SUPL_TIMEOUT)
      {
        UBX_LOG(LCAT_VERBOSE, "Sending SUPLEND (Timeout) ====>");
        generateSuplEndMsg(pHandler, StatusCode_unspecified);
        logAgps.write(0x22000001, "%d, # SUPL timeout", pHandler->sid);
        res = -1;
      }
    }
    else
    {
      UBX_LOG(LCAT_VERBOSE,
              "Msg from network: Present %i(%s)",
              pMsg->message.present,
              _LOOKUPSTR(pMsg->message.present, UlpMessageType));
      /* check incoming message */
      if (pMsg->message.present == UlpMessage_PR_msSUPLPOS)
      {
        UBX_LOG(LCAT_VERBOSE, "Received SUPLPOS <====");
        suplPosParam_t par;
        aux_t aux;

        logAgps.write(0x02000000,
                      "%d, 1.0.0, SUPL_POS # SUPL_MODE : %s : recv",
                      pHandler->sid,
                      pHandler->networkInitiated ? "NI" : "SI");

        if (pMsg->message.choice.msSUPLPOS.posPayLoad.present == PosPayLoad_PR_rrlpPayload)
        {
          par.buffer =
            rrlpProcessing(pHandler->sid,
                           pMsg->message.choice.msSUPLPOS.posPayLoad.choice.rrlpPayload.buf,
                           pMsg->message.choice.msSUPLPOS.posPayLoad.choice.rrlpPayload.size,
                           &(par.size),
                           &aux);
          UBX_LOG(LCAT_INFO, "rrlp data prepared");

          /* Do not report an error if assistance data is not received */
          /*
                                                  if
             ((pHandler->networkInitiated) && (pHandler->assistanceRequested) &&
             (!aux.assistDataReceived))
                                                  {
                                                          // Terminate NI
             because no assist data received
                                                          UBX_LOG(LCAT_VERBOSE,
             "Assistance data not received");
                                                          UBX_LOG(LCAT_VERBOSE,
             "Sending SUPLEND ====>");
                                                          generateSuplEndMsg(pHandler,
             StatusCode_dataMissing);
                                                          res = -1;
                                                          pHandler->state = END;
                                                  }
                                                  else
          */
          if ((pHandler->networkInitiated) && (pHandler->pNiMsg) &&
              (pHandler->pNiMsg->sessionID.setSessionID) &&
              ((pHandler->pNiMsg->sessionID.setSessionID->sessionId !=
                pMsg->sessionID.setSessionID->sessionId) ||
               (memcmp(&pHandler->pNiMsg->sessionID.setSessionID->setId,
                       &pMsg->sessionID.setSessionID->setId,
                       sizeof(SETId_t)) != 0)))
          {
            UBX_LOG(LCAT_VERBOSE, "Invalid session id received, SUPLEND sent");
            generateSuplEndMsg(pHandler, StatusCode_invalidSessionId);
            res = -1;
          }
          else if (pHandler->pNiMsg && pHandler->pNiMsg->sessionID.slpSessionID == NULL)
          {
            UBX_LOG(LCAT_VERBOSE, "Invalid slp session id, SUPLEND sent");
            generateSuplEndMsg(pHandler, StatusCode_invalidSessionId);
            res = -1;
          }
          else if ((pHandler->networkInitiated) && (pHandler->pNiMsg) &&
                   !verifySlpSessionId(pHandler->pNiMsg->sessionID.slpSessionID,
                                       pMsg->sessionID.slpSessionID))
          {
            ASN_STRUCT_FREE(asn_DEF_SlpSessionID, pHandler->pSlpId);
            pHandler->pSlpId = copySlpId(pMsg->sessionID.slpSessionID);
            UBX_LOG(LCAT_VERBOSE, "%d Invalid slp session id, SUPLEND sent", __LINE__);
            generateSuplEndMsg(pHandler, StatusCode_invalidSessionId);
            res = -1;
          }
          else if ((!pHandler->networkInitiated) &&
                   !verifySlpSessionId(pHandler->pSlpId, pMsg->sessionID.slpSessionID))
          {
            ASN_STRUCT_FREE(asn_DEF_SlpSessionID, pHandler->pSlpId);
            pHandler->pSlpId = copySlpId(pMsg->sessionID.slpSessionID);
            UBX_LOG(LCAT_VERBOSE, "%d Invalid slp session id, SUPLEND sent", __LINE__);
            generateSuplEndMsg(pHandler, StatusCode_invalidSessionId);
            res = -1;
          }
          else if (!verifySetSessionId(pMsg->sessionID.setSessionID, pHandler->pSetId))
          {
            /* Inconsistency of session ID */
            UBX_LOG(LCAT_VERBOSE,
                    "wrong sid %d inseat of %d",
                    pHandler->sid,
                    (int)pMsg->sessionID.setSessionID->sessionId);
            UBX_LOG(LCAT_VERBOSE, "Sending SUPLEND ====>");
            ASN_STRUCT_FREE(asn_DEF_SetSessionID, pHandler->pSetId);
            pHandler->pSetId = copySetId(pMsg->sessionID.setSessionID);
            generateSuplEndMsg(pHandler, StatusCode_invalidSessionId);
            res = -1;
          }
          else if (par.buffer == NULL)
          {
            // No immediate response - so check response time
            if (aux.responseTime <= 0)
            {
              // Wrong RRLP packet - so end session
              UBX_LOG(LCAT_VERBOSE, "Wrong RRLP packet %d", cmd);
              UBX_LOG(LCAT_VERBOSE, "Sending SUPLEND ====>");
              generateSuplEndMsg(pHandler, StatusCode_protocolError);
              res = -1;
            }
            else
            {
              /* Now I have a timeout for answering with a position response
               * RRLP message */
              time_t now = time(NULL);
              int maxDelay = CUbxGpsState::getInstance()->getNiResponseTimeout();

              if (aux.responseType == RESP_MSA_DATA)
              {
                // MSA
                pHandler->timeout = -1; // There is no timeout as such
                pHandler->msaPosResponseTime =
                  maxDelay < aux.responseTime ? maxDelay : aux.responseTime;
                pHandler->msaPosResponseTime += now;
              }
              else
              {
                // MSB
                if (pHandler->QopDelay > 0)
                {
                  pHandler->timeout = pHandler->QopDelay;
                }
                else
                {
                  pHandler->timeout = aux.responseTime;
                }

                // Config setting gives upper limit on delay
                if (pHandler->timeout > maxDelay)
                {
                  pHandler->timeout = maxDelay;
                }
                pHandler->timeout += now;
              }

              pHandler->state = RRLP_RESP;
              pHandler->rrlpRefNum = aux.referenceNumber;
              pHandler->reqHorAccuracy = aux.reqAccuracy;
              pHandler->reqVerAccuracy = -1;
              pHandler->responseType = aux.responseType;

              logAgps.write(0x00000002,
                            "%d, %d, %d, %d, %d, %d # QoP",
                            pHandler->sid,
                            pHandler->reqHorAccuracy,
                            pHandler->reqVerAccuracy,
                            0,
                            0,
                            pHandler->msaPosResponseTime); // todo
              UBX_LOG(LCAT_VERBOSE, "Need to response to SUPL request in %is", aux.responseTime);
            }
          }
          else
          {
            // Send a position response to server
            par.pSlpId = pHandler->pSlpId;
            par.pSetId = pHandler->pSetId;

            /* Ok, this is what we expect... */
            UBX_LOG(LCAT_INFO, "Sending SUPLPOS ====>");
            logAgps.write(0x02000000,
                          "%d, 1.0.0, SUPL_POS # SUPL_MODE : %s",
                          pHandler->sid,
                          pHandler->networkInitiated ? "NI" : "SI");
            sendSuplPos(pHandler->supl_connection, &par);

            MC_FREE(par.buffer);

            /* Set the timeout */
            pHandler->timeout = time(NULL) + SUPL_STD_TIMEOUT;
          }
        }
        else
        {
          // Unexpected SUPLPOS payload
          UBX_LOG(LCAT_VERBOSE, "Not an RRLP payload");
          UBX_LOG(LCAT_VERBOSE, "Sending SUPLEND ====>");
          generateSuplEndMsg(pHandler, StatusCode_posProtocolMismatch);
          res = -1;
          pHandler->state = END;
        }
      }
      else if (pMsg->message.present == UlpMessage_PR_msSUPLEND)
      {
        UBX_LOG(LCAT_VERBOSE, "Received SUPLEND <====");
        if (pMsg->message.choice.msSUPLEND.statusCode != NULL)
        {
          logAgps.write(0x22000000,
                        "%d, %d, SUPL_END",
                        pHandler->sid,
                        pMsg->message.choice.msSUPLEND.statusCode);
          UBX_LOG(LCAT_VERBOSE, "Reason is :%ld", *(pMsg->message.choice.msSUPLEND.statusCode));
        }
        else
          logAgps.write(0x02000000,
                        "%d, 1.0.0, SUPL_END # SUPL_MODE : %s",
                        pHandler->sid,
                        pHandler->networkInitiated ? "NI" : "SI");

        if (pMsg->message.choice.msSUPLEND.position != NULL)
        {
          // Position estimate received, send to receiver
          long latSign = pMsg->message.choice.msSUPLEND.position->positionEstimate.latitudeSign;
          double lat = pMsg->message.choice.msSUPLEND.position->positionEstimate.latitude;
          if (latSign)
          {
            lat *= -1;
          }
          double lng = pMsg->message.choice.msSUPLEND.position->positionEstimate.longitude;
          lat = lat * 90 / (1 << 23);
          lng = lng * 360 / (1 << 24);

          UBX_LOG(LCAT_VERBOSE, "Received position is :%ld %f %f", latSign, lat, lng);

          GPS_UBX_AID_INI_U5__t aidingData;
          memset(&aidingData, 0, sizeof(aidingData));
          aidingData.flags =
            GPS_UBX_AID_INI_U5__FLAGS_POS_MASK | GPS_UBX_AID_INI_U5__FLAGS_LLA_MASK;
          aidingData.ecefXOrLat = (int)(lat * 10000000);
          aidingData.ecefYOrLon = (int)(lng * 10000000);

          CUbxGpsState *pUbxGps = CUbxGpsState::getInstance();
          pUbxGps->lock();
          pUbxGps->sendAidingData(&aidingData);
          pUbxGps->unlock();

          if (CGpsIf::getInstance()->m_callbacks.location_cb)
          {
            // Publish
            GpsLocation loc;
            memset(&loc, 0, sizeof(GpsLocation));
            loc.size = sizeof(GpsLocation);
            loc.flags |= GPS_LOCATION_HAS_LAT_LONG;
            loc.latitude = lat;
            loc.longitude = lng;
            CGpsIf::getInstance()->m_callbacks.location_cb(&loc);
          }
        }
        res = -1;
        pHandler->state = END;
      }
      else
      {
        /* unexpected message... */
        UBX_LOG(LCAT_VERBOSE, "Timeout %d", cmd);
        UBX_LOG(LCAT_VERBOSE, "Sending SUPLEND (Timeout) ====>");
        generateSuplEndMsg(pHandler, StatusCode_unexpectedMessage);
        res = -1;
        pHandler->state = END;
      }
    }
    break;

  case RRLP_RESP:
    UBX_LOG(LCAT_VERBOSE, "Handling RRLP_RESP");
    if (pMsg == 0)
    {
      // No incoming msg, so expecting a command from device
      char *pSendBuffer = NULL;
      int size = 0;

      switch (cmd)
      {
      case SUPL_TIMEOUT:
        /*
                          // timeout existing session
                                              UBX_LOG(LCAT_VERBOSE, "Sending
         SUPLEND (Timeout) ====>");
                          //pSendBuffer = buildRrlpPosRespTimeout(&size,
         pHandler->rrlpRefNum);
                                          logAgps.write(0x22000001, "%d, #
         timeout", pHandler->sid);
                                              generateSuplEndMsg(pHandler,
         StatusCode_protocolError);
                                              res = -1;
                                              break;
      */

      case SUPL_POS_AVAIL:
      {
        // position is available and server waiting to receive this
        double lat = 0;
        double lon = 0;
        double alt = 0;
        double tow = 0;
        double ttff = 0;
        CDatabase *pDatabase = CAndroidDatabase::getInstance();
        pDatabase->GetOutput(CDatabase::DATA_LATITUDE_DEGREES, lat);
        pDatabase->GetOutput(CDatabase::DATA_LONGITUDE_DEGREES, lon);
        pDatabase->GetOutput(CDatabase::DATA_ALTITUDE_SEALEVEL_METERS, alt);
        pDatabase->GetOutput(CDatabase::DATA_UBX_GPSTIME_TOW, tow);
        pDatabase->GetOutput(CDatabase::DATA_UBX_TTFF, ttff);
        TIMESTAMP ts;
        memset(&ts, 0, sizeof(ts));
        logAgps.write(0x00000003,
                      "%d, %04d%02d%02d%02d%02d%06.3f,%10.6f,%11.6f,%d "
                      "#position(time_stamp,lat,lon,ttff)",
                      pHandler->sid,
                      ts.wYear,
                      ts.wMonth,
                      ts.wDay,
                      ts.wHour,
                      ts.wMinute,
                      1e-6 * ts.lMicroseconds,
                      lat,
                      lon,
                      ttff * 1e-3);
        pSendBuffer = buildRrlpPosResp(&size, pHandler->rrlpRefNum, lat, lon, alt, tow);
        UBX_LOG(LCAT_VERBOSE, "Sending SUPLPOS (position available) ====>");
        res = sendPositionResponse(pHandler, pSendBuffer, size);
        break;
      }
      case SUPL_MSA_DATA_AVAIL:
        // MSA data available and server waiting to receive this
        UBX_LOG(LCAT_VERBOSE, "Sending SUPLPOS (msa data) ====>");
        pSendBuffer = buildRrlpMsaPosResp(pHandler->sid, &size, pHandler->rrlpRefNum);
        logAgps.write(0x00000002, "%d, # msa", pHandler->sid);
        res = sendPositionResponse(pHandler, pSendBuffer, size);

        //// No acknowledgement required. Transaction finished - This my change
        // pHandler->state = END;
        // res = -1;
        break;

      default:
        assert(0); // Shouldn't happen
        break;
      }
    }
    break;

  case END:
    UBX_LOG(LCAT_VERBOSE, "Handling END");
    break;
  }

  UBX_LOG(LCAT_VERBOSE,
          "SM exit with Id = %d in state %s (%d) with value %d",
          pHandler->sid,
          stateInfo.at(pHandler->state).c_str(),
          pHandler->state,
          res);

  return res;
}

///////////////////////////////////////////////////////////////////////////////
//! Allocate a new Supl state handling structure
/*!
  \return Pointer to the new Supl state handling structure. NULL if a failure
  happens
*/
static suplHandler_t *allocateSM(bool ni)
{
  // Allocate the handler - remember to deallocate in case of error
  suplHandler_t *pHandler = (suplHandler_t *)MC_CALLOC(sizeof(suplHandler_t), 1);
  if (pHandler == NULL)
  {
    UBX_LOG(LCAT_VERBOSE, "Error in allocation");
    return NULL;
  }

  pHandler->state = START;                             /* set the start of the state machine */
  pHandler->timeout = -1;                              /* set the timer initial value */
  pHandler->requestedPosMethod = PosMethod_noPosition; // No pos method defined
  pHandler->networkInitiated = ni;                     // Initiation type

  pHandler->pSetId = NULL;
  pHandler->pSlpId = NULL;
  pHandler->pNiMsg = NULL;

  return pHandler; /* return the handler pointer */
}

static bool suplInsertHandler(suplHandler_t *pHandler, const SetSessionID_t *pSetId)
{
  assert(pHandler);
  assert(MAX_SM_INSTANCES < (MAX_SID_NUMBER - START_SID_NUMBER));
  std::lock_guard<std::mutex> lock(s_handlerMutex);

  if (s_pQueueTail == NULL)
  {
    /* the queue is empty.. */
    s_pQueueTail = pHandler;
  }
  else
  {
    /* browse the queue */
    suplHandler_t *pTmp = s_pQueueTail;
    int cnt = 0;
    while (pTmp->pNext != NULL)
    {
      pTmp = pTmp->pNext;

      /* check number of SM instancies */
      cnt++;
      if (cnt > MAX_SM_INSTANCES)
      {
        /* Error in number of instancies */
        UBX_LOG(LCAT_VERBOSE, "too many instancies... %d", cnt);
        return false;
      }
    }
    // Space for insertion
    pTmp->pNext = pHandler; // Insert
  }

  if (pSetId != NULL)
  {
    pHandler->pSetId = copySetId(pSetId);
    pHandler->sid = pSetId->sessionId;
  }
  else
  {
    pHandler->sid = getNewSid(); // Get a unique sid - NB handler list mutex is locked
    pHandler->pSetId = fillDefaultSetId(pHandler->sid);
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////////
//! Releases a Supl state handling structure
/*!
  \param pHandler : Pointer to the Supl state handling structure to release
*/
static void freeSuplHandler(suplHandler_t *pHandler)
{
  assert(pHandler);
  assert(pthread_self() == g_gpsDrvMainThread);
  std::lock_guard<std::mutex> lock(s_handlerMutex);

  /* Check the consistency of the queue */
  assert(s_pQueueTail);

  /* close the socket, if it is open... */
  if (pHandler->supl_connection != 0)
  {
    supl_connection_dealloc(pHandler->supl_connection);
    UBX_LOG(LCAT_VERBOSE, "closed SUPL connection");
    pHandler->supl_connection = 0;
  }

  /* check if it's the first element */
  suplHandler_t *pTmp = s_pQueueTail;
  if (pTmp == pHandler)
  {
    s_pQueueTail = pHandler->pNext;
  }
  else
  {
    while (pTmp->pNext != pHandler)
    {
      /* Check the consistency of the queue */
      assert(pTmp->pNext != NULL);

      /* Browse the tree to get the right handler */
      pTmp = pTmp->pNext;
    }

    /* shift the elements */
    pTmp->pNext = pHandler->pNext;
  }

  UBX_LOG(LCAT_VERBOSE, "deallocate handler");

  /* deallocate it */
  safeFreeHandler(pHandler);

  UBX_LOG(LCAT_VERBOSE, "handler deallocated");
}

///////////////////////////////////////////////////////////////////////////////
//! Find an existing Supl state handling structure
/*! Find an existing Supl state handling structure based on the Session ID of
    an ongoing Supl transaction
  \param sid : Session ID
  \return Pointer to the Supl state handling structure with matching session ID.
          NULL if no match found.
*/
static suplHandler_t *searchSMInst(int sid, bool lock)
{
  if (lock)
  {
    std::lock_guard<std::mutex> lock(s_handlerMutex);

    /* take a pointer to the tail */
    suplHandler_t *pTmp = s_pQueueTail;

    while (pTmp != NULL)
    {
      if (pTmp->sid == sid)
      {
        /* found the correct one */
        break;
      }
      /* browse the queu */
      pTmp = pTmp->pNext;
    }

    /* no instance available with the sid */
    return pTmp;
  }
  else
  {
    /* take a pointer to the tail */
    suplHandler_t *pTmp = s_pQueueTail;

    while (pTmp != NULL)
    {
      if (pTmp->sid == sid)
      {
        /* found the correct one */
        break;
      }
      /* browse the queu */
      pTmp = pTmp->pNext;
    }

    /* no instance available with the sid */
    return pTmp;
  }
}

///////////////////////////////////////////////////////////////////////////////
//! Generates a new Session ID
/*!
  \return New session ID
*/
static int getNewSid(void)
{
  /* Start with sid = START_SID_NUMBER */
  static int ret = START_SID_NUMBER;

  /* Use the search function.. */
  do
  {
    ret++;
    if (ret > MAX_SID_NUMBER)
    {
      ret = START_SID_NUMBER;
    }
  } while (searchSMInst(ret, false) != NULL);
  return ret;
}

///////////////////////////////////////////////////////////////////////////////
//! Generate a new Supl session
/*! Generates a new Supl session by setting up a socket connection to a the Supl
    server and filling the supplied supl state structure with appropriate
  session
    information
  \param pHandler : Pointer to Supl state structure
  \return 0 id successful, < 0 if failed
*/
static int createUplSession(suplHandler_t *pHandler)
{
  assert(pHandler);
  assert(pthread_self() == g_gpsDrvMainThread);

  CAgpsIf *pAgps = CAgpsIf::getInstance();
  int r;
  int use_tls;
  int suplPort;
  char *pSuplServerAddress;
  int attempt, max_connection_attempts, have_connection;

  /* prepare the address */
  pAgps->getSuplServerInfo(&pSuplServerAddress, &suplPort);
  if (pSuplServerAddress == nullptr)
  {
    UBX_LOG(LCAT_ERROR, "Could not get supl server address");
    return -1;
  }

  use_tls = 0;
  if (pAgps->isTlsActive())
    use_tls = 1;

  pHandler->supl_connection = supl_connection_init();
  if (!pHandler->supl_connection)
    return -1;

  max_connection_attempts = pAgps->getConnectRetries() + 1;
  have_connection = 0;

  for (attempt = 1; attempt <= max_connection_attempts; attempt++)
  {
    int timeout_seconds;

    UBX_LOG(LCAT_ERROR, "SUPL connection, attempt %d out of %d", attempt, max_connection_attempts);

    if (attempt > 1)
      usleep(100000);

    logAgps.write(0x10000002, "%d # server connecting...", pHandler->sid);

    timeout_seconds = 5;

    r = supl_connection_connect(pHandler->supl_connection,
                                timeout_seconds,
                                pSuplServerAddress,
                                suplPort,
                                use_tls,
                                pAgps->getCertificateFileName());
    if (r == 0)
    {
      have_connection = 1;
      break;
    }
    if (r == -2)
    {
      logAgps.write(0x21000001, "%d # no network", pHandler->sid);
      return -1;
    }
    if (r == -3)
    {
      logAgps.write(0x21000001, "%d # tls failed", pHandler->sid);
      return -1;
    }

    // r == -1, retry possible
  }

  if (!have_connection)
  {
    UBX_LOG(LCAT_ERROR, "No SUPL connection, out of retries");
    return -1;
  }

  logAgps.write(0x10000003, "%d # server connection success", pHandler->sid);

  return 0;
}

///////////////////////////////////////////////////////////////////////////////
//! Calculate a hash based on the contents of the supplied buffer
/*!
  \param buffer : Pointer to buffer
  \param size   : Size of buffer
  \return 64 bit hash value
*/
static long long calculateHash(const char *buffer, int size)
{
  assert(buffer);

  unsigned char *returnCode;
  long long res;
  unsigned int sizeOut;
  unsigned char outHash[EVP_MAX_MD_SIZE];
  CAgpsIf *pAgps = CAgpsIf::getInstance();
  int suplPort;
#if defined SUPL_FQDN_SLP
  char pSuplServerAddress[] = SUPL_FQDN_SLP;
#else
  char *pSuplServerAddress;
  pAgps->getSuplServerInfo(&pSuplServerAddress, &suplPort);
#endif
  if (pSuplServerAddress == nullptr)
  {
    UBX_LOG(LCAT_ERROR, "Could not get supl server address");
    return -1;
  }

  UBX_LOG(LCAT_VERBOSE, "Size of the HMAC input buffer is %d", size);
  UBX_LOG(LCAT_VERBOSE, "the key is %s", pSuplServerAddress);
  UBX_LOG(LCAT_VERBOSE, "the key length is %zd", strlen(pSuplServerAddress));
  returnCode = HMAC(EVP_sha1(),
                    pSuplServerAddress,
                    (int)strlen(pSuplServerAddress),
                    (unsigned char *)const_cast<char *>(buffer),
                    (unsigned int)size,
                    outHash,
                    &sizeOut);
  if (returnCode == nullptr)
  {
    UBX_LOG(LCAT_ERROR, "Could not Calculate HMAC");
    return -1;
  }
  UBX_LOG(LCAT_VERBOSE, "size of the hash = %d", size);

  memcpy(&res, returnCode, sizeof(res));
  UBX_LOG(LCAT_VERBOSE, "The calculated HASH is: %.16LX", res);

  return res;
}

///////////////////////////////////////////////////////////////////////////////
//! Generate and send a Supl End message to server
/*!
  \param pHandler   : Pointer to Supl state structure
  \param statusCode : Status code to include in Supl End message
*/
static void generateSuplEndMsg(suplHandler_t *pHandler, StatusCode statusCode)
{
  assert(pHandler);
  suplEndParam_t par;

  par.pSetId = pHandler->pSetId;
  par.pSlpId = pHandler->pSlpId;
  par.posEn = 0;

  if (pHandler->hash == 0)
  {
    par.verEn = 0;
  }
  else
  {
    par.verEn = 1;
  }
  par.hash = pHandler->hash;

  /* Error code: not specified */
  par.status = statusCode;

  /* we must send the SUPL_END, and then close the connection */
  sendSuplEnd(pHandler->supl_connection, &par);

  /* Next state: END */
  pHandler->state = END;
}

///////////////////////////////////////////////////////////////////////////////
//! Generate and send a Supl Position Response message to server
/*!
  \param pHandler    : Pointer to Supl state structure
  \param pSendBuffer : Pointer to the RRLP encodes message which will be the
  payload for the Supl message
  \param size        : Size of RRLP message buffer
  \return  0 successful, < 0 session is finished
*/
static int sendPositionResponse(suplHandler_t *pHandler, char *pSendBuffer, int size)
{
  assert(pHandler);

  int res = 0;

  if (pSendBuffer == NULL)
  {
    // Only response is to end session
    generateSuplEndMsg(pHandler, StatusCode_protocolError);
    res = -1;
  }
  else
  {
    suplPosParam_t par;
    memset(&par, 0, sizeof(par));

    // Send position response
    par.pSlpId = pHandler->pSlpId;
    par.pSetId = pHandler->pSetId;
    par.buffer = pSendBuffer;
    par.size = size;

    /* Ok, this is what we expect... */
    sendSuplPos(pHandler->supl_connection, &par);

    /* Set the timeout */
    pHandler->timeout = time(NULL) + SUPL_STD_TIMEOUT;
    pHandler->state = RRLP;

    MC_FREE(pSendBuffer);
  }

  return res;
}

///////////////////////////////////////////////////////////////////////////////
//! Starts a Ni Supl session
/*! Starts a Ni Supl session by preparing a notify/verify request which is sent
    to the framework. If the request is accepted, the framework will callback to
        the driver to continue the Supl session process.
  \param pHandler    : Pointer to Supl state structure
*/
static void startNiSession(suplHandler_t *pHandler)
{
  assert(pHandler);
  assert(pthread_self() == g_gpsDrvMainThread);

  logAgps.write(0x01000000, "%d # network connection success", pHandler->sid);
  // wait authorization or timeout
  pHandler->state = AUTH;

  CNiIf *pNiIf = CNiIf::getInstance();

  GpsNiNotification notification;
  memset(&notification, 0, sizeof(GpsNiNotification));

  notification.size = sizeof(GpsNiNotification);
  notification.notification_id = pHandler->sid;
  notification.ni_type = GPS_NI_TYPE_UMTS_SUPL; // TODO - Is this the right type?

  setNotificationAndResponse(
    pHandler->notificationType, &notification.notify_flags, &notification.default_response);
  notification.timeout = CUbxGpsState::getInstance()->getNiUiTimeout();
  UBX_LOG(LCAT_VERBOSE, "notify timeout %d", notification.timeout);

  if (pHandler->pRequestorId != NULL)
  {
    copyAndConvertText(
      notification.requestor_id, GPS_NI_SHORT_STRING_MAXLEN, pHandler->pRequestorId);
  }
  else
  {
    notification.requestor_id[0] = '\0';
  }

  if (pHandler->pClientName != NULL)
  {
    copyAndConvertText(notification.text, GPS_NI_LONG_STRING_MAXLEN, pHandler->pClientName);
  }
  else
  {
    notification.text[0] = '\0';
  }

  notification.requestor_id_encoding = convertAsn1Encoding(pHandler->encodingType);
  notification.text_encoding = notification.requestor_id_encoding;
  // No 'notification.extras' need to be added

  CNiIf::request(&notification); // This will cause a recursive call to suplSM
                                 // pHandler could be now be a floating pointer
                                 // if error occured in recursed suplSm

  // Be sure the request has complete the operations
  pNiIf->lock();

  // here we should be sure the notification is completed
  suplHandleAuthorization(notification.notification_id, pNiIf->getSuplState());
}

///////////////////////////////////////////////////////////////////////////////
//! Converts a Supl text to the 'hex' representation needed by the framework
/*!
  \param pDest       : Pointer the destination buffer
  \param maxLen      : Size of the destination buffer
  \param pSuplString : Pointer to Supl text to convert
*/
static void copyAndConvertText(char *pDest, int maxLen, const OCTET_STRING_t *pSuplString)
{
  assert(pDest);
  assert(pSuplString);

  int len = pSuplString->size;
  uint8_t *pSrc = pSuplString->buf;

  if (len * 2 >= maxLen)
  {
    len = (maxLen / 2) - 1;
  }

  for (int i = 0; i < len; i++)
  {
    *pDest = (*pSrc >> 4);
    *pDest += *pDest > 9 ? 55 : 48;
    pDest++;

    *pDest = (*pSrc & 0x0F);
    *pDest += *pDest > 9 ? 55 : 48;
    pDest++;
    pSrc++;
  }

  *pDest = 0;
}

///////////////////////////////////////////////////////////////////////////////
//! Convert Supl encoding to framework encoding
/*! Function to convert a Supl text encoding type into a text encoding type
    used by the framework
  \param encodingType : Supl encoding type
  \return Framework encoding type
*/
static GpsNiEncodingType convertAsn1Encoding(EncodingType_t encodingType)
{
  GpsNiEncodingType niEncodingType = GPS_ENC_UNKNOWN;

  switch (encodingType)
  {
  case EncodingType_ucs2:
    niEncodingType = GPS_ENC_SUPL_UCS2;
    break;

  case EncodingType_gsmDefault:
    niEncodingType = GPS_ENC_SUPL_GSM_DEFAULT;
    break;

  case EncodingType_utf8:
    niEncodingType = GPS_ENC_SUPL_UTF8;
    break;

  default:
    break;
  }

  return niEncodingType;
}

///////////////////////////////////////////////////////////////////////////////
//! Converts a Supl notify/verify request to framework equivalent
/*! Converts a Supl notify/verify request to framework equivalent in the form
    of Notify flags and default response
  \param ans1cNotifyType  : Supl nofity/verify request
  \param pNotifyFlags     : Pointer to framework notify flags
  \param pDefaultResponse : Pointer to framework default response
*/
static void setNotificationAndResponse(NotificationType_t ans1cNotifyType,
                                       GpsNiNotifyFlags *pNotifyFlags,
                                       GpsUserResponseType *pDefaultResponse)
{
  assert(pNotifyFlags);
  assert(pDefaultResponse);

  *pNotifyFlags = 0;
  *pDefaultResponse = GPS_NI_RESPONSE_NORESP;

  switch (ans1cNotifyType)
  {
  case NotificationType_noNotificationNoVerification:
    // No need to set any flags
    break;

  case NotificationType_notificationOnly:
    *pNotifyFlags = GPS_NI_NEED_NOTIFY;
    break;

  case NotificationType_notificationAndVerficationAllowedNA:
    *pNotifyFlags = GPS_NI_NEED_NOTIFY | GPS_NI_NEED_VERIFY;
    *pDefaultResponse = GPS_NI_RESPONSE_ACCEPT;
    break;

  case NotificationType_notificationAndVerficationDeniedNA:
    *pNotifyFlags = GPS_NI_NEED_NOTIFY | GPS_NI_NEED_VERIFY;
    *pDefaultResponse = GPS_NI_RESPONSE_DENY;
    break;

  case NotificationType_privacyOverride:
    *pNotifyFlags = GPS_NI_PRIVACY_OVERRIDE;
    break;

  default:
    UBX_LOG(LCAT_VERBOSE, "Can't translate ans1cNotifyType(%li)", ans1cNotifyType);
    break;
  }
}

///////////////////////////////////////////////////////////////////////////////
//! Determines if a Supl transaction can be started or allowed to proceeed
/*!
  \return : true if Supl transaction can start/proceed. false if not
*/
static bool isSuplPossible(void)
{
  int suplPort;
  char *pSuplServerAddress;
  CAgpsIf::getInstance()->getSuplServerInfo(&pSuplServerAddress, &suplPort);
  if ((pSuplServerAddress == NULL) || (suplPort < 0))
  {
    // No Supl server specified.
    UBX_LOG(LCAT_ERROR, "No server specified");
    return false;
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////////
//! Determine if assistance dataneeds to be requested from Supl server
/*!
  \param pHandler		: Pointer to Supl session state structure
  \param pPosInitParams	: Pointer to SuplPosInit message parameters
  structure to populate
*/
static void determineAssistance(suplHandler_t *pHandler, suplPosInitParam_t *pPosInitParams)
{
  assert(pHandler);
  assert(pPosInitParams);

  double lat, lon;

  CDatabase *pDatabase = CAndroidDatabase::getInstance();
  assert(pDatabase);

  if (pDatabase->GetOutput(CDatabase::DATA_LATITUDE_DEGREES, lat) &&
      pDatabase->GetOutput(CDatabase::DATA_LONGITUDE_DEGREES, lon))
  {
    // Don't need assistance as we already have position
    pPosInitParams->posEn = 1;
    pPosInitParams->lat = lat;
    pPosInitParams->lon = lon;
    pPosInitParams->assEn = 0;
    pHandler->assistanceRequested = false;
  }
  else
  {
    // Don't have position, request assistance
    pPosInitParams->posEn = 0;
    pPosInitParams->assEn = 1;
    pHandler->assistanceRequested = true;
  }
}

///////////////////////////////////////////////////////////////////////////////
//! Starts a Si Supl session
/*!
  \param pHandler    : Pointer to Supl state structure
*/
static int startSiSession(suplHandler_t *pHandler)
{
  assert(pHandler);
  assert(pthread_self() == g_gpsDrvMainThread);

  logAgps.write(0x01000000, "%d # network connection success", pHandler->sid);
  int res = 0; // success by default;

  /* Create a UPL session */
  if (createUplSession(pHandler) == -1)
  {
    pHandler->state = END;
    res = -1;
  }
  else
  {
    UBX_LOG(LCAT_VERBOSE, "Sending SUPLSTART ====>");
    sendSuplStart(pHandler->supl_connection, pHandler->pSetId);
    /* Next state */
    pHandler->state = WAIT_RES;
    /* set the timeout */
    pHandler->timeout = time(NULL) + SUPL_STD_TIMEOUT;
  }

  return res;
}

///////////////////////////////////////////////////////////////////////////////
//! Perform any clean up required when ending a Supl session
/*!
  \param pHandler    : Pointer to Supl state structure
*/
static void endSuplSession(suplHandler_t *pHandler)
{
  assert(pHandler);
  assert(pthread_self() == g_gpsDrvMainThread);

  logAgps.write(0x00000001, "%d # a-gps session ends", pHandler->sid);

  if (pHandler->networkInitiated)
  {
    // End of NI session - signal engine to stop
    if (s_pGpsControlInterface)
    {
      s_pGpsControlInterface->requestStop_cb(s_pEventContext);
    }
  }
  else
  {
    // End of SI session - Allow publish if MSA
    if (CGpsIf::getInstance()->getMode() == GPS_POSITION_MODE_MS_ASSISTED)
    {
      CAndroidDatabase::getInstance()->incPublish();
    }
  }

  freeSuplHandler(pHandler);
}

///////////////////////////////////////////////////////////////////////////////
//! Verify consistency of slp session ID
/*!
  \param pSlpSessionId_1   : Pointer to slp session Id
  \param pSlpSessionId_2   : Pointer to slp session Id
  \return                  : true if consistent
*/
static bool verifySlpSessionId(SlpSessionID_t *pSlpSessionId_1, SlpSessionID_t *pSlpSessionId_2)
{
  if (pSlpSessionId_1 == NULL && pSlpSessionId_2 == NULL)
    return true;
  else if (pSlpSessionId_1 == NULL || pSlpSessionId_2 == NULL)
    return false;
  else
  {
    if (!compareOctets(&pSlpSessionId_1->sessionID, &pSlpSessionId_2->sessionID))
      return false;

    if (pSlpSessionId_1->slpId.present != pSlpSessionId_2->slpId.present)
      return false;

    if (pSlpSessionId_1->slpId.present == SLPAddress_PR_iPAddress)
    {
      if (pSlpSessionId_1->slpId.choice.iPAddress.present !=
          pSlpSessionId_2->slpId.choice.iPAddress.present)
        return false;

      if (pSlpSessionId_1->slpId.choice.iPAddress.present == IPAddress_PR_ipv4Address)
      {
        OCTET_STRING_t *pOct1 = &pSlpSessionId_1->slpId.choice.iPAddress.choice.ipv4Address;
        OCTET_STRING_t *pOct2 = &pSlpSessionId_2->slpId.choice.iPAddress.choice.ipv4Address;
        if (!compareOctets(pOct1, pOct2))
          return false;
      }

      if (pSlpSessionId_1->slpId.choice.iPAddress.present == IPAddress_PR_ipv6Address)
      {
        OCTET_STRING_t *pOct1 = &pSlpSessionId_1->slpId.choice.iPAddress.choice.ipv6Address;
        OCTET_STRING_t *pOct2 = &pSlpSessionId_2->slpId.choice.iPAddress.choice.ipv6Address;
        if (!compareOctets(pOct1, pOct2))
          return false;
      }
    }
    if (pSlpSessionId_1->slpId.present == SLPAddress_PR_fQDN)
    {
      OCTET_STRING_t *pOct1 = &pSlpSessionId_1->slpId.choice.fQDN;
      OCTET_STRING_t *pOct2 = &pSlpSessionId_2->slpId.choice.fQDN;
      if (!compareOctets(pOct1, pOct2))
        return false;
    }
  }
  return true;
}

///////////////////////////////////////////////////////////////////////////////
//! Verify consistency of set session ID
/*!
  \param pSetSessionId_1   : Pointer to set session Id
  \param pSetSessionId_2   : Pointer to set session Id
  \return                  : true if consistent
*/
static bool verifySetSessionId(const SetSessionID_t *pSetSessionId_1,
                               const SetSessionID_t *pSetSessionId_2)
{
  if (pSetSessionId_1 == NULL && pSetSessionId_2 == NULL)
    return true;

  if (pSetSessionId_1 == NULL || pSetSessionId_2 == NULL)
    return false;

  if (pSetSessionId_1->sessionId != pSetSessionId_2->sessionId)
    return false;

  if (pSetSessionId_1->setId.present != pSetSessionId_2->setId.present)
    return false;

  /* TODO add also check if the content of setId is consistent... */

  return true;
}

///////////////////////////////////////////////////////////////////////////////
//! Compare cotets
/*!
  \param pOct1   : Pointer to octet
  \param pOct2   : Pointer to octet
  \return        : true if equal
*/
static bool compareOctets(const OCTET_STRING_t *pOct1, const OCTET_STRING_t *pOct2)
{
  int i;
  if (pOct1->size != pOct2->size)
    return false;
  for (i = 0; i < pOct1->size; i++)
  {
    if (pOct1->buf[i] != pOct2->buf[i])
      return false;
  }

  return true;
}

static void safeFreeHandler(suplHandler_t *pHandler)
{
  /* deallocate the pSetId if allocated */
  if (pHandler->pSetId != NULL)
  {
    ASN_STRUCT_FREE(asn_DEF_SetSessionID, pHandler->pSetId);
    pHandler->pSetId = NULL;
  }

  /* deallocate the pSlpId if allocated */
  if (pHandler->pSlpId != NULL)
  {
    ASN_STRUCT_FREE(asn_DEF_SlpSessionID, pHandler->pSlpId);
    pHandler->pSlpId = NULL;
  }

  /* deallocate the pNiMsg */
  if (pHandler->pNiMsg != NULL)
  {
    ASN_STRUCT_FREE(asn_DEF_ULP_PDU, pHandler->pNiMsg);
    pHandler->pNiMsg = NULL;
  }

  /* Deallocating the hanlder itself */
  MC_FREE(pHandler);
}

// suplSMmanager.cpp
