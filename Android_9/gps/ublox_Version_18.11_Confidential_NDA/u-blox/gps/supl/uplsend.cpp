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
  \brief  ULP message manager

  Module for managing SUPL messages sent from the SET to the SPL
*/

#include <malloc.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "std_types.h"
#include "uplsend.h"

#include "hardware/gps.h"
#include "ubx_log.h"
#include "ubx_rilIf.h"
#include "ubxgpsstate.h"

///////////////////////////////////////////////////////////////////////////////
// Types & Definitions

#define KILOMETRES_PER_KNOT 1.852
using namespace ublox::log::stringtables;

///////////////////////////////////////////////////////////////////////////////
// Static functions

static void fillVersion(ULP_PDU_t *pMsg);
static void fillSessionId(ULP_PDU_t *pMsg, SetSessionID_t *pSetId, SlpSessionID_t *pSlpId);
static void fillLocation(LocationId_t *pLocId);
static void fillAssistRequestFlags(struct RequestedAssistData *pAssistedData);
static void sendAndFree(SUPL_CONNECTION *supl_connection, ULP_PDU_t *pMsg);
static void allocateAndWriteBitString(BIT_STRING_t *pDest, long long int value, int size);
static void setCapability(SETCapabilities_t *pCapab, PosMethod_t requestedPosMethod);
static struct Position *allocatePosition(double lat, double lon);
static Ver_t *allocateVer(long long hash);
static bool charToBytes(uint8_t *pBuffer, const char *pIdentity);
static void fillSubscriberIdentity(OCTET_STRING_t *pSubscriberIdentity, const char *pIdentity);

///////////////////////////////////////////////////////////////////////////////
//! Utility for copying the slpSessionId
/*! Copy and allocate the slpSessionId
  \param pOrig : Source session structure to be copied
  \return Destination session structure
*/
SlpSessionID_t *copySlpId(const SlpSessionID_t *pOrig)
{
  if (pOrig == NULL)
  {
    return NULL;
  }

  SlpSessionID_t *pDest = (SlpSessionID_t *)MC_CALLOC(sizeof(SlpSessionID_t), 1);
  if (pDest == NULL)
  {
    UBX_LOG(LCAT_ERROR, "error in allocation");
    return NULL;
  }

  /* copy all the SlpSessionId structure */
  memcpy(pDest, pOrig, sizeof(SlpSessionID_t));

  /* Allocation of the buffer used for the octect string */
  if (pDest->slpId.present == SLPAddress_PR_iPAddress)
  {
    /* it is a IP address... */
    if (pDest->slpId.choice.iPAddress.present == IPAddress_PR_ipv4Address)
    {
      /* Allocate 4 bytes */
      pDest->slpId.choice.iPAddress.choice.ipv4Address.buf = (uint8_t *)MC_CALLOC(4, 1);
      if (pDest->slpId.choice.iPAddress.choice.ipv4Address.buf == NULL)
      {
        UBX_LOG(LCAT_ERROR, "error in allocation");
        pDest->slpId.choice.iPAddress.choice.ipv4Address.size = 0;
      }
      else
      {
        /* copy from the input 4 bytes*/
        memcpy(pDest->slpId.choice.iPAddress.choice.ipv4Address.buf,
               pOrig->slpId.choice.iPAddress.choice.ipv4Address.buf,
               4);
      }
    }
    else if (pDest->slpId.choice.iPAddress.present == IPAddress_PR_ipv6Address)
    {
      /* Allocate 16 bytes */
      pDest->slpId.choice.iPAddress.choice.ipv6Address.buf = (uint8_t *)MC_CALLOC(16, 1);
      if (pDest->slpId.choice.iPAddress.choice.ipv6Address.buf == NULL)
      {
        UBX_LOG(LCAT_ERROR, "error in allocation");
        pDest->slpId.choice.iPAddress.choice.ipv6Address.size = 0;
      }
      else
      {
        /* copy from the input 16 bytes*/
        memcpy(pDest->slpId.choice.iPAddress.choice.ipv6Address.buf,
               pOrig->slpId.choice.iPAddress.choice.ipv6Address.buf,
               16);
      }
    }
  }
  else if (pDest->slpId.present == SLPAddress_PR_fQDN)
  {
    /* It is an fQDN... */
    /* Allocate size bytes */
    int size = pOrig->slpId.choice.fQDN.size;
    pDest->slpId.choice.fQDN.buf = (uint8_t *)MC_CALLOC((unsigned int)size, 1);
    if (pDest->slpId.choice.fQDN.buf == NULL)
    {
      UBX_LOG(LCAT_ERROR, "error in allocation");
      pDest->slpId.choice.fQDN.size = 0;
    }
    else
    {
      /* copy the buffer */
      memcpy(pDest->slpId.choice.fQDN.buf, pOrig->slpId.choice.fQDN.buf, (unsigned int)size);
    }
  }

  /* writing session Id, need to allocate the 4octets buffer */
  pDest->sessionID.buf = (uint8_t *)MC_CALLOC(4, 1);
  if (pDest->sessionID.buf == 0)
  {
    UBX_LOG(LCAT_ERROR, "error in allocation");
    pDest->sessionID.size = 0;
  }
  else
  {
    pDest->sessionID.size = 4;
    /* copy the content of the session id, 4 bytes */
    memcpy(pDest->sessionID.buf, pOrig->sessionID.buf, 4);
  }

  return pDest;
}

///////////////////////////////////////////////////////////////////////////////
//! Utility for copying the setSessionId
/*! Copy and allocate the slpSessionId
  \param pOrig : Source session structure to be copied
  \return Destination session structure
*/
SetSessionID_t *copySetId(const SetSessionID_t *pOrig)
{
  if (pOrig == NULL)
  {
    return NULL;
  }

  SetSessionID_t *pDest = (SetSessionID_t *)MC_CALLOC(sizeof(SetSessionID_t), 1);
  if (pDest == NULL)
  {
    UBX_LOG(LCAT_ERROR, "error in allocation");
    return NULL;
  }

  /* copy all the SetSessionId structure */
  memcpy(pDest, pOrig, sizeof(SetSessionID_t));

  switch (pOrig->setId.present)
  {
  case SETId_PR_msisdn:
    pDest->setId.choice.msisdn.buf =
      (uint8_t *)MC_CALLOC((unsigned int)pDest->setId.choice.msisdn.size, 1);
    if (pDest->setId.choice.msisdn.buf == NULL)
    {
      UBX_LOG(LCAT_ERROR, "error in allocation");
      free(pDest);
      return NULL;
    }
    memcpy(pDest->setId.choice.msisdn.buf,
           pOrig->setId.choice.msisdn.buf,
           (unsigned int)pDest->setId.choice.msisdn.size);
    break;
  case SETId_PR_mdn:
    pDest->setId.choice.mdn.buf =
      (uint8_t *)MC_CALLOC((unsigned int)pDest->setId.choice.mdn.size, 1);
    if (pDest->setId.choice.mdn.buf == NULL)
    {
      UBX_LOG(LCAT_ERROR, "error in allocation");
      free(pDest);
      return NULL;
    }
    memcpy(pDest->setId.choice.mdn.buf,
           pOrig->setId.choice.mdn.buf,
           (unsigned int)pDest->setId.choice.mdn.size);
    break;
  case SETId_PR_min:
    pDest->setId.choice.min.buf =
      (uint8_t *)MC_CALLOC((unsigned int)pDest->setId.choice.min.size, 1);
    if (pDest->setId.choice.min.buf == NULL)
    {
      UBX_LOG(LCAT_ERROR, "error in allocation");
      free(pDest);
      return NULL;
    }
    memcpy(pDest->setId.choice.min.buf,
           pOrig->setId.choice.min.buf,
           (unsigned int)pDest->setId.choice.min.size);
    break;
  case SETId_PR_imsi:
    pDest->setId.choice.imsi.buf =
      (uint8_t *)MC_CALLOC((unsigned int)pDest->setId.choice.imsi.size, 1);
    if (pDest->setId.choice.imsi.buf == NULL)
    {
      UBX_LOG(LCAT_ERROR, "error in allocation");
      free(pDest);
      return NULL;
    }
    memcpy(pDest->setId.choice.imsi.buf,
           pOrig->setId.choice.imsi.buf,
           (unsigned int)pDest->setId.choice.imsi.size);
    break;
  case SETId_PR_nai:
    pDest->setId.choice.nai.buf =
      (uint8_t *)MC_CALLOC((unsigned int)pDest->setId.choice.nai.size, 1);
    if (pDest->setId.choice.nai.buf == NULL)
    {
      UBX_LOG(LCAT_ERROR, "error in allocation");
      free(pDest);
      return NULL;
    }
    memcpy(pDest->setId.choice.nai.buf,
           pOrig->setId.choice.nai.buf,
           (unsigned int)pDest->setId.choice.nai.size);
    break;
  case SETId_PR_iPAddress:
    /* We assume the octet of ipv4 and ip6 are identically distributed */
    pDest->setId.choice.iPAddress.choice.ipv4Address.buf =
      (uint8_t *)MC_CALLOC((unsigned int)pDest->setId.choice.iPAddress.choice.ipv4Address.size, 1);
    if (pDest->setId.choice.iPAddress.choice.ipv4Address.buf == NULL)
    {
      UBX_LOG(LCAT_ERROR, "error in allocation");
      free(pDest);
      return NULL;
    }
    memcpy(pDest->setId.choice.iPAddress.choice.ipv4Address.buf,
           pOrig->setId.choice.iPAddress.choice.ipv4Address.buf,
           (unsigned int)pDest->setId.choice.iPAddress.choice.ipv4Address.size);
    break;
  case SETId_PR_NOTHING:
  default:
    break;
  }

  return pDest;
}

///////////////////////////////////////////////////////////////////////////////
//! initiate a SUPL session from SET
/*! Send a SUPL START message to the SLP server
  \param supl_connection    : file descriptor of the socket
  \param pSetId : pointer to set session Id
  \return 0 if succesfull, <0 if not
*/
int sendSuplStart(SUPL_CONNECTION *supl_connection, SetSessionID_t *pSetId)
{
  ULP_PDU_t msg;

  SETId_t noSetId_1;

  /* SuplStart always use noSetId */
  memset(&noSetId_1, 0, sizeof(SETId_t));
  noSetId_1.present = SETId_PR_NOTHING;

  /* clean the memory */
  memset(&msg, 0, sizeof(msg));

  /* Length magic number... will be replaced after with the correct one! */
  msg.length = 0x1311;

  /* Set the version */
  fillVersion(&msg);

  /* session id without sld id */
  fillSessionId(&msg, pSetId, NULL);

  /* Pointer to the Supl Start payload */
  msg.message.present = UlpMessage_PR_msSUPLSTART;

  /* Pointer to the Supl Start payload */
  SUPLSTART_t *pSuplStart = &(msg.message.choice.msSUPLSTART);

  /* Store the capability temporary pointer */
  setCapability(&pSuplStart->sETCapabilities, PosMethod_noPosition);

  /* Location filled using the information from the GSM block... */
  fillLocation(&(pSuplStart->locationId));

  /* QOP assumed (for the moment) always NULL */
  pSuplStart->qoP = NULL;

  logAgps.write(0x02000000, "%d, 1.0.0, SUPL_START", pSetId->sessionId);
  sendAndFree(supl_connection, &msg);

  return 0;
}

///////////////////////////////////////////////////////////////////////////////
//! Send a SUPL end session from SET
/*! Send a SUPL END message to the SLP server
  \param supl_connection    : file descriptor of the socket
  \param pParam : parameter structure pointer
  \return 0 if succesfull, <0 if not
*/
int sendSuplEnd(SUPL_CONNECTION *supl_connection, const suplEndParam_t *pParam)
{
  ULP_PDU_t msg;

  /* clean the memory */
  memset(&msg, 0, sizeof(msg));

  /* Length magic numbver... will be replaed after with the correct one! */
  msg.length = 0x1311;

  /* Set the version */
  fillVersion(&msg);

  /* session id without sld id */
  fillSessionId(&msg, pParam->pSetId, pParam->pSlpId);

  /* Pointer to the Supl Start payload */
  msg.message.present = UlpMessage_PR_msSUPLEND;

  /* Pointer to the Supl Start payload */
  SUPLEND_t *pSuplEnd = &(msg.message.choice.msSUPLEND);

  /* Status is optional, only if > 0 will be allocated */
  if (pParam->status > 0)
  {
    /* allocate the status structure... */
    StatusCode_t *pStatus = (StatusCode_t *)MC_CALLOC(sizeof(StatusCode_t), 1);
    if (pStatus == NULL)
    {
      UBX_LOG(LCAT_ERROR, "allocation error");
    }
    else
    {
      /* Write the value */
      *pStatus = pParam->status;
    }

    /* link the allocated object */
    pSuplEnd->statusCode = pStatus;
  }

  /* Position is optional, only if posEn == 1 will be allocated */
  if (pParam->posEn == 1)
  {
    CDatabase *pDatabase = CAndroidDatabase::getInstance();

    double lat, lon = 0.0;
    bool posAvail = (pDatabase->GetOutput(CDatabase::DATA_LATITUDE_DEGREES, lat) &&
                     pDatabase->GetOutput(CDatabase::DATA_LONGITUDE_DEGREES, lon));
    if (posAvail)
    {
      pSuplEnd->position = allocatePosition(lat, lon);
    }
  }

  /* Verification hash is optional, only if posEn == 1 will be allocated */
  if (pParam->verEn == 1)
  {
    pSuplEnd->ver = allocateVer(pParam->hash);
  }

  logAgps.write(0x02000000, "%d, 1.0.0, SUPL_END", pParam->pSetId->sessionId);
  sendAndFree(supl_connection, &msg);

  return 0;
}

///////////////////////////////////////////////////////////////////////////////
//! SUPL POS session from SET
/*! Send a SUPL POS message to the SLP server
    for the moment only RRLP payload is available
  \param supl_connection    : file descriptor of the socket
  \param pParam : parameter structure pointer
  \return 0 if succesfull, <0 if not
*/
int sendSuplPos(SUPL_CONNECTION *supl_connection, const suplPosParam_t *pParam)
{
  CDatabase *pDatabase = CAndroidDatabase::getInstance();
  ULP_PDU_t msg;

  /* clean the memory */
  memset(&msg, 0, sizeof(msg));

  /* Length magic numbver... will be replaced after with the correct one! */
  msg.length = 0x1311;

  /* Set the version */
  fillVersion(&msg);

  /* session id */
  fillSessionId(&msg, pParam->pSetId, pParam->pSlpId);

  /* Pointer to the Supl Pos payload */
  msg.message.present = UlpMessage_PR_msSUPLPOS;

  /* Pointer to the Supl Pos payload */
  SUPLPOS_t *pSuplPos = &(msg.message.choice.msSUPLPOS);

  /* Only the RRLP payload is available */
  pSuplPos->posPayLoad.present = PosPayLoad_PR_rrlpPayload;

  /* Allocate the buffer and clear it */
  pSuplPos->posPayLoad.choice.rrlpPayload.buf = (uint8_t *)MC_CALLOC((unsigned int)pParam->size, 1);
  if (pSuplPos->posPayLoad.choice.rrlpPayload.buf == NULL)
  {
    UBX_LOG(LCAT_ERROR, "Out of memory");
  }
  else
  {
    /* Copy the buffer content */
    memcpy(pSuplPos->posPayLoad.choice.rrlpPayload.buf, pParam->buffer, (unsigned int)pParam->size);
    /* Set the dimension of the buffer in octets */
    pSuplPos->posPayLoad.choice.rrlpPayload.size = pParam->size;
  }

  /* Only if speed is given by the GPS... */
  double speed;
  if (pDatabase->GetOutput(CDatabase::DATA_SPEED_KNOTS, speed)) // getGpsData(GPS_IS_SPEED_AVAIL))
  {
    speed *= KILOMETRES_PER_KNOT; // Convert to kmh
    /* velocity is optional.. must be allocated if necessary */
    struct Velocity *pVel = (Velocity *)MC_CALLOC(sizeof(struct Velocity), 1);
    if (pVel == NULL)
    {
      UBX_LOG(LCAT_ERROR, "allocation error");
    }
    else
    {
      /* For the moment, only the horizontal speed is foreseen.
         may be extended in the future!!! */
      pVel->present = Velocity_PR_horvel;

      /* retrieve and write the bering */
      allocateAndWriteBitString(&(pVel->choice.horvel.horspeed), (long long)(speed + 0.5), 16);

      /* retrieve and write the speed */
      double bearing = 0;
      pDatabase->GetOutput(CDatabase::DATA_TRUE_HEADING_DEGREES, bearing);
      allocateAndWriteBitString(&(pVel->choice.horvel.bearing), (long long)bearing, 9);
    }

    /* link the allocated velocity field */
    pSuplPos->velocity = pVel;
  }

  logAgps.write(0x02000000, "%d, 1.0.0, SUPL_POS # send", pParam->pSetId->sessionId);
  sendAndFree(supl_connection, &msg);

  return 0;
}

///////////////////////////////////////////////////////////////////////////////
//! SUPL POS INIT session from SET
/*!
  Send a SUPL POS INIT message to the SLP server
  \param supl_connection    : file descriptor of the socket
  \param pParam : parameter structure pointer
  \return 0 if succesfull, <0 if not
 */
int sendSuplPosInit(SUPL_CONNECTION *supl_connection, const suplPosInitParam_t *pParam)
{
  ULP_PDU_t msg;

  /* clean the memory */
  memset(&msg, 0, sizeof(msg));

  /* Length magic number... will be replaced after with the correct one! */
  msg.length = 0x1311;

  /* Set the version */
  fillVersion(&msg);

  /* session id without sld id */
  fillSessionId(&msg, pParam->pSetId, pParam->pSlpId);

  /* Pointer to the Supl Start payload */
  msg.message.present = UlpMessage_PR_msSUPLPOSINIT;

  /* Pointer to the Supl Start payload */
  SUPLPOSINIT_t *pSuplPosInit = &(msg.message.choice.msSUPLPOSINIT);

  /* set the capability */
  setCapability(&pSuplPosInit->sETCapabilities, pParam->requestedPosMethod);

  /* Assisted data is optional, only if assEn == 1 will be allocated */
  if (pParam->assEn == 1)
  {
    /* allocate the assisted data structure */
    struct RequestedAssistData *pAssistedData =
      (RequestedAssistData *)MC_CALLOC(sizeof(struct RequestedAssistData), 1);
    if (pAssistedData == NULL)
    {
      UBX_LOG(LCAT_ERROR, "allocation error");
    }
    else
    {
      fillAssistRequestFlags(pAssistedData);
    }
    /* link the allocated object */
    pSuplPosInit->requestedAssistData = pAssistedData;
  }

  /* Location filled using the information from the GSM block... */
  fillLocation(&(pSuplPosInit->locationId));

  /* Position is optional, only if posEn == 1 will be allocated */
  if (pParam->posEn == 1)
  {
    pSuplPosInit->position = allocatePosition(pParam->lat, pParam->lon);
  }

  /* Verification hash is optional, only if posEn == 1 will be allocated */
  if (pParam->verEn == 1)
  {
    pSuplPosInit->ver = allocateVer(pParam->hash);
  }

  logAgps.write(0x02000000,
                (pSuplPosInit->locationId.cellInfo.present == CellInfo_PR_gsmCell) ?
                  "%d, 1.0.0, SUPL_POS_INIT # LAC_CELLID : %d, %d" :
                  "%d, 1.0.0, SUPL_POS_INIT",
                pParam->pSetId->sessionId,
                pSuplPosInit->locationId.cellInfo.choice.gsmCell.refLAC,
                pSuplPosInit->locationId.cellInfo.choice.gsmCell.refCI);
  sendAndFree(supl_connection, &msg);

  return 0;
}

///////////////////////////////////////////////////////////////////////////////
//! Function used for allocating the Set session Id
/*! this function is used globally to allocate and fill the default SetSessionId

  \param sessionId      : session identifier
  \return               : pointer to the newly allocated SetSessionId
*/
SetSessionID_t *fillDefaultSetId(int sessionId)
{
  CRilIf *pRIL = CRilIf::getInstance();

  if (sessionId < 0)
  {
    return NULL;
  }

  /* allocate the space for the set session id */
  SetSessionID_t *setSessionId = (SetSessionID_t *)MC_CALLOC(sizeof(SetSessionID_t), 1);
  if (setSessionId == NULL)
  {
    UBX_LOG(LCAT_ERROR, "error in allocation");
    return NULL;
  }

  /* set the sessionId */
  setSessionId->sessionId = sessionId;

  /* Check the preference for the SETID */
  if (strcmp(pRIL->getIsmi(), "") != 0)
  {
    // IMSI present
    UBX_LOG(LCAT_VERBOSE, "Using IMSI %s", pRIL->getIsmi());
    setSessionId->setId.present = SETId_PR_imsi;
    fillSubscriberIdentity(&setSessionId->setId.choice.imsi, pRIL->getIsmi());
  }
  else if (strcmp(pRIL->getMsisdn(), "") != 0)
  {
    // MSISDN present
    UBX_LOG(LCAT_VERBOSE, "Using MSISDN %s", pRIL->getMsisdn());
    setSessionId->setId.present = SETId_PR_msisdn;
    fillSubscriberIdentity(&setSessionId->setId.choice.msisdn, pRIL->getMsisdn());
  }
  else
  {
    // Use IP address
    UBX_LOG(LCAT_VERBOSE, "Type AGPS_SETID_TYPE_NONE");
    struct in_addr ipAddr;

    setSessionId->setId.present = SETId_PR_iPAddress;
    setSessionId->setId.choice.iPAddress.present = IPAddress_PR_ipv4Address;
    setSessionId->setId.choice.iPAddress.choice.ipv4Address.size = sizeof(ipAddr.s_addr);
    /* allocate and verify allocation of the 4 bytes */
    {
      /* using tmpPtr to get rid of incompatible type warning */
      __u32 *tmpPtr;
      tmpPtr = reinterpret_cast<__u32 *>(MC_CALLOC(sizeof(ipAddr.s_addr), 1));
      setSessionId->setId.choice.iPAddress.choice.ipv4Address.buf =
        reinterpret_cast<uint8_t *>(tmpPtr);
    }
    if (setSessionId->setId.choice.iPAddress.choice.ipv4Address.buf == 0)
    {
      UBX_LOG(LCAT_ERROR, "error in allocation");
      setSessionId->setId.choice.iPAddress.choice.ipv4Address.size = 0;
    }
    else
    {
      /* Retrieving the local IP address */
      ipAddr = pRIL->getClientIP();

      if (ipAddr.s_addr != 0)
      {
        memcpy(setSessionId->setId.choice.iPAddress.choice.ipv4Address.buf,
               &ipAddr.s_addr,
               sizeof(ipAddr.s_addr));
        UBX_LOG(LCAT_VERBOSE,
                "Type AGPS_SETID_TYPE_NONE - IP Address %u.%u.%u.%u",
                setSessionId->setId.choice.iPAddress.choice.ipv4Address.buf[0],
                setSessionId->setId.choice.iPAddress.choice.ipv4Address.buf[1],
                setSessionId->setId.choice.iPAddress.choice.ipv4Address.buf[2],
                setSessionId->setId.choice.iPAddress.choice.ipv4Address.buf[3]);
      }
      else
      {
        UBX_LOG(LCAT_WARNING, "Cannot set the local address");
      }
    }
  }

  /* returning the pointer to the allocated SetSessionId */
  return setSessionId;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// static functions

///////////////////////////////////////////////////////////////////////////////
//! Function used for filling the version field
/*! this function is used locally to fill the ULP message version field.
  \param pMsg   : pointer to the ULP message
  \return Pointer to allocated ver structure
*/
static void fillVersion(ULP_PDU_t *pMsg)
{
  /* always set the version to 1.0.0 */
  pMsg->version.maj = 1;
  pMsg->version.min = 0;
  pMsg->version.servind = 0;

  return;
}

///////////////////////////////////////////////////////////////////////////////
//! Function used for filling in a subscriber identity field
/*!
  \param pSubscriberIdentity : Pointer to destination subscriber identity field
  \param pIdentity           : Pointer to string representation of the
  subscriber's identity
*/
static void fillSubscriberIdentity(OCTET_STRING_t *pSubscriberIdentity, const char *pIdentity)
{
  pSubscriberIdentity->size = 8;
  /* allocate and verify allocation of the 8 bytes */
  pSubscriberIdentity->buf = (uint8_t *)MC_CALLOC(8, 1);
  if (pSubscriberIdentity->buf == NULL)
  {
    UBX_LOG(LCAT_ERROR, "error in allocation");
    pSubscriberIdentity->size = 0;
  }
  else
  {
    // Convert to bytes
    if (!charToBytes(pSubscriberIdentity->buf, pIdentity))
    {
      UBX_LOG(LCAT_WARNING, "Cannot set the identity");
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
//! Function used for filling the session Id structures
/*! this function is used locally to fill the ULP message session Id on the
    ULP message.
    it needs to allocate the optional field:
    - slp session ID
    - set session Id
    - the octet type string used for the local and the SLP server
    - the octet for the slp session Id

  \param pMsg      : Pointer to the ULP message
  \param pSetId    : Integer representing the SET session ID
  \param pSlpId    : Pointer to complete structure of the SLP ID (session + ID)
*/
static void fillSessionId(ULP_PDU_t *pMsg, SetSessionID_t *pSetId, SlpSessionID_t *pSlpId)
{
  UBX_LOG(LCAT_VERBOSE, "pSetId (%p)  pSlpId (%p)", pSetId, pSlpId);

  if (pSlpId != NULL)
  {
    /* allocate the space for the slpsession id */
    SlpSessionID_t *slpSessionId = copySlpId(pSlpId);
    /* linking the slpSession of the message */
    pMsg->sessionID.slpSessionID = slpSessionId;
  }
  else
  {
    pMsg->sessionID.slpSessionID = NULL;
  }

  if (pSetId != NULL)
  {
    /* allocate the space for the slpsession id */
    SetSessionID_t *setSessionId = copySetId(pSetId);
    /* linking the slpSession of the message */
    pMsg->sessionID.setSessionID = setSessionId;
  }
  else
  {
    pMsg->sessionID.setSessionID = NULL;
  }
}

///////////////////////////////////////////////////////////////////////////////
//! Function used for converting a string subscriber identity to a byte sequence
/*!
  \param pBuffer   : Pointer to destination buffer
  \param pIdentity : Pointer to string representation of the subscriber's
  identity
  \return          : true if conversion successful, false if not
*/
static bool charToBytes(uint8_t *pBuffer, const char *pIdentity)
{
  if (pBuffer == NULL)
  {
    return false;
  }

  memset(pBuffer, 0, 8);
  unsigned int len = strlen(pIdentity);
  const char *ptr = pIdentity;

  /* generate the buffer as SUPL want */
  for (unsigned int i = 0; i < len; i++)
  {
    if (*ptr < '0' || *ptr > '9')
    {
      /* Out of range, only numeric are possible */
      UBX_LOG(LCAT_ERROR, "Identity value not numeric (0x%.2X at position %d)", *ptr, i);
      return false;
    }

    if ((i % 2) == 0)
    {
      pBuffer[i / 2] = (unsigned char)(*ptr - '0');
    }
    else
    {
      pBuffer[i / 2] += (unsigned char)((*ptr - '0') << 4);
    }
    ptr++;
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////////
//! Function used for filling the location structure
/*! Function used for filling in the location parameters, retrieved from the GSM
  subsystem
  \param pLocId   : Pointer to the location structure
*/
static void fillLocation(LocationId_t *pLocId)
{
  CRilIf *pRIL = CRilIf::getInstance();

  /* Get the GSM informations.. */
  AGpsRefLocation refloc = pRIL->getRefLocation();

  UBX_LOG(
    LCAT_VERBOSE, "Location type %d (%s)", refloc.type, _LOOKUPSTR(refloc.type, AGpsRefLocation));

  /* depending on the cell type, different informations are requested... */
  switch (refloc.type)
  {
  case AGPS_REF_LOCATION_TYPE_GSM_CELLID:
    pLocId->cellInfo.present = CellInfo_PR_gsmCell;
    pLocId->cellInfo.choice.gsmCell.refMCC = refloc.u.cellID.mcc;
    pLocId->cellInfo.choice.gsmCell.refMNC = refloc.u.cellID.mnc;
    pLocId->cellInfo.choice.gsmCell.refLAC = refloc.u.cellID.lac;
    pLocId->cellInfo.choice.gsmCell.refCI = (long)refloc.u.cellID.cid;
    pLocId->status = Status_current;
    UBX_LOG(LCAT_VERBOSE,
            "GSM (2G) cell, cell mcc: %ld, mnc: %ld, lac: %ld uc: %ld",
            pLocId->cellInfo.choice.gsmCell.refMCC,
            pLocId->cellInfo.choice.gsmCell.refMNC,
            pLocId->cellInfo.choice.gsmCell.refLAC,
            pLocId->cellInfo.choice.gsmCell.refCI);
    break;

  case AGPS_REF_LOCATION_TYPE_UMTS_CELLID:
    pLocId->cellInfo.present = CellInfo_PR_wcdmaCell;
    pLocId->cellInfo.choice.wcdmaCell.refMCC = refloc.u.cellID.mcc;
    pLocId->cellInfo.choice.wcdmaCell.refMNC = refloc.u.cellID.mnc;
    pLocId->cellInfo.choice.wcdmaCell.refUC = (long)refloc.u.cellID.cid;
    pLocId->status = Status_current;
    UBX_LOG(LCAT_VERBOSE,
            "WCDMA (3G) cell mcc: %ld, mnc: %ld, uc: %ld",
            pLocId->cellInfo.choice.wcdmaCell.refMCC,
            pLocId->cellInfo.choice.wcdmaCell.refMNC,
            pLocId->cellInfo.choice.wcdmaCell.refUC);
    break;

  case AGPS_REG_LOCATION_TYPE_MAC:
  default:
    // pLocId->cellInfo.present = CellInfo_PR_NOTHING;
    pLocId->cellInfo.present = CellInfo_PR_gsmCell;
    pLocId->cellInfo.choice.gsmCell.refMCC = 0;
    pLocId->cellInfo.choice.gsmCell.refMNC = 0;
    pLocId->cellInfo.choice.gsmCell.refLAC = 0;
    pLocId->cellInfo.choice.gsmCell.refCI = 0;
    pLocId->status = Status_unknown;
    UBX_LOG(LCAT_WARNING, "No cell info");
    break;
  }

  /* Assuming we have a cell available */
}

///////////////////////////////////////////////////////////////////////////////
//! Function used for filling the assistance data request structure
/*!
  \param pAssistedData : Pointer to the assistance request structure
*/
static void fillAssistRequestFlags(struct RequestedAssistData *pAssistedData)
{
  CUbxGpsState *pUbxGpsState = CUbxGpsState::getInstance();
  if (pUbxGpsState == nullptr)
  {
    UBX_LOG(LCAT_ERROR, "Could not get UbxGpsState instance");
    return;
  }

  pAssistedData->almanacRequested = pUbxGpsState->getAlamanacRequest();
  pAssistedData->utcModelRequested = pUbxGpsState->getUtcModelRequest();
  pAssistedData->ionosphericModelRequested = pUbxGpsState->getIonosphericModelRequest();
  pAssistedData->dgpsCorrectionsRequested = pUbxGpsState->getDgpsCorrectionsRequest();
  pAssistedData->referenceLocationRequested = pUbxGpsState->getRefLocRequest();
  pAssistedData->referenceTimeRequested = pUbxGpsState->getRefTimeRequest();
  pAssistedData->acquisitionAssistanceRequested = pUbxGpsState->getAcquisitionAssistRequest();
  pAssistedData->realTimeIntegrityRequested = pUbxGpsState->getRealTimeIntegrityRequest();
  pAssistedData->navigationModelRequested = pUbxGpsState->getNavigationModelRequest();

  /* For the time being, this is always NULL */
  pAssistedData->navigationModelData = NULL;
  // specs say we have to fill this navigationModelData if
  // navigationModelRequested is set
  if (pAssistedData->navigationModelRequested)
  {
    pAssistedData->navigationModelData =
      (NavigationModel_1 *)MC_CALLOC(sizeof(struct NavigationModel_1), 1);
    if (pAssistedData->navigationModelData == nullptr)
    {
      UBX_LOG(LCAT_ERROR, "Could not allocate navigationModelData");
      return;
    }
    // from NavigationModel_1.h
    // we should report the satellites that we valid ephemeris in our handset
    // we report nSAT 0 so we have higher load to download
    pAssistedData->navigationModelData->gpsWeek = 0;
    pAssistedData->navigationModelData->gpsToe = 0;
    pAssistedData->navigationModelData->nSAT = 0;
    pAssistedData->navigationModelData->toeLimit = 0;
    pAssistedData->navigationModelData->satInfo = NULL;
  }
}

///////////////////////////////////////////////////////////////////////////////
//! Send the message, after free the structure
/*! This function is used to invoke the ASN1 encoder, and then send to the
    socket / file, the content of the message.
    When complete, the message will be cleared, EXCEPT the msg structure itself,
  that is
    not generated dynamically, being a local variable of the calling function.
  \param supl_connection  : Pointer to file descriptor of file or socket
  \param pMsg : Pointer to the SUPL message structure
*/
static void sendAndFree(SUPL_CONNECTION *supl_connection, ULP_PDU_t *pMsg)
{
  ssize_t res;
  char *buf = NULL;

  res = uper_encode_to_new_buffer(&asn_DEF_ULP_PDU, NULL, pMsg, (void **)&buf);
  if (res < 0)
  {
    UBX_LOG(LCAT_ERROR, "Encoding failure (%li)", (long int)res);
    ASN_STRUCT_FREE_CONTENTS_ONLY(asn_DEF_ULP_PDU, pMsg);
    return;
  }

  UBX_LOG(LCAT_VERBOSE, "sent %d bytes", (int)res);

  if ((buf[0] != 0x13) || (buf[1] != 0x11))
  {
    UBX_LOG(LCAT_ERROR,
            "Strange, dimension do not fit.. (buf[0] = 0x%.2X, buf[1] = 0x%.2X)",
            buf[0],
            buf[1]);
  }

  buf[0] = (char)((res >> 8) & 0xFF);
  buf[1] = (char)(res & 0xFF);

  int nWrt = supl_connection_write(supl_connection, (unsigned char *)buf, res);
  if (nWrt != res)
  {
    UBX_LOG(LCAT_ERROR, "not all the bytes are written: %d out of %d", nWrt, (int)res);
  }

  /* overwrite the magic number with the correct length and log */
  pMsg->length = (long)res;
  logSupl(pMsg, false);

  /* freeing the buffer */
  MC_FREE(buf);

  /* Freeing the msg structure, is no more necessary... */
  ASN_STRUCT_FREE_CONTENTS_ONLY(asn_DEF_ULP_PDU, pMsg);
}

///////////////////////////////////////////////////////////////////////////////
//! Allocate and fill the BitString structure
/*! This function is used to allocate the buffer necessary for a certain
  bitstring
    (that is the minimum char vector containing the number of bits requested)
    and then fill the content with the value given by the value parameter.
  \param pDest       : Ponter to the destination structure
  \param value       : value to be filled... maximum 64 bits
  \param size        : number of bits to be filled
*/
static void allocateAndWriteBitString(BIT_STRING_t *pDest, long long value, int size)
{
  int dim = (size - 1) / 8 + 1; //!< minimum number of octets necessary to contain size bits
  long long bitMask =
    (size == 64) ? (long long)0xFFFFFFFFFFFFFFFFLL : (1LL << size) - 1LL; //!< bitmask for the vaule
  int i;

  /* Maximum size is 64 bits */
  if (size > 64)
  {
    UBX_LOG(LCAT_ERROR, "maximum bitstring size is 64");
  }

  /* allocate and reset the buffer = size in bytes!!!*/
  pDest->buf = (uint8_t *)MC_CALLOC((unsigned int)dim, 1);
  if (pDest->buf == 0)
  {
    UBX_LOG(LCAT_ERROR, "error in allocation");
    pDest->size = 0;
    return;
  }

  /* clear the unnecessary bit of the value */
  value &= bitMask;
  for (i = 0; i < dim; i++)
  {
    pDest->buf[i] = (unsigned char)value;
    value = (value >> 8);
  }

  /* Set the size of the bit string */
  pDest->size = dim;

  /* set the trailing bits */
  pDest->bits_unused = (dim * 8) - size;
}

///////////////////////////////////////////////////////////////////////////////
//! Function used for filling the a subset of the SET capabilities structure
/*! Only fills in the position technology and preferred method fields
    of the capabilities structure
  \param pCapab             : Pointer to the capabilities structure
  \param requestedPosMethod : Requested / desired position method
*/
static void setupPosMethod(SETCapabilities_t *pCapab, PosMethod_t requestedPosMethod)
{
  uint32_t capabilities = CGpsIf::getInstance()->getCapabilities();

  // Set up default response
  pCapab->posTechnology.agpsSETassisted = true;
  pCapab->posTechnology.agpsSETBased = true;

  switch (requestedPosMethod)
  {
  case PosMethod_agpsSETassisted:
    // MSA wanted
    if (capabilities & GPS_CAPABILITY_MSA)
    {
      pCapab->prefMethod = PosMethod_agpsSETassisted;
      UBX_LOG(LCAT_VERBOSE, "Request MSA, Got MSA");
    }
    break;

  case PosMethod_agpsSETbased:
    // MSB wanted
    if (capabilities & GPS_CAPABILITY_MSB)
    {
      pCapab->prefMethod = PrefMethod_agpsSETBasedPreferred;
      UBX_LOG(LCAT_VERBOSE, "Request MSB, Got MSB");
    }
    break;

  case PosMethod_agpsSETassistedpref:
    // MSA desired
    if (capabilities & GPS_CAPABILITY_MSA)
    {
      pCapab->prefMethod = PosMethod_agpsSETassisted;
      UBX_LOG(LCAT_VERBOSE, "Desired MSA, Got MSA");
    }
    else if (capabilities & GPS_CAPABILITY_MSB)
    {
      pCapab->prefMethod = PrefMethod_agpsSETBasedPreferred;
      UBX_LOG(LCAT_VERBOSE, "Desired MSA, Got MSB");
    }
    break;

  case PosMethod_agpsSETbasedpref:
    // MSB desired
    if (capabilities & GPS_CAPABILITY_MSB)
    {
      pCapab->prefMethod = PrefMethod_agpsSETBasedPreferred;
      UBX_LOG(LCAT_VERBOSE, "Desired MSB, Got MSB");
    }
    else if (capabilities & GPS_CAPABILITY_MSA)
    {
      pCapab->prefMethod = PosMethod_agpsSETassisted;
      UBX_LOG(LCAT_VERBOSE, "Desired MSB, Got MSA");
    }
    break;

  case PosMethod_eCID:
    // MSB desired
    if (capabilities & GPS_CAPABILITY_MSB)
    {
      pCapab->prefMethod = PrefMethod_agpsSETBasedPreferred;
      UBX_LOG(LCAT_VERBOSE, "Desired MSB, Got MSB");
    }
    else if (capabilities & GPS_CAPABILITY_MSA)
    {
      pCapab->prefMethod = PosMethod_agpsSETassisted;
      UBX_LOG(LCAT_VERBOSE, "Desired MSB, Got MSA");
    }
    break;

  default:
    // Shouldn't happen
    UBX_LOG(LCAT_VERBOSE, "Bad option");
    break;
  }
}

///////////////////////////////////////////////////////////////////////////////
//! Function used for filling the SET capabilities structure
/*! This function is used to fill the capability of the sevice
  \param pCapab             : Pointer to the capability structure
  \param requestedPosMethod : Requested / desired position method (if any)
*/
static void setCapability(SETCapabilities_t *pCapab, PosMethod_t requestedPosMethod)
{
  if (requestedPosMethod == PosMethod_noPosition)
  {
    // No position type requested so, well say what mode we are in
    CGpsIf *pGps = CGpsIf::getInstance();

    if (pGps->getMode() == GPS_POSITION_MODE_MS_BASED)
    {
      UBX_LOG(LCAT_VERBOSE, "set to MS-BASED");
      pCapab->posTechnology.agpsSETassisted = true;
      pCapab->posTechnology.agpsSETBased = true;
      /* The prefered way is to be a SET based! */
      pCapab->prefMethod = PrefMethod_agpsSETBasedPreferred;
    }
    else
    {
      UBX_LOG(LCAT_VERBOSE, "set to MS-ASSISTED");
      pCapab->posTechnology.agpsSETassisted = true;
      pCapab->posTechnology.agpsSETBased = true;
      /* The prefered way is to be a SET assisted! */
      pCapab->prefMethod = PrefMethod_agpsSETassistedPreferred;
    }
  }
  else
  {
    // See how well our capabilities match request
    setupPosMethod(pCapab, requestedPosMethod);
  }

  pCapab->posTechnology.autonomousGPS = false;
  pCapab->posTechnology.aFLT = false;
  pCapab->posTechnology.eCID = false;
  pCapab->posTechnology.eOTD = false;
  pCapab->posTechnology.oTDOA = false;

  /* we support for the moment only the RRLP! */
  pCapab->posProtocol.tia801 = false;
  pCapab->posProtocol.rrlp = true;
  pCapab->posProtocol.rrc = false;
}

///////////////////////////////////////////////////////////////////////////////
//! Allocate and fill position
/*! This function is used to allocate and fill the position structure
  \param lat       : Latitude to put into position structure
  \param lon       : Longiture to put into position structure
  \return          : Pointer to the allocated position structure
*/
static struct Position *allocatePosition(double lat, double lon)
{
  CDatabase *pDatabase = CAndroidDatabase::getInstance();

  /* allocate the position structure */
  struct Position *pPos = (Position *)MC_CALLOC(sizeof(struct Position), 1);
  if (pPos == NULL)
  {
    UBX_LOG(LCAT_ERROR, "allocation error");
    return NULL;
  }

  /*retrieving the current GPS time if available, otherwise leave the content
   * with 0's */
  struct tm curTime;
  memset(&curTime, 0, sizeof(curTime));
  TIMESTAMP timeStamp;
  if (pDatabase->GetOutput(CDatabase::DATA_COMMIT_TIMESTAMP, timeStamp))
  {
    // getGpsTime(&curTime);
    curTime.tm_sec = (int)(timeStamp.lMicroseconds / 1000000);
    curTime.tm_min = timeStamp.wMinute; /* minutes */
    curTime.tm_hour = timeStamp.wHour;  /* hours */
    curTime.tm_mday = timeStamp.wDay;   /* day of the month */
    curTime.tm_mon = timeStamp.wMonth;  /* month */
    curTime.tm_year = timeStamp.wYear;  /* year */
  }
  UBX_LOG(
    LCAT_VERBOSE, "Seconds=%d,Minutes=%d,Hour=%d", curTime.tm_sec, curTime.tm_min, curTime.tm_hour);
  /* transform to the asn1 field */
  asn_time2UT(&(pPos->timestamp), &curTime, 0);

  char buffer[100];
  memset(buffer, 0, sizeof(buffer));
  memcpy(buffer, pPos->timestamp.buf, (unsigned int)pPos->timestamp.size);
  UBX_LOG(LCAT_VERBOSE, "size is %d '%s'", pPos->timestamp.size, buffer);

  /* Only if speed is given by the GPS... */
  double speed;
  if (pDatabase->GetOutput(CDatabase::DATA_SPEED_KNOTS, speed))
  {
    speed *= KILOMETRES_PER_KNOT; // Convert to kmh
    /* velocity is optional.. must be allocated if necessary */
    struct Velocity *pVel = (Velocity *)MC_CALLOC(sizeof(struct Velocity), 1);
    if (pVel == NULL)
    {
      UBX_LOG(LCAT_ERROR, "allocation error");
    }
    else
    {
      /* For the moment, only the horizontal speed is foreseen.
         may be extended in the future!!! */
      pVel->present = Velocity_PR_horvel;

      /* retrieve nad write the bering */
      allocateAndWriteBitString(&(pVel->choice.horvel.horspeed), (long long)(speed + 0.5), 16);

      /* retrieve and write the speed */
      double bearing = 0;
      pDatabase->GetOutput(CDatabase::DATA_TRUE_HEADING_DEGREES, bearing);
      allocateAndWriteBitString(&(pVel->choice.horvel.bearing), (long long)bearing, 9);
    }
    /* link the allocated velocity field */
    pPos->velocity = pVel;
  }

  pPos->positionEstimate.latitudeSign = lat <= 0 ? 1 : 0; // Set North/South flag appropriately
  pPos->positionEstimate.latitude = (long)((fabs(lat) * 8388608.0) / 90.0);
  pPos->positionEstimate.longitude = (long)((lon * 16777216.0) / 360.0);

  /* all the rest of the information are optional...
     will be extended... */
  pPos->positionEstimate.uncertainty = NULL;
  pPos->positionEstimate.confidence = NULL;

  double alt;
  if (pDatabase->GetOutput(CDatabase::DATA_ALTITUDE_SEALEVEL_METERS, alt))
  {
    /* The altitude is available... */
    AltitudeInfo_t *pAlt = (AltitudeInfo_t *)MC_CALLOC(sizeof(AltitudeInfo_t), 1);
    if (pAlt == NULL)
    {
      UBX_LOG(LCAT_ERROR, "allocation error");
    }
    else
    {
      pAlt->altitudeDirection = (alt <= 0) ? 1 : 0; // Set 'sign' flag appropriately
      pAlt->altitude = (int)fabs(alt);
      /* No uncertainity on the altitude value... */
      pAlt->altUncertainty = 0;

      pPos->positionEstimate.altitudeInfo = pAlt;
    }
    /* save the allocated area on the main structure */
  }

  return pPos;
}

///////////////////////////////////////////////////////////////////////////////
//! allocate and fill hash ver structure
/*! This function is used to allocate and fill the ver structure
  \param hash :
  \return     : Pointer to the allocated ver structure
*/
static Ver_t *allocateVer(long long hash)
{
  Ver_t *pVer;

  /* allocate the version structure */
  pVer = (Ver_t *)MC_CALLOC(sizeof(Ver_t), 1);
  if (pVer == NULL)
  {
    UBX_LOG(LCAT_ERROR, "allocation error");
  }
  else
  {
    /* the hash should be place here... what is it??? */
    allocateAndWriteBitString(pVer, hash, 64);
  }

  return pVer;
}

// uplsend.cpp
