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
  \brief  ULP message decode

  Module for decoding SUPL messages sent from the SLP to the SET
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <malloc.h>
#include <time.h>

#include "ubx_log.h"
#include "upldecod.h"
#include "uplsend.h"

///////////////////////////////////////////////////////////////////////////////
//! Decode a SUPL message
/*! Function for decoding an incoming SUPL message. it will accept the payload
  of the
    message as argment, and it will return a pointer to the received structure.
  \param pBuffer     : payload recieved
  \param size       : size of the payload
  \return             pointer to ULP PDU message
*/
struct ULP_PDU *uplDecode(const char *pBuffer, int size)
{
  asn_dec_rval_t rval;
  ULP_PDU_t *pMsg;

  pMsg = (ULP_PDU_t *)MC_CALLOC(sizeof(ULP_PDU_t), 1);
  if (pMsg == NULL)
  {
    UBX_LOG(LCAT_ERROR, "allocation error");
  }

  rval = uper_decode_complete(0, &asn_DEF_ULP_PDU, (void **)&pMsg, pBuffer, (unsigned int)size);

  logSupl(pMsg, true);
  switch (rval.code)
  {
  case RC_OK:
    UBX_LOG(LCAT_VERBOSE, "succeed");
    break;
  case RC_FAIL:
    UBX_LOG(LCAT_WARNING, "Decoding failure... %zd", rval.consumed);
    ASN_STRUCT_FREE(asn_DEF_ULP_PDU, pMsg);
    pMsg = NULL;
    break;
  case RC_WMORE:
    UBX_LOG(LCAT_WARNING, "Want more? %zd", rval.consumed);
    ASN_STRUCT_FREE(asn_DEF_ULP_PDU, pMsg);
    pMsg = NULL;
    break;
  }

  return pMsg;
}

///////////////////////////////////////////////////////////////////////////////
//! Extract the session ID
/*! Function for extracting the session ID from a Supl message
  \param pMsg : pointer to the Supl message
  \return Extracted session ID
*/
int extractSid(const struct ULP_PDU *pMsg)
{
  /* Verify the prence of the session ID field */
  if (pMsg->sessionID.setSessionID == NULL)
  {
    UBX_LOG(LCAT_WARNING, "No session ID field... must be a SUPL INIT");
    if (pMsg->message.present != UlpMessage_PR_msSUPLINIT)
    {
      UBX_LOG(LCAT_ERROR, "session ID not present AND not SUPL INIT!!!!");
    }
    return 0;
  }

  /* returning the SET session ID, the one we consider for the session */
  return (pMsg->sessionID.setSessionID->sessionId);
}
