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
  \brief  SET state Machine administrator interface

  Managing and allocating of the state machines for SUPL SET
*/

#pragma once

#include "rrlpmanager.h"
#include <unistd.h>

///////////////////////////////////////////////////////////////////////////////
// Types & Definitions

//! Commands to Supl state machine
typedef enum {
  SUPL_NO_CMD,           //!< No command present. Use message input data to determine
                         //!action
  SUPL_TIMEOUT,          //!< Timeout has occurred
  SUPL_POS_AVAIL,        //! The GPS position is available with the desired QOP
  SUPL_ASK_FOR_AGPS,     //!< UI asking for ASGP using SUPL server: SET Init
  SUPL_AUTH_GRANT,       //!< UI granting authorization for a NI session
  SUPL_AUTH_DENIED,      //!< UI denying authorization for a NI session
  SUPL_MSA_DATA_AVAIL,   //!< MS-ASSIST data available
  SUPL_NETWORK_CONNECTED //!< Network is now available
} SuplCmd_t;

///////////////////////////////////////////////////////////////////////////////
// Functions
void suplRegisterEventCallbacks(GpsControlEventInterface *pEventInterface, void *pContext);
int suplAddUplListeners(fd_set *pRfds, int *pMaxFd);
int suplReadUplSock(fd_set *pRfds);
bool suplStartSetInitiatedAction(void);
void suplHandleNetworkInitiatedAction(const char *buffer, int size);
void suplCheckPendingActions(void);
void suplCheckForSiAction(void);
void suplHandleAuthorization(int sid, int cmd);
bool suplActiveSessions(void);
void suplInitSsl(void);
int suplCountSessions(bool ni);
