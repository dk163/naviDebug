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
 * $Id: uplsend.h 108055 2015-12-11 14:22:01Z marcus.picasso $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/supl/uplsend.h $
 *****************************************************************************/

/*!
  \file
  \brief  ULP message manager interface

  Interface description of the module for managing SUPL messages sent
  from the SET to the SPL
*/

#ifndef __ULPSEND_H__
#define __ULPSEND_H__

#include "ubx_localDb.h"
#include "ubx_rilIf.h"
#include "supl_connection.h"

///////////////////////////////////////////////////////////////////////////////
// Types & Definitions

//! structure passed as parameter to the function SuplEnd
typedef struct suplEndParam
{
    SlpSessionID_t *pSlpId; //!< SLP session Id
    SetSessionID_t *pSetId; //!< SLP session Id
    StatusCode_t status;    //!< status
    int posEn;              //!< if 1, retrieve position
    int verEn;              //!< if 1, retrieve hash
    long long hash;         //!< hash code when requested
} suplEndParam_t;

//! structure passed as parameter to the function SuplPos
typedef struct suplPosParam
{
    SlpSessionID_t *pSlpId; //!< SLP session Id
    SetSessionID_t *pSetId; //!< SLP session Id
    char *buffer;           //!< buffer containing the RRLP payload
    int size;               //!< size of the payload
} suplPosParam_t;

//! structure passed as parameter to the function SuplPosInit
typedef struct suplPosInitParam
{
    SlpSessionID_t *pSlpId; //!< SLP session Id
    SetSessionID_t *pSetId; //!< SLP session Id
    int posEn;              		//!< if 1, send position to server
    int assEn;              		//!< if 1, ask for assisted data from server
    int verEn;              		//!< if 1, send hash to server
    long long hash;         		//!< hash code when requested
	PosMethod_t requestedPosMethod;	//!< Type of position method to request
	double lat;						//!< lat to send to server
	double lon;						//!< lon to send to server
} suplPosInitParam_t;

///////////////////////////////////////////////////////////////////////////////
// Functions
int sendSuplStart(SUPL_CONNECTION *supl_connection, SetSessionID_t *pSetId);
int sendSuplEnd(SUPL_CONNECTION *supl_connection, const suplEndParam_t *pParam);
int sendSuplPos(SUPL_CONNECTION *supl_connection, const suplPosParam_t *pParam);
int sendSuplPosInit(SUPL_CONNECTION *supl_connection, const suplPosInitParam_t *pParam);
SlpSessionID_t *copySlpId(const SlpSessionID_t *pOrig);
SetSessionID_t *copySetId(const SetSessionID_t *pOrig);
SetSessionID_t *fillDefaultSetId(int sessionId);

#endif /* __ULPSEND_H__ */

// uplsend.h
