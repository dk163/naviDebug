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
 * $Id: rrlpmanager.h 79666 2014-03-19 15:45:20Z marcus.picasso $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/supl/rrlpmanager.h $
 *****************************************************************************/

/*!
  \file
  \brief  RRLP proecssingSET state Machine administrator interface

  Managing and allocating of the state machines for SUPL SET
*/

#ifndef __RRLPMANAGER_H__
#define __RRLPMANAGER_H__

#include "ubx_localDb.h"

///////////////////////////////////////////////////////////////////////////////
// Types & Definitions

typedef enum
{
	RESP_NONE = 0,
	RESP_POSITION_DATA,
	RESP_MSA_DATA
} RrlpRespData_t;

//! structure passed as parameter to the function rrlpProcessing
typedef struct
{
    int responseTime;       		//!< requested responce time
    int referenceNumber;    		//!< RRLP session reference number to be given in the answer
    int reqAccuracy;        		//!< Requested horizontal accuracy (in cm)
	RrlpRespData_t responseType;	//!< Type of response needed
//	bool assistDataReceived;		//!< Flag if assistance data has been received
} aux_t;

///////////////////////////////////////////////////////////////////////////////
// Functions

char *rrlpProcessing(int sid, unsigned char *inBuffer, int inSize, int *pOutSize, aux_t *pAux);
char *buildRrlpPosResp(int *pOutSize, int referenceNumber, double lat, double lon, double alt, double tow);
char* buildRrlpPosRespTimeout(int *pOutSize, int referenceNumber);
char* buildRrlpMsaPosResp(int sid, int *pOutSize, int referenceNumber);

#endif /* __RRLPMANAGER_H__ */
