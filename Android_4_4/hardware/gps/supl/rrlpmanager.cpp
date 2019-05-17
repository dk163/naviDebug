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
 * $Id: rrlpmanager.cpp 105520 2015-11-05 10:28:23Z fabio.robbiani $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/supl/rrlpmanager.cpp $
 *****************************************************************************/

/*!
  \file
  \brief  RRLP processing manager administrator

  Managing the RRLP protocol
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <time.h>
#include <malloc.h>

#include "nav_gps_msgmacros.h"
#include "std_types.h"
#include "std_lang_def.h"
#include "rrlpdecod.h"
#include "rrlpmanager.h"
#include "../agnss/helper/helperFunctions.h"

#include "ubxgpsstate.h"
#include "ubx_messageDef.h"
#include "ubx_log.h"

///////////////////////////////////////////////////////////////////////////////
// Types and Definitions

//#define TIME_REFERENCE_OFFSET_MILLS  1500		/*!< offset for assumption how old is the time reference: now it seems 1500 in average! */
#define TIME_REFERENCE_OFFSET_MILLS  0		/*!< offset for assumption how old is the time reference: now it seems 1500 in average! */
#define GPS_TIME_REFERENCE_TOLERANCE 2500      	//!< the tolerance in setting the time reference should be less then 3 seconds


//! Ephemeris collection structure
/*! This structure contains ALL data from the GPS message
    subframes 1,2 and 3 plus some additional flags used for data collection.

    \warning Don't change this struct, ephemeris hash calculation depends on it!
*/
typedef struct
{
	// subframes content
	I4	M0;					//!< SF2: Mean anomaly at reference time
	U4	e;					//!< SF2: eccentricity
	I4	Omega0;				//!< SF3: Long of asc node of orbit plane at weekly epoch
	I4	i0;					//!< SF3: Inclination angle at reference time
	I4	omega;				//!< SF3: Argument of perigee
	U4	sqrtA;				//!< SF2: Square root of the semi-major axis A
	U	wnoe 		: 10;	//!< SF1: Reference week number of ephemeris
	U	L2code		: 2;	//!< SF1: Code on L2
	U	URAi		: 4;	//!< SF1: URA index
	U	health		: 6;	//!< SF1: SV health
	U	IOD			: 8;	//!< SF1: IOD (8 LSB of IODC)
	U	IODCmsb		: 2;	//!< SF1: 2 MSB of IODC
	U	toc			: 16;	//!< SF1: Clock data reference time
	I	TGD			: 8;	//!< SF1: Group delay differential
	I	af2			: 8;	//!< SF1: Time polynomial coeficient 2
	I	Crs			: 16;	//!< SF2: Amp of cos harmonic corr term to orbit radius
	I	deltan		: 16;	//!< SF2: Mean motion difference from computed value
	I	Cuc			: 16;	//!< SF2: Amp of cosine harmonic corr term to arg of lat
	I	Cus			: 16;	//!< SF2: Amp of sine harmonic corr term to arg of lat
	I	Crc			: 16;	//!< SF3: Amp of cosine harmonic corr term to orbit radius
	U	toe			: 16;	//!< SF2: Reference time of ephemeris
	I	Cic			: 16;	//!< SF3: Amp of cos harmonic corr term to angle of inclination
	I	Cis			: 16;	//!< SF3: Amp of sine harmonic corr term to angle of inclination
	I	af1			: 16;	//!< SF1: Time polynomial coeficient 1
	I	IDOT		: 14;	//!< SF3: Rate of inclination angle	
	U	L2P			: 1;	//!< SF1: L2 P data flag
	U 	fit			: 1;	//!< SF2: Fit interval flag
	I	af0			: 22;	//!< SF1: Time polynomial coeficient 0
	U	svix		: 8;	//!<      SV index
	U	newtip		: 1;	//!<      mark as new for TIP
	U 	cno35		: 1; 	//!<      cno at time of reception was >35dbHz flag
	I	OmegaDot	: 24;	//!< SF3: Rate of right ascension
	// additional flags
	U	status		: 2;	//!<      Ephemeris status flag (FALSE/TRUE/CONFIRMED)
	U	sf1			: 2;	//!<      Subframe 1 status flag
	U	sf2			: 2;	//!<      Subframe 2 status flag
	U	sf3			: 2;	//!<      Subframe 3 status flag	
} NAV_GPS_EPH_COLLECT_t;

///////////////////////////////////////////////////////////////////////////////
// Local data

//! Strings for decoding various values to make logging more readable
static const char* _strRrlpMessageType[] = {
	"NOTHING",
	"msrPositionReq",
	"msrPositionRsp",
	"assistanceData",
	"assistanceDataAck",
	"protocolError",
};

static const char* _strPositionMethodType[] = {
	"eotd",
	"gps",
	"gpsOrEOTD"
};

static const char* _strMethodTypeType[] = {
	"NOTHING",
	"msAssisted",
	"msBased",
	"msBasedPref",
	"msAssistedPref"
};

///////////////////////////////////////////////////////////////////////////////
// Local Functions
static char *buildRrlpAssistAck(int *pOutSize, int referenceNumber);
static void convertNavModelForSat(int satellite, const UncompressedEphemeris_t *pemer, int tow, int wn);
static void manageAssistanceData(int sid, const ControlHeader_t *pCtl);
static void fillRefTime(GPS_UBX_AID_INI_U5__t *pHandl, U4 tow, U2 wn);
static void fillRefLoc(GPS_UBX_AID_INI_U5__t *pHandl, const unsigned char *pBuf, int size);

///////////////////////////////////////////////////////////////////////////////
//! RRLP processing
/*! Function for decoding and processing RRLP messages
    Processing can result in the generation of an RRLP message in response
  \param inBuffer     : payload buffer recieved containing RRLP message
  \param inSize       : size of the payload buffer
  \param pOutSize     : size of the response payload to be returned to the server
  \param pAux         : Auxiliary parameter OUT, for RRLP responce management
  \return             pointer to RRLP response message to be returned to the server
*/
char *rrlpProcessing(int sid, unsigned char *inBuffer, int inSize, int *pOutSize, aux_t *pAux)
{
    char *res = NULL;

    assert(pAux != NULL);	
	//UBX_LOG(LCAT_VERBOSE, "inBuffer %p, inSize %i", inBuffer, inSize);
    /* Set the out dimension = 0 by default */
    *pOutSize = 0;

	/* Set the reference number 0 by default... */
	pAux->referenceNumber = 0;
	pAux->responseTime = 0;
	/* the default accuracy level should be 100mt */
	pAux->reqAccuracy = 10000;

/* Do not notify if assist data not received any  more, need to answer anyhow! */
//	pAux->assistDataReceived = true;
    /* Decode the incoming RRLP message */
    PDU_t *rrlpMsg = rrlpDecode(inBuffer, inSize);
    if (rrlpMsg == NULL)
    {
        UBX_LOG(LCAT_VERBOSE, "Wrong packet returned");
        /* not a valid RRLP messge... */
        return NULL;
    }
	
    /* check if is assitance data supplied */
	UBX_LOG(LCAT_VERBOSE, "decoded RRLP message %i (%s)"
	        , rrlpMsg->component.present
	        , _LOOKUPSTR(rrlpMsg->component.present, RrlpMessageType));
    if (rrlpMsg->component.present == RRLP_Component_PR_assistanceData)
    {
		UBX_LOG(LCAT_VERBOSE, "Handling RRLP_Component_PR_assistanceData");
        if (rrlpMsg->component.choice.assistanceData.gps_AssistData != NULL)
        {
			// Assistance data received
			UBX_LOG(LCAT_VERBOSE, "Assistance data supplied");
//			pAux->assistDataReceived = true;
            manageAssistanceData(sid, &(rrlpMsg->component.choice.assistanceData.gps_AssistData->controlHeader));
        }
		else
		{
			UBX_LOG(LCAT_VERBOSE, "No GPS assistance data present");
		}

        // Build the RRLP confirmation - Caller will send back to Supl server
        res = buildRrlpAssistAck(pOutSize, rrlpMsg->referenceNumber);
        UBX_LOG(LCAT_INFO, "%d assistDataAck prepared", __LINE__);
    }
    /* Check if position request */
    else if (rrlpMsg->component.present == RRLP_Component_PR_msrPositionReq)
    {
		UBX_LOG(LCAT_VERBOSE, "Handling RRLP_Component_PR_msrPositionReq");

		// See if there is any gps assistance data supplied
		/* all the rest of the PositionInstruction informations are ignored... */
		if (rrlpMsg->component.choice.msrPositionReq.gps_AssistData != NULL)
		{
			/* GPS assitance data present */
			UBX_LOG(LCAT_VERBOSE, "Assistance data supplied");
//			pAux->assistDataReceived = true;
			manageAssistanceData(sid,  &(rrlpMsg->component.choice.msrPositionReq.gps_AssistData->controlHeader) );
		}
			
		UBX_LOG(LCAT_VERBOSE, "PositionInstruct: PosMethod (%s)  :  PosType (%s)",
			 _LOOKUPSTR(rrlpMsg->component.choice.msrPositionReq.positionInstruct.positionMethod, PositionMethodType),
			 _LOOKUPSTR(rrlpMsg->component.choice.msrPositionReq.positionInstruct.methodType.present, MethodTypeType));

        // Now check the positioning instruction */
        if ((rrlpMsg->component.choice.msrPositionReq.positionInstruct.methodType.present == MethodType_PR_msBased) ||
            (rrlpMsg->component.choice.msrPositionReq.positionInstruct.methodType.present == MethodType_PR_msBasedPref))
        {
			UBX_LOG(LCAT_VERBOSE, "Handle MethodType msBased or msBasedPref");
            /* now check the position method */

			/* Inform about the requested position! */
			int accuracy = rrlpMsg->component.choice.msrPositionReq.positionInstruct.methodType.choice.msBased;
			/* the method is allowed... */
			/* there is an union, so I can take whatever choice! */
			pAux->reqAccuracy = (int) (10 * (pow(1.1, (double) accuracy) - 1));
			UBX_LOG(LCAT_VERBOSE, "Request accuracy is: %dm", pAux->reqAccuracy);
			
			pAux->responseTime = 1 << rrlpMsg->component.choice.msrPositionReq.positionInstruct.measureResponseTime;
			pAux->referenceNumber = rrlpMsg->referenceNumber;
			pAux->responseType = RESP_POSITION_DATA;
			
			UBX_LOG(LCAT_VERBOSE, "Must reply with the position within %d seconds", pAux->responseTime);
        }
        else if ((rrlpMsg->component.choice.msrPositionReq.positionInstruct.methodType.present == MethodType_PR_msAssisted) ||
				 (rrlpMsg->component.choice.msrPositionReq.positionInstruct.methodType.present == MethodType_PR_msAssistedPref))
        {
            // MS-ASSIST data requested
            UBX_LOG(LCAT_VERBOSE, "Handle MethodType msAssisted or msAssistedPref");
			
			pAux->reqAccuracy = -1;
			pAux->responseTime = 1 << rrlpMsg->component.choice.msrPositionReq.positionInstruct.measureResponseTime;
			pAux->referenceNumber = rrlpMsg->referenceNumber;
			pAux->responseType = RESP_MSA_DATA;
			
			UBX_LOG(LCAT_VERBOSE, "MS-ASSIST responce time : %d(s) ref num : %d",
				 pAux->responseTime,
				 pAux->referenceNumber);
        }
        else
        {
            // Wrong method
			UBX_LOG(LCAT_VERBOSE, "Unknown/wrong positional request method (%i). Not supported",
				 rrlpMsg->component.choice.msrPositionReq.positionInstruct.methodType.present);
				
            // Calling function will tidy up and send protocol error because no response msg is returned
			// and the deferred response time (pAux->responseTime) is 0
        }
    }
    else
	{
		// Otherwise send a protocol error
		UBX_LOG(LCAT_VERBOSE, "Unknown/wrong RRLP method (%i). Not supported", rrlpMsg->component.present);

		// Calling function will tidy up and send protocol error because no response msg is returned
		// and the deferred response time (pAux->responseTime) is 0 		
    }

    ASN_STRUCT_FREE(asn_DEF_PDU, rrlpMsg);
    return res;
}

///////////////////////////////////////////////////////////////////////////////
//! Build an RRLP position response message due to a timeout
/*! Function for building an RRLP position response message due to a timeout
  \param pOutSize         : size of the encoded RRLP response message to be returned
  \param referenceNumber  : RRLP req reference number to use in the response message
  \return   Pointer to the encoded RRLP response message to be returned to the server
*/
char* buildRrlpPosRespTimeout(int *pOutSize, int referenceNumber)
{
    PDU_t msg;
    char *buf = NULL;

    /* clean the memory */
    memset(&msg, 0, sizeof(msg));

    msg.referenceNumber = referenceNumber;
    msg.component.present = RRLP_Component_PR_msrPositionRsp;
	
	*pOutSize = uper_encode_to_new_buffer(&asn_DEF_PDU, NULL, &msg, (void **) &buf);
	logRRLP(&msg, false);
    if (*pOutSize < 0)
    {
        UBX_LOG(LCAT_ERROR, "Encoding failure!!!");
    }
    ASN_STRUCT_FREE_CONTENTS_ONLY(asn_DEF_PDU, &msg);

    return buf;
}

///////////////////////////////////////////////////////////////////////////////
//! Build an RRLP position response message containing MSA data
/*! Function for building an RRLP position response message containing MSA data
  \param pOutSize         : size of the encoded RRLP response message to be returned
  \param referenceNumber  : RRLP req reference number to use in the response message
  \return   Pointer to the encoded RRLP response message to be returned to the server
*/
char* buildRrlpMsaPosResp(int sid, int *pOutSize, int referenceNumber)
{
    PDU_t msg;
    char *outBuf = NULL;
	int ret;

    /* clean the memory */
    memset(&msg,0,sizeof(msg));
	
    msg.referenceNumber = referenceNumber;
    msg.component.present = RRLP_Component_PR_msrPositionRsp;
	
	CDatabase* pDatabase = CAndroidDatabase::getInstance();
	I4 svCount = 0;
	bool r;
	
	r = pDatabase->GetOutput(CDatabase::DATA_UBX_SATELLITES_IN_MEAS_COUNT, svCount);
	assert(r == true);
	if (svCount > 0)
	{
		// Only add optional GPS_MeasureInfo as we have 1 or more satellites
		msg.component.choice.msrPositionRsp.gps_MeasureInfo = (GPS_MeasureInfo_t*) MC_CALLOC(sizeof(GPS_MeasureInfo_t), 1);
	
		I4 gnssTow = 0;
		r = pDatabase->GetOutput(CDatabase::DATA_UBX_GNSS_TOW, gnssTow);
		assert(r == true);

		I4 dopCenter = 0;
		r = pDatabase->GetOutput(CDatabase::DATA_UBX_GNSS_DOP_CENTER, dopCenter);
		assert(r == true);
		UBX_LOG(LCAT_VERBOSE, "dop Center %i (Hz) ", dopCenter);
		dopCenter *= 5;			// Scale to 0.2 resolution
		
		GPS_MsrSetElement* pMsrSetElement = (GPS_MsrSetElement*) MC_CALLOC(sizeof(GPS_MsrSetElement), 1);
        assert(pMsrSetElement);
		pMsrSetElement->refFrame = NULL;
		// FIXME: handle the case where we round the gnssTow!
		pMsrSetElement->gpsTOW = (GPSTOW24b_t)( gnssTow % 14400000); // reset every 4 hours
		UBX_LOG(LCAT_VERBOSE, "svCount=%i gnssTow=%i GPSTOW24b_t=%li", svCount, gnssTow, pMsrSetElement->gpsTOW);

	 	char buf[256];
		int iBuf = sprintf(buf, "%d, 1.0.0, %d, %d, %d", sid, svCount, 0, 0);

		int validSv = 0;
		for(int i = 0; i < svCount; i++)
		{
			I4 svid = 0;
			r = pDatabase->GetOutput(DATA_UBX_SATELLITES_IN_MEAS_(i), svid);
			assert(r == true);
			
			I4 cNo = 0;
			r = pDatabase->GetOutput(DATA_UBX_SATELLITES_IN_MEAS_CNO_(i), cNo);
			assert(r == true);
			
			iBuf += sprintf(&buf[iBuf], " ,%d,%d", svid, cNo);
		
			R8 prRms = 0;
			r = pDatabase->GetOutput(DATA_UBX_SATELLITES_IN_MEAS_PRRMS_(i), prRms);
			assert(r == true);
			
			U4 multipathIndicator = 0;
			r = pDatabase->GetOutput(DATA_UBX_SATELLITES_IN_MEAS_MULTIPATH_IND_(i), multipathIndicator);
			assert(r == true);
			
			I4 redSigtow = 0;
			r = pDatabase->GetOutput(DATA_UBX_SATELLITES_IN_MEAS_REL_CODEPHASE_(i), redSigtow);
			assert(r == true);
			
			I4 doppler = 0;
			pDatabase->GetOutput(DATA_UBX_SATELLITES_IN_MEAS_DOPPLER_(i), doppler);
			assert(r == true);

			I4 adjustedDoppler = (doppler / (0x1000 / 5)) - dopCenter;
			
			if ((adjustedDoppler >= -32768) && (adjustedDoppler <= 32767))
			{
				validSv++;
				GPS_MsrElement* pMsrElement = (GPS_MsrElement*) MC_CALLOC(sizeof(GPS_MsrElement), 1);

                assert(pMsrElement);

				pMsrElement->satelliteID = svid - 1; // SUPL 0..63 = 1..64 GPS SV id
				pMsrElement->cNo = cNo;
				pMsrElement->doppler = adjustedDoppler;

				// 1023 chips = 1 ms
                // We need to calculate the code phase of the pseudoranges
                R8 tmp2;
                if (redSigtow > 0)
                {
                    tmp2 = ((R8) redSigtow / (R8) 0x200000);               // Calculate the floating point version of the pseudorange codephase
                    tmp2 = tmp2 - floor(tmp2);                                  // Remove the integer part (it remains only the millisecond fraction)
                    tmp2 = 1 - tmp2;                                        // Here we have to consider the residual part of the millisecond.
                }
                else
                {
                    tmp2 = - ((R8) redSigtow / (R8) 0x200000);               // Calculate the floating point version of the pseudorange codephase
                    tmp2 = tmp2 - floor(tmp2);                                  // Remove the integer part (it remains only the millisecond fraction)
                }

                tmp2 *= 1023.0;                                             // Scale to number of chips
				pMsrElement->wholeChips =  (long) floor(tmp2);                 // whole number of chips
				pMsrElement->fracChips =  (long) ((tmp2-floor(tmp2)) * 1024.); // fraction of a chip, in 10 bits representation
                // UBX_LOG(LCAT_VERBOSE, "new phase %li %li (redSigtow=%d tmp=%lf)", pMsrElement->wholeChips, pMsrElement->fracChips, redSigtow, tmp2);

				pMsrElement->mpathIndic = MpathIndic_notMeasured; //multipathIndicator;
				pMsrElement->pseuRangeRMSErr = 0;
				
				UBX_LOG(LCAT_VERBOSE, "MSA Data - sv G%3ld : CNO %2ld : Doppler(-dopCenter) %7.1f : Chips %9.4f : MPI %ld : PRE %ld",
					 pMsrElement->satelliteID + 1,
					 pMsrElement->cNo,
					 (double) pMsrElement->doppler / 5.0,
					 (((double) pMsrElement->wholeChips * (double) 0x400) + (double) (pMsrElement->fracChips)) / (double)0x400,
					 pMsrElement->mpathIndic,
					 pMsrElement->pseuRangeRMSErr);
					
				ret = ASN_SEQUENCE_ADD(&pMsrSetElement->gps_msrList, pMsrElement);
				assert(ret == 0);
			}
			else
			{
				// Doppler outside acceptable range
			}
		}
		iBuf += sprintf(&buf[iBuf], " #measurement Info");
		logAgps.write(0x00000004, buf);

		
		if (validSv)
		{
			// Add list of valid satellite info
			ret = ASN_SEQUENCE_ADD(&msg.component.choice.msrPositionRsp.gps_MeasureInfo->gpsMsrSetList, pMsrSetElement);
			assert(ret == 0);
		}
		else
		{
			// Need to release the sv info manually and tidy up, as we won't be sending it to the server
			MC_FREE(msg.component.choice.msrPositionRsp.gps_MeasureInfo);
			msg.component.choice.msrPositionRsp.gps_MeasureInfo = NULL;
			MC_FREE(pMsrSetElement);
		}
	}
	*pOutSize = uper_encode_to_new_buffer(&asn_DEF_PDU, NULL, &msg, (void **) &outBuf);
	logRRLP(&msg, false);
    if (*pOutSize < 0)
    {
        UBX_LOG(LCAT_ERROR, "Encoding failure!!!");
    }
    ASN_STRUCT_FREE_CONTENTS_ONLY(asn_DEF_PDU, &msg);

    return outBuf;
}

///////////////////////////////////////////////////////////////////////////////
//! Build an RRLP position response message containing a position
/*! Function for building an RRLP position response message containing a position
  \param pOutSize         : size of the encoded RRLP response message to be returned
  \param referenceNumber  : RRLP req reference number to use in the response message
  \return   Pointer to the encoded RRLP response message to be returned to the server
*/
char *buildRrlpPosResp(int *pOutSize, int referenceNumber, double lat, double lon, double alt, double tow)
{
    PDU_t msg;
    char *buf = NULL;
    ((void)(tow));

    /* clean the memory */
    memset(&msg,0,sizeof(msg));

    msg.referenceNumber = referenceNumber;
    msg.component.present = RRLP_Component_PR_msrPositionRsp;

	msg.component.choice.msrPositionRsp.locationInfo = (LocationInfo_t*) MC_CALLOC(sizeof(LocationInfo_t), 1);

	/* TBD reference frame? */
	msg.component.choice.msrPositionRsp.locationInfo->refFrame = 0;
	
//	msg.component.choice.msrPositionRsp.locationInfo->refFrame = 0xFFFF;
//	msg.component.choice.msrPositionRsp.locationInfo->gpsTOW = (long *) MC_CALLOC(sizeof(long), 1);
//	*msg.component.choice.msrPositionRsp.locationInfo->gpsTOW = (long) tow;

	/* TBD Fix type? */
	msg.component.choice.msrPositionRsp.locationInfo->fixType = FixType_threeDFix;

	// Ellipsoid point
	msg.component.choice.msrPositionRsp.locationInfo->posEstimate.size = 14;
	msg.component.choice.msrPositionRsp.locationInfo->posEstimate.buf = (uint8_t*) MC_CALLOC(14, 1);
	// Type: ellipsoid point with altitude and uncertain ellipsoid = 0x90
	msg.component.choice.msrPositionRsp.locationInfo->posEstimate.buf[0] = 0x90;

	int32_t tmp = (int32_t) ((fabs(lat) / 90) * 0x800000);
	UBX_LOG(LCAT_VERBOSE, "Latitude %f (%i)", lat, tmp);
	msg.component.choice.msrPositionRsp.locationInfo->posEstimate.buf[3] = (tmp & 0x000000FF);
	msg.component.choice.msrPositionRsp.locationInfo->posEstimate.buf[2] = ( (tmp >> 8) & 0x000000FF);
	msg.component.choice.msrPositionRsp.locationInfo->posEstimate.buf[1] = ( (tmp >> 16) & 0x0000007F);
	if (lat < 0)
	{
		msg.component.choice.msrPositionRsp.locationInfo->posEstimate.buf[1] |= 0x80;	// -ve : Set sign bit
	}

	tmp = (int32_t) ((lon / 360) * 0x1000000);
	UBX_LOG(LCAT_VERBOSE, "Longitude %f (%i)", lon, tmp);
	msg.component.choice.msrPositionRsp.locationInfo->posEstimate.buf[6] = (tmp & 0x000000FF);
	msg.component.choice.msrPositionRsp.locationInfo->posEstimate.buf[5] = ( (tmp >> 8) & 0x000000FF);
	msg.component.choice.msrPositionRsp.locationInfo->posEstimate.buf[4] = ( (tmp >> 16) & 0x000000FF);

	tmp = (int32_t) fabs(alt);
	UBX_LOG(LCAT_VERBOSE, "Altitude %f (%i)", alt, tmp);
	msg.component.choice.msrPositionRsp.locationInfo->posEstimate.buf[8] = (tmp & 0x000000FF);
	msg.component.choice.msrPositionRsp.locationInfo->posEstimate.buf[7] = ( (tmp >> 8) & 0x0000007F);
	if (alt < 0)
	{
		msg.component.choice.msrPositionRsp.locationInfo->posEstimate.buf[7] |= 0x80;	// -ve : Set sign bit
	}
	msg.component.choice.msrPositionRsp.locationInfo->posEstimate.buf[9] = 0;
	msg.component.choice.msrPositionRsp.locationInfo->posEstimate.buf[10] = 0;
	msg.component.choice.msrPositionRsp.locationInfo->posEstimate.buf[11] = 0;
	msg.component.choice.msrPositionRsp.locationInfo->posEstimate.buf[12] = 0;
	msg.component.choice.msrPositionRsp.locationInfo->posEstimate.buf[13] = 0;



	*pOutSize = uper_encode_to_new_buffer(&asn_DEF_PDU, NULL, &msg, (void **) &buf);
	logRRLP(&msg, false);
    if (*pOutSize < 0)
    {
        UBX_LOG(LCAT_ERROR, "Encoding failure!!!");
    }
    ASN_STRUCT_FREE_CONTENTS_ONLY(asn_DEF_PDU, &msg);

    return buf;
}

///////////////////////////////////////////////////////////////////////////////
//! Build an RRLP assistance acknowledge response message
/*! Function for building an RRLP assistance acknowledge response message
  \param pOutSize         : size of the encoded RRLP response message to be returned
  \param referenceNumber  : RRLP req reference number to use in the response message
  \return   Pointer to the encoded RRLP response message to be returned to the server
*/
static char *buildRrlpAssistAck(int *pOutSize, int referenceNumber)
{
    PDU_t msg;
    char *buf = NULL;

    UBX_LOG(LCAT_INFO, "reference number = %d", referenceNumber);

    /* clean the memory */
    memset(&msg,0,sizeof(msg));

    msg.referenceNumber = referenceNumber;
    msg.component.present = RRLP_Component_PR_assistanceDataAck;
    msg.component.choice.assistanceDataAck = 1;

    *pOutSize = uper_encode_to_new_buffer(&asn_DEF_PDU, NULL, &msg, (void **) &buf);
	logRRLP(&msg, false);
    if (*pOutSize < 0)
    {
        UBX_LOG(LCAT_ERROR, "Encoding failure!!!");
    }
    ASN_STRUCT_FREE_CONTENTS_ONLY(asn_DEF_PDU, &msg);

    return buf;
}

///////////////////////////////////////////////////////////////////////////////
//! Strip the parity bits from the subframe data
/*! This will strip the 6 parity bits and place the subframe data to bits 0-23
	\param pData : Pointer to the subframe words
	\param words : Number of subframe words
*/
static void nav_orb_aidSfrStripParity(U4 *pData, U words)
{
	while (words--)
	{
		*(pData) = (*pData >> 6);
		pData++;
	}
}

///////////////////////////////////////////////////////////////////////////////
//! Pack ephemeris data into ephemeris sub-frame data format
/*! Pack ephemeris data into ephemeris sub-frame data format
	\param pkEph    : Pointer to ephemeris data conatined in a general structure
	\param pRawData : Pointer to sub frame data buffer
*/
static bool nav_gps_reconstructRawEph(const NAV_GPS_EPH_COLLECT_t *pkEph, U4 *pRawData)
{
	// get time of ephemeris
	I toe = (pkEph->toe << 4);
	
	// write HOW
	// as we don't have the time of reception,
	// we just use the time of ephemeris
	HOWTOW_R(pRawData, toe);
	GPSSFRSW_R(pRawData, 49, 3, 1);
	
	// fill in all the subframes 1 data
	GPSSFRSW_R(pRawData,  60,  10, pkEph->wnoe);
	GPSSFRSW_R(pRawData,  70,   2, pkEph->L2code);
	GPSSFRSW_R(pRawData,  72,   4, pkEph->URAi);
	GPSSFRSW_R(pRawData,  76,   6, pkEph->health);
	GPSSFRSW_R(pRawData, 210,   8, pkEph->IOD);
	GPSSFRSW_R(pRawData,  82,   2, pkEph->IODCmsb);
	GPSSFRSW_R(pRawData, 218,  16, pkEph->toc);
	GPSSFRSW_R(pRawData, 196,   8, (I4)pkEph->TGD);
	GPSSFRSW_R(pRawData,  90,   1, pkEph->L2P);
	GPSSFRSW_R(pRawData, 270,  22, (I4)pkEph->af0);
	GPSSFRSW_R(pRawData, 248,  16, (I4)pkEph->af1);
	GPSSFRSW_R(pRawData, 240,   8, (I4)pkEph->af2);
	
	// fill in all the subframes 2 data
	pRawData += 8;
	GPSSFRSW_R(pRawData,  60,   8, pkEph->IOD);
	GPSSFRDW_R(pRawData, 106,  32, (I4)pkEph->M0);
	GPSSFRDW_R(pRawData, 166,  32, pkEph->e);
	GPSSFRDW_R(pRawData, 226,  32, pkEph->sqrtA);
	GPSSFRSW_R(pRawData,  68,  16, (I4)pkEph->Crs);
	GPSSFRSW_R(pRawData,  90,  16, (I4)pkEph->deltan);
	GPSSFRSW_R(pRawData, 150,  16, (I4)pkEph->Cuc);
	GPSSFRSW_R(pRawData, 210,  16, (I4)pkEph->Cus);
	GPSSFRSW_R(pRawData, 270,  16, pkEph->toe);
	GPSSFRSW_R(pRawData, 286,   1, pkEph->fit);
	
	// fill in all the subframes 3 data
	pRawData += 8;
	GPSSFRDW_R(pRawData,  76,  32, (I4)pkEph->Omega0);
	GPSSFRDW_R(pRawData, 136,  32, (I4)pkEph->i0);
	GPSSFRDW_R(pRawData, 196,  32, (I4)pkEph->omega);
	GPSSFRSW_R(pRawData, 180,  16, (I4)pkEph->Crc);
	GPSSFRSW_R(pRawData,  60,  16, (I4)pkEph->Cic);
	GPSSFRSW_R(pRawData, 120,  16, (I4)pkEph->Cis);
	GPSSFRSW_R(pRawData, 270,   8, pkEph->IOD);
	GPSSFRSW_R(pRawData, 278,  14, (I4)pkEph->IDOT);
	GPSSFRSW_R(pRawData, 240,  24, (I4)pkEph->OmegaDot);

	return true;
}

///////////////////////////////////////////////////////////////////////////////
//! Convert and send Supl/RRLP ephemeris data to receiver
/*! Converts the nav model data (ephemeris) for a given satellite into the raw
    sub frame with parity removed format that u-blox receivers understand.
	Then transmits this data to the receiver.
	\param satellite : Satellite number Gxx (1..32)
	\param pEphemer  : Pointer to Supl/RRLP ephemeris data to send
	\parame tow      : time of week in seconds
	\param wn        : week number
*/
static void convertNavModelForSat(int satellite, const UncompressedEphemeris_t *pEphemer, int tow, int wn)
{
    NAV_GPS_EPH_COLLECT_t eph;

    /* clear the structure first of all */
    memset(&eph,0,sizeof(eph));
    // According to IS-GPS-200, a broadcast ephemeris may have a maximum fit interval of 98 hours, roughly +/- 2 days
	// Thus a valid ephemeris which is parametrized in the subframe format as defined by IS-GPS-200 cannot be older than 2 days.

	// Values to be filled into AID-EPH:
	// Make sure, wno_eph corresponds to HOW derived from TOE
    // ephemToe is scaled by 16
	int tmp = (tow - pEphemer->ephemToe * 16);

	UBX_LOG(LCAT_VERBOSE, "Verification of Week Rollover tow=%i toe=%li", tow, pEphemer->ephemToe);

	if (tmp > 302400)
		wn ++; // toe_eph is from next week
	else if ( tmp < -302400)
		wn --; // toe_eph is from previous week
	
    eph.wnoe     = wn & 0x3FF;

	// SF 1 data
    eph.L2code   = (U) pEphemer->ephemCodeOnL2;
    eph.URAi     = (U) pEphemer->ephemURA;
    eph.health   = (U) pEphemer->ephemSVhealth;
    eph.IODCmsb  = (pEphemer->ephemIODC >> 8);
    eph.IOD      = (pEphemer->ephemIODC & 0xFF);
    eph.L2P      = (U) pEphemer->ephemL2Pflag;
    eph.TGD      = pEphemer->ephemTgd;
    eph.toc      = (U) pEphemer->ephemToc;
    eph.af2      = pEphemer->ephemAF2;
    eph.af1      = pEphemer->ephemAF1;
    eph.af0      = pEphemer->ephemAF0;
	
	// SF 2 data
    eph.Crs      = pEphemer->ephemCrs;
    eph.deltan   = pEphemer->ephemDeltaN;
    eph.M0       = pEphemer->ephemM0;
    eph.Cuc      = pEphemer->ephemCuc;
    eph.e        = pEphemer->ephemE;
    eph.Cus      = pEphemer->ephemCus;
    eph.sqrtA    = pEphemer->ephemAPowerHalf;
    eph.toe      = (U) pEphemer->ephemToe;
    eph.fit      = (U) pEphemer->ephemFitFlag;
//    eph.    = pEphemer->ephemAODA; //Removed, not relevant

	// SF 3 data
    eph.Cic      = pEphemer->ephemCic;
    eph.Omega0   = pEphemer->ephemOmegaA0;
    eph.Cis      = pEphemer->ephemCis;
    eph.i0       = pEphemer->ephemI0;
    eph.Crc      = pEphemer->ephemCrc;
    eph.omega    = pEphemer->ephemW;
    eph.OmegaDot = pEphemer->ephemOmegaADot;
    eph.IDOT     = pEphemer->ephemIDot;

	// get the ephemeris data
	U4 rawData[26];
	memset(rawData, 0, sizeof(rawData));

    if (nav_gps_reconstructRawEph(&eph, rawData))
    {
        nav_orb_aidSfrStripParity(rawData, 26);
    }
	rawData[0] = (U) satellite;
	
	UBX_LOG(LCAT_VERBOSE, "Sending SUPL nav model data for sat G%i to receiver", satellite);
	CUbxGpsState* pUbxGps = CUbxGpsState::getInstance();
	pUbxGps->lock();
    pUbxGps->sendEph(rawData, sizeof(rawData));
	pUbxGps->unlock();
}

///////////////////////////////////////////////////////////////////////////////
//! Converts utc model for Ubx use
/*! Converts data in the Supl/RRLP Utc model to the equivalent Ubx message format
	\param pUbxUtcModel  : Pointer to Ubx message Utc model structure
	\param pSuplUtcModel : Pointer to Supl/RRLP Utc model structure
*/
static void fillUtcModel(GPS_UBX_AID_HUI_t* pUbxUtcModel, const UTCModel_t *pSuplUtcModel)
{
	pUbxUtcModel->utcA0 = pSuplUtcModel->utcA0;
	pUbxUtcModel->utcA1 = pSuplUtcModel->utcA1;
	pUbxUtcModel->utcTOW = pSuplUtcModel->utcTot;
	pUbxUtcModel->utcWNT = (I2) pSuplUtcModel->utcWNt;
	pUbxUtcModel->utcLS = (I2) pSuplUtcModel->utcDeltaTls;
	pUbxUtcModel->utcWNF = (I2) pSuplUtcModel->utcWNlsf;
	pUbxUtcModel->utcDN = (I2) pSuplUtcModel->utcDN;
	pUbxUtcModel->utcLSF = (I2) pSuplUtcModel->utcDeltaTlsf;

	pUbxUtcModel->flags |= GPS_UBX_AID_HUI_FLAGS_UTC_MASK;	
}

///////////////////////////////////////////////////////////////////////////////
//! Converts iono model for Ubx use
/*! Converts data in the Supl/RRLP Utc model to the equivalent Ubx message format
	\param pUbxUtcModel : Pointer to Ubx message iono model structure
	\param pIonoModel   : Pointer to Supl/RRLP iono model structure
*/
static void fillIonosphericModel(GPS_UBX_AID_HUI_t* pUbxUtcModel, const IonosphericModel_t* pIonoModel)
{
	pUbxUtcModel->klobA0 = pIonoModel->alfa0;
	pUbxUtcModel->klobA1 = pIonoModel->alfa1;
	pUbxUtcModel->klobA2 = pIonoModel->alfa2;
	pUbxUtcModel->klobA3 = pIonoModel->alfa3;
	pUbxUtcModel->klobB0 = pIonoModel->beta0;
	pUbxUtcModel->klobB1 = pIonoModel->beta1;
	pUbxUtcModel->klobB2 = pIonoModel->beta2;
	pUbxUtcModel->klobB3 = pIonoModel->beta3;

	pUbxUtcModel->flags |= GPS_UBX_AID_HUI_FLAGS_KLOB_MASK;
}

///////////////////////////////////////////////////////////////////////////////
//! Converts and transmits assistance data to receiver
/*! Converts the Supl/RRLP any assistance data to the appropriate Ubx format and
    transmits to the receiver
	\param pCtl : Pointer to the Supl/RRLP assistance data structure
*/
static void manageAssistanceData(int sid, const ControlHeader_t *pCtl)
{
    int wnoe = -1;
	int towe = -1;
    /* assist data handler for gps section */
    GPS_UBX_AID_INI_U5__t hdl;
	memset(&hdl, 0, sizeof(GPS_UBX_AID_INI_U5__t));
    bool aidingPresent = false;
	
	UBX_LOG(LCAT_VERBOSE, "Almanac assisted data %p", pCtl->almanac);
	UBX_LOG(LCAT_VERBOSE, "UTC model assisted data %p", pCtl->utcModel);
	UBX_LOG(LCAT_VERBOSE, "IONO model assisted data %p", pCtl->ionosphericModel);
	UBX_LOG(LCAT_VERBOSE, "DGPS correction assisted data %p", pCtl->dgpsCorrections);
	UBX_LOG(LCAT_VERBOSE, "Reference location assisted data %p", pCtl->refLocation);
    UBX_LOG(LCAT_VERBOSE, "Reference time assisted data %p", pCtl->referenceTime);
	UBX_LOG(LCAT_VERBOSE, "Acquis assisted data %p", pCtl->acquisAssist);
	UBX_LOG(LCAT_VERBOSE, "Real time integrity assisted data %p", pCtl->realTimeIntegrity);	
	UBX_LOG(LCAT_VERBOSE, "Nav model data %p", pCtl->navigationModel);
	
	
    /* Reference time assisted data is present ! */
    if (pCtl->referenceTime != NULL)
    {
        /* Tore the Week Number for ephemeris usage */
        wnoe = pCtl->referenceTime->gpsTime.gpsWeek;
		towe = pCtl->referenceTime->gpsTime.gpsTOW23b;

        /* Fill the data with the information coming from the RRLP message */
        fillRefTime(&hdl, towe, wnoe);
		aidingPresent = true;
    }

    /* Reference location assisted data is present ! */
    if (pCtl->refLocation != NULL)
    {
        /* Fill the data with the information coming from the RRLP message */
        fillRefLoc(&hdl,
                   pCtl->refLocation->threeDLocation.buf,
                   pCtl->refLocation->threeDLocation.size);
		aidingPresent = true;
    }

	if (aidingPresent)
	{
		// sending aiding data
		UBX_LOG(LCAT_VERBOSE, "Sending aiding data, received from SUPL server, to receiver");
		CUbxGpsState* pUbxGps = CUbxGpsState::getInstance();
		pUbxGps->lock();
		pUbxGps->sendAidingData(&hdl);
		pUbxGps->unlock();
	}
	
	UBX_LOG(LCAT_VERBOSE, "wnoe %i", wnoe);
    if ( (pCtl->navigationModel != NULL) && (wnoe != -1) && (towe != -1))
    {
		/* Navigation model assisted data is present ! (and the time reference has been received) */
        int i;
        UBX_LOG(LCAT_VERBOSE, "Navigation model assisted data is present for %d satellites",
             pCtl->navigationModel->navModelList.list.count);

	 	char buf[256];
		int iBuf = sprintf(buf, "%d, 1.0.0, %d, %d, %d", sid, pCtl->navigationModel->navModelList.list.count, towe, wnoe);
			
        /* browse all the satellites */
        for (i = 0; i < pCtl->navigationModel->navModelList.list.count; i++)
        {
            NavModelElement_t *pElement = pCtl->navigationModel->navModelList.list.array[i];
            if (pElement == NULL)
            {
                UBX_LOG(LCAT_WARNING, "Navigation Model Element missing!");
            }
            else
            {
				iBuf += sprintf(&buf[iBuf], ", %d", (int)pElement->satelliteID);
		        UBX_LOG(LCAT_VERBOSE, "Navigation Model found for satellite G%ld", pElement->satelliteID + 1);
                if (pElement->satStatus.present == SatStatus_PR_newSatelliteAndModelUC)
                {
                    UBX_LOG(LCAT_VERBOSE, "ephemeris for the new satellite and model");
                    /* the satellite number is offset +1 */
                    convertNavModelForSat(pElement->satelliteID + 1,
                                          &pElement->satStatus.choice.newSatelliteAndModelUC,
                                          (towe * 80) / 1000,
                                          wnoe);
                }
                else if (pElement->satStatus.present == SatStatus_PR_newNaviModelUC)
                {
                    UBX_LOG(LCAT_VERBOSE, "ephemeris for the new navi model");
                    /* the satellite number is offset +1 */
                    convertNavModelForSat(pElement->satelliteID + 1,
                                          &pElement->satStatus.choice.newNaviModelUC,
                                          (towe * 80) / 1000,
                                          wnoe);
                }
                else
                {
                    UBX_LOG(LCAT_WARNING, "there are no ephemeris to be sent");
                }
            }
        }
		iBuf += sprintf(&buf[iBuf], " # ephemeris info");
		logAgps.write(0x00000004, buf);
    }
	
	bool utcModelPresent = false;
	GPS_UBX_AID_HUI_t ubxUtcModel;
	memset(&ubxUtcModel, 0, sizeof(GPS_UBX_AID_HUI_t));
	
	if (pCtl->utcModel != NULL)
	{
		// Use Utc model
		fillUtcModel(&ubxUtcModel, pCtl->utcModel);
		utcModelPresent = true;
	}
	
	if (pCtl->ionosphericModel)
	{
		fillIonosphericModel(&ubxUtcModel, pCtl->ionosphericModel);
		utcModelPresent = true;
	}
	
	if (utcModelPresent)
	{
		CUbxGpsState* pUbxGps = CUbxGpsState::getInstance();
		pUbxGps->lock();
		pUbxGps->sendUtcModel(&ubxUtcModel);
		pUbxGps->unlock();
	}
}

///////////////////////////////////////////////////////////////////////////////
//! Converts latitude to Ubx format
/*! Converts a Supl/RRLP latitude value to its equivalent in Ubx format
	\param orig : Supl/RRLp latitude to convert
	\return The converted latitude value
*/
static int convertLatitude (int orig)
{
    int temp;

    if(orig == 0x7FFFFF)
    {
        /* Noth/south pole */
        temp = 90 * 10000000;
    }
    else
    {
        temp = (int) (( (long long) orig * 90 * 10000000) / (1<<23));
    }

    return temp;
}

///////////////////////////////////////////////////////////////////////////////
//! Converts longitude to Ubx format
/*! Converts a Supl/RRLP longitude value to its equivalent in Ubx format
	\param orig : Supl/RRLp longitude to convert
	\return The converted longitude value
*/
static int convertLongitude (int orig)
{
    int temp;

    temp = (int) (( (long long) orig * 360 * 10000000) / (1<<24));

    return temp;
}

///////////////////////////////////////////////////////////////////////////////
//! Converts current reference time for Ubx use
/*! Converts a current reference time for use in Ubx message format
    \warning If roll-over information is not available in the provided wn,
    this function will start to create wrong results approximatly 20 years after
    the earliest sane time \ref SRC_CREAT_TIME_SEC
	\param pHandl : Pointer to Ubx message structure containing reference time
	\param tow    : Time of week
	\param wn     : Week number
*/
static void fillRefTime(GPS_UBX_AID_INI_U5__t *pHandl, U4 tow, U2 wn)
{

    /* Convert to ms (RRLP uses a 80ms resolution)
     * and adding a reference offset */
    U4 tow_ms = tow * 80 + TIME_REFERENCE_OFFSET_MILLS;

    GPS_TIME_t t;
    t.ro          = 0; // This information is not available.
    t.wno         = wn;
    t.tow.tv_sec  = tow_ms / MSEC_PER_SEC;
    t.tow.tv_nsec = (tow_ms % MSEC_PER_SEC) * NSEC_PER_MSEC;

    // Get absolute GPS Time (relative to GPS Epoch)
    relToAbsGpsTime(&t);

    pHandl->wn  = t.ro * GPS_WEEKS_BEF_RO + t.wno;
    pHandl->tow = t.tow.tv_sec  * MSEC_PER_SEC
                + t.tow.tv_nsec / NSEC_PER_MSEC;
}

///////////////////////////////////////////////////////////////////////////////
//! Converts reference location for Ubx use
/*! Converts a RRLP reference location for use in Ubx message format
	\param pHandl : Pointer to Ubx message structure containing reference location
	\param pBuf : Pointer to buffer containing encoded reference location
	\param size : Size of buffer
*/
static void fillRefLoc(GPS_UBX_AID_INI_U5__t *pHandl, const unsigned char *pBuf, int size)
{
    int writeOk = 1;
    int type = (pBuf[0] >> 4);
    /* Get the pointer to the handler */

	UBX_LOG(LCAT_VERBOSE, "type %x", type);
    /* check the contrains derived from size and code */
    if ( ( (type == 0x00) && (size !=  7) ) ||
         ( (type == 0x01) && (size !=  8) ) ||
         ( (type == 0x03) && (size != 11) ) ||
         ( (type == 0x08) && (size !=  9) ) ||
         ( (type == 0x09) && (size != 14) ) )
    {
        /* size checks */
        writeOk = 0;
        UBX_LOG(LCAT_VERBOSE, "Error in receiving size for refloc %d (size %d)", type, size);
    }

    switch (type)
    {
    case 0x00:
        /* ellipsoid point */
        /* set the flag */
        pHandl->flags |= ( GPS_UBX_AID_INI_U5__FLAGS_POS_MASK |
                           GPS_UBX_AID_INI_U5__FLAGS_LLA_MASK |
                           GPS_UBX_AID_INI_U5__FLAGS_ALTINV_MASK);
        break;
    case 0x01:
        /* ellipsoid point with uncertain circle */
        /* put the uncertainty value in cm, with a complex calculation... */
        pHandl->posAcc = 1000 * (unsigned int) ( pow( 1.1 , (double) ( pBuf[7] & 0x7F ) ) - 1 );
        UBX_LOG(LCAT_VERBOSE, "Accuracy is %d", pHandl->posAcc);

        /* set the flag */
        pHandl->flags |= ( GPS_UBX_AID_INI_U5__FLAGS_POS_MASK |
                           GPS_UBX_AID_INI_U5__FLAGS_LLA_MASK |
                           GPS_UBX_AID_INI_U5__FLAGS_ALTINV_MASK);
        break;
    case 0x03:
        /* ellipsoid point with uncertain ellipse */
        /* I consider the semi major as if it was a circle */
        /* put the uncertainty value in cm, with a complex calculation... */
        pHandl->posAcc = 1000 * (unsigned int) ( pow( 1.1 , (double) ( pBuf[7] & 0x7F ) ) - 1 );
        UBX_LOG(LCAT_VERBOSE, "Accuracy is %d", pHandl->posAcc);

        /* set the flag */
        pHandl->flags |= ( GPS_UBX_AID_INI_U5__FLAGS_POS_MASK |
                           GPS_UBX_AID_INI_U5__FLAGS_LLA_MASK |
                           GPS_UBX_AID_INI_U5__FLAGS_ALTINV_MASK);
        break;
    case 0x08:
        /* ellipsoid point with altitude */
        /* read the altitude */
        pHandl->ecefZOrAlt = ( ( (pBuf[7] & 0x7F) << 8) + ( pBuf[8] << 8 ));
        /* Set the sign of the altitude */
        pHandl->ecefZOrAlt = (pBuf[7] & 0x80) ? -pHandl->ecefZOrAlt : pHandl->ecefZOrAlt;

        /* set the flag */
        pHandl->flags |= ( GPS_UBX_AID_INI_U5__FLAGS_POS_MASK |
                           GPS_UBX_AID_INI_U5__FLAGS_LLA_MASK |
                           GPS_UBX_AID_INI_U5__FLAGS_ALTINV_MASK);
        break;
    case 0x09:
        /* ellipsoid point with altitude and uncertainty ellipsoid */
        /* read the altitude */
        pHandl->ecefZOrAlt = ( ( (pBuf[7] & 0x7F) << 8) + pBuf[8]);
        /* Set the sign of the altitude */
        pHandl->ecefZOrAlt = (pBuf[7] & 0x80) ? -pHandl->ecefZOrAlt : pHandl->ecefZOrAlt;
        UBX_LOG(LCAT_VERBOSE, "Altitude is %d", pHandl->ecefZOrAlt);

        /* get the accuracy */
        /* I consider the semi major as if it was a circle */
        /* put the uncertainty value in cm, with a complex calculation... */
        pHandl->posAcc = 1000 * (unsigned int) ( pow( 1.1 , (double) ( pBuf[9] & 0x7F ) ) - 1 );
        /* the accuracy in altitude is not considered at all!!! */
        UBX_LOG(LCAT_VERBOSE, "Accuracy is %d", pHandl->posAcc);

        /* set the flag */
        pHandl->flags |= ( GPS_UBX_AID_INI_U5__FLAGS_POS_MASK |
                           GPS_UBX_AID_INI_U5__FLAGS_LLA_MASK |
                           GPS_UBX_AID_INI_U5__FLAGS_ALTINV_MASK);
        break;
    default:
        writeOk = 0;
        UBX_LOG(LCAT_VERBOSE, "Type not recognized... %d", type);
        break;
    }

    if (writeOk)
    {
        /* the first two parameters are always the same! */
        int latitude,longitude;

        latitude = convertLatitude( ( (pBuf[1] & 0x7F) << 16) +
                                    ( pBuf[2] << 8 ) +
                                    pBuf[3]);

        latitude = (pBuf[1] & 0x80) ? -latitude : latitude;

        longitude = convertLongitude( ( pBuf[4] << 16 ) +
                                      ( pBuf[5] << 8  ) +
                                      pBuf[6]);

        UBX_LOG(LCAT_VERBOSE, "The reference position is: lat = %d, long = %d", latitude, longitude);

        pHandl->ecefXOrLat = latitude;
        pHandl->ecefYOrLon = longitude;
    }
}
