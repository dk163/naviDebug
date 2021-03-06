/*
 * Generated by asn1c-0.9.24 (http://lionet.info/asn1c)
 * From ASN.1 module "RRLP-Components"
 * 	found in "rrlp-components.asn"
 * 	`asn1c -gen-PER -fnative-types`
 */

#ifndef _AcquisElement_H_
#define _AcquisElement_H_

#include <asn_application.h>

/* Including external dependencies */
#include "SatelliteID.h"
#include <NativeInteger.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct AddionalDopplerFields;
struct AddionalAngleFields;

/* AcquisElement */
typedef struct AcquisElement
{
  SatelliteID_t svid;
  long doppler0;
  struct AddionalDopplerFields *addionalDoppler /* OPTIONAL */;
  long codePhase;
  long intCodePhase;
  long gpsBitNumber;
  long codePhaseSearchWindow;
  struct AddionalAngleFields *addionalAngle /* OPTIONAL */;

  /* Context for parsing across buffer boundaries */
  asn_struct_ctx_t _asn_ctx;
} AcquisElement_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_AcquisElement;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "AddionalAngleFields.h"
#include "AddionalDopplerFields.h"

#endif /* _AcquisElement_H_ */
#include <asn_internal.h>
