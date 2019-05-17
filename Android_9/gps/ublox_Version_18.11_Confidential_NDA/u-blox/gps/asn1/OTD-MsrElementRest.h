/*
 * Generated by asn1c-0.9.24 (http://lionet.info/asn1c)
 * From ASN.1 module "RRLP-Components"
 * 	found in "rrlp-components.asn"
 * 	`asn1c -gen-PER -fnative-types`
 */

#ifndef _OTD_MsrElementRest_H_
#define _OTD_MsrElementRest_H_

#include <asn_application.h>

/* Including external dependencies */
#include "ModuloTimeSlot.h"
#include "StdResolution.h"
#include <NativeInteger.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct TOA_MeasurementsOfRef;
struct SeqOfOTD_MsrsOfOtherSets;

/* OTD-MsrElementRest */
typedef struct OTD_MsrElementRest
{
  long refFrameNumber;
  ModuloTimeSlot_t referenceTimeSlot;
  struct TOA_MeasurementsOfRef *toaMeasurementsOfRef /* OPTIONAL */;
  StdResolution_t stdResolution;
  long *taCorrection /* OPTIONAL */;
  struct SeqOfOTD_MsrsOfOtherSets *otd_MsrsOfOtherSets /* OPTIONAL */;

  /* Context for parsing across buffer boundaries */
  asn_struct_ctx_t _asn_ctx;
} OTD_MsrElementRest_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_OTD_MsrElementRest;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "SeqOfOTD-MsrsOfOtherSets.h"
#include "TOA-MeasurementsOfRef.h"

#endif /* _OTD_MsrElementRest_H_ */
#include <asn_internal.h>
