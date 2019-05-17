/*
 * Generated by asn1c-0.9.24 (http://lionet.info/asn1c)
 * From ASN.1 module "RRLP-Components"
 * 	found in "rrlp-components.asn"
 * 	`asn1c -gen-PER -fnative-types`
 */

#ifndef _SeqOfGPS_MsrSetElement_H_
#define _SeqOfGPS_MsrSetElement_H_

#include <asn_application.h>

/* Including external dependencies */
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct GPS_MsrSetElement;

/* SeqOfGPS-MsrSetElement */
typedef struct SeqOfGPS_MsrSetElement
{
  A_SEQUENCE_OF(struct GPS_MsrSetElement) list;

  /* Context for parsing across buffer boundaries */
  asn_struct_ctx_t _asn_ctx;
} SeqOfGPS_MsrSetElement_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_SeqOfGPS_MsrSetElement;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "GPS-MsrSetElement.h"

#endif /* _SeqOfGPS_MsrSetElement_H_ */
#include <asn_internal.h>