/*
 * Generated by asn1c-0.9.24 (http://lionet.info/asn1c)
 * From ASN.1 module "RRLP-Components"
 * 	found in "rrlp-components.asn"
 * 	`asn1c -gen-PER -fnative-types`
 */

#ifndef _SeqOfOTD_MsrElementRest_H_
#define _SeqOfOTD_MsrElementRest_H_

#include <asn_application.h>

/* Including external dependencies */
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct OTD_MsrElementRest;

/* SeqOfOTD-MsrElementRest */
typedef struct SeqOfOTD_MsrElementRest
{
  A_SEQUENCE_OF(struct OTD_MsrElementRest) list;

  /* Context for parsing across buffer boundaries */
  asn_struct_ctx_t _asn_ctx;
} SeqOfOTD_MsrElementRest_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_SeqOfOTD_MsrElementRest;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "OTD-MsrElementRest.h"

#endif /* _SeqOfOTD_MsrElementRest_H_ */
#include <asn_internal.h>
