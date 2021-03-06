/*
 * Generated by asn1c-0.9.24 (http://lionet.info/asn1c)
 * From ASN.1 module "RRLP-Components"
 * 	found in "rrlp-components.asn"
 * 	`asn1c -gen-PER -fnative-types`
 */

#ifndef _AssistBTSData_R98_ExpOTD_H_
#define _AssistBTSData_R98_ExpOTD_H_

#include <asn_application.h>

/* Including external dependencies */
#include "ExpOTDUncertainty.h"
#include "ExpectedOTD.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* AssistBTSData-R98-ExpOTD */
typedef struct AssistBTSData_R98_ExpOTD
{
  ExpectedOTD_t expectedOTD;
  ExpOTDUncertainty_t expOTDuncertainty;

  /* Context for parsing across buffer boundaries */
  asn_struct_ctx_t _asn_ctx;
} AssistBTSData_R98_ExpOTD_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_AssistBTSData_R98_ExpOTD;

#ifdef __cplusplus
}
#endif

#endif /* _AssistBTSData_R98_ExpOTD_H_ */
#include <asn_internal.h>
