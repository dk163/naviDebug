/*
 * Generated by asn1c-0.9.24 (http://lionet.info/asn1c)
 * From ASN.1 module "RRLP-Components"
 * 	found in "rrlp-components.asn"
 * 	`asn1c -gen-PER -fnative-types`
 */

#ifndef _Rel98_Ext_ExpOTD_H_
#define _Rel98_Ext_ExpOTD_H_

#include <asn_application.h>

/* Including external dependencies */
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct MsrAssistData_R98_ExpOTD;
struct SystemInfoAssistData_R98_ExpOTD;

/* Rel98-Ext-ExpOTD */
typedef struct Rel98_Ext_ExpOTD
{
  struct MsrAssistData_R98_ExpOTD *msrAssistData_R98_ExpOTD /* OPTIONAL */;
  struct SystemInfoAssistData_R98_ExpOTD *systemInfoAssistData_R98_ExpOTD /* OPTIONAL */;

  /* Context for parsing across buffer boundaries */
  asn_struct_ctx_t _asn_ctx;
} Rel98_Ext_ExpOTD_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_Rel98_Ext_ExpOTD;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "MsrAssistData-R98-ExpOTD.h"
#include "SystemInfoAssistData-R98-ExpOTD.h"

#endif /* _Rel98_Ext_ExpOTD_H_ */
#include <asn_internal.h>
