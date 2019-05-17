/*
 * Generated by asn1c-0.9.24 (http://lionet.info/asn1c)
 * From ASN.1 module "RRLP-Components"
 * 	found in "rrlp-components.asn"
 * 	`asn1c -gen-PER -fnative-types`
 */

#ifndef _Rel_5_MsrPosition_Rsp_Extension_H_
#define _Rel_5_MsrPosition_Rsp_Extension_H_

#include <asn_application.h>

/* Including external dependencies */
#include "UlPseudoSegInd.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct Extended_reference;
struct SeqOfOTD_MsrElementRest;

/* Rel-5-MsrPosition-Rsp-Extension */
typedef struct Rel_5_MsrPosition_Rsp_Extension
{
  struct Extended_reference *extended_reference /* OPTIONAL */;
  struct SeqOfOTD_MsrElementRest *otd_MeasureInfo_5_Ext /* OPTIONAL */;
  UlPseudoSegInd_t *ulPseudoSegInd /* OPTIONAL */;
  /*
   * This type is extensible,
   * possible extensions are below.
   */

  /* Context for parsing across buffer boundaries */
  asn_struct_ctx_t _asn_ctx;
} Rel_5_MsrPosition_Rsp_Extension_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_Rel_5_MsrPosition_Rsp_Extension;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "Extended-reference.h"
#include "OTD-MeasureInfo-5-Ext.h"

#endif /* _Rel_5_MsrPosition_Rsp_Extension_H_ */
#include <asn_internal.h>
