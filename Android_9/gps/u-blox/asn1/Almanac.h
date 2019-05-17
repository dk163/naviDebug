/*
 * Generated by asn1c-0.9.24 (http://lionet.info/asn1c)
 * From ASN.1 module "RRLP-Components"
 * 	found in "rrlp-components.asn"
 * 	`asn1c -gen-PER -fnative-types`
 */

#ifndef _Almanac_H_
#define _Almanac_H_

#include <asn_application.h>

/* Including external dependencies */
#include "SeqOfAlmanacElement.h"
#include <NativeInteger.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Almanac */
typedef struct Almanac
{
  long alamanacWNa;
  SeqOfAlmanacElement_t almanacList;

  /* Context for parsing across buffer boundaries */
  asn_struct_ctx_t _asn_ctx;
} Almanac_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_Almanac;

#ifdef __cplusplus
}
#endif

#endif /* _Almanac_H_ */
#include <asn_internal.h>
