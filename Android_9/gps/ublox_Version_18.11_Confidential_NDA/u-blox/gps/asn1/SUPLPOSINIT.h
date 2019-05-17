/*
 * Generated by asn1c-0.9.24 (http://lionet.info/asn1c)
 * From ASN.1 module "SUPL-POS-INIT"
 * 	found in "supl.asn"
 * 	`asn1c -gen-PER -fnative-types`
 */

#ifndef _SUPLPOSINIT_H_
#define _SUPLPOSINIT_H_

#include <asn_application.h>

/* Including external dependencies */
#include "LocationId.h"
#include "SETCapabilities.h"
#include "Ver.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct RequestedAssistData;
struct Position;
struct SUPLPOS;

/* SUPLPOSINIT */
typedef struct SUPLPOSINIT
{
  SETCapabilities_t sETCapabilities;
  struct RequestedAssistData *requestedAssistData /* OPTIONAL */;
  LocationId_t locationId;
  struct Position *position /* OPTIONAL */;
  struct SUPLPOS *sUPLPOS /* OPTIONAL */;
  Ver_t *ver /* OPTIONAL */;
  /*
   * This type is extensible,
   * possible extensions are below.
   */

  /* Context for parsing across buffer boundaries */
  asn_struct_ctx_t _asn_ctx;
} SUPLPOSINIT_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_SUPLPOSINIT;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "Position.h"
#include "RequestedAssistData.h"
#include "SUPLPOS.h"

#endif /* _SUPLPOSINIT_H_ */
#include <asn_internal.h>
