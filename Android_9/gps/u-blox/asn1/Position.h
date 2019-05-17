/*
 * Generated by asn1c-0.9.24 (http://lionet.info/asn1c)
 * From ASN.1 module "ULP-Components"
 * 	found in "supl.asn"
 * 	`asn1c -gen-PER -fnative-types`
 */

#ifndef _Position_H_
#define _Position_H_

#include <asn_application.h>

/* Including external dependencies */
#include "PositionEstimate.h"
#include <UTCTime.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct Velocity;

/* Position */
typedef struct Position
{
  UTCTime_t timestamp;
  PositionEstimate_t positionEstimate;
  struct Velocity *velocity /* OPTIONAL */;
  /*
   * This type is extensible,
   * possible extensions are below.
   */

  /* Context for parsing across buffer boundaries */
  asn_struct_ctx_t _asn_ctx;
} Position_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_Position;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "Velocity.h"

#endif /* _Position_H_ */
#include <asn_internal.h>