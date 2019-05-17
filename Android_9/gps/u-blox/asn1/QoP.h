/*
 * Generated by asn1c-0.9.24 (http://lionet.info/asn1c)
 * From ASN.1 module "ULP-Components"
 * 	found in "supl.asn"
 * 	`asn1c -gen-PER -fnative-types`
 */

#ifndef _QoP_H_
#define _QoP_H_

#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* QoP */
typedef struct QoP
{
  long horacc;
  long *veracc /* OPTIONAL */;
  long *maxLocAge /* OPTIONAL */;
  long *delay /* OPTIONAL */;
  /*
   * This type is extensible,
   * possible extensions are below.
   */

  /* Context for parsing across buffer boundaries */
  asn_struct_ctx_t _asn_ctx;
} QoP_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_QoP;

#ifdef __cplusplus
}
#endif

#endif /* _QoP_H_ */
#include <asn_internal.h>
