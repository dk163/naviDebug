/*
 * Generated by asn1c-0.9.24 (http://lionet.info/asn1c)
 * From ASN.1 module "SUPL-INIT"
 * 	found in "supl.asn"
 * 	`asn1c -gen-PER -fnative-types`
 */

#ifndef _SLPMode_H_
#define _SLPMode_H_

#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum SLPMode { SLPMode_proxy = 0, SLPMode_nonProxy = 1 } e_SLPMode;

/* SLPMode */
typedef long SLPMode_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_SLPMode;
asn_struct_free_f SLPMode_free;
asn_struct_print_f SLPMode_print;
asn_constr_check_f SLPMode_constraint;
ber_type_decoder_f SLPMode_decode_ber;
der_type_encoder_f SLPMode_encode_der;
xer_type_decoder_f SLPMode_decode_xer;
xer_type_encoder_f SLPMode_encode_xer;
per_type_decoder_f SLPMode_decode_uper;
per_type_encoder_f SLPMode_encode_uper;

#ifdef __cplusplus
}
#endif

#endif /* _SLPMode_H_ */
#include <asn_internal.h>
