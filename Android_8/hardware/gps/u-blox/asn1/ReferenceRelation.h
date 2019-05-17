/*
 * Generated by asn1c-0.9.24 (http://lionet.info/asn1c)
 * From ASN.1 module "RRLP-Components"
 * 	found in "rrlp-components.asn"
 * 	`asn1c -gen-PER -fnative-types`
 */

#ifndef	_ReferenceRelation_H_
#define	_ReferenceRelation_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum ReferenceRelation {
	ReferenceRelation_secondBTSThirdSet	= 0,
	ReferenceRelation_secondBTSSecondSet	= 1,
	ReferenceRelation_firstBTSFirstSet	= 2
} e_ReferenceRelation;

/* ReferenceRelation */
typedef long	 ReferenceRelation_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ReferenceRelation;
asn_struct_free_f ReferenceRelation_free;
asn_struct_print_f ReferenceRelation_print;
asn_constr_check_f ReferenceRelation_constraint;
ber_type_decoder_f ReferenceRelation_decode_ber;
der_type_encoder_f ReferenceRelation_encode_der;
xer_type_decoder_f ReferenceRelation_decode_xer;
xer_type_encoder_f ReferenceRelation_encode_xer;
per_type_decoder_f ReferenceRelation_decode_uper;
per_type_encoder_f ReferenceRelation_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _ReferenceRelation_H_ */
#include <asn_internal.h>
