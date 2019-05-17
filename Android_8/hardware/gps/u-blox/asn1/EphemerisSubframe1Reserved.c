/*
 * Generated by asn1c-0.9.24 (http://lionet.info/asn1c)
 * From ASN.1 module "RRLP-Components"
 * 	found in "rrlp-components.asn"
 * 	`asn1c -gen-PER -fnative-types`
 */

#include "EphemerisSubframe1Reserved.h"

static int
memb_reserved1_constraint_1(asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		_ASN_CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= 0 && value <= 8388607)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		_ASN_CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static int
memb_reserved2_constraint_1(asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		_ASN_CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= 0 && value <= 16777215)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		_ASN_CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static int
memb_reserved3_constraint_1(asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		_ASN_CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= 0 && value <= 16777215)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		_ASN_CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static int
memb_reserved4_constraint_1(asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		_ASN_CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= 0 && value <= 65535)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		_ASN_CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static asn_per_constraints_t asn_PER_memb_reserved1_constr_2 GCC_NOTUSED = {
	{ APC_CONSTRAINED,	 23, -1,  0,  8388607 }	/* (0..8388607) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_per_constraints_t asn_PER_memb_reserved2_constr_3 GCC_NOTUSED = {
	{ APC_CONSTRAINED,	 24, -1,  0,  16777215 }	/* (0..16777215) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_per_constraints_t asn_PER_memb_reserved3_constr_4 GCC_NOTUSED = {
	{ APC_CONSTRAINED,	 24, -1,  0,  16777215 }	/* (0..16777215) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_per_constraints_t asn_PER_memb_reserved4_constr_5 GCC_NOTUSED = {
	{ APC_CONSTRAINED,	 16,  16,  0,  65535 }	/* (0..65535) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_TYPE_member_t asn_MBR_EphemerisSubframe1Reserved_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct EphemerisSubframe1Reserved, reserved1),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		memb_reserved1_constraint_1,
		&asn_PER_memb_reserved1_constr_2,
		0,
		"reserved1"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct EphemerisSubframe1Reserved, reserved2),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		memb_reserved2_constraint_1,
		&asn_PER_memb_reserved2_constr_3,
		0,
		"reserved2"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct EphemerisSubframe1Reserved, reserved3),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		memb_reserved3_constraint_1,
		&asn_PER_memb_reserved3_constr_4,
		0,
		"reserved3"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct EphemerisSubframe1Reserved, reserved4),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		memb_reserved4_constraint_1,
		&asn_PER_memb_reserved4_constr_5,
		0,
		"reserved4"
		},
};
static ber_tlv_tag_t asn_DEF_EphemerisSubframe1Reserved_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static asn_TYPE_tag2member_t asn_MAP_EphemerisSubframe1Reserved_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* reserved1 */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* reserved2 */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* reserved3 */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 } /* reserved4 */
};
static asn_SEQUENCE_specifics_t asn_SPC_EphemerisSubframe1Reserved_specs_1 = {
	sizeof(struct EphemerisSubframe1Reserved),
	offsetof(struct EphemerisSubframe1Reserved, _asn_ctx),
	asn_MAP_EphemerisSubframe1Reserved_tag2el_1,
	4,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* Start extensions */
	-1	/* Stop extensions */
};
asn_TYPE_descriptor_t asn_DEF_EphemerisSubframe1Reserved = {
	"EphemerisSubframe1Reserved",
	"EphemerisSubframe1Reserved",
	SEQUENCE_free,
	SEQUENCE_print,
	SEQUENCE_constraint,
	SEQUENCE_decode_ber,
	SEQUENCE_encode_der,
	SEQUENCE_decode_xer,
	SEQUENCE_encode_xer,
	SEQUENCE_decode_uper,
	SEQUENCE_encode_uper,
	0,	/* Use generic outmost tag fetcher */
	asn_DEF_EphemerisSubframe1Reserved_tags_1,
	sizeof(asn_DEF_EphemerisSubframe1Reserved_tags_1)
		/sizeof(asn_DEF_EphemerisSubframe1Reserved_tags_1[0]), /* 1 */
	asn_DEF_EphemerisSubframe1Reserved_tags_1,	/* Same as above */
	sizeof(asn_DEF_EphemerisSubframe1Reserved_tags_1)
		/sizeof(asn_DEF_EphemerisSubframe1Reserved_tags_1[0]), /* 1 */
	0,	/* No PER visible constraints */
	asn_MBR_EphemerisSubframe1Reserved_1,
	4,	/* Elements count */
	&asn_SPC_EphemerisSubframe1Reserved_specs_1	/* Additional specs */
};

