/*
 * Generated by asn1c-0.9.24 (http://lionet.info/asn1c)
 * From ASN.1 module "RRLP-Components"
 * 	found in "rrlp-components.asn"
 * 	`asn1c -gen-PER -fnative-types`
 */

#include "MsrPosition-Rsp.h"

static asn_TYPE_member_t asn_MBR_MsrPosition_Rsp_1[] = {
	{ ATF_POINTER, 9, offsetof(struct MsrPosition_Rsp, multipleSets),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_MultipleSets,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"multipleSets"
		},
	{ ATF_POINTER, 8, offsetof(struct MsrPosition_Rsp, referenceIdentity),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ReferenceIdentity,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"referenceIdentity"
		},
	{ ATF_POINTER, 7, offsetof(struct MsrPosition_Rsp, otd_MeasureInfo),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_OTD_MeasureInfo,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"otd-MeasureInfo"
		},
	{ ATF_POINTER, 6, offsetof(struct MsrPosition_Rsp, locationInfo),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_LocationInfo,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"locationInfo"
		},
	{ ATF_POINTER, 5, offsetof(struct MsrPosition_Rsp, gps_MeasureInfo),
		(ASN_TAG_CLASS_CONTEXT | (4 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_GPS_MeasureInfo,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"gps-MeasureInfo"
		},
	{ ATF_POINTER, 4, offsetof(struct MsrPosition_Rsp, locationError),
		(ASN_TAG_CLASS_CONTEXT | (5 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_LocationError,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"locationError"
		},
	{ ATF_POINTER, 3, offsetof(struct MsrPosition_Rsp, extensionContainer),
		(ASN_TAG_CLASS_CONTEXT | (6 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ExtensionContainer,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"extensionContainer"
		},
	{ ATF_POINTER, 2, offsetof(struct MsrPosition_Rsp, rel_98_MsrPosition_Rsp_Extension),
		(ASN_TAG_CLASS_CONTEXT | (7 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_Rel_98_MsrPosition_Rsp_Extension,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"rel-98-MsrPosition-Rsp-Extension"
		},
	{ ATF_POINTER, 1, offsetof(struct MsrPosition_Rsp, rel_5_MsrPosition_Rsp_Extension),
		(ASN_TAG_CLASS_CONTEXT | (8 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_Rel_5_MsrPosition_Rsp_Extension,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"rel-5-MsrPosition-Rsp-Extension"
		},
};
static int asn_MAP_MsrPosition_Rsp_oms_1[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8 };
static ber_tlv_tag_t asn_DEF_MsrPosition_Rsp_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static asn_TYPE_tag2member_t asn_MAP_MsrPosition_Rsp_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* multipleSets */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* referenceIdentity */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* otd-MeasureInfo */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 }, /* locationInfo */
    { (ASN_TAG_CLASS_CONTEXT | (4 << 2)), 4, 0, 0 }, /* gps-MeasureInfo */
    { (ASN_TAG_CLASS_CONTEXT | (5 << 2)), 5, 0, 0 }, /* locationError */
    { (ASN_TAG_CLASS_CONTEXT | (6 << 2)), 6, 0, 0 }, /* extensionContainer */
    { (ASN_TAG_CLASS_CONTEXT | (7 << 2)), 7, 0, 0 }, /* rel-98-MsrPosition-Rsp-Extension */
    { (ASN_TAG_CLASS_CONTEXT | (8 << 2)), 8, 0, 0 } /* rel-5-MsrPosition-Rsp-Extension */
};
static asn_SEQUENCE_specifics_t asn_SPC_MsrPosition_Rsp_specs_1 = {
	sizeof(struct MsrPosition_Rsp),
	offsetof(struct MsrPosition_Rsp, _asn_ctx),
	asn_MAP_MsrPosition_Rsp_tag2el_1,
	9,	/* Count of tags in the map */
	asn_MAP_MsrPosition_Rsp_oms_1,	/* Optional members */
	7, 2,	/* Root/Additions */
	6,	/* Start extensions */
	10	/* Stop extensions */
};
asn_TYPE_descriptor_t asn_DEF_MsrPosition_Rsp = {
	"MsrPosition-Rsp",
	"MsrPosition-Rsp",
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
	asn_DEF_MsrPosition_Rsp_tags_1,
	sizeof(asn_DEF_MsrPosition_Rsp_tags_1)
		/sizeof(asn_DEF_MsrPosition_Rsp_tags_1[0]), /* 1 */
	asn_DEF_MsrPosition_Rsp_tags_1,	/* Same as above */
	sizeof(asn_DEF_MsrPosition_Rsp_tags_1)
		/sizeof(asn_DEF_MsrPosition_Rsp_tags_1[0]), /* 1 */
	0,	/* No PER visible constraints */
	asn_MBR_MsrPosition_Rsp_1,
	9,	/* Elements count */
	&asn_SPC_MsrPosition_Rsp_specs_1	/* Additional specs */
};

