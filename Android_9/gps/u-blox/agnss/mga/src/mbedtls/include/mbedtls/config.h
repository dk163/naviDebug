/**
 * \file config.h
 *
 * \brief Configuration options (set of defines)
 *
 *  This set of compile-time options may be used to enable
 *  or disable features selectively, and reduce the global
 *  memory footprint.
 *
 *  Copyright (C) 2006-2015, ARM Limited, All Rights Reserved
 *  SPDX-License-Identifier: Apache-2.0
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may
 *  not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *  This file is part of mbed TLS (https://tls.mbed.org)
 */

/* This file was modified by u-blox */

#ifndef MBEDTLS_CONFIG_H
#define MBEDTLS_CONFIG_H

#if defined(_MSC_VER) && !defined(_CRT_SECURE_NO_DEPRECATE)
#define _CRT_SECURE_NO_DEPRECATE 1
#endif

/**
 * \name SECTION: System support
 *
 * This section sets system specific settings.
 * \{
 */

/**
 * \def MBEDTLS_HAVE_ASM
 *
 * The compiler has support for asm().
 *
 * Requires support for asm() in compiler.
 *
 * Used in:
 *      library/timing.c
 *      library/padlock.c
 *      include/mbedtls/bn_mul.h
 *
 * Comment to disable the use of assembly code.
 */
#define MBEDTLS_HAVE_ASM

/**
 * \def MBEDTLS_HAVE_SSE2
 *
 * CPU supports SSE2 instruction set.
 *
 * Uncomment if the CPU supports SSE2 (IA-32 specific).
 */
//#define MBEDTLS_HAVE_SSE2

/**
 * \def MBEDTLS_HAVE_TIME
 *
 * System has time.h and time().
 * The time does not need to be correct, only time differences are used,
 * by contrast with MBEDTLS_HAVE_TIME_DATE
 *
 * Comment if your system does not support time functions
 */
#define MBEDTLS_HAVE_TIME

/**
 * \def MBEDTLS_HAVE_TIME_DATE
 *
 * System has time.h and time(), gmtime() and the clock is correct.
 * The time needs to be correct (not necesarily very accurate, but at least
 * the date should be correct). This is used to verify the validity period of
 * X.509 certificates.
 *
 * Comment if your system does not have a correct clock.
 */
#define MBEDTLS_HAVE_TIME_DATE

/* \} name SECTION: System support */

/**
* \name SECTION: mbed TLS feature support
*
* This section sets support for features that are or are not needed
* within the modules that are enabled.
* \{
*/

/**
 * \def MBEDTLS_CIPHER_MODE_CBC
 *
 * Enable Cipher Block Chaining mode (CBC) for symmetric ciphers.
 */
#define MBEDTLS_CIPHER_MODE_CBC

/**
 * \def MBEDTLS_CIPHER_MODE_CFB
 *
 * Enable Cipher Feedback mode (CFB) for symmetric ciphers.
 */
#define MBEDTLS_CIPHER_MODE_CFB

/**
 * \def MBEDTLS_CIPHER_MODE_CTR
 *
 * Enable Counter Block Cipher mode (CTR) for symmetric ciphers.
 */
#define MBEDTLS_CIPHER_MODE_CTR


/**
 * \def MBEDTLS_CIPHER_PADDING_PKCS7
 *
 * MBEDTLS_CIPHER_PADDING_XXX: Uncomment or comment macros to add support for
 * specific padding modes in the cipher layer with cipher modes that support
 * padding (e.g. CBC)
 *
 * If you disable all padding modes, only full blocks can be used with CBC.
 *
 * Enable padding modes in the cipher layer.
 */
#define MBEDTLS_CIPHER_PADDING_PKCS7
#define MBEDTLS_CIPHER_PADDING_ONE_AND_ZEROS
#define MBEDTLS_CIPHER_PADDING_ZEROS_AND_LEN
#define MBEDTLS_CIPHER_PADDING_ZEROS

/**
 * \def MBEDTLS_REMOVE_ARC4_CIPHERSUITES
 *
 * Remove RC4 ciphersuites by default in SSL / TLS.
 * This flag removes the ciphersuites based on RC4 from the default list as
 * returned by mbedtls_ssl_list_ciphersuites(). However, it is still possible to
 * enable (some of) them with mbedtls_ssl_conf_ciphersuites() by including them
 * explicitly.
 *
 * Uncomment this macro to remove RC4 ciphersuites by default.
 */
#define MBEDTLS_REMOVE_ARC4_CIPHERSUITES

/**
 * \def MBEDTLS_ECP_DP_SECP192R1_ENABLED
 *
 * MBEDTLS_ECP_XXXX_ENABLED: Enables specific curves within the Elliptic Curve
 * module.  By default all supported curves are enabled.
 *
 * Comment macros to disable the curve and functions for it
 */
#define MBEDTLS_ECP_DP_SECP192R1_ENABLED
#define MBEDTLS_ECP_DP_SECP224R1_ENABLED
#define MBEDTLS_ECP_DP_SECP256R1_ENABLED
#define MBEDTLS_ECP_DP_SECP384R1_ENABLED
#define MBEDTLS_ECP_DP_SECP521R1_ENABLED
#define MBEDTLS_ECP_DP_SECP192K1_ENABLED
#define MBEDTLS_ECP_DP_SECP224K1_ENABLED
#define MBEDTLS_ECP_DP_SECP256K1_ENABLED
#define MBEDTLS_ECP_DP_BP256R1_ENABLED
#define MBEDTLS_ECP_DP_BP384R1_ENABLED
#define MBEDTLS_ECP_DP_BP512R1_ENABLED
#define MBEDTLS_ECP_DP_CURVE25519_ENABLED

/**
 * \def MBEDTLS_ECP_NIST_OPTIM
 *
 * Enable specific 'modulo p' routines for each NIST prime.
 * Depending on the prime and architecture, makes operations 4 to 8 times
 * faster on the corresponding curve.
 *
 * Comment this macro to disable NIST curves optimisation.
 */
#define MBEDTLS_ECP_NIST_OPTIM

/**
 * \def MBEDTLS_ECDSA_DETERMINISTIC
 *
 * Enable deterministic ECDSA (RFC 6979).
 * Standard ECDSA is "fragile" in the sense that lack of entropy when signing
 * may result in a compromise of the long-term signing key. This is avoided by
 * the deterministic variant.
 *
 * Requires: MBEDTLS_HMAC_DRBG_C
 *
 * Comment this macro to disable deterministic ECDSA.
 */
#define MBEDTLS_ECDSA_DETERMINISTIC

/**
 * \def MBEDTLS_KEY_EXCHANGE_PSK_ENABLED
 *
 * Enable the PSK based ciphersuite modes in SSL / TLS.
 *
 * This enables the following ciphersuites (if other requisites are
 * enabled as well):
 *      MBEDTLS_TLS_PSK_WITH_AES_256_GCM_SHA384
 *      MBEDTLS_TLS_PSK_WITH_AES_256_CBC_SHA384
 *      MBEDTLS_TLS_PSK_WITH_AES_256_CBC_SHA
 *      MBEDTLS_TLS_PSK_WITH_CAMELLIA_256_GCM_SHA384
 *      MBEDTLS_TLS_PSK_WITH_CAMELLIA_256_CBC_SHA384
 *      MBEDTLS_TLS_PSK_WITH_AES_128_GCM_SHA256
 *      MBEDTLS_TLS_PSK_WITH_AES_128_CBC_SHA256
 *      MBEDTLS_TLS_PSK_WITH_AES_128_CBC_SHA
 *      MBEDTLS_TLS_PSK_WITH_CAMELLIA_128_GCM_SHA256
 *      MBEDTLS_TLS_PSK_WITH_CAMELLIA_128_CBC_SHA256
 *      MBEDTLS_TLS_PSK_WITH_3DES_EDE_CBC_SHA
 *      MBEDTLS_TLS_PSK_WITH_RC4_128_SHA
 */
#define MBEDTLS_KEY_EXCHANGE_PSK_ENABLED

/**
 * \def MBEDTLS_KEY_EXCHANGE_DHE_PSK_ENABLED
 *
 * Enable the DHE-PSK based ciphersuite modes in SSL / TLS.
 *
 * Requires: MBEDTLS_DHM_C
 *
 * This enables the following ciphersuites (if other requisites are
 * enabled as well):
 *      MBEDTLS_TLS_DHE_PSK_WITH_AES_256_GCM_SHA384
 *      MBEDTLS_TLS_DHE_PSK_WITH_AES_256_CBC_SHA384
 *      MBEDTLS_TLS_DHE_PSK_WITH_AES_256_CBC_SHA
 *      MBEDTLS_TLS_DHE_PSK_WITH_CAMELLIA_256_GCM_SHA384
 *      MBEDTLS_TLS_DHE_PSK_WITH_CAMELLIA_256_CBC_SHA384
 *      MBEDTLS_TLS_DHE_PSK_WITH_AES_128_GCM_SHA256
 *      MBEDTLS_TLS_DHE_PSK_WITH_AES_128_CBC_SHA256
 *      MBEDTLS_TLS_DHE_PSK_WITH_AES_128_CBC_SHA
 *      MBEDTLS_TLS_DHE_PSK_WITH_CAMELLIA_128_GCM_SHA256
 *      MBEDTLS_TLS_DHE_PSK_WITH_CAMELLIA_128_CBC_SHA256
 *      MBEDTLS_TLS_DHE_PSK_WITH_3DES_EDE_CBC_SHA
 *      MBEDTLS_TLS_DHE_PSK_WITH_RC4_128_SHA
 */
#define MBEDTLS_KEY_EXCHANGE_DHE_PSK_ENABLED

/**
 * \def MBEDTLS_KEY_EXCHANGE_ECDHE_PSK_ENABLED
 *
 * Enable the ECDHE-PSK based ciphersuite modes in SSL / TLS.
 *
 * Requires: MBEDTLS_ECDH_C
 *
 * This enables the following ciphersuites (if other requisites are
 * enabled as well):
 *      MBEDTLS_TLS_ECDHE_PSK_WITH_AES_256_CBC_SHA384
 *      MBEDTLS_TLS_ECDHE_PSK_WITH_AES_256_CBC_SHA
 *      MBEDTLS_TLS_ECDHE_PSK_WITH_CAMELLIA_256_CBC_SHA384
 *      MBEDTLS_TLS_ECDHE_PSK_WITH_AES_128_CBC_SHA256
 *      MBEDTLS_TLS_ECDHE_PSK_WITH_AES_128_CBC_SHA
 *      MBEDTLS_TLS_ECDHE_PSK_WITH_CAMELLIA_128_CBC_SHA256
 *      MBEDTLS_TLS_ECDHE_PSK_WITH_3DES_EDE_CBC_SHA
 *      MBEDTLS_TLS_ECDHE_PSK_WITH_RC4_128_SHA
 */
#define MBEDTLS_KEY_EXCHANGE_ECDHE_PSK_ENABLED

/**
 * \def MBEDTLS_KEY_EXCHANGE_RSA_PSK_ENABLED
 *
 * Enable the RSA-PSK based ciphersuite modes in SSL / TLS.
 *
 * Requires: MBEDTLS_RSA_C, MBEDTLS_PKCS1_V15,
 *           MBEDTLS_X509_CRT_PARSE_C
 *
 * This enables the following ciphersuites (if other requisites are
 * enabled as well):
 *      MBEDTLS_TLS_RSA_PSK_WITH_AES_256_GCM_SHA384
 *      MBEDTLS_TLS_RSA_PSK_WITH_AES_256_CBC_SHA384
 *      MBEDTLS_TLS_RSA_PSK_WITH_AES_256_CBC_SHA
 *      MBEDTLS_TLS_RSA_PSK_WITH_CAMELLIA_256_GCM_SHA384
 *      MBEDTLS_TLS_RSA_PSK_WITH_CAMELLIA_256_CBC_SHA384
 *      MBEDTLS_TLS_RSA_PSK_WITH_AES_128_GCM_SHA256
 *      MBEDTLS_TLS_RSA_PSK_WITH_AES_128_CBC_SHA256
 *      MBEDTLS_TLS_RSA_PSK_WITH_AES_128_CBC_SHA
 *      MBEDTLS_TLS_RSA_PSK_WITH_CAMELLIA_128_GCM_SHA256
 *      MBEDTLS_TLS_RSA_PSK_WITH_CAMELLIA_128_CBC_SHA256
 *      MBEDTLS_TLS_RSA_PSK_WITH_3DES_EDE_CBC_SHA
 *      MBEDTLS_TLS_RSA_PSK_WITH_RC4_128_SHA
 */
#define MBEDTLS_KEY_EXCHANGE_RSA_PSK_ENABLED

/**
 * \def MBEDTLS_KEY_EXCHANGE_RSA_ENABLED
 *
 * Enable the RSA-only based ciphersuite modes in SSL / TLS.
 *
 * Requires: MBEDTLS_RSA_C, MBEDTLS_PKCS1_V15,
 *           MBEDTLS_X509_CRT_PARSE_C
 *
 * This enables the following ciphersuites (if other requisites are
 * enabled as well):
 *      MBEDTLS_TLS_RSA_WITH_AES_256_GCM_SHA384
 *      MBEDTLS_TLS_RSA_WITH_AES_256_CBC_SHA256
 *      MBEDTLS_TLS_RSA_WITH_AES_256_CBC_SHA
 *      MBEDTLS_TLS_RSA_WITH_CAMELLIA_256_GCM_SHA384
 *      MBEDTLS_TLS_RSA_WITH_CAMELLIA_256_CBC_SHA256
 *      MBEDTLS_TLS_RSA_WITH_CAMELLIA_256_CBC_SHA
 *      MBEDTLS_TLS_RSA_WITH_AES_128_GCM_SHA256
 *      MBEDTLS_TLS_RSA_WITH_AES_128_CBC_SHA256
 *      MBEDTLS_TLS_RSA_WITH_AES_128_CBC_SHA
 *      MBEDTLS_TLS_RSA_WITH_CAMELLIA_128_GCM_SHA256
 *      MBEDTLS_TLS_RSA_WITH_CAMELLIA_128_CBC_SHA256
 *      MBEDTLS_TLS_RSA_WITH_CAMELLIA_128_CBC_SHA
 *      MBEDTLS_TLS_RSA_WITH_3DES_EDE_CBC_SHA
 *      MBEDTLS_TLS_RSA_WITH_RC4_128_SHA
 *      MBEDTLS_TLS_RSA_WITH_RC4_128_MD5
 */
#define MBEDTLS_KEY_EXCHANGE_RSA_ENABLED

/**
 * \def MBEDTLS_KEY_EXCHANGE_DHE_RSA_ENABLED
 *
 * Enable the DHE-RSA based ciphersuite modes in SSL / TLS.
 *
 * Requires: MBEDTLS_DHM_C, MBEDTLS_RSA_C, MBEDTLS_PKCS1_V15,
 *           MBEDTLS_X509_CRT_PARSE_C
 *
 * This enables the following ciphersuites (if other requisites are
 * enabled as well):
 *      MBEDTLS_TLS_DHE_RSA_WITH_AES_256_GCM_SHA384
 *      MBEDTLS_TLS_DHE_RSA_WITH_AES_256_CBC_SHA256
 *      MBEDTLS_TLS_DHE_RSA_WITH_AES_256_CBC_SHA
 *      MBEDTLS_TLS_DHE_RSA_WITH_CAMELLIA_256_GCM_SHA384
 *      MBEDTLS_TLS_DHE_RSA_WITH_CAMELLIA_256_CBC_SHA256
 *      MBEDTLS_TLS_DHE_RSA_WITH_CAMELLIA_256_CBC_SHA
 *      MBEDTLS_TLS_DHE_RSA_WITH_AES_128_GCM_SHA256
 *      MBEDTLS_TLS_DHE_RSA_WITH_AES_128_CBC_SHA256
 *      MBEDTLS_TLS_DHE_RSA_WITH_AES_128_CBC_SHA
 *      MBEDTLS_TLS_DHE_RSA_WITH_CAMELLIA_128_GCM_SHA256
 *      MBEDTLS_TLS_DHE_RSA_WITH_CAMELLIA_128_CBC_SHA256
 *      MBEDTLS_TLS_DHE_RSA_WITH_CAMELLIA_128_CBC_SHA
 *      MBEDTLS_TLS_DHE_RSA_WITH_3DES_EDE_CBC_SHA
 */
#define MBEDTLS_KEY_EXCHANGE_DHE_RSA_ENABLED

/**
 * \def MBEDTLS_KEY_EXCHANGE_ECDHE_RSA_ENABLED
 *
 * Enable the ECDHE-RSA based ciphersuite modes in SSL / TLS.
 *
 * Requires: MBEDTLS_ECDH_C, MBEDTLS_RSA_C, MBEDTLS_PKCS1_V15,
 *           MBEDTLS_X509_CRT_PARSE_C
 *
 * This enables the following ciphersuites (if other requisites are
 * enabled as well):
 *      MBEDTLS_TLS_ECDHE_RSA_WITH_AES_256_GCM_SHA384
 *      MBEDTLS_TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA384
 *      MBEDTLS_TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA
 *      MBEDTLS_TLS_ECDHE_RSA_WITH_CAMELLIA_256_GCM_SHA384
 *      MBEDTLS_TLS_ECDHE_RSA_WITH_CAMELLIA_256_CBC_SHA384
 *      MBEDTLS_TLS_ECDHE_RSA_WITH_AES_128_GCM_SHA256
 *      MBEDTLS_TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA256
 *      MBEDTLS_TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA
 *      MBEDTLS_TLS_ECDHE_RSA_WITH_CAMELLIA_128_GCM_SHA256
 *      MBEDTLS_TLS_ECDHE_RSA_WITH_CAMELLIA_128_CBC_SHA256
 *      MBEDTLS_TLS_ECDHE_RSA_WITH_3DES_EDE_CBC_SHA
 *      MBEDTLS_TLS_ECDHE_RSA_WITH_RC4_128_SHA
 */
#define MBEDTLS_KEY_EXCHANGE_ECDHE_RSA_ENABLED

/**
 * \def MBEDTLS_KEY_EXCHANGE_ECDHE_ECDSA_ENABLED
 *
 * Enable the ECDHE-ECDSA based ciphersuite modes in SSL / TLS.
 *
 * Requires: MBEDTLS_ECDH_C, MBEDTLS_ECDSA_C, MBEDTLS_X509_CRT_PARSE_C,
 *
 * This enables the following ciphersuites (if other requisites are
 * enabled as well):
 *      MBEDTLS_TLS_ECDHE_ECDSA_WITH_AES_256_GCM_SHA384
 *      MBEDTLS_TLS_ECDHE_ECDSA_WITH_AES_256_CBC_SHA384
 *      MBEDTLS_TLS_ECDHE_ECDSA_WITH_AES_256_CBC_SHA
 *      MBEDTLS_TLS_ECDHE_ECDSA_WITH_CAMELLIA_256_GCM_SHA384
 *      MBEDTLS_TLS_ECDHE_ECDSA_WITH_CAMELLIA_256_CBC_SHA384
 *      MBEDTLS_TLS_ECDHE_ECDSA_WITH_AES_128_GCM_SHA256
 *      MBEDTLS_TLS_ECDHE_ECDSA_WITH_AES_128_CBC_SHA256
 *      MBEDTLS_TLS_ECDHE_ECDSA_WITH_AES_128_CBC_SHA
 *      MBEDTLS_TLS_ECDHE_ECDSA_WITH_CAMELLIA_128_GCM_SHA256
 *      MBEDTLS_TLS_ECDHE_ECDSA_WITH_CAMELLIA_128_CBC_SHA256
 *      MBEDTLS_TLS_ECDHE_ECDSA_WITH_3DES_EDE_CBC_SHA
 *      MBEDTLS_TLS_ECDHE_ECDSA_WITH_RC4_128_SHA
 */
#define MBEDTLS_KEY_EXCHANGE_ECDHE_ECDSA_ENABLED

/**
 * \def MBEDTLS_KEY_EXCHANGE_ECDH_ECDSA_ENABLED
 *
 * Enable the ECDH-ECDSA based ciphersuite modes in SSL / TLS.
 *
 * Requires: MBEDTLS_ECDH_C, MBEDTLS_X509_CRT_PARSE_C
 *
 * This enables the following ciphersuites (if other requisites are
 * enabled as well):
 *      MBEDTLS_TLS_ECDH_ECDSA_WITH_RC4_128_SHA
 *      MBEDTLS_TLS_ECDH_ECDSA_WITH_3DES_EDE_CBC_SHA
 *      MBEDTLS_TLS_ECDH_ECDSA_WITH_AES_128_CBC_SHA
 *      MBEDTLS_TLS_ECDH_ECDSA_WITH_AES_256_CBC_SHA
 *      MBEDTLS_TLS_ECDH_ECDSA_WITH_AES_128_CBC_SHA256
 *      MBEDTLS_TLS_ECDH_ECDSA_WITH_AES_256_CBC_SHA384
 *      MBEDTLS_TLS_ECDH_ECDSA_WITH_AES_128_GCM_SHA256
 *      MBEDTLS_TLS_ECDH_ECDSA_WITH_AES_256_GCM_SHA384
 *      MBEDTLS_TLS_ECDH_ECDSA_WITH_CAMELLIA_128_CBC_SHA256
 *      MBEDTLS_TLS_ECDH_ECDSA_WITH_CAMELLIA_256_CBC_SHA384
 *      MBEDTLS_TLS_ECDH_ECDSA_WITH_CAMELLIA_128_GCM_SHA256
 *      MBEDTLS_TLS_ECDH_ECDSA_WITH_CAMELLIA_256_GCM_SHA384
 */
#define MBEDTLS_KEY_EXCHANGE_ECDH_ECDSA_ENABLED

/**
 * \def MBEDTLS_KEY_EXCHANGE_ECDH_RSA_ENABLED
 *
 * Enable the ECDH-RSA based ciphersuite modes in SSL / TLS.
 *
 * Requires: MBEDTLS_ECDH_C, MBEDTLS_X509_CRT_PARSE_C
 *
 * This enables the following ciphersuites (if other requisites are
 * enabled as well):
 *      MBEDTLS_TLS_ECDH_RSA_WITH_RC4_128_SHA
 *      MBEDTLS_TLS_ECDH_RSA_WITH_3DES_EDE_CBC_SHA
 *      MBEDTLS_TLS_ECDH_RSA_WITH_AES_128_CBC_SHA
 *      MBEDTLS_TLS_ECDH_RSA_WITH_AES_256_CBC_SHA
 *      MBEDTLS_TLS_ECDH_RSA_WITH_AES_128_CBC_SHA256
 *      MBEDTLS_TLS_ECDH_RSA_WITH_AES_256_CBC_SHA384
 *      MBEDTLS_TLS_ECDH_RSA_WITH_AES_128_GCM_SHA256
 *      MBEDTLS_TLS_ECDH_RSA_WITH_AES_256_GCM_SHA384
 *      MBEDTLS_TLS_ECDH_RSA_WITH_CAMELLIA_128_CBC_SHA256
 *      MBEDTLS_TLS_ECDH_RSA_WITH_CAMELLIA_256_CBC_SHA384
 *      MBEDTLS_TLS_ECDH_RSA_WITH_CAMELLIA_128_GCM_SHA256
 *      MBEDTLS_TLS_ECDH_RSA_WITH_CAMELLIA_256_GCM_SHA384
 */
#define MBEDTLS_KEY_EXCHANGE_ECDH_RSA_ENABLED


/**
 * \def MBEDTLS_PK_PARSE_EC_EXTENDED
 *
 * Enhance support for reading EC keys using variants of SEC1 not allowed by
 * RFC 5915 and RFC 5480.
 *
 * Currently this means parsing the SpecifiedECDomain choice of EC
 * parameters (only known groups are supported, not arbitrary domains, to
 * avoid validation issues).
 *
 * Disable if you only need to support RFC 5915 + 5480 key formats.
 */
#define MBEDTLS_PK_PARSE_EC_EXTENDED

/**
 * \def MBEDTLS_ERROR_STRERROR_DUMMY
 *
 * Enable a dummy error function to make use of mbedtls_strerror() in
 * third party libraries easier when MBEDTLS_ERROR_C is disabled
 * (no effect when MBEDTLS_ERROR_C is enabled).
 *
 * You can safely disable this if MBEDTLS_ERROR_C is enabled, or if you're
 * not using mbedtls_strerror() or error_strerror() in your application.
 *
 * Disable if you run into name conflicts and want to really remove the
 * mbedtls_strerror()
 */
#define MBEDTLS_ERROR_STRERROR_DUMMY

/**
 * \def MBEDTLS_GENPRIME
 *
 * Enable the prime-number generation code.
 *
 * Requires: MBEDTLS_BIGNUM_C
 */
#define MBEDTLS_GENPRIME

/**
 * \def MBEDTLS_FS_IO
 *
 * Enable functions that use the filesystem.
 */
#define MBEDTLS_FS_IO


/**
 * \def MBEDTLS_PK_RSA_ALT_SUPPORT
 *
 * Support external private RSA keys (eg from a HSM) in the PK layer.
 *
 * Comment this macro to disable support for external private RSA keys.
 */
#define MBEDTLS_PK_RSA_ALT_SUPPORT

/**
 * \def MBEDTLS_PKCS1_V15
 *
 * Enable support for PKCS#1 v1.5 encoding.
 *
 * Requires: MBEDTLS_RSA_C
 *
 * This enables support for PKCS#1 v1.5 operations.
 */
#define MBEDTLS_PKCS1_V15

/**
 * \def MBEDTLS_PKCS1_V21
 *
 * Enable support for PKCS#1 v2.1 encoding.
 *
 * Requires: MBEDTLS_MD_C, MBEDTLS_RSA_C
 *
 * This enables support for RSAES-OAEP and RSASSA-PSS operations.
 */
#define MBEDTLS_PKCS1_V21


/**
 * \def MBEDTLS_SELF_TEST
 *
 * Enable the checkup functions (*_self_test).
 */
#define MBEDTLS_SELF_TEST


/**
 * \def MBEDTLS_SSL_ALL_ALERT_MESSAGES
 *
 * Enable sending of alert messages in case of encountered errors as per RFC.
 * If you choose not to send the alert messages, mbed TLS can still communicate
 * with other servers, only debugging of failures is harder.
 *
 * The advantage of not sending alert messages, is that no information is given
 * about reasons for failures thus preventing adversaries of gaining intel.
 *
 * Enable sending of all alert messages
 */
#define MBEDTLS_SSL_ALL_ALERT_MESSAGES


/** \def MBEDTLS_SSL_ENCRYPT_THEN_MAC
 *
 * Enable support for Encrypt-then-MAC, RFC 7366.
 *
 * This allows peers that both support it to use a more robust protection for
 * ciphersuites using CBC, providing deep resistance against timing attacks
 * on the padding or underlying cipher.
 *
 * This only affects CBC ciphersuites, and is useless if none is defined.
 *
 * Requires: MBEDTLS_SSL_PROTO_TLS1    or
 *           MBEDTLS_SSL_PROTO_TLS1_1  or
 *           MBEDTLS_SSL_PROTO_TLS1_2
 *
 * Comment this macro to disable support for Encrypt-then-MAC
 */
#define MBEDTLS_SSL_ENCRYPT_THEN_MAC

/** \def MBEDTLS_SSL_EXTENDED_MASTER_SECRET
 *
 * Enable support for Extended Master Secret, aka Session Hash
 * (draft-ietf-tls-session-hash-02).
 *
 * This was introduced as "the proper fix" to the Triple Handshake familiy of
 * attacks, but it is recommended to always use it (even if you disable
 * renegotiation), since it actually fixes a more fundamental issue in the
 * original SSL/TLS design, and has implications beyond Triple Handshake.
 *
 * Requires: MBEDTLS_SSL_PROTO_TLS1    or
 *           MBEDTLS_SSL_PROTO_TLS1_1  or
 *           MBEDTLS_SSL_PROTO_TLS1_2
 *
 * Comment this macro to disable support for Extended Master Secret.
 */
#define MBEDTLS_SSL_EXTENDED_MASTER_SECRET

/**
 * \def MBEDTLS_SSL_FALLBACK_SCSV
 *
 * Enable support for FALLBACK_SCSV (draft-ietf-tls-downgrade-scsv-00).
 *
 * For servers, it is recommended to always enable this, unless you support
 * only one version of TLS, or know for sure that none of your clients
 * implements a fallback strategy.
 *
 * For clients, you only need this if you're using a fallback strategy, which
 * is not recommended in the first place, unless you absolutely need it to
 * interoperate with buggy (version-intolerant) servers.
 *
 * Comment this macro to disable support for FALLBACK_SCSV
 */
#define MBEDTLS_SSL_FALLBACK_SCSV


/**
 * \def MBEDTLS_SSL_RENEGOTIATION
 *
 * Disable support for TLS renegotiation.
 *
 * The two main uses of renegotiation are (1) refresh keys on long-lived
 * connections and (2) client authentication after the initial handshake.
 * If you don't need renegotiation, it's probably better to disable it, since
 * it has been associated with security issues in the past and is easy to
 * misuse/misunderstand.
 *
 * Comment this to disable support for renegotiation.
 */
#define MBEDTLS_SSL_RENEGOTIATION


/**
 * \def MBEDTLS_SSL_MAX_FRAGMENT_LENGTH
 *
 * Enable support for RFC 6066 max_fragment_length extension in SSL.
 *
 * Comment this macro to disable support for the max_fragment_length extension
 */
#define MBEDTLS_SSL_MAX_FRAGMENT_LENGTH


/**
 * \def MBEDTLS_SSL_PROTO_TLS1_2
 *
 * Enable support for TLS 1.2 (and DTLS 1.2 if DTLS is enabled).
 *
 * Requires: MBEDTLS_SHA1_C or MBEDTLS_SHA256_C or MBEDTLS_SHA512_C
 *           (Depends on ciphersuites)
 *
 * Comment this macro to disable support for TLS 1.2 / DTLS 1.2
 */
#define MBEDTLS_SSL_PROTO_TLS1_2


/**
 * \def MBEDTLS_SSL_ALPN
 *
 * Enable support for RFC 7301 Application Layer Protocol Negotiation.
 *
 * Comment this macro to disable support for ALPN.
 */
#define MBEDTLS_SSL_ALPN


/**
 * \def MBEDTLS_SSL_SESSION_TICKETS
 *
 * Enable support for RFC 5077 session tickets in SSL.
 * Client-side, provides full support for session tickets (maintainance of a
 * session store remains the responsibility of the application, though).
 * Server-side, you also need to provide callbacks for writing and parsing
 * tickets, including authenticated encryption and key management. Example
 * callbacks are provided by MBEDTLS_SSL_TICKET_C.
 *
 * Comment this macro to disable support for SSL session tickets
 */
#define MBEDTLS_SSL_SESSION_TICKETS

/**
 * \def MBEDTLS_SSL_EXPORT_KEYS
 *
 * Enable support for exporting key block and master secret.
 * This is required for certain users of TLS, e.g. EAP-TLS.
 *
 * Comment this macro to disable support for key export
 */
#define MBEDTLS_SSL_EXPORT_KEYS

/**
 * \def MBEDTLS_SSL_SERVER_NAME_INDICATION
 *
 * Enable support for RFC 6066 server name indication (SNI) in SSL.
 *
 * Requires: MBEDTLS_X509_CRT_PARSE_C
 *
 * Comment this macro to disable support for server name indication in SSL
 */
#define MBEDTLS_SSL_SERVER_NAME_INDICATION

/**
 * \def MBEDTLS_SSL_TRUNCATED_HMAC
 *
 * Enable support for RFC 6066 truncated HMAC in SSL.
 *
 * Comment this macro to disable support for truncated HMAC in SSL
 */
#define MBEDTLS_SSL_TRUNCATED_HMAC


/**
 * \def MBEDTLS_VERSION_FEATURES
 *
 * Allow run-time checking of compile-time enabled features. Thus allowing users
 * to check at run-time if the library is for instance compiled with threading
 * support via mbedtls_version_check_feature().
 *
 * Requires: MBEDTLS_VERSION_C
 *
 * Comment this to disable run-time checking and save ROM space
 */
#define MBEDTLS_VERSION_FEATURES


/**
 * \def MBEDTLS_X509_CHECK_KEY_USAGE
 *
 * Enable verification of the keyUsage extension (CA and leaf certificates).
 *
 * Disabling this avoids problems with mis-issued and/or misused
 * (intermediate) CA and leaf certificates.
 *
 * \warning Depending on your PKI use, disabling this can be a security risk!
 *
 * Comment to skip keyUsage checking for both CA and leaf certificates.
 */
#define MBEDTLS_X509_CHECK_KEY_USAGE

/**
 * \def MBEDTLS_X509_CHECK_EXTENDED_KEY_USAGE
 *
 * Enable verification of the extendedKeyUsage extension (leaf certificates).
 *
 * Disabling this avoids problems with mis-issued and/or misused certificates.
 *
 * \warning Depending on your PKI use, disabling this can be a security risk!
 *
 * Comment to skip extendedKeyUsage checking for certificates.
 */
#define MBEDTLS_X509_CHECK_EXTENDED_KEY_USAGE

/**
 * \def MBEDTLS_X509_RSASSA_PSS_SUPPORT
 *
 * Enable parsing and verification of X.509 certificates, CRLs and CSRS
 * signed with RSASSA-PSS (aka PKCS#1 v2.1).
 *
 * Comment this macro to disallow using RSASSA-PSS in certificates.
 */
#define MBEDTLS_X509_RSASSA_PSS_SUPPORT


/* \} name SECTION: mbed TLS feature support */

/**
 * \name SECTION: mbed TLS modules
 *
 * This section enables or disables entire modules in mbed TLS
 * \{
 */

/**
 * \def MBEDTLS_AESNI_C
 *
 * Enable AES-NI support on x86-64.
 *
 * Module:  library/aesni.c
 * Caller:  library/aes.c
 *
 * Requires: MBEDTLS_HAVE_ASM
 *
 * This modules adds support for the AES-NI instructions on x86-64
 */
#define MBEDTLS_AESNI_C

/**
 * \def MBEDTLS_AES_C
 *
 * Enable the AES block cipher.
 *
 * Module:  library/aes.c
 * Caller:  library/ssl_tls.c
 *          library/pem.c
 *          library/ctr_drbg.c
 *
 * This module enables the following ciphersuites (if other requisites are
 * enabled as well):
 *      MBEDTLS_TLS_ECDH_ECDSA_WITH_AES_128_CBC_SHA
 *      MBEDTLS_TLS_ECDH_ECDSA_WITH_AES_256_CBC_SHA
 *      MBEDTLS_TLS_ECDH_RSA_WITH_AES_128_CBC_SHA
 *      MBEDTLS_TLS_ECDH_RSA_WITH_AES_256_CBC_SHA
 *      MBEDTLS_TLS_ECDH_ECDSA_WITH_AES_128_CBC_SHA256
 *      MBEDTLS_TLS_ECDH_ECDSA_WITH_AES_256_CBC_SHA384
 *      MBEDTLS_TLS_ECDH_RSA_WITH_AES_128_CBC_SHA256
 *      MBEDTLS_TLS_ECDH_RSA_WITH_AES_256_CBC_SHA384
 *      MBEDTLS_TLS_ECDH_ECDSA_WITH_AES_128_GCM_SHA256
 *      MBEDTLS_TLS_ECDH_ECDSA_WITH_AES_256_GCM_SHA384
 *      MBEDTLS_TLS_ECDH_RSA_WITH_AES_128_GCM_SHA256
 *      MBEDTLS_TLS_ECDH_RSA_WITH_AES_256_GCM_SHA384
 *      MBEDTLS_TLS_ECDHE_ECDSA_WITH_AES_256_GCM_SHA384
 *      MBEDTLS_TLS_ECDHE_RSA_WITH_AES_256_GCM_SHA384
 *      MBEDTLS_TLS_DHE_RSA_WITH_AES_256_GCM_SHA384
 *      MBEDTLS_TLS_ECDHE_ECDSA_WITH_AES_256_CBC_SHA384
 *      MBEDTLS_TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA384
 *      MBEDTLS_TLS_DHE_RSA_WITH_AES_256_CBC_SHA256
 *      MBEDTLS_TLS_ECDHE_ECDSA_WITH_AES_256_CBC_SHA
 *      MBEDTLS_TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA
 *      MBEDTLS_TLS_DHE_RSA_WITH_AES_256_CBC_SHA
 *      MBEDTLS_TLS_ECDHE_ECDSA_WITH_AES_128_GCM_SHA256
 *      MBEDTLS_TLS_ECDHE_RSA_WITH_AES_128_GCM_SHA256
 *      MBEDTLS_TLS_DHE_RSA_WITH_AES_128_GCM_SHA256
 *      MBEDTLS_TLS_ECDHE_ECDSA_WITH_AES_128_CBC_SHA256
 *      MBEDTLS_TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA256
 *      MBEDTLS_TLS_DHE_RSA_WITH_AES_128_CBC_SHA256
 *      MBEDTLS_TLS_ECDHE_ECDSA_WITH_AES_128_CBC_SHA
 *      MBEDTLS_TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA
 *      MBEDTLS_TLS_DHE_RSA_WITH_AES_128_CBC_SHA
 *      MBEDTLS_TLS_DHE_PSK_WITH_AES_256_GCM_SHA384
 *      MBEDTLS_TLS_ECDHE_PSK_WITH_AES_256_CBC_SHA384
 *      MBEDTLS_TLS_DHE_PSK_WITH_AES_256_CBC_SHA384
 *      MBEDTLS_TLS_ECDHE_PSK_WITH_AES_256_CBC_SHA
 *      MBEDTLS_TLS_DHE_PSK_WITH_AES_256_CBC_SHA
 *      MBEDTLS_TLS_DHE_PSK_WITH_AES_128_GCM_SHA256
 *      MBEDTLS_TLS_ECDHE_PSK_WITH_AES_128_CBC_SHA256
 *      MBEDTLS_TLS_DHE_PSK_WITH_AES_128_CBC_SHA256
 *      MBEDTLS_TLS_ECDHE_PSK_WITH_AES_128_CBC_SHA
 *      MBEDTLS_TLS_DHE_PSK_WITH_AES_128_CBC_SHA
 *      MBEDTLS_TLS_RSA_WITH_AES_256_GCM_SHA384
 *      MBEDTLS_TLS_RSA_WITH_AES_256_CBC_SHA256
 *      MBEDTLS_TLS_RSA_WITH_AES_256_CBC_SHA
 *      MBEDTLS_TLS_RSA_WITH_AES_128_GCM_SHA256
 *      MBEDTLS_TLS_RSA_WITH_AES_128_CBC_SHA256
 *      MBEDTLS_TLS_RSA_WITH_AES_128_CBC_SHA
 *      MBEDTLS_TLS_RSA_PSK_WITH_AES_256_GCM_SHA384
 *      MBEDTLS_TLS_RSA_PSK_WITH_AES_256_CBC_SHA384
 *      MBEDTLS_TLS_RSA_PSK_WITH_AES_256_CBC_SHA
 *      MBEDTLS_TLS_RSA_PSK_WITH_AES_128_GCM_SHA256
 *      MBEDTLS_TLS_RSA_PSK_WITH_AES_128_CBC_SHA256
 *      MBEDTLS_TLS_RSA_PSK_WITH_AES_128_CBC_SHA
 *      MBEDTLS_TLS_PSK_WITH_AES_256_GCM_SHA384
 *      MBEDTLS_TLS_PSK_WITH_AES_256_CBC_SHA384
 *      MBEDTLS_TLS_PSK_WITH_AES_256_CBC_SHA
 *      MBEDTLS_TLS_PSK_WITH_AES_128_GCM_SHA256
 *      MBEDTLS_TLS_PSK_WITH_AES_128_CBC_SHA256
 *      MBEDTLS_TLS_PSK_WITH_AES_128_CBC_SHA
 *
 * PEM_PARSE uses AES for decrypting encrypted keys.
 */
#define MBEDTLS_AES_C

/**
 * \def MBEDTLS_ARC4_C
 *
 * Enable the ARCFOUR stream cipher.
 *
 * Module:  library/arc4.c
 * Caller:  library/ssl_tls.c
 *
 * This module enables the following ciphersuites (if other requisites are
 * enabled as well):
 *      MBEDTLS_TLS_ECDH_ECDSA_WITH_RC4_128_SHA
 *      MBEDTLS_TLS_ECDH_RSA_WITH_RC4_128_SHA
 *      MBEDTLS_TLS_ECDHE_ECDSA_WITH_RC4_128_SHA
 *      MBEDTLS_TLS_ECDHE_RSA_WITH_RC4_128_SHA
 *      MBEDTLS_TLS_ECDHE_PSK_WITH_RC4_128_SHA
 *      MBEDTLS_TLS_DHE_PSK_WITH_RC4_128_SHA
 *      MBEDTLS_TLS_RSA_WITH_RC4_128_SHA
 *      MBEDTLS_TLS_RSA_WITH_RC4_128_MD5
 *      MBEDTLS_TLS_RSA_PSK_WITH_RC4_128_SHA
 *      MBEDTLS_TLS_PSK_WITH_RC4_128_SHA
 */
#define MBEDTLS_ARC4_C

/**
 * \def MBEDTLS_ASN1_PARSE_C
 *
 * Enable the generic ASN1 parser.
 *
 * Module:  library/asn1.c
 * Caller:  library/x509.c
 *          library/dhm.c
 *          library/pkcs12.c
 *          library/pkcs5.c
 *          library/pkparse.c
 */
#define MBEDTLS_ASN1_PARSE_C

/**
 * \def MBEDTLS_ASN1_WRITE_C
 *
 * Enable the generic ASN1 writer.
 *
 * Module:  library/asn1write.c
 * Caller:  library/ecdsa.c
 *          library/pkwrite.c
 *          library/x509_create.c
 *          library/x509write_crt.c
 *          library/mbedtls_x509write_csr.c
 */
#define MBEDTLS_ASN1_WRITE_C

/**
 * \def MBEDTLS_BASE64_C
 *
 * Enable the Base64 module.
 *
 * Module:  library/base64.c
 * Caller:  library/pem.c
 *
 * This module is required for PEM support (required by X.509).
 */
#define MBEDTLS_BASE64_C

/**
 * \def MBEDTLS_BIGNUM_C
 *
 * Enable the multi-precision integer library.
 *
 * Module:  library/bignum.c
 * Caller:  library/dhm.c
 *          library/ecp.c
 *          library/ecdsa.c
 *          library/rsa.c
 *          library/ssl_tls.c
 *
 * This module is required for RSA, DHM and ECC (ECDH, ECDSA) support.
 */
#define MBEDTLS_BIGNUM_C

/**
 * \def MBEDTLS_BLOWFISH_C
 *
 * Enable the Blowfish block cipher.
 *
 * Module:  library/blowfish.c
 */
#define MBEDTLS_BLOWFISH_C

/**
 * \def MBEDTLS_CAMELLIA_C
 *
 * Enable the Camellia block cipher.
 *
 * Module:  library/camellia.c
 * Caller:  library/ssl_tls.c
 *
 * This module enables the following ciphersuites (if other requisites are
 * enabled as well):
 *      MBEDTLS_TLS_ECDH_ECDSA_WITH_CAMELLIA_128_CBC_SHA256
 *      MBEDTLS_TLS_ECDH_ECDSA_WITH_CAMELLIA_256_CBC_SHA384
 *      MBEDTLS_TLS_ECDH_RSA_WITH_CAMELLIA_128_CBC_SHA256
 *      MBEDTLS_TLS_ECDH_RSA_WITH_CAMELLIA_256_CBC_SHA384
 *      MBEDTLS_TLS_ECDH_ECDSA_WITH_CAMELLIA_128_GCM_SHA256
 *      MBEDTLS_TLS_ECDH_ECDSA_WITH_CAMELLIA_256_GCM_SHA384
 *      MBEDTLS_TLS_ECDH_RSA_WITH_CAMELLIA_128_GCM_SHA256
 *      MBEDTLS_TLS_ECDH_RSA_WITH_CAMELLIA_256_GCM_SHA384
 *      MBEDTLS_TLS_ECDHE_ECDSA_WITH_CAMELLIA_256_GCM_SHA384
 *      MBEDTLS_TLS_ECDHE_RSA_WITH_CAMELLIA_256_GCM_SHA384
 *      MBEDTLS_TLS_DHE_RSA_WITH_CAMELLIA_256_GCM_SHA384
 *      MBEDTLS_TLS_ECDHE_ECDSA_WITH_CAMELLIA_256_CBC_SHA384
 *      MBEDTLS_TLS_ECDHE_RSA_WITH_CAMELLIA_256_CBC_SHA384
 *      MBEDTLS_TLS_DHE_RSA_WITH_CAMELLIA_256_CBC_SHA256
 *      MBEDTLS_TLS_DHE_RSA_WITH_CAMELLIA_256_CBC_SHA
 *      MBEDTLS_TLS_ECDHE_ECDSA_WITH_CAMELLIA_128_GCM_SHA256
 *      MBEDTLS_TLS_ECDHE_RSA_WITH_CAMELLIA_128_GCM_SHA256
 *      MBEDTLS_TLS_DHE_RSA_WITH_CAMELLIA_128_GCM_SHA256
 *      MBEDTLS_TLS_ECDHE_ECDSA_WITH_CAMELLIA_128_CBC_SHA256
 *      MBEDTLS_TLS_ECDHE_RSA_WITH_CAMELLIA_128_CBC_SHA256
 *      MBEDTLS_TLS_DHE_RSA_WITH_CAMELLIA_128_CBC_SHA256
 *      MBEDTLS_TLS_DHE_RSA_WITH_CAMELLIA_128_CBC_SHA
 *      MBEDTLS_TLS_DHE_PSK_WITH_CAMELLIA_256_GCM_SHA384
 *      MBEDTLS_TLS_ECDHE_PSK_WITH_CAMELLIA_256_CBC_SHA384
 *      MBEDTLS_TLS_DHE_PSK_WITH_CAMELLIA_256_CBC_SHA384
 *      MBEDTLS_TLS_DHE_PSK_WITH_CAMELLIA_128_GCM_SHA256
 *      MBEDTLS_TLS_DHE_PSK_WITH_CAMELLIA_128_CBC_SHA256
 *      MBEDTLS_TLS_ECDHE_PSK_WITH_CAMELLIA_128_CBC_SHA256
 *      MBEDTLS_TLS_RSA_WITH_CAMELLIA_256_GCM_SHA384
 *      MBEDTLS_TLS_RSA_WITH_CAMELLIA_256_CBC_SHA256
 *      MBEDTLS_TLS_RSA_WITH_CAMELLIA_256_CBC_SHA
 *      MBEDTLS_TLS_RSA_WITH_CAMELLIA_128_GCM_SHA256
 *      MBEDTLS_TLS_RSA_WITH_CAMELLIA_128_CBC_SHA256
 *      MBEDTLS_TLS_RSA_WITH_CAMELLIA_128_CBC_SHA
 *      MBEDTLS_TLS_RSA_PSK_WITH_CAMELLIA_256_GCM_SHA384
 *      MBEDTLS_TLS_RSA_PSK_WITH_CAMELLIA_256_CBC_SHA384
 *      MBEDTLS_TLS_RSA_PSK_WITH_CAMELLIA_128_GCM_SHA256
 *      MBEDTLS_TLS_RSA_PSK_WITH_CAMELLIA_128_CBC_SHA256
 *      MBEDTLS_TLS_PSK_WITH_CAMELLIA_256_GCM_SHA384
 *      MBEDTLS_TLS_PSK_WITH_CAMELLIA_256_CBC_SHA384
 *      MBEDTLS_TLS_PSK_WITH_CAMELLIA_128_GCM_SHA256
 *      MBEDTLS_TLS_PSK_WITH_CAMELLIA_128_CBC_SHA256
 */
#define MBEDTLS_CAMELLIA_C

/**
 * \def MBEDTLS_CCM_C
 *
 * Enable the Counter with CBC-MAC (CCM) mode for 128-bit block cipher.
 *
 * Module:  library/ccm.c
 *
 * Requires: MBEDTLS_AES_C or MBEDTLS_CAMELLIA_C
 *
 * This module enables the AES-CCM ciphersuites, if other requisites are
 * enabled as well.
 */
#define MBEDTLS_CCM_C

/**
 * \def MBEDTLS_CERTS_C
 *
 * Enable the test certificates.
 *
 * Module:  library/certs.c
 * Caller:
 *
 * This module is used for testing (ssl_client/server).
 */
#define MBEDTLS_CERTS_C

/**
 * \def MBEDTLS_CIPHER_C
 *
 * Enable the generic cipher layer.
 *
 * Module:  library/cipher.c
 * Caller:  library/ssl_tls.c
 *
 * Uncomment to enable generic cipher wrappers.
 */
#define MBEDTLS_CIPHER_C

/**
 * \def MBEDTLS_CTR_DRBG_C
 *
 * Enable the CTR_DRBG AES-256-based random generator.
 *
 * Module:  library/ctr_drbg.c
 * Caller:
 *
 * Requires: MBEDTLS_AES_C
 *
 * This module provides the CTR_DRBG AES-256 random number generator.
 */
#define MBEDTLS_CTR_DRBG_C

/**
 * \def MBEDTLS_DEBUG_C
 *
 * Enable the debug functions.
 *
 * Module:  library/debug.c
 * Caller:  library/ssl_cli.c
 *          library/ssl_srv.c
 *          library/ssl_tls.c
 *
 * This module provides debugging functions.
 */
#define MBEDTLS_DEBUG_C

/**
 * \def MBEDTLS_DES_C
 *
 * Enable the DES block cipher.
 *
 * Module:  library/des.c
 * Caller:  library/pem.c
 *          library/ssl_tls.c
 *
 * This module enables the following ciphersuites (if other requisites are
 * enabled as well):
 *      MBEDTLS_TLS_ECDH_ECDSA_WITH_3DES_EDE_CBC_SHA
 *      MBEDTLS_TLS_ECDH_RSA_WITH_3DES_EDE_CBC_SHA
 *      MBEDTLS_TLS_ECDHE_ECDSA_WITH_3DES_EDE_CBC_SHA
 *      MBEDTLS_TLS_ECDHE_RSA_WITH_3DES_EDE_CBC_SHA
 *      MBEDTLS_TLS_DHE_RSA_WITH_3DES_EDE_CBC_SHA
 *      MBEDTLS_TLS_ECDHE_PSK_WITH_3DES_EDE_CBC_SHA
 *      MBEDTLS_TLS_DHE_PSK_WITH_3DES_EDE_CBC_SHA
 *      MBEDTLS_TLS_RSA_WITH_3DES_EDE_CBC_SHA
 *      MBEDTLS_TLS_RSA_PSK_WITH_3DES_EDE_CBC_SHA
 *      MBEDTLS_TLS_PSK_WITH_3DES_EDE_CBC_SHA
 *
 * PEM_PARSE uses DES/3DES for decrypting encrypted keys.
 */
#define MBEDTLS_DES_C

/**
 * \def MBEDTLS_DHM_C
 *
 * Enable the Diffie-Hellman-Merkle module.
 *
 * Module:  library/dhm.c
 * Caller:  library/ssl_cli.c
 *          library/ssl_srv.c
 *
 * This module is used by the following key exchanges:
 *      DHE-RSA, DHE-PSK
 */
#define MBEDTLS_DHM_C

/**
 * \def MBEDTLS_ECDH_C
 *
 * Enable the elliptic curve Diffie-Hellman library.
 *
 * Module:  library/ecdh.c
 * Caller:  library/ssl_cli.c
 *          library/ssl_srv.c
 *
 * This module is used by the following key exchanges:
 *      ECDHE-ECDSA, ECDHE-RSA, DHE-PSK
 *
 * Requires: MBEDTLS_ECP_C
 */
#define MBEDTLS_ECDH_C

/**
 * \def MBEDTLS_ECDSA_C
 *
 * Enable the elliptic curve DSA library.
 *
 * Module:  library/ecdsa.c
 * Caller:
 *
 * This module is used by the following key exchanges:
 *      ECDHE-ECDSA
 *
 * Requires: MBEDTLS_ECP_C, MBEDTLS_ASN1_WRITE_C, MBEDTLS_ASN1_PARSE_C
 */
#define MBEDTLS_ECDSA_C


/**
 * \def MBEDTLS_ECP_C
 *
 * Enable the elliptic curve over GF(p) library.
 *
 * Module:  library/ecp.c
 * Caller:  library/ecdh.c
 *          library/ecdsa.c
 *          library/ecjpake.c
 *
 * Requires: MBEDTLS_BIGNUM_C and at least one MBEDTLS_ECP_DP_XXX_ENABLED
 */
#define MBEDTLS_ECP_C

/**
 * \def MBEDTLS_ENTROPY_C
 *
 * Enable the platform-specific entropy code.
 *
 * Module:  library/entropy.c
 * Caller:
 *
 * Requires: MBEDTLS_SHA512_C or MBEDTLS_SHA256_C
 *
 * This module provides a generic entropy pool
 */
#define MBEDTLS_ENTROPY_C

/**
 * \def MBEDTLS_ERROR_C
 *
 * Enable error code to error string conversion.
 *
 * Module:  library/error.c
 * Caller:
 *
 * This module enables mbedtls_strerror().
 */
#define MBEDTLS_ERROR_C

/**
 * \def MBEDTLS_GCM_C
 *
 * Enable the Galois/Counter Mode (GCM) for AES.
 *
 * Module:  library/gcm.c
 *
 * Requires: MBEDTLS_AES_C or MBEDTLS_CAMELLIA_C
 *
 * This module enables the AES-GCM and CAMELLIA-GCM ciphersuites, if other
 * requisites are enabled as well.
 */
#define MBEDTLS_GCM_C


/**
 * \def MBEDTLS_HMAC_DRBG_C
 *
 * Enable the HMAC_DRBG random generator.
 *
 * Module:  library/hmac_drbg.c
 * Caller:
 *
 * Requires: MBEDTLS_MD_C
 *
 * Uncomment to enable the HMAC_DRBG random number geerator.
 */
#define MBEDTLS_HMAC_DRBG_C

/**
 * \def MBEDTLS_MD_C
 *
 * Enable the generic message digest layer.
 *
 * Module:  library/mbedtls_md.c
 * Caller:
 *
 * Uncomment to enable generic message digest wrappers.
 */
#define MBEDTLS_MD_C


/**
 * \def MBEDTLS_MD5_C
 *
 * Enable the MD5 hash algorithm.
 *
 * Module:  library/mbedtls_md5.c
 * Caller:  library/mbedtls_md.c
 *          library/pem.c
 *          library/ssl_tls.c
 *
 * This module is required for SSL/TLS and X.509.
 * PEM_PARSE uses MD5 for decrypting encrypted keys.
 */
#define MBEDTLS_MD5_C


/**
 * \def MBEDTLS_NET_C
 *
 * Enable the TCP and UDP over IPv6/IPv4 networking routines.
 *
 * \note This module only works on POSIX/Unix (including Linux, BSD and OS X)
 * and Windows. For other platforms, you'll want to disable it, and write your
 * own networking callbacks to be passed to \c mbedtls_ssl_set_bio().
 *
 * \note See also our Knowledge Base article about porting to a new
 * environment:
 * https://tls.mbed.org/kb/how-to/how-do-i-port-mbed-tls-to-a-new-environment-OS
 *
 * Module:  library/net.c
 *
 * This module provides networking routines.
 */
#define MBEDTLS_NET_C

/**
 * \def MBEDTLS_OID_C
 *
 * Enable the OID database.
 *
 * Module:  library/oid.c
 * Caller:  library/asn1write.c
 *          library/pkcs5.c
 *          library/pkparse.c
 *          library/pkwrite.c
 *          library/rsa.c
 *          library/x509.c
 *          library/x509_create.c
 *          library/mbedtls_x509_crl.c
 *          library/mbedtls_x509_crt.c
 *          library/mbedtls_x509_csr.c
 *          library/x509write_crt.c
 *          library/mbedtls_x509write_csr.c
 *
 * This modules translates between OIDs and internal values.
 */
#define MBEDTLS_OID_C

/**
 * \def MBEDTLS_PADLOCK_C
 *
 * Enable VIA Padlock support on x86.
 *
 * Module:  library/padlock.c
 * Caller:  library/aes.c
 *
 * Requires: MBEDTLS_HAVE_ASM
 *
 * This modules adds support for the VIA PadLock on x86.
 */
#define MBEDTLS_PADLOCK_C

/**
 * \def MBEDTLS_PEM_PARSE_C
 *
 * Enable PEM decoding / parsing.
 *
 * Module:  library/pem.c
 * Caller:  library/dhm.c
 *          library/pkparse.c
 *          library/mbedtls_x509_crl.c
 *          library/mbedtls_x509_crt.c
 *          library/mbedtls_x509_csr.c
 *
 * Requires: MBEDTLS_BASE64_C
 *
 * This modules adds support for decoding / parsing PEM files.
 */
#define MBEDTLS_PEM_PARSE_C

/**
 * \def MBEDTLS_PEM_WRITE_C
 *
 * Enable PEM encoding / writing.
 *
 * Module:  library/pem.c
 * Caller:  library/pkwrite.c
 *          library/x509write_crt.c
 *          library/mbedtls_x509write_csr.c
 *
 * Requires: MBEDTLS_BASE64_C
 *
 * This modules adds support for encoding / writing PEM files.
 */
#define MBEDTLS_PEM_WRITE_C

/**
 * \def MBEDTLS_PK_C
 *
 * Enable the generic public (asymetric) key layer.
 *
 * Module:  library/pk.c
 * Caller:  library/ssl_tls.c
 *          library/ssl_cli.c
 *          library/ssl_srv.c
 *
 * Requires: MBEDTLS_RSA_C or MBEDTLS_ECP_C
 *
 * Uncomment to enable generic public key wrappers.
 */
#define MBEDTLS_PK_C

/**
 * \def MBEDTLS_PK_PARSE_C
 *
 * Enable the generic public (asymetric) key parser.
 *
 * Module:  library/pkparse.c
 * Caller:  library/mbedtls_x509_crt.c
 *          library/mbedtls_x509_csr.c
 *
 * Requires: MBEDTLS_PK_C
 *
 * Uncomment to enable generic public key parse functions.
 */
#define MBEDTLS_PK_PARSE_C

/**
 * \def MBEDTLS_PK_WRITE_C
 *
 * Enable the generic public (asymetric) key writer.
 *
 * Module:  library/pkwrite.c
 * Caller:  library/x509write.c
 *
 * Requires: MBEDTLS_PK_C
 *
 * Uncomment to enable generic public key write functions.
 */
#define MBEDTLS_PK_WRITE_C

/**
 * \def MBEDTLS_PKCS5_C
 *
 * Enable PKCS#5 functions.
 *
 * Module:  library/pkcs5.c
 *
 * Requires: MBEDTLS_MD_C
 *
 * This module adds support for the PKCS#5 functions.
 */
#define MBEDTLS_PKCS5_C

/**
 * \def MBEDTLS_PKCS12_C
 *
 * Enable PKCS#12 PBE functions.
 * Adds algorithms for parsing PKCS#8 encrypted private keys
 *
 * Module:  library/pkcs12.c
 * Caller:  library/pkparse.c
 *
 * Requires: MBEDTLS_ASN1_PARSE_C, MBEDTLS_CIPHER_C, MBEDTLS_MD_C
 * Can use:  MBEDTLS_ARC4_C
 *
 * This module enables PKCS#12 functions.
 */
#define MBEDTLS_PKCS12_C

/**
 * \def MBEDTLS_PLATFORM_C
 *
 * Enable the platform abstraction layer that allows you to re-assign
 * functions like calloc(), free(), snprintf(), printf(), fprintf(), exit().
 *
 * Enabling MBEDTLS_PLATFORM_C enables to use of MBEDTLS_PLATFORM_XXX_ALT
 * or MBEDTLS_PLATFORM_XXX_MACRO directives, allowing the functions mentioned
 * above to be specified at runtime or compile time respectively.
 *
 * \note This abstraction layer must be enabled on Windows (including MSYS2)
 * as other module rely on it for a fixed snprintf implementation.
 *
 * Module:  library/platform.c
 * Caller:  Most other .c files
 *
 * This module enables abstraction of common (libc) functions.
 */
#define MBEDTLS_PLATFORM_C

/**
 * \def MBEDTLS_RIPEMD160_C
 *
 * Enable the RIPEMD-160 hash algorithm.
 *
 * Module:  library/mbedtls_ripemd160.c
 * Caller:  library/mbedtls_md.c
 *
 */
#define MBEDTLS_RIPEMD160_C

/**
 * \def MBEDTLS_RSA_C
 *
 * Enable the RSA public-key cryptosystem.
 *
 * Module:  library/rsa.c
 * Caller:  library/ssl_cli.c
 *          library/ssl_srv.c
 *          library/ssl_tls.c
 *          library/x509.c
 *
 * This module is used by the following key exchanges:
 *      RSA, DHE-RSA, ECDHE-RSA, RSA-PSK
 *
 * Requires: MBEDTLS_BIGNUM_C, MBEDTLS_OID_C
 */
#define MBEDTLS_RSA_C

/**
 * \def MBEDTLS_SHA1_C
 *
 * Enable the SHA1 cryptographic hash algorithm.
 *
 * Module:  library/mbedtls_sha1.c
 * Caller:  library/mbedtls_md.c
 *          library/ssl_cli.c
 *          library/ssl_srv.c
 *          library/ssl_tls.c
 *          library/x509write_crt.c
 *
 * This module is required for SSL/TLS and SHA1-signed certificates.
 */
#define MBEDTLS_SHA1_C

/**
 * \def MBEDTLS_SHA256_C
 *
 * Enable the SHA-224 and SHA-256 cryptographic hash algorithms.
 *
 * Module:  library/mbedtls_sha256.c
 * Caller:  library/entropy.c
 *          library/mbedtls_md.c
 *          library/ssl_cli.c
 *          library/ssl_srv.c
 *          library/ssl_tls.c
 *
 * This module adds support for SHA-224 and SHA-256.
 * This module is required for the SSL/TLS 1.2 PRF function.
 */
#define MBEDTLS_SHA256_C

/**
 * \def MBEDTLS_SHA512_C
 *
 * Enable the SHA-384 and SHA-512 cryptographic hash algorithms.
 *
 * Module:  library/mbedtls_sha512.c
 * Caller:  library/entropy.c
 *          library/mbedtls_md.c
 *          library/ssl_cli.c
 *          library/ssl_srv.c
 *
 * This module adds support for SHA-384 and SHA-512.
 */
#define MBEDTLS_SHA512_C

/**
 * \def MBEDTLS_SSL_CACHE_C
 *
 * Enable simple SSL cache implementation.
 *
 * Module:  library/ssl_cache.c
 * Caller:
 *
 * Requires: MBEDTLS_SSL_CACHE_C
 */
#define MBEDTLS_SSL_CACHE_C

/**
 * \def MBEDTLS_SSL_COOKIE_C
 *
 * Enable basic implementation of DTLS cookies for hello verification.
 *
 * Module:  library/ssl_cookie.c
 * Caller:
 */
#define MBEDTLS_SSL_COOKIE_C

/**
 * \def MBEDTLS_SSL_TICKET_C
 *
 * Enable an implementation of TLS server-side callbacks for session tickets.
 *
 * Module:  library/ssl_ticket.c
 * Caller:
 *
 * Requires: MBEDTLS_CIPHER_C
 */
#define MBEDTLS_SSL_TICKET_C

/**
 * \def MBEDTLS_SSL_CLI_C
 *
 * Enable the SSL/TLS client code.
 *
 * Module:  library/ssl_cli.c
 * Caller:
 *
 * Requires: MBEDTLS_SSL_TLS_C
 *
 * This module is required for SSL/TLS client support.
 */
#define MBEDTLS_SSL_CLI_C

/**
 * \def MBEDTLS_SSL_SRV_C
 *
 * Enable the SSL/TLS server code.
 *
 * Module:  library/ssl_srv.c
 * Caller:
 *
 * Requires: MBEDTLS_SSL_TLS_C
 *
 * This module is required for SSL/TLS server support.
 */
#define MBEDTLS_SSL_SRV_C

/**
 * \def MBEDTLS_SSL_TLS_C
 *
 * Enable the generic SSL/TLS code.
 *
 * Module:  library/ssl_tls.c
 * Caller:  library/ssl_cli.c
 *          library/ssl_srv.c
 *
 * Requires: MBEDTLS_CIPHER_C, MBEDTLS_MD_C
 *           and at least one of the MBEDTLS_SSL_PROTO_XXX defines
 *
 * This module is required for SSL/TLS.
 */
#define MBEDTLS_SSL_TLS_C


/**
 * \def MBEDTLS_TIMING_C
 *
 * Enable the semi-portable timing interface.
 *
 * \note The provided implementation only works on POSIX/Unix (including Linux,
 * BSD and OS X) and Windows. On other platforms, you can either disable that
 * module and provide your own implementations of the callbacks needed by
 * \c mbedtls_ssl_set_timer_cb() for DTLS, or leave it enabled and provide
 * your own implementation of the whole module by setting
 * \c MBEDTLS_TIMING_ALT in the current file.
 *
 * \note See also our Knowledge Base article about porting to a new
 * environment:
 * https://tls.mbed.org/kb/how-to/how-do-i-port-mbed-tls-to-a-new-environment-OS
 *
 * Module:  library/timing.c
 * Caller:  library/havege.c
 *
 * This module is used by the HAVEGE random number generator.
 */
#define MBEDTLS_TIMING_C

/**
 * \def MBEDTLS_VERSION_C
 *
 * Enable run-time version information.
 *
 * Module:  library/version.c
 *
 * This module provides run-time version information.
 */
#define MBEDTLS_VERSION_C

/**
 * \def MBEDTLS_X509_USE_C
 *
 * Enable X.509 core for using certificates.
 *
 * Module:  library/x509.c
 * Caller:  library/mbedtls_x509_crl.c
 *          library/mbedtls_x509_crt.c
 *          library/mbedtls_x509_csr.c
 *
 * Requires: MBEDTLS_ASN1_PARSE_C, MBEDTLS_BIGNUM_C, MBEDTLS_OID_C,
 *           MBEDTLS_PK_PARSE_C
 *
 * This module is required for the X.509 parsing modules.
 */
#define MBEDTLS_X509_USE_C

/**
 * \def MBEDTLS_X509_CRT_PARSE_C
 *
 * Enable X.509 certificate parsing.
 *
 * Module:  library/mbedtls_x509_crt.c
 * Caller:  library/ssl_cli.c
 *          library/ssl_srv.c
 *          library/ssl_tls.c
 *
 * Requires: MBEDTLS_X509_USE_C
 *
 * This module is required for X.509 certificate parsing.
 */
#define MBEDTLS_X509_CRT_PARSE_C

/**
 * \def MBEDTLS_X509_CRL_PARSE_C
 *
 * Enable X.509 CRL parsing.
 *
 * Module:  library/mbedtls_x509_crl.c
 * Caller:  library/mbedtls_x509_crt.c
 *
 * Requires: MBEDTLS_X509_USE_C
 *
 * This module is required for X.509 CRL parsing.
 */
#define MBEDTLS_X509_CRL_PARSE_C

/**
 * \def MBEDTLS_X509_CSR_PARSE_C
 *
 * Enable X.509 Certificate Signing Request (CSR) parsing.
 *
 * Module:  library/mbedtls_x509_csr.c
 * Caller:  library/x509_crt_write.c
 *
 * Requires: MBEDTLS_X509_USE_C
 *
 * This module is used for reading X.509 certificate request.
 */
#define MBEDTLS_X509_CSR_PARSE_C

/**
 * \def MBEDTLS_X509_CREATE_C
 *
 * Enable X.509 core for creating certificates.
 *
 * Module:  library/x509_create.c
 *
 * Requires: MBEDTLS_BIGNUM_C, MBEDTLS_OID_C, MBEDTLS_PK_WRITE_C
 *
 * This module is the basis for creating X.509 certificates and CSRs.
 */
#define MBEDTLS_X509_CREATE_C

/**
 * \def MBEDTLS_X509_CRT_WRITE_C
 *
 * Enable creating X.509 certificates.
 *
 * Module:  library/x509_crt_write.c
 *
 * Requires: MBEDTLS_X509_CREATE_C
 *
 * This module is required for X.509 certificate creation.
 */
#define MBEDTLS_X509_CRT_WRITE_C

/**
 * \def MBEDTLS_X509_CSR_WRITE_C
 *
 * Enable creating X.509 Certificate Signing Requests (CSR).
 *
 * Module:  library/x509_csr_write.c
 *
 * Requires: MBEDTLS_X509_CREATE_C
 *
 * This module is required for X.509 certificate request writing.
 */
#define MBEDTLS_X509_CSR_WRITE_C

/**
 * \def MBEDTLS_XTEA_C
 *
 * Enable the XTEA block cipher.
 *
 * Module:  library/xtea.c
 * Caller:
 */
#define MBEDTLS_XTEA_C

/* \} name SECTION: mbed TLS modules */


#if defined(YOTTA_CFG_MBEDTLS_USER_CONFIG_FILE)
#include YOTTA_CFG_MBEDTLS_USER_CONFIG_FILE
#elif defined(MBEDTLS_USER_CONFIG_FILE)
#include MBEDTLS_USER_CONFIG_FILE
#endif

#include "check_config.h"

#endif /* MBEDTLS_CONFIG_H */
