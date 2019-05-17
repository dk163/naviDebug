###############################################################################
#
# Copyright (C) u-blox AG
# u-blox AG, Thalwil, Switzerland
#
# All rights reserved.
#
# Permission to use, copy, modify, and distribute this software for any
# purpose without fee is hereby granted, provided that this entire notice is
# included in all copies of any software which is or includes a copy or
# modification of this software and in all copies of the supporting
# documentation for such software.
#
# THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
# WARRANTY. IN PARTICULAR, NEITHER THE AUTHOR NOR U-BLOX MAKES ANY
# REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY OF
# THIS SOFTWARE OR ITS FITNESS FOR ANY PARTICULAR PURPOSE.
#
###############################################################################
#
# Project: libMGA
# Purpose: Makefile to create statically linked libraries for Linux OS
#
###############################################################################
#
# This file was modified by u-blox

CFLAGS	?= -O2
WARNING_CFLAGS ?=  -Wall -W -Wdeclaration-after-statement
LDFLAGS ?=

LOCAL_CFLAGS = $(WARNING_CFLAGS) -Iinclude -D_FILE_OFFSET_BITS=64
LOCAL_LDFLAGS =

ifdef DEBUG
LOCAL_CFLAGS += -g3
endif

# MicroBlaze specific options:
# CFLAGS += -mno-xl-soft-mul -mxl-barrel-shift

# To compile on Plan9:
# CFLAGS += -D_BSD_EXTENSION


OBJS_CRYPTO=	src/aes.o		src/aesni.o		src/arc4.o		\
		src/asn1parse.o	src/asn1write.o	src/base64.o	\
		src/bignum.o	src/blowfish.o	src/camellia.o	\
		src/ccm.o		src/cipher.o	src/cipher_wrap.o	\
		src/ctr_drbg.o	src/des.o		src/dhm.o		\
		src/ecdh.o		src/ecdsa.o		src/ecjpake.o	\
		src/ecp.o						\
		src/ecp_curves.o	src/entropy.o	src/entropy_poll.o	\
		src/error.o		src/gcm.o		src/havege.o	\
		src/hmac_drbg.o	src/md.o		src/md2.o		\
		src/md4.o		src/md5.o		src/md_wrap.o	\
		src/memory_buffer_alloc.o		src/oid.o		\
		src/padlock.o	src/pem.o		src/pk.o		\
		src/pk_wrap.o	src/pkcs12.o	src/pkcs5.o		\
		src/pkparse.o	src/pkwrite.o	src/platform.o	\
		src/ripemd160.o	src/rsa.o		src/sha1.o		\
		src/sha256.o	src/sha512.o	src/threading.o	\
		src/timing.o	src/version.o			\
		src/version_features.o		src/xtea.o

OBJS_X509=	src/certs.o		src/pkcs11.o	src/x509.o		\
		src/x509_create.o	src/x509_crl.o	src/x509_crt.o	\
		src/x509_csr.o	src/x509write_crt.o	src/x509write_csr.o

OBJS_TLS=	src/debug.o		src/net.o		src/ssl_cache.o	\
		src/ssl_ciphersuites.o		src/ssl_cli.o	\
		src/ssl_cookie.o	src/ssl_srv.o	src/ssl_ticket.o	\
		src/ssl_tls.o

.SILENT:

.PHONY: all static shared clean


all: static

static: libmbedcrypto.a libmbedx509.a libmbedtls.a

# tls
libmbedtls.a: $(OBJS_TLS)
	echo "  AR    $@"
	$(AR) -rc bin/$@ $(OBJS_TLS)
	echo "  RL    $@"
	$(AR) -s bin/$@

# x509
libmbedx509.a: $(OBJS_X509)
	echo "  AR    $@"
	$(AR) -rc bin/$@ $(OBJS_X509)
	echo "  RL    $@"
	$(AR) -s bin/$@

# crypto
libmbedcrypto.a: $(OBJS_CRYPTO)
	echo "  AR    $@"
	$(AR) -rc bin/$@ $(OBJS_CRYPTO)
	echo "  RL    $@"
	$(AR) -s bin/$@

.c.o:
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) -c $< -o $@

clean:
ifndef WINDOWS
	rm -f src/*.o libmbed*
else
	del /Q /F src/*.o libmbed*
endif
