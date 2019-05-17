##############################################################################
#
# Copyright (C) u-blox AG
# u-blox AG, Thalwil, Switzerland
#
# All rights reserved.
#
# Permission to use, copy, modify, and distribute this software for any
# purpose without fee is hereby granted, provided that this entire notice
# is included in all copies of any software which is or includes a copy
# or modification of this software and in all copies of the supporting
# documentation for such software.
#
# THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
# WARRANTY. IN PARTICULAR, NEITHER THE AUTHOR NOR U-BLOX MAKES ANY
# REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY
# OF THIS SOFTWARE OR ITS FITNESS FOR ANY PARTICULAR PURPOSE.
#
###############################################################################
#
# Project: Android GNSS Driver
#
###############################################################################
# $Id: Android.mk 114051 2016-04-20 15:30:31Z fabio.robbiani $
# $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/hal/Android.mk $
###############################################################################

# Android v2.2 does not yet support SUPL/AGPS/RIL/NI interfaces
ifneq ($(PLATFORM_SDK_VERSION),8)
SUPL_ENABLED := 1
endif

LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_PARSER_PATH=../parser
LOCAL_SUPL_PATH=../supl
LOCAL_ASN1_PATH=../asn1

LOCAL_SHARED_LIBRARIES := \
	liblog \
	libcutils

LOCAL_C_INCLUDES += \
	$(LOCAL_PATH) \
	$(LOCAL_PATH)/$(LOCAL_PARSER_PATH)

PARSER_SRC_FILES := \
	$(LOCAL_PARSER_PATH)/database.cpp \
	$(LOCAL_PARSER_PATH)/parserbuffer.cpp \
	$(LOCAL_PARSER_PATH)/protocolnmea.cpp \
	$(LOCAL_PARSER_PATH)/protocolubx.cpp \
	$(LOCAL_PARSER_PATH)/protocolunknown.cpp

LOCAL_SRC_FILES := \
	$(PARSER_SRC_FILES) \
	gps_thread.cpp \
	ubx_cfg.cpp \
	ubx_localDb.cpp \
	ubx_log.cpp \
	ubx_moduleIf.cpp \
	ubx_rilIf.cpp \
	ubx_timer.cpp \
	ubx_serial.cpp \
	ubx_tcpServer.cpp \
	ubx_udpServer.cpp \
	ubx_xtraIf.cpp \
	ubxgpsstate.cpp

LOCAL_CFLAGS := \
	-DPLATFORM_SDK_VERSION=$(PLATFORM_SDK_VERSION) \
	-DUNIX_API \
	-DANDROID_BUILD

# Uncomment the following lines to enable
# the local UDP tunnel for communication
# with the receiver. For debugging use only!
# LOCAL_CFLAGS += \
#	-DUDP_SERVER_PORT=46434

# Uncomment the following lines to enable
# the local TCP tunnel for communication
# with the receiver. For debugging use only!
# LOCAL_CFLAGS += \
#	-DTCP_SERVER_PORT=42434


# Uncomment the following linnes to enable
# debugging messages in logcat for the
# following message types after they have
# been read from the receiver and parsed
# by the parser:
# * Sent and received UBX commands (hex)
# * Received NMEA messages (string)
# * Received unknown messages (hex)
#LOCAL_CFLAGS += \
#	-DMSG_UBX_LOG_OUTPUT \
#	-DMSG_NMEA_LOG_OUTPUT \
#	-DMSG_UNKNOWN_LOG_OUTPUT

# Uncomment the following lines to enable
# debugging messages for data written to
# the serial line and data read from the
# serial line (the same is valid for
# I2C logging):
# LOCAL_CFLAGS += \
#   -DUBX_SERIAL_EXTENSIVE_LOGGING

LOCAL_STATIC_LIBRARIES += libAgnss

# Additions for SUPL
ifeq ($(SUPL_ENABLED),1)
LOCAL_C_INCLUDES += external/openssl/include/
LOCAL_C_INCLUDES += external/openssl/
LOCAL_C_INCLUDES += external/

LOCAL_SHARED_LIBRARIES += libcrypto
LOCAL_SHARED_LIBRARIES += libssl
LOCAL_STATIC_LIBRARIES += libAsn1

LOCAL_C_INCLUDES += \
	$(LOCAL_PATH)/$(LOCAL_SUPL_PATH) \
	$(LOCAL_PATH)/$(LOCAL_ASN1_PATH)

SUPL_SOURCE_FILES := \
	$(LOCAL_SUPL_PATH)/rrlpdecod.cpp \
	$(LOCAL_SUPL_PATH)/rrlpmanager.cpp \
	$(LOCAL_SUPL_PATH)/suplSMmanager.cpp \
	$(LOCAL_SUPL_PATH)/upldecod.cpp \
	$(LOCAL_SUPL_PATH)/uplsend.cpp \
	$(LOCAL_SUPL_PATH)/supl_connection.cpp \
	ubx_agpsIf.cpp \
	ubx_niIf.cpp

LOCAL_SRC_FILES += $(SUPL_SOURCE_FILES)
LOCAL_CFLAGS += -DSUPL_ENABLED
# LOCAL_CFLAGS += -UNDEBUG

# Uncomment the line below for test with SUPL Test suite
#LOCAL_CFLAGS += -DSUPL_FQDN_SLP='"slp.rs.de"'

endif

LOCAL_MODULE := gps.default

LOCAL_MODULE_TAGS := eng

LOCAL_PRELINK_MODULE := false

# For multi-arch targets LOCAL_MODULE_RELATIVE_PATH must be used
# to make sure each build has its own destination
ifeq ($(TARGET_2ND_ARCH),)
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
else
LOCAL_MODULE_RELATIVE_PATH := hw
endif

include $(BUILD_SHARED_LIBRARY)

NOTICE-TARGET-STATIC_LIBRARIES-%:
