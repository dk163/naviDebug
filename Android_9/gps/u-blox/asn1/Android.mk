###############################################################################
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
# $Id$
# $HeadURL$
###############################################################################

LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

# LOCAL_CLANG:=true
# LOCAL_SANITIZE:=address

# For some targets it might be required to use sysv as hash style
# LOCAL_LDFLAGS += -Wl,--hash-style=sysv

LOCAL_C_INCLUDES += \
	$(LOCAL_PATH) 

LOCAL_SRC_FILES := \
	$(wildcard $(LOCAL_PATH)/*.c)

LOCAL_SRC_FILES := $(subst $(LOCAL_PATH)/,,${LOCAL_SRC_FILES})

LOCAL_MODULE := libAsn1

ifeq ($(strip $(LOCAL_SRC_FILES)),)
    $(error "The driver source files could not be found for ${LOCAL_MODULE}! Aborting...")
endif

LOCAL_MODULE_TAGS := eng
LOCAL_PROPRIETARY_MODULE := true

LOCAL_CFLAGS := -std=c99          \
                -DHAVE_TM_GMTOFF

LOCAL_PRELINK_MODULE := false

include $(BUILD_STATIC_LIBRARY)
