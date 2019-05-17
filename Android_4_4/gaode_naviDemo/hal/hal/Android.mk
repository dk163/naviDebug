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

$(warning "=================PLATFORM_SDK_VERSION=$(PLATFORM_SDK_VERSION)")

#navi add by kang
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := gps.default
LOCAL_MODULE_TAGS := optional

LOCAL_SHARED_LIBRARIES := \
	liblog \
	libcutils

LOCAL_SRC_FILES += \
    common.cpp \
    aip_parser.cpp \
    gps_thread.cpp \
    aip_hw_if.cpp \
    aip_serial.cpp \
    aip_log.cpp

LOCAL_CFLAGS += \
    -DUNIX_API \
    -DANDROID_BUILD \
    -DAIP_MODULE \
    -DOUPUT_SPEED \
    -DSPEED_PWM

LOCAL_LDFLAGS += -Wl,--gc-sections \
               -Wl,--no-wchar-size-warning

LOCAL_LDLIBS := -llog

## Includes
#LOCAL_C_INCLUDES:= \
#   $(LOCAL_PATH)
#   $(LOCAL_PATH)/hardware/libhardware/include \
#   $(LOCAL_PATH)/system/core/include \

LOCAL_PRELINK_MODULE := false

ifeq ($(TARGET_2ND_ARCH),)
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
else
LOCAL_MODULE_RELATIVE_PATH := hw
endif

include $(BUILD_SHARED_LIBRARY)

NOTICE-TARGET-STATIC_LIBRARIES-%:
