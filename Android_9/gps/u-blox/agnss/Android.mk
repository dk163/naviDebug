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

MGA_VERSION := $(shell cat ${LOCAL_PATH}/mga/src/libMga.h | grep "define LIBMGA_VERSION" | grep -oPe '(?<=").*(?=\")')
ifeq ($(MGA_VERSION), "")
$(error Was not able to read base version)
endif
ANDROID_DRIVER_VERSION := $(shell cat ${LOCAL_PATH}/../hal/version.h | grep "define BASE_VERSION" | grep -oPe '(?<=").*(?=\")')
ifeq ($(ANDROID_DRIVER_VERSION), "")
$(error Was not able to read base version)
endif

GIT_VERSION := $(shell git -C ${LOCAL_PATH}/../.. describe --abbrev=10 --dirty --always --tags)
$(info $$GIT_VERSION is [${GIT_VERSION}])

#Adding MGA user agent version to build
MGA_USER_AGENT := "libMga/${MGA_VERSION} (Android GNSS Driver ${ANDROID_DRIVER_VERSION}/${GIT_VERSION})"
$(info $$MGA_USER_AGENT is [${MGA_USER_AGENT}])
LOCAL_CFLAGS +=        -DMGA_USER_AGENT=\"$(MGA_USER_AGENT)\"


# LOCAL_CLANG:=true
# LOCAL_SANITIZE:=address

# For some targets it might be required to use sysv as hash style
# LOCAL_LDFLAGS += -Wl,--hash-style=sysv

LOCAL_C_INCLUDES :=                             \
    $(LOCAL_PATH)                               \
    $(LOCAL_PATH)/mga/src                       \
    $(LOCAL_PATH)/com                           \
    $(LOCAL_PATH)/func                          \
    $(LOCAL_PATH)/helper                        \
    $(LOCAL_PATH)/list                          \
    $(LOCAL_PATH)/storage

LOCAL_SRC_FILES :=                             \
    Agnss.cpp                                  \
    AssistNowLeg.cpp                           \
    AssistNowMga.cpp                           \
    com/ComThread.cpp                          \
    com/ThreadedPacketIf.cpp                   \
    func/FuncMngr.cpp                          \
    helper/helperFunctions.cpp                 \
    helper/UbxMsg.cpp                          \
    helper/UbxAidEphAlm.cpp                    \
    helper/UbxAidIni.cpp                       \
    helper/UbxCfgNavX5.cpp                     \
    helper/UbxMgaIniPosLlh.cpp                 \
    helper/UbxMgaIniTimeUtc.cpp                \
    helper/UbxMgaDbd.cpp                       \
    helper/UbxPollAidEphAlm.cpp                \
    helper/UbxPollMgaDbd.cpp                   \
    mga/src/libMga.cpp                         \
    storage/LockedStorage.cpp                  \
    storage/PositionHandler.cpp                \
    storage/TimeHandler.cpp

LOCAL_CFLAGS += -Wall -Wextra

LOCAL_PROPRIETARY_MODULE := true
LOCAL_MODULE := libAgnss

LOCAL_MODULE_TAGS := eng

LOCAL_PRELINK_MODULE := false

include $(BUILD_STATIC_LIBRARY)


