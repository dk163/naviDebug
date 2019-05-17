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
# $Id: Android.mk 114052 2016-04-20 15:35:15Z fabio.robbiani $
# $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/agnss/Android.mk $
###############################################################################

LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_C_INCLUDES :=                             \
    $(LOCAL_PATH)                               \
    $(LOCAL_PATH)/mga                           \
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
    mga/libMga.cpp                             \
    storage/LockedStorage.cpp                  \
    storage/PositionHandler.cpp                \
    storage/TimeHandler.cpp

LOCAL_CFLAGS += -Wall -Wextra


LOCAL_MODULE := libAgnss

LOCAL_MODULE_TAGS := eng

LOCAL_PRELINK_MODULE := false

include $(BUILD_STATIC_LIBRARY)

