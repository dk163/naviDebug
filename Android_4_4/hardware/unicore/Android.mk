$(warning "=========modules========CUSTOM_GPS=$(CUSTOM_GPS)")
ifeq ($(CUSTOM_GPS),unicore)
LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

LOCAL_PRELINK_MODULE := false

#LOCAL_MODULE_RELATIVE_PATH := hw
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
LOCAL_SHARED_LIBRARIES := liblog libcutils libhardware

LOCAL_SRC_FILES := bd_gps.c
LOCAL_SRC_FILES += gps.c
LOCAL_C_FLAGS := -Wno-unused-parameter

LOCAL_MODULE := gps.$(TARGET_BOOTLOADER_BOARD_NAME)
LOCAL_MODULE_TAGS := optional

include $(BUILD_SHARED_LIBRARY)
endif