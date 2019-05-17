LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)
LOCAL_PRELINK_MODULE := false
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
LOCAL_SHARED_LIBRARIES := liblog libcutils libhardware
LOCAL_SRC_FILES := gps.c protocolnmea.c tbox_buffer.c main.c
LOCAL_MODULE := gps.tbox
LOCAL_MODULE_TAGS := eng
include $(BUILD_SHARED_LIBRARY)
