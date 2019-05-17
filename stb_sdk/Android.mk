
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := gps.default
LOCAL_MODULE_FILENAME := gps.default
LOCAL_MODULE_TAGS := optional

LOCAL_SRC_FILES += \
    autonavi_parser.cpp \
    gps_thread.cpp \
    stb_hw_if.cpp \
    rs232.c \
    $(wildcard $(LOCAL_PATH)/*.cpp) \
    $(wildcard $(LOCAL_PATH)/*.c)

LOCAL_CFLAGS += \
    -fno-short-enums \
    -D_ANDROID_ \
    -DHAVE_SYS_UIO_H \
    -x c++ \
    -fvisibility=hidden \
    -ffunction-sections \
    -fdata-sections \
    -Os \
    -shared \
    -fPIC \
    -pie

LOCAL_LDFLAGS += -Wl,--gc-sections \
                -Wl,--no-wchar-size-warning

LOCAL_LDLIBS := -llog

## Includes
LOCAL_C_INCLUDES:= \
    $(LOCAL_PATH) \
    $(LOCAL_PATH)/hardware/libhardware/include \
    $(LOCAL_PATH)/system/core/include \

LOCAL_PRELINK_MODULE := false

include $(BUILD_SHARED_LIBRARY)
