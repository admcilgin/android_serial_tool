LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
    serial.c

LOCAL_SHARED_LIBRARIES := \
    libhardware \
    libutils \
    libcutils

LOCAL_CFLAGS := -O2 -Wall -g
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE:= test_serial

include $(BUILD_EXECUTABLE)
