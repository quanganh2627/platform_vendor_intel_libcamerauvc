# Copyright (c) 2009-2010 Wind River Systems, Inc.
ifeq ($(USE_CAMERA_STUB),false)
#
# libcamera
#

LOCAL_PATH := $(call my-dir)

LIBCAMERA_TOP := $(LOCAL_PATH)

include $(CLEAR_VARS)

$(info Intel MRST Camera Hardware Interface)

LOCAL_MODULE := libcamera

LOCAL_SHARED_LIBRARIES := \
	libui \
	libutils \
	libcutils \
	libdl \
	libbinder \
	libsensor \
	libisphal \
	libci \
	libadvci

LOCAL_SRC_FILES += \
	CameraHardware.cpp \
	IntelCamera.cpp

LOCAL_CFLAGS += -DDEBUG_LEVEL=2

LOCAL_C_INCLUDES += \
	frameworks/base/include/ui \
	hardware/intel/libci/include \
	hardware/intel/libadvci/include

LOCAL_STATIC_LIBRARIES +=

include $(BUILD_SHARED_LIBRARY)

# ci-app test program
include $(CLEAR_VARS)
LOCAL_MODULE := ci-app

LOCAL_SHARED_LIBRARIES := \
	libui \
	libutils \
	libcutils \
	libisphal \
	libci \
	libadvci

LOCAL_SRC_FILES += ci-app.c

LOCAL_CFLAGS += -DLOG_TAG=\"CI-APP\" -DBOOL_ENABLE

LOCAL_C_INCLUDES += \
	hardware/intel/libci/include \
	hardware/intel/libadvci/include

include $(BUILD_EXECUTABLE)

endif
