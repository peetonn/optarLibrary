LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

OPENCV_CAMERA_MODULES:=off
OPENCV_INSTALL_MODULES:=on
OPENCV_LIB_TYPE:=SHARED
include ../thirdparty/OpenCV-android-sdk/sdk/native/jni/OpenCV.mk

LOCAL_MODULE    := optar
LOCAL_CFLAGS    := -std=c++11
LOCAL_SRC_FILES := ../../source/optar.cpp ../../source/logging.cpp
LOCAL_C_INCLUDES += ../../source ../thirdparty/spdlog/include
#LOCAL_LDLIBS    += -llog -ldl -lGLESv2
LOCAL_LDLIBS    += -llog -ldl -lGLESv3
include $(BUILD_SHARED_LIBRARY)
