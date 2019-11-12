LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

OPENCV_CAMERA_MODULES:=off
OPENCV_INSTALL_MODULES:=on
OPENCV_LIB_TYPE:=SHARED #STATIC
include thirdparty/OpenCV-android-sdk/sdk/native/jni/OpenCV.mk

LOCAL_MODULE    := optar
LOCAL_CFLAGS    := -std=c++11
LOCAL_SRC_FILES := ../source/optar.cpp
LOCAL_C_INCLUDES += $(LOCAL_PATH)/../source
#LOCAL_LDLIBS    += -llog -ldl -lGLESv2
LOCAL_LDLIBS    += -llog -ldl -lGLESv3
include $(BUILD_SHARED_LIBRARY)
