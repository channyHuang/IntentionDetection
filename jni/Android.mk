LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_CFLAGS += -Wall
LOCAL_CFLAGS += -O3 -fPIC -std=c++17

LOCAL_C_INCLUDES := $(LOCAL_PATH)/include

LOCAL_C_INCLUDES += $(LOCAL_PATH)/include/eigen-3.4.0

LOCAL_C_INCLUDES += ${LOCAL_PATH}/include/magic_enum-master/include

LOCAL_SRC_FILES := \
	$(subst $(LOCAL_PATH)/,, \
	$(wildcard $(LOCAL_PATH)/src/*.cpp)) 

LOCAL_EXPORT_C_INCLUDES := $(LOCAL_C_INCLUDES)

LOCAL_CPP_FEATURES += exceptions

LOCAL_LDLIBS := -L$(LOCAL_PATH)/libs 

LOCAL_MODULE := IntentionDetection


include $(BUILD_SHARED_LIBRARY)
#include $(BUILD_EXECUTABLE)
