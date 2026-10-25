LOCAL_PATH:= $(call my-dir)

# gb_loopback_test is also shipped by other in-tree kernels' greybus
# tools/Android.mk (e.g. kernel/huawei/kirin710), and the Android.mk module
# scan is global — so defining it unconditionally collides ("already defined
# by ...") whenever a non-msm8909 device is built.  Only register it when this
# kernel tree is the one actually being built.
ifeq ($(TARGET_KERNEL_SOURCE),kernel/qcom/msm8909-common)

include $(CLEAR_VARS)

LOCAL_SRC_FILES:= loopback_test.c
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE := gb_loopback_test

include $(BUILD_EXECUTABLE)

endif
