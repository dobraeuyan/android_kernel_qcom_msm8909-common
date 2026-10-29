ccflags-y += -Idrivers/media/platform/msm/camera_v2
ccflags-y += -Idrivers/media/platform/msm/camera_v2/common
ccflags-y += -Idrivers/media/platform/msm/camera_v2/msm_vb2
ccflags-y += -Idrivers/media/platform/msm/camera_v2/camera
ccflags-y += -Idrivers/media/platform/msm/camera_v2/sensor/io
ccflags-y += -Idrivers/media/platform/msm/camera_v2/sensor/cci
obj-$(CONFIG_MSMB_CAMERA) += cci/ io/ csiphy/ csid/ actuator/ eeprom/ ois/ flash/ ir_led/ ir_cut/
obj-$(CONFIG_MSMB_CAMERA) += laser_led/
obj-$(CONFIG_MSM_CAMERA_SENSOR) += msm_sensor_init.o msm_sensor_driver.o msm_sensor.o
# Sensors probed by the kernel (per-driver model), see msm_sensor.c tail
obj-$(CONFIG_OV8858_HTC) += ov8858_htc.o
obj-$(CONFIG_OV2680_HTC) += ov2680_htc.o
obj-$(CONFIG_S5K5E_HTC) += s5k5e_htc.o
obj-$(CONFIG_HTC_AWB_CAL) += htc_awb_cal.o
