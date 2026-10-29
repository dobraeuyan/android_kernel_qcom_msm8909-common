// SPDX-License-Identifier: GPL-2.0
/*
 * Expose the camera AWB/LSC calibration blob that the HTC bootloader
 * injects into the device tree (/calibration_data/cam_awb) to the
 * camera daemon through /sys/android_camera_awb_cal.
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/of.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/types.h>

#define CALIBRATION_DATA_PATH	"/calibration_data"
#define CAM_AWB_CAL_DATA	"cam_awb"
#define AWB_CAL_MAX_SIZE	0x4000U
#define AWB_CAL_BLOCK_SIZE	0x1000U
#define DUAL_CAL_SIZE		4095

/* Byte layout matches HTC's qct_awb_lsc_struct; only sizes matter here. */
struct qct_lsc_struct {
	u32 lsc_verify;
	u32 lsc_fuseid[4];
	float pCalcParam[17 * 13 * 4];
	u32 lsc_checksum;
};

struct qct_awb_lsc_struct {
	u32 caBuff[8];
	struct qct_lsc_struct qct_lsc_data;
	u32 flashcaBuff[8];
	u32 aec_caBuff[9];
	u32 alight_caBuff[8];
	u32 dualflashcaBuff[12];
	u32 awb_verify;
};

static unsigned char cam_awb_ram[AWB_CAL_MAX_SIZE];

static ssize_t awb_calibration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	memcpy(buf, cam_awb_ram, sizeof(struct qct_awb_lsc_struct));
	return sizeof(struct qct_awb_lsc_struct);
}

static ssize_t awb_calibration_front_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	memcpy(buf, cam_awb_ram + AWB_CAL_BLOCK_SIZE,
		sizeof(struct qct_awb_lsc_struct));
	return sizeof(struct qct_awb_lsc_struct);
}

static ssize_t awb_calibration_sub_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	memcpy(buf, cam_awb_ram + 2 * AWB_CAL_BLOCK_SIZE,
		sizeof(struct qct_awb_lsc_struct));
	return sizeof(struct qct_awb_lsc_struct);
}

static ssize_t awb_calibration_3D_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	memcpy(buf, cam_awb_ram + 3 * AWB_CAL_BLOCK_SIZE, DUAL_CAL_SIZE);
	return DUAL_CAL_SIZE;
}

static DEVICE_ATTR(awb_cal, 0444, awb_calibration_show, NULL);
static DEVICE_ATTR(awb_cal_front, 0444, awb_calibration_front_show, NULL);
static DEVICE_ATTR(awb_cal_sub, 0444, awb_calibration_sub_show, NULL);
static DEVICE_ATTR(awb_cal_3D, 0444, awb_calibration_3D_show, NULL);

static int __init cam_get_awb_cal_init(void)
{
	struct device_node *node;
	struct kobject *cam_awb_cal;
	const void *p_data;
	int p_size = 0;

	BUILD_BUG_ON(sizeof(struct qct_awb_lsc_struct) != 3744);
	BUILD_BUG_ON(sizeof(struct qct_awb_lsc_struct) > AWB_CAL_BLOCK_SIZE);
	BUILD_BUG_ON(3 * AWB_CAL_BLOCK_SIZE + DUAL_CAL_SIZE > AWB_CAL_MAX_SIZE);

	node = of_find_node_by_path(CALIBRATION_DATA_PATH);
	if (!node)
		return 0;

	p_data = of_get_property(node, CAM_AWB_CAL_DATA, &p_size);
	if (!p_data || p_size <= 0) {
		pr_info("[CAM] no %s/%s calibration data\n",
			CALIBRATION_DATA_PATH, CAM_AWB_CAL_DATA);
		return 0;
	}

	memcpy(cam_awb_ram, p_data, min_t(int, p_size, AWB_CAL_MAX_SIZE));

	cam_awb_cal = kobject_create_and_add("android_camera_awb_cal", NULL);
	if (!cam_awb_cal)
		return 0;

	if (sysfs_create_file(cam_awb_cal, &dev_attr_awb_cal.attr) ||
	    sysfs_create_file(cam_awb_cal, &dev_attr_awb_cal_front.attr) ||
	    sysfs_create_file(cam_awb_cal, &dev_attr_awb_cal_sub.attr) ||
	    sysfs_create_file(cam_awb_cal, &dev_attr_awb_cal_3D.attr)) {
		pr_info("[CAM] failed to create android_camera_awb_cal attrs\n");
		kobject_del(cam_awb_cal);
		kobject_put(cam_awb_cal);
		return 0;
	}

	pr_info("[CAM] cam_awb calibration data: %d bytes\n", p_size);
	return 0;
}
late_initcall(cam_get_awb_cal_init);
