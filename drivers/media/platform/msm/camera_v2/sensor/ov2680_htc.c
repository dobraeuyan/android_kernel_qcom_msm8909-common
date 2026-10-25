/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * OmniVision OV2680 - HTC msm8909 front camera (a13 family).
 */

#include "msm_sensor.h"

#define OV2680_HTC_SENSOR_NAME "ov2680_htc"

DEFINE_MSM_MUTEX(ov2680_htc_mut);

static struct msm_sensor_ctrl_t ov2680_htc_s_ctrl;

static struct msm_sensor_power_setting ov2680_htc_power_setting[] = {
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_CUSTOM1,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 1,
		.delay = 15,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 1,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 1,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},
	{
		/* V_CAM_D1V2 */
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VAF,
		.config_val = 1,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct v4l2_subdev_info ov2680_htc_subdev_info[] = {
	{
		.code       = MEDIA_BUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt        = 1,
		.order      = 0,
	},
};

static const struct i2c_device_id ov2680_htc_i2c_id[] = {
	{OV2680_HTC_SENSOR_NAME, (kernel_ulong_t)&ov2680_htc_s_ctrl},
	{ }
};

static int32_t msm_ov2680_htc_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	return msm_sensor_i2c_probe(client, id, &ov2680_htc_s_ctrl);
}

static struct i2c_driver ov2680_htc_i2c_driver = {
	.id_table = ov2680_htc_i2c_id,
	.probe  = msm_ov2680_htc_i2c_probe,
	.driver = {
		.name = OV2680_HTC_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ov2680_htc_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id ov2680_htc_dt_match[] = {
	{.compatible = "htc,ov2680_htc", .data = &ov2680_htc_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, ov2680_htc_dt_match);

static int32_t ov2680_htc_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;

	pr_info("%s:%d\n", __func__, __LINE__);
	match = of_match_device(ov2680_htc_dt_match, &pdev->dev);
	if (!match) {
		pr_err("%s:%d match is null\n", __func__, __LINE__);
		return -ENODEV;
	}
	rc = msm_sensor_platform_probe(pdev, match->data);
	pr_info("%s %s rc %d\n", __func__, dev_name(&pdev->dev), rc);
	return rc;
}

static struct platform_driver ov2680_htc_platform_driver = {
	.driver = {
		.name = "htc,ov2680_htc",
		.owner = THIS_MODULE,
		.of_match_table = ov2680_htc_dt_match,
	},
	.probe = ov2680_htc_platform_probe,
};

static const char *ov2680_htcVendor = "Omnivision";
static const char *ov2680_htcNAME = "ov2680_htc";
static const char *ov2680_htcSize = "2M";

static ssize_t sensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%s %s %s\n",
		ov2680_htcVendor, ov2680_htcNAME, ov2680_htcSize);
}

static DEVICE_ATTR(sensor, 0444, sensor_vendor_show, NULL);

static struct kobject *android_ov2680_htc;

static int ov2680_htc_sysfs_init(void)
{
	int ret;

	android_ov2680_htc = kobject_create_and_add("android_camera2", NULL);
	if (android_ov2680_htc == NULL) {
		pr_info("ov2680_htc_sysfs_init: subsystem_register failed\n");
		return -ENOMEM;
	}
	ret = sysfs_create_file(android_ov2680_htc, &dev_attr_sensor.attr);
	if (ret) {
		pr_info("ov2680_htc_sysfs_init: sysfs_create_file failed\n");
		kobject_del(android_ov2680_htc);
		android_ov2680_htc = NULL;
	}

	return 0;
}

static int32_t ov2680_htc_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	static int first;
	int32_t status;

	status = msm_sensor_match_id(s_ctrl);
	if (status >= 0 && first == 0) {
		ov2680_htc_sysfs_init();
		first = 1;
	}
	pr_info("%s: status = %d\n", __func__, status);
	return status;
}

static struct msm_sensor_fn_t ov2680_htc_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
#ifdef CONFIG_COMPAT
	.sensor_config32 = msm_sensor_config32,
#endif
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = ov2680_htc_sensor_match_id,
};

static struct msm_sensor_ctrl_t ov2680_htc_s_ctrl = {
	.sensor_i2c_client = &ov2680_htc_sensor_i2c_client,
	.power_setting_array.power_setting = ov2680_htc_power_setting,
	.power_setting_array.size = ARRAY_SIZE(ov2680_htc_power_setting),
	.msm_sensor_mutex = &ov2680_htc_mut,
	.sensor_v4l2_subdev_info = ov2680_htc_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ov2680_htc_subdev_info),
	.func_tbl = &ov2680_htc_sensor_func_tbl,
};

static int __init ov2680_htc_init_module(void)
{
	int32_t rc;

	/*
	 * See ov8858_htc.c: the factory DT hangs the sensor off the QUP i2c
	 * controller so the i2c_driver is what actually binds; register the
	 * platform driver too so a CCI-style plain-bus node still binds.
	 * 3.10 returned early after i2c_add_driver(), leaving the platform
	 * driver unregistered -- fixed here like in the other ports.
	 */
	rc = i2c_add_driver(&ov2680_htc_i2c_driver);
	if (rc)
		pr_err("%s: i2c_add_driver rc %d\n", __func__, rc);

	rc = platform_driver_register(&ov2680_htc_platform_driver);
	if (rc)
		pr_err("%s: platform_driver_register rc %d\n", __func__, rc);

	return 0;
}

static void __exit ov2680_htc_exit_module(void)
{
	platform_driver_unregister(&ov2680_htc_platform_driver);
	i2c_del_driver(&ov2680_htc_i2c_driver);
}

late_initcall(ov2680_htc_init_module);
module_exit(ov2680_htc_exit_module);
MODULE_DESCRIPTION("ov2680_htc");
MODULE_LICENSE("GPL v2");
