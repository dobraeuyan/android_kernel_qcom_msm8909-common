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
 * Samsung S5K5E - HTC msm8909 (a16 / HTC Desire 530) front camera.
 */

#include "msm_sensor.h"

#define S5K5E_HTC_SENSOR_NAME "s5k5e_htc"

DEFINE_MSM_MUTEX(s5k5e_htc_mut);

static struct msm_sensor_ctrl_t s5k5e_htc_s_ctrl;

static struct msm_sensor_power_setting s5k5e_htc_power_setting[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VAF,
		.config_val = 1,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 1,
		.delay = 5,
	},
	{
		/* gpio 8, cam analog */
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_CUSTOM1,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 1,
		.delay = 5,
	},
	{
		/* gpio 28 */
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},
	{
		/* gpio 27 */
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 5,
	},
};

static struct v4l2_subdev_info s5k5e_htc_subdev_info[] = {
	{
		.code       = MEDIA_BUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt        = 1,
		.order      = 0,
	},
};

static const struct i2c_device_id s5k5e_htc_i2c_id[] = {
	{S5K5E_HTC_SENSOR_NAME, (kernel_ulong_t)&s5k5e_htc_s_ctrl},
	{ }
};

static int32_t msm_s5k5e_htc_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	return msm_sensor_i2c_probe(client, id, &s5k5e_htc_s_ctrl);
}

static struct i2c_driver s5k5e_htc_i2c_driver = {
	.id_table = s5k5e_htc_i2c_id,
	.probe  = msm_s5k5e_htc_i2c_probe,
	.driver = {
		.name = S5K5E_HTC_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client s5k5e_htc_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id s5k5e_htc_dt_match[] = {
	{.compatible = "htc,s5k5e_htc", .data = &s5k5e_htc_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, s5k5e_htc_dt_match);

static int32_t s5k5e_htc_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;

	pr_info("%s:%d\n", __func__, __LINE__);
	match = of_match_device(s5k5e_htc_dt_match, &pdev->dev);
	if (!match) {
		pr_err("%s:%d match is null\n", __func__, __LINE__);
		return -ENODEV;
	}
	rc = msm_sensor_platform_probe(pdev, match->data);
	pr_info("%s %s rc %d\n", __func__, dev_name(&pdev->dev), rc);
	return rc;
}

static struct platform_driver s5k5e_htc_platform_driver = {
	.driver = {
		.name = "htc,s5k5e_htc",
		.owner = THIS_MODULE,
		.of_match_table = s5k5e_htc_dt_match,
	},
	.probe = s5k5e_htc_platform_probe,
};

static const char *s5k5e_htcVendor = "Samsung";
static const char *s5k5e_htcNAME = "s5k5e_htc";
static const char *s5k5e_htcSize = "5.0M";

static ssize_t sensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%s %s %s\n",
		s5k5e_htcVendor, s5k5e_htcNAME, s5k5e_htcSize);
}

static DEVICE_ATTR(sensor, 0444, sensor_vendor_show, NULL);

static struct kobject *android_s5k5e_htc;

static int s5k5e_htc_sysfs_init(void)
{
	int ret;

	android_s5k5e_htc = kobject_create_and_add("android_camera2", NULL);
	if (android_s5k5e_htc == NULL) {
		pr_info("s5k5e_htc_sysfs_init: subsystem_register failed\n");
		return -ENOMEM;
	}
	ret = sysfs_create_file(android_s5k5e_htc, &dev_attr_sensor.attr);
	if (ret) {
		pr_info("s5k5e_htc_sysfs_init: sysfs_create_file failed\n");
		kobject_del(android_s5k5e_htc);
		android_s5k5e_htc = NULL;
	}

	return 0;
}

/*
 * OTP read.  As in ov8858_htc.c the CFG_I2C_IOCTL_R_OTP hand-off to userspace
 * is dropped (the sensorb_cfg_data.cfg.fuse / .lens_id members it needs are
 * HTC-private UAPI that does not exist in this tree, and adding them would
 * change a union shared by the other msm8909 devices).  Values are logged.
 */
static int s5k5e_htc_read_otp(struct msm_sensor_ctrl_t *s_ctrl)
{
	static const uint16_t id_addr[6] = {
		0x0A04, 0x0A05, 0x0A06, 0x0A07, 0x0A08, 0x0A09
	};
	static uint16_t id_data[6];
	static int32_t valid_page = -1;
	struct msm_camera_i2c_client *cli = s_ctrl->sensor_i2c_client;
	uint16_t read_data = 0;
	int32_t rc = 0;
	int i, j;

	for (j = 4; j >= 2; j--) {
		rc = cli->i2c_func_tbl->i2c_write(cli, 0x0A00, 0x04,
			MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0)
			pr_err("%s: i2c_write failed\n", __func__);
		rc = cli->i2c_func_tbl->i2c_write(cli, 0x0A02, j,
			MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0)
			pr_info("%s: i2c_write failed\n", __func__);
		rc = cli->i2c_func_tbl->i2c_write(cli, 0x0A00, 0x01,
			MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0)
			pr_info("%s: i2c_write failed\n", __func__);
		msleep(20);
		for (i = 0; i < 6; i++) {
			rc = cli->i2c_func_tbl->i2c_read(cli, id_addr[i],
				&read_data, MSM_CAMERA_I2C_BYTE_DATA);
			if (rc < 0)
				pr_err("%s: i2c_read 0x%x failed\n", __func__,
					id_addr[i]);
			id_data[i] = read_data & 0xff;
		}
		if (read_data)
			valid_page = j;
		if (valid_page != -1)
			break;
	}

	rc = cli->i2c_func_tbl->i2c_write(cli, 0x0A00, 0x04,
		MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0)
		pr_err("%s: i2c_write failed\n", __func__);
	rc = cli->i2c_func_tbl->i2c_write(cli, 0x0A00, 0x00,
		MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0)
		pr_err("%s: i2c_write failed\n", __func__);

	pr_info("%s: OTP valid page = %d\n", __func__, valid_page);
	pr_info("%s: OTP Module vendor = 0x%x\n", __func__, id_data[0]);
	pr_info("%s: OTP LENS = 0x%x\n", __func__, id_data[1]);
	pr_info("%s: OTP Sensor Version = 0x%x\n", __func__, id_data[2]);
	pr_info("%s: fuse id: 0x0 0x%x 0x%x 0x%x\n", __func__,
		id_data[3], id_data[4], id_data[5]);

	return rc;
}

static int32_t s5k5e_htc_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	static int first;
	int32_t status;

	status = msm_sensor_match_id(s_ctrl);
	if (status >= 0 && first == 0) {
		s5k5e_htc_read_otp(s_ctrl);
		s5k5e_htc_sysfs_init();
		first = 1;
	}
	pr_info("%s: status = %d\n", __func__, status);
	return status;
}

static struct msm_sensor_fn_t s5k5e_htc_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
#ifdef CONFIG_COMPAT
	.sensor_config32 = msm_sensor_config32,
#endif
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = s5k5e_htc_sensor_match_id,
};

static struct msm_sensor_ctrl_t s5k5e_htc_s_ctrl = {
	.sensor_i2c_client = &s5k5e_htc_sensor_i2c_client,
	.power_setting_array.power_setting = s5k5e_htc_power_setting,
	.power_setting_array.size = ARRAY_SIZE(s5k5e_htc_power_setting),
	.msm_sensor_mutex = &s5k5e_htc_mut,
	.sensor_v4l2_subdev_info = s5k5e_htc_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(s5k5e_htc_subdev_info),
	.func_tbl = &s5k5e_htc_sensor_func_tbl,
};

static int __init s5k5e_htc_init_module(void)
{
	int32_t rc;

	/* See ov8858_htc.c: both bus flavours are registered on purpose. */
	rc = i2c_add_driver(&s5k5e_htc_i2c_driver);
	if (rc)
		pr_err("%s: i2c_add_driver rc %d\n", __func__, rc);

	rc = platform_driver_register(&s5k5e_htc_platform_driver);
	if (rc)
		pr_err("%s: platform_driver_register rc %d\n", __func__, rc);

	return 0;
}

static void __exit s5k5e_htc_exit_module(void)
{
	platform_driver_unregister(&s5k5e_htc_platform_driver);
	i2c_del_driver(&s5k5e_htc_i2c_driver);
}

late_initcall(s5k5e_htc_init_module);
module_exit(s5k5e_htc_exit_module);
MODULE_DESCRIPTION("s5k5e_htc");
MODULE_LICENSE("GPL v2");
