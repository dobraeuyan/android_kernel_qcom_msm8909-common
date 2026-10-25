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
 * OmniVision OV8858 - HTC msm8909 (a16 / HTC Desire 530) rear camera.
 *
 */

#include "msm_sensor.h"

#define OV8858_HTC_SENSOR_NAME "ov8858_htc"

DEFINE_MSM_MUTEX(ov8858_htc_mut);

static struct msm_sensor_ctrl_t ov8858_htc_s_ctrl;

/* OTP "Driver IC Vendor & Version" byte, HTC-private (was s_ctrl->driver_ic) */
static uint8_t ov8858_htc_driver_ic;

static struct msm_sensor_power_setting ov8858_htc_power_setting[] = {
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
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 1,
		.delay = 5,
	},
	{
		/* V_CAM_D1V2 */
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
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct v4l2_subdev_info ov8858_htc_subdev_info[] = {
	{
		.code       = MEDIA_BUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt        = 1,
		.order      = 0,
	},
};

static const struct i2c_device_id ov8858_htc_i2c_id[] = {
	{OV8858_HTC_SENSOR_NAME, (kernel_ulong_t)&ov8858_htc_s_ctrl},
	{ }
};

static int32_t msm_ov8858_htc_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	return msm_sensor_i2c_probe(client, id, &ov8858_htc_s_ctrl);
}

static struct i2c_driver ov8858_htc_i2c_driver = {
	.id_table = ov8858_htc_i2c_id,
	.probe  = msm_ov8858_htc_i2c_probe,
	.driver = {
		.name = OV8858_HTC_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ov8858_htc_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id ov8858_htc_dt_match[] = {
	{.compatible = "htc,ov8858_htc", .data = &ov8858_htc_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, ov8858_htc_dt_match);

static int32_t ov8858_htc_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;

	pr_info("%s:%d\n", __func__, __LINE__);
	match = of_match_device(ov8858_htc_dt_match, &pdev->dev);
	if (!match) {
		pr_err("%s:%d match is null\n", __func__, __LINE__);
		return -ENODEV;
	}
	rc = msm_sensor_platform_probe(pdev, match->data);
	pr_info("%s %s rc %d\n", __func__, dev_name(&pdev->dev), rc);
	return rc;
}

static struct platform_driver ov8858_htc_platform_driver = {
	.driver = {
		.name = "htc,ov8858_htc",
		.owner = THIS_MODULE,
		.of_match_table = ov8858_htc_dt_match,
	},
	.probe = ov8858_htc_platform_probe,
};

static const char *ov8858_htcVendor = "Omnivision";
static const char *ov8858_htcNAME = "ov8858_htc";
static const char *ov8858_htcSize = "8M";

static ssize_t sensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%s %s %s\n",
		ov8858_htcVendor, ov8858_htcNAME, ov8858_htcSize);
}

static DEVICE_ATTR(sensor, 0444, sensor_vendor_show, NULL);

static struct kobject *android_ov8858_htc;

static int ov8858_htc_sysfs_init(void)
{
	int ret;

	android_ov8858_htc = kobject_create_and_add("android_camera", NULL);
	if (android_ov8858_htc == NULL) {
		pr_info("ov8858_htc_sysfs_init: subsystem_register failed\n");
		return -ENOMEM;
	}
	ret = sysfs_create_file(android_ov8858_htc, &dev_attr_sensor.attr);
	if (ret) {
		pr_info("ov8858_htc_sysfs_init: sysfs_create_file failed\n");
		kobject_del(android_ov8858_htc);
		android_ov8858_htc = NULL;
	}

	return 0;
}

#define OV8858_LITEON_OTP_SIZE 0x14

/*
 * Read the LiteOn OTP block (module vendor / lens / sensor version / driver IC
 * / actuator ID, fuse id and the AF infinity+macro calibration codes).
 *
 * The 3.10 driver also handed this back to userspace through
 * CFG_I2C_IOCTL_R_OTP -> sensorb_cfg_data.cfg.fuse / .af_value.  Those UAPI
 * members do not exist in this tree and adding them would change the layout of
 * a union shared with x104f/bamboo/x103f, so the values are only latched and
 * logged here.  If the HTC HAL turns out to need them, the way to do it
 * without an ABI break is a device-private cfgtype carrying its own struct.
 */
static int ov8858_htc_read_otp(struct msm_sensor_ctrl_t *s_ctrl)
{
	static const short addr[3][OV8858_LITEON_OTP_SIZE] = {
		/* layer 1 */
		{0x7010, 0x7011, 0x7012, 0x7013, 0x7014, 0x7015, 0x7016,
		 0x7017, 0x7018, 0x7019, 0x701A, 0x701B, 0x701C, 0x7021,
		 0x7022, 0x7023, 0x7024, 0x7034, 0x7035, 0x7036},
		/* layer 2 */
		{0x7037, 0x7038, 0x7039, 0x703A, 0x703B, 0x703C, 0x703D,
		 0x703E, 0x703F, 0x7040, 0x7041, 0x7042, 0x7043, 0x7048,
		 0x7049, 0x704A, 0x704B, 0x705B, 0x705C, 0x705D},
		/* layer 3 */
		{0x705E, 0x705F, 0x7060, 0x7061, 0x7062, 0x7063, 0x7064,
		 0x7065, 0x7066, 0x7067, 0x7068, 0x7069, 0x706A, 0x706F,
		 0x7070, 0x7071, 0x7072, 0x7082, 0x7083, 0x7084},
	};
	static uint8_t otp[OV8858_LITEON_OTP_SIZE];
	static int32_t valid_layer = -1;
	struct msm_camera_i2c_client *cli = s_ctrl->sensor_i2c_client;
	const uint16_t addr_start = 0x7000;
	const uint16_t addr_end = 0x73ff;
	uint16_t read_data = 0;
	int32_t i, j;
	int32_t rc = 0;

	rc = cli->i2c_func_tbl->i2c_write(cli, 0x0100, 0x01,
		MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0)
		pr_info("%s: i2c_write b 0x0100 fail\n", __func__);

	rc = cli->i2c_func_tbl->i2c_write(cli, 0x5002, 0x05,
		MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0)
		pr_info("%s: i2c_write b 0x5002 fail\n", __func__);

	rc = cli->i2c_func_tbl->i2c_write(cli, 0x3d84, 0x40,
		MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0)
		pr_info("%s: i2c_write b 0x3d84 fail\n", __func__);

	rc = cli->i2c_func_tbl->i2c_write(cli, 0x3d88, addr_start,
		MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0)
		pr_info("%s: i2c_write w 0x3d88 fail\n", __func__);

	rc = cli->i2c_func_tbl->i2c_write(cli, 0x3d8a, addr_end,
		MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0)
		pr_info("%s: i2c_write w 0x3d8a fail\n", __func__);

	rc = cli->i2c_func_tbl->i2c_write(cli, 0x3d81, 0x01,
		MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0)
		pr_info("%s: i2c_write b 0x3d81 fail\n", __func__);

	msleep(20);

	/* start from layer 3 downwards, first non-blank layer wins */
	for (j = 2; j >= 0; j--) {
		for (i = 0; i < OV8858_LITEON_OTP_SIZE; i++) {
			rc = cli->i2c_func_tbl->i2c_read(cli, addr[j][i],
				&read_data, MSM_CAMERA_I2C_BYTE_DATA);
			if (rc < 0) {
				pr_err("%s: i2c_read 0x%x failed\n", __func__,
					addr[j][i]);
				return rc;
			}
			otp[i] = read_data;
			if (read_data)
				valid_layer = j;
		}
		if (valid_layer != -1)
			break;
	}
	pr_info("%s: OTP valid layer = %d\n", __func__, valid_layer);

	ov8858_htc_driver_ic = otp[3];

	rc = cli->i2c_func_tbl->i2c_write(cli, 0x0100, 0x00,
		MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0)
		pr_info("%s: i2c_write b 0x0100 fail\n", __func__);

	rc = cli->i2c_func_tbl->i2c_write(cli, 0x5002, 0x07,
		MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0)
		pr_info("%s: i2c_write b 0x5002 fail\n", __func__);

	pr_info("%s: OTP Module vendor = 0x%x\n", __func__, otp[0]);
	pr_info("%s: OTP LENS = 0x%x\n", __func__, otp[1]);
	pr_info("%s: OTP Sensor Version = 0x%x\n", __func__, otp[2]);
	pr_info("%s: OTP Driver IC Vendor & Version = 0x%x\n", __func__,
		otp[3]);
	pr_info("%s: OTP Actuator vendor ID & Version = 0x%x\n", __func__,
		otp[4]);
	pr_info("%s: fuse id: 0x%x 0x%x 0x%x 0x%x\n", __func__,
		otp[5], otp[6], otp[7], otp[8]);
	pr_info("%s: OTP AF inf 0x%x%02x macro 0x%x%02x\n", __func__,
		otp[0xD], otp[0xE], otp[0xF], otp[0x10]);

	return rc;
}

static int32_t ov8858_htc_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	static int first;
	int32_t status;

	status = msm_sensor_match_id(s_ctrl);
	if (status >= 0 && first == 0) {
		ov8858_htc_read_otp(s_ctrl);
		ov8858_htc_sysfs_init();
		first = 1;
	}
	pr_info("%s: status = %d\n", __func__, status);
	return status;
}

static struct msm_sensor_fn_t ov8858_htc_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
#ifdef CONFIG_COMPAT
	.sensor_config32 = msm_sensor_config32,
#endif
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = ov8858_htc_sensor_match_id,
};

static struct msm_sensor_ctrl_t ov8858_htc_s_ctrl = {
	.sensor_i2c_client = &ov8858_htc_sensor_i2c_client,
	.power_setting_array.power_setting = ov8858_htc_power_setting,
	.power_setting_array.size = ARRAY_SIZE(ov8858_htc_power_setting),
	.msm_sensor_mutex = &ov8858_htc_mut,
	.sensor_v4l2_subdev_info = ov8858_htc_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ov8858_htc_subdev_info),
	.func_tbl = &ov8858_htc_sensor_func_tbl,
};

static int __init ov8858_htc_init_module(void)
{
	int32_t rc;

	/*
	 * The factory DT hangs the sensor off the QUP i2c controller, so the
	 * i2c_driver is what actually binds (the i2c core strips the "htc,"
	 * prefix off the compatible and matches it against id_table).  Register
	 * the platform driver too so a node placed on a plain bus (CCI style)
	 * still binds.  3.10 returned early after i2c_add_driver(); that left
	 * the platform driver unregistered, which was a bug.
	 */
	rc = i2c_add_driver(&ov8858_htc_i2c_driver);
	if (rc)
		pr_err("%s: i2c_add_driver rc %d\n", __func__, rc);

	rc = platform_driver_register(&ov8858_htc_platform_driver);
	if (rc)
		pr_err("%s: platform_driver_register rc %d\n", __func__, rc);

	return 0;
}

static void __exit ov8858_htc_exit_module(void)
{
	platform_driver_unregister(&ov8858_htc_platform_driver);
	i2c_del_driver(&ov8858_htc_i2c_driver);
}

late_initcall(ov8858_htc_init_module);
module_exit(ov8858_htc_exit_module);
MODULE_DESCRIPTION("ov8858_htc");
MODULE_LICENSE("GPL v2");
