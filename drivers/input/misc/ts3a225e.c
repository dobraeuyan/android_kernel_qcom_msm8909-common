/*
 * TI TS3A225E — Audio Headset Switch with autonomous accessory detection.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "ts3a225e: " fmt

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/ts3a225e.h>

/* Register map (subset used by the stock driver). */
#define TS3A225E_REG_DEVICE_ID		0x01
#define TS3A225E_REG_SWITCH_CTL1	0x02
#define TS3A225E_REG_SWITCH_CTL2	0x03
#define TS3A225E_REG_DETECT_CTL		0x04
#define TS3A225E_REG_DETECT_STATUS	0x05
#define TS3A225E_REG_ACCESSORY		0x06
#define TS3A225E_REG_SWITCH_STATUS	0x07

#define TS3A225E_DETECT_START		0x01

/* Values the stock driver writes for each configuration. */
#define TS3A225E_SWITCH_CTL1_DEFAULT	0x07
#define TS3A225E_SWITCH_CTL2_DEFAULT	0x01	/* probe-time / idle */
#define TS3A225E_SWITCH_CTL2_HEADPHONE	0xf3	/* 3-pole plug inserted */

/* Time the chip needs to finish one autonomous detection cycle. */
#define TS3A225E_DETECT_DELAY_MS	500

/* Retries, mirroring the stock driver's tolerance for a flaky bus. */
#define TS3A225E_XFER_RETRIES		5

#define TS3A225E_PROBE_READ_TRIES	3
#define TS3A225E_PROBE_RETRY_MS		20
#define TS3A225E_PROBE_DEFERRALS	5

static struct i2c_client *ts3a225e_client;
static DEFINE_MUTEX(ts3a225e_lock);

bool ts3a225e_is_present(void)
{
	return READ_ONCE(ts3a225e_client) != NULL;
}
EXPORT_SYMBOL(ts3a225e_is_present);

int ts3a225e_read_byte(u8 reg, u8 *val)
{
	struct i2c_client *client;
	int retry, ret = -EIO;

	if (!val)
		return -EINVAL;

	mutex_lock(&ts3a225e_lock);
	client = ts3a225e_client;
	if (!client) {
		mutex_unlock(&ts3a225e_lock);
		return -ENODEV;
	}

	for (retry = 0; retry < TS3A225E_XFER_RETRIES; retry++) {
		ret = i2c_smbus_read_byte_data(client, reg);
		if (ret >= 0) {
			*val = (u8)ret;
			ret = 0;
			break;
		}
		usleep_range(1000, 1500);
	}
	mutex_unlock(&ts3a225e_lock);

	if (ret)
		pr_err("i2c read of reg 0x%02x failed\n", reg);

	return ret;
}
EXPORT_SYMBOL(ts3a225e_read_byte);

int ts3a225e_write_byte(u8 reg, u8 val)
{
	struct i2c_client *client;
	int retry, ret = -EIO;

	mutex_lock(&ts3a225e_lock);
	client = ts3a225e_client;
	if (!client) {
		mutex_unlock(&ts3a225e_lock);
		return -ENODEV;
	}

	for (retry = 0; retry < TS3A225E_XFER_RETRIES; retry++) {
		ret = i2c_smbus_write_byte_data(client, reg, val);
		if (!ret)
			break;
		usleep_range(1000, 1500);
	}
	/* The stock driver lets each write settle before the next one. */
	if (!ret)
		usleep_range(2000, 2500);
	mutex_unlock(&ts3a225e_lock);

	if (ret)
		pr_err("i2c write of reg 0x%02x failed\n", reg);

	return ret;
}
EXPORT_SYMBOL(ts3a225e_write_byte);

int ts3a225e_detect_accessory(bool *special_headset)
{
	u8 status = 0, accessory = 0;
	int ret;

	if (special_headset)
		*special_headset = false;

	if (!ts3a225e_is_present())
		return -ENODEV;

	/* Kick off one autonomous detection cycle. */
	ret = ts3a225e_write_byte(TS3A225E_REG_DETECT_CTL,
				  TS3A225E_DETECT_START);
	if (ret)
		return ret;

	msleep(TS3A225E_DETECT_DELAY_MS);

	ret = ts3a225e_read_byte(TS3A225E_REG_DETECT_STATUS, &status);
	if (ret)
		return ret;

	ret = ts3a225e_read_byte(TS3A225E_REG_ACCESSORY, &accessory);
	if (ret)
		return ret;

	pr_debug("detect: status(0x05)=0x%02x accessory(0x06)=0x%02x\n",
		 status, accessory);

	switch (accessory) {
	case TS3A225E_ACC_HEADPHONE:
		/*
		 * 3-pole plug: close the switches so the sleeve reaches the
		 * codec's GND (this is what carries the speakers' return
		 * current -- without it the jack is simply mute).
		 */
		ts3a225e_write_byte(TS3A225E_REG_SWITCH_CTL1,
				    TS3A225E_SWITCH_CTL1_DEFAULT);
		ts3a225e_write_byte(TS3A225E_REG_SWITCH_CTL2,
				    TS3A225E_SWITCH_CTL2_HEADPHONE);
		return TS3A225E_ACC_HEADPHONE;

	case TS3A225E_ACC_HEADSET:
		/*
		 * 4-pole plug: the chip has already routed MIC/GND. Flag the
		 * impedance buckets the stock driver treats as a "special"
		 * headset so the caller can run the extra micbias ramp.
		 */
		if (special_headset &&
		    (((status & 0x07) == 0x07) || ((status & 0x38) == 0x38)))
			*special_headset = true;
		return TS3A225E_ACC_HEADSET;

	default:
		pr_debug("detect failed (accessory=0x%02x)\n", accessory);
		return -EIO;
	}
}
EXPORT_SYMBOL(ts3a225e_detect_accessory);

/* Probes that ended in -EPROBE_DEFER because the chip would not answer. */
static int ts3a225e_deferrals;

static int ts3a225e_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	u8 dev_id = 0;
	int attempt, ret;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "SMBus byte-data not supported\n");
		return -EIO;
	}

	mutex_lock(&ts3a225e_lock);
	ts3a225e_client = client;
	mutex_unlock(&ts3a225e_lock);

	for (attempt = 0; attempt < TS3A225E_PROBE_READ_TRIES; attempt++) {
		ret = ts3a225e_read_byte(TS3A225E_REG_DEVICE_ID, &dev_id);
		if (!ret)
			break;
		msleep(TS3A225E_PROBE_RETRY_MS);
	}

	if (ret) {
		mutex_lock(&ts3a225e_lock);
		ts3a225e_client = NULL;
		mutex_unlock(&ts3a225e_lock);

		if (++ts3a225e_deferrals <= TS3A225E_PROBE_DEFERRALS) {
			dev_warn(&client->dev,
				 "chip not responding (%d), deferring probe (%d/%d)\n",
				 ret, ts3a225e_deferrals,
				 TS3A225E_PROBE_DEFERRALS);
			return -EPROBE_DEFER;
		}

		dev_err(&client->dev,
			"chip not responding (%d) after %d deferrals, giving up -- the 3.5mm jack will misdetect as LINEOUT\n",
			ret, TS3A225E_PROBE_DEFERRALS);
		return ret;
	}

	/* Idle switch configuration, same values the stock driver writes. */
	ts3a225e_write_byte(TS3A225E_REG_SWITCH_CTL1,
			    TS3A225E_SWITCH_CTL1_DEFAULT);
	ts3a225e_write_byte(TS3A225E_REG_SWITCH_CTL2,
			    TS3A225E_SWITCH_CTL2_DEFAULT);

	dev_info(&client->dev, "TS3A225E at 0x%02x, device id 0x%02x\n",
		 client->addr, dev_id);

	return 0;
}

static int ts3a225e_remove(struct i2c_client *client)
{
	mutex_lock(&ts3a225e_lock);
	if (ts3a225e_client == client)
		ts3a225e_client = NULL;
	mutex_unlock(&ts3a225e_lock);

	return 0;
}

static const struct i2c_device_id ts3a225e_id[] = {
	{ "ts3a225e", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ts3a225e_id);

#ifdef CONFIG_OF
static const struct of_device_id ts3a225e_of_match[] = {
	{ .compatible = "hs_switch,ts3a225e", },
	{ }
};
MODULE_DEVICE_TABLE(of, ts3a225e_of_match);
#endif

static struct i2c_driver ts3a225e_driver = {
	.driver = {
		.name		= "ts3a225e",
		.owner		= THIS_MODULE,
		.of_match_table	= of_match_ptr(ts3a225e_of_match),
	},
	.probe		= ts3a225e_probe,
	.remove		= ts3a225e_remove,
	.id_table	= ts3a225e_id,
};

module_i2c_driver(ts3a225e_driver);

MODULE_DESCRIPTION("TI TS3A225E audio headset switch");
MODULE_LICENSE("GPL v2");
