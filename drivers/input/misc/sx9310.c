// SPDX-License-Identifier: GPL-2.0

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/bitops.h>
#include <linux/pm.h>

#define SX9310_DRIVER_NAME	"sx9310"
#define SX9310_INPUT_NAME	"SX9310 Cap Touch"	/* stock input dev name */

/* --- generic SX9310 register map (matches mainline iio sx9310) --- */
#define SX9310_REG_IRQ_SRC	0x00	/* cleared on read */
#define SX9310_REG_STAT0	0x01	/* per-CS proximity status */
#define SX9310_REG_STAT1	0x02
#define SX9310_REG_IRQ_MSK	0x03
#define SX9310_REG_PROX_CTRL0	0x10	/* SENSOREN in low nibble */
#define SX9310_REG_SENSOR_SEL	0x30	/* select CS for the readback block */
#define SX9310_REG_USE_MSB	0x31
#define SX9310_REG_USE_LSB	0x32
#define SX9310_REG_AVG_MSB	0x33
#define SX9310_REG_AVG_LSB	0x34
#define SX9310_REG_DIFF_MSB	0x35
#define SX9310_REG_DIFF_LSB	0x36
#define SX9310_REG_OFFSET_MSB	0x37
#define SX9310_REG_OFFSET_LSB	0x38
#define SX9310_REG_WHOAMI	0x42
#define SX9310_REG_SOFTRESET	0x7f

#define SX9310_SOFTRESET_CMD	0xde
#define SX9310_WHOAMI_VALUE	0x01

/* IRQ_SRC / IRQ_MSK bits (reg-init programs the mask to 0x60 = CLOSE|FAR) */
#define SX9310_RESET_IRQ	BIT(7)
#define SX9310_CLOSE_IRQ	BIT(6)	/* aggregate: some channel went near */
#define SX9310_FAR_IRQ		BIT(5)	/* aggregate: some channel went far  */
#define SX9310_COMPDONE_IRQ	BIT(4)
#define SX9310_CONVDONE_IRQ	BIT(3)

/* STAT0 low nibble = per-CS proximity (PROXSTAT), CS0..CS3 = bits 0..3 */
#define SX9310_PROXSTAT_MASK	0x0f

/* one register/value pair, as packed in Semtech,reg-init */
struct sx9310_reg {
	u8 reg;
	u8 val;
};

/* one input button, mapped to a CS channel's PROXSTAT bit */
struct sx9310_button {
	int keycode;
	u8  mask;	/* STAT0 bit for this channel */
	bool pressed;
};

/*
 * The reg-init enables CS0..CS2 (PROX_CTRL0 = 0x27). Keycodes are the Semtech
 * reference defaults for those channels (see file header — model confirmed,
 * numeric codes are the reference defaults).
 */
static struct sx9310_button sx9310_buttons[] = {
	{ KEY_0, BIT(0), false },
	{ KEY_1, BIT(1), false },
	{ KEY_2, BIT(2), false },
};

struct sx9310_data {
	struct i2c_client	*client;
	struct input_dev	*input;
	struct regulator	*vdd;
	struct sx9310_reg	*reg_init;	/* parsed from DT */
	int			reg_num;
	int			irq_gpio;
	struct mutex		lock;
};

static int sx9310_write(struct sx9310_data *data, u8 reg, u8 val)
{
	int ret = i2c_smbus_write_byte_data(data->client, reg, val);

	if (ret < 0)
		dev_err(&data->client->dev,
			"i2c write reg 0x%02x=0x%02x failed: %d\n",
			reg, val, ret);
	return ret;
}

static int sx9310_read(struct sx9310_data *data, u8 reg)
{
	int ret = i2c_smbus_read_byte_data(data->client, reg);

	if (ret < 0)
		dev_err(&data->client->dev,
			"i2c read reg 0x%02x failed: %d\n", reg, ret);
	return ret;
}

/* signed 16-bit readback of one CS channel's USE/AVG/DIFF/OFFSET register */
static int sx9310_read_s16(struct sx9310_data *data, u8 msb_reg)
{
	int msb = sx9310_read(data, msb_reg);
	int lsb = sx9310_read(data, msb_reg + 1);

	if (msb < 0 || lsb < 0)
		return 0;
	return (s16)((msb << 8) | lsb);
}

/* Apply the stock register dump carried verbatim in the DT. */
static int sx9310_apply_reg_init(struct sx9310_data *data)
{
	int i, ret;

	for (i = 0; i < data->reg_num; i++) {
		ret = sx9310_write(data, data->reg_init[i].reg,
				   data->reg_init[i].val);
		if (ret < 0)
			return ret;
	}
	return 0;
}

static int sx9310_hw_init(struct sx9310_data *data)
{
	int ret;

	/* Soft reset, then wait out the power-on/reset sequence. */
	ret = sx9310_write(data, SX9310_REG_SOFTRESET, SX9310_SOFTRESET_CMD);
	if (ret < 0)
		return ret;
	msleep(20);

	/* Clear the reset IRQ latched by the reset above. */
	sx9310_read(data, SX9310_REG_IRQ_SRC);

	ret = sx9310_apply_reg_init(data);
	if (ret < 0)
		return ret;

	/* Read the IRQ source once more so the line starts de-asserted. */
	sx9310_read(data, SX9310_REG_IRQ_SRC);
	return 0;
}

static void sx9310_process(struct sx9310_data *data)
{
	int stat = sx9310_read(data, SX9310_REG_STAT0);
	int i;
	bool near;

	if (stat < 0)
		return;

	for (i = 0; i < ARRAY_SIZE(sx9310_buttons); i++) {
		struct sx9310_button *b = &sx9310_buttons[i];

		near = !!(stat & b->mask & SX9310_PROXSTAT_MASK);
		if (near == b->pressed)
			continue;
		b->pressed = near;
		input_report_key(data->input, b->keycode, near ? 1 : 0);
		dev_dbg(&data->client->dev, "cap button %d %s\n",
			i, near ? "touched" : "released");
	}
	input_sync(data->input);
}

static irqreturn_t sx9310_irq_thread(int irq, void *dev_id)
{
	struct sx9310_data *data = dev_id;
	int src;

	mutex_lock(&data->lock);
	src = sx9310_read(data, SX9310_REG_IRQ_SRC);	/* cleared on read */
	if (src < 0)
		goto out;

	/* Ignore the reset/compensation/conversion housekeeping interrupts. */
	if (src & (SX9310_CLOSE_IRQ | SX9310_FAR_IRQ))
		sx9310_process(data);
out:
	mutex_unlock(&data->lock);
	return IRQ_HANDLED;
}

/* --- sysfs: reproduce the stock ABI (names + print formats) --- */

static ssize_t raw_data_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct sx9310_data *data = dev_get_drvdata(dev);
	ssize_t len = 0;
	int cs;

	mutex_lock(&data->lock);
	for (cs = 0; cs < ARRAY_SIZE(sx9310_buttons); cs++) {
		int use, avg, diff, off;

		sx9310_write(data, SX9310_REG_SENSOR_SEL, cs);
		use  = sx9310_read_s16(data, SX9310_REG_USE_MSB);
		avg  = sx9310_read_s16(data, SX9310_REG_AVG_MSB);
		diff = sx9310_read_s16(data, SX9310_REG_DIFF_MSB);
		off  = sx9310_read_s16(data, SX9310_REG_OFFSET_MSB);
		len += scnprintf(buf + len, PAGE_SIZE - len,
			"[CS: %d] Useful = %d Average = %d, DIFF = %d Offset = %d\n",
			cs, use, avg, diff, off);
	}
	mutex_unlock(&data->lock);
	return len;
}

static ssize_t register_read_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct sx9310_data *data = dev_get_drvdata(dev);
	unsigned int reg;
	int val;

	if (kstrtouint(buf, 0, &reg) < 0 || reg > 0xff)
		return -EINVAL;

	mutex_lock(&data->lock);
	val = sx9310_read(data, reg);
	mutex_unlock(&data->lock);
	if (val < 0)
		return val;

	dev_info(dev, "Register(0x%2x) data(0x%2x)\n", reg, val);
	return count;
}

static ssize_t register_write_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct sx9310_data *data = dev_get_drvdata(dev);
	unsigned int reg, val;
	int ret;

	if (sscanf(buf, "%x,%x", &reg, &val) != 2 &&
	    sscanf(buf, "%x %x", &reg, &val) != 2)
		return -EINVAL;
	if (reg > 0xff || val > 0xff)
		return -EINVAL;

	mutex_lock(&data->lock);
	ret = sx9310_write(data, reg, val);
	mutex_unlock(&data->lock);
	if (ret < 0)
		return ret;

	dev_info(dev, "Register(0x%2x) data(0x%2x)\n", reg, val);
	return count;
}

static DEVICE_ATTR_RO(raw_data);
static DEVICE_ATTR_WO(register_read);
static DEVICE_ATTR_WO(register_write);

static struct attribute *sx9310_attrs[] = {
	&dev_attr_raw_data.attr,
	&dev_attr_register_read.attr,
	&dev_attr_register_write.attr,
	NULL,
};

static const struct attribute_group sx9310_group = {
	.attrs = sx9310_attrs,
};

static int sx9310_parse_dt(struct sx9310_data *data)
{
	struct device *dev = &data->client->dev;
	struct device_node *np = dev->of_node;
	const u8 *prop;
	u32 reg_num;
	int len, i, ret;

	if (!np)
		return -EINVAL;

	ret = of_property_read_u32(np, "Semtech,reg-num", &reg_num);
	if (ret < 0) {
		dev_err(dev, "missing Semtech,reg-num\n");
		return ret;
	}

	prop = of_get_property(np, "Semtech,reg-init", &len);
	if (!prop) {
		dev_err(dev, "missing Semtech,reg-init\n");
		return -EINVAL;
	}
	if (len < (int)(reg_num * 2)) {
		dev_err(dev, "Semtech,reg-init too short: %d bytes for %u pairs\n",
			len, reg_num);
		return -EINVAL;
	}

	data->reg_num = reg_num;
	data->reg_init = devm_kcalloc(dev, reg_num, sizeof(*data->reg_init),
				      GFP_KERNEL);
	if (!data->reg_init)
		return -ENOMEM;

	for (i = 0; i < (int)reg_num; i++) {
		data->reg_init[i].reg = prop[i * 2];
		data->reg_init[i].val = prop[i * 2 + 1];
	}
	dev_info(dev, "parse_dt complete: %u register pairs\n", reg_num);

	data->irq_gpio = of_get_named_gpio(np, "Semtech,nirq-gpio", 0);
	if (!gpio_is_valid(data->irq_gpio)) {
		dev_err(dev, "invalid Semtech,nirq-gpio\n");
		return data->irq_gpio;
	}

	return 0;
}

static int sx9310_power_on(struct sx9310_data *data)
{
	int ret;

	data->vdd = devm_regulator_get_optional(&data->client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		ret = PTR_ERR(data->vdd);
		data->vdd = NULL;
		if (ret == -ENODEV)	/* board wired no separate rail */
			return 0;
		return ret;
	}

	ret = regulator_enable(data->vdd);
	if (ret < 0) {
		dev_err(&data->client->dev, "vdd enable failed: %d\n", ret);
		return ret;
	}
	usleep_range(1000, 2000);	/* let the rail settle */
	return 0;
}

static int sx9310_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct sx9310_data *data;
	int ret, whoami, i;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	mutex_init(&data->lock);
	i2c_set_clientdata(client, data);

	ret = sx9310_parse_dt(data);
	if (ret < 0)
		return ret;

	ret = sx9310_power_on(data);
	if (ret < 0)
		return ret;

	ret = devm_gpio_request_one(&client->dev, data->irq_gpio, GPIOF_IN,
				    "sx9310_nirq");
	if (ret < 0) {
		dev_err(&client->dev, "nirq gpio request failed: %d\n", ret);
		goto err_power;
	}
	client->irq = gpio_to_irq(data->irq_gpio);

	/* Hardware check: whoami is advisory (mismatch logged, not fatal). */
	whoami = sx9310_read(data, SX9310_REG_WHOAMI);
	if (whoami != SX9310_WHOAMI_VALUE)
		dev_warn(&client->dev,
			 "sx9310 failcode: unexpected whoami 0x%02x (want 0x%02x)\n",
			 whoami, SX9310_WHOAMI_VALUE);

	ret = sx9310_hw_init(data);
	if (ret < 0) {
		dev_err(&client->dev, "hw init failed: %d\n", ret);
		goto err_power;
	}

	data->input = devm_input_allocate_device(&client->dev);
	if (!data->input) {
		ret = -ENOMEM;
		goto err_power;
	}
	data->input->name = SX9310_INPUT_NAME;
	data->input->id.bustype = BUS_I2C;
	for (i = 0; i < ARRAY_SIZE(sx9310_buttons); i++)
		input_set_capability(data->input, EV_KEY,
				     sx9310_buttons[i].keycode);

	ret = input_register_device(data->input);
	if (ret < 0) {
		dev_err(&client->dev, "input register failed: %d\n", ret);
		goto err_power;
	}

	/* stock sysfs ABI: raw_data / register_read / register_write */
	ret = sysfs_create_group(&client->dev.kobj, &sx9310_group);
	if (ret < 0) {
		dev_err(&client->dev, "sysfs create failed: %d\n", ret);
		goto err_power;
	}

	/*
	 * nIRQ is active-low and held asserted until IRQ_SRC (0x00) is read
	 * (cleared on read), so a level-low one-shot threaded handler cannot
	 * miss or re-fire on a stuck line.
	 */
	ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
					sx9310_irq_thread,
					IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					SX9310_DRIVER_NAME, data);
	if (ret < 0) {
		dev_err(&client->dev, "request irq %d failed: %d\n",
			client->irq, ret);
		goto err_sysfs;
	}

	dev_info(&client->dev,
		 "SX9310 Cap Touch probed (UNVALIDATED on hardware)\n");
	return 0;

err_sysfs:
	sysfs_remove_group(&client->dev.kobj, &sx9310_group);
err_power:
	if (data->vdd)
		regulator_disable(data->vdd);
	return ret;
}

static int sx9310_remove(struct i2c_client *client)
{
	struct sx9310_data *data = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &sx9310_group);
	if (data->vdd)
		regulator_disable(data->vdd);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int sx9310_suspend(struct device *dev)
{
	struct sx9310_data *data = dev_get_drvdata(dev);

	/* The SAR decision only matters while the modem transmits (awake). */
	disable_irq(data->client->irq);
	return 0;
}

static int sx9310_resume(struct device *dev)
{
	struct sx9310_data *data = dev_get_drvdata(dev);

	enable_irq(data->client->irq);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(sx9310_pm_ops, sx9310_suspend, sx9310_resume);

static const struct i2c_device_id sx9310_id[] = {
	{ SX9310_DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sx9310_id);

static const struct of_device_id sx9310_of_match[] = {
	{ .compatible = "Semtech,sx9310", },
	{ }
};
MODULE_DEVICE_TABLE(of, sx9310_of_match);

static struct i2c_driver sx9310_driver = {
	.driver = {
		.name		= SX9310_DRIVER_NAME,
		.of_match_table	= sx9310_of_match,
		.pm		= &sx9310_pm_ops,
	},
	.probe		= sx9310_probe,
	.remove		= sx9310_remove,
	.id_table	= sx9310_id,
};
module_i2c_driver(sx9310_driver);

MODULE_DESCRIPTION("Semtech SX9310 SAR proximity driver (Lenovo Tab E10 LTE)");
MODULE_LICENSE("GPL v2");
