/*
 * Driver for IS31FL3191 (SN3191) Breath LED
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>

/* Registers [cite: 241, 244, 251, 260, 275, 254, 280, 286, 299, 291, 294, 302] */
#define SN3191_REG_SHUTDOWN		0x00
#define SN3191_REG_BREATH_CTRL		0x01
#define SN3191_REG_LED_MODE		0x02
#define SN3191_REG_CURRENT		0x03
#define SN3191_REG_PWM			0x04
#define SN3191_REG_PWM_UPDATE		0x07
#define SN3191_REG_T0			0x0A
#define SN3191_REG_T1_T2		0x10
#define SN3191_REG_T3_T4		0x16
#define SN3191_REG_TIME_UPDATE		0x1C
#define SN3191_REG_LED_CTRL		0x1D
#define SN3191_REG_RESET		0x2F

/* 02h LED Mode Register (Table 5): the LED-mode select bit is D5, not D0.
 *   D5=0 -> PWM Control Mode, D5=1 -> One Shot Programming Mode (auto breath). */
#define SN3191_MODE_PWM			0x00
#define SN3191_MODE_BREATH		0x20 /* D5=1: One Shot Programming (continuous breath) */

/* 00h Shutdown Register (Table 3): D5=EN (channel control), D0=SSD (sw shutdown). */
#define SN3191_CMD_ENABLE		0x20 /* EN=1, SSD=0 (normal, channel enabled) */
#define SN3191_CMD_SHUTDOWN		0x01 /* SSD=1 (software shutdown) */

/* 03h Current Setting Register (Table 6): the CS field occupies D4:D2, so each
 * code must be shifted left by 2. Datasheet IMAX codes (CS = D4:D2):
 *   000=42mA 001=10mA 010=5mA 011=30mA 1xx=17.5mA. IMAX is the breathing peak
 * and the full-scale of the 256-step PWM, so it sets the LED's max brightness. */
#define SN3191_CS_SHIFT			2
#define SN3191_CS_42MA			0x0
#define SN3191_CS_10MA			0x1
#define SN3191_CS_5MA			0x2
#define SN3191_CS_30MA			0x3
#define SN3191_CS_17P5MA		0x4
#define SN3191_CS_DEFAULT_UA		5000 /* gentle white indicator unless DT overrides */

struct sn3191_led_data {
	struct i2c_client *client;
	struct led_classdev cdev;
	struct mutex lock;
	struct regulator *vdd;
	int sdb_gpio;
	u8 cur_reg;	/* current-setting register value, pre-shifted into D4:D2 */
};

/* Map a requested full-scale current (microamp) to the nearest CS code that does
 * not exceed it (floor, min 5 mA) so the LED is never over-driven by rounding. */
static u8 sn3191_ua_to_cs(u32 ua)
{
	if (ua >= 42000)
		return SN3191_CS_42MA;
	if (ua >= 30000)
		return SN3191_CS_30MA;
	if (ua >= 17500)
		return SN3191_CS_17P5MA;
	if (ua >= 10000)
		return SN3191_CS_10MA;
	return SN3191_CS_5MA;
}

/* Time lookup tables (ms), datasheet Tables 9 (10h T1&T2) and 10 (16h T3&T4).
 * T1/T3 (rise/fall) have 8 steps starting at 0.13s; T4 (off) has 11 steps
 * starting at 0s. T2 (hold at peak) also exists (9 steps from 0s) but is always
 * programmed to index 0 = 0s here, matching the factory preset -- see
 * sn3191_blink_set() -- so it needs no table. */
static const int t_rise_fall_ms[] = { 130, 260, 520, 1040, 2080, 4160, 8320, 16640 };
static const int t_hold_ms[]      = { 0, 130, 260, 520, 1040, 2080, 4160, 8320, 16640 };
static const int t_off_ms[]       = { 0, 130, 260, 520, 1040, 2080, 4160, 8320, 16640, 33280, 66560 };

/* Find index that provides a time >= requested time (Ceiling) */
static int find_nearest_index(const int *table, int size, int ms)
{
	int i;
	for (i = 0; i < size - 1; i++) {
		if (ms <= table[i])
			return i;
	}
	return size - 1;
}

static int find_floor_index(const int *table, int size, int ms)
{
	int i;
	for (i = size - 1; i > 0; i--) {
		if (table[i] <= ms)
			return i;
	}
	return 0;
}

static int sn3191_write_reg(struct sn3191_led_data *led, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(led->client, reg, val);
}

/* brightness_set_blocking: i2c transfers may sleep, so this is the blocking
 * variant (the plain brightness_set callback must not sleep per the LED core). */
static int sn3191_set_brightness(struct led_classdev *cdev,
				 enum led_brightness brightness)
{
	struct sn3191_led_data *led = container_of(cdev, struct sn3191_led_data, cdev);
	int ret;

	mutex_lock(&led->lock);

	if (brightness == LED_OFF) {
		/* Software Shutdown */
		ret = sn3191_write_reg(led, SN3191_REG_SHUTDOWN, SN3191_CMD_SHUTDOWN);
	} else {
		/* Sequence for PWM Mode: IOUT = (IMAX/256) * PWM */
		sn3191_write_reg(led, SN3191_REG_SHUTDOWN, SN3191_CMD_ENABLE);
		sn3191_write_reg(led, SN3191_REG_LED_CTRL, 0x01); /* Enable OUT */
		sn3191_write_reg(led, SN3191_REG_LED_MODE, SN3191_MODE_PWM); /* PWM Control Mode */
		sn3191_write_reg(led, SN3191_REG_PWM, brightness); /* Set duty cycle */
		ret = sn3191_write_reg(led, SN3191_REG_PWM_UPDATE, 0x00); /* Latch (07h) */
	}

	mutex_unlock(&led->lock);

	return ret < 0 ? ret : 0;
}

static int sn3191_blink_set_locked(struct sn3191_led_data *led,
				   unsigned long *delay_on,
				   unsigned long *delay_off,
				   u8 pwm)
{
	int t1_idx, t2_idx, t3_idx, t4_idx;
	u8 t1_t2_val, t3_t4_val;

	if (!*delay_on && !*delay_off) {
		*delay_on = 1000;
		*delay_off = 1000;
	}

	if (!pwm)
		pwm = 255;

	sn3191_write_reg(led, SN3191_REG_LED_MODE, SN3191_MODE_PWM);

	t4_idx = find_nearest_index(t_off_ms, ARRAY_SIZE(t_off_ms), *delay_off);
	t1_idx = find_floor_index(t_rise_fall_ms, ARRAY_SIZE(t_rise_fall_ms), *delay_on / 3);
	t3_idx = t1_idx;
	if (*delay_on > (unsigned long)(2 * t_rise_fall_ms[t1_idx]))
		t2_idx = find_nearest_index(t_hold_ms, ARRAY_SIZE(t_hold_ms),
					    *delay_on - 2 * t_rise_fall_ms[t1_idx]);
	else
		t2_idx = 0;

	t1_t2_val = (t1_idx << 5) | (t2_idx << 1);
	t3_t4_val = (t3_idx << 5) | (t4_idx << 1);

	sn3191_write_reg(led, SN3191_REG_SHUTDOWN, SN3191_CMD_ENABLE);
	sn3191_write_reg(led, SN3191_REG_BREATH_CTRL, 0x00);
	sn3191_write_reg(led, SN3191_REG_T0, 0x00);
	sn3191_write_reg(led, SN3191_REG_T1_T2, t1_t2_val);
	sn3191_write_reg(led, SN3191_REG_T3_T4, t3_t4_val);
	sn3191_write_reg(led, SN3191_REG_TIME_UPDATE, 0x00);

	sn3191_write_reg(led, SN3191_REG_CURRENT, led->cur_reg);
	sn3191_write_reg(led, SN3191_REG_LED_CTRL, 0x01);

	sn3191_write_reg(led, SN3191_REG_PWM, pwm);
	sn3191_write_reg(led, SN3191_REG_PWM_UPDATE, 0x00);

	sn3191_write_reg(led, SN3191_REG_LED_MODE, SN3191_MODE_BREATH);

	return 0;
}

static int sn3191_blink_set(struct led_classdev *cdev,
			    unsigned long *delay_on,
			    unsigned long *delay_off)
{
	struct sn3191_led_data *led = container_of(cdev, struct sn3191_led_data, cdev);
	int ret;

	mutex_lock(&led->lock);
	ret = sn3191_blink_set_locked(led, delay_on, delay_off, 255);
	mutex_unlock(&led->lock);

	return ret;
}

static ssize_t sn3191_blink_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct sn3191_led_data *led = container_of(cdev, struct sn3191_led_data, cdev);
	unsigned long delay_on = 0, delay_off = 0;
	unsigned int pwm = 255;
	int parsed;

	parsed = sscanf(buf, "%lu %lu %u", &delay_on, &delay_off, &pwm);
	if (parsed >= 2) {
		if (delay_on == 0 || delay_off == 0) {
			sn3191_set_brightness(cdev, LED_OFF);
			return count;
		}
		if (pwm == 0)
			pwm = 1;
		else if (pwm > 255)
			pwm = 255;

		mutex_lock(&led->lock);
		sn3191_blink_set_locked(led, &delay_on, &delay_off, (u8)pwm);
		mutex_unlock(&led->lock);
		return count;
	}

	if (parsed == 1 && delay_on > 0) {
		delay_on = 1000;
		delay_off = 1000;
		mutex_lock(&led->lock);
		sn3191_blink_set_locked(led, &delay_on, &delay_off, 255);
		mutex_unlock(&led->lock);
		return count;
	}

	if (parsed == 1 && delay_on == 0) {
		sn3191_set_brightness(cdev, LED_OFF);
		return count;
	}

	return -EINVAL;
}

static DEVICE_ATTR(blink, 0664, NULL, sn3191_blink_store);

static struct attribute *sn3191_attrs[] = {
	&dev_attr_blink.attr,
	NULL,
};

static const struct attribute_group sn3191_attr_group = {
	.attrs = sn3191_attrs,
};

static const struct attribute_group *sn3191_attr_groups[] = {
	&sn3191_attr_group,
	NULL,
};

static int sn3191_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device_node *np = client->dev.of_node;
	struct sn3191_led_data *led;
	const char *led_name;
	int ret;

	led = devm_kzalloc(&client->dev, sizeof(*led), GFP_KERNEL);
	if (!led) return -ENOMEM;

	led->client = client;
	i2c_set_clientdata(client, led);
	mutex_init(&led->lock);

	/* GPIO Setup */
	led->sdb_gpio = of_get_named_gpio(np, "si-en,sdb-gpio", 0);
	if (gpio_is_valid(led->sdb_gpio)) {
		ret = devm_gpio_request_one(&client->dev, led->sdb_gpio, GPIOF_OUT_INIT_HIGH, "sn3191_sdb");
		if (ret < 0) dev_err(&client->dev, "SDB GPIO request failed\n");
	}

	/* Regulator Setup */
	led->vdd = devm_regulator_get(&client->dev, "vdd");
	if (!IS_ERR(led->vdd)) {
		ret = regulator_enable(led->vdd);
		if (ret) dev_err(&client->dev, "VDD enable failed\n");
	}

	/* Full-scale current (IMAX). Optional "si-en,max-current-ua" in the DT
	 * overrides the default; mapped to the nearest CS code that does not
	 * exceed the request (so the LED is never over-driven). */
	{
		u32 ua = SN3191_CS_DEFAULT_UA;

		of_property_read_u32(np, "si-en,max-current-ua", &ua);
		led->cur_reg = sn3191_ua_to_cs(ua) << SN3191_CS_SHIFT;
	}

	/* Chip Reset & Init: reset all registers to defaults (blank display),
	 * then program IMAX. RESET (2Fh<-0x00) leaves the chip in sw shutdown
	 * (Shutdown Register default 0x01), so the LED stays off until a write. */
	sn3191_write_reg(led, SN3191_REG_RESET, 0x00);
	sn3191_write_reg(led, SN3191_REG_CURRENT, led->cur_reg);

	/* Register LED */
	if (of_property_read_string(np, "si-en,led-name", &led_name))
		led->cdev.name = "sn3191-led";
	else
		led->cdev.name = led_name;

	of_property_read_string(np, "linux,default-trigger",
				&led->cdev.default_trigger);

	/* i2c can sleep -> blocking variant (brightness_set must not sleep). */
	led->cdev.brightness_set_blocking = sn3191_set_brightness;
	led->cdev.blink_set = sn3191_blink_set;
	led->cdev.max_brightness = 255;
	led->cdev.groups = sn3191_attr_groups;

	ret = led_classdev_register(&client->dev, &led->cdev);
	if (ret < 0) return ret;

	if (of_property_read_bool(np, "si-en,boot-blink") &&
	    (!saved_command_line ||
	     !strstr(saved_command_line, "androidboot.mode=charger"))) {
		unsigned long delay_on = 1000;
		unsigned long delay_off = 1000;

		mutex_lock(&led->lock);
		sn3191_blink_set_locked(led, &delay_on, &delay_off, 255);
		mutex_unlock(&led->lock);
	}

	dev_info(&client->dev, "IS31FL3191 (Bamboo) probed.\n");
	return 0;
}

static int sn3191_remove(struct i2c_client *client)
{
	struct sn3191_led_data *led = i2c_get_clientdata(client);
	led_classdev_unregister(&led->cdev);
	sn3191_write_reg(led, SN3191_REG_RESET, 0x00);
	if (gpio_is_valid(led->sdb_gpio)) gpio_set_value(led->sdb_gpio, 0);
	if (!IS_ERR(led->vdd)) regulator_disable(led->vdd);
	return 0;
}

static const struct of_device_id of_sn3191_match[] = {
	{ .compatible = "si-en,sn3191", }, { },
};
MODULE_DEVICE_TABLE(of, of_sn3191_match);

static const struct i2c_device_id sn3191_id[] = {
	{ "sn3191", 0 }, { },
};
MODULE_DEVICE_TABLE(i2c, sn3191_id);

static struct i2c_driver sn3191_driver = {
	.driver = {
		.name = "leds-sn3191",
		.owner = THIS_MODULE,
		.of_match_table = of_sn3191_match,
	},
	.probe = sn3191_probe,
	.remove = sn3191_remove,
	.id_table = sn3191_id,
};
module_i2c_driver(sn3191_driver);
MODULE_AUTHOR("Gemini & Fernando");
MODULE_DESCRIPTION("IS31FL3191 Driver for LineageOS");
MODULE_LICENSE("GPL v2");
