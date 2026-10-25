// SPDX-License-Identifier: GPL-2.0
/*
 * TI LM3697 11-bit backlight LED driver — minimal port.
 *
 * Used on the HTC Desire 530 (a16, MSM8909) for the panel backlight. The stock
 * a16 MDSS driver drove this chip through a "bl_ctrl_i2c" hook that does not
 * exist in the 4.9 MDSS core, so the panel is declared "bl_ctrl_wled" instead
 * and this led_classdev hangs off the "bkl-trigger" LED trigger that the 4.9
 * BL_WLED path already fires. The single Android-facing brightness node stays
 * mdss_fb's own "lcd-backlight"; this one is named "lm3697-bl" so it does not
 * collide with it (the led core silently renames the loser of that race to
 * "lcd-backlight_1", which the lights HAL never writes).
 *
 * This is a bring-up driver, not the full mainline LM3697 (which models each
 * control bank as a separate led_classdev via device_child nodes).
 *
 * Two hardware facts drive the shape of the code below; both were measured on
 * the a16 (20260725), see the register-level notes at lm3697_program():
 *
 *   - The chip's HWEN pin is MDSS's "qcom,platform-bklight-en-gpio", so every
 *     panel blank hardware-resets it and every register goes back to its
 *     power-on default. Programming only at probe therefore lasts exactly
 *     until the first screen-off.
 *   - The HVLED-to-bank mapping (OUTPUT_CFG) is set up by the bootloader and
 *     must not be overwritten blindly: forcing it to 0x00 routes all three
 *     strings to control bank A while the a16 only enables bank B, which is
 *     unlit at any brightness.
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>

#define LM3697_REG_REV			0x00
#define LM3697_REG_RESET		0x01
#define LM3697_REG_OUTPUT_CFG		0x10
#define LM3697_REG_CTRL_A_RAMP		0x11
#define LM3697_REG_CTRL_B_RAMP		0x12
#define LM3697_REG_RT_RAMP		0x14
#define LM3697_REG_RAMP_CFG		0x15
#define LM3697_REG_BRT_CFG		0x16
#define LM3697_REG_CTRL_A_FS_CURR	0x17
#define LM3697_REG_CTRL_B_FS_CURR	0x18
#define LM3697_REG_FEEDBACK_EN		0x19
#define LM3697_REG_BOOST_CTL		0x1a
#define LM3697_REG_PWM_CFG		0x1c
#define LM3697_REG_CTRL_A_BRT_LSB	0x20
#define LM3697_REG_CTRL_A_BRT_MSB	0x21
#define LM3697_REG_CTRL_B_BRT_LSB	0x22
#define LM3697_REG_CTRL_B_BRT_MSB	0x23
#define LM3697_REG_CTRL_ENABLE		0x24

#define LM3697_CTRL_A_EN		0x01
#define LM3697_CTRL_B_EN		0x02

/*
 * Enable both banks unless DT says otherwise (the a16 says 0x02, bank B only).
 * The remaining defaults are the stock a16 register values; OUTPUT_CFG has no
 * default on purpose — it is only written when "ti,output-cfg" is present,
 * otherwise whatever the bootloader programmed is left alone.
 */
#define LM3697_CTRL_ENABLE_DEFAULT	0x03
#define LM3697_BRT_CFG_DEFAULT		0x01
#define LM3697_BOOST_CTL_DEFAULT	0x03
#define LM3697_PWM_CFG_DEFAULT		0x0e

#define LM3697_MAX_BRIGHTNESS		255

struct lm3697_data {
	struct i2c_client *client;
	struct led_classdev led_dev;
	struct mutex lock;
	int enable_gpio;
	bool have_output_cfg;
	u32 output_cfg;
	u32 ctrl_enable;
	u32 brt_cfg;
	u32 boost_ctl;
	u32 pwm_cfg;
};

static int lm3697_write(struct lm3697_data *drv, u8 reg, u8 val)
{
	int ret = i2c_smbus_write_byte_data(drv->client, reg, val);

	/*
	 * Rate-limited: with HWEN low the chip is off the bus entirely (reads
	 * and writes NACK with -ENOTCONN), and MDSS can still push levels at us
	 * with the panel down, so a hard failure here is not necessarily a bug.
	 */
	if (ret < 0)
		dev_err_ratelimited(&drv->client->dev,
				    "i2c write reg 0x%02x = 0x%02x failed: %d\n",
				    reg, val, ret);
	return ret;
}

/*
 * Bring the chip back to a known configuration. Called lazily, before lighting
 * up, because MDSS owns the HWEN pin (its "qcom,platform-bklight-en-gpio") and
 * pulses it around every blank: measured on the a16 right after one
 * screen-off/on cycle, every register had reverted to its power-on default
 * (0x10=0x06 0x16=0x00 0x1a=0x00 0x1c=0x0c) even though probe had programmed
 * them. This mirrors what the stock HTC driver does.
 *
 * Deliberately NOT done here: the software reset (0x01) and the full-scale
 * current registers (0x17/0x18). The reset would drop the bootloader's
 * configuration — including the HVLED-to-bank mapping — on the boot path,
 * blanking the panel between probe and the first brightness write, and the
 * power-on default of the FS current registers (0x13, 20.2 mA) is already the
 * value the stock kernel relies on.
 */
static void lm3697_program(struct lm3697_data *drv)
{
	lm3697_write(drv, LM3697_REG_BOOST_CTL, drv->boost_ctl);
	lm3697_write(drv, LM3697_REG_PWM_CFG, drv->pwm_cfg);
	lm3697_write(drv, LM3697_REG_BRT_CFG, drv->brt_cfg);
	if (drv->have_output_cfg)
		lm3697_write(drv, LM3697_REG_OUTPUT_CFG, drv->output_cfg);
	lm3697_write(drv, LM3697_REG_CTRL_ENABLE, drv->ctrl_enable);
}

/*
 * Ask the chip itself instead of tracking what we last wrote: the bank-enable
 * register reads back as 0 both after an HWEN reset and after our own turn-off,
 * so one read covers every way the configuration can have been lost — including
 * paths that drop HWEN without routing a zero brightness through here.
 */
static bool lm3697_needs_program(struct lm3697_data *drv)
{
	int val = i2c_smbus_read_byte_data(drv->client, LM3697_REG_CTRL_ENABLE);

	/* A failed read is no reason to skip the programming. */
	if (val < 0)
		return true;

	return (val & drv->ctrl_enable) != drv->ctrl_enable;
}

/*
 * Registered as brightness_set_blocking: every update is an i2c transfer, so
 * it may sleep, and the led core promises callers that plain brightness_set
 * does not (led_set_brightness() is called from the MDSS backlight path). The
 * core hands blocking setters to its own work queue, which is what the stock
 * driver open-coded with schedule_work().
 */
static int lm3697_set_brightness(struct led_classdev *led_cdev,
				 enum led_brightness value)
{
	struct lm3697_data *drv =
		container_of(led_cdev, struct lm3697_data, led_dev);
	/*
	 * 11-bit brightness: the LSB register holds the low 3 bits. Stock runs
	 * them at 0x7 (register reads back masked) so a given MSB is as bright
	 * as the chip can make it, and zeroes both registers to turn off.
	 */
	u8 msb = value & 0xff;
	u8 lsb = value ? 0xff : 0x00;

	mutex_lock(&drv->lock);

	if (gpio_is_valid(drv->enable_gpio))
		gpio_set_value(drv->enable_gpio, value ? 1 : 0);

	if (value && lm3697_needs_program(drv))
		lm3697_program(drv);

	if (drv->ctrl_enable & LM3697_CTRL_A_EN) {
		lm3697_write(drv, LM3697_REG_CTRL_A_BRT_LSB, lsb);
		lm3697_write(drv, LM3697_REG_CTRL_A_BRT_MSB, msb);
	}
	if (drv->ctrl_enable & LM3697_CTRL_B_EN) {
		lm3697_write(drv, LM3697_REG_CTRL_B_BRT_LSB, lsb);
		lm3697_write(drv, LM3697_REG_CTRL_B_BRT_MSB, msb);
	}

	/* Also what makes the next lm3697_needs_program() say yes. */
	if (!value)
		lm3697_write(drv, LM3697_REG_CTRL_ENABLE, 0x00);

	mutex_unlock(&drv->lock);
	return 0;
}

static void lm3697_read_u32_default(struct device_node *np, const char *prop,
				    u32 *out, u32 def)
{
	if (of_property_read_u32(np, prop, out))
		*out = def;
}

static int lm3697_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device_node *np = client->dev.of_node;
	struct lm3697_data *drv;
	int ret;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "SMBUS byte data not supported\n");
		return -EIO;
	}

	drv = devm_kzalloc(&client->dev, sizeof(*drv), GFP_KERNEL);
	if (!drv)
		return -ENOMEM;

	drv->client = client;
	mutex_init(&drv->lock);
	i2c_set_clientdata(client, drv);

	/*
	 * No default: the HVLED-to-bank mapping is the bootloader's, and the
	 * power-on default (0x06: HVLED1 to bank A, HVLED2/3 to bank B) already
	 * matches how the a16 is wired. Only override it if DT asks.
	 */
	drv->have_output_cfg = !of_property_read_u32(np, "ti,output-cfg",
						     &drv->output_cfg);

	lm3697_read_u32_default(np, "ctl-bank-en", &drv->ctrl_enable,
				LM3697_CTRL_ENABLE_DEFAULT);
	lm3697_read_u32_default(np, "brightness-cfg", &drv->brt_cfg,
				LM3697_BRT_CFG_DEFAULT);
	lm3697_read_u32_default(np, "boost-ctl", &drv->boost_ctl,
				LM3697_BOOST_CTL_DEFAULT);
	lm3697_read_u32_default(np, "pwm-cfg", &drv->pwm_cfg,
				LM3697_PWM_CFG_DEFAULT);

	drv->enable_gpio = of_get_named_gpio(np, "enable-gpio", 0);
	if (gpio_is_valid(drv->enable_gpio)) {
		ret = devm_gpio_request_one(&client->dev, drv->enable_gpio,
					    GPIOF_OUT_INIT_HIGH,
					    "lm3697-hwen");
		if (ret) {
			dev_warn(&client->dev,
				 "enable-gpio %d request failed: %d\n",
				 drv->enable_gpio, ret);
			drv->enable_gpio = -EINVAL;
		} else {
			usleep_range(1000, 1500);
		}
	}

	/*
	 * Never write to the chip from probe: on a continuous-splash boot the
	 * bootloader has already programmed it and lit the panel, and any write
	 * (least of all a software reset) would black the screen out until
	 * userspace comes up with a brightness. One read is enough — it proves
	 * the chip is there (a wrong address NACKs here, which is how the 8-bit
	 * vs 7-bit address bug showed up) and tells us whether the bootloader
	 * left it configured. Everything else happens on the first brightness.
	 */
	ret = i2c_smbus_read_byte_data(client, LM3697_REG_CTRL_ENABLE);
	if (ret < 0) {
		dev_err(&client->dev, "chip not responding: %d\n", ret);
		return ret;
	}

	/*
	 * a16 20260827 (DEVICE-STATUS §2 0-ter): clear the panel-PWM gating
	 * right here, once.  With pwm-cfg=<0x0e> (what the bootloader leaves)
	 * emitted light = i2c brightness x the LEDPWM duty produced by the
	 * PANEL itself -- and that duty only exists after mdss_dsi_panel_on()
	 * sends DCS 0x51.  In a continuous-splash recovery nobody sends that
	 * DCS until the screensaver blanks and kills everything, so the chip
	 * sat there with canonical registers (BRT mirroring sysfs exactly --
	 * verified by a two-page register sweep: recovery 0x7f vs system
	 * 0xc8) while emitting nothing: backlight off AND image off from one
	 * single cause.  Writing the DT's pwm-cfg here opens the gate for the
	 * whole session, making brightness purely i2c everywhere (Android,
	 * recovery, charger), which is what the stock HTC flow relied on anyway.
	 *
	 * This deliberately rides on the same safety margin as the presence
	 * read above: HWEN is still high and LK still owns the display, so an
	 * i2c write succeeds reliably.  It is NOT the "don't write brightness
	 * in probe" case -- no brightness or enable bit changes here, only the
	 * pwm-gate configuration word whose DT value we own.
	 */
	ret = lm3697_write(drv, LM3697_REG_PWM_CFG, drv->pwm_cfg);
	if (!ret)
		dev_info(&client->dev,
			 "pwm gate cleared at probe (pwm-cfg=0x%02x)\n",
			 drv->pwm_cfg);

	drv->led_dev.name = "lm3697-bl";
	drv->led_dev.brightness_set_blocking = lm3697_set_brightness;
	drv->led_dev.max_brightness = LM3697_MAX_BRIGHTNESS;
	drv->led_dev.default_trigger = "bkl-trigger";

	ret = led_classdev_register(&client->dev, &drv->led_dev);
	if (ret < 0) {
		dev_err(&client->dev, "led_classdev register failed: %d\n",
			ret);
		return ret;
	}

	dev_info(&client->dev,
		 "LM3697 backlight registered as \"%s\" (banks 0x%02x, enable reg 0x%02x at probe)\n",
		 drv->led_dev.name, drv->ctrl_enable, ret);
	return 0;
}

static int lm3697_remove(struct i2c_client *client)
{
	struct lm3697_data *drv = i2c_get_clientdata(client);

	lm3697_write(drv, LM3697_REG_CTRL_ENABLE, 0x00);
	if (gpio_is_valid(drv->enable_gpio))
		gpio_set_value(drv->enable_gpio, 0);
	led_classdev_unregister(&drv->led_dev);
	return 0;
}

static const struct i2c_device_id lm3697_id[] = {
	{ "ti-lm3697", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lm3697_id);

static const struct of_device_id of_lm3697_match[] = {
	{ .compatible = "ti-lm3697", },
	{ .compatible = "ti,lm3697", },
	{ }
};
MODULE_DEVICE_TABLE(of, of_lm3697_match);

static struct i2c_driver lm3697_i2c_driver = {
	.probe = lm3697_probe,
	.remove = lm3697_remove,
	.id_table = lm3697_id,
	.driver = {
		.name = "leds-lm3697",
		.of_match_table = of_lm3697_match,
	},
};

module_i2c_driver(lm3697_i2c_driver);

MODULE_DESCRIPTION("TI LM3697 backlight LED driver (a16 bring-up)");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:leds-lm3697");
