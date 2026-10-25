/*
 * Qualcomm SMB358 Battery Charger Driver
 *
 * Copyright (c) 2014-2015 The Linux Foundation. All rights reserved.
 * Adapted for Linux 4.9 compatibility.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#define pr_fmt(fmt) "SMB358 %s: " fmt, __func__
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/mutex.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/regmap.h>
#include <linux/extcon.h>

#define _SMB358_MASK(BITS, POS) \
	((unsigned char)(((1 << (BITS)) - 1) << (POS)))
#define SMB358_MASK(LEFT_BIT_POS, RIGHT_BIT_POS) \
		_SMB358_MASK((LEFT_BIT_POS) - (RIGHT_BIT_POS) + 1, \
			(RIGHT_BIT_POS))

/* Config/Control registers */
#define CHG_CURRENT_CTRL_REG		0x0
#define CHG_OTH_CURRENT_CTRL_REG	0x1
#define VARIOUS_FUNC_REG		0x2
#define VFLOAT_REG			0x3
#define CHG_CTRL_REG			0x4
#define STAT_AND_TIMER_CTRL_REG		0x5
#define CHG_PIN_EN_CTRL_REG		0x6
#define THERM_A_CTRL_REG		0x7
#define SYSOK_AND_USB3_REG		0x8
#define OTHER_CTRL_REG			0x9
#define OTG_CONTROL			0xA
#define FAULT_INT_REG			0xC
#define STATUS_INT_REG			0xD

/* Command registers */
#define CMD_A_REG			0x30
#define CMD_B_REG			0x31

/* IRQ status registers */
#define IRQ_A_REG			0x35
#define IRQ_B_REG			0x36
#define IRQ_C_REG			0x37
#define IRQ_D_REG			0x38
#define IRQ_E_REG			0x39
#define IRQ_F_REG			0x3A

/* Status registers */
#define STATUS_C_REG			0x3D
#define STATUS_D_REG			0x3E
#define STATUS_E_REG			0x3F

#define SMB358_MAX_REGISTER		0x3F

/* Config bits */
#define CHG_INHI_EN_MASK			BIT(1)
#define CHG_INHI_EN_BIT				BIT(1)
#define CMD_A_CHG_ENABLE_BIT			BIT(1)
#define CMD_A_VOLATILE_W_PERM_BIT		BIT(7)
#define CMD_A_CHG_SUSP_EN_BIT			BIT(2)
#define CMD_A_CHG_SUSP_EN_MASK			BIT(2)
#define CMD_A_OTG_ENABLE_BIT			BIT(4)
#define CMD_A_OTG_ENABLE_MASK			BIT(4)
#define CMD_B_CHG_HC_ENABLE_BIT			BIT(0)
#define USB3_ENABLE_BIT				BIT(5)
#define USB3_ENABLE_MASK			BIT(5)
#define CMD_B_CHG_USB_500_900_ENABLE_BIT	BIT(1)
#define CHG_CTRL_AUTO_RECHARGE_ENABLE_BIT	0x0
#define CHG_CTRL_CURR_TERM_END_CHG_BIT		0x0
#define CHG_CTRL_BATT_MISSING_DET_THERM_IO	SMB358_MASK(5, 4)
#define CHG_CTRL_AUTO_RECHARGE_MASK		BIT(7)
#define CHG_AUTO_RECHARGE_DIS_BIT		BIT(7)
#define CHG_CTRL_CURR_TERM_END_MASK		BIT(6)
#define CHG_CTRL_BATT_MISSING_DET_MASK		SMB358_MASK(5, 4)
#define CHG_CTRL_APSD_EN_BIT			BIT(2)
#define CHG_CTRL_APSD_EN_MASK			BIT(2)
#define CHG_ITERM_MASK				0x07
#define CHG_PIN_CTRL_USBCS_REG_BIT		0x0
#define CHG_PIN_CTRL_CHG_EN_LOW_PIN_BIT		SMB358_MASK(6, 5)
#define CHG_PIN_CTRL_CHG_EN_LOW_REG_BIT		0x0
#define CHG_PIN_CTRL_CHG_EN_MASK		SMB358_MASK(6, 5)

#define CHG_LOW_BATT_THRESHOLD			SMB358_MASK(3, 0)
#define CHG_PIN_CTRL_USBCS_REG_MASK		BIT(4)
#define CHG_PIN_CTRL_APSD_IRQ_BIT		BIT(1)
#define CHG_PIN_CTRL_APSD_IRQ_MASK		BIT(1)
#define CHG_PIN_CTRL_CHG_ERR_IRQ_BIT		BIT(2)
#define CHG_PIN_CTRL_CHG_ERR_IRQ_MASK		BIT(2)
#define VARIOUS_FUNC_USB_SUSP_EN_REG_BIT	BIT(6)
#define VARIOUS_FUNC_USB_SUSP_MASK		BIT(6)
#define FAULT_INT_HOT_COLD_HARD_BIT		BIT(7)
#define FAULT_INT_HOT_COLD_SOFT_BIT		BIT(6)
#define FAULT_INT_INPUT_OV_BIT			BIT(3)
#define FAULT_INT_INPUT_UV_BIT			BIT(2)
#define FAULT_INT_AICL_COMPLETE_BIT		BIT(1)
#define STATUS_INT_CHG_TIMEOUT_BIT		BIT(7)
#define STATUS_INT_OTG_DETECT_BIT		BIT(6)
#define STATUS_INT_BATT_OV_BIT			BIT(5)
#define STATUS_INT_CHGING_BIT			BIT(4)
#define STATUS_INT_CHG_INHI_BIT			BIT(3)
#define STATUS_INT_INOK_BIT			BIT(2)
#define STATUS_INT_MISSING_BATT_BIT		BIT(1)
#define STATUS_INT_LOW_BATT_BIT			BIT(0)
#define THERM_A_THERM_MONITOR_EN_BIT		0x0
#define THERM_A_THERM_MONITOR_EN_MASK		BIT(4)
#define VFLOAT_MASK				0x3F

/* IRQ status bits */
#define IRQ_A_HOT_HARD_BIT			BIT(6)
#define IRQ_A_COLD_HARD_BIT			BIT(4)
#define IRQ_A_HOT_SOFT_BIT			BIT(2)
#define IRQ_A_COLD_SOFT_BIT			BIT(0)
#define IRQ_B_BATT_MISSING_BIT			BIT(4)
#define IRQ_B_BATT_LOW_BIT			BIT(2)
#define IRQ_B_BATT_OV_BIT			BIT(6)
#define IRQ_B_PRE_FAST_CHG_BIT			BIT(0)
#define IRQ_C_TAPER_CHG_BIT			BIT(2)
#define IRQ_C_TERM_BIT				BIT(0)
#define IRQ_C_INT_OVER_TEMP_BIT			BIT(6)
#define IRQ_D_CHG_TIMEOUT_BIT			(BIT(0) | BIT(2))
#define IRQ_D_AICL_DONE_BIT			BIT(4)
#define IRQ_D_APSD_COMPLETE			BIT(6)
#define IRQ_E_INPUT_UV_BIT			BIT(0)
#define IRQ_E_INPUT_OV_BIT			BIT(2)
#define IRQ_E_AFVC_ACTIVE                       BIT(4)
#define IRQ_F_OTG_VALID_BIT			BIT(2)
#define IRQ_F_OTG_BATT_FAIL_BIT			BIT(4)
#define IRQ_F_OTG_OC_BIT			BIT(6)
#define IRQ_F_POWER_OK				BIT(0)

/* Status bits */
#define STATUS_C_CHARGING_MASK			SMB358_MASK(2, 1)
#define STATUS_C_FAST_CHARGING			BIT(2)
#define STATUS_C_PRE_CHARGING			BIT(1)
#define STATUS_C_TAPER_CHARGING			SMB358_MASK(2, 1)
#define STATUS_C_CHG_ERR_STATUS_BIT		BIT(6)
#define STATUS_C_CHG_ENABLE_STATUS_BIT		BIT(0)
#define STATUS_C_CHG_HOLD_OFF_BIT		BIT(3)
#define STATUS_D_CHARGING_PORT_MASK		SMB358_MASK(3, 0)
#define STATUS_D_PORT_ACA_DOCK			BIT(3)
#define STATUS_D_PORT_SDP			BIT(2)
#define STATUS_D_PORT_DCP			BIT(1)
#define STATUS_D_PORT_CDP			BIT(0)
#define STATUS_D_PORT_OTHER			SMB358_MASK(1, 0)
#define STATUS_D_PORT_ACA_A			(BIT(2) | BIT(0))
#define STATUS_D_PORT_ACA_B			SMB358_MASK(2, 1)
#define STATUS_D_PORT_ACA_C			SMB358_MASK(2, 0)

/* constants */
#define USB2_MIN_CURRENT_MA		100
#define USB2_MAX_CURRENT_MA		500
#define USB3_MIN_CURRENT_MA		150
#define USB3_MAX_CURRENT_MA		900
/* Input current to open for a dumb wall charger (DCP/CDP), no enumeration */
#define SMB358_DCP_INPUT_CURRENT_MA	1500
#define AC_CHG_CURRENT_MASK		0x70
#define AC_CHG_CURRENT_SHIFT		4
#define SMB358_IRQ_REG_COUNT		6
#define SMB358_FAST_CHG_MIN_MA		200
#define SMB358_FAST_CHG_MAX_MA		2000
#define SMB358_FAST_CHG_SHIFT		5
#define SMB_FAST_CHG_CURRENT_MASK	0xE0
#define SMB358_DEFAULT_BATT_CAPACITY	50
#define SMB358_BATT_GOOD_THRE_2P5	0x1

enum {
	USER		= BIT(0),
	THERMAL		= BIT(1),
	CURRENT		= BIT(2),
	SOC		= BIT(3),
	FAKE_BATTERY	= BIT(4),
	/*
	 * Battery temperature out of range (smb358_temp_work). Distinct from
	 * THERMAL, which is the thermal-engine's mitigation level: they must be
	 * able to hold charging off independently, or whichever released last
	 * would re-enable it behind the other's back.
	 */
	BATT_TEMP	= BIT(5),
};

struct smb358_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

struct smb358_charger {
	struct i2c_client	*client;
	struct device		*dev;
	struct regmap		*regmap;

	bool			inhibit_disabled;
	bool			recharge_disabled;
	int			recharge_mv;
	bool			iterm_disabled;
	int			iterm_ma;
	int			vfloat_mv;
	int			chg_valid_gpio;
	int			chg_valid_act_low;
	int			chg_present;
	int			fake_battery_soc;
	bool			chg_autonomous_mode;
	bool			disable_apsd;
	bool			using_pmic_therm;
	bool			pmic_vbat_sns;
	bool			battery_missing;
	const char		*bms_psy_name;
	bool			resume_completed;
	bool			irq_waiting;
	bool			bms_controlled_charging;
	bool			skip_usb_suspend_for_fake_battery;

	/*
	 * Software battery-temperature charge suspend. Boards that route the pack
	 * thermistor to the PMIC's adc_tm (qcom,using-pmic-therm) get the hardware
	 * JEITA path instead; this is for the ones that do not, and it is opt-in
	 * through the two threshold properties. See smb358_temp_work().
	 */
	struct delayed_work	temp_work;
	bool			temp_suspend_supported;
	bool			temp_suspended;
	int			temp_hot_decidegc;
	int			temp_cold_decidegc;
	int			temp_suspended_at;
	unsigned long		temp_last_change;	/* boottime seconds */

	struct mutex		path_suspend_lock;
	struct mutex		irq_complete;
	
	u8			irq_cfg_mask[2];
	int			irq_gpio;
	int			charging_disabled;
	int			battery_id_info;
	unsigned int		thermal_levels;
	unsigned int		therm_lvl_sel;
	unsigned int		*thermal_mitigation;
	int			fastchg_current_max_ma;
	int			dcp_input_current_ma;
	unsigned int		cool_bat_ma;
	unsigned int		warm_bat_ma;
	unsigned int		cool_bat_mv;
	unsigned int		warm_bat_mv;
	unsigned int		connected_rid;

	/* status tracking */
	bool			batt_full;
	bool			batt_hot;
	bool			batt_cold;
	bool			batt_warm;
	bool			batt_cool;
	bool			jeita_supported;
	int			charging_disabled_status;
	int			usb_suspended;

	/* power supply */
	struct power_supply	*usb_psy;
	struct power_supply	*bms_psy;
	struct power_supply	*batt_psy;

	/*
	 * USB cable detect -> extcon. In 3.18 the SMB358 pushed VBUS PRESENT
	 * into msm_otg's "usb" power_supply; in 4.9 msm_otg consumes VBUS only
	 * via extcon EXTCON_USB, so bridge chg_present to an extcon device that
	 * usb_otg references with "extcon = <&smb358_otg_supply>".
	 */
	struct extcon_dev	*extcon_dev;
	int			vbus_notified;

	/* USB PSY state (owned by this driver in 4.9) */
	enum power_supply_type	usb_supply_type;
	int			usb_health;
	int			usb_current_max;

	/* otg 5V regulator */
	struct smb358_regulator	otg_vreg;

	/* adc_tm parameters */
	struct qpnp_vadc_chip		*vadc_dev;
	struct qpnp_adc_tm_chip		*adc_tm_dev;
	struct qpnp_adc_tm_btm_param	adc_param;
	/*
	 * qpnp-adc-tm lives on the SPMI bus and registers a few hundred ms
	 * after this i2c driver probes, and probe deliberately does not defer
	 * (see smb358_probe). Arming the hardware JEITA therefore has to be
	 * retried from a worker instead of given up on. See smb358_arm_adc_tm().
	 */
	struct delayed_work		adc_tm_arm_work;
	int				adc_tm_arm_retries;
	bool				adc_tm_armed;
	int				cold_bat_decidegc;
	int				hot_bat_decidegc;
	int				cool_bat_decidegc;
	int				warm_bat_decidegc;
	int				bat_present_decidegc;
	
	/* i2c pull up regulator */
	struct regulator		*vcc_i2c;

	/* Procfs entries */
	struct proc_dir_entry		*otg_proc;
	struct proc_dir_entry		*batt_proc;
	int				otg_enable;
	const char			*battery_name;
};

struct smb_irq_info {
	const char	*name;
	int		(*smb_irq)(struct smb358_charger *chip, u8 rt_stat);
	int		high;
	int		low;
};

struct irq_handler_info {
	u8			stat_reg;
	u8			val;
	u8			prev_val;
	struct smb_irq_info	irq_info[4];
};

static int chg_current[] = {
	300, 500, 700, 1000, 1200, 1500, 1800, 2000,
};

static int fast_chg_current[] = {
	200, 450, 600, 900, 1300, 1500, 1800, 2000,
};

static char *pm_batt_supplied_to[] = {
	"bms",
};

static int64_t read_battery_id(struct smb358_charger *chip);

/*
 * Regmap volatile & readable callbacks to ensure proper caching behavior.
 */
static bool smb358_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case IRQ_A_REG:
	case IRQ_B_REG:
	case IRQ_C_REG:
	case IRQ_D_REG:
	case IRQ_E_REG:
	case IRQ_F_REG:
	case STATUS_C_REG:
	case STATUS_D_REG:
	case STATUS_E_REG:
	case CMD_A_REG:
	case CMD_B_REG:
		return true;
	}
	return false;
}

static bool smb358_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case CHG_CURRENT_CTRL_REG:
	case CHG_OTH_CURRENT_CTRL_REG:
	case VARIOUS_FUNC_REG:
	case VFLOAT_REG:
	case CHG_CTRL_REG:
	case STAT_AND_TIMER_CTRL_REG:
	case CHG_PIN_EN_CTRL_REG:
	case THERM_A_CTRL_REG:
	case SYSOK_AND_USB3_REG:
	case OTHER_CTRL_REG:
	case OTG_CONTROL:
	case FAULT_INT_REG:
	case STATUS_INT_REG:
	case CMD_A_REG:
	case CMD_B_REG:
		return true;
	}

	return smb358_volatile_reg(dev, reg);
}

static const struct regmap_config smb358_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = SMB358_MAX_REGISTER,
	.volatile_reg = smb358_volatile_reg,
	.readable_reg = smb358_readable_reg,
};

static int smb358_read_reg(struct smb358_charger *chip, int reg, u8 *val)
{
	unsigned int temp;
	int rc;

	rc = regmap_read(chip->regmap, reg, &temp);
	if (rc < 0) {
		dev_err(chip->dev, "i2c read fail: can't read from %02x: %d\n", reg, rc);
		return rc;
	}
	*val = (u8)temp;
	return 0;
}

static int smb358_write_reg(struct smb358_charger *chip, int reg, u8 val)
{
	int rc;

	rc = regmap_write(chip->regmap, reg, val);
	if (rc < 0) {
		dev_err(chip->dev, "i2c write fail: can't write %02x to %02x: %d\n",
			val, reg, rc);
	}
	return rc;
}

static int smb358_masked_write(struct smb358_charger *chip, int reg, u8 mask, u8 val)
{
	int rc;

	rc = regmap_update_bits(chip->regmap, reg, mask, val);
	if (rc < 0) {
		dev_err(chip->dev, "smb358_masked_write Failed: reg=%03X, rc=%d\n", reg, rc);
	}
	return rc;
}

/*
 * Lazy VADC/ADC_TM acquisition helpers.
 *
 * These subsystems may not be available at smb358 probe time (they init
 * later), but the OTG regulator and USB PSY must be registered on the
 * first probe attempt so that ehci-msm and msm_otg can find them.
 * Moving VADC/ADC_TM out of probe avoids -EPROBE_DEFER before the
 * critical registrations happen.
 */
static struct qpnp_vadc_chip *smb358_get_vadc(struct smb358_charger *chip)
{
	if (IS_ERR_OR_NULL(chip->vadc_dev))
		chip->vadc_dev = qpnp_get_vadc(chip->dev, "chg");
	return chip->vadc_dev;
}

static struct qpnp_adc_tm_chip *smb358_get_adc_tm(struct smb358_charger *chip)
{
	if (IS_ERR_OR_NULL(chip->adc_tm_dev))
		chip->adc_tm_dev = qpnp_get_adc_tm(chip->dev, "chg");
	return chip->adc_tm_dev;
}

/*
 * USB power supply implementation.
 *
 * In the 4.9 Qualcomm BSP, the "usb" PSY is owned by the charger driver
 * (not by msm_otg as in 3.18). msm_otg consumes it via
 * power_supply_get_by_name("usb") and listens for change notifications.
 */
static enum power_supply_property smb358_usb_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	/*
	 * The 4.9 msm_otg reports the negotiated current via SDP_CURRENT_MAX
	 * (newer Qualcomm charger ABI), so accept it as an alias for the input
	 * current limit. Without this the set silently fails (-EINVAL) and the
	 * input stays clamped to 100mA.
	 */
	POWER_SUPPLY_PROP_SDP_CURRENT_MAX,
};

static int smb358_usb_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct smb358_charger *chip = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = chip->chg_present;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chip->chg_present && !chip->usb_suspended;
		break;
	case POWER_SUPPLY_PROP_TYPE:
		val->intval = chip->usb_supply_type;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = chip->usb_health;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case POWER_SUPPLY_PROP_SDP_CURRENT_MAX:
		val->intval = chip->usb_current_max;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int smb358_usb_set_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   const union power_supply_propval *val)
{
	struct smb358_charger *chip = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case POWER_SUPPLY_PROP_SDP_CURRENT_MAX:
		chip->usb_current_max = val->intval;
		break;
	case POWER_SUPPLY_PROP_TYPE:
		chip->usb_supply_type = val->intval;
		break;
	default:
		return -EINVAL;
	}
	power_supply_changed(psy);
	return 0;
}

static int smb358_usb_property_is_writeable(struct power_supply *psy,
					    enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case POWER_SUPPLY_PROP_SDP_CURRENT_MAX:
	case POWER_SUPPLY_PROP_TYPE:
		return 1;
	default:
		return 0;
	}
}

/* Not const: .type is retargeted from apsd_complete() on port detection. */
static struct power_supply_desc smb358_usb_psy_desc = {
	.name		= "usb",
	.type		= POWER_SUPPLY_TYPE_USB,
	.properties	= smb358_usb_properties,
	.num_properties	= ARRAY_SIZE(smb358_usb_properties),
	.get_property	= smb358_usb_get_property,
	.set_property	= smb358_usb_set_property,
	.property_is_writeable = smb358_usb_property_is_writeable,
};

static int smb358_otg_set_limit_current(struct smb358_charger *chip)
{
	int rc;
	u8 reg;
	
	rc = smb358_masked_write(chip, OTG_CONTROL, SMB358_MASK(3,2), 0xC);
	if (rc)
		dev_err(chip->dev, "Couldn't write OTG_CONTROL rc=%d\n", rc);
		
	rc = smb358_read_reg(chip, OTG_CONTROL, &reg);
	pr_err("smb358 otg set control %2x\n", reg);
	
	return rc;
}

static int smb358_enable_volatile_writes(struct smb358_charger *chip)
{
	int rc;

	rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_VOLATILE_W_PERM_BIT,
						CMD_A_VOLATILE_W_PERM_BIT);
	if (rc)
		dev_err(chip->dev, "Couldn't write VOLATILE_W_PERM_BIT rc=%d\n", rc);

	return rc;
}

static int smb358_fastchg_current_set(struct smb358_charger *chip,
					unsigned int fastchg_current)
{
	int i;

	if ((fastchg_current < SMB358_FAST_CHG_MIN_MA) ||
		(fastchg_current >  SMB358_FAST_CHG_MAX_MA)) {
		dev_dbg(chip->dev, "bad fastchg current mA=%d asked to set\n",
						fastchg_current);
		return -EINVAL;
	}

	for (i = ARRAY_SIZE(fast_chg_current) - 1; i >= 0; i--) {
		if (fast_chg_current[i] <= fastchg_current)
			break;
	}

	if (i < 0) {
		dev_err(chip->dev, "Invalid current setting %dmA\n",
						fastchg_current);
		i = 0;
	}

	i = i << SMB358_FAST_CHG_SHIFT;
	dev_dbg(chip->dev, "fastchg limit=%d setting %02x\n",
					fastchg_current, i);

	return smb358_masked_write(chip, CHG_CURRENT_CTRL_REG,
				SMB_FAST_CHG_CURRENT_MASK, i);
}

#define MIN_FLOAT_MV		3500
#define MAX_FLOAT_MV		4500
#define VFLOAT_STEP_MV		20
#define VFLOAT_4350MV		4350
static int smb358_float_voltage_set(struct smb358_charger *chip, int vfloat_mv)
{
	u8 temp;

	if ((vfloat_mv < MIN_FLOAT_MV) || (vfloat_mv > MAX_FLOAT_MV)) {
		dev_err(chip->dev, "bad float voltage mv =%d asked to set\n",
					vfloat_mv);
		return -EINVAL;
	}

	if (VFLOAT_4350MV == vfloat_mv)
		temp = 0x2B;
	else if (vfloat_mv > VFLOAT_4350MV)
		temp = (vfloat_mv - MIN_FLOAT_MV) / VFLOAT_STEP_MV + 1;
	else
		temp = (vfloat_mv - MIN_FLOAT_MV) / VFLOAT_STEP_MV;

	return smb358_masked_write(chip, VFLOAT_REG, VFLOAT_MASK, temp);
}

#define CHG_ITERM_30MA			0x00
#define CHG_ITERM_40MA			0x01
#define CHG_ITERM_60MA			0x02
#define CHG_ITERM_80MA			0x03
#define CHG_ITERM_100MA			0x04
#define CHG_ITERM_125MA			0x05
#define CHG_ITERM_150MA			0x06
#define CHG_ITERM_200MA			0x07
static int smb358_term_current_set(struct smb358_charger *chip)
{
	u8 reg = 0;
	int rc;

	if (chip->iterm_ma != -EINVAL) {
		if (chip->iterm_disabled)
			dev_err(chip->dev, "Error: Both iterm_disabled and iterm_ma set\n");

		if (chip->iterm_ma <= 30)
			reg = CHG_ITERM_30MA;
		else if (chip->iterm_ma <= 40)
			reg = CHG_ITERM_40MA;
		else if (chip->iterm_ma <= 60)
			reg = CHG_ITERM_60MA;
		else if (chip->iterm_ma <= 80)
			reg = CHG_ITERM_80MA;
		else if (chip->iterm_ma <= 100)
			reg = CHG_ITERM_100MA;
		else if (chip->iterm_ma <= 125)
			reg = CHG_ITERM_125MA;
		else if (chip->iterm_ma <= 150)
			reg = CHG_ITERM_150MA;
		else
			reg = CHG_ITERM_200MA;

		rc = smb358_masked_write(chip, CHG_CURRENT_CTRL_REG,
							CHG_ITERM_MASK, reg);
		if (rc) {
			dev_err(chip->dev,
				"Couldn't set iterm rc = %d\n", rc);
			return rc;
		}
	}

	if (chip->iterm_disabled) {
		rc = smb358_masked_write(chip, CHG_CTRL_REG,
					CHG_CTRL_CURR_TERM_END_MASK,
					CHG_CTRL_CURR_TERM_END_MASK);
		if (rc) {
			dev_err(chip->dev, "Couldn't set iterm rc = %d\n", rc);
			return rc;
		}
	} else {
		rc = smb358_masked_write(chip, CHG_CTRL_REG,
					CHG_CTRL_CURR_TERM_END_MASK, 0);
		if (rc) {
			dev_err(chip->dev, "Couldn't enable iterm rc = %d\n", rc);
			return rc;
		}
	}

	return 0;
}

#define VFLT_300MV			0x0C
#define VFLT_200MV			0x08
#define VFLT_100MV			0x04
#define VFLT_50MV			0x00
#define VFLT_MASK			0x0C
static int smb358_recharge_and_inhibit_set(struct smb358_charger *chip)
{
	u8 reg = 0;
	int rc;

	if (chip->recharge_disabled)
		rc = smb358_masked_write(chip, CHG_CTRL_REG,
		CHG_CTRL_AUTO_RECHARGE_MASK, CHG_AUTO_RECHARGE_DIS_BIT);
	else
		rc = smb358_masked_write(chip, CHG_CTRL_REG,
			CHG_CTRL_AUTO_RECHARGE_MASK, 0x0);
	if (rc) {
		dev_err(chip->dev, "Couldn't set auto recharge en reg rc = %d\n", rc);
	}

	if (chip->inhibit_disabled)
		rc = smb358_masked_write(chip, CHG_OTH_CURRENT_CTRL_REG,
					CHG_INHI_EN_MASK, 0x0);
	else
		rc = smb358_masked_write(chip, CHG_OTH_CURRENT_CTRL_REG,
					CHG_INHI_EN_MASK, CHG_INHI_EN_BIT);
	if (rc) {
		dev_err(chip->dev, "Couldn't set inhibit en reg rc = %d\n", rc);
	}

	if (chip->recharge_mv != -EINVAL) {
		if (chip->recharge_mv <= 50)
			reg = VFLT_50MV;
		else if (chip->recharge_mv <= 100)
			reg = VFLT_100MV;
		else if (chip->recharge_mv <= 200)
			reg = VFLT_200MV;
		else
			reg = VFLT_300MV;

		rc = smb358_masked_write(chip, CHG_OTH_CURRENT_CTRL_REG,
						VFLT_MASK, reg);
		if (rc) {
			dev_err(chip->dev, "Couldn't set inhibit threshold rc = %d\n", rc);
			return rc;
		}
	}

	return 0;
}

/* OTG Procfs Handler */
#define OTG_PROC_FILE "smb358_otg_enable"

static int smb358_otg_read_proc(struct seq_file *m, void *data)
{
	struct smb358_charger *chip = m->private;
	seq_printf(m, "%d\n", chip->otg_enable);
	return 0;
}

static ssize_t smb358_otg_write_proc(struct file *filp, const char __user *buffer, size_t count, loff_t *off)
{
	struct smb358_charger *chip = PDE_DATA(file_inode(filp));
	int rc = 0, retry = 0;
	long cmode;
	char tmp[32];
	size_t len = min(count, sizeof(tmp) - 1);

	if (copy_from_user(tmp, buffer, len))
		return -EFAULT;
	tmp[len] = '\0';

	if (kstrtol(tmp, 10, &cmode) == 0) {
		if (cmode == 1 || cmode == 0) {
			chip->otg_enable = cmode;
		}
	}

	if (chip->otg_enable) {
		for (retry = 3; retry > 0; retry--) {
			rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_OTG_ENABLE_BIT, CMD_A_OTG_ENABLE_BIT);
			if (!rc) break;
			dev_err(chip->dev, "Couldn't enable OTG mode rc=%d\n", rc);
		}
	} else {
		for (retry = 3; retry > 0; retry--) {
			rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_OTG_ENABLE_BIT, 0);
			if (!rc) break;
			dev_err(chip->dev, "Couldn't disable OTG mode rc=%d\n", rc);
		}
	}

	return count;
}

static int smb358_otg_open_proc(struct inode *inode, struct file *file)
{
	struct smb358_charger *chip = PDE_DATA(inode);
	return single_open(file, smb358_otg_read_proc, chip);
}

static const struct file_operations smb358_otg_proc_ops = {
	.owner		= THIS_MODULE,
	.open		= smb358_otg_open_proc,
	.release	= single_release,
	.llseek		= seq_lseek,
	.read		= seq_read,
	.write		= smb358_otg_write_proc,
};

static int battery_proc_show(struct seq_file *seq, void *v)
{
	struct smb358_charger *chip = seq->private;
	seq_puts(seq, (const char *)chip->battery_name);
	return 0;
}

static int battery_proc_open(struct inode *inode, struct file *file)
{
	struct smb358_charger *chip = PDE_DATA(inode);
	return single_open(file, battery_proc_show, chip);
}

static const struct file_operations battery_type_fops = {
	.open		= battery_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int smb358_chg_otg_regulator_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smb358_charger *chip = rdev_get_drvdata(rdev);

	/*
	 * Boost VBUS whenever a regulator consumer (msm_otg on host entry)
	 * enables us. This used to be gated behind chip->otg_enable, a flag
	 * only ever set by the debug procfs node (default 0), so the regulator
	 * enable from msm_otg wrote nothing and host devices got no 5V. Keep
	 * the flag in sync for the procfs/is_enabled view.
	 */
	rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_OTG_ENABLE_BIT,
						CMD_A_OTG_ENABLE_BIT);
	if (rc)
		dev_err(chip->dev, "Couldn't enable OTG mode rc=%d, reg=%2x\n",
								rc, CMD_A_REG);
	else
		chip->otg_enable = true;

	return rc;
}

static int smb358_chg_otg_regulator_disable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smb358_charger *chip = rdev_get_drvdata(rdev);

	rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_OTG_ENABLE_BIT, 0);
	if (rc)
		dev_err(chip->dev, "Couldn't disable OTG mode rc=%d, reg=%2x\n",
								rc, CMD_A_REG);
	else
		chip->otg_enable = false;
	return rc;
}

static int smb358_chg_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	u8 reg = 0;
	struct smb358_charger *chip = rdev_get_drvdata(rdev);

	rc = smb358_read_reg(chip, CMD_A_REG, &reg);
	if (rc) {
		dev_err(chip->dev,
			"Couldn't read OTG enable bit rc=%d, reg=%2x\n",
							rc, CMD_A_REG);
		return rc;
	}

	return  (reg & CMD_A_OTG_ENABLE_BIT) ? 1 : 0;
}

struct regulator_ops smb358_chg_otg_reg_ops = {
	.enable		= smb358_chg_otg_regulator_enable,
	.disable	= smb358_chg_otg_regulator_disable,
	.is_enabled	= smb358_chg_otg_regulator_is_enable,
};

static int smb358_regulator_init(struct smb358_charger *chip)
{
	int rc = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};

	if (smb358_masked_write(chip, CMD_A_REG, CMD_A_OTG_ENABLE_BIT, 0)) {
		pr_err("Couldn't disable OTG mode at init, reg=%2x\n", CMD_A_REG);
	}

	init_data = of_get_regulator_init_data(chip->dev, chip->dev->of_node,
					       &chip->otg_vreg.rdesc);
	if (!init_data) {
		dev_err(chip->dev, "Allocate memory failed\n");
		return -ENOMEM;
	}

	/* Give the name, then will register */
	if (init_data->constraints.name) {
		chip->otg_vreg.rdesc.owner = THIS_MODULE;
		chip->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		chip->otg_vreg.rdesc.ops = &smb358_chg_otg_reg_ops;
		chip->otg_vreg.rdesc.name = init_data->constraints.name;

		cfg.dev = chip->dev;
		cfg.init_data = init_data;
		cfg.driver_data = chip;
		cfg.of_node = chip->dev->of_node;

		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_STATUS;

		chip->otg_vreg.rdev = devm_regulator_register(chip->dev,
					&chip->otg_vreg.rdesc, &cfg);
		if (IS_ERR(chip->otg_vreg.rdev)) {
			rc = PTR_ERR(chip->otg_vreg.rdev);
			chip->otg_vreg.rdev = NULL;
			if (rc != -EPROBE_DEFER)
				dev_err(chip->dev, "OTG reg failed, rc=%d\n", rc);
		}
	}
	return rc;
}

static int __smb358_path_suspend(struct smb358_charger *chip, bool suspend)
{
	int rc;

	rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_CHG_SUSP_EN_MASK,
					suspend ? CMD_A_CHG_SUSP_EN_BIT : 0);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set CMD_A reg, rc = %d\n", rc);

	return rc;
}

static int smb358_path_suspend(struct smb358_charger *chip, int reason,
								bool suspend)
{
	int rc = 0;
	int suspended;

	mutex_lock(&chip->path_suspend_lock);
	suspended = chip->usb_suspended;

	if (suspend == false)
		suspended &= ~reason;
	else
		suspended |= reason;

	if (!chip->usb_suspended && suspended) {
		rc = __smb358_path_suspend(chip, true);
		chip->usb_suspended = suspended;
		if (chip->usb_psy)
			power_supply_changed(chip->usb_psy);
	} else if (chip->usb_suspended && !suspended) {
		rc = __smb358_path_suspend(chip, false);
		chip->usb_suspended = suspended;
		if (chip->usb_psy)
			power_supply_changed(chip->usb_psy);
	}

	if (rc)
		dev_err(chip->dev, "Couldn't set/unset suspend rc = %d\n", rc);

	mutex_unlock(&chip->path_suspend_lock);

	return rc;
}

static int __smb358_charging_disable(struct smb358_charger *chip, bool disable)
{
	int rc;

	rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_CHG_ENABLE_BIT,
			disable ? 0 : CMD_A_CHG_ENABLE_BIT);
	if (rc < 0)
		pr_err("Couldn't set CHG_ENABLE_BIT diable = %d, rc = %d\n",
				disable, rc);
	return rc;
}

static int smb358_charging_disable(struct smb358_charger *chip,
						int reason, int disable)
{
	int rc = 0;
	int disabled;

	disabled = chip->charging_disabled_status;

	pr_debug("reason = %d requested_disable = %d disabled_status = %d\n",
						reason, disable, disabled);

	if (disable == true)
		disabled |= reason;
	else
		disabled &= ~reason;

	if (!!disabled == !!chip->charging_disabled_status)
		goto skip;

	rc = __smb358_charging_disable(chip, !!disabled);
	if (rc) {
		pr_err("Failed to disable charging rc = %d\n", rc);
		return rc;
	} else {
		if (chip->batt_psy)
			power_supply_changed(chip->batt_psy);
	}

skip:
	chip->charging_disabled_status = disabled;
	return rc;
}

#define MAX_INV_BATT_ID		7700
#define MIN_INV_BATT_ID		7300
#define OTG_FREQUENCY		0X80
#define SWITCHING_FREQUENCY 	BIT(7)

static int smb358_hw_init(struct smb358_charger *chip)
{
	int rc;
	u8 reg = 0, mask = 0;

	if (chip->chg_autonomous_mode) {
		dev_dbg(chip->dev, "Charger configured for autonomous mode\n");
		return 0;
	}

	rc = smb358_enable_volatile_writes(chip);
	if (rc) {
		dev_err(chip->dev, "Couldn't configure volatile writes rc=%d\n", rc);
		return rc;
	}

	reg = CHG_CTRL_BATT_MISSING_DET_THERM_IO;
	mask = CHG_CTRL_BATT_MISSING_DET_MASK;
	rc = smb358_masked_write(chip, CHG_CTRL_REG, mask, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set CHG_CTRL_REG rc=%d\n", rc);
		return rc;
	}

	reg = CHG_PIN_CTRL_USBCS_REG_BIT | CHG_PIN_CTRL_CHG_EN_LOW_REG_BIT |
		CHG_PIN_CTRL_APSD_IRQ_BIT | CHG_PIN_CTRL_CHG_ERR_IRQ_BIT;
	mask = CHG_PIN_CTRL_CHG_EN_MASK | CHG_PIN_CTRL_USBCS_REG_MASK |
		CHG_PIN_CTRL_APSD_IRQ_MASK | CHG_PIN_CTRL_CHG_ERR_IRQ_MASK;
	rc = smb358_masked_write(chip, CHG_PIN_EN_CTRL_REG, mask, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set CHG_PIN_EN_CTRL_REG rc=%d\n", rc);
		return rc;
	}

	rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_CHG_SUSP_EN_MASK,0);

	rc = smb358_masked_write(chip, VARIOUS_FUNC_REG,
		VARIOUS_FUNC_USB_SUSP_MASK, VARIOUS_FUNC_USB_SUSP_EN_REG_BIT);
	if (rc) {
		dev_err(chip->dev, "Couldn't set VARIOUS_FUNC_REG rc=%d\n", rc);
		return rc;
	}

	if (!chip->disable_apsd)
		reg = CHG_CTRL_APSD_EN_BIT;
	else
		reg = 0;

	rc = smb358_otg_set_limit_current(chip);
	if (rc < 0)
		pr_err("set otg current limit fail rc =%d\n",rc);

	rc = smb358_masked_write(chip, CHG_CTRL_REG, CHG_CTRL_APSD_EN_MASK, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set CHG_CTRL_REG rc=%d\n", rc);
		return rc;
	}

	reg = FAULT_INT_HOT_COLD_HARD_BIT | FAULT_INT_HOT_COLD_SOFT_BIT
		| FAULT_INT_INPUT_UV_BIT | FAULT_INT_AICL_COMPLETE_BIT
		| FAULT_INT_INPUT_OV_BIT;
	rc = smb358_write_reg(chip, FAULT_INT_REG, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set FAULT_INT_REG rc=%d\n", rc);
		return rc;
	}

	reg = STATUS_INT_CHG_TIMEOUT_BIT | STATUS_INT_OTG_DETECT_BIT |
		STATUS_INT_BATT_OV_BIT | STATUS_INT_CHGING_BIT |
		STATUS_INT_CHG_INHI_BIT | STATUS_INT_INOK_BIT |
		STATUS_INT_LOW_BATT_BIT | STATUS_INT_MISSING_BATT_BIT;
	rc = smb358_write_reg(chip, STATUS_INT_REG, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set STATUS_INT_REG rc=%d\n", rc);
		return rc;
	}

	rc = smb358_masked_write(chip, THERM_A_CTRL_REG,
		THERM_A_THERM_MONITOR_EN_MASK | SWITCHING_FREQUENCY, OTG_FREQUENCY);
	if (rc) {
		dev_err(chip->dev, "Couldn't set THERM_A_CTRL_REG rc=%d\n", rc);
		return rc;
	}

	rc = smb358_fastchg_current_set(chip, chip->fastchg_current_max_ma);
	if (rc) {
		dev_err(chip->dev, "Couldn't set fastchg current rc=%d\n", rc);
		return rc;
	}

	rc = smb358_float_voltage_set(chip, chip->vfloat_mv);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set float voltage rc = %d\n", rc);
		return rc;
	}

	rc = smb358_term_current_set(chip);
	if (rc)
		dev_err(chip->dev, "Couldn't set term current rc=%d\n", rc);

	rc = smb358_recharge_and_inhibit_set(chip);
	if (rc)
		dev_err(chip->dev, "Couldn't set recharge para rc=%d\n", rc);

	if (!chip->skip_usb_suspend_for_fake_battery) {
		if ((chip->connected_rid >= MIN_INV_BATT_ID) &&
				(chip->connected_rid <= MAX_INV_BATT_ID)) {
			rc = smb358_path_suspend(chip, FAKE_BATTERY, true);
			if (!rc)
				dev_info(chip->dev, "Suspended USB path reason FAKE_BATTERY\n");
		}
	}

	if (chip->charging_disabled) {
		rc = smb358_charging_disable(chip, USER, 1);
		if (rc)
			dev_err(chip->dev, "Couldn't '%s' charging rc = %d\n",
			chip->charging_disabled ? "disable" : "enable", rc);
	} else {
		rc = __smb358_charging_disable(chip, 0);
		if (rc)
			dev_err(chip->dev, "Couldn't enable charging\n");
	}

	rc = smb358_masked_write(chip, OTHER_CTRL_REG, CHG_LOW_BATT_THRESHOLD,
						SMB358_BATT_GOOD_THRE_2P5);
	if (rc)
		dev_err(chip->dev, "Couldn't write OTHER_CTRL_REG, rc = %d\n", rc);

	return rc;
}

static enum power_supply_property smb358_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_BATTERY_TYPE,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL,
};

static int smb358_get_prop_batt_status(struct smb358_charger *chip)
{
	int rc;
	u8 reg = 0;

	if (chip->batt_full)
		return POWER_SUPPLY_STATUS_FULL;

	rc = smb358_read_reg(chip, STATUS_C_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read STAT_C rc = %d\n", rc);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	if (reg & STATUS_C_CHG_HOLD_OFF_BIT)
		return POWER_SUPPLY_STATUS_NOT_CHARGING;

	/*
	 * Gate CHARGING on chip->chg_present (real VBUS, tracked by the APSD /
	 * input-UV IRQs). The STATUS_C charging bits can stay latched from a
	 * stale charge FSM state even with no charger attached; without this
	 * guard the "battery" node reports CHARGING with nothing plugged. This
	 * matches the Lenovo vendor SMB358 driver (the generic CAF/CM driver
	 * this is derived from omits it).
	 */
	if ((reg & STATUS_C_CHARGING_MASK) &&
			!(reg & STATUS_C_CHG_ERR_STATUS_BIT) &&
			chip->chg_present)
		return POWER_SUPPLY_STATUS_CHARGING;

	return POWER_SUPPLY_STATUS_DISCHARGING;
}

static int smb358_get_prop_batt_present(struct smb358_charger *chip)
{
	return !chip->battery_missing;
}

/*
 * Resolve the bms fuel-gauge power_supply, caching it on first success. It is
 * normally populated by smb358_external_power_changed(), but that only runs if
 * the power-supply core delivers a supply-change notification. On bamboo the
 * gauge (bq27500, registered as "bms") can probe after this charger and never
 * trigger that callback, leaving chip->bms_psy NULL so capacity/current_now/
 * charge_counter fall back to the SMB358_DEFAULT_BATT_CAPACITY / -EINVAL stubs
 * (dumpsys battery then shows 50%, current -22, charge_counter -22). Resolve it
 * lazily by name here as a robust fallback for any probe order.
 */
static struct power_supply *smb358_bms_psy(struct smb358_charger *chip)
{
	if (!chip->bms_psy && chip->bms_psy_name)
		chip->bms_psy = power_supply_get_by_name(chip->bms_psy_name);
	return chip->bms_psy;
}

/*
 * Read one property off the gauge. power_supply_get_property() leaves *val
 * untouched when the gauge does not implement the property, so a caller that
 * ignores the return code silently reports the zero-initialised value as if it
 * were a real reading -- that is how charge_counter ended up pinned at 0 on
 * bamboo, whose bq27xxx gauge has no CHARGE_COUNTER. Report success separately
 * so callers can fall back.
 */
static bool smb358_bms_read(struct smb358_charger *chip,
		enum power_supply_property psp, int *val)
{
	union power_supply_propval ret = {0, };

	if (!smb358_bms_psy(chip))
		return false;

	if (power_supply_get_property(chip->bms_psy, psp, &ret))
		return false;

	*val = ret.intval;
	return true;
}

static int smb358_get_prop_batt_current_now(struct smb358_charger *chip)
{
	int val;

	if (smb358_bms_read(chip, POWER_SUPPLY_PROP_CURRENT_NOW, &val))
		return val;

	return -22;
}

/*
 * charge_counter (uAh) lives on the fuel gauge, not on this charger's "battery"
 * (type=Battery) node. The health@2.0 BatteryMonitor only reads charge_counter
 * from the type=Battery node, so proxy it from the gauge here -- same pattern as
 * current_now/capacity above. Without this, dumpsys battery /
 * BATTERY_PROPERTY_CHARGE_COUNTER report 0 and batterystats falls back to
 * level%*capacity for drain accounting.
 *
 * The QTI vm-bms gauge answers CHARGE_COUNTER directly; the TI bq27xxx on bamboo
 * does not implement it and reports the same quantity (remaining charge in uAh,
 * the gauge's NAC register) as CHARGE_NOW, which is exactly what Android wants
 * here. Ask for the former and fall back to the latter.
 */
static int smb358_get_prop_batt_charge_counter(struct smb358_charger *chip)
{
	int val;

	if (smb358_bms_read(chip, POWER_SUPPLY_PROP_CHARGE_COUNTER, &val))
		return val;

	if (smb358_bms_read(chip, POWER_SUPPLY_PROP_CHARGE_NOW, &val))
		return val;

	return -22;
}

/*
 * charge_full (uAh) -- the gauge's learned/nominal full capacity. Same story as
 * charge_counter above: it lives on the fuel gauge, but healthd only probes the
 * type=Battery node for it (BatteryMonitor::init() walks every entry under
 * /sys/class/power_supply and skips anything whose "type" is not "Battery" --
 * our gauge reports "BMS"), so without this proxy HealthInfo.batteryFullCharge
 * stays 0 and the framework has no idea how big the pack is.
 *
 * Both gauges in this tree answer it: qpnp-vm-bms exports CHARGE_FULL from the
 * selected battery profile's FCC (scaled by temperature and, when
 * qcom,batt-aging-comp is set, by learned charge cycles); bq27xxx reports the
 * gauge's FCC register. Report 0 rather than an error when neither answers --
 * that is what "unknown" means to healthd, and a negative uAh would be worse
 * than useless to a consumer doing arithmetic on it.
 */
static int smb358_get_prop_batt_charge_full(struct smb358_charger *chip)
{
	int val;

	if (smb358_bms_read(chip, POWER_SUPPLY_PROP_CHARGE_FULL, &val))
		return val;

	return 0;
}

static int smb358_get_prop_batt_capacity(struct smb358_charger *chip)
{
	int val;

	if (chip->fake_battery_soc >= 0)
		return chip->fake_battery_soc;

	if (smb358_bms_read(chip, POWER_SUPPLY_PROP_CAPACITY, &val))
		return val;

	return SMB358_DEFAULT_BATT_CAPACITY;
}

static int smb358_get_prop_charge_type(struct smb358_charger *chip)
{
	int rc;
	u8 reg = 0;

	rc = smb358_read_reg(chip, STATUS_C_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read STAT_C rc = %d\n", rc);
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}

	reg &= STATUS_C_CHARGING_MASK;

	if (reg == STATUS_C_FAST_CHARGING)
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	else if (reg == STATUS_C_TAPER_CHARGING)
		return POWER_SUPPLY_CHARGE_TYPE_TAPER;
	else if (reg == STATUS_C_PRE_CHARGING)
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	else
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

static int smb358_get_prop_batt_health(struct smb358_charger *chip)
{
	if (chip->batt_hot)
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (chip->batt_cold)
		return POWER_SUPPLY_HEALTH_COLD;
	else if (chip->batt_warm)
		return POWER_SUPPLY_HEALTH_WARM;
	else if (chip->batt_cool)
		return POWER_SUPPLY_HEALTH_COOL;
		
	return POWER_SUPPLY_HEALTH_GOOD;
}

#define DEFAULT_TEMP 250

/*
 * The pack thermistor read through the PMIC's ADC. This is the charger's own
 * safety sensor -- it is what the hardware JEITA (adc_tm) trips on, and what
 * smb358_temp_work() suspends charging on. Distinct from the temperature we
 * report to Android below, and the stock driver keeps the same split.
 */
static int smb358_get_batt_therm_decidegc(struct smb358_charger *chip)
{
	struct qpnp_vadc_result results;
	int rc;

	if (!smb358_get_prop_batt_present(chip))
		return DEFAULT_TEMP;

	if (IS_ERR_OR_NULL(smb358_get_vadc(chip)))
		return DEFAULT_TEMP;

	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX1_BATT_THERM, &results);
	if (rc) {
		pr_debug("Unable to read batt temperature rc=%d\n", rc);
		return DEFAULT_TEMP;
	}

	return (int)results.physical;
}

/*
 * Temperature and voltage as reported to Android. A board that routes the pack
 * thermistor and VBAT to the PMIC's ADC says so with qcom,using-pmic-therm and
 * qcom,using-vbat-sns (x104 does both) and we hand those readings straight up.
 * bamboo declares neither: it has a real gas gauge (bq27500) sitting on the
 * pack, so the gauge is the better sensor and VBAT_SNS is not wired at all --
 * which is why voltage_now used to read 0. Defer to the gauge there, exactly as
 * the stock driver does: stock's battery/temp and battery/voltage_now are
 * identical to the gauge's (226 and 3911000), even though stock still trips its
 * charge-suspend logic on the PMIC thermistor above.
 */
static int smb358_get_prop_batt_temp(struct smb358_charger *chip)
{
	int val;

	if (!smb358_get_prop_batt_present(chip))
		return DEFAULT_TEMP;

	if (!chip->using_pmic_therm) {
		if (smb358_bms_read(chip, POWER_SUPPLY_PROP_TEMP, &val))
			return val;
		return DEFAULT_TEMP;
	}

	return smb358_get_batt_therm_decidegc(chip);
}

static int smb358_get_prop_battery_voltage_now(struct smb358_charger *chip)
{
	int rc = 0, val;
	struct qpnp_vadc_result results;

	if (!chip->pmic_vbat_sns) {
		if (smb358_bms_read(chip, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val))
			return val;
		return 0;
	}

	if (IS_ERR_OR_NULL(smb358_get_vadc(chip)))
		return 0;

	rc = qpnp_vadc_read(chip->vadc_dev, VBAT_SNS, &results);
	if (rc) {
		pr_err("Unable to read vbat rc=%d\n", rc);
		return 0;
	}
	return results.physical;
}

/*
 * Software battery-temperature charge suspend.
 *
 * The hardware JEITA in this driver rides on the PMIC's adc_tm threshold
 * notifications, which are only armed for boards that declare
 * qcom,using-pmic-therm. bamboo does not, so it had no temperature protection
 * at all above the SMB358's own THERM pin. Stock covered that gap the same way
 * -- a polling worker (smb358_temp_work) that suspends charging outside a
 * temperature window and resumes it once the pack has recovered -- so mirror it,
 * thresholds and hysteresis included, rather than inventing a policy.
 *
 * Note this is a hard cut, not the current/voltage derating of a classic soft
 * JEITA: that is what the board shipped with.
 */
#define TEMP_SETTLE_S			300
#define TEMP_POLL_NORMAL_MS		10000
#define TEMP_POLL_FAST_MS		1000
#define TEMP_POLL_NORMAL_LO_DECIDEGC	100
#define TEMP_POLL_NORMAL_HI_DECIDEGC	400
/*
 * Recovery is measured against the temperature we stopped at, not against the
 * trip point, so a pack sitting right on the threshold cannot chatter. Below
 * the band we are coming back from a cold stop and want to see it warm up; above
 * it we are coming back from a hot stop and want to see it cool down.
 */
#define TEMP_RECOVER_BAND_DECIDEGC	250
#define TEMP_RECOVER_WARMED_DECIDEGC	10
#define TEMP_RECOVER_COOLED_DECIDEGC	25

static void smb358_temp_suspend(struct smb358_charger *chip, bool hot, bool cold)
{
	int temp = smb358_get_batt_therm_decidegc(chip);
	int rc;

	rc = smb358_charging_disable(chip, BATT_TEMP, true);
	if (rc) {
		pr_err("Couldn't suspend charging on batt temp rc = %d\n", rc);
		return;
	}

	chip->batt_hot = hot;
	chip->batt_cold = cold;
	chip->temp_suspended = true;
	chip->temp_suspended_at = temp;

	pr_info("Charging suspended: batt temp %d outside [%d, %d] decidegC\n",
		temp, chip->temp_cold_decidegc, chip->temp_hot_decidegc);

	power_supply_changed(chip->batt_psy);
}

static void smb358_temp_resume(struct smb358_charger *chip)
{
	int rc;

	if (!chip->temp_suspended)
		return;

	rc = smb358_charging_disable(chip, BATT_TEMP, false);
	if (rc) {
		pr_err("Couldn't resume charging after batt temp rc = %d\n", rc);
		return;
	}

	chip->batt_hot = false;
	chip->batt_cold = false;
	chip->temp_suspended = false;

	pr_info("Charging resumed: batt temp back in range (%d decidegC)\n",
		smb358_get_batt_therm_decidegc(chip));

	power_supply_changed(chip->batt_psy);
}

static void smb358_temp_check(struct smb358_charger *chip)
{
	struct timespec ts;
	int temp, delta;

	/*
	 * Unplugging always releases the hold, ahead of the settle window: the
	 * charger is off anyway, and leaving BATT_TEMP latched would keep the
	 * next plug-in from charging. Clearing the timestamp lets a still-hot
	 * pack be re-suspended immediately on that next plug rather than after
	 * a stale settle window.
	 */
	if (!chip->chg_present) {
		smb358_temp_resume(chip);
		chip->temp_last_change = 0;
		return;
	}

	get_monotonic_boottime(&ts);
	if (chip->temp_last_change &&
	    ts.tv_sec - chip->temp_last_change < TEMP_SETTLE_S)
		return;

	temp = smb358_get_batt_therm_decidegc(chip);

	if (chip->temp_suspended) {
		delta = temp - chip->temp_suspended_at;

		if (temp < TEMP_RECOVER_BAND_DECIDEGC) {
			if (delta < TEMP_RECOVER_WARMED_DECIDEGC)
				return;
		} else {
			if (delta > -TEMP_RECOVER_COOLED_DECIDEGC)
				return;
		}

		smb358_temp_resume(chip);
		chip->temp_last_change = ts.tv_sec;
		return;
	}

	if (temp > chip->temp_hot_decidegc)
		smb358_temp_suspend(chip, true, false);
	else if (temp < chip->temp_cold_decidegc)
		smb358_temp_suspend(chip, false, true);
	else
		return;

	chip->temp_last_change = ts.tv_sec;
}

static void smb358_temp_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct smb358_charger *chip = container_of(dwork,
			struct smb358_charger, temp_work);
	int temp;

	smb358_temp_check(chip);

	/*
	 * Only poll while there is something to hold off. The cable paths kick
	 * the worker back awake, so a charger plugged in while we are idle is
	 * still picked up.
	 */
	if (!chip->chg_present)
		return;

	temp = smb358_get_batt_therm_decidegc(chip);
	schedule_delayed_work(&chip->temp_work,
			msecs_to_jiffies(
				(temp > TEMP_POLL_NORMAL_LO_DECIDEGC &&
				 temp < TEMP_POLL_NORMAL_HI_DECIDEGC) ?
				TEMP_POLL_NORMAL_MS : TEMP_POLL_FAST_MS));
}

/* Re-evaluate now; safe to call from the cable IRQ paths (process context). */
static void smb358_temp_kick(struct smb358_charger *chip)
{
	if (chip->temp_suspend_supported)
		mod_delayed_work(system_wq, &chip->temp_work, 0);
}

static int smb358_set_usb_chg_current(struct smb358_charger *chip,
		int current_ma)
{
	int i, rc = 0;
	u8 reg1 = 0, reg2 = 0, mask = 0;

	if (chip->chg_autonomous_mode) {
		return 0;
	}

	if (current_ma < USB3_MIN_CURRENT_MA && current_ma != 2)
		current_ma = USB2_MIN_CURRENT_MA;

	if (current_ma == USB2_MIN_CURRENT_MA) {
		reg1 &= ~USB3_ENABLE_BIT;
		reg2 &= ~CMD_B_CHG_USB_500_900_ENABLE_BIT;
	} else if (current_ma == USB2_MAX_CURRENT_MA) {
		reg1 &= ~USB3_ENABLE_BIT;
		reg2 |= CMD_B_CHG_USB_500_900_ENABLE_BIT;
	} else if (current_ma == USB3_MAX_CURRENT_MA) {
		reg1 |= USB3_ENABLE_BIT;
		reg2 |= CMD_B_CHG_USB_500_900_ENABLE_BIT;
	} else if (current_ma > USB2_MAX_CURRENT_MA) {
		reg2 |= CMD_B_CHG_HC_ENABLE_BIT;

		for (i = ARRAY_SIZE(chg_current) - 1; i >= 0; i--) {
			if (chg_current[i] <= current_ma)
				break;
		}
		if (i < 0) {
			dev_err(chip->dev, "Cannot find %dmA\n", current_ma);
			i = 0;
		}

		i = i << AC_CHG_CURRENT_SHIFT;
		rc = smb358_masked_write(chip, CHG_OTH_CURRENT_CTRL_REG,
						AC_CHG_CURRENT_MASK, i);
		if (rc)
			dev_err(chip->dev, "Couldn't set input mA rc=%d\n", rc);
	}

	mask = CMD_B_CHG_HC_ENABLE_BIT | CMD_B_CHG_USB_500_900_ENABLE_BIT;
	rc = smb358_masked_write(chip, CMD_B_REG, mask, reg2);

	mask = USB3_ENABLE_MASK;
	rc = smb358_masked_write(chip, SYSOK_AND_USB3_REG, mask, reg1);

	if (current_ma == 2 && chip->chg_present) {
		rc = smb358_path_suspend(chip, CURRENT, true);
	} else if (chip->chg_present) {
		rc = smb358_path_suspend(chip, CURRENT, false);
	}

	return rc;
}

static int smb358_system_temp_level_set(struct smb358_charger *chip,
		int lvl_sel)
{
	int rc = 0;
	int prev_therm_lvl;

	if (!chip->thermal_mitigation) {
		pr_err("Thermal mitigation not supported\n");
		return -EINVAL;
	}

	if (lvl_sel < 0) {
		pr_err("Unsupported level selected %d\n", lvl_sel);
		return -EINVAL;
	}

       if (lvl_sel >= chip->thermal_levels) {
		pr_err("Unsupported level selected %d forcing %d\n", lvl_sel,
				chip->thermal_levels - 1);
		lvl_sel = chip->thermal_levels - 1;
	}

	if (lvl_sel == chip->therm_lvl_sel)
		return 0;

	prev_therm_lvl = chip->therm_lvl_sel;
	chip->therm_lvl_sel = lvl_sel;

	/*
	 * Honour qcom,thermal-mitigation instead of the 450/2000 mA the code used
	 * to hardcode: both boards ask for <1500 700 600 0>, so every throttled
	 * level was being clamped to 450 mA. The terminal 0 mA entry means "stop
	 * charging" -- feeding it to smb358_fastchg_current_set() would just trip
	 * its SMB358_FAST_CHG_MIN_MA check and return -EINVAL, leaving the charger
	 * running at full current at the hottest level. That is what the unused
	 * THERMAL disable reason is for.
	 */
	if (chip->therm_lvl_sel == chip->thermal_levels - 1) {
		rc = smb358_charging_disable(chip, THERMAL, true);
		if (rc) {
			pr_err("Couldn't disable charging for thermal rc = %d\n", rc);
			goto restore_lvl;
		}
		return 0;
	}

	rc = smb358_fastchg_current_set(chip,
			chip->thermal_mitigation[chip->therm_lvl_sel]);
	if (rc) {
		pr_err("Couldn't set fastchg current rc = %d\n", rc);
		goto restore_lvl;
	}

	if (prev_therm_lvl == chip->thermal_levels - 1) {
		rc = smb358_charging_disable(chip, THERMAL, false);
		if (rc) {
			pr_err("Couldn't re-enable charging rc = %d\n", rc);
			goto restore_lvl;
		}
	}

	return 0;

restore_lvl:
	chip->therm_lvl_sel = prev_therm_lvl;
	return rc;
}

static int smb358_batt_property_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		return 1;
	default:
		break;
	}
	return 0;
}

static int bound_soc(int soc)
{
	soc = max(0, soc);
	soc = min(soc, 100);
	return soc;
}

static int smb358_battery_set_property(struct power_supply *psy,
					enum power_supply_property prop,
					const union power_supply_propval *val)
{
	int rc;
	struct smb358_charger *chip = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		if (!chip->bms_controlled_charging)
			return -EINVAL;
		switch (val->intval) {
		case POWER_SUPPLY_STATUS_FULL:
			rc = smb358_charging_disable(chip, SOC, true);
			if (rc == 0) chip->batt_full = true;
			break;
		case POWER_SUPPLY_STATUS_DISCHARGING:
			chip->batt_full = false;
			power_supply_changed(chip->batt_psy);
			break;
		case POWER_SUPPLY_STATUS_CHARGING:
			rc = smb358_charging_disable(chip, SOC, false);
			if (rc == 0) chip->batt_full = false;
			break;
		default:
			return -EINVAL;
		}
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		smb358_charging_disable(chip, USER, !val->intval);
		smb358_path_suspend(chip, USER, !val->intval);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		chip->fake_battery_soc = bound_soc(val->intval);
		power_supply_changed(chip->batt_psy);
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		smb358_system_temp_level_set(chip, val->intval);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int smb358_battery_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct smb358_charger *chip = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = smb358_get_prop_batt_status(chip);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = smb358_get_prop_batt_present(chip);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = smb358_get_prop_batt_capacity(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = !(chip->charging_disabled_status & USER);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = smb358_get_prop_charge_type(chip);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = smb358_get_prop_batt_health(chip);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "SMB358";
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = smb358_get_prop_batt_temp(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = smb358_get_prop_battery_voltage_now(chip);
		break;
	case POWER_SUPPLY_PROP_BATTERY_TYPE:
		val->strval = chip->battery_name;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = smb358_get_prop_batt_current_now(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		val->intval = smb358_get_prop_batt_charge_counter(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = smb358_get_prop_batt_charge_full(chip);
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		val->intval = chip->therm_lvl_sel;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static const unsigned int smb358_extcon_cable[] = {
	EXTCON_USB,
	/*
	 * msm_otg registers an ID (host) notifier on the same extcon device, so
	 * the device must declare EXTCON_USB_HOST or that registration fails and
	 * tears down the VBUS notifier too. We never assert it (no ID detection
	 * on this charger) -> ID stays floating -> peripheral mode, which is what
	 * charging + adb sideload need.
	 */
	EXTCON_USB_HOST,
	EXTCON_NONE,
};

/*
 * Bridge VBUS detection to msm_otg. In 4.9 the OTG state machine learns about
 * the cable solely through EXTCON_USB, so reflect chg_present onto the extcon
 * device that usb_otg references. Idempotent: only notify on real edge changes.
 */
static void smb358_notify_extcon(struct smb358_charger *chip)
{
	int state = !!chip->chg_present;

	if (!chip->extcon_dev)
		return;
	if (chip->vbus_notified == state)
		return;

	extcon_set_cable_state_(chip->extcon_dev, EXTCON_USB, state);
	chip->vbus_notified = state;
	dev_dbg(chip->dev, "extcon EXTCON_USB -> %d\n", state);
}

static int apsd_complete(struct smb358_charger *chip, u8 status)
{
	int rc;
	u8 reg = 0;
	enum power_supply_type type = POWER_SUPPLY_TYPE_UNKNOWN;

	if (chip->disable_apsd || status == 0)
		return 0;

	rc = smb358_read_reg(chip, STATUS_D_REG, &reg);
	if (rc)
		return rc;

	/* Determine port type */
	switch (reg & STATUS_D_CHARGING_PORT_MASK) {
	case STATUS_D_PORT_ACA_DOCK:
	case STATUS_D_PORT_ACA_C:
	case STATUS_D_PORT_ACA_B:
	case STATUS_D_PORT_ACA_A:
		type = POWER_SUPPLY_TYPE_USB_ACA;
		break;
	case STATUS_D_PORT_CDP:
		type = POWER_SUPPLY_TYPE_USB_CDP;
		break;
	case STATUS_D_PORT_DCP:
		type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	case STATUS_D_PORT_SDP:
		type = POWER_SUPPLY_TYPE_USB;
		break;
	case STATUS_D_PORT_OTHER:
		type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	default:
		type = POWER_SUPPLY_TYPE_USB;
		break;
	}

	chip->chg_present = !!status;

	if (chip->usb_psy) {
		chip->usb_supply_type = type;
		/*
		 * power_supply_sysfs reads desc->type directly instead of
		 * calling get_property(), so healthd only ever sees what is
		 * stored here. Without this the port always reads back as
		 * plain USB (SDP) and Android reports a wall charger as
		 * "USB powered" with a 500mA budget.
		 */
		smb358_usb_psy_desc.type = type;
		power_supply_changed(chip->usb_psy);
	}

	/*
	 * A dumb wall charger (DCP/CDP) never enumerates, so msm_otg won't
	 * report a current and the input would stay clamped to 100mA. In the
	 * stock 3.18 tree msm_otg owned the usb psy and opened the high-current
	 * path on DCP detection; here the SMB358 owns the psy and is the one
	 * that knows the APSD-detected type, so open it from this side. SDP is
	 * left to USB enumeration (handled via SDP_CURRENT_MAX).
	 *
	 * AICL only re-arms when the input path enters high-current mode, so
	 * the limit has to be programmed here, before that transition. Raising
	 * it later leaves AICL latched at whatever it settled on.
	 */
	if (type == POWER_SUPPLY_TYPE_USB_DCP ||
	    type == POWER_SUPPLY_TYPE_USB_CDP) {
		int input_ma = (type == POWER_SUPPLY_TYPE_USB_DCP) ?
				chip->dcp_input_current_ma :
				SMB358_DCP_INPUT_CURRENT_MA;

		chip->usb_current_max = input_ma * 1000;
		smb358_enable_volatile_writes(chip);
		smb358_set_usb_chg_current(chip, input_ma);
	}

	smb358_notify_extcon(chip);

	smb358_temp_kick(chip);

	return 0;
}

static int chg_uv(struct smb358_charger *chip, u8 status)
{
	int rc;

	if (chip->disable_apsd && status == 0) {
		chip->chg_present = true;

		if (chip->usb_psy)
			chip->usb_supply_type = POWER_SUPPLY_TYPE_USB;

		if (chip->bms_controlled_charging) {
			rc = smb358_charging_disable(chip, SOC, false);
		}
	}

	if (status != 0) {
		chip->chg_present = false;
		/* Cable out: drop the APSD-detected port type. */
		chip->usb_supply_type = POWER_SUPPLY_TYPE_USB;
		smb358_usb_psy_desc.type = POWER_SUPPLY_TYPE_USB;
	}

	rc = smb358_charging_disable(chip, SOC, false);
	if (chip->usb_psy)
		power_supply_changed(chip->usb_psy);

	smb358_notify_extcon(chip);

	/* Cable in: start watching the pack. Cable out: release any hold. */
	smb358_temp_kick(chip);

	return 0;
}

static int chg_ov(struct smb358_charger *chip, u8 status)
{
	u8 psy_health_sts;

	if (status)
		psy_health_sts = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	else
		psy_health_sts = POWER_SUPPLY_HEALTH_GOOD;

	if (chip->usb_psy) {
		chip->usb_health = psy_health_sts;
		power_supply_changed(chip->usb_psy);
	}

	return 0;
}

#define STATUS_FAST_CHARGING BIT(6)
static int fast_chg(struct smb358_charger *chip, u8 status)
{
	if (status & STATUS_FAST_CHARGING)
		chip->batt_full = false;
	return 0;
}

static int chg_term(struct smb358_charger *chip, u8 status)
{
	if (!chip->iterm_disabled)
		chip->batt_full = !!status;
	return 0;
}

static int taper_chg(struct smb358_charger *chip, u8 status)
{
	return 0;
}

static int chg_recharge(struct smb358_charger *chip, u8 status)
{
	chip->batt_full = !status;
	return 0;
}

static void smb358_chg_set_appropriate_battery_current(
				struct smb358_charger *chip)
{
	int rc;
	unsigned int current_max = chip->fastchg_current_max_ma;

	if (chip->batt_cool)
		current_max = min(current_max, chip->cool_bat_ma);
	if (chip->batt_warm)
		current_max = min(current_max, chip->warm_bat_ma);

	rc = smb358_fastchg_current_set(chip, current_max);
}

static void smb358_chg_set_appropriate_vddmax(
				struct smb358_charger *chip)
{
	int rc;
	unsigned int vddmax = chip->vfloat_mv;

	if (chip->batt_cool)
		vddmax = min(vddmax, chip->cool_bat_mv);
	if (chip->batt_warm)
		vddmax = min(vddmax, chip->warm_bat_mv);

	rc = smb358_float_voltage_set(chip, vddmax);
}

#define HYSTERESIS_DECIDEGC 20
static void smb_chg_adc_notification(enum qpnp_tm_state state, void *ctx)
{
	struct smb358_charger *chip = ctx;
	bool bat_hot = 0, bat_cold = 0, bat_present = 0, bat_warm = 0,
							bat_cool = 0;
	int temp;

	if (state >= ADC_TM_STATE_NUM)
		return;

	temp = smb358_get_prop_batt_temp(chip);

	if (state == ADC_TM_WARM_STATE) {
		if (temp >= chip->hot_bat_decidegc) {
			bat_hot = true;
			bat_warm = false;
			bat_cold = false;
			bat_cool = false;
			bat_present = true;

			chip->adc_param.low_temp =
				chip->hot_bat_decidegc - HYSTERESIS_DECIDEGC;
			chip->adc_param.state_request =
				ADC_TM_COOL_THR_ENABLE;
		} else if (temp >=
			chip->warm_bat_decidegc && chip->jeita_supported) {
			bat_hot = false;
			bat_warm = true;
			bat_cold = false;
			bat_cool = false;
			bat_present = true;

			chip->adc_param.low_temp =
				chip->warm_bat_decidegc - HYSTERESIS_DECIDEGC;
			chip->adc_param.high_temp =
				chip->hot_bat_decidegc;
		} else if (temp >=
			chip->cool_bat_decidegc && chip->jeita_supported) {
			bat_hot = false;
			bat_warm = false;
			bat_cold = false;
			bat_cool = false;
			bat_present = true;

			chip->adc_param.low_temp =
				chip->cool_bat_decidegc - HYSTERESIS_DECIDEGC;
			chip->adc_param.high_temp =
				chip->warm_bat_decidegc;
		} else if (temp >=
			chip->cold_bat_decidegc) {
			bat_hot = false;
			bat_warm = false;
			bat_cold = false;
			bat_cool = true;
			bat_present = true;

			chip->adc_param.low_temp =
				chip->cold_bat_decidegc - HYSTERESIS_DECIDEGC;
			if (chip->jeita_supported)
				chip->adc_param.high_temp =
						chip->cool_bat_decidegc;
			else
				chip->adc_param.high_temp =
						chip->hot_bat_decidegc;
			chip->adc_param.state_request =
					ADC_TM_HIGH_LOW_THR_ENABLE;
		} else if (temp >= chip->bat_present_decidegc) {
			bat_hot = false;
			bat_warm = false;
			bat_cold = true;
			bat_cool = false;
			bat_present = true;

			chip->adc_param.high_temp = chip->cold_bat_decidegc;
			chip->adc_param.low_temp = chip->bat_present_decidegc
							- HYSTERESIS_DECIDEGC;
			chip->adc_param.state_request =
					ADC_TM_HIGH_LOW_THR_ENABLE;
		}
	} else {
		if (temp <= chip->bat_present_decidegc) {
			bat_cold = true;
			bat_cool = false;
			bat_hot = false;
			bat_warm = false;
			bat_present = false;
			chip->adc_param.high_temp = chip->bat_present_decidegc
							+ HYSTERESIS_DECIDEGC;
			chip->adc_param.state_request =
				ADC_TM_WARM_THR_ENABLE;
		} else if (temp <= chip->cold_bat_decidegc) {
			bat_hot = false;
			bat_warm = false;
			bat_cold = true;
			bat_cool = false;
			bat_present = true;
			chip->adc_param.high_temp =
				chip->cold_bat_decidegc + HYSTERESIS_DECIDEGC;
			chip->adc_param.low_temp =
				chip->bat_present_decidegc;
			chip->adc_param.state_request =
				ADC_TM_HIGH_LOW_THR_ENABLE;
		} else if (temp <= chip->cool_bat_decidegc &&
					chip->jeita_supported) {
			bat_hot = false;
			bat_warm = false;
			bat_cold = false;
			bat_cool = true;
			bat_present = true;
			chip->adc_param.high_temp =
				chip->cool_bat_decidegc + HYSTERESIS_DECIDEGC;
			chip->adc_param.low_temp =
				chip->cold_bat_decidegc;
			chip->adc_param.state_request =
				ADC_TM_HIGH_LOW_THR_ENABLE;
		} else if (temp <= chip->warm_bat_decidegc &&
					chip->jeita_supported) {
			bat_hot = false;
			bat_warm = false;
			bat_cold = false;
			bat_cool = false;
			bat_present = true;
			chip->adc_param.high_temp =
				chip->warm_bat_decidegc + HYSTERESIS_DECIDEGC;
			chip->adc_param.low_temp =
				chip->cool_bat_decidegc;
			chip->adc_param.state_request =
				ADC_TM_HIGH_LOW_THR_ENABLE;
		} else if (temp <= chip->hot_bat_decidegc) {
			bat_hot = false;
			bat_warm = true;
			bat_cold = false;
			bat_cool = false;
			bat_present = true;
			if (chip->jeita_supported)
				chip->adc_param.low_temp =
					chip->warm_bat_decidegc;
			else
				chip->adc_param.low_temp =
					chip->cold_bat_decidegc;
			chip->adc_param.high_temp =
				chip->hot_bat_decidegc + HYSTERESIS_DECIDEGC;
			chip->adc_param.state_request =
					ADC_TM_HIGH_LOW_THR_ENABLE;
		}
	}

	if (bat_present)
		chip->battery_missing = false;
	else
		chip->battery_missing = true;

	if (bat_hot ^ chip->batt_hot || bat_cold ^ chip->batt_cold) {
		chip->batt_hot = bat_hot;
		chip->batt_cold = bat_cold;
		if (bat_hot || bat_cold || chip->battery_missing)
			smb358_charging_disable(chip, THERMAL, 1);
		else
			smb358_charging_disable(chip, THERMAL, 0);
	}

	if ((chip->batt_warm ^ bat_warm || chip->batt_cool ^ bat_cool)
						&& chip->jeita_supported) {
		chip->batt_warm = bat_warm;
		chip->batt_cool = bat_cool;
		smb358_chg_set_appropriate_battery_current(chip);
		smb358_chg_set_appropriate_vddmax(chip);
	}

	if (qpnp_adc_tm_channel_measure(chip->adc_tm_dev, &chip->adc_param))
		pr_err("request ADC error\n");
}

/*
 * Arm the hardware JEITA on the PMIC's adc_tm batt_therm channel.
 *
 * Returns 0 once armed, -EAGAIN while qpnp-adc-tm has not registered yet.
 * Idempotent, so the retry worker below can call it blindly.
 */
static int smb358_arm_adc_tm(struct smb358_charger *chip)
{
	struct qpnp_adc_tm_chip *adc_tm;
	int rc;

	if (chip->adc_tm_armed)
		return 0;

	adc_tm = smb358_get_adc_tm(chip);
	if (IS_ERR_OR_NULL(adc_tm))
		return -EAGAIN;

	if (!chip->jeita_supported) {
		chip->adc_param.low_temp = chip->cold_bat_decidegc;
		chip->adc_param.high_temp = chip->hot_bat_decidegc;
	} else {
		chip->adc_param.low_temp = chip->cool_bat_decidegc;
		chip->adc_param.high_temp = chip->warm_bat_decidegc;
	}
	chip->adc_param.timer_interval = ADC_MEAS2_INTERVAL_1S;
	chip->adc_param.state_request = ADC_TM_HIGH_LOW_THR_ENABLE;
	chip->adc_param.btm_ctx = chip;
	chip->adc_param.threshold_notification = smb_chg_adc_notification;
	chip->adc_param.channel = LR_MUX1_BATT_THERM;

	rc = qpnp_adc_tm_channel_measure(adc_tm, &chip->adc_param);
	if (rc) {
		dev_err(chip->dev, "adc_tm channel measure failed rc=%d\n", rc);
		return rc;
	}

	chip->adc_tm_armed = true;
	dev_info(chip->dev,
		 "hardware JEITA armed on adc_tm (low=%d high=%d decidegC)\n",
		 chip->adc_param.low_temp, chip->adc_param.high_temp);
	return 0;
}

/*
 * qpnp-adc-tm is not up yet when this driver probes, so keep trying for a
 * few seconds. Without this the JEITA window is never armed for the whole
 * boot and charging runs with no temperature limiting at all (measured on
 * a16 HW 20260724: "ADC_TM not available, thermal monitoring disabled").
 */
#define ADC_TM_ARM_RETRY_MS	2000
#define ADC_TM_ARM_RETRIES	15

static void smb358_adc_tm_arm_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct smb358_charger *chip = container_of(dwork,
			struct smb358_charger, adc_tm_arm_work);

	if (!smb358_arm_adc_tm(chip))
		return;

	if (++chip->adc_tm_arm_retries >= ADC_TM_ARM_RETRIES) {
		dev_warn(chip->dev,
			 "ADC_TM not available after %d attempts, thermal monitoring disabled\n",
			 chip->adc_tm_arm_retries);
		return;
	}

	schedule_delayed_work(&chip->adc_tm_arm_work,
			msecs_to_jiffies(ADC_TM_ARM_RETRY_MS));
}

static int hot_hard_handler(struct smb358_charger *chip, u8 status)
{
	chip->batt_hot = !!status;
	return 0;
}
static int cold_hard_handler(struct smb358_charger *chip, u8 status)
{
	chip->batt_cold = !!status;
	return 0;
}
static int hot_soft_handler(struct smb358_charger *chip, u8 status)
{
	chip->batt_warm = !!status;
	return 0;
}
static int cold_soft_handler(struct smb358_charger *chip, u8 status)
{
	chip->batt_cool = !!status;
	return 0;
}

static int battery_missing(struct smb358_charger *chip, u8 status)
{
	chip->battery_missing = !!status;
	return 0;
}

static struct irq_handler_info handlers[] = {
	[0] = {
		.stat_reg	= IRQ_A_REG,
		.irq_info	= {
			{ .name = "cold_soft", .smb_irq = cold_soft_handler, },
			{ .name = "hot_soft",  .smb_irq = hot_soft_handler,  },
			{ .name = "cold_hard", .smb_irq = cold_hard_handler, },
			{ .name = "hot_hard",  .smb_irq = hot_hard_handler,  },
		},
	},
	[1] = {
		.stat_reg	= IRQ_B_REG,
		.irq_info	= {
			{ .name = "chg_hot", },
			{ .name = "vbat_low", },
			{ .name = "battery_missing", .smb_irq = battery_missing },
			{ .name = "battery_ov", },
		},
	},
	[2] = {
		.stat_reg	= IRQ_C_REG,
		.irq_info	= {
			{ .name = "chg_term", .smb_irq = chg_term, },
			{ .name = "taper",    .smb_irq = taper_chg, },
			{ .name = "recharge", .smb_irq = chg_recharge, },
			{ .name = "fast_chg", .smb_irq = fast_chg, },
		},
	},
	[3] = {
		.stat_reg	= IRQ_D_REG,
		.irq_info	= {
			{ .name = "prechg_timeout", },
			{ .name = "safety_timeout", },
			{ .name = "aicl_complete", },
			{ .name = "src_detect", .smb_irq = apsd_complete, },
		},
	},
	[4] = {
		.stat_reg	= IRQ_E_REG,
		.irq_info	= {
			{ .name = "usbin_uv", .smb_irq = chg_uv, },
			{ .name = "usbin_ov", .smb_irq = chg_ov, },
			{ .name = "unknown", },
			{ .name = "unknown", },
		},
	},
	[5] = {
		.stat_reg	= IRQ_F_REG,
		.irq_info	= {
			{ .name = "power_ok", },
			{ .name = "otg_det", },
			{ .name = "otg_batt_uv", },
			{ .name = "otg_oc", },
		},
	},
};

#define IRQ_LATCHED_MASK	0x02
#define IRQ_STATUS_MASK		0x01
#define BITS_PER_IRQ		2
static irqreturn_t smb358_chg_stat_handler(int irq, void *dev_id)
{
	struct smb358_charger *chip = dev_id;
	int i, j;
	u8 triggered, changed, rt_stat, prev_rt_stat;
	int rc;
	int handler_count = 0;

	mutex_lock(&chip->irq_complete);

	chip->irq_waiting = true;
	if (!chip->resume_completed) {
		disable_irq_nosync(irq);
		mutex_unlock(&chip->irq_complete);
		return IRQ_HANDLED;
	}
	chip->irq_waiting = false;

	for (i = 0; i < ARRAY_SIZE(handlers); i++) {
		rc = smb358_read_reg(chip, handlers[i].stat_reg,
						&handlers[i].val);
		if (rc < 0)
			continue;

		for (j = 0; j < ARRAY_SIZE(handlers[i].irq_info); j++) {
			triggered = handlers[i].val
			       & (IRQ_LATCHED_MASK << (j * BITS_PER_IRQ));
			rt_stat = handlers[i].val
				& (IRQ_STATUS_MASK << (j * BITS_PER_IRQ));
			prev_rt_stat = handlers[i].prev_val
				& (IRQ_STATUS_MASK << (j * BITS_PER_IRQ));
			changed = prev_rt_stat ^ rt_stat;

			if (triggered || changed)
				rt_stat ? handlers[i].irq_info[j].high++ :
						handlers[i].irq_info[j].low++;

			if ((triggered || changed)
				&& handlers[i].irq_info[j].smb_irq != NULL) {
				handler_count++;
				handlers[i].irq_info[j].smb_irq(chip, rt_stat);
			}
		}
		handlers[i].prev_val = handlers[i].val;
	}

	if (handler_count && chip->batt_psy) {
		power_supply_changed(chip->batt_psy);
	}

	mutex_unlock(&chip->irq_complete);

	return IRQ_HANDLED;
}

static irqreturn_t smb358_chg_valid_handler(int irq, void *dev_id)
{
	struct smb358_charger *chip = dev_id;
	int present;

	present = gpio_get_value_cansleep(chip->chg_valid_gpio);
	if (present < 0) {
		return IRQ_HANDLED;
	}
	present ^= chip->chg_valid_act_low;

	if (present != chip->chg_present) {
		chip->chg_present = present;
		if (chip->usb_psy)
			power_supply_changed(chip->usb_psy);
	}

	return IRQ_HANDLED;
}

static void smb358_external_power_changed(struct power_supply *psy)
{
	struct smb358_charger *chip = power_supply_get_drvdata(psy);
	int current_limit = 0;

	if (chip->bms_psy_name)
		chip->bms_psy = power_supply_get_by_name(chip->bms_psy_name);

	current_limit = chip->usb_current_max / 1000;

	smb358_enable_volatile_writes(chip);
	smb358_set_usb_chg_current(chip, current_limit);
}

static int smb_parse_dt(struct smb358_charger *chip)
{
	int rc;
	enum of_gpio_flags gpio_flags;
	struct device_node *node = chip->dev->of_node;
	int batt_present_degree_negative;

	if (!node) {
		return -EINVAL;
	}

	chip->charging_disabled = of_property_read_bool(node, "qcom,charger-disabled");
	chip->inhibit_disabled = of_property_read_bool(node, "qcom,chg-inhibit-disabled");
	chip->chg_autonomous_mode = of_property_read_bool(node, "qcom,chg-autonomous-mode");
	chip->disable_apsd = of_property_read_bool(node, "qcom,disable-apsd");
	chip->using_pmic_therm = of_property_read_bool(node, "qcom,using-pmic-therm");
	chip->pmic_vbat_sns = of_property_read_bool(node, "qcom,using-vbat-sns");
	chip->bms_controlled_charging = of_property_read_bool(node, "qcom,bms-controlled-charging");

	rc = of_property_read_string(node, "qcom,bms-psy-name", &chip->bms_psy_name);
	if (rc)
		chip->bms_psy_name = NULL;

	chip->chg_valid_gpio = of_get_named_gpio_flags(node,
				"qcom,chg-valid-gpio", 0, &gpio_flags);
	if (gpio_is_valid(chip->chg_valid_gpio))
		chip->chg_valid_act_low = gpio_flags & OF_GPIO_ACTIVE_LOW;

	rc = of_property_read_u32(node, "qcom,fastchg-current-max-ma",
						&chip->fastchg_current_max_ma);
	if (rc)
		chip->fastchg_current_max_ma = SMB358_FAST_CHG_MAX_MA;

	rc = of_property_read_u32(node, "qcom,dcp-input-current-ma",
						&chip->dcp_input_current_ma);
	if (rc)
		chip->dcp_input_current_ma = SMB358_DCP_INPUT_CURRENT_MA;
	else
		/* Keep it inside chg_current[]; set_usb_chg_current() rounds down. */
		chip->dcp_input_current_ma = clamp(chip->dcp_input_current_ma,
					chg_current[0],
					chg_current[ARRAY_SIZE(chg_current) - 1]);

	chip->iterm_disabled = of_property_read_bool(node, "qcom,iterm-disabled");

	rc = of_property_read_u32(node, "qcom,iterm-ma", &chip->iterm_ma);
	if (rc < 0) chip->iterm_ma = -EINVAL;

	rc = of_property_read_u32(node, "qcom,float-voltage-mv", &chip->vfloat_mv);
	if (rc < 0) {
		chip->vfloat_mv = -EINVAL;
		return -EINVAL;
	}

	rc = of_property_read_u32(node, "qcom,recharge-mv", &chip->recharge_mv);
	if (rc < 0) chip->recharge_mv = -EINVAL;

	chip->recharge_disabled = of_property_read_bool(node, "qcom,recharge-disabled");

	rc = of_property_read_u32(node, "qcom,cold-bat-decidegc", &chip->cold_bat_decidegc);
	if (rc < 0) chip->cold_bat_decidegc = -EINVAL;

	rc = of_property_read_u32(node, "qcom,hot-bat-decidegc", &chip->hot_bat_decidegc);
	if (rc < 0) chip->hot_bat_decidegc = -EINVAL;

	rc = of_property_read_u32(node, "qcom,warm-bat-decidegc", &chip->warm_bat_decidegc);
	rc |= of_property_read_u32(node, "qcom,cool-bat-decidegc", &chip->cool_bat_decidegc);

	if (!rc) {
		rc = of_property_read_u32(node, "qcom,cool-bat-mv", &chip->cool_bat_mv);
		rc |= of_property_read_u32(node, "qcom,warm-bat-mv", &chip->warm_bat_mv);
		rc |= of_property_read_u32(node, "qcom,cool-bat-ma", &chip->cool_bat_ma);
		rc |= of_property_read_u32(node, "qcom,warm-bat-ma", &chip->warm_bat_ma);
		chip->jeita_supported = (rc == 0);
	}

	/*
	 * Opt-in software temperature cutoff, for boards with no adc_tm JEITA (see
	 * smb358_temp_work). Both thresholds or neither -- a one-sided window would
	 * be a config mistake, not a policy.
	 */
	rc = of_property_read_u32(node, "qcom,batt-temp-hot-suspend-decidegc",
			&chip->temp_hot_decidegc);
	rc |= of_property_read_u32(node, "qcom,batt-temp-cold-suspend-decidegc",
			&chip->temp_cold_decidegc);
	chip->temp_suspend_supported = (rc == 0);

	if (chip->temp_suspend_supported && chip->using_pmic_therm)
		pr_warn("Software batt-temp cutoff on a board that also arms adc_tm JEITA\n");

	rc = of_property_read_u32(node, "qcom,bat-present-decidegc", &batt_present_degree_negative);
	if (rc < 0)
		chip->bat_present_decidegc = -EINVAL;
	else
		chip->bat_present_decidegc = -batt_present_degree_negative;

	if (of_get_property(node, "qcom,vcc-i2c-supply", NULL)) {
		chip->vcc_i2c = devm_regulator_get(chip->dev, "vcc-i2c");
		if (IS_ERR(chip->vcc_i2c)) {
			return PTR_ERR(chip->vcc_i2c);
		}
	}

	if (of_find_property(node, "qcom,thermal-mitigation", &chip->thermal_levels)) {
		chip->thermal_mitigation = devm_kzalloc(chip->dev, chip->thermal_levels, GFP_KERNEL);
		if (chip->thermal_mitigation == NULL) return -ENOMEM;
		
		chip->thermal_levels /= sizeof(int);
		rc = of_property_read_u32_array(node, "qcom,thermal-mitigation",
				chip->thermal_mitigation, chip->thermal_levels);
		if (rc) return rc;
	}

	return 0;
}

static int determine_initial_state(struct smb358_charger *chip)
{
	int rc;
	u8 reg = 0;

	rc = smb358_read_reg(chip, IRQ_B_REG, &reg);
	if (rc) return rc;

	rc = smb358_read_reg(chip, IRQ_C_REG, &reg);
	if (rc) return rc;
	chip->batt_full = (reg & IRQ_C_TERM_BIT) ? true : false;

	rc = smb358_read_reg(chip, IRQ_A_REG, &reg);
	if (rc < 0) return rc;

	if (reg & IRQ_A_HOT_HARD_BIT) chip->batt_hot = true;
	if (reg & IRQ_A_COLD_HARD_BIT) chip->batt_cold = true;
	if (reg & IRQ_A_HOT_SOFT_BIT) chip->batt_warm = true;
	if (reg & IRQ_A_COLD_SOFT_BIT) chip->batt_cool = true;

	rc = smb358_read_reg(chip, IRQ_E_REG, &reg);
	if (rc) return rc;

	if (reg & IRQ_E_INPUT_UV_BIT) {
		chg_uv(chip, 1);
	} else {
		chg_uv(chip, 0);
		apsd_complete(chip, 1);
	}

	return 0;
}

static int64_t read_battery_id(struct smb358_charger *chip)
{
	int rc;
	struct qpnp_vadc_result result;

	if (IS_ERR_OR_NULL(smb358_get_vadc(chip)))
		return 0;

	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX2_BAT_ID, &result);
	if (rc) return rc;
	return result.physical;
}

static const struct power_supply_desc smb358_batt_psy_desc = {
	.name			= "battery",
	.type			= POWER_SUPPLY_TYPE_BATTERY,
	.properties		= smb358_battery_properties,
	.num_properties		= ARRAY_SIZE(smb358_battery_properties),
	.get_property		= smb358_battery_get_property,
	.set_property		= smb358_battery_set_property,
	.property_is_writeable	= smb358_batt_property_is_writeable,
	.external_power_changed = smb358_external_power_changed,
};

#define SMB_I2C_VTG_MIN_UV 1800000
#define SMB_I2C_VTG_MAX_UV 1800000

static int smb358_charger_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int rc, irq;
	struct smb358_charger *chip;
	struct power_supply_config batt_cfg = {};
	struct power_supply_config usb_cfg = {};
	u8 reg = 0;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	chip->dev = &client->dev;
	chip->fake_battery_soc = -EINVAL;
	chip->otg_enable = 0;
	chip->battery_name = "Unknown";
	chip->usb_supply_type = POWER_SUPPLY_TYPE_USB;
	chip->usb_health = POWER_SUPPLY_HEALTH_GOOD;

	chip->regmap = devm_regmap_init_i2c(client, &smb358_regmap_config);
	if (IS_ERR(chip->regmap))
		return PTR_ERR(chip->regmap);

	/* Parse DT first (no external dependencies) */
	rc = smb_parse_dt(chip);
	if (rc)
		return rc;

	if (chip->vcc_i2c) {
		if (regulator_count_voltages(chip->vcc_i2c) > 0) {
			rc = regulator_set_voltage(chip->vcc_i2c,
						   SMB_I2C_VTG_MIN_UV,
						   SMB_I2C_VTG_MAX_UV);
			if (rc)
				return rc;
		}
		rc = regulator_enable(chip->vcc_i2c);
		if (rc)
			goto err_set_vtg_i2c;
	}

	mutex_init(&chip->irq_complete);
	mutex_init(&chip->path_suspend_lock);

	/* Verify I2C communication works */
	rc = smb358_read_reg(chip, CHG_OTH_CURRENT_CTRL_REG, &reg);
	if (rc)
		goto err_set_vtg_i2c;

	i2c_set_clientdata(client, chip);

	/*
	 * Register USB PSY and OTG regulator EARLY, before any dependency
	 * that can defer (VADC, ADC_TM). msm_otg and ehci-msm need these
	 * to be present at their own probe time. If we defer here, they
	 * probe first, fail to find vbus_otg, and never retry.
	 */
	usb_cfg.drv_data = chip;
	chip->usb_psy = devm_power_supply_register(chip->dev,
						    &smb358_usb_psy_desc,
						    &usb_cfg);
	if (IS_ERR(chip->usb_psy)) {
		rc = PTR_ERR(chip->usb_psy);
		dev_err(chip->dev, "Unable to register usb PSY rc=%d\n", rc);
		chip->usb_psy = NULL;
		goto err_set_vtg_i2c;
	}

	/*
	 * Register the extcon device that bridges VBUS detection to msm_otg.
	 * Done before determine_initial_state() (which may call apsd_complete)
	 * so a cable already present at boot is reported. msm_otg references
	 * this device via "extcon = <&smb358_otg_supply>".
	 */
	chip->extcon_dev = devm_extcon_dev_allocate(chip->dev,
						    smb358_extcon_cable);
	if (IS_ERR(chip->extcon_dev)) {
		rc = PTR_ERR(chip->extcon_dev);
		dev_err(chip->dev, "Unable to allocate extcon rc=%d\n", rc);
		chip->extcon_dev = NULL;
		goto err_set_vtg_i2c;
	}
	rc = devm_extcon_dev_register(chip->dev, chip->extcon_dev);
	if (rc) {
		dev_err(chip->dev, "Unable to register extcon rc=%d\n", rc);
		chip->extcon_dev = NULL;
		goto err_set_vtg_i2c;
	}

	batt_cfg.drv_data = chip;
	batt_cfg.supplied_to = pm_batt_supplied_to;
	batt_cfg.num_supplicants = ARRAY_SIZE(pm_batt_supplied_to);

	chip->resume_completed = true;

	chip->batt_psy = power_supply_register(chip->dev,
					       &smb358_batt_psy_desc,
					       &batt_cfg);
	if (IS_ERR(chip->batt_psy)) {
		rc = PTR_ERR(chip->batt_psy);
		goto err_set_vtg_i2c;
	}

	rc = smb358_regulator_init(chip);
	if (rc)
		goto fail_regulator_register;

	/*
	 * VADC and ADC_TM are acquired lazily on first use (smb358_get_vadc/
	 * smb358_get_adc_tm). This avoids deferring probe before the USB PSY
	 * and OTG regulator are registered, which would prevent ehci-msm
	 * from finding vbus_otg.
	 */

	/*
	 * Must be initialised before determine_initial_state(), which reaches the
	 * cable paths (apsd_complete) and can therefore kick this worker.
	 */
	if (chip->temp_suspend_supported)
		INIT_DELAYED_WORK(&chip->temp_work, smb358_temp_work);

	rc = smb358_hw_init(chip);
	if (rc)
		goto fail_smb358_hw_init;

	rc = determine_initial_state(chip);
	if (rc)
		goto fail_smb358_hw_init;

	/* Booting on a charger with a pack already out of range must be caught. */
	smb358_temp_kick(chip);

	if (gpio_is_valid(chip->chg_valid_gpio)) {
		rc = devm_gpio_request(chip->dev, chip->chg_valid_gpio, "smb358_chg_valid");
		if (rc) goto fail_chg_valid_irq;
		
		irq = gpio_to_irq(chip->chg_valid_gpio);
		if (irq < 0) goto fail_chg_valid_irq;
		
		rc = devm_request_threaded_irq(&client->dev, irq,
				NULL, smb358_chg_valid_handler,
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				"smb358_chg_valid_irq", chip);
		if (rc) goto fail_chg_valid_irq;
		
		smb358_chg_valid_handler(irq, chip);
		enable_irq_wake(irq);
	}

	chip->irq_gpio = of_get_named_gpio_flags(chip->dev->of_node, "qcom,irq-gpio", 0, NULL);

	if (gpio_is_valid(chip->irq_gpio)) {
		rc = devm_gpio_request(chip->dev, chip->irq_gpio, "smb358_irq");
		if (rc) goto fail_smb358_hw_init;
		
		rc = gpio_direction_input(chip->irq_gpio);
		if (rc) goto fail_irq_gpio;

		irq = gpio_to_irq(chip->irq_gpio);
		if (irq < 0) goto fail_irq_gpio;
		
		rc = devm_request_threaded_irq(&client->dev, irq, NULL,
				smb358_chg_stat_handler,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"smb358_chg_stat_irq", chip);
		if (rc) goto fail_irq_gpio;
		
		enable_irq_wake(irq);
	}

	chip->otg_proc = proc_create_data(OTG_PROC_FILE, 0666, NULL, &smb358_otg_proc_ops, chip);
	chip->batt_proc = proc_create_data("app_info", 0666, NULL, &battery_type_fops, chip);
	
	if (!chip->otg_proc || !chip->batt_proc) {
		rc = -ENOMEM;
		goto fail_irq_gpio;
	}

	if (chip->using_pmic_therm) {
		INIT_DELAYED_WORK(&chip->adc_tm_arm_work, smb358_adc_tm_arm_work);

		/*
		 * qpnp-adc-tm has not registered this early, so the first
		 * attempt is expected to fail; the worker keeps trying.
		 */
		if (smb358_arm_adc_tm(chip))
			schedule_delayed_work(&chip->adc_tm_arm_work,
					msecs_to_jiffies(ADC_TM_ARM_RETRY_MS));
	}

	if ((read_battery_id(chip) > 500000)) {
		chip->battery_id_info = 1;
		chip->battery_missing = 0;
		chip->battery_name = "battery_name:FMT\n";
	} else {
		chip->battery_id_info = 2;
		chip->battery_missing = 0;
		chip->battery_name = "battery_name:SWD\n";
	}

	dev_info(chip->dev, "SMB358 probe OK, USB PSY and OTG regulator '%s' registered\n",
		 chip->otg_vreg.rdesc.name ? chip->otg_vreg.rdesc.name : "none");

	return 0;

fail_chg_valid_irq:
fail_irq_gpio:
fail_smb358_hw_init:
fail_regulator_register:
	if (chip->otg_proc) remove_proc_entry(OTG_PROC_FILE, NULL);
	if (chip->batt_proc) remove_proc_entry("app_info", NULL);
	if (chip->batt_psy && !IS_ERR(chip->batt_psy)) power_supply_unregister(chip->batt_psy);
err_set_vtg_i2c:
	if (chip->vcc_i2c && regulator_count_voltages(chip->vcc_i2c) > 0)
		regulator_set_voltage(chip->vcc_i2c, 0, SMB_I2C_VTG_MAX_UV);
	return rc;
}

static int smb358_charger_remove(struct i2c_client *client)
{
	struct smb358_charger *chip = i2c_get_clientdata(client);

	if (chip->temp_suspend_supported)
		cancel_delayed_work_sync(&chip->temp_work);

	if (chip->using_pmic_therm)
		cancel_delayed_work_sync(&chip->adc_tm_arm_work);

	if (chip->otg_proc) remove_proc_entry(OTG_PROC_FILE, NULL);
	if (chip->batt_proc) remove_proc_entry("app_info", NULL);

	power_supply_unregister(chip->batt_psy);
	
	if (chip->vcc_i2c)
		regulator_disable(chip->vcc_i2c);

	mutex_destroy(&chip->irq_complete);
	return 0;
}

static int smb358_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct smb358_charger *chip = i2c_get_clientdata(client);
	int rc;
	int i;

	for (i = 0; i < 2; i++) {
		rc = smb358_read_reg(chip, FAULT_INT_REG + i, &chip->irq_cfg_mask[i]);
		if (rc)
			dev_err(chip->dev,
				"Couldn't save irq cfg regs rc = %d\n", rc);
	}

	/* enable wake up IRQs */
	rc = smb358_write_reg(chip, FAULT_INT_REG,
			FAULT_INT_HOT_COLD_HARD_BIT | FAULT_INT_INPUT_UV_BIT);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set fault_irq_cfg rc = %d\n", rc);

	rc = smb358_write_reg(chip, STATUS_INT_REG,
			STATUS_INT_LOW_BATT_BIT | STATUS_INT_MISSING_BATT_BIT |
			STATUS_INT_CHGING_BIT | STATUS_INT_INOK_BIT |
			STATUS_INT_OTG_DETECT_BIT | STATUS_INT_CHG_INHI_BIT);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set status_irq_cfg rc = %d\n", rc);

	mutex_lock(&chip->irq_complete);
	if (chip->vcc_i2c) {
		rc = regulator_disable(chip->vcc_i2c);
		if (rc) {
			dev_err(chip->dev,
				"Regulator vcc_i2c disable failed rc=%d\n", rc);
			mutex_unlock(&chip->irq_complete);
			return rc;
		}
	}

	chip->resume_completed = false;
	mutex_unlock(&chip->irq_complete);
	return 0;
}

static int smb358_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct smb358_charger *chip = i2c_get_clientdata(client);

	if (chip->irq_waiting) {
		return -EBUSY;
	}
	return 0;
}

static int smb358_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct smb358_charger *chip = i2c_get_clientdata(client);
	int rc;
	int i;

	if (chip->vcc_i2c) {
		rc = regulator_enable(chip->vcc_i2c);
		if (rc) {
			dev_err(chip->dev,
				"Regulator vcc_i2c enable failed rc=%d\n", rc);
			return rc;
		}
	}

	/* Restore IRQ config */
	for (i = 0; i < 2; i++) {
		rc = smb358_write_reg(chip, FAULT_INT_REG + i, chip->irq_cfg_mask[i]);
		if (rc)
			dev_err(chip->dev,
				"Couldn't restore irq cfg regs rc=%d\n", rc);
	}

	mutex_lock(&chip->irq_complete);
	chip->resume_completed = true;
	mutex_unlock(&chip->irq_complete);

	if (chip->irq_waiting) {
		smb358_chg_stat_handler(client->irq, chip);
		enable_irq(client->irq);
	}
	return 0;
}

static const struct dev_pm_ops smb358_pm_ops = {
	.suspend	= smb358_suspend,
	.suspend_noirq	= smb358_suspend_noirq,
	.resume		= smb358_resume,
};

static const struct of_device_id smb358_match_table[] = {
	{ .compatible = "qcom,smb358-charger",},
	{ },
};
MODULE_DEVICE_TABLE(of, smb358_match_table);

static const struct i2c_device_id smb358_charger_id[] = {
	{"smb358-charger", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, smb358_charger_id);

static struct i2c_driver smb358_charger_driver = {
	.driver		= {
		.name		= "smb358-charger",
		.owner		= THIS_MODULE,
		.of_match_table = smb358_match_table,
		.pm		= &smb358_pm_ops,
	},
	.probe		= smb358_charger_probe,
	.remove		= smb358_charger_remove,
	.id_table	= smb358_charger_id,
};

module_i2c_driver(smb358_charger_driver);

MODULE_DESCRIPTION("SMB358 Charger (Linux 4.9 Port)");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:smb358-charger");
