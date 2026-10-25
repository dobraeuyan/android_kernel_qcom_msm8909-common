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
#ifndef __TS3A225E_H__
#define __TS3A225E_H__

#include <linux/types.h>
#include <linux/errno.h>

/* Accessory classes reported by ts3a225e_detect_accessory(). */
enum ts3a225e_accessory {
	TS3A225E_ACC_HEADPHONE = 1,	/* 3-pole: L/R/GND, no mic */
	TS3A225E_ACC_HEADSET   = 2,	/* 4-pole: L/R/GND/MIC */
};

#if IS_ENABLED(CONFIG_TS3A225E)

/*
 * True only once an i2c client has actually bound to a "hs_switch,ts3a225e"
 * device-tree node. Boards without the chip (x104f/x103f/a16) never populate
 * such a node, so every entry point below stays a strict no-op there.
 */
bool ts3a225e_is_present(void);

int ts3a225e_read_byte(u8 reg, u8 *val);
int ts3a225e_write_byte(u8 reg, u8 val);

/*
 * Run one detection cycle and configure the MIC/GND switches accordingly.
 *
 * Returns an enum ts3a225e_accessory value on success, or a negative errno
 * (-ENODEV when the chip is absent, -EIO/-EINVAL when the chip failed to
 * classify the plug). On a negative return the caller MUST fall back to its
 * own detection logic.
 *
 * *special_headset, when non-NULL, is set to true for a 4-pole plug whose
 * mic-side impedance lands in the ranges the stock driver treats as a
 * "special" headset needing the extra micbias ramp.
 */
int ts3a225e_detect_accessory(bool *special_headset);

#else /* !CONFIG_TS3A225E */

static inline bool ts3a225e_is_present(void)
{
	return false;
}

static inline int ts3a225e_read_byte(u8 reg, u8 *val)
{
	return -ENODEV;
}

static inline int ts3a225e_write_byte(u8 reg, u8 val)
{
	return -ENODEV;
}

static inline int ts3a225e_detect_accessory(bool *special_headset)
{
	return -ENODEV;
}

#endif /* CONFIG_TS3A225E */

#endif /* __TS3A225E_H__ */
