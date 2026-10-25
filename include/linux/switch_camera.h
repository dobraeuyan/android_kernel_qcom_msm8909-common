/*
 * Rotating camera barrel position (Lenovo Yoga Tab 3 / bamboo).
 *
 * Copyright (C) 2026 The LineageOS Project
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */
#ifndef __LINUX_SWITCH_CAMERA_H
#define __LINUX_SWITCH_CAMERA_H

enum camera_barrel_position {
	CAMERA_BARREL_UNKNOWN = -1,
	CAMERA_BARREL_BACK = 0,
	CAMERA_BARREL_FRONT = 1,
};

void msm_sensor_barrel_changed(void);

#if IS_ENABLED(CONFIG_INPUT_SWITCH_CAMERA)
int camera_barrel_position(void);
#else
static inline int camera_barrel_position(void)
{
	return CAMERA_BARREL_UNKNOWN;
}
#endif

#endif /* __LINUX_SWITCH_CAMERA_H */
