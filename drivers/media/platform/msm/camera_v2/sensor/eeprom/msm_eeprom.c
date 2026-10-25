/* Copyright (c) 2011-2019, The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/crc32.h>
#include "msm_sd.h"
#include "msm_cci.h"
#include "msm_eeprom.h"

#undef CDBG
#define CDBG(fmt, args...) pr_debug(fmt, ##args)

DEFINE_MSM_MUTEX(msm_eeprom_mutex);
#ifdef CONFIG_COMPAT
static struct v4l2_file_operations msm_eeprom_v4l2_subdev_fops;
#endif

/*
 * Lenovo x104 camera OTP support, ported from the stock 3.18 kernel
 * ("stoneadd"/"hmz" vendor code).  The rear-camera OTPs (gc5025 family
 * and hi556) are not I2C EEPROMs: they are read through the sensor's
 * own registers with an indirect protocol (write address -> strobe ->
 * read a fixed data register), which the generic qcom,mem linear
 * i2c_read_seq cannot express -- without this the probes die with -107
 * and no calibration/module data ever reaches mm-camera.
 *
 * The hi556 OTP additionally carries the camera-module variant id at
 * byte 8 ("vcm_id": 1 = hi556, 2 = hi556n).  The stock kernel latches
 * it into a global that msm_sensor_match_id() uses to steer the
 * userspace probe to the right sensor lib (and thus the right
 * chromatix set): effective chip id = 0x557 - vcm_id, matched against
 * 0x556 (libmmcamera_hi556) / 0x555 (libmmcamera_hi556n).
 */
uint8_t msm_eeprom_hi556_vcm_id;
EXPORT_SYMBOL(msm_eeprom_hi556_vcm_id);

#define IIC_WRITE_WORD(addr, data) \
	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client), \
		addr, data, MSM_CAMERA_I2C_WORD_DATA)
#define IIC_WRITE_BYTE(addr, data) \
	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client), \
		addr, data, MSM_CAMERA_I2C_BYTE_DATA)
#define IIC_READ_BYTE(addr, data) \
	e_ctrl->i2c_client.i2c_func_tbl->i2c_read(&(e_ctrl->i2c_client), \
		addr, data, MSM_CAMERA_I2C_BYTE_DATA)

struct otp_init_t {
	uint8_t reg_addr;
	uint8_t reg_data;
};

static const struct otp_init_t gc5025_otp_init_setting[9] = {
	{0xfe, 0x00},
	{0xfe, 0x00},
	{0xfe, 0x00},
	{0xf7, 0x01},
	{0xf8, 0x11},
	{0xf9, 0x00},
	{0xfa, 0xb0},
	{0xfc, 0x2e},
	{0xd4, 0x80},
};

/* hi556 OTP-mode init sequence (word regs), verbatim from stock */
static const uint16_t hi556_otp_init_setting[][2] = {
	{0x0a00, 0x0000}, {0x0e00, 0x0102}, {0x0e02, 0x0102},
	{0x0e0c, 0x0100}, {0x2000, 0x7400}, {0x2002, 0x001c},
	{0x2004, 0x0242}, {0x2006, 0x0942}, {0x2008, 0x7007},
	{0x200a, 0x0fd9}, {0x200c, 0x0259}, {0x200e, 0x7008},
	{0x2010, 0x160e}, {0x2012, 0x0047}, {0x2014, 0x2118},
	{0x2016, 0x0041}, {0x2018, 0x00d8}, {0x201a, 0x0145},
	{0x201c, 0x0006}, {0x201e, 0x0181}, {0x2020, 0x13cc},
	{0x2022, 0x2057}, {0x2024, 0x7001}, {0x2026, 0x0fca},
	{0x2028, 0x00cb}, {0x202a, 0x009f}, {0x202c, 0x7002},
	{0x202e, 0x13cc}, {0x2030, 0x019b}, {0x2032, 0x014d},
	{0x2034, 0x2987}, {0x2036, 0x2766}, {0x2038, 0x0020},
	{0x203a, 0x2060}, {0x203c, 0x0e5d}, {0x203e, 0x181d},
	{0x2040, 0x2066}, {0x2042, 0x20c4}, {0x2044, 0x5000},
	{0x2046, 0x0005}, {0x2048, 0x0000}, {0x204a, 0x01db},
	{0x204c, 0x025a}, {0x204e, 0x00c0}, {0x2050, 0x0005},
	{0x2052, 0x0006}, {0x2054, 0x0ad9}, {0x2056, 0x0259},
	{0x2058, 0x0618}, {0x205a, 0x0258}, {0x205c, 0x2266},
	{0x205e, 0x20c8}, {0x2060, 0x2060}, {0x2062, 0x707b},
	{0x2064, 0x0fdd}, {0x2066, 0x81b8}, {0x2068, 0x5040},
	{0x206a, 0x0020}, {0x206c, 0x5060}, {0x206e, 0x3143},
	{0x2070, 0x5081}, {0x2072, 0x025c}, {0x2074, 0x7800},
	{0x2076, 0x7400}, {0x2078, 0x001c}, {0x207a, 0x0242},
	{0x207c, 0x0942}, {0x207e, 0x0bd9}, {0x2080, 0x0259},
	{0x2082, 0x7008}, {0x2084, 0x160e}, {0x2086, 0x0047},
	{0x2088, 0x2118}, {0x208a, 0x0041}, {0x208c, 0x00d8},
	{0x208e, 0x0145}, {0x2090, 0x0006}, {0x2092, 0x0181},
	{0x2094, 0x13cc}, {0x2096, 0x2057}, {0x2098, 0x7001},
	{0x209a, 0x0fca}, {0x209c, 0x00cb}, {0x209e, 0x009f},
	{0x20a0, 0x7002}, {0x20a2, 0x13cc}, {0x20a4, 0x019b},
	{0x20a6, 0x014d}, {0x20a8, 0x2987}, {0x20aa, 0x2766},
	{0x20ac, 0x0020}, {0x20ae, 0x2060}, {0x20b0, 0x0e5d},
	{0x20b2, 0x181d}, {0x20b4, 0x2066}, {0x20b6, 0x20c4},
	{0x20b8, 0x50a0}, {0x20ba, 0x0005}, {0x20bc, 0x0000},
	{0x20be, 0x01db}, {0x20c0, 0x025a}, {0x20c2, 0x00c0},
	{0x20c4, 0x0005}, {0x20c6, 0x0006}, {0x20c8, 0x0ad9},
	{0x20ca, 0x0259}, {0x20cc, 0x0618}, {0x20ce, 0x0258},
	{0x20d0, 0x2266}, {0x20d2, 0x20c8}, {0x20d4, 0x2060},
	{0x20d6, 0x707b}, {0x20d8, 0x0fdd}, {0x20da, 0x86b8},
	{0x20dc, 0x50e0}, {0x20de, 0x0020}, {0x20e0, 0x5100},
	{0x20e2, 0x3143}, {0x20e4, 0x5121}, {0x20e6, 0x7800},
	{0x20e8, 0x3140}, {0x20ea, 0x01c4}, {0x20ec, 0x01c1},
	{0x20ee, 0x01c0}, {0x20f0, 0x01c4}, {0x20f2, 0x2700},
	{0x20f4, 0x3d40}, {0x20f6, 0x7800}, {0x20f8, 0xffff},
	{0x27fe, 0xe000}, {0x3000, 0x60f8}, {0x3002, 0x187f},
	{0x3004, 0x7060}, {0x3006, 0x0114}, {0x3008, 0x60b0},
	{0x300a, 0x1473}, {0x300c, 0x0013}, {0x300e, 0x140f},
	{0x3010, 0x0040}, {0x3012, 0x100f}, {0x3014, 0x60f8},
	{0x3016, 0x187f}, {0x3018, 0x7060}, {0x301a, 0x0114},
	{0x301c, 0x60b0}, {0x301e, 0x1473}, {0x3020, 0x0013},
	{0x3022, 0x140f}, {0x3024, 0x0040}, {0x3026, 0x000f},
	{0x0b00, 0x0000}, {0x0b02, 0x0045}, {0x0b04, 0xb405},
	{0x0b06, 0xc403}, {0x0b08, 0x0081}, {0x0b0a, 0x8252},
	{0x0b0c, 0xf814}, {0x0b0e, 0xc618}, {0x0b10, 0xa828},
	{0x0b12, 0x004c}, {0x0b14, 0x4068}, {0x0b16, 0x0000},
	{0x0f30, 0x6e25}, {0x0f32, 0x7067}, {0x0954, 0x0009},
	{0x0956, 0x1100}, {0x0958, 0xcc80}, {0x095a, 0x0000},
	{0x0c00, 0x1110}, {0x0c02, 0x0011}, {0x0c04, 0x0000},
	{0x0c06, 0x0200}, {0x0c10, 0x0040}, {0x0c12, 0x0040},
	{0x0c14, 0x0040}, {0x0c16, 0x0040}, {0x0a10, 0x4000},
	{0x3068, 0xf800}, {0x306a, 0xf876}, {0x006c, 0x0000},
	{0x005e, 0x0200}, {0x000e, 0x0200}, {0x0e0a, 0x0001},
	{0x004a, 0x0100}, {0x004c, 0x0000}, {0x004e, 0x0100},
	{0x000c, 0x0022}, {0x0008, 0x0b00}, {0x005a, 0x0202},
	{0x0012, 0x000e}, {0x0018, 0x0a31}, {0x0022, 0x0008},
	{0x0028, 0x0017}, {0x0024, 0x0028}, {0x002a, 0x002d},
	{0x0026, 0x0030}, {0x002c, 0x07c7}, {0x002e, 0x1111},
	{0x0030, 0x1111}, {0x0032, 0x1111}, {0x0006, 0x07bc},
	{0x0a22, 0x0000}, {0x0a12, 0x0a20}, {0x0a14, 0x0798},
	{0x003e, 0x0000}, {0x0074, 0x080e}, {0x0070, 0x0407},
	{0x0002, 0x0000}, {0x0a02, 0x0100}, {0x0a24, 0x0100},
	{0x0046, 0x0000}, {0x0076, 0x0000}, {0x0060, 0x0000},
	{0x0062, 0x0530}, {0x0064, 0x0500}, {0x0066, 0x0530},
	{0x0068, 0x0500}, {0x0122, 0x0300}, {0x015a, 0xff08},
	{0x0804, 0x0200}, {0x005c, 0x0102}, {0x0a1a, 0x0800},
};

static int is_tsp5025(struct msm_eeprom_ctrl_t *e_ctrl)
{
	uint8_t sensoridaddr = 0xf0;
	uint16_t sensorid = 0;
	int rc;

	e_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_read(&(e_ctrl->i2c_client),
		sensoridaddr, &sensorid, MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0) {
		pr_err("%s: stoneadd read sensor id failed\n", __func__);
		return rc;
	}
	if (sensorid == 0x5025) {
		pr_err("stoneadd is a gc5025 module!\n");
		return 0;
	}
	pr_err("stoneadd is not a gc5025 module! id=0x%x\n", sensorid);
	return 1;
}

static int is_hlt556(struct msm_eeprom_ctrl_t *e_ctrl)
{
	uint16_t sensoridaddr = 0x0f16;
	uint16_t sensorid = 0;
	int rc;

	e_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_read(&(e_ctrl->i2c_client),
		sensoridaddr, &sensorid, MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0) {
		pr_err("%s: stoneadd read sensor id failed\n", __func__);
		return rc;
	}
	if (sensorid == 0x0556) {
		pr_err("stoneadd is a hlt556 module!\n");
		return 0;
	}
	pr_err("stoneadd is not a hlt556 module! id=0x%x\n", sensorid);
	return 1;
}

static void gc5025_otp_init(struct msm_eeprom_ctrl_t *e_ctrl)
{
	uint8_t i;

	e_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	for (i = 0; i < ARRAY_SIZE(gc5025_otp_init_setting); i++)
		e_ctrl->i2c_client.i2c_func_tbl->i2c_write(
			&(e_ctrl->i2c_client),
			gc5025_otp_init_setting[i].reg_addr,
			gc5025_otp_init_setting[i].reg_data,
			MSM_CAMERA_I2C_BYTE_DATA);
	pr_err("%s stoneadd write otp init OK", __func__);
}

static void hi556_otp_init(struct msm_eeprom_ctrl_t *e_ctrl)
{
	int i;

	e_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	for (i = 0; i < ARRAY_SIZE(hi556_otp_init_setting); i++)
		IIC_WRITE_WORD(hi556_otp_init_setting[i][0],
			       hi556_otp_init_setting[i][1]);

	/* OTP reading prepare */
	IIC_WRITE_BYTE(0x0a02, 0x01);	/* Fast sleep on */
	IIC_WRITE_BYTE(0x0a00, 0x00);	/* stand by on */
	msleep(10);
	IIC_WRITE_BYTE(0x0f02, 0x00);	/* pll disable */
	IIC_WRITE_BYTE(0x011a, 0x01);	/* CP TRIM_H */
	IIC_WRITE_BYTE(0x011b, 0x09);	/* IPGM TRIM_H */
	IIC_WRITE_BYTE(0x0d04, 0x01);	/* OTP busy output enable */
	IIC_WRITE_BYTE(0x0d00, 0x07);	/* OTP busy output drivability */
	IIC_WRITE_BYTE(0x003e, 0x10);	/* OTP R/W mode */
	IIC_WRITE_BYTE(0x0a00, 0x01);	/* stand by off */
}

/*
 * msm_get_read_mem_size - Get the total size for allocation
 * @eeprom_map_array:	mem map
 *
 * Returns size after computation size, returns error in case of error
 */
static int msm_get_read_mem_size
	(struct msm_eeprom_memory_map_array *eeprom_map_array) {
	int size = 0, i, j;
	struct msm_eeprom_mem_map_t *eeprom_map;

	if (eeprom_map_array->msm_size_of_max_mappings >
		MSM_EEPROM_MAX_MEM_MAP_CNT) {
		pr_err("%s:%d Memory map cnt greter then expected: %d",
			__func__, __LINE__,
			eeprom_map_array->msm_size_of_max_mappings);
		return -EINVAL;
	}
	for (j = 0; j < eeprom_map_array->msm_size_of_max_mappings; j++) {
		eeprom_map = &(eeprom_map_array->memory_map[j]);
		if (eeprom_map->memory_map_size >
			MSM_EEPROM_MEMORY_MAP_MAX_SIZE) {
			pr_err("%s:%d Memory map size greter then expected: %d",
				__func__, __LINE__,
				eeprom_map->memory_map_size);
			return -EINVAL;
		}
		for (i = 0; i < eeprom_map->memory_map_size; i++) {
			if ((eeprom_map->mem_settings[i].i2c_operation ==
				MSM_CAM_READ) ||
				(eeprom_map->mem_settings[i].i2c_operation ==
				MSM_CAM_READ_LOOP)) {
				size += eeprom_map->mem_settings[i].reg_data;
			}
		}
	}
	CDBG("Total Data Size: %d\n", size);
	return size;
}

/*
 * msm_eeprom_verify_sum - verify crc32 checksum
 * @mem:	data buffer
 * @size:	size of data buffer
 * @sum:	expected checksum
 *
 * Returns 0 if checksum match, -EINVAL otherwise.
 */
static int msm_eeprom_verify_sum(const char *mem, uint32_t size, uint32_t sum)
{
	uint32_t crc = ~0;

	/* check overflow */
	if (size > crc - sizeof(uint32_t))
		return -EINVAL;

	crc = crc32_le(crc, mem, size);
	if (~crc != sum) {
		CDBG("%s: expect 0x%x, result 0x%x\n", __func__, sum, ~crc);
		return -EINVAL;
	}
	CDBG("%s: checksum pass 0x%x\n", __func__, sum);
	return 0;
}

/*
 * msm_eeprom_match_crc - verify multiple regions using crc
 * @data:	data block to be verified
 *
 * Iterates through all regions stored in @data.  Regions with odd index
 * are treated as data, and its next region is treated as checksum.  Thus
 * regions of even index must have valid_size of 4 or 0 (skip verification).
 * Returns a bitmask of verified regions, starting from LSB.  1 indicates
 * a checksum match, while 0 indicates checksum mismatch or not verified.
 */
static uint32_t msm_eeprom_match_crc(struct msm_eeprom_memory_block_t *data)
{
	int j, rc;
	uint32_t *sum;
	uint32_t ret = 0;
	uint8_t *memptr;
	struct msm_eeprom_memory_map_t *map;

	if (!data) {
		pr_err("%s data is NULL", __func__);
		return -EINVAL;
	}
	map = data->map;
	memptr = data->mapdata;

	for (j = 0; j + 1 < data->num_map; j += 2) {
		/* empty table or no checksum */
		if (!map[j].mem.valid_size || !map[j+1].mem.valid_size) {
			memptr += map[j].mem.valid_size
				+ map[j+1].mem.valid_size;
			continue;
		}
		if (map[j+1].mem.valid_size != sizeof(uint32_t)) {
			CDBG("%s: malformatted data mapping\n", __func__);
			return -EINVAL;
		}
		sum = (uint32_t *) (memptr + map[j].mem.valid_size);
		rc = msm_eeprom_verify_sum(memptr, map[j].mem.valid_size,
					   *sum);
		if (!rc)
			ret |= 1 << (j/2);
		memptr += map[j].mem.valid_size + map[j+1].mem.valid_size;
	}
	return ret;
}

/*
 * read_eeprom_memory() - read map data into buffer
 * @e_ctrl:	eeprom control struct
 * @block:	block to be read
 *
 * This function iterates through blocks stored in block->map, reads each
 * region and concatenate them into the pre-allocated block->mapdata
 */
static int read_eeprom_memory(struct msm_eeprom_ctrl_t *e_ctrl,
	struct msm_eeprom_memory_block_t *block)
{
	int rc = 0;
	int j;
	int gc = 0;
	uint16_t gc_read = 0;
	bool is_gc5025 = false, is_hi556 = false;
	struct msm_eeprom_memory_map_t *emap = block->map;
	struct msm_eeprom_board_info *eb_info;
	uint8_t *memptr = block->mapdata;

	if (!e_ctrl) {
		pr_err("%s e_ctrl is NULL", __func__);
		return -EINVAL;
	}

	eb_info = e_ctrl->eboard_info;

	if (!strcmp(eb_info->eeprom_name, "gc5025_otp") ||
	    !strcmp(eb_info->eeprom_name, "gc5025h_otp")) {
		if (!is_tsp5025(e_ctrl)) {
			is_gc5025 = true;
			gc5025_otp_init(e_ctrl);
		} else {
			CDBG("stoneadd bypass gc5025 module\n");
			return -1;
		}
	} else if (!strcmp(eb_info->eeprom_name, "hi556_otp")) {
		if (is_hlt556(e_ctrl)) {
			CDBG("stoneadd bypass hlt556 module\n");
			return -1;
		}
		is_hi556 = true;
	}

	for (j = 0; j < block->num_map; j++) {
		if (emap[j].saddr.addr) {
			eb_info->i2c_slaveaddr = emap[j].saddr.addr;
			e_ctrl->i2c_client.cci_client->sid =
					eb_info->i2c_slaveaddr >> 1;
			pr_err("qcom,slave-addr = 0x%X\n",
				eb_info->i2c_slaveaddr);
		}

		if (emap[j].page.valid_size) {
			e_ctrl->i2c_client.addr_type = emap[j].page.addr_t;
			rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_write(
				&(e_ctrl->i2c_client), emap[j].page.addr,
				emap[j].page.data, emap[j].page.data_t);
				msleep(emap[j].page.delay);
			if (rc < 0) {
				pr_err("%s: page write failed\n", __func__);
				return rc;
			}
		}
		if (emap[j].pageen.valid_size) {
			e_ctrl->i2c_client.addr_type = emap[j].pageen.addr_t;
			rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_write(
				&(e_ctrl->i2c_client), emap[j].pageen.addr,
				emap[j].pageen.data, emap[j].pageen.data_t);
				msleep(emap[j].pageen.delay);
			if (rc < 0) {
				pr_err("%s: page enable failed\n", __func__);
				return rc;
			}
		}
		if (emap[j].poll.valid_size) {
			e_ctrl->i2c_client.addr_type = emap[j].poll.addr_t;
			rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_poll(
				&(e_ctrl->i2c_client), emap[j].poll.addr,
				emap[j].poll.data, emap[j].poll.data_t,
				emap[j].poll.delay);
			if (rc < 0) {
				pr_err("%s: poll failed\n", __func__);
				return rc;
			}
		}

		if (emap[j].mem.valid_size) {
			if (is_gc5025) {
				/* indirect read: 0xd4/0xd5 = addr, 0xf3 strobe,
				 * then each byte pops out of 0xd7 */
				pr_err("stoneadd reading gc5025 otp eeprom\n");
				e_ctrl->i2c_client.addr_type =
					MSM_CAMERA_I2C_BYTE_ADDR;
				IIC_WRITE_BYTE(0xd4,
					((emap[j].mem.addr >> 8) | 0x80) & 0xff);
				IIC_WRITE_BYTE(0xd5, emap[j].mem.addr & 0xff);
				IIC_WRITE_BYTE(0xf3, 0x20);
				IIC_WRITE_BYTE(0xf3, 0x88);
				msleep(emap[j].mem.delay);
				for (gc = 0; gc < emap[j].mem.valid_size; gc++) {
					rc = IIC_READ_BYTE(0xd7, &gc_read);
					if (rc < 0) {
						pr_err("%s: read failed %d\n",
							__func__, __LINE__);
						return rc;
					}
					*memptr++ = (uint8_t)gc_read;
					CDBG("stoneadd gc5025 otp:0x%x\n",
						gc_read);
				}
			} else if (is_hi556) {
				pr_err("stoneadd reading hi556 otp eeprom\n");
				hi556_otp_init(e_ctrl);
				/* start address (0x0401), continuous read */
				IIC_WRITE_BYTE(0x10a,
					(emap[j].mem.addr >> 8) & 0xff);
				IIC_WRITE_BYTE(0x10b, emap[j].mem.addr & 0xff);
				IIC_WRITE_BYTE(0x102, 0x01);
				for (gc = 0; gc < emap[j].mem.valid_size; gc++) {
					rc = IIC_READ_BYTE(0x108, &gc_read);
					CDBG("stoneadd hi556_otp:[%d] = 0x%X\n",
						gc, (uint8_t)gc_read);
					if (rc < 0) {
						pr_err("%s: read failed\n",
							__func__);
						return rc;
					}
					/* byte 8 = module variant ("vcm_id"):
					 * 1 = hi556, 2 = hi556n */
					if (gc == 8)
						msm_eeprom_hi556_vcm_id =
							(uint8_t)gc_read;
					*memptr++ = (uint8_t)gc_read;
				}
				pr_err("hmz stoneadd hi556_otp vcm_id = 0x%X\n",
					msm_eeprom_hi556_vcm_id);
			} else {
				e_ctrl->i2c_client.addr_type = emap[j].mem.addr_t;
				rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_read_seq(
					&(e_ctrl->i2c_client), emap[j].mem.addr,
					memptr, emap[j].mem.valid_size);
				if (rc < 0) {
					pr_err("%s: read failed\n", __func__);
					return rc;
				}
				memptr += emap[j].mem.valid_size;
			}
		}
		if (emap[j].pageen.valid_size) {
			e_ctrl->i2c_client.addr_type = emap[j].pageen.addr_t;
			rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_write(
				&(e_ctrl->i2c_client), emap[j].pageen.addr,
				0, emap[j].pageen.data_t);
			if (rc < 0) {
				pr_err("%s: page disable failed\n", __func__);
				return rc;
			}
		}
	}
	return rc;
}
/*
 * msm_eeprom_parse_memory_map() - parse memory map in device node
 * @of:	device node
 * @data:	memory block for output
 *
 * This functions parses @of to fill @data.  It allocates map itself, parses
 * the @of node, calculate total data length, and allocates required buffer.
 * It only fills the map, but does not perform actual reading.
 */
static int msm_eeprom_parse_memory_map(struct device_node *of,
	struct msm_eeprom_memory_block_t *data)
{
	int i, rc = 0;
	char property[PROPERTY_MAXSIZE];
	uint32_t count = 6;
	struct msm_eeprom_memory_map_t *map;

	snprintf(property, PROPERTY_MAXSIZE, "qcom,num-blocks");
	rc = of_property_read_u32(of, property, &data->num_map);
	CDBG("%s: %s %d\n", __func__, property, data->num_map);
	if (rc < 0) {
		pr_err("%s failed rc %d\n", __func__, rc);
		return rc;
	}

	map = kzalloc((sizeof(*map) * data->num_map), GFP_KERNEL);
	if (!map) {
		rc = -ENOMEM;
		pr_err("%s failed line %d\n", __func__, __LINE__);
		return rc;
	}
	data->map = map;

	for (i = 0; i < data->num_map; i++) {
		snprintf(property, PROPERTY_MAXSIZE, "qcom,page%d", i);
		rc = of_property_read_u32_array(of, property,
			(uint32_t *) &map[i].page, count);
		if (rc < 0) {
			pr_err("%s: failed %d\n", __func__, __LINE__);
			goto ERROR;
		}

		snprintf(property, PROPERTY_MAXSIZE,
			"qcom,pageen%d", i);
		rc = of_property_read_u32_array(of, property,
			(uint32_t *) &map[i].pageen, count);
		if (rc < 0)
			CDBG("%s: pageen not needed\n", __func__);

		snprintf(property, PROPERTY_MAXSIZE, "qcom,saddr%d", i);
		rc = of_property_read_u32_array(of, property,
			(uint32_t *) &map[i].saddr.addr, 1);
		if (rc < 0)
			CDBG("%s: saddr not needed - block %d\n", __func__, i);

		snprintf(property, PROPERTY_MAXSIZE, "qcom,poll%d", i);
		rc = of_property_read_u32_array(of, property,
			(uint32_t *) &map[i].poll, count);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto ERROR;
		}

		snprintf(property, PROPERTY_MAXSIZE, "qcom,mem%d", i);
		rc = of_property_read_u32_array(of, property,
			(uint32_t *) &map[i].mem, count);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto ERROR;
		}
		data->num_data += map[i].mem.valid_size;
	}

	CDBG("%s num_bytes %d\n", __func__, data->num_data);

	data->mapdata = kzalloc(data->num_data, GFP_KERNEL);
	if (!data->mapdata) {
		rc = -ENOMEM;
		pr_err("%s failed line %d\n", __func__, __LINE__);
		goto ERROR;
	}
	return rc;

ERROR:
	kfree(data->map);
	memset(data, 0, sizeof(*data));
	return rc;
}

/*
 * eeprom_parse_memory_map - Parse mem map
 * @e_ctrl:	ctrl structure
 * @eeprom_map_array: eeprom map
 *
 * Returns success or failure
 */
static int eeprom_parse_memory_map(struct msm_eeprom_ctrl_t *e_ctrl,
	struct msm_eeprom_memory_map_array *eeprom_map_array)
{
	int rc =  0, i, j, gc;
	uint8_t *memptr;
	uint16_t gc_read = 0;

	struct msm_eeprom_mem_map_t *eeprom_map;

	e_ctrl->cal_data.mapdata = NULL;
	e_ctrl->cal_data.num_data = msm_get_read_mem_size(eeprom_map_array);
	if (e_ctrl->cal_data.num_data <= 0) {
		pr_err("%s:%d Error in reading mem size\n",
			__func__, __LINE__);
		e_ctrl->cal_data.num_data = 0;
		return -EINVAL;
	}
	e_ctrl->cal_data.mapdata =
		kzalloc(e_ctrl->cal_data.num_data, GFP_KERNEL);
	if (!e_ctrl->cal_data.mapdata)
		return -ENOMEM;

	memptr = e_ctrl->cal_data.mapdata;
	for (j = 0; j < eeprom_map_array->msm_size_of_max_mappings; j++) {
		eeprom_map = &(eeprom_map_array->memory_map[j]);
		if (e_ctrl->i2c_client.cci_client) {
			e_ctrl->i2c_client.cci_client->sid =
				eeprom_map->slave_addr >> 1;
		} else if (e_ctrl->i2c_client.client) {
			e_ctrl->i2c_client.client->addr =
				eeprom_map->slave_addr >> 1;
		}
		CDBG("Slave Addr: 0x%X\n", eeprom_map->slave_addr);
		CDBG("Memory map Size: %d",
			eeprom_map->memory_map_size);
		for (i = 0; i < eeprom_map->memory_map_size; i++) {
			switch (eeprom_map->mem_settings[i].i2c_operation) {
			case MSM_CAM_WRITE: {
				e_ctrl->i2c_client.addr_type =
					eeprom_map->mem_settings[i].addr_type;
				rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_write(
					&(e_ctrl->i2c_client),
					eeprom_map->mem_settings[i].reg_addr,
					eeprom_map->mem_settings[i].reg_data,
					eeprom_map->mem_settings[i].data_type);
				msleep(eeprom_map->mem_settings[i].delay);
				if (rc < 0) {
					pr_err("%s: page write failed\n",
						__func__);
					goto clean_up;
				}
			}
			break;
			case MSM_CAM_POLL: {
				e_ctrl->i2c_client.addr_type =
					eeprom_map->mem_settings[i].addr_type;
				rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_poll(
					&(e_ctrl->i2c_client),
					eeprom_map->mem_settings[i].reg_addr,
					eeprom_map->mem_settings[i].reg_data,
					eeprom_map->mem_settings[i].data_type,
					eeprom_map->mem_settings[i].delay);
				if (rc < 0) {
					pr_err("%s: poll failed\n",
						__func__);
					goto clean_up;
				}
			}
			break;
			case MSM_CAM_READ: {
				e_ctrl->i2c_client.addr_type =
					eeprom_map->mem_settings[i].addr_type;
				rc = e_ctrl->i2c_client.i2c_func_tbl->
					i2c_read_seq(&(e_ctrl->i2c_client),
					eeprom_map->mem_settings[i].reg_addr,
					memptr,
					eeprom_map->mem_settings[i].reg_data);
				msleep(eeprom_map->mem_settings[i].delay);
				if (rc < 0) {
					pr_err("%s: read failed\n",
						__func__);
					goto clean_up;
				}
				memptr += eeprom_map->mem_settings[i].reg_data;
			}
			break;

			case MSM_CAM_READ_LOOP: {
				e_ctrl->i2c_client.addr_type =
				 eeprom_map->mem_settings[i].addr_type;

				for (gc = 0;
					gc < eeprom_map->mem_settings[i].
						reg_data;
					gc++) {
					msleep(eeprom_map->mem_settings[i].
						delay);
					rc = e_ctrl->i2c_client.i2c_func_tbl->
						i2c_read(
						&(e_ctrl->i2c_client),
						eeprom_map->mem_settings[i].
						reg_addr,
						&gc_read,
						eeprom_map->mem_settings[i].
						data_type);
					if (rc < 0) {
						pr_err("%s: read failed\n",
							__func__);
						goto clean_up;
					}
					*memptr = (uint8_t)gc_read;
					memptr++;
				}
			}
			break;

			default:
				pr_err("%s: %d Invalid i2c operation LC:%d, op: %d\n",
					__func__, __LINE__, i,
					eeprom_map->mem_settings[i].
						i2c_operation);
				return -EINVAL;
			}
		}
	}
	memptr = e_ctrl->cal_data.mapdata;
	for (i = 0; i < e_ctrl->cal_data.num_data; i++)
		CDBG("memory_data[%d] = 0x%X\n", i, memptr[i]);
	return rc;

clean_up:
	kfree(e_ctrl->cal_data.mapdata);
	e_ctrl->cal_data.num_data = 0;
	e_ctrl->cal_data.mapdata = NULL;
	return rc;
}

/*
 * msm_eeprom_power_up - Do eeprom power up here
 * @e_ctrl:	ctrl structure
 * @power_info: power up info for eeprom
 *
 * Returns success or failure
 */
static int msm_eeprom_power_up(struct msm_eeprom_ctrl_t *e_ctrl,
	struct msm_camera_power_ctrl_t *power_info) {
	int32_t rc = 0;

	rc = msm_camera_fill_vreg_params(
		power_info->cam_vreg, power_info->num_vreg,
		power_info->power_setting, power_info->power_setting_size);
	if (rc < 0) {
		pr_err("%s:%d failed in camera_fill_vreg_params  rc %d",
			__func__, __LINE__, rc);
		return rc;
	}

	/* Parse and fill vreg params for powerdown settings*/
	rc = msm_camera_fill_vreg_params(
		power_info->cam_vreg, power_info->num_vreg,
		power_info->power_down_setting,
		power_info->power_down_setting_size);
	if (rc < 0) {
		pr_err("%s:%d failed msm_camera_fill_vreg_params for PDOWN rc %d",
			__func__, __LINE__, rc);
		return rc;
	}

	rc = msm_camera_power_up(power_info, e_ctrl->eeprom_device_type,
			&e_ctrl->i2c_client);
	if (rc) {
		pr_err("%s:%d failed in eeprom Power up rc %d\n",
		__func__, __LINE__, rc);
		return rc;
	}
	return rc;
}

/*
 * msm_eeprom_power_up - Do power up, parse and power down
 * @e_ctrl: ctrl structure
 * Returns success or failure
 */
static int eeprom_init_config(struct msm_eeprom_ctrl_t *e_ctrl,
	void *argp)
{
	int rc =  0;
	struct msm_eeprom_cfg_data *cdata = argp;
	struct msm_sensor_power_setting_array *power_setting_array = NULL;
	struct msm_camera_power_ctrl_t *power_info;
	struct msm_eeprom_memory_map_array *memory_map_arr = NULL;

	power_setting_array =
		kzalloc(sizeof(struct msm_sensor_power_setting_array),
			GFP_KERNEL);
	if (!power_setting_array) {
		pr_err("%s:%d Mem Alloc Fail\n", __func__, __LINE__);
		rc = -ENOMEM;
		return rc;
	}
	memory_map_arr = kzalloc(sizeof(struct msm_eeprom_memory_map_array),
		GFP_KERNEL);
	if (!memory_map_arr) {
		rc = -ENOMEM;
		pr_err("%s:%d Mem Alloc Fail\n", __func__, __LINE__);
		goto free_mem;
	}

	if (copy_from_user(power_setting_array,
		(void __user *)cdata->cfg.eeprom_info.power_setting_array,
		sizeof(struct msm_sensor_power_setting_array))) {
		pr_err("%s copy_from_user failed %d\n",
			__func__, __LINE__);
		goto free_mem;
	}
	CDBG("%s:%d Size of power setting array: %d\n",
		__func__, __LINE__, power_setting_array->size);
	if (copy_from_user(memory_map_arr,
		(void __user *)cdata->cfg.eeprom_info.mem_map_array,
		sizeof(struct msm_eeprom_memory_map_array))) {
		rc = -EINVAL;
		pr_err("%s copy_from_user failed for memory map%d\n",
			__func__, __LINE__);
		goto free_mem;
	}

	power_info = &(e_ctrl->eboard_info->power_info);

	power_info->power_setting =
		power_setting_array->power_setting_a;
	power_info->power_down_setting =
		power_setting_array->power_down_setting_a;

	power_info->power_setting_size =
		power_setting_array->size;
	power_info->power_down_setting_size =
		power_setting_array->size_down;

	if ((power_info->power_setting_size >
		MAX_POWER_CONFIG) ||
		(power_info->power_down_setting_size >
		MAX_POWER_CONFIG) ||
		(!power_info->power_down_setting_size) ||
		(!power_info->power_setting_size)) {
		rc = -EINVAL;
		pr_err("%s:%d Invalid power setting size :%d, %d\n",
			__func__, __LINE__,
			power_info->power_setting_size,
			power_info->power_down_setting_size);
		goto free_mem;
	}

	if (e_ctrl->i2c_client.cci_client) {
		e_ctrl->i2c_client.cci_client->i2c_freq_mode =
			cdata->cfg.eeprom_info.i2c_freq_mode;
		if (e_ctrl->i2c_client.cci_client->i2c_freq_mode >
			I2C_MAX_MODES) {
			pr_err("%s::%d Improper I2C freq mode\n",
				__func__, __LINE__);
			e_ctrl->i2c_client.cci_client->i2c_freq_mode =
				I2C_STANDARD_MODE;
		}
	}

	/* Fill vreg power info and power up here */
	rc = msm_eeprom_power_up(e_ctrl, power_info);
	if (rc < 0) {
		pr_err("Power Up failed for eeprom\n");
		goto free_mem;
	}

	rc = eeprom_parse_memory_map(e_ctrl, memory_map_arr);
	if (rc < 0) {
		pr_err("%s::%d memory map parse failed\n", __func__, __LINE__);
		goto free_mem;
	}

	rc = msm_camera_power_down(power_info, e_ctrl->eeprom_device_type,
		&e_ctrl->i2c_client);
	if (rc < 0) {
		pr_err("%s:%d Power down failed rc %d\n",
			__func__, __LINE__, rc);
		goto free_mem;
	}

free_mem:
	kfree(power_setting_array);
	kfree(memory_map_arr);
	power_setting_array = NULL;
	memory_map_arr = NULL;
	return rc;
}

static int msm_eeprom_get_cmm_data(struct msm_eeprom_ctrl_t *e_ctrl,
				       struct msm_eeprom_cfg_data *cdata)
{
	int rc = 0;
	struct msm_eeprom_cmm_t *cmm_data = &e_ctrl->eboard_info->cmm_data;

	cdata->cfg.get_cmm_data.cmm_support = cmm_data->cmm_support;
	cdata->cfg.get_cmm_data.cmm_compression = cmm_data->cmm_compression;
	cdata->cfg.get_cmm_data.cmm_size = cmm_data->cmm_size;
	return rc;
}

static int eeprom_config_read_cal_data(struct msm_eeprom_ctrl_t *e_ctrl,
	struct msm_eeprom_cfg_data *cdata)
{
	int rc;

	/* check range */
	if (cdata->cfg.read_data.num_bytes >
		e_ctrl->cal_data.num_data) {
		CDBG("%s: Invalid size. exp %u, req %u\n", __func__,
			e_ctrl->cal_data.num_data,
			cdata->cfg.read_data.num_bytes);
		return -EINVAL;
	}

	rc = copy_to_user((void __user *)cdata->cfg.read_data.dbuffer,
		e_ctrl->cal_data.mapdata,
		cdata->cfg.read_data.num_bytes);

	return rc;
}

static int msm_eeprom_config(struct msm_eeprom_ctrl_t *e_ctrl,
	void *argp)
{
	struct msm_eeprom_cfg_data *cdata =
		(struct msm_eeprom_cfg_data *)argp;
	int rc = 0;
	size_t length = 0;

	CDBG("%s E\n", __func__);
	switch (cdata->cfgtype) {
	case CFG_EEPROM_GET_INFO:
		if (e_ctrl->userspace_probe == 1) {
			pr_err("%s:%d Eeprom name should be module driver",
				__func__, __LINE__);
			rc = -EINVAL;
			break;
		}
		CDBG("%s E CFG_EEPROM_GET_INFO\n", __func__);
		cdata->is_supported = e_ctrl->is_supported;
		length = strlen(e_ctrl->eboard_info->eeprom_name) + 1;
		if (length > MAX_EEPROM_NAME) {
			pr_err("%s:%d invalid eeprom_name length %d\n",
				__func__, __LINE__, (int)length);
			rc = -EINVAL;
			break;
		}
		memcpy(cdata->cfg.eeprom_name,
			e_ctrl->eboard_info->eeprom_name, length);
		break;
	case CFG_EEPROM_GET_CAL_DATA:
		CDBG("%s E CFG_EEPROM_GET_CAL_DATA\n", __func__);
		cdata->cfg.get_data.num_bytes =
			e_ctrl->cal_data.num_data;
		break;
	case CFG_EEPROM_READ_CAL_DATA:
		CDBG("%s E CFG_EEPROM_READ_CAL_DATA\n", __func__);
		rc = eeprom_config_read_cal_data(e_ctrl, cdata);
		break;
	case CFG_EEPROM_GET_MM_INFO:
		CDBG("%s E CFG_EEPROM_GET_MM_INFO\n", __func__);
		rc = msm_eeprom_get_cmm_data(e_ctrl, cdata);
		break;
	case CFG_EEPROM_INIT:
		if (e_ctrl->userspace_probe == 0) {
			pr_err("%s:%d Eeprom already probed at kernel boot",
				__func__, __LINE__);
			rc = -EINVAL;
			break;
		}
		if (e_ctrl->cal_data.num_data == 0) {
			rc = eeprom_init_config(e_ctrl, argp);
			if (rc < 0) {
				pr_err("%s:%d Eeprom init failed\n",
					__func__, __LINE__);
				return rc;
			}
		} else {
			CDBG("%s:%d Already read eeprom\n",
				__func__, __LINE__);
		}
		break;
	default:
		break;
	}

	CDBG("%s X rc: %d\n", __func__, rc);
	return rc;
}

static int msm_eeprom_get_subdev_id(struct msm_eeprom_ctrl_t *e_ctrl,
				    void *arg)
{
	uint32_t *subdev_id = (uint32_t *)arg;

	CDBG("%s E\n", __func__);
	if (!subdev_id) {
		pr_err("%s failed\n", __func__);
		return -EINVAL;
	}
	*subdev_id = e_ctrl->subdev_id;
	CDBG("subdev_id %d\n", *subdev_id);
	CDBG("%s X\n", __func__);
	return 0;
}

static long msm_eeprom_subdev_ioctl(struct v4l2_subdev *sd,
		unsigned int cmd, void *arg)
{
	struct msm_eeprom_ctrl_t *e_ctrl = v4l2_get_subdevdata(sd);
	void *argp = (void *)arg;

	CDBG("%s E\n", __func__);
	CDBG("%s:%d a_ctrl %pK argp %pK\n", __func__, __LINE__, e_ctrl, argp);
	switch (cmd) {
	case VIDIOC_MSM_SENSOR_GET_SUBDEV_ID:
		return msm_eeprom_get_subdev_id(e_ctrl, argp);
	case VIDIOC_MSM_EEPROM_CFG:
		return msm_eeprom_config(e_ctrl, argp);
	default:
		return -ENOIOCTLCMD;
	}

	CDBG("%s X\n", __func__);
}

static struct msm_camera_i2c_fn_t msm_eeprom_cci_func_tbl = {
	.i2c_read = msm_camera_cci_i2c_read,
	.i2c_read_seq = msm_camera_cci_i2c_read_seq,
	.i2c_write = msm_camera_cci_i2c_write,
	.i2c_write_seq = msm_camera_cci_i2c_write_seq,
	.i2c_write_table = msm_camera_cci_i2c_write_table,
	.i2c_write_seq_table = msm_camera_cci_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
	msm_camera_cci_i2c_write_table_w_microdelay,
	.i2c_util = msm_sensor_cci_i2c_util,
	.i2c_poll = msm_camera_cci_i2c_poll,
};

static struct msm_camera_i2c_fn_t msm_eeprom_qup_func_tbl = {
	.i2c_read = msm_camera_qup_i2c_read,
	.i2c_read_seq = msm_camera_qup_i2c_read_seq,
	.i2c_write = msm_camera_qup_i2c_write,
	.i2c_write_table = msm_camera_qup_i2c_write_table,
	.i2c_write_seq_table = msm_camera_qup_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
	msm_camera_qup_i2c_write_table_w_microdelay,
};

static struct msm_camera_i2c_fn_t msm_eeprom_spi_func_tbl = {
	.i2c_read = msm_camera_spi_read,
	.i2c_read_seq = msm_camera_spi_read_seq,
};

static int msm_eeprom_open(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh) {
	int rc = 0;
	struct msm_eeprom_ctrl_t *e_ctrl =  v4l2_get_subdevdata(sd);

	CDBG("%s E\n", __func__);
	if (!e_ctrl) {
		pr_err("%s failed e_ctrl is NULL\n", __func__);
		return -EINVAL;
	}
	CDBG("%s X\n", __func__);
	return rc;
}

static int msm_eeprom_close(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh) {
	int rc = 0;
	struct msm_eeprom_ctrl_t *e_ctrl =  v4l2_get_subdevdata(sd);

	CDBG("%s E\n", __func__);
	if (!e_ctrl) {
		pr_err("%s failed e_ctrl is NULL\n", __func__);
		return -EINVAL;
	}
	CDBG("%s X\n", __func__);
	return rc;
}

static const struct v4l2_subdev_internal_ops msm_eeprom_internal_ops = {
	.open = msm_eeprom_open,
	.close = msm_eeprom_close,
};

static struct v4l2_subdev_core_ops msm_eeprom_subdev_core_ops = {
	.ioctl = msm_eeprom_subdev_ioctl,
};

static struct v4l2_subdev_ops msm_eeprom_subdev_ops = {
	.core = &msm_eeprom_subdev_core_ops,
};

static int msm_eeprom_get_dt_data(struct msm_eeprom_ctrl_t *e_ctrl)
{
	int rc = 0, i = 0;
	struct msm_eeprom_board_info *eb_info;
	struct msm_camera_power_ctrl_t *power_info =
		&e_ctrl->eboard_info->power_info;
	struct device_node *of_node = NULL;
	struct msm_camera_gpio_conf *gconf = NULL;
	int8_t gpio_array_size = 0;
	uint16_t *gpio_array = NULL;

	eb_info = e_ctrl->eboard_info;
	if (e_ctrl->eeprom_device_type == MSM_CAMERA_SPI_DEVICE)
		of_node = e_ctrl->i2c_client.
			spi_client->spi_master->dev.of_node;
	else if (e_ctrl->eeprom_device_type == MSM_CAMERA_PLATFORM_DEVICE)
		of_node = e_ctrl->pdev->dev.of_node;
	else if (e_ctrl->eeprom_device_type == MSM_CAMERA_I2C_DEVICE)
		of_node = e_ctrl->i2c_client.client->dev.of_node;

	if (!of_node) {
		pr_err("%s: %d of_node is NULL\n", __func__, __LINE__);
		return -ENOMEM;
	}
	rc = msm_camera_get_dt_vreg_data(of_node, &power_info->cam_vreg,
					     &power_info->num_vreg);
	if (rc < 0)
		return rc;

	if (e_ctrl->userspace_probe == 0) {
		rc = msm_camera_get_dt_power_setting_data(of_node,
			power_info->cam_vreg, power_info->num_vreg,
			power_info);
		if (rc < 0)
			goto ERROR1;
	}

	power_info->gpio_conf = kzalloc(sizeof(struct msm_camera_gpio_conf),
					GFP_KERNEL);
	if (!power_info->gpio_conf) {
		rc = -ENOMEM;
		goto ERROR2;
	}
	gconf = power_info->gpio_conf;
	gpio_array_size = of_gpio_count(of_node);
	CDBG("%s gpio count %d\n", __func__, gpio_array_size);

	if (gpio_array_size > 0) {
		gpio_array = kcalloc(gpio_array_size, sizeof(uint16_t),
			GFP_KERNEL);
		if (!gpio_array)
			goto ERROR3;
		for (i = 0; i < gpio_array_size; i++) {
			gpio_array[i] = of_get_gpio(of_node, i);
			CDBG("%s gpio_array[%d] = %d\n", __func__, i,
				gpio_array[i]);
		}

		rc = msm_camera_get_dt_gpio_req_tbl(of_node, gconf,
			gpio_array, gpio_array_size);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto ERROR4;
		}

		rc = msm_camera_init_gpio_pin_tbl(of_node, gconf,
			gpio_array, gpio_array_size);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto ERROR4;
		}
		kfree(gpio_array);
	}

	return rc;
ERROR4:
	kfree(gpio_array);
ERROR3:
	kfree(power_info->gpio_conf);
ERROR2:
	kfree(power_info->cam_vreg);
ERROR1:
	kfree(power_info->power_setting);
	return rc;
}

static int msm_eeprom_i2c_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int rc = 0;
	uint32_t temp = 0;
	struct msm_eeprom_ctrl_t *e_ctrl = NULL;
	struct msm_eeprom_board_info *eb_info = NULL;
	struct device_node *of_node = client->dev.of_node;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	CDBG("%s E\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s i2c_check_functionality failed\n", __func__);
		goto probe_failure;
	}

	e_ctrl = kzalloc(sizeof(*e_ctrl), GFP_KERNEL);
	if (!e_ctrl)
		return -ENOMEM;

	e_ctrl->eeprom_v4l2_subdev_ops = &msm_eeprom_subdev_ops;
	e_ctrl->eeprom_mutex = &msm_eeprom_mutex;

	e_ctrl->cal_data.mapdata = NULL;
	e_ctrl->cal_data.map = NULL;
	e_ctrl->userspace_probe = 0;
	e_ctrl->is_supported = 0;
	if (!of_node) {
		pr_err("%s dev.of_node NULL\n", __func__);
		rc = -EINVAL;
		goto ectrl_free;
	}
	/* Set device type as I2C */
	e_ctrl->eeprom_device_type = MSM_CAMERA_I2C_DEVICE;
	e_ctrl->i2c_client.i2c_func_tbl = &msm_eeprom_qup_func_tbl;

	e_ctrl->eboard_info = kzalloc(sizeof(
		struct msm_eeprom_board_info), GFP_KERNEL);
	if (!e_ctrl->eboard_info) {
		rc = -ENOMEM;
		goto ectrl_free;
	}
	eb_info = e_ctrl->eboard_info;
	power_info = &eb_info->power_info;
	e_ctrl->i2c_client.client = client;
	power_info->dev = &client->dev;

	/*Get clocks information*/
	rc = msm_camera_i2c_dev_get_clk_info(
		&e_ctrl->i2c_client.client->dev,
		&power_info->clk_info,
		&power_info->clk_ptr,
		&power_info->clk_info_size);
	if (rc < 0) {
		pr_err("failed: msm_camera_get_clk_info rc %d", rc);
		goto board_free;
	}

	rc = of_property_read_u32(of_node, "cell-index",
		&e_ctrl->subdev_id);
	CDBG("cell-index %d, rc %d\n", e_ctrl->subdev_id, rc);
	if (rc < 0) {
		pr_err("failed rc %d\n", rc);
		goto board_free;
	}

	rc = of_property_read_string(of_node, "qcom,eeprom-name",
		&eb_info->eeprom_name);
	CDBG("%s qcom,eeprom-name %s, rc %d\n", __func__,
		eb_info->eeprom_name, rc);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		e_ctrl->userspace_probe = 1;
	}

	rc = msm_eeprom_get_dt_data(e_ctrl);
	if (rc < 0)
		goto board_free;

	if (e_ctrl->userspace_probe == 0) {
		rc = of_property_read_u32(of_node, "qcom,slave-addr",
			&temp);
		if (rc < 0) {
			pr_err("%s failed rc %d\n", __func__, rc);
			goto board_free;
		}

		rc = of_property_read_u32(of_node, "qcom,i2c-freq-mode",
			&e_ctrl->i2c_freq_mode);
		CDBG("qcom,i2c_freq_mode %d, rc %d\n",
			e_ctrl->i2c_freq_mode, rc);
		if (rc < 0) {
			pr_err("%s qcom,i2c-freq-mode read fail. Setting to 0 %d\n",
				__func__, rc);
			e_ctrl->i2c_freq_mode = 0;
		}
		if (e_ctrl->i2c_freq_mode >= I2C_MAX_MODES) {
			pr_err("%s:%d invalid i2c_freq_mode = %d\n",
				__func__, __LINE__, e_ctrl->i2c_freq_mode);
			e_ctrl->i2c_freq_mode = 0;
		}
		eb_info->i2c_slaveaddr = temp;
		CDBG("qcom,slave-addr = 0x%X\n", eb_info->i2c_slaveaddr);
		eb_info->i2c_freq_mode = e_ctrl->i2c_freq_mode;

		rc = msm_eeprom_parse_memory_map(of_node, &e_ctrl->cal_data);
		if (rc < 0)
			goto board_free;

		rc = msm_camera_power_up(power_info, e_ctrl->eeprom_device_type,
			&e_ctrl->i2c_client);
		if (rc) {
			pr_err("failed rc %d\n", rc);
			goto memdata_free;
		}

		rc = read_eeprom_memory(e_ctrl, &e_ctrl->cal_data);
		if (rc < 0) {
			pr_err("%s read_eeprom_memory failed\n", __func__);
			goto power_down;
		}
		CDBG("%s cal_data: %*ph\n", __func__,
			e_ctrl->cal_data.num_data, e_ctrl->cal_data.mapdata);

		e_ctrl->is_supported |= msm_eeprom_match_crc(&e_ctrl->cal_data);

		rc = msm_camera_power_down(power_info,
			e_ctrl->eeprom_device_type, &e_ctrl->i2c_client);
		if (rc) {
			pr_err("failed rc %d\n", rc);
			goto memdata_free;
		}
	} else
		e_ctrl->is_supported = 1;

	/* Initialize sub device */
	v4l2_i2c_subdev_init(&e_ctrl->msm_sd.sd,
		e_ctrl->i2c_client.client,
		e_ctrl->eeprom_v4l2_subdev_ops);
	v4l2_set_subdevdata(&e_ctrl->msm_sd.sd, e_ctrl);
	e_ctrl->msm_sd.sd.internal_ops = &msm_eeprom_internal_ops;
	e_ctrl->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(e_ctrl->msm_sd.sd.name,
		ARRAY_SIZE(e_ctrl->msm_sd.sd.name), "msm_eeprom");
	media_entity_pads_init(&e_ctrl->msm_sd.sd.entity, 0, NULL);
	e_ctrl->msm_sd.sd.entity.function = MSM_CAMERA_SUBDEV_EEPROM;
	msm_sd_register(&e_ctrl->msm_sd);
	e_ctrl->is_supported = (e_ctrl->is_supported << 1) | 1;
	pr_err("%s success result=%d X\n", __func__, rc);
	return rc;

power_down:
	msm_camera_power_down(power_info, e_ctrl->eeprom_device_type,
		&e_ctrl->i2c_client);
memdata_free:
	kfree(e_ctrl->cal_data.mapdata);
	kfree(e_ctrl->cal_data.map);
board_free:
	kfree(e_ctrl->eboard_info);
ectrl_free:
	kfree(e_ctrl);
probe_failure:
	pr_err("%s failed! rc = %d\n", __func__, rc);
	return rc;
}

static int msm_eeprom_i2c_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct msm_eeprom_ctrl_t  *e_ctrl;

	if (!sd) {
		pr_err("%s: Subdevice is NULL\n", __func__);
		return 0;
	}

	e_ctrl = (struct msm_eeprom_ctrl_t *)v4l2_get_subdevdata(sd);
	if (!e_ctrl) {
		pr_err("%s: eeprom device is NULL\n", __func__);
		return 0;
	}

	if (!e_ctrl->eboard_info) {
		pr_err("%s: eboard_info is NULL\n", __func__);
		return 0;
	}

	msm_camera_i2c_dev_put_clk_info(&e_ctrl->i2c_client.client->dev,
		&e_ctrl->eboard_info->power_info.clk_info,
		&e_ctrl->eboard_info->power_info.clk_ptr,
		e_ctrl->eboard_info->power_info.clk_info_size);

	kfree(e_ctrl->cal_data.mapdata);
	kfree(e_ctrl->cal_data.map);
	if (e_ctrl->eboard_info) {
		kfree(e_ctrl->eboard_info->power_info.gpio_conf);
		kfree(e_ctrl->eboard_info);
	}
	e_ctrl->cal_data.mapdata = NULL;
	kfree(e_ctrl);
	e_ctrl = NULL;

	return 0;
}

static int msm_eeprom_spi_parse_of(struct msm_camera_spi_client *spic)
{
	int rc = -EFAULT;
	uint32_t tmp[3];

	if (of_property_read_u32_array(
		spic->spi_master->dev.of_node,
		"qcom,spiop,read", tmp, 3)) {
		return -EFAULT;
	}
	spic->cmd_tbl.read.opcode = tmp[0];
	spic->cmd_tbl.read.addr_len = tmp[1];
	spic->cmd_tbl.read.dummy_len = tmp[2];

	if (of_property_read_u32_array(
		spic->spi_master->dev.of_node,
		"qcom,spiop,readseq", tmp, 3)) {
		return -EFAULT;
	}
	spic->cmd_tbl.read_seq.opcode = tmp[0];
	spic->cmd_tbl.read_seq.addr_len = tmp[1];
	spic->cmd_tbl.read_seq.dummy_len = tmp[2];

	if (of_property_read_u32_array(
		spic->spi_master->dev.of_node,
		"qcom,spiop,queryid", tmp, 3)) {
		return -EFAULT;
	}
	spic->cmd_tbl.query_id.opcode = tmp[0];
	spic->cmd_tbl.query_id.addr_len = tmp[1];
	spic->cmd_tbl.query_id.dummy_len = tmp[2];

	rc = of_property_read_u32_array(spic->spi_master->dev.of_node,
					"qcom,eeprom-id", tmp, 2);
	if (rc) {
		pr_err("%s: Failed to get eeprom id\n", __func__);
		return rc;
	}
	spic->mfr_id0 = tmp[0];
	spic->device_id0 = tmp[1];

	return 0;
}

static int msm_eeprom_match_id(struct msm_eeprom_ctrl_t *e_ctrl)
{
	int rc;
	struct msm_camera_i2c_client *client = &e_ctrl->i2c_client;
	uint8_t id[2];

	rc = msm_camera_spi_query_id(client, 0, &id[0], 2);
	if (rc < 0)
		return rc;
	CDBG("%s: read 0x%x 0x%x, check 0x%x 0x%x\n", __func__, id[0],
		id[1], client->spi_client->mfr_id0,
		client->spi_client->device_id0);
	if (id[0] != client->spi_client->mfr_id0
		|| id[1] != client->spi_client->device_id0)
		return -ENODEV;

	return 0;
}

static int msm_eeprom_cmm_dts(struct msm_eeprom_board_info *eb_info,
	struct device_node *of_node)
{
	int rc = 0;
	struct msm_eeprom_cmm_t *cmm_data = &eb_info->cmm_data;

	cmm_data->cmm_support =
		of_property_read_bool(of_node, "qcom,cmm-data-support");
	if (!cmm_data->cmm_support)
		return -EINVAL;
	cmm_data->cmm_compression =
		of_property_read_bool(of_node, "qcom,cmm-data-compressed");
	if (!cmm_data->cmm_compression)
		CDBG("No MM compression data\n");

	rc = of_property_read_u32(of_node, "qcom,cmm-data-offset",
		&cmm_data->cmm_offset);
	if (rc < 0)
		CDBG("No MM offset data\n");

	rc = of_property_read_u32(of_node, "qcom,cmm-data-size",
		&cmm_data->cmm_size);
	if (rc < 0)
		CDBG("No MM size data\n");

	CDBG("cmm_support: cmm_compr %d, cmm_offset %d, cmm_size %d\n",
		cmm_data->cmm_compression,
		cmm_data->cmm_offset,
		cmm_data->cmm_size);
	return 0;
}

static int msm_eeprom_spi_setup(struct spi_device *spi)
{
	struct msm_eeprom_ctrl_t *e_ctrl = NULL;
	struct msm_camera_i2c_client *client = NULL;
	struct msm_camera_spi_client *spi_client;
	struct msm_eeprom_board_info *eb_info;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	int rc = 0;

	e_ctrl = kzalloc(sizeof(*e_ctrl), GFP_KERNEL);
	if (!e_ctrl)
		return -ENOMEM;
	e_ctrl->eeprom_v4l2_subdev_ops = &msm_eeprom_subdev_ops;
	e_ctrl->eeprom_mutex = &msm_eeprom_mutex;
	client = &e_ctrl->i2c_client;
	e_ctrl->is_supported = 0;
	e_ctrl->userspace_probe = 0;
	e_ctrl->cal_data.mapdata = NULL;
	e_ctrl->cal_data.map = NULL;

	spi_client = kzalloc(sizeof(*spi_client), GFP_KERNEL);
	if (!spi_client) {
		kfree(e_ctrl);
		return -ENOMEM;
	}

	rc = of_property_read_u32(spi->dev.of_node, "cell-index",
				  &e_ctrl->subdev_id);
	CDBG("cell-index %d, rc %d\n", e_ctrl->subdev_id, rc);
	if (rc < 0) {
		pr_err("failed rc %d\n", rc);
		return rc;
	}

	eb_info = kzalloc(sizeof(*eb_info), GFP_KERNEL);
	if (!eb_info)
		goto spi_free;
	e_ctrl->eboard_info = eb_info;

	rc = of_property_read_string(spi->dev.of_node, "qcom,eeprom-name",
		&eb_info->eeprom_name);
	CDBG("%s qcom,eeprom-name %s, rc %d\n", __func__,
		eb_info->eeprom_name, rc);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		e_ctrl->userspace_probe = 1;
		goto board_free;
	}

	e_ctrl->eeprom_device_type = MSM_CAMERA_SPI_DEVICE;
	client->spi_client = spi_client;
	spi_client->spi_master = spi;
	client->i2c_func_tbl = &msm_eeprom_spi_func_tbl;
	client->addr_type = MSM_CAMERA_I2C_3B_ADDR;

	rc = msm_eeprom_cmm_dts(e_ctrl->eboard_info, spi->dev.of_node);
	if (rc < 0)
		CDBG("%s MM data miss:%d\n", __func__, __LINE__);

	power_info = &eb_info->power_info;
	power_info->dev = &spi->dev;

	/*Get clocks information*/
	rc = msm_camera_i2c_dev_get_clk_info(
		&spi->dev,
		&power_info->clk_info,
		&power_info->clk_ptr,
		&power_info->clk_info_size);
	if (rc < 0) {
		pr_err("failed: msm_camera_get_clk_info rc %d", rc);
		goto board_free;
	}

	rc = msm_eeprom_get_dt_data(e_ctrl);
	if (rc < 0)
		goto board_free;

	/* set spi instruction info */
	spi_client->retry_delay = 1;
	spi_client->retries = 0;

	rc = msm_eeprom_spi_parse_of(spi_client);
	if (rc < 0) {
		dev_err(&spi->dev,
			"%s: Error parsing device properties\n", __func__);
		goto board_free;
	}

	if (e_ctrl->userspace_probe == 0) {
		/* prepare memory buffer */
		rc = msm_eeprom_parse_memory_map(spi->dev.of_node,
			&e_ctrl->cal_data);
		if (rc < 0)
			CDBG("%s: no cal memory map\n", __func__);

		/* power up eeprom for reading */
		rc = msm_camera_power_up(power_info, e_ctrl->eeprom_device_type,
			&e_ctrl->i2c_client);
		if (rc < 0) {
			pr_err("failed rc %d\n", rc);
			goto caldata_free;
		}

		/* check eeprom id */
		rc = msm_eeprom_match_id(e_ctrl);
		if (rc < 0) {
			CDBG("%s: eeprom not matching %d\n", __func__, rc);
			goto power_down;
		}
		/* read eeprom */
		if (e_ctrl->cal_data.map) {
			rc = read_eeprom_memory(e_ctrl, &e_ctrl->cal_data);
			if (rc < 0) {
				pr_err("%s: read cal data failed\n", __func__);
				goto power_down;
			}
			e_ctrl->is_supported |= msm_eeprom_match_crc(
				&e_ctrl->cal_data);
		}

		rc = msm_camera_power_down(power_info,
			e_ctrl->eeprom_device_type, &e_ctrl->i2c_client);
		if (rc < 0) {
			pr_err("failed rc %d\n", rc);
			goto caldata_free;
		}
	} else
		e_ctrl->is_supported = 1;

	/* initiazlie subdev */
	v4l2_spi_subdev_init(&e_ctrl->msm_sd.sd,
		e_ctrl->i2c_client.spi_client->spi_master,
		e_ctrl->eeprom_v4l2_subdev_ops);
	v4l2_set_subdevdata(&e_ctrl->msm_sd.sd, e_ctrl);
	e_ctrl->msm_sd.sd.internal_ops = &msm_eeprom_internal_ops;
	e_ctrl->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	media_entity_pads_init(&e_ctrl->msm_sd.sd.entity, 0, NULL);
	e_ctrl->msm_sd.sd.entity.function = MSM_CAMERA_SUBDEV_EEPROM;
	msm_sd_register(&e_ctrl->msm_sd);
	e_ctrl->is_supported = (e_ctrl->is_supported << 1) | 1;
	CDBG("%s success result=%d supported=%x X\n", __func__, rc,
	     e_ctrl->is_supported);

	return 0;

power_down:
	msm_camera_power_down(power_info, e_ctrl->eeprom_device_type,
		&e_ctrl->i2c_client);
caldata_free:
	msm_camera_i2c_dev_put_clk_info(
		&e_ctrl->i2c_client.spi_client->spi_master->dev,
		&e_ctrl->eboard_info->power_info.clk_info,
		&e_ctrl->eboard_info->power_info.clk_ptr,
		e_ctrl->eboard_info->power_info.clk_info_size);
	kfree(e_ctrl->cal_data.mapdata);
	kfree(e_ctrl->cal_data.map);
board_free:
	kfree(e_ctrl->eboard_info);
spi_free:
	kfree(spi_client);
	kfree(e_ctrl);
	return rc;
}

static int msm_eeprom_spi_probe(struct spi_device *spi)
{
	int irq, cs, cpha, cpol, cs_high;

	CDBG("%s\n", __func__);
	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;
	spi_setup(spi);

	irq = spi->irq;
	cs = spi->chip_select;
	cpha = (spi->mode & SPI_CPHA) ? 1 : 0;
	cpol = (spi->mode & SPI_CPOL) ? 1 : 0;
	cs_high = (spi->mode & SPI_CS_HIGH) ? 1 : 0;
	CDBG("%s: irq[%d] cs[%x] CPHA[%x] CPOL[%x] CS_HIGH[%x]\n",
			__func__, irq, cs, cpha, cpol, cs_high);
	CDBG("%s: max_speed[%u]\n", __func__, spi->max_speed_hz);

	return msm_eeprom_spi_setup(spi);
}

static int msm_eeprom_spi_remove(struct spi_device *sdev)
{
	struct v4l2_subdev *sd = spi_get_drvdata(sdev);
	struct msm_eeprom_ctrl_t  *e_ctrl;

	if (!sd) {
		pr_err("%s: Subdevice is NULL\n", __func__);
		return 0;
	}

	e_ctrl = (struct msm_eeprom_ctrl_t *)v4l2_get_subdevdata(sd);
	if (!e_ctrl) {
		pr_err("%s: eeprom device is NULL\n", __func__);
		return 0;
	}

	if (!e_ctrl->eboard_info) {
		pr_err("%s: board info is NULL\n", __func__);
		return 0;
	}

	msm_camera_i2c_dev_put_clk_info(
		&e_ctrl->i2c_client.spi_client->spi_master->dev,
		&e_ctrl->eboard_info->power_info.clk_info,
		&e_ctrl->eboard_info->power_info.clk_ptr,
		e_ctrl->eboard_info->power_info.clk_info_size);

	kfree(e_ctrl->i2c_client.spi_client);
	kfree(e_ctrl->cal_data.mapdata);
	kfree(e_ctrl->cal_data.map);
	if (e_ctrl->eboard_info) {
		kfree(e_ctrl->eboard_info->power_info.gpio_conf);
		kfree(e_ctrl->eboard_info);
	}
	e_ctrl->cal_data.mapdata = NULL;
	kfree(e_ctrl);
	e_ctrl = NULL;

	return 0;
}

#ifdef CONFIG_COMPAT
static void msm_eeprom_copy_power_settings_compat(
	struct msm_sensor_power_setting_array *ps,
	struct msm_sensor_power_setting_array32 *ps32)
{
	uint16_t i = 0;

	ps->size = ps32->size;
	for (i = 0; i < ps32->size; i++) {
		ps->power_setting_a[i].config_val =
			ps32->power_setting_a[i].config_val;
		ps->power_setting_a[i].delay =
			ps32->power_setting_a[i].delay;
		ps->power_setting_a[i].seq_type =
			ps32->power_setting_a[i].seq_type;
		ps->power_setting_a[i].seq_val =
			ps32->power_setting_a[i].seq_val;
	}

	ps->size_down = ps32->size_down;
	for (i = 0; i < ps32->size_down; i++) {
		ps->power_down_setting_a[i].config_val =
			ps32->power_down_setting_a[i].config_val;
		ps->power_down_setting_a[i].delay =
			ps32->power_down_setting_a[i].delay;
		ps->power_down_setting_a[i].seq_type =
			ps32->power_down_setting_a[i].seq_type;
		ps->power_down_setting_a[i].seq_val =
			ps32->power_down_setting_a[i].seq_val;
	}
}

static int eeprom_config_read_cal_data32(struct msm_eeprom_ctrl_t *e_ctrl,
	void *arg)
{
	int rc;
	uint8_t __user *ptr_dest = NULL;
	struct msm_eeprom_cfg_data32 *cdata32 =
		(struct msm_eeprom_cfg_data32 *) arg;
	struct msm_eeprom_cfg_data cdata;

	cdata.cfgtype = cdata32->cfgtype;
	cdata.is_supported = cdata32->is_supported;
	cdata.cfg.read_data.num_bytes = cdata32->cfg.read_data.num_bytes;
	/* check range */
	if (cdata.cfg.read_data.num_bytes >
	    e_ctrl->cal_data.num_data) {
		CDBG("%s: Invalid size. exp %u, req %u\n", __func__,
			e_ctrl->cal_data.num_data,
			cdata.cfg.read_data.num_bytes);
		return -EINVAL;
	}
	if (!e_ctrl->cal_data.mapdata)
		return -EFAULT;

	ptr_dest = (uint8_t __user *)compat_ptr(cdata32->cfg.read_data.dbuffer);

	rc = copy_to_user(ptr_dest, e_ctrl->cal_data.mapdata,
		cdata.cfg.read_data.num_bytes);

	return rc;
}

static int eeprom_init_config32(struct msm_eeprom_ctrl_t *e_ctrl,
	void *argp)
{
	int rc =  0;
	struct msm_eeprom_cfg_data32 *cdata32 = argp;
	struct msm_sensor_power_setting_array *power_setting_array = NULL;
	struct msm_sensor_power_setting_array32 *power_setting_array32 = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	struct msm_eeprom_memory_map_array *mem_map_array = NULL;

	power_setting_array32 =
		kzalloc(sizeof(struct msm_sensor_power_setting_array32),
			GFP_KERNEL);
	if (!power_setting_array32) {
		pr_err("%s:%d Mem Alloc Fail\n", __func__, __LINE__);
		rc = -ENOMEM;
		return rc;
	}
	power_setting_array =
		kzalloc(sizeof(struct msm_sensor_power_setting_array),
			GFP_KERNEL);
	if (power_setting_array ==  NULL) {
		pr_err("%s:%d Mem Alloc Fail\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto free_mem;
	}
	mem_map_array =
		kzalloc(sizeof(struct msm_eeprom_memory_map_array),
			GFP_KERNEL);
	if (mem_map_array == NULL) {
		pr_err("%s:%d Mem Alloc Fail\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto free_mem;
	}

	if (copy_from_user(power_setting_array32,
		(void __user *)compat_ptr(cdata32->cfg.eeprom_info.
		power_setting_array),
		sizeof(struct msm_sensor_power_setting_array32))) {
		pr_err("%s:%d copy_from_user failed\n",
			__func__, __LINE__);
		goto free_mem;
	}
	CDBG("%s:%d Size of power setting array: %d",
		__func__, __LINE__, power_setting_array32->size);
	if (copy_from_user(mem_map_array,
		(void __user *)
		compat_ptr(cdata32->cfg.eeprom_info.mem_map_array),
		sizeof(struct msm_eeprom_memory_map_array))) {
		pr_err("%s:%d copy_from_user failed for memory map\n",
			__func__, __LINE__);
		goto free_mem;
	}

	power_info = &(e_ctrl->eboard_info->power_info);

	if ((power_setting_array32->size > MAX_POWER_CONFIG) ||
		(power_setting_array32->size_down > MAX_POWER_CONFIG) ||
		(!power_setting_array32->size) ||
		(!power_setting_array32->size_down)) {
		pr_err("%s:%d invalid power setting size=%d size_down=%d\n",
			__func__, __LINE__, power_setting_array32->size,
			power_setting_array32->size_down);
		rc = -EINVAL;
		goto free_mem;
	}
	msm_eeprom_copy_power_settings_compat(
		power_setting_array,
		power_setting_array32);

	power_info->power_setting =
		power_setting_array->power_setting_a;
	power_info->power_down_setting =
		power_setting_array->power_down_setting_a;

	power_info->power_setting_size =
		power_setting_array->size;
	power_info->power_down_setting_size =
		power_setting_array->size_down;

	if (e_ctrl->i2c_client.cci_client) {
		e_ctrl->i2c_client.cci_client->i2c_freq_mode =
			cdata32->cfg.eeprom_info.i2c_freq_mode;
		if (e_ctrl->i2c_client.cci_client->i2c_freq_mode >
			I2C_MAX_MODES) {
			pr_err("%s::%d Improper I2C Freq Mode\n",
				__func__, __LINE__);
			e_ctrl->i2c_client.cci_client->i2c_freq_mode =
				I2C_STANDARD_MODE;
		}
		CDBG("%s:%d Not CCI probe", __func__, __LINE__);
	}
	/* Fill vreg power info and power up here */
	rc = msm_eeprom_power_up(e_ctrl, power_info);
	if (rc < 0) {
		pr_err("%s:%d Power Up failed for eeprom\n",
			__func__, __LINE__);
		goto free_mem;
	}

	rc = eeprom_parse_memory_map(e_ctrl, mem_map_array);
	if (rc < 0) {
		pr_err("%s:%d memory map parse failed\n",
			__func__, __LINE__);
		goto free_mem;
	}

	rc = msm_camera_power_down(power_info,
		e_ctrl->eeprom_device_type, &e_ctrl->i2c_client);
	if (rc < 0)
		pr_err("%s:%d Power down failed rc %d\n",
			__func__, __LINE__, rc);

free_mem:
	kfree(power_setting_array32);
	kfree(power_setting_array);
	kfree(mem_map_array);
	power_setting_array32 = NULL;
	power_setting_array = NULL;
	mem_map_array = NULL;
	return rc;
}

static int msm_eeprom_config32(struct msm_eeprom_ctrl_t *e_ctrl,
	void *argp)
{
	struct msm_eeprom_cfg_data32 *cdata =
		(struct msm_eeprom_cfg_data32 *)argp;
	int rc = 0;
	size_t length = 0;

	CDBG("%s E\n", __func__);
	switch (cdata->cfgtype) {
	case CFG_EEPROM_GET_INFO:
		if (e_ctrl->userspace_probe == 1) {
			pr_err("%s:%d Eeprom name should be module driver",
				__func__, __LINE__);
			rc = -EINVAL;
			break;
		}
		CDBG("%s E CFG_EEPROM_GET_INFO\n", __func__);
		cdata->is_supported = e_ctrl->is_supported;
		length = strlen(e_ctrl->eboard_info->eeprom_name) + 1;
		if (length > MAX_EEPROM_NAME) {
			pr_err("%s:%d invalid eeprom_name length %d\n",
				__func__, __LINE__, (int)length);
			rc = -EINVAL;
			break;
		}
		memcpy(cdata->cfg.eeprom_name,
			e_ctrl->eboard_info->eeprom_name, length);
		break;
	case CFG_EEPROM_GET_CAL_DATA:
		CDBG("%s E CFG_EEPROM_GET_CAL_DATA\n", __func__);
		cdata->cfg.get_data.num_bytes =
			e_ctrl->cal_data.num_data;
		break;
	case CFG_EEPROM_READ_CAL_DATA:
		CDBG("%s E CFG_EEPROM_READ_CAL_DATA\n", __func__);
		rc = eeprom_config_read_cal_data32(e_ctrl, argp);
		break;
	case CFG_EEPROM_INIT:
		if (e_ctrl->userspace_probe == 0) {
			pr_err("%s:%d Eeprom already probed at kernel boot",
				__func__, __LINE__);
			rc = -EINVAL;
			break;
		}
		if (e_ctrl->cal_data.num_data == 0) {
			rc = eeprom_init_config32(e_ctrl, argp);
			if (rc < 0)
				pr_err("%s:%d Eeprom init failed\n",
					__func__, __LINE__);
		} else {
			CDBG("%s:%d Already read eeprom\n",
				__func__, __LINE__);
		}
		break;
	default:
		break;
	}

	CDBG("%s X rc: %d\n", __func__, rc);
	return rc;
}

static long msm_eeprom_subdev_ioctl32(struct v4l2_subdev *sd,
		unsigned int cmd, void *arg)
{
	struct msm_eeprom_ctrl_t *e_ctrl = v4l2_get_subdevdata(sd);
	void *argp = (void *)arg;

	CDBG("%s E\n", __func__);
	CDBG("%s:%d a_ctrl %pK argp %pK\n", __func__, __LINE__, e_ctrl, argp);
	switch (cmd) {
	case VIDIOC_MSM_SENSOR_GET_SUBDEV_ID:
		return msm_eeprom_get_subdev_id(e_ctrl, argp);
	case VIDIOC_MSM_EEPROM_CFG32:
		return msm_eeprom_config32(e_ctrl, argp);
	default:
		return -ENOIOCTLCMD;
	}

	CDBG("%s X\n", __func__);
}

static long msm_eeprom_subdev_do_ioctl32(
	struct file *file, unsigned int cmd, void *arg)
{
	struct video_device *vdev = video_devdata(file);
	struct v4l2_subdev *sd = vdev_to_v4l2_subdev(vdev);

	return msm_eeprom_subdev_ioctl32(sd, cmd, arg);
}

static long msm_eeprom_subdev_fops_ioctl32(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	return video_usercopy(file, cmd, arg, msm_eeprom_subdev_do_ioctl32);
}

#endif

static int msm_eeprom_platform_probe(struct platform_device *pdev)
{
	int rc = 0;
	int j = 0;
	uint32_t temp;

	struct msm_camera_cci_client *cci_client = NULL;
	struct msm_eeprom_ctrl_t *e_ctrl = NULL;
	struct msm_eeprom_board_info *eb_info = NULL;
	struct device_node *of_node = pdev->dev.of_node;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	CDBG("%s E\n", __func__);

	e_ctrl = kzalloc(sizeof(*e_ctrl), GFP_KERNEL);
	if (!e_ctrl)
		return -ENOMEM;
	e_ctrl->eeprom_v4l2_subdev_ops = &msm_eeprom_subdev_ops;
	e_ctrl->eeprom_mutex = &msm_eeprom_mutex;

	e_ctrl->cal_data.mapdata = NULL;
	e_ctrl->cal_data.map = NULL;
	e_ctrl->userspace_probe = 0;
	e_ctrl->is_supported = 0;
	if (!of_node) {
		pr_err("%s dev.of_node NULL\n", __func__);
		rc = -EINVAL;
		goto ectrl_free;
	}

	/* Set platform device handle */
	e_ctrl->pdev = pdev;
	/* Set device type as platform device */
	e_ctrl->eeprom_device_type = MSM_CAMERA_PLATFORM_DEVICE;
	e_ctrl->i2c_client.i2c_func_tbl = &msm_eeprom_cci_func_tbl;
	e_ctrl->i2c_client.cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!e_ctrl->i2c_client.cci_client) {
		rc = -ENOMEM;
		goto ectrl_free;
	}

	e_ctrl->eboard_info = kzalloc(sizeof(
		struct msm_eeprom_board_info), GFP_KERNEL);
	if (!e_ctrl->eboard_info) {
		rc = -ENOMEM;
		goto cciclient_free;
	}

	eb_info = e_ctrl->eboard_info;
	power_info = &eb_info->power_info;
	cci_client = e_ctrl->i2c_client.cci_client;
	cci_client->cci_subdev = msm_cci_get_subdev();
	cci_client->retries = 3;
	cci_client->id_map = 0;
	power_info->dev = &pdev->dev;

	/*Get clocks information*/
	rc = msm_camera_get_clk_info(e_ctrl->pdev,
		&power_info->clk_info,
		&power_info->clk_ptr,
		&power_info->clk_info_size);
	if (rc < 0) {
		pr_err("failed: msm_camera_get_clk_info rc %d", rc);
		goto board_free;
	}

	rc = of_property_read_u32(of_node, "cell-index",
		&pdev->id);
	CDBG("cell-index %d, rc %d\n", pdev->id, rc);
	if (rc < 0) {
		pr_err("failed rc %d\n", rc);
		goto board_free;
	}
	e_ctrl->subdev_id = pdev->id;

	rc = of_property_read_u32(of_node, "qcom,cci-master",
		&e_ctrl->cci_master);
	CDBG("qcom,cci-master %d, rc %d\n", e_ctrl->cci_master, rc);
	if (rc < 0) {
		pr_err("%s failed rc %d\n", __func__, rc);
		goto board_free;
	}
	cci_client->cci_i2c_master = e_ctrl->cci_master;

	rc = of_property_read_string(of_node, "qcom,eeprom-name",
		&eb_info->eeprom_name);
	CDBG("%s qcom,eeprom-name %s, rc %d\n", __func__,
		eb_info->eeprom_name, rc);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		e_ctrl->userspace_probe = 1;
	}

	rc = msm_eeprom_get_dt_data(e_ctrl);
	if (rc < 0)
		goto board_free;

	if (e_ctrl->userspace_probe == 0) {
		rc = of_property_read_u32(of_node, "qcom,slave-addr",
			&temp);
		if (rc < 0) {
			pr_err("%s failed rc %d\n", __func__, rc);
			goto board_free;
		}

		rc = of_property_read_u32(of_node, "qcom,i2c-freq-mode",
			&e_ctrl->i2c_freq_mode);
		CDBG("qcom,i2c_freq_mode %d, rc %d\n",
			e_ctrl->i2c_freq_mode, rc);
		if (rc < 0) {
			pr_err("%s qcom,i2c-freq-mode read fail. Setting to 0 %d\n",
				__func__, rc);
			e_ctrl->i2c_freq_mode = 0;
		}
		if (e_ctrl->i2c_freq_mode >= I2C_MAX_MODES) {
			pr_err("%s:%d invalid i2c_freq_mode = %d\n",
				__func__, __LINE__, e_ctrl->i2c_freq_mode);
			e_ctrl->i2c_freq_mode = 0;
		}
		eb_info->i2c_slaveaddr = temp;
		CDBG("qcom,slave-addr = 0x%X\n", eb_info->i2c_slaveaddr);
		eb_info->i2c_freq_mode = e_ctrl->i2c_freq_mode;
		cci_client->i2c_freq_mode = e_ctrl->i2c_freq_mode;
		cci_client->sid = eb_info->i2c_slaveaddr >> 1;

		rc = msm_eeprom_parse_memory_map(of_node, &e_ctrl->cal_data);
		if (rc < 0)
			goto board_free;

		rc = msm_camera_power_up(power_info, e_ctrl->eeprom_device_type,
			&e_ctrl->i2c_client);
		if (rc) {
			pr_err("failed rc %d\n", rc);
			goto memdata_free;
		}
		rc = read_eeprom_memory(e_ctrl, &e_ctrl->cal_data);
		if (rc < 0) {
			pr_err("%s read_eeprom_memory failed\n", __func__);
			goto power_down;
		}
		for (j = 0; j < e_ctrl->cal_data.num_data; j++)
			CDBG("memory_data[%d] = 0x%X\n", j,
				e_ctrl->cal_data.mapdata[j]);

		e_ctrl->is_supported |= msm_eeprom_match_crc(&e_ctrl->cal_data);

		rc = msm_camera_power_down(power_info,
			e_ctrl->eeprom_device_type, &e_ctrl->i2c_client);
		if (rc) {
			pr_err("failed rc %d\n", rc);
			goto memdata_free;
		}
	} else
		e_ctrl->is_supported = 1;

	v4l2_subdev_init(&e_ctrl->msm_sd.sd,
		e_ctrl->eeprom_v4l2_subdev_ops);
	v4l2_set_subdevdata(&e_ctrl->msm_sd.sd, e_ctrl);
	platform_set_drvdata(pdev, &e_ctrl->msm_sd.sd);
	e_ctrl->msm_sd.sd.internal_ops = &msm_eeprom_internal_ops;
	e_ctrl->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(e_ctrl->msm_sd.sd.name,
		ARRAY_SIZE(e_ctrl->msm_sd.sd.name), "msm_eeprom");
	media_entity_pads_init(&e_ctrl->msm_sd.sd.entity, 0, NULL);
	e_ctrl->msm_sd.sd.entity.function = MSM_CAMERA_SUBDEV_EEPROM;
	msm_sd_register(&e_ctrl->msm_sd);

#ifdef CONFIG_COMPAT
	msm_cam_copy_v4l2_subdev_fops(&msm_eeprom_v4l2_subdev_fops);
	msm_eeprom_v4l2_subdev_fops.compat_ioctl32 =
		msm_eeprom_subdev_fops_ioctl32;
	e_ctrl->msm_sd.sd.devnode->fops = &msm_eeprom_v4l2_subdev_fops;
#endif

	e_ctrl->is_supported = (e_ctrl->is_supported << 1) | 1;
	CDBG("%s X\n", __func__);
	return rc;

power_down:
	msm_camera_power_down(power_info, e_ctrl->eeprom_device_type,
		&e_ctrl->i2c_client);
memdata_free:
	kfree(e_ctrl->cal_data.mapdata);
	kfree(e_ctrl->cal_data.map);
board_free:
	kfree(e_ctrl->eboard_info);
cciclient_free:
	kfree(e_ctrl->i2c_client.cci_client);
ectrl_free:
	kfree(e_ctrl);
	return rc;
}

static int msm_eeprom_platform_remove(struct platform_device *pdev)
{
	struct v4l2_subdev *sd = platform_get_drvdata(pdev);
	struct msm_eeprom_ctrl_t  *e_ctrl;

	if (!sd) {
		pr_err("%s: Subdevice is NULL\n", __func__);
		return 0;
	}

	e_ctrl = (struct msm_eeprom_ctrl_t *)v4l2_get_subdevdata(sd);
	if (!e_ctrl) {
		pr_err("%s: eeprom device is NULL\n", __func__);
		return 0;
	}

	if (!e_ctrl->eboard_info) {
		pr_err("%s: board info is NULL\n", __func__);
		return 0;
	}

	msm_camera_put_clk_info(e_ctrl->pdev,
		&e_ctrl->eboard_info->power_info.clk_info,
		&e_ctrl->eboard_info->power_info.clk_ptr,
		e_ctrl->eboard_info->power_info.clk_info_size);

	kfree(e_ctrl->i2c_client.cci_client);
	kfree(e_ctrl->cal_data.mapdata);
	kfree(e_ctrl->cal_data.map);
	if (e_ctrl->eboard_info) {
		kfree(e_ctrl->eboard_info->power_info.gpio_conf);
		kfree(e_ctrl->eboard_info);
	}
	kfree(e_ctrl);
	return 0;
}

static const struct of_device_id msm_eeprom_dt_match[] = {
	{ .compatible = "qcom,eeprom" },
	{ }
};

MODULE_DEVICE_TABLE(of, msm_eeprom_dt_match);

static struct platform_driver msm_eeprom_platform_driver = {
	.driver = {
		.name = "qcom,eeprom",
		.owner = THIS_MODULE,
		.of_match_table = msm_eeprom_dt_match,
	},
	.probe = msm_eeprom_platform_probe,
	.remove = msm_eeprom_platform_remove,
};

static const struct i2c_device_id msm_eeprom_i2c_id[] = {
	{ "msm_eeprom", (kernel_ulong_t)NULL},
	{ }
};

static struct i2c_driver msm_eeprom_i2c_driver = {
	.id_table = msm_eeprom_i2c_id,
	.probe  = msm_eeprom_i2c_probe,
	.remove = msm_eeprom_i2c_remove,
	.driver = {
		/*
		 * Camera OTP EEPROMs (gc5025_otp / hi556_otp) hang off a BLSP
		 * I2C bus (i2c_3) on this board, so they are probed as DT i2c
		 * clients.  Such clients get client->name set to the compatible
		 * with the vendor prefix stripped ("eeprom"), which never matches
		 * the id_table name "msm_eeprom" -> the EEPROM never bound and the
		 * rear sensor found no OTP subdev.  Match by OF compatible like
		 * the 3.18 stock driver (and the platform/spi drivers below) do.
		 */
		.name = "qcom,eeprom",
		.of_match_table = msm_eeprom_dt_match,
	},
};

static struct spi_driver msm_eeprom_spi_driver = {
	.driver = {
		.name = "qcom_eeprom",
		.owner = THIS_MODULE,
		.of_match_table = msm_eeprom_dt_match,
	},
	.probe = msm_eeprom_spi_probe,
	.remove = msm_eeprom_spi_remove,
};

static int __init msm_eeprom_init_module(void)
{
	int rc = 0;

	CDBG("%s E\n", __func__);
	rc = platform_driver_register(&msm_eeprom_platform_driver);
	CDBG("%s:%d platform rc %d\n", __func__, __LINE__, rc);
	rc = spi_register_driver(&msm_eeprom_spi_driver);
	CDBG("%s:%d spi rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&msm_eeprom_i2c_driver);
}

static void __exit msm_eeprom_exit_module(void)
{
	platform_driver_unregister(&msm_eeprom_platform_driver);
	spi_unregister_driver(&msm_eeprom_spi_driver);
	i2c_del_driver(&msm_eeprom_i2c_driver);
}

module_init(msm_eeprom_init_module);
module_exit(msm_eeprom_exit_module);
MODULE_DESCRIPTION("MSM EEPROM driver");
MODULE_LICENSE("GPL v2");
