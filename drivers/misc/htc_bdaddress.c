/*
 * drivers/misc/htc_bdaddress.c
 *
 * Original Copyright (C) 2010 HTC Corporation
 * Author: Yomin Lin <yomin_lin@htc.com>
 * Author: Allen Ou <allen_ou@htc.com>
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/of.h>

#define MAX_BT_SIZE            0x8U
#define CALIBRATION_DATA_PATH  "/calibration_data"
#define BT_FLASH_DATA          "bt_flash"

static unsigned char bt_bd_ram[MAX_BT_SIZE];
static char bdaddress[20];

static unsigned char *get_bt_bd_ram(void)
{
	struct device_node *offset = of_find_node_by_path(CALIBRATION_DATA_PATH);
	const unsigned char *p_data = NULL;
	int p_size = 0;

	if (offset) {
		p_data = of_get_property(offset, BT_FLASH_DATA, &p_size);
		of_node_put(offset);
	}

	if (p_data != NULL && p_size > 0) {
		if (p_size > MAX_BT_SIZE)
			p_size = MAX_BT_SIZE;
		memcpy(bt_bd_ram, p_data, p_size);
	}

	return bt_bd_ram;
}

void bt_export_bd_address(void)
{
	unsigned char cTemp[6];

	memcpy(cTemp, get_bt_bd_ram(), 6);
	snprintf(bdaddress, sizeof(bdaddress),
		 "%02x:%02x:%02x:%02x:%02x:%02x",
		 cTemp[0], cTemp[1], cTemp[2],
		 cTemp[3], cTemp[4], cTemp[5]);

	pr_info("htc_bdaddress: BD_ADDRESS=%s\n", bdaddress);
}
EXPORT_SYMBOL(bt_export_bd_address);

static int __init htc_bdaddress_init(void)
{
	bt_export_bd_address();
	return 0;
}
late_initcall(htc_bdaddress_init);

module_param_string(bdaddress, bdaddress, sizeof(bdaddress),
		    S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(bdaddress, "BT MAC ADDRESS");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("HTC Bluetooth BD address exporter from DT /calibration_data");
