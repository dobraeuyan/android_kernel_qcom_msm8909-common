// SPDX-License-Identifier: GPL-2.0

#define pr_fmt(fmt) "htc_radio_smem: " fmt

#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/string.h>

#include "htc_radio_smem.h"

#ifndef RCMS_NAME
#define RCMS_NAME ""
#endif
#ifndef SKU_NAME
#define SKU_NAME ""
#endif

#define HTC_ROM_VERSION_PATH		"/chosen/misc"
#define HTC_ROM_VERSION_PROPERTY	"firmware_main_version"

static int boot_mode = APP_IN_HLOS;
static unsigned long radioflag;
static unsigned long radioflagex1;
static unsigned long radioflagex2;

static int __init cmdline_boot_mode_read(char *s)
{
	if (!strcmp(s, "ftm"))
		boot_mode = APP_IN_FTM;
	else if (!strcmp(s, "mfgkernel"))
		boot_mode = APP_IN_MFGKERNEL;
	else if (!strcmp(s, "mfgkernel:diag58"))
		boot_mode = APP_IN_DIAG;
	else if (!strcmp(s, "recovery") || !strcmp(s, "recovery_manual") ||
		 !strcmp(s, "offmode_charging") || !strcmp(s, "repartition"))
		boot_mode = APP_IN_RECOVERY;
	else if (!strcmp(s, "power_test"))
		boot_mode = APP_IN_TESTBOOTLOADER;
	else
		boot_mode = APP_IN_HLOS;

	return 1;
}
__setup("androidboot.mode=", cmdline_boot_mode_read);

static int __init cmdline_radioflag_read(char *s)
{
	if (kstrtoul(s, 16, &radioflag))
		radioflag = 0;
	return 1;
}
__setup("radioflag=", cmdline_radioflag_read);

static int __init cmdline_radioflagex1_read(char *s)
{
	if (kstrtoul(s, 16, &radioflagex1))
		radioflagex1 = 0;
	return 1;
}
__setup("radioflagex1=", cmdline_radioflagex1_read);

static int __init cmdline_radioflagex2_read(char *s)
{
	if (kstrtoul(s, 16, &radioflagex2))
		radioflagex2 = 0;
	return 1;
}
__setup("radioflagex2=", cmdline_radioflagex2_read);

static void htc_radio_smem_write(struct htc_smem_type __iomem *smem,
				 const char *rom_version)
{
	struct htc_smem_type buf;

	memset(&buf, 0, sizeof(buf));

	buf.version = HTC_RADIO_SMEM_VERSION;
	buf.struct_size = sizeof(struct htc_smem_type);
	buf.htc_smem_app_run_mode = boot_mode;
	buf.htc_smem_ce_radio_dbg_flag = (uint32_t)radioflag;
	buf.htc_smem_ce_radio_dbg_flag_ext1 = (uint32_t)radioflagex1;
	buf.htc_smem_ce_radio_dbg_flag_ext2 = (uint32_t)radioflagex2;

	buf.htc_smlog_magic = 0;
	buf.htc_smlog_base = 0;
	buf.htc_smlog_size = 0;

	buf.htc_smem_is_nv_backup = 0;
	strlcpy((char *)buf.RCMS_name, RCMS_NAME, sizeof(buf.RCMS_name));
	strlcpy((char *)buf.SKU_Name, SKU_NAME, sizeof(buf.SKU_Name));

	if (rom_version)
		strlcpy((char *)buf.htc_rom_ver, rom_version,
			sizeof(buf.htc_rom_ver));
	else
		pr_warn("no se encontró %s en %s\n",
			HTC_ROM_VERSION_PROPERTY, HTC_ROM_VERSION_PATH);

	memcpy_toio(smem, &buf, sizeof(buf));

	pr_info("app_run_mode=0x%x radioflag=0x%x/0x%x/0x%x rom_ver='%s'\n",
		buf.htc_smem_app_run_mode, buf.htc_smem_ce_radio_dbg_flag,
		buf.htc_smem_ce_radio_dbg_flag_ext1,
		buf.htc_smem_ce_radio_dbg_flag_ext2,
		(char *)buf.htc_rom_ver);
}

static int htc_radio_smem_probe(struct platform_device *pdev)
{
	struct htc_smem_type __iomem *smem;
	struct device_node *dnp;
	const char *rom_version = NULL;
	struct resource *res;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "smem-start-addr");
	if (!res) {
		dev_err(&pdev->dev, "falta reg-names = \"smem-start-addr\"\n");
		return -ENODEV;
	}

	smem = ioremap(res->start, sizeof(struct htc_smem_type));
	if (!smem) {
		dev_err(&pdev->dev, "ioremap de %pa falló\n", &res->start);
		return -ENOMEM;
	}

	dnp = of_find_node_by_path(HTC_ROM_VERSION_PATH);
	if (dnp) {
		rom_version = of_get_property(dnp, HTC_ROM_VERSION_PROPERTY,
					      NULL);
		of_node_put(dnp);
	}

	htc_radio_smem_write(smem, rom_version);

	iounmap(smem);
	return 0;
}

static const struct of_device_id htc_radio_smem_of_match[] = {
	{ .compatible = "htc,htc_radio_smem", },
	{ },
};
MODULE_DEVICE_TABLE(of, htc_radio_smem_of_match);

static struct platform_driver htc_radio_smem_driver = {
	.probe = htc_radio_smem_probe,
	.driver = {
		.name = "htc_radio_smem",
		.owner = THIS_MODULE,
		.of_match_table = htc_radio_smem_of_match,
	},
};

static int __init htc_radio_smem_init(void)
{
	return platform_driver_register(&htc_radio_smem_driver);
}

arch_initcall(htc_radio_smem_init);

MODULE_DESCRIPTION("HTC radio shared memory (AP run-mode handshake)");
MODULE_LICENSE("GPL v2");
