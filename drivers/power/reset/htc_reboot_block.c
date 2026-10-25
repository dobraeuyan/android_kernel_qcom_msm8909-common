/*
 * drivers/power/reset/htc_reboot_block.c
 *
 * Canal PERSISTENTE de "reiniciar a donde toca" de las placas HTC: en cada
 * SYS_RESTART escribe {u32 reason; char msg[64];} en una partición cruda que el
 * bootloader (LK HTC) lee y borra en el arranque siguiente.
 *
 * Original Copyright (C) 2015 HTC Corporation
 * (kernel 3.10: drivers/htc_debug/stability/htc_reboot_block.c)
 *
 * Portado del kernel HTC 3.10 (dobraeuyan/android_kernel_htc_a16) al 4.9 de
 * msm8909-common (LineageOS 17.1).
 *
 * POR QUÉ HACE FALTA
 * ------------------
 * El LK decide el modo de arranque con `check_reboot_mode()`: lee una cookie de
 * IMEM 0x8600065c y, si no lleva prefijo 0x776655xx / 0x6f656dxx, cae a ESTE
 * bloque. En el a16 el reset del PMIC es en FRÍO (medido en HW 20260724:
 * "Power-on reason: Triggered from Hard Reset and 'cold' boot"), así que IMEM
 * NO retiene y el bloque es el único canal que sobrevive — también a un corte
 * de alimentación. Mapa completo del mecanismo y de los valores que acepta el
 * LK en device/htc/a16/aboot-reboot-modes.md.
 *
 * CAMBIOS RESPECTO AL ORIGINAL
 * ----------------------------
 *  - No es un platform_driver: el nodo se localiza con
 *    `of_find_compatible_node()`, que recorre el árbol entero sin importar
 *    dónde lo haya colgado el bootloader. El original hacía
 *    `platform_driver_register()` con .of_match_table = "htc,reboot_block", y
 *    eso ata el driver a que el nodo cuelgue de un sitio del que
 *    `of_platform_populate()` cree platform devices — detalle frágil, porque
 *    los mensajes del propio LK hablan de añadirlo "under chosen" y `/chosen`
 *    NO genera platform devices (`of_platform_bus_create()` lo descarta con
 *    strict=true por no tener `compatible`, drivers/of/platform.c:364).
 *    ✅ Verificado en HW 20260725: este LK lo cuelga de la RAÍZ (`/reboot_block`,
 *    con compatible/pname/poffset), así que un platform_driver también habría
 *    bindado — pero la búsqueda por árbol es correcta en ambos casos.
 *  - Bug del original corregido: `strlcpy(&block.msg[0], msg, strlen(msg))`
 *    truncaba el último carácter (strlcpy quiere el TAMAÑO del destino, no la
 *    longitud del origen). Ahora `sizeof(block.msg)`.
 *  - La tabla de comandos replica la de `msm_restart_prepare()` (msm-poweroff.c)
 *    en criterio de comparación (prefijo vs exacto) para que los dos canales no
 *    puedan discrepar, y añade los que sólo entiende HTC (download, ftm,
 *    eraseflash).
 *  - Se prueban las dos rutas by-name (bootdevice/ y la plana) por si el
 *    symlink de init.qcom.rc no estuviera.
 *
 * PENDIENTE DE VALIDAR EN HW.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation.
 */

#define pr_fmt(fmt) "htc_reboot_block: " fmt

#include <linux/err.h>
#include <linux/fcntl.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/reboot.h>
#include <linux/string.h>
#include <linux/types.h>

/* Constantes que interpreta el LK. Ver aboot-reboot-modes.md. */
#define RESTART_REASON_BOOT_BASE	0x77665500
#define RESTART_REASON_BOOTLOADER	(RESTART_REASON_BOOT_BASE | 0x00)
#define RESTART_REASON_REBOOT		(RESTART_REASON_BOOT_BASE | 0x01)
#define RESTART_REASON_RECOVERY		(RESTART_REASON_BOOT_BASE | 0x02)
#define RESTART_REASON_LOGGING		(RESTART_REASON_BOOT_BASE | 0x08)
#define RESTART_REASON_RAMDUMP		(RESTART_REASON_BOOT_BASE | 0xAA)
#define RESTART_REASON_ERASE_FLASH	(RESTART_REASON_BOOT_BASE | 0xEF)

#define RESTART_REASON_OEM_BASE		0x6f656d00
#define RESTART_REASON_DOWNLOAD		(RESTART_REASON_OEM_BASE | 0xE0)
#define RESTART_REASON_FTM		(RESTART_REASON_OEM_BASE | 0xE1)
#define RESTART_REASON_RIL_FATAL	(RESTART_REASON_OEM_BASE | 0x99)

/*
 * Formato exacto que espera el LK: lee 68 bytes crudos de la partición y los
 * copia a su struct (read_reboot_block() @0x8ea425f8 del aboot del a16).
 */
struct reboot_block {
	u32  reason;
	char msg[64];
};

static char pname[32];
static u64  poffset;
static bool armed;

static void reboot_block_write(u32 reason, const char *msg)
{
	/*
	 * init.qcom.rc del común crea el symlink `bootdevice`; se prueba también
	 * la ruta plana por si el reinicio ocurre antes de que exista.
	 */
	static const char * const dirs[] = {
		"/dev/block/bootdevice/by-name/",
		"/dev/block/by-name/",
	};
	struct reboot_block block;
	struct file *filp = ERR_PTR(-ENOENT);
	char path[96];
	ssize_t n;
	int i;

	memset(&block, 0, sizeof(block));
	block.reason = reason;
	strlcpy(block.msg, msg ? : "", sizeof(block.msg));

	for (i = 0; i < ARRAY_SIZE(dirs); i++) {
		snprintf(path, sizeof(path), "%s%s", dirs[i], pname);
		filp = filp_open(path, O_RDWR | O_SYNC, 0);
		if (!IS_ERR(filp))
			break;
	}

	if (IS_ERR(filp)) {
		pr_err("no se pudo abrir la particion '%s' (%ld): se pierde reason=0x%08x\n",
		       pname, PTR_ERR(filp), reason);
		return;
	}

	n = kernel_write(filp, (const char *)&block, sizeof(block), poffset);
	if (n == sizeof(block)) {
		/* El reset viene justo detrás: forzar que llegue al medio. */
		vfs_fsync(filp, 0);
		/*
		 * pr_notice (5), NO pr_info: Android deja console_loglevel en 6 y
		 * un mensaje sólo sale si su nivel es MENOR que ese valor, así que
		 * un pr_info aquí es invisible justo cuando hace falta (medido en
		 * HW 20260725: el driver parecía no escribir nunca).
		 */
		pr_notice("reason=0x%08x msg='%s' -> %s+0x%llx\n",
			  reason, block.msg, path, poffset);
	} else {
		pr_err("kernel_write fallo (%zd): se pierde reason=0x%08x\n",
		       n, reason);
	}

	filp_close(filp, NULL);
}

/*
 * `len` = longitud a comparar con strncmp (coincidencia por PREFIJO), o 0 para
 * exigir igualdad exacta. Mismo criterio que msm_restart_prepare().
 */
static const struct cmd_reason_map {
	const char *cmd;
	size_t      len;
	u32         reason;
} cmd_reason_map[] = {
	{ "bootloader",			10,	RESTART_REASON_BOOTLOADER },
	{ "recovery",			 8,	RESTART_REASON_RECOVERY },
	/* Sólo HTC: los sirve la partición `hosd`. */
	{ "download",			 8,	RESTART_REASON_DOWNLOAD },
	{ "ftm",			 3,	RESTART_REASON_FTM },
	{ "eraseflash",			 0,	RESTART_REASON_ERASE_FLASH },
	/* Reinicios anómalos: el LK los desvía a ramdump. */
	{ "power-key-force-hard",	 0,	RESTART_REASON_RAMDUMP },
	{ "force-dog-bark",		 0,	RESTART_REASON_RAMDUMP },
	{ "force-hard",			 0,	RESTART_REASON_RAMDUMP },
	{ "dm-verity device corrupted",	 0,	RESTART_REASON_LOGGING },
};

static int reboot_block_notify(struct notifier_block *nb, unsigned long event,
			       void *data)
{
	const char *cmd = data;
	unsigned int code;
	int i;

	/* `data` es NULL en reboot sin comando y en apagado. */
	if (event != SYS_RESTART || !armed)
		return NOTIFY_DONE;

	cmd = cmd ? : "";

	for (i = 0; i < ARRAY_SIZE(cmd_reason_map); i++) {
		const struct cmd_reason_map *m = &cmd_reason_map[i];

		if (m->len ? !strncmp(cmd, m->cmd, m->len) : !strcmp(cmd, m->cmd)) {
			reboot_block_write(m->reason, cmd);
			return NOTIFY_DONE;
		}
	}

	/*
	 * `edl` no pasa por aquí a propósito: lo sirve el magic de EMERGENCY
	 * DLOAD en IMEM (enable_emergency_dload_mode()), no el LK.
	 */
	if (!strncmp(cmd, "edl", 3))
		return NOTIFY_DONE;

	if (!strncmp(cmd, "oem-", 4) && !kstrtouint(cmd + 4, 16, &code)) {
		code &= 0xff;
		/* oem-93..98 son RIL fatal; el LK sólo reconoce el 99. */
		if (code >= 0x93 && code <= 0x98)
			code = 0x99;
		reboot_block_write(RESTART_REASON_OEM_BASE | code, cmd);
		return NOTIFY_DONE;
	}

	/* Reinicio normal (incluye cmd vacío) y cualquier comando desconocido. */
	reboot_block_write(RESTART_REASON_REBOOT, cmd);
	return NOTIFY_DONE;
}

static struct notifier_block reboot_block_nb = {
	.notifier_call = reboot_block_notify,
};

static int __init htc_reboot_block_init(void)
{
	struct device_node *np;
	const char *tmp;
	int ret;

	BUILD_BUG_ON(sizeof(struct reboot_block) != 68);

	/*
	 * El nodo lo inyecta el LK en tiempo de arranque, no está en el DTS: en
	 * el a16 aparece como /reboot_block (raíz). of_find_compatible_node()
	 * recorre el árbol entero, así que da igual dónde lo cuelgue.
	 */
	np = of_find_compatible_node(NULL, NULL, "htc,reboot_block");
	if (!np) {
		pr_info("sin nodo htc,reboot_block: desactivado (¿bootloader no-HTC?)\n");
		return 0;
	}

	ret = of_property_read_string(np, "pname", &tmp);
	if (ret) {
		pr_err("el nodo no trae 'pname' (%d)\n", ret);
		goto out;
	}
	strlcpy(pname, tmp, sizeof(pname));

	ret = of_property_read_u64(np, "poffset", &poffset);
	if (ret) {
		pr_err("el nodo no trae 'poffset' (%d)\n", ret);
		goto out;
	}

	ret = register_reboot_notifier(&reboot_block_nb);
	if (ret) {
		pr_err("register_reboot_notifier fallo (%d)\n", ret);
		goto out;
	}

	armed = true;
	pr_info("armado: particion '%s' offset 0x%llx\n", pname, poffset);
out:
	of_node_put(np);
	return 0;
}
/*
 * La partición sólo se abre desde el notificador de reinicio, así que no hay
 * dependencia del eMMC en el arranque (el original usaba fs_initcall por eso).
 */
late_initcall(htc_reboot_block_init);
