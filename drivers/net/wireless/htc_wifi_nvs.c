/*
 * arch/arm/mach-msm/htc_wifi_nvs.c
 *
 * Code to extract and provide WiFi calibration (NVS) information.
 * NVS data is primarily sourced from the Device Tree.
 *
 * Original Copyright (C) 2008 Google, Inc.
 * Original Author: Dmitry Shmidt <dimitrysh@google.com>
 *
 * This driver has been refactored for clarity, efficiency, and robustness.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>   /* For min_t, max_t, pr_info, printk, etc. */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/string.h>   /* For memcpy, memset */
#include <linux/export.h>
#include <linux/proc_fs.h>  /* For procfs functions */
#include <linux/of.h>       /* For Device Tree functions like of_find_node_by_path, of_get_property, of_node_put */
#include <linux/uaccess.h>  /* For copy_to_user */
#include <linux/fs.h>       /* For struct file_operations */
#include <linux/if_ether.h> /* For ETH_ALEN */
#include <linux/slab.h>     /* For kstrdup, kfree (not used in this version for NVS data) */
#include <linux/errno.h>    /* For error codes like -ENOMEM, -EFAULT */

/*
 * NVS Data Structure (inferred from offsets):
 * The NVS data blob is expected to have a header.
 * - NVS_LEN_OFFSET: Offset within the NVS blob where the 32-bit length
 * of the actual calibration payload is stored (assumed little-endian).
 * - NVS_DATA_OFFSET: Offset where the actual calibration payload begins.
 * - NVS_MAX_SIZE: The maximum expected size of the entire NVS blob (header + payload).
 */
#define NVS_MAX_SIZE            0x800U /* 2048 bytes */
#define NVS_LEN_OFFSET          0x0CU  /* 12 bytes */
#define NVS_DATA_OFFSET         0x40U  /* 64 bytes */

/* Device Tree configuration for NVS data */
#define CALIBRATION_DATA_NODE_PATH "/calibration_data"
#define WIFI_NVS_PROPERTY_NAME     "wifi_eeprom"

/* Global cache for WiFi NVS data read from DT */
static unsigned char wifi_nvs_ram_cache[NVS_MAX_SIZE];
/* Flag: true if NVS data initialization from DT has been attempted */
static bool nvs_initialization_attempted = false;
/* Flag: true if NVS data was successfully loaded and its size parsed */
static bool nvs_data_is_valid = false;
/* Cached size of the NVS payload (the actual calibration data part) */
static unsigned int cached_nvs_payload_size = 0;

/* Procfs entries */
static struct proc_dir_entry *wifi_calibration_payload_proc_entry;
static struct proc_dir_entry *wifi_nvs_header_proc_entry;


/**
 * get_wifi_nvs_ram - Retrieve a pointer to the cached WiFi NVS data.
 *
 * The NVS data is intended to be read from the Device Tree once during
 * module initialization and cached in an internal buffer. This function
 * returns a pointer to this cache.
 * If the NVS data could not be loaded during initialization, the content
 * of the returned buffer might be all zeros.
 *
 * Return: Pointer to the global NVS data buffer.
 */
unsigned char *get_wifi_nvs_ram(void)
{
    /* Data is loaded into wifi_nvs_ram_cache by load_nvs_data_from_dt() at init time */
    return wifi_nvs_ram_cache;
}
EXPORT_SYMBOL(get_wifi_nvs_ram);

/**
 * wlan_random_mac - Get or set a statically stored WLAN MAC address.
 * @set_mac_addr: If not NULL, this MAC address will be copied into the
 * internal static storage. The buffer must contain ETH_ALEN bytes.
 *
 * Note: The "random" in the function name is a misnomer due to its original
 * implementation. This function does NOT generate a random MAC address.
 * It serves as a simple getter/setter for a specific MAC address,
 * typically the device's factory MAC address retrieved from NVS data.
 * The WLAN driver would use this to store and consistently retrieve the MAC.
 *
 * Return: Pointer to the statically stored MAC address buffer (ETH_ALEN bytes).
 * The buffer is initialized to a locally administered address
 * (02:00:00:00:00:00) if never explicitly set.
 */
unsigned char *wlan_random_mac(unsigned char *set_mac_addr)
{
    static unsigned char mac_addr[ETH_ALEN];
    static bool mac_initialized = false;

    if (!mac_initialized) {
        /* Initialize to a generic locally administered MAC if never set */
        mac_addr[0] = 0x02; /* Locally administered, unicast */
        mac_addr[1] = 0x00;
        mac_addr[2] = 0x00;
        mac_addr[3] = 0x00;
        mac_addr[4] = 0x00;
        mac_addr[5] = 0x00;
        mac_initialized = true;
    }

    if (set_mac_addr != NULL) {
        memcpy(mac_addr, set_mac_addr, ETH_ALEN);
    }
    return mac_addr;
}
EXPORT_SYMBOL(wlan_random_mac);

/* Internal helper to get the cached NVS payload size; ensures validity check. */
static unsigned int internal_get_nvs_payload_size(void)
{
    if (!nvs_data_is_valid) {
        return 0;
    }
    return cached_nvs_payload_size;
}

/*
 * Reads NVS data from the Device Tree and populates the global cache.
 * This function is called once during module initialization.
 */
static void load_nvs_data_from_dt(void)
{
    struct device_node *cal_data_node;
    const void *dt_property_data_void = NULL; /* Use void* for of_get_property */
    const unsigned char *dt_property_data_uchar = NULL;
    int dt_property_size = 0;
    u32 payload_len_from_nvs_header_le; /* Assuming length is LE32 */

    if (nvs_initialization_attempted) {
        /* Should not happen if init logic is correct, but as a safeguard */
        pr_debug("%s: NVS data initialization already attempted.\n", __func__);
        return;
    }
    nvs_initialization_attempted = true;

    /* Ensure the cache is in a known state (zeroed) before attempting to load */
    memset(wifi_nvs_ram_cache, 0, NVS_MAX_SIZE);

    cal_data_node = of_find_node_by_path(CALIBRATION_DATA_NODE_PATH);
    if (!cal_data_node) {
        pr_warn("%s: Device Tree node '%s' not found.\n", __func__, CALIBRATION_DATA_NODE_PATH);
        return;
    }

    dt_property_data_void = of_get_property(cal_data_node, WIFI_NVS_PROPERTY_NAME, &dt_property_size);
    /* of_node_put() must be called after of_find_node_by_path() or similar functions that return a node with an incremented refcount. */
    of_node_put(cal_data_node);

    if (!dt_property_data_void || dt_property_size <= 0) {
        pr_warn("%s: DT property '%s' not found, empty, or error under '%s'.\n",
                __func__, WIFI_NVS_PROPERTY_NAME, CALIBRATION_DATA_NODE_PATH);
        return;
    }
    dt_property_data_uchar = (const unsigned char *)dt_property_data_void;


    if (dt_property_size > NVS_MAX_SIZE) {
        pr_warn("%s: NVS data from DT (%d bytes) exceeds cache size (%u bytes). Truncating.\n",
                __func__, dt_property_size, NVS_MAX_SIZE);
        dt_property_size = NVS_MAX_SIZE; /* Truncate to fit the cache */
    }

    memcpy(wifi_nvs_ram_cache, dt_property_data_uchar, dt_property_size);

    /* Validate that we can safely read the payload length from the NVS header in our cache */
    if (dt_property_size < (NVS_LEN_OFFSET + sizeof(u32))) {
        pr_err("%s: Cached NVS data is too short (%d bytes) to contain the 32-bit payload length field at offset %u.\n",
               __func__, dt_property_size, NVS_LEN_OFFSET);
        return; /* NVS data is considered invalid as we can't parse its structure */
    }

    /* Read the payload length from the NVS header (assuming little-endian 32-bit format) */
    memcpy(&payload_len_from_nvs_header_le, &wifi_nvs_ram_cache[NVS_LEN_OFFSET], sizeof(u32));
    cached_nvs_payload_size = le32_to_cpu(payload_len_from_nvs_header_le);

    /*
     * Sanity check the parsed payload length:
     * 1. It cannot be larger than the available space in the cache after NVS_DATA_OFFSET.
     * 2. The end of the payload (NVS_DATA_OFFSET + parsed length) cannot exceed the total data read from DT.
     */
    if (cached_nvs_payload_size > (NVS_MAX_SIZE - NVS_DATA_OFFSET)) {
        pr_warn("%s: NVS header's payload length (%u) exceeds max possible payload size (%u). Clamping.\n",
                __func__, cached_nvs_payload_size, (NVS_MAX_SIZE - NVS_DATA_OFFSET));
        cached_nvs_payload_size = NVS_MAX_SIZE - NVS_DATA_OFFSET;
    }

    if ((NVS_DATA_OFFSET + cached_nvs_payload_size) > dt_property_size) {
         pr_warn("%s: NVS declared payload end (offset %u + len %u = %u) exceeds actual DT data size (%d). Adjusting payload size.\n",
                 __func__, NVS_DATA_OFFSET, cached_nvs_payload_size,
                 NVS_DATA_OFFSET + cached_nvs_payload_size, dt_property_size);
        /* Adjust payload size to what's actually available from DT after the header */
        cached_nvs_payload_size = (dt_property_size > NVS_DATA_OFFSET) ? (dt_property_size - NVS_DATA_OFFSET) : 0;
    }

    nvs_data_is_valid = true; /* Mark NVS data as successfully loaded and parsed */
    pr_info("WiFi NVS data loaded. Total DT blob size: %d bytes. Calculated payload size: %u bytes.\n",
            dt_property_size, cached_nvs_payload_size);
}

/* Procfs read handler for the NVS calibration payload part */
static ssize_t wifi_calibration_payload_read_proc(struct file *file, char __user *user_buf,
                                             size_t user_buf_size, loff_t *ppos)
{
    unsigned int payload_len;
    unsigned long offset_in_payload = *ppos; /* Current reading position in the payload */
    unsigned int bytes_to_copy;

    if (!nvs_data_is_valid) {
        pr_warn_once("%s: Attempt to read NVS calibration payload, but data is not valid/initialized.\n", __func__);
        return 0; /* No data available or error state */
    }

    payload_len = internal_get_nvs_payload_size();

    if (offset_in_payload >= payload_len) {
        return 0; /* End of file */
    }

    bytes_to_copy = min_t(size_t, user_buf_size, payload_len - offset_in_payload);

    if (copy_to_user(user_buf, &wifi_nvs_ram_cache[NVS_DATA_OFFSET + offset_in_payload], bytes_to_copy)) {
        pr_err("%s: Failed to copy NVS calibration payload to user space.\n", __func__);
        return -EFAULT;
    }

    *ppos += bytes_to_copy; /* Advance the file position */
    return bytes_to_copy;   /* Return number of bytes read */
}

/* Procfs read handler for the NVS header part */
static ssize_t wifi_nvs_header_read_proc(struct file *file, char __user *user_buf,
                                    size_t user_buf_size, loff_t *ppos)
{
    unsigned long offset_in_header = *ppos; /* Current reading position in the header */
    unsigned int bytes_to_copy;

    /*
     * The header should only be considered valid if the entire NVS data loading
     * and parsing process was successful.
     */
    if (!nvs_data_is_valid) {
         pr_warn_once("%s: Attempt to read NVS header, but NVS data is not valid/initialized.\n", __func__);
        return 0; /* No data available or error state */
    }

    /* The NVS header part is defined to be NVS_DATA_OFFSET bytes long */
    if (offset_in_header >= NVS_DATA_OFFSET) {
        return 0; /* End of file for the header part */
    }

    bytes_to_copy = min_t(size_t, user_buf_size, NVS_DATA_OFFSET - offset_in_header);

    if (copy_to_user(user_buf, &wifi_nvs_ram_cache[offset_in_header], bytes_to_copy)) {
        pr_err("%s: Failed to copy NVS header to user space.\n", __func__);
        return -EFAULT;
    }

    *ppos += bytes_to_copy; /* Advance the file position */
    return bytes_to_copy;   /* Return number of bytes read */
}

/* File operations for /proc/calibration (payload part) */
static const struct file_operations htc_wifi_calibration_payload_fops = {
    .owner = THIS_MODULE,
    .read  = wifi_calibration_payload_read_proc,
    /* No write, llseek, etc. needed for this simple read-only proc file */
};

/* File operations for /proc/wifi_data (header part) */
static const struct file_operations htc_wifi_nvs_header_fops = {
    .owner = THIS_MODULE,
    .read  = wifi_nvs_header_read_proc,
    /* No write, llseek, etc. needed for this simple read-only proc file */
};

static int __init wifi_nvs_module_init(void)
{
    pr_info("Initializing WiFi NVS Data loader driver.\n");

    /* Attempt to load NVS data from Device Tree into the cache */
    load_nvs_data_from_dt(); /* This also sets nvs_data_is_valid and cached_nvs_payload_size */

    /* Create /proc/calibration for the payload part */
    wifi_calibration_payload_proc_entry = proc_create_data("calibration", 0444, NULL,
                                                      &htc_wifi_calibration_payload_fops, NULL);
    if (!wifi_calibration_payload_proc_entry) {
        pr_err("Failed to create /proc/calibration procfs entry.\n");
        return -ENOMEM;
    }
    /*
     * The ->size member of proc_dir_entry is not directly settable in a portable
     * or always safe way, especially if the full struct definition is not
     * available (causing "incomplete type" errors). The actual data length is
     * handled by the read function. Removing direct ->size assignments.
     */
    // if (wifi_calibration_payload_proc_entry) { // Original problematic line removed
    //     wifi_calibration_payload_proc_entry->size = internal_get_nvs_payload_size();
    // }


    /* Create /proc/wifi_data for the header part */
    wifi_nvs_header_proc_entry = proc_create_data("wifi_data", 0444, NULL,
                                               &htc_wifi_nvs_header_fops, NULL);
    if (!wifi_nvs_header_proc_entry) {
        pr_err("Failed to create /proc/wifi_data procfs entry.\n");
        remove_proc_entry("calibration", NULL); /* Clean up the previously created entry */
        return -ENOMEM;
    }
    // if (wifi_nvs_header_proc_entry) { // Original problematic line removed
    //    wifi_nvs_header_proc_entry->size = NVS_DATA_OFFSET;
    // }


    pr_info("WiFi NVS procfs entries (/proc/calibration, /proc/wifi_data) created.\n");
    return 0;
}

static void __exit wifi_nvs_module_exit(void)
{
    pr_info("Unloading WiFi NVS Data loader driver.\n");
    remove_proc_entry("calibration", NULL);
    remove_proc_entry("wifi_data", NULL);
    pr_info("WiFi NVS procfs entries removed.\n");
}

/*
 * Use late_initcall to ensure the Device Tree is parsed and available,
 * and that other essential kernel subsystems are initialized.
 */
late_initcall(wifi_nvs_module_init);
module_exit(wifi_nvs_module_exit); /* Define exit if this can be compiled as a loadable module */

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("WiFi NVS (Non-Volatile Storage) Data Loader from Device Tree");
MODULE_AUTHOR("Dmitry Shmidt <dimitrysh@google.com> (Original); Refactored in 2025");


