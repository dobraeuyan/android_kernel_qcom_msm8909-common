/* drivers/input/touchscreen/gslX68X.h
 *
 * 2010 - 2013 SLIEAD Technology.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the SLIEAD's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */
#ifndef __GSL_TS_DRIVER_H_
#define __GSL_TS_DRIVER_H_

//#include <mach/gpio.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/mutex.h>
#include <linux/input.h>
/* msm-3.18 -> msm-4.9 port: <linux/wakelock.h> was removed upstream; the
 * gesture wake path now uses the PM wakeup_source API (struct wakeup_source). */
#include <linux/pm_wakeup.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif


#define GSL_DEBUG
//#define GSL_TIMER                   //esd  timer  switch
#define TPD_PROC_DEBUG              //proc    sysfile  interface  switch
#define GSL9XX_VDDIO_1800   1
#define GSL_REPORT_POINT_SLOT

/* GSL_GESTURE deliberately OFF (20260719).
 *
 * LineageOS charter, Kernel section: "All devices MUST NOT implement software
 * based touchscreen wake features such as double tap to wake, swipe to wake or
 * gestures if there is no hardware-backed support for them in the touchscreen
 * firmware."
 *
 * Silead has no firmware-side gesture support. With this defined the driver
 * pulls RAW coordinates out of DATA_REG, runs them through gsl_alg_id_main()
 * and then calls gsl_obtain_gesture() -- a symbol that exists only inside the
 * prebuilt ARM32 blob gsl_point_id (GestureMain/GestureModel/GestureLRUD/...).
 * Recognition therefore happens on the application processor, which is exactly
 * what the charter forbids. Contrast the GT9110 (gt9xx.c), which programs the
 * controller into firmware doze via reg 0x8046 and reads an already-classified
 * gesture byte back from 0x814B -- that one is hardware-backed and stays.
 *
 * Three concrete consequences of leaving it on, all fixed by this change:
 *   1. gsl_gesture_flag defaults to 1 (gsl_ts_driver.c:83) and the Silead
 *      driver never creates the gesture_mode/tp_gesture_status/gesture node
 *      that power-qcom.c writes -- only GT9110 does -- so the framework had no
 *      way to turn it off on a Silead unit.
 *   2. gsl_ts_suspend() took the gsl_enter_doze() early-return
 *      (gsl_ts_driver.c:1601-1606) and never reached the real suspend path at
 *      :1608-1611, so the touch controller kept its IRQ enabled and its reset
 *      GPIO high across suspend. Undefining this compiles that #ifndef branch
 *      back in: disable_irq_nosync() + reset GPIO low.
 *   3. It drops the struct wakeup_source *gsl_wake_lock member below.
 *
 * All 16 #ifdef GSL_GESTURE blocks in gsl_ts_driver.c are cleanly paired and
 * every gesture symbol lives inside one, so this is a self-contained switch.
 * Normal touch is unaffected: gsl_alg_id_main() (GSL_ALG_ID) still runs.
 */
//#define GSL_GESTURE
//#define GSL_COMPATIBLE_GPIO
#define GSL_ALG_ID

/*define i2c addr and device name*/
#define GSL_TS_ADDR                 0x40
#define GSL_TS_NAME             "GSL_TP"
#define GSL_IRQ_GPIO_NUM  13
#define GSL_IRQ_NUM			gpio_to_irq(GSL_IRQ_GPIO_NUM)
#define GSL_IRQ_NAME        "gsl_irq"
#define GSL_RST_NAME        "gsl_reset"
#define UINT unsigned int 
#define GSL_COORDS_ARR_SIZE 4
#define GSL_VTG_MIN_UV      2700000
#define GSL_VTG_MAX_UV      3300000
#define GSL_I2C_VTG_MIN_UV  1800000
#define GSL_I2C_VTG_MAX_UV  1800000
#define MAX_BUTTONS         4
#define GSL_PINCTRL_EN 1

#define GSL_TIMER_CHECK_CIRCLE      200
#define GSL_PRESSURE                50

#define HW_RST_CHIP

/*debug of time*/
#define TPD_DEBUG_TIME              0x20141209

/*define screen of resolution ratio*/
#define GSL_MAX_X       800
#define GSL_MAX_Y       1280
u8 gsl_cfg_index = 0;
/*virtual keys*/
//#define TOUCH_VIRTUAL_KEYS
#if GSL_PINCTRL_EN
    #include <linux/pinctrl/pinctrl.h>
    #include <linux/pinctrl/consumer.h>
#endif

/*button of key*/
//#define GSL_HAVE_TOUCH_KEY            0
#ifdef GSL_HAVE_TOUCH_KEY
    struct key_data{
        u16 key;
        u32 x_min;
        u32 x_max;
        u32 y_min;
        u32 y_max;
    };
    #define GSL_KEY_NUM  3
#endif

struct gsl_touch_info{
    int x[10];
    int y[10];
    int id[10];
    int finger_num;
};

struct gsl_ts_platform_data {
    const char *name;
    u32 irq_gpio;
    u32 irq_gpio_flags;
    u32 irq_num;
    u32 reset_gpio;
    u32 reset_gpio_flags;
    u32 x_max;
    u32 y_max;
    u32 x_min;
    u32 y_min;
    u32 panel_minx;
    u32 panel_miny;
    u32 panel_maxx;
    u32 panel_maxy;
    u32 num_max_touches;
    u32 panel_tx_num;
    u32 panel_rx_num;
    u32 button_map[MAX_BUTTONS];
    u32 num_buttons;
    u32 hard_reset_delay_ms;
    u32 post_hard_reset_delay_ms;
};

struct gsl_ts_data{
    struct i2c_client *client;
    struct input_dev *input_dev;
    struct work_struct work;
    struct workqueue_struct *wq;
    struct regulator *vdd;
    struct regulator *vcc_i2c;
    struct mutex gsl_i2c_lock;
    /* Modified by zhangyijie for fix system dump without tp ZQ1989-129 2018-04-04 begin */
    #ifdef GSL_GESTURE
    /* msm-3.18 -> msm-4.9 port: the old android wake_lock API was removed in
     * 4.9; use the PM wakeup_source API instead (pointer + register). */
    struct wakeup_source *gsl_wake_lock;
    #endif
    /* Modified by zhangyijie for fix system dump without tp ZQ1989-129 2018-04-04 end */
    struct gsl_ts_platform_data *pdata;

    #if GSL_PINCTRL_EN
    struct pinctrl *pinctrl;
    struct pinctrl_state *pins_active;
    #endif

    #if defined(CONFIG_FB)
    struct notifier_block fb_notif;
    #elif defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend pm;
    #endif

    struct gsl_touch_info *cinfo;
    bool suspended;                         //0 normal;1 the machine is suspend;
    bool updating_fw;                       //0 normal;1 download the firmware;
    u32 gsl_up_flag;                        //0 normal;1 have one up event;
    u32 gsl_point_state;                    //the point down and up of state
#ifdef GSL_TIMER
    struct delayed_work     timer_work;
    struct workqueue_struct *timer_wq;
    volatile int gsl_timer_flag;            //0:first test  1:second test 2:doing gsl_load_fw
    unsigned int gsl_timer_data;
#endif
#ifdef GSL_HAVE_TOUCH_KEY
    int gsl_key_state;
#endif
    struct work_struct resume_work;
    u8 fw_version;
    u8 pannel_id;
//  struct ts_func_test_device ts_test_dev;
};



#ifdef GSL_ALG_ID
extern unsigned int gsl_version_id(void);
extern void gsl_alg_id_main(struct gsl_touch_info *cinfo);
extern void gsl_DataInit(int *ret);
extern unsigned int gsl_mask_tiaoping(void);
extern int gsl_obtain_gesture(void);
extern void gsl_FunIICRead(unsigned int (*fun) (unsigned int *,unsigned int,unsigned int));
#endif
struct fw_data
{
    u32 offset : 8;
    u32 : 0;
    u32 val;
};
#include"gsl_ts_fw.h"
//static unsigned  char gsl_cfg_index=0;
struct fw_config_type
{
	const struct fw_data *fw;
	unsigned int fw_size;
    #ifdef GSL_ALG_ID
	unsigned int *data_id;
	unsigned int data_size;
    #endif
};
static const struct fw_config_type gsl_cfg_table[9] = {
/*0*/{GSLX68X_FW_DJ,(sizeof(GSLX68X_FW_DJ)/sizeof(struct fw_data)),
gsl_config_data_id_DJ,(sizeof(gsl_config_data_id_DJ)/4)},
///*1*/{NULL,0,NULL,0},
///*2*/{NULL,0,NULL,0},
///*3*/{NULL,0,NULL,0},
///*4*/{NULL,0,NULL,0},
///*5*/{NULL,0,NULL,0},
///*6*/{NULL,0,NULL,0},
///*7*/{NULL,0,NULL,0},
//*8*/{NULL,0,NULL,0},
};
#endif


