/*
 * Synaptics RMI4 touchscreen driver
 *
 * Copyright (C) 2012 Synaptics Incorporated
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
 * Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/input/synaptics_dsx.h>
#include <linux/of_gpio.h>
#include "gn_synaptics_rmi4_ts.h"
#include <linux/input/mt.h>

#define DRIVER_NAME "synaptics_rmi4_i2c"
#define INPUT_PHYS_NAME "synaptics_rmi4_i2c/input0"

#define RESET_DELAY 100

#define TYPE_B_PROTOCOL

#define DOUBLE_CLICK_WAKE

#define INIT_TP_WHEN_RESUME

//#define NO_0D_WHILE_2D
/*
#define REPORT_2D_Z
*/
#define REPORT_2D_W

#define RPT_TYPE (1 << 0)
#define RPT_X_LSB (1 << 1)
#define RPT_X_MSB (1 << 2)
#define RPT_Y_LSB (1 << 3)
#define RPT_Y_MSB (1 << 4)
#define RPT_Z (1 << 5)
#define RPT_WX (1 << 6)
#define RPT_WY (1 << 7)
#define RPT_DEFAULT (RPT_TYPE | RPT_X_LSB | RPT_X_MSB | RPT_Y_LSB | RPT_Y_MSB)

#define EXP_FN_DET_INTERVAL 1000 /* ms */
#define POLLING_PERIOD 1 /* ms */
#define SYN_I2C_RETRY_TIMES 10
#define MAX_ABS_MT_TOUCH_MAJOR 15

#define F01_STD_QUERY_LEN 21
#define F01_BUID_ID_OFFSET 18
#define F11_STD_QUERY_LEN 9
#define F11_STD_CTRL_LEN 10
#define F11_STD_DATA_LEN 12

#define NORMAL_OPERATION (0 << 0)
#define SENSOR_SLEEP (1 << 0)
#define NO_SLEEP_OFF (0 << 2)
#define NO_SLEEP_ON (1 << 2)

enum device_status {
	STATUS_NO_ERROR = 0x00,
	STATUS_RESET_OCCURED = 0x01,
	STATUS_INVALID_CONFIG = 0x02,
	STATUS_DEVICE_FAILURE = 0x03,
	STATUS_CONFIG_CRC_FAILURE = 0x04,
	STATUS_FIRMWARE_CRC_FAILURE = 0x05,
	STATUS_CRC_IN_PROGRESS = 0x06
};

#define RMI4_VTG_MIN_UV		2700000
#define RMI4_VTG_MAX_UV		3300000
#define RMI4_ACTIVE_LOAD_UA	15000

#define RMI4_I2C_VTG_MIN_UV	1800000
#define RMI4_I2C_VTG_MAX_UV	1800000
#define RMI4_I2C_LOAD_UA	10000

static int synaptics_rmi4_i2c_read(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data,
		unsigned short length);

static int synaptics_rmi4_i2c_write(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data,
		unsigned short length);

static int synaptics_rmi4_reset_device(struct synaptics_rmi4_data *rmi4_data);

static int synaptics_rmi4_suspend(struct device *dev);

static int synaptics_rmi4_resume(struct device *dev);

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data);
#endif

static ssize_t synaptics_rmi4_f01_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_f01_productinfo_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_f01_buildid_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_f01_flashprog_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_0dbutton_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_0dbutton_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_flipx_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_flipx_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_flipy_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_flipy_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);


struct synaptics_rmi4_f01_device_status {
	union {
		struct {
			unsigned char status_code:4;
			unsigned char reserved:2;
			unsigned char flash_prog:1;
			unsigned char unconfigured:1;
		} __packed;
		unsigned char data[1];
	};
};

struct synaptics_rmi4_f1a_query {
	union {
		struct {
			unsigned char max_button_count:3;
			unsigned char reserved:5;
			unsigned char has_general_control:1;
			unsigned char has_interrupt_enable:1;
			unsigned char has_multibutton_select:1;
			unsigned char has_tx_rx_map:1;
			unsigned char has_perbutton_threshold:1;
			unsigned char has_release_threshold:1;
			unsigned char has_strongestbtn_hysteresis:1;
			unsigned char has_filter_strength:1;
		} __packed;
		unsigned char data[2];
	};
};

struct synaptics_rmi4_f1a_control_0 {
	union {
		struct {
			unsigned char multibutton_report:2;
			unsigned char filter_mode:2;
			unsigned char reserved:4;
		} __packed;
		unsigned char data[1];
	};
};

struct synaptics_rmi4_f1a_control_3_4 {
	unsigned char transmitterbutton;
	unsigned char receiverbutton;
};

struct synaptics_rmi4_f1a_control {
	struct synaptics_rmi4_f1a_control_0 general_control;
	unsigned char *button_int_enable;
	unsigned char *multi_button;
	struct synaptics_rmi4_f1a_control_3_4 *electrode_map;
	unsigned char *button_threshold;
	unsigned char button_release_threshold;
	unsigned char strongest_button_hysteresis;
	unsigned char filter_strength;
};

struct synaptics_rmi4_f1a_handle {
	int button_bitmask_size;
	unsigned char button_count;
	unsigned char valid_button_count;
	unsigned char *button_data_buffer;
	unsigned char *button_map;
	struct synaptics_rmi4_f1a_query button_query;
	struct synaptics_rmi4_f1a_control button_control;
};

struct synaptics_rmi4_exp_fn {
	enum exp_fn fn_type;
	bool inserted;
	int (*func_init)(struct synaptics_rmi4_data *rmi4_data);
	void (*func_remove)(struct synaptics_rmi4_data *rmi4_data);
	void (*func_attn)(struct synaptics_rmi4_data *rmi4_data,
			unsigned char intr_mask);
	struct list_head link;
};

static int tp_max_finger = 0;
static int suspend_flag = 1;

static int pr_debug = 0;
#define ts_debug(args...) \
	do { \
		if (1 == pr_debug) { \
			printk("=== rmi4 tpd === "args); \
		} \
	} while (0)

#ifdef INIT_TP_WHEN_RESUME
int init_not_complete = 0;
EXPORT_SYMBOL(init_not_complete);
int failed_init_flag = 0;
#endif

#if defined(CONFIG_GN_DEVICE_TYPE_CHECK)
#include <linux/gn_device_check.h>
extern int gn_set_device_info(struct gn_device_info gn_dev_info);
struct gn_device_info gn_mydev_info;
#endif

#ifdef DOUBLE_CLICK_WAKE
enum {
    GESTURE_U_UP = 1,
    GESTURE_U_DOWN = 2,
    GESTURE_U_LEFT = 4,
    GESTURE_U_RIGHT = 8,
};

static int wake_switch = 1;
static int gesture_switch = 0;
static int wake_suspend_compelete = 0;
struct synaptics_rmi4_data * test_rmi4_data;

static struct platform_device gn_tp_wake_device = {
       .name   = "tp_wake_switch",
       .id     = -1,
};

static ssize_t tp_wake_switch_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n", wake_switch);
}

static ssize_t tp_wake_switch_write(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int rt;
	unsigned long val;
       unsigned char test_wake_flag = 0, device_ctrl = 0;
       
	rt = strict_strtoul(buf, 10, &val); 
	if(rt != 0){
		pr_err("%s, invalid value\n", __func__);
		return rt;
	}
	wake_switch = val;
	printk("%s, %d\n", __func__, wake_switch);

       if (2 == wake_switch) {
           synaptics_rmi4_i2c_read(test_rmi4_data, 0x0014, &test_wake_flag, 1);
           printk("wanglei tpd: test_wake_flag ====> %d\n", test_wake_flag);
       }

       if (3 == wake_switch) {
           device_ctrl = 0x08;
	    synaptics_rmi4_i2c_write(test_rmi4_data,
			0x57,
			&device_ctrl,
			sizeof(device_ctrl));
       }
    
	return size;
}

static DEVICE_ATTR(double_wake, S_IRWXUGO, tp_wake_switch_show, tp_wake_switch_write);

static ssize_t tp_gesture_switch_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n", gesture_switch);
}

static ssize_t tp_gesture_switch_write(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int rt;
	unsigned long val;
	rt = strict_strtoul(buf, 10, &val); 
	if(rt != 0){
		pr_err("%s, invalid value\n", __func__);
		return rt;
	}
	gesture_switch = val;
	printk("%s, %d\n", __func__, gesture_switch);
	return size;
}

static DEVICE_ATTR(gesture_wake, S_IRWXUGO, tp_gesture_switch_show, tp_gesture_switch_write);

static struct device_attribute *wake_tp_attr_list[] =
{
	&dev_attr_double_wake,
       &dev_attr_gesture_wake,
};

static int wake_tp_create_attr(struct device *dev) 
{
	int idx, err = 0;
	int num = (int)(sizeof(wake_tp_attr_list)/sizeof(wake_tp_attr_list[0]));
	printk("tpd %s\n", __func__);
	if(!dev)
	{
		return -EINVAL;
	}	

	for(idx = 0; idx < num; idx++)
	{
		if((err = device_create_file(dev, wake_tp_attr_list[idx])))
		{            
			printk("tpd wanglei: device_create_file (%s) = %d\n", wake_tp_attr_list[idx]->attr.name, err);        
			break;
		}
	}

       return err;
}

#endif

struct synaptics_rmi4_data * test_rmi4_data;
#ifdef TP_GLOVE_SUPPORT
static unsigned char glove_switch = 0;
static unsigned char glove_enable = 0;
static unsigned char glove_status = 0x0f;
static unsigned char charger_enable = 0;
struct synaptics_rmi4_data * glove_rmi4_data;
unsigned char tp_fw_update_compelete = 0;
EXPORT_SYMBOL(tp_fw_update_compelete);

static ssize_t tp_glove_switch_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n", glove_switch);
}

void if_enable_tp_glove( unsigned char enable);

static ssize_t tp_glove_switch_write(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int rt;
	unsigned long val;
	rt = strict_strtoul(buf, 10, &val); 
	if(rt != 0){
		pr_err("%s, invalid value\n", __func__);
		return rt;
	}

       glove_enable = (val > 0)?0x01:0x00;
	//when glove setting enable and charger disable, then enable the glove func
       if (glove_enable && !charger_enable)
	    if_enable_tp_glove(0x00);
       //when glove setting disable, then disable the glove func
       else
           if_enable_tp_glove(0x02);
       
	printk("%s, %d\n", __func__, glove_enable);

	return size;
}

static void gn_tp_glove_work(struct work_struct *work)
{
    //when glove setting enable and charger disable, then enable the glove func
    if (glove_enable && !charger_enable)
        if_enable_tp_glove(0x00);
    //when glove setting disable, then disable the glove func
    else
        if_enable_tp_glove(0x02);
}

static ssize_t tp_charger_status_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n", charger_enable);
}

static ssize_t tp_charger_status_write(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int rt;
	unsigned long val;
	rt = strict_strtoul(buf, 10, &val); 
	if(rt != 0){
		pr_err("tpd: %s, invalid value\n", __func__);
		return rt;
	}

       charger_enable = (val > 0)?0x01:0x00;
       queue_delayed_work(glove_rmi4_data->gn_glove_queue, &glove_rmi4_data->gn_glove_work, 0);
       
	printk("tpd: %s, %d\n", __func__, charger_enable);

	return size;
}
#endif

void if_enable_tp_glove( unsigned char enable)
{
#ifdef TP_GLOVE_SUPPORT
	int retval;

    printk("tpd: enable = %d, glove_status = %d\n", enable, glove_status);

    if (enable == glove_status) {
        printk("tpd: The same command: %d, so return.\n", enable);
        return;
    }

    if (1 == suspend_flag) {
        printk("tpd: TP IC is not running, so return\n");
        return;
    }

    //IF TP Firmware update is not compeleted, then return
    if (0 == tp_fw_update_compelete) {
        printk("tpd: TP Firmware is updating, so return...\n");
        return;
    }

    //when setting is closed, charger cannot open the switch
    if (0 == glove_enable && 0 == enable) {
        printk("tpd: glove disable, so can not enable....\n");
        return ;
    }

    printk("tpd: set glove enable = %d\n", enable);
	retval = synaptics_rmi4_i2c_write(glove_rmi4_data,
			0x400,
			&enable,
			sizeof(enable));
	if (retval < 0) {
		dev_err(&(glove_rmi4_data->input_dev->dev),
				"tpd: %s: Failed to set glove mode\n",
				__func__);
		return ;
	}

    //glove_switch = enable;

    enable = 0x0f;
    synaptics_rmi4_i2c_read(glove_rmi4_data,
			0x400,
			&enable,
			sizeof(enable));

    printk("tpd: glove mode = 0x%x\n", enable);
    glove_switch = (0 == enable)?0x01:0x00; 
    glove_status = enable;

    return ;
#endif
}

EXPORT_SYMBOL(if_enable_tp_glove);

static ssize_t tp_debug_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n", pr_debug);
}

static ssize_t tp_debug_write(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int rt;
	unsigned long val;
	rt = strict_strtoul(buf, 10, &val); 
	if(rt != 0){
		pr_err("%s, invalid value\n", __func__);
		return rt;
	}
	pr_debug = val;
	ts_debug("%s, %d\n", __func__, pr_debug);
	return size;
}

static struct device_attribute attrs[] = {
#ifdef TP_GLOVE_SUPPORT
       __ATTR(glove_enable, (S_IRUGO | S_IWUGO),
			tp_glove_switch_show,
			tp_glove_switch_write),
	__ATTR(charger_enable, (S_IRUGO | S_IWUGO),
			tp_charger_status_show,
			tp_charger_status_write),
#endif
       __ATTR(log_enable, (S_IRUGO | S_IWUGO),
			tp_debug_show,
			tp_debug_write),
	__ATTR(reset, S_IWUGO,
			synaptics_rmi4_show_error,
			synaptics_rmi4_f01_reset_store),
	__ATTR(productinfo, S_IRUGO,
			synaptics_rmi4_f01_productinfo_show,
			synaptics_rmi4_store_error),
	__ATTR(buildid, S_IRUGO,
			synaptics_rmi4_f01_buildid_show,
			synaptics_rmi4_store_error),
	__ATTR(flashprog, S_IRUGO,
			synaptics_rmi4_f01_flashprog_show,
			synaptics_rmi4_store_error),
	__ATTR(0dbutton, (S_IRUGO | S_IWUGO),
			synaptics_rmi4_0dbutton_show,
			synaptics_rmi4_0dbutton_store),
	__ATTR(flipx, (S_IRUGO | S_IWUGO),
			synaptics_rmi4_flipx_show,
			synaptics_rmi4_flipx_store),
	__ATTR(flipy, (S_IRUGO | S_IWUGO),
			synaptics_rmi4_flipy_show,
			synaptics_rmi4_flipy_store),
};

static bool exp_fn_inited;
static struct mutex exp_fn_list_mutex;
static struct list_head exp_fn_list;

#ifdef CONFIG_FB
static void configure_sleep(struct synaptics_rmi4_data *rmi4_data)
{
	int retval = 0;

	rmi4_data->fb_notif.notifier_call = fb_notifier_callback;

	retval = fb_register_client(&rmi4_data->fb_notif);
	if (retval)
		dev_err(&rmi4_data->i2c_client->dev,
			"Unable to register fb_notifier: %d\n", retval);
	return;
}
#else
static void configure_sleep(struct synaptics_rmi4_data *rmi4_data)
{
	return;
}
#endif

static ssize_t synaptics_rmi4_f01_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int reset;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &reset) != 1)
		return -EINVAL;

	if (reset != 1)
		return -EINVAL;

	retval = synaptics_rmi4_reset_device(rmi4_data);
	if (retval < 0) {
		dev_err(dev,
				"%s: Failed to issue reset command, error = %d\n",
				__func__, retval);
		return retval;
	}

	return count;
}

static ssize_t synaptics_rmi4_f01_productinfo_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "0x%02x 0x%02x\n",
			(rmi4_data->rmi4_mod_info.product_info[0]),
			(rmi4_data->rmi4_mod_info.product_info[1]));
}

static ssize_t synaptics_rmi4_f01_buildid_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned int build_id;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	build_id = (unsigned int)rmi->build_id[0] +
			(unsigned int)rmi->build_id[1] * 0x100 +
			(unsigned int)rmi->build_id[2] * 0x10000;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			build_id);
}

static ssize_t synaptics_rmi4_f01_flashprog_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int retval;
	struct synaptics_rmi4_f01_device_status device_status;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_data_base_addr,
			device_status.data,
			sizeof(device_status.data));
	if (retval < 0) {
		dev_err(dev,
				"%s: Failed to read device status, error = %d\n",
				__func__, retval);
		return retval;
	}

	return snprintf(buf, PAGE_SIZE, "%u\n",
			device_status.flash_prog);
}

static ssize_t synaptics_rmi4_0dbutton_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			rmi4_data->button_0d_enabled);
}

static ssize_t synaptics_rmi4_0dbutton_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int input;
	unsigned char ii;
	unsigned char intr_enable;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	input = input > 0 ? 1 : 0;

	if (rmi4_data->button_0d_enabled == input)
		return count;

	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->fn_number == SYNAPTICS_RMI4_F1A) {
				ii = fhandler->intr_reg_num;

				retval = synaptics_rmi4_i2c_read(rmi4_data,
						rmi4_data->f01_ctrl_base_addr +
						1 + ii,
						&intr_enable,
						sizeof(intr_enable));
				if (retval < 0)
					return retval;

				if (input == 1)
					intr_enable |= fhandler->intr_mask;
				else
					intr_enable &= ~fhandler->intr_mask;

				retval = synaptics_rmi4_i2c_write(rmi4_data,
						rmi4_data->f01_ctrl_base_addr +
						1 + ii,
						&intr_enable,
						sizeof(intr_enable));
				if (retval < 0)
					return retval;
			}
		}
	}

	rmi4_data->button_0d_enabled = input;

	return count;
}

static ssize_t synaptics_rmi4_flipx_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
		rmi4_data->flip_x);
}

static ssize_t synaptics_rmi4_flipx_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	rmi4_data->flip_x = input > 0 ? 1 : 0;

	return count;
}

static ssize_t synaptics_rmi4_flipy_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
		rmi4_data->flip_y);
}

static ssize_t synaptics_rmi4_flipy_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	rmi4_data->flip_y = input > 0 ? 1 : 0;

	return count;
}

 /**
 * synaptics_rmi4_set_page()
 *
 * Called by synaptics_rmi4_i2c_read() and synaptics_rmi4_i2c_write().
 *
 * This function writes to the page select register to switch to the
 * assigned page.
 */
static int synaptics_rmi4_set_page(struct synaptics_rmi4_data *rmi4_data,
		unsigned int address)
{
	int retval = 0;
	unsigned char retry;
	unsigned char buf[PAGE_SELECT_LEN];
	unsigned char page;
	struct i2c_client *i2c = rmi4_data->i2c_client;

	page = ((address >> 8) & MASK_8BIT);
	if (page != rmi4_data->current_page) {
		buf[0] = MASK_8BIT;
		buf[1] = page;
		for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
			retval = i2c_master_send(i2c, buf, PAGE_SELECT_LEN);
			if (retval != PAGE_SELECT_LEN) {
				dev_err(&i2c->dev,
						"tpd: %s: I2C retry %d\n",
						__func__, retry + 1);
				msleep(20);
			} else {
				rmi4_data->current_page = page;
				break;
			}
		}
	} else
		return PAGE_SELECT_LEN;
	return (retval == PAGE_SELECT_LEN) ? retval : -EIO;
}

 /**
 * synaptics_rmi4_i2c_read()
 *
 * Called by various functions in this driver, and also exported to
 * other expansion Function modules such as rmi_dev.
 *
 * This function reads data of an arbitrary length from the sensor,
 * starting from an assigned register address of the sensor, via I2C
 * with a retry mechanism.
 */
static int synaptics_rmi4_i2c_read(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char retry;
	unsigned char buf;
	struct i2c_msg msg[] = {
		{
			.addr = rmi4_data->i2c_client->addr,
			.flags = 0,
			.len = 1,
			.buf = &buf,
		},
		{
			.addr = rmi4_data->i2c_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		},
	};

	buf = addr & MASK_8BIT;

	mutex_lock(&(rmi4_data->rmi4_io_ctrl_mutex));

	retval = synaptics_rmi4_set_page(rmi4_data, addr);
    
	if (retval != PAGE_SELECT_LEN)
		goto exit;

	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(rmi4_data->i2c_client->adapter, msg, 2) == 2) {
			retval = length;
			break;
		}
		dev_err(&rmi4_data->i2c_client->dev,
				"tpd: %s: I2C retry %d\n",
				__func__, retry + 1);
		msleep(20);
	}

	if (retry == SYN_I2C_RETRY_TIMES) {
		dev_err(&rmi4_data->i2c_client->dev,
				"tpd: %s: I2C read over retry limit\n",
				__func__);
		retval = -EIO;
	}

exit:
	mutex_unlock(&(rmi4_data->rmi4_io_ctrl_mutex));

	return retval;
}

 /**
 * synaptics_rmi4_i2c_write()
 *
 * Called by various functions in this driver, and also exported to
 * other expansion Function modules such as rmi_dev.
 *
 * This function writes data of an arbitrary length to the sensor,
 * starting from an assigned register address of the sensor, via I2C with
 * a retry mechanism.
 */
static int synaptics_rmi4_i2c_write(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char retry;
	unsigned char buf[length + 1];
	struct i2c_msg msg[] = {
		{
			.addr = rmi4_data->i2c_client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	mutex_lock(&(rmi4_data->rmi4_io_ctrl_mutex));

	retval = synaptics_rmi4_set_page(rmi4_data, addr);
	if (retval != PAGE_SELECT_LEN)
		goto exit;

	buf[0] = addr & MASK_8BIT;
	memcpy(&buf[1], &data[0], length);

	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(rmi4_data->i2c_client->adapter, msg, 1) == 1) {
			retval = length;
			break;
		}
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: I2C retry %d\n",
				__func__, retry + 1);
		msleep(20);
	}

	if (retry == SYN_I2C_RETRY_TIMES) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: I2C write over retry limit\n",
				__func__);
		retval = -EIO;
	}

exit:
	mutex_unlock(&(rmi4_data->rmi4_io_ctrl_mutex));

	return retval;
}

 /**
 * synaptics_rmi4_f11_abs_report()
 *
 * Called by synaptics_rmi4_report_touch() when valid Function $11
 * finger data has been detected.
 *
 * This function reads the Function $11 data registers, determines the
 * status of each finger supported by the Function, processes any
 * necessary coordinate manipulation, reports the finger data to
 * the input subsystem, and returns the number of fingers detected.
 */
static int synaptics_rmi4_f11_abs_report(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler)
{
	int retval;
	unsigned char touch_count = 0; /* number of touch points */
	unsigned char reg_index;
	unsigned char finger;
	unsigned char fingers_supported;
	unsigned char num_of_finger_status_regs;
	unsigned char finger_shift;
	unsigned char finger_status;
	unsigned char data_reg_blk_size;
	unsigned char finger_status_reg[3];
	unsigned char data[F11_STD_DATA_LEN];
	unsigned short data_addr;
	unsigned short data_offset;
	int x;
	int y;
	int wx;
	int wy;
	int z;

	/*
	 * The number of finger status registers is determined by the
	 * maximum number of fingers supported - 2 bits per finger. So
	 * the number of finger status registers to read is:
	 * register_count = ceil(max_num_of_fingers / 4)
	 */
	fingers_supported = fhandler->num_of_data_points;
	num_of_finger_status_regs = (fingers_supported + 3) / 4;
	data_addr = fhandler->full_addr.data_base;
	data_reg_blk_size = fhandler->size_of_data_register_block;
       tp_max_finger = fingers_supported;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			data_addr,
			finger_status_reg,
			num_of_finger_status_regs);
	if (retval < 0)
		return 0;

	for (finger = 0; finger < fingers_supported; finger++) {
		reg_index = finger / 4;
		finger_shift = (finger % 4) * 2;
		finger_status = (finger_status_reg[reg_index] >> finger_shift)
				& MASK_2BIT;

		/*
		 * Each 2-bit finger status field represents the following:
		 * 00 = finger not present
		 * 01 = finger present and data accurate
		 * 10 = finger present but data may be inaccurate
		 * 11 = reserved
		 */
#ifdef TYPE_B_PROTOCOL
		input_mt_slot(rmi4_data->input_dev, finger);
		input_mt_report_slot_state(rmi4_data->input_dev,
				MT_TOOL_FINGER, finger_status != 0);
#endif

		if (finger_status) {
			data_offset = data_addr +
					num_of_finger_status_regs +
					(finger * data_reg_blk_size);
			retval = synaptics_rmi4_i2c_read(rmi4_data,
					data_offset,
					data,
					data_reg_blk_size);
			if (retval < 0)
				return 0;

			x = (data[0] << 4) | (data[2] & MASK_4BIT);
			y = (data[1] << 4) | ((data[2] >> 4) & MASK_4BIT);
			wx = (data[3] & MASK_4BIT);
			wy = (data[3] >> 4) & MASK_4BIT;
			z = data[4];

			if (rmi4_data->flip_x)
				x = rmi4_data->sensor_max_x - x;
			if (rmi4_data->flip_y)
				y = rmi4_data->sensor_max_y - y;

                    //x = (x * 857)/1000;
                    //y = (y * 8205)/10000;

		    ts_debug("tpd: %s: Finger %d, status = 0x%02x, x = %d, y = %d, wx = %d, wy = %d\n",
					__func__, finger, finger_status, x, y, wx, wy);

			input_report_abs(rmi4_data->input_dev,
					ABS_MT_POSITION_X, x);
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_POSITION_Y, y);
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_PRESSURE, z);

#ifdef REPORT_2D_W
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_TOUCH_MAJOR, max(wx, wy));
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_TOUCH_MINOR, min(wx, wy));
#endif
#ifndef TYPE_B_PROTOCOL
			input_mt_sync(rmi4_data->input_dev);
#endif
			touch_count++;
		}
	}

	input_report_key(rmi4_data->input_dev, BTN_TOUCH, touch_count > 0);
	input_report_key(rmi4_data->input_dev,
			BTN_TOOL_FINGER, touch_count > 0);

#ifndef TYPE_B_PROTOCOL
	if (!touch_count)
		input_mt_sync(rmi4_data->input_dev);
#else
	/* sync after groups of events */
	#ifdef KERNEL_ABOVE_3_7
	input_mt_sync_frame(rmi4_data->input_dev);
	#endif
#endif

	input_sync(rmi4_data->input_dev);

	return touch_count;
}

static void synaptics_rmi4_f1a_report(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler)
{
	int retval;
	unsigned char button;
	unsigned char index;
	unsigned char shift;
	unsigned char status;
	unsigned char *data;
	unsigned short data_addr = fhandler->full_addr.data_base;
	struct synaptics_rmi4_f1a_handle *f1a = fhandler->data;
	static unsigned char do_once = 1;
	static bool current_status[MAX_NUMBER_OF_BUTTONS];
#ifdef NO_0D_WHILE_2D
	static bool before_2d_status[MAX_NUMBER_OF_BUTTONS];
	static bool while_2d_status[MAX_NUMBER_OF_BUTTONS];
#endif

	if (do_once) {
		memset(current_status, 0, sizeof(current_status));
#ifdef NO_0D_WHILE_2D
		memset(before_2d_status, 0, sizeof(before_2d_status));
		memset(while_2d_status, 0, sizeof(while_2d_status));
#endif
		do_once = 0;
	}

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			data_addr,
			f1a->button_data_buffer,
			f1a->button_bitmask_size);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"tpd %s: Failed to read button data registers\n",
				__func__);
		return;
	}

	data = f1a->button_data_buffer;

	for (button = 0; button < f1a->valid_button_count; button++) {
		index = button / 8;
		shift = button % 8;
		status = ((data[index] >> shift) & MASK_1BIT);

		if (current_status[button] == status)
			continue;
		else
			current_status[button] = status;

		printk("wanglei tpd: %s: Button %d (code %d) ->%d\n",
				__func__, button,
				f1a->button_map[button],
				status);
#ifdef NO_0D_WHILE_2D
		if (rmi4_data->fingers_on_2d == false) {
			if (status == 1) {
				before_2d_status[button] = 1;
			} else {
				if (while_2d_status[button] == 1) {
					while_2d_status[button] = 0;
					continue;
				} else {
					before_2d_status[button] = 0;
				}
			}
			input_report_key(rmi4_data->input_dev,
					f1a->button_map[button],
					status);
		} else {
			if (before_2d_status[button] == 1) {
				before_2d_status[button] = 0;
				input_report_key(rmi4_data->input_dev,
						f1a->button_map[button],
						status);
			} else {
				if (status == 1)
					while_2d_status[button] = 1;
				else
					while_2d_status[button] = 0;
			}
		}
#else
		input_report_key(rmi4_data->input_dev,
				f1a->button_map[button],
				status);
#endif
	}

	input_sync(rmi4_data->input_dev);

	return;
}

 /**
 * synaptics_rmi4_report_touch()
 *
 * Called by synaptics_rmi4_sensor_report().
 *
 * This function calls the appropriate finger data reporting function
 * based on the function handler it receives and returns the number of
 * fingers detected.
 */
static void synaptics_rmi4_report_touch(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler,
		unsigned char *touch_count)
{
	unsigned char touch_count_2d;

	ts_debug("%s: Function %02x reporting\n",
			__func__, fhandler->fn_number);

	switch (fhandler->fn_number) {
	case SYNAPTICS_RMI4_F11:
		touch_count_2d = synaptics_rmi4_f11_abs_report(rmi4_data,
				fhandler);

		*touch_count += touch_count_2d;

		if (touch_count_2d)
			rmi4_data->fingers_on_2d = true;
		else
			rmi4_data->fingers_on_2d = false;
		break;

	case SYNAPTICS_RMI4_F1A:
		synaptics_rmi4_f1a_report(rmi4_data, fhandler);
		break;

	default:
		break;
	}

	return;
}

#ifdef DOUBLE_CLICK_WAKE
static void WakeUp_LCD(struct synaptics_rmi4_data *rmi4_data)
{
      input_report_key(rmi4_data->input_dev, KEY_F17, 1);
      input_sync(rmi4_data->input_dev);
      input_report_key(rmi4_data->input_dev, KEY_F17, 0);
      input_sync(rmi4_data->input_dev);
}

static void GestureU_work(struct synaptics_rmi4_data *rmi4_data, unsigned char gesture)
{
      switch (gesture) {
      case GESTURE_U_UP:
              input_report_key(rmi4_data->input_dev, KEY_F13, 1);
              input_sync(rmi4_data->input_dev);
              input_report_key(rmi4_data->input_dev, KEY_F13, 0);
              input_sync(rmi4_data->input_dev);
              return;
              break;
      case GESTURE_U_DOWN:
              input_report_key(rmi4_data->input_dev, KEY_F14, 1);
              input_sync(rmi4_data->input_dev);
              input_report_key(rmi4_data->input_dev, KEY_F14, 0);
              input_sync(rmi4_data->input_dev);
              return;
              break;
      case GESTURE_U_LEFT:
              input_report_key(rmi4_data->input_dev, KEY_F15, 1);
              input_sync(rmi4_data->input_dev);
              input_report_key(rmi4_data->input_dev, KEY_F15, 0);
              input_sync(rmi4_data->input_dev);
              return;
              break;
      case 64://case GESTURE_U_RIGHT:
              input_report_key(rmi4_data->input_dev, KEY_F16, 1);
              input_sync(rmi4_data->input_dev);
              input_report_key(rmi4_data->input_dev, KEY_F16, 0);
              input_sync(rmi4_data->input_dev);
              return;
              break;
      default:
              break;
      }
}
#endif


 /**
 * synaptics_rmi4_sensor_report()
 *
 * Called by synaptics_rmi4_irq().
 *
 * This function determines the interrupt source(s) from the sensor
 * and calls synaptics_rmi4_report_touch() with the appropriate
 * function handler for each function with valid data inputs.
 */
static int synaptics_rmi4_sensor_report(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char touch_count = 0;
	unsigned char intr[MAX_INTR_REGISTERS];
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_exp_fn *exp_fhandler;
	struct synaptics_rmi4_device_info *rmi;
#ifdef DOUBLE_CLICK_WAKE
       unsigned char double_wake_flag = 0;
#endif

#ifdef INIT_TP_WHEN_RESUME
       if (init_not_complete) {
            printk("wanglei: tpd sensor_report, but init_not_complete = 0, so return\n");
            return 0;
       }
#endif

	rmi = &(rmi4_data->rmi4_mod_info);

	/*
	 * Get interrupt status information from F01 Data1 register to
	 * determine the source(s) that are flagging the interrupt.
	 */
	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_data_base_addr + 1,
			intr,
			rmi4_data->num_of_intr_regs);
	if (retval < 0) {
              printk("wanglei tpd: i2c read err...\n");
		return retval;
       }

#ifdef DOUBLE_CLICK_WAKE
       if (wake_suspend_compelete) {
           if (wake_switch || gesture_switch) {      
               if (intr[0] == 0x04) {
                   retval = synaptics_rmi4_i2c_read(rmi4_data,
			    0x004B,
			    &double_wake_flag,
			    1);
                   ts_debug("double_wake_flag = %d\n", double_wake_flag);
                }
            }

            //double wake
            if (wake_switch) {
                if (double_wake_flag == 0x01) {
                    WakeUp_LCD(rmi4_data);
                    return 0;
                }
           }

           //gesture U
           if (gesture_switch) {
               if (double_wake_flag == 0x40) {
                   GestureU_work(rmi4_data, double_wake_flag);
                   return 0;
               }
           }
       }
#endif


	/*
	 * Traverse the function handler list and service the source(s)
	 * of the interrupt accordingly.
	 */
	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->num_of_data_sources) {
				if (fhandler->intr_mask &
						intr[fhandler->intr_reg_num]) {
					synaptics_rmi4_report_touch(rmi4_data,
							fhandler, &touch_count);
				}
			}
		}
	} 

#ifdef INIT_TP_WHEN_RESUME
       if (failed_init_flag)
           return touch_count;
#endif

	mutex_lock(&exp_fn_list_mutex);
	if (!list_empty(&exp_fn_list)) {
		list_for_each_entry(exp_fhandler, &exp_fn_list, link) {
			if (exp_fhandler->inserted &&
					(exp_fhandler->func_attn != NULL))
				exp_fhandler->func_attn(rmi4_data, intr[0]);
		}
	}
	mutex_unlock(&exp_fn_list_mutex);

	return touch_count;
}

 /**
 * synaptics_rmi4_irq()
 *
 * Called by the kernel when an interrupt occurs (when the sensor
 * asserts the attention irq).
 *
 * This function is the ISR thread and handles the acquisition
 * and the reporting of finger data when the presence of fingers
 * is detected.
 */
static irqreturn_t synaptics_rmi4_irq(int irq, void *data)
{
	struct synaptics_rmi4_data *rmi4_data = data;

	synaptics_rmi4_sensor_report(rmi4_data);

	return IRQ_HANDLED;
}

static int synaptics_rmi4_parse_dt(struct device *dev,
				struct synaptics_rmi4_platform_data *rmi4_pdata)
{
	struct device_node *np = dev->of_node;
	struct property *prop;
	u32 temp_val, num_buttons;
	u32 button_map[MAX_NUMBER_OF_BUTTONS];
	int rc, i;

	rmi4_pdata->i2c_pull_up = of_property_read_bool(np,
			"synaptics,i2c-pull-up");
	rmi4_pdata->regulator_en = of_property_read_bool(np,
			"synaptics,reg-en");
	rmi4_pdata->x_flip = of_property_read_bool(np, "synaptics,x-flip");
	rmi4_pdata->y_flip = of_property_read_bool(np, "synaptics,y-flip");

	rc = of_property_read_u32(np, "synaptics,panel-x", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "tpd: Unable to read panel X dimension\n");
		return rc;
	} else {
		rmi4_pdata->panel_x = temp_val;
	}

	rc = of_property_read_u32(np, "synaptics,panel-y", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "tpd: Unable to read panel Y dimension\n");
		return rc;
	} else {
		rmi4_pdata->panel_y = temp_val;
	}

	rc = of_property_read_string(np, "synaptics,fw-image-name1",
		&rmi4_pdata->fw_image_name1);
       rc = of_property_read_string(np, "synaptics,fw-image-name2",
		&rmi4_pdata->fw_image_name2);
       rc = of_property_read_string(np, "synaptics,fw-image-name3",
		&rmi4_pdata->fw_image_name3);
       rc = of_property_read_string(np, "synaptics,fw-image-name4",
		&rmi4_pdata->fw_image_name4);
       rc = of_property_read_string(np, "synaptics,fw-image-name5",
		&rmi4_pdata->fw_image_name5);
       rc = of_property_read_string(np, "synaptics,fw-image-name6",
		&rmi4_pdata->fw_image_name6);
       rc = of_property_read_string(np, "synaptics,fw-image-name7",
		&rmi4_pdata->fw_image_name7);
       rc = of_property_read_string(np, "synaptics,fw-image-name8",
		&rmi4_pdata->fw_image_name8);
       rc = of_property_read_string(np, "synaptics,fw-image-name9",
		&rmi4_pdata->fw_image_name9);
       rc = of_property_read_string(np, "synaptics,fw-image-name10",
		&rmi4_pdata->fw_image_name10);
       rc = of_property_read_string(np, "synaptics,fw-image-name11",
		&rmi4_pdata->fw_image_name11);

       printk("tpd: rmi4_pdata->fw_image_name1 = %s\n", rmi4_pdata->fw_image_name1);
       printk("tpd: rmi4_pdata->fw_image_name2 = %s\n", rmi4_pdata->fw_image_name2);
       printk("tpd: rmi4_pdata->fw_image_name3 = %s\n", rmi4_pdata->fw_image_name3);
       printk("tpd: rmi4_pdata->fw_image_name4 = %s\n", rmi4_pdata->fw_image_name4);
       printk("tpd: rmi4_pdata->fw_image_name5 = %s\n", rmi4_pdata->fw_image_name5);
       printk("tpd: rmi4_pdata->fw_image_name6 = %s\n", rmi4_pdata->fw_image_name6);
       printk("tpd: rmi4_pdata->fw_image_name7 = %s\n", rmi4_pdata->fw_image_name7);
       printk("tpd: rmi4_pdata->fw_image_name8 = %s\n", rmi4_pdata->fw_image_name8);
       printk("tpd: rmi4_pdata->fw_image_name9 = %s\n", rmi4_pdata->fw_image_name9);
       printk("tpd: rmi4_pdata->fw_image_name10 = %s\n", rmi4_pdata->fw_image_name10);
       printk("tpd: rmi4_pdata->fw_image_name11 = %s\n", rmi4_pdata->fw_image_name11);
       
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "tpd: Unable to read fw image name\n");
		return rc;
	}

	/* reset, irq gpio info */
	rmi4_pdata->reset_gpio = of_get_named_gpio_flags(np,
			"synaptics,reset-gpio", 0, &rmi4_pdata->reset_flags);
	rmi4_pdata->irq_gpio = of_get_named_gpio_flags(np,
			"synaptics,irq-gpio", 0, &rmi4_pdata->irq_flags);
    rmi4_pdata->en_gpio = of_get_named_gpio_flags(np,
			"synaptics,en-gpio", 0, &rmi4_pdata->irq_flags);

	prop = of_find_property(np, "synaptics,button-map", NULL);
	if (prop) {
		num_buttons = prop->length / sizeof(temp_val);

              ts_debug("parse_dt --> num_buttons = %d\n", num_buttons);

		rmi4_pdata->capacitance_button_map = devm_kzalloc(dev,
			sizeof(*rmi4_pdata->capacitance_button_map),
			GFP_KERNEL);
		if (!rmi4_pdata->capacitance_button_map)
			return -ENOMEM;

		rmi4_pdata->capacitance_button_map->map = devm_kzalloc(dev,
			sizeof(*rmi4_pdata->capacitance_button_map->map) *
			MAX_NUMBER_OF_BUTTONS, GFP_KERNEL);
		if (!rmi4_pdata->capacitance_button_map->map)
			return -ENOMEM;

		if (num_buttons <= MAX_NUMBER_OF_BUTTONS) {
			rc = of_property_read_u32_array(np,
				"synaptics,button-map", button_map,
				num_buttons);
			if (rc) {
				dev_err(dev, "tpd: Unable to read key codes\n");
				return rc;
			}
			for (i = 0; i < num_buttons; i++)
				rmi4_pdata->capacitance_button_map->map[i] =
					button_map[i];
			rmi4_pdata->capacitance_button_map->nbuttons =
				num_buttons;
		} else {
			return -EINVAL;
		}
	}
	return 0;
}

 /**
 * synaptics_rmi4_irq_enable()
 *
 * Called by synaptics_rmi4_probe() and the power management functions
 * in this driver and also exported to other expansion Function modules
 * such as rmi_dev.
 *
 * This function handles the enabling and disabling of the attention
 * irq including the setting up of the ISR thread.
 */
static int synaptics_rmi4_irq_enable(struct synaptics_rmi4_data *rmi4_data,
		bool enable)
{
	int retval = 0;
	unsigned char *intr_status;

	if (enable) {
		if (rmi4_data->irq_enabled)
			return retval;

		intr_status = kzalloc(rmi4_data->num_of_intr_regs, GFP_KERNEL);
		if (!intr_status) {
			dev_err(&rmi4_data->i2c_client->dev,
					"%s: Failed to alloc memory\n",
					__func__);
			return -ENOMEM;
		}
		/* Clear interrupts first */
		retval = synaptics_rmi4_i2c_read(rmi4_data,
				rmi4_data->f01_data_base_addr + 1,
				intr_status,
				rmi4_data->num_of_intr_regs);
		kfree(intr_status);
		if (retval < 0)
			return retval;

		enable_irq(rmi4_data->irq);
		rmi4_data->irq_enabled = true;
	} else {
		if (rmi4_data->irq_enabled) {
			disable_irq(rmi4_data->irq);
			rmi4_data->irq_enabled = false;
		}
	}

	return retval;
}

 /**
 * synaptics_rmi4_f11_init()
 *
 * Called by synaptics_rmi4_query_device().
 *
 * This funtion parses information from the Function 11 registers
 * and determines the number of fingers supported, x and y data ranges,
 * offset to the associated interrupt status register, interrupt bit
 * mask, and gathers finger data acquisition capabilities from the query
 * registers.
 */
static int synaptics_rmi4_f11_init(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count)
{
	int retval;
	unsigned char ii;
	unsigned char intr_offset;
	unsigned char abs_data_size;
	unsigned char abs_data_blk_size;
	unsigned char query[F11_STD_QUERY_LEN];
	unsigned char control[F11_STD_CTRL_LEN];

	fhandler->fn_number = fd->fn_number;
	fhandler->num_of_data_sources = fd->intr_src_count;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.query_base,
			query,
			sizeof(query));
	if (retval < 0)
		return retval;

	/* Maximum number of fingers supported */
	if ((query[1] & MASK_3BIT) <= 4)
		fhandler->num_of_data_points = (query[1] & MASK_3BIT) + 1;
	else if ((query[1] & MASK_3BIT) == 5)
		fhandler->num_of_data_points = 10;

	rmi4_data->num_of_fingers = fhandler->num_of_data_points;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.ctrl_base,
			control,
			sizeof(control));
	if (retval < 0)
		return retval;

	/* Maximum x and y */
	rmi4_data->sensor_max_x = ((control[6] & MASK_8BIT) << 0) |
			((control[7] & MASK_4BIT) << 8);
	rmi4_data->sensor_max_y = ((control[8] & MASK_8BIT) << 0) |
			((control[9] & MASK_4BIT) << 8);
	ts_debug("%s: Function %02x max x = %d max y = %d\n",
			__func__, fhandler->fn_number,
			rmi4_data->sensor_max_x,
			rmi4_data->sensor_max_y);

	fhandler->intr_reg_num = (intr_count + 7) / 8;
	if (fhandler->intr_reg_num != 0)
		fhandler->intr_reg_num -= 1;

	/* Set an enable bit for each data source */
	intr_offset = intr_count % 8;
	fhandler->intr_mask = 0;
	for (ii = intr_offset;
			ii < ((fd->intr_src_count & MASK_3BIT) +
			intr_offset);
			ii++)
		fhandler->intr_mask |= 1 << ii;

	abs_data_size = query[5] & MASK_2BIT;
	abs_data_blk_size = 3 + (2 * (abs_data_size == 0 ? 1 : 0));
	fhandler->size_of_data_register_block = abs_data_blk_size;

	return retval;
}

static int synaptics_rmi4_f1a_alloc_mem(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler)
{
	int retval;
	struct synaptics_rmi4_f1a_handle *f1a;

	f1a = kzalloc(sizeof(*f1a), GFP_KERNEL);
	if (!f1a) {
		dev_err(&rmi4_data->i2c_client->dev,
				"tpd %s: Failed to alloc mem for function handle\n",
				__func__);
		return -ENOMEM;
	}

	fhandler->data = (void *)f1a;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.query_base,
			f1a->button_query.data,
			sizeof(f1a->button_query.data));
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"tpd %s: Failed to read query registers\n",
				__func__);
		return retval;
	}

	f1a->button_count = f1a->button_query.max_button_count + 1;
	f1a->button_bitmask_size = (f1a->button_count + 7) / 8;

	f1a->button_data_buffer = kcalloc(f1a->button_bitmask_size,
			sizeof(*(f1a->button_data_buffer)), GFP_KERNEL);
	if (!f1a->button_data_buffer) {
		dev_err(&rmi4_data->i2c_client->dev,
				"tpd %s: Failed to alloc mem for data buffer\n",
				__func__);
		return -ENOMEM;
	}

	f1a->button_map = kcalloc(f1a->button_count,
			sizeof(*(f1a->button_map)), GFP_KERNEL);
	if (!f1a->button_map) {
		dev_err(&rmi4_data->i2c_client->dev,
				"tpd %s: Failed to alloc mem for button map\n",
				__func__);
		return -ENOMEM;
	}

	return 0;
}

static int synaptics_rmi4_capacitance_button_map(
				struct synaptics_rmi4_data *rmi4_data,
				struct synaptics_rmi4_fn *fhandler)
{
	unsigned char ii;
	struct synaptics_rmi4_f1a_handle *f1a = fhandler->data;
	const struct synaptics_rmi4_platform_data *pdata = rmi4_data->board;

	if (!pdata->capacitance_button_map) {
		dev_err(&rmi4_data->i2c_client->dev,
				"tpd %s: capacitance_button_map is" \
				"NULL in board file\n",
				__func__);
		return -ENODEV;
	} else if (!pdata->capacitance_button_map->map) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Button map is missing in board file\n",
				__func__);
		return -ENODEV;
	} else {
		if (pdata->capacitance_button_map->nbuttons !=
			f1a->button_count) {
			f1a->valid_button_count = min(f1a->button_count,
				pdata->capacitance_button_map->nbuttons);
		} else {
			f1a->valid_button_count = f1a->button_count;
		}

		for (ii = 0; ii < f1a->valid_button_count; ii++)
			f1a->button_map[ii] =
					pdata->capacitance_button_map->map[ii];
	}

	return 0;
}

static void synaptics_rmi4_f1a_kfree(struct synaptics_rmi4_fn *fhandler)
{
	struct synaptics_rmi4_f1a_handle *f1a = fhandler->data;

	if (f1a) {
		kfree(f1a->button_data_buffer);
		kfree(f1a->button_map);
		kfree(f1a);
		fhandler->data = NULL;
	}

	return;
}

static int synaptics_rmi4_f1a_init(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count)
{
	int retval;
	unsigned char ii;
	unsigned short intr_offset;

	fhandler->fn_number = fd->fn_number;
	fhandler->num_of_data_sources = fd->intr_src_count;

	fhandler->intr_reg_num = (intr_count + 7) / 8;
	if (fhandler->intr_reg_num != 0)
		fhandler->intr_reg_num -= 1;

	/* Set an enable bit for each data source */
	intr_offset = intr_count % 8;
	fhandler->intr_mask = 0;
	for (ii = intr_offset;
			ii < ((fd->intr_src_count & MASK_3BIT) +
			intr_offset);
			ii++)
		fhandler->intr_mask |= 1 << ii;

	retval = synaptics_rmi4_f1a_alloc_mem(rmi4_data, fhandler);
	if (retval < 0)
		goto error_exit;

	retval = synaptics_rmi4_capacitance_button_map(rmi4_data, fhandler);
	if (retval < 0)
		goto error_exit;

	rmi4_data->button_0d_enabled = 1;

	return 0;

error_exit:
	synaptics_rmi4_f1a_kfree(fhandler);

	return retval;
}

static int synaptics_rmi4_alloc_fh(struct synaptics_rmi4_fn **fhandler,
		struct synaptics_rmi4_fn_desc *rmi_fd, int page_number)
{
	*fhandler = kzalloc(sizeof(**fhandler), GFP_KERNEL);
	if (!(*fhandler))
		return -ENOMEM;

	(*fhandler)->full_addr.data_base =
			(rmi_fd->data_base_addr |
			(page_number << 8));
	(*fhandler)->full_addr.ctrl_base =
			(rmi_fd->ctrl_base_addr |
			(page_number << 8));
	(*fhandler)->full_addr.cmd_base =
			(rmi_fd->cmd_base_addr |
			(page_number << 8));
	(*fhandler)->full_addr.query_base =
			(rmi_fd->query_base_addr |
			(page_number << 8));
	(*fhandler)->fn_number = rmi_fd->fn_number;

	return 0;
}


 /**
 * synaptics_rmi4_query_device_info()
 *
 * Called by synaptics_rmi4_query_device().
 *
 */
static int synaptics_rmi4_query_device_info(
					struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char f01_query[F01_STD_QUERY_LEN];
	struct synaptics_rmi4_device_info *rmi = &(rmi4_data->rmi4_mod_info);

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_query_base_addr,
			f01_query,
			sizeof(f01_query));
	if (retval < 0)
		return retval;

	/* RMI Version 4.0 currently supported */
	rmi->version_major = 4;
	rmi->version_minor = 0;

	rmi->manufacturer_id = f01_query[0];
	rmi->product_props = f01_query[1];
	rmi->product_info[0] = f01_query[2] & MASK_7BIT;
	rmi->product_info[1] = f01_query[3] & MASK_7BIT;
	rmi->date_code[0] = f01_query[4] & MASK_5BIT;
	rmi->date_code[1] = f01_query[5] & MASK_4BIT;
	rmi->date_code[2] = f01_query[6] & MASK_5BIT;
	rmi->tester_id = ((f01_query[7] & MASK_7BIT) << 8) |
			(f01_query[8] & MASK_7BIT);
	rmi->serial_number = ((f01_query[9] & MASK_7BIT) << 8) |
			(f01_query[10] & MASK_7BIT);
	memcpy(rmi->product_id_string, &f01_query[11], 10);

	if (rmi->manufacturer_id != 1) {
		dev_err(&rmi4_data->i2c_client->dev,
				"tpd: %s: Non-Synaptics device found, manufacturer ID = %d\n",
				__func__, rmi->manufacturer_id);
	}

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_query_base_addr + F01_BUID_ID_OFFSET,
			rmi->build_id,
			sizeof(rmi->build_id));
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to read firmware build id (code %d)\n",
				__func__, retval);
		return retval;
	}
	return 0;
}

 /**
 * synaptics_rmi4_query_device()
 *
 * Called by synaptics_rmi4_probe().
 *
 * This funtion scans the page description table, records the offsets
 * to the register types of Function $01, sets up the function handlers
 * for Function $11 and Function $12, determines the number of interrupt
 * sources from the sensor, adds valid Functions with data inputs to the
 * Function linked list, parses information from the query registers of
 * Function $01, and enables the interrupt sources from the valid Functions
 * with data inputs.
 */
static int synaptics_rmi4_query_device(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char ii;
	unsigned char page_number;
	unsigned char intr_count = 0;
	unsigned char data_sources = 0;
	unsigned short pdt_entry_addr;
	unsigned short intr_addr;
#if defined(CONFIG_GN_DEVICE_TYPE_CHECK) 
       unsigned char product_id[10] = {0};
       unsigned char firmware_id[10] = {0};
#endif
	struct synaptics_rmi4_f01_device_status status;
	struct synaptics_rmi4_fn_desc rmi_fd;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	INIT_LIST_HEAD(&rmi->support_fn_list);

	/* Scan the page description tables of the pages to service */
	for (page_number = 0; page_number < PAGES_TO_SERVICE; page_number++) {
		for (pdt_entry_addr = PDT_START; pdt_entry_addr > PDT_END;
				pdt_entry_addr -= PDT_ENTRY_SIZE) {
			pdt_entry_addr |= (page_number << 8);

			retval = synaptics_rmi4_i2c_read(rmi4_data,
					pdt_entry_addr,
					(unsigned char *)&rmi_fd,
					sizeof(rmi_fd));
			if (retval < 0)
				return retval;

			fhandler = NULL;

			if (rmi_fd.fn_number == 0) {
				ts_debug("%s: Reached end of PDT\n",
						__func__);
				break;
			}

			ts_debug("%s: F%02x found (page %d)\n",
					__func__, rmi_fd.fn_number,
					page_number);

			switch (rmi_fd.fn_number) {
			case SYNAPTICS_RMI4_F01:
				rmi4_data->f01_query_base_addr =
						rmi_fd.query_base_addr;
				rmi4_data->f01_ctrl_base_addr =
						rmi_fd.ctrl_base_addr;
				rmi4_data->f01_data_base_addr =
						rmi_fd.data_base_addr;
				rmi4_data->f01_cmd_base_addr =
						rmi_fd.cmd_base_addr;

				retval =
				synaptics_rmi4_query_device_info(rmi4_data);
				if (retval < 0)
					return retval;

				retval = synaptics_rmi4_i2c_read(rmi4_data,
						rmi4_data->f01_data_base_addr,
						status.data,
						sizeof(status.data));
				if (retval < 0)
					return retval;

				while (status.status_code == STATUS_CRC_IN_PROGRESS) {
					msleep(1);
					retval = synaptics_rmi4_i2c_read(rmi4_data,
						rmi4_data->f01_data_base_addr,
						status.data,
						sizeof(status.data));
					if (retval < 0)
						return retval;
				}

				if (status.flash_prog == 1) {
					pr_notice("tpd: %s: In flash prog mode, status = 0x%02x\n",
							__func__,
							status.status_code);
					goto flash_prog_mode;
				}
				break;

			case SYNAPTICS_RMI4_F11:
				if (rmi_fd.intr_src_count == 0)
					break;

				retval = synaptics_rmi4_alloc_fh(&fhandler,
						&rmi_fd, page_number);
				if (retval < 0) {
					dev_err(&rmi4_data->i2c_client->dev,
							"%s: Failed to alloc for F%d\n",
							__func__,
							rmi_fd.fn_number);
					return retval;
				}

				retval = synaptics_rmi4_f11_init(rmi4_data,
						fhandler, &rmi_fd, intr_count);
				if (retval < 0)
					return retval;
				break;

			case SYNAPTICS_RMI4_F1A:
				if (rmi_fd.intr_src_count == 0)
					break;

				retval = synaptics_rmi4_alloc_fh(&fhandler,
						&rmi_fd, page_number);
				if (retval < 0) {
					dev_err(&rmi4_data->i2c_client->dev,
							"tpd %s: Failed to alloc for F%d\n",
							__func__,
							rmi_fd.fn_number);
					return retval;
				}

				retval = synaptics_rmi4_f1a_init(rmi4_data,
						fhandler, &rmi_fd, intr_count);
				if (retval < 0)
					return retval;
				break;
			}

			/* Accumulate the interrupt count */
			intr_count += (rmi_fd.intr_src_count & MASK_3BIT);

			if (fhandler && rmi_fd.intr_src_count) {
				list_add_tail(&fhandler->link,
						&rmi->support_fn_list);
			}
		}
	}

#if defined(CONFIG_GN_DEVICE_TYPE_CHECK) 
       //Read product ID
       retval = synaptics_rmi4_i2c_read(rmi4_data, rmi4_data->f01_query_base_addr + 11, product_id, 2);
       if (retval < 0) {
           printk("wanglei tpd: read product id failed...\n");
       }else {
           printk("wanglei tpd: Product id is %s\n", product_id);
       }

       //Read firmware ID
       retval = synaptics_rmi4_i2c_read(rmi4_data, 0x4d, firmware_id, 4);
       if (retval < 0) {
           printk("wanglei tpd: read firmware id failed...\n");
       }else {
           printk("wanglei tpd: Firmware id is 0x%02x 0x%02x 0x%02x 0x%02x\n", firmware_id[0], firmware_id[1], firmware_id[2], firmware_id[3]);
       }
       
       snprintf(gn_mydev_info.name, 10, "%s, %02x%02x", product_id, firmware_id[2], firmware_id[3]);
       gn_set_device_info(gn_mydev_info);
#endif

flash_prog_mode:
	rmi4_data->num_of_intr_regs = (intr_count + 7) / 8;
	ts_debug("%s: Number of interrupt registers = %d\n",
			__func__, rmi4_data->num_of_intr_regs);

	memset(rmi4_data->intr_mask, 0x00, sizeof(rmi4_data->intr_mask));

	/*
	 * Map out the interrupt bit masks for the interrupt sources
	 * from the registered function handlers.
	 */
	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link)
			data_sources += fhandler->num_of_data_sources;
	}
	if (data_sources) {
		if (!list_empty(&rmi->support_fn_list)) {
			list_for_each_entry(fhandler,
						&rmi->support_fn_list, link) {
				if (fhandler->num_of_data_sources) {
					rmi4_data->intr_mask[fhandler->intr_reg_num] |=
							fhandler->intr_mask;
				}
			}
		}
	}

	/* Enable the interrupt sources */
	for (ii = 0; ii < rmi4_data->num_of_intr_regs; ii++) {
		if (rmi4_data->intr_mask[ii] != 0x00) {
			ts_debug("%s: Interrupt enable mask %d = 0x%02x\n",
					__func__, ii, rmi4_data->intr_mask[ii]);
			intr_addr = rmi4_data->f01_ctrl_base_addr + 1 + ii;
			retval = synaptics_rmi4_i2c_write(rmi4_data,
					intr_addr,
					&(rmi4_data->intr_mask[ii]),
					sizeof(rmi4_data->intr_mask[ii]));
			if (retval < 0)
				return retval;
		}
	}

	return 0;
}

static int synaptics_rmi4_reset_command(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	int page_number;
	unsigned char command = 0x01;
	unsigned short pdt_entry_addr;
	struct synaptics_rmi4_fn_desc rmi_fd;
	bool done = false;

	/* Scan the page description tables of the pages to service */
	for (page_number = 0; page_number < PAGES_TO_SERVICE; page_number++) {
		for (pdt_entry_addr = PDT_START; pdt_entry_addr > PDT_END;
				pdt_entry_addr -= PDT_ENTRY_SIZE) {
			retval = synaptics_rmi4_i2c_read(rmi4_data,
				pdt_entry_addr,
				(unsigned char *)&rmi_fd,
				sizeof(rmi_fd));
			if (retval < 0)
				return retval;

			if (rmi_fd.fn_number == 0)
				break;

			switch (rmi_fd.fn_number) {
			case SYNAPTICS_RMI4_F01:
				rmi4_data->f01_cmd_base_addr =
					rmi_fd.cmd_base_addr;
				done = true;
				break;
			}
		}
		if (done) {
			dev_info(&rmi4_data->i2c_client->dev,
				"wanglei tpd %s: Find F01 in page description table 0x%x\n",
				__func__, rmi4_data->f01_cmd_base_addr);
			break;
		}
	}

	if (!done) {
		dev_err(&rmi4_data->i2c_client->dev,
			"wanglei tpd %s: Cannot find F01 in page description table\n",
			__func__);
		return -EINVAL;
	}

	retval = synaptics_rmi4_i2c_write(rmi4_data,
			rmi4_data->f01_cmd_base_addr,
			&command,
			sizeof(command));
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"wanglei tpd %s: Failed to issue reset command, error = %d\n",
				__func__, retval);
		return retval;
	}

	msleep(RESET_DELAY);
	return retval;
};

static int synaptics_rmi4_reset_device(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	retval = synaptics_rmi4_reset_command(rmi4_data);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
			"%s: Failed to send command reset\n",
			__func__);
		return retval;
	}

	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->fn_number == SYNAPTICS_RMI4_F1A)
				synaptics_rmi4_f1a_kfree(fhandler);
			else
				kfree(fhandler->data);
			kfree(fhandler);
		}
	}

	retval = synaptics_rmi4_query_device(rmi4_data);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"wanglei --> tpd %s: Failed to query device\n",
				__func__);
		return retval;
	}

	return 0;
}

/**
* synaptics_rmi4_detection_work()
*
* Called by the kernel at the scheduled time.
*
* This function is a self-rearming work thread that checks for the
* insertion and removal of other expansion Function modules such as
* rmi_dev and calls their initialization and removal callback functions
* accordingly.
*/
static void synaptics_rmi4_detection_work(struct work_struct *work)
{
	struct synaptics_rmi4_exp_fn *exp_fhandler, *next_list_entry;
	struct synaptics_rmi4_data *rmi4_data =
			container_of(work, struct synaptics_rmi4_data,
			det_work.work);

	mutex_lock(&exp_fn_list_mutex);
	if (!list_empty(&exp_fn_list)) {
		list_for_each_entry_safe(exp_fhandler,
				next_list_entry,
				&exp_fn_list,
				link) {
			if ((exp_fhandler->func_init != NULL) &&
					(exp_fhandler->inserted == false)) {
				exp_fhandler->func_init(rmi4_data);
				exp_fhandler->inserted = true;
			} else if ((exp_fhandler->func_init == NULL) &&
					(exp_fhandler->inserted == true)) {
				exp_fhandler->func_remove(rmi4_data);
				list_del(&exp_fhandler->link);
				kfree(exp_fhandler);
			}
		}
	}
	mutex_unlock(&exp_fn_list_mutex);

	return;
}

/**
* synaptics_rmi4_new_function()
*
* Called by other expansion Function modules in their module init and
* module exit functions.
*
* This function is used by other expansion Function modules such as
* rmi_dev to register themselves with the driver by providing their
* initialization and removal callback function pointers so that they
* can be inserted or removed dynamically at module init and exit times,
* respectively.
*/
void synaptics_rmi4_new_function(enum exp_fn fn_type, bool insert,
		int (*func_init)(struct synaptics_rmi4_data *rmi4_data),
		void (*func_remove)(struct synaptics_rmi4_data *rmi4_data),
		void (*func_attn)(struct synaptics_rmi4_data *rmi4_data,
		unsigned char intr_mask))
{
	struct synaptics_rmi4_exp_fn *exp_fhandler;

	if (!exp_fn_inited) {
		mutex_init(&exp_fn_list_mutex);
		INIT_LIST_HEAD(&exp_fn_list);
		exp_fn_inited = 1;
	}

	mutex_lock(&exp_fn_list_mutex);
	if (insert) {
		exp_fhandler = kzalloc(sizeof(*exp_fhandler), GFP_KERNEL);
		if (!exp_fhandler) {
			pr_err("%s: Failed to alloc mem for expansion function\n",
					__func__);
			goto exit;
		}
		exp_fhandler->fn_type = fn_type;
		exp_fhandler->func_init = func_init;
		exp_fhandler->func_attn = func_attn;
		exp_fhandler->func_remove = func_remove;
		exp_fhandler->inserted = false;
		list_add_tail(&exp_fhandler->link, &exp_fn_list);
	} else {
		if (!list_empty(&exp_fn_list)) {
			list_for_each_entry(exp_fhandler, &exp_fn_list, link) {
				if (exp_fhandler->func_init == func_init) {
					exp_fhandler->inserted = false;
					exp_fhandler->func_init = NULL;
					exp_fhandler->func_attn = NULL;
					goto exit;
				}
			}
		}
	}

exit:
	mutex_unlock(&exp_fn_list_mutex);

	return;
}
EXPORT_SYMBOL(synaptics_rmi4_new_function);


static int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
	return (regulator_count_voltages(reg) > 0) ?
		regulator_set_optimum_mode(reg, load_uA) : 0;
}

static int synaptics_rmi4_regulator_configure(struct synaptics_rmi4_data
						*rmi4_data, bool on)
{
	int retval;

	if (on == false)
		goto hw_shutdown;

	if (rmi4_data->board->regulator_en) {
		rmi4_data->vdd = regulator_get(&rmi4_data->i2c_client->dev,
						"vdd");
		if (IS_ERR(rmi4_data->vdd)) {
			dev_err(&rmi4_data->i2c_client->dev,
					"tpd: %s: Failed to get vdd regulator\n",
					__func__);
			return PTR_ERR(rmi4_data->vdd);
		}

		if (regulator_count_voltages(rmi4_data->vdd) > 0) {
			retval = regulator_set_voltage(rmi4_data->vdd,
				RMI4_VTG_MIN_UV, RMI4_VTG_MAX_UV);
			if (retval) {
				dev_err(&rmi4_data->i2c_client->dev,
					"tpd: regulator set_vtg failed retval =%d\n",
					retval);
				goto err_set_vtg_vdd;
			}
		}
	}

	if (rmi4_data->board->i2c_pull_up) {
		rmi4_data->vcc_i2c = regulator_get(&rmi4_data->i2c_client->dev,
						"vcc_i2c");
		if (IS_ERR(rmi4_data->vcc_i2c)) {
			dev_err(&rmi4_data->i2c_client->dev,
					"tpd: %s: Failed to get i2c regulator\n",
					__func__);
			retval = PTR_ERR(rmi4_data->vcc_i2c);
			goto err_get_vtg_i2c;
		}

		if (regulator_count_voltages(rmi4_data->vcc_i2c) > 0) {
			retval = regulator_set_voltage(rmi4_data->vcc_i2c,
				RMI4_I2C_VTG_MIN_UV, RMI4_I2C_VTG_MAX_UV);
			if (retval) {
				dev_err(&rmi4_data->i2c_client->dev,
					"tpd: reg set i2c vtg failed retval =%d\n",
					retval);
			goto err_set_vtg_i2c;
			}
		}
	}
	return 0;

err_set_vtg_i2c:
	if (rmi4_data->board->i2c_pull_up)
		regulator_put(rmi4_data->vcc_i2c);
err_get_vtg_i2c:
	if (rmi4_data->board->regulator_en)
		if (regulator_count_voltages(rmi4_data->vdd) > 0)
			regulator_set_voltage(rmi4_data->vdd, 0,
				RMI4_VTG_MAX_UV);
err_set_vtg_vdd:
	if (rmi4_data->board->regulator_en)
		regulator_put(rmi4_data->vdd);
	return retval;

hw_shutdown:
	if (rmi4_data->board->regulator_en) {
		if (regulator_count_voltages(rmi4_data->vdd) > 0)
			regulator_set_voltage(rmi4_data->vdd, 0,
				RMI4_VTG_MAX_UV);
		regulator_put(rmi4_data->vdd);
	}
	if (rmi4_data->board->i2c_pull_up) {
		if (regulator_count_voltages(rmi4_data->vcc_i2c) > 0)
			regulator_set_voltage(rmi4_data->vcc_i2c, 0,
					RMI4_I2C_VTG_MAX_UV);
		regulator_put(rmi4_data->vcc_i2c);
	}
	return 0;
};

static int synaptics_rmi4_power_on(struct synaptics_rmi4_data *rmi4_data,
					bool on) {
	int retval;

	if (on == false)
		goto power_off;

	if (rmi4_data->board->regulator_en) {
		retval = reg_set_optimum_mode_check(rmi4_data->vdd,
			RMI4_ACTIVE_LOAD_UA);
		if (retval < 0) {
			dev_err(&rmi4_data->i2c_client->dev,
				"tpd: Regulator vdd set_opt failed rc=%d\n",
				retval);
			return retval;
		}

		retval = regulator_enable(rmi4_data->vdd);
		if (retval) {
			dev_err(&rmi4_data->i2c_client->dev,
				"tpd: Regulator vdd enable failed rc=%d\n",
				retval);
			goto error_reg_en_vdd;
		}
	}

	if (rmi4_data->board->i2c_pull_up) {
		retval = reg_set_optimum_mode_check(rmi4_data->vcc_i2c,
			RMI4_I2C_LOAD_UA);
		if (retval < 0) {
			dev_err(&rmi4_data->i2c_client->dev,
				"tpd: Regulator vcc_i2c set_opt failed rc=%d\n",
				retval);
			goto error_reg_opt_i2c;
		}

		retval = regulator_enable(rmi4_data->vcc_i2c);
		if (retval) {
			dev_err(&rmi4_data->i2c_client->dev,
				"tpd: Regulator vcc_i2c enable failed rc=%d\n",
				retval);
			goto error_reg_en_vcc_i2c;
		}
	}
	return 0;

error_reg_en_vcc_i2c:
	if (rmi4_data->board->i2c_pull_up)
		reg_set_optimum_mode_check(rmi4_data->vdd, 0);
error_reg_opt_i2c:
	if (rmi4_data->board->regulator_en)
		regulator_disable(rmi4_data->vdd);
error_reg_en_vdd:
	if (rmi4_data->board->regulator_en)
		reg_set_optimum_mode_check(rmi4_data->vdd, 0);
	return retval;

power_off:
	if (rmi4_data->board->regulator_en) {
		reg_set_optimum_mode_check(rmi4_data->vdd, 0);
		regulator_disable(rmi4_data->vdd);
	}
	if (rmi4_data->board->i2c_pull_up) {
		reg_set_optimum_mode_check(rmi4_data->vcc_i2c, 0);
		regulator_disable(rmi4_data->vcc_i2c);
	}
	return 0;
}

#ifdef INIT_TP_WHEN_RESUME
static void gn_resume_init(struct synaptics_rmi4_data *rmi4_data)
{
       struct synaptics_rmi4_f1a_handle *f1a;
	struct synaptics_rmi4_fn *fhandler;
       struct synaptics_rmi4_device_info *rmi;
       unsigned char ii;
       int retval = 0;
       unsigned char attr_count;

       rmi = &(rmi4_data->rmi4_mod_info);

       printk("tpd wanglei: init_not_complete = %d\n", init_not_complete);
    
       if (init_not_complete) {
            retval = synaptics_rmi4_query_device(rmi4_data);
            if (retval < 0)
                return;
            printk("tpd wanglei: query device success\n");
            
            input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_POSITION_X, 0,
			rmi4_data->sensor_max_x, 0, 0);
	     input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_POSITION_Y, 0,
			rmi4_data->sensor_max_y, 0, 0);
	     input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_PRESSURE, 0, 255, 0, 0);
    
#ifdef REPORT_2D_W
	     input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_TOUCH_MAJOR, 0,
			MAX_ABS_MT_TOUCH_MAJOR, 0, 0);
#endif

#ifdef TYPE_B_PROTOCOL
	     input_mt_init_slots(rmi4_data->input_dev,
			rmi4_data->num_of_fingers);
#endif

	     i2c_set_clientdata(rmi4_data->i2c_client, rmi4_data);

	     f1a = NULL;
	     if (!list_empty(&rmi->support_fn_list)) {
		    list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->fn_number == SYNAPTICS_RMI4_F1A)
				f1a = fhandler->data;
		   }
	     }

	     if (f1a) {
	         for (ii = 0; ii < f1a->valid_button_count; ii++) {
			set_bit(f1a->button_map[ii],
					rmi4_data->input_dev->keybit);
			input_set_capability(rmi4_data->input_dev,
					EV_KEY, f1a->button_map[ii]);
		  }
#ifdef DOUBLE_CLICK_WAKE
                set_bit(KEY_POWER, rmi4_data->input_dev->keybit);
	         input_set_capability(rmi4_data->input_dev, EV_KEY, KEY_POWER);
                set_bit(KEY_F13, rmi4_data->input_dev->keybit);
	         input_set_capability(rmi4_data->input_dev, EV_KEY, KEY_F13);
                set_bit(KEY_F14, rmi4_data->input_dev->keybit);
	         input_set_capability(rmi4_data->input_dev, EV_KEY, KEY_F14);
                set_bit(KEY_F15, rmi4_data->input_dev->keybit);
	         input_set_capability(rmi4_data->input_dev, EV_KEY, KEY_F15);
                set_bit(KEY_F16, rmi4_data->input_dev->keybit);
	         input_set_capability(rmi4_data->input_dev, EV_KEY, KEY_F16);
                set_bit(KEY_F17, rmi4_data->input_dev->keybit);
	         input_set_capability(rmi4_data->input_dev, EV_KEY, KEY_F17);
#endif

	     }

	     retval = input_register_device(rmi4_data->input_dev);
	     if (retval) {
	         printk("tpd: Failed to register input device\n");
	     }
            for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
	         retval = sysfs_create_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
		  if (retval < 0) {
		      printk("tpd: Failed to create sysfs attributes\n");
                    break;
                }
	     }
         
            init_not_complete = 0;
            printk("wanglei: tpd set init_not_complete = 0\n");
       }
}
#endif

#ifdef GN_ESD_PROTECT
static void gn_esd_reset_work(struct work_struct *work)
{
    struct synaptics_rmi4_data *rmi4_data =
			container_of(work, struct synaptics_rmi4_data, esd_work.work);

    gpio_direction_output(rmi4_data->board->en_gpio, 0);
    synaptics_rmi4_power_on(rmi4_data, false);
    mdelay(300);
    synaptics_rmi4_power_on(rmi4_data, true);
    gpio_direction_output(rmi4_data->board->en_gpio, 1);
    queue_delayed_work(rmi4_data->esd_queue, &rmi4_data->esd_work, msecs_to_jiffies(2000));
    printk("wanglei tpd: gn_esd_reset_work --> reset TP\n");
}
#endif

#ifdef GN_TP_INVALID_TEST
static void gn_tp_test_work(struct work_struct *work)
{
    int retval = 0;
    unsigned char int_test_flag = 0;
    struct synaptics_rmi4_data *rmi4_data =
			container_of(work, struct synaptics_rmi4_data, gn_test_work.work);

    retval = synaptics_rmi4_i2c_read(rmi4_data, 0x0014, &int_test_flag, 1);
    if (retval < 0) {
        printk("wanglei tpd: read int_test_flag (0x0014) failed...\n");
    }
    printk("wanglei tpd: int_test_flag = %d\n", int_test_flag);
    queue_delayed_work(rmi4_data->gn_test_queue, &rmi4_data->gn_test_work, msecs_to_jiffies(5000));
}
#endif

int gn_tp_reset(struct synaptics_rmi4_data *rmi4_data)
{
    gpio_direction_output(rmi4_data->board->en_gpio, 0);
    synaptics_rmi4_power_on(rmi4_data, false);
    mdelay(200);
    synaptics_rmi4_power_on(rmi4_data, true);
    gpio_direction_output(rmi4_data->board->en_gpio, 1);
    mdelay(300);
    printk("wanglei tpd: gn_tp_reset --> reset TP Compelete\n");
    return 0;
}

 /**
 * synaptics_rmi4_probe()
 *
 * Called by the kernel when an association with an I2C device of the
 * same name is made (after doing i2c_add_driver).
 *
 * This funtion allocates and initializes the resources for the driver
 * as an input driver, turns on the power to the sensor, queries the
 * sensor for its supported Functions and characteristics, registers
 * the driver to the input subsystem, sets up the interrupt, handles
 * the registration of the early_suspend and late_resume functions,
 * and creates a work queue for detection of other expansion Function
 * modules.
 */
static int __devinit synaptics_rmi4_probe(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
	int retval = 0;
	unsigned char ii;
	unsigned char attr_count;
	struct synaptics_rmi4_f1a_handle *f1a;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_data *rmi4_data;
	struct synaptics_rmi4_device_info *rmi;
	struct synaptics_rmi4_platform_data *platform_data =
			client->dev.platform_data;

#if defined(CONFIG_GN_DEVICE_TYPE_CHECK) 
	gn_mydev_info.gn_dev_type = GN_DEVICE_TYPE_TP;
#endif

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev,
				"%s: SMBus byte data not supported\n",
				__func__);
		return -EIO;
	}

	if (client->dev.of_node) {
		platform_data = devm_kzalloc(&client->dev,
			sizeof(*platform_data),
			GFP_KERNEL);
		if (!platform_data) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		retval = synaptics_rmi4_parse_dt(&client->dev, platform_data);
		if (retval)
			return retval;
	} else {
		platform_data = client->dev.platform_data;
	}

	if (!platform_data) {
		dev_err(&client->dev,
				"%s: No platform data found\n",
				__func__);
		return -EINVAL;
	}

	rmi4_data = kzalloc(sizeof(*rmi4_data) * 2, GFP_KERNEL);
	if (!rmi4_data) {
		dev_err(&client->dev,
				"%s: Failed to alloc mem for rmi4_data\n",
				__func__);
		return -ENOMEM;
	}

	rmi = &(rmi4_data->rmi4_mod_info);

	rmi4_data->input_dev = input_allocate_device();
	if (rmi4_data->input_dev == NULL) {
		dev_err(&client->dev,
				"%s: Failed to allocate input device\n",
				__func__);
		retval = -ENOMEM;
		goto err_input_device;
	}

#ifdef TP_GLOVE_SUPPORT
      glove_rmi4_data = rmi4_data;
#endif
#ifdef DOUBLE_CLICK_WAKE
      test_rmi4_data = rmi4_data;
#endif


	rmi4_data->i2c_client = client;
	rmi4_data->current_page = MASK_8BIT;
	rmi4_data->board = platform_data;
	rmi4_data->touch_stopped = false;
	rmi4_data->sensor_sleep = false;
	rmi4_data->irq_enabled = false;

	rmi4_data->i2c_read = synaptics_rmi4_i2c_read;
	rmi4_data->i2c_write = synaptics_rmi4_i2c_write;
	rmi4_data->irq_enable = synaptics_rmi4_irq_enable;
	rmi4_data->reset_device = synaptics_rmi4_reset_device;
       rmi4_data->reset_fw = gn_tp_reset;

	rmi4_data->flip_x = rmi4_data->board->x_flip;
	rmi4_data->flip_y = rmi4_data->board->y_flip;

	rmi4_data->fw_image_name = rmi4_data->board->fw_image_name;

	rmi4_data->input_dev->name = DRIVER_NAME;
	rmi4_data->input_dev->phys = INPUT_PHYS_NAME;
	rmi4_data->input_dev->id.bustype = BUS_I2C;
	rmi4_data->input_dev->id.product = SYNAPTICS_DSX_DRIVER_PRODUCT;
	rmi4_data->input_dev->id.version = SYNAPTICS_DSX_DRIVER_VERSION;
	rmi4_data->input_dev->dev.parent = &client->dev;
	input_set_drvdata(rmi4_data->input_dev, rmi4_data);

	set_bit(EV_SYN, rmi4_data->input_dev->evbit);
	set_bit(EV_KEY, rmi4_data->input_dev->evbit);
	set_bit(EV_ABS, rmi4_data->input_dev->evbit);
	set_bit(BTN_TOUCH, rmi4_data->input_dev->keybit);
	set_bit(BTN_TOOL_FINGER, rmi4_data->input_dev->keybit);

#ifdef INPUT_PROP_DIRECT
	set_bit(INPUT_PROP_DIRECT, rmi4_data->input_dev->propbit);
#endif

	retval = synaptics_rmi4_regulator_configure(rmi4_data, true);
	if (retval < 0) {
		dev_err(&client->dev, "Failed to configure regulators\n");
		goto err_reg_configure;
	}

	retval = synaptics_rmi4_power_on(rmi4_data, true);
	if (retval < 0) {
		dev_err(&client->dev, "Failed to power on\n");
		goto err_power_device;
	}

       if (gpio_is_valid(platform_data->en_gpio)) {
		/* configure touchscreen enable gpio */
		retval = gpio_request(platform_data->en_gpio,
				"rmi4_en_gpio");
		if (retval) {
			dev_err(&client->dev, "tpd: unable to request gpio [%d]\n",
						platform_data->en_gpio);
			goto err_irq_gpio_dir;
		}

		retval = gpio_direction_output(platform_data->en_gpio, 1);
		if (retval) {
			dev_err(&client->dev,
				"tpd: unable to set direction for gpio [%d]\n",
				platform_data->en_gpio);
			goto err_reset_gpio_dir;
		}
	} else
		synaptics_rmi4_reset_command(rmi4_data);

	if (gpio_is_valid(platform_data->irq_gpio)) {
		/* configure touchscreen irq gpio */
		retval = gpio_request(platform_data->irq_gpio, "rmi4_irq_gpio");
		if (retval) {
			dev_err(&client->dev, "tpd: unable to request gpio [%d]\n",
						platform_data->irq_gpio);
			goto err_irq_gpio_req;
		}
		retval = gpio_direction_input(platform_data->irq_gpio);
		if (retval) {
			dev_err(&client->dev,
				"tpd: unable to set direction for gpio [%d]\n",
				platform_data->irq_gpio);
			goto err_irq_gpio_dir;
		}
	} else {
		dev_err(&client->dev, "tpd: irq gpio not provided\n");
		goto err_irq_gpio_req;
	}

	if (gpio_is_valid(platform_data->reset_gpio)) {
		/* configure touchscreen reset out gpio */
		retval = gpio_request(platform_data->reset_gpio,
				"rmi4_reset_gpio");
		if (retval) {
			dev_err(&client->dev, "tpd: unable to request gpio [%d]\n",
						platform_data->reset_gpio);
			goto err_irq_gpio_dir;
		}

		retval = gpio_direction_output(platform_data->reset_gpio, 1);
		if (retval) {
			dev_err(&client->dev,
				"tpd: unable to set direction for gpio [%d]\n",
				platform_data->reset_gpio);
			goto err_reset_gpio_dir;
		}
	} else
		synaptics_rmi4_reset_command(rmi4_data);


	init_waitqueue_head(&rmi4_data->wait);
	mutex_init(&(rmi4_data->rmi4_io_ctrl_mutex));

	retval = synaptics_rmi4_query_device(rmi4_data);
	if (retval < 0) {
		dev_err(&client->dev,
				"tpd: %s: Failed to query device\n",
				__func__);
		//goto err_reset_gpio_dir;
#ifdef INIT_TP_WHEN_RESUME
		init_not_complete = 1;
              failed_init_flag = 1;
	}else {
	       init_not_complete = 0;
#endif
       }

       suspend_flag = 0;
#ifdef GN_ESD_PROTECT
       rmi4_data->esd_queue = create_singlethread_workqueue("gn_esd_workqueue");
	INIT_DELAYED_WORK(&rmi4_data->esd_work, gn_esd_reset_work);
	queue_delayed_work(rmi4_data->esd_queue, &rmi4_data->esd_work, msecs_to_jiffies(10000));
#endif

#ifdef GN_TP_INVALID_TEST
      rmi4_data->gn_test_queue = create_singlethread_workqueue("gn_tp_test_workqueue");
	INIT_DELAYED_WORK(&rmi4_data->gn_test_work, gn_tp_test_work);
	queue_delayed_work(rmi4_data->gn_test_queue, &rmi4_data->gn_test_work, msecs_to_jiffies(5000));
#endif

#ifdef TP_GLOVE_SUPPORT
       rmi4_data->gn_glove_queue = create_singlethread_workqueue("gn_tp_glove_workqueue");
	INIT_DELAYED_WORK(&rmi4_data->gn_glove_work, gn_tp_glove_work);
#endif

#ifdef INIT_TP_WHEN_RESUME
       if (init_not_complete)
           goto init_break;
#endif
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_POSITION_X, 0,
			rmi4_data->sensor_max_x, 0, 0);
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_POSITION_Y, 0,
			rmi4_data->sensor_max_y, 0, 0);
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_PRESSURE, 0, 255, 0, 0);
    
#ifdef REPORT_2D_W
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_TOUCH_MAJOR, 0,
			MAX_ABS_MT_TOUCH_MAJOR, 0, 0);
#endif

#ifdef TYPE_B_PROTOCOL
	input_mt_init_slots(rmi4_data->input_dev,
			rmi4_data->num_of_fingers);
#endif

	i2c_set_clientdata(client, rmi4_data);

	f1a = NULL;
	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->fn_number == SYNAPTICS_RMI4_F1A)
				f1a = fhandler->data;
		}
	}

	if (f1a) {
		for (ii = 0; ii < f1a->valid_button_count; ii++) {
			set_bit(f1a->button_map[ii],
					rmi4_data->input_dev->keybit);
			input_set_capability(rmi4_data->input_dev,
					EV_KEY, f1a->button_map[ii]);
		}
#ifdef DOUBLE_CLICK_WAKE
              set_bit(KEY_POWER, rmi4_data->input_dev->keybit);
		input_set_capability(rmi4_data->input_dev, EV_KEY, KEY_POWER);
              set_bit(KEY_F13, rmi4_data->input_dev->keybit);
	       input_set_capability(rmi4_data->input_dev, EV_KEY, KEY_F13);
              set_bit(KEY_F14, rmi4_data->input_dev->keybit);
	       input_set_capability(rmi4_data->input_dev, EV_KEY, KEY_F14);
              set_bit(KEY_F15, rmi4_data->input_dev->keybit);
	       input_set_capability(rmi4_data->input_dev, EV_KEY, KEY_F15);
              set_bit(KEY_F16, rmi4_data->input_dev->keybit);
	       input_set_capability(rmi4_data->input_dev, EV_KEY, KEY_F16);
              set_bit(KEY_F17, rmi4_data->input_dev->keybit);
	       input_set_capability(rmi4_data->input_dev, EV_KEY, KEY_F17);
#endif

	}

	retval = input_register_device(rmi4_data->input_dev);
	if (retval) {
		dev_err(&client->dev,
				"tpd: %s: Failed to register input device\n",
				__func__);
		goto err_register_input;
	}

#ifdef INIT_TP_WHEN_RESUME
init_break:
#endif
	configure_sleep(rmi4_data);

	if (!exp_fn_inited) {
		mutex_init(&exp_fn_list_mutex);
		INIT_LIST_HEAD(&exp_fn_list);
		exp_fn_inited = 1;
	}

	rmi4_data->det_workqueue =
			create_singlethread_workqueue("rmi_det_workqueue");
	INIT_DELAYED_WORK(&rmi4_data->det_work,
			synaptics_rmi4_detection_work);
	queue_delayed_work(rmi4_data->det_workqueue,
			&rmi4_data->det_work,
			msecs_to_jiffies(EXP_FN_DET_INTERVAL));

	rmi4_data->irq = gpio_to_irq(platform_data->irq_gpio);

	retval = request_threaded_irq(rmi4_data->irq, NULL,
		synaptics_rmi4_irq, platform_data->irq_flags,
		DRIVER_NAME, rmi4_data);
	rmi4_data->irq_enabled = true;

	if (retval < 0) {
		dev_err(&client->dev,
				"tpd: %s: Failed to create irq thread\n",
				__func__);
		goto err_enable_irq;
	}

#ifdef INIT_TP_WHEN_RESUME
       if (init_not_complete)
           goto init_break2;
#endif
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		retval = sysfs_create_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
		if (retval < 0) {
			dev_err(&client->dev,
					"tpd: %s: Failed to create sysfs attributes\n",
					__func__);
			goto err_sysfs;
		}
	}
#ifdef INIT_TP_WHEN_RESUME
init_break2:
#endif
	retval = synaptics_rmi4_irq_enable(rmi4_data, true);
	if (retval < 0) {
		dev_err(&client->dev,
			"tpd: %s: Failed to enable attention interrupt\n",
			__func__);
		goto err_sysfs;
	}

#ifdef DOUBLE_CLICK_WAKE
       retval = platform_device_register(&gn_tp_wake_device);
	if(retval){
		printk("tpd: create gn_tp_wake_device failed\n");
	}else {
	       printk("tpd: create gn_tp_wake_device success\n");
       }

	retval = wake_tp_create_attr(&(gn_tp_wake_device.dev));
	if (retval) {
		printk("tpd: wake_tp_create_attr failed.\n");
	}else {
	       printk("tpd: wake_tp_create_attr success\n");
       }
#endif

	return retval;

err_sysfs:
	for (attr_count--; attr_count >= 0; attr_count--) {
		sysfs_remove_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
	}

err_enable_irq:
	cancel_delayed_work_sync(&rmi4_data->det_work);
	flush_workqueue(rmi4_data->det_workqueue);
	destroy_workqueue(rmi4_data->det_workqueue);
	input_unregister_device(rmi4_data->input_dev);

err_register_input:
	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->fn_number == SYNAPTICS_RMI4_F1A)
				synaptics_rmi4_f1a_kfree(fhandler);
			else
				kfree(fhandler->data);
			kfree(fhandler);
		}
	}
err_reset_gpio_dir:
	if (gpio_is_valid(platform_data->reset_gpio))
		gpio_free(platform_data->reset_gpio);
err_irq_gpio_dir:
	if (gpio_is_valid(platform_data->irq_gpio))
		gpio_free(platform_data->irq_gpio);
err_irq_gpio_req:
	synaptics_rmi4_power_on(rmi4_data, false);
err_power_device:
	synaptics_rmi4_regulator_configure(rmi4_data, false);
err_reg_configure:
	input_free_device(rmi4_data->input_dev);
	rmi4_data->input_dev = NULL;
err_input_device:
	kfree(rmi4_data);

	return retval;
}

 /**
 * synaptics_rmi4_remove()
 *
 * Called by the kernel when the association with an I2C device of the
 * same name is broken (when the driver is unloaded).
 *
 * This funtion terminates the work queue, stops sensor data acquisition,
 * frees the interrupt, unregisters the driver from the input subsystem,
 * turns off the power to the sensor, and frees other allocated resources.
 */
static int __devexit synaptics_rmi4_remove(struct i2c_client *client)
{
	unsigned char attr_count;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_data *rmi4_data = i2c_get_clientdata(client);
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	cancel_delayed_work_sync(&rmi4_data->det_work);
	flush_workqueue(rmi4_data->det_workqueue);
	destroy_workqueue(rmi4_data->det_workqueue);

	rmi4_data->touch_stopped = true;
	wake_up(&rmi4_data->wait);

	free_irq(rmi4_data->irq, rmi4_data);

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		sysfs_remove_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
	}

	input_unregister_device(rmi4_data->input_dev);

	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->fn_number == SYNAPTICS_RMI4_F1A)
				synaptics_rmi4_f1a_kfree(fhandler);
			else
				kfree(fhandler->data);
			kfree(fhandler);
		}
	}

	if (gpio_is_valid(rmi4_data->board->reset_gpio))
		gpio_free(rmi4_data->board->reset_gpio);
	if (gpio_is_valid(rmi4_data->board->irq_gpio))
		gpio_free(rmi4_data->board->irq_gpio);

	synaptics_rmi4_power_on(rmi4_data, false);
	synaptics_rmi4_regulator_configure(rmi4_data, false);

	kfree(rmi4_data);

	return 0;
}

#ifdef CONFIG_PM
 /**
 * synaptics_rmi4_sensor_sleep()
 *
 * Called by synaptics_rmi4_early_suspend() and synaptics_rmi4_suspend().
 *
 * This function stops finger data acquisition and puts the sensor to sleep.
 */
static void synaptics_rmi4_sensor_sleep(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char device_ctrl;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		dev_err(&(rmi4_data->input_dev->dev),
				"%s: Failed to enter sleep mode\n",
				__func__);
		rmi4_data->sensor_sleep = false;
		return;
	}

	device_ctrl = (device_ctrl & ~MASK_3BIT);
	device_ctrl = (device_ctrl | NO_SLEEP_OFF | SENSOR_SLEEP);

	retval = synaptics_rmi4_i2c_write(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		dev_err(&(rmi4_data->input_dev->dev),
				"%s: Failed to enter sleep mode\n",
				__func__);
		rmi4_data->sensor_sleep = false;
		return;
	} else {
		rmi4_data->sensor_sleep = true;
	}

	return;
}

#ifdef DOUBLE_CLICK_WAKE 
static void synaptics_rmi4_double_wakeup_enter(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char device_ctrl, button_ctrl;

       printk("wanglei: enter double wakeup...\n");

       //Disable 0D Button
       button_ctrl = 0x00;
       retval = synaptics_rmi4_i2c_write(rmi4_data,
			0x0202,
			&button_ctrl,
			sizeof(button_ctrl));
       if (retval < 0) {
           printk("wanglei tpd: Disable 0D button Failed...\n");
       }

       //Clear Interrupt
       synaptics_rmi4_i2c_read(rmi4_data,
			0x0014,
			&button_ctrl,
			sizeof(button_ctrl));

       //Enter gesture wake mode
       device_ctrl = 0x0C;
	retval = synaptics_rmi4_i2c_write(rmi4_data,
			0x57,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		dev_err(&(rmi4_data->input_dev->dev),
				"wanglei: %s: Failed to enter sleep mode\n",
				__func__);
		rmi4_data->sensor_sleep = false;
		return;
	} else {
		rmi4_data->sensor_sleep = true;
	}

       device_ctrl = 0x00;
       synaptics_rmi4_i2c_read(rmi4_data,
			0x57,
			&device_ctrl,
			sizeof(device_ctrl));

       printk("wanglei: 0x57 register = 0x%x\n", device_ctrl);

	return;
}
#endif


 /**
 * synaptics_rmi4_sensor_wake()
 *
 * Called by synaptics_rmi4_resume() and synaptics_rmi4_late_resume().
 *
 * This function wakes the sensor from sleep.
 */
static void synaptics_rmi4_sensor_wake(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char device_ctrl;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		dev_err(&(rmi4_data->input_dev->dev),
				"%s: Failed to wake from sleep mode\n",
				__func__);
		rmi4_data->sensor_sleep = true;
		return;
	}

	device_ctrl = (device_ctrl & ~MASK_3BIT);
	device_ctrl = (device_ctrl | NO_SLEEP_OFF | NORMAL_OPERATION);

	retval = synaptics_rmi4_i2c_write(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		dev_err(&(rmi4_data->input_dev->dev),
				"%s: Failed to wake from sleep mode\n",
				__func__);
		rmi4_data->sensor_sleep = true;
		return;
	} else {
		rmi4_data->sensor_sleep = false;
	}

	return;
}

#ifdef DOUBLE_CLICK_WAKE
 static void synaptics_rmi4_double_wakeup_exit(struct synaptics_rmi4_data *rmi4_data)
{
	int retval, count = 0;
	unsigned char device_ctrl, button_ctrl, int_test = 0;

       printk("wanglei tpd: double wakeup exit...\n");

       do {
	    device_ctrl = 0x08;
	    retval = synaptics_rmi4_i2c_write(rmi4_data,
			0x57,
			&device_ctrl,
			sizeof(device_ctrl));
	    if (retval < 0) {
		    dev_err(&(rmi4_data->input_dev->dev),
				"%s tpd: Failed to wake from sleep mode\n",
				__func__);
		    rmi4_data->sensor_sleep = true;
		    return;
	    } else {
		    rmi4_data->sensor_sleep = false;
	    }

           device_ctrl = 0x00;
           synaptics_rmi4_i2c_read(rmi4_data,
			0x57,
			&device_ctrl,
			sizeof(device_ctrl));

           printk("wanglei tpd: 0x57 register = 0x%x\n", device_ctrl);
        }while(device_ctrl != 0x08 && count++ < 10);

       //Enable 0D Button
       button_ctrl = 0x07;
       retval = synaptics_rmi4_i2c_write(rmi4_data,
			0x0202,
			&button_ctrl,
			sizeof(button_ctrl));
       if (retval < 0) {
           printk("wanglei tpd: Enable 0D button Failed...\n");
       }

       synaptics_rmi4_i2c_read(rmi4_data, 0x0014, &int_test, 1);
       printk("wanglei tpd: read int_test = 0x%x\n", int_test);

	return;
}
#endif


#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int new_status;
	int ev;
	struct synaptics_rmi4_data *rmi4_data =
		container_of(self, struct synaptics_rmi4_data, fb_notif);

	switch (event) {
		case FB_EVENT_BLANK :
			ev = (*(int *)evdata->data);

			/*
			 * Normal Screen Wakeup
			 *
			 * <6>[   43.486172] [syna] Event: 4 -> 0
			 * <6>[   50.488192] [syna] Event: 0 -> 4
			 *
			 * Doze Wakeup
			 *
			 * <6>[   81.869758] [syna] Event: 4 -> 1
			 * <6>[   86.458247] [syna] Event: 1 -> 4
			 *
			 */
			switch (ev) {
				/* Screen On */
				case FB_BLANK_UNBLANK:
				case FB_BLANK_NORMAL:
				case FB_BLANK_VSYNC_SUSPEND:
				case FB_BLANK_HSYNC_SUSPEND:
					new_status = 0;
					break;
				default:
					/* Default to screen off to match previous
					   behaviour */
					printk("[syna] Unhandled event %i\n", ev);
					/* Fall through */
				case FB_BLANK_POWERDOWN:
					new_status = 1;
					break;
			}

			if (new_status == rmi4_data->old_status)
				break;

			if (new_status) {
				printk("[syna]:suspend tp\n");
				synaptics_rmi4_suspend(&(rmi4_data->input_dev->dev));
			}
			else {
				printk("[syna]:resume tp\n");
				synaptics_rmi4_resume(&(rmi4_data->input_dev->dev));
			}
			rmi4_data->old_status = new_status;
			break;
	}
	return 0;
}
#endif

 /**
 * synaptics_rmi4_suspend()
 *
 * Called by the kernel during the suspend phase when the system
 * enters suspend.
 *
 * This function stops finger data acquisition and puts the sensor to
 * sleep (if not already done so during the early suspend phase),
 * disables the interrupt, and turns off the power to the sensor.
 */
static int synaptics_rmi4_suspend(struct device *dev)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
    int count = 0;

#ifdef INIT_TP_WHEN_RESUME
       if (init_not_complete) {
            printk("wanglei: TP init not complete, suspend return 0\n");
            return 0;
       }
#endif

#ifdef DOUBLE_CLICK_WAKE
       if (wake_switch || gesture_switch) {
           printk("tpd wanglei: synaptics_rmi4_suspend --> rmi4_data->irq = %d\n", rmi4_data->irq);
           wake_suspend_compelete = 1;
           //TP reset first
           gn_tp_reset(rmi4_data);
       #ifdef TP_GLOVE_SUPPORT
           if (glove_enable) {
               do {
                   if_enable_tp_glove(0x02);
               }while(!glove_switch && (count++ < 5));
           }
       #endif
           //If TP IC is not running, disable set glove mode
           suspend_flag = 1;
           enable_irq_wake(rmi4_data->irq);
           synaptics_rmi4_double_wakeup_enter(rmi4_data);
           return 0;
       }
#endif

       //If TP IC is not running, disable set glove mode
       suspend_flag = 1;
       glove_status = 0x0f;

	if (!rmi4_data->sensor_sleep) {
		rmi4_data->touch_stopped = true;
		wake_up(&rmi4_data->wait);
		synaptics_rmi4_irq_enable(rmi4_data, false);
		synaptics_rmi4_sensor_sleep(rmi4_data);
	}
       
       printk("tpd: synaptics_rmi4_suspend... Success!\n");

	return 0;
}

 /**
 * synaptics_rmi4_resume()
 *
 * Called by the kernel during the resume phase when the system
 * wakes up from suspend.
 *
 * This function turns on the power to the sensor, wakes the sensor
 * from sleep, enables the interrupt, and starts finger data
 * acquisition.
 */
static int synaptics_rmi4_resume(struct device *dev)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	int count = 0, finger = 0;

#ifdef INIT_TP_WHEN_RESUME
       gn_resume_init(rmi4_data);

       if (init_not_complete) {
           printk("wanglei: TP init not complete, resume return 0\n");
           return 0;
       }
#endif

//Release Finger
#ifdef TYPE_B_PROTOCOL
       for (finger = 0; finger < tp_max_finger; finger++) {
	    input_mt_slot(rmi4_data->input_dev, finger);
	    input_mt_report_slot_state(rmi4_data->input_dev, MT_TOOL_FINGER, 0);
       }
#endif
       input_report_key(rmi4_data->input_dev, BTN_TOUCH, 0);
	input_report_key(rmi4_data->input_dev, BTN_TOOL_FINGER, 0);
       input_sync(rmi4_data->input_dev);

#ifdef DOUBLE_CLICK_WAKE
       if (wake_suspend_compelete) {
           printk("wanglei tpd: double wake resume...\n");
           wake_suspend_compelete = 0;
           synaptics_rmi4_double_wakeup_exit(rmi4_data);
           suspend_flag = 0;
    #ifdef TP_GLOVE_SUPPORT
       if (glove_enable) {
           do {
               if_enable_tp_glove(0x00);
           }while(!glove_switch && (count++ < 5));
       }
    #endif
           disable_irq_wake(rmi4_data->irq);
           return 0;
       }else {
           //enable_irq(rmi4_data->irq);
       }
#endif

	synaptics_rmi4_sensor_wake(rmi4_data);
	rmi4_data->touch_stopped = false;
	synaptics_rmi4_irq_enable(rmi4_data, true);

       ts_debug("synaptics_rmi4_resume... Success!\n");
       suspend_flag = 0;
#ifdef TP_GLOVE_SUPPORT
       if (glove_enable) {
           do {
               if_enable_tp_glove(0x00);
           }while(!glove_switch && (count++ < 5));
       }
#endif
	return 0;
}

#ifndef CONFIG_FB
static const struct dev_pm_ops synaptics_rmi4_dev_pm_ops = {
	.suspend = synaptics_rmi4_suspend,
	.resume  = synaptics_rmi4_resume,
};
#endif
#endif

static const struct i2c_device_id synaptics_rmi4_id_table[] = {
	{DRIVER_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, synaptics_rmi4_id_table);

#ifdef CONFIG_OF
static struct of_device_id rmi4_match_table[] = {
	{ .compatible = "synaptics,rmi4",},
	{ },
};
#else
#define rmi4_match_table NULL
#endif

static struct i2c_driver synaptics_rmi4_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
#ifndef CONFIG_FB
		.pm = &synaptics_rmi4_dev_pm_ops,
#endif
#endif
		.of_match_table = rmi4_match_table,
	},
	.probe = synaptics_rmi4_probe,
	.remove = __devexit_p(synaptics_rmi4_remove),
	.id_table = synaptics_rmi4_id_table,
};

 /**
 * synaptics_rmi4_init()
 *
 * Called by the kernel during do_initcalls (if built-in)
 * or when the driver is loaded (if a module).
 *
 * This function registers the driver to the I2C subsystem.
 *
 */
static int __init synaptics_rmi4_init(void)
{
	return i2c_add_driver(&synaptics_rmi4_driver);
}

 /**
 * synaptics_rmi4_exit()
 *
 * Called by the kernel when the driver is unloaded.
 *
 * This funtion unregisters the driver from the I2C subsystem.
 *
 */
static void __exit synaptics_rmi4_exit(void)
{
	i2c_del_driver(&synaptics_rmi4_driver);

	return;
}

module_init(synaptics_rmi4_init);
module_exit(synaptics_rmi4_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics RMI4 I2C Touch Driver");
MODULE_LICENSE("GPL v2");
