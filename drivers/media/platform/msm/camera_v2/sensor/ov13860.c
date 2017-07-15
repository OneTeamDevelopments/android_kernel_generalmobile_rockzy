/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include "msm_sensor.h"
/* Gionee pangfei,201140213, add by gionee device check ,start*/
#if defined(CONFIG_GN_Q_BSP_DEVICE_TYPE_CHECK_SUPPORT)
#include <linux/gn_device_check.h>
#endif
/* Gionee pangfei,201140213, add by gionee device check ,end*/

#define OV13860_SENSOR_NAME "ov13860"
DEFINE_MSM_MUTEX(ov13860_mut);

/* Gionee pangfei,201140213, add by gionee device check ,start*/
#if defined(CONFIG_GN_Q_BSP_DEVICE_TYPE_CHECK_SUPPORT)
extern int gn_set_device_info(struct gn_device_info gn_dev_info);
static struct gn_device_info gn_cameradev_info;
#endif
/* Gionee pangfei,201140213, add by gionee device check ,end*/

static struct msm_sensor_ctrl_t ov13860_s_ctrl;
static struct msm_sensor_power_setting ov13860_power_setting[] = {

    {
	    .seq_type = SENSOR_VREG,
		.seq_val =  CAM_VDIG,   //SUB DVDD GPIO POWER
		.config_val = 0,
		.delay = 10,
	},
	#if 1
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 10,
	},
	#endif
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 10,
	},
	#if 1
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VAF,
		.config_val = 0,
		.delay = 5,
	},
	#endif
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},	
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},	
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_AF_PWDM,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},	
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 10,
	},	
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_AF_PWDM,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},	
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct v4l2_subdev_info ov13860_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id ov13860_i2c_id[] = {
	{OV13860_SENSOR_NAME, (kernel_ulong_t)&ov13860_s_ctrl},
	{ }
};

static int msm_ov13860_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &ov13860_s_ctrl);
}

static struct i2c_driver ov13860_i2c_driver = {
	.id_table = ov13860_i2c_id,
	.probe  = msm_ov13860_i2c_probe,
	.driver = {
		.name = OV13860_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ov13860_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id ov13860_dt_match[] = {
	{.compatible = "qcom,ov13860", .data = &ov13860_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, ov13860_dt_match);

static struct platform_driver ov13860_platform_driver = {
	.driver = {
		.name = "qcom,ov13860",
		.owner = THIS_MODULE,
		.of_match_table = ov13860_dt_match,
	},
};

static int32_t ov13860_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(ov13860_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init ov13860_init_module(void)
{
	int32_t rc = 0;
	printk("pangfei %s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&ov13860_platform_driver,
		ov13860_platform_probe);
	if (!rc){ 
/* Gionee pangfei,20140213, add by gionee device check ,start*/
#if defined(CONFIG_GN_Q_BSP_DEVICE_TYPE_CHECK_SUPPORT)
		gn_cameradev_info.gn_dev_type = GN_DEVICE_TYPE_BACK_CAM;
		memcpy(gn_cameradev_info.name, "sunny_ov13860",sizeof("sunny_ov13860"));
		memcpy(gn_cameradev_info.vendor,"sunny_ov13860",sizeof("sunny_ov13860"));
		gn_set_device_info(gn_cameradev_info);
#endif
/* Gionee pangfei,20140213, add by gionee device check ,end*/
		return rc;
	}	
	return i2c_add_driver(&ov13860_i2c_driver);
}

static void __exit ov13860_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (ov13860_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&ov13860_s_ctrl);
		platform_driver_unregister(&ov13860_platform_driver);
	} else {
		i2c_del_driver(&ov13860_i2c_driver);
	}
	return;
}

static struct msm_sensor_ctrl_t ov13860_s_ctrl = {
	.sensor_i2c_client = &ov13860_sensor_i2c_client,
	.power_setting_array.power_setting = ov13860_power_setting,
	.power_setting_array.size = ARRAY_SIZE(ov13860_power_setting),
	.msm_sensor_mutex = &ov13860_mut,
	.sensor_v4l2_subdev_info = ov13860_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ov13860_subdev_info),
};

module_init(ov13860_init_module);
module_exit(ov13860_exit_module);
MODULE_DESCRIPTION("ov13860");
MODULE_LICENSE("GPL v2");
