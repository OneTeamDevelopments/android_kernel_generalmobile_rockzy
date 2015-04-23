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
#define GN_SUNNY_OV16825_SENSOR_NAME "gn_sunny_ov16825"
DEFINE_MSM_MUTEX(gn_sunny_ov16825_mut);
#if defined(CONFIG_GN_Q_BSP_DEVICE_TYPE_CHECK_SUPPORT)
#include <linux/gn_device_check.h>
#endif
#if defined(CONFIG_GN_Q_BSP_DEVICE_TYPE_CHECK_SUPPORT)
extern int gn_set_device_info(struct gn_device_info gn_dev_info);
static struct gn_device_info gn_cameradev_info;
#endif
static struct msm_sensor_ctrl_t gn_sunny_ov16825_s_ctrl;

static struct msm_sensor_power_setting gn_sunny_ov16825_power_setting[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 5,
	}, 
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VDIG,   //SUB DVDD GPIO POWER
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	}, 
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VAF,
		.config_val = 0,
		.delay = 15,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_AF_PWDM,  
		.config_val = GPIO_OUT_HIGH,
		.delay = 15,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 15,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 40,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 40,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 40,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct v4l2_subdev_info gn_sunny_ov16825_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id gn_sunny_ov16825_i2c_id[] = {
	{GN_SUNNY_OV16825_SENSOR_NAME, (kernel_ulong_t)&gn_sunny_ov16825_s_ctrl},
	{ }
};

static int32_t msm_gn_sunny_ov16825_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &gn_sunny_ov16825_s_ctrl);
}
static struct i2c_driver gn_sunny_ov16825_i2c_driver = {
	.id_table = gn_sunny_ov16825_i2c_id,
	.probe  = msm_gn_sunny_ov16825_i2c_probe,
	.driver = {
		.name = GN_SUNNY_OV16825_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client gn_sunny_ov16825_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id gn_sunny_ov16825_dt_match[] = {
	{.compatible = "qcom,gn_sunny_ov16825", .data = &gn_sunny_ov16825_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, gn_sunny_ov16825_dt_match);

static struct platform_driver gn_sunny_ov16825_platform_driver = {
	.driver = {
		.name = "qcom,gn_sunny_ov16825",
		.owner = THIS_MODULE,
		.of_match_table = gn_sunny_ov16825_dt_match,
	},
};

static int32_t gn_sunny_ov16825_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(gn_sunny_ov16825_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init gn_sunny_ov16825_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&gn_sunny_ov16825_platform_driver,
		gn_sunny_ov16825_platform_probe);
	if (!rc){
	#if defined(CONFIG_GN_Q_BSP_DEVICE_TYPE_CHECK_SUPPORT)
		gn_cameradev_info.gn_dev_type = GN_DEVICE_TYPE_BACK_CAM;
		memcpy(gn_cameradev_info.name, "sunny_ov16825",sizeof("sunny_ov16825"));
		memcpy(gn_cameradev_info.vendor,"sunny_ov16825",sizeof("sunny_ov16825"));
		gn_set_device_info(gn_cameradev_info);
	#endif
		return rc;
	}
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&gn_sunny_ov16825_i2c_driver);
}

static void __exit gn_sunny_ov16825_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (gn_sunny_ov16825_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&gn_sunny_ov16825_s_ctrl);
		platform_driver_unregister(&gn_sunny_ov16825_platform_driver);
	} else
		i2c_del_driver(&gn_sunny_ov16825_i2c_driver);
	return;
}

static struct msm_sensor_ctrl_t gn_sunny_ov16825_s_ctrl = {
	.sensor_i2c_client = &gn_sunny_ov16825_sensor_i2c_client,
	.power_setting_array.power_setting = gn_sunny_ov16825_power_setting,
	.power_setting_array.size = ARRAY_SIZE(gn_sunny_ov16825_power_setting),
	.msm_sensor_mutex = &gn_sunny_ov16825_mut,
	.sensor_v4l2_subdev_info = gn_sunny_ov16825_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(gn_sunny_ov16825_subdev_info),
};
module_init(gn_sunny_ov16825_init_module);
module_exit(gn_sunny_ov16825_exit_module);
MODULE_DESCRIPTION("gn_sunny_ov16825");
MODULE_LICENSE("GPL v2");
