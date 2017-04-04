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
#include <linux/delay.h>
#define GN_SUNNY_OV4688_SENSOR_NAME "gn_sunny_ov4688"
#define RG_TYPICAL 0xE7	//0x138    
#define BG_TYPICAL 0x10F //0x127
DEFINE_MSM_MUTEX(gn_sunny_ov4688_mut);
#if defined(CONFIG_GN_Q_BSP_DEVICE_TYPE_CHECK_SUPPORT)
#include <linux/gn_device_check.h>
#endif
#if defined(CONFIG_GN_Q_BSP_DEVICE_TYPE_CHECK_SUPPORT)
extern int gn_set_device_info(struct gn_device_info gn_dev_info);
static struct gn_device_info gn_cameradev_info;
#endif
static struct msm_sensor_ctrl_t gn_sunny_ov4688_s_ctrl;
static struct msm_sensor_power_setting gn_sunny_ov4688_power_setting[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
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
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
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
		.config_val = GPIO_OUT_HIGH,
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

static struct v4l2_subdev_info gn_sunny_ov4688_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id gn_sunny_ov4688_i2c_id[] = {
	{GN_SUNNY_OV4688_SENSOR_NAME, (kernel_ulong_t)&gn_sunny_ov4688_s_ctrl},
	{ }
};

static int32_t msm_gn_sunny_ov4688_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &gn_sunny_ov4688_s_ctrl);
}
static struct i2c_driver gn_sunny_ov4688_i2c_driver = {
	.id_table = gn_sunny_ov4688_i2c_id,
	.probe  = msm_gn_sunny_ov4688_i2c_probe,
	.driver = {
		.name = GN_SUNNY_OV4688_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client gn_sunny_ov4688_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id gn_sunny_ov4688_dt_match[] = {
	{.compatible = "qcom,gn_sunny_ov4688", .data = &gn_sunny_ov4688_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, gn_sunny_ov4688_dt_match);

static struct platform_driver gn_sunny_ov4688_platform_driver = {
	.driver = {
		.name = "qcom,gn_sunny_ov4688",
		.owner = THIS_MODULE,
		.of_match_table = gn_sunny_ov4688_dt_match,
	},
};
static void gn_sunny_ov4688_write_cmos_sensor(uint32_t addr, uint16_t value)
{
    gn_sunny_ov4688_s_ctrl.sensor_i2c_client->i2c_func_tbl->i2c_write(
            gn_sunny_ov4688_s_ctrl.sensor_i2c_client,
            addr,value,MSM_CAMERA_I2C_BYTE_DATA);      
}

static uint8_t gn_sunny_4688_read_cmos_sensor(uint16_t addr)
{
    int32_t rc = 0;
    uint16_t value;
    rc=gn_sunny_ov4688_s_ctrl.sensor_i2c_client->i2c_func_tbl->i2c_read(
			gn_sunny_ov4688_s_ctrl.sensor_i2c_client,
			addr,&value, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        printk("%s: %d: read failed\n", __func__,rc);
		return rc;
	}
    return value;
}
// return: 0, group index is empty
// 1, group index has invalid data
// 2, group index has valid data
int check_otp_wb(int index)
{
	int flag, i;
	int address_start, address_end;
	if(index==1) {
		address_start = 0x7110;
		address_end = 0x7110;
	}
	else if(index==2) {
		address_start = 0x7120;
		address_end = 0x7120;
	}
	else {
		address_start = 0x7130;
		address_end = 0x7130;
	}
	// read otp into buffer
	gn_sunny_ov4688_write_cmos_sensor(0x3d84, 0xc0); // program disable, manual mode
	//partial mode OTP write start address
	gn_sunny_ov4688_write_cmos_sensor(0x3d88, (address_start>>8));
	gn_sunny_ov4688_write_cmos_sensor(0x3d89, (address_start & 0xff));
	// partial mode OTP write end address
	gn_sunny_ov4688_write_cmos_sensor(0x3d8A, (address_end>>8));
	gn_sunny_ov4688_write_cmos_sensor(0x3d8B, (address_end & 0xff));
	gn_sunny_ov4688_write_cmos_sensor(0x3d81, 0x01); // read OTP
	mdelay(5);
	//select group
	flag = gn_sunny_4688_read_cmos_sensor(address_start);
	// clear otp buffer
	for (i=address_start;i<=address_end;i++) {
		gn_sunny_ov4688_write_cmos_sensor(i, 0x00);
	}
	if (flag == 0x00) {
		return 0;
	}
	else if (flag & 0x02) {
		return 1;
	}
	else {
		return 2;
	}
}
// index: index of otp group. (1, 2, 3)
// code: 0 for start, 1 for end
// return: 0, group index is empty
// 1, group index has invalid data
// 2, group index has valid data
int check_otp_VCM(int index, int code)
{
	int flag, i;
	int address_start, address_end;
	if(index==1) {
		address_start = 0x7140 + code*2;
		address_end = 0x7140 + code*2;
	}
	else if(index==2) {
		address_start = 0x7144 + code*2;
		address_end = 0x7144 + code*2;
	}
	else {
		address_start = 0x7148 + code*2;
		address_end = 0x7148 + code*2;
	}
	// read otp into buffer
	gn_sunny_ov4688_write_cmos_sensor(0x3d84, 0xc0); // program disable, manual mode
	//partial mode OTP write start address
	gn_sunny_ov4688_write_cmos_sensor(0x3d88, (address_start>>8));
	gn_sunny_ov4688_write_cmos_sensor(0x3d89, (address_start & 0xff));
	// partial mode OTP write end address
	gn_sunny_ov4688_write_cmos_sensor(0x3d8A, (address_end>>8));
	gn_sunny_ov4688_write_cmos_sensor(0x3d8B, (address_end & 0xff));
	gn_sunny_ov4688_write_cmos_sensor(0x3d81, 0x01);
	mdelay(5);
	//select group
	flag = gn_sunny_4688_read_cmos_sensor(address_start);
	flag = flag & 0xc0;
	// clear otp buffer
	for (i=address_start;i<=address_end;i++) {
		gn_sunny_ov4688_write_cmos_sensor(i, 0x00);
	}
	if (flag == 0x00) {
	return 0;
	}
	else if (flag & 0x80) {
	return 1;
	}
	else {
	return 2;
	}
}
// index: index of otp group. (1, 2, 3)
// otp_ptr: pointer of otp_struct
// return: 1,
int read_otp_wb(int index, struct gn_sunny_ov4688_otp_struct *otp_ptr)
{
	int i, temp;
	int address_start, address_end;
	// read otp into buffer
	gn_sunny_ov4688_write_cmos_sensor(0x3d84, 0xc0); // program disable, manual mode
	//select group
	if(index==1)
	{
		address_start = 0x7111;
		address_end = 0x711f;
	}
	else if(index==2)
	{
		address_start = 0x7121;
		address_end = 0x712f;
	}
	else
	{
		address_start = 0x7131;
		address_end = 0x713f;
	}
	//partial mode OTP write start address
	gn_sunny_ov4688_write_cmos_sensor(0x3d88, (address_start>>8));
	gn_sunny_ov4688_write_cmos_sensor(0x3d89, (address_start & 0xff));
	// partial mode OTP write end address
	gn_sunny_ov4688_write_cmos_sensor(0x3d8A, (address_end>>8));
	gn_sunny_ov4688_write_cmos_sensor(0x3d8B, (address_end & 0xff));
	gn_sunny_ov4688_write_cmos_sensor(0x3d81, 0x01); // load otp into buffer
	mdelay(5);
	otp_ptr->module_integrator_id = gn_sunny_4688_read_cmos_sensor(address_start);
	otp_ptr->lens_id = gn_sunny_4688_read_cmos_sensor(address_start + 1);
	otp_ptr->production_year = gn_sunny_4688_read_cmos_sensor(address_start + 2);
	otp_ptr->production_month = gn_sunny_4688_read_cmos_sensor( address_start + 3);
	otp_ptr->production_day = gn_sunny_4688_read_cmos_sensor(address_start + 4);
	temp = gn_sunny_4688_read_cmos_sensor(address_start + 9);
	otp_ptr->rg_ratio = (gn_sunny_4688_read_cmos_sensor(address_start + 5)<<2) + ((temp>>6) & 0x03);
	otp_ptr->bg_ratio = (gn_sunny_4688_read_cmos_sensor(address_start + 6)<<2) + ((temp>>4) & 0x03);
	otp_ptr->light_rg = (gn_sunny_4688_read_cmos_sensor(address_start + 7) <<2) + ((temp>>2) & 0x03);
	otp_ptr->light_bg = (gn_sunny_4688_read_cmos_sensor(address_start + 8)<<2) + (temp & 0x03);
	otp_ptr->user_data[0] = gn_sunny_4688_read_cmos_sensor(address_start + 10);
	otp_ptr->user_data[1] = gn_sunny_4688_read_cmos_sensor(address_start + 11);
	otp_ptr->user_data[2] = gn_sunny_4688_read_cmos_sensor(address_start + 12);
	otp_ptr->user_data[3] = gn_sunny_4688_read_cmos_sensor(address_start + 13);
	otp_ptr->user_data[4] = gn_sunny_4688_read_cmos_sensor(address_start + 14);

	printk("%s, rg_ratio 0x%x, bg_ratio 0x%x, light_rg 0x%x, light_bg 0x%x\n", __func__,
		otp_ptr->rg_ratio, otp_ptr->bg_ratio, otp_ptr->light_rg, otp_ptr->light_bg);
	// clear otp buffer
	for (i=address_start;i<=address_end;i++) {
		gn_sunny_ov4688_write_cmos_sensor(i, 0x00);
	}
	return 1;
}

// index: index of otp group. (1, 2, 3)
// code: 0 start code, 1 stop code
// return: 1
int read_otp_VCM(int index, int code, struct gn_sunny_ov4688_otp_struct * otp_ptr)
{
	int i, temp, vcm;
	int address;
	int address_start, address_end;
	// read otp into buffer
	gn_sunny_ov4688_write_cmos_sensor(0x3d84, 0xc0); // program disable, manual mode
	//check group
	if(index==1)
	{
		address_start = 0x7140;
		address_end = 0x7143;
	}
	else if(index==2)
	{
		address_start = 0x7144;
		address_end = 0x7147;
	}
	else
	{
		address_start = 0x7148;
		address_end = 0x714b;
	}
	//partial mode OTP write start address
	gn_sunny_ov4688_write_cmos_sensor(0x3d88, (address_start>>8));
	gn_sunny_ov4688_write_cmos_sensor(0x3d89, (address_start & 0xff));
	// partial mode OTP write end address
	gn_sunny_ov4688_write_cmos_sensor(0x3d8A, (address_end>>8));
	gn_sunny_ov4688_write_cmos_sensor(0x3d8B, (address_end & 0xff));
	gn_sunny_ov4688_write_cmos_sensor(0x3d81, 0x01); // load otp into buffer
	mdelay(5);
	//flag and lsb of VCM start code
	address = address_start + code*2;
	temp = gn_sunny_4688_read_cmos_sensor(address);
	vcm = (gn_sunny_4688_read_cmos_sensor(address_start + 1) << 2) | (temp & 0x03);
	if(code==1) {
		otp_ptr->VCM_end = vcm;
	}
	else {
		otp_ptr->VCM_start = vcm;
	}
	// clear otp buffer
	for (i=address_start;i<=address_end;i++) {
		gn_sunny_ov4688_write_cmos_sensor(i, 0x00);
	}
	return 1;
}
// R_gain, sensor red gain of AWB, 0x400 =1
// G_gain, sensor green gain of AWB, 0x400 =1
// B_gain, sensor blue gain of AWB, 0x400 =1
// return 1;
int update_awb_gain(int R_gain, int G_gain, int B_gain)
{
	printk("%s, R_gain 0x%x, R_gain 0x%x, R_gain 0x%x \n", __func__,
		R_gain, G_gain, B_gain);
	
	if (R_gain>0x400) {
		//long WB R gain
		gn_sunny_ov4688_write_cmos_sensor(0x500c, R_gain>>8);
		gn_sunny_ov4688_write_cmos_sensor(0x500d, R_gain & 0x00ff);
		//middle WB R gain
		gn_sunny_ov4688_write_cmos_sensor(0x5012, R_gain>>8);
		gn_sunny_ov4688_write_cmos_sensor(0x5013, R_gain & 0x00ff);
		//short WB R gain
		gn_sunny_ov4688_write_cmos_sensor(0x5018, R_gain>>8);
		gn_sunny_ov4688_write_cmos_sensor(0x5019, R_gain & 0x00ff);
	}
	if (G_gain>0x400) {
		//long WB G gain
		gn_sunny_ov4688_write_cmos_sensor(0x500e, G_gain>>8);
		gn_sunny_ov4688_write_cmos_sensor(0x500f, G_gain & 0x00ff);
		//middle WB G gain
		gn_sunny_ov4688_write_cmos_sensor(0x5014, G_gain>>8);
		gn_sunny_ov4688_write_cmos_sensor(0x5015, G_gain & 0x00ff);
		//short WB G gain
		gn_sunny_ov4688_write_cmos_sensor(0x501A, G_gain>>8);
		gn_sunny_ov4688_write_cmos_sensor(0x501B, G_gain & 0x00ff);
	}
	if (B_gain>0x400) {
		//long WB B gain
		gn_sunny_ov4688_write_cmos_sensor(0x5010, B_gain>>8);
		gn_sunny_ov4688_write_cmos_sensor(0x5011, B_gain & 0x00ff);
		//middle WB B gain
		gn_sunny_ov4688_write_cmos_sensor(0x5016, B_gain>>8);
		gn_sunny_ov4688_write_cmos_sensor(0x5017, B_gain & 0x00ff);
		//short WB B gain
		gn_sunny_ov4688_write_cmos_sensor(0x501C, B_gain>>8);
		gn_sunny_ov4688_write_cmos_sensor(0x501D, B_gain & 0x00ff);
	}
#ifdef ORIGINAL_VERSION
#else
	gn_sunny_ov4688_write_cmos_sensor(0x5000, 0xf3);
#endif
	
	return 1;
}

// call this function after OV4688 initialization
// return value: 0 update success
// 1, no OTP
int32_t RG_Ratio_Typical = RG_TYPICAL;
int32_t BG_Ratio_Typical = BG_TYPICAL;
static int32_t update_otp_wb(struct msm_sensor_ctrl_t *s_ctrl)
{
	struct gn_sunny_ov4688_otp_struct current_otp;
	int i;
	int otp_index;// bank 1,2,3
	int temp;
	int R_gain, G_gain, B_gain, G_gain_R, G_gain_B;
	int rg,bg;
	// R/G and B/G of current camera module is read out from sensor OTP
	// check first OTP with valid data
	for(i=1;i<4;i++) {
	temp = check_otp_wb(i);
	if (temp == 2) {
		otp_index = i;
		break;
	}
	}
	if (i==4) {
		// no valid wb OTP data
		return 0;
	}
	// set right bank
	read_otp_wb(otp_index, &current_otp);
	if(current_otp.light_rg==0) {
		// no light source information in OTP, light factor = 1
		rg = current_otp.rg_ratio;
	}
	else {
		rg = current_otp.rg_ratio * (current_otp.light_rg + 512) / 1024;
	}
	if(current_otp.light_bg==0) {
		// not light source information in OTP, light factor = 1
		bg = current_otp.bg_ratio;
	}
	else {
		bg = current_otp.bg_ratio * (current_otp.light_bg + 512) / 1024;
	}
	//calculate G gain
	//0x400 = 1x gain
	if(bg < BG_Ratio_Typical) {
		if (rg< RG_Ratio_Typical) {
			// current_otp.bg_ratio < BG_Ratio_typical &&
			// current_otp.rg_ratio < RG_Ratio_typical
			G_gain = 0x400;
			B_gain = 0x400 * BG_Ratio_Typical / bg;
			R_gain = 0x400 * RG_Ratio_Typical / rg;
		}
		else {
			// current_otp.bg_ratio < BG_Ratio_typical &&
			// current_otp.rg_ratio >= RG_Ratio_typical
			R_gain = 0x400;
			G_gain = 0x400 * rg / RG_Ratio_Typical;
			B_gain = G_gain * BG_Ratio_Typical /bg;
		}
	}
	else {
		if (rg < RG_Ratio_Typical) {
		// current_otp.bg_ratio >= BG_Ratio_typical &&
		// current_otp.rg_ratio < RG_Ratio_typical
		B_gain = 0x400;
		G_gain = 0x400 * bg / BG_Ratio_Typical;
		R_gain = G_gain * RG_Ratio_Typical / rg;
		}
		else {
			// current_otp.bg_ratio >= BG_Ratio_typical &&
			// current_otp.rg_ratio >= RG_Ratio_typical
			G_gain_B = 0x400 * bg / BG_Ratio_Typical;
			G_gain_R = 0x400 * rg / RG_Ratio_Typical;
			if(G_gain_B > G_gain_R ) {
				B_gain = 0x400;
				G_gain = G_gain_B;
				R_gain = G_gain * RG_Ratio_Typical /rg;
			}
			else {
				R_gain = 0x400;
				G_gain = G_gain_R;
				B_gain = G_gain * BG_Ratio_Typical / bg;
			}
		}
	}
	update_awb_gain(R_gain, G_gain, B_gain);
    return 1;
}
static struct gn_otp_sensor_fn_t gn_otp_func = {  
    .gn_sensor_otp_support = update_otp_wb,
};



static int32_t gn_sunny_ov4688_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(gn_sunny_ov4688_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init gn_sunny_ov4688_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&gn_sunny_ov4688_platform_driver,
		gn_sunny_ov4688_platform_probe);
	if (!rc){
	#if defined(CONFIG_GN_Q_BSP_DEVICE_TYPE_CHECK_SUPPORT)
		gn_cameradev_info.gn_dev_type = GN_DEVICE_TYPE_FRONT_CAM;
		memcpy(gn_cameradev_info.name, "sunny_ov4688",sizeof("sunny_ov4688"));
		memcpy(gn_cameradev_info.vendor,"sunny_ov4688",sizeof("sunny_ov4688"));
		gn_set_device_info(gn_cameradev_info);
	#endif
		return rc;
	}
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&gn_sunny_ov4688_i2c_driver);
}

static void __exit gn_sunny_ov4688_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (gn_sunny_ov4688_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&gn_sunny_ov4688_s_ctrl);
		platform_driver_unregister(&gn_sunny_ov4688_platform_driver);
	} else
		i2c_del_driver(&gn_sunny_ov4688_i2c_driver);
	return;
}

static struct msm_sensor_ctrl_t gn_sunny_ov4688_s_ctrl = {
	.sensor_i2c_client = &gn_sunny_ov4688_sensor_i2c_client,
	.power_setting_array.power_setting = gn_sunny_ov4688_power_setting,
	.power_setting_array.size = ARRAY_SIZE(gn_sunny_ov4688_power_setting),
	.msm_sensor_mutex = &gn_sunny_ov4688_mut,
	.sensor_v4l2_subdev_info = gn_sunny_ov4688_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(gn_sunny_ov4688_subdev_info),
	.gn_otp_func_tbl = &gn_otp_func, 
};
module_init(gn_sunny_ov4688_init_module);
module_exit(gn_sunny_ov4688_exit_module);
MODULE_DESCRIPTION("gn_sunny_ov4688");
MODULE_LICENSE("GPL v2");
