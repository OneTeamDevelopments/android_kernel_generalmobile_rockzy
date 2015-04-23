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
#define GN_SUNNY_OV8835_SENSOR_NAME "gn_sunny_ov8835"
#define RG_TYPICAL 0x138   
#define BG_TYPICAL 0x127

#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

DEFINE_MSM_MUTEX(gn_sunny_ov8835_mut);
static int8_t gn_otp_wb_count=0;
static int8_t gn_otp_lenc_count=0;
static struct gn_sunny_ov8835_otp_struct *current_otp_wb;
static struct gn_sunny_ov8835_otp_struct *current_otp_lenc;

static struct msm_sensor_ctrl_t gn_sunny_ov8835_s_ctrl;

static struct msm_sensor_power_setting gn_sunny_ov8835_power_setting[] = {
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
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_AF_PWDM,   //SUB VCM GPIO POWER
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
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

static struct v4l2_subdev_info gn_sunny_ov8835_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id gn_sunny_ov8835_i2c_id[] = {
	{GN_SUNNY_OV8835_SENSOR_NAME, (kernel_ulong_t)&gn_sunny_ov8835_s_ctrl},
	{ }
};

static int32_t msm_gn_sunny_ov8835_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &gn_sunny_ov8835_s_ctrl);
}
static struct i2c_driver gn_sunny_ov8835_i2c_driver = {
	.id_table = gn_sunny_ov8835_i2c_id,
	.probe  = msm_gn_sunny_ov8835_i2c_probe,
	.driver = {
		.name = GN_SUNNY_OV8835_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client gn_sunny_ov8835_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id gn_sunny_ov8835_dt_match[] = {
	{.compatible = "qcom,gn_sunny_ov8835", .data = &gn_sunny_ov8835_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, gn_sunny_ov8835_dt_match);

static struct platform_driver gn_sunny_ov8835_platform_driver = {
	.driver = {
		.name = "qcom,gn_sunny_ov8835",
		.owner = THIS_MODULE,
		.of_match_table = gn_sunny_ov8835_dt_match,
	},
};
static void gn_sunny_ov8835_write_cmos_sensor(uint32_t addr, uint16_t value)
{
    gn_sunny_ov8835_s_ctrl.sensor_i2c_client->i2c_func_tbl->i2c_write(
            gn_sunny_ov8835_s_ctrl.sensor_i2c_client,
            addr,value,MSM_CAMERA_I2C_BYTE_DATA);      
}

static uint8_t gn_sunny_ov8835_read_cmos_sensor(uint16_t addr)
{
    int32_t rc = 0;
    uint16_t value;
    rc=gn_sunny_ov8835_s_ctrl.sensor_i2c_client->i2c_func_tbl->i2c_read(
			gn_sunny_ov8835_s_ctrl.sensor_i2c_client,
			addr,&value, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        printk("%s: %d: read failed\n", __func__,rc);
		return rc;
	}
    return value;
}

static int16_t gn_sunny_ov8835_check_otp_wb(int16_t index)
{
   	int16_t flag,i,bank;
   	int32_t address;
  	//select bank index
   	bank=0xc0|(index);
   	gn_sunny_ov8835_write_cmos_sensor(0x3d84,bank);//OTP Program disable and manual memory band enable 		
   	//read otp to buffer
   	gn_sunny_ov8835_write_cmos_sensor(0x3d81,0x01);//Trigger Load OTP
   	mdelay(10);
   	//read flag
   	address = 0x3d00;
   	flag = gn_sunny_ov8835_read_cmos_sensor(address);
      //disable otp read
   	gn_sunny_ov8835_write_cmos_sensor(0x3d81,0x00);
	//clear otp buffer
   	for(i=0;i<16;i++)
   	{
   	gn_sunny_ov8835_write_cmos_sensor(0x3d00+i,0x00);
   	}
   	flag=flag&0xc0;
   	if(!flag)
		{
			CDBG("[gn_sunny_ov8835_check_otp_wb_empty_index[%x]read flag[%x][0]\n",index,flag);
			return 0;	
		}
	else if(flag==0x40)
		{
			CDBG("[gn_sunny_ov8835_check_otp_wb_valid _data_index[%x]read flag[%x][2]\n",index,flag);
			return 2;
		}
	else
		{
			CDBG("[gn_sunny_ov8835_check_otp_wb_invalid_data_index[%x]read flag[%x][1]\n",index,flag);
		  	return 1;
		}

}
static int16_t gn_sunny_ov8835_read_otp_wb(int16_t index, struct gn_sunny_ov8835_otp_struct *otp)
{
    int16_t i,bank;
    int32_t address;
    int16_t AWB_light_LSB,rg_ratio_MSB,bg_ratio_MSB,light_rg_MSB,light_bg_MSB;
    //select bank index
    bank=0xc0|index;
    gn_sunny_ov8835_write_cmos_sensor(0x3d84,bank);
    //read otp to buffer
    gn_sunny_ov8835_write_cmos_sensor(0x3d81,0x01);
    mdelay(10);	
    address = 0x3d00;
    otp->module_integrator_id = gn_sunny_ov8835_read_cmos_sensor(address+1);
    otp->lens_id = gn_sunny_ov8835_read_cmos_sensor(address+2);
    //otp->rg_ratio = (gn_sunny_ov8835_read_cmos_sensor(address+6))<<2+(gn_sunny_ov8835_read_cmos_sensor(address+10)&0xc0)>>6;
    //otp->bg_ratio =(gn_sunny_ov8835_read_cmos_sensor(address+7))<<2+(gn_sunny_ov8835_read_cmos_sensor(address+10)&0x30)>>4;
    //otp->light_rg = gn_sunny_ov8835_read_cmos_sensor(address+8)<<2+gn_sunny_ov8835_read_cmos_sensor(address+10)&0x0c>>2;
    //otp->light_bg = gn_sunny_ov8835_read_cmos_sensor(address+9)<<2+gn_sunny_ov8835_read_cmos_sensor(address+10)&0x03;
    rg_ratio_MSB=gn_sunny_ov8835_read_cmos_sensor(address+6);
    bg_ratio_MSB=gn_sunny_ov8835_read_cmos_sensor(address+7);
    light_rg_MSB=gn_sunny_ov8835_read_cmos_sensor(address+8);
    light_bg_MSB=gn_sunny_ov8835_read_cmos_sensor(address+9);
    AWB_light_LSB=gn_sunny_ov8835_read_cmos_sensor(address+10);
   
    otp->rg_ratio=(rg_ratio_MSB<<2)|((AWB_light_LSB&0xc0)>>6);
    otp->bg_ratio=(bg_ratio_MSB<<2)|((AWB_light_LSB&0x30)>>4);

    otp->light_rg=(light_rg_MSB<<2)|((AWB_light_LSB&0x0c)>>2);
    otp->light_bg=(light_bg_MSB<<2)|(AWB_light_LSB&0x03);
    otp->user_data[0] = gn_sunny_ov8835_read_cmos_sensor(address+11);
    otp->user_data[1] = gn_sunny_ov8835_read_cmos_sensor(address+12);
    otp->user_data[2] = gn_sunny_ov8835_read_cmos_sensor(address+13);
    otp->user_data[3] = gn_sunny_ov8835_read_cmos_sensor(address+14);
    otp->user_data[4] = gn_sunny_ov8835_read_cmos_sensor(address+15);

    CDBG("[gn_sunny_ov8835_read_otp_wb]address[%x]module_integrator_id[%x]\n",address,otp->module_integrator_id);
    CDBG("[gn_sunny_ov8835_read_otp_wb]address[%x]lens_id[%x]\n",address,otp->lens_id);
    CDBG("[gn_sunny_ov8835_read_otp_wb]address[%x]rg_ratio[%x]\n",address,otp->rg_ratio);
    CDBG("[gn_sunny_ov8835_read_otp_wb]address[%x]bg_ratio[%x]\n",address,otp->bg_ratio);
    CDBG("[gn_sunny_ov8835_read_otp_wb]address[%x]light_rg[%x]\n",address,otp->light_rg);
    CDBG("[gn_sunny_ov8835_read_otp_wb]address[%x]light_bg[%x]\n",address,otp->light_bg);
    CDBG("[gn_sunny_ov8835_read_otp_wb]address[%x]user_data[0][%x]\n",address,otp->user_data[0]);
    CDBG("[gn_sunny_ov8835_read_otp_wb]address[%x]user_data[1][%x]\n",address,otp->user_data[1]);
    CDBG("[gn_sunny_ov8835_read_otp_wb]address[%x]user_data[2][%x]\n",address,otp->user_data[2]);	
    CDBG("[gn_sunny_ov8835_read_otp_wb]address[%x]user_data[3][%x]\n",address,otp->user_data[3]);
    CDBG("[gn_sunny_ov8835_read_otp_wb]address[%x]user_data[4][%x]\n",address,otp->user_data[4]);

    //disable otp read
    gn_sunny_ov8835_write_cmos_sensor(0x3d81,0x00);
    //clear otp buffer
    for(i =0; i<16; i++)
    {
     gn_sunny_ov8835_write_cmos_sensor(0x3d00+i, 0x00);
    }
    if (otp->rg_ratio==0||otp->bg_ratio==0)
    {
     CDBG("[Read_otp_wb_Error]");
     return 0;
    }
    return 1;
}
static void gn_sunny_ov8835_update_wb_gain(int32_t R_gain, int32_t G_gain, int32_t B_gain)
{   
	if(R_gain > 0x400)
		{
			gn_sunny_ov8835_write_cmos_sensor(0x3400,R_gain >> 8);
			gn_sunny_ov8835_write_cmos_sensor(0x3401,(R_gain&0x00ff));
		}
	if(G_gain > 0x400)
		{
			gn_sunny_ov8835_write_cmos_sensor(0x3402,G_gain >> 8);
			gn_sunny_ov8835_write_cmos_sensor(0x3403,(G_gain&0x00ff));
		}
	if(B_gain >0x400)
		{
			gn_sunny_ov8835_write_cmos_sensor(0x3404,B_gain >> 8);
			gn_sunny_ov8835_write_cmos_sensor(0x3405,(B_gain&0x00ff));
		}
	CDBG("[gn_sunny_ov8835_update_wb_gain]R_gain[%x]G_gain[%x]B_gain[%x]\n",R_gain,G_gain,B_gain);
	CDBG("[gn_sunny_ov8835_update_wb_gain_Finished]");
}
//R/G and B/G ratio of typical camera module is defined here

int32_t mRG_Ratio_typical = RG_TYPICAL;
int32_t mBG_Ratio_typical = BG_TYPICAL;
static int16_t gn_sunny_ov8835_update_wb_register_from_otp(void)
{
	int16_t temp, i, otp_index,rg,bg;
	//struct gn_sunny_ov8835_otp_struct current_otp; //wb
	int32_t R_gain, B_gain, G_gain, G_gain_R,G_gain_B;
	CDBG("gn_sunny_ov8835_update_wb_register_from_otp_Start\n");
	//check first wb OTP with valid OTP
	for(i = 1; i < 4; i++)
	{
		temp = gn_sunny_ov8835_check_otp_wb(i);
		if(temp == 2)
		{
		otp_index = i;
		break;
		}
	}
	if( i == 4)
	{
	CDBG("[gn_sunny_ov8835_update_wb_register_from_otp]no valid wb OTP data!\r\n");
	return 0;
	}
        if(gn_otp_wb_count <= 0) 
        {	

		if(!gn_sunny_ov8835_read_otp_wb(otp_index,current_otp_wb)) 
		{
		return 0;
		}    
                gn_otp_wb_count++;        
        }	
//calculate gain
	//0x400 = 1x gain
	if((*current_otp_wb).light_rg==0)
	{
	//if no light source information in OTP,light factor=1;
	rg=(*current_otp_wb).rg_ratio;
	}
	else
	{
	rg = (*current_otp_wb).rg_ratio * ((*current_otp_wb).light_rg +512) / 1024;   
	}

	if((*current_otp_wb).light_bg==0)
	{
	//if no light source information in OTP,light factor=1;
	bg=(*current_otp_wb).bg_ratio;
	}
	else
	{
	bg = (*current_otp_wb).bg_ratio * ((*current_otp_wb).light_bg +512) / 1024;  
	}
	//calculate G gain
	//0x400=1xgain
	if(bg < mBG_Ratio_typical)
	{
	if(rg < mRG_Ratio_typical)
		{
			//current_opt.bg_ratio < mBG_Ratio_typical &&
			//cuttent_otp.rg < mRG_Ratio_typical
			G_gain = 0x400;
			B_gain = 0x400 * mBG_Ratio_typical /bg;
			R_gain = 0x400 * mRG_Ratio_typical /rg;
		}
		else
		{
			//current_otp.bg_ratio < mBG_Ratio_typical &&
			//current_otp.rg_ratio >= mRG_Ratio_typical
			R_gain = 0x400;
			G_gain = 0x400 * rg / mRG_Ratio_typical;
			B_gain = G_gain * mBG_Ratio_typical /bg;
		}
		}
	else
       {
		if(rg < mRG_Ratio_typical)
		{
			//current_otp.bg_ratio >= mBG_Ratio_typical &&
			//current_otp.rg_ratio < mRG_Ratio_typical
			B_gain = 0x400;
			G_gain = 0x400 * bg/ mBG_Ratio_typical;
			R_gain = G_gain * mRG_Ratio_typical /rg;	
		}
		else
		{
			//current_otp.bg_ratio >= mBG_Ratio_typical &&
			//current_otp.rg_ratio >= mRG_Ratio_typical
			G_gain_B = 0x400*bg/ mBG_Ratio_typical;
			G_gain_R = 0x400*rg/ mRG_Ratio_typical;	
			if(G_gain_B > G_gain_R)
			{
				B_gain = 0x400;
				G_gain = G_gain_B;
				R_gain = G_gain * mRG_Ratio_typical /rg;
			}
			else
			{
				R_gain = 0x400;
				G_gain = G_gain_R;
				B_gain = G_gain * mBG_Ratio_typical /bg;
			}		        
		}
	}
	//write sensor wb gain to register
	gn_sunny_ov8835_update_wb_gain(R_gain,G_gain,B_gain);
	pr_err("[gn_sunny_ov8835_update_wb_register_from_otp_Finished\n]");
    return 1;
}
static int16_t gn_sunny_ov8835_check_otp_lenc(int16_t index)
{
   	int16_t flag,i,bank;
   	int32_t address;
   	//select bank :index*4
   	bank=0xc0|(index<<2);
   	gn_sunny_ov8835_write_cmos_sensor(0x3d84,bank);
   	//read otp into buffer 
   	gn_sunny_ov8835_write_cmos_sensor(0x3d81,0x01);
   	mdelay(10);
   	//read flag
   	address = 0x3d00; 
   	flag = gn_sunny_ov8835_read_cmos_sensor(address);
   	//disable otp read
   	gn_sunny_ov8835_write_cmos_sensor(0x3d81,0x00);
   	//clear otp buffer
   	for(i = 0;i<16; i++)
   	{
   		gn_sunny_ov8835_write_cmos_sensor(0x3d00+i,0x00);
   	}
   	flag = flag & 0xc0;
   	if(!flag)
   	{
   	    CDBG("[gn_sunny_ov8835_check_otp_lenc]index[%x]read flag[%x][0]\n",index,flag);
   	    return 0;
   	}
   	else if(flag==0x40)
   	{
   	    CDBG("[gn_sunny_ov8835_check_otp_lenc]index[%x]read flag[%x][2]\n",index,flag);
   	    return 2;
   	}
   else
   	{
   	    CDBG("[gn_sunny_ov8835_check_otp_lenc]index[%x]read flag[%x][1]\n",index,flag);
	    return 1;
   	}

}
static int16_t gn_sunny_ov8835_read_otp_lenc(int16_t index,struct gn_sunny_ov8835_otp_struct *otp)
{
	int16_t bank,i,k;
	int32_t address;
	 //select bank: index*4;
	bank = 0xc0+(index<<2);
	gn_sunny_ov8835_write_cmos_sensor(0x3d84,bank);

	//read otp into buffer
	gn_sunny_ov8835_write_cmos_sensor(0x3d81,0x01);
	mdelay(10);
	
	address = 0x3d01;
	for(i = 0; i < 15; i++)
	{
		otp->lenc[i] = gn_sunny_ov8835_read_cmos_sensor(address);
		address++;
   	}
       
	//disabe otp read
	gn_sunny_ov8835_write_cmos_sensor(0x3d81,0x00);

	//clear otp buffer
	for(i=0;i<16;i++)
	{
	gn_sunny_ov8835_write_cmos_sensor(0x3d00+i,0x00);
	}

	//select 2nd bank
	bank++;
	gn_sunny_ov8835_write_cmos_sensor(0x3d84,bank);

	//read otp
	gn_sunny_ov8835_write_cmos_sensor(0x3d81,0x01);
	mdelay(10);

    address = 0x3d00;
    for(i = 15; i < 31; i++)
    {
        otp->lenc[i] = gn_sunny_ov8835_read_cmos_sensor(address);
        address++;
    }

	//disabe otp read
	gn_sunny_ov8835_write_cmos_sensor(0x3d81,0x00);
	//clear otp buffer
	for(i=0;i<16;i++)
	{
	gn_sunny_ov8835_write_cmos_sensor(0x3d00+i,0x00);
	}

	//select 3rd bank
	bank++;
	gn_sunny_ov8835_write_cmos_sensor(0x3d84,bank);

	//read otp
	gn_sunny_ov8835_write_cmos_sensor(0x3d81,0x01);
	mdelay(10);

	address = 0x3d00;
	for(i = 31; i < 47; i++)
	{
    	otp->lenc[i] = gn_sunny_ov8835_read_cmos_sensor(address);
        address++;
	}

     //disabe otp read
	gn_sunny_ov8835_write_cmos_sensor(0x3d81,0x00);
	//clear otp buffer
	for(i=0;i<16;i++)
	{
	gn_sunny_ov8835_write_cmos_sensor(0x3d00+i,0x00);
	}

	//select 4th bank
	bank++;
	gn_sunny_ov8835_write_cmos_sensor(0x3d84,bank);

	//read otp
	gn_sunny_ov8835_write_cmos_sensor(0x3d81,0x01);
	mdelay(10);

	address = 0x3d00;
	for(i = 47; i < 62; i++)
	{
    	otp->lenc[i] = gn_sunny_ov8835_read_cmos_sensor(address);
        address++;
	}

	//disabe otp read
	gn_sunny_ov8835_write_cmos_sensor(0x3d81,0x00);
	if((otp->lenc[0]==0)||(otp->lenc[31]==0)||(otp->lenc[61]==0))
       	{
       	CDBG("[gn_sunny_ov8835_read_lenc_Error]\n");
	  return 0;
       	}
	for(k = 0; k < 62; k++)
	{
              CDBG("[gn_sunny_ov8835_read_lenc]otp->lenc[%d][%x]\n",k,otp->lenc[k]);
	}
    //clear otp buffer
	for(i=0;i<16;i++)
	{
		gn_sunny_ov8835_write_cmos_sensor(0x3d00+i,0x00);
	}
	return 1;
}
static void gn_sunny_ov8835_update_lenc(struct gn_sunny_ov8835_otp_struct *otp)
{
        int16_t i;
        //int16_t temp;
	//temp=0x80|otp->lenc[0];
	//gn_sunny_ov8835_write_cmos_sensor(0x5800,temp);
	//lenc g
        for(i = 0; i < 62; i++)
        {
                gn_sunny_ov8835_write_cmos_sensor(0x5800+i,otp->lenc[i]);
                CDBG("[gn_sunny_ov8835_update_lenc]otp->lenc[%d][%x]\n",i,otp->lenc[i]);
        }
        CDBG("[gn_sunny_ov8835_update_lenc_Finished]\n");
}
static int16_t gn_sunny_ov8835_update_lenc_register_from_otp(void)
{
    int16_t temp,i,otp_index;
    //struct gn_sunny_ov8835_otp_struct current_otp;  //lenc
	
    CDBG("gn_sunny_ov8835_update_lenc_register_from_otp_Start\n");
    for(i = 1; i<4;i++)
	{
		temp = gn_sunny_ov8835_check_otp_lenc(i);
		if(2 == temp)
		{
			otp_index = i;
			break;
		}
	}
	if(i == 4) 
		{
		 	CDBG("[gn_sunny_ov8835_update_lenc_register_from_otp]no valid wb OTP data!\r\n");
			return 0;
		}
        if(gn_otp_lenc_count <= 0) 
        {	

		if(!gn_sunny_ov8835_read_otp_lenc(otp_index,current_otp_lenc))
		{
		return 0;
		}
                gn_otp_lenc_count++;
        }
	gn_sunny_ov8835_update_lenc(current_otp_lenc);
	pr_err("gn_sunny_ov8835_update_lenc_register_from_otp_Finished\n");
	return 1;
}
static int32_t gn_sunny_ov8835_otp_support(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	rc = gn_sunny_ov8835_update_wb_register_from_otp();
	if(0 == rc){
	pr_err("gn_sunny_ov8835_update_wb_register_from_otp invalid\n");
	}else if(1 == rc){
	pr_err("gn_sunny_ov8835_update_wb_register_from_otp success\n");
	}

	rc = gn_sunny_ov8835_update_lenc_register_from_otp();
	if(0 == rc){
	pr_err("gn_sunny_ov8835_update_lenc_register_from_otp invalid\n");
	}else if(1 == rc)
	{
	pr_err("gn_sunny_ov8835_update_lenc_register_from_otp success\n");
	}

	return rc;
}
static int gn_sunny_ov8835_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(gn_sunny_ov8835_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init gn_sunny_ov8835_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);
	current_otp_wb   = kzalloc(sizeof(struct gn_sunny_ov8835_otp_struct),GFP_KERNEL);
        current_otp_lenc = kzalloc(sizeof(struct gn_sunny_ov8835_otp_struct),GFP_KERNEL);
	rc = platform_driver_probe(&gn_sunny_ov8835_platform_driver,
		gn_sunny_ov8835_platform_probe);
	if (!rc)
		return rc;
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&gn_sunny_ov8835_i2c_driver);
}

static void __exit gn_sunny_ov8835_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (gn_sunny_ov8835_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&gn_sunny_ov8835_s_ctrl);
		platform_driver_unregister(&gn_sunny_ov8835_platform_driver);
	} else
		i2c_del_driver(&gn_sunny_ov8835_i2c_driver);
        kzfree(current_otp_wb);
        kzfree(current_otp_lenc);
	return;
}

static struct gn_otp_sensor_fn_t gn_otp_func = {  
	.gn_sensor_otp_support = gn_sunny_ov8835_otp_support,

};
static struct msm_sensor_ctrl_t gn_sunny_ov8835_s_ctrl = {
	.sensor_i2c_client = &gn_sunny_ov8835_sensor_i2c_client,
	.power_setting_array.power_setting = gn_sunny_ov8835_power_setting,
	.power_setting_array.size = ARRAY_SIZE(gn_sunny_ov8835_power_setting),
	.msm_sensor_mutex = &gn_sunny_ov8835_mut,
	.sensor_v4l2_subdev_info = gn_sunny_ov8835_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(gn_sunny_ov8835_subdev_info),
        .gn_otp_func_tbl = &gn_otp_func,  
};
module_init(gn_sunny_ov8835_init_module);
module_exit(gn_sunny_ov8835_exit_module);
MODULE_DESCRIPTION("gn_sunny_ov8835");
MODULE_LICENSE("GPL v2");
