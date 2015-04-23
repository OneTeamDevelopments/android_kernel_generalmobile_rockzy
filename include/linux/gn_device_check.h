/* Copyright Statement:
 *
 */

#ifndef __GN_DEVICE_CHECK_H__
#define __GN_DEVICE_CHECK_H__

/*static int gn_device_debug_enable = 1;
#define GN_DEVICE_DEBUG(format, args...) do{ \
	if(gn_device_debug_enable) \
	{\
		printk(KERN_EMERG format,##args);\
	}\
}while(0)
*/
#define GN_DEVICE_NAME_LEN 30
#define GN_DEVICE_VENDOR_LEN 20
#define GN_DEVICE_VERSION_LEN 20

enum gn_device_type
{
    GN_DEVICE_TYPE_LCD = 0,
    GN_DEVICE_TYPE_CPU,
    GN_DEVICE_TYPE_ACCELEROMETER, 
    GN_DEVICE_TYPE_TP, 
	//GN_DEVICE_TYPE_TP_IC,
	//GN_DEVICE_TYPE_TP_LCM,
    GN_DEVICE_TYPE_MAGNETIC_FIELD,
    GN_DEVICE_TYPE_GYROSCOPE,
    GN_DEVICE_TYPE_LIGHT ,
    GN_DEVICE_TYPE_PRESSURE,
    GN_DEVICE_TYPE_TEMPERATURE,
    GN_DEVICE_TYPE_PROXIMITY,
    GN_DEVICE_TYPE_CAP_KEY,
    GN_DEVICE_TYPE_BACK_CAM,
    GN_DEVICE_TYPE_FRONT_CAM,
    GN_DEVICE_TYPE_FM,
    GN_DEVICE_TYPE_TOTAL, 
};

struct gn_device_info
{
	char     name[GN_DEVICE_NAME_LEN];				
	char     vendor[GN_DEVICE_VENDOR_LEN];				
	char     version[GN_DEVICE_VERSION_LEN];			
	int	    gn_dev_type;								
};

//#ifdef CUSTOM_KERNEL_LCM
#define GN_DEVICE_LCD
//#endif
#define GN_DEVICE_CPU

//#ifdef CUSTOM_KERNEL_ACCELEROMETER
#define GN_DEVICE_ACCELEROMETER
//#endif

//#ifdef CUSTOM_KERNEL_TOUCHPANEL
#define GN_DEVICE_TOUCHPANEL
//#endif

//#ifdef CUSTOM_KERNEL_MAGNETOMETER
#define GN_DEVICE_MAGNETIC_FIELD
//#endif

//#ifdef GN_DEVICE_BACK_CAMERA
#define GN_DEVICE_BACK_CAMERA
//#endif

//#ifdef GN_DEVICE_FRONT_CAMERA
#define GN_DEVICE_FRONT_CAMERA
//#endif

#endif // __GN_DEVICE_CHECK_H__
