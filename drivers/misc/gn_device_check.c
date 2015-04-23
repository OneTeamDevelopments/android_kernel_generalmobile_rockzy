/* Copyright Statement:
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <asm/atomic.h>
#include <asm/uaccess.h>
 
#include <linux/gn_device_check.h>

#define GN_DEVICE_NAME	"gn_device_check"

 struct gn_device_info dev_info[GN_DEVICE_TYPE_TOTAL];

struct gn_device_info gn_dev_info;
void gn_set_device_info(struct gn_device_info gn_dev_info)
{
	printk("%s\n", gn_dev_info.name);
	strcpy(dev_info[gn_dev_info.gn_dev_type].name, gn_dev_info.name);
	strcpy(dev_info[gn_dev_info.gn_dev_type].vendor, gn_dev_info.vendor);
	strcpy(dev_info[gn_dev_info.gn_dev_type].version, gn_dev_info.version);
}

static ssize_t gn_device_name(struct device *dev,struct device_attribute *attr, char *buf_name)
{
	int len=0;
	#ifdef GN_DEVICE_LCD
	len+=snprintf(buf_name+len, GN_DEVICE_NAME_LEN,"LCD: %s\n", dev_info[GN_DEVICE_TYPE_LCD].name);
	#endif
	#ifdef GN_DEVICE_CPU
	len+=snprintf(buf_name+len, GN_DEVICE_NAME_LEN,"CPU: %s\n", dev_info[GN_DEVICE_TYPE_CPU].name);
	#endif
	#ifdef GN_DEVICE_ACCELEROMETER
	len+=snprintf(buf_name+len,GN_DEVICE_NAME_LEN, "G-sensor: %s\n", dev_info[GN_DEVICE_TYPE_ACCELEROMETER].name);
	#endif	
	#ifdef GN_DEVICE_TOUCHPANEL
	len+=snprintf(buf_name+len, GN_DEVICE_NAME_LEN,"TP: %s\n", dev_info[GN_DEVICE_TYPE_TP].name);
    #endif
	#ifdef GN_DEVICE_MAGNETIC_FIELD
	len+=snprintf(buf_name+len,GN_DEVICE_NAME_LEN, "%s\n", dev_info[GN_DEVICE_TYPE_MAGNETIC_FIELD].name);
	#endif
	#ifdef GN_DEVICE_GYROSCOPE
	len+=snprintf(buf_name+len, GN_DEVICE_NAME_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_GYROSCOPE].name);
	#endif
	#ifdef GN_DEVICE_LIGHT
	len+=snprintf(buf_name+len,GN_DEVICE_NAME_LEN, "%s\n", dev_info[GN_DEVICE_TYPE_LIGHT].name);
	#endif
	#ifdef GN_DEVICE_PROXIMITY
	len+=snprintf(buf_name+len, GN_DEVICE_NAME_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_PROXIMITY].name);
	#endif
	#ifdef GN_DEVICE_CAP_KEY
	len+=snprintf(buf_name+len, GN_DEVICE_NAME_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_CAP_KEY].name);
	#endif
	#ifdef GN_DEVICE_TYPE_CPU
	len+=snprintf(buf_name+len, GN_DEVICE_NAME_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_CPU].name);
	#endif
    #ifdef GN_DEVICE_BACK_CAMERA
	len+=snprintf(buf_name+len, GN_DEVICE_NAME_LEN,"Main_CAM:%s\n", dev_info[GN_DEVICE_TYPE_BACK_CAM].name);
	#endif
    #ifdef GN_DEVICE_FRONT_CAMERA
	len+=snprintf(buf_name+len, GN_DEVICE_NAME_LEN,"SUB_CAM:%s\n", dev_info[GN_DEVICE_TYPE_FRONT_CAM].name);
	#endif
    return len;
}

static ssize_t gn_device_vendor(struct device *dev,struct device_attribute *attr, char *buf_vendor)
{
	int len=0;
	#ifdef GN_DEVICE_LCD
	len+=snprintf(buf_vendor+len, GN_DEVICE_VENDOR_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_LCD].vendor);
	#endif
	#ifdef GN_DEVICE_ACCELEROMETER
	len+=snprintf(buf_vendor+len,GN_DEVICE_VENDOR_LEN, "%s\n", dev_info[GN_DEVICE_TYPE_ACCELEROMETER].vendor);
	#endif
	#ifdef GN_DEVICE_TOUCHPANEL
	len+=snprintf(buf_vendor+len, GN_DEVICE_VENDOR_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_TP].vendor);
	#endif
	#ifdef GN_DEVICE_MAGNETIC_FIELD
	len+=snprintf(buf_vendor+len, GN_DEVICE_VENDOR_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_MAGNETIC_FIELD].vendor);
	#endif
	#ifdef GN_DEVICE_GYROSCOPE
	len+=snprintf(buf_vendor+len, GN_DEVICE_VENDOR_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_GYROSCOPE].vendor);
	#endif
	#ifdef GN_DEVICE_LIGHT
	len+=snprintf(buf_vendor+len,GN_DEVICE_VENDOR_LEN, "%s\n", dev_info[GN_DEVICE_TYPE_LIGHT].vendor);
	#endif
	#ifdef GN_DEVICE_PROXIMITY
	len+=snprintf(buf_vendor+len, GN_DEVICE_VENDOR_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_PROXIMITY].vendor);
	#endif
	#ifdef GN_DEVICE_CAP_KEY
	len+=snprintf(buf_vendor+len, GN_DEVICE_VENDOR_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_CAP_KEY].vendor);
	#endif
	#ifdef GN_DEVICE_TYPE_CPU
	len+=snprintf(buf_vendor+len, GN_DEVICE_VENDOR_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_CPU].vendor);
	#endif
    #ifdef GN_DEVICE_BACK_CAMERA
	len+=snprintf(buf_vendor+len, GN_DEVICE_VENDOR_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_BACK_CAM].vendor);
    #endif
	#ifdef GN_DEVICE_FRONT_CAMERA
	len+=snprintf(buf_vendor+len, GN_DEVICE_VENDOR_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_FRONT_CAM].vendor);
    #endif
return len;	
}

static ssize_t gn_device_version(struct device *dev,struct device_attribute *attr, char *buf_version)
{	
	int len=0;
	#ifdef GN_DEVICE_LCD
	len+=snprintf(buf_version+len, GN_DEVICE_VERSION_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_LCD].version);
	#endif
	#ifdef GN_DEVICE_CPU
	len+=snprintf(buf_version+len, GN_DEVICE_VERSION_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_CPU].version);
	#endif
	#ifdef GN_DEVICE_ACCELEROMETER
	len+=snprintf(buf_version+len,GN_DEVICE_VERSION_LEN, "%s\n", dev_info[GN_DEVICE_TYPE_ACCELEROMETER].version);
	#endif
	#ifdef GN_DEVICE_TOUCHPANEL
	len+=snprintf(buf_version+len,GN_DEVICE_VERSION_LEN, "%s\n", dev_info[GN_DEVICE_TYPE_TP].version);
	#endif
	#ifdef GN_DEVICE_MAGNETIC_FIELD
	len+=snprintf(buf_version+len, GN_DEVICE_VERSION_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_MAGNETIC_FIELD].version);
	#endif
	#ifdef GN_DEVICE_GYROSCOPE
	len+=snprintf(buf_version+len,GN_DEVICE_VERSION_LEN, "%s\n", dev_info[GN_DEVICE_TYPE_GYROSCOPE].version);
	#endif
	#ifdef GN_DEVICE_LIGHT
	len+=snprintf(buf_version+len, GN_DEVICE_VERSION_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_LIGHT].version);
	#endif
	#ifdef GN_DEVICE_PROXIMITY
	len+=snprintf(buf_version+len, GN_DEVICE_VERSION_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_PROXIMITY].version);
	#endif
	#ifdef GN_DEVICE_CAP_KEY
	len+=snprintf(buf_version+len, GN_DEVICE_VERSION_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_CAP_KEY].version);
	#endif
	#ifdef GN_DEVICE_TYPE_CPU
	len+=snprintf(buf_version+len, GN_DEVICE_VERSION_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_CPU].version);
	#endif
    #ifdef GN_DEVICE_BACK_CAMERA
	len+=snprintf(buf_version+len, GN_DEVICE_VERSION_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_BACK_CAM].version);
    #endif
	#ifdef GN_DEVICE_FRONT_CAMERA
	len+=snprintf(buf_version+len, GN_DEVICE_VERSION_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_FRONT_CAM].version);
    #endif
return len;	
}

DEVICE_ATTR(name,     S_IWUSR | S_IRUGO, gn_device_name, NULL);
DEVICE_ATTR(vendor,     S_IWUSR | S_IRUGO, gn_device_vendor, NULL);
DEVICE_ATTR(version,     S_IWUSR | S_IRUGO, gn_device_version, NULL);

static struct device_attribute *gn_device_attr_list[] =
{
	&dev_attr_name,
	&dev_attr_vendor,
	&dev_attr_version,
};

static int gn_device_create_attr(struct device *dev) 
{
	int idx, err = 0;
	int num = (int)(sizeof(gn_device_attr_list)/sizeof(gn_device_attr_list[0]));
	printk("%s\n", __func__);
	if(!dev)
	{
		return -EINVAL;
	}	

	for(idx = 0; idx < num; idx++)
	{
		if((err = device_create_file(dev, gn_device_attr_list[idx])))
		{            
			printk("wanglei22: device_create_file (%s) = %d\n", gn_device_attr_list[idx]->attr.name, err);        
			break;
		}
	}

	return err;
}

static int gn_device_pdrv_probe(struct platform_device *pdev)
{
	int err=0;
       printk("wanglei22: gn_device_pdrv_probe\n");
	if(gn_device_create_attr(&pdev->dev) != 0)
	{
		printk("wanglei22:unable to create attributes!!\n");
		goto exit_create_attr_failed;
	}
	return 0;
exit_create_attr_failed:
	printk("%s: err = %d\n", __func__, err);        
	return err;
}

/* should never be called */
static int gn_device_pdrv_remove(struct platform_device *pdev)
{
	printk("%s\n", __func__);
	return 0;
}

#ifdef CONFIG_OF
/*static struct of_device_id gn_check_match_table[] = {
	{ .compatible = GN_DEVICE_NAME,},
	{ },
};*/
#else
#define gn_check_match_table NULL
#endif

static struct platform_driver gn_device_pdrv = {
	.probe		= gn_device_pdrv_probe,
	.remove		= gn_device_pdrv_remove,
	.driver		= {
		.name	= GN_DEVICE_NAME,
		.owner	= THIS_MODULE,
		//.of_match_table = gn_check_match_table,
	},
};

static struct platform_device gn_device_check_pdev = {
       .name   = GN_DEVICE_NAME,
       .id     = -1,
};

static int gn_device_mod_init(void)
{
	int ret = 0;
	printk("%s\n", __func__);
	ret = platform_driver_register(&gn_device_pdrv);
	if (ret) {
		printk("gn_device_mod_init --> register driver failed (%d)\n", ret);
		return ret;
	}

       ret = platform_device_register(&gn_device_check_pdev);
	if (ret != 0){
              printk("gn_device_mod_init --> register device failed (%d)\n", ret);
		return ret;
	}

	return ret;
}

/* should never be called */
static void __exit gn_device_mod_exit(void)
{
	printk("%s\n", __func__);
}

//module_init core_initcall
module_init(gn_device_mod_init);
module_exit(gn_device_mod_exit);

MODULE_AUTHOR("<www.gionee.com>");
MODULE_DESCRIPTION("gn_device_check Driver v0.1");
MODULE_LICENSE("GPL");
