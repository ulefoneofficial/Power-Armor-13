/***********************************************************
 *  版权所有 (C) 2015-2020, 深圳市铂睿智恒科技有限公司 
 *
 *  文件名称: hall_device.c
 *  内容摘要: hall driver for hall device
 *  当前版本: V1.0
 *  作    者: 丁俊
 *  完成日期: 2015-04-10
 *  修改记录: 
 *  修改日期: 
 *  版本号  :
 *  修改人  :
 ***********************************************************/
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/ctype.h>
#include <linux/semaphore.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/workqueue.h>
#include <linux/delay.h>

#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/kthread.h>
#include <linux/input.h>
#if defined(CONFIG_PM_WAKELOCKS)
#include <linux/pm_wakeup.h>
#else
#include <linux/wakelock.h>
#endif
#include <linux/time.h>

#include <linux/string.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
/*----------------------------------------------------------------------
static variable defination
----------------------------------------------------------------------*/
#define FM_ANT_DEVNAME    "fm_ant_dev"

#define EN_DEBUG

#if defined(EN_DEBUG)
		
#define TRACE_FUNC 	printk("[fm_ant_dev] function: %s, line: %d \n", __func__, __LINE__);

#define FM_ANT_DEBUG  printk
#else

#define TRACE_FUNC(x,...)

#define FM_ANT_DEBUG(x,...)
#endif
/*
&pio {
	fm_ant_lna_default: fm_ant_lna_default {
	};
	fm_ant_lna_en_high: fm_ant_lna_en_high {
		pins_cmd_dat {
			pinmux = <PINMUX_GPIO153__FUNC_GPIO153>;
			slew-rate = <1>;
			output-high;
		};
	};
	fm_ant_lna_en_low: fm_ant_lna_en_low {
		pins_cmd_dat {
			pinmux = <PINMUX_GPIO153__FUNC_GPIO153>;
			slew-rate = <1>;
			output-low;
		};
	};
	fm_ant_switch_en_high: fm_ant_switch_en_high {
		pins_cmd_dat {
			pinmux = <PINMUX_GPIO165__FUNC_GPIO165>;
			slew-rate = <1>;
			output-high;
		};
	};
	fm_ant_switch_en_low: fm_ant_switch_en_low {
		pins_cmd_dat {
			pinmux = <PINMUX_GPIO165__FUNC_GPIO165>;
			slew-rate = <1>;
			output-low;
		};
	};
	fm_ant_switch_sel_high: fm_ant_switch_sel_high {
		pins_cmd_dat {
			pinmux = <PINMUX_GPIO150__FUNC_GPIO150>;
			slew-rate = <1>;
			output-high;
		};
	};
	fm_ant_switch_sel_low: fm_ant_switch_sel_low {
		pins_cmd_dat {
			pinmux = <PINMUX_GPIO150__FUNC_GPIO150>;
			slew-rate = <1>;
			output-low;
		};
	};
};
&odm {
	fm_ant: fm_ant {
		compatible = "prize,fm_ant";
		pinctrl-names = "default", "lna_en_high","lna_en_low","switch_en_high","switch_en_low","switch_sel_high","switch_sel_low";
		pinctrl-0 = <&fm_ant_lna_default>;
		pinctrl-1 = <&fm_ant_lna_en_high>;
		pinctrl-2 = <&fm_ant_lna_en_low>;
		pinctrl-3 = <&fm_ant_switch_en_high>;
		pinctrl-4 = <&fm_ant_switch_en_low>;
		pinctrl-5 = <&fm_ant_switch_sel_high>;
		pinctrl-6 = <&fm_ant_switch_sel_low>;
	};
};


*/

/****************************************************************/
/*******static function defination                             **/
/****************************************************************/

#define  FM_CLOSE_SWITCH_CLOSE   0
#define  FM_OPEN_SWITCH_D2    1
#define  FM_OPEN_SWITCH_D1    2

//static struct kobject *fm_ant_sys_device;
static volatile int cur_fm_en_status = FM_CLOSE_SWITCH_CLOSE;


static struct pinctrl *fm_ant_pinctrl;
static struct pinctrl_state *fm_ant_default;
static struct pinctrl_state *fm_ant_lna_en_high;
static struct pinctrl_state *fm_ant_lna_en_low;
static struct pinctrl_state *fm_ant_switch_en_high;
static struct pinctrl_state *fm_ant_switch_en_low;
static struct pinctrl_state *fm_ant_switch_sel_high;
static struct pinctrl_state *fm_ant_switch_sel_low;


static ssize_t device_fm_ant_status_show(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
	FM_ANT_DEBUG("[fm_ant_dev] cur_fm_en_status=%d\n", cur_fm_en_status);
	return sprintf(buf, "%u\n", cur_fm_en_status);
}
static ssize_t device_fm_ant_status_store(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t size)
{
	FM_ANT_DEBUG("[fm_ant_dev] %s ON/OFF value = %d:\n ", __func__, cur_fm_en_status);
	if(sscanf(buf, "%u", &cur_fm_en_status) != 1)
	{
		FM_ANT_DEBUG("[fm_ant_dev]: Invalid values\n");
		return -EINVAL;
	}
	switch(cur_fm_en_status)
	{
		case 2:
		//fm ant enable   切换到D1 
		pinctrl_select_state(fm_ant_pinctrl, fm_ant_lna_en_high);
		pinctrl_select_state(fm_ant_pinctrl, fm_ant_switch_en_low);
		pinctrl_select_state(fm_ant_pinctrl, fm_ant_switch_sel_low);
		break;
		case 1:
		//fm ant enable  插入耳机切换到D2
		pinctrl_select_state(fm_ant_pinctrl, fm_ant_lna_en_low);
		pinctrl_select_state(fm_ant_pinctrl, fm_ant_switch_en_low);
		pinctrl_select_state(fm_ant_pinctrl, fm_ant_switch_sel_high);
		break;
		case 0:
		//fm ant diable
		pinctrl_select_state(fm_ant_pinctrl, fm_ant_lna_en_low);
		pinctrl_select_state(fm_ant_pinctrl, fm_ant_switch_en_high);
		pinctrl_select_state(fm_ant_pinctrl, fm_ant_switch_sel_low);
		break;
		default:
		FM_ANT_DEBUG("[fm_ant_dev]: Invalid values_%d\n",cur_fm_en_status);
		break;
	}
	return size;
}

static DEVICE_ATTR(fm_ant_status, S_IRUGO|S_IWUSR|S_IWGRP, device_fm_ant_status_show, device_fm_ant_status_store);


static int fm_ant_get_dts_fun(struct platform_device *pdev)
{
	int ret = 0;
	fm_ant_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(fm_ant_pinctrl)) {
		FM_ANT_DEBUG("Cannot find fm ant pinctrl!");
		ret = PTR_ERR(fm_ant_pinctrl);
	}
	//fm ant eint pin initialization 
	fm_ant_default= pinctrl_lookup_state(fm_ant_pinctrl, "default");
	if (IS_ERR(fm_ant_default)) {
		ret = PTR_ERR(fm_ant_default);
		FM_ANT_DEBUG("%s : init err, fm_ant_default\n", __func__);
	}

	fm_ant_lna_en_high = pinctrl_lookup_state(fm_ant_pinctrl, "lna_en_high");
	if (IS_ERR(fm_ant_lna_en_high)) {
		ret = PTR_ERR(fm_ant_lna_en_high);
		FM_ANT_DEBUG("%s : init err, fm_ant_lna_en_high\n", __func__);
	}

	fm_ant_lna_en_low = pinctrl_lookup_state(fm_ant_pinctrl, "lna_en_low");
	if (IS_ERR(fm_ant_lna_en_low)) {
		ret = PTR_ERR(fm_ant_lna_en_low);
		FM_ANT_DEBUG("%s : init err, fm_ant_lna_en_low\n", __func__);
	}
	fm_ant_switch_en_high = pinctrl_lookup_state(fm_ant_pinctrl, "switch_en_high");
	if (IS_ERR(fm_ant_switch_en_high)) {
		ret = PTR_ERR(fm_ant_switch_en_high);
		FM_ANT_DEBUG("%s : init err, fm_ant_switch_en_high\n", __func__);
	}
	fm_ant_switch_en_low = pinctrl_lookup_state(fm_ant_pinctrl, "switch_en_low");
	if (IS_ERR(fm_ant_switch_en_low)) {
		ret = PTR_ERR(fm_ant_switch_en_low);
		FM_ANT_DEBUG("%s : init err, fm_ant_switch_en_low\n", __func__);
	}
	
	fm_ant_switch_sel_high = pinctrl_lookup_state(fm_ant_pinctrl, "switch_sel_high");
	if (IS_ERR(fm_ant_switch_sel_high)) {
		ret = PTR_ERR(fm_ant_switch_sel_high);
		FM_ANT_DEBUG("%s : init err, fm_ant_switch_sel_high\n", __func__);
	}
	fm_ant_switch_sel_low = pinctrl_lookup_state(fm_ant_pinctrl, "switch_sel_low");
	if (IS_ERR(fm_ant_switch_sel_low)) {
		ret = PTR_ERR(fm_ant_switch_sel_low);
		FM_ANT_DEBUG("%s : init err, fm_ant_switch_sel_low\n", __func__);
	}
	//设置状态0
	pinctrl_select_state(fm_ant_pinctrl, fm_ant_lna_en_low);
	pinctrl_select_state(fm_ant_pinctrl, fm_ant_switch_en_high);
	pinctrl_select_state(fm_ant_pinctrl, fm_ant_switch_sel_low);
	return ret;
}
/****************************************************************/
/*******export function defination                             **/
/****************************************************************/
struct class *fm_ant_class;
static int fm_ant_probe(struct platform_device *pdev)
{
	   //int ret = 0;
	   struct device *fm_ant_dev;
	   TRACE_FUNC;
       fm_ant_get_dts_fun(pdev);
	  
	   fm_ant_class = class_create(THIS_MODULE, "fm_ant");
	
	if (IS_ERR(fm_ant_class)) {
		FM_ANT_DEBUG("Failed to create class(fm_ant_class)!");
		return PTR_ERR(fm_ant_class);
	}

	fm_ant_dev = device_create(fm_ant_class, NULL, 0, NULL, "fm_ant_data");
	if (IS_ERR(fm_ant_dev))
		FM_ANT_DEBUG("Failed to create fm_ant_dev device");
	
	if (device_create_file(fm_ant_dev, &dev_attr_fm_ant_status) < 0)
		FM_ANT_DEBUG("Failed to create device file(%s)!",dev_attr_fm_ant_status.attr.name);	
#if 0
	   fm_ant_sys_device = kobject_create_and_add("fm_ant_state", NULL);
	   if (fm_ant_sys_device == NULL)
	   {
				FM_ANT_DEBUG("[fm_ant_dev]:%s: subsystem_register failed\n", __func__);
				ret = -ENXIO;
				return ret ;
	   }
		ret = sysfs_create_file(fm_ant_sys_device, &dev_attr_fm_ant_status.attr);
		if (ret) 
		{
			FM_ANT_DEBUG("[fm_ant_dev]:%s: sysfs_create_file failed\n", __func__);
			kobject_del(fm_ant_sys_device);
		}
#endif
		return 0;
}

static int fm_ant_remove(struct platform_device *dev)	
{
	FM_ANT_DEBUG("[fm_ant_dev]:fm_ant_remove begin!\n");
	class_destroy(fm_ant_class);
	FM_ANT_DEBUG("[fm_ant_dev]:fm_ant_remove Done!\n");
	return 0;
}
static const struct of_device_id fm_ant_dt_match[] = {
	{.compatible = "prize,fm_ant"},
	{},
};

static struct platform_driver fm_ant_driver = {
	.probe	= fm_ant_probe,
	.remove  = fm_ant_remove,
	.driver    = {
		.name       = "Fm_Ant_Driver",
		.of_match_table = of_match_ptr(fm_ant_dt_match),
	},
};

static int __init fm_ant_init(void)
{
    int retval = 0;
    TRACE_FUNC;
	//  retval = platform_device_register(&hall_device);
    printk("[%s]: fm_ant_driver, retval=%d \n!", __func__, retval);
	  if (retval != 0) {
		  return retval;
	  }
    platform_driver_register(&fm_ant_driver);
    return 0;
}

static void __exit fm_ant_exit(void)
{
    TRACE_FUNC;
    platform_driver_unregister(&fm_ant_driver);
}

module_init(fm_ant_init);
module_exit(fm_ant_exit);
MODULE_DESCRIPTION("FM ANT DEVICE driver");
MODULE_AUTHOR("huangjiwu <huangjiwu@boruizhiheng.com>");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("FMANTdevice");

