/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */


#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/module.h>
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
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/workqueue.h>
#include <linux/delay.h>

#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/input.h>
#include <linux/time.h>

#include <linux/sysfs.h>




#define UHF_DEVNAME "uhf_dev"


static int uhf_remove(struct platform_device *dev);
static int uhf_probe(struct platform_device *pdev);
static void uhf_shutdown(struct platform_device *dev);
void uhf_rfid_pwr(u8 enable);

static const struct of_device_id uhf_of_match[] = {
	{.compatible = "mediatek,uhf"},
	{},
};
MODULE_DEVICE_TABLE(of, uhf_of_match);

//prize add by lipengpeng 20210330 start 
static int air_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	printk("lpp air enter suspend\n");
	uhf_rfid_pwr(0);// close air chip  20210421 setgpio 0
        return 0;
}

static int air_resume(struct platform_device *pdev)
{
	printk("lpp air enter resume\n");
	//uhf_rfid_pwr(0);// open air chip
        return 0;
}
//prize add by lipengpeng 20210330 end

static struct platform_driver uhf_platform_driver = {
	.probe = uhf_probe,
	.remove = uhf_remove,
	.shutdown = uhf_shutdown,
//prize add by lipengpeng 20210330 start 
	.suspend = air_suspend,
    .resume = air_resume,
//prize add by lipengpeng 20210330 end 
	.driver = {
		   .name = UHF_DEVNAME,
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = uhf_of_match,
#endif
	},
};

/*----------------------------------------------------------------------------*/

#ifdef CONFIG_PINCTRL
static struct pinctrl *uhf_gpio;
static struct pinctrl_state *uhfprt_gpio_funen0;
static struct pinctrl_state *uhfprt_gpio_funen1;
static struct pinctrl_state *uhf_3V_enable0;
static struct pinctrl_state *uhf_3V_enable1;
static struct pinctrl_state *uhf_ir_boost0;
static struct pinctrl_state *uhf_ir_boost1;
static struct pinctrl_state *uhf_tof_pwr0;
static struct pinctrl_state *uhf_tof_pwr1;

#if 1
static unsigned int gpio_uhfprt_en;
static unsigned int gpio_uhf_3V_enable;
static unsigned int gpio_uhf_ir_boost;
static unsigned int gpio_tof_rfid_pwr;
#endif
#endif

static int uhf_gpio_init(struct device	*dev)
{    
	int ret=0;
//	unsigned int mode;
	//const struct of_device_id *match;

	pr_debug("[uhf][GPIO] enter %s, %d\n", __func__, __LINE__);

	uhf_gpio = devm_pinctrl_get(dev);
	if (IS_ERR(uhf_gpio)) {
		ret = PTR_ERR(uhf_gpio);
		pr_info("[uhf][ERROR] Cannot find uhf_gpio!\n");
		return ret;
	}

	uhfprt_gpio_funen0 = pinctrl_lookup_state(uhf_gpio, "uhfprt_gpio_funen0");
	if (IS_ERR(uhfprt_gpio_funen0)) {
		ret = PTR_ERR(uhfprt_gpio_funen0);
		pr_info("[uhf][ERROR] Cannot find uhfprt_gpio_funen0 %d!\n",
			ret);
	}
	uhfprt_gpio_funen1= pinctrl_lookup_state(uhf_gpio, "uhfprt_gpio_funen1");
	if (IS_ERR(uhfprt_gpio_funen1)) {
		ret = PTR_ERR(uhfprt_gpio_funen1);
		pr_info("[uhf][ERROR] Cannot find uhfprt_gpio_funen1 %d!\n",
			ret);
	}
	uhf_3V_enable0 = pinctrl_lookup_state(uhf_gpio, "uhf_3V_enable0");
	if (IS_ERR(uhf_3V_enable0)) {
		ret = PTR_ERR(uhf_3V_enable0);
		pr_info("[uhf][ERROR] Cannot find uhf_3V_enable0 %d!\n",
			ret);
	}
	uhf_3V_enable1= pinctrl_lookup_state(uhf_gpio, "uhf_3V_enable1");
	if (IS_ERR(uhf_3V_enable1)) {
		ret = PTR_ERR(uhf_3V_enable1);
		pr_info("[uhf][ERROR] Cannot find uhf_3V_enable1 %d!\n",
			ret);
	}
 	uhf_ir_boost0 = pinctrl_lookup_state(uhf_gpio, "uhf_ir_boost0");
	if (IS_ERR(uhf_ir_boost0)) {
		ret = PTR_ERR(uhf_ir_boost0);
		pr_info("[uhf][ERROR] Cannot find uhf_ir_boost0 %d!\n",
			ret);
	}
	uhf_ir_boost1= pinctrl_lookup_state(uhf_gpio, "uhf_ir_boost1");
	if (IS_ERR(uhf_ir_boost1)) {
		ret = PTR_ERR(uhf_ir_boost1);
		pr_info("[uhf][ERROR] Cannot find uhf_ir_boost1 %d!\n",
			ret);
	}
	uhf_tof_pwr0 = pinctrl_lookup_state(uhf_gpio, "uhf_tof_pwr0");
	if (IS_ERR(uhf_tof_pwr0)) {
		ret = PTR_ERR(uhf_tof_pwr0);
		pr_info("[uhf][ERROR] Cannot find uhf_tof_pwr0 %d!\n",
			ret);
	}
	uhf_tof_pwr1= pinctrl_lookup_state(uhf_gpio, "uhf_tof_pwr1");
	if (IS_ERR(uhf_tof_pwr1)) {
		ret = PTR_ERR(uhf_tof_pwr1);
		pr_info("[uhf][ERROR] Cannot find uhf_tof_pwr1 %d!\n",
			ret);
	}
//prize add by lipengpeng 20210401 start
	//pinctrl_select_state(uhf_gpio,uhf_tof_pwr1); // default mode set WAKEUP gpio low statu to close chip
//prize add by lipengpeng 20210401 end 
	gpio_uhfprt_en =of_get_named_gpio(dev->of_node, "gpio_uhfprt_en", 0);
	gpio_uhf_3V_enable =of_get_named_gpio(dev->of_node, "gpio_uhf_3V_enable", 0);
	gpio_uhf_ir_boost =of_get_named_gpio(dev->of_node, "gpio_uhf_ir_boost", 0);
	gpio_tof_rfid_pwr =of_get_named_gpio(dev->of_node, "gpio_uhf_tof_pwr", 0);



	printk("[uhf][GPIO] uhf_gpio_get_info end!\n");

    return ret;

}
void uhfprt_enable(u8 enable)
{
    printk("%s enable =%d\n", __func__);

    if(enable)
        pinctrl_select_state(uhf_gpio,uhfprt_gpio_funen1);
    else
        pinctrl_select_state(uhf_gpio,uhfprt_gpio_funen0);
 
}
//prize add by lipengpeng 20210221 start 
void uhfprt_3V_enable(u8 enable)
{
    printk("%s enable =%d\n", __func__);

    if(enable)
        pinctrl_select_state(uhf_gpio,uhf_3V_enable1);
    else
        pinctrl_select_state(uhf_gpio,uhf_3V_enable0);
 
}
//prize add by lipengpeng 20210221 end 
void uhf_5V_enable(u8 enable)
{
    pr_debug("%s enable =%d\n", __func__);

    if(enable)
        pinctrl_select_state(uhf_gpio,uhf_3V_enable1);
    else
        pinctrl_select_state(uhf_gpio,uhf_3V_enable0);
        
}
void uhf_5V_boost(u8 enable)
{
    pr_debug("%s enable =%d\n", __func__);

    if(enable)
        pinctrl_select_state(uhf_gpio,uhf_ir_boost1);
    else
        pinctrl_select_state(uhf_gpio,uhf_ir_boost0);
        
}
void uhf_rfid_pwr(u8 enable)
{
    pr_debug("%s enable =%d\n", __func__);

    if(enable)
        pinctrl_select_state(uhf_gpio,uhf_tof_pwr1);
    else
        pinctrl_select_state(uhf_gpio,uhf_tof_pwr0);

}
/***************************************
node:/sys/bus/platform/drivers/uhf_dev/uhf_3V_funen
1:使能UART 3V3电压
0:关闭UART 3V3电压
***********************************/
static ssize_t uhf_3V_funen_show(struct device_driver *ddri, char *buf)
{
	int uhf_3V_value = 0;
    ssize_t res;


    uhf_3V_value = __gpio_get_value(gpio_uhfprt_en);
    
	res = snprintf(buf, PAGE_SIZE, "uhf_3V_value = %d\n",uhf_3V_value);

	return res;
}

static ssize_t uhf_3V_funen_store(struct device_driver *ddri,
				      const char *buf, size_t tCount)
{

    int uhf_3V_flag;
	int ret = 0;

	if (strlen(buf) < 1) {
		pr_notice("%s() Invalid input!!\n", __func__);
		return -EINVAL;
	}

    ret = sscanf(buf, "%d", &uhf_3V_flag);


	if (uhf_3V_flag == 1){
//prize add by lipengpeng 20210330 start 
		//uhf_rfid_pwr(0);// open air chip 
		//mdelay(10);
//prize add by lipengpeng 20210330 end
		uhfprt_enable(1);
		uhfprt_3V_enable(1);//prize add by lipengpeng 20210221 start 
     }
	else{
		uhfprt_enable(0);
		uhfprt_3V_enable(0);//prize add by lipengpeng 20210221 start 
//prize add by lipengpeng 20210330 start 
		//uhf_rfid_pwr(1);// close air chip 
//prize add by lipengpeng 20210330 end
    }
   
	return tCount;
}  
/***************************************
node:/sys/bus/platform/drivers/uhf_dev/uhf_5V_boost
1:使能uhf 5V电压
0:关闭uhf 5V电压
***********************************/                      
static ssize_t uhf_5V_boost_show(struct device_driver *ddri, char *buf)
{
  int uhf_5V_boost_value = 0;
  int uhf_5V_value = 0;
  ssize_t res;


  uhf_5V_boost_value = __gpio_get_value(gpio_uhf_3V_enable);
  uhf_5V_value = __gpio_get_value(gpio_uhf_ir_boost);

  res = snprintf(buf, PAGE_SIZE, "uhf_5V_boost_value = %d,uhf_5V_value = %d\n",uhf_5V_boost_value,uhf_5V_value);

  return res;
}

static ssize_t uhf_5V_boost_store(struct device_driver *ddri,
                    const char *buf, size_t tCount)
{

  int uhf_5V_boost_flag;
  int ret = 0;

  if (strlen(buf) < 1) {
      pr_notice("%s() Invalid input!!\n", __func__);
      return -EINVAL;
  }

  ret = sscanf(buf, "%d", &uhf_5V_boost_flag);


  if (uhf_5V_boost_flag == 1){
      uhf_5V_enable(1);
      uhf_5V_boost(1);
   }
  else{
      uhf_5V_enable(0);
      uhf_5V_boost(0);
  }
 
  return tCount;
}   
/***************************************
node:/sys/bus/platform/drivers/uhf_dev/uhf_rfid_pwr
1:使能单片机RFID 3.3V电压
0:关闭单片机RFID 3.3V电压
***********************************/ 
static ssize_t uhf_tof_pwr_show(struct device_driver *ddri, char *buf)
{
  int uhf_rfid_pwr_value = 0;
  int uhf_rfid_pwr_gpio_value = 0;
  ssize_t res;

  uhf_rfid_pwr_gpio_value = __gpio_get_value(gpio_tof_rfid_pwr);

  if(uhf_rfid_pwr_gpio_value == 0)
	  uhf_rfid_pwr_value = 1;
  else
	  uhf_rfid_pwr_value = 0;
  res = snprintf(buf, PAGE_SIZE, "gpio = %d uhf_rfid_pwr_value = %d\n",uhf_rfid_pwr_gpio_value,uhf_rfid_pwr_value);

  return res;
}

static ssize_t uhf_tof_pwr_store(struct device_driver *ddri,
                    const char *buf, size_t tCount)
{

  int uhf_rfid_pwr_flag;
  int ret = 0;

  if (strlen(buf) < 1) {
      pr_notice("%s() Invalid input!!\n", __func__);
      return -EINVAL;
  }

  ret = sscanf(buf, "%d", &uhf_rfid_pwr_flag);


  if (uhf_rfid_pwr_flag == 1){
//prize add by lipengpeng 20210330 start 
      uhf_rfid_pwr(1);// open air chip  20210421 setgpio 1
//prize add by lipengpeng 20210330 end
   }
  else{
//prize add by lipengpeng 20210330 start 
      uhf_rfid_pwr(0);// close air chip  20210421 setgpio 0
//prize add by lipengpeng 20210330 end 
  }
 
  return tCount;
}   

/*----------------------------------------------------------------------------*/
#if 0                   
static DRIVER_ATTR(uhf_3V_funen, 0644, uhf_3V_funen_show,uhf_3V_funen_store);
static DRIVER_ATTR(uhf_5V_boost, 0644, uhf_5V_boost_show,uhf_5V_boost_store);
static DRIVER_ATTR(uhf_tof_pwr, 0644, uhf_rfid_tof_show,uhf_tof_pwr_store);
#else
static DRIVER_ATTR_RW(uhf_3V_funen);
static DRIVER_ATTR_RW(uhf_5V_boost);
static DRIVER_ATTR_RW(uhf_tof_pwr);
#endif

/*----------------------------------------------------------------------------*/
static struct driver_attribute *uhf_attr_list[] = {    
	&driver_attr_uhf_3V_funen,
	&driver_attr_uhf_5V_boost, 
	&driver_attr_uhf_tof_pwr,
};


/*----------------------------------------------------------------------------*/
static int uhf_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)ARRAY_SIZE(uhf_attr_list);

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, uhf_attr_list[idx]);
		if (err) {
			pr_err("driver_create_file (%s) = %d\n",
				   uhf_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int uhf_delete_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)ARRAY_SIZE(uhf_attr_list);

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, uhf_attr_list[idx]);

	return err;
}

static int uhf_probe(struct platform_device *pdev)
{

    const struct of_device_id *id;
	struct device	*dev = &pdev->dev;
    int err =0 ;
  	printk("uhf_probe start\n");
        
	id = of_match_node(uhf_of_match, pdev->dev.of_node);
	if (!id)
		return -ENODEV;
    uhf_gpio_init(dev);

    /* Register sysfs attribute */
	err = uhf_create_attr(&uhf_platform_driver.driver);
	if (err) {
		pr_err("create attribute err = %d\n", err);
		goto exit_sysfs_create_group_failed;
	}
    
    //uhfprt_enable(1);

	printk("uhf_probe done\n");

	return 0;
exit_sysfs_create_group_failed:
    return -1;

}

static int uhf_remove(struct platform_device *dev)
{
    int err = 0;
	err = uhf_delete_attr(&uhf_platform_driver.driver);
	if (err)
		pr_err("uhf_delete_attr fail: %d\n", err);

	return err;
}

static void uhf_shutdown(struct platform_device *dev)
{

}


static int __init uhf_init(void)
{
	int ret;

	pr_debug("Init start\n");    

	ret = platform_driver_register(&uhf_platform_driver);
	if (ret) {
		pr_err("Failed to register platform driver\n");
		return ret;
	}

	pr_debug("Init done\n");

	return 0;
}

static void __exit uhf_exit(void)
{
	pr_debug("Exit start\n");

	platform_driver_unregister(&uhf_platform_driver);

	pr_debug("Exit done\n");
}

module_init(uhf_init);
module_exit(uhf_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yongsheng Wang <Yongsheng Wang@szprize.com>");
MODULE_DESCRIPTION("MTK uhf Core Driver");

