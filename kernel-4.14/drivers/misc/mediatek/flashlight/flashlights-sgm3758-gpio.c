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

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/pinctrl/consumer.h>

#include "flashlight-core.h"
#include "flashlight-dt.h"

/* define device tree */
/* TODO: modify temp device tree name */
#ifndef SGM3758_DTNAME
#define SGM3758_DTNAME "mediatek,flashlights_sgm3758_gpio"
#endif

/* TODO: define driver name */
#define SGM3758_NAME "flashlights-sgm3758-gpio"

/* define registers */
/* TODO: define register */

/* define mutex and work queue */
static DEFINE_MUTEX(sgm3758_mutex);
static struct work_struct sgm3758_work;

typedef enum {
	HWEN_PIN = 0,
	FLASH_PIN,
}DEV_PIN;
/* define pinctrl */
/* TODO: define pinctrl */
#define SGM3758_PINCTRL_PINSTATE_LOW 0
#define SGM3758_PINCTRL_PINSTATE_HIGH 1
static struct pinctrl *sgm3758_pinctrl;
static struct pinctrl_state *sgm3758_hwen_high;
static struct pinctrl_state *sgm3758_hwen_low;

#define SGM3758_LEVEL_NUM (1+1)
#define SGM3758_HW_TIMEOUT 350

/* define usage count */
static int use_count;

static int sgm3758_level = -1;

/* platform data */
struct sgm3758_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};

/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int sgm3758_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;

	/* get pinctrl */
	sgm3758_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(sgm3758_pinctrl)) {
		pr_err("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(sgm3758_pinctrl);
	}

	/* TODO: Flashlight XXX pin initialization */
	sgm3758_hwen_high = pinctrl_lookup_state(sgm3758_pinctrl, "hwen_high");
	if (IS_ERR(sgm3758_hwen_high)) {
		pr_err("Failed to init (hwen_high)\n");
		ret = PTR_ERR(sgm3758_hwen_high);
	}
	sgm3758_hwen_low = pinctrl_lookup_state(sgm3758_pinctrl, "hwen_low");
	if (IS_ERR(sgm3758_hwen_low)) {
		pr_err("Failed to init (hwen_low)\n");
		ret = PTR_ERR(sgm3758_hwen_low);
	}

	return ret;
}

static int sgm3758_pinctrl_set(DEV_PIN pin, int state)
{
	int ret = 0;

	if (IS_ERR(sgm3758_pinctrl)) {
		pr_err("pinctrl is not available\n");
		return -1;
	}

	switch (pin) {
	case HWEN_PIN:
		if (state == SGM3758_PINCTRL_PINSTATE_LOW && !IS_ERR(sgm3758_hwen_low))
			pinctrl_select_state(sgm3758_pinctrl, sgm3758_hwen_low);
		else if (state == SGM3758_PINCTRL_PINSTATE_HIGH && !IS_ERR(sgm3758_hwen_high))
			pinctrl_select_state(sgm3758_pinctrl, sgm3758_hwen_high);
		else
			pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	case FLASH_PIN:
		pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	default:
		pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	}
	pr_debug("pin(%d) state(%d)\n", pin, state);

	return ret;
}


/******************************************************************************
 * sgm3758 operations
 *****************************************************************************/
/* flashlight enable function */
static int sgm3758_enable(void)
{
	
	//if (sgm3758_level > 1){
		sgm3758_pinctrl_set(HWEN_PIN, 1);
	//}

	return 0;
}

/* flashlight disable function */
static int sgm3758_disable(void)
{
	return sgm3758_pinctrl_set(HWEN_PIN, 0);;
}

/* set flashlight level */
static int sgm3758_set_level(int level)
{
	//int state = 0;

	/* TODO: wrap set level function */
	sgm3758_level = level;
	printk("%s flashlight duty = %d\n",__func__,level);

	return 0;
}

/* flashlight init */
static int sgm3758_init(void)
{
	return sgm3758_pinctrl_set(HWEN_PIN, 0);
}

/* flashlight uninit */
static int sgm3758_uninit(void)
{
	return sgm3758_pinctrl_set(HWEN_PIN, 0);
}

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer sgm3758_timer;
static unsigned int sgm3758_timeout_ms;

static void sgm3758_work_disable(struct work_struct *data)
{
	pr_debug("work queue callback\n");
	sgm3758_disable();
}

static enum hrtimer_restart sgm3758_timer_func(struct hrtimer *timer)
{
	schedule_work(&sgm3758_work);
	return HRTIMER_NORESTART;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int sgm3758_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	ktime_t ktime;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		pr_debug("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		sgm3758_timeout_ms = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_debug("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		sgm3758_set_level(fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		pr_debug("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (sgm3758_timeout_ms) {
				ktime = ktime_set(sgm3758_timeout_ms / 1000,
						(sgm3758_timeout_ms % 1000) * 1000000);
				hrtimer_start(&sgm3758_timer, ktime, HRTIMER_MODE_REL);
			}
			sgm3758_enable();
		} else {
			sgm3758_disable();
			hrtimer_cancel(&sgm3758_timer);
		}
		break;
	case FLASH_IOC_GET_DUTY_NUMBER:
		pr_debug("FLASH_IOC_GET_DUTY_NUMBER(%d)\n", channel);
		fl_arg->arg = SGM3758_LEVEL_NUM;
		break;
	case FLASH_IOC_GET_MAX_TORCH_DUTY:
		pr_debug("FLASH_IOC_GET_MAX_TORCH_DUTY(%d)\n", channel);
		fl_arg->arg = SGM3758_LEVEL_NUM - 1;
		break;
	case FLASH_IOC_GET_HW_TIMEOUT:
		pr_debug("FLASH_IOC_GET_HW_TIMEOUT(%d)\n", channel);
		fl_arg->arg = SGM3758_HW_TIMEOUT;
		break;
	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int sgm3758_open(void)
{
	/* Actual behavior move to set driver function since power saving issue */
	return 0;
}

static int sgm3758_release(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int sgm3758_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&sgm3758_mutex);
	if (set) {
		if (!use_count)
			ret = sgm3758_init();
		use_count++;
		pr_debug("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = sgm3758_uninit();
		if (use_count < 0)
			use_count = 0;
		pr_debug("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&sgm3758_mutex);

	return ret;
}

static ssize_t sgm3758_strobe_store(struct flashlight_arg arg)
{
	sgm3758_set_driver(1);
	sgm3758_set_level(arg.level);
	sgm3758_timeout_ms = 0;
	sgm3758_enable();
	msleep(arg.dur);
	sgm3758_disable();
	sgm3758_set_driver(0);

	return 0;
}

static struct flashlight_operations sgm3758_ops = {
	sgm3758_open,
	sgm3758_release,
	sgm3758_ioctl,
	sgm3758_strobe_store,
	sgm3758_set_driver
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int sgm3758_chip_init(void)
{
	/* NOTE: Chip initialication move to "set driver" operation for power saving issue.
	 * sgm3758_init();
	 */

	return 0;
}

static int sgm3758_parse_dt(struct device *dev,
		struct sgm3758_platform_data *pdata)
{
	struct device_node *np, *cnp;
	u32 decouple = 0;
	int i = 0;

	if (!dev || !dev->of_node || !pdata)
		return -ENODEV;

	np = dev->of_node;

	pdata->channel_num = of_get_child_count(np);
	if (!pdata->channel_num) {
		pr_info("Parse no dt, node.\n");
		return 0;
	}
	pr_info("Channel number(%d).\n", pdata->channel_num);

	if (of_property_read_u32(np, "decouple", &decouple))
		pr_info("Parse no dt, decouple.\n");

	pdata->dev_id = devm_kzalloc(dev,
			pdata->channel_num * sizeof(struct flashlight_device_id),
			GFP_KERNEL);
	if (!pdata->dev_id)
		return -ENOMEM;

	for_each_child_of_node(np, cnp) {
		if (of_property_read_u32(cnp, "type", &pdata->dev_id[i].type))
			goto err_node_put;
		if (of_property_read_u32(cnp, "ct", &pdata->dev_id[i].ct))
			goto err_node_put;
		if (of_property_read_u32(cnp, "part", &pdata->dev_id[i].part))
			goto err_node_put;
		snprintf(pdata->dev_id[i].name, FLASHLIGHT_NAME_SIZE, SGM3758_NAME);
		pdata->dev_id[i].channel = i;
		pdata->dev_id[i].decouple = decouple;

		pr_info("Parse dt (type,ct,part,name,channel,decouple)=(%d,%d,%d,%s,%d,%d).\n",
				pdata->dev_id[i].type, pdata->dev_id[i].ct,
				pdata->dev_id[i].part, pdata->dev_id[i].name,
				pdata->dev_id[i].channel, pdata->dev_id[i].decouple);
		i++;
	}	

	return 0;

err_node_put:
	of_node_put(cnp);
	return -EINVAL;
}

static int sgm3758_probe(struct platform_device *pdev)
{
	struct sgm3758_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int err;
	int i;

	pr_debug("Probe start.\n");

	/* init pinctrl */
	if (sgm3758_pinctrl_init(pdev)) {
		pr_debug("Failed to init pinctrl.\n");
		err = -EFAULT;
		goto err;
	}

	/* init platform data */
	if (!pdata) {
		pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err;
		}
		pdev->dev.platform_data = pdata;
		err = sgm3758_parse_dt(&pdev->dev, pdata);
		if (err)
		goto err;
	}

	/* init work queue */
	INIT_WORK(&sgm3758_work, sgm3758_work_disable);

	/* init timer */
	hrtimer_init(&sgm3758_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	sgm3758_timer.function = sgm3758_timer_func;
	sgm3758_timeout_ms = 100;

	/* init chip hw */
	sgm3758_chip_init();

	/* clear usage count */
	use_count = 0;

	/* register flashlight device */
	if (pdata->channel_num) {
		for (i = 0; i < pdata->channel_num; i++)
			if (flashlight_dev_register_by_device_id(&pdata->dev_id[i], &sgm3758_ops)) {
				err = -EFAULT;
				goto err;
			}
	} else {
		if (flashlight_dev_register(SGM3758_NAME, &sgm3758_ops)) {
			err = -EFAULT;
			goto err;
		}
	}

	pr_debug("Probe done.\n");

	return 0;
err:
	return err;
}

static int sgm3758_remove(struct platform_device *pdev)
{
	struct sgm3758_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int i;

	pr_debug("Remove start.\n");

	pdev->dev.platform_data = NULL;

	/* unregister flashlight device */
	if (pdata && pdata->channel_num)
		for (i = 0; i < pdata->channel_num; i++)
			flashlight_dev_unregister_by_device_id(&pdata->dev_id[i]);
	else
		flashlight_dev_unregister(SGM3758_NAME);

	/* flush work queue */
	flush_work(&sgm3758_work);

	pr_debug("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id sgm3758_gpio_of_match[] = {
	{.compatible = SGM3758_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, sgm3758_gpio_of_match);
#else
static struct platform_device sgm3758_gpio_platform_device[] = {
	{
		.name = SGM3758_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, sgm3758_gpio_platform_device);
#endif

static struct platform_driver sgm3758_platform_driver = {
	.probe = sgm3758_probe,
	.remove = sgm3758_remove,
	.driver = {
		.name = SGM3758_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = sgm3758_gpio_of_match,
#endif
	},
};

static int __init flashlight_sgm3758_init(void)
{
	int ret;

	pr_debug("Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&sgm3758_gpio_platform_device);
	if (ret) {
		pr_err("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&sgm3758_platform_driver);
	if (ret) {
		pr_err("Failed to register platform driver\n");
		return ret;
	}

	pr_debug("Init done.\n");

	return 0;
}

static void __exit flashlight_sgm3758_exit(void)
{
	pr_debug("Exit start.\n");

	platform_driver_unregister(&sgm3758_platform_driver);

	pr_debug("Exit done.\n");
}

module_init(flashlight_sgm3758_init);
module_exit(flashlight_sgm3758_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Wang <Simon-TCH.Wang@mediatek.com>");
MODULE_DESCRIPTION("MTK Flashlight SGM3758 GPIO Driver");

