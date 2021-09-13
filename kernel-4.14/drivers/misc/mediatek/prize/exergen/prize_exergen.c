/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */
#include "prize_exergen.h"
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/fb.h>


int exergen_load_status;
static struct work_struct exergen_resume_work;
static struct workqueue_struct *exergen_resume_workqueue;
/* ********************************************** */
/* #endif */


/* function definitions */
static int __init exergen_device_init(void);
static void __exit exergen_device_exit(void);
static int exergen_probe(struct platform_device *pdev);
static int exergen_remove(struct platform_device *pdev);
static struct work_struct exergen_init_work;
static struct workqueue_struct *exergen_init_workqueue;
static int exergen_suspend_flag;
int exergen_register_flag;
/* global variable definitions */
struct exergen_device *exergen;
static struct kobject *exergen_kobj = NULL;
static struct exergen_driver_t *g_exergen_drv;
static struct notifier_block exergen_fb_notifier;
static struct exergen_driver_t exergen_driver_list[EXERGEN_DRV_MAX_COUNT];	/* = {0}; */

static ssize_t device_open_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	g_exergen_drv->open();
	return sprintf(buf, "%d\n", 0);
}
static DEVICE_ATTR(open, 0444, device_open_show, NULL);

static ssize_t temp_value_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	g_exergen_drv->getvalue();
	return sprintf(buf, "%d\n", 0);
}
static DEVICE_ATTR(value, 0444, temp_value_show, NULL);

static ssize_t device_close_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	g_exergen_drv->close();
	return sprintf(buf, "%d\n", 0);
}
static DEVICE_ATTR(close, 0444, device_close_show, NULL);

static ssize_t device_type_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%d\n", 0);
}
static DEVICE_ATTR(type, 0444, device_type_show, NULL);

const struct of_device_id exergen_of_match[] = {
	{ .compatible = "mediatek,exergen", },
	{},
};

struct platform_device exergen_device = {
	.name		= EXERGEN_DEVICE,
	.id			= -1,
};
const struct dev_pm_ops exergen_pm_ops = {
	.suspend = NULL,
	.resume = NULL,
};
static struct platform_driver exergen_driver = {
	.remove = exergen_remove,
	.shutdown = NULL,
	.probe = exergen_probe,
	.driver = {
			.name = EXERGEN_DEVICE,
			.pm = &exergen_pm_ops,
			.owner = THIS_MODULE,
			.of_match_table = exergen_of_match,
	},
};

static void exergen_resume_workqueue_callback(struct work_struct *work)
{
	EXERGEN_DMESG("EXERGEN %s\n", __func__);
	g_exergen_drv->resume(NULL);
	exergen_suspend_flag = 0;
}

static int exergen_fb_notifier_callback(
			struct notifier_block *self,
			unsigned long event, void *data)
{
	struct fb_event *evdata = NULL;
	int blank;
	int err = 0;

	EXERGEN_DMESG("%s\n", __func__);

	evdata = data;
	/* If we aren't interested in this event, skip it immediately ... */
	if (event != FB_EVENT_BLANK)
		return 0;

	blank = *(int *)evdata->data;
	EXERGEN_DMESG("fb_notify(blank=%d)\n", blank);
	switch (blank) {
	case FB_BLANK_UNBLANK:
		EXERGEN_DMESG("LCD ON Notify\n");
		if (g_exergen_drv && exergen_suspend_flag) {
			err = queue_work(exergen_resume_workqueue,&exergen_resume_work);
			if (!err) {
				EXERGEN_DMESG("start resume_workqueue failed\n");
				return err;
			}
		}
		break;
	case FB_BLANK_POWERDOWN:
		EXERGEN_DMESG("LCD OFF Notify\n");
		if (g_exergen_drv && !exergen_suspend_flag) {
			err = cancel_work_sync(&exergen_resume_work);
			if (!err)
				EXERGEN_DMESG("cancel resume_workqueue failed\n");
			g_exergen_drv->suspend(NULL);
		}
		exergen_suspend_flag = 1;
		break;
	default:
		break;
	}
	return 0;
}
/* Add driver: if find TPD_TYPE_CAPACITIVE driver successfully, loading it */
int exergen_driver_add(struct exergen_driver_t *exergen_drv)
{
	int i;

	if (g_exergen_drv != NULL) {
		EXERGEN_DMESG("exergen driver exist\n");
		return -1;
	}
	/* check parameter */
	if (exergen_drv == NULL)
		return -1;

	for (i = 1; i < EXERGEN_DRV_MAX_COUNT; i++) {
		/* add exergen driver into list */
		if (exergen_driver_list[i].exergen_device_name == NULL) {
			exergen_driver_list[i].exergen_device_name = exergen_drv->exergen_device_name;
			exergen_driver_list[i].exergen_local_init = exergen_drv->exergen_local_init;
			exergen_driver_list[i].open = exergen_drv->open;
			exergen_driver_list[i].getvalue = exergen_drv->getvalue;
			exergen_driver_list[i].close = exergen_drv->close;
			exergen_driver_list[i].suspend = exergen_drv->suspend;
			exergen_driver_list[i].resume = exergen_drv->resume;
			break;
		}
		if (strcmp(exergen_driver_list[i].exergen_device_name,exergen_drv->exergen_device_name) == 0)
			return 1;	/* driver exist */
	}

	return 0;
}

int exergen_driver_remove(struct exergen_driver_t *exergen_drv)
{
	int i = 0;
	/* check parameter */
	if (exergen_drv == NULL)
		return -1;
	for (i = 0; i < EXERGEN_DRV_MAX_COUNT; i++) {
		/* find it */
		if (strcmp(exergen_driver_list[i].exergen_device_name,
				exergen_drv->exergen_device_name) == 0) {
			memset(&exergen_driver_list[i], 0,
				sizeof(struct exergen_driver_t));
			break;
		}
	}
	return 0;
}

/* exergen probe */
static int exergen_probe(struct platform_device *pdev)
{
	int exergen_type = 1;
	int i = 0,ret = 0;

	EXERGEN_DMESG("enter %s, %d\n", __func__, __LINE__);
	
	exergen = kmalloc(sizeof(struct exergen_device), GFP_KERNEL);
	if (exergen == NULL)
		return -ENOMEM;
	memset(exergen, 0, sizeof(struct exergen_device));

	exergen->exergen_dev = &pdev->dev;
	for (i = 1; i < EXERGEN_DRV_MAX_COUNT; i++) {
		/* add exergen driver into list */
		if (exergen_driver_list[i].exergen_device_name != NULL) {
			exergen_driver_list[i].exergen_local_init();
			/* msleep(1); */
			if (exergen_load_status == 1) {
				EXERGEN_DMESG("%s, exergen_driver_name=%s\n", __func__,
					  exergen_driver_list[i].exergen_device_name);
				g_exergen_drv = &exergen_driver_list[i];
				break;
			}
		}
	}
	if (g_exergen_drv == NULL) {
		if (exergen_driver_list[0].exergen_device_name != NULL) {
			g_exergen_drv = &exergen_driver_list[0];
			exergen_type = 0;
			g_exergen_drv->exergen_local_init();
			EXERGEN_DMESG("Generic exergen driver\n");
		} else {
			EXERGEN_DMESG("no exergen driver is loaded!!\n");
			return 0;
		}
	}
	exergen_resume_workqueue = create_singlethread_workqueue("exergen_resume");
	INIT_WORK(&exergen_resume_work, exergen_resume_workqueue_callback);

	exergen_fb_notifier.notifier_call = exergen_fb_notifier_callback;
	if (fb_register_client(&exergen_fb_notifier))
		EXERGEN_DMESG("register fb_notifier fail!\n");

	//create sysfs
	ret = sysfs_create_link(exergen_kobj,&pdev->dev.kobj,"exergen");
	if (ret){
		EXERGEN_DMESG("EXERGEN sysfs_create_link fail\n");
	}
	ret = device_create_file(&pdev->dev, &dev_attr_open);
	if (ret){
		EXERGEN_DMESG("EXERGEN failed device_create_file(dev_attr_open)\n");
	}
	ret = device_create_file(&pdev->dev, &dev_attr_value);
	if (ret){
		EXERGEN_DMESG("EXERGEN failed device_create_file(dev_attr_value)\n");
	}
	ret = device_create_file(&pdev->dev, &dev_attr_close);
	if (ret){
		EXERGEN_DMESG("EXERGEN failed device_create_file(dev_attr_close)\n");
	}
	ret = device_create_file(&pdev->dev, &dev_attr_type);
	if (ret){
		EXERGEN_DMESG("EXERGEN failed device_create_file(dev_attr_type)\n");
	}
	
	return 0;
}
static int exergen_remove(struct platform_device *pdev)
{
	return 0;
}

/* called when loaded into kernel */
static void exergen_init_work_callback(struct work_struct *work)
{
	EXERGEN_DMESG("MediaTek exergen driver init\n");
	if (platform_driver_register(&exergen_driver) != 0)
	EXERGEN_DMESG("unable to register exergen driver.\n");
}
static int __init exergen_device_init(void)
{
	int res = 0;
	
	exergen_kobj = kobject_create_and_add("prize", kernel_kobj);
	if (!exergen_kobj){
		EXERGEN_DMESG(" kernel kobject_create_and_add error \r\n"); 
		return -1;
	}
	exergen_init_workqueue = create_singlethread_workqueue("mtk-exergen");
	INIT_WORK(&exergen_init_work, exergen_init_work_callback);

	res = queue_work(exergen_init_workqueue, &exergen_init_work);
	if (!res)
		pr_info("exergen : exergen device init failed res:%d\n", res);
	return 0;
}
/* should never be called */
static void __exit exergen_device_exit(void)
{
	EXERGEN_DMESG("MediaTek exergen driver exit\n");
	platform_driver_unregister(&exergen_driver);
	if (!exergen_kobj){
		return;
	}
	if (exergen_kobj){
		kobject_put(exergen_kobj);
	}
}

late_initcall(exergen_device_init);
module_exit(exergen_device_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek exergen driver");
MODULE_AUTHOR("Liao Jie<liaojie@szprize.com>");
