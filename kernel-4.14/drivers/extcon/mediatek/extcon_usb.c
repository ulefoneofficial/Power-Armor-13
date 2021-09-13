/*
 * Copyright (C) 2017 MediaTek Inc.
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

#include <linux/extcon.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "extcon_usb.h"

struct usb_extcon_info {
	struct device *dev;
	struct extcon_dev *edev;
	unsigned int dr; /* data role */
	struct workqueue_struct *extcon_workq;
};

struct mt_usb_work {
	struct delayed_work dwork;
	unsigned int dr; /* data role */
};

static const unsigned int usb_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_NONE,
};

enum {
	DUAL_PROP_MODE_UFP = 0,
	DUAL_PROP_MODE_DFP,
	DUAL_PROP_MODE_NONE,
};

enum {
	DUAL_PROP_PR_SRC = 0,
	DUAL_PROP_PR_SNK,
	DUAL_PROP_PR_NONE,
};

enum {
	DUAL_PROP_DR_HOST = 0,
	DUAL_PROP_DR_DEVICE,
	DUAL_PROP_DR_NONE,
};

static struct usb_extcon_info *g_extcon_info;
static void usb_extcon_detect_cable(struct work_struct *work)
{
	struct mt_usb_work *info = container_of(to_delayed_work(work),
						    struct mt_usb_work,
						    dwork);
	unsigned int cur_dr, new_dr;

	if (!g_extcon_info) {
		pr_info("g_extcon_info = NULL\n");
		return;
	}
	cur_dr = g_extcon_info->dr;
	new_dr = info->dr;
	pr_info("cur_dr(%d) new_dr(%d)\n", cur_dr, new_dr);

	/* none -> device */
	if (cur_dr == DUAL_PROP_DR_NONE &&
			new_dr == DUAL_PROP_DR_DEVICE) {
		extcon_set_state_sync(g_extcon_info->edev,
			EXTCON_USB, true);
	/* none -> host */
	} else if (cur_dr == DUAL_PROP_DR_NONE &&
			new_dr == DUAL_PROP_DR_HOST) {
		extcon_set_state_sync(g_extcon_info->edev,
			EXTCON_USB_HOST, true);
	/* device -> none */
	} else if (cur_dr == DUAL_PROP_DR_DEVICE &&
			new_dr == DUAL_PROP_DR_NONE) {
		extcon_set_state_sync(g_extcon_info->edev,
			EXTCON_USB, false);
	/* host -> none */
	} else if (cur_dr == DUAL_PROP_DR_HOST &&
			new_dr == DUAL_PROP_DR_NONE) {
		extcon_set_state_sync(g_extcon_info->edev,
			EXTCON_USB_HOST, false);
	/* device -> host */
	} else if (cur_dr == DUAL_PROP_DR_DEVICE &&
			new_dr == DUAL_PROP_DR_HOST) {
		pr_info("device -> host, it's illegal\n");
	/* host -> device */
	} else if (cur_dr == DUAL_PROP_DR_HOST &&
			new_dr == DUAL_PROP_DR_DEVICE) {
		pr_info("host -> device, it's illegal\n");
	}

	g_extcon_info->dr = new_dr;
	kfree(info);
}

static void issue_connection_work(unsigned int dr)
{
	struct mt_usb_work *work;

	if (!g_extcon_info) {
		pr_info("g_extcon_info = NULL\n");
		return;
	}
	/* create and prepare worker */
	work = kzalloc(sizeof(struct mt_usb_work), GFP_ATOMIC);
	if (!work)
		return;

	work->dr = dr;
	INIT_DELAYED_WORK(&work->dwork, usb_extcon_detect_cable);
	/* issue connection work */
	queue_delayed_work(g_extcon_info->extcon_workq, &work->dwork, 0);
}

#if !defined(CONFIG_USB_MU3D_DRV)
void mt_usb_connect(void)
{
#ifndef CONFIG_TCPC_CLASS
#ifdef CONFIG_DUAL_ROLE_USB_INTF
	mt_usb_dual_role_to_device();
#endif
#endif

	pr_info("%s\n", __func__);
	issue_connection_work(DUAL_PROP_DR_DEVICE);
}
EXPORT_SYMBOL_GPL(mt_usb_connect);

void mt_usb_disconnect(void)
{
#ifndef CONFIG_TCPC_CLASS
#ifdef CONFIG_DUAL_ROLE_USB_INTF
	mt_usb_dual_role_to_none();
#endif
#endif

	pr_info("%s\n", __func__);
	issue_connection_work(DUAL_PROP_DR_NONE);
}
EXPORT_SYMBOL_GPL(mt_usb_disconnect);
#endif

void mt_usbhost_connect(void)
{
#ifndef CONFIG_TCPC_CLASS
#ifdef CONFIG_DUAL_ROLE_USB_INTF
	mt_usb_dual_role_to_host();
#endif
#endif

	pr_info("%s\n", __func__);
	issue_connection_work(DUAL_PROP_DR_HOST);
}
EXPORT_SYMBOL_GPL(mt_usbhost_connect);

void mt_usbhost_disconnect(void)
{
#ifndef CONFIG_TCPC_CLASS
#ifdef CONFIG_DUAL_ROLE_USB_INTF
	mt_usb_dual_role_to_none();
#endif
#endif

	pr_info("%s\n", __func__);
	issue_connection_work(DUAL_PROP_DR_NONE);
}
EXPORT_SYMBOL_GPL(mt_usbhost_disconnect);

//prize added by huarui, MT5725, 20210205-start
#if defined(CONFIG_PRIZE_MT5725_SUPPORT_15W)
extern int set_otg_gpio(int en);
extern int turn_off_5725(int en);
#endif
//prize added by huarui, MT5725, 20210205-end
void mt_vbus_on(void)
{
//prize added by huarui, MT5725, 20210205-start
#if defined(CONFIG_PRIZE_MT5725_SUPPORT_15W)
#ifdef CONFIG_PRIZE_REVERE_CHARGING_MODE
	turn_off_5725(0); //into otg set OD5 low. add by lipengpeng 20210409 start  //GPIO88 OD5  //OD5-->  1-->high  0---> low
#else
	turn_off_5725(1);  //GPIO88 OD5
#endif 
	set_otg_gpio(1);  //OD7  87
#endif
//prize added by huarui, MT5725, 20210205-end
	usb_otg_set_vbus(true);
}
EXPORT_SYMBOL_GPL(mt_vbus_on);

void mt_vbus_off(void)
{
//prize added by huarui, MT5725, 20210205-start
#if defined(CONFIG_PRIZE_MT5725_SUPPORT_15W)
	set_otg_gpio(0);//OD7
	turn_off_5725(0);//OD5   0--->low  1--->high
#endif
//prize added by huarui, MT5725, 20210205-end
	usb_otg_set_vbus(false);
}
EXPORT_SYMBOL_GPL(mt_vbus_off);

#if defined (CONFIG_PRIZE_REVERE_CHARGING_MODE)

//prize add by lipengpeng 20210308 start
void mt_vbus_revere_on(void)
{
	usb_otg_set_vbus(true);
}
EXPORT_SYMBOL_GPL(mt_vbus_revere_on);

void mt_vbus_revere_off(void)
{
	usb_otg_set_vbus(false);
}
EXPORT_SYMBOL_GPL(mt_vbus_revere_off); 
//prize add by lipengpeng 20210308 end 
#endif

//prize add by lipengpeng 20210309 start
#if defined (CONFIG_PRIZE_REVERE_CHARGING_MODE)
void mt_vbus_reverse_on(void)
{
	usb_otg_set_reverse_vbus(true);
}
EXPORT_SYMBOL_GPL(mt_vbus_reverse_on);

void mt_vbus_reverse_on_limited_current(void)
{
	usb_otg_set_reverse_vbus_limited_current(true);
}
EXPORT_SYMBOL_GPL(mt_vbus_reverse_on_limited_current);

void mt_vbus_reverse_off_limited_current(void)
{
	usb_otg_set_reverse_vbus_limited_current(false);
}
EXPORT_SYMBOL_GPL(mt_vbus_reverse_off_limited_current);

#endif
//prize add by lipengpeng 20210309 end

static int usb_extcon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct usb_extcon_info *info;
	int ret;

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = dev;

	info->edev = devm_extcon_dev_allocate(dev, usb_extcon_cable);
	if (IS_ERR(info->edev)) {
		dev_info(dev, "failed to allocate extcon device\n");
		return -ENOMEM;
	}

	ret = devm_extcon_dev_register(dev, info->edev);
	if (ret < 0) {
		dev_info(dev, "failed to register extcon device\n");
		return ret;
	}
	platform_set_drvdata(pdev, info);
	info->dr = DUAL_PROP_DR_NONE;
	info->extcon_workq = create_singlethread_workqueue("usb_extcon_workq");
	g_extcon_info = info;

#ifndef CONFIG_TCPC_CLASS
#ifdef CONFIG_DUAL_ROLE_USB_INTF
	mt_usb_dual_role_init(g_extcon_info->dev);
#endif
#endif

	/* Perform initial detection */
	/* issue_connection_work(DUAL_PROP_DR_NONE); */
	return 0;
}

static int usb_extcon_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id usb_extcon_dt_match[] = {
	{ .compatible = "mediatek,extcon-usb", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, usb_extcon_dt_match);

static struct platform_driver usb_extcon_driver = {
	.probe		= usb_extcon_probe,
	.remove		= usb_extcon_remove,
	.driver		= {
		.name	= "mediatek,extcon-usb",
		.of_match_table = usb_extcon_dt_match,
	},
};

module_platform_driver(usb_extcon_driver);

