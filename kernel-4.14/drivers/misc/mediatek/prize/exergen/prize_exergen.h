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

#ifndef __TPD_H
#define __TPD_H
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/types.h>
#include <linux/seq_file.h>
#include <linux/list.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <generated/autoconf.h>
#include <linux/kobject.h>
#include <linux/regulator/consumer.h>

/*debug macros */
#define EXERGEN_DEVICE "mtk-exergen"
#define EXERGEN_DRV_MAX_COUNT 10
#define EXERGEN_DEBUG
/* #define EXERGEN_DEBUG_TRACK */
#define EXERGEN_DMESG(a, arg...) \
	pr_err(EXERGEN_DEVICE ":[%s:%d] " a, __func__, __LINE__, ##arg)
#if defined(EXERGEN_DEBUG)
#undef EXERGEN_DEBUG
#define EXERGEN_DMESG(a, arg...) \
	pr_err(EXERGEN_DEVICE ":[%s:%d] " a, __func__, __LINE__, ##arg)
#else
#define EXERGEN_DMESG(arg...)
#endif

struct exergen_device {
	struct device *exergen_dev;
	struct regulator *reg;
	struct regulator *io_reg;
};
struct exergen_driver_t {
	char *exergen_device_name;
	int (*exergen_local_init)(void);
	void (*open)(void);
	void (*getvalue)(void);
	void (*close)(void);
	void (*suspend)(struct device *h);
	void (*resume)(struct device *h);
};

extern int exergen_load_status;
extern int exergen_driver_add(struct exergen_driver_t *tpd_drv);
extern int exergen_driver_remove(struct exergen_driver_t *tpd_drv);
#endif
