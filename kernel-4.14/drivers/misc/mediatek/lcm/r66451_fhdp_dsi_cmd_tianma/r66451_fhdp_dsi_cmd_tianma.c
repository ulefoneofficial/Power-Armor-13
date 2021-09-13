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

#define LOG_TAG "LCM"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"

#ifdef MTK_ROUND_CORNER_SUPPORT
#include "data_hw_roundedpattern.h"
#endif

#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#include <debug.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
//#else
//#include <mach/mt_pm_ldo.h>
//#include <mach/mt_gpio.h>
#endif

#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(CRITICAL, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(INFO, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_notice("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

static struct LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))
#define MDELAY(n)		(lcm_util.mdelay(n))
#define UDELAY(n)		(lcm_util.udelay(n))

#define _HIGH_FRM_
#ifdef __FIX_120__
#define DEFAULT_FPS     12000
#else
#define DEFAULT_FPS		6000
#endif

/* ------------------------------------------------------------------------ */
/* Local Functions */
/* ------------------------------------------------------------------------ */

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
		lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
		lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
		lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#define dfps_dsi_send_cmd(cmdq, cmd, count, para_list, force_update, sendmode) \
		lcm_util.dsi_dynfps_send_cmd(cmdq, cmd, count, para_list, force_update, sendmode)

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
/* #include <linux/jiffies.h> */
/* #include <linux/delay.h> */
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#endif

/* ------------------------------------------------------------------------ */
/* Local Constants */
/* ------------------------------------------------------------------------ */
#define LCM_DSI_CMD_MODE		1
#define FRAME_WIDTH			(1080)
#define FRAME_HEIGHT			(2340)

#define REGFLAG_DELAY			0xFFFC
#define REGFLAG_UDELAY			0xFFFB
#define REGFLAG_END_OF_TABLE		0xFFFD
#define REGFLAG_RESET_LOW		0xFFFE
#define REGFLAG_RESET_HIGH		0xFFFF

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[200];
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28, 0, {} },
	{REGFLAG_DELAY, 20, {} },
	{0x10, 0, {} },
	{REGFLAG_DELAY, 120, {} }
};

static struct LCM_setting_table init_setting[] = {

	{REGFLAG_DELAY, 200, {} },

#ifdef _HIGH_FRM_
	/* set PLL to 380M */
	{0xB0, 1, {0x00} },
	{0xB6, 12, {0x6A, 0x00, 0x06, 0x23, 0x8A, 0x13, 0x1A, 0x05,
		      0x04, 0xFA, 0x05, 0x20} },
#else
	/* set PLL to 190M */
	{0xB0, 1, {0x00} },
	{0xB6, 12, {0x51, 0x00, 0x06, 0x23, 0x8A, 0x13, 0x1A, 0x05,
		      0x04, 0xFA, 0x05, 0x20} },
#endif

	{0x51, 2, {0x0F, 0xff} },
	{0x53, 1, {0x04} },
	{0x35, 1, {0x00} },
	{0x53, 1, {0x04} },
	{0x53, 1, {0x04} },
	{0x44, 2, {0x08, 0x66} }, /* set TE event @ line 0x866(2150) */
	{0x53, 1, {0x04} },
	{0x53, 1, {0x04} },
	{0x2a, 4, {0x00, 0x00, 0x04, 0x37} },
	{0x2b, 4, {0x00, 0x00, 0x09, 0x23} },

	{REGFLAG_DELAY, 200, {} },

	{0xB0, 1, {0x80} },
	{0xD4, 1, {0x93} },
	{0x50, 41, {0x42, 0x58, 0x81, 0x2D, 0x00, 0x00, 0x00,
		    0x00, 0x00, 0x00, 0x6B, 0x00, 0x00, 0x00, 0x00,
		    0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0xFF, 0xD4,
		    0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x00,
		    0x53, 0x18, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00,
		    0x00, 0x00} },

	/* set PPS to table and choose table 0 */
	{0xF7, 1, {0x01} }, /* key */
	{0xF8, 128, {0x11, 0x00, 0x00, 0x89, 0x30, 0x80, 0x09, 0x24, 0x04, 0x38,
		0x00, 0x14, 0x02, 0x1c, 0x02, 0x1c, 0x02, 0x00, 0x02, 0x0e,
		0x00, 0x20, 0x01, 0xe8, 0x00, 0x07, 0x00, 0x0c, 0x05, 0x0e,
		0x05, 0x16, 0x18, 0x00, 0x10, 0xf0, 0x03, 0x0c, 0x20, 0x00,
		0x06, 0x0b, 0x0b, 0x33, 0x0e, 0x1c, 0x2a, 0x38, 0x46, 0x54,
		0x62, 0x69, 0x70, 0x77, 0x79, 0x7b, 0x7d, 0x7e, 0x01, 0x02,
		0x01, 0x00, 0x09, 0x40, 0x09, 0xbe, 0x19, 0xfc, 0x19, 0xfa,
		0x19, 0xf8, 0x1a, 0x38, 0x1a, 0x78, 0x1a, 0xb6, 0x2a, 0xf6,
		0x2b, 0x34, 0x2b, 0x74, 0x3b, 0x74, 0x6b, 0xf4, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} },

	/* Turn on DSC */
	{0xB0, 1, {0x00} },
	{0xEB, 2, {0x8B, 0x8B} },

	/* Flash QE setting */
	{0xDF, 2, {0x50, 0x40} },
	{0xF3, 5, {0x50, 0x00, 0x00, 0x00, 0x00} },
	{0xF2, 1, {0x11} },

	{REGFLAG_DELAY, 10, {} },

	{0xF3, 5, {0x01, 0x00, 0x00, 0x00, 0x01} },
	{0xF4, 2, {0x00, 0x02} },
	{0xF2, 1, {0x19} },

	{REGFLAG_DELAY, 20, {} },

	{0xDF, 2, {0x50, 0x42} },
	{0xB0, 1, {0x84} },
	{0xE6, 1, {0x01} },

	{0x11, 0, {} },

	{REGFLAG_DELAY, 120, {} },
	{0x29, 0, {} },
};

#ifdef __FIX_120__
#if (DEFAULT_FPS == 12000)
static struct LCM_setting_table switch_120_setting[] = {
	{0xB0, 1, {0x04} },
	{0xC1, 43, {0x94, 0x42, 0x00,
		0x16, 0x05, 0x00, 0x00, 0x00, 0x10, 0x00, 0x10,
		0x00, 0xAA, 0x8A, 0x02, 0x10, 0x00, 0x10, 0x00,
		0x00, 0x3F, 0x3F, 0x03, 0xFF, 0x03, 0xFF, 0x23,
		0xFF, 0x03, 0xFF, 0x23, 0xFF, 0x03, 0xFF, 0x00,
		0x40, 0x40, 0x00, 0x00, 0x10, 0x01, 0x00, 0x0C} },
	{0xC2, 6, {0x09, 0x24, 0x0E,
		0x00, 0x00, 0x0E} },
	{0xC4, 18, {0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x02, 0x00, 0x00, 0x00, 0x35, 0x00, 0x01} },
	{0xCF, 133, {0x64, 0x0B, 0x00,
		0x28, 0x02, 0x4B, 0x02, 0xDE, 0x0B, 0x77, 0x0B,
		0x8B, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
		0x04, 0x04, 0x05, 0x05, 0x05, 0x00, 0x3D, 0x00,
		0xCD, 0x00, 0xCD, 0x00, 0xCD, 0x00, 0xCD, 0x01,
		0x00, 0x01, 0x00, 0x03, 0x98, 0x03, 0xA9, 0x03,
		0xA9, 0x03, 0xA9, 0x03, 0xA9, 0x00, 0x3D, 0x00,
		0xCD, 0x00, 0xCD, 0x00, 0xCD, 0x00, 0xCD, 0x01,
		0x00, 0x01, 0x00, 0x03, 0x98, 0x03, 0xA9, 0x03,
		0xA9, 0x03, 0xA9, 0x03, 0xA9, 0x01, 0x42, 0x01,
		0x42, 0x01, 0x42, 0x01, 0x42, 0x01, 0x42, 0x01,
		0x42, 0x01, 0x42, 0x01, 0x42, 0x01, 0x42, 0x01,
		0x42, 0x01, 0x42, 0x01, 0x42, 0x1C, 0x1C, 0x1C,
		0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C,
		0x1C, 0x00, 0x88, 0x00, 0xB1, 0x00, 0xB1, 0x09,
		0xA6, 0x09, 0xA6, 0x09, 0xA4, 0x09, 0xA4, 0x09,
		0xA4, 0x09, 0xA4, 0x09, 0xA4, 0x09, 0xA4, 0x0F,
		0xC3, 0x19} },
	{0xD7, 9, {0x00, 0x69, 0x34,
		0x00, 0xA0, 0x0A, 0x00, 0x00, 0x39} },
	{0xD8, 59, {0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x39, 0x00,
		0x3A, 0x00, 0x3A, 0x00, 0x3A, 0x00, 0x3A, 0x05,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x20, 0x40, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x0F, 0x00, 0x32, 0x00, 0x00, 0x00, 0x17,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} },
	{0xBB, 49, {0x59, 0xC8, 0xC8,
		0xC8, 0xC8, 0xC8, 0xC8, 0xC8, 0xC8, 0xC8, 0x4A,
		0x48, 0x46, 0x44, 0x42, 0x40, 0x3E, 0x3C, 0x3A,
		0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0x04, 0x00, 0x02, 0x02, 0x00, 0x04,
		0x69, 0x5A, 0x00, 0x0B, 0x76, 0x0F, 0xFF, 0x0F,
		0xFF, 0x0F, 0xFF, 0x14, 0x81, 0xF4} },
	{0xE8, 2, {0x00, 0x02} },
	{0xE4, 2, {0x00, 0x07} },
	{0xB0, 1, {0x84} },
	{0xE4, 9, {0x33, 0xB4, 0x00,
		0x00, 0x00, 0x39, 0x04, 0x09, 0x34} },
	{0xE6, 1, {0x01} },
};
#else
static struct LCM_setting_table switch_90_setting[] = {
	{0xB0, 1, {0x04} },
	{0xF1, 1, {0x2A} },
	{0xC1, 1, {0x0C} },
	{0xC2, 6, {0x09, 0x24, 0x0E, 0x00, 0x00, 0x0E} },
	{0xC1, 5, {0x94, 0x42, 0x00, 0x16, 0x05} },
	{0xCF, 133, {0x64, 0x0B, 0x00,
			0x28, 0x02, 0x4B, 0x02, 0xDE, 0x0B, 0x77, 0x0B,
			0x8B, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,
			0x02, 0x02, 0x03, 0x03, 0x03, 0x00, 0x3D, 0x00,
			0xCD, 0x00, 0xCD, 0x00, 0xCD, 0x00, 0xCD, 0x01,
			0x00, 0x01, 0x00, 0x03, 0x98, 0x03, 0x98, 0x03,
			0x98, 0x03, 0x98, 0x03, 0x98, 0x00, 0x3D, 0x00,
			0xCD, 0x00, 0xCD, 0x00, 0xCD, 0x00, 0xCD, 0x01,
			0x00, 0x01, 0x00, 0x03, 0x98, 0x03, 0x98, 0x03,
			0x98, 0x03, 0x98, 0x03, 0x98, 0x01, 0x42, 0x01,
			0x42, 0x01, 0x42, 0x01, 0x42, 0x01, 0x42, 0x01,
			0x42, 0x01, 0x42, 0x01, 0x42, 0x01, 0x42, 0x01,
			0x42, 0x01, 0x42, 0x01, 0x42, 0x1C, 0x1C, 0x1C,
			0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C,
			0x1C, 0x00, 0x88, 0x00, 0xB1, 0x00, 0xB1, 0x09,
			0xA6, 0x09, 0xA6, 0x09, 0xA4, 0x09, 0xA4, 0x09,
			0xA4, 0x09, 0xA4, 0x09, 0xA4, 0x09, 0xA4, 0x0F,
			0xC3, 0x19} },
	{0xC4,  18, {0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x02, 0x00, 0x00, 0x00, 0x48, 0x00, 0x01} },
	{0xD7, 9, {0x00, 0x69, 0x34,
			0x00, 0xA0, 0x0A, 0x00, 0x00, 0x4E} },
	{0xD8, 59, {0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4D, 0x00,
			0x4D, 0x00, 0x4D, 0x00, 0x4D, 0x00, 0x4D, 0x05,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x4E, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x16, 0x00, 0x44, 0x00, 0x00, 0x00, 0x1F,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} },
	{0xBB, 49, {0x59, 0xC8, 0xC8,
			0xC8, 0xC8, 0xC8, 0xC8, 0xC8, 0xC8, 0xC8, 0x4A,
			0x48, 0x46, 0x44, 0x42, 0x40, 0x3E, 0x3C, 0x3A,
			0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
			0xFF, 0xFF, 0x04, 0x00, 0x02, 0x02, 0x00, 0x04,
			0x69, 0x5A, 0x00, 0x0B, 0x76, 0x0F, 0xFF, 0x0F,
			0xFF, 0x0F, 0xFF, 0x14, 0x81, 0xF4} },
	{0xE8, 2, {0x00, 0x02} },
	{0xE4, 2, {0x00, 0x0A} },
	{0xB0, 1, {0x84} },
	{0xE4, 9, {0x33, 0xB4, 0x00,
			0x00, 0x00, 0x4E, 0x04, 0x04, 0x9A} },
	{0xE6, 1, {0x01} },
};
#endif
#endif

static struct LCM_setting_table bl_level[] = {
	{0x51, 2, {0xF, 0xFF} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

#ifdef CONFIG_MTK_HIGH_FRAME_RATE

#define DFPS_MAX_CMD_NUM 100

struct LCM_dfps_cmd_table {
	bool need_send_cmd;
	enum LCM_Send_Cmd_Mode sendmode;
	struct LCM_setting_table prev_f_cmd[DFPS_MAX_CMD_NUM];
};

static struct LCM_dfps_cmd_table
	dfps_cmd_table[DFPS_LEVELNUM] = {
	/*switch to 60*/
	[DFPS_LEVEL0] = {
		true,//false
		LCM_SEND_IN_CMD,
		/*prev_frame cmd*/
		{
			{0xB0, 1, {0x04} },
			{0xF1, 1, {0x2A} },
			{0xC1, 1, {0x0C} },
			{0xC2, 6, {0x09, 0x24, 0x0E, 0x00, 0x00, 0x0E} },
			{0xC1, 5, {0x94, 0x42, 0x00, 0x16, 0x05} },
			{0xCF, 133, {0x64, 0x0B, 0x00,
					0x28, 0x02, 0x4B, 0x02, 0xDE, 0x0B, 0x77, 0x0B,
					0x8B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x3D, 0x00,
					0xCD, 0x00, 0xCD, 0x00, 0xCD, 0x00, 0xCD, 0x01,
					0x00, 0x01, 0x00, 0x03, 0x98, 0x03, 0x98, 0x03,
					0x98, 0x03, 0x98, 0x03, 0x98, 0x00, 0x3D, 0x00,
					0xCD, 0x00, 0xCD, 0x00, 0xCD, 0x00, 0xCD, 0x01,
					0x00, 0x01, 0x00, 0x03, 0x98, 0x03, 0x98, 0x03,
					0x98, 0x03, 0x98, 0x03, 0x98, 0x01, 0x42, 0x01,
					0x42, 0x01, 0x42, 0x01, 0x42, 0x01, 0x42, 0x01,
					0x42, 0x01, 0x42, 0x01, 0x42, 0x01, 0x42, 0x01,
					0x42, 0x01, 0x42, 0x01, 0x42, 0x1C, 0x1C, 0x1C,
					0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C,
					0x1C, 0x00, 0x88, 0x00, 0xB1, 0x00, 0xB1, 0x09,
					0xA6, 0x09, 0xA6, 0x09, 0xA4, 0x09, 0xA4, 0x09,
					0xA4, 0x09, 0xA4, 0x09, 0xA4, 0x09, 0xA4, 0x0F,
					0xC3, 0x19} },
			{0xC4, 18, {0x00, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					0x02, 0x00, 0x00, 0x00, 0x68, 0x00, 0x01} },
			{0xD7, 9, {0x00, 0x69, 0x34,
					0x00, 0xA0, 0x0A, 0x00, 0x00, 0x75} },
			{0xD8, 59, {0x00, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x74, 0x00,
					0x74, 0x00, 0x74, 0x00, 0x74, 0x00, 0x74, 0x05,
					0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x75, 0x00, 0x00, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					0x00, 0x22, 0x00, 0x65, 0x00, 0x00, 0x00, 0x2F,
					0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} },
			{0xBB, 49, {0x59, 0xC8, 0xC8,
					0xC8, 0xC8, 0xC8, 0xC8, 0xC8, 0xC8, 0xC8, 0x4A,
					0x48, 0x46, 0x44, 0x42, 0x40, 0x3E, 0x3C, 0x3A,
					0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
					0xFF, 0xFF, 0x04, 0x00, 0x01, 0x01, 0x00, 0x04,
					0x69, 0x5A, 0x00, 0x0B, 0x76, 0x0F, 0xFF, 0x0F,
					0xFF, 0x0F, 0xFF, 0x14, 0x81, 0xF4} },
			{0xE8, 2, {0x00, 0x02} },
			{0xE4, 2, {0x00, 0x0A} },
			{0xB0, 1, {0x84} },
			{0xE4, 9, {0x33, 0xB4, 0x00,
					0x00, 0x00, 0x75, 0x04, 0x00, 0x00} },
			{0xE6, 1, {0x01} },
			{REGFLAG_END_OF_TABLE, 0x00, {} },
		},
	},
#if 0
	/*switch to 90*/
	[DFPS_LEVEL1] = {
		true,//false
		LCM_SEND_IN_CMD,
		/*prev_frame cmd*/
		{
			{0xB0, 1, {0x04} },
			{0xF1, 1, {0x2A} },
			{0xC1, 1, {0x0C} },
			{0xC2, 6, {0x09, 0x24, 0x0E, 0x00, 0x00, 0x0E} },
			{0xC1, 5, {0x94, 0x42, 0x00, 0x16, 0x05} },
			{0xCF, 133, {0x64, 0x0B, 0x00,
					0x28, 0x02, 0x4B, 0x02, 0xDE, 0x0B, 0x77, 0x0B,
					0x8B, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,
					0x02, 0x02, 0x03, 0x03, 0x03, 0x00, 0x3D, 0x00,
					0xCD, 0x00, 0xCD, 0x00, 0xCD, 0x00, 0xCD, 0x01,
					0x00, 0x01, 0x00, 0x03, 0x98, 0x03, 0x98, 0x03,
					0x98, 0x03, 0x98, 0x03, 0x98, 0x00, 0x3D, 0x00,
					0xCD, 0x00, 0xCD, 0x00, 0xCD, 0x00, 0xCD, 0x01,
					0x00, 0x01, 0x00, 0x03, 0x98, 0x03, 0x98, 0x03,
					0x98, 0x03, 0x98, 0x03, 0x98, 0x01, 0x42, 0x01,
					0x42, 0x01, 0x42, 0x01, 0x42, 0x01, 0x42, 0x01,
					0x42, 0x01, 0x42, 0x01, 0x42, 0x01, 0x42, 0x01,
					0x42, 0x01, 0x42, 0x01, 0x42, 0x1C, 0x1C, 0x1C,
					0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C,
					0x1C, 0x00, 0x88, 0x00, 0xB1, 0x00, 0xB1, 0x09,
					0xA6, 0x09, 0xA6, 0x09, 0xA4, 0x09, 0xA4, 0x09,
					0xA4, 0x09, 0xA4, 0x09, 0xA4, 0x09, 0xA4, 0x0F,
					0xC3, 0x19} },
			{0xC4, 18, {0x00, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					0x02, 0x00, 0x00, 0x00, 0x48, 0x00, 0x01} },
			{0xD7, 9, {0x00, 0x69, 0x34,
					0x00, 0xA0, 0x0A, 0x00, 0x00, 0x4E} },
			{0xD8, 59, {0x00, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4D, 0x00,
					0x4D, 0x00, 0x4D, 0x00, 0x4D, 0x00, 0x4D, 0x05,
					0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x4E, 0x00, 0x00, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					0x00, 0x16, 0x00, 0x44, 0x00, 0x00, 0x00, 0x1F,
					0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} },
			{0xBB, 49, {0x59, 0xC8, 0xC8,
					0xC8, 0xC8, 0xC8, 0xC8, 0xC8, 0xC8, 0xC8, 0x4A,
					0x48, 0x46, 0x44, 0x42, 0x40, 0x3E, 0x3C, 0x3A,
					0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
					0xFF, 0xFF, 0x04, 0x00, 0x02, 0x02, 0x00, 0x04,
					0x69, 0x5A, 0x00, 0x0B, 0x76, 0x0F, 0xFF, 0x0F,
					0xFF, 0x0F, 0xFF, 0x14, 0x81, 0xF4} },
			{0xE8, 2, {0x00, 0x02} },
			{0xE4, 2, {0x00, 0x0A} },
			{0xB0, 1, {0x84} },
			{0xE4, 9, {0x33, 0xB4, 0x00,
					0x00, 0x00, 0x4E, 0x04, 0x04, 0x9A} },
			{0xE6, 1, {0x01} },
			{REGFLAG_END_OF_TABLE, 0x00, {} },
		},
	},
#endif
	/*switch to 120*/
	[DFPS_LEVEL1] = {
		true,//false
		LCM_SEND_IN_CMD,
		/*prev_frame cmd*/
		{
			{0xB0, 1, {0x04} },
			{0xC1, 43, {0x94, 0x42, 0x00,
					0x16, 0x05, 0x00, 0x00, 0x00, 0x10, 0x00, 0x10,
					0x00, 0xAA, 0x8A, 0x02, 0x10, 0x00, 0x10, 0x00,
					0x00, 0x3F, 0x3F, 0x03, 0xFF, 0x03, 0xFF, 0x23,
					0xFF, 0x03, 0xFF, 0x23, 0xFF, 0x03, 0xFF, 0x00,
					0x40, 0x40, 0x00, 0x00, 0x10, 0x01, 0x00, 0x0C} },
			{0xC2, 6, {0x09, 0x24, 0x0E, 0x00, 0x00, 0x0E} },
			{0xC4, 18, {0x00, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					0x02, 0x00, 0x00, 0x00, 0x35, 0x00, 0x01} },
			{0xCF, 133, {0x64, 0x0B, 0x00,
					0x28, 0x02, 0x4B, 0x02, 0xDE, 0x0B, 0x77, 0x0B,
					0x8B, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
					0x04, 0x04, 0x05, 0x05, 0x05, 0x00, 0x3D, 0x00,
					0xCD, 0x00, 0xCD, 0x00, 0xCD, 0x00, 0xCD, 0x01,
					0x00, 0x01, 0x00, 0x03, 0x98, 0x03, 0xA9, 0x03,
					0xA9, 0x03, 0xA9, 0x03, 0xA9, 0x00, 0x3D, 0x00,
					0xCD, 0x00, 0xCD, 0x00, 0xCD, 0x00, 0xCD, 0x01,
					0x00, 0x01, 0x00, 0x03, 0x98, 0x03, 0xA9, 0x03,
					0xA9, 0x03, 0xA9, 0x03, 0xA9, 0x01, 0x42, 0x01,
					0x42, 0x01, 0x42, 0x01, 0x42, 0x01, 0x42, 0x01,
					0x42, 0x01, 0x42, 0x01, 0x42, 0x01, 0x42, 0x01,
					0x42, 0x01, 0x42, 0x01, 0x42, 0x1C, 0x1C, 0x1C,
					0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C,
					0x1C, 0x00, 0x88, 0x00, 0xB1, 0x00, 0xB1, 0x09,
					0xA6, 0x09, 0xA6, 0x09, 0xA4, 0x09, 0xA4, 0x09,
					0xA4, 0x09, 0xA4, 0x09, 0xA4, 0x09, 0xA4, 0x0F,
					0xC3, 0x19} },
			{0xD7, 9, {0x00, 0x69, 0x34,
					0x00, 0xA0, 0x0A, 0x00, 0x00, 0x39} },
			{0xD8, 59, {0x00, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3A, 0x00,
					0x3A, 0x00, 0x3A, 0x00, 0x3A, 0x00, 0x3A, 0x05,
					0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					0x00, 0x00, 0x20, 0x40, 0x00, 0x00, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					0x00, 0x0F, 0x00, 0x32, 0x00, 0x00, 0x00, 0x17,
					0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} },
			{0xBB, 49, {0x59, 0xC8, 0xC8,
					0xC8, 0xC8, 0xC8, 0xC8, 0xC8, 0xC8, 0xC8, 0x4A,
					0x48, 0x46, 0x44, 0x42, 0x40, 0x3E, 0x3C, 0x3A,
					0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
					0xFF, 0xFF, 0x04, 0x00, 0x02, 0x02, 0x00, 0x04,
					0x69, 0x5A, 0x00, 0x0B, 0x76, 0x0F, 0xFF, 0x0F,
					0xFF, 0x0F, 0xFF, 0x14, 0x81, 0xF4} },
			{0xE8, 2, {0x00, 0x02} },
			{0xE4, 2, {0x00, 0x0A} },
			{0xB0, 1, {0x84} },
			{0xE4, 9, {0x33, 0xB4, 0x00,
					0x00, 0x00, 0x39, 0x04, 0x09, 0x34} },
			{0xE6, 1, {0x01} },
			{REGFLAG_END_OF_TABLE, 0x00, {} },
		},
	},
};
#endif

static void push_table(struct LCM_setting_table *table, unsigned int count,
		       unsigned char force_update)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		unsigned int cmd;

		cmd = table[i].cmd;
		switch (cmd) {
		case REGFLAG_DELAY:
			if (table[i].count <= 10)
				MDELAY(table[i].count);
			else
				MDELAY(table[i].count);
			break;
		case REGFLAG_UDELAY:
			UDELAY(table[i].count);
			break;
		case REGFLAG_END_OF_TABLE:
			break;
		default:
			if (table[i].count || table[i].cmd) {
				dsi_set_cmdq_V2(cmd, table[i].count,
						table[i].para_list, force_update);
				if (table[i].count > 1)
					MDELAY(1);
			}
			break;
		}
	}
}

/* ----------------------------------------------------------------------- */
/* LCM Driver Implementations */
/* ----------------------------------------------------------------------- */

static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}

#ifdef CONFIG_MTK_HIGH_FRAME_RATE
static void lcm_dfps_int(struct LCM_DSI_PARAMS *dsi)
{
	struct dfps_info *dfps_params = dsi->dfps_params;

	dsi->dfps_enable = 1;
	dsi->dfps_default_fps = DEFAULT_FPS;/*real fps * 100, to support float*/
	dsi->dfps_def_vact_tim_fps = DEFAULT_FPS;/*real vact timing fps * 100*/

	/*traversing array must less than DFPS_LEVELS*/
	/*DPFS_LEVEL0*/
	dfps_params[0].level = DFPS_LEVEL0;
	dfps_params[0].fps = 6000;/*real fps * 100, to support float*/
	dfps_params[0].vact_timing_fps = 6000;/*real vact timing fps * 100*/
	/*if mipi clock solution*/
	/*dfps_params[0].PLL_CLOCK = xx;*/
	/*dfps_params[0].data_rate = xx; */

	/*DPFS_LEVEL1*/
	dfps_params[1].level = DFPS_LEVEL1;
	dfps_params[1].fps = 12000;/*real fps * 100, to support float*/
	dfps_params[1].vact_timing_fps = 12000;/*real vact timing fps * 100*/
	/*if mipi clock solution*/
	/*dfps_params[1].PLL_CLOCK = xx;*/
	/*dfps_params[1].data_rate = xx; */
#if 0
	/*DPFS_LEVEL2*/
	dfps_params[2].level = DFPS_LEVEL2;
	dfps_params[2].fps = 12000;/*real fps * 100, to support float*/
	dfps_params[2].vact_timing_fps = 12000;/*real vact timing fps * 100*/
#endif
	dsi->dfps_num = 2;
}
#endif

static void lcm_get_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode = CMD_MODE;
	params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
#else
	params->dsi.mode = SYNC_PULSE_VDO_MODE;
	params->dsi.switch_mode = CMD_MODE;
#endif
	params->dsi.switch_mode_enable = 0;

	/* DSI */
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	/* video mode timing */

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active = 2;
	params->dsi.vertical_backporch = 8;
	params->dsi.vertical_frontporch = 20;
	params->dsi.vertical_frontporch_for_low_power = 620;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 10;
	params->dsi.horizontal_backporch = 20;
	params->dsi.horizontal_frontporch = 40;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	/* params->dsi.ssc_disable = 1; */
	params->dsi.ssc_disable = 1;
	params->dsi.bdg_ssc_disable = 1;
	params->dsi.bdg_dsc_enable = 1;
	params->dsi.dsc_enable = 0;
#ifndef MACH_FPGA
#if (LCM_DSI_CMD_MODE)
	params->dsi.PLL_CLOCK = 190;//190; /* ori clk = 570M, 1/3 compression */
#else
	params->dsi.PLL_CLOCK = 190;
#endif
#else
	params->dsi.pll_div1 = 0;
	params->dsi.pll_div2 = 0;
	params->dsi.fbk_div = 0x1;
#endif
#ifdef _HIGH_FRM_
	params->dsi.PLL_CLOCK = 380;
#else
	params->dsi.PLL_CLOCK = 190;
#endif
	params->dsi.CLK_HS_POST = 36;
	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 0;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.lcm_esd_check_table[0].cmd = 0x53;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x24;

#ifdef MTK_ROUND_CORNER_SUPPORT
	params->round_corner_params.round_corner_en = 1;
	params->round_corner_params.full_content = 0;
	params->round_corner_params.h = ROUND_CORNER_H_TOP;
	params->round_corner_params.h_bot = ROUND_CORNER_H_BOT;
	params->round_corner_params.tp_size = sizeof(top_rc_pattern);
	params->round_corner_params.lt_addr = (void *)top_rc_pattern;
#endif
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	/****DynFPS start****/
	lcm_dfps_int(&(params->dsi));
	/****DynFPS end****/
#endif

#if 0
	params->dsi.dsc_enable = 1;
	params->dsi.dsc_params.ver = 17;
	params->dsi.dsc_params.slice_mode = 1;
	params->dsi.dsc_params.rgb_swap = 0;
	params->dsi.dsc_params.dsc_cfg = 34;
	params->dsi.dsc_params.rct_on = 1;
	params->dsi.dsc_params.bit_per_channel = 8;
	params->dsi.dsc_params.dsc_line_buf_depth = 9;
	params->dsi.dsc_params.bp_enable = 1;
	params->dsi.dsc_params.bit_per_pixel = 128;
	params->dsi.dsc_params.pic_height = 2340;
	params->dsi.dsc_params.pic_width = 1080;
	params->dsi.dsc_params.slice_height = 20;
	params->dsi.dsc_params.slice_width = 540;
	params->dsi.dsc_params.chunk_size = 540;
	params->dsi.dsc_params.xmit_delay = 512;
	params->dsi.dsc_params.dec_delay = 526;
	params->dsi.dsc_params.scale_value = 32;
	params->dsi.dsc_params.increment_interval = 488;
	params->dsi.dsc_params.decrement_interval = 7;
	params->dsi.dsc_params.line_bpg_offset = 12;
	params->dsi.dsc_params.nfl_bpg_offset = 1294;
	params->dsi.dsc_params.slice_bpg_offset = 1302;
	params->dsi.dsc_params.initial_offset = 6144;
	params->dsi.dsc_params.final_offset = 4336;
	params->dsi.dsc_params.flatness_minqp = 3;
	params->dsi.dsc_params.flatness_maxqp = 12;
	params->dsi.dsc_params.rc_model_size = 8192;
	params->dsi.dsc_params.rc_edge_factor = 6;
	params->dsi.dsc_params.rc_quant_incr_limit0 = 11;
	params->dsi.dsc_params.rc_quant_incr_limit1 = 11;
	params->dsi.dsc_params.rc_tgt_offset_hi = 3;
	params->dsi.dsc_params.rc_tgt_offset_lo = 3;
#endif
}

static void lcm_init_power(void)
{
}

static void lcm_suspend_power(void)
{
}

static void lcm_resume_power(void)
{
}

static void lcm_setbacklight(unsigned int level)
{
	unsigned int BacklightLevel;

	LCM_LOGI("%s,lk samsung backlight: level = %d\n", __func__, level);
	if (level > 255)
		level = 255;

	if (level > 0)
		BacklightLevel = level * 4095 / 255;
	else
		BacklightLevel = 0;

	bl_level[0].para_list[0] = ((BacklightLevel >> 8) & 0xF);
	bl_level[0].para_list[1] = (BacklightLevel & 0xFF);

	push_table(bl_level, ARRAY_SIZE(bl_level), 1);
}

static void lcm_init(void)
{
//	int ret = 0;

	SET_RESET_PIN(0);
	MDELAY(10);

	SET_RESET_PIN(1);
	MDELAY(10);

	push_table(init_setting, sizeof(init_setting) /
		   sizeof(init_setting[0]), 1);
#ifdef __FIX_120__
#if (DEFAULT_FPS == 12000)
	LCM_LOGI("%s, gavin lcm 120\n", __func__);
	push_table(switch_120_setting, sizeof(switch_120_setting) /
		   sizeof(switch_120_setting[0]), 1);
#else
	LCM_LOGI("%s, gavin lcm 90\n", __func__);
	push_table(switch_90_setting, sizeof(switch_90_setting) /
		   sizeof(switch_90_setting[0]), 1);
#endif
#endif

	/* lcm_setbacklight(255); */
}

static void lcm_suspend(void)
{
//	int ret;

	push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting) /
		   sizeof(lcm_suspend_setting[0]), 1);

	SET_RESET_PIN(0);
	MDELAY(10);
}

static void lcm_resume(void)
{
	lcm_init();
}

static void lcm_update(unsigned int x, unsigned int y, unsigned int width,
		       unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);
	unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
	unsigned char y0_LSB = (y0 & 0xFF);
	unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
	unsigned char y1_LSB = (y1 & 0xFF);

	unsigned int data_array[3];

	data_array[0] = 0x00053902;
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00053902;
	data_array[1] = (y1_MSB << 24) | (y0_LSB << 16) | (y0_MSB << 8) | 0x2b;
	data_array[2] = (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
}

static unsigned int lcm_compare_id(void)
{
	unsigned int id = 0;// version_id = 0;
	unsigned char buffer[1];
	unsigned int array[1];

	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);

	SET_RESET_PIN(1);
	MDELAY(20);

	array[0] = 0x00023700; /* read id return two byte,version and id */
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0xda, buffer, 1);
	id = buffer[0]; /* we only need ID */

	LCM_LOGI("%s,samsung=0x%08x\n", __func__, id);

	if (id == 0x01) /* TODO */
		return 1;
	else
		return 0;
}

static unsigned int lcm_ata_check(unsigned char *buffer)
{
	unsigned int ret = 0;
#ifndef BUILD_LK
	unsigned int x0 = FRAME_WIDTH / 4;
	unsigned int x1 = FRAME_WIDTH * 3 / 4;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);

	unsigned int data_array[3];
	unsigned char read_buf[4];

	LCM_LOGI("ATA check size = 0x%x,0x%x,0x%x,0x%x\n",
		 x0_MSB, x0_LSB, x1_MSB, x1_LSB);
	data_array[0] = 0x0005390A; /* HS packet */
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00043700; /* return two byte, version and id */
	dsi_set_cmdq(data_array, 1, 1);

	read_reg_v2(0x2A, read_buf, 4);

	if ((read_buf[0] == x0_MSB) && (read_buf[1] == x0_LSB) &&
	    (read_buf[2] == x1_MSB) && (read_buf[3] == x1_LSB))
		ret = 1;
	else
		ret = 0;

	x0 = 0;
	x1 = FRAME_WIDTH - 1;

	x0_MSB = ((x0 >> 8) & 0xFF);
	x0_LSB = (x0 & 0xFF);
	x1_MSB = ((x1 >> 8) & 0xFF);
	x1_LSB = (x1 & 0xFF);

	data_array[0] = 0x0005390A; /* HS packet */
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
#endif
	return ret;
}

#ifdef CONFIG_MTK_HIGH_FRAME_RATE
#if 0
static void dfps_dsi_push_table(
	void *cmdq, struct LCM_setting_table *table,
	unsigned int count, unsigned char force_update, enum LCM_Send_Cmd_Mode sendmode)
{
	unsigned int i;
	unsigned int cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;
		switch (cmd) {
		case REGFLAG_END_OF_TABLE:
			return;
		default:
			dfps_dsi_send_cmd(
				cmdq, cmd, table[i].count,
				table[i].para_list, force_update, sendmode);
			break;
		}
	}

}
#endif

static bool lcm_dfps_need_inform_lcm(
	unsigned int from_level, unsigned int to_level, struct LCM_PARAMS *params)
{
	struct LCM_dfps_cmd_table *p_dfps_cmds = NULL;

	if (from_level == to_level) {
		LCM_LOGI("%s,same level\n", __func__);
		return false;
	}
	p_dfps_cmds = &(dfps_cmd_table[to_level]);
	params->sendmode = p_dfps_cmds->sendmode;

	return p_dfps_cmds->need_send_cmd;
}

static void lcm_dfps_inform_lcm(void *cmdq_handle,
	unsigned int from_level, unsigned int to_level, struct LCM_PARAMS *params)
{
	struct LCM_dfps_cmd_table *p_dfps_cmds = NULL;
	enum LCM_Send_Cmd_Mode sendmode = LCM_SEND_IN_CMD;

	if (from_level == to_level) {
		LCM_LOGI("%s,same level\n", __func__);
		goto done;
	}
	p_dfps_cmds =
		&(dfps_cmd_table[to_level]);
	LCM_LOGI("%s,send cmd[L%d->L%d]\n",
		__func__, from_level, to_level);

	if (p_dfps_cmds &&
		!(p_dfps_cmds->need_send_cmd)) {
		LCM_LOGI("%s,no cmd[L%d->L%d]\n",
			__func__, from_level, to_level);
		goto done;
	}

	sendmode = params->sendmode;
	LCM_LOGI("%s count=%d\n ", __func__, ARRAY_SIZE(p_dfps_cmds->prev_f_cmd));
#if 0
	dfps_dsi_push_table(
		cmdq_handle, p_dfps_cmds->prev_f_cmd,
		ARRAY_SIZE(p_dfps_cmds->prev_f_cmd), 1, sendmode);
#else
	push_table(p_dfps_cmds->prev_f_cmd,
			ARRAY_SIZE(p_dfps_cmds->prev_f_cmd), 1);
#endif

done:
	LCM_LOGI("%s,done %d->%d\n",
		__func__, from_level, to_level, sendmode);

}
#endif


struct LCM_DRIVER r66451_fhdp_dsi_cmd_tianma_lcm_drv = {
	.name = "r66451_fhdp_dsi_cmd_tianma_drv",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.compare_id = lcm_compare_id,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
	.set_backlight = lcm_setbacklight,
	.ata_check = lcm_ata_check,
	.update = lcm_update,
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	/*DynFPS*/
	.dfps_send_lcm_cmd = lcm_dfps_inform_lcm,
	.dfps_need_send_cmd = lcm_dfps_need_inform_lcm,
#endif

};
