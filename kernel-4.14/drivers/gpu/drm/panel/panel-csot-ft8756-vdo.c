/*
 * Copyright (c) 2015 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/backlight.h>
#include <drm/drmP.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mtk_panel_ext.h"
#include "../mediatek/mtk_log.h"
#include "../mediatek/mtk_drm_graphics_base.h"
#endif

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mtk_corner_pattern/mtk_data_hw_roundedpattern.h"
#endif
//prize added by huangjiwu, lcm support, 20200918-start
#if defined(CONFIG_PRIZE_HARDWARE_INFO)
#include "../../../misc/mediatek/prize/hardware_info/hardware_info.h"
extern struct hardware_info current_lcm_info;
#endif
//prize added by huangjiwu, lcm support, 20200918-end
#include "prize_lcd_bias.h"

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *bias_pos, *bias_neg;

	bool prepared;
	bool enabled;

	int error;
};

#define lcm_dcs_write_seq(ctx, seq...) \
({\
	const u8 d[] = { seq };\
	BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 64, "DCS sequence too big for stack");\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

#define lcm_dcs_write_seq_static(ctx, seq...) \
({\
	static const u8 d[] = { seq };\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

static inline struct lcm *panel_to_lcm(struct drm_panel *panel)
{
	return container_of(panel, struct lcm, panel);
}

static void lcm_dcs_write(struct lcm *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;
	char *addr;

	if (ctx->error < 0)
		return;

	addr = (char *)data;
	if ((int)*addr < 0xB0)
		ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	else
		ret = mipi_dsi_generic_write(dsi, data, len);
	if (ret < 0) {
		dev_err(ctx->dev, "error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

#ifdef PANEL_SUPPORT_READBACK
static int lcm_dcs_read(struct lcm *ctx, u8 cmd, void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0)
		return 0;

	ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
	if (ret < 0) {
		dev_err(ctx->dev, "error %d reading dcs seq:(%#x)\n", ret, cmd);
		ctx->error = ret;
	}

	return ret;
}

static void lcm_panel_get_data(struct lcm *ctx)
{
	u8 buffer[3] = {0};
	static int ret;

	if (ret == 0) {
		ret = lcm_dcs_read(ctx,  0x0A, buffer, 1);
		dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
			 ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif

static void lcm_panel_init(struct lcm *ctx)
{
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return;
	}
	//gpiod_set_value(ctx->reset_gpio, 0);
	//udelay(15 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	msleep(10);
	gpiod_set_value(ctx->reset_gpio, 0);
	msleep(10);
	gpiod_set_value(ctx->reset_gpio, 1);
	msleep(5);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

lcm_dcs_write_seq_static(ctx,0x00,0x00);
lcm_dcs_write_seq_static(ctx,0xFF,0x87,0x56,0x01);
lcm_dcs_write_seq_static(ctx,0x00,0x80);
lcm_dcs_write_seq_static(ctx,0xFF,0x87,0x56);
lcm_dcs_write_seq_static(ctx,0x00,0xA1);
lcm_dcs_write_seq_static(ctx,0xB3,0x04,0x38,0x09,0x60,0x00,0xFC);
lcm_dcs_write_seq_static(ctx,0x00,0x80);
lcm_dcs_write_seq_static(ctx,0xC0,0x00,0x8D,0x00,0x08,0x00,0x24);
lcm_dcs_write_seq_static(ctx,0x00,0x90);
lcm_dcs_write_seq_static(ctx,0xC0,0x00,0x8D,0x00,0x08,0x00,0x24);
lcm_dcs_write_seq_static(ctx,0x00,0xA0);
lcm_dcs_write_seq_static(ctx,0xC0,0x01,0x18,0x00,0x08,0x00,0x24);
lcm_dcs_write_seq_static(ctx,0x00,0xB0);
lcm_dcs_write_seq_static(ctx,0xC0,0x00,0x8D,0x00,0x08,0x24);
lcm_dcs_write_seq_static(ctx,0x00,0xC1);
lcm_dcs_write_seq_static(ctx,0xC0,0x00,0xD0,0x00,0x9F,0x00,0x8B,0x00,0xEE);
lcm_dcs_write_seq_static(ctx,0x00,0xD7);
lcm_dcs_write_seq_static(ctx,0xC0,0x00,0x8B,0x00,0x08,0x00,0x24);
lcm_dcs_write_seq_static(ctx,0x00,0xA3);
lcm_dcs_write_seq_static(ctx,0xC1,0x00,0x2C,0x00,0x2C,0x00,0x02);
lcm_dcs_write_seq_static(ctx,0x00,0x80);
lcm_dcs_write_seq_static(ctx,0xCE,0x01,0x81,0x09,0x13,0x00,0xD0,0x00,0xE8,0x00,0x8A,0x00,0x9A,0x00,0x68,0x00,0x74);
lcm_dcs_write_seq_static(ctx,0x00,0x90);
lcm_dcs_write_seq_static(ctx,0xCE,0x00,0x8E,0x0C,0xDF,0x00,0x8E,0x80,0x09,0x13,0x00,0x04,0x00,0x21,0x1F,0x16);
lcm_dcs_write_seq_static(ctx,0x00,0xA0);
lcm_dcs_write_seq_static(ctx,0xCE,0x00,0x00,0x00);
lcm_dcs_write_seq_static(ctx,0x00,0xB0);
lcm_dcs_write_seq_static(ctx,0xCE,0x22,0x00,0x00);
lcm_dcs_write_seq_static(ctx,0x00,0xD1);
lcm_dcs_write_seq_static(ctx,0xCE,0x00,0x00,0x01,0x00,0x00,0x00,0x00);
lcm_dcs_write_seq_static(ctx,0x00,0xE1);
lcm_dcs_write_seq_static(ctx,0xCE,0x08,0x02,0x4D,0x02,0x4D,0x02,0x4D,0x00,0x00,0x00,0x00);
lcm_dcs_write_seq_static(ctx,0x00,0xF1);
lcm_dcs_write_seq_static(ctx,0xCE,0x13,0x09,0x0D,0x00,0xF6,0x01,0x96,0x00,0xC2);
lcm_dcs_write_seq_static(ctx,0x00,0xB0);
lcm_dcs_write_seq_static(ctx,0xCF,0x00,0x00,0xBF,0xC3);
lcm_dcs_write_seq_static(ctx,0x00,0xB5);
lcm_dcs_write_seq_static(ctx,0xCF,0x04,0x04,0xE4,0xE8);
lcm_dcs_write_seq_static(ctx,0x00,0xC0);
lcm_dcs_write_seq_static(ctx,0xCF,0x09,0x09,0x2C,0x30);
lcm_dcs_write_seq_static(ctx,0x00,0xC5);
lcm_dcs_write_seq_static(ctx,0xCF,0x00,0x00,0x08,0x0C);
lcm_dcs_write_seq_static(ctx,0x00,0xE8);
lcm_dcs_write_seq_static(ctx,0xC0,0x40);
lcm_dcs_write_seq_static(ctx,0x00,0x80);
lcm_dcs_write_seq_static(ctx,0xC2,0x84,0x00,0x01,0x8D,0x83,0x00,0x01,0x8D);
lcm_dcs_write_seq_static(ctx,0x00,0xA0);
lcm_dcs_write_seq_static(ctx,0xC2,0x82,0x04,0x00,0x02,0x8C,0x81,0x04,0x00,0x02,0x8C,0x00,0x04,0x00,0x02,0x8C);
lcm_dcs_write_seq_static(ctx,0x00,0xB0);
lcm_dcs_write_seq_static(ctx,0xC2,0x01,0x04,0x00,0x02,0x8C,0x02,0x04,0x00,0x02,0x8C,0x03,0x04,0x00,0x02,0x8C);
lcm_dcs_write_seq_static(ctx,0x00,0xC0);
lcm_dcs_write_seq_static(ctx,0xC2,0x04,0x04,0x00,0x02,0x8C,0x05,0x04,0x00,0x02,0x8C);
lcm_dcs_write_seq_static(ctx,0x00,0xE0);
lcm_dcs_write_seq_static(ctx,0xC2,0x77,0x77,0x77,0x77);
lcm_dcs_write_seq_static(ctx,0x00,0xC0);
lcm_dcs_write_seq_static(ctx,0xC3,0x99,0x99,0x99,0x99);
lcm_dcs_write_seq_static(ctx,0x00,0x80);
lcm_dcs_write_seq_static(ctx,0xCB,0x00,0xC5,0x00,0x00,0x05,0x05,0x00,0x05,0x0A,0x05,0xC5,0x00,0x05,0x05,0x00,0xC0);
lcm_dcs_write_seq_static(ctx,0x00,0x90);
lcm_dcs_write_seq_static(ctx,0xCB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
lcm_dcs_write_seq_static(ctx,0x00,0xA0);
lcm_dcs_write_seq_static(ctx,0xCB,0x00,0x00,0x00,0x00);
lcm_dcs_write_seq_static(ctx,0x00,0xB0);
lcm_dcs_write_seq_static(ctx,0xCB,0x10,0x51,0x94,0x50);
lcm_dcs_write_seq_static(ctx,0x00,0xC0);
lcm_dcs_write_seq_static(ctx,0xCB,0x10,0x51,0x94,0x50);
lcm_dcs_write_seq_static(ctx,0x00,0x80);
lcm_dcs_write_seq_static(ctx,0xCC,0x00,0x00,0x00,0x00,0x07,0x09,0x0B,0x0D,0x25,0x25,0x03,0x00,0x22,0x00,0x24,0x00);
lcm_dcs_write_seq_static(ctx,0x00,0x90);
lcm_dcs_write_seq_static(ctx,0xCC,0x29,0x16,0x17,0x18,0x19,0x1A,0x1B,0x26);
lcm_dcs_write_seq_static(ctx,0x00,0x80);
lcm_dcs_write_seq_static(ctx,0xCD,0x00,0x00,0x00,0x00,0x06,0x08,0x0A,0x0C,0x25,0x00,0x02,0x00,0x22,0x00,0x24,0x00);
lcm_dcs_write_seq_static(ctx,0x00,0x90);
lcm_dcs_write_seq_static(ctx,0xCD,0x29,0x16,0x17,0x18,0x19,0x1A,0x1B,0x26);
lcm_dcs_write_seq_static(ctx,0x00,0xA0);
lcm_dcs_write_seq_static(ctx,0xCC,0x00,0x00,0x00,0x00,0x08,0x06,0x0C,0x0A,0x25,0x25,0x02,0x00,0x24,0x00,0x23,0x00);
lcm_dcs_write_seq_static(ctx,0x00,0xB0);
lcm_dcs_write_seq_static(ctx,0xCC,0x29,0x16,0x17,0x18,0x19,0x1A,0x1B,0x26);
lcm_dcs_write_seq_static(ctx,0x00,0xA0);
lcm_dcs_write_seq_static(ctx,0xCD,0x00,0x00,0x00,0x00,0x09,0x07,0x0D,0x0B,0x25,0x00,0x02,0x00,0x24,0x00,0x23,0x00);
lcm_dcs_write_seq_static(ctx,0x00,0xB0);
lcm_dcs_write_seq_static(ctx,0xCD,0x29,0x16,0x17,0x18,0x19,0x1A,0x1B,0x26);
lcm_dcs_write_seq_static(ctx,0x00,0x80);
lcm_dcs_write_seq_static(ctx,0xA7,0x13);
lcm_dcs_write_seq_static(ctx,0x00,0x82);
lcm_dcs_write_seq_static(ctx,0xA7,0x22,0x02);
lcm_dcs_write_seq_static(ctx,0x00,0x85);
lcm_dcs_write_seq_static(ctx,0xC4,0x1C);
lcm_dcs_write_seq_static(ctx,0x00,0x97);
lcm_dcs_write_seq_static(ctx,0xC4,0x01);
lcm_dcs_write_seq_static(ctx,0x00,0xA0);
lcm_dcs_write_seq_static(ctx,0xC4,0x8D,0xD8,0x8D);
lcm_dcs_write_seq_static(ctx,0x00,0x93);
lcm_dcs_write_seq_static(ctx,0xC5,0x37);
lcm_dcs_write_seq_static(ctx,0x00,0x97);
lcm_dcs_write_seq_static(ctx,0xC5,0x37);
lcm_dcs_write_seq_static(ctx,0x00,0xB6);
lcm_dcs_write_seq_static(ctx,0xC5,0x23,0x23);
lcm_dcs_write_seq_static(ctx,0x00,0x9B);
lcm_dcs_write_seq_static(ctx,0xF5,0x4B);
lcm_dcs_write_seq_static(ctx,0x00,0x93);
lcm_dcs_write_seq_static(ctx,0xF5,0x00,0x00);
lcm_dcs_write_seq_static(ctx,0x00,0x9D);
lcm_dcs_write_seq_static(ctx,0xF5,0x49);
lcm_dcs_write_seq_static(ctx,0x00,0x82);
lcm_dcs_write_seq_static(ctx,0xF5,0x00,0x00);
lcm_dcs_write_seq_static(ctx,0x00,0x8C);
lcm_dcs_write_seq_static(ctx,0xC3,0x00,0x00,0x00);
lcm_dcs_write_seq_static(ctx,0x00,0x84);
lcm_dcs_write_seq_static(ctx,0xC5,0x28,0x28);
lcm_dcs_write_seq_static(ctx,0x00,0xA4);
lcm_dcs_write_seq_static(ctx,0xD7,0x00);
lcm_dcs_write_seq_static(ctx,0x00,0x80);
lcm_dcs_write_seq_static(ctx,0xF5,0x59,0x59);
lcm_dcs_write_seq_static(ctx,0x00,0x84);
lcm_dcs_write_seq_static(ctx,0xF5,0x59,0x59,0x59);
lcm_dcs_write_seq_static(ctx,0x00,0x96);
lcm_dcs_write_seq_static(ctx,0xF5,0x59);
lcm_dcs_write_seq_static(ctx,0x00,0xA6);
lcm_dcs_write_seq_static(ctx,0xF5,0x59);
lcm_dcs_write_seq_static(ctx,0x00,0xCA);
lcm_dcs_write_seq_static(ctx,0xC0,0x80);
lcm_dcs_write_seq_static(ctx,0x00,0xB1);
lcm_dcs_write_seq_static(ctx,0xF5,0x1F);
lcm_dcs_write_seq_static(ctx,0x00,0x00);
lcm_dcs_write_seq_static(ctx,0xD8,0x2B,0x2B);
lcm_dcs_write_seq_static(ctx,0x00,0x00);
lcm_dcs_write_seq_static(ctx,0xD9,0x1E,0x1E);
lcm_dcs_write_seq_static(ctx,0x00,0x86);
lcm_dcs_write_seq_static(ctx,0xC0,0x01,0x03,0x01,0x01,0x10,0x03);
lcm_dcs_write_seq_static(ctx,0x00,0x96);
lcm_dcs_write_seq_static(ctx,0xC0,0x01,0x03,0x01,0x01,0x10,0x03);
lcm_dcs_write_seq_static(ctx,0x00,0xA6);
lcm_dcs_write_seq_static(ctx,0xC0,0x01,0x03,0x01,0x01,0x10,0x03);
lcm_dcs_write_seq_static(ctx,0x00,0xE9);
lcm_dcs_write_seq_static(ctx,0xC0,0x01,0x03,0x01,0x01,0x10,0x03);
lcm_dcs_write_seq_static(ctx,0x00,0xA3);
lcm_dcs_write_seq_static(ctx,0xCE,0x01,0x03,0x01,0x01,0x10,0x03);
lcm_dcs_write_seq_static(ctx,0x00,0xB3);
lcm_dcs_write_seq_static(ctx,0xCE,0x01,0x03,0x01,0x01,0x10,0x03);

//Gamma2.2
lcm_dcs_write_seq_static(ctx,0x00,0x00);
lcm_dcs_write_seq_static(ctx, 0xE1, 0x05,0x06,0x09,0x10,0x6D,0x1A,0x21,0x27,0x31,0x6E,0x39,0x3F,0x45,0x4A,0xB9,0x4E,0x56,0x5D,0x64,0xE9,0x6A,0x71,0x79,0x81,0xE1,0x8A,0x90,0x96,0x9D,0xC6,0xA6,0xB1,0xBF,0xC8,0x40,0xD3,0xE6,0xF4,0xFF,0xCF);
lcm_dcs_write_seq_static(ctx, 0x00, 0x00);
lcm_dcs_write_seq_static(ctx, 0xE2, 0x05,0x06,0x09,0x10,0x6D,0x1A,0x21,0x27,0x31,0x6E,0x39,0x3F,0x45,0x4A,0xB9,0x4E,0x56,0x5D,0x64,0xE9,0x6A,0x71,0x79,0x81,0xE1,0x8A,0x90,0x96,0x9D,0xC6,0xA6,0xB1,0xBF,0xC8,0x40,0xD3,0xE6,0xF4,0xFF,0xCF);
lcm_dcs_write_seq_static(ctx, 0x00, 0x00);
lcm_dcs_write_seq_static(ctx, 0xE3, 0x05,0x06,0x09,0x10,0x6D,0x1A,0x21,0x27,0x31,0x6E,0x39,0x3F,0x45,0x4A,0xB9,0x4E,0x56,0x5D,0x64,0xE9,0x6A,0x71,0x79,0x81,0xE1,0x8A,0x90,0x96,0x9D,0xC6,0xA6,0xB1,0xBF,0xC8,0x40,0xD3,0xE6,0xF4,0xFF,0xCF);
lcm_dcs_write_seq_static(ctx, 0x00, 0x00);
lcm_dcs_write_seq_static(ctx, 0xE4, 0x05,0x06,0x09,0x10,0x6D,0x1A,0x21,0x27,0x31,0x6E,0x39,0x3F,0x45,0x4A,0xB9,0x4E,0x56,0x5D,0x64,0xE9,0x6A,0x71,0x79,0x81,0xE1,0x8A,0x90,0x96,0x9D,0xC6,0xA6,0xB1,0xBF,0xC8,0x40,0xD3,0xE6,0xF4,0xFF,0xCF);
lcm_dcs_write_seq_static(ctx, 0x00, 0x00);
lcm_dcs_write_seq_static(ctx, 0xE5, 0x05,0x06,0x09,0x10,0x6D,0x1A,0x21,0x27,0x31,0x6E,0x39,0x3F,0x45,0x4A,0xB9,0x4E,0x56,0x5D,0x64,0xE9,0x6A,0x71,0x79,0x81,0xE1,0x8A,0x90,0x96,0x9D,0xC6,0xA6,0xB1,0xBF,0xC8,0x40,0xD3,0xE6,0xF4,0xFF,0xCF);
lcm_dcs_write_seq_static(ctx, 0x00,0x00);
lcm_dcs_write_seq_static(ctx, 0xE6,0x05,0x06,0x09,0x10,0x6D,0x1A,0x21,0x27,0x31,0x6E,0x39,0x3F,0x45,0x4A,0xB9,0x4E,0x56,0x5D,0x64,0xE9,0x6A,0x71,0x79,0x81,0xE1,0x8A,0x90,0x96,0x9D,0xC6,0xA6,0xB1,0xBF,0xC8,0x40,0xD3,0xE6,0xF4,0xFF,0xCF);

lcm_dcs_write_seq_static(ctx,0x00,0xCC);
lcm_dcs_write_seq_static(ctx,0xC0,0x13);
lcm_dcs_write_seq_static(ctx,0x00,0x82);
lcm_dcs_write_seq_static(ctx,0xC5,0x3A);
lcm_dcs_write_seq_static(ctx,0x00,0x00);
lcm_dcs_write_seq_static(ctx,0xFF,0xFF,0xFF,0xFF);
lcm_dcs_write_seq_static(ctx, 0x11,0);
msleep(120);
lcm_dcs_write_seq_static(ctx, 0x29,0);
msleep(30);
lcm_dcs_write_seq_static(ctx, 0x35,0x00);
}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
		
	if (!ctx->enabled)
		return 0;
	pr_info("%s\n", __func__);
	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}
	pr_info("%s_end\n", __func__);
	ctx->enabled = false;

	return 0;
}

static int lcm_unprepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	pr_info("%s\n", __func__);
	if (!ctx->prepared)
		return 0;

	lcm_dcs_write_seq_static(ctx, 0x28);
	msleep(50);
	lcm_dcs_write_seq_static(ctx, 0x10);
	msleep(150);

	ctx->error = 0;
	ctx->prepared = false;
//lcm huangjiwu  for tp   20200821 --begin
#if !defined(CONFIG_PRIZE_LCM_POWEROFF_AFTER_TP)
//lcm huangjiwu  for tp   20200821 --end
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
//lcm huangjiwu  for tp   20200821 --begin
	msleep(50);
#endif
//lcm huangjiwu  for tp   20200821 --end
#if defined(CONFIG_PRIZE_LCD_BIAS)
//lcm huangjiwu  for tp   20200821 --begin
#if !defined(CONFIG_PRIZE_LCM_POWEROFF_AFTER_TP)
	display_bias_disable();
	msleep(20);
#endif
//lcm huangjiwu  for tp   20200821 --end
#else
	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(ctx->dev, "%s: cannot get bias_neg %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	gpiod_set_value(ctx->bias_neg, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);

	udelay(1000);

	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(ctx->dev, "%s: cannot get bias_pos %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	gpiod_set_value(ctx->bias_pos, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);
#endif
	msleep(20);

	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	pr_info("%s\n", __func__);
	if (ctx->prepared)
		return 0;

#if defined(CONFIG_PRIZE_LCD_BIAS)
	msleep(10);
	display_bias_enable();
#else
	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(ctx->dev, "%s: cannot get bias_pos %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	gpiod_set_value(ctx->bias_pos, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);

	udelay(2000);

	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(ctx->dev, "%s: cannot get bias_neg %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	gpiod_set_value(ctx->bias_neg, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);
#endif

	msleep(10);
	msleep(10);
	lcm_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_rst(panel);
#endif
#ifdef PANEL_SUPPORT_READBACK
	lcm_panel_get_data(ctx);
#endif

	return ret;
}

static int lcm_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	pr_info("%s\n", __func__);
	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}
	pr_info("%s_end\n", __func__);
	ctx->enabled = true;

	return 0;
}

#define HFP (16)
#define HSA (6)
#define HBP (16)
#define VFP (24)     //8  prize--huangjiwu --shanping
#define VSA (4)
#define VBP (32)
#define VAC (2400)//1920
#define HAC (1080)//1080
static const struct drm_display_mode default_mode = {
	.clock = 163944,     //htotal*vtotal*vrefresh/1000   163943   182495
	.hdisplay = HAC,
	.hsync_start = HAC + HFP,
	.hsync_end = HAC + HFP + HSA,
	.htotal = HAC + HFP + HSA + HBP,
	.vdisplay = VAC,
	.vsync_start = VAC + VFP,
	.vsync_end = VAC + VFP + VSA,
	.vtotal = VAC + VFP + VSA + VBP,
	.vrefresh = 60,
};

#if defined(CONFIG_MTK_PANEL_EXT)
//lcm huangjiwu  for tp   20200821 --begin
#if defined(CONFIG_PRIZE_LCM_POWEROFF_AFTER_TP)
static int panel_poweroff_ext(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
		ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	msleep(50);
	pr_info("%s_reset_gpio\n", __func__);
	display_bias_disable();
	msleep(20);
	return 0;
}
#endif
//lcm huangjiwu  for tp   20200821 --end
static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}

static int panel_ata_check(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	unsigned char data[3] = {0x00, 0x00, 0x00};
	unsigned char id[3] = {0x00, 0x00, 0x00};
	ssize_t ret;

	ret = mipi_dsi_dcs_read(dsi, 0x4, data, 3);
	if (ret < 0) {
		pr_err("%s error\n", __func__);
		return 0;
	}

	DDPINFO("ATA read data %x %x %x\n", data[0], data[1], data[2]);

	if (data[0] == id[0] &&
			data[1] == id[1] &&
			data[2] == id[2])
		return 1;

	DDPINFO("ATA expect read data is %x %x %x\n",
			id[0], id[1], id[2]);

	return 0;
}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb,
	void *handle, unsigned int level)
{
	char bl_tb0[] = {0x51, 0xFF};

	bl_tb0[1] = level;

	if (!cb)
		return -1;

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	return 0;
}

static struct mtk_panel_params ext_params = {
	.pll_clk = 510,
	.vfp_low_power = 450,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0a,
		.count = 1,
		.para_list[0] = 0x9C,
	},
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 90,
	},
};

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.ata_check = panel_ata_check,
	//lcm huangjiwu  for tp   20200821 --begin
	#if defined(CONFIG_PRIZE_LCM_POWEROFF_AFTER_TP)
	.poweroff_ext = panel_poweroff_ext,
	#endif
	//lcm huangjiwu  for tp   20200821 --end
};
#endif

struct panel_desc {
	const struct drm_display_mode *modes;
	unsigned int num_modes;

	unsigned int bpc;

	struct {
		unsigned int width;
		unsigned int height;
	} size;

	struct {
		unsigned int prepare;
		unsigned int enable;
		unsigned int disable;
		unsigned int unprepare;
	} delay;
};

static int lcm_get_modes(struct drm_panel *panel)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(panel->drm, &default_mode);
	if (!mode) {
		dev_err(panel->drm->dev, "failed to add mode %ux%ux@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			default_mode.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(panel->connector, mode);

	panel->connector->display_info.width_mm = 70;
	panel->connector->display_info.height_mm = 154;//sqrt((size*25.4)^2/(1600^2+720^2))*1600

	return 1;
}

static const struct drm_panel_funcs lcm_drm_funcs = {
	.disable = lcm_disable,
	.unprepare = lcm_unprepare,
	.prepare = lcm_prepare,
	.enable = lcm_enable,
	.get_modes = lcm_get_modes,
};

static int lcm_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct lcm *ctx;
	struct device_node *backlight;
	int ret;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;

	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);
		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				pr_info("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			pr_info("device node name:%s\n", remote_node->name);
		}
	}
	if (remote_node != dev->of_node) {
		pr_info("%s+ skip probe due to not current lcm\n", __func__);
		return -ENODEV;
	}

	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO
			 | MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET
			 | MIPI_DSI_CLOCK_NON_CONTINUOUS;

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(dev, "%s: cannot get reset-gpios %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);

	ctx->bias_pos = devm_gpiod_get_index(dev, "bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(dev, "%s: cannot get bias-pos 0 %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	devm_gpiod_put(dev, ctx->bias_pos);

	ctx->bias_neg = devm_gpiod_get_index(dev, "bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(dev, "%s: cannot get bias-neg 1 %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	devm_gpiod_put(dev, ctx->bias_neg);

	ctx->prepared = true;
	ctx->enabled = true;

	drm_panel_init(&ctx->panel);
	ctx->panel.dev = dev;
	ctx->panel.funcs = &lcm_drm_funcs;

	ret = drm_panel_add(&ctx->panel);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;
#endif

	pr_info("%s-\n", __func__);
//prize added by huangjiwu, lcm support, 20200918-start
#if defined(CONFIG_PRIZE_HARDWARE_INFO)
	{
	strcpy(current_lcm_info.chip,"ft8756");
	strcpy(current_lcm_info.vendor,"unknow");
    sprintf(current_lcm_info.id,"0x%04x",0x40);
    sprintf(current_lcm_info.more,"%d*%d",VAC,HAC);
	}
#endif
//prize added by huangjiwu, lcm support, 20200918-end
	return ret;
}

static int lcm_remove(struct mipi_dsi_device *dsi)
{
	struct lcm *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

	return 0;
}

static const struct of_device_id lcm_of_match[] = {
	{ .compatible = "csot,ft8756,vdo", },
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "panel-csot-ft8756-vdo",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("Yi-Lun Wang <Yi-Lun.Wang@mediatek.com>");
MODULE_DESCRIPTION("truly td4330 CMD LCD Panel Driver");
MODULE_LICENSE("GPL v2");
