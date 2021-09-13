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

// #include "prize_lcd_bias.h"

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	// struct gpio_desc *bias_pos, *bias_neg;
    struct gpio_desc *fd_en;

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
	gpiod_set_value(ctx->reset_gpio, 0);
	udelay(15 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	udelay(1 * 1000);
	gpiod_set_value(ctx->reset_gpio, 0);
	udelay(5 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	msleep(100);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

lcm_dcs_write_seq_static(ctx,0x35,0x00);
lcm_dcs_write_seq_static(ctx,0x53,0x20);
lcm_dcs_write_seq_static(ctx,0x51,0x00,0x00);
lcm_dcs_write_seq_static(ctx,0x6F,0x02);
lcm_dcs_write_seq_static(ctx,0xF0,0xAA,0x10);
lcm_dcs_write_seq_static(ctx,0xB0,0x02,0x1C,0x87,0x00,0x87);
lcm_dcs_write_seq_static(ctx,0xB1,0x01,0x50,0x00,0x0C,0x00,0x08,0x00);
lcm_dcs_write_seq_static(ctx,0xB2,0x01,0x50,0x00,0x0C,0x00,0x08,0x01);
lcm_dcs_write_seq_static(ctx,0xB6,0x40);
lcm_dcs_write_seq_static(ctx,0xBD,0x01);
lcm_dcs_write_seq_static(ctx,0xC3,0x55);
lcm_dcs_write_seq_static(ctx,0xC5,0x00);
lcm_dcs_write_seq_static(ctx,0xCF,0x0B,0x0B,0x0B);
lcm_dcs_write_seq_static(ctx,0xD0,0x80,0x13,0x50,0x14,0x14,0x00,0x29,0x2D,0x14,0x33,0x00,0x00,0x2D,0x33,0x00,0x00,0x00);
lcm_dcs_write_seq_static(ctx,0xF0,0xAA,0x12);
lcm_dcs_write_seq_static(ctx,0xB5,0xA7,0xA7);
lcm_dcs_write_seq_static(ctx,0xB6,0x97,0x97);
lcm_dcs_write_seq_static(ctx,0xB7,0x30,0x30);
lcm_dcs_write_seq_static(ctx,0xB8,0x20,0x20);
lcm_dcs_write_seq_static(ctx,0xB9,0x36,0x37,0x37);
lcm_dcs_write_seq_static(ctx,0xBA,0x38,0x20);
lcm_dcs_write_seq_static(ctx,0xD2,0x0F,0x00,0x00,0x02,0x04,0x02,0x04,0x06,0x00,0x00,0x02,0x02,0x00);
lcm_dcs_write_seq_static(ctx,0xF0,0xAA,0x11);
lcm_dcs_write_seq_static(ctx,0xC2,0x10,0x00,0x60,0x10,0x11,0x21,0x31,0x41,0x51,0x52);
lcm_dcs_write_seq_static(ctx,0xC3,0x00,0x10,0x00,0x40,0x00,0x80,0x00,0xC8,0x00,0xFF,0x01,0xFF);
lcm_dcs_write_seq_static(ctx,0xC4,0x08,0x84,0x08,0x30,0x06,0xF6,0x05,0x08,0x02,0x70,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x80,0x18,0x80,0x18);
lcm_dcs_write_seq_static(ctx,0xF0,0xAA,0x13);
lcm_dcs_write_seq_static(ctx,0xBF,0x00);
lcm_dcs_write_seq_static(ctx,0xB0,0x00,0x00,0x00,0x9C,0x00,0x9D,0x00,0xB9,0x00,0xCE,0x00,0xF0,0x01,0x08,0x01,0x1C);
lcm_dcs_write_seq_static(ctx,0xB1,0x01,0x2F,0x01,0x3D,0x01,0x4B,0x01,0x59,0x01,0x65,0x01,0x7B,0x01,0x91,0x01,0xA4);
lcm_dcs_write_seq_static(ctx,0xB2,0x01,0xB5,0x01,0xC5,0x01,0xE5,0x02,0x02,0x02,0x1E,0x02,0x56,0x02,0x8A,0x02,0xC0);
lcm_dcs_write_seq_static(ctx,0xB3,0x02,0xF7);
lcm_dcs_write_seq_static(ctx,0xB4,0x00,0x00,0x00,0x9A,0x00,0x9B,0x00,0xAB,0x00,0xBA,0x00,0xD6,0x00,0xEE,0x01,0x02);
lcm_dcs_write_seq_static(ctx,0xB5,0x01,0x14,0x01,0x24,0x01,0x32,0x01,0x3F,0x01,0x4B,0x01,0x62,0x01,0x78,0x01,0x8C);
lcm_dcs_write_seq_static(ctx,0xB6,0x01,0x9D,0x01,0xAD,0x01,0xCA,0x01,0xE7,0x02,0x01,0x02,0x37,0x02,0x68,0x02,0x9C);
lcm_dcs_write_seq_static(ctx,0xB7,0x02,0xCE);
lcm_dcs_write_seq_static(ctx,0xB8,0x00,0x00,0x00,0xD9,0x00,0xDA,0x00,0xF5,0x01,0x09,0x01,0x22,0x01,0x34,0x01,0x44);
lcm_dcs_write_seq_static(ctx,0xB9,0x01,0x57,0x01,0x67,0x01,0x75,0x01,0x82,0x01,0x8F,0x01,0xA6,0x01,0xBC,0x01,0xCF);
lcm_dcs_write_seq_static(ctx,0xBA,0x01,0xE1,0x01,0xF1,0x02,0x0F,0x02,0x2E,0x02,0x48,0x02,0x80,0x02,0xB1,0x02,0xE8);
lcm_dcs_write_seq_static(ctx,0xBB,0x03,0x1D);
lcm_dcs_write_seq_static(ctx,0xBD,0x2A,0x00,0x15,0x00,0x00,0x2A,0x00,0x15);
lcm_dcs_write_seq_static(ctx,0xD2,0x14,0x92,0x24);
lcm_dcs_write_seq_static(ctx,0xD4,0x00,0x40);
lcm_dcs_write_seq_static(ctx,0xD6,0x01);
lcm_dcs_write_seq_static(ctx,0xC3,0x00,0x31,0x42,0x56,0x32,0x51,0x46);
lcm_dcs_write_seq_static(ctx,0xF0,0xAA,0x14);
lcm_dcs_write_seq_static(ctx,0xB2,0x03,0x33);
lcm_dcs_write_seq_static(ctx,0xB4,0x0A,0x88,0x28,0x14);
lcm_dcs_write_seq_static(ctx,0xB5,0x00,0x00,0x00,0x02,0x00);
lcm_dcs_write_seq_static(ctx,0xB9,0x00,0x00,0x05,0x00);
lcm_dcs_write_seq_static(ctx,0xBC,0x20,0x00,0x00,0x03,0x11,0x20,0x8E);
lcm_dcs_write_seq_static(ctx,0xBE,0x20,0x10,0x00,0x04,0x22,0x00,0x67);
lcm_dcs_write_seq_static(ctx,0xF0,0xAA,0x15);
lcm_dcs_write_seq_static(ctx,0xB2,0x53);
lcm_dcs_write_seq_static(ctx,0xB3,0x84,0x23,0x30);
lcm_dcs_write_seq_static(ctx,0xB4,0x83,0x04);
lcm_dcs_write_seq_static(ctx,0xB5,0x83,0x04);
lcm_dcs_write_seq_static(ctx,0xB6,0x83,0x00,0x04);
lcm_dcs_write_seq_static(ctx,0xB7,0x83,0x00,0x04);
lcm_dcs_write_seq_static(ctx,0xBC,0x62);
lcm_dcs_write_seq_static(ctx,0xBD,0x48);
lcm_dcs_write_seq_static(ctx,0xBE,0x11,0x00,0x20,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x01);
lcm_dcs_write_seq_static(ctx,0xBF,0x01);
lcm_dcs_write_seq_static(ctx,0xC1,0x02);
lcm_dcs_write_seq_static(ctx,0xC3,0x01);
lcm_dcs_write_seq_static(ctx,0xF0,0xAA,0x16);
lcm_dcs_write_seq_static(ctx,0xB0,0x25,0x25,0x25,0x25);
lcm_dcs_write_seq_static(ctx,0xB1,0x00,0x25,0x0A,0x0B);
lcm_dcs_write_seq_static(ctx,0xB2,0x25,0x04,0x12,0x25);
lcm_dcs_write_seq_static(ctx,0xB3,0x13,0x25,0x25,0x25);
lcm_dcs_write_seq_static(ctx,0xB4,0x25,0x25,0x25,0x25);
lcm_dcs_write_seq_static(ctx,0xB5,0x25,0x25,0x25,0x25);
lcm_dcs_write_seq_static(ctx,0xB6,0x25,0x25,0x25,0x13);
lcm_dcs_write_seq_static(ctx,0xB7,0x25,0x12,0x25,0x25);
lcm_dcs_write_seq_static(ctx,0xB8,0x0B,0x0A,0x25,0x25);
lcm_dcs_write_seq_static(ctx,0xB9,0x25,0x25,0x25,0x25);
lcm_dcs_write_seq_static(ctx,0xF0,0xAA,0x17);
lcm_dcs_write_seq_static(ctx,0xB0,0x04,0x10,0x00,0x09,0x00,0x00,0x00,0x00,0x0f,0xdc,0xc0,0x0d,0x00);
lcm_dcs_write_seq_static(ctx,0xB1,0x0c,0xe0,0xff,0x6f,0xe8,0x00,0x20,0x20,0x00,0x20,0x20,0x00,0x20,0x20,0x00,0x20);
lcm_dcs_write_seq_static(ctx,0xB2,0x00,0x00,0x04,0x00,0x80,0xc0,0x80,0x00,0x14,0x14,0x80,0x14,0x14,0x00,0x14,0x94);
lcm_dcs_write_seq_static(ctx,0xB3,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x80,0xc0,0x40,0x0d,0x40,0xa0,0xc0);
lcm_dcs_write_seq_static(ctx,0xB4,0x67,0x67,0x67,0x67,0x67,0x67,0x3f);
lcm_dcs_write_seq_static(ctx,0xE5,0x11);
lcm_dcs_write_seq_static(ctx,0xB5,0x31,0x42,0x56,0x12,0x53,0x64);
    
    lcm_dcs_write_seq_static(ctx, 0x11);
    msleep(120);
	lcm_dcs_write_seq_static(ctx, 0x29);
	msleep(120);
}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (!ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = false;

	return 0;
}

static int lcm_unprepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (!ctx->prepared)
		return 0;

	lcm_dcs_write_seq_static(ctx, 0x28);
	msleep(50);
	lcm_dcs_write_seq_static(ctx, 0x10);
	msleep(150);

	ctx->error = 0;
	ctx->prepared = false;

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

#if defined(CONFIG_PRIZE_LCD_BIAS)
	//display_bias_disable();
	#if 0
	ctx->fd_en =
		devm_gpiod_get(ctx->dev, "fd-en", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->fd_en)) {
		dev_err(ctx->dev, "%s: cannot get fd_en %ld\n",
			__func__, PTR_ERR(ctx->fd_en));
		return PTR_ERR(ctx->fd_en);;
	}
	gpiod_set_value(ctx->fd_en, 0);
	msleep(15);
	devm_gpiod_put(ctx->dev, ctx->fd_en);
	#endif
#else
	#if 0
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
#endif

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
	//display_bias_enable();
	#if 0
	ctx->fd_en =
		devm_gpiod_get(ctx->dev, "fd-en", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->fd_en)) {
		dev_err(ctx->dev, "%s: cannot get fd_en %ld\n",
			__func__, PTR_ERR(ctx->fd_en));
		return PTR_ERR(ctx->fd_en);
	}
	gpiod_set_value(ctx->fd_en, 1);
	msleep(15);
	devm_gpiod_put(ctx->dev, ctx->fd_en);
	#endif
#else
	#if 0
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
#endif

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

	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = true;

	return 0;
}

#define HFP (26)
#define HSA (4)
#define HBP (36)
#define VFP (8)
#define VSA (4)
#define VBP (12)
#define VAC (2160)//1920
#define HAC (1080)//1080
static const struct drm_display_mode default_mode = {
	.clock = 163406,
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
	char bl_tb0[] = {0x51, 0x00, 0x00};

	bl_tb0[2] = level;

	if (!cb)
		return -1;

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	return 0;
}

static struct mtk_panel_params ext_params = {
	.pll_clk = 450,
	.vfp_low_power = 750,
	.cust_esd_check = 0,
	.esd_check_enable = 0,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0a,
		.count = 1,
		.para_list[0] = 0x1c,
	},
	.physical_width_um = 68269,
	.physical_height_um = 136538,
};

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.ata_check = panel_ata_check,
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

	panel->connector->display_info.width_mm = 68;
	panel->connector->display_info.height_mm = 137;

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

	// ctx->bias_pos = devm_gpiod_get_index(dev, "bias", 0, GPIOD_OUT_HIGH);
	// if (IS_ERR(ctx->bias_pos)) {
		// dev_err(dev, "%s: cannot get bias-pos 0 %ld\n",
			// __func__, PTR_ERR(ctx->bias_pos));
		// return PTR_ERR(ctx->bias_pos);
	// }
	// devm_gpiod_put(dev, ctx->bias_pos);

	// ctx->bias_neg = devm_gpiod_get_index(dev, "bias", 1, GPIOD_OUT_HIGH);
	// if (IS_ERR(ctx->bias_neg)) {
		// dev_err(dev, "%s: cannot get bias-neg 1 %ld\n",
			// __func__, PTR_ERR(ctx->bias_neg));
		// return PTR_ERR(ctx->bias_neg);
	// }
	// devm_gpiod_put(dev, ctx->bias_neg);

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
	{ .compatible = "rrj,vtdr6111,vdo", },
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "panel-rrj-vtdr6111-vdo",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("Yi-Lun Wang <Yi-Lun.Wang@mediatek.com>");
MODULE_DESCRIPTION("truly td4330 CMD LCD Panel Driver");
MODULE_LICENSE("GPL v2");
