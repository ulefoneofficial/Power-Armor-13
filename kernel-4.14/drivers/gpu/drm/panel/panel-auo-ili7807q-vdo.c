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
	lcm_dcs_write_seq_static(ctx,0XFF,0X78,0X07,0X01);
	lcm_dcs_write_seq_static(ctx,0X00,0X63);
	lcm_dcs_write_seq_static(ctx,0X01,0XE2);
	lcm_dcs_write_seq_static(ctx,0X02,0X34);
	lcm_dcs_write_seq_static(ctx,0X03,0X1f);
	lcm_dcs_write_seq_static(ctx,0X04,0X23);
	lcm_dcs_write_seq_static(ctx,0X05,0X42);
	lcm_dcs_write_seq_static(ctx,0X06,0X00);
	lcm_dcs_write_seq_static(ctx,0X07,0X00);
	lcm_dcs_write_seq_static(ctx,0X08,0XA3);
	lcm_dcs_write_seq_static(ctx,0X09,0X04);
	lcm_dcs_write_seq_static(ctx,0X0A,0X30);
	lcm_dcs_write_seq_static(ctx,0X0B,0X01);
	lcm_dcs_write_seq_static(ctx,0X0C,0X00);
	lcm_dcs_write_seq_static(ctx,0X0E,0X5A);
	lcm_dcs_write_seq_static(ctx,0X31,0X07);
	lcm_dcs_write_seq_static(ctx,0X32,0X07);    
	lcm_dcs_write_seq_static(ctx,0X33,0X07);    
	lcm_dcs_write_seq_static(ctx,0X34,0X3D);    
	lcm_dcs_write_seq_static(ctx,0X35,0X3D);    
	lcm_dcs_write_seq_static(ctx,0X36,0X02); 		
	lcm_dcs_write_seq_static(ctx,0X37,0X2E);    
	lcm_dcs_write_seq_static(ctx,0X38,0X2F);    
	lcm_dcs_write_seq_static(ctx,0X39,0X30);    
	lcm_dcs_write_seq_static(ctx,0X3A,0X31);    
	lcm_dcs_write_seq_static(ctx,0X3B,0X32);    
	lcm_dcs_write_seq_static(ctx,0X3C,0X33);    
	lcm_dcs_write_seq_static(ctx,0X3D,0X01);    
	lcm_dcs_write_seq_static(ctx,0X3E,0X00);    
	lcm_dcs_write_seq_static(ctx,0X3F,0X11);    
	lcm_dcs_write_seq_static(ctx,0X40,0X11);    
	lcm_dcs_write_seq_static(ctx,0X41,0X13);    
	lcm_dcs_write_seq_static(ctx,0X42,0X13);    
	lcm_dcs_write_seq_static(ctx,0X43,0X40);    
	lcm_dcs_write_seq_static(ctx,0X44,0X08);    
	lcm_dcs_write_seq_static(ctx,0X45,0X0C);    
	lcm_dcs_write_seq_static(ctx,0X46,0X2C);    
	lcm_dcs_write_seq_static(ctx,0X47,0X28);    
	lcm_dcs_write_seq_static(ctx,0X48,0X28);    
	lcm_dcs_write_seq_static(ctx,0X49,0X07); 		
	lcm_dcs_write_seq_static(ctx,0X4A,0X07);    
	lcm_dcs_write_seq_static(ctx,0X4B,0X07);    
	lcm_dcs_write_seq_static(ctx,0X4C,0X3D);   	
	lcm_dcs_write_seq_static(ctx,0X4D,0X3D);   	
	lcm_dcs_write_seq_static(ctx,0X4E,0X02);  	
	lcm_dcs_write_seq_static(ctx,0X4F,0X2E);    
	lcm_dcs_write_seq_static(ctx,0X50,0X2F);    
	lcm_dcs_write_seq_static(ctx,0X51,0X30);    
	lcm_dcs_write_seq_static(ctx,0X52,0X31);    
	lcm_dcs_write_seq_static(ctx,0X53,0X32);    
	lcm_dcs_write_seq_static(ctx,0X54,0X33);    
	lcm_dcs_write_seq_static(ctx,0X55,0X01);    
	lcm_dcs_write_seq_static(ctx,0X56,0X00);    
	lcm_dcs_write_seq_static(ctx,0X57,0X10);    
	lcm_dcs_write_seq_static(ctx,0X58,0X10);    
	lcm_dcs_write_seq_static(ctx,0X59,0X12);    
	lcm_dcs_write_seq_static(ctx,0X5A,0X12);    
	lcm_dcs_write_seq_static(ctx,0X5B,0X40);   	
	lcm_dcs_write_seq_static(ctx,0X5C,0X08);    
	lcm_dcs_write_seq_static(ctx,0X5D,0X0C);   	
	lcm_dcs_write_seq_static(ctx,0X5E,0X2C);    
	lcm_dcs_write_seq_static(ctx,0X5F,0X28);    
	lcm_dcs_write_seq_static(ctx,0X60,0X28);    
	lcm_dcs_write_seq_static(ctx,0X61,0X07); 		
	lcm_dcs_write_seq_static(ctx,0X62,0X07);    
	lcm_dcs_write_seq_static(ctx,0X63,0X07);    
	lcm_dcs_write_seq_static(ctx,0X64,0X3D);    
	lcm_dcs_write_seq_static(ctx,0X65,0X3D);    
	lcm_dcs_write_seq_static(ctx,0X66,0X28);    
	lcm_dcs_write_seq_static(ctx,0X67,0X2E);    
	lcm_dcs_write_seq_static(ctx,0X68,0X2F);    
	lcm_dcs_write_seq_static(ctx,0X69,0X30);    
	lcm_dcs_write_seq_static(ctx,0X6A,0X31);    
	lcm_dcs_write_seq_static(ctx,0X6B,0X32);    
	lcm_dcs_write_seq_static(ctx,0X6C,0X33);    
	lcm_dcs_write_seq_static(ctx,0X6D,0X01);    
	lcm_dcs_write_seq_static(ctx,0X6E,0X00);    
	lcm_dcs_write_seq_static(ctx,0X6F,0X12);    
	lcm_dcs_write_seq_static(ctx,0X70,0X12);    
	lcm_dcs_write_seq_static(ctx,0X71,0X10);    
	lcm_dcs_write_seq_static(ctx,0X72,0X10);    
	lcm_dcs_write_seq_static(ctx,0X73,0X3C);    
	lcm_dcs_write_seq_static(ctx,0X74,0X0C);    
	lcm_dcs_write_seq_static(ctx,0X75,0X08);    
	lcm_dcs_write_seq_static(ctx,0X76,0X2C);    
	lcm_dcs_write_seq_static(ctx,0X77,0X28);    
	lcm_dcs_write_seq_static(ctx,0X78,0X28);    
	lcm_dcs_write_seq_static(ctx,0X79,0X07);    
	lcm_dcs_write_seq_static(ctx,0X7A,0X07);    
	lcm_dcs_write_seq_static(ctx,0X7B,0X07);    
	lcm_dcs_write_seq_static(ctx,0X7C,0X3D);   	
	lcm_dcs_write_seq_static(ctx,0X7D,0X3D);   	
	lcm_dcs_write_seq_static(ctx,0X7E,0X28);    
	lcm_dcs_write_seq_static(ctx,0X7F,0X2E);    
	lcm_dcs_write_seq_static(ctx,0X80,0X2F);    
	lcm_dcs_write_seq_static(ctx,0X81,0X30);    
	lcm_dcs_write_seq_static(ctx,0X82,0X31);    
	lcm_dcs_write_seq_static(ctx,0X83,0X32);    
	lcm_dcs_write_seq_static(ctx,0X84,0X33);    
	lcm_dcs_write_seq_static(ctx,0X85,0X01);    
	lcm_dcs_write_seq_static(ctx,0X86,0X00);    
	lcm_dcs_write_seq_static(ctx,0X87,0X13);    
	lcm_dcs_write_seq_static(ctx,0X88,0X13);    
	lcm_dcs_write_seq_static(ctx,0X89,0X11);    
	lcm_dcs_write_seq_static(ctx,0X8A,0X11);    
	lcm_dcs_write_seq_static(ctx,0X8B,0X3C);   	
	lcm_dcs_write_seq_static(ctx,0X8C,0X0C);   	
	lcm_dcs_write_seq_static(ctx,0X8D,0X08);    
	lcm_dcs_write_seq_static(ctx,0X8E,0X2C);    
	lcm_dcs_write_seq_static(ctx,0X8F,0X28);    
	lcm_dcs_write_seq_static(ctx,0X90,0X28);    
	lcm_dcs_write_seq_static(ctx,0X91,0XE1);
	lcm_dcs_write_seq_static(ctx,0X92,0X19);
	lcm_dcs_write_seq_static(ctx,0X93,0X08);
	lcm_dcs_write_seq_static(ctx,0X94,0X00);
	lcm_dcs_write_seq_static(ctx,0X95,0X21);
	lcm_dcs_write_seq_static(ctx,0X96,0X19);
	lcm_dcs_write_seq_static(ctx,0X97,0X08);
	lcm_dcs_write_seq_static(ctx,0X98,0X00);
	lcm_dcs_write_seq_static(ctx,0XA0,0X83);
	lcm_dcs_write_seq_static(ctx,0XA1,0X44);
	lcm_dcs_write_seq_static(ctx,0XA2,0X83);
	lcm_dcs_write_seq_static(ctx,0XA3,0X44);
	lcm_dcs_write_seq_static(ctx,0XA4,0X61);
	lcm_dcs_write_seq_static(ctx,0XA5,0X00);  		
	lcm_dcs_write_seq_static(ctx,0XA6,0X15);
	lcm_dcs_write_seq_static(ctx,0XA7,0X50);
	lcm_dcs_write_seq_static(ctx,0XA8,0X1A);
	lcm_dcs_write_seq_static(ctx,0XAE,0X00);
	lcm_dcs_write_seq_static(ctx,0XB0,0X00);
	lcm_dcs_write_seq_static(ctx,0XB1,0X00);
	lcm_dcs_write_seq_static(ctx,0XB2,0X02);
	lcm_dcs_write_seq_static(ctx,0XB3,0X00);
	lcm_dcs_write_seq_static(ctx,0XB4,0X02);
	lcm_dcs_write_seq_static(ctx,0XC1,0X60);
	lcm_dcs_write_seq_static(ctx,0XC2,0X60);
	lcm_dcs_write_seq_static(ctx,0XC5,0X29);
	lcm_dcs_write_seq_static(ctx,0XC6,0X20);
	lcm_dcs_write_seq_static(ctx,0XC7,0X20);
	lcm_dcs_write_seq_static(ctx,0XC8,0X1F);
	lcm_dcs_write_seq_static(ctx,0XC9,0X00); 		
	lcm_dcs_write_seq_static(ctx,0XCA,0X01);
	lcm_dcs_write_seq_static(ctx,0XD1,0X11);
	lcm_dcs_write_seq_static(ctx,0XD2,0X00);
	lcm_dcs_write_seq_static(ctx,0XD3,0X01);
	lcm_dcs_write_seq_static(ctx,0XD4,0X00);
	lcm_dcs_write_seq_static(ctx,0XD5,0X00);
	lcm_dcs_write_seq_static(ctx,0XD6,0X3D);
	lcm_dcs_write_seq_static(ctx,0XD7,0X00);
	lcm_dcs_write_seq_static(ctx,0XD8,0X01);
	lcm_dcs_write_seq_static(ctx,0XD9,0X54);
	lcm_dcs_write_seq_static(ctx,0XDA,0X00);
	lcm_dcs_write_seq_static(ctx,0XDB,0X00);
	lcm_dcs_write_seq_static(ctx,0XDC,0X00);
	lcm_dcs_write_seq_static(ctx,0XDD,0X00);
	lcm_dcs_write_seq_static(ctx,0XDE,0X00);
	lcm_dcs_write_seq_static(ctx,0XDF,0X00);
	lcm_dcs_write_seq_static(ctx,0XE0,0X00);
	lcm_dcs_write_seq_static(ctx,0XE1,0X04);		
	lcm_dcs_write_seq_static(ctx,0XE2,0X00);
	lcm_dcs_write_seq_static(ctx,0XE3,0X1B);
	lcm_dcs_write_seq_static(ctx,0XE4,0X52);
	lcm_dcs_write_seq_static(ctx,0XE5,0X4B);      
	lcm_dcs_write_seq_static(ctx,0XE6,0X44); 		
	lcm_dcs_write_seq_static(ctx,0XE7,0X00);
	lcm_dcs_write_seq_static(ctx,0XE8,0X01);
	lcm_dcs_write_seq_static(ctx,0XED,0X55);
	lcm_dcs_write_seq_static(ctx,0XEF,0X30);
	lcm_dcs_write_seq_static(ctx,0XF0,0X00);
	lcm_dcs_write_seq_static(ctx,0XF4,0X54);
	lcm_dcs_write_seq_static(ctx,0XFF,0X78,0X07,0X02);
	lcm_dcs_write_seq_static(ctx,0X01,0X7D); 		
	lcm_dcs_write_seq_static(ctx,0X02,0X08); 		
	lcm_dcs_write_seq_static(ctx,0X06,0X6B);		
	lcm_dcs_write_seq_static(ctx,0X08,0X00);		
	lcm_dcs_write_seq_static(ctx,0X0E,0X14);
	lcm_dcs_write_seq_static(ctx,0X0F,0X34);
	lcm_dcs_write_seq_static(ctx,0X40,0X0F);	
	lcm_dcs_write_seq_static(ctx,0X41,0X00);
	lcm_dcs_write_seq_static(ctx,0X42,0X09);
	lcm_dcs_write_seq_static(ctx,0X43,0X12);	
	lcm_dcs_write_seq_static(ctx,0X46,0X32);
	lcm_dcs_write_seq_static(ctx,0X4D,0X02);	
	lcm_dcs_write_seq_static(ctx,0X47,0X00);	
	lcm_dcs_write_seq_static(ctx,0X53,0X09);
	lcm_dcs_write_seq_static(ctx,0X54,0X05);
	lcm_dcs_write_seq_static(ctx,0X56,0X02);
	lcm_dcs_write_seq_static(ctx,0X5D,0X07);
	lcm_dcs_write_seq_static(ctx,0X5E,0XC0);
	lcm_dcs_write_seq_static(ctx,0X80,0X3F);   
	lcm_dcs_write_seq_static(ctx,0XF4,0X00);
	lcm_dcs_write_seq_static(ctx,0XF5,0X00);
	lcm_dcs_write_seq_static(ctx,0XF6,0X00);
	lcm_dcs_write_seq_static(ctx,0XF7,0X00);
	lcm_dcs_write_seq_static(ctx,0XFF,0X78,0X07,0X04);
	lcm_dcs_write_seq_static(ctx,0XB7,0X45); 		
	lcm_dcs_write_seq_static(ctx,0XFF,0X78,0X07,0X05);   
	lcm_dcs_write_seq_static(ctx,0X5B,0X7E);		
	lcm_dcs_write_seq_static(ctx,0X5C,0X7E);		
	lcm_dcs_write_seq_static(ctx,0X3C,0X00);		
	lcm_dcs_write_seq_static(ctx,0X52,0X60); 		
	lcm_dcs_write_seq_static(ctx,0X56,0X5B);		
	lcm_dcs_write_seq_static(ctx,0X5A,0X54);		
	lcm_dcs_write_seq_static(ctx,0X54,0X56); 		
	lcm_dcs_write_seq_static(ctx,0X02,0X00); 		
	lcm_dcs_write_seq_static(ctx,0X03,0X87);	  
	lcm_dcs_write_seq_static(ctx,0X44,0XEF);		
	lcm_dcs_write_seq_static(ctx,0X4E,0X3F);		
	lcm_dcs_write_seq_static(ctx,0X20,0X13); 		
	lcm_dcs_write_seq_static(ctx,0X2A,0X13);		
	lcm_dcs_write_seq_static(ctx,0X2B,0X08);	
	lcm_dcs_write_seq_static(ctx,0X23,0XF7); 		
	lcm_dcs_write_seq_static(ctx,0X2D,0X54);		
	lcm_dcs_write_seq_static(ctx,0X2E,0X74);		
	lcm_dcs_write_seq_static(ctx,0XB5,0X54);		
	lcm_dcs_write_seq_static(ctx,0XB7,0X74);		
	lcm_dcs_write_seq_static(ctx,0XFF,0X78,0X07,0X06);
	lcm_dcs_write_seq_static(ctx,0X0A,0X0C);		
	lcm_dcs_write_seq_static(ctx,0X0B,0XAF); 		
	lcm_dcs_write_seq_static(ctx,0X0E,0X06); 		
	lcm_dcs_write_seq_static(ctx,0X83,0X00);		
	lcm_dcs_write_seq_static(ctx,0X84,0X00);	
	lcm_dcs_write_seq_static(ctx,0XFF,0X78,0X07,0X08);
	lcm_dcs_write_seq_static(ctx,0XE0,0X00,0X00,0X1A,0X44,0X00,0X67,0X81,0X9A,0X00,0XB0,0XC3,0XD5,0X15,0X0E,0X3B,0X7E,0X25,0XAE,0XFA,0X35,0X2A,0X36,0X6E,0XAF,0X3E,0XD7,0X0A,0X30,0X3F,0X55,0X63,0X71,0X3F,0X7E,0X91,0XA2,0X3F,0XBD,0XD8,0XD9);
	lcm_dcs_write_seq_static(ctx,0XE1,0X00,0X00,0X1A,0X44,0X00,0X67,0X81,0X9A,0X00,0XB0,0XC3,0XD5,0X15,0X0E,0X3B,0X7E,0X25,0XAE,0XFA,0X35,0X2A,0X36,0X6E,0XAF,0X3E,0XD7,0X0A,0X30,0X3F,0X55,0X63,0X71,0X3F,0X7E,0X91,0XA2,0X3F,0XBD,0XD8,0XD9);
	lcm_dcs_write_seq_static(ctx,0XFF,0X78,0X07,0X0B);
	lcm_dcs_write_seq_static(ctx,0X94,0X88);
	lcm_dcs_write_seq_static(ctx,0X95,0X22);
	lcm_dcs_write_seq_static(ctx,0X96,0X06);
	lcm_dcs_write_seq_static(ctx,0X97,0X06);
	lcm_dcs_write_seq_static(ctx,0X98,0XCB);
	lcm_dcs_write_seq_static(ctx,0X99,0XCB);
	lcm_dcs_write_seq_static(ctx,0X9A,0X06);
	lcm_dcs_write_seq_static(ctx,0X9B,0XCD);
	lcm_dcs_write_seq_static(ctx,0X9C,0X05);
	lcm_dcs_write_seq_static(ctx,0X9D,0X05);
	lcm_dcs_write_seq_static(ctx,0X9E,0XAA);
	lcm_dcs_write_seq_static(ctx,0X9F,0XAA);
	lcm_dcs_write_seq_static(ctx,0XAB,0XF0);
	lcm_dcs_write_seq_static(ctx,0XFF,0X78,0X07,0X0E);
	lcm_dcs_write_seq_static(ctx,0X00,0XA3); 	
	lcm_dcs_write_seq_static(ctx,0X02,0X12); 	
	lcm_dcs_write_seq_static(ctx,0X40,0X07); 	
	lcm_dcs_write_seq_static(ctx,0X49,0X2C); 	
	lcm_dcs_write_seq_static(ctx,0X47,0X90); 	
	lcm_dcs_write_seq_static(ctx,0X45,0X0A); 	
	lcm_dcs_write_seq_static(ctx,0X46,0XA1);
	lcm_dcs_write_seq_static(ctx,0X4D,0XC4);	
	lcm_dcs_write_seq_static(ctx,0X51,0X00);
	lcm_dcs_write_seq_static(ctx,0XB0,0X01);	
	lcm_dcs_write_seq_static(ctx,0XB1,0X60);
	lcm_dcs_write_seq_static(ctx,0XB3,0X00);
	lcm_dcs_write_seq_static(ctx,0XB4,0X33);
	lcm_dcs_write_seq_static(ctx,0XBC,0X04);
	lcm_dcs_write_seq_static(ctx,0XBD,0XFC);
	lcm_dcs_write_seq_static(ctx,0XC0,0X63); 
	lcm_dcs_write_seq_static(ctx,0XC7,0X61);
	lcm_dcs_write_seq_static(ctx,0XC8,0X61);
	lcm_dcs_write_seq_static(ctx,0XC9,0X61);
	lcm_dcs_write_seq_static(ctx,0XD2,0X00);
	lcm_dcs_write_seq_static(ctx,0XD3,0XA8);
	lcm_dcs_write_seq_static(ctx,0XD4,0X77);
	lcm_dcs_write_seq_static(ctx,0XD5,0XA8);
	lcm_dcs_write_seq_static(ctx,0XD6,0X00);
	lcm_dcs_write_seq_static(ctx,0XD7,0XA8);
	lcm_dcs_write_seq_static(ctx,0XD8,0X00);
	lcm_dcs_write_seq_static(ctx,0XD9,0XA8);
	lcm_dcs_write_seq_static(ctx,0XE0,0X00);
	lcm_dcs_write_seq_static(ctx,0XE1,0X00);
	lcm_dcs_write_seq_static(ctx,0XE2,0X09);
	lcm_dcs_write_seq_static(ctx,0XE3,0X17);
	lcm_dcs_write_seq_static(ctx,0XE4,0X04);
	lcm_dcs_write_seq_static(ctx,0XE5,0X04);
	lcm_dcs_write_seq_static(ctx,0XE6,0X00);
	lcm_dcs_write_seq_static(ctx,0XE7,0X05);
	lcm_dcs_write_seq_static(ctx,0XE8,0X00);
	lcm_dcs_write_seq_static(ctx,0XE9,0X02);
	lcm_dcs_write_seq_static(ctx,0XEA,0X07);
	lcm_dcs_write_seq_static(ctx,0X07,0X21);  
	lcm_dcs_write_seq_static(ctx,0X4B,0X14);
	lcm_dcs_write_seq_static(ctx,0XFF,0X78,0X07,0X0C);
	lcm_dcs_write_seq_static(ctx,0X00,0X12);
	lcm_dcs_write_seq_static(ctx,0X01,0X26);
	lcm_dcs_write_seq_static(ctx,0X02,0X11);
	lcm_dcs_write_seq_static(ctx,0X03,0X1B);
	lcm_dcs_write_seq_static(ctx,0X04,0X11);
	lcm_dcs_write_seq_static(ctx,0X05,0X18);
	lcm_dcs_write_seq_static(ctx,0X06,0X11);
	lcm_dcs_write_seq_static(ctx,0X07,0X18);
	lcm_dcs_write_seq_static(ctx,0X08,0X12);
	lcm_dcs_write_seq_static(ctx,0X09,0X27);
	lcm_dcs_write_seq_static(ctx,0X0A,0X11);
	lcm_dcs_write_seq_static(ctx,0X0B,0X14);
	lcm_dcs_write_seq_static(ctx,0X0C,0X11);
	lcm_dcs_write_seq_static(ctx,0X0D,0X17);
	lcm_dcs_write_seq_static(ctx,0X0E,0X12);
	lcm_dcs_write_seq_static(ctx,0X0F,0X24);
	lcm_dcs_write_seq_static(ctx,0X10,0X12);
	lcm_dcs_write_seq_static(ctx,0X11,0X28);
	lcm_dcs_write_seq_static(ctx,0X12,0X12);
	lcm_dcs_write_seq_static(ctx,0X13,0X23);
	lcm_dcs_write_seq_static(ctx,0X14,0X11);
	lcm_dcs_write_seq_static(ctx,0X15,0X1E);
	lcm_dcs_write_seq_static(ctx,0X16,0X11);
	lcm_dcs_write_seq_static(ctx,0X17,0X1C);
	lcm_dcs_write_seq_static(ctx,0X18,0X11);
	lcm_dcs_write_seq_static(ctx,0X19,0X20);
	lcm_dcs_write_seq_static(ctx,0X1A,0X11);
	lcm_dcs_write_seq_static(ctx,0X1B,0X16);
	lcm_dcs_write_seq_static(ctx,0X1C,0X11);
	lcm_dcs_write_seq_static(ctx,0X1D,0X15);
	lcm_dcs_write_seq_static(ctx,0X1E,0X11);
	lcm_dcs_write_seq_static(ctx,0X1F,0X19);
	lcm_dcs_write_seq_static(ctx,0X20,0X12);
	lcm_dcs_write_seq_static(ctx,0X21,0X21);
	lcm_dcs_write_seq_static(ctx,0X22,0X11);
	lcm_dcs_write_seq_static(ctx,0X23,0X1F);
	lcm_dcs_write_seq_static(ctx,0X24,0X11);
	lcm_dcs_write_seq_static(ctx,0X25,0X1A);
	lcm_dcs_write_seq_static(ctx,0X26,0X11);
	lcm_dcs_write_seq_static(ctx,0X27,0X1D);
	lcm_dcs_write_seq_static(ctx,0X28,0X12);
	lcm_dcs_write_seq_static(ctx,0X29,0X25);
	lcm_dcs_write_seq_static(ctx,0XFF,0X78,0X07,0X00);

	lcm_dcs_write_seq_static(ctx, 0x11);
	msleep(120);
	lcm_dcs_write_seq_static(ctx, 0x29);
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
	pr_info("%s_reset_gpio\n", __func__);
#endif
//lcm huangjiwu  for tp   20200821 --end
#if defined(CONFIG_PRIZE_LCD_BIAS)
//lcm huangjiwu  for tp   20200821 --begin
#if !defined(CONFIG_PRIZE_LCM_POWEROFF_AFTER_TP)
	display_bias_disable();
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
	
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
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
	
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, 
		"reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	udelay(15 * 1000);
	

#if defined(CONFIG_PRIZE_LCD_BIAS)
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

	udelay(10*1000);
	
	gpiod_set_value(ctx->reset_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	
	udelay(10*1000);

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

#define HFP (38)
#define HSA (4)
#define HBP (28)
#define VFP (34)
#define VSA (2)
#define VBP (13)
#define VAC (2400)//1920
#define HAC (1080)//1080
static const struct drm_display_mode default_mode = {
	.clock = 168981,     //htotal*vtotal*vrefresh/1000
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
	
	pr_info("%s_reset_gpio\n", __func__);
	display_bias_disable();
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
	.pll_clk = 509,   //420
	.vfp_low_power = 750,
	.cust_esd_check = 0,
	.esd_check_enable = 0,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0a,
		.count = 1,
		.para_list[0] = 0x1c,
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
	dsi->mode_flags = MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET
			 | MIPI_DSI_CLOCK_NON_CONTINUOUS |MIPI_DSI_MODE_VIDEO;

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
	strcpy(current_lcm_info.chip,"ili7807q");
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
	{ .compatible = "auo,ili7807q,vdo", },
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "panel-auo-ili7807q-vdo",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("Yi-Lun Wang <Yi-Lun.Wang@mediatek.com>");
MODULE_DESCRIPTION("truly td4330 CMD LCD Panel Driver");
MODULE_LICENSE("GPL v2");
