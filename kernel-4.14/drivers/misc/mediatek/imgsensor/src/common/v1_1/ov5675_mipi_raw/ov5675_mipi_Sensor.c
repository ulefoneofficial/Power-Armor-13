/*
 * Copyright (C) 2018 MediaTek Inc.
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

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "ov5675_mipi_Sensor.h"

/**************** Modify Following Strings for Debug ******************/
#define PFX "ov5675_camera_sensor"
#define LOG_1 printk("OV5675_ov5675MIPI, 2LANE\n")
/********************   Modify end    *********************************/

#define cam_pr_debug(format, args...) \
	pr_debug(PFX "[%s] " format, __func__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);
/* used for shutter compensation */

static kal_uint32 Dgain_ratio = OV5675_SENSOR_DGAIN_BASE;
//static struct ov5675_otp_t ov5675_otp_data;

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = OV5675_SENSOR_ID,
	.checksum_value = 0xdc9f7d95,

	.pre = {
		.pclk = 87600000,
		.linelength = 1460,
		.framelength = 2008,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1296,
		.grabwindow_height = 972,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 87600000,
		.max_framerate = 300,
	},
	.cap = {
		.pclk = 175200000,
		.linelength = 2920,
		.framelength = 2008,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 175200000,
		.max_framerate = 300,
	},
	.cap1 = {
		.pclk = 141600000,
		.linelength = 2920,
		.framelength = 2008,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 141600000,
		.max_framerate = 240,             /*less than 13M(include 13M)*/
	},
	.normal_video = {
		.pclk = 87600000,
		.linelength = 1460,
		.framelength = 2008,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1296,
		.grabwindow_height = 972,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 87600000,
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 175200000,
		.linelength = 1896,
		.framelength = 1536,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 720,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 175200000,
		.max_framerate = 600,
	},
	.slim_video = {
		.pclk = 87600000,
		.linelength = 1460,
		.framelength = 2008,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 720,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 87600000,
		.max_framerate = 300,
	},
    // huangjiwu for  captrue black --begin 
	.min_gain = 73,
	.max_gain = 4096,
	.min_gain_iso = 100,
	.exp_step = 2,
	.gain_step = 1,
	.gain_type = 0,
	.temperature_support=0,
	// huangjiwu for  captrue black --begin 
	.margin = 16,
	.min_shutter = 4,
	.max_frame_length = 0x3fff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0,
	.ihdr_le_firstline = 0,
	.sensor_mode_num = 5,

	.cap_delay_frame = 2,
	.pre_delay_frame = 2,
	.video_delay_frame = 2,
	.hs_video_delay_frame = 2,
	.slim_video_delay_frame = 2,

	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,
#if OV5675_MIRROR_FLIP_ENABLE
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,
#else
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,
#endif
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_2_LANE,
	.i2c_addr_table = {0x6c,  0xff},//record sensor support all write id addr, only supprt 4must end with 0xff
};

static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x258,
	.gain = 0x40,
	.dummy_pixel = 0,
	.dummy_line = 0,
	.current_fps = 300,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	.ihdr_en = 0,
	.i2c_write_id = 0x6c,//record current sensor's i2c write id
};

/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] = {
	{ 2592, 1944,   0,   0, 2592, 1944, 1296,  972, 0, 0,
		1296,  972, 0, 0, 1296,  972 },
	{ 2592, 1944,   0,   0, 2592, 1944, 2592, 1944, 0, 0,
		2592, 1944, 0, 0, 2592, 1944 },
	{ 2592, 1944,   0,   0, 2592, 1944, 1296,  972, 0, 0,
		1296,  972, 0, 0, 1296,  972 },
	{ 2592, 1944, 656, 492, 1280,  960,  640,  480, 0, 0,
		640,  480, 0, 0,  640,  480 },
	{ 2592, 1944,  16, 252, 2560, 1440, 1280,  720, 0, 0,
		1280,  720, 0, 0, 1280,  720 }
};

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;

	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
	iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}


static void set_dummy(void)
{

	write_cmos_sensor(0x380e, (imgsensor.frame_length >> 8) & 0xFF);
	write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);	  
	write_cmos_sensor(0x380c, (imgsensor.line_length >> 8) & 0xFF);
	write_cmos_sensor(0x380d, imgsensor.line_length & 0xFF);
	printk("OV5675_set_dummy\n");
}

static kal_uint32 return_sensor_id(void)
{
	return ((read_cmos_sensor(0x300B) << 8) | read_cmos_sensor(0x300C));
	//return ((read_cmos_sensor(0x300A) << 8) | read_cmos_sensor(0x300B));
}

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;

	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length)
		? frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length -
		imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length -
			imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}

/*************************************************************************
 * FUNCTION
 *    set_shutter
 *
 * DESCRIPTION
 *    This function set e-shutter of sensor to change exposure time.
 *
 * PARAMETERS
 *    iShutter : exposured lines
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0, cal_shutter = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	/*if shutter bigger than framelength, should extend frame length first*/
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ?
		imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length -
		imgsensor_info.margin)) ?
		(imgsensor_info.max_frame_length -
		 imgsensor_info.margin) : shutter;

	realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 /
		imgsensor.frame_length;

	if (imgsensor.autoflicker_en) {
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else
			set_max_framerate(realtime_fps, 0);
	} else
		set_max_framerate(realtime_fps, 0);

	cal_shutter = shutter >> 2;
	cal_shutter = cal_shutter << 2;
	Dgain_ratio = 256 * shutter / cal_shutter;

	write_cmos_sensor(0x3502, 0x00);
	write_cmos_sensor(0x3501, (cal_shutter >> 8) & 0x3F);
	write_cmos_sensor(0x3500, cal_shutter & 0xFF);

	printk("OV5675_Exit! shutter = %d, framelength = %d\n",
			shutter, imgsensor.frame_length);
	printk("OV5675_Exit! cal_shutter = %d, ", cal_shutter);
}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = gain << 2;

	if (reg_gain < OV5675_SENSOR_GAIN_BASE)
		reg_gain = OV5675_SENSOR_GAIN_BASE;
	else if (reg_gain > OV5675_SENSOR_GAIN_MAX)
		reg_gain = OV5675_SENSOR_GAIN_MAX;

	return (kal_uint16)reg_gain;
}

/*************************************************************************
 * FUNCTION
 *    set_gain
 *
 * DESCRIPTION
 *    This function is to set global gain to sensor.
 *
 * PARAMETERS
 *    iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *    the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
    kal_uint16 reg_gain;
 
		reg_gain = gain2reg(gain);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.gain = reg_gain; 
		spin_unlock(&imgsensor_drv_lock);
		printk("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);
	
		write_cmos_sensor(0x3508, reg_gain >> 8);
		write_cmos_sensor(0x3509, reg_gain & 0xFF);    
		
		return gain;
}

static void ihdr_write_shutter_gain(kal_uint16 le,
		kal_uint16 se, kal_uint16 gain)
{
	printk("OV5675_le: 0x%x, se: 0x%x, gain: 0x%x\n", le, se, gain);
}

#if 0
static void set_mirror_flip(kal_uint8 image_mirror)
{
	printk("OV5675_image_mirror = %d\n", image_mirror);
}
#endif

/*************************************************************************
 * FUNCTION
 *    night_mode
 *
 * DESCRIPTION
 *    This function night mode of sensor.
 *
 * PARAMETERS
 *    bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void night_mode(kal_bool enable)
{
	/* No Need to implement this function */
}

static void sensor_init(void)
{
	printk("OV5675_OV5675_Sensor_Init_2lane E\n");
write_cmos_sensor(0x0103 ,0x01);
	mdelay(10);
write_cmos_sensor(0x0100 ,0x00);
write_cmos_sensor(0x0300 ,0x04);
write_cmos_sensor(0x0301 ,0x00);
write_cmos_sensor(0x0302 ,0x69);
write_cmos_sensor(0x0303 ,0x00);
write_cmos_sensor(0x0304 ,0x00);
write_cmos_sensor(0x0305 ,0x01);
write_cmos_sensor(0x0307 ,0x00);
write_cmos_sensor(0x030b ,0x00);
write_cmos_sensor(0x030c ,0x00);
write_cmos_sensor(0x030d ,0x1e);
write_cmos_sensor(0x030e ,0x04);
write_cmos_sensor(0x030f ,0x03);
write_cmos_sensor(0x0312 ,0x01);
write_cmos_sensor(0x3000 ,0x00);
write_cmos_sensor(0x3002 ,0x21);
write_cmos_sensor(0x3016 ,0x32);
write_cmos_sensor(0x3022 ,0x51);
write_cmos_sensor(0x3106 ,0x15);
write_cmos_sensor(0x3107 ,0x01);
write_cmos_sensor(0x3108 ,0x05);
write_cmos_sensor(0x3500 ,0x00);
write_cmos_sensor(0x3501 ,0x7e);
write_cmos_sensor(0x3502 ,0x00);
write_cmos_sensor(0x3503 ,0x08);
write_cmos_sensor(0x3504 ,0x03);
write_cmos_sensor(0x3505 ,0x8c);
write_cmos_sensor(0x3507 ,0x03);
write_cmos_sensor(0x3508 ,0x00);
write_cmos_sensor(0x3509 ,0x10);
write_cmos_sensor(0x350c ,0x00);
write_cmos_sensor(0x350d ,0x80);
write_cmos_sensor(0x3510 ,0x00);
write_cmos_sensor(0x3511 ,0x02);
write_cmos_sensor(0x3512 ,0x00);
write_cmos_sensor(0x3601 ,0x55);
write_cmos_sensor(0x3602 ,0x58);
write_cmos_sensor(0x3611 ,0x58);
write_cmos_sensor(0x3614 ,0x30);
write_cmos_sensor(0x3615 ,0x77);
write_cmos_sensor(0x3621 ,0x08);
write_cmos_sensor(0x3624 ,0x40);
write_cmos_sensor(0x3633 ,0x0c);
write_cmos_sensor(0x3634 ,0x0c);
write_cmos_sensor(0x3635 ,0x0c);
write_cmos_sensor(0x3636 ,0x0c);
write_cmos_sensor(0x3638 ,0x00);
write_cmos_sensor(0x3639 ,0x00);
write_cmos_sensor(0x363a ,0x00);
write_cmos_sensor(0x363b ,0x00);
write_cmos_sensor(0x363c ,0xff);
write_cmos_sensor(0x363d ,0xfa);
write_cmos_sensor(0x3650 ,0x44);
write_cmos_sensor(0x3651 ,0x44);
write_cmos_sensor(0x3652 ,0x44);
write_cmos_sensor(0x3653 ,0x44);
write_cmos_sensor(0x3654 ,0x44);
write_cmos_sensor(0x3655 ,0x44);
write_cmos_sensor(0x3656 ,0x44);
write_cmos_sensor(0x3657 ,0x44);
write_cmos_sensor(0x3660 ,0x00);
write_cmos_sensor(0x3661 ,0x00);
write_cmos_sensor(0x3662 ,0x00);
write_cmos_sensor(0x366a ,0x00);
write_cmos_sensor(0x366e ,0x18);
write_cmos_sensor(0x3673 ,0x04);
write_cmos_sensor(0x3700 ,0x14);
write_cmos_sensor(0x3703 ,0x0c);
write_cmos_sensor(0x3706 ,0x24);
write_cmos_sensor(0x3714 ,0x23);
write_cmos_sensor(0x3715 ,0x01);
write_cmos_sensor(0x3716 ,0x00);
write_cmos_sensor(0x3717 ,0x02);
write_cmos_sensor(0x3733 ,0x10);
write_cmos_sensor(0x3734 ,0x40);
write_cmos_sensor(0x373f ,0xa0);
write_cmos_sensor(0x3765 ,0x20);
write_cmos_sensor(0x37a1 ,0x1d);
write_cmos_sensor(0x37a8 ,0x26);
write_cmos_sensor(0x37ab ,0x14);
write_cmos_sensor(0x37c2 ,0x04);
write_cmos_sensor(0x37c3 ,0xf0);
write_cmos_sensor(0x37cb ,0x09);
write_cmos_sensor(0x37cc ,0x13);
write_cmos_sensor(0x37cd ,0x1f);
write_cmos_sensor(0x37ce ,0x1f);
write_cmos_sensor(0x3800 ,0x00);
write_cmos_sensor(0x3801 ,0x00);
write_cmos_sensor(0x3802 ,0x00);
write_cmos_sensor(0x3803 ,0x04);
write_cmos_sensor(0x3804 ,0x0a);
write_cmos_sensor(0x3805 ,0x3f);
write_cmos_sensor(0x3806 ,0x07);
write_cmos_sensor(0x3807 ,0xab);
write_cmos_sensor(0x3808 ,0x0a);
write_cmos_sensor(0x3809 ,0x20);
write_cmos_sensor(0x380a ,0x07);
write_cmos_sensor(0x380b ,0x98);
write_cmos_sensor(0x380c ,0x02);
write_cmos_sensor(0x380d ,0xe4);
write_cmos_sensor(0x380e ,0x07);
write_cmos_sensor(0x380f ,0xe8);
write_cmos_sensor(0x3810 ,0x00);
write_cmos_sensor(0x3811 ,0x10);
write_cmos_sensor(0x3812 ,0x00);
write_cmos_sensor(0x3813 ,0x08);
write_cmos_sensor(0x3814 ,0x01);
write_cmos_sensor(0x3815 ,0x01);
write_cmos_sensor(0x3816 ,0x01);
write_cmos_sensor(0x3817 ,0x01);
write_cmos_sensor(0x3818 ,0x00);
write_cmos_sensor(0x3819 ,0x00);
write_cmos_sensor(0x381a ,0x00);
write_cmos_sensor(0x381b ,0x01);
write_cmos_sensor(0x3820 ,0x88);
write_cmos_sensor(0x3821 ,0x00);
write_cmos_sensor(0x3c80 ,0x08);
write_cmos_sensor(0x3c82 ,0x00);
write_cmos_sensor(0x3c83 ,0x00);
write_cmos_sensor(0x3c88 ,0x00);
write_cmos_sensor(0x3d85 ,0x14);
write_cmos_sensor(0x3f02 ,0x08);
write_cmos_sensor(0x3f03 ,0x10);
write_cmos_sensor(0x4008 ,0x04);
write_cmos_sensor(0x4009 ,0x13);
write_cmos_sensor(0x404e ,0x20);
write_cmos_sensor(0x4501 ,0x00);
write_cmos_sensor(0x4502 ,0x10);
write_cmos_sensor(0x4800 ,0x00);
write_cmos_sensor(0x481f ,0x2a);
write_cmos_sensor(0x4837 ,0x13);
write_cmos_sensor(0x5000 ,0x13);
write_cmos_sensor(0x5780 ,0x3e);
write_cmos_sensor(0x5781 ,0x0f);
write_cmos_sensor(0x5782 ,0x44);
write_cmos_sensor(0x5783 ,0x02);
write_cmos_sensor(0x5784 ,0x01);
write_cmos_sensor(0x5785 ,0x01);
write_cmos_sensor(0x5786 ,0x00);
write_cmos_sensor(0x5787 ,0x04);
write_cmos_sensor(0x5788 ,0x02);
write_cmos_sensor(0x5789 ,0x0f);
write_cmos_sensor(0x578a,0xfd);
write_cmos_sensor(0x578b ,0xf5);
write_cmos_sensor(0x578c ,0xf5);
write_cmos_sensor(0x578d ,0x03);
write_cmos_sensor(0x578e ,0x08);
write_cmos_sensor(0x578f ,0x0c);
write_cmos_sensor(0x5790 ,0x08);
write_cmos_sensor(0x5791 ,0x06);
write_cmos_sensor(0x5792 ,0x00);
write_cmos_sensor(0x5793 ,0x52);
write_cmos_sensor(0x5794 ,0xa3);
write_cmos_sensor(0x5b00 ,0x00);
write_cmos_sensor(0x5b01 ,0x1c);
write_cmos_sensor(0x5b02 ,0x00);
write_cmos_sensor(0x5b03 ,0x7f);
write_cmos_sensor(0x5b05 ,0x6c);
write_cmos_sensor(0x5e10 ,0xfc);
write_cmos_sensor(0x4010 ,0xf1);
write_cmos_sensor(0x3503 ,0x08);
write_cmos_sensor(0x3505 ,0x8c);
write_cmos_sensor(0x3507 ,0x03);
write_cmos_sensor(0x3508 ,0x00);
write_cmos_sensor(0x3509 ,0xf8);
write_cmos_sensor(0x0100 ,0x01);
	//OV5675R1A_AM01d 
//;;base_on_1280x720_30FPS_MIPI_2_LANE_900Mbps
//;Xclk 24Mhz
//;Pclk clock frequency: 45Mhz
//;linelength = 750(0x2ee)
//;framelength = 2000(0x7d0)
//;grabwindow_width  = 1296
//;grabwindow_height = 972
//;max_framerate: 30fps	
//;mipi_datarate per lane: 900Mbps
#if 0
write_cmos_sensor(0x0103 ,0x01);
	mdelay(10);
write_cmos_sensor(0x0100 ,0x00);
write_cmos_sensor(0x0300 ,0x04);
write_cmos_sensor(0x0301 ,0x00);
write_cmos_sensor(0x0302 ,0x69);
write_cmos_sensor(0x0303 ,0x00);
write_cmos_sensor(0x0304 ,0x00);
write_cmos_sensor(0x0305 ,0x01);
write_cmos_sensor(0x0307 ,0x00);
write_cmos_sensor(0x030b ,0x00);
write_cmos_sensor(0x030c ,0x00);
write_cmos_sensor(0x030d ,0x1e);
write_cmos_sensor(0x030e ,0x04);
write_cmos_sensor(0x030f ,0x03);
write_cmos_sensor(0x0312 ,0x01);
write_cmos_sensor(0x3000 ,0x00);
write_cmos_sensor(0x3002 ,0x21);
write_cmos_sensor(0x3016 ,0x32);
write_cmos_sensor(0x3022 ,0x51);
write_cmos_sensor(0x3106 ,0x15);
write_cmos_sensor(0x3107 ,0x01);
write_cmos_sensor(0x3108 ,0x05);
write_cmos_sensor(0x3500 ,0x00);
write_cmos_sensor(0x3501 ,0x3e);
write_cmos_sensor(0x3502 ,0x00);
write_cmos_sensor(0x3503 ,0x08);
write_cmos_sensor(0x3504 ,0x03);
write_cmos_sensor(0x3505 ,0x8c);
write_cmos_sensor(0x3507 ,0x03);
write_cmos_sensor(0x3508 ,0x00);
write_cmos_sensor(0x3509 ,0x10);
write_cmos_sensor(0x350c ,0x00);
write_cmos_sensor(0x350d ,0x80);
write_cmos_sensor(0x3510 ,0x00);
write_cmos_sensor(0x3511 ,0x02);
write_cmos_sensor(0x3512 ,0x00);
write_cmos_sensor(0x3601 ,0x55);
write_cmos_sensor(0x3602 ,0x58);
write_cmos_sensor(0x3611 ,0x58);
write_cmos_sensor(0x3614 ,0x30);
write_cmos_sensor(0x3615 ,0x77);
write_cmos_sensor(0x3621 ,0x08);
write_cmos_sensor(0x3624 ,0x40);
write_cmos_sensor(0x3633 ,0x0c);
write_cmos_sensor(0x3634 ,0x0c);
write_cmos_sensor(0x3635 ,0x0c);
write_cmos_sensor(0x3636 ,0x0c);
write_cmos_sensor(0x3638 ,0x00);
write_cmos_sensor(0x3639 ,0x00);
write_cmos_sensor(0x363a ,0x00);
write_cmos_sensor(0x363b ,0x00);
write_cmos_sensor(0x363c ,0xff);
write_cmos_sensor(0x363d ,0xfa);
write_cmos_sensor(0x3650 ,0x44);
write_cmos_sensor(0x3651 ,0x44);
write_cmos_sensor(0x3652 ,0x44);
write_cmos_sensor(0x3653 ,0x44);
write_cmos_sensor(0x3654 ,0x44);
write_cmos_sensor(0x3655 ,0x44);
write_cmos_sensor(0x3656 ,0x44);
write_cmos_sensor(0x3657 ,0x44);
write_cmos_sensor(0x3660 ,0x00);
write_cmos_sensor(0x3661 ,0x00);
write_cmos_sensor(0x3662 ,0x00);
write_cmos_sensor(0x366a ,0x00);
write_cmos_sensor(0x366e ,0x0c);
write_cmos_sensor(0x3673 ,0x04);
write_cmos_sensor(0x3700 ,0x14);
write_cmos_sensor(0x3703 ,0x0c);
write_cmos_sensor(0x3706 ,0x24);
write_cmos_sensor(0x3714 ,0x27);
write_cmos_sensor(0x3715 ,0x01);
write_cmos_sensor(0x3716 ,0x00);
write_cmos_sensor(0x3717 ,0x02);
write_cmos_sensor(0x3733 ,0x10);
write_cmos_sensor(0x3734 ,0x40);
write_cmos_sensor(0x373f ,0xa0);
write_cmos_sensor(0x3765 ,0x20);
write_cmos_sensor(0x37a1 ,0x1d);
write_cmos_sensor(0x37a8 ,0x26);
write_cmos_sensor(0x37ab ,0x14);
write_cmos_sensor(0x37c2 ,0x04);
write_cmos_sensor(0x37c3 ,0xf0);
write_cmos_sensor(0x37cb ,0x09);
write_cmos_sensor(0x37cc ,0x13);
write_cmos_sensor(0x37cd ,0x1f);
write_cmos_sensor(0x37ce ,0x1f);
write_cmos_sensor(0x3800 ,0x00);
write_cmos_sensor(0x3801 ,0x00);
write_cmos_sensor(0x3802 ,0x00);
write_cmos_sensor(0x3803 ,0x00);
write_cmos_sensor(0x3804 ,0x0a);
write_cmos_sensor(0x3805 ,0x3f);
write_cmos_sensor(0x3806 ,0x07);
write_cmos_sensor(0x3807 ,0xaf);
write_cmos_sensor(0x3808 ,0x05);
write_cmos_sensor(0x3809 ,0x10);
write_cmos_sensor(0x380a ,0x03);
write_cmos_sensor(0x380b ,0xcc);
write_cmos_sensor(0x380c ,0x02);
write_cmos_sensor(0x380d ,0xe4);
write_cmos_sensor(0x380e ,0x03);
write_cmos_sensor(0x380f ,0xf4);
write_cmos_sensor(0x3810 ,0x00);
write_cmos_sensor(0x3811 ,0x00);
write_cmos_sensor(0x3812 ,0x00);
write_cmos_sensor(0x3813 ,0x06);
write_cmos_sensor(0x3814 ,0x03);
write_cmos_sensor(0x3815 ,0x01);
write_cmos_sensor(0x3816 ,0x03);
write_cmos_sensor(0x3817 ,0x01);
write_cmos_sensor(0x3818 ,0x00);
write_cmos_sensor(0x3819 ,0x00);
write_cmos_sensor(0x381a ,0x00);
write_cmos_sensor(0x381b ,0x01);
write_cmos_sensor(0x3820 ,0x8b);
write_cmos_sensor(0x3821 ,0x01);
write_cmos_sensor(0x3c80 ,0x08);
write_cmos_sensor(0x3c82 ,0x00);
write_cmos_sensor(0x3c83 ,0x00);
write_cmos_sensor(0x3c88 ,0x00);
write_cmos_sensor(0x3d85 ,0x14);
write_cmos_sensor(0x3f02 ,0x08);
write_cmos_sensor(0x3f03 ,0x10);
write_cmos_sensor(0x4008 ,0x02);
write_cmos_sensor(0x4009 ,0x09);
write_cmos_sensor(0x404e ,0x20);
write_cmos_sensor(0x4501 ,0x00);
write_cmos_sensor(0x4502 ,0x10);
write_cmos_sensor(0x4800 ,0x00);
write_cmos_sensor(0x481f ,0x2a);
write_cmos_sensor(0x4837 ,0x13);
write_cmos_sensor(0x5000 ,0x13);
write_cmos_sensor(0x5780 ,0x3e);
write_cmos_sensor(0x5781 ,0x0f);
write_cmos_sensor(0x5782 ,0x44);
write_cmos_sensor(0x5783 ,0x02);
write_cmos_sensor(0x5784 ,0x01);
write_cmos_sensor(0x5785 ,0x01);
write_cmos_sensor(0x5786 ,0x00);
write_cmos_sensor(0x5787 ,0x04);
write_cmos_sensor(0x5788 ,0x02);
write_cmos_sensor(0x5789 ,0x0f);
write_cmos_sensor(0x578a ,0xfd);
write_cmos_sensor(0x578b ,0xf5);
write_cmos_sensor(0x578c ,0xf5);
write_cmos_sensor(0x578d ,0x03);
write_cmos_sensor(0x578e ,0x08);
write_cmos_sensor(0x578f ,0x0c);
write_cmos_sensor(0x5790 ,0x08);
write_cmos_sensor(0x5791 ,0x06);
write_cmos_sensor(0x5792 ,0x00);
write_cmos_sensor(0x5793 ,0x52);
write_cmos_sensor(0x5794 ,0xa3);
write_cmos_sensor(0x5b00 ,0x00);
write_cmos_sensor(0x5b01 ,0x1c);
write_cmos_sensor(0x5b02 ,0x00);
write_cmos_sensor(0x5b03 ,0x7f);
write_cmos_sensor(0x5b05 ,0x6c);
write_cmos_sensor(0x5e10 ,0xfc);
write_cmos_sensor(0x4010 ,0xf1);
write_cmos_sensor(0x3503 ,0x08);
write_cmos_sensor(0x3505 ,0x8c);
write_cmos_sensor(0x3507 ,0x03);
write_cmos_sensor(0x3508 ,0x00);
write_cmos_sensor(0x3509 ,0xf8);
write_cmos_sensor(0x0100 ,0x01);
#endif
#if 0
write_cmos_sensor(0x0103 ,0x01);
write_cmos_sensor(0x0100 ,0x00);
write_cmos_sensor(0x0300 ,0x04);
write_cmos_sensor(0x0301 ,0x00);
write_cmos_sensor(0x0302 ,0x69);
write_cmos_sensor(0x0303 ,0x00);
write_cmos_sensor(0x0304 ,0x00);
write_cmos_sensor(0x0305 ,0x01);
write_cmos_sensor(0x0307 ,0x00);
write_cmos_sensor(0x030b ,0x00);
write_cmos_sensor(0x030c ,0x00);
write_cmos_sensor(0x030d ,0x1e);
write_cmos_sensor(0x030e ,0x04);
write_cmos_sensor(0x030f ,0x03);
write_cmos_sensor(0x0312 ,0x01);
write_cmos_sensor(0x3000 ,0x00);
write_cmos_sensor(0x3002 ,0x21);
write_cmos_sensor(0x3016 ,0x32);
write_cmos_sensor(0x3022 ,0x51);
write_cmos_sensor(0x3106 ,0x15);
write_cmos_sensor(0x3107 ,0x01);
write_cmos_sensor(0x3108 ,0x05);
write_cmos_sensor(0x3500 ,0x00);
write_cmos_sensor(0x3501 ,0x7e);
write_cmos_sensor(0x3502, 0x00);
write_cmos_sensor(0x3503 ,0x08);
write_cmos_sensor(0x3504 ,0x03);
write_cmos_sensor(0x3505 ,0x8c);
write_cmos_sensor(0x3507 ,0x03);
write_cmos_sensor(0x3508 ,0x00);
write_cmos_sensor(0x3509 ,0x10);
write_cmos_sensor(0x350c, 0x00);
write_cmos_sensor(0x350d ,0x80);
write_cmos_sensor(0x3510 ,0x00);
write_cmos_sensor(0x3511, 0x02);
write_cmos_sensor(0x3512, 0x00);
write_cmos_sensor(0x3601 ,0x55);
write_cmos_sensor(0x3602 ,0x58);
write_cmos_sensor(0x3611 ,0x58);
write_cmos_sensor(0x3614 ,0x30);
write_cmos_sensor(0x3615 ,0x77);
write_cmos_sensor(0x3621, 0x08);
write_cmos_sensor(0x3624 ,0x40);
write_cmos_sensor(0x3633 ,0x0c);
write_cmos_sensor(0x3634 ,0x0c);
write_cmos_sensor(0x3635 ,0x0c);
write_cmos_sensor(0x3636 ,0x0c);
write_cmos_sensor(0x3638 ,0x00);
write_cmos_sensor(0x3639 ,0x00);
write_cmos_sensor(0x363a, 0x00);
write_cmos_sensor(0x363b, 0x00);
write_cmos_sensor(0x363c ,0xff);
write_cmos_sensor(0x363d ,0xfa);
write_cmos_sensor(0x3650 ,0x44);
write_cmos_sensor(0x3651 ,0x44);
write_cmos_sensor(0x3652 ,0x44);
write_cmos_sensor(0x3653 ,0x44);
write_cmos_sensor(0x3654 ,0x44);
write_cmos_sensor(0x3655 ,0x44);
write_cmos_sensor(0x3656, 0x44);
write_cmos_sensor(0x3657, 0x44);
write_cmos_sensor(0x3660, 0x00);
write_cmos_sensor(0x3661, 0x00);
write_cmos_sensor(0x3662, 0x00);
write_cmos_sensor(0x366a ,0x00);
write_cmos_sensor(0x366e, 0x18);
write_cmos_sensor(0x3673, 0x04);
write_cmos_sensor(0x3700, 0x14);
write_cmos_sensor(0x3703, 0x0c);
write_cmos_sensor(0x3706, 0x24);
write_cmos_sensor(0x3714, 0x23);
write_cmos_sensor(0x3715 ,0x01);
write_cmos_sensor(0x3716, 0x00);
write_cmos_sensor(0x3717, 0x02);
write_cmos_sensor(0x3733, 0x10);
write_cmos_sensor(0x3734, 0x40);
write_cmos_sensor(0x373f, 0xa0);
write_cmos_sensor(0x3765,,0x20);
write_cmos_sensor(0x37a1 ,0x1d);
write_cmos_sensor(0x37a8 ,0x26);
write_cmos_sensor(0x37ab ,0x14);
write_cmos_sensor(0x37c2 ,0x04);
write_cmos_sensor(0x37c3 ,0xf0);
write_cmos_sensor(0x37cb, 0x09);
write_cmos_sensor(0x37cc, 0x13);
write_cmos_sensor(0x37cd ,0x1f);
write_cmos_sensor(0x37ce ,0x1f);
write_cmos_sensor(0x3800 ,0x00);
write_cmos_sensor(0x3801, 0x00);
write_cmos_sensor(0x3802, 0x00);
write_cmos_sensor(0x3803 ,0x04);
write_cmos_sensor(0x3804 ,0x0a);
write_cmos_sensor(0x3805, 0x3f);
write_cmos_sensor(0x3806 ,0x07);
write_cmos_sensor(0x3807 ,0xab);
write_cmos_sensor(0x3808 ,0x0a);
write_cmos_sensor(0x3809, 0x20);
write_cmos_sensor(0x380a, 0x07);
write_cmos_sensor(0x380b, 0x98);
write_cmos_sensor(0x380c ,0x02);
write_cmos_sensor(0x380d, 0xe4);
write_cmos_sensor(0x380e ,0x07);
write_cmos_sensor(0x380f, 0xe8);
write_cmos_sensor(0x3810, 0x00);
write_cmos_sensor(0x3811 ,0x10);
write_cmos_sensor(0x3812, 0x00);
write_cmos_sensor(0x3813, 0x08);
write_cmos_sensor(0x3814 ,0x01);
write_cmos_sensor(0x3815, 0x01);
write_cmos_sensor(0x3816, 0x01);
write_cmos_sensor(0x3817, 0x01);
write_cmos_sensor(0x3818, 0x00);
write_cmos_sensor(0x3819, 0x00);
write_cmos_sensor(0x381a, 0x00);
write_cmos_sensor(0x381b, 0x01);
write_cmos_sensor(0x3820, 0x88);
write_cmos_sensor(0x3821, 0x00);
write_cmos_sensor(0x3c80 ,0x08);
write_cmos_sensor(0x3c82, 0x00);
write_cmos_sensor(0x3c83, 0x00);
write_cmos_sensor(0x3c88 ,0x00);
write_cmos_sensor(0x3d85, 0x14);
write_cmos_sensor(0x3f02, 0x08);
write_cmos_sensor(0x3f03 ,0x10);
write_cmos_sensor(0x4008, 0x04);
write_cmos_sensor(0x4009 ,0x13);
write_cmos_sensor(0x404e ,0x20);
write_cmos_sensor(0x4501 ,0x00);
write_cmos_sensor(0x4502, 0x10);
write_cmos_sensor(0x4800 ,0x00);
write_cmos_sensor(0x481f, 0x2a);
write_cmos_sensor(0x4837 ,0x13);
write_cmos_sensor(0x5000, 0x13);
write_cmos_sensor(0x5780, 0x3e);
write_cmos_sensor(0x5781, 0x0f);
write_cmos_sensor(0x5782, 0x44);
write_cmos_sensor(0x5783, 0x02);
write_cmos_sensor(0x5784, 0x01);
write_cmos_sensor(0x5785, 0x01);
write_cmos_sensor(0x5786, 0x00);
write_cmos_sensor(0x5787, 0x04);
write_cmos_sensor(0x5788, 0x02);
write_cmos_sensor(0x5789, 0x0f);
write_cmos_sensor(0x578a, 0xfd);
write_cmos_sensor(0x578b, 0xf5);
write_cmos_sensor(0x578c, 0xf5);
write_cmos_sensor(0x578d, 0x03);
write_cmos_sensor(0x578e, 0x08);
write_cmos_sensor(0x578f, 0x0c);
write_cmos_sensor(0x5790, 0x08);
write_cmos_sensor(0x5791, 0x06);
write_cmos_sensor(0x5792, 0x00);
write_cmos_sensor(0x5793, 0x52);
write_cmos_sensor(0x5794, 0xa3);
write_cmos_sensor(0x5b00 ,0x00);
write_cmos_sensor(0x5b01 ,0x1c);
write_cmos_sensor(0x5b02, 0x00);
write_cmos_sensor(0x5b03 ,0x7f);
write_cmos_sensor(0x5b05 ,0x6c);
write_cmos_sensor(0x5e10, 0xfc);
write_cmos_sensor(0x4010 ,0xf1);
write_cmos_sensor(0x3503, 0x08);
write_cmos_sensor(0x3505, 0x8c);
write_cmos_sensor(0x3507 ,0x03);
write_cmos_sensor(0x3508, 0x00);
write_cmos_sensor(0x3509 ,0xf8);
write_cmos_sensor(0x0100, 0x01);
#endif
#if 0
write_cmos_sensor(0x0100, 0x00);
write_cmos_sensor(0x0103, 0x01);
mDELAY(10);
write_cmos_sensor(0x0300,0x05);
write_cmos_sensor(0x0302,0x96);
write_cmos_sensor(0x0303,0x00);
write_cmos_sensor(0x3002,0x21);
write_cmos_sensor(0x3107,0x23);
write_cmos_sensor(0x3501,0x20); 
write_cmos_sensor(0x3503,0x0c);
write_cmos_sensor(0x3508,0x03);
write_cmos_sensor(0x3509,0x00);
write_cmos_sensor(0x3600,0x66);
write_cmos_sensor(0x3602,0x30);
write_cmos_sensor(0x3610,0xa5);
write_cmos_sensor(0x3612,0x93);
write_cmos_sensor(0x3620,0x80);
write_cmos_sensor(0x3642,0x0e);
write_cmos_sensor(0x3661,0x00);
write_cmos_sensor(0x3662,0x10);
write_cmos_sensor(0x3664,0xf3);
write_cmos_sensor(0x3665,0x9e);
write_cmos_sensor(0x3667,0xa5);
write_cmos_sensor(0x366e,0x55);
write_cmos_sensor(0x366f,0x55);
write_cmos_sensor(0x3670,0x11);
write_cmos_sensor(0x3671,0x11);
write_cmos_sensor(0x3672,0x11);
write_cmos_sensor(0x3673,0x11);
write_cmos_sensor(0x3714,0x24);
write_cmos_sensor(0x371a,0x3e);
write_cmos_sensor(0x3733,0x10);
write_cmos_sensor(0x3734,0x00);
write_cmos_sensor(0x373d,0x24);   // "24"flip off ; "26"flip on
write_cmos_sensor(0x3764,0x20);
write_cmos_sensor(0x3765,0x20);
write_cmos_sensor(0x3766,0x12);
write_cmos_sensor(0x37a1,0x14);
write_cmos_sensor(0x37a8,0x1c);
write_cmos_sensor(0x37ab,0x0f);
write_cmos_sensor(0x37c2,0x04);
write_cmos_sensor(0x37cb,0x09);
write_cmos_sensor(0x37cc,0x15);
write_cmos_sensor(0x37cd,0x1f);
write_cmos_sensor(0x37ce,0x1f);
write_cmos_sensor(0x37d8,0x02);
write_cmos_sensor(0x37d9,0x08);
write_cmos_sensor(0x37dc,0x04);
write_cmos_sensor(0x3800,0x00);
write_cmos_sensor(0x3801,0x00);
write_cmos_sensor(0x3802,0x00);
write_cmos_sensor(0x3803,0x04);
write_cmos_sensor(0x3804,0x0a);
write_cmos_sensor(0x3805,0x3f);
write_cmos_sensor(0x3806,0x07);
write_cmos_sensor(0x3807,0xb3);
write_cmos_sensor(0x3808,0x0a);    // x output size H
write_cmos_sensor(0x3809,0x20);    // x output size L
write_cmos_sensor(0x380a,0x07);    // y outout size H
write_cmos_sensor(0x380b,0x98);    // y output size L
write_cmos_sensor(0x380c,0x02);    //HTS H
write_cmos_sensor(0x380d,0xee);    //HTS L
write_cmos_sensor(0x380e,0x07);    //VTS H
write_cmos_sensor(0x380f,0xd0);    //VTS L
write_cmos_sensor(0x3811,0x10);
write_cmos_sensor(0x3813,0x0c);
write_cmos_sensor(0x3814,0x01);
write_cmos_sensor(0x3815,0x01);
write_cmos_sensor(0x3816,0x01);
write_cmos_sensor(0x3817,0x01);
write_cmos_sensor(0x381e,0x02);
write_cmos_sensor(0x3820,0x88);     // "88" mirror on    "80" mirror off
write_cmos_sensor(0x3821,0x01);
write_cmos_sensor(0x3832,0x04);
write_cmos_sensor(0x3c80,0x01);
write_cmos_sensor(0x3c82,0x00);
write_cmos_sensor(0x3c83,0xc8);
write_cmos_sensor(0x3c8c,0x0f);
write_cmos_sensor(0x3c8d,0xa0);
write_cmos_sensor(0x3c90,0x07);
write_cmos_sensor(0x3c91,0x00);
write_cmos_sensor(0x3c92,0x00);
write_cmos_sensor(0x3c93,0x00);
write_cmos_sensor(0x3c94,0xd0);
write_cmos_sensor(0x3c95,0x50);
write_cmos_sensor(0x3c96,0x35);
write_cmos_sensor(0x3c97,0x00);
write_cmos_sensor(0x4001,0xe0);
write_cmos_sensor(0x4008,0x02);
write_cmos_sensor(0x4009,0x0d);
write_cmos_sensor(0x400f,0x80);
write_cmos_sensor(0x4013,0x02);
write_cmos_sensor(0x4040,0x00);
write_cmos_sensor(0x4041,0x07);
write_cmos_sensor(0x404c,0x50);
write_cmos_sensor(0x404e,0x20);
write_cmos_sensor(0x4500,0x06);
write_cmos_sensor(0x4503,0x00);
write_cmos_sensor(0x450a,0x04);
write_cmos_sensor(0x4800,0x20);    //mipi\C1\AC\D0\F8\B7\C7\C1\AC\D0\F8  20\B7\C7\C1\AC\D0\F8   00\C1\AC\D0\F8
write_cmos_sensor(0x4809,0x04);
write_cmos_sensor(0x480c,0x12);
write_cmos_sensor(0x4819,0x70);
write_cmos_sensor(0x4825,0x32);
write_cmos_sensor(0x4826,0x32);
write_cmos_sensor(0x482a,0x06);
write_cmos_sensor(0x4833,0x08);
write_cmos_sensor(0x4837,0x0d);
write_cmos_sensor(0x5000,0x77);
write_cmos_sensor(0x5b00,0x01);
write_cmos_sensor(0x5b01,0x10);
write_cmos_sensor(0x5b02,0x01);
write_cmos_sensor(0x5b03,0xdb);
write_cmos_sensor(0x5b05,0x6c);
write_cmos_sensor(0x5e10,0xfc);
write_cmos_sensor(0x3500,0x00);
write_cmos_sensor(0x3501,0x3E);      //max expo= ([380e,380f]-4)/2.
write_cmos_sensor(0x3502,0x60);
write_cmos_sensor(0x3503,0x08);
write_cmos_sensor(0x3508,0x04);
write_cmos_sensor(0x3509,0x00);
write_cmos_sensor(0x3832,0x48);
write_cmos_sensor(0x3c90,0x00);
write_cmos_sensor(0x5780,0x3e);
write_cmos_sensor(0x5781,0x0f);
write_cmos_sensor(0x5782,0x44);
write_cmos_sensor(0x5783,0x02);
write_cmos_sensor(0x5784,0x01);
write_cmos_sensor(0x5785,0x01);
write_cmos_sensor(0x5786,0x00);
write_cmos_sensor(0x5787,0x04);
write_cmos_sensor(0x5788,0x02);
write_cmos_sensor(0x5789,0x0f);
write_cmos_sensor(0x578a,0xfd);
write_cmos_sensor(0x578b,0xf5);
write_cmos_sensor(0x578c,0xf5);
write_cmos_sensor(0x578d,0x03);
write_cmos_sensor(0x578e,0x08);
write_cmos_sensor(0x578f,0x0c);
write_cmos_sensor(0x5790,0x08);
write_cmos_sensor(0x5791,0x06);
write_cmos_sensor(0x5792,0x00);
write_cmos_sensor(0x5793,0x52);
write_cmos_sensor(0x5794,0xa3);
write_cmos_sensor(0x4003,0x40);
write_cmos_sensor(0x3107,0x01);
write_cmos_sensor(0x3c80,0x08);
write_cmos_sensor(0x3c83,0xb1);
write_cmos_sensor(0x3c8c,0x10);
write_cmos_sensor(0x3c8d,0x00);
write_cmos_sensor(0x3c90,0x00);
write_cmos_sensor(0x3c94,0x00);
write_cmos_sensor(0x3c95,0x00);
write_cmos_sensor(0x3c96,0x00);
write_cmos_sensor(0x3d8c,0x71);
write_cmos_sensor(0x3d8d,0xE7);
write_cmos_sensor(0x0100,0x01);
#endif

}

static void preview_setting(void)
{
	#if 0
	LOG_INF(" OV5675PreviewSetting_2lane enter\n");

//@@1296X972_30fps
//;;1296X972_HBIN_VBIN_30FPS_MIPI_2_LANE
//102 3601 1770	
//;Xclk 24Mhz
//;Pclk clock frequency: 45Mhz
//;linelength = 672(0x2a0)
//;framelength = 2232(0x8b8)
//;grabwindow_width  = 1296
//;grabwindow_height = 972
//;max_framerate: 30fps	
//;mipi_datarate per lane: 840Mbps				  
																			  
	  write_cmos_sensor(0x0100, 0x00);  // 
    write_cmos_sensor(0x3501, 0x45);
    write_cmos_sensor(0x366e, 0x0c);
    write_cmos_sensor(0x3800, 0x00);
    write_cmos_sensor(0x3801, 0x00);
    write_cmos_sensor(0x3802, 0x00);
    write_cmos_sensor(0x3803, 0x00);
    write_cmos_sensor(0x3804, 0x0a);
    write_cmos_sensor(0x3805, 0x3f);
    write_cmos_sensor(0x3806, 0x07);
    write_cmos_sensor(0x3807, 0xaf);
    write_cmos_sensor(0x3808, 0x05);
    write_cmos_sensor(0x3809, 0x10);
    write_cmos_sensor(0x380a, 0x03);
    write_cmos_sensor(0x380b, 0xcc);
    write_cmos_sensor(0x380c, 0x02);
    write_cmos_sensor(0x380d, 0xa0);
    write_cmos_sensor(0x380e, 0x08);
    write_cmos_sensor(0x380f, 0xb8);
    write_cmos_sensor(0x3811, 0x06);
    write_cmos_sensor(0x3813, 0x06);
    write_cmos_sensor(0x3814, 0x03);
    write_cmos_sensor(0x3816, 0x03);
    write_cmos_sensor(0x3817, 0x01);
    write_cmos_sensor(0x3820, 0x8b);
    write_cmos_sensor(0x3821, 0x01);
    write_cmos_sensor(0x4501, 0x00);
    write_cmos_sensor(0x4008, 0x02);
    write_cmos_sensor(0x4009, 0x09);
	  write_cmos_sensor(0x0100, 0x01);  // 
	#endif

	
}

static void capture_setting(kal_uint16 currefps)
{
  #if 0
	LOG_INF("OV5675CaptureSetting_2lane enter! currefps:%d\n",currefps);
	if (currefps == 150) { //15fps for PIP
	
		write_cmos_sensor(0x0100, 0x00);
		write_cmos_sensor(0x3662, 0x10);
		write_cmos_sensor(0x3714, 0x24);
		write_cmos_sensor(0x371a, 0x3e);
		write_cmos_sensor(0x37c2, 0x04);
		write_cmos_sensor(0x37d9, 0x08);
		write_cmos_sensor(0x3800, 0x00);
		write_cmos_sensor(0x3801, 0x00);
		write_cmos_sensor(0x3802, 0x00);
		write_cmos_sensor(0x3803, 0x04);
		write_cmos_sensor(0x3804, 0x0a);
		write_cmos_sensor(0x3805, 0x3f);
		write_cmos_sensor(0x3806, 0x07);
		write_cmos_sensor(0x3807, 0xb3);
		write_cmos_sensor(0x3808, 0x0a);
		write_cmos_sensor(0x3809, 0x20);
		write_cmos_sensor(0x380a, 0x07);
		write_cmos_sensor(0x380b, 0x98);
		write_cmos_sensor(0x380c, 0x02);
		write_cmos_sensor(0x380d, 0xee);
		write_cmos_sensor(0x380e, 0x0f);
		write_cmos_sensor(0x380f, 0xa0);
		write_cmos_sensor(0x3811, 0x10);
		write_cmos_sensor(0x3813, 0x0c);
		write_cmos_sensor(0x3814, 0x01);
		write_cmos_sensor(0x3815, 0x01);
		write_cmos_sensor(0x3816, 0x01);
		write_cmos_sensor(0x3820, 0x88);
		write_cmos_sensor(0x3821, 0x01);
		write_cmos_sensor(0x4008, 0x02);
		write_cmos_sensor(0x4009, 0x0d);
		write_cmos_sensor(0x4041, 0x07);
		write_cmos_sensor(0x0100, 0x01);
		
	} else{ // for 30fps need ti update
		write_cmos_sensor(0x0100, 0x00);
		write_cmos_sensor(0x3662, 0x10);
		write_cmos_sensor(0x3714, 0x24);
		write_cmos_sensor(0x371a, 0x3e);
		write_cmos_sensor(0x37c2, 0x04);
		write_cmos_sensor(0x37d9, 0x08);
		write_cmos_sensor(0x3800, 0x00);
		write_cmos_sensor(0x3801, 0x00);
		write_cmos_sensor(0x3802, 0x00);
		write_cmos_sensor(0x3803, 0x04);
		write_cmos_sensor(0x3804, 0x0a);
		write_cmos_sensor(0x3805, 0x3f);
		write_cmos_sensor(0x3806, 0x07);
		write_cmos_sensor(0x3807, 0xb3);
		write_cmos_sensor(0x3808, 0x0a);
		write_cmos_sensor(0x3809, 0x20);
		write_cmos_sensor(0x380a, 0x07);
		write_cmos_sensor(0x380b, 0x98);
		write_cmos_sensor(0x380c, 0x02);
		write_cmos_sensor(0x380d, 0xee);
		write_cmos_sensor(0x380e, 0x07);
		write_cmos_sensor(0x380f, 0xd0);
		write_cmos_sensor(0x3811, 0x10);
		write_cmos_sensor(0x3813, 0x0c);
		write_cmos_sensor(0x3814, 0x01);
		write_cmos_sensor(0x3815, 0x01);
		write_cmos_sensor(0x3816, 0x01);
		write_cmos_sensor(0x3820, 0x88);
		write_cmos_sensor(0x3821, 0x01);
		write_cmos_sensor(0x4008, 0x02);
		write_cmos_sensor(0x4009, 0x0d);
		write_cmos_sensor(0x4041, 0x07);
		write_cmos_sensor(0x0100, 0x01);
		
	} 
	
	#endif
		
}
static void normal_video_setting(kal_uint16 currefps)
{
	#if 0
	LOG_INF("normal_video_setting Enter! currefps:%d\n",currefps);
	//preview_setting();
	capture_setting(currefps);
	#endif
}

static void hs_video_setting(void)
{
	#if 0
	LOG_INF("hs_video_setting enter!\n");

//VGA 120fps
		write_cmos_sensor(0x0100, 0x00);
		write_cmos_sensor(0x3662, 0x08);
		write_cmos_sensor(0x3714, 0x24);
		write_cmos_sensor(0x371a, 0x3f);
		write_cmos_sensor(0x37c2, 0x24);
		write_cmos_sensor(0x37d9, 0x04);
		write_cmos_sensor(0x3800, 0x00);
		write_cmos_sensor(0x3801, 0x00);
		write_cmos_sensor(0x3802, 0x00);
		write_cmos_sensor(0x3803, 0x10);
		write_cmos_sensor(0x3804, 0x0a);
		write_cmos_sensor(0x3805, 0x3f);
		write_cmos_sensor(0x3806, 0x07);
		write_cmos_sensor(0x3807, 0xaf);
		write_cmos_sensor(0x3808, 0x02);
		write_cmos_sensor(0x3809, 0x80);
		write_cmos_sensor(0x380a, 0x01);
		write_cmos_sensor(0x380b, 0xe0);
		write_cmos_sensor(0x380c, 0x02);
		write_cmos_sensor(0x380d, 0xee);
		write_cmos_sensor(0x380e, 0x01);
		write_cmos_sensor(0x380f, 0xf4);
		write_cmos_sensor(0x3811, 0x08);
		write_cmos_sensor(0x3813, 0x02);
		write_cmos_sensor(0x3814, 0x07);
		write_cmos_sensor(0x3815, 0x01);
		write_cmos_sensor(0x3816, 0x07);
		write_cmos_sensor(0x3820, 0x8d);
		write_cmos_sensor(0x3821, 0x00);
		write_cmos_sensor(0x4008, 0x00);
		write_cmos_sensor(0x4009, 0x03);
		write_cmos_sensor(0x4041, 0x03);
		write_cmos_sensor(0x0100, 0x01);
		
	#endif
		
}

static void slim_video_setting(void)
{
	#if 0
	LOG_INF("slim_video_setting enter!\n");
		preview_setting();
	#endif
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	printk("OV5675_enable: %d\n", enable);

	write_cmos_sensor(0xfe, 0x01);
	if (enable)
		write_cmos_sensor(0x8c, 0x11);
	else
		write_cmos_sensor(0x8c, 0x10);
	write_cmos_sensor(0xfe, 0x00);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}

static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;

	printk("OV5675_E\n");

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
			if (*sensor_id == imgsensor_info.sensor_id) {
				//ov5675_otp_identify();
				printk("OV5675_i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, *sensor_id);
				printk("ov5675_i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, *sensor_id);
				return ERROR_NONE;
			}
			printk("OV5675_Read sensor id fail, write id: 0x%x, id: 0x%x\n",
				imgsensor.i2c_write_id, *sensor_id);
			printk("ov5675_Read sensor id fail, write id: 0x%x, id: 0x%x\n",
				imgsensor.i2c_write_id, *sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}

	printk("OV5675_sensor_id: %d\n", *sensor_id);

	if (*sensor_id != imgsensor_info.sensor_id) {
		/*if Sensor ID is not correct, set *sensor_id to 0xFFFFFFFF*/
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}

	return ERROR_NONE;
}

static kal_uint32 open(void)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint32 sensor_id = 0;

	LOG_1;
	printk("OV5675_ov5675_open\n");

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {
				pr_debug("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
				printk("OV5675_i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
				break;
			}
			pr_debug("Read sensor id fail, write id: 0x%x, id: 0x%x\n",
				imgsensor.i2c_write_id, sensor_id);
				printk("OV5675_Read sensor id fail, write id: 0x%x, id: 0x%x\n",
				imgsensor.i2c_write_id, sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail sequence write in  */
	sensor_init();

	//ov5675_otp_function();

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x2D00;
	imgsensor.gain = 0x100;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 close(void)
{
	printk("OV5675_E\n");
	/* No Need to implement this function */
	return ERROR_NONE;
}


static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk("OV5675_E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	/* imgsensor.video_mode = KAL_FALSE; */
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();

	/* set_mirror_flip(sensor_config_data->SensorImageMirror); */
	return ERROR_NONE;
}


static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk("OV5675_E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else {
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
			printk("OV5675_Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
				imgsensor.current_fps,
				imgsensor_info.cap.max_framerate / 10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps);
	return ERROR_NONE;
}

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk("OV5675_E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	normal_video_setting(imgsensor.current_fps);
	/* set_mirror_flip(sensor_config_data->SensorImageMirror); */
	return ERROR_NONE;
}

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk("OV5675_E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	return ERROR_NONE;
}

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk("OV5675_E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	return ERROR_NONE;
}

static kal_uint32 get_resolution(
		MSDK_SENSOR_RESOLUTION_INFO_STRUCT * sensor_resolution)
{
	printk("OV5675_E\n");
	sensor_resolution->SensorFullWidth =
		imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight =
		imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth =
		imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight =
		imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth =
		imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight =
		imgsensor_info.normal_video.grabwindow_height;

	sensor_resolution->SensorHighSpeedVideoWidth =
		imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight =
		imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth =
		imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight =
		imgsensor_info.slim_video.grabwindow_height;

	return ERROR_NONE;
}

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
	MSDK_SENSOR_INFO_STRUCT *sensor_info,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk("OV5675_scenario_id = %d\n", scenario_id);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4;
	sensor_info->SensorResetActiveHigh = FALSE;
	sensor_info->SensorResetDelayCount = 5;

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat =
		imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame =
		imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame =
		imgsensor_info.slim_video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0;
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	/*The frame of setting shutter default 0 for TG int*/
	sensor_info->AESensorGainDelayFrame =
		imgsensor_info.ae_sensor_gain_delay_frame;
	/*The frame of setting sensor gain*/
	sensor_info->AEISPGainDelayFrame =
		imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3;
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2;
	sensor_info->SensorPixelClockCount = 3;
	sensor_info->SensorDataLatchCount = 2;

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;
	sensor_info->SensorHightSampling = 0;
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.cap.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		sensor_info->SensorGrabStartX =
			imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY =
			imgsensor_info.normal_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		sensor_info->SensorGrabStartX =
			imgsensor_info.hs_video.startx;
		sensor_info->SensorGrabStartY =
			imgsensor_info.hs_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		sensor_info->SensorGrabStartX =
			imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY =
			imgsensor_info.slim_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;
		break;
	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk("OV5675_scenario_id = %d\n", scenario_id);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		preview(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		capture(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		normal_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		hs_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		slim_video(image_window, sensor_config_data);
		break;
	default:
		printk("OV5675_Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}
static kal_uint32 set_video_mode(UINT16 framerate)
{
	/*This Function not used after ROME*/
	printk("OV5675_framerate = %d\n ", framerate);

	/* SetVideoMode Function should fix framerate */
	/***********
	 *if (framerate == 0)	 //Dynamic frame rate
	 *	return ERROR_NONE;
	 *spin_lock(&imgsensor_drv_lock);
	 *if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
	 *	imgsensor.current_fps = 296;
	 *else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
	 *	imgsensor.current_fps = 146;
	 *else
	 *	imgsensor.current_fps = framerate;
	 *spin_unlock(&imgsensor_drv_lock);
	 *set_max_framerate(imgsensor.current_fps, 1);
	 ********/
	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	printk("OV5675_enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) /* enable auto flicker */
		imgsensor.autoflicker_en = KAL_TRUE;
	else        /* Cancel Auto flick */
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}

static kal_uint32 set_max_framerate_by_scenario(
	enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	printk("OV5675_scenario_id = %d, framerate = %d\n",
			scenario_id, framerate);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length = imgsensor_info.pre.pclk / framerate * 10 /
			imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length >
				imgsensor_info.pre.framelength) ?
			(frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength +
			imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length = imgsensor_info.normal_video.pclk /
			framerate * 10 /
			imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length >
				imgsensor_info.normal_video.framelength) ?
			(frame_length -
			 imgsensor_info.normal_video.framelength) : 0;
		imgsensor.frame_length =
			imgsensor_info.normal_video.framelength +
			imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (imgsensor.current_fps ==
				imgsensor_info.cap1.max_framerate) {
			frame_length = imgsensor_info.cap1.pclk /
				framerate * 10 / imgsensor_info.cap1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length >
					imgsensor_info.cap1.framelength) ?
				(frame_length -
				 imgsensor_info.cap1.framelength) : 0;
			imgsensor.frame_length =
				imgsensor_info.cap1.framelength +
				imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		} else {
			if (imgsensor.current_fps !=
					imgsensor_info.cap.max_framerate)

				printk("OV5675_Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
					framerate,
					imgsensor_info.cap.max_framerate / 10);
			frame_length = imgsensor_info.cap.pclk /
				framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length >
					imgsensor_info.cap.framelength) ?
				(frame_length -
				 imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length =
				imgsensor_info.cap.framelength +
				imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		}
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk /
			framerate * 10 / imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length >
				imgsensor_info.hs_video.framelength) ?
			(frame_length -
			 imgsensor_info.hs_video.framelength) : 0;
		imgsensor.frame_length =
			imgsensor_info.hs_video.framelength +
			imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk /
			framerate * 10 /
			imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length >
				imgsensor_info.slim_video.framelength) ?
			(frame_length -
			 imgsensor_info.slim_video.framelength) : 0;
		imgsensor.frame_length =
			imgsensor_info.slim_video.framelength +
			imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	default:  /*coding with  preview scenario by default*/
		frame_length = imgsensor_info.pre.pclk / framerate * 10 /
			imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length >
				imgsensor_info.pre.framelength) ?
			(frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength +
			imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		printk("OV5675_error scenario_id = %d, we use preview scenario\n",
			scenario_id);
		break;
	}
	return ERROR_NONE;
}

static kal_uint32 get_default_framerate_by_scenario(
	enum MSDK_SCENARIO_ID_ENUM scenario_id,
	MUINT32 *framerate)
{
	printk("OV5675_scenario_id = %d\n", scenario_id);

	if (framerate == NULL) {
		printk("OV5675_---- framerate is NULL ---- check here\n");
		return ERROR_NONE;
	}
	spin_lock(&imgsensor_drv_lock);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		*framerate = imgsensor_info.pre.max_framerate;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*framerate = imgsensor_info.normal_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*framerate = imgsensor_info.cap.max_framerate;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		*framerate = imgsensor_info.hs_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		*framerate = imgsensor_info.slim_video.max_framerate;
		break;
	default:
		break;
	}
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}


static kal_uint32 streaming_control(kal_bool enable)
{
	printk("OV5675_streaming_enable(0=Sw Standby,1=streaming): %d\n",
		enable);

	if (enable)
		write_cmos_sensor(0x0100, 0x01);
	else
		write_cmos_sensor(0x0100, 0x00);

	mdelay(10);
	return ERROR_NONE;
}

static kal_uint32 feature_control(
	MSDK_SENSOR_FEATURE_ENUM feature_id,
	UINT8 *feature_para,
	UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *)feature_para;

	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
		(MSDK_SENSOR_REG_INFO_STRUCT *)feature_para;

	if (feature_para == NULL) {
		printk("OV5675_ ---- %s ---- error params1\n", __func__);
		return ERROR_NONE;
	}

	printk("OV5675_feature_id = %d\n", feature_id);

	switch (feature_id) {
    // huangjiwu for  captrue black --begin	
	case SENSOR_FEATURE_GET_AWB_REQ_BY_SCENARIO:	
		switch (*feature_data) {	
            case MSDK_SCENARIO_ID_CUSTOM3:			
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 1;			
                break;		
            default:			
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;			
                break;		
		}		
		break;
	case SENSOR_FEATURE_GET_GAIN_RANGE_BY_SCENARIO:		
		*(feature_data + 1) = imgsensor_info.min_gain;		
		*(feature_data + 2) = imgsensor_info.max_gain;		
		break;
	case SENSOR_FEATURE_GET_BASE_GAIN_ISO_AND_STEP:		
		*(feature_data + 0) = imgsensor_info.min_gain_iso;		
		*(feature_data + 1) = imgsensor_info.gain_step;		
		*(feature_data + 2) = imgsensor_info.gain_type;		
		break;
	case SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO:		
		*(feature_data + 1) = imgsensor_info.min_shutter;		
		*(feature_data + 2) = imgsensor_info.exp_step;		
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.cap.pclk;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.normal_video.pclk;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.hs_video.pclk;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.slim_video.pclk;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.pclk;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.cap.framelength << 16)
				+ imgsensor_info.cap.linelength;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.normal_video.framelength << 16)
				+ imgsensor_info.normal_video.linelength;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.hs_video.framelength << 16)
				+ imgsensor_info.hs_video.linelength;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.slim_video.framelength << 16)
				+ imgsensor_info.slim_video.linelength;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.pre.framelength << 16)
				+ imgsensor_info.pre.linelength;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_BINNING_TYPE:		
		switch (*(feature_data + 1)) 
		{
            case MSDK_SCENARIO_ID_CUSTOM3:			
                *feature_return_para_32 = 1; /*BINNING_NONE*/			
                break;		
            case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:		
            case MSDK_SCENARIO_ID_VIDEO_PREVIEW:		
            case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:		
            case MSDK_SCENARIO_ID_SLIM_VIDEO:		
            case MSDK_SCENARIO_ID_CAMERA_PREVIEW:		
            case MSDK_SCENARIO_ID_CUSTOM4:		
            default:			
                *feature_return_para_32 = 1; /*BINNING_AVERAGED*/			
                break;
		}		
		pr_debug("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",*feature_return_para_32);		
		*feature_para_len = 4;		
		break;
    // huangjiwu for  captrue black --end	
	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		printk("OV5675_feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n",
			imgsensor.pclk, imgsensor.current_fps);
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		night_mode((BOOL) * feature_data);
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor(sensor_reg_data->RegAddr,
				sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData =
			read_cmos_sensor(sensor_reg_data->RegAddr);
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode((BOOL) * feature_data_16,
			*(feature_data_16 + 1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario(
			(enum MSDK_SCENARIO_ID_ENUM) *feature_data,
			*(feature_data + 1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario(
			(enum MSDK_SCENARIO_ID_ENUM) *(feature_data),
			(MUINT32 *) (uintptr_t) (*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL) * feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		printk("OV5675_current fps :%d\n", (UINT32) *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = *feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_HDR:
		printk("OV5675_ihdr enable :%d\n", (BOOL) * feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.ihdr_en = (BOOL) * feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		printk("OV5675_SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
			(UINT32) *feature_data);

		wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)
			(uintptr_t) (*(feature_data + 1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[1],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[2],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[3],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[4],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[0],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		printk("OV5675_SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
			(UINT16) *feature_data, (UINT16) *(feature_data + 1),
			(UINT16) *(feature_data + 2));
		ihdr_write_shutter_gain((UINT16) *feature_data,
			(UINT16) *(feature_data + 1),
			(UINT16) *(feature_data + 2));
		break;
		case SENSOR_FEATURE_GET_PIXEL_RATE:
			switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				(imgsensor_info.cap.pclk /
				(imgsensor_info.cap.linelength - 80))*
				imgsensor_info.cap.grabwindow_width;
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				(imgsensor_info.normal_video.pclk /
				(imgsensor_info.normal_video.linelength - 80))*
				imgsensor_info.normal_video.grabwindow_width;
				break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				(imgsensor_info.hs_video.pclk /
				(imgsensor_info.hs_video.linelength - 80))*
				imgsensor_info.hs_video.grabwindow_width;
				break;
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				(imgsensor_info.slim_video.pclk /
				(imgsensor_info.slim_video.linelength - 80))*
				imgsensor_info.slim_video.grabwindow_width;
				break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			default:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				(imgsensor_info.pre.pclk /
				(imgsensor_info.pre.linelength - 80))*
				imgsensor_info.pre.grabwindow_width;
			break;
		}
		//*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = rate;
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		printk("OV5675_SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		printk("OV5675_SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n",
			*feature_data);
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:			
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =				
					imgsensor_info.cap.mipi_pixel_rate; 		
				break;

		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.normal_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.hs_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.slim_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.pre.mipi_pixel_rate;
			break;
	}
		break;
	default:
		break;
	}

	return ERROR_NONE;
}

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 OV5675_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	printk("OV5675_E\n");
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;

	return ERROR_NONE;
}
