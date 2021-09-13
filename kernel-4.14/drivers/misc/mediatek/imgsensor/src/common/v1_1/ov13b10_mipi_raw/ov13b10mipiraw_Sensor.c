/*****************************************************************************
 *
 * Filename:
 * ---------
 *   OV13B10_REAR_rear_mipiraw_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * [201501] PDAF MP Version 
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "ov13b10mipiraw_Sensor.h"
//#include "ov13b10_eeprom.h"

#define PFX "OV13B10_camera_sensor"
#define LOG_1 LOG_INF("OV13B10,MIPI 4LANE,PDAF\n")
#define LOG_2 LOG_INF("preview 2336*1752@30fps,768Mbps/lane; video 4672*3504@30fps,1440Mbps/lane; capture 16M@30fps,1440Mbps/lane\n")
#define LOG_INF(fmt, args...)   pr_debug(PFX "[%s] " fmt, __FUNCTION__, ##args)
#define LOGE(format, args...)   pr_err("[%s] " format, __FUNCTION__, ##args)
#define MULTI_WRITE 1
//prize fengshangdong modify at 20190524
static DEFINE_SPINLOCK(imgsensor_drv_lock);
extern int curr_sensor_id;
static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = OV13B10_SENSOR_ID,
	.checksum_value = 0x8b86a64,//test_Pattern_mode

	.pre = {
        .pclk = 112000000,
		.linelength =  1176,
		.framelength = 3174,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2104,
		.grabwindow_height = 1560,
		.mipi_data_lp2hs_settle_dc = 85,	//unit(ns), 16/23/65/85 recommanded
		.max_framerate = 300,
		.mipi_pixel_rate =  302400000,

	},
	
	.cap = {
		.pclk = 112000000,
		.linelength = 1176,
		.framelength = 3196,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4208,
		.grabwindow_height = 3120,
		.mipi_data_lp2hs_settle_dc = 19,	//unit(ns), 16/23/65/85 recommanded
		.max_framerate = 300,	//29.800fps
		.mipi_pixel_rate = 441600000,
	},
		
	.cap1 = {
		.pclk = 112000000,
		.linelength = 1176,
		.framelength = 3995,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4208,
		.grabwindow_height = 3120,
		.mipi_data_lp2hs_settle_dc = 19,	//unit(ns), 16/23/65/85 recommanded
		.max_framerate = 240,	//29.800fps
		.mipi_pixel_rate = 441600000,
	},		
	.cap2 = {
		.pclk = 112000000,
		.linelength = 1176,
		.framelength = 6392,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4208,
		.grabwindow_height = 3120,
		.mipi_data_lp2hs_settle_dc = 19,	//unit(ns), 16/23/65/85 recommanded
		.max_framerate = 150,	//29.800fps
		.mipi_pixel_rate = 441600000,
	},

	.normal_video = {
        .pclk = 112000000,
        .linelength = 1176,
        .framelength = 3174,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1920,
        .grabwindow_height = 1080,
        .mipi_data_lp2hs_settle_dc = 85,	//unit(ns), 16/23/65/85 recommanded
        .max_framerate = 300,	//29.996fps
		.mipi_pixel_rate =  302400000,

	},
	.hs_video = {
		.pclk = 112000000,
		.linelength = 1176,
		.framelength = 794,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 640,
		.grabwindow_height = 480,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 1200,	//119.947fps
		.mipi_pixel_rate =  302400000,
	},

	.slim_video = {
        .pclk = 112000000,
		.linelength =  1176,
		.framelength = 3174,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2104,
		.grabwindow_height = 1560,
		.mipi_data_lp2hs_settle_dc = 85,	//unit(ns), 16/23/65/85 recommanded
		.max_framerate = 300,
		.mipi_pixel_rate =  302400000,

	},
	.margin = 8,		/* sensor framelength & shutter margin check*/
	.min_shutter = 0x4,	//min shutter
	.max_frame_length = 0x7fff,	/* max framelength by sensor register's limitation */
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0,	/* 1, support; 0,not support */
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 7,//support sensor mode num
	.frame_time_delay_frame = 3,	/* The delay frame of setting frame length  */
	.cap_delay_frame = 3,
	.pre_delay_frame = 2,         //enter preview delay frame num
	.video_delay_frame = 2,        //enter video delay frame num
	.hs_video_delay_frame = 2,    //enter high speed video  delay frame num
	.slim_video_delay_frame = 2,//enter slim video delay frame num

	.isp_driving_current = ISP_DRIVING_4MA,   //8MA
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_CSI2,	/* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
	.mipi_settle_delay_mode = 1,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,//sensor output first pixel color
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x6c,0x20,0xff},
	.i2c_speed = 400,// i2c read/write speed
};

//prize end
static  struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_H_MIRROR,
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x3D0,
	.gain = 0x100,
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
	.current_fps = 300,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,		//test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.ihdr_en = 0,		/* sensor need support LE, SE with HDR feature */
	.i2c_write_id = 0x20,
};


//* Sensor output window information */
static  struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] ={
    {4224, 3136,  0,  0,  4224, 3136, 2112, 1568,   4,   4, 2104, 1560, 0, 0, 2104, 1560},    /* Preview */
    {4224, 3136,  0,  0,  4224, 3136, 4224, 3136,   8,   8, 4208, 3120, 0, 0, 4208, 3120},//{4224, 3136,  0,  0,  4224, 3136, 4224, 3136,   8,   8, 4208, 3120, 0, 0, 4208, 3120},    /* capture */
    {4224, 3136,  0,  0,  4224, 3136, 2112, 1568,  96, 244, 1920,  1080, 0, 0,1920, 1080},    /* video */
    {4224, 3136,  800,584, 2624, 1968,  656,  492,  8,   6,  640,  480, 0, 0,  640,  480}, /* hight speed video*/
    {4224, 3136,  0,  0,  4224, 3136, 2112, 1568,   4,   4, 2104, 1560, 0, 0, 2104, 1560},    /* slim video */
};// slim video

 	
 #if 	0
static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info =
{
    .i4OffsetX = 8,
    .i4OffsetY = 8,
    .i4PitchX  = 32,
    .i4PitchY  = 32,
    .i4PairNum  =8,
    .i4SubBlkW  =16,
    .i4SubBlkH  =8,
	 .i4BlockNumX = 131,
	 .i4BlockNumY = 97,
	.i4PosL = {
		{22, 14}, {38, 14}, {14, 18}, {30, 18}, {22, 30}, {38, 30}, {14, 34}, {30, 34}
	},
	.i4PosR = {
		{22, 10}, {38, 10}, {14, 22}, {30, 22}, {22, 26}, {38, 26}, {14, 38}, {30, 38}
	},
	.i4Crop = {
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}
	},
     .iMirrorFlip = 0,	/*0 IMAGE_NORMAL, 1 IMAGE_H_MIRROR, 2 IMAGE_V_MIRROR, 3 IMAGE_HV_MIRROR*/
};
#endif
#if MULTI_WRITE
#define I2C_BUFFER_LEN 225
#else
#define I2C_BUFFER_LEN 3
#endif

static kal_uint16 ov13b10_table_write_cmos_sensor(
					kal_uint16 *para, kal_uint32 len)
{
	char puSendCmd[I2C_BUFFER_LEN];
	kal_uint32 tosend, IDX;
	kal_uint16 addr = 0, addr_last = 0, data;

	tosend = 0;
	IDX = 0;
	while (len > IDX) {
		addr = para[IDX];

		{
			puSendCmd[tosend++] = (char)(addr >> 8);
			puSendCmd[tosend++] = (char)(addr & 0xFF);
			data = para[IDX + 1];
			puSendCmd[tosend++] = (char)(data & 0xFF);
			IDX += 2;
			addr_last = addr;

		}
#if MULTI_WRITE
		if ((I2C_BUFFER_LEN - tosend) < 3 ||
			len == IDX ||
			addr != addr_last) {
			iBurstWriteReg_multi(puSendCmd, tosend,
				imgsensor.i2c_write_id,
				3, imgsensor_info.i2c_speed);

			tosend = 0;
		}
#else
		iWriteRegI2C(puSendCmd, 3, imgsensor.i2c_write_id);
		tosend = 0;

#endif
	}
	return 0;
}
static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte, 2, imgsensor.i2c_write_id);
    return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);
}

static kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
    kal_uint16 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte,1,imgsensor.i2c_write_id);
    return get_byte;
}

static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para & 0xFF)};
    iWriteRegI2C(pusendcmd , 3, imgsensor.i2c_write_id);
}


static void set_dummy(void)
{
	 LOG_INF("dummyline = %d, dummypixels = %d ", imgsensor.dummy_line, imgsensor.dummy_pixel);
  //set vertical_total_size, means framelength
  	write_cmos_sensor_8(0x3208, 0x00);
	write_cmos_sensor_8(0x380c, imgsensor.line_length >> 8);
	write_cmos_sensor_8(0x380d, imgsensor.line_length & 0xFF);	 
	
	//set horizontal_total_size, means linelength 
	write_cmos_sensor_8(0x380e, imgsensor.frame_length >> 8);
	write_cmos_sensor_8(0x380f, imgsensor.frame_length & 0xFF);

}	/*	set_dummy  */


static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{

	kal_uint32 frame_length = imgsensor.frame_length;

	LOG_INF("framerate = %d, min framelength should enable = %d \n", framerate,min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */
//prize fengshangdong modify at 20190524 
static void write_shutter(kal_uint32 shutter)
{
	kal_uint16 realtime_fps = 0;
    spin_lock(&imgsensor_drv_lock);
    if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
        imgsensor.frame_length = shutter + imgsensor_info.margin;
    else
        imgsensor.frame_length = imgsensor.min_frame_length;
    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
        imgsensor.frame_length = imgsensor_info.max_frame_length;
    spin_unlock(&imgsensor_drv_lock);
	LOG_INF("Enter! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

			shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
			shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

			// Framelength should be an even number
			shutter = (shutter >> 1) << 1;
			imgsensor.frame_length = (imgsensor.frame_length >> 1) << 1;
			if (imgsensor.autoflicker_en) {
				realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
				if(realtime_fps >= 297 && realtime_fps <= 305)
					set_max_framerate(296,0);
				else if(realtime_fps >= 147 && realtime_fps <= 150)
					set_max_framerate(146,0);
				else {
					// Extend frame length
					imgsensor.frame_length = (imgsensor.frame_length >> 1) << 1;
					write_cmos_sensor_8(0x3208, 0x00);
					write_cmos_sensor_8(0x380e, (imgsensor.frame_length >> 8) & 0xFF);
					write_cmos_sensor_8(0x380f, imgsensor.frame_length & 0xFF);
				}
			} else {
				// Extend frame length
					imgsensor.frame_length = (imgsensor.frame_length >> 1) << 1;
				write_cmos_sensor_8(0x3208, 0x00);
				write_cmos_sensor_8(0x380e, (imgsensor.frame_length >> 8) & 0xFF);
				write_cmos_sensor_8(0x380f, imgsensor.frame_length & 0xFF);
			}	
			// Update Shutter
		write_cmos_sensor_8(0x3500, (shutter >> 16) & 0xFF);
		write_cmos_sensor_8(0x3501, (shutter >> 8) & 0xFF);
		write_cmos_sensor_8(0x3502, (shutter) & 0xFF);
		write_cmos_sensor_8(0x3208, 0x10);
		write_cmos_sensor_8(0x3208, 0xa0);

		LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);
		LOG_INF(" write_shutter 0x380c-0x380f valeu(0x%x 0x%x 0x%x 0x%x)",read_cmos_sensor_8(0x380c),read_cmos_sensor_8(0x380d),read_cmos_sensor_8(0x380e),read_cmos_sensor_8(0x380f));
		LOG_INF(" write_shutter 0x3500-0x3502 valeu(0x%x 0x%x 0x%x)",read_cmos_sensor_8(0x3500),read_cmos_sensor_8(0x3501),read_cmos_sensor_8(0x3502));

			
	
}
//prize end

/*************************************************************************
* FUNCTION
*	set_shutter
*
* DESCRIPTION
*	This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*	iShutter : exposured lines
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint32 shutter)
{
	unsigned long flags;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
}	/*	set_shutter */

static void set_shutter_frame_length(kal_uint16 shutter, kal_uint16 frame_length)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	//LOG_INF(" shutter =%d, framelength =%d", shutter,frame_length);
	//LOG_INF(" min_frame_length =%d, max_frame_length =%d, min_shutter=%d", imgsensor.min_frame_length,
						//imgsensor_info.max_frame_length, imgsensor_info.min_shutter);

	spin_lock(&imgsensor_drv_lock);
	/*Change frame time*/
	if(frame_length > 1)
		imgsensor.frame_length = frame_length;

	/* */
	if (shutter > imgsensor.frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	if (imgsensor.autoflicker_en) {
		
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);
		else {
		// Extend frame length
		 write_cmos_sensor_8(0x380e, imgsensor.frame_length >> 8);
		 write_cmos_sensor_8(0x380f, imgsensor.frame_length & 0xFF);
		}
	} else {
		// Extend frame length
		 write_cmos_sensor_8(0x380e, imgsensor.frame_length >> 8);
		 write_cmos_sensor_8(0x380f, imgsensor.frame_length & 0xFF);
	}

	// Update Shutter
		write_cmos_sensor_8(0x3822, 0x14);
		write_cmos_sensor_8(0x3500, (shutter >> 16) & 0xFF);
		write_cmos_sensor_8(0x3501, (shutter >> 8) & 0xFF);
		write_cmos_sensor_8(0x3502, (shutter) & 0xFF);

	//LOG_INF("Add for N3D! shutterlzl =%d, framelength =%d\n", shutter,imgsensor.frame_length);

}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 iReg = 0x0000;

	/* platform 1xgain = 64, sensor driver 1*gain = 0x80 */
	iReg = gain * 256 / BASEGAIN;

	if (iReg < 0x100) // sensor 1xGain
	{
		iReg = 0X100;
	}
	if (iReg > 0xF80) // sensor 15.5xGain
	{
		iReg = 0XF80;
	}
	return iReg;
}
/*************************************************************************
* FUNCTION
*	set_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	/*
	* sensor gain 1x = 0x10
	* max gain = 0xf8 = 15.5x
	*/
	kal_uint16 reg_gain = 0;
	
	reg_gain = gain2reg(gain);
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain; 
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor_8(0x3508, (reg_gain >> 8));
	write_cmos_sensor_8(0x3509, (reg_gain & 0xFF));
	return gain;
}	/*	set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n", le, se, gain);
}




/*************************************************************************
* FUNCTION
*	night_mode
*
* DESCRIPTION
*	This function night mode of sensor.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/

#if MULTI_WRITE
kal_uint16 addr_data_pair_init_ov13b10[] = {
	0x0103, 0x01,
	0x0303, 0x01,
	0x0305, 0x46,
	0x0321, 0x00,
	0x0323, 0x04,
	0x0324, 0x01,
	0x0325, 0x50,
	0x0326, 0x81,
	0x0327, 0x04,
	0x3012, 0x07,
	0x3013, 0x32,
	0x3107, 0x23,
	0x3501, 0x0c,
	0x3502, 0x10,
	0x3504, 0x08,
	0x3508, 0x07,
	0x3509, 0xc0,
	0x3600, 0x16,
	0x3601, 0x54,
	0x3612, 0x4e,
	0x3620, 0x00,
	0x3621, 0x68,
	0x3622, 0x66,
	0x3623, 0x03,
	0x3662, 0x92,
	0x3666, 0xbb,
	0x3667, 0x44,
	0x366e, 0xff,
	0x366f, 0xf3,
	0x3675, 0x44,
	0x3676, 0x00,
	0x367f, 0xe9,
	0x3681, 0x32,
	0x3682, 0x1f,
	0x3683, 0x0b,
	0x3684, 0x0b,
	0x3704, 0x0f,
	0x3706, 0x40,
	0x3708, 0x3b,
	0x3709, 0x72,
	0x370b, 0xa2,
	0x3714, 0x24,
	0x371a, 0x3e,
	0x3725, 0x42,
	0x3739, 0x12,
	0x3767, 0x00,
	0x377a, 0x0d,
	0x3789, 0x18,
	0x3790, 0x40,
	0x3791, 0xa2,
	0x37c2, 0x04,
	0x37c3, 0xf1,
	0x37d9, 0x0c,
	0x37da, 0x02,
	0x37dc, 0x02,
	0x37e1, 0x04,
	0x37e2, 0x0a,
	0x3800, 0x00,
	0x3801, 0x00,
	0x3802, 0x00,
	0x3803, 0x08,
	0x3804, 0x10,
	0x3805, 0x8f,
	0x3806, 0x0c,
	0x3807, 0x47,
	0x3808, 0x10,
	0x3809, 0x70,
	0x380a, 0x0c,
	0x380b, 0x30,
	0x380c, 0x04,
	0x380d, 0x98,
	0x380e, 0x0c,
	0x380f, 0x7c,
	0x3811, 0x0f,
	0x3813, 0x08,
	0x3814, 0x01,
	0x3815, 0x01,
	0x3816, 0x01,
	0x3817, 0x01,
	0x381f, 0x08,
	0x3820, 0x88,
	0x3821, 0x00,
	0x3822, 0x04,
	0x382e, 0xe6,
	0x3c80, 0x00,
	0x3c87, 0x01,
	0x3c8c, 0x19,
	0x3c8d, 0x1c,
	0x3ca0, 0x00,
	0x3ca1, 0x00,
	0x3ca2, 0x00,
	0x3ca3, 0x00,
	0x3ca4, 0x50,
	0x3ca5, 0x11,
	0x3ca6, 0x01,
	0x3ca7, 0x00,
	0x3ca8, 0x00,
	0x4008, 0x02,
	0x4009, 0x0f,
	0x400a, 0x01,
	0x400b, 0x19,
	0x4011, 0x21,
	0x4017, 0x08,
	0x4019, 0x04,
	0x401a, 0x58,
	0x4032, 0x1e,
	0x4050, 0x02,
	0x4051, 0x09,
	0x405e, 0x00,
	0x4066, 0x02,
	0x4501, 0x00,
	0x4502, 0x10,
	0x4505, 0x00,
	0x4800, 0x64,
	0x481b, 0x3e,
	0x481f, 0x30,
	0x4825, 0x34,
	0x4837, 0x0e,
	0x484b, 0x01,
	0x4883, 0x02,
	0x5000, 0xff,
	0x5001, 0x0f,
	0x5045, 0x20,
	0x5046, 0x20,
	0x5047, 0xa4,
	0x5048, 0x20,
	0x5049, 0xa4,
	0x4885, 0x0f,
	//0x0100, 0x00	//stream off
};
#endif

static void sensor_init(void)
{
      LOG_INF("sensor_init entry");
	write_cmos_sensor_8(0x0103, 0x01);//SW Reset, need delay
	mdelay(10);
#if MULTI_WRITE
	ov13b10_table_write_cmos_sensor(
		addr_data_pair_init_ov13b10,
		sizeof(addr_data_pair_init_ov13b10) / sizeof(kal_uint16));
#else
	write_cmos_sensor_8(0x0303, 0x01);
	write_cmos_sensor_8(0x0305, 0x46);
	write_cmos_sensor_8(0x0321, 0x00);
	write_cmos_sensor_8(0x0323, 0x04);
	write_cmos_sensor_8(0x0324, 0x01);
	write_cmos_sensor_8(0x0325, 0x50);
	write_cmos_sensor_8(0x0326, 0x81);
	write_cmos_sensor_8(0x0327, 0x04);
	write_cmos_sensor_8(0x3012, 0x07);
	write_cmos_sensor_8(0x3013, 0x32);
	write_cmos_sensor_8(0x3107, 0x23);
	write_cmos_sensor_8(0x3501, 0x0c);
	write_cmos_sensor_8(0x3502, 0x10);
	write_cmos_sensor_8(0x3504, 0x08);
	write_cmos_sensor_8(0x3508, 0x07);
	write_cmos_sensor_8(0x3509, 0xc0);
	write_cmos_sensor_8(0x3600, 0x16);
	write_cmos_sensor_8(0x3601, 0x54);
	write_cmos_sensor_8(0x3612, 0x4e);
	write_cmos_sensor_8(0x3620, 0x00);
	write_cmos_sensor_8(0x3621, 0x68);
	write_cmos_sensor_8(0x3622, 0x66);
	write_cmos_sensor_8(0x3623, 0x03);
	write_cmos_sensor_8(0x3662, 0x92);
	write_cmos_sensor_8(0x3666, 0xbb);
	write_cmos_sensor_8(0x3667, 0x44);
	write_cmos_sensor_8(0x366e, 0xff);
	write_cmos_sensor_8(0x366f, 0xf3);
	write_cmos_sensor_8(0x3675, 0x44);
	write_cmos_sensor_8(0x3676, 0x00);
	write_cmos_sensor_8(0x367f, 0xe9);
	write_cmos_sensor_8(0x3681, 0x32);
	write_cmos_sensor_8(0x3682, 0x1f);
	write_cmos_sensor_8(0x3683, 0x0b);
	write_cmos_sensor_8(0x3684, 0x0b);
	write_cmos_sensor_8(0x3704, 0x0f);
	write_cmos_sensor_8(0x3706, 0x40);
	write_cmos_sensor_8(0x3708, 0x3b);
	write_cmos_sensor_8(0x3709, 0x72);
	write_cmos_sensor_8(0x370b, 0xa2);
	write_cmos_sensor_8(0x3714, 0x24);
	write_cmos_sensor_8(0x371a, 0x3e);
	write_cmos_sensor_8(0x3725, 0x42);
	write_cmos_sensor_8(0x3739, 0x12);
	write_cmos_sensor_8(0x3767, 0x00);
	write_cmos_sensor_8(0x377a, 0x0d);
	write_cmos_sensor_8(0x3789, 0x18);
	write_cmos_sensor_8(0x3790, 0x40);
	write_cmos_sensor_8(0x3791, 0xa2);
	write_cmos_sensor_8(0x37c2, 0x04);
	write_cmos_sensor_8(0x37c3, 0xf1);
	write_cmos_sensor_8(0x37d9, 0x0c);
	write_cmos_sensor_8(0x37da, 0x02);
	write_cmos_sensor_8(0x37dc, 0x02);
	write_cmos_sensor_8(0x37e1, 0x04);
	write_cmos_sensor_8(0x37e2, 0x0a);
	write_cmos_sensor_8(0x3800, 0x00);
	write_cmos_sensor_8(0x3801, 0x00);
	write_cmos_sensor_8(0x3802, 0x00);
	write_cmos_sensor_8(0x3803, 0x08);
	write_cmos_sensor_8(0x3804, 0x10);
	write_cmos_sensor_8(0x3805, 0x8f);
	write_cmos_sensor_8(0x3806, 0x0c);
	write_cmos_sensor_8(0x3807, 0x47);
	write_cmos_sensor_8(0x3808, 0x10);
	write_cmos_sensor_8(0x3809, 0x70);
	write_cmos_sensor_8(0x380a, 0x0c);
	write_cmos_sensor_8(0x380b, 0x30);
	write_cmos_sensor_8(0x380c, 0x04);
	write_cmos_sensor_8(0x380d, 0x98);
	write_cmos_sensor_8(0x380e, 0x0c);
	write_cmos_sensor_8(0x380f, 0x7c);
	write_cmos_sensor_8(0x3811, 0x0f);
	write_cmos_sensor_8(0x3813, 0x08);
	write_cmos_sensor_8(0x3814, 0x01);
	write_cmos_sensor_8(0x3815, 0x01);
	write_cmos_sensor_8(0x3816, 0x01);
	write_cmos_sensor_8(0x3817, 0x01);
	write_cmos_sensor_8(0x381f, 0x08);
	write_cmos_sensor_8(0x3820, 0x88);
	write_cmos_sensor_8(0x3821, 0x00);
	write_cmos_sensor_8(0x3822, 0x04);
	write_cmos_sensor_8(0x382e, 0xe6);
	write_cmos_sensor_8(0x3c80, 0x00);
	write_cmos_sensor_8(0x3c87, 0x01);
	write_cmos_sensor_8(0x3c8c, 0x19);
	write_cmos_sensor_8(0x3c8d, 0x1c);
	write_cmos_sensor_8(0x3ca0, 0x00);
	write_cmos_sensor_8(0x3ca1, 0x00);
	write_cmos_sensor_8(0x3ca2, 0x00);
	write_cmos_sensor_8(0x3ca3, 0x00);
	write_cmos_sensor_8(0x3ca4, 0x50);
	write_cmos_sensor_8(0x3ca5, 0x11);
	write_cmos_sensor_8(0x3ca6, 0x01);
	write_cmos_sensor_8(0x3ca7, 0x00);
	write_cmos_sensor_8(0x3ca8, 0x00);
	write_cmos_sensor_8(0x4008, 0x02);
	write_cmos_sensor_8(0x4009, 0x0f);
	write_cmos_sensor_8(0x400a, 0x01);
	write_cmos_sensor_8(0x400b, 0x19);
	write_cmos_sensor_8(0x4011, 0x21);
	write_cmos_sensor_8(0x4017, 0x08);
	write_cmos_sensor_8(0x4019, 0x04);
	write_cmos_sensor_8(0x401a, 0x58);
	write_cmos_sensor_8(0x4032, 0x1e);
	write_cmos_sensor_8(0x4050, 0x02);
	write_cmos_sensor_8(0x4051, 0x09);
	write_cmos_sensor_8(0x405e, 0x00);
	write_cmos_sensor_8(0x4066, 0x02);
	write_cmos_sensor_8(0x4501, 0x00);
	write_cmos_sensor_8(0x4502, 0x10);
	write_cmos_sensor_8(0x4505, 0x00);
	write_cmos_sensor_8(0x4800, 0x64);
	write_cmos_sensor_8(0x481b, 0x3e);
	write_cmos_sensor_8(0x481f, 0x30);
	write_cmos_sensor_8(0x4825, 0x34);
	write_cmos_sensor_8(0x4837, 0x0e);
	write_cmos_sensor_8(0x484b, 0x01);
	write_cmos_sensor_8(0x4883, 0x02);
	write_cmos_sensor_8(0x5000, 0xff);
	write_cmos_sensor_8(0x5001, 0x0f);
	write_cmos_sensor_8(0x5045, 0x20);
	write_cmos_sensor_8(0x5046, 0x20);
	write_cmos_sensor_8(0x5047, 0xa4);
	write_cmos_sensor_8(0x5048, 0x20);
	write_cmos_sensor_8(0x5049, 0xa4);
	write_cmos_sensor_8(0x4885, 0x0f);
	//write_cmos_sensor_8(0x0100, 0x00);
#endif
	LOG_INF("preview 0x380c-0x380f valeu(0x%x 0x%x 0x%x 0x%x)",read_cmos_sensor_8(0x380c),read_cmos_sensor_8(0x380d),read_cmos_sensor_8(0x380e),read_cmos_sensor_8(0x380f));
}
static kal_uint32 streaming_control(kal_bool enable)
{
	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
	if (enable)
		write_cmos_sensor_8(0x0100, 0X01);
	else
		write_cmos_sensor_8(0x0100, 0x00);
	mdelay(30);
	return ERROR_NONE;
}
#if MULTI_WRITE
kal_uint16 addr_data_pair_preview_ov13b10[] = {
	//0x0100, 0x00,
	0x0305, 0x24,
	0x3501, 0x06,
	0x3502, 0x10,
	0x3662, 0x88,
	0x3714, 0x28,
	0x371a, 0x3e,
	0x3739, 0x10,
	0x37c2, 0x14,
	0x37d9, 0x06,
	0x3800, 0x00,
	0x3801, 0x00,
	0x3802, 0x00,
	0x3803, 0x08,
	0x3804, 0x10,
	0x3805, 0x8f,
	0x3806, 0x0c,
	0x3807, 0x47,
	0x3808, 0x08,
	0x3809, 0x38,
	0x380a, 0x06,
	0x380b, 0x18,
	0x380c, 0x04,
	0x380d, 0x98,
	0x380e, 0x0c,
	0x380f, 0x66,
	0x3811, 0x07,
	0x3813, 0x04,
	0x3814, 0x03,
	0x3815, 0x01,
	0x3816, 0x03,
	0x3817, 0x01,
	0x3820, 0x8b,
	0x3c8c, 0x18,
	0x4008, 0x00,
	0x4009, 0x05,
	0x4050, 0x00,
	0x4051, 0x05,
	0x4501, 0x08,
	0x4505, 0x04,
	0x4837, 0x1C,
	0x5000, 0xfd,
	0x5001, 0x0d,

	//0x0100, 0x01

};
#endif

static void preview_setting(void)
{
	/*
	* @@ OV16A10 4lane 2304x1728 30fps hvbin
	* ; Sysclk 160Mhz, MIPI4_768Mbps/Lane, 30Fps.
    *;Line_length =1400, Frame_length =3808
    *; V-blank:18ms
	*/
	
	//write_cmos_sensor_8(0x0100, 0x00);
#if MULTI_WRITE
	ov13b10_table_write_cmos_sensor(
		addr_data_pair_preview_ov13b10,
		sizeof(addr_data_pair_preview_ov13b10) / sizeof(kal_uint16));
#else	
	write_cmos_sensor_8(0x0305, 0x24);
	write_cmos_sensor_8(0x3501, 0x06);
	write_cmos_sensor_8(0x3502, 0x10);
	write_cmos_sensor_8(0x3662, 0x88);
	write_cmos_sensor_8(0x3714, 0x28);
	write_cmos_sensor_8(0x371a, 0x3e);
	write_cmos_sensor_8(0x3739, 0x10);
	write_cmos_sensor_8(0x37c2, 0x14);
	write_cmos_sensor_8(0x37d9, 0x06);
	write_cmos_sensor_8(0x3800, 0x00);
	write_cmos_sensor_8(0x3801, 0x00);
	write_cmos_sensor_8(0x3802, 0x00);
	write_cmos_sensor_8(0x3803, 0x08);
	write_cmos_sensor_8(0x3804, 0x10);
	write_cmos_sensor_8(0x3805, 0x8f);
	write_cmos_sensor_8(0x3806, 0x0c);
	write_cmos_sensor_8(0x3807, 0x47);
	write_cmos_sensor_8(0x3808, 0x08);
	write_cmos_sensor_8(0x3809, 0x38);
	write_cmos_sensor_8(0x380a, 0x06);
	write_cmos_sensor_8(0x380b, 0x18);
	write_cmos_sensor_8(0x380c, 0x04);
	write_cmos_sensor_8(0x380d, 0x98);
	write_cmos_sensor_8(0x380e, 0x0c);
	write_cmos_sensor_8(0x380f, 0x66);
	write_cmos_sensor_8(0x3811, 0x07);
	write_cmos_sensor_8(0x3813, 0x04);
	write_cmos_sensor_8(0x3814, 0x03);
	write_cmos_sensor_8(0x3815, 0x01);
	write_cmos_sensor_8(0x3816, 0x03);
	write_cmos_sensor_8(0x3817, 0x01);
	write_cmos_sensor_8(0x3820, 0x8b);
	write_cmos_sensor_8(0x3c8c, 0x18);
	write_cmos_sensor_8(0x4008, 0x00);
	write_cmos_sensor_8(0x4009, 0x05);
	write_cmos_sensor_8(0x4050, 0x00);
	write_cmos_sensor_8(0x4051, 0x05);
	write_cmos_sensor_8(0x4501, 0x08);
	write_cmos_sensor_8(0x4505, 0x04);
	write_cmos_sensor_8(0x4837, 0x1C);
	write_cmos_sensor_8(0x5000, 0xfd);
	write_cmos_sensor_8(0x5001, 0x0d);
#endif
}
#if MULTI_WRITE
kal_uint16 addr_data_pair_capture_15fps_ov13b10[] = {
//	0x0100, 0x00,
    0x0305, 0x45,
    0x3501, 0x0c,
    0x3502, 0x10,
    0x3662, 0x92,
    0x3714, 0x24,
    0x371a, 0x3e,
    0x3739, 0x12,
    0x37c2, 0x04,
    0x37d9, 0x0c,
    0x3800, 0x00,
    0x3801, 0x00,
    0x3802, 0x00,
    0x3803, 0x08,
    0x3804, 0x10,
    0x3805, 0x8f,
    0x3806, 0x0c,
    0x3807, 0x47,
    0x3808, 0x10,
    0x3809, 0x70,
    0x380a, 0x0c,
    0x380b, 0x30,
    0x380c, 0x04,
    0x380d, 0x98,
    0x380e, 0x18,
    0x380f, 0xf8,
    0x3811, 0x0f,
    0x3813, 0x08,
    0x3814, 0x01,
    0x3815, 0x01,
    0x3816, 0x01,
    0x3817, 0x01,
    0x3820, 0x88,
    0x3c8c, 0x19,
    0x4008, 0x02,
    0x4009, 0x0f,
    0x4050, 0x02,
    0x4051, 0x09,
    0x4501, 0x00,
    0x4505, 0x00,
    0x4837, 0x0e,
    0x5000, 0xff,
    0x5001, 0x0f,

//	0x0100, 0x01
};

kal_uint16 addr_data_pair_capture_24fps_ov13b10[] = {
//	0x0100, 0x00,
	0x0305, 0x45,
	0x3501, 0x0c,
	0x3502, 0x10,
	0x3662, 0x92,
	0x3714, 0x24,
	0x371a, 0x3e,
	0x3739, 0x12,
	0x37c2, 0x04,
	0x37d9, 0x0c,
	0x3800, 0x00,
	0x3801, 0x00,
	0x3802, 0x00,
	0x3803, 0x08,
	0x3804, 0x10,
	0x3805, 0x8f,
	0x3806, 0x0c,
	0x3807, 0x47,
	0x3808, 0x10,
	0x3809, 0x70,
	0x380a, 0x0c,
	0x380b, 0x30,
	0x380c, 0x04,
	0x380d, 0x98,
	0x380e, 0x0f,
	0x380f, 0x9b,
	0x3811, 0x0f,
	0x3813, 0x08,
	0x3814, 0x01,
	0x3815, 0x01,
	0x3816, 0x01,
	0x3817, 0x01,
	0x3820, 0x88,
	0x3c8c, 0x19,
	0x4008, 0x02,
	0x4009, 0x0f,
	0x4050, 0x02,
	0x4051, 0x09,
	0x4501, 0x00,
	0x4505, 0x00,
	0x4837, 0x0e,
	0x5000, 0xff,
	0x5001, 0x0f,
//	0x0100, 0x01
};
kal_uint16 addr_data_pair_capture_30fps_ov13b10[] = {
//	0x0100, 0x00,
	0x0305, 0x45,
	0x3501, 0x0c,
	0x3502, 0x10,
	0x3662, 0x92,
	0x3714, 0x24,
	0x371a, 0x3e,
	0x3739, 0x12,
	0x37c2, 0x04,
	0x37d9, 0x0c,
	0x3800, 0x00,
	0x3801, 0x00,
	0x3802, 0x00,
	0x3803, 0x08,
	0x3804, 0x10,
	0x3805, 0x8f,
	0x3806, 0x0c,
	0x3807, 0x47,
	0x3808, 0x10,
	0x3809, 0x70,
	0x380a, 0x0c,
	0x380b, 0x30,
	0x380c, 0x04,
	0x380d, 0x98,
	0x380e, 0x0c,
	0x380f, 0x7c,
	0x3811, 0x0f,
	0x3813, 0x08,
	0x3814, 0x01,
	0x3815, 0x01,
	0x3816, 0x01,
	0x3817, 0x01,
	0x3820, 0x88,
	0x3c8c, 0x19,
	0x4008, 0x02,
	0x4009, 0x0f,
	0x4050, 0x02,
	0x4051, 0x09,
	0x4501, 0x00,
	0x4505, 0x00,
	0x4837, 0x0e,
	0x5000, 0xff,
	0x5001, 0x0f,
//	0x0100, 0x01
};
#endif
static void capture_setting(kal_uint16 currefps)
{	/*	preview_setting  */
	LOG_INF("capture_setting capture_setting=%d\n",capture_setting);
#if MULTI_WRITE
	if (currefps == 150) {
	ov13b10_table_write_cmos_sensor(
		addr_data_pair_capture_15fps_ov13b10,
		sizeof(addr_data_pair_capture_15fps_ov13b10) /
		sizeof(kal_uint16));
	}
	if (currefps == 240) {
	ov13b10_table_write_cmos_sensor(
		addr_data_pair_capture_24fps_ov13b10,
		sizeof(addr_data_pair_capture_24fps_ov13b10) /
		sizeof(kal_uint16));
	} else {
	ov13b10_table_write_cmos_sensor(
		addr_data_pair_capture_30fps_ov13b10,
		sizeof(addr_data_pair_capture_30fps_ov13b10) /
		sizeof(kal_uint16));
	}
#else
	if (currefps == 300){			//30fps
//	write_cmos_sensor_8(0x0100, 0x00);
	write_cmos_sensor_8(0x0305, 0x45);
	write_cmos_sensor_8(0x3501, 0x0c);
	write_cmos_sensor_8(0x3502, 0x10);
	write_cmos_sensor_8(0x3662, 0x92);
	write_cmos_sensor_8(0x3714, 0x24);
	write_cmos_sensor_8(0x371a, 0x3e);
	write_cmos_sensor_8(0x3739, 0x12);
	write_cmos_sensor_8(0x37c2, 0x04);
	write_cmos_sensor_8(0x37d9, 0x0c);
	write_cmos_sensor_8(0x3800, 0x00);
	write_cmos_sensor_8(0x3801, 0x00);
	write_cmos_sensor_8(0x3802, 0x00);
	write_cmos_sensor_8(0x3803, 0x08);
	write_cmos_sensor_8(0x3804, 0x10);
	write_cmos_sensor_8(0x3805, 0x8f);
	write_cmos_sensor_8(0x3806, 0x0c);
	write_cmos_sensor_8(0x3807, 0x47);
	write_cmos_sensor_8(0x3808, 0x10);
	write_cmos_sensor_8(0x3809, 0x70);
	write_cmos_sensor_8(0x380a, 0x0c);
	write_cmos_sensor_8(0x380b, 0x30);
	write_cmos_sensor_8(0x380c, 0x04);
	write_cmos_sensor_8(0x380d, 0x98);
	write_cmos_sensor_8(0x380e, 0x0c);
	write_cmos_sensor_8(0x380f, 0x7c);
	write_cmos_sensor_8(0x3811, 0x0f);
	write_cmos_sensor_8(0x3813, 0x08);
	write_cmos_sensor_8(0x3814, 0x01);
	write_cmos_sensor_8(0x3815, 0x01);
	write_cmos_sensor_8(0x3816, 0x01);
	write_cmos_sensor_8(0x3817, 0x01);
	write_cmos_sensor_8(0x3820, 0x88);
	write_cmos_sensor_8(0x3c8c, 0x19);
	write_cmos_sensor_8(0x4008, 0x02);
	write_cmos_sensor_8(0x4009, 0x0f);
	write_cmos_sensor_8(0x4050, 0x02);
	write_cmos_sensor_8(0x4051, 0x09);
	write_cmos_sensor_8(0x4501, 0x00);
	write_cmos_sensor_8(0x4505, 0x00);
	write_cmos_sensor_8(0x4837, 0x0e);
	write_cmos_sensor_8(0x5000, 0xff);
	write_cmos_sensor_8(0x5001, 0x0f);
//	write_cmos_sensor_8(0x0100, 0x01);
}
else if (currefps == 240){			//24fps
//	write_cmos_sensor_8(0x0100, 0x00);
	write_cmos_sensor_8(0x0305, 0x45);
	write_cmos_sensor_8(0x3501, 0x0c);
	write_cmos_sensor_8(0x3502, 0x10);
	write_cmos_sensor_8(0x3662, 0x92);
	write_cmos_sensor_8(0x3714, 0x24);
	write_cmos_sensor_8(0x371a, 0x3e);
	write_cmos_sensor_8(0x3739, 0x12);
	write_cmos_sensor_8(0x37c2, 0x04);
	write_cmos_sensor_8(0x37d9, 0x0c);
	write_cmos_sensor_8(0x3800, 0x00);
	write_cmos_sensor_8(0x3801, 0x00);
	write_cmos_sensor_8(0x3802, 0x00);
	write_cmos_sensor_8(0x3803, 0x08);
	write_cmos_sensor_8(0x3804, 0x10);
	write_cmos_sensor_8(0x3805, 0x8f);
	write_cmos_sensor_8(0x3806, 0x0c);
	write_cmos_sensor_8(0x3807, 0x47);
	write_cmos_sensor_8(0x3808, 0x10);
	write_cmos_sensor_8(0x3809, 0x70);
	write_cmos_sensor_8(0x380a, 0x0c);
	write_cmos_sensor_8(0x380b, 0x30);
	write_cmos_sensor_8(0x380c, 0x04);
	write_cmos_sensor_8(0x380d, 0x98);
	write_cmos_sensor_8(0x380e, 0x0f);
	write_cmos_sensor_8(0x380f, 0x9b);
	write_cmos_sensor_8(0x3811, 0x0f);
	write_cmos_sensor_8(0x3813, 0x08);
	write_cmos_sensor_8(0x3814, 0x01);
	write_cmos_sensor_8(0x3815, 0x01);
	write_cmos_sensor_8(0x3816, 0x01);
	write_cmos_sensor_8(0x3817, 0x01);
	write_cmos_sensor_8(0x3820, 0x88);
	write_cmos_sensor_8(0x3c8c, 0x19);
	write_cmos_sensor_8(0x4008, 0x02);
	write_cmos_sensor_8(0x4009, 0x0f);
	write_cmos_sensor_8(0x4050, 0x02);
	write_cmos_sensor_8(0x4051, 0x09);
	write_cmos_sensor_8(0x4501, 0x00);
	write_cmos_sensor_8(0x4505, 0x00);
	write_cmos_sensor_8(0x4837, 0x0e);
	write_cmos_sensor_8(0x5000, 0xff);
	write_cmos_sensor_8(0x5001, 0x0f);
//	write_cmos_sensor_8(0x0100, 0x01);
}
	else {
	//15fps for PIP

//	write_cmos_sensor_8(0x0100, 0x00);
	write_cmos_sensor_8(0x0305, 0x45);
	write_cmos_sensor_8(0x3501, 0x0c);
	write_cmos_sensor_8(0x3502, 0x10);
	write_cmos_sensor_8(0x3662, 0x92);
	write_cmos_sensor_8(0x3714, 0x24);
	write_cmos_sensor_8(0x371a, 0x3e);
	write_cmos_sensor_8(0x3739, 0x12);
	write_cmos_sensor_8(0x37c2, 0x04);
	write_cmos_sensor_8(0x37d9, 0x0c);
	write_cmos_sensor_8(0x3800, 0x00);
	write_cmos_sensor_8(0x3801, 0x00);
	write_cmos_sensor_8(0x3802, 0x00);
	write_cmos_sensor_8(0x3803, 0x08);
	write_cmos_sensor_8(0x3804, 0x10);
	write_cmos_sensor_8(0x3805, 0x8f);
	write_cmos_sensor_8(0x3806, 0x0c);
	write_cmos_sensor_8(0x3807, 0x47);
	write_cmos_sensor_8(0x3808, 0x10);
	write_cmos_sensor_8(0x3809, 0x70);
	write_cmos_sensor_8(0x380a, 0x0c);
	write_cmos_sensor_8(0x380b, 0x30);
	write_cmos_sensor_8(0x380c, 0x04);
	write_cmos_sensor_8(0x380d, 0x98);
	write_cmos_sensor_8(0x380e, 0x18);
	write_cmos_sensor_8(0x380f, 0xf8);
	write_cmos_sensor_8(0x3811, 0x0f);
	write_cmos_sensor_8(0x3813, 0x08);
	write_cmos_sensor_8(0x3814, 0x01);
	write_cmos_sensor_8(0x3815, 0x01);
	write_cmos_sensor_8(0x3816, 0x01);
	write_cmos_sensor_8(0x3817, 0x01);
	write_cmos_sensor_8(0x3820, 0x88);
	write_cmos_sensor_8(0x3c8c, 0x19);
	write_cmos_sensor_8(0x4008, 0x02);
	write_cmos_sensor_8(0x4009, 0x0f);
	write_cmos_sensor_8(0x4050, 0x02);
	write_cmos_sensor_8(0x4051, 0x09);
	write_cmos_sensor_8(0x4501, 0x00);
	write_cmos_sensor_8(0x4505, 0x00);
	write_cmos_sensor_8(0x4837, 0x0e);
	write_cmos_sensor_8(0x5000, 0xff);
	write_cmos_sensor_8(0x5001, 0x0f);
//	write_cmos_sensor_8(0x0100, 0x01);


	}
#endif
}

#if MULTI_WRITE
kal_uint16 addr_data_pair_video_ov13b10[] = {
//	0x0100, 0x00,
	0x0305, 0x24,
	0x3501, 0x06,
	0x3502, 0x00,
	0x3662, 0x88,
	0x3714, 0x28,
	0x371a, 0x3e,
	0x3739, 0x10,
	0x37c2, 0x14,
	0x37d9, 0x06,
	0x3800, 0x00,
	0x3801, 0xb0,
	0x3802, 0x01,
	0x3803, 0xe0,
	0x3804, 0x0f,
	0x3805, 0xdf,
	0x3806, 0x0a,
	0x3807, 0x6f,
	0x3808, 0x07,
	0x3809, 0x80,
	0x380a, 0x04,
	0x380b, 0x38,
	0x380c, 0x04,
	0x380d, 0x98,
	0x380e, 0x0c,
	0x380f, 0x66,
	0x3811, 0x0b,
	0x3813, 0x08,
	0x3814, 0x03,
	0x3815, 0x01,
	0x3816, 0x03,
	0x3817, 0x01,
	0x3820, 0x8b,
	0x3c8c, 0x18,
	0x4008, 0x00,
	0x4009, 0x05,
	0x4050, 0x00,
	0x4051, 0x05,
	0x4501, 0x08,
	0x4505, 0x04,
	0x4837, 0x1C,
	0x5000, 0xfd,
	0x5001, 0x0d,
//	0x0100, 0x01
};
#endif
#if 1
static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n",currefps);

	//write_cmos_sensor_8(0x0100, 0x00);
#if MULTI_WRITE
	ov13b10_table_write_cmos_sensor(
		addr_data_pair_video_ov13b10,
		sizeof(addr_data_pair_video_ov13b10) /
		sizeof(kal_uint16));
#else
	write_cmos_sensor_8(0x0305, 0x24);
	write_cmos_sensor_8(0x3501, 0x06);
	write_cmos_sensor_8(0x3502, 0x00);
	write_cmos_sensor_8(0x3662, 0x88);
	write_cmos_sensor_8(0x3714, 0x28);
	write_cmos_sensor_8(0x371a, 0x3e);
	write_cmos_sensor_8(0x3739, 0x10);
	write_cmos_sensor_8(0x37c2, 0x14);
	write_cmos_sensor_8(0x37d9, 0x06);
	write_cmos_sensor_8(0x3800, 0x00);
	write_cmos_sensor_8(0x3801, 0xb0);
	write_cmos_sensor_8(0x3802, 0x01);
	write_cmos_sensor_8(0x3803, 0xe0);
	write_cmos_sensor_8(0x3804, 0x0f);
	write_cmos_sensor_8(0x3805, 0xdf);
	write_cmos_sensor_8(0x3806, 0x0a);
	write_cmos_sensor_8(0x3807, 0x6f);
	write_cmos_sensor_8(0x3808, 0x07);
	write_cmos_sensor_8(0x3809, 0x80);
	write_cmos_sensor_8(0x380a, 0x04);
	write_cmos_sensor_8(0x380b, 0x38);
	write_cmos_sensor_8(0x380c, 0x04);
	write_cmos_sensor_8(0x380d, 0x98);
	write_cmos_sensor_8(0x380e, 0x0c);
	write_cmos_sensor_8(0x380f, 0x66);
	write_cmos_sensor_8(0x3811, 0x0b);
	write_cmos_sensor_8(0x3813, 0x08);
	write_cmos_sensor_8(0x3814, 0x03);
	write_cmos_sensor_8(0x3815, 0x01);
	write_cmos_sensor_8(0x3816, 0x03);
	write_cmos_sensor_8(0x3817, 0x01);
	write_cmos_sensor_8(0x3820, 0x8b);
	write_cmos_sensor_8(0x3c8c, 0x18);
	write_cmos_sensor_8(0x4008, 0x00);
	write_cmos_sensor_8(0x4009, 0x05);
	write_cmos_sensor_8(0x4050, 0x00);
	write_cmos_sensor_8(0x4051, 0x05);
	write_cmos_sensor_8(0x4501, 0x08);
	write_cmos_sensor_8(0x4505, 0x04);
	write_cmos_sensor_8(0x4837, 0x1C);
	write_cmos_sensor_8(0x5000, 0xfd);
	write_cmos_sensor_8(0x5001, 0x0d);
#endif
	//write_cmos_sensor_8(0x0100, 0x01);

}
#endif
#if MULTI_WRITE
kal_uint16 addr_data_pair_hs_video_ov13b10[] = {
	//0x0100, 0x00,
	0x0305, 0x24,
	0x3501, 0x02,
	0x3502, 0x00,
	0x3662, 0x84,
	0x3714, 0x30,
	0x371a, 0x3f,
	0x3739, 0x10,
	0x37c2, 0x2c,
	0x37d9, 0x06,
	0x3800, 0x03,
	0x3801, 0x20,
	0x3802, 0x02,
	0x3803, 0x48,
	0x3804, 0x0d,
	0x3805, 0x6f,
	0x3806, 0x0a,
	0x3807, 0x07,
	0x3808, 0x02,
	0x3809, 0x80,
	0x380a, 0x01,
	0x380b, 0xe0,
	0x380c, 0x04,
	0x380d, 0x98,
	0x380e, 0x03,
	0x380f, 0x1a,
	0x3811, 0x09,
	0x3813, 0x08,
	0x3814, 0x07,
	0x3815, 0x01,
	0x3816, 0x07,
	0x3817, 0x01,
	0x3820, 0x8c,
	0x3c8c, 0x18,
	0x4008, 0x00,
	0x4009, 0x03,
	0x4050, 0x00,
	0x4051, 0x05,
	0x4501, 0x30,
	0x4505, 0x04,
	0x4837, 0x1c,
	0x5000, 0xfd,
	0x5001, 0x0d,

	//0x0100, 0x01
};
#endif
static void hs_video_setting(void)
{
#if MULTI_WRITE
	ov13b10_table_write_cmos_sensor(
		addr_data_pair_hs_video_ov13b10,
		sizeof(addr_data_pair_hs_video_ov13b10) /
		sizeof(kal_uint16));
#else
	write_cmos_sensor_8(0x0305, 0x24);
	write_cmos_sensor_8(0x3501, 0x02);
	write_cmos_sensor_8(0x3502, 0x00);
	write_cmos_sensor_8(0x3662, 0x84);
	write_cmos_sensor_8(0x3714, 0x30);
	write_cmos_sensor_8(0x371a, 0x3f);
	write_cmos_sensor_8(0x3739, 0x10);
	write_cmos_sensor_8(0x37c2, 0x2c);
	write_cmos_sensor_8(0x37d9, 0x06);
	write_cmos_sensor_8(0x3800, 0x03);
	write_cmos_sensor_8(0x3801, 0x20);
	write_cmos_sensor_8(0x3802, 0x02);
	write_cmos_sensor_8(0x3803, 0x48);
	write_cmos_sensor_8(0x3804, 0x0d);
	write_cmos_sensor_8(0x3805, 0x6f);
	write_cmos_sensor_8(0x3806, 0x0a);
	write_cmos_sensor_8(0x3807, 0x07);
	write_cmos_sensor_8(0x3808, 0x02);
	write_cmos_sensor_8(0x3809, 0x80);
	write_cmos_sensor_8(0x380a, 0x01);
	write_cmos_sensor_8(0x380b, 0xe0);
	write_cmos_sensor_8(0x380c, 0x04);
	write_cmos_sensor_8(0x380d, 0x98);
	write_cmos_sensor_8(0x380e, 0x03);
	write_cmos_sensor_8(0x380f, 0x1a);
	write_cmos_sensor_8(0x3811, 0x09);
	write_cmos_sensor_8(0x3813, 0x08);
	write_cmos_sensor_8(0x3814, 0x07);
	write_cmos_sensor_8(0x3815, 0x01);
	write_cmos_sensor_8(0x3816, 0x07);
	write_cmos_sensor_8(0x3817, 0x01);
	write_cmos_sensor_8(0x3820, 0x8c);
	write_cmos_sensor_8(0x3c8c, 0x18);
	write_cmos_sensor_8(0x4008, 0x00);
	write_cmos_sensor_8(0x4009, 0x03);
	write_cmos_sensor_8(0x4050, 0x00);
	write_cmos_sensor_8(0x4051, 0x05);
	write_cmos_sensor_8(0x4501, 0x30);
	write_cmos_sensor_8(0x4505, 0x04);
	write_cmos_sensor_8(0x4837, 0x1C);
	write_cmos_sensor_8(0x5000, 0xfd);
	write_cmos_sensor_8(0x5001, 0x0d);
	//write_cmos_sensor_8(0x0100, 0x01);
#endif
}

static void slim_video_setting(void)
{
	LOG_INF("E\n");

  	preview_setting();
}


//extern void preload_eeprom_data(void);

/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID
*
* PARAMETERS
*	*sensorID : return the sensor ID
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	if(curr_sensor_id != 1)
	{
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;		
	}
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {	
			*sensor_id = (read_cmos_sensor_8(0x300A) << 16) | (read_cmos_sensor_8(0x300B)<<8) | read_cmos_sensor_8(0x300C);
			if (*sensor_id == imgsensor_info.sensor_id) {
				*sensor_id = OV13B10_SENSOR_ID;
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
				//preload_eeprom_data();	  
				return ERROR_NONE;
			}
			LOG_INF("Read sensor id fail, write id:0x%x ,sensor Id:0x%x\n", imgsensor.i2c_write_id,*sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		// if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF 
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*	open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
	//const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2};
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint32 sensor_id = 0;
	LOG_1;
	LOG_2;
	if(curr_sensor_id != 1)
	{
		return ERROR_SENSOR_CONNECT_FAIL;		
	}
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = (read_cmos_sensor_8(0x300A) << 16) | (read_cmos_sensor_8(0x300B)<<8) | read_cmos_sensor_8(0x300C);
			LOG_INF("Read sensor id, write id:0x%x ,sensor Id:0x%x\n", imgsensor.i2c_write_id, sensor_id);//lx_revised
			if (sensor_id == imgsensor_info.sensor_id) {				
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);	  
				break;
			}
			LOG_INF("Read sensor id fail, write id:0x%x ,sensor Id:0x%x\n", imgsensor.i2c_write_id,sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail sequence write in  */
	sensor_init();

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en= KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = KAL_FALSE;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}	/*	open  */



/*************************************************************************
* FUNCTION
*	close
*
* DESCRIPTION
*
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");

	/*No Need to implement this function*/

	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	//set_mirror_flip(IMAGE_NORMAL);
	return ERROR_NONE;
}	/*	preview   */

/*************************************************************************
* FUNCTION
*	capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;

    if (imgsensor.current_fps == imgsensor_info.cap.max_framerate) // 30fps
    {
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	else if(imgsensor.current_fps == 240)//PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
    {
		if (imgsensor.current_fps != imgsensor_info.cap1.max_framerate)
			LOG_INF("Warning: current_fps %d fps is not support, so use cap1's setting: %d fps!\n",imgsensor.current_fps,imgsensor_info.cap1.max_framerate/10);
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	else //PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
    {
		if (imgsensor.current_fps != imgsensor_info.cap2.max_framerate)
			LOG_INF("Warning: current_fps %d fps is not support, so use cap1's setting: %d fps!\n",imgsensor.current_fps,imgsensor_info.cap1.max_framerate/10);
		imgsensor.pclk = imgsensor_info.cap2.pclk;
		imgsensor.line_length = imgsensor_info.cap2.linelength;
		imgsensor.frame_length = imgsensor_info.cap2.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap2.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);

	capture_setting(imgsensor.current_fps);

	return ERROR_NONE;
}/* capture() */	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);
	//preview_setting();
	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	//set_mirror_flip(IMAGE_NORMAL);
	return ERROR_NONE;
}	/*	hs_video   */


static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E");

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
	//set_mirror_flip(IMAGE_NORMAL);
	return ERROR_NONE;
}	/*	slim_video	 */



static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


	sensor_resolution->SensorHighSpeedVideoWidth	 = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight	 = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth	 = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight	 = imgsensor_info.slim_video.grabwindow_height;

	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
					  MSDK_SENSOR_INFO_STRUCT *sensor_info,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d", scenario_id);


	//sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; /* not use */
	//sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; /* not use */
	//imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; /* not use */

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame; 		 /* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->FrameTimeDelayFrame = imgsensor_info.frame_time_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	sensor_info->PDAF_Support = PDAF_SUPPORT_RAW;
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
	sensor_info->SensorHightSampling = 0;	// 0 is default 1x
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

			sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

			break;
		default:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
	}

	return ERROR_NONE;
}	/*	get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d", scenario_id);
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
			LOG_INF("Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps,1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d ", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable)
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);
/*Gionee <BUG> <shenweikun> <2017-8-17> modify for GNSPR #92075 begin*/
	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (imgsensor.frame_length>imgsensor.shutter)
			  set_dummy();

			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if(framerate == 0)
				return ERROR_NONE;
			frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
           if (imgsensor.frame_length>imgsensor.shutter)
             set_dummy();;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			if((framerate==150)&&(imgsensor.pclk ==imgsensor_info.cap1.pclk))
			{
			frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			}
			else if(framerate==300)
			{
			frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			}
			else
			{
			frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			}
		           if (imgsensor.frame_length>imgsensor.shutter)
             set_dummy();
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//Gionee <bug> <xionggh> <2017-08-21> add for #99134 begin
			#if 0
			set_dummy();
			#else
			//set_dummy();
			#endif
			//Gionee <bug> <xionggh> <2017-08-21> add for #99134 end
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;
			imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		           if (imgsensor.frame_length>imgsensor.shutter)
             set_dummy();
			break;
		default:  //coding with  preview scenario by default
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (imgsensor.frame_length>imgsensor.shutter)
             set_dummy();
			LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
			break;
	}
/*Gionee <BUG> <shenweikun> <2017-8-17> add for GNSPR #92075 end*/
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

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

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);
    if (enable) {
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		write_cmos_sensor_8(0x5000, 0x81);
		write_cmos_sensor_8(0x5080, 0x80);

	} else {
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		write_cmos_sensor_8(0x5000, 0xff);
		write_cmos_sensor_8(0x5080, 0x00);
	}	  

	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
							 UINT8 *feature_para,UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16=(UINT16 *) feature_para;
	UINT16 *feature_data_16=(UINT16 *) feature_para;
	UINT32 *feature_return_para_32=(UINT32 *) feature_para;
	UINT32 *feature_data_32=(UINT32 *) feature_para;
	unsigned long long *feature_data=(unsigned long long *) feature_para;

	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	//struct SET_PD_BLOCK_INFO_T *PDAFinfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	LOG_INF("feature_id = %d", feature_id);
	switch (feature_id) 
	{
		case SENSOR_FEATURE_GET_PERIOD:
			*feature_return_para_16++ = imgsensor.line_length;
			*feature_return_para_16 = imgsensor.frame_length;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			LOG_INF("feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n", imgsensor.pclk,imgsensor.current_fps);
			*feature_return_para_32 = imgsensor.pclk;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_ESHUTTER:
			set_shutter(*feature_data);
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			break;
		case SENSOR_FEATURE_SET_GAIN:
			set_gain((UINT16) *feature_data);
			break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
			break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			if((sensor_reg_data->RegData>>8)>0)
			write_cmos_sensor_8(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
			else
			write_cmos_sensor_8(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
			break;
		case SENSOR_FEATURE_GET_REGISTER:
			sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
			break;
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
		// if EEPROM does not exist in camera module.
			*feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
			set_video_mode(*feature_data);
			break;
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			get_imgsensor_id(feature_return_para_32);
			break;
		case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
			set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
			break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
			break;
		case SENSOR_FEATURE_GET_PDAF_DATA:
			LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
			//read_eeprom((kal_uint16 )(*feature_data),(char*)(uintptr_t)(*(feature_data+1)),(kal_uint32)(*(feature_data+2)));
			//read_ov13b10_eeprom((kal_uint16 )(*feature_data),(char*)(uintptr_t)(*(feature_data+1)),(kal_uint32)(*(feature_data+2)));
			break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:
			set_test_pattern_mode((BOOL)*feature_data);
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
			*feature_return_para_32 = imgsensor_info.checksum_value;
			*feature_para_len=4;
			break;
	//prize-camera  add for  by zhuzhengjiang 20190830-begin
		case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_INF("current fps :%d\n", (UINT16)*feature_data_32);
			spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = (UINT16)*feature_data_32;
			spin_unlock(&imgsensor_drv_lock);
			break;
		case SENSOR_FEATURE_SET_HDR:
		LOG_INF("ihdr enable :%d\n", (UINT8)*feature_data_32);
			LOG_INF("Warning! Not Support IHDR Feature");
			spin_lock(&imgsensor_drv_lock);
			imgsensor.ihdr_en = (UINT8) *feature_data_32;
			//imgsensor.ihdr_en = *feature_data_32;
			spin_unlock(&imgsensor_drv_lock);
			break;
	//prize-camera  add for  by zhuzhengjiang 20190830-end
		case SENSOR_FEATURE_GET_CROP_INFO:
			LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);
			wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

			switch (*feature_data_32) 
			{
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
				break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
				break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
				break;
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
				memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
				break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				default:
				memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
				break;
			}
			break;
		#if 0
		case SENSOR_FEATURE_GET_PDAF_INFO:
			LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n", (UINT32)*feature_data);
			PDAFinfo= (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));

			switch (*feature_data) 
			{
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(struct SET_PD_BLOCK_INFO_T));
				break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				default:
				break;
			}
			break;
		case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
			LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%llu\n", *feature_data);
			//PDAF capacity enable or not, OV13B10 only full size support PDAF
			switch (*feature_data) 
			{
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0; // video & capture use same setting
				break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
				default:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
			}
			break;
		#endif
		case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
			LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
			streaming_control(KAL_FALSE);
			break;
		case SENSOR_FEATURE_SET_STREAMING_RESUME:
			LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n", *feature_data);
			if (*feature_data != 0)
				set_shutter(*feature_data);
			streaming_control(KAL_TRUE);
			break;
		case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
			LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
			ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
			break;
		case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
		{
			kal_uint32 rate;

			switch (*feature_data) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				rate = imgsensor_info.cap.mipi_pixel_rate;
				break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				rate = imgsensor_info.normal_video.mipi_pixel_rate;
				break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				rate = imgsensor_info.hs_video.mipi_pixel_rate;
				break;
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
				rate = imgsensor_info.slim_video.mipi_pixel_rate;
				break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				default:
				rate = imgsensor_info.pre.mipi_pixel_rate;
				break;
			}
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = rate;
		}
		break;
		case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME://lzl
			set_shutter_frame_length((UINT16)*feature_data,(UINT16)*(feature_data+1));
			break;
		default:
			break;
	}

	return ERROR_NONE;
}	/*	feature_control()  */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 OV13B10_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}	/*	OV13B10_MIPI_RAW_SensorInit	*/
