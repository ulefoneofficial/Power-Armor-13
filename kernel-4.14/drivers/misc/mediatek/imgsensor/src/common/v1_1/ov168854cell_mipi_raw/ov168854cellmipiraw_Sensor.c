/*****************************************************************************
 *
 * Filename:
 * ---------
 *   OV168854CELL_REAR_rear_mipiraw_Sensor.c
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

#include "ov168854cellmipiraw_Sensor.h"
//#include "ov168854cell_eeprom.h"

#define PFX "OV168854CELL_camera_sensor"
#define LOG_1 LOG_INF("OV168854CELL,MIPI 4LANE,PDAF\n")
#define LOG_2 LOG_INF("preview 2336*1752@30fps,768Mbps/lane; video 4672*3504@30fps,1440Mbps/lane; capture 16M@30fps,1440Mbps/lane\n")
#define LOG_INF(fmt, args...)   pr_debug(PFX "[%s] " fmt, __FUNCTION__, ##args)
#define LOGE(format, args...)   pr_err("[%s] " format, __FUNCTION__, ##args)
//prize fengshangdong modify at 20190524
static DEFINE_SPINLOCK(imgsensor_drv_lock);

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = OV168854CELL_SENSOR_ID,
	.checksum_value = 0xb42138aa,	/* checksum value for Camera Auto Test */

	.pre = {
		.pclk = 160000000,											//record different mode's pclk
		.linelength = 1400,		/*OV168854CELL Note: linelength/4,it means line length per lane*/
		.framelength = 3808,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 2336,
		.grabwindow_height = 1752,	
		.mipi_data_lp2hs_settle_dc = 120,
		.max_framerate = 300,
		.mipi_pixel_rate =  307200000,

	},
	
	.cap = { /* 95:line 5312, 52/35:line 5336 */
		.pclk = 160000000,
		.linelength = 1400, /*OV168854CELL Note: linelength/4,it means line length per lane*/
		.framelength = 3808,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2336,
		.grabwindow_height = 1752,	
		.mipi_data_lp2hs_settle_dc = 120,
		.max_framerate = 300,
		.mipi_pixel_rate =  576000000,
		},
		
	.cap1 = {
		.pclk = 160000000,	
		.linelength = 1400, 
		.framelength = 3808,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2336,
		.grabwindow_height = 1752,	
		.mipi_data_lp2hs_settle_dc = 120,
		.max_framerate = 240,
		.mipi_pixel_rate =  430080000,
	},
	
  .cap2 = {
		.pclk = 80000000,
		.linelength = 1400, /*OV168854CELL Note: linelength/4,it means line length per lane*/
		.framelength = 3808,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4608,
		.grabwindow_height = 3456,
		.mipi_data_lp2hs_settle_dc = 120,//unit , ns
		.max_framerate = 150,//less than 13M(include 13M),cap1 max framerate is 24fps,16M max framerate is 20fps, 20M max framerate is 15fps
		.mipi_pixel_rate =  288000000,
  },

	.normal_video = {
 		.pclk = 160000000,											//record different mode's pclk
		.linelength = 1400,		/*OV168854CELL Note: linelength/4,it means line length per lane*/
		.framelength = 3808,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 2336,
		.grabwindow_height = 1752,	
		.mipi_data_lp2hs_settle_dc = 120,
		.max_framerate = 300,
		.mipi_pixel_rate =  307200000,
    },
     /*prize modify by zhuzhengjiang for CTS: testConstrainedHighSpeedRecording 2019618 start*/
	.hs_video = {
		.pclk = 160000000,	
		.linelength = 1400,	
		.framelength = 1904,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 720,	
		.mipi_data_lp2hs_settle_dc = 120,
		.max_framerate = 600,
		.mipi_pixel_rate =  384000000,
	},

	.slim_video = {
		.pclk = 160000000,											//record different mode's pclk
		.linelength = 1400,		/*OV168854CELL Note: linelength/4,it means line length per lane*/
		.framelength = 3808,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 2336,
		.grabwindow_height = 1752,	
		.mipi_data_lp2hs_settle_dc = 120,
		.max_framerate = 300,
		.mipi_pixel_rate =  307200000,
	},
	.margin = 12,		/* sensor framelength & shutter margin check*/
	.min_shutter = 8,	/* min shutter */
	.max_frame_length = 0xffff,	/* max framelength by sensor register's limitation */
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0,	/* 1, support; 0,not support */
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 5,	  //support sensor mode num
	.frame_time_delay_frame = 2,	/* The delay frame of setting frame length  */
	.cap_delay_frame = 3,
	.pre_delay_frame = 3,
	.video_delay_frame = 3,	/* enter video delay frame num */
	.hs_video_delay_frame = 3,	/* enter high speed video  delay frame num */
	.slim_video_delay_frame = 3,	/* enter slim video delay frame num */

	.isp_driving_current = ISP_DRIVING_2MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_CSI2,	/* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
	.mipi_settle_delay_mode = 1,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,//sensor output first pixel color
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x20, 0xff},
	.i2c_speed = 400,// i2c read/write speed
};

//prize end
static  struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_HV_MIRROR,//the truth is mirror and flip 
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x4C00,	/* current shutter */
	.gain = 0x200,		/* current gain */
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
    {4704, 3536, 0, 2, 4704, 3532, 2352, 1766, 9, 9, 2336, 1752, 0, 0, 2336, 1752},    /* Preview */
	{4704, 3536, 0, 2, 4704, 3532, 2352, 1766, 9, 9, 2336, 1752, 0, 0, 2336, 1752},    /* capture */
	{4704, 3536, 0, 2, 4704, 3532, 2352, 1766, 9, 9, 2336, 1752, 0, 0, 2336, 1752},    /* video */
    {4704, 3536, 0, 2, 4704, 3532, 2352, 1766, 537, 521, 1280,  720, 0, 0, 1280, 720}, /* hight speed video*/
    {4704, 3536, 0, 2, 4704, 3532, 2352, 1766, 9, 9, 2336, 1752, 0, 0, 2336, 1752},    /* slim video */
};// slim video

 	
 	
static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info =
{
    .i4OffsetX = 0,
    .i4OffsetY = 0,
    .i4PitchX  = 32,
    .i4PitchY  = 32,
    .i4PairNum  =8,
    .i4SubBlkW  =16,
    .i4SubBlkH  =8,
    .i4PosL = {{10,1},{26, 1},{2,13},  {18,13},{10,17},{26,17},{2,29},{18,29}},
    .i4PosR = {{10,5},{26,5},{2,9},{18,9},{10,21},{26,21},{2,25},{18,25}},
    .iMirrorFlip = IMAGE_NORMAL,
    .i4BlockNumX = 144,
    .i4BlockNumY = 108,
};

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
	write_cmos_sensor_8(0x380e, imgsensor.frame_length >> 8);
	write_cmos_sensor_8(0x380f, imgsensor.frame_length & 0xFF);	 
	
	//set horizontal_total_size, means linelength 
	write_cmos_sensor_8(0x380c, (imgsensor.line_length) >> 8);
	write_cmos_sensor_8(0x380d, (imgsensor.line_length) & 0xFF);

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
					write_cmos_sensor_8(0x380e, (imgsensor.frame_length >> 8) & 0xFF);
					write_cmos_sensor_8(0x380f, imgsensor.frame_length & 0xFF);
				}
			} else {
				// Extend frame length
				imgsensor.frame_length = (imgsensor.frame_length >> 1) << 1;
				write_cmos_sensor_8(0x380e, (imgsensor.frame_length >> 8) & 0xFF);
				write_cmos_sensor_8(0x380f, imgsensor.frame_length & 0xFF);
			}	
			// Update Shutter
		write_cmos_sensor_8(0x3500, (shutter >> 12) & 0x0F);
		write_cmos_sensor_8(0x3501, (shutter >> 4) & 0xFF);
		write_cmos_sensor_8(0x3502, (shutter << 4) & 0xF0);
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);
			
	
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
	write_cmos_sensor_8(0x3502, (shutter << 4) & 0xFF);
	write_cmos_sensor_8(0x3501, (shutter >> 4) & 0xFF);	  
	write_cmos_sensor_8(0x3500, (shutter >> 12) & 0x0F);	

	//LOG_INF("Add for N3D! shutterlzl =%d, framelength =%d\n", shutter,imgsensor.frame_length);

}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 iReg = 0x0000;

	/* platform 1xgain = 64, sensor driver 1*gain = 0x80 */
	iReg = gain * 128 / BASEGAIN;

	if (iReg < 0x80)	/* sensor 1xGain */
		iReg = 0X80;

	if (iReg > 0x7c0)	/* sensor 15.5xGain */
		iReg = 0X7C0;

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
	write_cmos_sensor_8(0x350c, (reg_gain >> 8));
	write_cmos_sensor_8(0x350d, (reg_gain & 0xFF));
	return gain;
}	/*	set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
#if 1
	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
	if (imgsensor.ihdr_en) {

		spin_lock(&imgsensor_drv_lock);
			if (le > imgsensor.min_frame_length - imgsensor_info.margin)
				imgsensor.frame_length = le + imgsensor_info.margin;
			else
				imgsensor.frame_length = imgsensor.min_frame_length;
			if (imgsensor.frame_length > imgsensor_info.max_frame_length)
				imgsensor.frame_length = imgsensor_info.max_frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (le < imgsensor_info.min_shutter) le = imgsensor_info.min_shutter;
			if (se < imgsensor_info.min_shutter) se = imgsensor_info.min_shutter;


		write_cmos_sensor_8(0x380e, imgsensor.frame_length >> 8);
		write_cmos_sensor_8(0x380f, imgsensor.frame_length & 0xFF);

	    write_cmos_sensor_8(0x3500, (le >> 12) & 0x0F);
	    write_cmos_sensor_8(0x3501, (le >> 4) & 0xFF);
	    write_cmos_sensor_8(0x3502, (le << 4) & 0xF0);
		
	    write_cmos_sensor_8(0x3510, (se >> 12) & 0x0F);
	    write_cmos_sensor_8(0x3511, (se >> 4) & 0xFF);
	    write_cmos_sensor_8(0x3512, (se << 4) & 0xF0);

		
	LOG_INF("iHDR:imgsensor.frame_length=%d\n",imgsensor.frame_length);
		set_gain(gain);
	}

#endif

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


static void sensor_init(void)
{
	LOG_INF("E\n");

	// Sysclk 160Mhz, MIPI4_768Mbps/Lane, 30Fps.
	// Line_length =1400, Frame_length =3808, V-blanking=17.99ms
	write_cmos_sensor_8(0x0103, 0x01);  /* SW Reset, need delay */
	mdelay(10);
	write_cmos_sensor_8(0x0102, 0x01);
	write_cmos_sensor_8(0x0300, 0xf3);
	write_cmos_sensor_8(0x0301, 0xa5);
	write_cmos_sensor_8(0x0302, 0x10);
	write_cmos_sensor_8(0x0304, 0x29);//0x28  /*prize  add  for k6307  RF intrusion by zhuzhengjiang    20190925-begin*/
	write_cmos_sensor_8(0x0314, 0x02);
	write_cmos_sensor_8(0x0316, 0xa0);
	write_cmos_sensor_8(0x0319, 0x00);
	write_cmos_sensor_8(0x031a, 0x01);
	write_cmos_sensor_8(0x031e, 0x09);
	write_cmos_sensor_8(0x0320, 0x0f);
	write_cmos_sensor_8(0x300d, 0x11);
	write_cmos_sensor_8(0x3012, 0x41);
	write_cmos_sensor_8(0x3016, 0xf0);
	write_cmos_sensor_8(0x3018, 0xf0);
	write_cmos_sensor_8(0x3025, 0x03);
	write_cmos_sensor_8(0x301e, 0x98);
	write_cmos_sensor_8(0x3031, 0x88);
	write_cmos_sensor_8(0x3400, 0x00);
	write_cmos_sensor_8(0x3406, 0x08);
	write_cmos_sensor_8(0x3408, 0x03);
	write_cmos_sensor_8(0x3410, 0x00);
	write_cmos_sensor_8(0x3412, 0x00);
	write_cmos_sensor_8(0x3413, 0x00);
	write_cmos_sensor_8(0x3414, 0x00);
	write_cmos_sensor_8(0x3415, 0x00);
	write_cmos_sensor_8(0x3500, 0x00);
	write_cmos_sensor_8(0x3501, 0xec);
	write_cmos_sensor_8(0x3502, 0x00);
	write_cmos_sensor_8(0x3503, 0x08);
	write_cmos_sensor_8(0x3505, 0x8c);
	write_cmos_sensor_8(0x3507, 0x00);
	write_cmos_sensor_8(0x3508, 0x02);
	write_cmos_sensor_8(0x3509, 0x00);
	write_cmos_sensor_8(0x350c, 0x02);
	write_cmos_sensor_8(0x350d, 0x00);
	write_cmos_sensor_8(0x3510, 0x00);
	write_cmos_sensor_8(0x3511, 0xec);
	write_cmos_sensor_8(0x3512, 0x00);
	write_cmos_sensor_8(0x3600, 0x40);
	write_cmos_sensor_8(0x3601, 0x00);
	write_cmos_sensor_8(0x3602, 0x84);
	write_cmos_sensor_8(0x3608, 0xc7);
	write_cmos_sensor_8(0x3609, 0xd0);
	write_cmos_sensor_8(0x360a, 0xff);
	write_cmos_sensor_8(0x360b, 0x6c);
	write_cmos_sensor_8(0x360c, 0x00);
	write_cmos_sensor_8(0x3611, 0x00);
	write_cmos_sensor_8(0x3612, 0x00);
	write_cmos_sensor_8(0x3613, 0x8e);
	write_cmos_sensor_8(0x3618, 0x00);
	write_cmos_sensor_8(0x3619, 0x90);
	write_cmos_sensor_8(0x361a, 0x00);
	write_cmos_sensor_8(0x361b, 0x01);
	write_cmos_sensor_8(0x361c, 0xc5);
	write_cmos_sensor_8(0x3620, 0x50);
	write_cmos_sensor_8(0x3621, 0x66);
	write_cmos_sensor_8(0x3622, 0x88);
	write_cmos_sensor_8(0x3623, 0x88);
	write_cmos_sensor_8(0x3624, 0x88);
	write_cmos_sensor_8(0x3625, 0x88);
	write_cmos_sensor_8(0x3626, 0x03);
	write_cmos_sensor_8(0x3627, 0x88);
	write_cmos_sensor_8(0x3628, 0x1c);
	write_cmos_sensor_8(0x3629, 0x00);
	write_cmos_sensor_8(0x362a, 0x00);
	write_cmos_sensor_8(0x3632, 0x00);
	write_cmos_sensor_8(0x3633, 0x10);
	write_cmos_sensor_8(0x3634, 0x10);
	write_cmos_sensor_8(0x3635, 0x10);
	write_cmos_sensor_8(0x3636, 0x10);
	write_cmos_sensor_8(0x3637, 0x77);
	write_cmos_sensor_8(0x3638, 0x77);
	write_cmos_sensor_8(0x3639, 0x66);
	write_cmos_sensor_8(0x363a, 0x66);
	write_cmos_sensor_8(0x3652, 0x00);
	write_cmos_sensor_8(0x3653, 0x00);
	write_cmos_sensor_8(0x3654, 0x77);
	write_cmos_sensor_8(0x3655, 0x77);
	write_cmos_sensor_8(0x3656, 0x77);
	write_cmos_sensor_8(0x3657, 0x77);
	write_cmos_sensor_8(0x3658, 0x00);
	write_cmos_sensor_8(0x3659, 0x84);
	write_cmos_sensor_8(0x365a, 0x81);
	write_cmos_sensor_8(0x365b, 0x8e);
	write_cmos_sensor_8(0x365c, 0x1c);
	write_cmos_sensor_8(0x3660, 0x40);
	write_cmos_sensor_8(0x3661, 0x0c);
	write_cmos_sensor_8(0x3662, 0x00);
	write_cmos_sensor_8(0x3663, 0x40);
	write_cmos_sensor_8(0x3664, 0x03);
	write_cmos_sensor_8(0x3666, 0xac);
	write_cmos_sensor_8(0x3668, 0xf0);
	write_cmos_sensor_8(0x3669, 0x0e);
	write_cmos_sensor_8(0x366a, 0x10);
	write_cmos_sensor_8(0x366b, 0x42);
	write_cmos_sensor_8(0x366c, 0x03);
	write_cmos_sensor_8(0x366d, 0x05);
	write_cmos_sensor_8(0x366e, 0x05);
	write_cmos_sensor_8(0x3674, 0x04);
	write_cmos_sensor_8(0x3675, 0x16);
	write_cmos_sensor_8(0x3680, 0x00);
	write_cmos_sensor_8(0x3681, 0x33);
	write_cmos_sensor_8(0x3682, 0x33);
	write_cmos_sensor_8(0x3683, 0x33);
	write_cmos_sensor_8(0x368a, 0x04);
	write_cmos_sensor_8(0x368b, 0x04);
	write_cmos_sensor_8(0x368c, 0x04);
	write_cmos_sensor_8(0x368d, 0x04);
	write_cmos_sensor_8(0x368e, 0x04);
	write_cmos_sensor_8(0x368f, 0x04);
	write_cmos_sensor_8(0x3694, 0x10);
	write_cmos_sensor_8(0x3696, 0x30);
	write_cmos_sensor_8(0x3698, 0x30);
	write_cmos_sensor_8(0x3699, 0x00);
	write_cmos_sensor_8(0x369a, 0x44);
	write_cmos_sensor_8(0x369c, 0x28);
	write_cmos_sensor_8(0x369e, 0x28);
	write_cmos_sensor_8(0x36a0, 0x28);
	write_cmos_sensor_8(0x36a2, 0x30);
	write_cmos_sensor_8(0x36a4, 0x3b);
	write_cmos_sensor_8(0x36a5, 0x00);
	write_cmos_sensor_8(0x36a6, 0x43);
	write_cmos_sensor_8(0x36a7, 0x00);
	write_cmos_sensor_8(0x36a8, 0x48);
	write_cmos_sensor_8(0x36a9, 0x00);
	write_cmos_sensor_8(0x36aa, 0x48);
	write_cmos_sensor_8(0x36ab, 0x00);
	write_cmos_sensor_8(0x36ac, 0x48);
	write_cmos_sensor_8(0x36c1, 0x33);
	write_cmos_sensor_8(0x36c3, 0x33);
	write_cmos_sensor_8(0x36ca, 0x04);
	write_cmos_sensor_8(0x36cb, 0x04);
	write_cmos_sensor_8(0x36cc, 0x04);
	write_cmos_sensor_8(0x36cd, 0x04);
	write_cmos_sensor_8(0x36ce, 0x04);
	write_cmos_sensor_8(0x36cf, 0x04);
	write_cmos_sensor_8(0x3700, 0x13);
	write_cmos_sensor_8(0x3701, 0x14);
	write_cmos_sensor_8(0x3702, 0x12);
	write_cmos_sensor_8(0x3704, 0x0e);
	write_cmos_sensor_8(0x3706, 0x23);
	write_cmos_sensor_8(0x3708, 0x17);
	write_cmos_sensor_8(0x3709, 0x38);
	write_cmos_sensor_8(0x370b, 0x63);
	write_cmos_sensor_8(0x3713, 0x00);
	write_cmos_sensor_8(0x3714, 0x64);
	write_cmos_sensor_8(0x371d, 0x10);
	write_cmos_sensor_8(0x371f, 0x05);
	write_cmos_sensor_8(0x3726, 0x20);
	write_cmos_sensor_8(0x3727, 0x23);
	write_cmos_sensor_8(0x373b, 0x06);
	write_cmos_sensor_8(0x373d, 0x07);
	write_cmos_sensor_8(0x374f, 0x0d);
	write_cmos_sensor_8(0x3754, 0x88);
	write_cmos_sensor_8(0x375a, 0x08);
	write_cmos_sensor_8(0x3764, 0x12);
	write_cmos_sensor_8(0x3765, 0x0b);
	write_cmos_sensor_8(0x3767, 0x0c);
	write_cmos_sensor_8(0x3768, 0x18);
	write_cmos_sensor_8(0x3769, 0x08);
	write_cmos_sensor_8(0x376a, 0x0c);
	write_cmos_sensor_8(0x376b, 0x80);
	write_cmos_sensor_8(0x37a2, 0x04);
	write_cmos_sensor_8(0x37b1, 0x40);
	write_cmos_sensor_8(0x37d0, 0x06);
	write_cmos_sensor_8(0x37d9, 0x88);
	write_cmos_sensor_8(0x37f4, 0x80);
	write_cmos_sensor_8(0x37fc, 0x05);
	write_cmos_sensor_8(0x37fd, 0x00);
	write_cmos_sensor_8(0x37fe, 0x0b);
	write_cmos_sensor_8(0x37ff, 0x00);
	write_cmos_sensor_8(0x3800, 0x00);
	write_cmos_sensor_8(0x3801, 0x00);
	write_cmos_sensor_8(0x3802, 0x00);
	write_cmos_sensor_8(0x3803, 0x02);
	write_cmos_sensor_8(0x3804, 0x12);
	write_cmos_sensor_8(0x3805, 0x5f);
	write_cmos_sensor_8(0x3806, 0x0d);
	write_cmos_sensor_8(0x3807, 0xd1);
	write_cmos_sensor_8(0x3808, 0x09);
	write_cmos_sensor_8(0x3809, 0x20);
	write_cmos_sensor_8(0x380a, 0x06);
	write_cmos_sensor_8(0x380b, 0xd8);
	write_cmos_sensor_8(0x380c, 0x05);
	write_cmos_sensor_8(0x380d, 0x78);
	write_cmos_sensor_8(0x380e, 0x0e);
	write_cmos_sensor_8(0x380f, 0xe0);
	write_cmos_sensor_8(0x3810, 0x00);
	write_cmos_sensor_8(0x3811, 0x09);
	write_cmos_sensor_8(0x3812, 0x00);
	write_cmos_sensor_8(0x3813, 0x09);
	write_cmos_sensor_8(0x3814, 0x11);
	write_cmos_sensor_8(0x3815, 0x22);
	write_cmos_sensor_8(0x3820, 0x01);
	write_cmos_sensor_8(0x3821, 0x00);
	write_cmos_sensor_8(0x3834, 0xf5);
	write_cmos_sensor_8(0x3836, 0x28);
	write_cmos_sensor_8(0x383d, 0x80);
	write_cmos_sensor_8(0x3841, 0x20);
	write_cmos_sensor_8(0x3883, 0x02);
	write_cmos_sensor_8(0x3886, 0x02);
	write_cmos_sensor_8(0x3889, 0x02);
	write_cmos_sensor_8(0x3891, 0x0f);
	write_cmos_sensor_8(0x38a0, 0x04);
	write_cmos_sensor_8(0x38a1, 0x00);
	write_cmos_sensor_8(0x38a2, 0x04);
	write_cmos_sensor_8(0x38a3, 0x04);
	write_cmos_sensor_8(0x38b0, 0x02);
	write_cmos_sensor_8(0x38b1, 0x02);
	write_cmos_sensor_8(0x3b8e, 0x00);
	write_cmos_sensor_8(0x3d84, 0x80);
	write_cmos_sensor_8(0x3d85, 0x1b);
	write_cmos_sensor_8(0x3d8c, 0x67);
	write_cmos_sensor_8(0x3d8d, 0xa8);
	write_cmos_sensor_8(0x3f00, 0xca);
	write_cmos_sensor_8(0x3f03, 0x1a);
	write_cmos_sensor_8(0x3f05, 0x66);
	write_cmos_sensor_8(0x4008, 0x00);
	write_cmos_sensor_8(0x4009, 0x02);
	write_cmos_sensor_8(0x400e, 0x00);
	write_cmos_sensor_8(0x4010, 0x29);
	write_cmos_sensor_8(0x4011, 0x01);
	write_cmos_sensor_8(0x4012, 0x0d);
	write_cmos_sensor_8(0x4013, 0x16);
	write_cmos_sensor_8(0x4014, 0x0a);
	write_cmos_sensor_8(0x4015, 0x02);
	write_cmos_sensor_8(0x4016, 0x15);
	write_cmos_sensor_8(0x4017, 0x00);
	write_cmos_sensor_8(0x4018, 0x09);
	write_cmos_sensor_8(0x4019, 0x00);
	write_cmos_sensor_8(0x401a, 0x40);
	write_cmos_sensor_8(0x4020, 0x04);
	write_cmos_sensor_8(0x4021, 0x00);
	write_cmos_sensor_8(0x4022, 0x04);
	write_cmos_sensor_8(0x4023, 0x00);
	write_cmos_sensor_8(0x4024, 0x04);
	write_cmos_sensor_8(0x4025, 0x00);
	write_cmos_sensor_8(0x4026, 0x04);
	write_cmos_sensor_8(0x4027, 0x00);
	write_cmos_sensor_8(0x4056, 0x05);
	write_cmos_sensor_8(0x4202, 0x00);
	write_cmos_sensor_8(0x4500, 0x25);
	write_cmos_sensor_8(0x4501, 0x04);
	write_cmos_sensor_8(0x4502, 0x80);
	write_cmos_sensor_8(0x4503, 0x10);
	write_cmos_sensor_8(0x450c, 0x05);
	write_cmos_sensor_8(0x450e, 0x16);
	write_cmos_sensor_8(0x450f, 0x88);
	write_cmos_sensor_8(0x4540, 0x86);
	write_cmos_sensor_8(0x4541, 0x07);
	write_cmos_sensor_8(0x4542, 0x04);
	write_cmos_sensor_8(0x4543, 0x05);
	write_cmos_sensor_8(0x4544, 0x06);
	write_cmos_sensor_8(0x4545, 0x07);
	write_cmos_sensor_8(0x4546, 0x04);
	write_cmos_sensor_8(0x4547, 0x05);
	write_cmos_sensor_8(0x4548, 0x02);
	write_cmos_sensor_8(0x4549, 0x03);
	write_cmos_sensor_8(0x454a, 0x00);
	write_cmos_sensor_8(0x454b, 0x01);
	write_cmos_sensor_8(0x454c, 0x02);
	write_cmos_sensor_8(0x454d, 0x03);
	write_cmos_sensor_8(0x454e, 0x00);
	write_cmos_sensor_8(0x454f, 0x01);
	write_cmos_sensor_8(0x4550, 0x06);
	write_cmos_sensor_8(0x4551, 0x07);
	write_cmos_sensor_8(0x4552, 0x04);
	write_cmos_sensor_8(0x4553, 0x05);
	write_cmos_sensor_8(0x4554, 0x06);
	write_cmos_sensor_8(0x4555, 0x07);
	write_cmos_sensor_8(0x4556, 0x04);
	write_cmos_sensor_8(0x4557, 0x05);
	write_cmos_sensor_8(0x4558, 0x02);
	write_cmos_sensor_8(0x4559, 0x03);
	write_cmos_sensor_8(0x455a, 0x00);
	write_cmos_sensor_8(0x455b, 0x01);
	write_cmos_sensor_8(0x455c, 0x02);
	write_cmos_sensor_8(0x455d, 0x03);
	write_cmos_sensor_8(0x455e, 0x00);
	write_cmos_sensor_8(0x455f, 0x01);
	write_cmos_sensor_8(0x4640, 0x01);
	write_cmos_sensor_8(0x4641, 0x04);
	write_cmos_sensor_8(0x4642, 0x02);
	write_cmos_sensor_8(0x4643, 0x00);
	write_cmos_sensor_8(0x4645, 0x03);
	write_cmos_sensor_8(0x4800, 0x00);
	write_cmos_sensor_8(0x4809, 0x2b);
	write_cmos_sensor_8(0x480e, 0x02);
	write_cmos_sensor_8(0x4813, 0x90);
	write_cmos_sensor_8(0x4817, 0x00);
	write_cmos_sensor_8(0x4837, 0x14);
	write_cmos_sensor_8(0x484b, 0x01);
	write_cmos_sensor_8(0x4850, 0x7c);
	write_cmos_sensor_8(0x4852, 0x03);
	write_cmos_sensor_8(0x4853, 0x12);
	write_cmos_sensor_8(0x4856, 0x58);
	write_cmos_sensor_8(0x4857, 0x02);
	write_cmos_sensor_8(0x4d00, 0x04);
	write_cmos_sensor_8(0x4d01, 0x5a);
	write_cmos_sensor_8(0x4d02, 0xb3);
	write_cmos_sensor_8(0x4d03, 0xf1);
	write_cmos_sensor_8(0x4d04, 0xaa);
	write_cmos_sensor_8(0x4d05, 0xc9);
	write_cmos_sensor_8(0x5080, 0x04);
	write_cmos_sensor_8(0x5084, 0x09);
	write_cmos_sensor_8(0x5085, 0x30);
	write_cmos_sensor_8(0x5086, 0x06);
	write_cmos_sensor_8(0x5087, 0xe6);
	write_cmos_sensor_8(0x5000, 0x89);
	write_cmos_sensor_8(0x5001, 0x42);
	write_cmos_sensor_8(0x5002, 0x01);
	write_cmos_sensor_8(0x5004, 0x81);
	write_cmos_sensor_8(0x5020, 0x00);
	write_cmos_sensor_8(0x5021, 0x10);
	write_cmos_sensor_8(0x5022, 0x12);
	write_cmos_sensor_8(0x5023, 0x50);
	write_cmos_sensor_8(0x5024, 0x00);
	write_cmos_sensor_8(0x5025, 0x08);
	write_cmos_sensor_8(0x5026, 0x0d);
	write_cmos_sensor_8(0x5027, 0xb8);
	write_cmos_sensor_8(0x5028, 0x00);
	write_cmos_sensor_8(0x5081, 0x00);
	write_cmos_sensor_8(0x5180, 0x03);
	write_cmos_sensor_8(0x5181, 0xb0);
	write_cmos_sensor_8(0x5184, 0x63);
	write_cmos_sensor_8(0x5185, 0x17);
	write_cmos_sensor_8(0x518c, 0x01);
	write_cmos_sensor_8(0x518d, 0x01);
	write_cmos_sensor_8(0x518e, 0x01);
	write_cmos_sensor_8(0x518f, 0x01);
	write_cmos_sensor_8(0x5190, 0x00);
	write_cmos_sensor_8(0x5191, 0x00);
	write_cmos_sensor_8(0x5192, 0x09);
	write_cmos_sensor_8(0x5193, 0x2f);
	write_cmos_sensor_8(0x5194, 0x00);
	write_cmos_sensor_8(0x5195, 0x00);
	write_cmos_sensor_8(0x5200, 0xbf);
	write_cmos_sensor_8(0x5201, 0xf3);
	write_cmos_sensor_8(0x5202, 0x09);
	write_cmos_sensor_8(0x5203, 0x1b);
	write_cmos_sensor_8(0x5204, 0xe0);
	write_cmos_sensor_8(0x5205, 0x10);
	write_cmos_sensor_8(0x5206, 0x3f);
	write_cmos_sensor_8(0x5207, 0x3c);
	write_cmos_sensor_8(0x5208, 0x24);
	write_cmos_sensor_8(0x5209, 0x0f);
	write_cmos_sensor_8(0x520a, 0x43);
	write_cmos_sensor_8(0x520b, 0x3b);
	write_cmos_sensor_8(0x520c, 0x33);
	write_cmos_sensor_8(0x520d, 0x33);
	write_cmos_sensor_8(0x520e, 0x63);
	write_cmos_sensor_8(0x5210, 0x06);
	write_cmos_sensor_8(0x5211, 0x03);
	write_cmos_sensor_8(0x5212, 0x08);
	write_cmos_sensor_8(0x5213, 0x08);
	write_cmos_sensor_8(0x5217, 0x04);
	write_cmos_sensor_8(0x5218, 0x02);
	write_cmos_sensor_8(0x5219, 0x01);
	write_cmos_sensor_8(0x521a, 0x04);
	write_cmos_sensor_8(0x521b, 0x02);
	write_cmos_sensor_8(0x521c, 0x01);
	write_cmos_sensor_8(0x5297, 0x04);
	write_cmos_sensor_8(0x5298, 0x02);
	write_cmos_sensor_8(0x5299, 0x01);
	write_cmos_sensor_8(0x529a, 0x04);
	write_cmos_sensor_8(0x529b, 0x02);
	write_cmos_sensor_8(0x529c, 0x01);
	write_cmos_sensor_8(0x5404, 0x00);
	write_cmos_sensor_8(0x5405, 0x00);
	write_cmos_sensor_8(0x5406, 0x06);
	write_cmos_sensor_8(0x5407, 0x18);
	write_cmos_sensor_8(0x5408, 0x08);
	write_cmos_sensor_8(0x5409, 0x1a);
	write_cmos_sensor_8(0x5410, 0x03);
	write_cmos_sensor_8(0x5413, 0xa0);
	write_cmos_sensor_8(0x5500, 0x4a);
	write_cmos_sensor_8(0x5501, 0x43);
	write_cmos_sensor_8(0x5502, 0x3c);
	write_cmos_sensor_8(0x5503, 0x38);
	write_cmos_sensor_8(0x5504, 0x34);
	write_cmos_sensor_8(0x5505, 0x32);
	write_cmos_sensor_8(0x5506, 0x30);
	write_cmos_sensor_8(0x5507, 0x2f);
	write_cmos_sensor_8(0x5508, 0x31);
	write_cmos_sensor_8(0x5509, 0x30);
	write_cmos_sensor_8(0x550a, 0x31);
	write_cmos_sensor_8(0x550b, 0x34);
	write_cmos_sensor_8(0x550c, 0x38);
	write_cmos_sensor_8(0x550d, 0x3d);
	write_cmos_sensor_8(0x550e, 0x44);
	write_cmos_sensor_8(0x550f, 0x51);
	write_cmos_sensor_8(0x5510, 0x3f);
	write_cmos_sensor_8(0x5511, 0x35);
	write_cmos_sensor_8(0x5512, 0x30);
	write_cmos_sensor_8(0x5513, 0x2c);
	write_cmos_sensor_8(0x5514, 0x2a);
	write_cmos_sensor_8(0x5515, 0x27);
	write_cmos_sensor_8(0x5516, 0x26);
	write_cmos_sensor_8(0x5517, 0x26);
	write_cmos_sensor_8(0x5518, 0x25);
	write_cmos_sensor_8(0x5519, 0x26);
	write_cmos_sensor_8(0x551a, 0x28);
	write_cmos_sensor_8(0x551b, 0x2a);
	write_cmos_sensor_8(0x551c, 0x2d);
	write_cmos_sensor_8(0x551d, 0x31);
	write_cmos_sensor_8(0x551e, 0x37);
	write_cmos_sensor_8(0x551f, 0x40);
	write_cmos_sensor_8(0x5520, 0x32);
	write_cmos_sensor_8(0x5521, 0x2c);
	write_cmos_sensor_8(0x5522, 0x28);
	write_cmos_sensor_8(0x5523, 0x24);
	write_cmos_sensor_8(0x5524, 0x21);
	write_cmos_sensor_8(0x5525, 0x1e);
	write_cmos_sensor_8(0x5526, 0x1d);
	write_cmos_sensor_8(0x5527, 0x1c);
	write_cmos_sensor_8(0x5528, 0x1d);
	write_cmos_sensor_8(0x5529, 0x1d);
	write_cmos_sensor_8(0x552a, 0x1f);
	write_cmos_sensor_8(0x552b, 0x22);
	write_cmos_sensor_8(0x552c, 0x25);
	write_cmos_sensor_8(0x552d, 0x29);
	write_cmos_sensor_8(0x552e, 0x2e);
	write_cmos_sensor_8(0x552f, 0x36);
	write_cmos_sensor_8(0x5530, 0x2b);
	write_cmos_sensor_8(0x5531, 0x25);
	write_cmos_sensor_8(0x5532, 0x20);
	write_cmos_sensor_8(0x5533, 0x1c);
	write_cmos_sensor_8(0x5534, 0x19);
	write_cmos_sensor_8(0x5535, 0x16);
	write_cmos_sensor_8(0x5536, 0x15);
	write_cmos_sensor_8(0x5537, 0x14);
	write_cmos_sensor_8(0x5538, 0x14);
	write_cmos_sensor_8(0x5539, 0x15);
	write_cmos_sensor_8(0x553a, 0x17);
	write_cmos_sensor_8(0x553b, 0x19);
	write_cmos_sensor_8(0x553c, 0x1d);
	write_cmos_sensor_8(0x553d, 0x22);
	write_cmos_sensor_8(0x553e, 0x27);
	write_cmos_sensor_8(0x553f, 0x2f);
	write_cmos_sensor_8(0x5540, 0x25);
	write_cmos_sensor_8(0x5541, 0x1f);
	write_cmos_sensor_8(0x5542, 0x1a);
	write_cmos_sensor_8(0x5543, 0x16);
	write_cmos_sensor_8(0x5544, 0x12);
	write_cmos_sensor_8(0x5545, 0x0f);
	write_cmos_sensor_8(0x5546, 0x0d);
	write_cmos_sensor_8(0x5547, 0x0d);
	write_cmos_sensor_8(0x5548, 0x0d);
	write_cmos_sensor_8(0x5549, 0x0e);
	write_cmos_sensor_8(0x554a, 0x10);
	write_cmos_sensor_8(0x554b, 0x13);
	write_cmos_sensor_8(0x554c, 0x16);
	write_cmos_sensor_8(0x554d, 0x1b);
	write_cmos_sensor_8(0x554e, 0x21);
	write_cmos_sensor_8(0x554f, 0x29);
	write_cmos_sensor_8(0x5550, 0x20);
	write_cmos_sensor_8(0x5551, 0x1a);
	write_cmos_sensor_8(0x5552, 0x15);
	write_cmos_sensor_8(0x5553, 0x10);
	write_cmos_sensor_8(0x5554, 0x0c);
	write_cmos_sensor_8(0x5555, 0x09);
	write_cmos_sensor_8(0x5556, 0x07);
	write_cmos_sensor_8(0x5557, 0x07);
	write_cmos_sensor_8(0x5558, 0x06);
	write_cmos_sensor_8(0x5559, 0x07);
	write_cmos_sensor_8(0x555a, 0x09);
	write_cmos_sensor_8(0x555b, 0x0d);
	write_cmos_sensor_8(0x555c, 0x11);
	write_cmos_sensor_8(0x555d, 0x16);
	write_cmos_sensor_8(0x555e, 0x1c);
	write_cmos_sensor_8(0x555f, 0x24);
	write_cmos_sensor_8(0x5560, 0x1e);
	write_cmos_sensor_8(0x5561, 0x17);
	write_cmos_sensor_8(0x5562, 0x11);
	write_cmos_sensor_8(0x5563, 0x0d);
	write_cmos_sensor_8(0x5564, 0x08);
	write_cmos_sensor_8(0x5565, 0x05);
	write_cmos_sensor_8(0x5566, 0x03);
	write_cmos_sensor_8(0x5567, 0x02);
	write_cmos_sensor_8(0x5568, 0x02);
	write_cmos_sensor_8(0x5569, 0x03);
	write_cmos_sensor_8(0x556a, 0x05);
	write_cmos_sensor_8(0x556b, 0x09);
	write_cmos_sensor_8(0x556c, 0x0d);
	write_cmos_sensor_8(0x556d, 0x12);
	write_cmos_sensor_8(0x556e, 0x19);
	write_cmos_sensor_8(0x556f, 0x20);
	write_cmos_sensor_8(0x5570, 0x1c);
	write_cmos_sensor_8(0x5571, 0x15);
	write_cmos_sensor_8(0x5572, 0x0f);
	write_cmos_sensor_8(0x5573, 0x0a);
	write_cmos_sensor_8(0x5574, 0x06);
	write_cmos_sensor_8(0x5575, 0x03);
	write_cmos_sensor_8(0x5576, 0x01);
	write_cmos_sensor_8(0x5577, 0x00);
	write_cmos_sensor_8(0x5578, 0x00);
	write_cmos_sensor_8(0x5579, 0x01);
	write_cmos_sensor_8(0x557a, 0x03);
	write_cmos_sensor_8(0x557b, 0x06);
	write_cmos_sensor_8(0x557c, 0x0b);
	write_cmos_sensor_8(0x557d, 0x10);
	write_cmos_sensor_8(0x557e, 0x17);
	write_cmos_sensor_8(0x557f, 0x1f);
	write_cmos_sensor_8(0x5580, 0x1c);
	write_cmos_sensor_8(0x5581, 0x15);
	write_cmos_sensor_8(0x5582, 0x0f);
	write_cmos_sensor_8(0x5583, 0x0a);
	write_cmos_sensor_8(0x5584, 0x06);
	write_cmos_sensor_8(0x5585, 0x03);
	write_cmos_sensor_8(0x5586, 0x01);
	write_cmos_sensor_8(0x5587, 0x00);
	write_cmos_sensor_8(0x5588, 0x00);
	write_cmos_sensor_8(0x5589, 0x01);
	write_cmos_sensor_8(0x558a, 0x03);
	write_cmos_sensor_8(0x558b, 0x06);
	write_cmos_sensor_8(0x558c, 0x0b);
	write_cmos_sensor_8(0x558d, 0x10);
	write_cmos_sensor_8(0x558e, 0x16);
	write_cmos_sensor_8(0x558f, 0x1e);
	write_cmos_sensor_8(0x5590, 0x1d);
	write_cmos_sensor_8(0x5591, 0x16);
	write_cmos_sensor_8(0x5592, 0x11);
	write_cmos_sensor_8(0x5593, 0x0c);
	write_cmos_sensor_8(0x5594, 0x08);
	write_cmos_sensor_8(0x5595, 0x05);
	write_cmos_sensor_8(0x5596, 0x03);
	write_cmos_sensor_8(0x5597, 0x02);
	write_cmos_sensor_8(0x5598, 0x02);
	write_cmos_sensor_8(0x5599, 0x03);
	write_cmos_sensor_8(0x559a, 0x04);
	write_cmos_sensor_8(0x559b, 0x08);
	write_cmos_sensor_8(0x559c, 0x0c);
	write_cmos_sensor_8(0x559d, 0x12);
	write_cmos_sensor_8(0x559e, 0x18);
	write_cmos_sensor_8(0x559f, 0x1f);
	write_cmos_sensor_8(0x55a0, 0x20);
	write_cmos_sensor_8(0x55a1, 0x19);
	write_cmos_sensor_8(0x55a2, 0x14);
	write_cmos_sensor_8(0x55a3, 0x0f);
	write_cmos_sensor_8(0x55a4, 0x0b);
	write_cmos_sensor_8(0x55a5, 0x08);
	write_cmos_sensor_8(0x55a6, 0x06);
	write_cmos_sensor_8(0x55a7, 0x05);
	write_cmos_sensor_8(0x55a8, 0x06);
	write_cmos_sensor_8(0x55a9, 0x06);
	write_cmos_sensor_8(0x55aa, 0x08);
	write_cmos_sensor_8(0x55ab, 0x0b);
	write_cmos_sensor_8(0x55ac, 0x0f);
	write_cmos_sensor_8(0x55ad, 0x14);
	write_cmos_sensor_8(0x55ae, 0x1b);
	write_cmos_sensor_8(0x55af, 0x22);
	write_cmos_sensor_8(0x55b0, 0x24);
	write_cmos_sensor_8(0x55b1, 0x1e);
	write_cmos_sensor_8(0x55b2, 0x19);
	write_cmos_sensor_8(0x55b3, 0x14);
	write_cmos_sensor_8(0x55b4, 0x10);
	write_cmos_sensor_8(0x55b5, 0x0e);
	write_cmos_sensor_8(0x55b6, 0x0c);
	write_cmos_sensor_8(0x55b7, 0x0b);
	write_cmos_sensor_8(0x55b8, 0x0b);
	write_cmos_sensor_8(0x55b9, 0x0c);
	write_cmos_sensor_8(0x55ba, 0x0e);
	write_cmos_sensor_8(0x55bb, 0x11);
	write_cmos_sensor_8(0x55bc, 0x14);
	write_cmos_sensor_8(0x55bd, 0x19);
	write_cmos_sensor_8(0x55be, 0x1f);
	write_cmos_sensor_8(0x55bf, 0x27);
	write_cmos_sensor_8(0x55c0, 0x29);
	write_cmos_sensor_8(0x55c1, 0x23);
	write_cmos_sensor_8(0x55c2, 0x1f);
	write_cmos_sensor_8(0x55c3, 0x1a);
	write_cmos_sensor_8(0x55c4, 0x17);
	write_cmos_sensor_8(0x55c5, 0x14);
	write_cmos_sensor_8(0x55c6, 0x13);
	write_cmos_sensor_8(0x55c7, 0x12);
	write_cmos_sensor_8(0x55c8, 0x12);
	write_cmos_sensor_8(0x55c9, 0x13);
	write_cmos_sensor_8(0x55ca, 0x14);
	write_cmos_sensor_8(0x55cb, 0x17);
	write_cmos_sensor_8(0x55cc, 0x1a);
	write_cmos_sensor_8(0x55cd, 0x1f);
	write_cmos_sensor_8(0x55ce, 0x25);
	write_cmos_sensor_8(0x55cf, 0x2d);
	write_cmos_sensor_8(0x55d0, 0x31);
	write_cmos_sensor_8(0x55d1, 0x2a);
	write_cmos_sensor_8(0x55d2, 0x25);
	write_cmos_sensor_8(0x55d3, 0x22);
	write_cmos_sensor_8(0x55d4, 0x1f);
	write_cmos_sensor_8(0x55d5, 0x1c);
	write_cmos_sensor_8(0x55d6, 0x1b);
	write_cmos_sensor_8(0x55d7, 0x1a);
	write_cmos_sensor_8(0x55d8, 0x1a);
	write_cmos_sensor_8(0x55d9, 0x1a);
	write_cmos_sensor_8(0x55da, 0x1c);
	write_cmos_sensor_8(0x55db, 0x1f);
	write_cmos_sensor_8(0x55dc, 0x22);
	write_cmos_sensor_8(0x55dd, 0x27);
	write_cmos_sensor_8(0x55de, 0x2c);
	write_cmos_sensor_8(0x55df, 0x34);
	write_cmos_sensor_8(0x55e0, 0x3b);
	write_cmos_sensor_8(0x55e1, 0x32);
	write_cmos_sensor_8(0x55e2, 0x2e);
	write_cmos_sensor_8(0x55e3, 0x2a);
	write_cmos_sensor_8(0x55e4, 0x27);
	write_cmos_sensor_8(0x55e5, 0x25);
	write_cmos_sensor_8(0x55e6, 0x23);
	write_cmos_sensor_8(0x55e7, 0x23);
	write_cmos_sensor_8(0x55e8, 0x23);
	write_cmos_sensor_8(0x55e9, 0x24);
	write_cmos_sensor_8(0x55ea, 0x25);
	write_cmos_sensor_8(0x55eb, 0x28);
	write_cmos_sensor_8(0x55ec, 0x2b);
	write_cmos_sensor_8(0x55ed, 0x30);
	write_cmos_sensor_8(0x55ee, 0x35);
	write_cmos_sensor_8(0x55ef, 0x3f);
	write_cmos_sensor_8(0x55f0, 0x4b);
	write_cmos_sensor_8(0x55f1, 0x41);
	write_cmos_sensor_8(0x55f2, 0x3a);
	write_cmos_sensor_8(0x55f3, 0x36);
	write_cmos_sensor_8(0x55f4, 0x32);
	write_cmos_sensor_8(0x55f5, 0x30);
	write_cmos_sensor_8(0x55f6, 0x2f);
	write_cmos_sensor_8(0x55f7, 0x2d);
	write_cmos_sensor_8(0x55f8, 0x2f);
	write_cmos_sensor_8(0x55f9, 0x2f);
	write_cmos_sensor_8(0x55fa, 0x30);
	write_cmos_sensor_8(0x55fb, 0x33);
	write_cmos_sensor_8(0x55fc, 0x37);
	write_cmos_sensor_8(0x55fd, 0x3d);
	write_cmos_sensor_8(0x55fe, 0x44);
	write_cmos_sensor_8(0x55ff, 0x4e);
	write_cmos_sensor_8(0x5600, 0x80);
	write_cmos_sensor_8(0x5601, 0x80);
	write_cmos_sensor_8(0x5602, 0x80);
	write_cmos_sensor_8(0x5603, 0x7f);
	write_cmos_sensor_8(0x5604, 0x7f);
	write_cmos_sensor_8(0x5605, 0x7f);
	write_cmos_sensor_8(0x5606, 0x7e);
	write_cmos_sensor_8(0x5607, 0x82);
	write_cmos_sensor_8(0x5608, 0x7e);
	write_cmos_sensor_8(0x5609, 0x80);
	write_cmos_sensor_8(0x560a, 0x80);
	write_cmos_sensor_8(0x560b, 0x7e);
	write_cmos_sensor_8(0x560c, 0x80);
	write_cmos_sensor_8(0x560d, 0x80);
	write_cmos_sensor_8(0x560e, 0x81);
	write_cmos_sensor_8(0x560f, 0x7a);
	write_cmos_sensor_8(0x5610, 0x80);
	write_cmos_sensor_8(0x5611, 0x80);
	write_cmos_sensor_8(0x5612, 0x80);
	write_cmos_sensor_8(0x5613, 0x80);
	write_cmos_sensor_8(0x5614, 0x7f);
	write_cmos_sensor_8(0x5615, 0x7f);
	write_cmos_sensor_8(0x5616, 0x80);
	write_cmos_sensor_8(0x5617, 0x7f);
	write_cmos_sensor_8(0x5618, 0x80);
	write_cmos_sensor_8(0x5619, 0x7f);
	write_cmos_sensor_8(0x561a, 0x7f);
	write_cmos_sensor_8(0x561b, 0x80);
	write_cmos_sensor_8(0x561c, 0x7f);
	write_cmos_sensor_8(0x561d, 0x7f);
	write_cmos_sensor_8(0x561e, 0x7f);
	write_cmos_sensor_8(0x561f, 0x82);
	write_cmos_sensor_8(0x5620, 0x80);
	write_cmos_sensor_8(0x5621, 0x80);
	write_cmos_sensor_8(0x5622, 0x7f);
	write_cmos_sensor_8(0x5623, 0x7f);
	write_cmos_sensor_8(0x5624, 0x7f);
	write_cmos_sensor_8(0x5625, 0x7f);
	write_cmos_sensor_8(0x5626, 0x7f);
	write_cmos_sensor_8(0x5627, 0x7f);
	write_cmos_sensor_8(0x5628, 0x7f);
	write_cmos_sensor_8(0x5629, 0x7f);
	write_cmos_sensor_8(0x562a, 0x7f);
	write_cmos_sensor_8(0x562b, 0x7f);
	write_cmos_sensor_8(0x562c, 0x7f);
	write_cmos_sensor_8(0x562d, 0x80);
	write_cmos_sensor_8(0x562e, 0x7f);
	write_cmos_sensor_8(0x562f, 0x81);
	write_cmos_sensor_8(0x5630, 0x80);
	write_cmos_sensor_8(0x5631, 0x7f);
	write_cmos_sensor_8(0x5632, 0x7f);
	write_cmos_sensor_8(0x5633, 0x7f);
	write_cmos_sensor_8(0x5634, 0x7f);
	write_cmos_sensor_8(0x5635, 0x7f);
	write_cmos_sensor_8(0x5636, 0x7f);
	write_cmos_sensor_8(0x5637, 0x7f);
	write_cmos_sensor_8(0x5638, 0x7f);
	write_cmos_sensor_8(0x5639, 0x7f);
	write_cmos_sensor_8(0x563a, 0x7f);
	write_cmos_sensor_8(0x563b, 0x7f);
	write_cmos_sensor_8(0x563c, 0x7f);
	write_cmos_sensor_8(0x563d, 0x7f);
	write_cmos_sensor_8(0x563e, 0x80);
	write_cmos_sensor_8(0x563f, 0x80);
	write_cmos_sensor_8(0x5640, 0x81);
	write_cmos_sensor_8(0x5641, 0x80);
	write_cmos_sensor_8(0x5642, 0x80);
	write_cmos_sensor_8(0x5643, 0x7f);
	write_cmos_sensor_8(0x5644, 0x7f);
	write_cmos_sensor_8(0x5645, 0x7f);
	write_cmos_sensor_8(0x5646, 0x7f);
	write_cmos_sensor_8(0x5647, 0x7f);
	write_cmos_sensor_8(0x5648, 0x7f);
	write_cmos_sensor_8(0x5649, 0x7f);
	write_cmos_sensor_8(0x564a, 0x7f);
	write_cmos_sensor_8(0x564b, 0x7f);
	write_cmos_sensor_8(0x564c, 0x7f);
	write_cmos_sensor_8(0x564d, 0x7f);
	write_cmos_sensor_8(0x564e, 0x80);
	write_cmos_sensor_8(0x564f, 0x7f);
	write_cmos_sensor_8(0x5650, 0x80);
	write_cmos_sensor_8(0x5651, 0x80);
	write_cmos_sensor_8(0x5652, 0x80);
	write_cmos_sensor_8(0x5653, 0x7f);
	write_cmos_sensor_8(0x5654, 0x7f);
	write_cmos_sensor_8(0x5655, 0x7f);
	write_cmos_sensor_8(0x5656, 0x7f);
	write_cmos_sensor_8(0x5657, 0x7f);
	write_cmos_sensor_8(0x5658, 0x7f);
	write_cmos_sensor_8(0x5659, 0x7f);
	write_cmos_sensor_8(0x565a, 0x7f);
	write_cmos_sensor_8(0x565b, 0x7f);
	write_cmos_sensor_8(0x565c, 0x7f);
	write_cmos_sensor_8(0x565d, 0x7f);
	write_cmos_sensor_8(0x565e, 0x80);
	write_cmos_sensor_8(0x565f, 0x7f);
	write_cmos_sensor_8(0x5660, 0x80);
	write_cmos_sensor_8(0x5661, 0x80);
	write_cmos_sensor_8(0x5662, 0x80);
	write_cmos_sensor_8(0x5663, 0x7f);
	write_cmos_sensor_8(0x5664, 0x7f);
	write_cmos_sensor_8(0x5665, 0x7f);
	write_cmos_sensor_8(0x5666, 0x7f);
	write_cmos_sensor_8(0x5667, 0x7f);
	write_cmos_sensor_8(0x5668, 0x7f);
	write_cmos_sensor_8(0x5669, 0x7f);
	write_cmos_sensor_8(0x566a, 0x80);
	write_cmos_sensor_8(0x566b, 0x80);
	write_cmos_sensor_8(0x566c, 0x80);
	write_cmos_sensor_8(0x566d, 0x80);
	write_cmos_sensor_8(0x566e, 0x7f);
	write_cmos_sensor_8(0x566f, 0x80);
	write_cmos_sensor_8(0x5670, 0x80);
	write_cmos_sensor_8(0x5671, 0x80);
	write_cmos_sensor_8(0x5672, 0x80);
	write_cmos_sensor_8(0x5673, 0x80);
	write_cmos_sensor_8(0x5674, 0x80);
	write_cmos_sensor_8(0x5675, 0x80);
	write_cmos_sensor_8(0x5676, 0x80);
	write_cmos_sensor_8(0x5677, 0x80);
	write_cmos_sensor_8(0x5678, 0x80);
	write_cmos_sensor_8(0x5679, 0x80);
	write_cmos_sensor_8(0x567a, 0x80);
	write_cmos_sensor_8(0x567b, 0x80);
	write_cmos_sensor_8(0x567c, 0x80);
	write_cmos_sensor_8(0x567d, 0x80);
	write_cmos_sensor_8(0x567e, 0x80);
	write_cmos_sensor_8(0x567f, 0x80);
	write_cmos_sensor_8(0x5680, 0x80);
	write_cmos_sensor_8(0x5681, 0x80);
	write_cmos_sensor_8(0x5682, 0x80);
	write_cmos_sensor_8(0x5683, 0x80);
	write_cmos_sensor_8(0x5684, 0x81);
	write_cmos_sensor_8(0x5685, 0x80);
	write_cmos_sensor_8(0x5686, 0x80);
	write_cmos_sensor_8(0x5687, 0x80);
	write_cmos_sensor_8(0x5688, 0x80);
	write_cmos_sensor_8(0x5689, 0x80);
	write_cmos_sensor_8(0x568a, 0x80);
	write_cmos_sensor_8(0x568b, 0x80);
	write_cmos_sensor_8(0x568c, 0x80);
	write_cmos_sensor_8(0x568d, 0x80);
	write_cmos_sensor_8(0x568e, 0x80);
	write_cmos_sensor_8(0x568f, 0x80);
	write_cmos_sensor_8(0x5690, 0x80);
	write_cmos_sensor_8(0x5691, 0x80);
	write_cmos_sensor_8(0x5692, 0x80);
	write_cmos_sensor_8(0x5693, 0x80);
	write_cmos_sensor_8(0x5694, 0x80);
	write_cmos_sensor_8(0x5695, 0x7f);
	write_cmos_sensor_8(0x5696, 0x80);
	write_cmos_sensor_8(0x5697, 0x80);
	write_cmos_sensor_8(0x5698, 0x80);
	write_cmos_sensor_8(0x5699, 0x80);
	write_cmos_sensor_8(0x569a, 0x80);
	write_cmos_sensor_8(0x569b, 0x80);
	write_cmos_sensor_8(0x569c, 0x80);
	write_cmos_sensor_8(0x569d, 0x80);
	write_cmos_sensor_8(0x569e, 0x80);
	write_cmos_sensor_8(0x569f, 0x81);
	write_cmos_sensor_8(0x56a0, 0x7f);
	write_cmos_sensor_8(0x56a1, 0x80);
	write_cmos_sensor_8(0x56a2, 0x80);
	write_cmos_sensor_8(0x56a3, 0x80);
	write_cmos_sensor_8(0x56a4, 0x80);
	write_cmos_sensor_8(0x56a5, 0x80);
	write_cmos_sensor_8(0x56a6, 0x7f);
	write_cmos_sensor_8(0x56a7, 0x7f);
	write_cmos_sensor_8(0x56a8, 0x7f);
	write_cmos_sensor_8(0x56a9, 0x7f);
	write_cmos_sensor_8(0x56aa, 0x80);
	write_cmos_sensor_8(0x56ab, 0x80);
	write_cmos_sensor_8(0x56ac, 0x80);
	write_cmos_sensor_8(0x56ad, 0x80);
	write_cmos_sensor_8(0x56ae, 0x80);
	write_cmos_sensor_8(0x56af, 0x80);
	write_cmos_sensor_8(0x56b0, 0x80);
	write_cmos_sensor_8(0x56b1, 0x80);
	write_cmos_sensor_8(0x56b2, 0x80);
	write_cmos_sensor_8(0x56b3, 0x7f);
	write_cmos_sensor_8(0x56b4, 0x80);
	write_cmos_sensor_8(0x56b5, 0x7f);
	write_cmos_sensor_8(0x56b6, 0x7f);
	write_cmos_sensor_8(0x56b7, 0x7f);
	write_cmos_sensor_8(0x56b8, 0x7f);
	write_cmos_sensor_8(0x56b9, 0x7f);
	write_cmos_sensor_8(0x56ba, 0x80);
	write_cmos_sensor_8(0x56bb, 0x7f);
	write_cmos_sensor_8(0x56bc, 0x80);
	write_cmos_sensor_8(0x56bd, 0x80);
	write_cmos_sensor_8(0x56be, 0x80);
	write_cmos_sensor_8(0x56bf, 0x80);
	write_cmos_sensor_8(0x56c0, 0x80);
	write_cmos_sensor_8(0x56c1, 0x80);
	write_cmos_sensor_8(0x56c2, 0x7f);
	write_cmos_sensor_8(0x56c3, 0x80);
	write_cmos_sensor_8(0x56c4, 0x7f);
	write_cmos_sensor_8(0x56c5, 0x7f);
	write_cmos_sensor_8(0x56c6, 0x7f);
	write_cmos_sensor_8(0x56c7, 0x7f);
	write_cmos_sensor_8(0x56c8, 0x7f);
	write_cmos_sensor_8(0x56c9, 0x7f);
	write_cmos_sensor_8(0x56ca, 0x7f);
	write_cmos_sensor_8(0x56cb, 0x80);
	write_cmos_sensor_8(0x56cc, 0x80);
	write_cmos_sensor_8(0x56cd, 0x7f);
	write_cmos_sensor_8(0x56ce, 0x80);
	write_cmos_sensor_8(0x56cf, 0x80);
	write_cmos_sensor_8(0x56d0, 0x81);
	write_cmos_sensor_8(0x56d1, 0x80);
	write_cmos_sensor_8(0x56d2, 0x7f);
	write_cmos_sensor_8(0x56d3, 0x80);
	write_cmos_sensor_8(0x56d4, 0x7f);
	write_cmos_sensor_8(0x56d5, 0x80);
	write_cmos_sensor_8(0x56d6, 0x7f);
	write_cmos_sensor_8(0x56d7, 0x7f);
	write_cmos_sensor_8(0x56d8, 0x80);
	write_cmos_sensor_8(0x56d9, 0x7f);
	write_cmos_sensor_8(0x56da, 0x80);
	write_cmos_sensor_8(0x56db, 0x7f);
	write_cmos_sensor_8(0x56dc, 0x7f);
	write_cmos_sensor_8(0x56dd, 0x80);
	write_cmos_sensor_8(0x56de, 0x80);
	write_cmos_sensor_8(0x56df, 0x81);
	write_cmos_sensor_8(0x56e0, 0x80);
	write_cmos_sensor_8(0x56e1, 0x7f);
	write_cmos_sensor_8(0x56e2, 0x80);
	write_cmos_sensor_8(0x56e3, 0x7f);
	write_cmos_sensor_8(0x56e4, 0x7f);
	write_cmos_sensor_8(0x56e5, 0x7f);
	write_cmos_sensor_8(0x56e6, 0x7f);
	write_cmos_sensor_8(0x56e7, 0x80);
	write_cmos_sensor_8(0x56e8, 0x7f);
	write_cmos_sensor_8(0x56e9, 0x80);
	write_cmos_sensor_8(0x56ea, 0x7f);
	write_cmos_sensor_8(0x56eb, 0x80);
	write_cmos_sensor_8(0x56ec, 0x80);
	write_cmos_sensor_8(0x56ed, 0x7f);
	write_cmos_sensor_8(0x56ee, 0x80);
	write_cmos_sensor_8(0x56ef, 0x7f);
	write_cmos_sensor_8(0x56f0, 0x7b);
	write_cmos_sensor_8(0x56f1, 0x81);
	write_cmos_sensor_8(0x56f2, 0x7f);
	write_cmos_sensor_8(0x56f3, 0x80);
	write_cmos_sensor_8(0x56f4, 0x81);
	write_cmos_sensor_8(0x56f5, 0x7f);
	write_cmos_sensor_8(0x56f6, 0x7f);
	write_cmos_sensor_8(0x56f7, 0x80);
	write_cmos_sensor_8(0x56f8, 0x80);
	write_cmos_sensor_8(0x56f9, 0x7f);
	write_cmos_sensor_8(0x56fa, 0x80);
	write_cmos_sensor_8(0x56fb, 0x7f);
	write_cmos_sensor_8(0x56fc, 0x7f);
	write_cmos_sensor_8(0x56fd, 0x81);
	write_cmos_sensor_8(0x56fe, 0x80);
	write_cmos_sensor_8(0x56ff, 0x7f);
	write_cmos_sensor_8(0x5700, 0x84);
	write_cmos_sensor_8(0x5701, 0x7e);
	write_cmos_sensor_8(0x5702, 0x81);
	write_cmos_sensor_8(0x5703, 0x7d);
	write_cmos_sensor_8(0x5704, 0x80);
	write_cmos_sensor_8(0x5705, 0x7f);
	write_cmos_sensor_8(0x5706, 0x7f);
	write_cmos_sensor_8(0x5707, 0x81);
	write_cmos_sensor_8(0x5708, 0x7e);
	write_cmos_sensor_8(0x5709, 0x7e);
	write_cmos_sensor_8(0x570a, 0x81);
	write_cmos_sensor_8(0x570b, 0x7f);
	write_cmos_sensor_8(0x570c, 0x80);
	write_cmos_sensor_8(0x570d, 0x7f);
	write_cmos_sensor_8(0x570e, 0x7f);
	write_cmos_sensor_8(0x570f, 0x81);
	write_cmos_sensor_8(0x5710, 0x7d);
	write_cmos_sensor_8(0x5711, 0x80);
	write_cmos_sensor_8(0x5712, 0x7e);
	write_cmos_sensor_8(0x5713, 0x80);
	write_cmos_sensor_8(0x5714, 0x7f);
	write_cmos_sensor_8(0x5715, 0x7f);
	write_cmos_sensor_8(0x5716, 0x7f);
	write_cmos_sensor_8(0x5717, 0x7f);
	write_cmos_sensor_8(0x5718, 0x80);
	write_cmos_sensor_8(0x5719, 0x80);
	write_cmos_sensor_8(0x571a, 0x7f);
	write_cmos_sensor_8(0x571b, 0x80);
	write_cmos_sensor_8(0x571c, 0x7f);
	write_cmos_sensor_8(0x571d, 0x80);
	write_cmos_sensor_8(0x571e, 0x7f);
	write_cmos_sensor_8(0x571f, 0x81);
	write_cmos_sensor_8(0x5720, 0x80);
	write_cmos_sensor_8(0x5721, 0x7f);
	write_cmos_sensor_8(0x5722, 0x80);
	write_cmos_sensor_8(0x5723, 0x7f);
	write_cmos_sensor_8(0x5724, 0x7f);
	write_cmos_sensor_8(0x5725, 0x7f);
	write_cmos_sensor_8(0x5726, 0x7f);
	write_cmos_sensor_8(0x5727, 0x7f);
	write_cmos_sensor_8(0x5728, 0x7f);
	write_cmos_sensor_8(0x5729, 0x7f);
	write_cmos_sensor_8(0x572a, 0x7f);
	write_cmos_sensor_8(0x572b, 0x7f);
	write_cmos_sensor_8(0x572c, 0x7f);
	write_cmos_sensor_8(0x572d, 0x7f);
	write_cmos_sensor_8(0x572e, 0x7f);
	write_cmos_sensor_8(0x572f, 0x81);
	write_cmos_sensor_8(0x5730, 0x7f);
	write_cmos_sensor_8(0x5731, 0x7f);
	write_cmos_sensor_8(0x5732, 0x7f);
	write_cmos_sensor_8(0x5733, 0x7f);
	write_cmos_sensor_8(0x5734, 0x7f);
	write_cmos_sensor_8(0x5735, 0x7f);
	write_cmos_sensor_8(0x5736, 0x7f);
	write_cmos_sensor_8(0x5737, 0x7f);
	write_cmos_sensor_8(0x5738, 0x7f);
	write_cmos_sensor_8(0x5739, 0x7f);
	write_cmos_sensor_8(0x573a, 0x7f);
	write_cmos_sensor_8(0x573b, 0x7f);
	write_cmos_sensor_8(0x573c, 0x7f);
	write_cmos_sensor_8(0x573d, 0x7f);
	write_cmos_sensor_8(0x573e, 0x7f);
	write_cmos_sensor_8(0x573f, 0x80);
	write_cmos_sensor_8(0x5740, 0x80);
	write_cmos_sensor_8(0x5741, 0x7f);
	write_cmos_sensor_8(0x5742, 0x80);
	write_cmos_sensor_8(0x5743, 0x7f);
	write_cmos_sensor_8(0x5744, 0x7f);
	write_cmos_sensor_8(0x5745, 0x7f);
	write_cmos_sensor_8(0x5746, 0x7f);
	write_cmos_sensor_8(0x5747, 0x7f);
	write_cmos_sensor_8(0x5748, 0x7f);
	write_cmos_sensor_8(0x5749, 0x7f);
	write_cmos_sensor_8(0x574a, 0x7f);
	write_cmos_sensor_8(0x574b, 0x7f);
	write_cmos_sensor_8(0x574c, 0x7f);
	write_cmos_sensor_8(0x574d, 0x7f);
	write_cmos_sensor_8(0x574e, 0x7f);
	write_cmos_sensor_8(0x574f, 0x80);
	write_cmos_sensor_8(0x5750, 0x80);
	write_cmos_sensor_8(0x5751, 0x7f);
	write_cmos_sensor_8(0x5752, 0x7f);
	write_cmos_sensor_8(0x5753, 0x7f);
	write_cmos_sensor_8(0x5754, 0x7f);
	write_cmos_sensor_8(0x5755, 0x7f);
	write_cmos_sensor_8(0x5756, 0x7f);
	write_cmos_sensor_8(0x5757, 0x7f);
	write_cmos_sensor_8(0x5758, 0x7f);
	write_cmos_sensor_8(0x5759, 0x7f);
	write_cmos_sensor_8(0x575a, 0x7f);
	write_cmos_sensor_8(0x575b, 0x7f);
	write_cmos_sensor_8(0x575c, 0x7f);
	write_cmos_sensor_8(0x575d, 0x7f);
	write_cmos_sensor_8(0x575e, 0x7f);
	write_cmos_sensor_8(0x575f, 0x80);
	write_cmos_sensor_8(0x5760, 0x7e);
	write_cmos_sensor_8(0x5761, 0x80);
	write_cmos_sensor_8(0x5762, 0x7f);
	write_cmos_sensor_8(0x5763, 0x7f);
	write_cmos_sensor_8(0x5764, 0x7f);
	write_cmos_sensor_8(0x5765, 0x7f);
	write_cmos_sensor_8(0x5766, 0x7f);
	write_cmos_sensor_8(0x5767, 0x7f);
	write_cmos_sensor_8(0x5768, 0x80);
	write_cmos_sensor_8(0x5769, 0x7f);
	write_cmos_sensor_8(0x576a, 0x80);
	write_cmos_sensor_8(0x576b, 0x7f);
	write_cmos_sensor_8(0x576c, 0x80);
	write_cmos_sensor_8(0x576d, 0x80);
	write_cmos_sensor_8(0x576e, 0x7f);
	write_cmos_sensor_8(0x576f, 0x80);
	write_cmos_sensor_8(0x5770, 0x80);
	write_cmos_sensor_8(0x5771, 0x80);
	write_cmos_sensor_8(0x5772, 0x80);
	write_cmos_sensor_8(0x5773, 0x80);
	write_cmos_sensor_8(0x5774, 0x80);
	write_cmos_sensor_8(0x5775, 0x80);
	write_cmos_sensor_8(0x5776, 0x80);
	write_cmos_sensor_8(0x5777, 0x80);
	write_cmos_sensor_8(0x5778, 0x80);
	write_cmos_sensor_8(0x5779, 0x80);
	write_cmos_sensor_8(0x577a, 0x80);
	write_cmos_sensor_8(0x577b, 0x80);
	write_cmos_sensor_8(0x577c, 0x7f);
	write_cmos_sensor_8(0x577d, 0x80);
	write_cmos_sensor_8(0x577e, 0x80);
	write_cmos_sensor_8(0x577f, 0x80);
	write_cmos_sensor_8(0x5780, 0x80);
	write_cmos_sensor_8(0x5781, 0x80);
	write_cmos_sensor_8(0x5782, 0x80);
	write_cmos_sensor_8(0x5783, 0x80);
	write_cmos_sensor_8(0x5784, 0x80);
	write_cmos_sensor_8(0x5785, 0x80);
	write_cmos_sensor_8(0x5786, 0x80);
	write_cmos_sensor_8(0x5787, 0x80);
	write_cmos_sensor_8(0x5788, 0x80);
	write_cmos_sensor_8(0x5789, 0x81);
	write_cmos_sensor_8(0x578a, 0x80);
	write_cmos_sensor_8(0x578b, 0x80);
	write_cmos_sensor_8(0x578c, 0x80);
	write_cmos_sensor_8(0x578d, 0x80);
	write_cmos_sensor_8(0x578e, 0x80);
	write_cmos_sensor_8(0x578f, 0x80);
	write_cmos_sensor_8(0x5790, 0x7f);
	write_cmos_sensor_8(0x5791, 0x80);
	write_cmos_sensor_8(0x5792, 0x80);
	write_cmos_sensor_8(0x5793, 0x80);
	write_cmos_sensor_8(0x5794, 0x80);
	write_cmos_sensor_8(0x5795, 0x80);
	write_cmos_sensor_8(0x5796, 0x80);
	write_cmos_sensor_8(0x5797, 0x80);
	write_cmos_sensor_8(0x5798, 0x80);
	write_cmos_sensor_8(0x5799, 0x7f);
	write_cmos_sensor_8(0x579a, 0x80);
	write_cmos_sensor_8(0x579b, 0x80);
	write_cmos_sensor_8(0x579c, 0x80);
	write_cmos_sensor_8(0x579d, 0x80);
	write_cmos_sensor_8(0x579e, 0x80);
	write_cmos_sensor_8(0x579f, 0x80);
	write_cmos_sensor_8(0x57a0, 0x7f);
	write_cmos_sensor_8(0x57a1, 0x80);
	write_cmos_sensor_8(0x57a2, 0x80);
	write_cmos_sensor_8(0x57a3, 0x7f);
	write_cmos_sensor_8(0x57a4, 0x80);
	write_cmos_sensor_8(0x57a5, 0x7f);
	write_cmos_sensor_8(0x57a6, 0x80);
	write_cmos_sensor_8(0x57a7, 0x7f);
	write_cmos_sensor_8(0x57a8, 0x7f);
	write_cmos_sensor_8(0x57a9, 0x7f);
	write_cmos_sensor_8(0x57aa, 0x80);
	write_cmos_sensor_8(0x57ab, 0x7f);
	write_cmos_sensor_8(0x57ac, 0x7f);
	write_cmos_sensor_8(0x57ad, 0x80);
	write_cmos_sensor_8(0x57ae, 0x80);
	write_cmos_sensor_8(0x57af, 0x81);
	write_cmos_sensor_8(0x57b0, 0x80);
	write_cmos_sensor_8(0x57b1, 0x7f);
	write_cmos_sensor_8(0x57b2, 0x7f);
	write_cmos_sensor_8(0x57b3, 0x7f);
	write_cmos_sensor_8(0x57b4, 0x7f);
	write_cmos_sensor_8(0x57b5, 0x7f);
	write_cmos_sensor_8(0x57b6, 0x7f);
	write_cmos_sensor_8(0x57b7, 0x7f);
	write_cmos_sensor_8(0x57b8, 0x7f);
	write_cmos_sensor_8(0x57b9, 0x7f);
	write_cmos_sensor_8(0x57ba, 0x7f);
	write_cmos_sensor_8(0x57bb, 0x7f);
	write_cmos_sensor_8(0x57bc, 0x80);
	write_cmos_sensor_8(0x57bd, 0x7f);
	write_cmos_sensor_8(0x57be, 0x80);
	write_cmos_sensor_8(0x57bf, 0x7f);
	write_cmos_sensor_8(0x57c0, 0x7f);
	write_cmos_sensor_8(0x57c1, 0x7f);
	write_cmos_sensor_8(0x57c2, 0x7f);
	write_cmos_sensor_8(0x57c3, 0x7f);
	write_cmos_sensor_8(0x57c4, 0x7f);
	write_cmos_sensor_8(0x57c5, 0x7f);
	write_cmos_sensor_8(0x57c6, 0x80);
	write_cmos_sensor_8(0x57c7, 0x7f);
	write_cmos_sensor_8(0x57c8, 0x80);
	write_cmos_sensor_8(0x57c9, 0x7f);
	write_cmos_sensor_8(0x57ca, 0x7f);
	write_cmos_sensor_8(0x57cb, 0x7f);
	write_cmos_sensor_8(0x57cc, 0x80);
	write_cmos_sensor_8(0x57cd, 0x80);
	write_cmos_sensor_8(0x57ce, 0x7f);
	write_cmos_sensor_8(0x57cf, 0x81);
	write_cmos_sensor_8(0x57d0, 0x80);
	write_cmos_sensor_8(0x57d1, 0x80);
	write_cmos_sensor_8(0x57d2, 0x80);
	write_cmos_sensor_8(0x57d3, 0x7f);
	write_cmos_sensor_8(0x57d4, 0x7f);
	write_cmos_sensor_8(0x57d5, 0x80);
	write_cmos_sensor_8(0x57d6, 0x7f);
	write_cmos_sensor_8(0x57d7, 0x7f);
	write_cmos_sensor_8(0x57d8, 0x7f);
	write_cmos_sensor_8(0x57d9, 0x7f);
	write_cmos_sensor_8(0x57da, 0x7f);
	write_cmos_sensor_8(0x57db, 0x80);
	write_cmos_sensor_8(0x57dc, 0x7f);
	write_cmos_sensor_8(0x57dd, 0x80);
	write_cmos_sensor_8(0x57de, 0x80);
	write_cmos_sensor_8(0x57df, 0x80);
	write_cmos_sensor_8(0x57e0, 0x7f);
	write_cmos_sensor_8(0x57e1, 0x7f);
	write_cmos_sensor_8(0x57e2, 0x80);
	write_cmos_sensor_8(0x57e3, 0x7f);
	write_cmos_sensor_8(0x57e4, 0x80);
	write_cmos_sensor_8(0x57e5, 0x7f);
	write_cmos_sensor_8(0x57e6, 0x80);
	write_cmos_sensor_8(0x57e7, 0x7f);
	write_cmos_sensor_8(0x57e8, 0x80);
	write_cmos_sensor_8(0x57e9, 0x7f);
	write_cmos_sensor_8(0x57ea, 0x80);
	write_cmos_sensor_8(0x57eb, 0x80);
	write_cmos_sensor_8(0x57ec, 0x80);
	write_cmos_sensor_8(0x57ed, 0x7f);
	write_cmos_sensor_8(0x57ee, 0x80);
	write_cmos_sensor_8(0x57ef, 0x80);
	write_cmos_sensor_8(0x57f0, 0x7f);
	write_cmos_sensor_8(0x57f1, 0x80);
	write_cmos_sensor_8(0x57f2, 0x7e);
	write_cmos_sensor_8(0x57f3, 0x80);
	write_cmos_sensor_8(0x57f4, 0x7e);
	write_cmos_sensor_8(0x57f5, 0x81);
	write_cmos_sensor_8(0x57f6, 0x7e);
	write_cmos_sensor_8(0x57f7, 0x81);
	write_cmos_sensor_8(0x57f8, 0x7e);
	write_cmos_sensor_8(0x57f9, 0x80);
	write_cmos_sensor_8(0x57fa, 0x80);
	write_cmos_sensor_8(0x57fb, 0x80);
	write_cmos_sensor_8(0x57fc, 0x7e);
	write_cmos_sensor_8(0x57fd, 0x81);
	write_cmos_sensor_8(0x57fe, 0x80);
	write_cmos_sensor_8(0x57ff, 0x7a);
	write_cmos_sensor_8(0x5820, 0x18);
	write_cmos_sensor_8(0x5821, 0x08);
	write_cmos_sensor_8(0x5822, 0x08);
	write_cmos_sensor_8(0x5823, 0x18);
	write_cmos_sensor_8(0x5824, 0x18);
	write_cmos_sensor_8(0x5825, 0x08);
	write_cmos_sensor_8(0x5826, 0x08);
	write_cmos_sensor_8(0x5827, 0x18);
	write_cmos_sensor_8(0x582c, 0x08);
	write_cmos_sensor_8(0x582d, 0x18);
	write_cmos_sensor_8(0x582e, 0x00);
	write_cmos_sensor_8(0x582f, 0x00);
	write_cmos_sensor_8(0x5830, 0x08);
	write_cmos_sensor_8(0x5831, 0x18);
	write_cmos_sensor_8(0x5836, 0x08);
	write_cmos_sensor_8(0x5837, 0x18);
	write_cmos_sensor_8(0x5838, 0x00);
	write_cmos_sensor_8(0x5839, 0x00);
	write_cmos_sensor_8(0x583a, 0x08);
	write_cmos_sensor_8(0x583b, 0x18);
	write_cmos_sensor_8(0x583c, 0x55);
	write_cmos_sensor_8(0x583e, 0x03);
	write_cmos_sensor_8(0x5860, 0x02);
	write_cmos_sensor_8(0x58a1, 0x04);
	write_cmos_sensor_8(0x58a2, 0x00);
	write_cmos_sensor_8(0x58a3, 0x00);
	write_cmos_sensor_8(0x58a4, 0x02);
	write_cmos_sensor_8(0x58a5, 0x00);
	write_cmos_sensor_8(0x58a6, 0x02);
	write_cmos_sensor_8(0x58a7, 0x00);
	write_cmos_sensor_8(0x58a8, 0x00);
	write_cmos_sensor_8(0x58a9, 0x00);
	write_cmos_sensor_8(0x58aa, 0x00);
	write_cmos_sensor_8(0x58ab, 0x00);
	write_cmos_sensor_8(0x58ac, 0x14);
	write_cmos_sensor_8(0x58ad, 0x60);
	write_cmos_sensor_8(0x58ae, 0x0f);
	write_cmos_sensor_8(0x58af, 0x50);
	write_cmos_sensor_8(0x58c4, 0x12);
	write_cmos_sensor_8(0x58c5, 0x60);
	write_cmos_sensor_8(0x58c6, 0x0d);
	write_cmos_sensor_8(0x58c7, 0xd0);
	write_cmos_sensor_8(0x5900, 0x3e);
	write_cmos_sensor_8(0x5901, 0x3e);
	write_cmos_sensor_8(0x5902, 0x3e);
	write_cmos_sensor_8(0x5903, 0x3e);
	write_cmos_sensor_8(0x5904, 0x3e);
	write_cmos_sensor_8(0x5905, 0x3e);
	write_cmos_sensor_8(0x5906, 0x3e);
	write_cmos_sensor_8(0x5907, 0x3e);
	write_cmos_sensor_8(0x5908, 0x3e);
	write_cmos_sensor_8(0x5909, 0x3e);
	write_cmos_sensor_8(0x590a, 0x3e);
	write_cmos_sensor_8(0x590b, 0x3e);
	write_cmos_sensor_8(0x590c, 0x3e);
	write_cmos_sensor_8(0x590d, 0x3e);
	write_cmos_sensor_8(0x590e, 0x3e);
	write_cmos_sensor_8(0x590f, 0x3e);
	write_cmos_sensor_8(0x5910, 0x3e);
	write_cmos_sensor_8(0x5911, 0x3e);
	write_cmos_sensor_8(0x5912, 0x3e);
	write_cmos_sensor_8(0x5913, 0x3e);
	write_cmos_sensor_8(0x5914, 0x3e);
	write_cmos_sensor_8(0x5915, 0x3e);
	write_cmos_sensor_8(0x5916, 0x3e);
	write_cmos_sensor_8(0x5917, 0x3e);
	write_cmos_sensor_8(0x5918, 0x3e);
	write_cmos_sensor_8(0x5919, 0x3e);
	write_cmos_sensor_8(0x591a, 0x3e);
	write_cmos_sensor_8(0x591b, 0x3e);
	write_cmos_sensor_8(0x591c, 0x3e);
	write_cmos_sensor_8(0x591d, 0x3e);
	write_cmos_sensor_8(0x591e, 0x3e);
	write_cmos_sensor_8(0x591f, 0x3e);
	write_cmos_sensor_8(0x5920, 0x3e);
	write_cmos_sensor_8(0x5921, 0x3e);
	write_cmos_sensor_8(0x5922, 0x3e);
	write_cmos_sensor_8(0x5923, 0x3e);
	write_cmos_sensor_8(0x5924, 0x3e);
	write_cmos_sensor_8(0x5925, 0x3e);
	write_cmos_sensor_8(0x5926, 0x3e);
	write_cmos_sensor_8(0x5927, 0x3e);
	write_cmos_sensor_8(0x5928, 0x3e);
	write_cmos_sensor_8(0x5929, 0x3e);
	write_cmos_sensor_8(0x592a, 0x3e);
	write_cmos_sensor_8(0x592b, 0x3e);
	write_cmos_sensor_8(0x592c, 0x3e);
	write_cmos_sensor_8(0x592d, 0x40);
	write_cmos_sensor_8(0x592e, 0x40);
	write_cmos_sensor_8(0x592f, 0x40);
	write_cmos_sensor_8(0x5930, 0x40);
	write_cmos_sensor_8(0x5931, 0x40);
	write_cmos_sensor_8(0x5932, 0x40);
	write_cmos_sensor_8(0x5933, 0x40);
	write_cmos_sensor_8(0x5934, 0x40);
	write_cmos_sensor_8(0x5935, 0x40);
	write_cmos_sensor_8(0x5936, 0x40);
	write_cmos_sensor_8(0x5937, 0x40);
	write_cmos_sensor_8(0x5938, 0x40);
	write_cmos_sensor_8(0x5939, 0x40);
	write_cmos_sensor_8(0x593a, 0x40);
	write_cmos_sensor_8(0x593b, 0x40);
	write_cmos_sensor_8(0x593c, 0x40);
	write_cmos_sensor_8(0x593d, 0x40);
	write_cmos_sensor_8(0x593e, 0x40);
	write_cmos_sensor_8(0x593f, 0x40);
	write_cmos_sensor_8(0x5940, 0x40);
	write_cmos_sensor_8(0x5941, 0x40);
	write_cmos_sensor_8(0x5942, 0x40);
	write_cmos_sensor_8(0x5943, 0x40);
	write_cmos_sensor_8(0x5944, 0x40);
	write_cmos_sensor_8(0x5945, 0x40);
	write_cmos_sensor_8(0x5946, 0x40);
	write_cmos_sensor_8(0x5947, 0x40);
	write_cmos_sensor_8(0x5948, 0x40);
	write_cmos_sensor_8(0x5949, 0x40);
	write_cmos_sensor_8(0x594a, 0x40);
	write_cmos_sensor_8(0x594b, 0x40);
	write_cmos_sensor_8(0x594c, 0x40);
	write_cmos_sensor_8(0x594d, 0x40);
	write_cmos_sensor_8(0x594e, 0x40);
	write_cmos_sensor_8(0x594f, 0x40);
	write_cmos_sensor_8(0x5950, 0x40);
	write_cmos_sensor_8(0x5951, 0x40);
	write_cmos_sensor_8(0x5952, 0x40);
	write_cmos_sensor_8(0x5953, 0x40);
	write_cmos_sensor_8(0x5954, 0x40);
	write_cmos_sensor_8(0x5955, 0x40);
	write_cmos_sensor_8(0x5956, 0x40);
	write_cmos_sensor_8(0x5957, 0x40);
	write_cmos_sensor_8(0x5958, 0x40);
	write_cmos_sensor_8(0x5959, 0x40);
	write_cmos_sensor_8(0x595a, 0x40);
	write_cmos_sensor_8(0x595b, 0x40);
	write_cmos_sensor_8(0x595c, 0x40);
	write_cmos_sensor_8(0x595d, 0x40);
	write_cmos_sensor_8(0x595e, 0x40);
	write_cmos_sensor_8(0x595f, 0x40);
	write_cmos_sensor_8(0x5960, 0x40);
	write_cmos_sensor_8(0x5961, 0x40);
	write_cmos_sensor_8(0x5962, 0x40);
	write_cmos_sensor_8(0x5963, 0x40);
	write_cmos_sensor_8(0x5964, 0x40);
	write_cmos_sensor_8(0x5965, 0x40);
	write_cmos_sensor_8(0x5966, 0x40);
	write_cmos_sensor_8(0x5967, 0x40);
	write_cmos_sensor_8(0x5968, 0x40);
	write_cmos_sensor_8(0x5969, 0x40);
	write_cmos_sensor_8(0x596a, 0x40);
	write_cmos_sensor_8(0x596b, 0x40);
	write_cmos_sensor_8(0x596c, 0x40);
	write_cmos_sensor_8(0x596d, 0x40);
	write_cmos_sensor_8(0x596e, 0x40);
	write_cmos_sensor_8(0x596f, 0x40);
	write_cmos_sensor_8(0x5970, 0x40);
	write_cmos_sensor_8(0x5971, 0x40);
	write_cmos_sensor_8(0x5972, 0x40);
	write_cmos_sensor_8(0x5973, 0x40);
	write_cmos_sensor_8(0x5974, 0x40);
	write_cmos_sensor_8(0x5975, 0x40);
	write_cmos_sensor_8(0x5976, 0x40);
	write_cmos_sensor_8(0x5977, 0x40);
	write_cmos_sensor_8(0x5978, 0x40);
	write_cmos_sensor_8(0x5979, 0x40);
	write_cmos_sensor_8(0x597a, 0x40);
	write_cmos_sensor_8(0x597b, 0x40);
	write_cmos_sensor_8(0x597c, 0x40);
	write_cmos_sensor_8(0x597d, 0x40);
	write_cmos_sensor_8(0x597e, 0x40);
	write_cmos_sensor_8(0x597f, 0x40);
	write_cmos_sensor_8(0x5980, 0x40);
	write_cmos_sensor_8(0x5981, 0x40);
	write_cmos_sensor_8(0x5982, 0x40);
	write_cmos_sensor_8(0x5983, 0x40);
	write_cmos_sensor_8(0x5984, 0x40);
	write_cmos_sensor_8(0x5985, 0x40);
	write_cmos_sensor_8(0x5986, 0x40);
	write_cmos_sensor_8(0x5987, 0x40);
	write_cmos_sensor_8(0x5988, 0x40);
	write_cmos_sensor_8(0x5989, 0x40);
	write_cmos_sensor_8(0x598a, 0x40);
	write_cmos_sensor_8(0x598b, 0x40);
	write_cmos_sensor_8(0x598c, 0x40);
	write_cmos_sensor_8(0x598d, 0x40);
	write_cmos_sensor_8(0x598e, 0x40);
	write_cmos_sensor_8(0x598f, 0x40);
	write_cmos_sensor_8(0x5990, 0x40);
	write_cmos_sensor_8(0x5991, 0x40);
	write_cmos_sensor_8(0x5992, 0x40);
	write_cmos_sensor_8(0x5993, 0x40);
	write_cmos_sensor_8(0x5994, 0x40);
	write_cmos_sensor_8(0x5995, 0x40);
	write_cmos_sensor_8(0x5996, 0x40);
	write_cmos_sensor_8(0x5997, 0x40);
	write_cmos_sensor_8(0x5998, 0x40);
	write_cmos_sensor_8(0x5999, 0x40);
	write_cmos_sensor_8(0x599a, 0x40);
	write_cmos_sensor_8(0x599b, 0x40);
	write_cmos_sensor_8(0x599c, 0x40);
	write_cmos_sensor_8(0x599d, 0x40);
	write_cmos_sensor_8(0x599e, 0x40);
	write_cmos_sensor_8(0x599f, 0x40);
	write_cmos_sensor_8(0x59a0, 0x40);
	write_cmos_sensor_8(0x59a1, 0x40);
	write_cmos_sensor_8(0x59a2, 0x40);
	write_cmos_sensor_8(0x59a3, 0x40);
	write_cmos_sensor_8(0x59a4, 0x40);
	write_cmos_sensor_8(0x59a5, 0x40);
	write_cmos_sensor_8(0x59a6, 0x40);
	write_cmos_sensor_8(0x59a7, 0x40);
	write_cmos_sensor_8(0x59a8, 0x40);
	write_cmos_sensor_8(0x59a9, 0x40);
	write_cmos_sensor_8(0x59aa, 0x40);
	write_cmos_sensor_8(0x59ab, 0x40);
	write_cmos_sensor_8(0x59ac, 0x40);
	write_cmos_sensor_8(0x59ad, 0x40);
	write_cmos_sensor_8(0x59ae, 0x40);
	write_cmos_sensor_8(0x59af, 0x40);
	write_cmos_sensor_8(0x59b0, 0x40);
	write_cmos_sensor_8(0x59b1, 0x40);
	write_cmos_sensor_8(0x59b2, 0x40);
	write_cmos_sensor_8(0x59b3, 0x40);
	write_cmos_sensor_8(0x59b4, 0x01);
	write_cmos_sensor_8(0x59b5, 0x02);
	write_cmos_sensor_8(0x59b8, 0x00);
	write_cmos_sensor_8(0x59b9, 0x7c);
	write_cmos_sensor_8(0x59ba, 0x00);
	write_cmos_sensor_8(0x59bb, 0xa8);
	write_cmos_sensor_8(0x59bc, 0x12);
	write_cmos_sensor_8(0x59bd, 0x60);
	write_cmos_sensor_8(0x59be, 0x0d);
	write_cmos_sensor_8(0x59bf, 0xd0);
	write_cmos_sensor_8(0x59c0, 0x00);
	write_cmos_sensor_8(0x59c1, 0x00);
	write_cmos_sensor_8(0x59c2, 0x00);
	write_cmos_sensor_8(0x59c3, 0x00);
	write_cmos_sensor_8(0x59c4, 0x00);
	write_cmos_sensor_8(0x59c5, 0x10);
	write_cmos_sensor_8(0x59c6, 0x12);
	write_cmos_sensor_8(0x59c7, 0x50);
	write_cmos_sensor_8(0x59c8, 0x00);
	write_cmos_sensor_8(0x59c9, 0x08);
	write_cmos_sensor_8(0x59ca, 0x0d);
	write_cmos_sensor_8(0x59cb, 0xb8);
	write_cmos_sensor_8(0x59cc, 0x01);
	write_cmos_sensor_8(0x59cd, 0x00);
	write_cmos_sensor_8(0x59ce, 0x01);
	write_cmos_sensor_8(0x59cf, 0x00);
	write_cmos_sensor_8(0x59d0, 0x01);
	write_cmos_sensor_8(0x59d1, 0x00);
	write_cmos_sensor_8(0x59d2, 0x01);
	write_cmos_sensor_8(0x59d3, 0x00);
	write_cmos_sensor_8(0x59d4, 0x00);
	write_cmos_sensor_8(0x59d5, 0x00);
	write_cmos_sensor_8(0x59d6, 0x00);
	write_cmos_sensor_8(0x59d7, 0x00);
	write_cmos_sensor_8(0x59d8, 0x00);
	write_cmos_sensor_8(0x59d9, 0x00);
	write_cmos_sensor_8(0x59da, 0x00);
	write_cmos_sensor_8(0x59db, 0x00);
	write_cmos_sensor_8(0x59dc, 0x20);
	write_cmos_sensor_8(0x59dd, 0x00);
	write_cmos_sensor_8(0x59de, 0x20);
	write_cmos_sensor_8(0x59df, 0x00);
	write_cmos_sensor_8(0x59e0, 0x00);
	write_cmos_sensor_8(0x59e1, 0x00);
	write_cmos_sensor_8(0x59e2, 0x00);
	write_cmos_sensor_8(0x59e3, 0x00);
	write_cmos_sensor_8(0x59e4, 0x00);
	write_cmos_sensor_8(0x59e5, 0x00);
	write_cmos_sensor_8(0x59e6, 0x00);
	write_cmos_sensor_8(0x59e7, 0x00);
	write_cmos_sensor_8(0x59e8, 0x00);
	write_cmos_sensor_8(0x59e9, 0x00);
	write_cmos_sensor_8(0x59ea, 0x00);
	write_cmos_sensor_8(0x59eb, 0x00);
	write_cmos_sensor_8(0x59ec, 0x20);
	write_cmos_sensor_8(0x59ed, 0x00);
	write_cmos_sensor_8(0x59ee, 0x20);
	write_cmos_sensor_8(0x59ef, 0x00);
	write_cmos_sensor_8(0x59f0, 0x00);
	write_cmos_sensor_8(0x59f1, 0x00);
	write_cmos_sensor_8(0x59f2, 0x00);
	write_cmos_sensor_8(0x59f3, 0x00);
	write_cmos_sensor_8(0x59f4, 0x00);
	write_cmos_sensor_8(0x59f5, 0x00);
	write_cmos_sensor_8(0x59f6, 0x00);
	write_cmos_sensor_8(0x59f7, 0x00);
	write_cmos_sensor_8(0x59f8, 0x00);
	write_cmos_sensor_8(0x59f9, 0x00);
	write_cmos_sensor_8(0x59fa, 0x00);
	write_cmos_sensor_8(0x59fb, 0x00);
	write_cmos_sensor_8(0x59fc, 0x00);
	write_cmos_sensor_8(0x59fd, 0x20);
	write_cmos_sensor_8(0x59fe, 0x00);
	write_cmos_sensor_8(0x59ff, 0x20);
	write_cmos_sensor_8(0x5a00, 0x00);
	write_cmos_sensor_8(0x5a01, 0x00);
	write_cmos_sensor_8(0x5a02, 0x00);
	write_cmos_sensor_8(0x5a03, 0x00);
	write_cmos_sensor_8(0x5a04, 0x00);
	write_cmos_sensor_8(0x5a05, 0x00);
	write_cmos_sensor_8(0x5a06, 0x00);
	write_cmos_sensor_8(0x5a07, 0x00);
	write_cmos_sensor_8(0x5a08, 0x00);
	write_cmos_sensor_8(0x5a09, 0x00);
	write_cmos_sensor_8(0x5a0a, 0x00);
	write_cmos_sensor_8(0x5a0b, 0x00);
	write_cmos_sensor_8(0x5a0c, 0x00);
	write_cmos_sensor_8(0x5a0d, 0x20);
	write_cmos_sensor_8(0x5a0e, 0x00);
	write_cmos_sensor_8(0x5a0f, 0x20);
	write_cmos_sensor_8(0x5a10, 0x00);
	write_cmos_sensor_8(0x5a11, 0x00);
	write_cmos_sensor_8(0x5a12, 0x00);
	write_cmos_sensor_8(0x5a13, 0x00);
	write_cmos_sensor_8(0x5a14, 0x00);
	write_cmos_sensor_8(0x5a15, 0x00);
	write_cmos_sensor_8(0x5a16, 0x00);
	write_cmos_sensor_8(0x5a17, 0x00);
	write_cmos_sensor_8(0x5a18, 0x00);
	write_cmos_sensor_8(0x5a19, 0x00);
	write_cmos_sensor_8(0x5a1a, 0x00);
	write_cmos_sensor_8(0x5a1b, 0x00);
	write_cmos_sensor_8(0x5a1c, 0x20);
	write_cmos_sensor_8(0x5a1d, 0x00);
	write_cmos_sensor_8(0x5a1e, 0x20);
	write_cmos_sensor_8(0x5a1f, 0x00);
	write_cmos_sensor_8(0x5a20, 0x00);
	write_cmos_sensor_8(0x5a21, 0x00);
	write_cmos_sensor_8(0x5a22, 0x00);
	write_cmos_sensor_8(0x5a23, 0x00);
	write_cmos_sensor_8(0x5a24, 0x00);
	write_cmos_sensor_8(0x5a25, 0x00);
	write_cmos_sensor_8(0x5a26, 0x00);
	write_cmos_sensor_8(0x5a27, 0x00);
	write_cmos_sensor_8(0x5a28, 0x00);
	write_cmos_sensor_8(0x5a29, 0x00);
	write_cmos_sensor_8(0x5a2a, 0x00);
	write_cmos_sensor_8(0x5a2b, 0x00);
	write_cmos_sensor_8(0x5a2c, 0x20);
	write_cmos_sensor_8(0x5a2d, 0x00);
	write_cmos_sensor_8(0x5a2e, 0x20);
	write_cmos_sensor_8(0x5a2f, 0x00);
	write_cmos_sensor_8(0x5a30, 0x00);
	write_cmos_sensor_8(0x5a31, 0x00);
	write_cmos_sensor_8(0x5a32, 0x00);
	write_cmos_sensor_8(0x5a33, 0x00);
	write_cmos_sensor_8(0x5a34, 0x00);
	write_cmos_sensor_8(0x5a35, 0x00);
	write_cmos_sensor_8(0x5a36, 0x00);
	write_cmos_sensor_8(0x5a37, 0x00);
	write_cmos_sensor_8(0x5a38, 0x00);
	write_cmos_sensor_8(0x5a39, 0x00);
	write_cmos_sensor_8(0x5a3a, 0x00);
	write_cmos_sensor_8(0x5a3b, 0x00);
	write_cmos_sensor_8(0x5a3c, 0x00);
	write_cmos_sensor_8(0x5a3d, 0x20);
	write_cmos_sensor_8(0x5a3e, 0x00);
	write_cmos_sensor_8(0x5a3f, 0x20);
	write_cmos_sensor_8(0x5a40, 0x00);
	write_cmos_sensor_8(0x5a41, 0x00);
	write_cmos_sensor_8(0x5a42, 0x00);
	write_cmos_sensor_8(0x5a43, 0x00);
	write_cmos_sensor_8(0x5a44, 0x00);
	write_cmos_sensor_8(0x5a45, 0x00);
	write_cmos_sensor_8(0x5a46, 0x00);
	write_cmos_sensor_8(0x5a47, 0x00);
	write_cmos_sensor_8(0x5a48, 0x00);
	write_cmos_sensor_8(0x5a49, 0x00);
	write_cmos_sensor_8(0x5a4a, 0x00);
	write_cmos_sensor_8(0x5a4b, 0x00);
	write_cmos_sensor_8(0x5a4c, 0x00);
	write_cmos_sensor_8(0x5a4d, 0x20);
	write_cmos_sensor_8(0x5a4e, 0x00);
	write_cmos_sensor_8(0x5a4f, 0x20);
	write_cmos_sensor_8(0x5a50, 0x00);
	write_cmos_sensor_8(0x5a51, 0x00);
	write_cmos_sensor_8(0x5a52, 0x00);
	write_cmos_sensor_8(0x5a53, 0x00);
	write_cmos_sensor_8(0x5a54, 0x00);
	write_cmos_sensor_8(0x5a55, 0x00);
	write_cmos_sensor_8(0x5a56, 0x00);
	write_cmos_sensor_8(0x5a57, 0x00);
	write_cmos_sensor_8(0x5a58, 0x00);
	write_cmos_sensor_8(0x5a59, 0x00);
	write_cmos_sensor_8(0x5a5a, 0x00);
	write_cmos_sensor_8(0x5a5b, 0x00);
	write_cmos_sensor_8(0x5a5c, 0x00);
	write_cmos_sensor_8(0x5a5d, 0x00);
	write_cmos_sensor_8(0x5a5e, 0x00);
	write_cmos_sensor_8(0x5a5f, 0x00);
	write_cmos_sensor_8(0x5a60, 0x00);
	write_cmos_sensor_8(0x5a61, 0x00);
	write_cmos_sensor_8(0x5a62, 0x00);
	write_cmos_sensor_8(0x5a63, 0x00);
	write_cmos_sensor_8(0x5a64, 0x08);
	write_cmos_sensor_8(0x5a65, 0x00);
	write_cmos_sensor_8(0x5a66, 0x00);
	write_cmos_sensor_8(0x5a67, 0x00);
	write_cmos_sensor_8(0x5a68, 0x08);
	write_cmos_sensor_8(0x5a69, 0x00);
	write_cmos_sensor_8(0x5a6a, 0x00);
	write_cmos_sensor_8(0x5a6b, 0x00);
	write_cmos_sensor_8(0x5a6c, 0x00);
	write_cmos_sensor_8(0x5a6d, 0x00);
	write_cmos_sensor_8(0x5a6e, 0x00);
	write_cmos_sensor_8(0x5a6f, 0x00);
	write_cmos_sensor_8(0x5a70, 0x00);
	write_cmos_sensor_8(0x5a71, 0x00);
	write_cmos_sensor_8(0x5a72, 0x00);
	write_cmos_sensor_8(0x5a73, 0x00);
	write_cmos_sensor_8(0x5a74, 0x00);
	write_cmos_sensor_8(0x5a75, 0x00);
	write_cmos_sensor_8(0x5a76, 0x00);
	write_cmos_sensor_8(0x5a77, 0x00);
	write_cmos_sensor_8(0x5a78, 0x00);
	write_cmos_sensor_8(0x5a79, 0x00);
	write_cmos_sensor_8(0x5a7a, 0x00);
	write_cmos_sensor_8(0x5a7b, 0x00);
	write_cmos_sensor_8(0x5a7c, 0x00);
	write_cmos_sensor_8(0x5a7d, 0x00);
	write_cmos_sensor_8(0x5a7e, 0x00);
	write_cmos_sensor_8(0x5a7f, 0x00);
	write_cmos_sensor_8(0x5a80, 0x00);
	write_cmos_sensor_8(0x5a81, 0x00);
	write_cmos_sensor_8(0x5a82, 0x00);
	write_cmos_sensor_8(0x5a83, 0x00);
	write_cmos_sensor_8(0x5a84, 0x0c);
	write_cmos_sensor_8(0x5a85, 0x00);
	write_cmos_sensor_8(0x5a86, 0x00);
	write_cmos_sensor_8(0x5a87, 0x00);
	write_cmos_sensor_8(0x5a88, 0x0c);
	write_cmos_sensor_8(0x5a89, 0x00);
	write_cmos_sensor_8(0x5a8a, 0x00);
	write_cmos_sensor_8(0x5a8b, 0x00);
	write_cmos_sensor_8(0x5a8c, 0x00);
	write_cmos_sensor_8(0x5a8d, 0x00);
	write_cmos_sensor_8(0x5a8e, 0x00);
	write_cmos_sensor_8(0x5a8f, 0x00);
	write_cmos_sensor_8(0x5a90, 0x00);
	write_cmos_sensor_8(0x5a91, 0x00);
	write_cmos_sensor_8(0x5a92, 0x00);
	write_cmos_sensor_8(0x5a93, 0x00);
	write_cmos_sensor_8(0x5a94, 0x00);
	write_cmos_sensor_8(0x5a95, 0x00);
	write_cmos_sensor_8(0x5a96, 0x00);
	write_cmos_sensor_8(0x5a97, 0x00);
	write_cmos_sensor_8(0x5a98, 0x00);
	write_cmos_sensor_8(0x5a99, 0x00);
	write_cmos_sensor_8(0x5a9a, 0x00);
	write_cmos_sensor_8(0x5a9b, 0x00);
	write_cmos_sensor_8(0x5a9c, 0x00);
	write_cmos_sensor_8(0x5a9d, 0x00);
	write_cmos_sensor_8(0x5a9e, 0x00);
	write_cmos_sensor_8(0x5a9f, 0x00);
	write_cmos_sensor_8(0x5aa0, 0x00);
	write_cmos_sensor_8(0x5aa1, 0x00);
	write_cmos_sensor_8(0x5aa2, 0x00);
	write_cmos_sensor_8(0x5aa3, 0x00);
	write_cmos_sensor_8(0x5aa4, 0x00);
	write_cmos_sensor_8(0x5aa5, 0x00);
	write_cmos_sensor_8(0x5aa6, 0x08);
	write_cmos_sensor_8(0x5aa7, 0x00);
	write_cmos_sensor_8(0x5aa8, 0x00);
	write_cmos_sensor_8(0x5aa9, 0x00);
	write_cmos_sensor_8(0x5aaa, 0x08);
	write_cmos_sensor_8(0x5aab, 0x00);
	write_cmos_sensor_8(0x5aac, 0x00);
	write_cmos_sensor_8(0x5aad, 0x00);
	write_cmos_sensor_8(0x5aae, 0x00);
	write_cmos_sensor_8(0x5aaf, 0x00);
	write_cmos_sensor_8(0x5ab0, 0x00);
	write_cmos_sensor_8(0x5ab1, 0x00);
	write_cmos_sensor_8(0x5ab2, 0x00);
	write_cmos_sensor_8(0x5ab3, 0x00);
	write_cmos_sensor_8(0x5ab4, 0x00);
	write_cmos_sensor_8(0x5ab5, 0x00);
	write_cmos_sensor_8(0x5ab6, 0x00);
	write_cmos_sensor_8(0x5ab7, 0x00);
	write_cmos_sensor_8(0x5ab8, 0x00);
	write_cmos_sensor_8(0x5ab9, 0x00);
	write_cmos_sensor_8(0x5aba, 0x00);
	write_cmos_sensor_8(0x5abb, 0x00);
	write_cmos_sensor_8(0x5abc, 0x00);
	write_cmos_sensor_8(0x5abd, 0x00);
	write_cmos_sensor_8(0x5abe, 0x00);
	write_cmos_sensor_8(0x5abf, 0x00);
	write_cmos_sensor_8(0x5ac0, 0x00);
	write_cmos_sensor_8(0x5ac1, 0x00);
	write_cmos_sensor_8(0x5ac2, 0x00);
	write_cmos_sensor_8(0x5ac3, 0x00);
	write_cmos_sensor_8(0x5ac4, 0x00);
	write_cmos_sensor_8(0x5ac5, 0x00);
	write_cmos_sensor_8(0x5ac6, 0x0c);
	write_cmos_sensor_8(0x5ac7, 0x00);
	write_cmos_sensor_8(0x5ac8, 0x00);
	write_cmos_sensor_8(0x5ac9, 0x00);
	write_cmos_sensor_8(0x5aca, 0x0c);
	write_cmos_sensor_8(0x5acb, 0x00);
	write_cmos_sensor_8(0x5acc, 0x00);
	write_cmos_sensor_8(0x5acd, 0x00);
	write_cmos_sensor_8(0x5ace, 0x00);
	write_cmos_sensor_8(0x5acf, 0x00);
	write_cmos_sensor_8(0x5ad0, 0x00);
	write_cmos_sensor_8(0x5ad1, 0x00);
	write_cmos_sensor_8(0x5ad2, 0x00);
	write_cmos_sensor_8(0x5ad3, 0x00);
	write_cmos_sensor_8(0x5ad4, 0x00);
	write_cmos_sensor_8(0x5ad5, 0x00);
	write_cmos_sensor_8(0x5ad6, 0x00);
	write_cmos_sensor_8(0x5ad7, 0x00);
	write_cmos_sensor_8(0x5ad8, 0x00);
	write_cmos_sensor_8(0x5ad9, 0x00);
	write_cmos_sensor_8(0x5ada, 0x00);
	write_cmos_sensor_8(0x5adb, 0x00);
	write_cmos_sensor_8(0x5adc, 0x00);
	write_cmos_sensor_8(0x5add, 0x00);
	write_cmos_sensor_8(0x5ade, 0x00);
	write_cmos_sensor_8(0x5adf, 0x00);
	write_cmos_sensor_8(0x5ae0, 0x00);
	write_cmos_sensor_8(0x5ae1, 0x00);
	write_cmos_sensor_8(0x5ae2, 0x00);
	write_cmos_sensor_8(0x5ae3, 0x00);
	write_cmos_sensor_8(0x5ae4, 0x08);
	write_cmos_sensor_8(0x5ae5, 0x00);
	write_cmos_sensor_8(0x5ae6, 0x00);
	write_cmos_sensor_8(0x5ae7, 0x00);
	write_cmos_sensor_8(0x5ae8, 0x08);
	write_cmos_sensor_8(0x5ae9, 0x00);
	write_cmos_sensor_8(0x5aea, 0x00);
	write_cmos_sensor_8(0x5aeb, 0x00);
	write_cmos_sensor_8(0x5aec, 0x00);
	write_cmos_sensor_8(0x5aed, 0x00);
	write_cmos_sensor_8(0x5aee, 0x00);
	write_cmos_sensor_8(0x5aef, 0x00);
	write_cmos_sensor_8(0x5af0, 0x00);
	write_cmos_sensor_8(0x5af1, 0x00);
	write_cmos_sensor_8(0x5af2, 0x00);
	write_cmos_sensor_8(0x5af3, 0x00);
	write_cmos_sensor_8(0x5af4, 0x00);
	write_cmos_sensor_8(0x5af5, 0x00);
	write_cmos_sensor_8(0x5af6, 0x00);
	write_cmos_sensor_8(0x5af7, 0x00);
	write_cmos_sensor_8(0x5af8, 0x00);
	write_cmos_sensor_8(0x5af9, 0x00);
	write_cmos_sensor_8(0x5afa, 0x00);
	write_cmos_sensor_8(0x5afb, 0x00);
	write_cmos_sensor_8(0x5afc, 0x00);
	write_cmos_sensor_8(0x5afd, 0x00);
	write_cmos_sensor_8(0x5afe, 0x00);
	write_cmos_sensor_8(0x5aff, 0x00);
	write_cmos_sensor_8(0x5b00, 0x00);
	write_cmos_sensor_8(0x5b01, 0x00);
	write_cmos_sensor_8(0x5b02, 0x00);
	write_cmos_sensor_8(0x5b03, 0x00);
	write_cmos_sensor_8(0x5b04, 0x0c);
	write_cmos_sensor_8(0x5b05, 0x00);
	write_cmos_sensor_8(0x5b06, 0x00);
	write_cmos_sensor_8(0x5b07, 0x00);
	write_cmos_sensor_8(0x5b08, 0x0c);
	write_cmos_sensor_8(0x5b09, 0x00);
	write_cmos_sensor_8(0x5b0a, 0x00);
	write_cmos_sensor_8(0x5b0b, 0x00);
	write_cmos_sensor_8(0x5b0c, 0x00);
	write_cmos_sensor_8(0x5b0d, 0x00);
	write_cmos_sensor_8(0x5b0e, 0x00);
	write_cmos_sensor_8(0x5b0f, 0x00);
	write_cmos_sensor_8(0x5b10, 0x00);
	write_cmos_sensor_8(0x5b11, 0x00);
	write_cmos_sensor_8(0x5b12, 0x00);
	write_cmos_sensor_8(0x5b13, 0x00);
	write_cmos_sensor_8(0x5b14, 0x00);
	write_cmos_sensor_8(0x5b15, 0x00);
	write_cmos_sensor_8(0x5b16, 0x00);
	write_cmos_sensor_8(0x5b17, 0x00);
	write_cmos_sensor_8(0x5b18, 0x00);
	write_cmos_sensor_8(0x5b19, 0x00);
	write_cmos_sensor_8(0x5b1a, 0x00);
	write_cmos_sensor_8(0x5b1b, 0x00);
	write_cmos_sensor_8(0x5b1c, 0x00);
	write_cmos_sensor_8(0x5b1d, 0x00);
	write_cmos_sensor_8(0x5b1e, 0x00);
	write_cmos_sensor_8(0x5b1f, 0x00);
	write_cmos_sensor_8(0x5b20, 0x00);
	write_cmos_sensor_8(0x5b21, 0x00);
	write_cmos_sensor_8(0x5b22, 0x00);
	write_cmos_sensor_8(0x5b23, 0x00);
	write_cmos_sensor_8(0x5b24, 0x00);
	write_cmos_sensor_8(0x5b25, 0x00);
	write_cmos_sensor_8(0x5b26, 0x08);
	write_cmos_sensor_8(0x5b27, 0x00);
	write_cmos_sensor_8(0x5b28, 0x00);
	write_cmos_sensor_8(0x5b29, 0x00);
	write_cmos_sensor_8(0x5b2a, 0x08);
	write_cmos_sensor_8(0x5b2b, 0x00);
	write_cmos_sensor_8(0x5b2c, 0x00);
	write_cmos_sensor_8(0x5b2d, 0x00);
	write_cmos_sensor_8(0x5b2e, 0x00);
	write_cmos_sensor_8(0x5b2f, 0x00);
	write_cmos_sensor_8(0x5b30, 0x00);
	write_cmos_sensor_8(0x5b31, 0x00);
	write_cmos_sensor_8(0x5b32, 0x00);
	write_cmos_sensor_8(0x5b33, 0x00);
	write_cmos_sensor_8(0x5b34, 0x00);
	write_cmos_sensor_8(0x5b35, 0x00);
	write_cmos_sensor_8(0x5b36, 0x00);
	write_cmos_sensor_8(0x5b37, 0x00);
	write_cmos_sensor_8(0x5b38, 0x00);
	write_cmos_sensor_8(0x5b39, 0x00);
	write_cmos_sensor_8(0x5b3a, 0x00);
	write_cmos_sensor_8(0x5b3b, 0x00);
	write_cmos_sensor_8(0x5b3c, 0x00);
	write_cmos_sensor_8(0x5b3d, 0x00);
	write_cmos_sensor_8(0x5b3e, 0x00);
	write_cmos_sensor_8(0x5b3f, 0x00);
	write_cmos_sensor_8(0x5b40, 0x00);
	write_cmos_sensor_8(0x5b41, 0x00);
	write_cmos_sensor_8(0x5b42, 0x00);
	write_cmos_sensor_8(0x5b43, 0x00);
	write_cmos_sensor_8(0x5b44, 0x00);
	write_cmos_sensor_8(0x5b45, 0x00);
	write_cmos_sensor_8(0x5b46, 0x0c);
	write_cmos_sensor_8(0x5b47, 0x00);
	write_cmos_sensor_8(0x5b48, 0x00);
	write_cmos_sensor_8(0x5b49, 0x00);
	write_cmos_sensor_8(0x5b4a, 0x0c);
	write_cmos_sensor_8(0x5b4b, 0x00);
	write_cmos_sensor_8(0x5b4c, 0x00);
	write_cmos_sensor_8(0x5b4d, 0x00);
	write_cmos_sensor_8(0x5b4e, 0x00);
	write_cmos_sensor_8(0x5b4f, 0x00);
	write_cmos_sensor_8(0x5b50, 0x00);
	write_cmos_sensor_8(0x5b51, 0x00);
	write_cmos_sensor_8(0x5b52, 0x00);
	write_cmos_sensor_8(0x5b53, 0x00);
	write_cmos_sensor_8(0x5b54, 0x00);
	write_cmos_sensor_8(0x5b55, 0x10);
	write_cmos_sensor_8(0x5b56, 0x00);
	write_cmos_sensor_8(0x5b57, 0x00);
	write_cmos_sensor_8(0x5b58, 0x00);
	write_cmos_sensor_8(0x5b59, 0x00);
	write_cmos_sensor_8(0x5b5a, 0x00);
	write_cmos_sensor_8(0x5b5b, 0x00);
	write_cmos_sensor_8(0x5b5c, 0x00);
	write_cmos_sensor_8(0x5b5d, 0x00);
	write_cmos_sensor_8(0x5b5e, 0x00);
	write_cmos_sensor_8(0x5b5f, 0x00);
	write_cmos_sensor_8(0x5b60, 0x00);
	//write_cmos_sensor_8(0x0100, 0x01);
}	/*	sensor_init  */

static kal_uint32 streaming_control(kal_bool enable)
{
	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
	if (enable)
		write_cmos_sensor_8(0x0100, 0X01);
	else
		write_cmos_sensor_8(0x0100, 0x00);
	return ERROR_NONE;
}

static void preview_setting(void)
{
	LOG_INF("E\n");
	
	// @@2336x1752_30fps_bin
	// Sysclk 160Mhz, MIPI4_768Mbps/Lane, 30Fps.
	// Line_length =1400, Frame_length =3808. V-blanking=17.99ms
	
	//write_cmos_sensor_8(0x0100, 0x00);
	write_cmos_sensor_8(0x0301, 0xa5);
	write_cmos_sensor_8(0x0304, 0x29);//0x28  /*prize  add  for k6307  RF intrusion by zhuzhengjiang    20190925-begin*/
	write_cmos_sensor_8(0x0316, 0xa0);
	write_cmos_sensor_8(0x3500, 0x00);
	write_cmos_sensor_8(0x3501, 0xec);
	write_cmos_sensor_8(0x3502, 0x00);
	write_cmos_sensor_8(0x3511, 0xec);
	write_cmos_sensor_8(0x3600, 0x40);
	write_cmos_sensor_8(0x3602, 0x84);
	write_cmos_sensor_8(0x3621, 0x66);
	write_cmos_sensor_8(0x3701, 0x14);
	write_cmos_sensor_8(0x3709, 0x38);
	write_cmos_sensor_8(0x37f4, 0x80);
	write_cmos_sensor_8(0x3803, 0x02);
	write_cmos_sensor_8(0x3807, 0xd1);
	write_cmos_sensor_8(0x3808, 0x09);
	write_cmos_sensor_8(0x3809, 0x20);
	write_cmos_sensor_8(0x380a, 0x06);
	write_cmos_sensor_8(0x380b, 0xd8);
	write_cmos_sensor_8(0x380c, 0x05);
	write_cmos_sensor_8(0x380d, 0x78);
	write_cmos_sensor_8(0x380e, 0x0e);
	write_cmos_sensor_8(0x380f, 0xe0);
	write_cmos_sensor_8(0x3810, 0x00);
	write_cmos_sensor_8(0x3811, 0x09);
	write_cmos_sensor_8(0x3812, 0x00);
	write_cmos_sensor_8(0x3813, 0x09);
	write_cmos_sensor_8(0x3815, 0x22);
	write_cmos_sensor_8(0x3820, 0x01);
	write_cmos_sensor_8(0x3834, 0xf5);
	write_cmos_sensor_8(0x3f03, 0x1a);
	write_cmos_sensor_8(0x4010, 0x29);
	write_cmos_sensor_8(0x4012, 0x0d);
	write_cmos_sensor_8(0x4013, 0x16);
	write_cmos_sensor_8(0x4014, 0x0a);
	write_cmos_sensor_8(0x4016, 0x15);
	write_cmos_sensor_8(0x4018, 0x09);
	write_cmos_sensor_8(0x4500, 0x25);
	write_cmos_sensor_8(0x4501, 0x04);
	write_cmos_sensor_8(0x4503, 0x10);
	write_cmos_sensor_8(0x450f, 0x88);
	write_cmos_sensor_8(0x4540, 0x86);
	write_cmos_sensor_8(0x4541, 0x07);
	write_cmos_sensor_8(0x4542, 0x04);
	write_cmos_sensor_8(0x4543, 0x05);
	write_cmos_sensor_8(0x4544, 0x06);
	write_cmos_sensor_8(0x4545, 0x07);
	write_cmos_sensor_8(0x4546, 0x04);
	write_cmos_sensor_8(0x4547, 0x05);
	write_cmos_sensor_8(0x4548, 0x02);
	write_cmos_sensor_8(0x4549, 0x03);
	write_cmos_sensor_8(0x454a, 0x00);
	write_cmos_sensor_8(0x454b, 0x01);
	write_cmos_sensor_8(0x454c, 0x02);
	write_cmos_sensor_8(0x454d, 0x03);
	write_cmos_sensor_8(0x454e, 0x00);
	write_cmos_sensor_8(0x454f, 0x01);
	write_cmos_sensor_8(0x4550, 0x06);
	write_cmos_sensor_8(0x4551, 0x07);
	write_cmos_sensor_8(0x4552, 0x04);
	write_cmos_sensor_8(0x4553, 0x05);
	write_cmos_sensor_8(0x4554, 0x06);
	write_cmos_sensor_8(0x4555, 0x07);
	write_cmos_sensor_8(0x4556, 0x04);
	write_cmos_sensor_8(0x4557, 0x05);
	write_cmos_sensor_8(0x4558, 0x02);
	write_cmos_sensor_8(0x4559, 0x03);
	write_cmos_sensor_8(0x455a, 0x00);
	write_cmos_sensor_8(0x455b, 0x01);
	write_cmos_sensor_8(0x455c, 0x02);
	write_cmos_sensor_8(0x455d, 0x03);
	write_cmos_sensor_8(0x455e, 0x00);
	write_cmos_sensor_8(0x455f, 0x01);
	write_cmos_sensor_8(0x4837, 0x14);
	write_cmos_sensor_8(0x5080, 0x04);
	write_cmos_sensor_8(0x5084, 0x09);
	write_cmos_sensor_8(0x5085, 0x30);
	write_cmos_sensor_8(0x5086, 0x06);
	write_cmos_sensor_8(0x5087, 0xe6);
	write_cmos_sensor_8(0x5000, 0x89);
	write_cmos_sensor_8(0x5001, 0x42);
	write_cmos_sensor_8(0x5004, 0x81);
	write_cmos_sensor_8(0x5184, 0x63);
	write_cmos_sensor_8(0x5185, 0x17);
	write_cmos_sensor_8(0x5192, 0x09);
	write_cmos_sensor_8(0x5193, 0x2f);
	write_cmos_sensor_8(0x5406, 0x06);
	write_cmos_sensor_8(0x5407, 0x18);
	write_cmos_sensor_8(0x5408, 0x08);
	write_cmos_sensor_8(0x5409, 0x1a);
	write_cmos_sensor_8(0x5410, 0x03);
	write_cmos_sensor_8(0x5500, 0x4a);
	write_cmos_sensor_8(0x5501, 0x43);
	write_cmos_sensor_8(0x5502, 0x3c);
	write_cmos_sensor_8(0x5503, 0x38);
	write_cmos_sensor_8(0x5504, 0x34);
	write_cmos_sensor_8(0x5505, 0x32);
	write_cmos_sensor_8(0x5506, 0x30);
	write_cmos_sensor_8(0x5507, 0x2f);
	write_cmos_sensor_8(0x5508, 0x31);
	write_cmos_sensor_8(0x5509, 0x30);
	write_cmos_sensor_8(0x550a, 0x31);
	write_cmos_sensor_8(0x550b, 0x34);
	write_cmos_sensor_8(0x550c, 0x38);
	write_cmos_sensor_8(0x550d, 0x3d);
	write_cmos_sensor_8(0x550e, 0x44);
	write_cmos_sensor_8(0x550f, 0x51);
	write_cmos_sensor_8(0x5510, 0x3f);
	write_cmos_sensor_8(0x5511, 0x35);
	write_cmos_sensor_8(0x5512, 0x30);
	write_cmos_sensor_8(0x5513, 0x2c);
	write_cmos_sensor_8(0x5514, 0x2a);
	write_cmos_sensor_8(0x5515, 0x27);
	write_cmos_sensor_8(0x5516, 0x26);
	write_cmos_sensor_8(0x5517, 0x26);
	write_cmos_sensor_8(0x5518, 0x25);
	write_cmos_sensor_8(0x5519, 0x26);
	write_cmos_sensor_8(0x551a, 0x28);
	write_cmos_sensor_8(0x551b, 0x2a);
	write_cmos_sensor_8(0x551c, 0x2d);
	write_cmos_sensor_8(0x551d, 0x31);
	write_cmos_sensor_8(0x551e, 0x37);
	write_cmos_sensor_8(0x551f, 0x40);
	write_cmos_sensor_8(0x5520, 0x32);
	write_cmos_sensor_8(0x5521, 0x2c);
	write_cmos_sensor_8(0x5522, 0x28);
	write_cmos_sensor_8(0x5523, 0x24);
	write_cmos_sensor_8(0x5524, 0x21);
	write_cmos_sensor_8(0x5525, 0x1e);
	write_cmos_sensor_8(0x5526, 0x1d);
	write_cmos_sensor_8(0x5527, 0x1c);
	write_cmos_sensor_8(0x5528, 0x1d);
	write_cmos_sensor_8(0x5529, 0x1d);
	write_cmos_sensor_8(0x552a, 0x1f);
	write_cmos_sensor_8(0x552b, 0x22);
	write_cmos_sensor_8(0x552c, 0x25);
	write_cmos_sensor_8(0x552d, 0x29);
	write_cmos_sensor_8(0x552e, 0x2e);
	write_cmos_sensor_8(0x552f, 0x36);
	write_cmos_sensor_8(0x5530, 0x2b);
	write_cmos_sensor_8(0x5531, 0x25);
	write_cmos_sensor_8(0x5532, 0x20);
	write_cmos_sensor_8(0x5533, 0x1c);
	write_cmos_sensor_8(0x5534, 0x19);
	write_cmos_sensor_8(0x5535, 0x16);
	write_cmos_sensor_8(0x5536, 0x15);
	write_cmos_sensor_8(0x5537, 0x14);
	write_cmos_sensor_8(0x5538, 0x14);
	write_cmos_sensor_8(0x5539, 0x15);
	write_cmos_sensor_8(0x553a, 0x17);
	write_cmos_sensor_8(0x553b, 0x19);
	write_cmos_sensor_8(0x553c, 0x1d);
	write_cmos_sensor_8(0x553d, 0x22);
	write_cmos_sensor_8(0x553e, 0x27);
	write_cmos_sensor_8(0x553f, 0x2f);
	write_cmos_sensor_8(0x5540, 0x25);
	write_cmos_sensor_8(0x5541, 0x1f);
	write_cmos_sensor_8(0x5542, 0x1a);
	write_cmos_sensor_8(0x5543, 0x16);
	write_cmos_sensor_8(0x5544, 0x12);
	write_cmos_sensor_8(0x5545, 0x0f);
	write_cmos_sensor_8(0x5546, 0x0d);
	write_cmos_sensor_8(0x5547, 0x0d);
	write_cmos_sensor_8(0x5548, 0x0d);
	write_cmos_sensor_8(0x5549, 0x0e);
	write_cmos_sensor_8(0x554a, 0x10);
	write_cmos_sensor_8(0x554b, 0x13);
	write_cmos_sensor_8(0x554c, 0x16);
	write_cmos_sensor_8(0x554d, 0x1b);
	write_cmos_sensor_8(0x554e, 0x21);
	write_cmos_sensor_8(0x554f, 0x29);
	write_cmos_sensor_8(0x5550, 0x20);
	write_cmos_sensor_8(0x5551, 0x1a);
	write_cmos_sensor_8(0x5552, 0x15);
	write_cmos_sensor_8(0x5553, 0x10);
	write_cmos_sensor_8(0x5554, 0x0c);
	write_cmos_sensor_8(0x5555, 0x09);
	write_cmos_sensor_8(0x5556, 0x07);
	write_cmos_sensor_8(0x5557, 0x07);
	write_cmos_sensor_8(0x5558, 0x06);
	write_cmos_sensor_8(0x5559, 0x07);
	write_cmos_sensor_8(0x555a, 0x09);
	write_cmos_sensor_8(0x555b, 0x0d);
	write_cmos_sensor_8(0x555c, 0x11);
	write_cmos_sensor_8(0x555d, 0x16);
	write_cmos_sensor_8(0x555e, 0x1c);
	write_cmos_sensor_8(0x555f, 0x24);
	write_cmos_sensor_8(0x5560, 0x1e);
	write_cmos_sensor_8(0x5561, 0x17);
	write_cmos_sensor_8(0x5562, 0x11);
	write_cmos_sensor_8(0x5563, 0x0d);
	write_cmos_sensor_8(0x5564, 0x08);
	write_cmos_sensor_8(0x5565, 0x05);
	write_cmos_sensor_8(0x5566, 0x03);
	write_cmos_sensor_8(0x5567, 0x02);
	write_cmos_sensor_8(0x5568, 0x02);
	write_cmos_sensor_8(0x5569, 0x03);
	write_cmos_sensor_8(0x556a, 0x05);
	write_cmos_sensor_8(0x556b, 0x09);
	write_cmos_sensor_8(0x556c, 0x0d);
	write_cmos_sensor_8(0x556d, 0x12);
	write_cmos_sensor_8(0x556e, 0x19);
	write_cmos_sensor_8(0x556f, 0x20);
	write_cmos_sensor_8(0x5570, 0x1c);
	write_cmos_sensor_8(0x5571, 0x15);
	write_cmos_sensor_8(0x5572, 0x0f);
	write_cmos_sensor_8(0x5573, 0x0a);
	write_cmos_sensor_8(0x5574, 0x06);
	write_cmos_sensor_8(0x5575, 0x03);
	write_cmos_sensor_8(0x5576, 0x01);
	write_cmos_sensor_8(0x5577, 0x00);
	write_cmos_sensor_8(0x5578, 0x00);
	write_cmos_sensor_8(0x5579, 0x01);
	write_cmos_sensor_8(0x557a, 0x03);
	write_cmos_sensor_8(0x557b, 0x06);
	write_cmos_sensor_8(0x557c, 0x0b);
	write_cmos_sensor_8(0x557d, 0x10);
	write_cmos_sensor_8(0x557e, 0x17);
	write_cmos_sensor_8(0x557f, 0x1f);
	write_cmos_sensor_8(0x5580, 0x1c);
	write_cmos_sensor_8(0x5581, 0x15);
	write_cmos_sensor_8(0x5582, 0x0f);
	write_cmos_sensor_8(0x5583, 0x0a);
	write_cmos_sensor_8(0x5584, 0x06);
	write_cmos_sensor_8(0x5585, 0x03);
	write_cmos_sensor_8(0x5589, 0x01);
	write_cmos_sensor_8(0x558a, 0x03);
	write_cmos_sensor_8(0x558b, 0x06);
	write_cmos_sensor_8(0x558c, 0x0b);
	write_cmos_sensor_8(0x558d, 0x10);
	write_cmos_sensor_8(0x558e, 0x16);
	write_cmos_sensor_8(0x558f, 0x1e);
	write_cmos_sensor_8(0x5590, 0x1d);
	write_cmos_sensor_8(0x5591, 0x16);
	write_cmos_sensor_8(0x5592, 0x11);
	write_cmos_sensor_8(0x5593, 0x0c);
	write_cmos_sensor_8(0x5594, 0x08);
	write_cmos_sensor_8(0x5595, 0x05);
	write_cmos_sensor_8(0x5598, 0x02);
	write_cmos_sensor_8(0x5599, 0x03);
	write_cmos_sensor_8(0x559a, 0x04);
	write_cmos_sensor_8(0x559b, 0x08);
	write_cmos_sensor_8(0x559c, 0x0c);
	write_cmos_sensor_8(0x559d, 0x12);
	write_cmos_sensor_8(0x559e, 0x18);
	write_cmos_sensor_8(0x559f, 0x1f);
	write_cmos_sensor_8(0x55a0, 0x20);
	write_cmos_sensor_8(0x55a1, 0x19);
	write_cmos_sensor_8(0x55a2, 0x14);
	write_cmos_sensor_8(0x55a3, 0x0f);
	write_cmos_sensor_8(0x55a4, 0x0b);
	write_cmos_sensor_8(0x55a5, 0x08);
	write_cmos_sensor_8(0x55a6, 0x06);
	write_cmos_sensor_8(0x55a7, 0x05);
	write_cmos_sensor_8(0x55a8, 0x06);
	write_cmos_sensor_8(0x55a9, 0x06);
	write_cmos_sensor_8(0x55aa, 0x08);
	write_cmos_sensor_8(0x55ab, 0x0b);
	write_cmos_sensor_8(0x55ac, 0x0f);
	write_cmos_sensor_8(0x55ad, 0x14);
	write_cmos_sensor_8(0x55ae, 0x1b);
	write_cmos_sensor_8(0x55af, 0x22);
	write_cmos_sensor_8(0x55b0, 0x24);
	write_cmos_sensor_8(0x55b1, 0x1e);
	write_cmos_sensor_8(0x55b2, 0x19);
	write_cmos_sensor_8(0x55b3, 0x14);
	write_cmos_sensor_8(0x55b4, 0x10);
	write_cmos_sensor_8(0x55b5, 0x0e);
	write_cmos_sensor_8(0x55b6, 0x0c);
	write_cmos_sensor_8(0x55b7, 0x0b);
	write_cmos_sensor_8(0x55b8, 0x0b);
	write_cmos_sensor_8(0x55b9, 0x0c);
	write_cmos_sensor_8(0x55ba, 0x0e);
	write_cmos_sensor_8(0x55bb, 0x11);
	write_cmos_sensor_8(0x55bc, 0x14);
	write_cmos_sensor_8(0x55bd, 0x19);
	write_cmos_sensor_8(0x55be, 0x1f);
	write_cmos_sensor_8(0x55bf, 0x27);
	write_cmos_sensor_8(0x55c0, 0x29);
	write_cmos_sensor_8(0x55c1, 0x23);
	write_cmos_sensor_8(0x55c2, 0x1f);
	write_cmos_sensor_8(0x55c3, 0x1a);
	write_cmos_sensor_8(0x55c4, 0x17);
	write_cmos_sensor_8(0x55c5, 0x14);
	write_cmos_sensor_8(0x55c6, 0x13);
	write_cmos_sensor_8(0x55c7, 0x12);
	write_cmos_sensor_8(0x55c8, 0x12);
	write_cmos_sensor_8(0x55c9, 0x13);
	write_cmos_sensor_8(0x55ca, 0x14);
	write_cmos_sensor_8(0x55cb, 0x17);
	write_cmos_sensor_8(0x55cc, 0x1a);
	write_cmos_sensor_8(0x55cd, 0x1f);
	write_cmos_sensor_8(0x55ce, 0x25);
	write_cmos_sensor_8(0x55cf, 0x2d);
	write_cmos_sensor_8(0x55d0, 0x31);
	write_cmos_sensor_8(0x55d1, 0x2a);
	write_cmos_sensor_8(0x55d2, 0x25);
	write_cmos_sensor_8(0x55d3, 0x22);
	write_cmos_sensor_8(0x55d4, 0x1f);
	write_cmos_sensor_8(0x55d5, 0x1c);
	write_cmos_sensor_8(0x55d6, 0x1b);
	write_cmos_sensor_8(0x55d7, 0x1a);
	write_cmos_sensor_8(0x55d8, 0x1a);
	write_cmos_sensor_8(0x55d9, 0x1a);
	write_cmos_sensor_8(0x55da, 0x1c);
	write_cmos_sensor_8(0x55db, 0x1f);
	write_cmos_sensor_8(0x55dc, 0x22);
	write_cmos_sensor_8(0x55dd, 0x27);
	write_cmos_sensor_8(0x55de, 0x2c);
	write_cmos_sensor_8(0x55df, 0x34);
	write_cmos_sensor_8(0x55e0, 0x3b);
	write_cmos_sensor_8(0x55e1, 0x32);
	write_cmos_sensor_8(0x55e2, 0x2e);
	write_cmos_sensor_8(0x55e3, 0x2a);
	write_cmos_sensor_8(0x55e4, 0x27);
	write_cmos_sensor_8(0x55e5, 0x25);
	write_cmos_sensor_8(0x55e6, 0x23);
	write_cmos_sensor_8(0x55e7, 0x23);
	write_cmos_sensor_8(0x55e8, 0x23);
	write_cmos_sensor_8(0x55e9, 0x24);
	write_cmos_sensor_8(0x55ea, 0x25);
	write_cmos_sensor_8(0x55eb, 0x28);
	write_cmos_sensor_8(0x55ec, 0x2b);
	write_cmos_sensor_8(0x55ed, 0x30);
	write_cmos_sensor_8(0x55ee, 0x35);
	write_cmos_sensor_8(0x55ef, 0x3f);
	write_cmos_sensor_8(0x55f0, 0x4b);
	write_cmos_sensor_8(0x55f1, 0x41);
	write_cmos_sensor_8(0x55f2, 0x3a);
	write_cmos_sensor_8(0x55f3, 0x36);
	write_cmos_sensor_8(0x55f4, 0x32);
	write_cmos_sensor_8(0x55f5, 0x30);
	write_cmos_sensor_8(0x55f6, 0x2f);
	write_cmos_sensor_8(0x55f7, 0x2d);
	write_cmos_sensor_8(0x55f8, 0x2f);
	write_cmos_sensor_8(0x55f9, 0x2f);
	write_cmos_sensor_8(0x55fa, 0x30);
	write_cmos_sensor_8(0x55fb, 0x33);
	write_cmos_sensor_8(0x55fc, 0x37);
	write_cmos_sensor_8(0x55fd, 0x3d);
	write_cmos_sensor_8(0x55fe, 0x44);
	write_cmos_sensor_8(0x55ff, 0x4e);
	write_cmos_sensor_8(0x5600, 0x80);
	write_cmos_sensor_8(0x5601, 0x80);
	write_cmos_sensor_8(0x5602, 0x80);
	write_cmos_sensor_8(0x5603, 0x7f);
	write_cmos_sensor_8(0x5605, 0x7f);
	write_cmos_sensor_8(0x5606, 0x7e);
	write_cmos_sensor_8(0x5607, 0x82);
	write_cmos_sensor_8(0x5608, 0x7e);
	write_cmos_sensor_8(0x5609, 0x80);
	write_cmos_sensor_8(0x560a, 0x80);
	write_cmos_sensor_8(0x560b, 0x7e);
	write_cmos_sensor_8(0x560c, 0x80);
	write_cmos_sensor_8(0x560e, 0x81);
	write_cmos_sensor_8(0x560f, 0x7a);
	write_cmos_sensor_8(0x5610, 0x80);
	write_cmos_sensor_8(0x5611, 0x80);
	write_cmos_sensor_8(0x5612, 0x80);
	write_cmos_sensor_8(0x5613, 0x80);
	write_cmos_sensor_8(0x5614, 0x7f);
	write_cmos_sensor_8(0x5615, 0x7f);
	write_cmos_sensor_8(0x5616, 0x80);
	write_cmos_sensor_8(0x5617, 0x7f);
	write_cmos_sensor_8(0x5618, 0x80);
	write_cmos_sensor_8(0x5619, 0x7f);
	write_cmos_sensor_8(0x561a, 0x7f);
	write_cmos_sensor_8(0x561b, 0x80);
	write_cmos_sensor_8(0x561c, 0x7f);
	write_cmos_sensor_8(0x561d, 0x7f);
	write_cmos_sensor_8(0x561e, 0x7f);
	write_cmos_sensor_8(0x561f, 0x82);
	write_cmos_sensor_8(0x5621, 0x80);
	write_cmos_sensor_8(0x5622, 0x7f);
	write_cmos_sensor_8(0x5623, 0x7f);
	write_cmos_sensor_8(0x5624, 0x7f);
	write_cmos_sensor_8(0x5625, 0x7f);
	write_cmos_sensor_8(0x5626, 0x7f);
	write_cmos_sensor_8(0x5627, 0x7f);
	write_cmos_sensor_8(0x5628, 0x7f);
	write_cmos_sensor_8(0x5629, 0x7f);
	write_cmos_sensor_8(0x562a, 0x7f);
	write_cmos_sensor_8(0x562b, 0x7f);
	write_cmos_sensor_8(0x562c, 0x7f);
	write_cmos_sensor_8(0x562d, 0x80);
	write_cmos_sensor_8(0x562e, 0x7f);
	write_cmos_sensor_8(0x562f, 0x81);
	write_cmos_sensor_8(0x5630, 0x80);
	write_cmos_sensor_8(0x5631, 0x7f);
	write_cmos_sensor_8(0x5632, 0x7f);
	write_cmos_sensor_8(0x5633, 0x7f);
	write_cmos_sensor_8(0x5634, 0x7f);
	write_cmos_sensor_8(0x5635, 0x7f);
	write_cmos_sensor_8(0x5636, 0x7f);
	write_cmos_sensor_8(0x5637, 0x7f);
	write_cmos_sensor_8(0x5638, 0x7f);
	write_cmos_sensor_8(0x5639, 0x7f);
	write_cmos_sensor_8(0x563a, 0x7f);
	write_cmos_sensor_8(0x563b, 0x7f);
	write_cmos_sensor_8(0x563c, 0x7f);
	write_cmos_sensor_8(0x563d, 0x7f);
	write_cmos_sensor_8(0x563e, 0x80);
	write_cmos_sensor_8(0x563f, 0x80);
	write_cmos_sensor_8(0x5640, 0x81);
	write_cmos_sensor_8(0x5641, 0x80);
	write_cmos_sensor_8(0x5643, 0x7f);
	write_cmos_sensor_8(0x5644, 0x7f);
	write_cmos_sensor_8(0x5645, 0x7f);
	write_cmos_sensor_8(0x5646, 0x7f);
	write_cmos_sensor_8(0x5647, 0x7f);
	write_cmos_sensor_8(0x5648, 0x7f);
	write_cmos_sensor_8(0x5649, 0x7f);
	write_cmos_sensor_8(0x564a, 0x7f);
	write_cmos_sensor_8(0x564b, 0x7f);
	write_cmos_sensor_8(0x564c, 0x7f);
	write_cmos_sensor_8(0x564d, 0x7f);
	write_cmos_sensor_8(0x564e, 0x80);
	write_cmos_sensor_8(0x564f, 0x7f);
	write_cmos_sensor_8(0x5650, 0x80);
	write_cmos_sensor_8(0x5652, 0x80);
	write_cmos_sensor_8(0x5653, 0x7f);
	write_cmos_sensor_8(0x5654, 0x7f);
	write_cmos_sensor_8(0x5655, 0x7f);
	write_cmos_sensor_8(0x5656, 0x7f);
	write_cmos_sensor_8(0x5657, 0x7f);
	write_cmos_sensor_8(0x5658, 0x7f);
	write_cmos_sensor_8(0x5659, 0x7f);
	write_cmos_sensor_8(0x565a, 0x7f);
	write_cmos_sensor_8(0x565b, 0x7f);
	write_cmos_sensor_8(0x565c, 0x7f);
	write_cmos_sensor_8(0x565d, 0x7f);
	write_cmos_sensor_8(0x565f, 0x7f);
	write_cmos_sensor_8(0x5660, 0x80);
	write_cmos_sensor_8(0x5661, 0x80);
	write_cmos_sensor_8(0x5663, 0x7f);
	write_cmos_sensor_8(0x5666, 0x7f);
	write_cmos_sensor_8(0x5667, 0x7f);
	write_cmos_sensor_8(0x5668, 0x7f);
	write_cmos_sensor_8(0x5669, 0x7f);
	write_cmos_sensor_8(0x566c, 0x80);
	write_cmos_sensor_8(0x566d, 0x80);
	write_cmos_sensor_8(0x566e, 0x7f);
	write_cmos_sensor_8(0x566f, 0x80);
	write_cmos_sensor_8(0x5671, 0x80);
	write_cmos_sensor_8(0x5672, 0x80);
	write_cmos_sensor_8(0x5673, 0x80);
	write_cmos_sensor_8(0x5676, 0x80);
	write_cmos_sensor_8(0x5677, 0x80);
	write_cmos_sensor_8(0x5678, 0x80);
	write_cmos_sensor_8(0x5679, 0x80);
	write_cmos_sensor_8(0x567a, 0x80);
	write_cmos_sensor_8(0x567b, 0x80);
	write_cmos_sensor_8(0x567c, 0x80);
	write_cmos_sensor_8(0x567d, 0x80);
	write_cmos_sensor_8(0x567e, 0x80);
	write_cmos_sensor_8(0x567f, 0x80);
	write_cmos_sensor_8(0x5680, 0x80);
	write_cmos_sensor_8(0x5681, 0x80);
	write_cmos_sensor_8(0x5682, 0x80);
	write_cmos_sensor_8(0x5684, 0x81);
	write_cmos_sensor_8(0x5685, 0x80);
	write_cmos_sensor_8(0x5686, 0x80);
	write_cmos_sensor_8(0x5687, 0x80);
	write_cmos_sensor_8(0x5688, 0x80);
	write_cmos_sensor_8(0x5689, 0x80);
	write_cmos_sensor_8(0x568a, 0x80);
	write_cmos_sensor_8(0x568b, 0x80);
	write_cmos_sensor_8(0x568c, 0x80);
	write_cmos_sensor_8(0x568d, 0x80);
	write_cmos_sensor_8(0x568e, 0x80);
	write_cmos_sensor_8(0x5691, 0x80);
	write_cmos_sensor_8(0x5692, 0x80);
	write_cmos_sensor_8(0x5695, 0x7f);
	write_cmos_sensor_8(0x5696, 0x80);
	write_cmos_sensor_8(0x5697, 0x80);
	write_cmos_sensor_8(0x5698, 0x80);
	write_cmos_sensor_8(0x5699, 0x80);
	write_cmos_sensor_8(0x569a, 0x80);
	write_cmos_sensor_8(0x569b, 0x80);
	write_cmos_sensor_8(0x569c, 0x80);
	write_cmos_sensor_8(0x569d, 0x80);
	write_cmos_sensor_8(0x569f, 0x81);
	write_cmos_sensor_8(0x56a1, 0x80);
	write_cmos_sensor_8(0x56a6, 0x7f);
	write_cmos_sensor_8(0x56a7, 0x7f);
	write_cmos_sensor_8(0x56a8, 0x7f);
	write_cmos_sensor_8(0x56a9, 0x7f);
	write_cmos_sensor_8(0x56aa, 0x80);
	write_cmos_sensor_8(0x56ab, 0x80);
	write_cmos_sensor_8(0x56ad, 0x80);
	write_cmos_sensor_8(0x56af, 0x80);
	write_cmos_sensor_8(0x56b5, 0x7f);
	write_cmos_sensor_8(0x56b6, 0x7f);
	write_cmos_sensor_8(0x56b7, 0x7f);
	write_cmos_sensor_8(0x56b8, 0x7f);
	write_cmos_sensor_8(0x56b9, 0x7f);
	write_cmos_sensor_8(0x56bb, 0x7f);
	write_cmos_sensor_8(0x56be, 0x80);
	write_cmos_sensor_8(0x56bf, 0x80);
	write_cmos_sensor_8(0x56c0, 0x80);
	write_cmos_sensor_8(0x56c3, 0x80);
	write_cmos_sensor_8(0x56c6, 0x7f);
	write_cmos_sensor_8(0x56c8, 0x7f);
	write_cmos_sensor_8(0x56c9, 0x7f);
	write_cmos_sensor_8(0x56ca, 0x7f);
	write_cmos_sensor_8(0x56cb, 0x80);
	write_cmos_sensor_8(0x56cc, 0x80);
	write_cmos_sensor_8(0x56d0, 0x81);
	write_cmos_sensor_8(0x56d1, 0x80);
	write_cmos_sensor_8(0x56d3, 0x80);
	write_cmos_sensor_8(0x56d4, 0x7f);
	write_cmos_sensor_8(0x56d5, 0x80);
	write_cmos_sensor_8(0x56d6, 0x7f);
	write_cmos_sensor_8(0x56d8, 0x80);
	write_cmos_sensor_8(0x56da, 0x80);
	write_cmos_sensor_8(0x56db, 0x7f);
	write_cmos_sensor_8(0x56dd, 0x80);
	write_cmos_sensor_8(0x56df, 0x81);
	write_cmos_sensor_8(0x56e1, 0x7f);
	write_cmos_sensor_8(0x56e2, 0x80);
	write_cmos_sensor_8(0x56e3, 0x7f);
	write_cmos_sensor_8(0x56e4, 0x7f);
	write_cmos_sensor_8(0x56e5, 0x7f);
	write_cmos_sensor_8(0x56e7, 0x80);
	write_cmos_sensor_8(0x56e8, 0x7f);
	write_cmos_sensor_8(0x56e9, 0x80);
	write_cmos_sensor_8(0x56ea, 0x7f);
	write_cmos_sensor_8(0x56eb, 0x80);
	write_cmos_sensor_8(0x56ec, 0x80);
	write_cmos_sensor_8(0x56ee, 0x80);
	write_cmos_sensor_8(0x56ef, 0x7f);
	write_cmos_sensor_8(0x56f0, 0x7b);
	write_cmos_sensor_8(0x56f1, 0x81);
	write_cmos_sensor_8(0x56f2, 0x7f);
	write_cmos_sensor_8(0x56f3, 0x80);
	write_cmos_sensor_8(0x56f5, 0x7f);
	write_cmos_sensor_8(0x56f6, 0x7f);
	write_cmos_sensor_8(0x56f7, 0x80);
	write_cmos_sensor_8(0x56f9, 0x7f);
	write_cmos_sensor_8(0x56fc, 0x7f);
	write_cmos_sensor_8(0x56fd, 0x81);
	write_cmos_sensor_8(0x56fe, 0x80);
	write_cmos_sensor_8(0x56ff, 0x7f);
	write_cmos_sensor_8(0x5700, 0x84);
	write_cmos_sensor_8(0x5701, 0x7e);
	write_cmos_sensor_8(0x5702, 0x81);
	write_cmos_sensor_8(0x5703, 0x7d);
	write_cmos_sensor_8(0x5704, 0x80);
	write_cmos_sensor_8(0x5705, 0x7f);
	write_cmos_sensor_8(0x5706, 0x7f);
	write_cmos_sensor_8(0x5707, 0x81);
	write_cmos_sensor_8(0x5708, 0x7e);
	write_cmos_sensor_8(0x5709, 0x7e);
	write_cmos_sensor_8(0x570a, 0x81);
	write_cmos_sensor_8(0x570b, 0x7f);
	write_cmos_sensor_8(0x570c, 0x80);
	write_cmos_sensor_8(0x570d, 0x7f);
	write_cmos_sensor_8(0x570e, 0x7f);
	write_cmos_sensor_8(0x570f, 0x81);
	write_cmos_sensor_8(0x5710, 0x7d);
	write_cmos_sensor_8(0x5711, 0x80);
	write_cmos_sensor_8(0x5712, 0x7e);
	write_cmos_sensor_8(0x5713, 0x80);
	write_cmos_sensor_8(0x5714, 0x7f);
	write_cmos_sensor_8(0x5715, 0x7f);
	write_cmos_sensor_8(0x5716, 0x7f);
	write_cmos_sensor_8(0x5717, 0x7f);
	write_cmos_sensor_8(0x5718, 0x80);
	write_cmos_sensor_8(0x5719, 0x80);
	write_cmos_sensor_8(0x571a, 0x7f);
	write_cmos_sensor_8(0x571b, 0x80);
	write_cmos_sensor_8(0x571c, 0x7f);
	write_cmos_sensor_8(0x571d, 0x80);
	write_cmos_sensor_8(0x571e, 0x7f);
	write_cmos_sensor_8(0x571f, 0x81);
	write_cmos_sensor_8(0x5720, 0x80);
	write_cmos_sensor_8(0x5721, 0x7f);
	write_cmos_sensor_8(0x5722, 0x80);
	write_cmos_sensor_8(0x5723, 0x7f);
	write_cmos_sensor_8(0x5724, 0x7f);
	write_cmos_sensor_8(0x5725, 0x7f);
	write_cmos_sensor_8(0x5726, 0x7f);
	write_cmos_sensor_8(0x5727, 0x7f);
	write_cmos_sensor_8(0x5728, 0x7f);
	write_cmos_sensor_8(0x5729, 0x7f);
	write_cmos_sensor_8(0x572a, 0x7f);
	write_cmos_sensor_8(0x572b, 0x7f);
	write_cmos_sensor_8(0x572c, 0x7f);
	write_cmos_sensor_8(0x572d, 0x7f);
	write_cmos_sensor_8(0x572e, 0x7f);
	write_cmos_sensor_8(0x572f, 0x81);
	write_cmos_sensor_8(0x5730, 0x7f);
	write_cmos_sensor_8(0x5731, 0x7f);
	write_cmos_sensor_8(0x5732, 0x7f);
	write_cmos_sensor_8(0x5733, 0x7f);
	write_cmos_sensor_8(0x5734, 0x7f);
	write_cmos_sensor_8(0x5735, 0x7f);
	write_cmos_sensor_8(0x5736, 0x7f);
	write_cmos_sensor_8(0x5737, 0x7f);
	write_cmos_sensor_8(0x5738, 0x7f);
	write_cmos_sensor_8(0x5739, 0x7f);
	write_cmos_sensor_8(0x573a, 0x7f);
	write_cmos_sensor_8(0x573b, 0x7f);
	write_cmos_sensor_8(0x573c, 0x7f);
	write_cmos_sensor_8(0x573d, 0x7f);
	write_cmos_sensor_8(0x573e, 0x7f);
	write_cmos_sensor_8(0x573f, 0x80);
	write_cmos_sensor_8(0x5740, 0x80);
	write_cmos_sensor_8(0x5741, 0x7f);
	write_cmos_sensor_8(0x5742, 0x80);
	write_cmos_sensor_8(0x5743, 0x7f);
	write_cmos_sensor_8(0x5744, 0x7f);
	write_cmos_sensor_8(0x5745, 0x7f);
	write_cmos_sensor_8(0x5746, 0x7f);
	write_cmos_sensor_8(0x5747, 0x7f);
	write_cmos_sensor_8(0x5748, 0x7f);
	write_cmos_sensor_8(0x5749, 0x7f);
	write_cmos_sensor_8(0x574a, 0x7f);
	write_cmos_sensor_8(0x574b, 0x7f);
	write_cmos_sensor_8(0x574c, 0x7f);
	write_cmos_sensor_8(0x574d, 0x7f);
	write_cmos_sensor_8(0x574e, 0x7f);
	write_cmos_sensor_8(0x574f, 0x80);
	write_cmos_sensor_8(0x5750, 0x80);
	write_cmos_sensor_8(0x5751, 0x7f);
	write_cmos_sensor_8(0x5752, 0x7f);
	write_cmos_sensor_8(0x5753, 0x7f);
	write_cmos_sensor_8(0x5754, 0x7f);
	write_cmos_sensor_8(0x5755, 0x7f);
	write_cmos_sensor_8(0x5756, 0x7f);
	write_cmos_sensor_8(0x5757, 0x7f);
	write_cmos_sensor_8(0x5758, 0x7f);
	write_cmos_sensor_8(0x5759, 0x7f);
	write_cmos_sensor_8(0x575a, 0x7f);
	write_cmos_sensor_8(0x575b, 0x7f);
	write_cmos_sensor_8(0x575c, 0x7f);
	write_cmos_sensor_8(0x575d, 0x7f);
	write_cmos_sensor_8(0x575e, 0x7f);
	write_cmos_sensor_8(0x575f, 0x80);
	write_cmos_sensor_8(0x5760, 0x7e);
	write_cmos_sensor_8(0x5761, 0x80);
	write_cmos_sensor_8(0x5762, 0x7f);
	write_cmos_sensor_8(0x5763, 0x7f);
	write_cmos_sensor_8(0x5764, 0x7f);
	write_cmos_sensor_8(0x5766, 0x7f);
	write_cmos_sensor_8(0x5767, 0x7f);
	write_cmos_sensor_8(0x5768, 0x80);
	write_cmos_sensor_8(0x5769, 0x7f);
	write_cmos_sensor_8(0x576a, 0x80);
	write_cmos_sensor_8(0x576b, 0x7f);
	write_cmos_sensor_8(0x576c, 0x80);
	write_cmos_sensor_8(0x576d, 0x80);
	write_cmos_sensor_8(0x576e, 0x7f);
	write_cmos_sensor_8(0x576f, 0x80);
	write_cmos_sensor_8(0x5770, 0x80);
	write_cmos_sensor_8(0x5771, 0x80);
	write_cmos_sensor_8(0x5772, 0x80);
	write_cmos_sensor_8(0x5774, 0x80);
	write_cmos_sensor_8(0x5775, 0x80);
	write_cmos_sensor_8(0x5776, 0x80);
	write_cmos_sensor_8(0x5778, 0x80);
	write_cmos_sensor_8(0x5779, 0x80);
	write_cmos_sensor_8(0x577a, 0x80);
	write_cmos_sensor_8(0x577b, 0x80);
	write_cmos_sensor_8(0x577c, 0x7f);
	write_cmos_sensor_8(0x577d, 0x80);
	write_cmos_sensor_8(0x577e, 0x80);
	write_cmos_sensor_8(0x577f, 0x80);
	write_cmos_sensor_8(0x5780, 0x80);
	write_cmos_sensor_8(0x5781, 0x80);
	write_cmos_sensor_8(0x5782, 0x80);
	write_cmos_sensor_8(0x5784, 0x80);
	write_cmos_sensor_8(0x5785, 0x80);
	write_cmos_sensor_8(0x5786, 0x80);
	write_cmos_sensor_8(0x5788, 0x80);
	write_cmos_sensor_8(0x5789, 0x81);
	write_cmos_sensor_8(0x578a, 0x80);
	write_cmos_sensor_8(0x578b, 0x80);
	write_cmos_sensor_8(0x578c, 0x80);
	write_cmos_sensor_8(0x578d, 0x80);
	write_cmos_sensor_8(0x578e, 0x80);
	write_cmos_sensor_8(0x578f, 0x80);
	write_cmos_sensor_8(0x5790, 0x7f);
	write_cmos_sensor_8(0x5791, 0x80);
	write_cmos_sensor_8(0x5792, 0x80);
	write_cmos_sensor_8(0x5795, 0x80);
	write_cmos_sensor_8(0x5798, 0x80);
	write_cmos_sensor_8(0x5799, 0x7f);
	write_cmos_sensor_8(0x579a, 0x80);
	write_cmos_sensor_8(0x579b, 0x80);
	write_cmos_sensor_8(0x579c, 0x80);
	write_cmos_sensor_8(0x579d, 0x80);
	write_cmos_sensor_8(0x579e, 0x80);
	write_cmos_sensor_8(0x579f, 0x80);
	write_cmos_sensor_8(0x57a0, 0x7f);
	write_cmos_sensor_8(0x57a1, 0x80);
	write_cmos_sensor_8(0x57a2, 0x80);
	write_cmos_sensor_8(0x57a3, 0x7f);
	write_cmos_sensor_8(0x57a4, 0x80);
	write_cmos_sensor_8(0x57a5, 0x7f);
	write_cmos_sensor_8(0x57a6, 0x80);
	write_cmos_sensor_8(0x57a7, 0x7f);
	write_cmos_sensor_8(0x57a8, 0x7f);
	write_cmos_sensor_8(0x57a9, 0x7f);
	write_cmos_sensor_8(0x57aa, 0x80);
	write_cmos_sensor_8(0x57ab, 0x7f);
	write_cmos_sensor_8(0x57ac, 0x7f);
	write_cmos_sensor_8(0x57ad, 0x80);
	write_cmos_sensor_8(0x57ae, 0x80);
	write_cmos_sensor_8(0x57af, 0x81);
	write_cmos_sensor_8(0x57b0, 0x80);
	write_cmos_sensor_8(0x57b1, 0x7f);
	write_cmos_sensor_8(0x57b2, 0x7f);
	write_cmos_sensor_8(0x57b3, 0x7f);
	write_cmos_sensor_8(0x57b4, 0x7f);
	write_cmos_sensor_8(0x57b5, 0x7f);
	write_cmos_sensor_8(0x57b6, 0x7f);
	write_cmos_sensor_8(0x57b7, 0x7f);
	write_cmos_sensor_8(0x57b8, 0x7f);
	write_cmos_sensor_8(0x57b9, 0x7f);
	write_cmos_sensor_8(0x57ba, 0x7f);
	write_cmos_sensor_8(0x57bb, 0x7f);
	write_cmos_sensor_8(0x57bc, 0x80);
	write_cmos_sensor_8(0x57bd, 0x7f);
	write_cmos_sensor_8(0x57be, 0x80);
	write_cmos_sensor_8(0x57bf, 0x7f);
	write_cmos_sensor_8(0x57c0, 0x7f);
	write_cmos_sensor_8(0x57c1, 0x7f);
	write_cmos_sensor_8(0x57c2, 0x7f);
	write_cmos_sensor_8(0x57c3, 0x7f);
	write_cmos_sensor_8(0x57c4, 0x7f);
	write_cmos_sensor_8(0x57c5, 0x7f);
	write_cmos_sensor_8(0x57c6, 0x80);
	write_cmos_sensor_8(0x57c7, 0x7f);
	write_cmos_sensor_8(0x57c8, 0x80);
	write_cmos_sensor_8(0x57c9, 0x7f);
	write_cmos_sensor_8(0x57ca, 0x7f);
	write_cmos_sensor_8(0x57cb, 0x7f);
	write_cmos_sensor_8(0x57cc, 0x80);
	write_cmos_sensor_8(0x57cd, 0x80);
	write_cmos_sensor_8(0x57ce, 0x7f);
	write_cmos_sensor_8(0x57cf, 0x81);
	write_cmos_sensor_8(0x57d0, 0x80);
	write_cmos_sensor_8(0x57d1, 0x80);
	write_cmos_sensor_8(0x57d2, 0x80);
	write_cmos_sensor_8(0x57d3, 0x7f);
	write_cmos_sensor_8(0x57d4, 0x7f);
	write_cmos_sensor_8(0x57d5, 0x80);
	write_cmos_sensor_8(0x57d6, 0x7f);
	write_cmos_sensor_8(0x57d7, 0x7f);
	write_cmos_sensor_8(0x57d8, 0x7f);
	write_cmos_sensor_8(0x57d9, 0x7f);
	write_cmos_sensor_8(0x57da, 0x7f);
	write_cmos_sensor_8(0x57db, 0x80);
	write_cmos_sensor_8(0x57dc, 0x7f);
	write_cmos_sensor_8(0x57dd, 0x80);
	write_cmos_sensor_8(0x57de, 0x80);
	write_cmos_sensor_8(0x57df, 0x80);
	write_cmos_sensor_8(0x57e0, 0x7f);
	write_cmos_sensor_8(0x57e1, 0x7f);
	write_cmos_sensor_8(0x57e2, 0x80);
	write_cmos_sensor_8(0x57e3, 0x7f);
	write_cmos_sensor_8(0x57e4, 0x80);
	write_cmos_sensor_8(0x57e5, 0x7f);
	write_cmos_sensor_8(0x57e6, 0x80);
	write_cmos_sensor_8(0x57e7, 0x7f);
	write_cmos_sensor_8(0x57e8, 0x80);
	write_cmos_sensor_8(0x57e9, 0x7f);
	write_cmos_sensor_8(0x57ea, 0x80);
	write_cmos_sensor_8(0x57eb, 0x80);
	write_cmos_sensor_8(0x57ec, 0x80);
	write_cmos_sensor_8(0x57ed, 0x7f);
	write_cmos_sensor_8(0x57ee, 0x80);
	write_cmos_sensor_8(0x57ef, 0x80);
	write_cmos_sensor_8(0x57f0, 0x7f);
	write_cmos_sensor_8(0x57f1, 0x80);
	write_cmos_sensor_8(0x57f2, 0x7e);
	write_cmos_sensor_8(0x57f3, 0x80);
	write_cmos_sensor_8(0x57f4, 0x7e);
	write_cmos_sensor_8(0x57f5, 0x81);
	write_cmos_sensor_8(0x57f6, 0x7e);
	write_cmos_sensor_8(0x57f7, 0x81);
	write_cmos_sensor_8(0x57f8, 0x7e);
	write_cmos_sensor_8(0x57f9, 0x80);
	write_cmos_sensor_8(0x57fa, 0x80);
	write_cmos_sensor_8(0x57fb, 0x80);
	write_cmos_sensor_8(0x57fc, 0x7e);
	write_cmos_sensor_8(0x57fd, 0x81);
	write_cmos_sensor_8(0x57fe, 0x80);
	write_cmos_sensor_8(0x57ff, 0x7a);
	//write_cmos_sensor_8(0x0100, 0x01);

}	/*	preview_setting  */


static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("E\n");

	if (currefps == 300) { //30fps            //30fps for Normal capture & ZSD
	
	// @@2336x1752_30fps_bin
	// Sysclk 160Mhz, MIPI4_768Mbps/Lane, 30Fps.
	// Line_length =1400, Frame_length =3808.  V-blanking=17.99ms
	
	//write_cmos_sensor_8(0x0100, 0x00);
	write_cmos_sensor_8(0x0301, 0xa5);
	write_cmos_sensor_8(0x0304, 0x29);//0x28  /*prize  add  for k6307  RF intrusion by zhuzhengjiang    20190925-begin*/
	write_cmos_sensor_8(0x0316, 0xa0);
	write_cmos_sensor_8(0x3500, 0x00);
	write_cmos_sensor_8(0x3501, 0xec);
	write_cmos_sensor_8(0x3502, 0x00);
	write_cmos_sensor_8(0x3511, 0xec);
	write_cmos_sensor_8(0x3600, 0x40);
	write_cmos_sensor_8(0x3602, 0x84);
	write_cmos_sensor_8(0x3621, 0x66);
	write_cmos_sensor_8(0x3701, 0x14);
	write_cmos_sensor_8(0x3709, 0x38);
	write_cmos_sensor_8(0x37f4, 0x80);
	write_cmos_sensor_8(0x3803, 0x02);
	write_cmos_sensor_8(0x3807, 0xd1);
	write_cmos_sensor_8(0x3808, 0x09);
	write_cmos_sensor_8(0x3809, 0x20);
	write_cmos_sensor_8(0x380a, 0x06);
	write_cmos_sensor_8(0x380b, 0xd8);
	write_cmos_sensor_8(0x380c, 0x05);
	write_cmos_sensor_8(0x380d, 0x78);
	write_cmos_sensor_8(0x380e, 0x0e);
	write_cmos_sensor_8(0x380f, 0xe0);
	write_cmos_sensor_8(0x3810, 0x00);
	write_cmos_sensor_8(0x3811, 0x09);
	write_cmos_sensor_8(0x3812, 0x00);
	write_cmos_sensor_8(0x3813, 0x09);
	write_cmos_sensor_8(0x3815, 0x22);
	write_cmos_sensor_8(0x3820, 0x01);
	write_cmos_sensor_8(0x3834, 0xf5);
	write_cmos_sensor_8(0x3f03, 0x1a);
	write_cmos_sensor_8(0x4010, 0x29);
	write_cmos_sensor_8(0x4012, 0x0d);
	write_cmos_sensor_8(0x4013, 0x16);
	write_cmos_sensor_8(0x4014, 0x0a);
	write_cmos_sensor_8(0x4016, 0x15);
	write_cmos_sensor_8(0x4018, 0x09);
	write_cmos_sensor_8(0x4500, 0x25);
	write_cmos_sensor_8(0x4501, 0x04);
	write_cmos_sensor_8(0x4503, 0x10);
	write_cmos_sensor_8(0x450f, 0x88);
	write_cmos_sensor_8(0x4540, 0x86);
	write_cmos_sensor_8(0x4541, 0x07);
	write_cmos_sensor_8(0x4542, 0x04);
	write_cmos_sensor_8(0x4543, 0x05);
	write_cmos_sensor_8(0x4544, 0x06);
	write_cmos_sensor_8(0x4545, 0x07);
	write_cmos_sensor_8(0x4546, 0x04);
	write_cmos_sensor_8(0x4547, 0x05);
	write_cmos_sensor_8(0x4548, 0x02);
	write_cmos_sensor_8(0x4549, 0x03);
	write_cmos_sensor_8(0x454a, 0x00);
	write_cmos_sensor_8(0x454b, 0x01);
	write_cmos_sensor_8(0x454c, 0x02);
	write_cmos_sensor_8(0x454d, 0x03);
	write_cmos_sensor_8(0x454e, 0x00);
	write_cmos_sensor_8(0x454f, 0x01);
	write_cmos_sensor_8(0x4550, 0x06);
	write_cmos_sensor_8(0x4551, 0x07);
	write_cmos_sensor_8(0x4552, 0x04);
	write_cmos_sensor_8(0x4553, 0x05);
	write_cmos_sensor_8(0x4554, 0x06);
	write_cmos_sensor_8(0x4555, 0x07);
	write_cmos_sensor_8(0x4556, 0x04);
	write_cmos_sensor_8(0x4557, 0x05);
	write_cmos_sensor_8(0x4558, 0x02);
	write_cmos_sensor_8(0x4559, 0x03);
	write_cmos_sensor_8(0x455a, 0x00);
	write_cmos_sensor_8(0x455b, 0x01);
	write_cmos_sensor_8(0x455c, 0x02);
	write_cmos_sensor_8(0x455d, 0x03);
	write_cmos_sensor_8(0x455e, 0x00);
	write_cmos_sensor_8(0x455f, 0x01);
	write_cmos_sensor_8(0x4837, 0x14);
	write_cmos_sensor_8(0x5080, 0x04);
	write_cmos_sensor_8(0x5084, 0x09);
	write_cmos_sensor_8(0x5085, 0x30);
	write_cmos_sensor_8(0x5086, 0x06);
	write_cmos_sensor_8(0x5087, 0xe6);
	write_cmos_sensor_8(0x5000, 0x89);
	write_cmos_sensor_8(0x5001, 0x42);
	write_cmos_sensor_8(0x5004, 0x81);
	write_cmos_sensor_8(0x5184, 0x63);
	write_cmos_sensor_8(0x5185, 0x17);
	write_cmos_sensor_8(0x5192, 0x09);
	write_cmos_sensor_8(0x5193, 0x2f);
	write_cmos_sensor_8(0x5406, 0x06);
	write_cmos_sensor_8(0x5407, 0x18);
	write_cmos_sensor_8(0x5408, 0x08);
	write_cmos_sensor_8(0x5409, 0x1a);
	write_cmos_sensor_8(0x5410, 0x03);
	write_cmos_sensor_8(0x5500, 0x4a);
	write_cmos_sensor_8(0x5501, 0x43);
	write_cmos_sensor_8(0x5502, 0x3c);
	write_cmos_sensor_8(0x5503, 0x38);
	write_cmos_sensor_8(0x5504, 0x34);
	write_cmos_sensor_8(0x5505, 0x32);
	write_cmos_sensor_8(0x5506, 0x30);
	write_cmos_sensor_8(0x5507, 0x2f);
	write_cmos_sensor_8(0x5508, 0x31);
	write_cmos_sensor_8(0x5509, 0x30);
	write_cmos_sensor_8(0x550a, 0x31);
	write_cmos_sensor_8(0x550b, 0x34);
	write_cmos_sensor_8(0x550c, 0x38);
	write_cmos_sensor_8(0x550d, 0x3d);
	write_cmos_sensor_8(0x550e, 0x44);
	write_cmos_sensor_8(0x550f, 0x51);
	write_cmos_sensor_8(0x5510, 0x3f);
	write_cmos_sensor_8(0x5511, 0x35);
	write_cmos_sensor_8(0x5512, 0x30);
	write_cmos_sensor_8(0x5513, 0x2c);
	write_cmos_sensor_8(0x5514, 0x2a);
	write_cmos_sensor_8(0x5515, 0x27);
	write_cmos_sensor_8(0x5516, 0x26);
	write_cmos_sensor_8(0x5517, 0x26);
	write_cmos_sensor_8(0x5518, 0x25);
	write_cmos_sensor_8(0x5519, 0x26);
	write_cmos_sensor_8(0x551a, 0x28);
	write_cmos_sensor_8(0x551b, 0x2a);
	write_cmos_sensor_8(0x551c, 0x2d);
	write_cmos_sensor_8(0x551d, 0x31);
	write_cmos_sensor_8(0x551e, 0x37);
	write_cmos_sensor_8(0x551f, 0x40);
	write_cmos_sensor_8(0x5520, 0x32);
	write_cmos_sensor_8(0x5521, 0x2c);
	write_cmos_sensor_8(0x5522, 0x28);
	write_cmos_sensor_8(0x5523, 0x24);
	write_cmos_sensor_8(0x5524, 0x21);
	write_cmos_sensor_8(0x5525, 0x1e);
	write_cmos_sensor_8(0x5526, 0x1d);
	write_cmos_sensor_8(0x5527, 0x1c);
	write_cmos_sensor_8(0x5528, 0x1d);
	write_cmos_sensor_8(0x5529, 0x1d);
	write_cmos_sensor_8(0x552a, 0x1f);
	write_cmos_sensor_8(0x552b, 0x22);
	write_cmos_sensor_8(0x552c, 0x25);
	write_cmos_sensor_8(0x552d, 0x29);
	write_cmos_sensor_8(0x552e, 0x2e);
	write_cmos_sensor_8(0x552f, 0x36);
	write_cmos_sensor_8(0x5530, 0x2b);
	write_cmos_sensor_8(0x5531, 0x25);
	write_cmos_sensor_8(0x5532, 0x20);
	write_cmos_sensor_8(0x5533, 0x1c);
	write_cmos_sensor_8(0x5534, 0x19);
	write_cmos_sensor_8(0x5535, 0x16);
	write_cmos_sensor_8(0x5536, 0x15);
	write_cmos_sensor_8(0x5537, 0x14);
	write_cmos_sensor_8(0x5538, 0x14);
	write_cmos_sensor_8(0x5539, 0x15);
	write_cmos_sensor_8(0x553a, 0x17);
	write_cmos_sensor_8(0x553b, 0x19);
	write_cmos_sensor_8(0x553c, 0x1d);
	write_cmos_sensor_8(0x553d, 0x22);
	write_cmos_sensor_8(0x553e, 0x27);
	write_cmos_sensor_8(0x553f, 0x2f);
	write_cmos_sensor_8(0x5540, 0x25);
	write_cmos_sensor_8(0x5541, 0x1f);
	write_cmos_sensor_8(0x5542, 0x1a);
	write_cmos_sensor_8(0x5543, 0x16);
	write_cmos_sensor_8(0x5544, 0x12);
	write_cmos_sensor_8(0x5545, 0x0f);
	write_cmos_sensor_8(0x5546, 0x0d);
	write_cmos_sensor_8(0x5547, 0x0d);
	write_cmos_sensor_8(0x5548, 0x0d);
	write_cmos_sensor_8(0x5549, 0x0e);
	write_cmos_sensor_8(0x554a, 0x10);
	write_cmos_sensor_8(0x554b, 0x13);
	write_cmos_sensor_8(0x554c, 0x16);
	write_cmos_sensor_8(0x554d, 0x1b);
	write_cmos_sensor_8(0x554e, 0x21);
	write_cmos_sensor_8(0x554f, 0x29);
	write_cmos_sensor_8(0x5550, 0x20);
	write_cmos_sensor_8(0x5551, 0x1a);
	write_cmos_sensor_8(0x5552, 0x15);
	write_cmos_sensor_8(0x5553, 0x10);
	write_cmos_sensor_8(0x5554, 0x0c);
	write_cmos_sensor_8(0x5555, 0x09);
	write_cmos_sensor_8(0x5556, 0x07);
	write_cmos_sensor_8(0x5557, 0x07);
	write_cmos_sensor_8(0x5558, 0x06);
	write_cmos_sensor_8(0x5559, 0x07);
	write_cmos_sensor_8(0x555a, 0x09);
	write_cmos_sensor_8(0x555b, 0x0d);
	write_cmos_sensor_8(0x555c, 0x11);
	write_cmos_sensor_8(0x555d, 0x16);
	write_cmos_sensor_8(0x555e, 0x1c);
	write_cmos_sensor_8(0x555f, 0x24);
	write_cmos_sensor_8(0x5560, 0x1e);
	write_cmos_sensor_8(0x5561, 0x17);
	write_cmos_sensor_8(0x5562, 0x11);
	write_cmos_sensor_8(0x5563, 0x0d);
	write_cmos_sensor_8(0x5564, 0x08);
	write_cmos_sensor_8(0x5565, 0x05);
	write_cmos_sensor_8(0x5566, 0x03);
	write_cmos_sensor_8(0x5567, 0x02);
	write_cmos_sensor_8(0x5568, 0x02);
	write_cmos_sensor_8(0x5569, 0x03);
	write_cmos_sensor_8(0x556a, 0x05);
	write_cmos_sensor_8(0x556b, 0x09);
	write_cmos_sensor_8(0x556c, 0x0d);
	write_cmos_sensor_8(0x556d, 0x12);
	write_cmos_sensor_8(0x556e, 0x19);
	write_cmos_sensor_8(0x556f, 0x20);
	write_cmos_sensor_8(0x5570, 0x1c);
	write_cmos_sensor_8(0x5571, 0x15);
	write_cmos_sensor_8(0x5572, 0x0f);
	write_cmos_sensor_8(0x5573, 0x0a);
	write_cmos_sensor_8(0x5574, 0x06);
	write_cmos_sensor_8(0x5575, 0x03);
	write_cmos_sensor_8(0x5576, 0x01);
	write_cmos_sensor_8(0x5577, 0x00);
	write_cmos_sensor_8(0x5578, 0x00);
	write_cmos_sensor_8(0x5579, 0x01);
	write_cmos_sensor_8(0x557a, 0x03);
	write_cmos_sensor_8(0x557b, 0x06);
	write_cmos_sensor_8(0x557c, 0x0b);
	write_cmos_sensor_8(0x557d, 0x10);
	write_cmos_sensor_8(0x557e, 0x17);
	write_cmos_sensor_8(0x557f, 0x1f);
	write_cmos_sensor_8(0x5580, 0x1c);
	write_cmos_sensor_8(0x5581, 0x15);
	write_cmos_sensor_8(0x5582, 0x0f);
	write_cmos_sensor_8(0x5583, 0x0a);
	write_cmos_sensor_8(0x5584, 0x06);
	write_cmos_sensor_8(0x5585, 0x03);
	write_cmos_sensor_8(0x5589, 0x01);
	write_cmos_sensor_8(0x558a, 0x03);
	write_cmos_sensor_8(0x558b, 0x06);
	write_cmos_sensor_8(0x558c, 0x0b);
	write_cmos_sensor_8(0x558d, 0x10);
	write_cmos_sensor_8(0x558e, 0x16);
	write_cmos_sensor_8(0x558f, 0x1e);
	write_cmos_sensor_8(0x5590, 0x1d);
	write_cmos_sensor_8(0x5591, 0x16);
	write_cmos_sensor_8(0x5592, 0x11);
	write_cmos_sensor_8(0x5593, 0x0c);
	write_cmos_sensor_8(0x5594, 0x08);
	write_cmos_sensor_8(0x5595, 0x05);
	write_cmos_sensor_8(0x5598, 0x02);
	write_cmos_sensor_8(0x5599, 0x03);
	write_cmos_sensor_8(0x559a, 0x04);
	write_cmos_sensor_8(0x559b, 0x08);
	write_cmos_sensor_8(0x559c, 0x0c);
	write_cmos_sensor_8(0x559d, 0x12);
	write_cmos_sensor_8(0x559e, 0x18);
	write_cmos_sensor_8(0x559f, 0x1f);
	write_cmos_sensor_8(0x55a0, 0x20);
	write_cmos_sensor_8(0x55a1, 0x19);
	write_cmos_sensor_8(0x55a2, 0x14);
	write_cmos_sensor_8(0x55a3, 0x0f);
	write_cmos_sensor_8(0x55a4, 0x0b);
	write_cmos_sensor_8(0x55a5, 0x08);
	write_cmos_sensor_8(0x55a6, 0x06);
	write_cmos_sensor_8(0x55a7, 0x05);
	write_cmos_sensor_8(0x55a8, 0x06);
	write_cmos_sensor_8(0x55a9, 0x06);
	write_cmos_sensor_8(0x55aa, 0x08);
	write_cmos_sensor_8(0x55ab, 0x0b);
	write_cmos_sensor_8(0x55ac, 0x0f);
	write_cmos_sensor_8(0x55ad, 0x14);
	write_cmos_sensor_8(0x55ae, 0x1b);
	write_cmos_sensor_8(0x55af, 0x22);
	write_cmos_sensor_8(0x55b0, 0x24);
	write_cmos_sensor_8(0x55b1, 0x1e);
	write_cmos_sensor_8(0x55b2, 0x19);
	write_cmos_sensor_8(0x55b3, 0x14);
	write_cmos_sensor_8(0x55b4, 0x10);
	write_cmos_sensor_8(0x55b5, 0x0e);
	write_cmos_sensor_8(0x55b6, 0x0c);
	write_cmos_sensor_8(0x55b7, 0x0b);
	write_cmos_sensor_8(0x55b8, 0x0b);
	write_cmos_sensor_8(0x55b9, 0x0c);
	write_cmos_sensor_8(0x55ba, 0x0e);
	write_cmos_sensor_8(0x55bb, 0x11);
	write_cmos_sensor_8(0x55bc, 0x14);
	write_cmos_sensor_8(0x55bd, 0x19);
	write_cmos_sensor_8(0x55be, 0x1f);
	write_cmos_sensor_8(0x55bf, 0x27);
	write_cmos_sensor_8(0x55c0, 0x29);
	write_cmos_sensor_8(0x55c1, 0x23);
	write_cmos_sensor_8(0x55c2, 0x1f);
	write_cmos_sensor_8(0x55c3, 0x1a);
	write_cmos_sensor_8(0x55c4, 0x17);
	write_cmos_sensor_8(0x55c5, 0x14);
	write_cmos_sensor_8(0x55c6, 0x13);
	write_cmos_sensor_8(0x55c7, 0x12);
	write_cmos_sensor_8(0x55c8, 0x12);
	write_cmos_sensor_8(0x55c9, 0x13);
	write_cmos_sensor_8(0x55ca, 0x14);
	write_cmos_sensor_8(0x55cb, 0x17);
	write_cmos_sensor_8(0x55cc, 0x1a);
	write_cmos_sensor_8(0x55cd, 0x1f);
	write_cmos_sensor_8(0x55ce, 0x25);
	write_cmos_sensor_8(0x55cf, 0x2d);
	write_cmos_sensor_8(0x55d0, 0x31);
	write_cmos_sensor_8(0x55d1, 0x2a);
	write_cmos_sensor_8(0x55d2, 0x25);
	write_cmos_sensor_8(0x55d3, 0x22);
	write_cmos_sensor_8(0x55d4, 0x1f);
	write_cmos_sensor_8(0x55d5, 0x1c);
	write_cmos_sensor_8(0x55d6, 0x1b);
	write_cmos_sensor_8(0x55d7, 0x1a);
	write_cmos_sensor_8(0x55d8, 0x1a);
	write_cmos_sensor_8(0x55d9, 0x1a);
	write_cmos_sensor_8(0x55da, 0x1c);
	write_cmos_sensor_8(0x55db, 0x1f);
	write_cmos_sensor_8(0x55dc, 0x22);
	write_cmos_sensor_8(0x55dd, 0x27);
	write_cmos_sensor_8(0x55de, 0x2c);
	write_cmos_sensor_8(0x55df, 0x34);
	write_cmos_sensor_8(0x55e0, 0x3b);
	write_cmos_sensor_8(0x55e1, 0x32);
	write_cmos_sensor_8(0x55e2, 0x2e);
	write_cmos_sensor_8(0x55e3, 0x2a);
	write_cmos_sensor_8(0x55e4, 0x27);
	write_cmos_sensor_8(0x55e5, 0x25);
	write_cmos_sensor_8(0x55e6, 0x23);
	write_cmos_sensor_8(0x55e7, 0x23);
	write_cmos_sensor_8(0x55e8, 0x23);
	write_cmos_sensor_8(0x55e9, 0x24);
	write_cmos_sensor_8(0x55ea, 0x25);
	write_cmos_sensor_8(0x55eb, 0x28);
	write_cmos_sensor_8(0x55ec, 0x2b);
	write_cmos_sensor_8(0x55ed, 0x30);
	write_cmos_sensor_8(0x55ee, 0x35);
	write_cmos_sensor_8(0x55ef, 0x3f);
	write_cmos_sensor_8(0x55f0, 0x4b);
	write_cmos_sensor_8(0x55f1, 0x41);
	write_cmos_sensor_8(0x55f2, 0x3a);
	write_cmos_sensor_8(0x55f3, 0x36);
	write_cmos_sensor_8(0x55f4, 0x32);
	write_cmos_sensor_8(0x55f5, 0x30);
	write_cmos_sensor_8(0x55f6, 0x2f);
	write_cmos_sensor_8(0x55f7, 0x2d);
	write_cmos_sensor_8(0x55f8, 0x2f);
	write_cmos_sensor_8(0x55f9, 0x2f);
	write_cmos_sensor_8(0x55fa, 0x30);
	write_cmos_sensor_8(0x55fb, 0x33);
	write_cmos_sensor_8(0x55fc, 0x37);
	write_cmos_sensor_8(0x55fd, 0x3d);
	write_cmos_sensor_8(0x55fe, 0x44);
	write_cmos_sensor_8(0x55ff, 0x4e);
	write_cmos_sensor_8(0x5600, 0x80);
	write_cmos_sensor_8(0x5601, 0x80);
	write_cmos_sensor_8(0x5602, 0x80);
	write_cmos_sensor_8(0x5603, 0x7f);
	write_cmos_sensor_8(0x5605, 0x7f);
	write_cmos_sensor_8(0x5606, 0x7e);
	write_cmos_sensor_8(0x5607, 0x82);
	write_cmos_sensor_8(0x5608, 0x7e);
	write_cmos_sensor_8(0x5609, 0x80);
	write_cmos_sensor_8(0x560a, 0x80);
	write_cmos_sensor_8(0x560b, 0x7e);
	write_cmos_sensor_8(0x560c, 0x80);
	write_cmos_sensor_8(0x560e, 0x81);
	write_cmos_sensor_8(0x560f, 0x7a);
	write_cmos_sensor_8(0x5610, 0x80);
	write_cmos_sensor_8(0x5611, 0x80);
	write_cmos_sensor_8(0x5612, 0x80);
	write_cmos_sensor_8(0x5613, 0x80);
	write_cmos_sensor_8(0x5614, 0x7f);
	write_cmos_sensor_8(0x5615, 0x7f);
	write_cmos_sensor_8(0x5616, 0x80);
	write_cmos_sensor_8(0x5617, 0x7f);
	write_cmos_sensor_8(0x5618, 0x80);
	write_cmos_sensor_8(0x5619, 0x7f);
	write_cmos_sensor_8(0x561a, 0x7f);
	write_cmos_sensor_8(0x561b, 0x80);
	write_cmos_sensor_8(0x561c, 0x7f);
	write_cmos_sensor_8(0x561d, 0x7f);
	write_cmos_sensor_8(0x561e, 0x7f);
	write_cmos_sensor_8(0x561f, 0x82);
	write_cmos_sensor_8(0x5621, 0x80);
	write_cmos_sensor_8(0x5622, 0x7f);
	write_cmos_sensor_8(0x5623, 0x7f);
	write_cmos_sensor_8(0x5624, 0x7f);
	write_cmos_sensor_8(0x5625, 0x7f);
	write_cmos_sensor_8(0x5626, 0x7f);
	write_cmos_sensor_8(0x5627, 0x7f);
	write_cmos_sensor_8(0x5628, 0x7f);
	write_cmos_sensor_8(0x5629, 0x7f);
	write_cmos_sensor_8(0x562a, 0x7f);
	write_cmos_sensor_8(0x562b, 0x7f);
	write_cmos_sensor_8(0x562c, 0x7f);
	write_cmos_sensor_8(0x562d, 0x80);
	write_cmos_sensor_8(0x562e, 0x7f);
	write_cmos_sensor_8(0x562f, 0x81);
	write_cmos_sensor_8(0x5630, 0x80);
	write_cmos_sensor_8(0x5631, 0x7f);
	write_cmos_sensor_8(0x5632, 0x7f);
	write_cmos_sensor_8(0x5633, 0x7f);
	write_cmos_sensor_8(0x5634, 0x7f);
	write_cmos_sensor_8(0x5635, 0x7f);
	write_cmos_sensor_8(0x5636, 0x7f);
	write_cmos_sensor_8(0x5637, 0x7f);
	write_cmos_sensor_8(0x5638, 0x7f);
	write_cmos_sensor_8(0x5639, 0x7f);
	write_cmos_sensor_8(0x563a, 0x7f);
	write_cmos_sensor_8(0x563b, 0x7f);
	write_cmos_sensor_8(0x563c, 0x7f);
	write_cmos_sensor_8(0x563d, 0x7f);
	write_cmos_sensor_8(0x563e, 0x80);
	write_cmos_sensor_8(0x563f, 0x80);
	write_cmos_sensor_8(0x5640, 0x81);
	write_cmos_sensor_8(0x5641, 0x80);
	write_cmos_sensor_8(0x5643, 0x7f);
	write_cmos_sensor_8(0x5644, 0x7f);
	write_cmos_sensor_8(0x5645, 0x7f);
	write_cmos_sensor_8(0x5646, 0x7f);
	write_cmos_sensor_8(0x5647, 0x7f);
	write_cmos_sensor_8(0x5648, 0x7f);
	write_cmos_sensor_8(0x5649, 0x7f);
	write_cmos_sensor_8(0x564a, 0x7f);
	write_cmos_sensor_8(0x564b, 0x7f);
	write_cmos_sensor_8(0x564c, 0x7f);
	write_cmos_sensor_8(0x564d, 0x7f);
	write_cmos_sensor_8(0x564e, 0x80);
	write_cmos_sensor_8(0x564f, 0x7f);
	write_cmos_sensor_8(0x5650, 0x80);
	write_cmos_sensor_8(0x5652, 0x80);
	write_cmos_sensor_8(0x5653, 0x7f);
	write_cmos_sensor_8(0x5654, 0x7f);
	write_cmos_sensor_8(0x5655, 0x7f);
	write_cmos_sensor_8(0x5656, 0x7f);
	write_cmos_sensor_8(0x5657, 0x7f);
	write_cmos_sensor_8(0x5658, 0x7f);
	write_cmos_sensor_8(0x5659, 0x7f);
	write_cmos_sensor_8(0x565a, 0x7f);
	write_cmos_sensor_8(0x565b, 0x7f);
	write_cmos_sensor_8(0x565c, 0x7f);
	write_cmos_sensor_8(0x565d, 0x7f);
	write_cmos_sensor_8(0x565f, 0x7f);
	write_cmos_sensor_8(0x5660, 0x80);
	write_cmos_sensor_8(0x5661, 0x80);
	write_cmos_sensor_8(0x5663, 0x7f);
	write_cmos_sensor_8(0x5666, 0x7f);
	write_cmos_sensor_8(0x5667, 0x7f);
	write_cmos_sensor_8(0x5668, 0x7f);
	write_cmos_sensor_8(0x5669, 0x7f);
	write_cmos_sensor_8(0x566c, 0x80);
	write_cmos_sensor_8(0x566d, 0x80);
	write_cmos_sensor_8(0x566e, 0x7f);
	write_cmos_sensor_8(0x566f, 0x80);
	write_cmos_sensor_8(0x5671, 0x80);
	write_cmos_sensor_8(0x5672, 0x80);
	write_cmos_sensor_8(0x5673, 0x80);
	write_cmos_sensor_8(0x5676, 0x80);
	write_cmos_sensor_8(0x5677, 0x80);
	write_cmos_sensor_8(0x5678, 0x80);
	write_cmos_sensor_8(0x5679, 0x80);
	write_cmos_sensor_8(0x567a, 0x80);
	write_cmos_sensor_8(0x567b, 0x80);
	write_cmos_sensor_8(0x567c, 0x80);
	write_cmos_sensor_8(0x567d, 0x80);
	write_cmos_sensor_8(0x567e, 0x80);
	write_cmos_sensor_8(0x567f, 0x80);
	write_cmos_sensor_8(0x5680, 0x80);
	write_cmos_sensor_8(0x5681, 0x80);
	write_cmos_sensor_8(0x5682, 0x80);
	write_cmos_sensor_8(0x5684, 0x81);
	write_cmos_sensor_8(0x5685, 0x80);
	write_cmos_sensor_8(0x5686, 0x80);
	write_cmos_sensor_8(0x5687, 0x80);
	write_cmos_sensor_8(0x5688, 0x80);
	write_cmos_sensor_8(0x5689, 0x80);
	write_cmos_sensor_8(0x568a, 0x80);
	write_cmos_sensor_8(0x568b, 0x80);
	write_cmos_sensor_8(0x568c, 0x80);
	write_cmos_sensor_8(0x568d, 0x80);
	write_cmos_sensor_8(0x568e, 0x80);
	write_cmos_sensor_8(0x5691, 0x80);
	write_cmos_sensor_8(0x5692, 0x80);
	write_cmos_sensor_8(0x5695, 0x7f);
	write_cmos_sensor_8(0x5696, 0x80);
	write_cmos_sensor_8(0x5697, 0x80);
	write_cmos_sensor_8(0x5698, 0x80);
	write_cmos_sensor_8(0x5699, 0x80);
	write_cmos_sensor_8(0x569a, 0x80);
	write_cmos_sensor_8(0x569b, 0x80);
	write_cmos_sensor_8(0x569c, 0x80);
	write_cmos_sensor_8(0x569d, 0x80);
	write_cmos_sensor_8(0x569f, 0x81);
	write_cmos_sensor_8(0x56a1, 0x80);
	write_cmos_sensor_8(0x56a6, 0x7f);
	write_cmos_sensor_8(0x56a7, 0x7f);
	write_cmos_sensor_8(0x56a8, 0x7f);
	write_cmos_sensor_8(0x56a9, 0x7f);
	write_cmos_sensor_8(0x56aa, 0x80);
	write_cmos_sensor_8(0x56ab, 0x80);
	write_cmos_sensor_8(0x56ad, 0x80);
	write_cmos_sensor_8(0x56af, 0x80);
	write_cmos_sensor_8(0x56b5, 0x7f);
	write_cmos_sensor_8(0x56b6, 0x7f);
	write_cmos_sensor_8(0x56b7, 0x7f);
	write_cmos_sensor_8(0x56b8, 0x7f);
	write_cmos_sensor_8(0x56b9, 0x7f);
	write_cmos_sensor_8(0x56bb, 0x7f);
	write_cmos_sensor_8(0x56be, 0x80);
	write_cmos_sensor_8(0x56bf, 0x80);
	write_cmos_sensor_8(0x56c0, 0x80);
	write_cmos_sensor_8(0x56c3, 0x80);
	write_cmos_sensor_8(0x56c6, 0x7f);
	write_cmos_sensor_8(0x56c8, 0x7f);
	write_cmos_sensor_8(0x56c9, 0x7f);
	write_cmos_sensor_8(0x56ca, 0x7f);
	write_cmos_sensor_8(0x56cb, 0x80);
	write_cmos_sensor_8(0x56cc, 0x80);
	write_cmos_sensor_8(0x56d0, 0x81);
	write_cmos_sensor_8(0x56d1, 0x80);
	write_cmos_sensor_8(0x56d3, 0x80);
	write_cmos_sensor_8(0x56d4, 0x7f);
	write_cmos_sensor_8(0x56d5, 0x80);
	write_cmos_sensor_8(0x56d6, 0x7f);
	write_cmos_sensor_8(0x56d8, 0x80);
	write_cmos_sensor_8(0x56da, 0x80);
	write_cmos_sensor_8(0x56db, 0x7f);
	write_cmos_sensor_8(0x56dd, 0x80);
	write_cmos_sensor_8(0x56df, 0x81);
	write_cmos_sensor_8(0x56e1, 0x7f);
	write_cmos_sensor_8(0x56e2, 0x80);
	write_cmos_sensor_8(0x56e3, 0x7f);
	write_cmos_sensor_8(0x56e4, 0x7f);
	write_cmos_sensor_8(0x56e5, 0x7f);
	write_cmos_sensor_8(0x56e7, 0x80);
	write_cmos_sensor_8(0x56e8, 0x7f);
	write_cmos_sensor_8(0x56e9, 0x80);
	write_cmos_sensor_8(0x56ea, 0x7f);
	write_cmos_sensor_8(0x56eb, 0x80);
	write_cmos_sensor_8(0x56ec, 0x80);
	write_cmos_sensor_8(0x56ee, 0x80);
	write_cmos_sensor_8(0x56ef, 0x7f);
	write_cmos_sensor_8(0x56f0, 0x7b);
	write_cmos_sensor_8(0x56f1, 0x81);
	write_cmos_sensor_8(0x56f2, 0x7f);
	write_cmos_sensor_8(0x56f3, 0x80);
	write_cmos_sensor_8(0x56f5, 0x7f);
	write_cmos_sensor_8(0x56f6, 0x7f);
	write_cmos_sensor_8(0x56f7, 0x80);
	write_cmos_sensor_8(0x56f9, 0x7f);
	write_cmos_sensor_8(0x56fc, 0x7f);
	write_cmos_sensor_8(0x56fd, 0x81);
	write_cmos_sensor_8(0x56fe, 0x80);
	write_cmos_sensor_8(0x56ff, 0x7f);
	write_cmos_sensor_8(0x5700, 0x84);
	write_cmos_sensor_8(0x5701, 0x7e);
	write_cmos_sensor_8(0x5702, 0x81);
	write_cmos_sensor_8(0x5703, 0x7d);
	write_cmos_sensor_8(0x5704, 0x80);
	write_cmos_sensor_8(0x5705, 0x7f);
	write_cmos_sensor_8(0x5706, 0x7f);
	write_cmos_sensor_8(0x5707, 0x81);
	write_cmos_sensor_8(0x5708, 0x7e);
	write_cmos_sensor_8(0x5709, 0x7e);
	write_cmos_sensor_8(0x570a, 0x81);
	write_cmos_sensor_8(0x570b, 0x7f);
	write_cmos_sensor_8(0x570c, 0x80);
	write_cmos_sensor_8(0x570d, 0x7f);
	write_cmos_sensor_8(0x570e, 0x7f);
	write_cmos_sensor_8(0x570f, 0x81);
	write_cmos_sensor_8(0x5710, 0x7d);
	write_cmos_sensor_8(0x5711, 0x80);
	write_cmos_sensor_8(0x5712, 0x7e);
	write_cmos_sensor_8(0x5713, 0x80);
	write_cmos_sensor_8(0x5714, 0x7f);
	write_cmos_sensor_8(0x5715, 0x7f);
	write_cmos_sensor_8(0x5716, 0x7f);
	write_cmos_sensor_8(0x5717, 0x7f);
	write_cmos_sensor_8(0x5718, 0x80);
	write_cmos_sensor_8(0x5719, 0x80);
	write_cmos_sensor_8(0x571a, 0x7f);
	write_cmos_sensor_8(0x571b, 0x80);
	write_cmos_sensor_8(0x571c, 0x7f);
	write_cmos_sensor_8(0x571d, 0x80);
	write_cmos_sensor_8(0x571e, 0x7f);
	write_cmos_sensor_8(0x571f, 0x81);
	write_cmos_sensor_8(0x5720, 0x80);
	write_cmos_sensor_8(0x5721, 0x7f);
	write_cmos_sensor_8(0x5722, 0x80);
	write_cmos_sensor_8(0x5723, 0x7f);
	write_cmos_sensor_8(0x5724, 0x7f);
	write_cmos_sensor_8(0x5725, 0x7f);
	write_cmos_sensor_8(0x5726, 0x7f);
	write_cmos_sensor_8(0x5727, 0x7f);
	write_cmos_sensor_8(0x5728, 0x7f);
	write_cmos_sensor_8(0x5729, 0x7f);
	write_cmos_sensor_8(0x572a, 0x7f);
	write_cmos_sensor_8(0x572b, 0x7f);
	write_cmos_sensor_8(0x572c, 0x7f);
	write_cmos_sensor_8(0x572d, 0x7f);
	write_cmos_sensor_8(0x572e, 0x7f);
	write_cmos_sensor_8(0x572f, 0x81);
	write_cmos_sensor_8(0x5730, 0x7f);
	write_cmos_sensor_8(0x5731, 0x7f);
	write_cmos_sensor_8(0x5732, 0x7f);
	write_cmos_sensor_8(0x5733, 0x7f);
	write_cmos_sensor_8(0x5734, 0x7f);
	write_cmos_sensor_8(0x5735, 0x7f);
	write_cmos_sensor_8(0x5736, 0x7f);
	write_cmos_sensor_8(0x5737, 0x7f);
	write_cmos_sensor_8(0x5738, 0x7f);
	write_cmos_sensor_8(0x5739, 0x7f);
	write_cmos_sensor_8(0x573a, 0x7f);
	write_cmos_sensor_8(0x573b, 0x7f);
	write_cmos_sensor_8(0x573c, 0x7f);
	write_cmos_sensor_8(0x573d, 0x7f);
	write_cmos_sensor_8(0x573e, 0x7f);
	write_cmos_sensor_8(0x573f, 0x80);
	write_cmos_sensor_8(0x5740, 0x80);
	write_cmos_sensor_8(0x5741, 0x7f);
	write_cmos_sensor_8(0x5742, 0x80);
	write_cmos_sensor_8(0x5743, 0x7f);
	write_cmos_sensor_8(0x5744, 0x7f);
	write_cmos_sensor_8(0x5745, 0x7f);
	write_cmos_sensor_8(0x5746, 0x7f);
	write_cmos_sensor_8(0x5747, 0x7f);
	write_cmos_sensor_8(0x5748, 0x7f);
	write_cmos_sensor_8(0x5749, 0x7f);
	write_cmos_sensor_8(0x574a, 0x7f);
	write_cmos_sensor_8(0x574b, 0x7f);
	write_cmos_sensor_8(0x574c, 0x7f);
	write_cmos_sensor_8(0x574d, 0x7f);
	write_cmos_sensor_8(0x574e, 0x7f);
	write_cmos_sensor_8(0x574f, 0x80);
	write_cmos_sensor_8(0x5750, 0x80);
	write_cmos_sensor_8(0x5751, 0x7f);
	write_cmos_sensor_8(0x5752, 0x7f);
	write_cmos_sensor_8(0x5753, 0x7f);
	write_cmos_sensor_8(0x5754, 0x7f);
	write_cmos_sensor_8(0x5755, 0x7f);
	write_cmos_sensor_8(0x5756, 0x7f);
	write_cmos_sensor_8(0x5757, 0x7f);
	write_cmos_sensor_8(0x5758, 0x7f);
	write_cmos_sensor_8(0x5759, 0x7f);
	write_cmos_sensor_8(0x575a, 0x7f);
	write_cmos_sensor_8(0x575b, 0x7f);
	write_cmos_sensor_8(0x575c, 0x7f);
	write_cmos_sensor_8(0x575d, 0x7f);
	write_cmos_sensor_8(0x575e, 0x7f);
	write_cmos_sensor_8(0x575f, 0x80);
	write_cmos_sensor_8(0x5760, 0x7e);
	write_cmos_sensor_8(0x5761, 0x80);
	write_cmos_sensor_8(0x5762, 0x7f);
	write_cmos_sensor_8(0x5763, 0x7f);
	write_cmos_sensor_8(0x5764, 0x7f);
	write_cmos_sensor_8(0x5766, 0x7f);
	write_cmos_sensor_8(0x5767, 0x7f);
	write_cmos_sensor_8(0x5768, 0x80);
	write_cmos_sensor_8(0x5769, 0x7f);
	write_cmos_sensor_8(0x576a, 0x80);
	write_cmos_sensor_8(0x576b, 0x7f);
	write_cmos_sensor_8(0x576c, 0x80);
	write_cmos_sensor_8(0x576d, 0x80);
	write_cmos_sensor_8(0x576e, 0x7f);
	write_cmos_sensor_8(0x576f, 0x80);
	write_cmos_sensor_8(0x5770, 0x80);
	write_cmos_sensor_8(0x5771, 0x80);
	write_cmos_sensor_8(0x5772, 0x80);
	write_cmos_sensor_8(0x5774, 0x80);
	write_cmos_sensor_8(0x5775, 0x80);
	write_cmos_sensor_8(0x5776, 0x80);
	write_cmos_sensor_8(0x5778, 0x80);
	write_cmos_sensor_8(0x5779, 0x80);
	write_cmos_sensor_8(0x577a, 0x80);
	write_cmos_sensor_8(0x577b, 0x80);
	write_cmos_sensor_8(0x577c, 0x7f);
	write_cmos_sensor_8(0x577d, 0x80);
	write_cmos_sensor_8(0x577e, 0x80);
	write_cmos_sensor_8(0x577f, 0x80);
	write_cmos_sensor_8(0x5780, 0x80);
	write_cmos_sensor_8(0x5781, 0x80);
	write_cmos_sensor_8(0x5782, 0x80);
	write_cmos_sensor_8(0x5784, 0x80);
	write_cmos_sensor_8(0x5785, 0x80);
	write_cmos_sensor_8(0x5786, 0x80);
	write_cmos_sensor_8(0x5788, 0x80);
	write_cmos_sensor_8(0x5789, 0x81);
	write_cmos_sensor_8(0x578a, 0x80);
	write_cmos_sensor_8(0x578b, 0x80);
	write_cmos_sensor_8(0x578c, 0x80);
	write_cmos_sensor_8(0x578d, 0x80);
	write_cmos_sensor_8(0x578e, 0x80);
	write_cmos_sensor_8(0x578f, 0x80);
	write_cmos_sensor_8(0x5790, 0x7f);
	write_cmos_sensor_8(0x5791, 0x80);
	write_cmos_sensor_8(0x5792, 0x80);
	write_cmos_sensor_8(0x5795, 0x80);
	write_cmos_sensor_8(0x5798, 0x80);
	write_cmos_sensor_8(0x5799, 0x7f);
	write_cmos_sensor_8(0x579a, 0x80);
	write_cmos_sensor_8(0x579b, 0x80);
	write_cmos_sensor_8(0x579c, 0x80);
	write_cmos_sensor_8(0x579d, 0x80);
	write_cmos_sensor_8(0x579e, 0x80);
	write_cmos_sensor_8(0x579f, 0x80);
	write_cmos_sensor_8(0x57a0, 0x7f);
	write_cmos_sensor_8(0x57a1, 0x80);
	write_cmos_sensor_8(0x57a2, 0x80);
	write_cmos_sensor_8(0x57a3, 0x7f);
	write_cmos_sensor_8(0x57a4, 0x80);
	write_cmos_sensor_8(0x57a5, 0x7f);
	write_cmos_sensor_8(0x57a6, 0x80);
	write_cmos_sensor_8(0x57a7, 0x7f);
	write_cmos_sensor_8(0x57a8, 0x7f);
	write_cmos_sensor_8(0x57a9, 0x7f);
	write_cmos_sensor_8(0x57aa, 0x80);
	write_cmos_sensor_8(0x57ab, 0x7f);
	write_cmos_sensor_8(0x57ac, 0x7f);
	write_cmos_sensor_8(0x57ad, 0x80);
	write_cmos_sensor_8(0x57ae, 0x80);
	write_cmos_sensor_8(0x57af, 0x81);
	write_cmos_sensor_8(0x57b0, 0x80);
	write_cmos_sensor_8(0x57b1, 0x7f);
	write_cmos_sensor_8(0x57b2, 0x7f);
	write_cmos_sensor_8(0x57b3, 0x7f);
	write_cmos_sensor_8(0x57b4, 0x7f);
	write_cmos_sensor_8(0x57b5, 0x7f);
	write_cmos_sensor_8(0x57b6, 0x7f);
	write_cmos_sensor_8(0x57b7, 0x7f);
	write_cmos_sensor_8(0x57b8, 0x7f);
	write_cmos_sensor_8(0x57b9, 0x7f);
	write_cmos_sensor_8(0x57ba, 0x7f);
	write_cmos_sensor_8(0x57bb, 0x7f);
	write_cmos_sensor_8(0x57bc, 0x80);
	write_cmos_sensor_8(0x57bd, 0x7f);
	write_cmos_sensor_8(0x57be, 0x80);
	write_cmos_sensor_8(0x57bf, 0x7f);
	write_cmos_sensor_8(0x57c0, 0x7f);
	write_cmos_sensor_8(0x57c1, 0x7f);
	write_cmos_sensor_8(0x57c2, 0x7f);
	write_cmos_sensor_8(0x57c3, 0x7f);
	write_cmos_sensor_8(0x57c4, 0x7f);
	write_cmos_sensor_8(0x57c5, 0x7f);
	write_cmos_sensor_8(0x57c6, 0x80);
	write_cmos_sensor_8(0x57c7, 0x7f);
	write_cmos_sensor_8(0x57c8, 0x80);
	write_cmos_sensor_8(0x57c9, 0x7f);
	write_cmos_sensor_8(0x57ca, 0x7f);
	write_cmos_sensor_8(0x57cb, 0x7f);
	write_cmos_sensor_8(0x57cc, 0x80);
	write_cmos_sensor_8(0x57cd, 0x80);
	write_cmos_sensor_8(0x57ce, 0x7f);
	write_cmos_sensor_8(0x57cf, 0x81);
	write_cmos_sensor_8(0x57d0, 0x80);
	write_cmos_sensor_8(0x57d1, 0x80);
	write_cmos_sensor_8(0x57d2, 0x80);
	write_cmos_sensor_8(0x57d3, 0x7f);
	write_cmos_sensor_8(0x57d4, 0x7f);
	write_cmos_sensor_8(0x57d5, 0x80);
	write_cmos_sensor_8(0x57d6, 0x7f);
	write_cmos_sensor_8(0x57d7, 0x7f);
	write_cmos_sensor_8(0x57d8, 0x7f);
	write_cmos_sensor_8(0x57d9, 0x7f);
	write_cmos_sensor_8(0x57da, 0x7f);
	write_cmos_sensor_8(0x57db, 0x80);
	write_cmos_sensor_8(0x57dc, 0x7f);
	write_cmos_sensor_8(0x57dd, 0x80);
	write_cmos_sensor_8(0x57de, 0x80);
	write_cmos_sensor_8(0x57df, 0x80);
	write_cmos_sensor_8(0x57e0, 0x7f);
	write_cmos_sensor_8(0x57e1, 0x7f);
	write_cmos_sensor_8(0x57e2, 0x80);
	write_cmos_sensor_8(0x57e3, 0x7f);
	write_cmos_sensor_8(0x57e4, 0x80);
	write_cmos_sensor_8(0x57e5, 0x7f);
	write_cmos_sensor_8(0x57e6, 0x80);
	write_cmos_sensor_8(0x57e7, 0x7f);
	write_cmos_sensor_8(0x57e8, 0x80);
	write_cmos_sensor_8(0x57e9, 0x7f);
	write_cmos_sensor_8(0x57ea, 0x80);
	write_cmos_sensor_8(0x57eb, 0x80);
	write_cmos_sensor_8(0x57ec, 0x80);
	write_cmos_sensor_8(0x57ed, 0x7f);
	write_cmos_sensor_8(0x57ee, 0x80);
	write_cmos_sensor_8(0x57ef, 0x80);
	write_cmos_sensor_8(0x57f0, 0x7f);
	write_cmos_sensor_8(0x57f1, 0x80);
	write_cmos_sensor_8(0x57f2, 0x7e);
	write_cmos_sensor_8(0x57f3, 0x80);
	write_cmos_sensor_8(0x57f4, 0x7e);
	write_cmos_sensor_8(0x57f5, 0x81);
	write_cmos_sensor_8(0x57f6, 0x7e);
	write_cmos_sensor_8(0x57f7, 0x81);
	write_cmos_sensor_8(0x57f8, 0x7e);
	write_cmos_sensor_8(0x57f9, 0x80);
	write_cmos_sensor_8(0x57fa, 0x80);
	write_cmos_sensor_8(0x57fb, 0x80);
	write_cmos_sensor_8(0x57fc, 0x7e);
	write_cmos_sensor_8(0x57fd, 0x81);
	write_cmos_sensor_8(0x57fe, 0x80);
	write_cmos_sensor_8(0x57ff, 0x7a);
	//write_cmos_sensor_8(0x0100, 0x01);
	}
	else {  //full size 24fps for PIP 
	//write_cmos_sensor_8(0x0100, 0x00);
	write_cmos_sensor_8(0x0301, 0xa5);
	write_cmos_sensor_8(0x0304, 0x29);//0x28  /*prize  add  for k6307  RF intrusion by zhuzhengjiang    20190925-begin*/
	write_cmos_sensor_8(0x0316, 0x80); // for 24fps
	write_cmos_sensor_8(0x3500, 0x00);
	write_cmos_sensor_8(0x3501, 0xec);
	write_cmos_sensor_8(0x3502, 0x00);
	write_cmos_sensor_8(0x3511, 0xec);
	write_cmos_sensor_8(0x3600, 0x40);
	write_cmos_sensor_8(0x3602, 0x84);
	write_cmos_sensor_8(0x3621, 0x66);
	write_cmos_sensor_8(0x3701, 0x14);
	write_cmos_sensor_8(0x3709, 0x38);
	write_cmos_sensor_8(0x37f4, 0x80);
	write_cmos_sensor_8(0x3803, 0x02);
	write_cmos_sensor_8(0x3807, 0xd1);
	write_cmos_sensor_8(0x3808, 0x09);
	write_cmos_sensor_8(0x3809, 0x20);
	write_cmos_sensor_8(0x380a, 0x06);
	write_cmos_sensor_8(0x380b, 0xd8);
	write_cmos_sensor_8(0x380c, 0x0a);
	write_cmos_sensor_8(0x380d, 0xf0);
	write_cmos_sensor_8(0x380e, 0x0e);
	write_cmos_sensor_8(0x380f, 0xe0);
	write_cmos_sensor_8(0x3810, 0x00);
	write_cmos_sensor_8(0x3811, 0x09);
	write_cmos_sensor_8(0x3812, 0x00);
	write_cmos_sensor_8(0x3813, 0x09);
	write_cmos_sensor_8(0x3815, 0x22);
	write_cmos_sensor_8(0x3820, 0x01);
	write_cmos_sensor_8(0x3834, 0xf5);
	write_cmos_sensor_8(0x3f03, 0x1a);
	write_cmos_sensor_8(0x4010, 0x29);
	write_cmos_sensor_8(0x4012, 0x0d);
	write_cmos_sensor_8(0x4013, 0x16);
	write_cmos_sensor_8(0x4014, 0x0a);
	write_cmos_sensor_8(0x4016, 0x15);
	write_cmos_sensor_8(0x4018, 0x09);
	write_cmos_sensor_8(0x4500, 0x25);
	write_cmos_sensor_8(0x4501, 0x04);
	write_cmos_sensor_8(0x4503, 0x10);
	write_cmos_sensor_8(0x450f, 0x88);
	write_cmos_sensor_8(0x4540, 0x86);
	write_cmos_sensor_8(0x4541, 0x07);
	write_cmos_sensor_8(0x4542, 0x04);
	write_cmos_sensor_8(0x4543, 0x05);
	write_cmos_sensor_8(0x4544, 0x06);
	write_cmos_sensor_8(0x4545, 0x07);
	write_cmos_sensor_8(0x4546, 0x04);
	write_cmos_sensor_8(0x4547, 0x05);
	write_cmos_sensor_8(0x4548, 0x02);
	write_cmos_sensor_8(0x4549, 0x03);
	write_cmos_sensor_8(0x454a, 0x00);
	write_cmos_sensor_8(0x454b, 0x01);
	write_cmos_sensor_8(0x454c, 0x02);
	write_cmos_sensor_8(0x454d, 0x03);
	write_cmos_sensor_8(0x454e, 0x00);
	write_cmos_sensor_8(0x454f, 0x01);
	write_cmos_sensor_8(0x4550, 0x06);
	write_cmos_sensor_8(0x4551, 0x07);
	write_cmos_sensor_8(0x4552, 0x04);
	write_cmos_sensor_8(0x4553, 0x05);
	write_cmos_sensor_8(0x4554, 0x06);
	write_cmos_sensor_8(0x4555, 0x07);
	write_cmos_sensor_8(0x4556, 0x04);
	write_cmos_sensor_8(0x4557, 0x05);
	write_cmos_sensor_8(0x4558, 0x02);
	write_cmos_sensor_8(0x4559, 0x03);
	write_cmos_sensor_8(0x455a, 0x00);
	write_cmos_sensor_8(0x455b, 0x01);
	write_cmos_sensor_8(0x455c, 0x02);
	write_cmos_sensor_8(0x455d, 0x03);
	write_cmos_sensor_8(0x455e, 0x00);
	write_cmos_sensor_8(0x455f, 0x01);
	write_cmos_sensor_8(0x4837, 0x14);
	write_cmos_sensor_8(0x5080, 0x04);
	write_cmos_sensor_8(0x5084, 0x09);
	write_cmos_sensor_8(0x5085, 0x30);
	write_cmos_sensor_8(0x5086, 0x06);
	write_cmos_sensor_8(0x5087, 0xe6);
	write_cmos_sensor_8(0x5000, 0x89);
	write_cmos_sensor_8(0x5001, 0x42);
	write_cmos_sensor_8(0x5004, 0x81);
	write_cmos_sensor_8(0x5184, 0x63);
	write_cmos_sensor_8(0x5185, 0x17);
	write_cmos_sensor_8(0x5192, 0x09);
	write_cmos_sensor_8(0x5193, 0x2f);
	write_cmos_sensor_8(0x5406, 0x06);
	write_cmos_sensor_8(0x5407, 0x18);
	write_cmos_sensor_8(0x5408, 0x08);
	write_cmos_sensor_8(0x5409, 0x1a);
	write_cmos_sensor_8(0x5410, 0x03);
	write_cmos_sensor_8(0x5500, 0x4a);
	write_cmos_sensor_8(0x5501, 0x43);
	write_cmos_sensor_8(0x5502, 0x3c);
	write_cmos_sensor_8(0x5503, 0x38);
	write_cmos_sensor_8(0x5504, 0x34);
	write_cmos_sensor_8(0x5505, 0x32);
	write_cmos_sensor_8(0x5506, 0x30);
	write_cmos_sensor_8(0x5507, 0x2f);
	write_cmos_sensor_8(0x5508, 0x31);
	write_cmos_sensor_8(0x5509, 0x30);
	write_cmos_sensor_8(0x550a, 0x31);
	write_cmos_sensor_8(0x550b, 0x34);
	write_cmos_sensor_8(0x550c, 0x38);
	write_cmos_sensor_8(0x550d, 0x3d);
	write_cmos_sensor_8(0x550e, 0x44);
	write_cmos_sensor_8(0x550f, 0x51);
	write_cmos_sensor_8(0x5510, 0x3f);
	write_cmos_sensor_8(0x5511, 0x35);
	write_cmos_sensor_8(0x5512, 0x30);
	write_cmos_sensor_8(0x5513, 0x2c);
	write_cmos_sensor_8(0x5514, 0x2a);
	write_cmos_sensor_8(0x5515, 0x27);
	write_cmos_sensor_8(0x5516, 0x26);
	write_cmos_sensor_8(0x5517, 0x26);
	write_cmos_sensor_8(0x5518, 0x25);
	write_cmos_sensor_8(0x5519, 0x26);
	write_cmos_sensor_8(0x551a, 0x28);
	write_cmos_sensor_8(0x551b, 0x2a);
	write_cmos_sensor_8(0x551c, 0x2d);
	write_cmos_sensor_8(0x551d, 0x31);
	write_cmos_sensor_8(0x551e, 0x37);
	write_cmos_sensor_8(0x551f, 0x40);
	write_cmos_sensor_8(0x5520, 0x32);
	write_cmos_sensor_8(0x5521, 0x2c);
	write_cmos_sensor_8(0x5522, 0x28);
	write_cmos_sensor_8(0x5523, 0x24);
	write_cmos_sensor_8(0x5524, 0x21);
	write_cmos_sensor_8(0x5525, 0x1e);
	write_cmos_sensor_8(0x5526, 0x1d);
	write_cmos_sensor_8(0x5527, 0x1c);
	write_cmos_sensor_8(0x5528, 0x1d);
	write_cmos_sensor_8(0x5529, 0x1d);
	write_cmos_sensor_8(0x552a, 0x1f);
	write_cmos_sensor_8(0x552b, 0x22);
	write_cmos_sensor_8(0x552c, 0x25);
	write_cmos_sensor_8(0x552d, 0x29);
	write_cmos_sensor_8(0x552e, 0x2e);
	write_cmos_sensor_8(0x552f, 0x36);
	write_cmos_sensor_8(0x5530, 0x2b);
	write_cmos_sensor_8(0x5531, 0x25);
	write_cmos_sensor_8(0x5532, 0x20);
	write_cmos_sensor_8(0x5533, 0x1c);
	write_cmos_sensor_8(0x5534, 0x19);
	write_cmos_sensor_8(0x5535, 0x16);
	write_cmos_sensor_8(0x5536, 0x15);
	write_cmos_sensor_8(0x5537, 0x14);
	write_cmos_sensor_8(0x5538, 0x14);
	write_cmos_sensor_8(0x5539, 0x15);
	write_cmos_sensor_8(0x553a, 0x17);
	write_cmos_sensor_8(0x553b, 0x19);
	write_cmos_sensor_8(0x553c, 0x1d);
	write_cmos_sensor_8(0x553d, 0x22);
	write_cmos_sensor_8(0x553e, 0x27);
	write_cmos_sensor_8(0x553f, 0x2f);
	write_cmos_sensor_8(0x5540, 0x25);
	write_cmos_sensor_8(0x5541, 0x1f);
	write_cmos_sensor_8(0x5542, 0x1a);
	write_cmos_sensor_8(0x5543, 0x16);
	write_cmos_sensor_8(0x5544, 0x12);
	write_cmos_sensor_8(0x5545, 0x0f);
	write_cmos_sensor_8(0x5546, 0x0d);
	write_cmos_sensor_8(0x5547, 0x0d);
	write_cmos_sensor_8(0x5548, 0x0d);
	write_cmos_sensor_8(0x5549, 0x0e);
	write_cmos_sensor_8(0x554a, 0x10);
	write_cmos_sensor_8(0x554b, 0x13);
	write_cmos_sensor_8(0x554c, 0x16);
	write_cmos_sensor_8(0x554d, 0x1b);
	write_cmos_sensor_8(0x554e, 0x21);
	write_cmos_sensor_8(0x554f, 0x29);
	write_cmos_sensor_8(0x5550, 0x20);
	write_cmos_sensor_8(0x5551, 0x1a);
	write_cmos_sensor_8(0x5552, 0x15);
	write_cmos_sensor_8(0x5553, 0x10);
	write_cmos_sensor_8(0x5554, 0x0c);
	write_cmos_sensor_8(0x5555, 0x09);
	write_cmos_sensor_8(0x5556, 0x07);
	write_cmos_sensor_8(0x5557, 0x07);
	write_cmos_sensor_8(0x5558, 0x06);
	write_cmos_sensor_8(0x5559, 0x07);
	write_cmos_sensor_8(0x555a, 0x09);
	write_cmos_sensor_8(0x555b, 0x0d);
	write_cmos_sensor_8(0x555c, 0x11);
	write_cmos_sensor_8(0x555d, 0x16);
	write_cmos_sensor_8(0x555e, 0x1c);
	write_cmos_sensor_8(0x555f, 0x24);
	write_cmos_sensor_8(0x5560, 0x1e);
	write_cmos_sensor_8(0x5561, 0x17);
	write_cmos_sensor_8(0x5562, 0x11);
	write_cmos_sensor_8(0x5563, 0x0d);
	write_cmos_sensor_8(0x5564, 0x08);
	write_cmos_sensor_8(0x5565, 0x05);
	write_cmos_sensor_8(0x5566, 0x03);
	write_cmos_sensor_8(0x5567, 0x02);
	write_cmos_sensor_8(0x5568, 0x02);
	write_cmos_sensor_8(0x5569, 0x03);
	write_cmos_sensor_8(0x556a, 0x05);
	write_cmos_sensor_8(0x556b, 0x09);
	write_cmos_sensor_8(0x556c, 0x0d);
	write_cmos_sensor_8(0x556d, 0x12);
	write_cmos_sensor_8(0x556e, 0x19);
	write_cmos_sensor_8(0x556f, 0x20);
	write_cmos_sensor_8(0x5570, 0x1c);
	write_cmos_sensor_8(0x5571, 0x15);
	write_cmos_sensor_8(0x5572, 0x0f);
	write_cmos_sensor_8(0x5573, 0x0a);
	write_cmos_sensor_8(0x5574, 0x06);
	write_cmos_sensor_8(0x5575, 0x03);
	write_cmos_sensor_8(0x5576, 0x01);
	write_cmos_sensor_8(0x5577, 0x00);
	write_cmos_sensor_8(0x5578, 0x00);
	write_cmos_sensor_8(0x5579, 0x01);
	write_cmos_sensor_8(0x557a, 0x03);
	write_cmos_sensor_8(0x557b, 0x06);
	write_cmos_sensor_8(0x557c, 0x0b);
	write_cmos_sensor_8(0x557d, 0x10);
	write_cmos_sensor_8(0x557e, 0x17);
	write_cmos_sensor_8(0x557f, 0x1f);
	write_cmos_sensor_8(0x5580, 0x1c);
	write_cmos_sensor_8(0x5581, 0x15);
	write_cmos_sensor_8(0x5582, 0x0f);
	write_cmos_sensor_8(0x5583, 0x0a);
	write_cmos_sensor_8(0x5584, 0x06);
	write_cmos_sensor_8(0x5585, 0x03);
	write_cmos_sensor_8(0x5589, 0x01);
	write_cmos_sensor_8(0x558a, 0x03);
	write_cmos_sensor_8(0x558b, 0x06);
	write_cmos_sensor_8(0x558c, 0x0b);
	write_cmos_sensor_8(0x558d, 0x10);
	write_cmos_sensor_8(0x558e, 0x16);
	write_cmos_sensor_8(0x558f, 0x1e);
	write_cmos_sensor_8(0x5590, 0x1d);
	write_cmos_sensor_8(0x5591, 0x16);
	write_cmos_sensor_8(0x5592, 0x11);
	write_cmos_sensor_8(0x5593, 0x0c);
	write_cmos_sensor_8(0x5594, 0x08);
	write_cmos_sensor_8(0x5595, 0x05);
	write_cmos_sensor_8(0x5598, 0x02);
	write_cmos_sensor_8(0x5599, 0x03);
	write_cmos_sensor_8(0x559a, 0x04);
	write_cmos_sensor_8(0x559b, 0x08);
	write_cmos_sensor_8(0x559c, 0x0c);
	write_cmos_sensor_8(0x559d, 0x12);
	write_cmos_sensor_8(0x559e, 0x18);
	write_cmos_sensor_8(0x559f, 0x1f);
	write_cmos_sensor_8(0x55a0, 0x20);
	write_cmos_sensor_8(0x55a1, 0x19);
	write_cmos_sensor_8(0x55a2, 0x14);
	write_cmos_sensor_8(0x55a3, 0x0f);
	write_cmos_sensor_8(0x55a4, 0x0b);
	write_cmos_sensor_8(0x55a5, 0x08);
	write_cmos_sensor_8(0x55a6, 0x06);
	write_cmos_sensor_8(0x55a7, 0x05);
	write_cmos_sensor_8(0x55a8, 0x06);
	write_cmos_sensor_8(0x55a9, 0x06);
	write_cmos_sensor_8(0x55aa, 0x08);
	write_cmos_sensor_8(0x55ab, 0x0b);
	write_cmos_sensor_8(0x55ac, 0x0f);
	write_cmos_sensor_8(0x55ad, 0x14);
	write_cmos_sensor_8(0x55ae, 0x1b);
	write_cmos_sensor_8(0x55af, 0x22);
	write_cmos_sensor_8(0x55b0, 0x24);
	write_cmos_sensor_8(0x55b1, 0x1e);
	write_cmos_sensor_8(0x55b2, 0x19);
	write_cmos_sensor_8(0x55b3, 0x14);
	write_cmos_sensor_8(0x55b4, 0x10);
	write_cmos_sensor_8(0x55b5, 0x0e);
	write_cmos_sensor_8(0x55b6, 0x0c);
	write_cmos_sensor_8(0x55b7, 0x0b);
	write_cmos_sensor_8(0x55b8, 0x0b);
	write_cmos_sensor_8(0x55b9, 0x0c);
	write_cmos_sensor_8(0x55ba, 0x0e);
	write_cmos_sensor_8(0x55bb, 0x11);
	write_cmos_sensor_8(0x55bc, 0x14);
	write_cmos_sensor_8(0x55bd, 0x19);
	write_cmos_sensor_8(0x55be, 0x1f);
	write_cmos_sensor_8(0x55bf, 0x27);
	write_cmos_sensor_8(0x55c0, 0x29);
	write_cmos_sensor_8(0x55c1, 0x23);
	write_cmos_sensor_8(0x55c2, 0x1f);
	write_cmos_sensor_8(0x55c3, 0x1a);
	write_cmos_sensor_8(0x55c4, 0x17);
	write_cmos_sensor_8(0x55c5, 0x14);
	write_cmos_sensor_8(0x55c6, 0x13);
	write_cmos_sensor_8(0x55c7, 0x12);
	write_cmos_sensor_8(0x55c8, 0x12);
	write_cmos_sensor_8(0x55c9, 0x13);
	write_cmos_sensor_8(0x55ca, 0x14);
	write_cmos_sensor_8(0x55cb, 0x17);
	write_cmos_sensor_8(0x55cc, 0x1a);
	write_cmos_sensor_8(0x55cd, 0x1f);
	write_cmos_sensor_8(0x55ce, 0x25);
	write_cmos_sensor_8(0x55cf, 0x2d);
	write_cmos_sensor_8(0x55d0, 0x31);
	write_cmos_sensor_8(0x55d1, 0x2a);
	write_cmos_sensor_8(0x55d2, 0x25);
	write_cmos_sensor_8(0x55d3, 0x22);
	write_cmos_sensor_8(0x55d4, 0x1f);
	write_cmos_sensor_8(0x55d5, 0x1c);
	write_cmos_sensor_8(0x55d6, 0x1b);
	write_cmos_sensor_8(0x55d7, 0x1a);
	write_cmos_sensor_8(0x55d8, 0x1a);
	write_cmos_sensor_8(0x55d9, 0x1a);
	write_cmos_sensor_8(0x55da, 0x1c);
	write_cmos_sensor_8(0x55db, 0x1f);
	write_cmos_sensor_8(0x55dc, 0x22);
	write_cmos_sensor_8(0x55dd, 0x27);
	write_cmos_sensor_8(0x55de, 0x2c);
	write_cmos_sensor_8(0x55df, 0x34);
	write_cmos_sensor_8(0x55e0, 0x3b);
	write_cmos_sensor_8(0x55e1, 0x32);
	write_cmos_sensor_8(0x55e2, 0x2e);
	write_cmos_sensor_8(0x55e3, 0x2a);
	write_cmos_sensor_8(0x55e4, 0x27);
	write_cmos_sensor_8(0x55e5, 0x25);
	write_cmos_sensor_8(0x55e6, 0x23);
	write_cmos_sensor_8(0x55e7, 0x23);
	write_cmos_sensor_8(0x55e8, 0x23);
	write_cmos_sensor_8(0x55e9, 0x24);
	write_cmos_sensor_8(0x55ea, 0x25);
	write_cmos_sensor_8(0x55eb, 0x28);
	write_cmos_sensor_8(0x55ec, 0x2b);
	write_cmos_sensor_8(0x55ed, 0x30);
	write_cmos_sensor_8(0x55ee, 0x35);
	write_cmos_sensor_8(0x55ef, 0x3f);
	write_cmos_sensor_8(0x55f0, 0x4b);
	write_cmos_sensor_8(0x55f1, 0x41);
	write_cmos_sensor_8(0x55f2, 0x3a);
	write_cmos_sensor_8(0x55f3, 0x36);
	write_cmos_sensor_8(0x55f4, 0x32);
	write_cmos_sensor_8(0x55f5, 0x30);
	write_cmos_sensor_8(0x55f6, 0x2f);
	write_cmos_sensor_8(0x55f7, 0x2d);
	write_cmos_sensor_8(0x55f8, 0x2f);
	write_cmos_sensor_8(0x55f9, 0x2f);
	write_cmos_sensor_8(0x55fa, 0x30);
	write_cmos_sensor_8(0x55fb, 0x33);
	write_cmos_sensor_8(0x55fc, 0x37);
	write_cmos_sensor_8(0x55fd, 0x3d);
	write_cmos_sensor_8(0x55fe, 0x44);
	write_cmos_sensor_8(0x55ff, 0x4e);
	write_cmos_sensor_8(0x5600, 0x80);
	write_cmos_sensor_8(0x5601, 0x80);
	write_cmos_sensor_8(0x5602, 0x80);
	write_cmos_sensor_8(0x5603, 0x7f);
	write_cmos_sensor_8(0x5605, 0x7f);
	write_cmos_sensor_8(0x5606, 0x7e);
	write_cmos_sensor_8(0x5607, 0x82);
	write_cmos_sensor_8(0x5608, 0x7e);
	write_cmos_sensor_8(0x5609, 0x80);
	write_cmos_sensor_8(0x560a, 0x80);
	write_cmos_sensor_8(0x560b, 0x7e);
	write_cmos_sensor_8(0x560c, 0x80);
	write_cmos_sensor_8(0x560e, 0x81);
	write_cmos_sensor_8(0x560f, 0x7a);
	write_cmos_sensor_8(0x5610, 0x80);
	write_cmos_sensor_8(0x5611, 0x80);
	write_cmos_sensor_8(0x5612, 0x80);
	write_cmos_sensor_8(0x5613, 0x80);
	write_cmos_sensor_8(0x5614, 0x7f);
	write_cmos_sensor_8(0x5615, 0x7f);
	write_cmos_sensor_8(0x5616, 0x80);
	write_cmos_sensor_8(0x5617, 0x7f);
	write_cmos_sensor_8(0x5618, 0x80);
	write_cmos_sensor_8(0x5619, 0x7f);
	write_cmos_sensor_8(0x561a, 0x7f);
	write_cmos_sensor_8(0x561b, 0x80);
	write_cmos_sensor_8(0x561c, 0x7f);
	write_cmos_sensor_8(0x561d, 0x7f);
	write_cmos_sensor_8(0x561e, 0x7f);
	write_cmos_sensor_8(0x561f, 0x82);
	write_cmos_sensor_8(0x5621, 0x80);
	write_cmos_sensor_8(0x5622, 0x7f);
	write_cmos_sensor_8(0x5623, 0x7f);
	write_cmos_sensor_8(0x5624, 0x7f);
	write_cmos_sensor_8(0x5625, 0x7f);
	write_cmos_sensor_8(0x5626, 0x7f);
	write_cmos_sensor_8(0x5627, 0x7f);
	write_cmos_sensor_8(0x5628, 0x7f);
	write_cmos_sensor_8(0x5629, 0x7f);
	write_cmos_sensor_8(0x562a, 0x7f);
	write_cmos_sensor_8(0x562b, 0x7f);
	write_cmos_sensor_8(0x562c, 0x7f);
	write_cmos_sensor_8(0x562d, 0x80);
	write_cmos_sensor_8(0x562e, 0x7f);
	write_cmos_sensor_8(0x562f, 0x81);
	write_cmos_sensor_8(0x5630, 0x80);
	write_cmos_sensor_8(0x5631, 0x7f);
	write_cmos_sensor_8(0x5632, 0x7f);
	write_cmos_sensor_8(0x5633, 0x7f);
	write_cmos_sensor_8(0x5634, 0x7f);
	write_cmos_sensor_8(0x5635, 0x7f);
	write_cmos_sensor_8(0x5636, 0x7f);
	write_cmos_sensor_8(0x5637, 0x7f);
	write_cmos_sensor_8(0x5638, 0x7f);
	write_cmos_sensor_8(0x5639, 0x7f);
	write_cmos_sensor_8(0x563a, 0x7f);
	write_cmos_sensor_8(0x563b, 0x7f);
	write_cmos_sensor_8(0x563c, 0x7f);
	write_cmos_sensor_8(0x563d, 0x7f);
	write_cmos_sensor_8(0x563e, 0x80);
	write_cmos_sensor_8(0x563f, 0x80);
	write_cmos_sensor_8(0x5640, 0x81);
	write_cmos_sensor_8(0x5641, 0x80);
	write_cmos_sensor_8(0x5643, 0x7f);
	write_cmos_sensor_8(0x5644, 0x7f);
	write_cmos_sensor_8(0x5645, 0x7f);
	write_cmos_sensor_8(0x5646, 0x7f);
	write_cmos_sensor_8(0x5647, 0x7f);
	write_cmos_sensor_8(0x5648, 0x7f);
	write_cmos_sensor_8(0x5649, 0x7f);
	write_cmos_sensor_8(0x564a, 0x7f);
	write_cmos_sensor_8(0x564b, 0x7f);
	write_cmos_sensor_8(0x564c, 0x7f);
	write_cmos_sensor_8(0x564d, 0x7f);
	write_cmos_sensor_8(0x564e, 0x80);
	write_cmos_sensor_8(0x564f, 0x7f);
	write_cmos_sensor_8(0x5650, 0x80);
	write_cmos_sensor_8(0x5652, 0x80);
	write_cmos_sensor_8(0x5653, 0x7f);
	write_cmos_sensor_8(0x5654, 0x7f);
	write_cmos_sensor_8(0x5655, 0x7f);
	write_cmos_sensor_8(0x5656, 0x7f);
	write_cmos_sensor_8(0x5657, 0x7f);
	write_cmos_sensor_8(0x5658, 0x7f);
	write_cmos_sensor_8(0x5659, 0x7f);
	write_cmos_sensor_8(0x565a, 0x7f);
	write_cmos_sensor_8(0x565b, 0x7f);
	write_cmos_sensor_8(0x565c, 0x7f);
	write_cmos_sensor_8(0x565d, 0x7f);
	write_cmos_sensor_8(0x565f, 0x7f);
	write_cmos_sensor_8(0x5660, 0x80);
	write_cmos_sensor_8(0x5661, 0x80);
	write_cmos_sensor_8(0x5663, 0x7f);
	write_cmos_sensor_8(0x5666, 0x7f);
	write_cmos_sensor_8(0x5667, 0x7f);
	write_cmos_sensor_8(0x5668, 0x7f);
	write_cmos_sensor_8(0x5669, 0x7f);
	write_cmos_sensor_8(0x566c, 0x80);
	write_cmos_sensor_8(0x566d, 0x80);
	write_cmos_sensor_8(0x566e, 0x7f);
	write_cmos_sensor_8(0x566f, 0x80);
	write_cmos_sensor_8(0x5671, 0x80);
	write_cmos_sensor_8(0x5672, 0x80);
	write_cmos_sensor_8(0x5673, 0x80);
	write_cmos_sensor_8(0x5676, 0x80);
	write_cmos_sensor_8(0x5677, 0x80);
	write_cmos_sensor_8(0x5678, 0x80);
	write_cmos_sensor_8(0x5679, 0x80);
	write_cmos_sensor_8(0x567a, 0x80);
	write_cmos_sensor_8(0x567b, 0x80);
	write_cmos_sensor_8(0x567c, 0x80);
	write_cmos_sensor_8(0x567d, 0x80);
	write_cmos_sensor_8(0x567e, 0x80);
	write_cmos_sensor_8(0x567f, 0x80);
	write_cmos_sensor_8(0x5680, 0x80);
	write_cmos_sensor_8(0x5681, 0x80);
	write_cmos_sensor_8(0x5682, 0x80);
	write_cmos_sensor_8(0x5684, 0x81);
	write_cmos_sensor_8(0x5685, 0x80);
	write_cmos_sensor_8(0x5686, 0x80);
	write_cmos_sensor_8(0x5687, 0x80);
	write_cmos_sensor_8(0x5688, 0x80);
	write_cmos_sensor_8(0x5689, 0x80);
	write_cmos_sensor_8(0x568a, 0x80);
	write_cmos_sensor_8(0x568b, 0x80);
	write_cmos_sensor_8(0x568c, 0x80);
	write_cmos_sensor_8(0x568d, 0x80);
	write_cmos_sensor_8(0x568e, 0x80);
	write_cmos_sensor_8(0x5691, 0x80);
	write_cmos_sensor_8(0x5692, 0x80);
	write_cmos_sensor_8(0x5695, 0x7f);
	write_cmos_sensor_8(0x5696, 0x80);
	write_cmos_sensor_8(0x5697, 0x80);
	write_cmos_sensor_8(0x5698, 0x80);
	write_cmos_sensor_8(0x5699, 0x80);
	write_cmos_sensor_8(0x569a, 0x80);
	write_cmos_sensor_8(0x569b, 0x80);
	write_cmos_sensor_8(0x569c, 0x80);
	write_cmos_sensor_8(0x569d, 0x80);
	write_cmos_sensor_8(0x569f, 0x81);
	write_cmos_sensor_8(0x56a1, 0x80);
	write_cmos_sensor_8(0x56a6, 0x7f);
	write_cmos_sensor_8(0x56a7, 0x7f);
	write_cmos_sensor_8(0x56a8, 0x7f);
	write_cmos_sensor_8(0x56a9, 0x7f);
	write_cmos_sensor_8(0x56aa, 0x80);
	write_cmos_sensor_8(0x56ab, 0x80);
	write_cmos_sensor_8(0x56ad, 0x80);
	write_cmos_sensor_8(0x56af, 0x80);
	write_cmos_sensor_8(0x56b5, 0x7f);
	write_cmos_sensor_8(0x56b6, 0x7f);
	write_cmos_sensor_8(0x56b7, 0x7f);
	write_cmos_sensor_8(0x56b8, 0x7f);
	write_cmos_sensor_8(0x56b9, 0x7f);
	write_cmos_sensor_8(0x56bb, 0x7f);
	write_cmos_sensor_8(0x56be, 0x80);
	write_cmos_sensor_8(0x56bf, 0x80);
	write_cmos_sensor_8(0x56c0, 0x80);
	write_cmos_sensor_8(0x56c3, 0x80);
	write_cmos_sensor_8(0x56c6, 0x7f);
	write_cmos_sensor_8(0x56c8, 0x7f);
	write_cmos_sensor_8(0x56c9, 0x7f);
	write_cmos_sensor_8(0x56ca, 0x7f);
	write_cmos_sensor_8(0x56cb, 0x80);
	write_cmos_sensor_8(0x56cc, 0x80);
	write_cmos_sensor_8(0x56d0, 0x81);
	write_cmos_sensor_8(0x56d1, 0x80);
	write_cmos_sensor_8(0x56d3, 0x80);
	write_cmos_sensor_8(0x56d4, 0x7f);
	write_cmos_sensor_8(0x56d5, 0x80);
	write_cmos_sensor_8(0x56d6, 0x7f);
	write_cmos_sensor_8(0x56d8, 0x80);
	write_cmos_sensor_8(0x56da, 0x80);
	write_cmos_sensor_8(0x56db, 0x7f);
	write_cmos_sensor_8(0x56dd, 0x80);
	write_cmos_sensor_8(0x56df, 0x81);
	write_cmos_sensor_8(0x56e1, 0x7f);
	write_cmos_sensor_8(0x56e2, 0x80);
	write_cmos_sensor_8(0x56e3, 0x7f);
	write_cmos_sensor_8(0x56e4, 0x7f);
	write_cmos_sensor_8(0x56e5, 0x7f);
	write_cmos_sensor_8(0x56e7, 0x80);
	write_cmos_sensor_8(0x56e8, 0x7f);
	write_cmos_sensor_8(0x56e9, 0x80);
	write_cmos_sensor_8(0x56ea, 0x7f);
	write_cmos_sensor_8(0x56eb, 0x80);
	write_cmos_sensor_8(0x56ec, 0x80);
	write_cmos_sensor_8(0x56ee, 0x80);
	write_cmos_sensor_8(0x56ef, 0x7f);
	write_cmos_sensor_8(0x56f0, 0x7b);
	write_cmos_sensor_8(0x56f1, 0x81);
	write_cmos_sensor_8(0x56f2, 0x7f);
	write_cmos_sensor_8(0x56f3, 0x80);
	write_cmos_sensor_8(0x56f5, 0x7f);
	write_cmos_sensor_8(0x56f6, 0x7f);
	write_cmos_sensor_8(0x56f7, 0x80);
	write_cmos_sensor_8(0x56f9, 0x7f);
	write_cmos_sensor_8(0x56fc, 0x7f);
	write_cmos_sensor_8(0x56fd, 0x81);
	write_cmos_sensor_8(0x56fe, 0x80);
	write_cmos_sensor_8(0x56ff, 0x7f);
	write_cmos_sensor_8(0x5700, 0x84);
	write_cmos_sensor_8(0x5701, 0x7e);
	write_cmos_sensor_8(0x5702, 0x81);
	write_cmos_sensor_8(0x5703, 0x7d);
	write_cmos_sensor_8(0x5704, 0x80);
	write_cmos_sensor_8(0x5705, 0x7f);
	write_cmos_sensor_8(0x5706, 0x7f);
	write_cmos_sensor_8(0x5707, 0x81);
	write_cmos_sensor_8(0x5708, 0x7e);
	write_cmos_sensor_8(0x5709, 0x7e);
	write_cmos_sensor_8(0x570a, 0x81);
	write_cmos_sensor_8(0x570b, 0x7f);
	write_cmos_sensor_8(0x570c, 0x80);
	write_cmos_sensor_8(0x570d, 0x7f);
	write_cmos_sensor_8(0x570e, 0x7f);
	write_cmos_sensor_8(0x570f, 0x81);
	write_cmos_sensor_8(0x5710, 0x7d);
	write_cmos_sensor_8(0x5711, 0x80);
	write_cmos_sensor_8(0x5712, 0x7e);
	write_cmos_sensor_8(0x5713, 0x80);
	write_cmos_sensor_8(0x5714, 0x7f);
	write_cmos_sensor_8(0x5715, 0x7f);
	write_cmos_sensor_8(0x5716, 0x7f);
	write_cmos_sensor_8(0x5717, 0x7f);
	write_cmos_sensor_8(0x5718, 0x80);
	write_cmos_sensor_8(0x5719, 0x80);
	write_cmos_sensor_8(0x571a, 0x7f);
	write_cmos_sensor_8(0x571b, 0x80);
	write_cmos_sensor_8(0x571c, 0x7f);
	write_cmos_sensor_8(0x571d, 0x80);
	write_cmos_sensor_8(0x571e, 0x7f);
	write_cmos_sensor_8(0x571f, 0x81);
	write_cmos_sensor_8(0x5720, 0x80);
	write_cmos_sensor_8(0x5721, 0x7f);
	write_cmos_sensor_8(0x5722, 0x80);
	write_cmos_sensor_8(0x5723, 0x7f);
	write_cmos_sensor_8(0x5724, 0x7f);
	write_cmos_sensor_8(0x5725, 0x7f);
	write_cmos_sensor_8(0x5726, 0x7f);
	write_cmos_sensor_8(0x5727, 0x7f);
	write_cmos_sensor_8(0x5728, 0x7f);
	write_cmos_sensor_8(0x5729, 0x7f);
	write_cmos_sensor_8(0x572a, 0x7f);
	write_cmos_sensor_8(0x572b, 0x7f);
	write_cmos_sensor_8(0x572c, 0x7f);
	write_cmos_sensor_8(0x572d, 0x7f);
	write_cmos_sensor_8(0x572e, 0x7f);
	write_cmos_sensor_8(0x572f, 0x81);
	write_cmos_sensor_8(0x5730, 0x7f);
	write_cmos_sensor_8(0x5731, 0x7f);
	write_cmos_sensor_8(0x5732, 0x7f);
	write_cmos_sensor_8(0x5733, 0x7f);
	write_cmos_sensor_8(0x5734, 0x7f);
	write_cmos_sensor_8(0x5735, 0x7f);
	write_cmos_sensor_8(0x5736, 0x7f);
	write_cmos_sensor_8(0x5737, 0x7f);
	write_cmos_sensor_8(0x5738, 0x7f);
	write_cmos_sensor_8(0x5739, 0x7f);
	write_cmos_sensor_8(0x573a, 0x7f);
	write_cmos_sensor_8(0x573b, 0x7f);
	write_cmos_sensor_8(0x573c, 0x7f);
	write_cmos_sensor_8(0x573d, 0x7f);
	write_cmos_sensor_8(0x573e, 0x7f);
	write_cmos_sensor_8(0x573f, 0x80);
	write_cmos_sensor_8(0x5740, 0x80);
	write_cmos_sensor_8(0x5741, 0x7f);
	write_cmos_sensor_8(0x5742, 0x80);
	write_cmos_sensor_8(0x5743, 0x7f);
	write_cmos_sensor_8(0x5744, 0x7f);
	write_cmos_sensor_8(0x5745, 0x7f);
	write_cmos_sensor_8(0x5746, 0x7f);
	write_cmos_sensor_8(0x5747, 0x7f);
	write_cmos_sensor_8(0x5748, 0x7f);
	write_cmos_sensor_8(0x5749, 0x7f);
	write_cmos_sensor_8(0x574a, 0x7f);
	write_cmos_sensor_8(0x574b, 0x7f);
	write_cmos_sensor_8(0x574c, 0x7f);
	write_cmos_sensor_8(0x574d, 0x7f);
	write_cmos_sensor_8(0x574e, 0x7f);
	write_cmos_sensor_8(0x574f, 0x80);
	write_cmos_sensor_8(0x5750, 0x80);
	write_cmos_sensor_8(0x5751, 0x7f);
	write_cmos_sensor_8(0x5752, 0x7f);
	write_cmos_sensor_8(0x5753, 0x7f);
	write_cmos_sensor_8(0x5754, 0x7f);
	write_cmos_sensor_8(0x5755, 0x7f);
	write_cmos_sensor_8(0x5756, 0x7f);
	write_cmos_sensor_8(0x5757, 0x7f);
	write_cmos_sensor_8(0x5758, 0x7f);
	write_cmos_sensor_8(0x5759, 0x7f);
	write_cmos_sensor_8(0x575a, 0x7f);
	write_cmos_sensor_8(0x575b, 0x7f);
	write_cmos_sensor_8(0x575c, 0x7f);
	write_cmos_sensor_8(0x575d, 0x7f);
	write_cmos_sensor_8(0x575e, 0x7f);
	write_cmos_sensor_8(0x575f, 0x80);
	write_cmos_sensor_8(0x5760, 0x7e);
	write_cmos_sensor_8(0x5761, 0x80);
	write_cmos_sensor_8(0x5762, 0x7f);
	write_cmos_sensor_8(0x5763, 0x7f);
	write_cmos_sensor_8(0x5764, 0x7f);
	write_cmos_sensor_8(0x5766, 0x7f);
	write_cmos_sensor_8(0x5767, 0x7f);
	write_cmos_sensor_8(0x5768, 0x80);
	write_cmos_sensor_8(0x5769, 0x7f);
	write_cmos_sensor_8(0x576a, 0x80);
	write_cmos_sensor_8(0x576b, 0x7f);
	write_cmos_sensor_8(0x576c, 0x80);
	write_cmos_sensor_8(0x576d, 0x80);
	write_cmos_sensor_8(0x576e, 0x7f);
	write_cmos_sensor_8(0x576f, 0x80);
	write_cmos_sensor_8(0x5770, 0x80);
	write_cmos_sensor_8(0x5771, 0x80);
	write_cmos_sensor_8(0x5772, 0x80);
	write_cmos_sensor_8(0x5774, 0x80);
	write_cmos_sensor_8(0x5775, 0x80);
	write_cmos_sensor_8(0x5776, 0x80);
	write_cmos_sensor_8(0x5778, 0x80);
	write_cmos_sensor_8(0x5779, 0x80);
	write_cmos_sensor_8(0x577a, 0x80);
	write_cmos_sensor_8(0x577b, 0x80);
	write_cmos_sensor_8(0x577c, 0x7f);
	write_cmos_sensor_8(0x577d, 0x80);
	write_cmos_sensor_8(0x577e, 0x80);
	write_cmos_sensor_8(0x577f, 0x80);
	write_cmos_sensor_8(0x5780, 0x80);
	write_cmos_sensor_8(0x5781, 0x80);
	write_cmos_sensor_8(0x5782, 0x80);
	write_cmos_sensor_8(0x5784, 0x80);
	write_cmos_sensor_8(0x5785, 0x80);
	write_cmos_sensor_8(0x5786, 0x80);
	write_cmos_sensor_8(0x5788, 0x80);
	write_cmos_sensor_8(0x5789, 0x81);
	write_cmos_sensor_8(0x578a, 0x80);
	write_cmos_sensor_8(0x578b, 0x80);
	write_cmos_sensor_8(0x578c, 0x80);
	write_cmos_sensor_8(0x578d, 0x80);
	write_cmos_sensor_8(0x578e, 0x80);
	write_cmos_sensor_8(0x578f, 0x80);
	write_cmos_sensor_8(0x5790, 0x7f);
	write_cmos_sensor_8(0x5791, 0x80);
	write_cmos_sensor_8(0x5792, 0x80);
	write_cmos_sensor_8(0x5795, 0x80);
	write_cmos_sensor_8(0x5798, 0x80);
	write_cmos_sensor_8(0x5799, 0x7f);
	write_cmos_sensor_8(0x579a, 0x80);
	write_cmos_sensor_8(0x579b, 0x80);
	write_cmos_sensor_8(0x579c, 0x80);
	write_cmos_sensor_8(0x579d, 0x80);
	write_cmos_sensor_8(0x579e, 0x80);
	write_cmos_sensor_8(0x579f, 0x80);
	write_cmos_sensor_8(0x57a0, 0x7f);
	write_cmos_sensor_8(0x57a1, 0x80);
	write_cmos_sensor_8(0x57a2, 0x80);
	write_cmos_sensor_8(0x57a3, 0x7f);
	write_cmos_sensor_8(0x57a4, 0x80);
	write_cmos_sensor_8(0x57a5, 0x7f);
	write_cmos_sensor_8(0x57a6, 0x80);
	write_cmos_sensor_8(0x57a7, 0x7f);
	write_cmos_sensor_8(0x57a8, 0x7f);
	write_cmos_sensor_8(0x57a9, 0x7f);
	write_cmos_sensor_8(0x57aa, 0x80);
	write_cmos_sensor_8(0x57ab, 0x7f);
	write_cmos_sensor_8(0x57ac, 0x7f);
	write_cmos_sensor_8(0x57ad, 0x80);
	write_cmos_sensor_8(0x57ae, 0x80);
	write_cmos_sensor_8(0x57af, 0x81);
	write_cmos_sensor_8(0x57b0, 0x80);
	write_cmos_sensor_8(0x57b1, 0x7f);
	write_cmos_sensor_8(0x57b2, 0x7f);
	write_cmos_sensor_8(0x57b3, 0x7f);
	write_cmos_sensor_8(0x57b4, 0x7f);
	write_cmos_sensor_8(0x57b5, 0x7f);
	write_cmos_sensor_8(0x57b6, 0x7f);
	write_cmos_sensor_8(0x57b7, 0x7f);
	write_cmos_sensor_8(0x57b8, 0x7f);
	write_cmos_sensor_8(0x57b9, 0x7f);
	write_cmos_sensor_8(0x57ba, 0x7f);
	write_cmos_sensor_8(0x57bb, 0x7f);
	write_cmos_sensor_8(0x57bc, 0x80);
	write_cmos_sensor_8(0x57bd, 0x7f);
	write_cmos_sensor_8(0x57be, 0x80);
	write_cmos_sensor_8(0x57bf, 0x7f);
	write_cmos_sensor_8(0x57c0, 0x7f);
	write_cmos_sensor_8(0x57c1, 0x7f);
	write_cmos_sensor_8(0x57c2, 0x7f);
	write_cmos_sensor_8(0x57c3, 0x7f);
	write_cmos_sensor_8(0x57c4, 0x7f);
	write_cmos_sensor_8(0x57c5, 0x7f);
	write_cmos_sensor_8(0x57c6, 0x80);
	write_cmos_sensor_8(0x57c7, 0x7f);
	write_cmos_sensor_8(0x57c8, 0x80);
	write_cmos_sensor_8(0x57c9, 0x7f);
	write_cmos_sensor_8(0x57ca, 0x7f);
	write_cmos_sensor_8(0x57cb, 0x7f);
	write_cmos_sensor_8(0x57cc, 0x80);
	write_cmos_sensor_8(0x57cd, 0x80);
	write_cmos_sensor_8(0x57ce, 0x7f);
	write_cmos_sensor_8(0x57cf, 0x81);
	write_cmos_sensor_8(0x57d0, 0x80);
	write_cmos_sensor_8(0x57d1, 0x80);
	write_cmos_sensor_8(0x57d2, 0x80);
	write_cmos_sensor_8(0x57d3, 0x7f);
	write_cmos_sensor_8(0x57d4, 0x7f);
	write_cmos_sensor_8(0x57d5, 0x80);
	write_cmos_sensor_8(0x57d6, 0x7f);
	write_cmos_sensor_8(0x57d7, 0x7f);
	write_cmos_sensor_8(0x57d8, 0x7f);
	write_cmos_sensor_8(0x57d9, 0x7f);
	write_cmos_sensor_8(0x57da, 0x7f);
	write_cmos_sensor_8(0x57db, 0x80);
	write_cmos_sensor_8(0x57dc, 0x7f);
	write_cmos_sensor_8(0x57dd, 0x80);
	write_cmos_sensor_8(0x57de, 0x80);
	write_cmos_sensor_8(0x57df, 0x80);
	write_cmos_sensor_8(0x57e0, 0x7f);
	write_cmos_sensor_8(0x57e1, 0x7f);
	write_cmos_sensor_8(0x57e2, 0x80);
	write_cmos_sensor_8(0x57e3, 0x7f);
	write_cmos_sensor_8(0x57e4, 0x80);
	write_cmos_sensor_8(0x57e5, 0x7f);
	write_cmos_sensor_8(0x57e6, 0x80);
	write_cmos_sensor_8(0x57e7, 0x7f);
	write_cmos_sensor_8(0x57e8, 0x80);
	write_cmos_sensor_8(0x57e9, 0x7f);
	write_cmos_sensor_8(0x57ea, 0x80);
	write_cmos_sensor_8(0x57eb, 0x80);
	write_cmos_sensor_8(0x57ec, 0x80);
	write_cmos_sensor_8(0x57ed, 0x7f);
	write_cmos_sensor_8(0x57ee, 0x80);
	write_cmos_sensor_8(0x57ef, 0x80);
	write_cmos_sensor_8(0x57f0, 0x7f);
	write_cmos_sensor_8(0x57f1, 0x80);
	write_cmos_sensor_8(0x57f2, 0x7e);
	write_cmos_sensor_8(0x57f3, 0x80);
	write_cmos_sensor_8(0x57f4, 0x7e);
	write_cmos_sensor_8(0x57f5, 0x81);
	write_cmos_sensor_8(0x57f6, 0x7e);
	write_cmos_sensor_8(0x57f7, 0x81);
	write_cmos_sensor_8(0x57f8, 0x7e);
	write_cmos_sensor_8(0x57f9, 0x80);
	write_cmos_sensor_8(0x57fa, 0x80);
	write_cmos_sensor_8(0x57fb, 0x80);
	write_cmos_sensor_8(0x57fc, 0x7e);
	write_cmos_sensor_8(0x57fd, 0x81);
	write_cmos_sensor_8(0x57fe, 0x80);
	write_cmos_sensor_8(0x57ff, 0x7a);
	//write_cmos_sensor_8(0x0100, 0x01);
	}

}

#if 0
static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n",currefps);

	normal_capture_setting();

}
#endif

static void hs_video_setting(void)
{
	// @@1280x720_60fps_bin
	// Sysclk 160Mhz, MIPI4_768Mbps/Lane, 30Fps.
	// Line_length =1400, Frame_length =1904.  V-blanking=1.33ms
	
	//write_cmos_sensor_8(0x0100, 0x00);
	write_cmos_sensor_8(0x0301, 0xa5);
	write_cmos_sensor_8(0x0304, 0x29);//0x28  /*prize  add  for k6307  RF intrusion by zhuzhengjiang    20190925-begin*/
	write_cmos_sensor_8(0x0316, 0xa0);
	write_cmos_sensor_8(0x3500, 0x00);
	write_cmos_sensor_8(0x3501, 0x75);
	write_cmos_sensor_8(0x3502, 0x00);
	write_cmos_sensor_8(0x3511, 0x75);
	write_cmos_sensor_8(0x3600, 0x40);
	write_cmos_sensor_8(0x3602, 0x84);
	write_cmos_sensor_8(0x3621, 0x66);
	write_cmos_sensor_8(0x3701, 0x14);
	write_cmos_sensor_8(0x3709, 0x38);
	write_cmos_sensor_8(0x37f4, 0x80);
	write_cmos_sensor_8(0x3803, 0x02);
	write_cmos_sensor_8(0x3807, 0xd1);
	write_cmos_sensor_8(0x3808, 0x05);
	write_cmos_sensor_8(0x3809, 0x00);
	write_cmos_sensor_8(0x380a, 0x02);
	write_cmos_sensor_8(0x380b, 0xd0);
	write_cmos_sensor_8(0x380c, 0x05);
	write_cmos_sensor_8(0x380d, 0x78);
	write_cmos_sensor_8(0x380e, 0x07);
	write_cmos_sensor_8(0x380f, 0x70);
	write_cmos_sensor_8(0x3810, 0x02);
	write_cmos_sensor_8(0x3811, 0x19);
	write_cmos_sensor_8(0x3812, 0x02);
	write_cmos_sensor_8(0x3813, 0x09);
	write_cmos_sensor_8(0x3815, 0x22);
	write_cmos_sensor_8(0x3820, 0x01);
	write_cmos_sensor_8(0x3834, 0xf5);
	write_cmos_sensor_8(0x3f03, 0x1a);
	write_cmos_sensor_8(0x4010, 0x29);
	write_cmos_sensor_8(0x4012, 0x0d);
	write_cmos_sensor_8(0x4013, 0x16);
	write_cmos_sensor_8(0x4014, 0x0a);
	write_cmos_sensor_8(0x4016, 0x15);
	write_cmos_sensor_8(0x4018, 0x09);
	write_cmos_sensor_8(0x4500, 0x25);
	write_cmos_sensor_8(0x4501, 0x04);
	write_cmos_sensor_8(0x4503, 0x10);
	write_cmos_sensor_8(0x450f, 0x88);
	write_cmos_sensor_8(0x4540, 0x86);
	write_cmos_sensor_8(0x4541, 0x07);
	write_cmos_sensor_8(0x4542, 0x04);
	write_cmos_sensor_8(0x4543, 0x05);
	write_cmos_sensor_8(0x4544, 0x06);
	write_cmos_sensor_8(0x4545, 0x07);
	write_cmos_sensor_8(0x4546, 0x04);
	write_cmos_sensor_8(0x4547, 0x05);
	write_cmos_sensor_8(0x4548, 0x02);
	write_cmos_sensor_8(0x4549, 0x03);
	write_cmos_sensor_8(0x454a, 0x00);
	write_cmos_sensor_8(0x454b, 0x01);
	write_cmos_sensor_8(0x454c, 0x02);
	write_cmos_sensor_8(0x454d, 0x03);
	write_cmos_sensor_8(0x454e, 0x00);
	write_cmos_sensor_8(0x454f, 0x01);
	write_cmos_sensor_8(0x4550, 0x06);
	write_cmos_sensor_8(0x4551, 0x07);
	write_cmos_sensor_8(0x4552, 0x04);
	write_cmos_sensor_8(0x4553, 0x05);
	write_cmos_sensor_8(0x4554, 0x06);
	write_cmos_sensor_8(0x4555, 0x07);
	write_cmos_sensor_8(0x4556, 0x04);
	write_cmos_sensor_8(0x4557, 0x05);
	write_cmos_sensor_8(0x4558, 0x02);
	write_cmos_sensor_8(0x4559, 0x03);
	write_cmos_sensor_8(0x455a, 0x00);
	write_cmos_sensor_8(0x455b, 0x01);
	write_cmos_sensor_8(0x455c, 0x02);
	write_cmos_sensor_8(0x455d, 0x03);
	write_cmos_sensor_8(0x455e, 0x00);
	write_cmos_sensor_8(0x455f, 0x01);
	write_cmos_sensor_8(0x4837, 0x14);
	write_cmos_sensor_8(0x5080, 0x04);
	write_cmos_sensor_8(0x5084, 0x09);
	write_cmos_sensor_8(0x5085, 0x30);
	write_cmos_sensor_8(0x5086, 0x06);
	write_cmos_sensor_8(0x5087, 0xe6);
	write_cmos_sensor_8(0x5000, 0x89);
	write_cmos_sensor_8(0x5001, 0x42);
	write_cmos_sensor_8(0x5004, 0x81);
	write_cmos_sensor_8(0x5184, 0x63);
	write_cmos_sensor_8(0x5185, 0x17);
	write_cmos_sensor_8(0x5192, 0x09);
	write_cmos_sensor_8(0x5193, 0x2f);
	write_cmos_sensor_8(0x5406, 0x06);
	write_cmos_sensor_8(0x5407, 0x18);
	write_cmos_sensor_8(0x5408, 0x08);
	write_cmos_sensor_8(0x5409, 0x1a);
	write_cmos_sensor_8(0x5410, 0x03);
	write_cmos_sensor_8(0x5500, 0x4a);
	write_cmos_sensor_8(0x5501, 0x43);
	write_cmos_sensor_8(0x5502, 0x3c);
	write_cmos_sensor_8(0x5503, 0x38);
	write_cmos_sensor_8(0x5504, 0x34);
	write_cmos_sensor_8(0x5505, 0x32);
	write_cmos_sensor_8(0x5506, 0x30);
	write_cmos_sensor_8(0x5507, 0x2f);
	write_cmos_sensor_8(0x5508, 0x31);
	write_cmos_sensor_8(0x5509, 0x30);
	write_cmos_sensor_8(0x550a, 0x31);
	write_cmos_sensor_8(0x550b, 0x34);
	write_cmos_sensor_8(0x550c, 0x38);
	write_cmos_sensor_8(0x550d, 0x3d);
	write_cmos_sensor_8(0x550e, 0x44);
	write_cmos_sensor_8(0x550f, 0x51);
	write_cmos_sensor_8(0x5510, 0x3f);
	write_cmos_sensor_8(0x5511, 0x35);
	write_cmos_sensor_8(0x5512, 0x30);
	write_cmos_sensor_8(0x5513, 0x2c);
	write_cmos_sensor_8(0x5514, 0x2a);
	write_cmos_sensor_8(0x5515, 0x27);
	write_cmos_sensor_8(0x5516, 0x26);
	write_cmos_sensor_8(0x5517, 0x26);
	write_cmos_sensor_8(0x5518, 0x25);
	write_cmos_sensor_8(0x5519, 0x26);
	write_cmos_sensor_8(0x551a, 0x28);
	write_cmos_sensor_8(0x551b, 0x2a);
	write_cmos_sensor_8(0x551c, 0x2d);
	write_cmos_sensor_8(0x551d, 0x31);
	write_cmos_sensor_8(0x551e, 0x37);
	write_cmos_sensor_8(0x551f, 0x40);
	write_cmos_sensor_8(0x5520, 0x32);
	write_cmos_sensor_8(0x5521, 0x2c);
	write_cmos_sensor_8(0x5522, 0x28);
	write_cmos_sensor_8(0x5523, 0x24);
	write_cmos_sensor_8(0x5524, 0x21);
	write_cmos_sensor_8(0x5525, 0x1e);
	write_cmos_sensor_8(0x5526, 0x1d);
	write_cmos_sensor_8(0x5527, 0x1c);
	write_cmos_sensor_8(0x5528, 0x1d);
	write_cmos_sensor_8(0x5529, 0x1d);
	write_cmos_sensor_8(0x552a, 0x1f);
	write_cmos_sensor_8(0x552b, 0x22);
	write_cmos_sensor_8(0x552c, 0x25);
	write_cmos_sensor_8(0x552d, 0x29);
	write_cmos_sensor_8(0x552e, 0x2e);
	write_cmos_sensor_8(0x552f, 0x36);
	write_cmos_sensor_8(0x5530, 0x2b);
	write_cmos_sensor_8(0x5531, 0x25);
	write_cmos_sensor_8(0x5532, 0x20);
	write_cmos_sensor_8(0x5533, 0x1c);
	write_cmos_sensor_8(0x5534, 0x19);
	write_cmos_sensor_8(0x5535, 0x16);
	write_cmos_sensor_8(0x5536, 0x15);
	write_cmos_sensor_8(0x5537, 0x14);
	write_cmos_sensor_8(0x5538, 0x14);
	write_cmos_sensor_8(0x5539, 0x15);
	write_cmos_sensor_8(0x553a, 0x17);
	write_cmos_sensor_8(0x553b, 0x19);
	write_cmos_sensor_8(0x553c, 0x1d);
	write_cmos_sensor_8(0x553d, 0x22);
	write_cmos_sensor_8(0x553e, 0x27);
	write_cmos_sensor_8(0x553f, 0x2f);
	write_cmos_sensor_8(0x5540, 0x25);
	write_cmos_sensor_8(0x5541, 0x1f);
	write_cmos_sensor_8(0x5542, 0x1a);
	write_cmos_sensor_8(0x5543, 0x16);
	write_cmos_sensor_8(0x5544, 0x12);
	write_cmos_sensor_8(0x5545, 0x0f);
	write_cmos_sensor_8(0x5546, 0x0d);
	write_cmos_sensor_8(0x5547, 0x0d);
	write_cmos_sensor_8(0x5548, 0x0d);
	write_cmos_sensor_8(0x5549, 0x0e);
	write_cmos_sensor_8(0x554a, 0x10);
	write_cmos_sensor_8(0x554b, 0x13);
	write_cmos_sensor_8(0x554c, 0x16);
	write_cmos_sensor_8(0x554d, 0x1b);
	write_cmos_sensor_8(0x554e, 0x21);
	write_cmos_sensor_8(0x554f, 0x29);
	write_cmos_sensor_8(0x5550, 0x20);
	write_cmos_sensor_8(0x5551, 0x1a);
	write_cmos_sensor_8(0x5552, 0x15);
	write_cmos_sensor_8(0x5553, 0x10);
	write_cmos_sensor_8(0x5554, 0x0c);
	write_cmos_sensor_8(0x5555, 0x09);
	write_cmos_sensor_8(0x5556, 0x07);
	write_cmos_sensor_8(0x5557, 0x07);
	write_cmos_sensor_8(0x5558, 0x06);
	write_cmos_sensor_8(0x5559, 0x07);
	write_cmos_sensor_8(0x555a, 0x09);
	write_cmos_sensor_8(0x555b, 0x0d);
	write_cmos_sensor_8(0x555c, 0x11);
	write_cmos_sensor_8(0x555d, 0x16);
	write_cmos_sensor_8(0x555e, 0x1c);
	write_cmos_sensor_8(0x555f, 0x24);
	write_cmos_sensor_8(0x5560, 0x1e);
	write_cmos_sensor_8(0x5561, 0x17);
	write_cmos_sensor_8(0x5562, 0x11);
	write_cmos_sensor_8(0x5563, 0x0d);
	write_cmos_sensor_8(0x5564, 0x08);
	write_cmos_sensor_8(0x5565, 0x05);
	write_cmos_sensor_8(0x5566, 0x03);
	write_cmos_sensor_8(0x5567, 0x02);
	write_cmos_sensor_8(0x5568, 0x02);
	write_cmos_sensor_8(0x5569, 0x03);
	write_cmos_sensor_8(0x556a, 0x05);
	write_cmos_sensor_8(0x556b, 0x09);
	write_cmos_sensor_8(0x556c, 0x0d);
	write_cmos_sensor_8(0x556d, 0x12);
	write_cmos_sensor_8(0x556e, 0x19);
	write_cmos_sensor_8(0x556f, 0x20);
	write_cmos_sensor_8(0x5570, 0x1c);
	write_cmos_sensor_8(0x5571, 0x15);
	write_cmos_sensor_8(0x5572, 0x0f);
	write_cmos_sensor_8(0x5573, 0x0a);
	write_cmos_sensor_8(0x5574, 0x06);
	write_cmos_sensor_8(0x5575, 0x03);
	write_cmos_sensor_8(0x5576, 0x01);
	write_cmos_sensor_8(0x5577, 0x00);
	write_cmos_sensor_8(0x5578, 0x00);
	write_cmos_sensor_8(0x5579, 0x01);
	write_cmos_sensor_8(0x557a, 0x03);
	write_cmos_sensor_8(0x557b, 0x06);
	write_cmos_sensor_8(0x557c, 0x0b);
	write_cmos_sensor_8(0x557d, 0x10);
	write_cmos_sensor_8(0x557e, 0x17);
	write_cmos_sensor_8(0x557f, 0x1f);
	write_cmos_sensor_8(0x5580, 0x1c);
	write_cmos_sensor_8(0x5581, 0x15);
	write_cmos_sensor_8(0x5582, 0x0f);
	write_cmos_sensor_8(0x5583, 0x0a);
	write_cmos_sensor_8(0x5584, 0x06);
	write_cmos_sensor_8(0x5585, 0x03);
	write_cmos_sensor_8(0x5589, 0x01);
	write_cmos_sensor_8(0x558a, 0x03);
	write_cmos_sensor_8(0x558b, 0x06);
	write_cmos_sensor_8(0x558c, 0x0b);
	write_cmos_sensor_8(0x558d, 0x10);
	write_cmos_sensor_8(0x558e, 0x16);
	write_cmos_sensor_8(0x558f, 0x1e);
	write_cmos_sensor_8(0x5590, 0x1d);
	write_cmos_sensor_8(0x5591, 0x16);
	write_cmos_sensor_8(0x5592, 0x11);
	write_cmos_sensor_8(0x5593, 0x0c);
	write_cmos_sensor_8(0x5594, 0x08);
	write_cmos_sensor_8(0x5595, 0x05);
	write_cmos_sensor_8(0x5598, 0x02);
	write_cmos_sensor_8(0x5599, 0x03);
	write_cmos_sensor_8(0x559a, 0x04);
	write_cmos_sensor_8(0x559b, 0x08);
	write_cmos_sensor_8(0x559c, 0x0c);
	write_cmos_sensor_8(0x559d, 0x12);
	write_cmos_sensor_8(0x559e, 0x18);
	write_cmos_sensor_8(0x559f, 0x1f);
	write_cmos_sensor_8(0x55a0, 0x20);
	write_cmos_sensor_8(0x55a1, 0x19);
	write_cmos_sensor_8(0x55a2, 0x14);
	write_cmos_sensor_8(0x55a3, 0x0f);
	write_cmos_sensor_8(0x55a4, 0x0b);
	write_cmos_sensor_8(0x55a5, 0x08);
	write_cmos_sensor_8(0x55a6, 0x06);
	write_cmos_sensor_8(0x55a7, 0x05);
	write_cmos_sensor_8(0x55a8, 0x06);
	write_cmos_sensor_8(0x55a9, 0x06);
	write_cmos_sensor_8(0x55aa, 0x08);
	write_cmos_sensor_8(0x55ab, 0x0b);
	write_cmos_sensor_8(0x55ac, 0x0f);
	write_cmos_sensor_8(0x55ad, 0x14);
	write_cmos_sensor_8(0x55ae, 0x1b);
	write_cmos_sensor_8(0x55af, 0x22);
	write_cmos_sensor_8(0x55b0, 0x24);
	write_cmos_sensor_8(0x55b1, 0x1e);
	write_cmos_sensor_8(0x55b2, 0x19);
	write_cmos_sensor_8(0x55b3, 0x14);
	write_cmos_sensor_8(0x55b4, 0x10);
	write_cmos_sensor_8(0x55b5, 0x0e);
	write_cmos_sensor_8(0x55b6, 0x0c);
	write_cmos_sensor_8(0x55b7, 0x0b);
	write_cmos_sensor_8(0x55b8, 0x0b);
	write_cmos_sensor_8(0x55b9, 0x0c);
	write_cmos_sensor_8(0x55ba, 0x0e);
	write_cmos_sensor_8(0x55bb, 0x11);
	write_cmos_sensor_8(0x55bc, 0x14);
	write_cmos_sensor_8(0x55bd, 0x19);
	write_cmos_sensor_8(0x55be, 0x1f);
	write_cmos_sensor_8(0x55bf, 0x27);
	write_cmos_sensor_8(0x55c0, 0x29);
	write_cmos_sensor_8(0x55c1, 0x23);
	write_cmos_sensor_8(0x55c2, 0x1f);
	write_cmos_sensor_8(0x55c3, 0x1a);
	write_cmos_sensor_8(0x55c4, 0x17);
	write_cmos_sensor_8(0x55c5, 0x14);
	write_cmos_sensor_8(0x55c6, 0x13);
	write_cmos_sensor_8(0x55c7, 0x12);
	write_cmos_sensor_8(0x55c8, 0x12);
	write_cmos_sensor_8(0x55c9, 0x13);
	write_cmos_sensor_8(0x55ca, 0x14);
	write_cmos_sensor_8(0x55cb, 0x17);
	write_cmos_sensor_8(0x55cc, 0x1a);
	write_cmos_sensor_8(0x55cd, 0x1f);
	write_cmos_sensor_8(0x55ce, 0x25);
	write_cmos_sensor_8(0x55cf, 0x2d);
	write_cmos_sensor_8(0x55d0, 0x31);
	write_cmos_sensor_8(0x55d1, 0x2a);
	write_cmos_sensor_8(0x55d2, 0x25);
	write_cmos_sensor_8(0x55d3, 0x22);
	write_cmos_sensor_8(0x55d4, 0x1f);
	write_cmos_sensor_8(0x55d5, 0x1c);
	write_cmos_sensor_8(0x55d6, 0x1b);
	write_cmos_sensor_8(0x55d7, 0x1a);
	write_cmos_sensor_8(0x55d8, 0x1a);
	write_cmos_sensor_8(0x55d9, 0x1a);
	write_cmos_sensor_8(0x55da, 0x1c);
	write_cmos_sensor_8(0x55db, 0x1f);
	write_cmos_sensor_8(0x55dc, 0x22);
	write_cmos_sensor_8(0x55dd, 0x27);
	write_cmos_sensor_8(0x55de, 0x2c);
	write_cmos_sensor_8(0x55df, 0x34);
	write_cmos_sensor_8(0x55e0, 0x3b);
	write_cmos_sensor_8(0x55e1, 0x32);
	write_cmos_sensor_8(0x55e2, 0x2e);
	write_cmos_sensor_8(0x55e3, 0x2a);
	write_cmos_sensor_8(0x55e4, 0x27);
	write_cmos_sensor_8(0x55e5, 0x25);
	write_cmos_sensor_8(0x55e6, 0x23);
	write_cmos_sensor_8(0x55e7, 0x23);
	write_cmos_sensor_8(0x55e8, 0x23);
	write_cmos_sensor_8(0x55e9, 0x24);
	write_cmos_sensor_8(0x55ea, 0x25);
	write_cmos_sensor_8(0x55eb, 0x28);
	write_cmos_sensor_8(0x55ec, 0x2b);
	write_cmos_sensor_8(0x55ed, 0x30);
	write_cmos_sensor_8(0x55ee, 0x35);
	write_cmos_sensor_8(0x55ef, 0x3f);
	write_cmos_sensor_8(0x55f0, 0x4b);
	write_cmos_sensor_8(0x55f1, 0x41);
	write_cmos_sensor_8(0x55f2, 0x3a);
	write_cmos_sensor_8(0x55f3, 0x36);
	write_cmos_sensor_8(0x55f4, 0x32);
	write_cmos_sensor_8(0x55f5, 0x30);
	write_cmos_sensor_8(0x55f6, 0x2f);
	write_cmos_sensor_8(0x55f7, 0x2d);
	write_cmos_sensor_8(0x55f8, 0x2f);
	write_cmos_sensor_8(0x55f9, 0x2f);
	write_cmos_sensor_8(0x55fa, 0x30);
	write_cmos_sensor_8(0x55fb, 0x33);
	write_cmos_sensor_8(0x55fc, 0x37);
	write_cmos_sensor_8(0x55fd, 0x3d);
	write_cmos_sensor_8(0x55fe, 0x44);
	write_cmos_sensor_8(0x55ff, 0x4e);
	write_cmos_sensor_8(0x5600, 0x80);
	write_cmos_sensor_8(0x5601, 0x80);
	write_cmos_sensor_8(0x5602, 0x80);
	write_cmos_sensor_8(0x5603, 0x7f);
	write_cmos_sensor_8(0x5605, 0x7f);
	write_cmos_sensor_8(0x5606, 0x7e);
	write_cmos_sensor_8(0x5607, 0x82);
	write_cmos_sensor_8(0x5608, 0x7e);
	write_cmos_sensor_8(0x5609, 0x80);
	write_cmos_sensor_8(0x560a, 0x80);
	write_cmos_sensor_8(0x560b, 0x7e);
	write_cmos_sensor_8(0x560c, 0x80);
	write_cmos_sensor_8(0x560e, 0x81);
	write_cmos_sensor_8(0x560f, 0x7a);
	write_cmos_sensor_8(0x5610, 0x80);
	write_cmos_sensor_8(0x5611, 0x80);
	write_cmos_sensor_8(0x5612, 0x80);
	write_cmos_sensor_8(0x5613, 0x80);
	write_cmos_sensor_8(0x5614, 0x7f);
	write_cmos_sensor_8(0x5615, 0x7f);
	write_cmos_sensor_8(0x5616, 0x80);
	write_cmos_sensor_8(0x5617, 0x7f);
	write_cmos_sensor_8(0x5618, 0x80);
	write_cmos_sensor_8(0x5619, 0x7f);
	write_cmos_sensor_8(0x561a, 0x7f);
	write_cmos_sensor_8(0x561b, 0x80);
	write_cmos_sensor_8(0x561c, 0x7f);
	write_cmos_sensor_8(0x561d, 0x7f);
	write_cmos_sensor_8(0x561e, 0x7f);
	write_cmos_sensor_8(0x561f, 0x82);
	write_cmos_sensor_8(0x5621, 0x80);
	write_cmos_sensor_8(0x5622, 0x7f);
	write_cmos_sensor_8(0x5623, 0x7f);
	write_cmos_sensor_8(0x5624, 0x7f);
	write_cmos_sensor_8(0x5625, 0x7f);
	write_cmos_sensor_8(0x5626, 0x7f);
	write_cmos_sensor_8(0x5627, 0x7f);
	write_cmos_sensor_8(0x5628, 0x7f);
	write_cmos_sensor_8(0x5629, 0x7f);
	write_cmos_sensor_8(0x562a, 0x7f);
	write_cmos_sensor_8(0x562b, 0x7f);
	write_cmos_sensor_8(0x562c, 0x7f);
	write_cmos_sensor_8(0x562d, 0x80);
	write_cmos_sensor_8(0x562e, 0x7f);
	write_cmos_sensor_8(0x562f, 0x81);
	write_cmos_sensor_8(0x5630, 0x80);
	write_cmos_sensor_8(0x5631, 0x7f);
	write_cmos_sensor_8(0x5632, 0x7f);
	write_cmos_sensor_8(0x5633, 0x7f);
	write_cmos_sensor_8(0x5634, 0x7f);
	write_cmos_sensor_8(0x5635, 0x7f);
	write_cmos_sensor_8(0x5636, 0x7f);
	write_cmos_sensor_8(0x5637, 0x7f);
	write_cmos_sensor_8(0x5638, 0x7f);
	write_cmos_sensor_8(0x5639, 0x7f);
	write_cmos_sensor_8(0x563a, 0x7f);
	write_cmos_sensor_8(0x563b, 0x7f);
	write_cmos_sensor_8(0x563c, 0x7f);
	write_cmos_sensor_8(0x563d, 0x7f);
	write_cmos_sensor_8(0x563e, 0x80);
	write_cmos_sensor_8(0x563f, 0x80);
	write_cmos_sensor_8(0x5640, 0x81);
	write_cmos_sensor_8(0x5641, 0x80);
	write_cmos_sensor_8(0x5643, 0x7f);
	write_cmos_sensor_8(0x5644, 0x7f);
	write_cmos_sensor_8(0x5645, 0x7f);
	write_cmos_sensor_8(0x5646, 0x7f);
	write_cmos_sensor_8(0x5647, 0x7f);
	write_cmos_sensor_8(0x5648, 0x7f);
	write_cmos_sensor_8(0x5649, 0x7f);
	write_cmos_sensor_8(0x564a, 0x7f);
	write_cmos_sensor_8(0x564b, 0x7f);
	write_cmos_sensor_8(0x564c, 0x7f);
	write_cmos_sensor_8(0x564d, 0x7f);
	write_cmos_sensor_8(0x564e, 0x80);
	write_cmos_sensor_8(0x564f, 0x7f);
	write_cmos_sensor_8(0x5650, 0x80);
	write_cmos_sensor_8(0x5652, 0x80);
	write_cmos_sensor_8(0x5653, 0x7f);
	write_cmos_sensor_8(0x5654, 0x7f);
	write_cmos_sensor_8(0x5655, 0x7f);
	write_cmos_sensor_8(0x5656, 0x7f);
	write_cmos_sensor_8(0x5657, 0x7f);
	write_cmos_sensor_8(0x5658, 0x7f);
	write_cmos_sensor_8(0x5659, 0x7f);
	write_cmos_sensor_8(0x565a, 0x7f);
	write_cmos_sensor_8(0x565b, 0x7f);
	write_cmos_sensor_8(0x565c, 0x7f);
	write_cmos_sensor_8(0x565d, 0x7f);
	write_cmos_sensor_8(0x565f, 0x7f);
	write_cmos_sensor_8(0x5660, 0x80);
	write_cmos_sensor_8(0x5661, 0x80);
	write_cmos_sensor_8(0x5663, 0x7f);
	write_cmos_sensor_8(0x5666, 0x7f);
	write_cmos_sensor_8(0x5667, 0x7f);
	write_cmos_sensor_8(0x5668, 0x7f);
	write_cmos_sensor_8(0x5669, 0x7f);
	write_cmos_sensor_8(0x566c, 0x80);
	write_cmos_sensor_8(0x566d, 0x80);
	write_cmos_sensor_8(0x566e, 0x7f);
	write_cmos_sensor_8(0x566f, 0x80);
	write_cmos_sensor_8(0x5671, 0x80);
	write_cmos_sensor_8(0x5672, 0x80);
	write_cmos_sensor_8(0x5673, 0x80);
	write_cmos_sensor_8(0x5676, 0x80);
	write_cmos_sensor_8(0x5677, 0x80);
	write_cmos_sensor_8(0x5678, 0x80);
	write_cmos_sensor_8(0x5679, 0x80);
	write_cmos_sensor_8(0x567a, 0x80);
	write_cmos_sensor_8(0x567b, 0x80);
	write_cmos_sensor_8(0x567c, 0x80);
	write_cmos_sensor_8(0x567d, 0x80);
	write_cmos_sensor_8(0x567e, 0x80);
	write_cmos_sensor_8(0x567f, 0x80);
	write_cmos_sensor_8(0x5680, 0x80);
	write_cmos_sensor_8(0x5681, 0x80);
	write_cmos_sensor_8(0x5682, 0x80);
	write_cmos_sensor_8(0x5684, 0x81);
	write_cmos_sensor_8(0x5685, 0x80);
	write_cmos_sensor_8(0x5686, 0x80);
	write_cmos_sensor_8(0x5687, 0x80);
	write_cmos_sensor_8(0x5688, 0x80);
	write_cmos_sensor_8(0x5689, 0x80);
	write_cmos_sensor_8(0x568a, 0x80);
	write_cmos_sensor_8(0x568b, 0x80);
	write_cmos_sensor_8(0x568c, 0x80);
	write_cmos_sensor_8(0x568d, 0x80);
	write_cmos_sensor_8(0x568e, 0x80);
	write_cmos_sensor_8(0x5691, 0x80);
	write_cmos_sensor_8(0x5692, 0x80);
	write_cmos_sensor_8(0x5695, 0x7f);
	write_cmos_sensor_8(0x5696, 0x80);
	write_cmos_sensor_8(0x5697, 0x80);
	write_cmos_sensor_8(0x5698, 0x80);
	write_cmos_sensor_8(0x5699, 0x80);
	write_cmos_sensor_8(0x569a, 0x80);
	write_cmos_sensor_8(0x569b, 0x80);
	write_cmos_sensor_8(0x569c, 0x80);
	write_cmos_sensor_8(0x569d, 0x80);
	write_cmos_sensor_8(0x569f, 0x81);
	write_cmos_sensor_8(0x56a1, 0x80);
	write_cmos_sensor_8(0x56a6, 0x7f);
	write_cmos_sensor_8(0x56a7, 0x7f);
	write_cmos_sensor_8(0x56a8, 0x7f);
	write_cmos_sensor_8(0x56a9, 0x7f);
	write_cmos_sensor_8(0x56aa, 0x80);
	write_cmos_sensor_8(0x56ab, 0x80);
	write_cmos_sensor_8(0x56ad, 0x80);
	write_cmos_sensor_8(0x56af, 0x80);
	write_cmos_sensor_8(0x56b5, 0x7f);
	write_cmos_sensor_8(0x56b6, 0x7f);
	write_cmos_sensor_8(0x56b7, 0x7f);
	write_cmos_sensor_8(0x56b8, 0x7f);
	write_cmos_sensor_8(0x56b9, 0x7f);
	write_cmos_sensor_8(0x56bb, 0x7f);
	write_cmos_sensor_8(0x56be, 0x80);
	write_cmos_sensor_8(0x56bf, 0x80);
	write_cmos_sensor_8(0x56c0, 0x80);
	write_cmos_sensor_8(0x56c3, 0x80);
	write_cmos_sensor_8(0x56c6, 0x7f);
	write_cmos_sensor_8(0x56c8, 0x7f);
	write_cmos_sensor_8(0x56c9, 0x7f);
	write_cmos_sensor_8(0x56ca, 0x7f);
	write_cmos_sensor_8(0x56cb, 0x80);
	write_cmos_sensor_8(0x56cc, 0x80);
	write_cmos_sensor_8(0x56d0, 0x81);
	write_cmos_sensor_8(0x56d1, 0x80);
	write_cmos_sensor_8(0x56d3, 0x80);
	write_cmos_sensor_8(0x56d4, 0x7f);
	write_cmos_sensor_8(0x56d5, 0x80);
	write_cmos_sensor_8(0x56d6, 0x7f);
	write_cmos_sensor_8(0x56d8, 0x80);
	write_cmos_sensor_8(0x56da, 0x80);
	write_cmos_sensor_8(0x56db, 0x7f);
	write_cmos_sensor_8(0x56dd, 0x80);
	write_cmos_sensor_8(0x56df, 0x81);
	write_cmos_sensor_8(0x56e1, 0x7f);
	write_cmos_sensor_8(0x56e2, 0x80);
	write_cmos_sensor_8(0x56e3, 0x7f);
	write_cmos_sensor_8(0x56e4, 0x7f);
	write_cmos_sensor_8(0x56e5, 0x7f);
	write_cmos_sensor_8(0x56e7, 0x80);
	write_cmos_sensor_8(0x56e8, 0x7f);
	write_cmos_sensor_8(0x56e9, 0x80);
	write_cmos_sensor_8(0x56ea, 0x7f);
	write_cmos_sensor_8(0x56eb, 0x80);
	write_cmos_sensor_8(0x56ec, 0x80);
	write_cmos_sensor_8(0x56ee, 0x80);
	write_cmos_sensor_8(0x56ef, 0x7f);
	write_cmos_sensor_8(0x56f0, 0x7b);
	write_cmos_sensor_8(0x56f1, 0x81);
	write_cmos_sensor_8(0x56f2, 0x7f);
	write_cmos_sensor_8(0x56f3, 0x80);
	write_cmos_sensor_8(0x56f5, 0x7f);
	write_cmos_sensor_8(0x56f6, 0x7f);
	write_cmos_sensor_8(0x56f7, 0x80);
	write_cmos_sensor_8(0x56f9, 0x7f);
	write_cmos_sensor_8(0x56fc, 0x7f);
	write_cmos_sensor_8(0x56fd, 0x81);
	write_cmos_sensor_8(0x56fe, 0x80);
	write_cmos_sensor_8(0x56ff, 0x7f);
	write_cmos_sensor_8(0x5700, 0x84);
	write_cmos_sensor_8(0x5701, 0x7e);
	write_cmos_sensor_8(0x5702, 0x81);
	write_cmos_sensor_8(0x5703, 0x7d);
	write_cmos_sensor_8(0x5704, 0x80);
	write_cmos_sensor_8(0x5705, 0x7f);
	write_cmos_sensor_8(0x5706, 0x7f);
	write_cmos_sensor_8(0x5707, 0x81);
	write_cmos_sensor_8(0x5708, 0x7e);
	write_cmos_sensor_8(0x5709, 0x7e);
	write_cmos_sensor_8(0x570a, 0x81);
	write_cmos_sensor_8(0x570b, 0x7f);
	write_cmos_sensor_8(0x570c, 0x80);
	write_cmos_sensor_8(0x570d, 0x7f);
	write_cmos_sensor_8(0x570e, 0x7f);
	write_cmos_sensor_8(0x570f, 0x81);
	write_cmos_sensor_8(0x5710, 0x7d);
	write_cmos_sensor_8(0x5711, 0x80);
	write_cmos_sensor_8(0x5712, 0x7e);
	write_cmos_sensor_8(0x5713, 0x80);
	write_cmos_sensor_8(0x5714, 0x7f);
	write_cmos_sensor_8(0x5715, 0x7f);
	write_cmos_sensor_8(0x5716, 0x7f);
	write_cmos_sensor_8(0x5717, 0x7f);
	write_cmos_sensor_8(0x5718, 0x80);
	write_cmos_sensor_8(0x5719, 0x80);
	write_cmos_sensor_8(0x571a, 0x7f);
	write_cmos_sensor_8(0x571b, 0x80);
	write_cmos_sensor_8(0x571c, 0x7f);
	write_cmos_sensor_8(0x571d, 0x80);
	write_cmos_sensor_8(0x571e, 0x7f);
	write_cmos_sensor_8(0x571f, 0x81);
	write_cmos_sensor_8(0x5720, 0x80);
	write_cmos_sensor_8(0x5721, 0x7f);
	write_cmos_sensor_8(0x5722, 0x80);
	write_cmos_sensor_8(0x5723, 0x7f);
	write_cmos_sensor_8(0x5724, 0x7f);
	write_cmos_sensor_8(0x5725, 0x7f);
	write_cmos_sensor_8(0x5726, 0x7f);
	write_cmos_sensor_8(0x5727, 0x7f);
	write_cmos_sensor_8(0x5728, 0x7f);
	write_cmos_sensor_8(0x5729, 0x7f);
	write_cmos_sensor_8(0x572a, 0x7f);
	write_cmos_sensor_8(0x572b, 0x7f);
	write_cmos_sensor_8(0x572c, 0x7f);
	write_cmos_sensor_8(0x572d, 0x7f);
	write_cmos_sensor_8(0x572e, 0x7f);
	write_cmos_sensor_8(0x572f, 0x81);
	write_cmos_sensor_8(0x5730, 0x7f);
	write_cmos_sensor_8(0x5731, 0x7f);
	write_cmos_sensor_8(0x5732, 0x7f);
	write_cmos_sensor_8(0x5733, 0x7f);
	write_cmos_sensor_8(0x5734, 0x7f);
	write_cmos_sensor_8(0x5735, 0x7f);
	write_cmos_sensor_8(0x5736, 0x7f);
	write_cmos_sensor_8(0x5737, 0x7f);
	write_cmos_sensor_8(0x5738, 0x7f);
	write_cmos_sensor_8(0x5739, 0x7f);
	write_cmos_sensor_8(0x573a, 0x7f);
	write_cmos_sensor_8(0x573b, 0x7f);
	write_cmos_sensor_8(0x573c, 0x7f);
	write_cmos_sensor_8(0x573d, 0x7f);
	write_cmos_sensor_8(0x573e, 0x7f);
	write_cmos_sensor_8(0x573f, 0x80);
	write_cmos_sensor_8(0x5740, 0x80);
	write_cmos_sensor_8(0x5741, 0x7f);
	write_cmos_sensor_8(0x5742, 0x80);
	write_cmos_sensor_8(0x5743, 0x7f);
	write_cmos_sensor_8(0x5744, 0x7f);
	write_cmos_sensor_8(0x5745, 0x7f);
	write_cmos_sensor_8(0x5746, 0x7f);
	write_cmos_sensor_8(0x5747, 0x7f);
	write_cmos_sensor_8(0x5748, 0x7f);
	write_cmos_sensor_8(0x5749, 0x7f);
	write_cmos_sensor_8(0x574a, 0x7f);
	write_cmos_sensor_8(0x574b, 0x7f);
	write_cmos_sensor_8(0x574c, 0x7f);
	write_cmos_sensor_8(0x574d, 0x7f);
	write_cmos_sensor_8(0x574e, 0x7f);
	write_cmos_sensor_8(0x574f, 0x80);
	write_cmos_sensor_8(0x5750, 0x80);
	write_cmos_sensor_8(0x5751, 0x7f);
	write_cmos_sensor_8(0x5752, 0x7f);
	write_cmos_sensor_8(0x5753, 0x7f);
	write_cmos_sensor_8(0x5754, 0x7f);
	write_cmos_sensor_8(0x5755, 0x7f);
	write_cmos_sensor_8(0x5756, 0x7f);
	write_cmos_sensor_8(0x5757, 0x7f);
	write_cmos_sensor_8(0x5758, 0x7f);
	write_cmos_sensor_8(0x5759, 0x7f);
	write_cmos_sensor_8(0x575a, 0x7f);
	write_cmos_sensor_8(0x575b, 0x7f);
	write_cmos_sensor_8(0x575c, 0x7f);
	write_cmos_sensor_8(0x575d, 0x7f);
	write_cmos_sensor_8(0x575e, 0x7f);
	write_cmos_sensor_8(0x575f, 0x80);
	write_cmos_sensor_8(0x5760, 0x7e);
	write_cmos_sensor_8(0x5761, 0x80);
	write_cmos_sensor_8(0x5762, 0x7f);
	write_cmos_sensor_8(0x5763, 0x7f);
	write_cmos_sensor_8(0x5764, 0x7f);
	write_cmos_sensor_8(0x5766, 0x7f);
	write_cmos_sensor_8(0x5767, 0x7f);
	write_cmos_sensor_8(0x5768, 0x80);
	write_cmos_sensor_8(0x5769, 0x7f);
	write_cmos_sensor_8(0x576a, 0x80);
	write_cmos_sensor_8(0x576b, 0x7f);
	write_cmos_sensor_8(0x576c, 0x80);
	write_cmos_sensor_8(0x576d, 0x80);
	write_cmos_sensor_8(0x576e, 0x7f);
	write_cmos_sensor_8(0x576f, 0x80);
	write_cmos_sensor_8(0x5770, 0x80);
	write_cmos_sensor_8(0x5771, 0x80);
	write_cmos_sensor_8(0x5772, 0x80);
	write_cmos_sensor_8(0x5774, 0x80);
	write_cmos_sensor_8(0x5775, 0x80);
	write_cmos_sensor_8(0x5776, 0x80);
	write_cmos_sensor_8(0x5778, 0x80);
	write_cmos_sensor_8(0x5779, 0x80);
	write_cmos_sensor_8(0x577a, 0x80);
	write_cmos_sensor_8(0x577b, 0x80);
	write_cmos_sensor_8(0x577c, 0x7f);
	write_cmos_sensor_8(0x577d, 0x80);
	write_cmos_sensor_8(0x577e, 0x80);
	write_cmos_sensor_8(0x577f, 0x80);
	write_cmos_sensor_8(0x5780, 0x80);
	write_cmos_sensor_8(0x5781, 0x80);
	write_cmos_sensor_8(0x5782, 0x80);
	write_cmos_sensor_8(0x5784, 0x80);
	write_cmos_sensor_8(0x5785, 0x80);
	write_cmos_sensor_8(0x5786, 0x80);
	write_cmos_sensor_8(0x5788, 0x80);
	write_cmos_sensor_8(0x5789, 0x81);
	write_cmos_sensor_8(0x578a, 0x80);
	write_cmos_sensor_8(0x578b, 0x80);
	write_cmos_sensor_8(0x578c, 0x80);
	write_cmos_sensor_8(0x578d, 0x80);
	write_cmos_sensor_8(0x578e, 0x80);
	write_cmos_sensor_8(0x578f, 0x80);
	write_cmos_sensor_8(0x5790, 0x7f);
	write_cmos_sensor_8(0x5791, 0x80);
	write_cmos_sensor_8(0x5792, 0x80);
	write_cmos_sensor_8(0x5795, 0x80);
	write_cmos_sensor_8(0x5798, 0x80);
	write_cmos_sensor_8(0x5799, 0x7f);
	write_cmos_sensor_8(0x579a, 0x80);
	write_cmos_sensor_8(0x579b, 0x80);
	write_cmos_sensor_8(0x579c, 0x80);
	write_cmos_sensor_8(0x579d, 0x80);
	write_cmos_sensor_8(0x579e, 0x80);
	write_cmos_sensor_8(0x579f, 0x80);
	write_cmos_sensor_8(0x57a0, 0x7f);
	write_cmos_sensor_8(0x57a1, 0x80);
	write_cmos_sensor_8(0x57a2, 0x80);
	write_cmos_sensor_8(0x57a3, 0x7f);
	write_cmos_sensor_8(0x57a4, 0x80);
	write_cmos_sensor_8(0x57a5, 0x7f);
	write_cmos_sensor_8(0x57a6, 0x80);
	write_cmos_sensor_8(0x57a7, 0x7f);
	write_cmos_sensor_8(0x57a8, 0x7f);
	write_cmos_sensor_8(0x57a9, 0x7f);
	write_cmos_sensor_8(0x57aa, 0x80);
	write_cmos_sensor_8(0x57ab, 0x7f);
	write_cmos_sensor_8(0x57ac, 0x7f);
	write_cmos_sensor_8(0x57ad, 0x80);
	write_cmos_sensor_8(0x57ae, 0x80);
	write_cmos_sensor_8(0x57af, 0x81);
	write_cmos_sensor_8(0x57b0, 0x80);
	write_cmos_sensor_8(0x57b1, 0x7f);
	write_cmos_sensor_8(0x57b2, 0x7f);
	write_cmos_sensor_8(0x57b3, 0x7f);
	write_cmos_sensor_8(0x57b4, 0x7f);
	write_cmos_sensor_8(0x57b5, 0x7f);
	write_cmos_sensor_8(0x57b6, 0x7f);
	write_cmos_sensor_8(0x57b7, 0x7f);
	write_cmos_sensor_8(0x57b8, 0x7f);
	write_cmos_sensor_8(0x57b9, 0x7f);
	write_cmos_sensor_8(0x57ba, 0x7f);
	write_cmos_sensor_8(0x57bb, 0x7f);
	write_cmos_sensor_8(0x57bc, 0x80);
	write_cmos_sensor_8(0x57bd, 0x7f);
	write_cmos_sensor_8(0x57be, 0x80);
	write_cmos_sensor_8(0x57bf, 0x7f);
	write_cmos_sensor_8(0x57c0, 0x7f);
	write_cmos_sensor_8(0x57c1, 0x7f);
	write_cmos_sensor_8(0x57c2, 0x7f);
	write_cmos_sensor_8(0x57c3, 0x7f);
	write_cmos_sensor_8(0x57c4, 0x7f);
	write_cmos_sensor_8(0x57c5, 0x7f);
	write_cmos_sensor_8(0x57c6, 0x80);
	write_cmos_sensor_8(0x57c7, 0x7f);
	write_cmos_sensor_8(0x57c8, 0x80);
	write_cmos_sensor_8(0x57c9, 0x7f);
	write_cmos_sensor_8(0x57ca, 0x7f);
	write_cmos_sensor_8(0x57cb, 0x7f);
	write_cmos_sensor_8(0x57cc, 0x80);
	write_cmos_sensor_8(0x57cd, 0x80);
	write_cmos_sensor_8(0x57ce, 0x7f);
	write_cmos_sensor_8(0x57cf, 0x81);
	write_cmos_sensor_8(0x57d0, 0x80);
	write_cmos_sensor_8(0x57d1, 0x80);
	write_cmos_sensor_8(0x57d2, 0x80);
	write_cmos_sensor_8(0x57d3, 0x7f);
	write_cmos_sensor_8(0x57d4, 0x7f);
	write_cmos_sensor_8(0x57d5, 0x80);
	write_cmos_sensor_8(0x57d6, 0x7f);
	write_cmos_sensor_8(0x57d7, 0x7f);
	write_cmos_sensor_8(0x57d8, 0x7f);
	write_cmos_sensor_8(0x57d9, 0x7f);
	write_cmos_sensor_8(0x57da, 0x7f);
	write_cmos_sensor_8(0x57db, 0x80);
	write_cmos_sensor_8(0x57dc, 0x7f);
	write_cmos_sensor_8(0x57dd, 0x80);
	write_cmos_sensor_8(0x57de, 0x80);
	write_cmos_sensor_8(0x57df, 0x80);
	write_cmos_sensor_8(0x57e0, 0x7f);
	write_cmos_sensor_8(0x57e1, 0x7f);
	write_cmos_sensor_8(0x57e2, 0x80);
	write_cmos_sensor_8(0x57e3, 0x7f);
	write_cmos_sensor_8(0x57e4, 0x80);
	write_cmos_sensor_8(0x57e5, 0x7f);
	write_cmos_sensor_8(0x57e6, 0x80);
	write_cmos_sensor_8(0x57e7, 0x7f);
	write_cmos_sensor_8(0x57e8, 0x80);
	write_cmos_sensor_8(0x57e9, 0x7f);
	write_cmos_sensor_8(0x57ea, 0x80);
	write_cmos_sensor_8(0x57eb, 0x80);
	write_cmos_sensor_8(0x57ec, 0x80);
	write_cmos_sensor_8(0x57ed, 0x7f);
	write_cmos_sensor_8(0x57ee, 0x80);
	write_cmos_sensor_8(0x57ef, 0x80);
	write_cmos_sensor_8(0x57f0, 0x7f);
	write_cmos_sensor_8(0x57f1, 0x80);
	write_cmos_sensor_8(0x57f2, 0x7e);
	write_cmos_sensor_8(0x57f3, 0x80);
	write_cmos_sensor_8(0x57f4, 0x7e);
	write_cmos_sensor_8(0x57f5, 0x81);
	write_cmos_sensor_8(0x57f6, 0x7e);
	write_cmos_sensor_8(0x57f7, 0x81);
	write_cmos_sensor_8(0x57f8, 0x7e);
	write_cmos_sensor_8(0x57f9, 0x80);
	write_cmos_sensor_8(0x57fa, 0x80);
	write_cmos_sensor_8(0x57fb, 0x80);
	write_cmos_sensor_8(0x57fc, 0x7e);
	write_cmos_sensor_8(0x57fd, 0x81);
	write_cmos_sensor_8(0x57fe, 0x80);
	write_cmos_sensor_8(0x57ff, 0x7a);
	//write_cmos_sensor_8(0x0100, 0x01);

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
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {	
			*sensor_id = (read_cmos_sensor_8(0x300A) << 16) | (read_cmos_sensor_8(0x300B)<<8) | read_cmos_sensor_8(0x300C)+1;
			if (*sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
				*sensor_id = OV168854CELL_SENSOR_ID;
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
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = (read_cmos_sensor_8(0x300A) << 16) | (read_cmos_sensor_8(0x300B)<<8) | read_cmos_sensor_8(0x300C)+1;
			LOG_INF("Read sensor id fail, write id:0x%x ,sensor Id:0x%x\n", imgsensor.i2c_write_id, sensor_id);//lx_revised
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
	LOG_INF("E");
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
	LOG_INF("Caputre fps:%d\n",imgsensor.current_fps);
	capture_setting(imgsensor.current_fps);
    //set_mirror_flip(IMAGE_NORMAL);

	return ERROR_NONE;
}	/* capture() */
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
	//normal_video_setting(imgsensor.current_fps);
	preview_setting();
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
		write_cmos_sensor_8(0x5081, 0x01);
		write_cmos_sensor_8(0x5000,(read_cmos_sensor(0x5000)&0xa1)|0x00 );

	} else {
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		write_cmos_sensor_8(0x5081, 0x00);
		write_cmos_sensor_8(0x5000,(read_cmos_sensor(0x5000)&0xa1)|0x040 );
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
	struct SET_PD_BLOCK_INFO_T *PDAFinfo;
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
			//read_ov168854cell_eeprom((kal_uint16 )(*feature_data),(char*)(uintptr_t)(*(feature_data+1)),(kal_uint32)(*(feature_data+2)));
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
			//PDAF capacity enable or not, OV168854CELL only full size support PDAF
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

UINT32 OV168854CELL_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}	/*	OV168854CELL_MIPI_RAW_SensorInit	*/
