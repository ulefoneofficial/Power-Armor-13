/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 IMX351mipi_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
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
#include <linux/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "imx351mipiraw_Sensor.h"

#define PFX "IMX351_camera_sensor"


#define LOG_INF(format, args...)	pr_debug(PFX "[%s] " format, __func__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);

static kal_uint8 test_pattern_flag=0;

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = IMX351_SENSOR_ID,

	.checksum_value =0xffb1ec31,

	.pre = {
		.pclk =319200000,
		.linelength = 3320,
		.framelength = 3300,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 2328,
		.grabwindow_height = 1744,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 319200000,
	},
	.cap = {

		.pclk =319200000,
		.linelength = 3320,
		.framelength = 3300,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 2328,
		.grabwindow_height = 1744,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 319200000,
	},
	.normal_video = {
		.pclk =319200000,
		.linelength = 3320,
		.framelength = 3300,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 2328,
		.grabwindow_height = 1744,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 319200000,
	},
	.hs_video = {
		.pclk =319200000,
		.linelength = 3320,
		.framelength = 3300,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 2328,
		.grabwindow_height = 1744,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 319200000,
	},
	.slim_video = {
		.pclk =319200000,
		.linelength = 3320,
		.framelength = 3300,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 2328,
		.grabwindow_height = 1744,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 319200000,
	},
	.margin = 18,
	.min_shutter = 8,
	.max_frame_length = 0xffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame =0,
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0,	  //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 5,	  //support sensor mode num

	.cap_delay_frame = 3,//3 
	.pre_delay_frame = 3,//3 
	.video_delay_frame = 3,
	.hs_video_delay_frame = 3,
	.slim_video_delay_frame = 3,
	
	.isp_driving_current = ISP_DRIVING_4MA,
	
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
	.mipi_settle_delay_mode = 0, //0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,//SENSOR_OUTPUT_FORMAT_RAW_Gr,//SENSOR_OUTPUT_FORMAT_RAW_R, //SENSOR_OUTPUT_FORMAT_RAW_B
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x34, 0x34, 0xff}, // {0x20, 0x21, 0xff},
	.i2c_speed = 400,// i2c read/write speed
};


static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_HV_MIRROR,//IMAGE_NORMAL,				//mirrorflip information //IMAGE_HV_MIRROR
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x3D0,					//current shutter
	.gain = 0x100,						//current gain
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
	.current_fps = 0,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,		//test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.ihdr_mode = 0, //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x34, // 0x20,

};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =
{
{ 2328, 1744, 0, 0, 2328, 1744, 2328, 1744,	0000, 0000, 2328, 1744, 0, 0, 2328, 1744},	/* Preview */
{ 2328, 1744, 0, 0, 2328, 1744, 2328, 1744,	0000, 0000, 2328, 1744, 0, 0, 2328, 1744},	/* Preview */
{ 2328,  1744,  0,  0,  2328,  1744,  2328,  1744,  0000,  0000,  2328,  1744,  0,  0,  2328,  1744}, // video
{ 2328,  1744,  0,  0,  2328,  1744,  2328,  1744,  0000,  0000,  2328,  1744,  0,  0,  2328,  1744}, //hight speed video
{ 2328,  1744,  0,  0,  2328,  1744,  2328,  1744,  0000,  0000,  2328,  1744,  0,  0,  2328,  1744},// slim video
};

extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
extern void kdSetI2CSpeed(u16 i2cSpeed);
//extern bool read_3P3_eeprom( kal_uint16 addr, BYTE* data, kal_uint32 size);

#if 0
static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
      iReadReg((u16) addr ,(u8*)&get_byte, imgsensor.i2c_write_id);
      return get_byte;
}

#define write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 1,  imgsensor.i2c_write_id)
#endif
#define RWB_ID_OFFSET 0x0F73

#define EEPROM_READ_ID  0xA0
#define EEPROM_WRITE_ID   0xA1

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    //kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte, 2, imgsensor.i2c_write_id);
    return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);
}


static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
    //kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
    iWriteRegI2C(pusendcmd , 4, imgsensor.i2c_write_id);
}

static kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
    kal_uint16 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    //kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
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
	LOG_INF("imx351 dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	//return; //for test
     write_cmos_sensor(0x0340, imgsensor.frame_length);
     write_cmos_sensor(0x0342, imgsensor.line_length);
}	/*	set_dummy  */


static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{

	kal_uint32 frame_length = imgsensor.frame_length;

	LOG_INF("imx351 framerate = %d, min framelength should enable %d \n", framerate,min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	if (frame_length >= imgsensor.min_frame_length)
		imgsensor.frame_length = frame_length;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
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

static void write_shutter(kal_uint16 shutter)
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
	if (shutter < imgsensor_info.min_shutter) shutter = imgsensor_info.min_shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);
		else {
		// Extend frame length
 		write_cmos_sensor_8(0x0104, 0x01);
		write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor_8(0x0104, 0x00);
        }
    } else {
        // Extend frame length
		write_cmos_sensor_8(0x0104, 0x01);
		write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor_8(0x0104, 0x00);
    }

    // Update Shutter
	write_cmos_sensor_8(0x0104, 0x01);
    write_cmos_sensor_8(0x0202, (shutter >> 8) & 0xFF);
    write_cmos_sensor_8(0x0203, shutter  & 0xFF);
    write_cmos_sensor_8(0x0104, 0x00);
    LOG_INF("imx351 Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

}	/*	write_shutter  */



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
static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
}	/*	set_shutter */




static kal_uint16 gain2reg(const kal_uint16 gain)
{
   kal_uint16 reg_gain;

   reg_gain = 1024 - (1024*64)/gain;

   LOG_INF("imx351 imx351 gain =%d, reg_gain =%d\n", gain,reg_gain);

   return reg_gain;
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
    kal_uint16 reg_gain;

    //gain=1024;//for test
    //return; //for test

    if (gain < BASEGAIN || gain > 16 * BASEGAIN) {
        LOG_INF("imx351 Error gain setting");

	if (gain < BASEGAIN)
		gain = BASEGAIN;
	else if (gain > 16 * BASEGAIN)
		gain = 16 * BASEGAIN;
	}

    reg_gain = gain2reg(gain);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.gain = reg_gain;
    spin_unlock(&imgsensor_drv_lock);
    LOG_INF("imx351 gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor_8(0x0104, 0x01);
	write_cmos_sensor_8(0x0204, (reg_gain>>8)& 0xFF);
	write_cmos_sensor_8(0x0205, reg_gain & 0xFF);
	write_cmos_sensor_8(0x0104, 0x00);

    return gain;
}	/*	set_gain  */

static void set_mirror_flip(kal_uint8 image_mirror)
{

	kal_uint8 itemp;

    LOG_INF("imx351 image_mirror = %d\n", image_mirror);
	itemp=read_cmos_sensor_8(0x0101);
	itemp &= ~0x03;

	switch(image_mirror)
		{

		   case IMAGE_NORMAL:
		   	     write_cmos_sensor_8(0x0101, itemp);
			      break;

		   case IMAGE_V_MIRROR:
			     write_cmos_sensor_8(0x0101, itemp | 0x02);
			     break;

		   case IMAGE_H_MIRROR:
			     write_cmos_sensor_8(0x0101, itemp | 0x01);
			     break;

		   case IMAGE_HV_MIRROR:
			     write_cmos_sensor_8(0x0101, itemp | 0x03);
			     break;
		}
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
#if 0
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/
}	/*	night_mode	*/
#endif
static void sensor_init(void)
{
	write_cmos_sensor_8(0x0136, 0x18);
	write_cmos_sensor_8(0x0137, 0x00);
	write_cmos_sensor_8(0x3C7D, 0x18);
	write_cmos_sensor_8(0x3C7E, 0x03);
	write_cmos_sensor_8(0x3C7F, 0x0F);
	write_cmos_sensor_8(0x3140, 0x02);
	write_cmos_sensor_8(0x3F7F, 0x01);
	write_cmos_sensor_8(0x4430, 0x05);
	write_cmos_sensor_8(0x4431, 0xDC);
	write_cmos_sensor_8(0x4ED0, 0x01);
	write_cmos_sensor_8(0x4ED1, 0x3E);
	write_cmos_sensor_8(0x4EDE, 0x01);
	write_cmos_sensor_8(0x4EDF, 0x45);
	write_cmos_sensor_8(0x5222, 0x02);
	write_cmos_sensor_8(0x5617, 0x0A);
	write_cmos_sensor_8(0x562B, 0x0A);
	write_cmos_sensor_8(0x562D, 0x0C);
	write_cmos_sensor_8(0x56B7, 0x74);
	write_cmos_sensor_8(0x6282, 0x83);
	write_cmos_sensor_8(0x6283, 0x22);
	write_cmos_sensor_8(0x6284, 0x0A);
	write_cmos_sensor_8(0x6285, 0x60);
	write_cmos_sensor_8(0x6286, 0x09);
	write_cmos_sensor_8(0x6287, 0x85);
	write_cmos_sensor_8(0x6288, 0x18);
	write_cmos_sensor_8(0x6289, 0xC0);
	write_cmos_sensor_8(0x628A, 0x1C);
	write_cmos_sensor_8(0x628B, 0x8C);
	write_cmos_sensor_8(0x628C, 0x41);
	write_cmos_sensor_8(0x628D, 0x80);
	write_cmos_sensor_8(0x628E, 0x38);
	write_cmos_sensor_8(0x628F, 0x18);
	write_cmos_sensor_8(0x6290, 0x83);
	write_cmos_sensor_8(0x6292, 0x1A);
	write_cmos_sensor_8(0x6293, 0x20);
	write_cmos_sensor_8(0x6294, 0x88);
	write_cmos_sensor_8(0x6296, 0x6C);
	write_cmos_sensor_8(0x6297, 0x41);
	write_cmos_sensor_8(0x6298, 0x4C);
	write_cmos_sensor_8(0x6299, 0x01);
	write_cmos_sensor_8(0x629A, 0x38);
	write_cmos_sensor_8(0x629B, 0x93);
	write_cmos_sensor_8(0x629C, 0x18);
	write_cmos_sensor_8(0x629D, 0x03);
	write_cmos_sensor_8(0x629E, 0x91);
	write_cmos_sensor_8(0x629F, 0x87);
	write_cmos_sensor_8(0x62A0, 0x30);
	write_cmos_sensor_8(0x62B1, 0x14);
	write_cmos_sensor_8(0x62B2, 0x20);
	write_cmos_sensor_8(0x62B3, 0x48);
	write_cmos_sensor_8(0x62B5, 0x58);
	write_cmos_sensor_8(0x62B6, 0x40);
	write_cmos_sensor_8(0x62B7, 0x8E);
	write_cmos_sensor_8(0x62B8, 0x01);
	write_cmos_sensor_8(0x62B9, 0x08);
	write_cmos_sensor_8(0x62BA, 0x91);
	write_cmos_sensor_8(0x62BB, 0x1C);
	write_cmos_sensor_8(0x62BC, 0x03);
	write_cmos_sensor_8(0x62BD, 0x21);
	write_cmos_sensor_8(0x62BE, 0x83);
	write_cmos_sensor_8(0x62BF, 0x38);
	write_cmos_sensor_8(0x62D0, 0x14);
	write_cmos_sensor_8(0x62D1, 0x20);
	write_cmos_sensor_8(0x62D2, 0x67);
	write_cmos_sensor_8(0x62D4, 0x54);
	write_cmos_sensor_8(0x62D5, 0x40);
	write_cmos_sensor_8(0x62D6, 0xCE);
	write_cmos_sensor_8(0x62D8, 0xF8);
	write_cmos_sensor_8(0x62D9, 0x92);
	write_cmos_sensor_8(0x62DA, 0x9C);
	write_cmos_sensor_8(0x62DB, 0x02);
	write_cmos_sensor_8(0x62DC, 0xE1);
	write_cmos_sensor_8(0x62DD, 0x85);
	write_cmos_sensor_8(0x62DE, 0x38);
	write_cmos_sensor_8(0x637A, 0x11);
	write_cmos_sensor_8(0x7BA0, 0x01);
	write_cmos_sensor_8(0x7BA9, 0x00);
	write_cmos_sensor_8(0x7BAA, 0x01);
	write_cmos_sensor_8(0x7BAD, 0x00);
	write_cmos_sensor_8(0x886B, 0x00);
	write_cmos_sensor_8(0x9002, 0x00);
	write_cmos_sensor_8(0x9003, 0x00);
	write_cmos_sensor_8(0x9004, 0x09);
	write_cmos_sensor_8(0x9006, 0x01);
	write_cmos_sensor_8(0x9200, 0x93);
	write_cmos_sensor_8(0x9201, 0x85);
	write_cmos_sensor_8(0x9202, 0x93);
	write_cmos_sensor_8(0x9203, 0x87);
	write_cmos_sensor_8(0x9204, 0x93);
	write_cmos_sensor_8(0x9205, 0x8D);
	write_cmos_sensor_8(0x9206, 0x93);
	write_cmos_sensor_8(0x9207, 0x8F);
	write_cmos_sensor_8(0x9208, 0x6A);
	write_cmos_sensor_8(0x9209, 0x22);
	write_cmos_sensor_8(0x920A, 0x6A);
	write_cmos_sensor_8(0x920B, 0x23);
	write_cmos_sensor_8(0x920C, 0x6A);
	write_cmos_sensor_8(0x920D, 0x0F);
	write_cmos_sensor_8(0x920E, 0x71);
	write_cmos_sensor_8(0x920F, 0x03);
	write_cmos_sensor_8(0x9210, 0x71);
	write_cmos_sensor_8(0x9211, 0x0B);
	write_cmos_sensor_8(0x935D, 0x01);
	write_cmos_sensor_8(0x9389, 0x05);
	write_cmos_sensor_8(0x938B, 0x05);
	write_cmos_sensor_8(0x9391, 0x05);
	write_cmos_sensor_8(0x9393, 0x05);
	write_cmos_sensor_8(0x9395, 0x82);
	write_cmos_sensor_8(0x9397, 0x78);
	write_cmos_sensor_8(0x9399, 0x05);
	write_cmos_sensor_8(0x939B, 0x05);
	write_cmos_sensor_8(0xA91F, 0x04);
	write_cmos_sensor_8(0xA921, 0x03);
	write_cmos_sensor_8(0xA923, 0x02);
	write_cmos_sensor_8(0xA93D, 0x05);
	write_cmos_sensor_8(0xA93F, 0x03);
	write_cmos_sensor_8(0xA941, 0x02);
	write_cmos_sensor_8(0xA9AF, 0x04);
	write_cmos_sensor_8(0xA9B1, 0x03);
	write_cmos_sensor_8(0xA9B3, 0x02);
	write_cmos_sensor_8(0xA9CD, 0x05);
	write_cmos_sensor_8(0xA9CF, 0x03);
	write_cmos_sensor_8(0xA9D1, 0x02);
	write_cmos_sensor_8(0xAA3F, 0x04);
	write_cmos_sensor_8(0xAA41, 0x03);
	write_cmos_sensor_8(0xAA43, 0x02);
	write_cmos_sensor_8(0xAA5D, 0x05);
	write_cmos_sensor_8(0xAA5F, 0x03);
	write_cmos_sensor_8(0xAA61, 0x02);
	write_cmos_sensor_8(0xAACF, 0x04);
	write_cmos_sensor_8(0xAAD1, 0x03);
	write_cmos_sensor_8(0xAAD3, 0x02);
	write_cmos_sensor_8(0xAAED, 0x05);
	write_cmos_sensor_8(0xAAEF, 0x03);
	write_cmos_sensor_8(0xAAF1, 0x02);
	write_cmos_sensor_8(0xAB87, 0x04);
	write_cmos_sensor_8(0xAB89, 0x03);
	write_cmos_sensor_8(0xAB8B, 0x02);
	write_cmos_sensor_8(0xABA5, 0x05);
	write_cmos_sensor_8(0xABA7, 0x03);
	write_cmos_sensor_8(0xABA9, 0x02);
	write_cmos_sensor_8(0xABB7, 0x04);
	write_cmos_sensor_8(0xABB9, 0x03);
	write_cmos_sensor_8(0xABBB, 0x02);
	write_cmos_sensor_8(0xABD5, 0x05);
	write_cmos_sensor_8(0xABD7, 0x03);
	write_cmos_sensor_8(0xABD9, 0x02);
	write_cmos_sensor_8(0xB388, 0x28);
	write_cmos_sensor_8(0xBC40, 0x03);
	write_cmos_sensor_8(0x7B80, 0x00);
	write_cmos_sensor_8(0x7B81, 0x00);
	write_cmos_sensor_8(0xA900, 0x20);
	write_cmos_sensor_8(0xA901, 0x20);
	write_cmos_sensor_8(0xA902, 0x20);
	write_cmos_sensor_8(0xA903, 0x15);
	write_cmos_sensor_8(0xA904, 0x15);
	write_cmos_sensor_8(0xA905, 0x15);
	write_cmos_sensor_8(0xA906, 0x20);
	write_cmos_sensor_8(0xA907, 0x20);
	write_cmos_sensor_8(0xA908, 0x20);
	write_cmos_sensor_8(0xA909, 0x15);
	write_cmos_sensor_8(0xA90A, 0x15);
	write_cmos_sensor_8(0xA90B, 0x15);
	write_cmos_sensor_8(0xA915, 0x3F);
	write_cmos_sensor_8(0xA916, 0x3F);
	write_cmos_sensor_8(0xA917, 0x3F);
	write_cmos_sensor_8(0xA949, 0x03);
	write_cmos_sensor_8(0xA94B, 0x03);
	write_cmos_sensor_8(0xA94D, 0x03);
	write_cmos_sensor_8(0xA94F, 0x06);
	write_cmos_sensor_8(0xA951, 0x06);
	write_cmos_sensor_8(0xA953, 0x06);
	write_cmos_sensor_8(0xA955, 0x03);
	write_cmos_sensor_8(0xA957, 0x03);
	write_cmos_sensor_8(0xA959, 0x03);
	write_cmos_sensor_8(0xA95B, 0x06);
	write_cmos_sensor_8(0xA95D, 0x06);
	write_cmos_sensor_8(0xA95F, 0x06);
	write_cmos_sensor_8(0xA98B, 0x1F);
	write_cmos_sensor_8(0xA98D, 0x1F);
	write_cmos_sensor_8(0xA98F, 0x1F);
	write_cmos_sensor_8(0xAA21, 0x20);
	write_cmos_sensor_8(0xAA22, 0x20);
	write_cmos_sensor_8(0xAA24, 0x15);
	write_cmos_sensor_8(0xAA25, 0x15);
	write_cmos_sensor_8(0xAA26, 0x20);
	write_cmos_sensor_8(0xAA27, 0x20);
	write_cmos_sensor_8(0xAA28, 0x20);
	write_cmos_sensor_8(0xAA29, 0x15);
	write_cmos_sensor_8(0xAA2A, 0x15);
	write_cmos_sensor_8(0xAA2B, 0x15);
	write_cmos_sensor_8(0xAA35, 0x3F);
	write_cmos_sensor_8(0xAA36, 0x3F);
	write_cmos_sensor_8(0xAA37, 0x3F);
	write_cmos_sensor_8(0xAA6B, 0x03);
	write_cmos_sensor_8(0xAA6D, 0x03);
	write_cmos_sensor_8(0xAA71, 0x06);
	write_cmos_sensor_8(0xAA73, 0x06);
	write_cmos_sensor_8(0xAA75, 0x03);
	write_cmos_sensor_8(0xAA77, 0x03);
	write_cmos_sensor_8(0xAA79, 0x03);
	write_cmos_sensor_8(0xAA7B, 0x06);
	write_cmos_sensor_8(0xAA7D, 0x06);
	write_cmos_sensor_8(0xAA7F, 0x06);
	write_cmos_sensor_8(0xAAAB, 0x1F);
	write_cmos_sensor_8(0xAAAD, 0x1F);
	write_cmos_sensor_8(0xAAAF, 0x1F);
	write_cmos_sensor_8(0xAAB0, 0x20);
	write_cmos_sensor_8(0xAAB1, 0x20);
	write_cmos_sensor_8(0xAAB2, 0x20);
	write_cmos_sensor_8(0xAB53, 0x20);
	write_cmos_sensor_8(0xAB54, 0x20);
	write_cmos_sensor_8(0xAB55, 0x20);
	write_cmos_sensor_8(0xAB57, 0x40);
	write_cmos_sensor_8(0xAB59, 0x40);
	write_cmos_sensor_8(0xAB5B, 0x40);
	write_cmos_sensor_8(0xAB63, 0x03);
	write_cmos_sensor_8(0xAB65, 0x03);
	write_cmos_sensor_8(0xAB67, 0x03);
	write_cmos_sensor_8(0x9A00, 0x0C);
	write_cmos_sensor_8(0x9A01, 0x0C);
	write_cmos_sensor_8(0x9A06, 0x0C);
	write_cmos_sensor_8(0x9A18, 0x0C);
	write_cmos_sensor_8(0x9A19, 0x0C);
	write_cmos_sensor_8(0xAA20, 0x3F);
	write_cmos_sensor_8(0xAA23, 0x3F);
	write_cmos_sensor_8(0xAA32, 0x3F);
	write_cmos_sensor_8(0xAA69, 0x3F);
	write_cmos_sensor_8(0xAA6F, 0x3F);
	write_cmos_sensor_8(0xAAC2, 0x3F);
	write_cmos_sensor_8(0x8D1F, 0x00);
	write_cmos_sensor_8(0x8D27, 0x00);
	write_cmos_sensor_8(0x9963, 0x64);
	write_cmos_sensor_8(0x9964, 0x50);
	write_cmos_sensor_8(0xAC01, 0x0A);
	write_cmos_sensor_8(0xAC03, 0x0A);
	write_cmos_sensor_8(0xAC05, 0x0A);
	write_cmos_sensor_8(0xAC06, 0x01);
	write_cmos_sensor_8(0xAC07, 0xC0);
	write_cmos_sensor_8(0xAC09, 0xC0);
	write_cmos_sensor_8(0xAC17, 0x0A);
	write_cmos_sensor_8(0xAC19, 0x0A);
	write_cmos_sensor_8(0xAC1B, 0x0A);
	write_cmos_sensor_8(0xAC1C, 0x01);
	write_cmos_sensor_8(0xAC1D, 0xC0);
	write_cmos_sensor_8(0xAC1F, 0xC0);


}	/*	sensor_init  */


static void preview_setting(void)
{
	LOG_INF("imx351 E\n");
	write_cmos_sensor_8(0x0100, 0x00);   
	mdelay(50);

	write_cmos_sensor_8(0x0112, 0x0A);
	write_cmos_sensor_8(0x0113, 0x0A);
	write_cmos_sensor_8(0x0114, 0x03);
	write_cmos_sensor_8(0x0342, 0x0C);
	write_cmos_sensor_8(0x0343, 0x94);
	write_cmos_sensor_8(0x0340, 0x0C);
	write_cmos_sensor_8(0x0341, 0xE4);
	write_cmos_sensor_8(0x0344, 0x00);
	write_cmos_sensor_8(0x0345, 0x00);
	write_cmos_sensor_8(0x0346, 0x00);
	write_cmos_sensor_8(0x0347, 0x00);
	write_cmos_sensor_8(0x0348, 0x12);
	write_cmos_sensor_8(0x0349, 0x2F);
	write_cmos_sensor_8(0x034A, 0x0D);
	write_cmos_sensor_8(0x034B, 0x9F);
	write_cmos_sensor_8(0x0220, 0x00);
	write_cmos_sensor_8(0x0221, 0x11);
	write_cmos_sensor_8(0x0222, 0x01);
	write_cmos_sensor_8(0x0381, 0x01);
	write_cmos_sensor_8(0x0383, 0x01);
	write_cmos_sensor_8(0x0385, 0x01);
	write_cmos_sensor_8(0x0387, 0x01);
	write_cmos_sensor_8(0x0900, 0x01);
	write_cmos_sensor_8(0x0901, 0x22);
	write_cmos_sensor_8(0x0902, 0x0A);
	write_cmos_sensor_8(0x3243, 0x00);
	write_cmos_sensor_8(0x3F4C, 0x01);
	write_cmos_sensor_8(0x3F4D, 0x01);
	write_cmos_sensor_8(0x4254, 0x7F);
	write_cmos_sensor_8(0x0401, 0x00);
	write_cmos_sensor_8(0x0404, 0x00);
	write_cmos_sensor_8(0x0405, 0x10);
	write_cmos_sensor_8(0x0408, 0x00);
	write_cmos_sensor_8(0x0409, 0x00);
	write_cmos_sensor_8(0x040A, 0x00);
	write_cmos_sensor_8(0x040B, 0x00);
	write_cmos_sensor_8(0x040C, 0x09);
	write_cmos_sensor_8(0x040D, 0x18);
	write_cmos_sensor_8(0x040E, 0x06);
	write_cmos_sensor_8(0x040F, 0xD0);
	write_cmos_sensor_8(0x034C, 0x09);
	write_cmos_sensor_8(0x034D, 0x18);
	write_cmos_sensor_8(0x034E, 0x06);
	write_cmos_sensor_8(0x034F, 0xD0);
	write_cmos_sensor_8(0x0301, 0x05);
	write_cmos_sensor_8(0x0303, 0x04);
	write_cmos_sensor_8(0x0305, 0x02);
	write_cmos_sensor_8(0x0306, 0x00);
	write_cmos_sensor_8(0x0307, 0x85);
	write_cmos_sensor_8(0x030B, 0x02);
	write_cmos_sensor_8(0x030D, 0x02);
	write_cmos_sensor_8(0x030E, 0x01);
	write_cmos_sensor_8(0x030F, 0x5E);
	write_cmos_sensor_8(0x0310, 0x00);
	write_cmos_sensor_8(0x0820, 0x0C);
	write_cmos_sensor_8(0x0821, 0x78);
	write_cmos_sensor_8(0x0822, 0x00);
	write_cmos_sensor_8(0x0823, 0x00);
	write_cmos_sensor_8(0xBC41, 0x03);
	write_cmos_sensor_8(0x3E20, 0x01);
	write_cmos_sensor_8(0x3E37, 0x00);
	write_cmos_sensor_8(0x3E3B, 0x00);
	write_cmos_sensor_8(0x0106, 0x00);
	write_cmos_sensor_8(0x0B00, 0x00);
	write_cmos_sensor_8(0x3230, 0x00);
	write_cmos_sensor_8(0x3C00, 0x5B);
	write_cmos_sensor_8(0x3C01, 0x54);
	write_cmos_sensor_8(0x3C02, 0x77);
	write_cmos_sensor_8(0x3C03, 0x66);
	write_cmos_sensor_8(0x3C04, 0x00);
	write_cmos_sensor_8(0x3C05, 0x00);
	write_cmos_sensor_8(0x3C06, 0x14);
	write_cmos_sensor_8(0x3C07, 0x00);
	write_cmos_sensor_8(0x3C08, 0x01);
	write_cmos_sensor_8(0x3F14, 0x01);
	write_cmos_sensor_8(0x3F17, 0x00);
	write_cmos_sensor_8(0x3F3C, 0x01);
	write_cmos_sensor_8(0x3F78, 0x03);
	write_cmos_sensor_8(0x3F79, 0xBC);
	write_cmos_sensor_8(0x3F7C, 0x00);
	write_cmos_sensor_8(0x3F7D, 0x00);
	write_cmos_sensor_8(0x97C1, 0x04);
	write_cmos_sensor_8(0x97C5, 0x0C);
	write_cmos_sensor_8(0x0202, 0x0C);
	write_cmos_sensor_8(0x0203, 0xD0);
	write_cmos_sensor_8(0x0224, 0x01);
	write_cmos_sensor_8(0x0225, 0xF4);
	write_cmos_sensor_8(0x0204, 0x00);
	write_cmos_sensor_8(0x0205, 0x00);
	write_cmos_sensor_8(0x0216, 0x00);
	write_cmos_sensor_8(0x0217, 0x00);
	write_cmos_sensor_8(0x020E, 0x01);
	write_cmos_sensor_8(0x020F, 0x00);
	write_cmos_sensor_8(0x0218, 0x01);
	write_cmos_sensor_8(0x0219, 0x00);
	write_cmos_sensor_8(0x3614, 0x00);
	write_cmos_sensor_8(0x3616, 0x0E);
	write_cmos_sensor_8(0x3617, 0x66);
	write_cmos_sensor_8(0xB612, 0x2C);
	write_cmos_sensor_8(0xB613, 0x2C);
	write_cmos_sensor_8(0xB614, 0x1C);
	write_cmos_sensor_8(0xB615, 0x1C);
	write_cmos_sensor_8(0xB616, 0x06);
	write_cmos_sensor_8(0xB617, 0x06);
	write_cmos_sensor_8(0xB618, 0x20);
	write_cmos_sensor_8(0xB619, 0x20);
	write_cmos_sensor_8(0xB61A, 0x0C);
	write_cmos_sensor_8(0xB61B, 0x0C);
	write_cmos_sensor_8(0xB61C, 0x06);
	write_cmos_sensor_8(0xB61D, 0x06);
	write_cmos_sensor_8(0xB666, 0x39);
	write_cmos_sensor_8(0xB667, 0x39);
	write_cmos_sensor_8(0xB668, 0x39);
	write_cmos_sensor_8(0xB669, 0x39);
	write_cmos_sensor_8(0xB66A, 0x13);
	write_cmos_sensor_8(0xB66B, 0x13);
	write_cmos_sensor_8(0xB66C, 0x20);
	write_cmos_sensor_8(0xB66D, 0x20);
	write_cmos_sensor_8(0xB66E, 0x20);
	write_cmos_sensor_8(0xB66F, 0x20);
	write_cmos_sensor_8(0xB670, 0x10);
	write_cmos_sensor_8(0xB671, 0x10);
	write_cmos_sensor_8(0x3900, 0x00);
	write_cmos_sensor_8(0x3901, 0x00);
	write_cmos_sensor_8(0x3237, 0x00);
	write_cmos_sensor_8(0x30AC, 0x00);

	write_cmos_sensor_8(0x0100, 0x01);

	mdelay(10);
}	/*	preview_setting  */
//====================================================
// 3P3SP EVT0
// Full resolution
// x_output_size: 2304
// y_output_size: 1728
// frame_rate: 30.000
// output_format: RAW10
// output_interface: MIPI
// output_lanes: 4
// output_clock_mhz: 720.00
// system_clock_mhz: 280.00
// input_clock_mhz: 24.00
//
// $Rev$: Revision 0.00
// $Date$: 20151201
//====================================================
//$MV1[MCLK:24,Width:2304,Height:1728,Format:MIPI_RAW10,mipi_lane:4,mipi_datarate:720,pvi_pclk_inverse:0]

// Pll Setting - VCO = 280Mhz
static void capture_setting(kal_uint16 currefps, kal_bool stream_on)
{

	
	
	write_cmos_sensor_8(0x0100, 0x00);   
	mdelay(50);

	write_cmos_sensor_8(0x0112, 0x0A);
	write_cmos_sensor_8(0x0113, 0x0A);
	write_cmos_sensor_8(0x0114, 0x03);
	write_cmos_sensor_8(0x0342, 0x0C);
	write_cmos_sensor_8(0x0343, 0x94);
	write_cmos_sensor_8(0x0340, 0x0C);
	write_cmos_sensor_8(0x0341, 0xE4);
	write_cmos_sensor_8(0x0344, 0x00);
	write_cmos_sensor_8(0x0345, 0x00);
	write_cmos_sensor_8(0x0346, 0x00);
	write_cmos_sensor_8(0x0347, 0x00);
	write_cmos_sensor_8(0x0348, 0x12);
	write_cmos_sensor_8(0x0349, 0x2F);
	write_cmos_sensor_8(0x034A, 0x0D);
	write_cmos_sensor_8(0x034B, 0x9F);
	write_cmos_sensor_8(0x0220, 0x00);
	write_cmos_sensor_8(0x0221, 0x11);
	write_cmos_sensor_8(0x0222, 0x01);
	write_cmos_sensor_8(0x0381, 0x01);
	write_cmos_sensor_8(0x0383, 0x01);
	write_cmos_sensor_8(0x0385, 0x01);
	write_cmos_sensor_8(0x0387, 0x01);
	write_cmos_sensor_8(0x0900, 0x01);
	write_cmos_sensor_8(0x0901, 0x22);
	write_cmos_sensor_8(0x0902, 0x0A);
	write_cmos_sensor_8(0x3243, 0x00);
	write_cmos_sensor_8(0x3F4C, 0x01);
	write_cmos_sensor_8(0x3F4D, 0x01);
	write_cmos_sensor_8(0x4254, 0x7F);
	write_cmos_sensor_8(0x0401, 0x00);
	write_cmos_sensor_8(0x0404, 0x00);
	write_cmos_sensor_8(0x0405, 0x10);
	write_cmos_sensor_8(0x0408, 0x00);
	write_cmos_sensor_8(0x0409, 0x00);
	write_cmos_sensor_8(0x040A, 0x00);
	write_cmos_sensor_8(0x040B, 0x00);
	write_cmos_sensor_8(0x040C, 0x09);
	write_cmos_sensor_8(0x040D, 0x18);
	write_cmos_sensor_8(0x040E, 0x06);
	write_cmos_sensor_8(0x040F, 0xD0);
	write_cmos_sensor_8(0x034C, 0x09);
	write_cmos_sensor_8(0x034D, 0x18);
	write_cmos_sensor_8(0x034E, 0x06);
	write_cmos_sensor_8(0x034F, 0xD0);
	write_cmos_sensor_8(0x0301, 0x05);
	write_cmos_sensor_8(0x0303, 0x04);
	write_cmos_sensor_8(0x0305, 0x02);
	write_cmos_sensor_8(0x0306, 0x00);
	write_cmos_sensor_8(0x0307, 0x85);
	write_cmos_sensor_8(0x030B, 0x02);
	write_cmos_sensor_8(0x030D, 0x02);
	write_cmos_sensor_8(0x030E, 0x01);
	write_cmos_sensor_8(0x030F, 0x5E);
	write_cmos_sensor_8(0x0310, 0x00);
	write_cmos_sensor_8(0x0820, 0x0C);
	write_cmos_sensor_8(0x0821, 0x78);
	write_cmos_sensor_8(0x0822, 0x00);
	write_cmos_sensor_8(0x0823, 0x00);
	write_cmos_sensor_8(0xBC41, 0x03);
	write_cmos_sensor_8(0x3E20, 0x01);
	write_cmos_sensor_8(0x3E37, 0x00);
	write_cmos_sensor_8(0x3E3B, 0x00);
	write_cmos_sensor_8(0x0106, 0x00);
	write_cmos_sensor_8(0x0B00, 0x00);
	write_cmos_sensor_8(0x3230, 0x00);
	write_cmos_sensor_8(0x3C00, 0x5B);
	write_cmos_sensor_8(0x3C01, 0x54);
	write_cmos_sensor_8(0x3C02, 0x77);
	write_cmos_sensor_8(0x3C03, 0x66);
	write_cmos_sensor_8(0x3C04, 0x00);
	write_cmos_sensor_8(0x3C05, 0x00);
	write_cmos_sensor_8(0x3C06, 0x14);
	write_cmos_sensor_8(0x3C07, 0x00);
	write_cmos_sensor_8(0x3C08, 0x01);
	write_cmos_sensor_8(0x3F14, 0x01);
	write_cmos_sensor_8(0x3F17, 0x00);
	write_cmos_sensor_8(0x3F3C, 0x01);
	write_cmos_sensor_8(0x3F78, 0x03);
	write_cmos_sensor_8(0x3F79, 0xBC);
	write_cmos_sensor_8(0x3F7C, 0x00);
	write_cmos_sensor_8(0x3F7D, 0x00);
	write_cmos_sensor_8(0x97C1, 0x04);
	write_cmos_sensor_8(0x97C5, 0x0C);
	write_cmos_sensor_8(0x0202, 0x0C);
	write_cmos_sensor_8(0x0203, 0xD0);
	write_cmos_sensor_8(0x0224, 0x01);
	write_cmos_sensor_8(0x0225, 0xF4);
	write_cmos_sensor_8(0x0204, 0x00);
	write_cmos_sensor_8(0x0205, 0x00);
	write_cmos_sensor_8(0x0216, 0x00);
	write_cmos_sensor_8(0x0217, 0x00);
	write_cmos_sensor_8(0x020E, 0x01);
	write_cmos_sensor_8(0x020F, 0x00);
	write_cmos_sensor_8(0x0218, 0x01);
	write_cmos_sensor_8(0x0219, 0x00);
	write_cmos_sensor_8(0x3614, 0x00);
	write_cmos_sensor_8(0x3616, 0x0E);
	write_cmos_sensor_8(0x3617, 0x66);
	write_cmos_sensor_8(0xB612, 0x2C);
	write_cmos_sensor_8(0xB613, 0x2C);
	write_cmos_sensor_8(0xB614, 0x1C);
	write_cmos_sensor_8(0xB615, 0x1C);
	write_cmos_sensor_8(0xB616, 0x06);
	write_cmos_sensor_8(0xB617, 0x06);
	write_cmos_sensor_8(0xB618, 0x20);
	write_cmos_sensor_8(0xB619, 0x20);
	write_cmos_sensor_8(0xB61A, 0x0C);
	write_cmos_sensor_8(0xB61B, 0x0C);
	write_cmos_sensor_8(0xB61C, 0x06);
	write_cmos_sensor_8(0xB61D, 0x06);
	write_cmos_sensor_8(0xB666, 0x39);
	write_cmos_sensor_8(0xB667, 0x39);
	write_cmos_sensor_8(0xB668, 0x39);
	write_cmos_sensor_8(0xB669, 0x39);
	write_cmos_sensor_8(0xB66A, 0x13);
	write_cmos_sensor_8(0xB66B, 0x13);
	write_cmos_sensor_8(0xB66C, 0x20);
	write_cmos_sensor_8(0xB66D, 0x20);
	write_cmos_sensor_8(0xB66E, 0x20);
	write_cmos_sensor_8(0xB66F, 0x20);
	write_cmos_sensor_8(0xB670, 0x10);
	write_cmos_sensor_8(0xB671, 0x10);
	write_cmos_sensor_8(0x3900, 0x00);
	write_cmos_sensor_8(0x3901, 0x00);
	write_cmos_sensor_8(0x3237, 0x00);
	write_cmos_sensor_8(0x30AC, 0x00);

	write_cmos_sensor_8(0x0100, 0x01);

	mdelay(10);
	

//#endif

}

static void normal_video_setting(kal_uint16 currefps)
	{
		LOG_INF("imx351 E\n");
		preview_setting();	
	}

static void hs_video_setting(void)
{
	LOG_INF("imx351 E\n");
	preview_setting();  
}

static void slim_video_setting(void)
{
	LOG_INF("imx351 E\n");
	preview_setting();
}



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
			*sensor_id = ((read_cmos_sensor_8(0x0016) << 8) | read_cmos_sensor_8(0x0017));
			printk("imx351 sensor id is %d\n",*sensor_id);
			LOG_INF("imx351 read_0x0000=0x%x, 0x0001=0x%x,0x0000_0001=0x%x\n",read_cmos_sensor_8(0x0000),read_cmos_sensor_8(0x0001),read_cmos_sensor(0x0000));
			if (*sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("imx351 i2c write id: 0x%x, 351sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
			
				return ERROR_NONE;
			}
			LOG_INF("imx351 Read sensor id fail, i2c_write_id: 0x%x .351sensor_id: 0x%x \n", imgsensor.i2c_write_id,*sensor_id);
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
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 sensor_id = 0;
	LOG_INF("imx351 PLATFORM:MT6595,MIPI 2LANE\n");
	LOG_INF("imx351 preview 1280*960@30fps,864Mbps/lane; video 1280*960@30fps,864Mbps/lane; capture 5M@30fps,864Mbps/lane\n");


	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
            sensor_id = ((read_cmos_sensor_8(0x0016) << 8) | read_cmos_sensor_8(0x0017));
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("imx351 i2c write id: 0x%x,open351 sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
				break;
			}
			LOG_INF("imx351 Read sensor id fail, id: 0x%x sensor_id=0x%x\n ", imgsensor.i2c_write_id,sensor_id);
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
	imgsensor.shutter = 0x3D0;
	imgsensor.gain = 0x100;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_mode = 0;
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
	LOG_INF("imx351 E\n");

	/*No Need to implement this function*/

	return ERROR_NONE;
}	/*	close  */

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("imx219 enable: %d\n", enable);
	if(imgsensor.current_scenario_id != MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG)
	   {
		   if(enable) 
		   {   
			   //1640 x 1232
			   // enable color bar
			   test_pattern_flag=KAL_TRUE;
			   write_cmos_sensor(0x0600, 0x00); 
			   write_cmos_sensor(0x0601, 0x02); 	 
			   write_cmos_sensor(0x0624, 0x06); //W:3280---h
			   write_cmos_sensor(0x0625, 0x68); //		  l
			   write_cmos_sensor(0x0626, 0x04); //H:2464   h
			   write_cmos_sensor(0x0627, 0xd0); //		  l
			   write_cmos_sensor(0x6128, 0x00); 
			   write_cmos_sensor(0x6129, 0x02); 		 
			   write_cmos_sensor(0x613C, 0x06); //W 		h
			   write_cmos_sensor(0x613D, 0x68); //		   l
			   write_cmos_sensor(0x613E, 0x04); //H 		h
			   write_cmos_sensor(0x613F, 0xd0); //			   l
			   write_cmos_sensor(0x6506, 0x00); 
			   write_cmos_sensor(0x6507, 0x00);
	
		   } 
		   else 
		   {   
			   //1640 x 1232
			   test_pattern_flag=KAL_FALSE;
			   write_cmos_sensor(0x0600, 0x00); 
			   write_cmos_sensor(0x0601, 0x00); 	 
			   write_cmos_sensor(0x0624, 0x06); //W:3280---h
			   write_cmos_sensor(0x0625, 0x68); //		  l
			   write_cmos_sensor(0x0626, 0x04); //H:2464   h
			   write_cmos_sensor(0x0627, 0xd0); //		  l
			   write_cmos_sensor(0x6128, 0x00); 
			   write_cmos_sensor(0x6129, 0x02); 		 
			   write_cmos_sensor(0x613C, 0x06); //W 		h
			   write_cmos_sensor(0x613D, 0x68); //		   l
			   write_cmos_sensor(0x613E, 0x04); //H 		h
			   write_cmos_sensor(0x613F, 0xd0); //			   l
			   write_cmos_sensor(0x6506, 0x00); 
			   write_cmos_sensor(0x6507, 0x00);
	
		   }
	   }
	   else
	   {
		   if(enable) 
		   {   
			   //3280 x 2464
			   // enable color bar
			   test_pattern_flag=KAL_TRUE;
			   write_cmos_sensor(0x0600, 0x00); 
			   write_cmos_sensor(0x0601, 0x02); 	 
			   write_cmos_sensor(0x0624, 0x0C); //W:3280---h
			   write_cmos_sensor(0x0625, 0xD0); //		  l
			   write_cmos_sensor(0x0626, 0x09); //H:2464   h
			   write_cmos_sensor(0x0627, 0xA0); //		  l
			   write_cmos_sensor(0x6128, 0x00); 
			   write_cmos_sensor(0x6129, 0x02); 		 
			   write_cmos_sensor(0x613C, 0x0C); //W 		h
			   write_cmos_sensor(0x613D, 0xD0); //		   l
			   write_cmos_sensor(0x613E, 0x09); //H 		h
			   write_cmos_sensor(0x613F, 0xA0); //			   l
			   write_cmos_sensor(0x6506, 0x00); 
			   write_cmos_sensor(0x6507, 0x00);
	
		   } 
		   else 
		   {   
			   test_pattern_flag=KAL_FALSE;
			   write_cmos_sensor(0x0600, 0x00); 
			   write_cmos_sensor(0x0601, 0x02); 	 
			   write_cmos_sensor(0x0624, 0x0C); //W:3280---h
			   write_cmos_sensor(0x0625, 0xD0); //		  l
			   write_cmos_sensor(0x0626, 0x09); //H:2464   h
			   write_cmos_sensor(0x0627, 0xA0); //		  l
			   write_cmos_sensor(0x6128, 0x00); 
			   write_cmos_sensor(0x6129, 0x02); 		 
			   write_cmos_sensor(0x613C, 0x0C); //W 		h
			   write_cmos_sensor(0x613D, 0xD0); //		   l
			   write_cmos_sensor(0x613E, 0x09); //H 		h
			   write_cmos_sensor(0x613F, 0xA0); //			   l
			   write_cmos_sensor(0x6506, 0x00); 
			   write_cmos_sensor(0x6507, 0x00);
	
	
		   }
	   }
		   
	   return ERROR_NONE;

	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

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
	LOG_INF("imx351 E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	preview_setting();
	set_mirror_flip(imgsensor.mirror);

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
	LOG_INF("imx351 E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {//PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}  else if(imgsensor.current_fps == imgsensor_info.cap2.max_framerate){
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
			LOG_INF("imx351 Warning: current_fps %d fps is not support, so use cap1's setting: %d fps!\n",imgsensor.current_fps,imgsensor_info.cap1.max_framerate/10);
		imgsensor.pclk = imgsensor_info.cap2.pclk;
		imgsensor.line_length = imgsensor_info.cap2.linelength;
		imgsensor.frame_length = imgsensor_info.cap2.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap2.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}else {
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
			LOG_INF("imx351 Warning: current_fps %d fps is not support, so use cap1's setting: %d fps!\n",imgsensor.current_fps,imgsensor_info.cap1.max_framerate/10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);


	capture_setting(imgsensor.current_fps, 1);

	set_mirror_flip(imgsensor.mirror);

	if(test_pattern_flag)
	{
		set_test_pattern_mode(KAL_TRUE);
		test_pattern_flag=KAL_FALSE;
	}

	return ERROR_NONE;
}	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("imx351 E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);


	normal_video_setting(imgsensor.current_fps);

	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("imx351 E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	hs_video_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("imx351 E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	slim_video_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	slim_video	 */



static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT(*sensor_resolution))
{
	LOG_INF("imx351 E\n");
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
	LOG_INF("imx351 scenario_id = %d\n", scenario_id);

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
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	sensor_info->PDAF_Support = 0;
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
	LOG_INF("imx351 scenario_id = %d\n", scenario_id);
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
			LOG_INF("imx351 Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("imx351 framerate = %d\n ", framerate);
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
	LOG_INF("imx351 enable = %d, framerate = %d \n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) //enable auto flicker
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("imx351 scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
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
			set_dummy();
			break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        	  if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
                frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            } else if (imgsensor.current_fps == imgsensor_info.cap2.max_framerate) {
                frame_length = imgsensor_info.cap2.pclk / framerate * 10 / imgsensor_info.cap2.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap2.framelength) ? (frame_length - imgsensor_info.cap2.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap2.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            } else {
        		    if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
                    LOG_INF("imx351 Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",framerate,imgsensor_info.cap.max_framerate/10);
                frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            }
            set_dummy();
            break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;
			imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			break;
		default:  //coding with  preview scenario by default
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			LOG_INF("imx351 error scenario_id = %d, we use preview scenario \n", scenario_id);
			break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	LOG_INF("imx351 scenario_id = %d\n", scenario_id);

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


static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
							 UINT8 *feature_para,UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16=(UINT16 *) feature_para;
	UINT16 *feature_data_16=(UINT16 *) feature_para;
	UINT32 *feature_return_para_32=(UINT32 *) feature_para;
	UINT32 *feature_data_32=(UINT32 *) feature_para;
    unsigned long long *feature_data=(unsigned long long *) feature_para;

    struct SET_PD_BLOCK_INFO_T *PDAFinfo;
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;


	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	LOG_INF("imx351 feature_id = %d\n", feature_id);
	switch (feature_id) {
		case SENSOR_FEATURE_GET_PERIOD:
			*feature_return_para_16++ = imgsensor.line_length;
			*feature_return_para_16 = imgsensor.frame_length;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            LOG_INF("imx351 feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n", imgsensor.pclk,imgsensor.current_fps);
			*feature_return_para_32 = imgsensor.pclk;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_ESHUTTER:
            set_shutter(*feature_data);
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
                     //night_mode((BOOL) *feature_data);
			break;
		case SENSOR_FEATURE_SET_GAIN:
            set_gain((UINT16) *feature_data);
			break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
			break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
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
			//set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
			set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*feature_data,*(feature_data + 1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
			break;
		//case SENSOR_FEATURE_GET_PDAF_DATA:
			//LOG_FLY("Fly SENSOR_FEATURE_GET_PDAF_DATA\n");
		//	read_ov12870_rear0_eeprom((kal_uint16 )(*feature_data),(char*)(uintptr_t)(*(feature_data+1)),(kal_uint32)(*(feature_data+2)));
		//	break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((BOOL)*feature_data);
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
			*feature_return_para_32 = imgsensor_info.checksum_value;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_FRAMERATE:
            LOG_INF("imx351 current fps :%d\n", (UINT32)*feature_data);
			spin_lock(&imgsensor_drv_lock);
                     imgsensor.current_fps = *feature_data;
			spin_unlock(&imgsensor_drv_lock);
			break;
		case SENSOR_FEATURE_SET_HDR:
            LOG_INF("imx351 ihdr enable :%d\n", (BOOL)*feature_data);
			spin_lock(&imgsensor_drv_lock);
			imgsensor.ihdr_mode = *feature_data;
			spin_unlock(&imgsensor_drv_lock);
			break;
		case SENSOR_FEATURE_GET_CROP_INFO:
            //LOG_INF("imx351 SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);
            wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

			switch (*feature_data_32) {
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
		//	LOG_INF("imx351 SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n", *feature_data);
			PDAFinfo= (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));

			switch (*feature_data) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					//memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(SET_PD_BLOCK_INFO_T));
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
		//	LOG_INF("imx351 SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%d\n", *feature_data);
			//PDAF capacity enable or not, 2p8 only full size support PDAF
			switch (*feature_data) {
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

		case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
            LOG_INF("imx351 SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            //ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
			break;
        case SENSOR_FEATURE_SET_AWB_GAIN:
            break;
        case SENSOR_FEATURE_SET_HDR_SHUTTER:
            LOG_INF("imx351 SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1));
            //ihdr_write_shutter((UINT16)*feature_data,(UINT16)*(feature_data+1));
            break;
       
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
}	/*	feature_control()  */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 IMX351_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}	/*	imx351_MIPI_RAW_SensorInit	*/
