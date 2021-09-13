/*****************************************************************************
 *
 * Filename:
 * ---------
 *     gc2385mipi_Sensor.h
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     CMOS sensor header file
 *
 ****************************************************************************/
#ifndef _GC2385MIPI_SENSOR_H
#define _GC2385MIPI_SENSOR_H

/* SENSOR MIRROR FLIP INFO */
#define GC2385_MIRROR_FLIP_ENABLE    1
#if GC2385_MIRROR_FLIP_ENABLE
#define GC2385_MIRROR        0xd7
#define GC2385_STARTY        0x04
#define GC2385_STARTX        0x06
#define GC2385_BLK_Select_H  0x00
#define GC2385_BLK_Select_L  0x3c
#else
#define GC2385_MIRROR        0xd4
#define GC2385_STARTY        0x03
#define GC2385_STARTX        0x05
#define GC2385_BLK_Select_H  0x3c
#define GC2385_BLK_Select_L  0x00
#endif

enum IMGSENSOR_MODE {
	IMGSENSOR_MODE_INIT,
	IMGSENSOR_MODE_PREVIEW,
	IMGSENSOR_MODE_CAPTURE,
	IMGSENSOR_MODE_VIDEO,
	IMGSENSOR_MODE_HIGH_SPEED_VIDEO,
	IMGSENSOR_MODE_SLIM_VIDEO,
    IMGSENSOR_MODE_CUSTOM1,
    IMGSENSOR_MODE_CUSTOM2,	
};

struct imgsensor_mode_struct {
	kal_uint32 pclk;	/* record different mode's pclk */
	kal_uint32 linelength;	/* record different mode's linelength */
	kal_uint32 framelength;	/* record different mode's framelength */

	kal_uint8 startx;	/* record different mode's startx of grabwindow */
	kal_uint8 starty;	/* record different mode's startx of grabwindow */

	kal_uint16 grabwindow_width;	/* record different mode's width of grabwindow */
	kal_uint16 grabwindow_height;	/* record different mode's height of grabwindow */

	/*       following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario   */
	kal_uint8 mipi_data_lp2hs_settle_dc;
	kal_uint32 mipi_pixel_rate;
	kal_uint16 max_framerate;
};

/* SENSOR PRIVATE STRUCT FOR VARIABLES*/
struct imgsensor_struct {
	kal_uint8 mirror;
	kal_uint8 sensor_mode;
	kal_uint32 shutter;
	kal_uint16 gain;
	kal_uint32 pclk;
	kal_uint32 frame_length;
	kal_uint32 line_length;
	kal_uint32 min_frame_length;
	kal_uint16 dummy_pixel;
	kal_uint16 dummy_line;
	kal_uint16 current_fps;
	kal_bool   autoflicker_en;
	kal_bool   test_pattern;
	enum MSDK_SCENARIO_ID_ENUM current_scenario_id;//current scenario id
	kal_uint8  ihdr_en;
	kal_uint8 i2c_write_id;
};

/* SENSOR PRIVATE STRUCT FOR CONSTANT */
struct imgsensor_info_struct {
	kal_uint32 sensor_id;
	kal_uint32 checksum_value;
	struct imgsensor_mode_struct pre;	/* preview scenario relative information */
	struct imgsensor_mode_struct cap;	/* capture scenario relative information */
	struct imgsensor_mode_struct cap1;	/* capture for PIP 24fps relative information,
					 * capture1 mode must use same framelength,
					 * linelength with Capture mode for shutter calculate
					 */
	struct imgsensor_mode_struct normal_video;	/* normal video  scenario relative information */
	struct imgsensor_mode_struct hs_video;	/* high speed video scenario relative information */
	struct imgsensor_mode_struct slim_video;	/* slim video for VT scenario relative information */
    struct imgsensor_mode_struct custom1;      //custom1 scenario relative information
    struct imgsensor_mode_struct custom2;      //custom2 scenario relative information
	kal_uint8  ae_shut_delay_frame;
	kal_uint8  ae_sensor_gain_delay_frame;
	kal_uint8  ae_ispGain_delay_frame;
    kal_uint8 frame_time_delay_frame;	/* The delay frame of setting frame length  */
	kal_uint8  ihdr_support;
	kal_uint8  ihdr_le_firstline;
	kal_uint8  sensor_mode_num;
	kal_uint8  cap_delay_frame;
	kal_uint8  pre_delay_frame;
	kal_uint8  video_delay_frame;
	kal_uint8  hs_video_delay_frame;
	kal_uint8  slim_video_delay_frame;
    kal_uint8  custom1_delay_frame;     //enter custom1 delay frame num
    kal_uint8  custom2_delay_frame;     //enter custom1 delay frame num
	// huangjiwu for  captrue black --begin 
	kal_uint8 temperature_support;	/* 1, support; 0,not support */
	kal_uint32 min_gain;
	kal_uint32 max_gain;
	kal_uint32 min_gain_iso;
	kal_uint32 gain_step;
	kal_uint32 exp_step;
	kal_uint32 gain_type;
	//huangjiwu for  captrue black --end
	kal_uint8  margin;
	kal_uint32 min_shutter;
	kal_uint32 max_frame_length;
	kal_uint8  isp_driving_current;
	kal_uint8  sensor_interface_type;
	kal_uint8  mipi_sensor_type;
	kal_uint8  mipi_settle_delay_mode;
	kal_uint8  sensor_output_dataformat;
	kal_uint8  mclk;
	kal_uint8  mipi_lane_num;
	kal_uint8  i2c_addr_table[5];
};

extern int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u16 i2cId);
extern int iWriteReg(u16 a_u2Addr, u32 a_u4Data, u32 a_u4Bytes, u16 i2cId);
extern void kdSetI2CSpeed(u16 i2cSpeed);

#endif
