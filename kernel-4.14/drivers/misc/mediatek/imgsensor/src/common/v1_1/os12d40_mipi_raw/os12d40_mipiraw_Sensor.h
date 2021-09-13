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

/*****************************************************************************
 *
 * Filename:
 * ---------
 *     OS12D40mipiraw_Sensor.h
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 CMOS sensor header file
 *
 ****************************************************************************/
#ifndef _OS12D40MIPIRAW_SENSOR_H
#define _OS12D40MIPIRAW_SENSOR_H

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#define OS12D40_EEPROM_ENABLE 1
#if OS12D40_EEPROM_ENABLE
#define OS12D40_EEPROM_I2C_ADDR 0xA2
//#include <linux/hardware_info.h>
#define EEPROM_CROSSTALK_FLAG_ADDR 0x0782
#define EEPROM_CROSSTALK_LENGTH 288
#define EEPROM_CROSSTALK_CHECKSUM_ADDR 0x08A3
#define SENSOR_CROSSTALK_W_ADDR 0x53C0
#define EEPROM_PDC_FLAG_ADDR 0x08A4
#define EEPROM_PDC_LENGTH 732
#define EEPROM_PDC_CHECKSUM_ADDR 0x0B81
#define SENSOR_PDC_W_GAIN_CURVE_RATIO_ADDR 0x5C0E
#define SENSOR_PDC_W_GAIN_CURVE_RATIO_LENGTH 8
#define SENSOR_PDC_W_COMPENSATION_GAIN_ADDR 0x5900
#define SENSOR_PDC_W_COMPENSATION_GAIN_LENGTH 180
#define SENSOR_PDC_W_SHADOW_GAIN_ADDR 0x59B4
#define SENSOR_PDC_W_SHADOW_GAIN_LENGTH 540
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
        IMGSENSOR_MODE_CUSTOM3,
        IMGSENSOR_MODE_CUSTOM4,
        IMGSENSOR_MODE_CUSTOM5,
  
};

struct imgsensor_mode_struct {
	kal_uint32 pclk;
	kal_uint32 linelength;
	kal_uint32 framelength;

	kal_uint8 startx;
	kal_uint8 starty;

	kal_uint16 grabwindow_width;
	kal_uint16 grabwindow_height;

	/* MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario */
	kal_uint8 mipi_data_lp2hs_settle_dc;

	/*	 following for GetDefaultFramerateByScenario()	*/
	kal_uint16 max_framerate;
	kal_uint32 mipi_pixel_rate;
};

/* SENSOR PRIVATE STRUCT FOR VARIABLES*/
struct imgsensor_struct {
	kal_uint8 mirror;				//mirrorflip information

	kal_uint8 sensor_mode; //record IMGSENSOR_MODE enum value

	kal_uint32 shutter;				//current shutter
	kal_uint16 gain;				//current gain

	kal_uint32 pclk;				//current pclk

	kal_uint32 frame_length;		//current framelength
	kal_uint32 line_length;			//current linelength

	kal_uint32 min_frame_length; //current min  framelength to max
	kal_int32 dummy_pixel;			//current dummypixel
	kal_int32 dummy_line;			//current dummline
	kal_uint16 current_fps;			//current max fps
	kal_bool   autoflicker_en; //record autoflicker enable or disable
	kal_bool test_pattern; //record test pattern mode or not
	enum MSDK_SCENARIO_ID_ENUM current_scenario_id;//current scenario id
	kal_uint8  ihdr_en;				//ihdr enable or disable
	kal_uint8  pdaf_mode;				//ihdr enable or disable
	kal_uint8 i2c_write_id; //record current sensor's i2c write id
	struct IMGSENSOR_AE_FRM_MODE ae_frm_mode;//
	kal_uint8 current_ae_effective_frame;
};

/* SENSOR PRIVATE STRUCT FOR CONSTANT*/
struct imgsensor_info_struct {
	kal_uint32 sensor_id; //record sensor id defined in Kd_imgsensor.h
        kal_uint32 module_id;
	kal_uint32 checksum_value; //checksum value for Camera Auto Test
	struct imgsensor_mode_struct pre; //preview scenario information
	struct imgsensor_mode_struct cap; //capture scenario  information
	struct imgsensor_mode_struct cap1; //capture for PIP 24fps info
	//normal video  scenario  information
	struct imgsensor_mode_struct normal_video;
	//high speed video scenario  information
	struct imgsensor_mode_struct hs_video;
        struct imgsensor_mode_struct slim_video;
	//slim video for VT scenario  information
	struct imgsensor_mode_struct custom1;
	struct imgsensor_mode_struct custom2;
	struct imgsensor_mode_struct custom3;
	struct imgsensor_mode_struct custom4;
	struct imgsensor_mode_struct custom5;

        kal_uint8 margin;					/* sensor framelength & shutter margin */
	kal_uint8 min_shutter;				/* min shutter */
	kal_uint8 min_gain; /*1x gain*/
	kal_uint32 max_gain; /*64x gain*/
	kal_uint8 min_gain_iso;
	kal_uint8 gain_step;
	kal_uint8 gain_type;/*to be modify,no gain table for sony*/
	kal_uint32 max_frame_length;

	kal_uint8  ae_shut_delay_frame;	//shutter delay frame for AE cycle
	//sensor gain delay frame for AE cycle
	kal_uint8  ae_sensor_gain_delay_frame;
	kal_uint8  ae_ispGain_delay_frame; //ispgaindelayframe for AEcycle
	kal_uint8  ihdr_support;		//
	kal_uint8  ihdr_le_firstline;	//
	kal_uint8 temperature_support;    /* 1, support; 0,not support */
	kal_uint8  sensor_mode_num;		//support sensor mode num

	kal_uint8  cap_delay_frame; //enter capture delay frame num
	kal_uint8  pre_delay_frame; //enter preview delay frame num
	kal_uint8  video_delay_frame; //enter video delay frame num
	kal_uint8  hs_video_delay_frame; //enter hspeedvideo delayframe num
	kal_uint8  slim_video_delay_frame; //enter svideo delay frame num

	kal_uint8 custom1_delay_frame;	//enter custom1 delay frame num
	kal_uint8 custom2_delay_frame;		//enter custom2 delay frame num
	kal_uint8 custom3_delay_frame;		//enter custom3 delay frame num
	kal_uint8 custom4_delay_frame;		//enter custom4 delay frame num
	kal_uint8 custom5_delay_frame;		//enter custom5 delay frame num
	kal_uint8 frame_time_delay_frame;

	kal_uint8  isp_driving_current;	//mclk driving current
	kal_uint8  sensor_interface_type;//sensor_interface_type

	kal_uint8  mipi_sensor_type;
	kal_uint8  mipi_settle_delay_mode;
	kal_uint8  sensor_output_dataformat;
	kal_uint8  mclk; //mclk value, suggest 24Mhz or 26Mhz

	kal_uint8  mipi_lane_num;		//mipi lane num

	kal_uint8  i2c_addr_table[5];
	kal_uint32  i2c_speed; //khz
};

extern int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData,
		u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iReadRegI2CTiming(u8 *a_pSendData, u16 a_sizeSendData,
		u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId, u16 timing);
extern int iWriteRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u16 i2cId);
extern int iWriteRegI2CTiming(u8 *a_pSendData, u16 a_sizeSendData,
		u16 i2cId, u16 timing);
extern int iBurstWriteReg_multi(u8 *pData, u32 bytes, u16 i2cId,
		u16 transfer_length, u16 timing);
extern int iBurstWriteReg(u8 *pData, u32 bytes, u16 i2cId);
extern void register_imgsensor_deviceinfo(char *name,
					char *version, u8 module_id);

#endif
