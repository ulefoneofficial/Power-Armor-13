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



#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>  
//#include <asm/system.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "s5k4h7yxsubmipiraw_Sensor.h"

#define USHORT             unsigned short
#define BYTE               unsigned char
#define S5K4H7SUB_OTP_LSCFLAG_ADDR	0x0A3D
#define S5K4H7SUB_OTP_FLAGFLAG_ADDR	0x0A04
#define S5K4H7SUB_OTP_AWBFLAG_ADDR	0x0A12 // for group1
#define S5K4H7SUB_LSC_PAGE		         0
#define S5K4H7SUB_FLAG_PAGE		    21
#define S5K4H7SUB_AWB_PAGE		        21

typedef struct {
	unsigned short	infoflag;
	unsigned short	lsc_infoflag;
	unsigned short	flag_infoflag;
	unsigned short	flag_module_integrator_id;
	int		awb_offset;
	int		flag_offset;
	int		lsc_offset;
	int 		lsc_group;
	int 		flag_group;
	int 		group;
	unsigned short	frgcur;
	unsigned short	fbgcur;
	unsigned int	nr_gain;
	unsigned int	ng_gain;
	unsigned int	nb_gain;
	unsigned int	ngrcur;
	unsigned int	ngbcur;
	unsigned int	ngcur;
	unsigned int	nrcur;
	unsigned int	nbcur;
	unsigned int	nggolden;
	unsigned int	nrgolden;
	unsigned int	nbgolden;
	unsigned int	ngrgolden;
	unsigned int	ngbgolden;
	unsigned int	frggolden;
	unsigned int	fbggolden;
	unsigned int	awb_flag_sum;
	unsigned int	lsc_sum;
	unsigned int	lsc_check_flag;
}OTP;

OTP otp_data_info_sub = {0};

#define GAIN_DEFAULT       0x0100

static USHORT R_GAIN;
static USHORT B_GAIN;
static USHORT Gr_GAIN;
static USHORT Gb_GAIN;
static USHORT G_GAIN;

/**********************************************************
 * get_4h7sub_page_data
 * get page data
 * return true or false
 * *******************************************************/
void get_4h7sub_page_data(int pageidx, unsigned char *pdata)
{
	unsigned short get_byte=0;
	unsigned int addr = 0x0A04;
	int i = 0;
	otp_4h7sub_write_cmos_sensor_8(0x0A02,pageidx);
	otp_4h7sub_write_cmos_sensor_8(0x0A00,0x01);

	do
	{
		mdelay(1);
		get_byte = otp_4h7_read_cmos_subsensor(0x0A01);
	}while((get_byte & 0x01) != 1);

	for(i = 0; i < 64; i++){
		pdata[i] = otp_4h7_read_cmos_subsensor(addr);
		addr++;
	}

	otp_4h7sub_write_cmos_sensor_8(0x0A00,0x00);
}

unsigned short selective_read_region_sub(int pageidx,unsigned int addr)
{
	unsigned short get_byte = 0;
	otp_4h7sub_write_cmos_sensor_8(0x0A02,pageidx);
	otp_4h7sub_write_cmos_sensor_8(0x0A00,0x01);
	do
	{
		mdelay(1);
		get_byte = otp_4h7_read_cmos_subsensor(0x0A01);
	}while((get_byte & 0x01) != 1);

	get_byte = otp_4h7_read_cmos_subsensor(addr);
	otp_4h7sub_write_cmos_sensor_8(0x0A00,0x00);

	return get_byte;
}

unsigned int selective_read_region_sub_16_sub(int pageidx,unsigned int addr)
{
	unsigned int get_byte = 0;
	static int old_pageidx = 0;
	if(pageidx != old_pageidx){
		otp_4h7sub_write_cmos_sensor_8(0x0A00,0x00);
		otp_4h7sub_write_cmos_sensor_8(0x0A02,pageidx);
		otp_4h7sub_write_cmos_sensor_8(0x0A00,0x01);
		do
		{
			mdelay(1);
			get_byte = otp_4h7_read_cmos_subsensor(0x0A01);
		}while((get_byte & 0x01) != 1);
	}
    //prize fengshangdong at 20190127
	//get_byte = ((otp_4h7_read_cmos_subsensor(addr) << 8) | otp_4h7_read_cmos_subsensor(addr+1));
	get_byte = otp_4h7_read_cmos_subsensor(addr);
	//prize end
	old_pageidx = pageidx;
	return get_byte;
}

/*****************************************************
 * cal_rgb_gain
 * **************************************************/

#if 0
void cal_rgb_gain(int* r_gain, int* g_gain, int* b_gain, unsigned int r_ration, unsigned int b_ration)
{
	int gain_default = 0x0100;
	if(r_ration >= 1){
		if(b_ration >= 1){
			*g_gain = gain_default;
			*r_gain = (int)((gain_default*1000 * r_ration + 500)/1000);
			*b_gain = (int)((gain_default*1000 * b_ration + 500)/1000);
		}
		else{
			*b_gain = gain_default;
			*g_gain = (int)((gain_default * 1000 / b_ration + 500)/1000);
			*r_gain = (int)((gain_default * r_ration *1000 / b_ration + 500)/1000);
		}
	}
	else{
		if(b_ration >= 1){
			*r_gain = gain_default;
			*g_gain = (int)((gain_default * 1000 / r_ration + 500)/1000);
			*b_gain = (int)((gain_default * b_ration*1000 / r_ration + 500) / 1000);
		}
		else{
			if(r_ration >= b_ration){
				*b_gain = gain_default;
				*g_gain = (int)((gain_default * 1000 / b_ration + 500) / 1000);
				*r_gain = (int)((gain_default * r_ration * 1000 / b_ration + 500) / 1000);
			}
			else{
				*r_gain = gain_default;
				*g_gain = (int)((gain_default * 1000 / r_ration + 500)/1000);
				*b_gain = (int)((gain_default * b_ration * 1000 / r_ration + 500) / 1000);
			}
		}
	}
}
#endif
#if 0
if (r_ratio >= 512)
	{
		if (b_ratio >= 512) 
		{
			R_GAIN = (USHORT)((GAIN_DEFAULT*r_ratio)/512);
			G_GAIN = GAIN_DEFAULT;
			B_GAIN = (USHORT)((GAIN_DEFAULT*b_ratio)/512);
		} else
		{
			R_GAIN = (USHORT)((GAIN_DEFAULT*r_ratio)/b_ratio);
			G_GAIN = (USHORT)((GAIN_DEFAULT*512)/b_ratio);
			B_GAIN = GAIN_DEFAULT;
		}
	} else  
	{
		if (b_ratio >= 512)
		{
			R_GAIN = GAIN_DEFAULT;
			G_GAIN = (USHORT)((GAIN_DEFAULT*512)/r_ratio);
			B_GAIN = (USHORT)((GAIN_DEFAULT*b_ratio)/r_ratio);

		} else 
		{
			Gr_GAIN = (USHORT)((GAIN_DEFAULT*512)/r_ratio);
			Gb_GAIN = (USHORT)((GAIN_DEFAULT*512)/b_ratio);
			if (Gr_GAIN >= Gb_GAIN)
			{
				R_GAIN = GAIN_DEFAULT;
				G_GAIN = (USHORT)((GAIN_DEFAULT*512)/r_ratio);
				B_GAIN = (USHORT)((GAIN_DEFAULT*b_ratio)/r_ratio);
    			} else
			{
				R_GAIN =  (USHORT)((GAIN_DEFAULT*r_ratio)/b_ratio);
				G_GAIN = (USHORT)((GAIN_DEFAULT*512)/b_ratio);
				B_GAIN = GAIN_DEFAULT;
			}
		}
	}

#endif
/**********************************************************
 * apply_4h7sub_otp_awb
 * apply otp
 * *******************************************************/
#define GOLDEN_RGAIN  620 // R/Gr
#define GOLDEN_GGAIN  1024 // B/Gr
#define GOLDEN_BGAIN  719 // Gr/Gb
void apply_4h7sub_otp_awb(void)
{
	//char r_gain_h, r_gain_l, g_gain_h, g_gain_l, b_gain_h, b_gain_l;
	unsigned int r_ratio, b_ratio;
    #if 0
	otp_data_info_sub.ngcur = (unsigned int)((otp_data_info_sub.ngrcur + otp_data_info_sub.ngbcur)*1000/2 + 500);
	otp_data_info_sub.frgcur = (unsigned int)(otp_data_info_sub.nrcur*1000 / otp_data_info_sub.ngcur + 500);
	otp_data_info_sub.fbgcur = (unsigned int)(otp_data_info_sub.nbcur*1000 / otp_data_info_sub.ngcur + 500);
    #endif
	otp_data_info_sub.ngcur = (unsigned int)((otp_data_info_sub.ngrcur + otp_data_info_sub.ngbcur)/2);
	otp_data_info_sub.frgcur = (unsigned int)(otp_data_info_sub.nrcur*1024 / otp_data_info_sub.ngcur);
	otp_data_info_sub.fbgcur = (unsigned int)(otp_data_info_sub.nbcur*1024 / otp_data_info_sub.ngcur);
	
	printk("otp_data_info_sub.ngcur = %d\n",otp_data_info_sub.ngcur);
	printk("otp_data_info_sub.frgcur = %d\n",otp_data_info_sub.frgcur);
	printk("otp_data_info_sub.fbgcur = %d\n",otp_data_info_sub.fbgcur);

	otp_data_info_sub.nggolden = (unsigned int)(1024);
	otp_data_info_sub.frggolden = (unsigned int)(1024*GOLDEN_RGAIN/GOLDEN_GGAIN);
	otp_data_info_sub.fbggolden = (unsigned int)(1024*GOLDEN_BGAIN/GOLDEN_GGAIN);

	printk("otp_data_info_sub.nggolden = %d\n",otp_data_info_sub.nggolden);
	printk("otp_data_info_sub.frggolden = %d\n",otp_data_info_sub.frggolden);
	printk("otp_data_info_sub.fbggolden = %d\n",otp_data_info_sub.fbggolden);


	r_ratio = (unsigned int)(512*otp_data_info_sub.frggolden/otp_data_info_sub.frgcur);
	b_ratio = (unsigned int)(512*otp_data_info_sub.fbggolden /otp_data_info_sub.fbgcur);

	printk("r_ratio = %d\n",r_ratio);
	printk("b_ratio = %d\n",b_ratio);

	//cal_rgb_gain(&otp_data_info_sub.nr_gain, &otp_data_info_sub.ng_gain, &otp_data_info_sub.nb_gain, r_ratio, b_ratio);

	if (r_ratio >= 512)
	{
		if (b_ratio >= 512) 
		{
			R_GAIN = (USHORT)((GAIN_DEFAULT*r_ratio)/512);
			G_GAIN = GAIN_DEFAULT;
			B_GAIN = (USHORT)((GAIN_DEFAULT*b_ratio)/512);
		} else
		{
			R_GAIN = (USHORT)((GAIN_DEFAULT*r_ratio)/b_ratio);
			G_GAIN = (USHORT)((GAIN_DEFAULT*512)/b_ratio);
			B_GAIN = GAIN_DEFAULT;
		}
	} else  
	{
		if (b_ratio >= 512)
		{
			R_GAIN = GAIN_DEFAULT;
			G_GAIN = (USHORT)((GAIN_DEFAULT*512)/r_ratio);
			B_GAIN = (USHORT)((GAIN_DEFAULT*b_ratio)/r_ratio);

		} else 
		{
			Gr_GAIN = (USHORT)((GAIN_DEFAULT*512)/r_ratio);
			Gb_GAIN = (USHORT)((GAIN_DEFAULT*512)/b_ratio);
			if (Gr_GAIN >= Gb_GAIN)
			{
				R_GAIN = GAIN_DEFAULT;
				G_GAIN = (USHORT)((GAIN_DEFAULT*512)/r_ratio);
				B_GAIN = (USHORT)((GAIN_DEFAULT*b_ratio)/r_ratio);
    			} else
			{
				R_GAIN =  (USHORT)((GAIN_DEFAULT*r_ratio)/b_ratio);
				G_GAIN = (USHORT)((GAIN_DEFAULT*512)/b_ratio);
				B_GAIN = GAIN_DEFAULT;
			}
		}
	}
	printk("[S5K4H7Y]  R_GAIN = %d, B_GAIN = %d, G_GAIN = %d\n", R_GAIN, B_GAIN, G_GAIN);
	
	if ((R_GAIN != 0) && (B_GAIN != 0) && (G_GAIN != 0))
		{
			otp_4h7sub_write_cmos_sensor_8(0x3c0f, 0x00);
			
			otp_4h7sub_write_cmos_sensor_8(0x0210, R_GAIN>>8);
			otp_4h7sub_write_cmos_sensor_8(0x0211, R_GAIN&0xff);
			otp_4h7sub_write_cmos_sensor_8(0x0212, B_GAIN>>8);
			otp_4h7sub_write_cmos_sensor_8(0x0213, B_GAIN&0xff);
			otp_4h7sub_write_cmos_sensor_8(0x020E, G_GAIN>>8);
			otp_4h7sub_write_cmos_sensor_8(0x020F, G_GAIN&0xff);
			otp_4h7sub_write_cmos_sensor_8(0x0214, G_GAIN>>8);
			otp_4h7sub_write_cmos_sensor_8(0x0215, G_GAIN&0xff);
	
		printk("[S5K4H7Y] otp wb R_GAIN = %d, B_GAIN = %d, G_GAIN = %d\n", R_GAIN, B_GAIN, G_GAIN);																												
		}
		printk("[S5K4H7Y] otp wb R_GAIN = %d, B_GAIN = %d, G_GAIN = %d\n", R_GAIN, B_GAIN, G_GAIN); 
	
	printk("OTP apply_4h7sub_otp_awb\n");
}

/*********************************************************
 *apply_4h7_otp_lsc
 * ******************************************************/
#if 0
void apply_4h7_otp_enb_lsc(void)
{
	printk("OTP enable lsc\n");
	otp_4h7sub_write_cmos_sensor_8(0x0B00,0x01);
}
#endif
/*********************************************************
 * otp_group_info_4h7sub
 * *****************************************************/
int otp_group_info_4h7sub(void)
{
	memset(&otp_data_info_sub,0,sizeof(OTP));

	otp_data_info_sub.lsc_infoflag =
		selective_read_region_sub(S5K4H7SUB_LSC_PAGE,S5K4H7SUB_OTP_LSCFLAG_ADDR);

	if( otp_data_info_sub.lsc_infoflag == 0x01 ){
		otp_data_info_sub.lsc_offset = 0;
		otp_data_info_sub.lsc_group = 1;
		otp_data_info_sub.lsc_sum = selective_read_region_sub_16_sub(S5K4H7SUB_AWB_PAGE,0x0A32);
	}
	else if ( otp_data_info_sub.lsc_infoflag == 0x03 ){
		otp_data_info_sub.lsc_offset = 1;
		otp_data_info_sub.lsc_group = 2;
		otp_data_info_sub.lsc_sum = selective_read_region_sub_16_sub(S5K4H7SUB_AWB_PAGE,0x0A34);
	}
	else{
		printk("4H7 OTP read data fail lsc empty!!!\n");
		goto error;
	}

	otp_data_info_sub.flag_infoflag =
		selective_read_region_sub(S5K4H7SUB_FLAG_PAGE,S5K4H7SUB_OTP_FLAGFLAG_ADDR);

	if( (otp_data_info_sub.flag_infoflag>>4 & 0x0c) == 0x04 ){
		otp_data_info_sub.flag_offset = 0;
		otp_data_info_sub.flag_group = 1;
	}
	else if ( (otp_data_info_sub.flag_infoflag>>4 & 0x03) == 0x01 ){
		otp_data_info_sub.flag_offset = 4;
		otp_data_info_sub.flag_group = 2;
	}
	else{
		printk("4h7 OTP read data fail flag empty!!!\n");
		goto error;
	}

	otp_data_info_sub.flag_module_integrator_id =
		otp_4h7_read_cmos_subsensor(S5K4H7SUB_OTP_FLAGFLAG_ADDR + otp_data_info_sub.flag_offset + 1);

	otp_data_info_sub.infoflag = selective_read_region_sub(S5K4H7SUB_AWB_PAGE,S5K4H7SUB_OTP_AWBFLAG_ADDR);
	printk("otp_data_info_sub.infoflag = %d\n",otp_data_info_sub.infoflag);

	if(otp_data_info_sub.infoflag == 85)
	{
	    //group1
	    
		printk("read awb otp group1\n");
		otp_data_info_sub.nrcur = selective_read_region_sub_16_sub(S5K4H7SUB_AWB_PAGE,0x0A13);
		otp_data_info_sub.nbcur = selective_read_region_sub_16_sub(S5K4H7SUB_AWB_PAGE,0x0A16);
		otp_data_info_sub.ngrcur = selective_read_region_sub_16_sub(S5K4H7SUB_AWB_PAGE,0x0A14);
		otp_data_info_sub.ngbcur = selective_read_region_sub_16_sub(S5K4H7SUB_AWB_PAGE,0x0A15);

		
		printk("otp_data_info_sub.nrcur = %d\n",otp_data_info_sub.nrcur);
		printk("otp_data_info_sub.nbcur = %d\n",otp_data_info_sub.nbcur);
		printk("otp_data_info_sub.ngrcur = %d\n",otp_data_info_sub.ngrcur);
		printk("otp_data_info_sub.ngbcur = %d\n",otp_data_info_sub.ngbcur);

		otp_data_info_sub.nrgolden = selective_read_region_sub_16_sub(S5K4H7SUB_AWB_PAGE,0x0A17);
		otp_data_info_sub.nbgolden = selective_read_region_sub_16_sub(S5K4H7SUB_AWB_PAGE,0x0A1A);
		otp_data_info_sub.ngrgolden = selective_read_region_sub_16_sub(S5K4H7SUB_AWB_PAGE,0x0A18);
		otp_data_info_sub.ngbgolden = selective_read_region_sub_16_sub(S5K4H7SUB_AWB_PAGE,0x0A19);

		printk("otp_data_info_sub.nrgolden = %d\n",otp_data_info_sub.nrgolden);
		printk("otp_data_info_sub.nbgolden = %d\n",otp_data_info_sub.nbgolden);
		printk("otp_data_info_sub.ngrgolden = %d\n",otp_data_info_sub.ngrgolden);
		printk("otp_data_info_sub.ngbgolden = %d\n",otp_data_info_sub.ngbgolden);
	}else{
	    //group2
	    printk("read awb otp group2\n");
		otp_data_info_sub.infoflag = selective_read_region_sub(S5K4H7SUB_AWB_PAGE,0x0A1D);// flag of group2
		
		otp_data_info_sub.nrcur = selective_read_region_sub_16_sub(S5K4H7SUB_AWB_PAGE,0x0A1E);
		otp_data_info_sub.nbcur = selective_read_region_sub_16_sub(S5K4H7SUB_AWB_PAGE,0x0A21);
		otp_data_info_sub.ngrcur = selective_read_region_sub_16_sub(S5K4H7SUB_AWB_PAGE,0x0A1F);
		otp_data_info_sub.ngbcur = selective_read_region_sub_16_sub(S5K4H7SUB_AWB_PAGE,0x0A20);

		printk("otp_data_info_sub.infoflag = %d\n",otp_data_info_sub.infoflag);
		printk("otp_data_info_sub.nrcur = %d\n",otp_data_info_sub.nrcur);
		printk("otp_data_info_sub.nbcur = %d\n",otp_data_info_sub.nbcur);
		printk("otp_data_info_sub.ngrcur = %d\n",otp_data_info_sub.ngrcur);
		printk("otp_data_info_sub.ngbcur = %d\n",otp_data_info_sub.ngbcur);

		otp_data_info_sub.nrgolden = selective_read_region_sub_16_sub(S5K4H7SUB_AWB_PAGE,0x0A22);
		otp_data_info_sub.nbgolden = selective_read_region_sub_16_sub(S5K4H7SUB_AWB_PAGE,0x0A25);
		otp_data_info_sub.ngrgolden = selective_read_region_sub_16_sub(S5K4H7SUB_AWB_PAGE,0x0A23);
		otp_data_info_sub.ngbgolden = selective_read_region_sub_16_sub(S5K4H7SUB_AWB_PAGE,0x0A24);

		printk("otp_data_info_sub.nrgolden = %d\n",otp_data_info_sub.nrgolden);
		printk("otp_data_info_sub.nbgolden = %d\n",otp_data_info_sub.nbgolden);
		printk("otp_data_info_sub.ngrgolden = %d\n",otp_data_info_sub.ngrgolden);
		printk("otp_data_info_sub.ngbgolden = %d\n",otp_data_info_sub.ngbgolden);


	}
	if( (otp_data_info_sub.infoflag>>4 & 0x0c) == 0x04 ){
		otp_data_info_sub.awb_offset = 0;
		otp_data_info_sub.group = 1;
		otp_data_info_sub.awb_flag_sum = selective_read_region_sub_16_sub(S5K4H7SUB_AWB_PAGE,0x0A1C);
	}
	else{
		printk("4h7 OTP read data fail otp_data_info_sub.empty!!!\n");
		goto error;
	}

	return  0;
error:
	return  -1;
}
/*********************************************************
 * read_4h7sub_page
 * read_Page1~Page21 of data
 * return true or false
 ********************************************************/
bool read_4h7sub_page(int page_start,int page_end,unsigned char *pdata)
{
	bool bresult = true;
	int st_page_start = page_start;
	if (page_start <= 0 || page_end > 21){
		bresult = false;
		printk(" OTP page_end is large!");
		return bresult;
	}
	for(; st_page_start <= page_end; st_page_start++){
		get_4h7sub_page_data(st_page_start, pdata);
	}
	return bresult;
}
#if 0
unsigned int sum_awb_flag_lsc(unsigned int sum_start, unsigned int sum_end, unsigned char *pdata)
{
	int i = 0;
	unsigned int start;
	unsigned int re_sum = 0;
	for(start = 0x0A04; i < 64; i++, start++){
		if((start >= sum_start) && (start <= sum_end)){
			re_sum += pdata[i];
		}
	}
	return  re_sum;
}
#endif

bool check_sum_flag_awb_sub(void)
{
    #if 0
	int page_start = 21,page_end = 21;

	unsigned char data_p[21][64] = {};
	bool bresult = true;
	unsigned int  sum_awbfg = 0;

	bresult &= read_4h7sub_page(page_start, page_end, data_p[page_start-1]);

	if(otp_data_info_sub.group == 1){
		sum_awbfg = sum_awb_flag_lsc(0x0A05, 0X0A08, data_p[page_start-1]);
		sum_awbfg += sum_awb_flag_lsc(0x0A0E, 0X0A1D, data_p[page_start-1]);
	}
	else if (otp_data_info_sub.group == 2){
		sum_awbfg = sum_awb_flag_lsc(0x0A09, 0X0A0C, data_p[page_start-1]);
		sum_awbfg += sum_awb_flag_lsc(0x0A1E, 0X0A2D, data_p[page_start-1]);
	}
	if(sum_awbfg == otp_data_info_sub.awb_flag_sum){
		apply_4h7sub_otp_awb();
	}
	else{
		printk("OTP 4h7 check awb flag sum fail!!!");
		bresult &= 0;
	}
	return  bresult;
	#endif
	apply_4h7sub_otp_awb();
	return 1;
	
}
#if 0
bool  check_sum_flag_lsc(void)
{
	int page_start = 21,page_end = 21;

	unsigned char data_p[21][64] = {};
	bool bresult = true;
	unsigned int  sum_slc = 0;

	if(otp_data_info_sub.lsc_group == 1){
		for(page_start = 1, page_end = 6; page_start <= page_end; page_start++){
			bresult &= read_4h7sub_page(page_start, page_start, data_p[page_start-1]);
			if(page_start == 6){
				sum_slc += sum_awb_flag_lsc(0x0A04, 0x0A2B, data_p[page_start-1]);
				continue;
			}
			sum_slc += sum_awb_flag_lsc(0x0A04, 0X0A43,data_p[page_start-1]);
		}
	}
	else if(otp_data_info_sub.lsc_group == 2){
		for(page_start = 6, page_end = 12; page_start <= page_end; page_start++){
			bresult &= read_4h7sub_page(page_start, page_start, data_p[page_start-1]);
			if(page_start == 6){
				sum_slc += sum_awb_flag_lsc(0x0A2C, 0x0A43, data_p[page_start-1]);
				continue;
			}
			else if(page_start <12){
				sum_slc += sum_awb_flag_lsc(0x0A04, 0X0A43, data_p[page_start-1]);
			}
			else{
				sum_slc += sum_awb_flag_lsc(0x0A04, 0X0A13, data_p[page_start-1]);
			}
		}
	}

	if(sum_slc == otp_data_info_sub.lsc_sum){
		apply_4h7_otp_enb_lsc();
		otp_data_info_sub.lsc_check_flag = 1;
	}
	else{
		printk("OTP 4h7 check lsc sum fail!!!");
		bresult &= 0;
		otp_data_info_sub.lsc_check_flag = 0;
	}
	return  bresult;
}
#endif
bool update_otp_sub(void)
{
	int result = 1;
	if(otp_group_info_4h7sub() == -1){
		printk("OTP read data fail  empty!!!\n");
		result &= 0;
	}
	else{
		if(check_sum_flag_awb_sub() == 0){
			printk("OTP 4h7 check sum fail!!!\n");
			result &= 0;
		}
		else{
			printk("OTP 4h7 check ok\n");
		}
	}
	return  result;
}
