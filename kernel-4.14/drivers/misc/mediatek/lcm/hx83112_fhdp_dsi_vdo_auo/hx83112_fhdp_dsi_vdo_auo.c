#define LOG_TAG "LCM"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"


#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
/*#include <mach/mt_pm_ldo.h>*/
#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_gpio.h>
#endif
#endif

#ifdef BUILD_LK
#define LCM_LOGI(fmt, args...)  printk(KERN_INFO  " LCM file=%s: %s: line=%d: "fmt"\n", __FILE__,__func__,  __LINE__,##args)
#define LCM_LOGD(fmt, args...)  printk(KERN_DEBUG " LCM file=%s: %s: line=%d: "fmt"\n", __FILE__,__func__,  __LINE__,##args)
#else
#define LCM_LOGI(fmt, args...)  printk(KERN_INFO " LCM :"fmt"\n", ##args)
#define LCM_LOGD(fmt, args...)  printk(KERN_DEBUG " LCM :"fmt"\n", ##args)
#endif

#define I2C_I2C_LCD_BIAS_CHANNEL 0
static struct LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)			(lcm_util.set_reset_pin((v)))
#define MDELAY(n)					(lcm_util.mdelay(n))

/* --------------------------------------------------------------------------- */
/* Local Functions */
/* --------------------------------------------------------------------------- */
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
	lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) \
	lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
	lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
	lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
	lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

static const unsigned char LCD_MODULE_ID = 0x01;
/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */
#define LCM_DSI_CMD_MODE	0
#define FRAME_WIDTH  										1080
#define FRAME_HEIGHT 										2340
//prize-penggy modify LCD size-20190328-start
#define LCM_PHYSICAL_WIDTH                  				(67070)
//prize by ycj for antutu
#define LCM_PHYSICAL_HEIGHT                  				(151636)//(145310)
//prize-penggy modify LCD size-20190328-end

#define REGFLAG_DELAY             							 0xFFFA
#define REGFLAG_UDELAY             							 0xFFFB
#define REGFLAG_PORT_SWAP									 0xFFFC
#define REGFLAG_END_OF_TABLE      							 0xFFFD   // END OF REGISTERS MARKER

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/* --------------------------------------------------------------------------- */
/* Local Variables */
/* --------------------------------------------------------------------------- */

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};
/*prize penggy add-for LCD ESD-20190228-start*/

static struct LCM_setting_table lcm_initialization_setting[] = {

	{ 0xB9, 3, {0x83, 0x11, 0x2A }},
	{ 0xB1, 8, {0x08, 0x27, 0x27, 0x80, 0x80, 0x4B, 0x4E, 0xCC }},
	{ 0xBD, 1, {0x01 }},
	{ 0xB1, 7, {0x64, 0x01, 0x09, 0x20, 0x50, 0x00, 0x00 }},
	{ 0xBF, 2, {0x1A, 0xAF }},
	{ 0xBD, 1, {0x00 }},
	{ 0xB2, 15,{0x00, 0x02, 0x00, 0x90, 0x24, 0x00, 0x02, 0x20, 0xE9, 0x11, 0x11, 0x00, 0x15, 0xA3, 0x87 }},
	{ 0xD2, 2, {0x2C, 0x2C }},
	{ 0xB4, 27,{0x66, 0x66, 0xB0, 0x70, 0xC7, 0xDC, 0x3F, 0xAA, 0x01, 0x01, 0x00, 0xC5, 0x00, 0xFF, 0x00, 0xFF, 
				0x03, 0x00, 0x02, 0x03, 0x00, 0x31, 0x05, 0x08, 0x09, 0x00, 0x31 }},
	{ 0xBD, 1, {0x02 }},
	{ 0xB4, 9, {0x00, 0x92, 0x12, 0x11, 0x88, 0x12, 0x12, 0x00, 0x70 }},
	{ 0xBD, 1, {0x00 }},
	{ 0xC1, 1, {0x01 }},
	{ 0xBD, 1, {0x01 }},
	{ 0xC1, 57,{0xFF, 0xF9, 0xF3, 0xED, 0xE8, 0xE2, 0xDD, 0xD1, 0xCB, 0xC5, 0xBF, 0xB9, 0xB3, 0xAD, 0xA7, 0xA1, 
				0x9C, 0x96, 0x91, 0x87, 0x7E, 0x75, 0x6D, 0x64, 0x5D, 0x56, 0x4F, 0x48, 0x43, 0x3D, 0x37, 0x31, 
				0x2B, 0x25, 0x20, 0x1A, 0x15, 0x10, 0x0B, 0x06, 0x05, 0x02, 0x02, 0x01, 0x00, 0x17, 0xB7, 0x9A, 
				0x5B, 0x3A, 0x77, 0x93, 0x10, 0x27, 0xE6, 0x31, 0x00 }},
	{ 0xBD, 1, {0x02 }},
	{ 0xC1, 57,{0xFF, 0xF9, 0xF3, 0xED, 0xE8, 0xE2, 0xDD, 0xD1, 0xCB, 0xC5, 0xBF, 0xB9, 0xB3, 0xAD, 0xA7, 0xA1, 
				0x9C, 0x96, 0x91, 0x87, 0x7E, 0x75, 0x6D, 0x64, 0x5D, 0x56, 0x4F, 0x48, 0x43, 0x3D, 0x37, 0x31, 
				0x2B, 0x25, 0x20, 0x1A, 0x15, 0x10, 0x0B, 0x06, 0x05, 0x02, 0x02, 0x01, 0x00, 0x17, 0xB7, 0x9A, 
				0x5B, 0x3A, 0x77, 0x93, 0x10, 0x27, 0xE6, 0x31, 0x00 }},
	{ 0xBD, 1, {0x03 }},
	{ 0xC1, 57,{0xFF, 0xF9, 0xF3, 0xED, 0xE8, 0xE2, 0xDD, 0xD1, 0xCB, 0xC5, 0xBF, 0xB9, 0xB3, 0xAD, 0xA7, 0xA1, 
				0x9C, 0x96, 0x91, 0x87, 0x7E, 0x75, 0x6D, 0x64, 0x5D, 0x56, 0x4F, 0x48, 0x43, 0x3D, 0x37, 0x31, 
				0x2B, 0x25, 0x20, 0x1A, 0x15, 0x10, 0x0B, 0x06, 0x05, 0x02, 0x02, 0x01, 0x00, 0x17, 0xB7, 0x9A, 
				0x5B, 0x3A, 0x77, 0x93, 0x10, 0x27, 0xE6, 0x31, 0x00 }},
	{ 0xBD, 1, {0x00 }},
	{ 0xC0, 2, {0x23, 0x23 }},
	{ 0xCC, 1, {0x08 }},
	{ 0xD3, 43,{0xC0, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x03, 0x47, 0x23, 0x34, 0x05, 0x05, 0x05, 
				0x05, 0x32, 0x10, 0x03, 0x00, 0x03, 0x32, 0x10, 0x07, 0x00, 0x07, 0x32, 0x10, 0x01, 0x00, 0x01, 
				0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0C, 0x11, 0x09, 0x36, 0x00 }},
	{ 0xBD, 1, {0x01 }},
	{ 0xD3, 8, {0x00, 0x00, 0x19, 0x00, 0x00, 0x4A, 0x00, 0xA1 }},
	{ 0xBD, 1, {0x00 }},
	{ 0xD5, 48,{0x18, 0x18, 0x19, 0x18, 0x18, 0x18, 0x18, 0x20, 0x18, 0x18, 0x10, 0x10, 0x18, 0x18, 0x18, 0x18, 
				0x03, 0x03, 0x02, 0x02, 0x01, 0x01, 0x00, 0x00, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 
				0x18, 0x18, 0x31, 0x31, 0x2F, 0x2F, 0x30, 0x30, 0x18, 0x18, 0x35, 0x35, 0x36, 0x36, 0x37, 0x37 }},
	{ 0xD6, 48,{0x18, 0x18, 0x19, 0x18, 0x18, 0x18, 0x19, 0x20, 0x18, 0x18, 0x10, 0x10, 0x18, 0x18, 0x18, 0x18, 
				0x00, 0x00, 0x01, 0x01, 0x02, 0x02, 0x03, 0x03, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 
				0x18, 0x18, 0x31, 0x31, 0x2F, 0x2F, 0x30, 0x30, 0x18, 0x18, 0x35, 0x35, 0x36, 0x36, 0x37, 0x37 }},
	{ 0xD8, 24,{0xAA, 0xAA, 0xFF, 0xAA, 0xBF, 0xAA, 0xAA, 0xAA, 0xFF, 0xAA, 0xBF, 0xAA, 0xAA, 0xAA, 0x55, 0xAA, 
				0xBF, 0xAA, 0xAA, 0xAA, 0x55, 0xAA, 0xBF, 0xAA }},
	{ 0xBD, 1, {0x01 }},
	{ 0xD8, 24,{0xAA, 0xAA, 0xFF, 0xAA, 0xBF, 0xAA, 0xAA, 0xAA, 0xFF, 0xAA, 0xBF, 0xAA, 0xAA, 0xAA, 0xFF, 0xAA, 
				0xBF, 0xAA, 0xAA, 0xAA, 0xFF, 0xAA, 0xBF, 0xAA }},
	{ 0xBD, 1, {0x02 }},
	{ 0xD8, 12,{0xAA, 0xAA, 0xFF, 0xAA, 0xBF, 0xAA, 0xAA, 0xAA, 0xFF, 0xAA, 0xBF, 0xAA }},
	{ 0xBD, 1, {0x03 }},
	{ 0xD8, 24,{0xBA, 0xAA, 0xAA, 0xAA, 0xBF, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xBF, 0xAA, 0xBA, 0xAA, 0xAA, 0xAA, 
				0xBF, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xBF, 0xAA }},
	{ 0xBD, 1, {0x00 }},
	{ 0xE7, 24,{0x0E, 0x0E, 0x1E, 0x66, 0x1E, 0x66, 0x00, 0x32, 0x02, 0x02, 0x00, 0x00, 0x02, 0x02, 0x02, 0x05, 
				0x14, 0x14, 0x32, 0xB9, 0x23, 0xB9, 0x08, 0x03 }},
	{ 0xBD, 1, {0x01 }},
	{ 0xE7, 10,{0x02, 0x07, 0xA8, 0x01, 0xA8, 0x0D, 0xA1, 0x0E, 0x01, 0x01 }},
	{ 0xBD, 1, {0x02 }},
	{ 0xE7, 29,{0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x22, 0x00 }},
	{ 0xBD, 1, {0x00 }},
	{ 0xC6, 2, {0x77, 0xBF }},
	{ 0xC7, 6, {0x70, 0x00, 0x04, 0xE0, 0x33, 0x00 }},
	{ 0xE9, 1, {0xC3 }},
	{ 0xCB, 2, {0xD1, 0xD3 }},
	{ 0xE9, 1, {0x3F }},
	{ 0xBA, 3, {0x73, 0x03, 0xE4 }},
	{ 0xCF, 4, {0x00, 0x14, 0x00, 0x40 }},
	{ 0x35, 1, {0x00 }},

	//prize penggy add-for LCD ESD-20190219-start//
	{0xCD,1,{0x01}},
	//prize penggy add-for LCD ESD-20190219-end//
	{0x11,1,{0x00}},
	{REGFLAG_DELAY,120,{}},


	{0x29,1,{0x00}},
	{REGFLAG_DELAY, 50, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}    
              
};


#if 0
static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28, 1,{0x00} },
	{REGFLAG_DELAY, 120, {} },
	{0x10, 1,{0x00} },
	{REGFLAG_DELAY, 10, {} },
//	{0x04, 1,{0x5a} },
//	{0x05, 1,{0x5a} },
	{REGFLAG_DELAY, 120, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {}}  
};
#endif
/*prize penggy add-for LCD ESD-20190228-end*/

static void push_table(struct LCM_setting_table *table, unsigned int count,
		       unsigned char force_update)
{
	unsigned int i;
    LCM_LOGI("nt35695----tps6132-lcm_init   push_table++++++++++++++===============================devin----\n");
	for (i = 0; i < count; i++) {
		unsigned cmd;

		cmd = table[i].cmd;

		switch (cmd) {

		case REGFLAG_DELAY:
			if (table[i].count <= 10)
				MDELAY(table[i].count);
			else
				MDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}



/* --------------------------------------------------------------------------- */
/* LCM Driver Implementations */
/* --------------------------------------------------------------------------- */

static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}

static void lcm_get_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));
	
	params->type = LCM_TYPE_DSI;
	
	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	
	// enable tearing-free
	params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
	params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
	#if (LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
	params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
	#else
	params->dsi.mode   = SYNC_PULSE_VDO_MODE;//SYNC_EVENT_VDO_MODE;//BURST_VDO_MODE;////
	#endif
		
	// DSI
	/* Command mode setting */
	//1 Three lane or Four lane
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding 	= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format		= LCM_DSI_FORMAT_RGB888;
		
		
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		
	#if (LCM_DSI_CMD_MODE)
	params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage
	params->dsi.word_count=FRAME_WIDTH*3;	//DSI CMD mode need set these two bellow params, different to 6577
	#else
	params->dsi.intermediat_buffer_num = 0; //because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage
	#endif
	
	// Video mode setting
	params->dsi.packet_size=256;
	
	params->dsi.vertical_sync_active				=  5;
	params->dsi.vertical_backporch					= 12;//5;//16 25 30 35 12 8
	params->dsi.vertical_frontporch					= 20;//27;
	params->dsi.vertical_active_line				= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 8;
	params->dsi.horizontal_backporch = 8;//32
	params->dsi.horizontal_frontporch = 28;//78
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	/* params->dsi.ssc_disable                                                       = 1; */

	params->dsi.PLL_CLOCK = 517;//244;
	params->dsi.ssc_disable = 0;
	params->dsi.ssc_range = 1;
	params->dsi.cont_clock = 0;
	params->dsi.clk_lp_per_line_enable = 0;
	//prize-penggy modify LCD size-20190328-start
	params->physical_width = LCM_PHYSICAL_WIDTH/1000;
	params->physical_height = LCM_PHYSICAL_HEIGHT/1000;
	//prize-penggy modify LCD size-20190328-end
	params->physical_width_um = LCM_PHYSICAL_WIDTH;
	params->physical_height_um = LCM_PHYSICAL_HEIGHT;
	/*prize penggy add-for LCD ESD-20190219-start*/
	#if 1
	params->dsi.ssc_disable = 0;
	params->dsi.lcm_ext_te_monitor = FALSE;
		
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd					= 0x0a;
	params->dsi.lcm_esd_check_table[0].count			= 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9d;
	params->dsi.lcm_esd_check_table[1].cmd					= 0x09;
	params->dsi.lcm_esd_check_table[1].count			= 3;
	params->dsi.lcm_esd_check_table[1].para_list[0] = 0x80;
	params->dsi.lcm_esd_check_table[1].para_list[1] = 0x73;
	params->dsi.lcm_esd_check_table[1].para_list[2] = 0x06;
	#endif
}

/*void init_lcm_registers(void)
{
unsigned int data_array[16];

data_array[0] = 0x00043902;
data_array[1] = 0x2A1183B9;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0] = 0x00023902;
data_array[1] = 0x000021CD;
dsi_set_cmdq(data_array, 2, 1);
//MDELAY(1);

data_array[0] = 0x00110500;
dsi_set_cmdq(data_array, 1, 1);
//MDELAY(150);

data_array[0] = 0x00290500;
dsi_set_cmdq(data_array, 1, 1);
MDELAY(70);
}*/

static unsigned int lcm_compare_id(void)
{
   return 1; 
}
#ifndef BUILD_LK
extern atomic_t ESDCheck_byCPU;
#endif
static unsigned int lcm_ata_check(unsigned char *bufferr)
{
	//prize-Solve ATA testing-pengzhipeng-20181127-start
	unsigned char buffer1[2]={0};
	unsigned char buffer2[2]={0};
	
	unsigned int data_array[6]; 
	 
	data_array[0]= 0x00023902;//LS packet 
	data_array[1]= 0x000050b8; 
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00013700;// read id return two byte,version and id
	dsi_set_cmdq(data_array, 1, 1);
	atomic_set(&ESDCheck_byCPU, 1);
	
	read_reg_v2(0xb8, buffer1, 1);
	atomic_set(&ESDCheck_byCPU, 0);
	
	
	data_array[0]= 0x0002390a;//HS packet 
	data_array[1]= 0x000031b8; 
	dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0] = 0x00013700;// read id return two byte,version and id
	dsi_set_cmdq(data_array, 1, 1);
	atomic_set(&ESDCheck_byCPU, 1);
	
	read_reg_v2(0xb8, buffer2, 1);
	atomic_set(&ESDCheck_byCPU, 0);
	
	
	LCM_LOGI("%s, Kernel TDDI id buffer1= 0x%04x buffer2= 0x%04x\n", __func__, buffer1[0],buffer2[0]);
	return ((0x50 == buffer1[0])&&(0x31 == buffer2[0]))?1:0; 
	//prize-Solve ATA testing-pengzhipeng-20181127-end
}

static void lcm_init(void)
{
	
	display_bias_enable();
	MDELAY(10);

	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);  

	SET_RESET_PIN(1);
	MDELAY(60);//250
	
    //init_lcm_registers();
	push_table(lcm_initialization_setting,sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
	unsigned int data_array[16];

	data_array[0]=0x00280500; // Display Off
	dsi_set_cmdq(data_array, 1, 1);
    //MDELAY(120);

	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);
    MDELAY(120);
	     
	display_bias_disable();
	MDELAY(10);//100
	
	//SET_RESET_PIN(0);	//prize-wyq 20190315 keep reset pin high to fix suspend current leakage
	MDELAY(10);//100

}

static void lcm_resume(void)
{
	lcm_init();
}

static void lcm_init_power(void)
{
	display_bias_enable();
}

static void lcm_suspend_power(void)
{
	display_bias_disable();
}

static void lcm_resume_power(void)
{
	SET_RESET_PIN(0); //prize-wyq 20190315 keep reset pin high to fix suspend current leakage
	display_bias_enable();
}

struct LCM_DRIVER hx83112_fhdp_dsi_vdo_auo_drv = 
{
    .name			= "hx83112_fhdp_dsi_vdo_auo",
	#if defined(CONFIG_PRIZE_HARDWARE_INFO) && !defined (BUILD_LK)
	.lcm_info = {
		.chip	= "hx83112",
		.vendor	= "himax",
		.id		= "0x80",
		.more	= "1080*2340",
	},
	#endif
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
	.ata_check 		= lcm_ata_check,

};
