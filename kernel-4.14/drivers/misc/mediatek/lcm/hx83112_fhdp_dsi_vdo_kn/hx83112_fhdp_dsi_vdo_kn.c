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
#define FRAME_HEIGHT 										2400
//prize-penggy modify LCD size-20190328-start
#define LCM_PHYSICAL_WIDTH                  				(69500)
#define LCM_PHYSICAL_HEIGHT                  				(154440)
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

static struct LCM_setting_table lcm_initialization_setting[] = {
	{0xB9,3, {0x83,0x11,0x2A}},
	{0xB1,8, {0x08,0x27,0x27,0x80,0x80,0x4A,0x4F,0xAA}},
	{0xB2,14,{0x00,0x02,0x00,0x90,0x60,0x00,0x0F,0x34,0xE2,0x11,0x00,0x00,0x15,0xA3}},
	{0xD2,2, {0x2C,0x2C}},
	{0xB4,27,{0x02,0xB0,0x02,0xB0,0x02,0xB4,0x04,0xB6,0x00,0x00,0x04,0xB6,0x00,0xFF,
			  0x00,0xFF,0x00,0x00,0x0C,0x12,0x00,0x29,0x06,0x06,0x0F,0x00,0x86}},
	{0xCC,1, {0x08}},
	{0xD3,47,{0xC0,0x00,0x00,0x00,0x00,0x01,0x00,0x14,0x14,0x03,0x01,0x11,0x08,0x08,
			  0x08,0x08,0x08,0x32,0x10,0x05,0x00,0x00,0x32,0x10,0x0F,0x00,0x0F,0x32,
			  0x10,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
			  0x00,0x00,0x00,0xEA,0x1C}},
	
	{0xD5,48,{0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,
			  0xC0,0xC0,0xC0,0xC0,0x18,0x18,0x40,0x40,0x18,0x18,0x18,0x18,0x31,0x31,
			  0x30,0x30,0x2F,0x2F,0x01,0x00,0x03,0x02,0x20,0x20,0xC0,0xC0,0xC0,0xC0,
			  0x19,0x19,0x40,0x40,0x25,0x24}},
	
	{0xD6,48,{0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,
			  0x40,0x40,0x40,0x40,0x18,0x18,0x40,0x40,0x18,0x18,0x18,0x18,0x31,0x31,
			  0x30,0x30,0x2F,0x2F,0x02,0x03,0x00,0x01,0x20,0x20,0x40,0x40,0x40,0x40,
			  0x40,0x40,0x19,0x19,0x24,0x25}},
	
	{0xD8,12,{0xAA,0xAA,0xAA,0xAA,0xFF,0xEA,0xAA,0xAA,0xAA,0xAA,0xFF,0xEA}},
	{0xBD,1, {0x01}},
	{0xD8,12,{0xAA,0xAA,0xAF,0xFF,0xFF,0xFF,0xAA,0xAA,0xAF,0xFF,0xFF,0xFF}},
	{0xBD,1, {0x02}},
	{0xD8,12,{0xAA,0xAA,0xAF,0xFF,0xFF,0xFF,0xAA,0xAA,0xAF,0xFF,0xFF,0xFF}},
	{0xBD,1, {0x03}},
	{0xD8,24,{0xAA,0xAA,0xAE,0xAA,0xAA,0xBE,0xAA,0xAA,0xAE,0xAA,0xAA,0xBE,
			  0xAA,0xAA,0xAF,0xFF,0xFF,0xFF,0xAA,0xAA,0xAF,0xFF,0xFF,0xFF}},
	{0xBD,1, {0x00}},
	{0xC1,1, {0x01}},
	{0xBD,1, {0x01}},
	{0xC1,57,{0xFF,0xFA,0xF5,0xF0,0xEC,0xE8,0xE3,0xDA,0xD5,0xD1,0xCC,0xC7,0xC3,0xBE,
			  0xBA,0xB5,0xB0,0xAC,0xA7,0x9E,0x95,0x8D,0x84,0x7D,0x75,0x6D,0x64,0x5D,
			  0x55,0x4D,0x47,0x40,0x39,0x32,0x2A,0x23,0x1D,0x16,0x10,0x09,0x06,0x05,
			  0x02,0x01,0x00,0x1F,0x45,0xEF,0x75,0xDE,0xDC,0x0D,0xF6,0x8F,0x73,0x5E,
			  0x00}},
	{0xBD,1, {0x02}},
	{0xC1,57,{0xFF,0xFA,0xF5,0xF0,0xEC,0xE8,0xE3,0xDA,0xD5,0xD1,0xCC,0xC7,0xC3,0xBE,
			  0xBA,0xB5,0xB0,0xAC,0xA7,0x9E,0x95,0x8D,0x84,0x7D,0x75,0x6D,0x64,0x5D,
			  0x55,0x4D,0x47,0x40,0x39,0x32,0x2A,0x23,0x1D,0x16,0x10,0x09,0x06,0x05,
			  0x02,0x01,0x00,0x1F,0x45,0xEF,0x75,0xDE,0xDC,0x0D,0xF6,0x8F,0x73,0x5E,
			  0x00}},
	{0xBD,1, {0x03}},
	{0xC1,57,{0xFF,0xFA,0xF5,0xF0,0xEC,0xE8,0xE3,0xDA,0xD5,0xD1,0xCC,0xC7,0xC3,0xBE,
			  0xBA,0xB5,0xB0,0xAC,0xA7,0x9E,0x95,0x8D,0x84,0x7D,0x75,0x6D,0x64,0x5D,
			  0x55,0x4D,0x47,0x40,0x39,0x32,0x2A,0x23,0x1D,0x16,0x10,0x09,0x06,0x05,
			  0x02,0x01,0x00,0x1F,0x45,0xEF,0x75,0xDE,0xDC,0x0D,0xF6,0x8F,0x73,0x5E,
			  0x00}},
	{0xBD,1, {0x00}},
	{0xE7,25,{0x0F,0x0F,0x1E,0x5E,0x1E,0x5D,0x00,0x50,0x20,0x14,0x00,0x00,0x02,0x02,
			  0x02,0x05,0x14,0x14,0x32,0xB9,0x23,0xB9,0x08,0x13,0x69}},
	{0xBD,1, {0x01}},
	{0xE7,8, {0x02,0x00,0xA8,0x01,0xAA,0x0E,0x20,0x0F}},
	{0xBD,1, {0x02}},
	{0xE7,29,{0x00,0x00,0x08,0x40,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
			  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x70,0x01,0x70,0x02,
			  0x00}},
	{0xBD,1, {0x00}},
	{0xC0,2, {0x22,0x22}},
	{0xBD,1, {0x02}},
	{0xB4,9, {0x00,0x92,0x12,0x11,0x88,0x12,0x12,0x00,0x53}},
	{0xBD,1, {0x01}},
	{0xB1,3, {0x64,0x01,0x09}},
	{0xBD,1, {0x00}},
	{0xC7,6, {0x00,0x00,0x04,0xE0,0x33,0x00}},
	{0xBA,16,{0x73,0x03,0x64,0x33,0x33,0x33,0x33,0x11,0x10,0x00,0x00,0x10,0x08,0x3D,0x02,0x76}},
	{0xE9,1, {0xC3}},
	{0xCB,4, {0xD1,0xB3,0x00,0x02}},
	{0xE9,1, {0x3F}},
	{0x11,0, {}},
	{REGFLAG_DELAY,120, {}},
	{0x29,0, {}},
	{REGFLAG_DELAY,20, {}},
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28, 0, {}},
	{REGFLAG_DELAY, 50, {} },
	{0x10, 0, {}},
	{REGFLAG_DELAY, 120, {} },

};

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
	
	params->dsi.vertical_sync_active     = 2;
	params->dsi.vertical_backporch       = 15;
	params->dsi.vertical_frontporch      = 54;
	params->dsi.vertical_active_line     = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active   = 20;
	params->dsi.horizontal_backporch     = 20;//36
	params->dsi.horizontal_frontporch    = 32;//78
	params->dsi.horizontal_active_pixel  = FRAME_WIDTH;

    params->dsi.PLL_CLOCK= 555;        //4LANE   175
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
	params->dsi.lcm_esd_check_table[1].para_list[2] = 0x04;
	#endif
}

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
	//display_ldo28_enable(1);
   // display_ldo18_enable(1);

    MDELAY(5);
	display_bias_enable();
	MDELAY(10);

	//przie-modify gpio rst-pengzhipeng-20210830-start
    mt_dsi_pinctrl_set(LCM_RESET_PIN_NO, 1);
	//SET_RESET_PIN(1);
	MDELAY(10);
	mt_dsi_pinctrl_set(LCM_RESET_PIN_NO, 0);
	//SET_RESET_PIN(0);
	MDELAY(10);  
    mt_dsi_pinctrl_set(LCM_RESET_PIN_NO, 1);
	//przie-modify gpio rst-pengzhipeng-20210830-end
	MDELAY(20);  
	
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting)/sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
    push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting)/sizeof(struct LCM_setting_table), 1);
	//przie-modify gpio rst-pengzhipeng-20210830-start
	mt_dsi_pinctrl_set(LCM_RESET_PIN_NO, 0);  
	MDELAY(20);
	//przie-modify gpio rst-pengzhipeng-20210830-end
	     
	display_bias_disable();
	MDELAY(10);

}

static void lcm_resume(void)
{
	lcm_init();
}


struct LCM_DRIVER hx83112_fhdp_dsi_vdo_kn_drv = 
{
    .name			= "hx83112_fhdp_dsi_vdo_kn",
	#if defined(CONFIG_PRIZE_HARDWARE_INFO) && !defined (BUILD_LK)
	.lcm_info = {
		.chip	= "hx83112",
		.vendor	= "himax",
		.id		= "0x80",
		.more	= "1080*2400",
	},
	#endif
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
	.ata_check 		= lcm_ata_check,
};
