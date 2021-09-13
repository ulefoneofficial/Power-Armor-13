/************************************************************                            $
*
* file: cv_8035_wireless_power.h 
*
* Description: prize AP to cv_8035 IIC firmware
*
*------------------------------------------------------------

*************************************************************/

#ifndef __CVS8035_WIRELESS_POWER_H__
#define __CVS8035_WIRELESS_POWER_H__

#define REG_CHIP_ID_H         0x00 //cvs 8035 chip id
#define REG_CHIP_ID_L         0x01 //cvs 8035 chip id
#define REG_FW_VER_H          0x04 //cvs 8035 fw ver
#define REG_FW_VER_L          0x05 //cvs 8035 fw ver

#define REG_ADC_VOUT_H         0x20 //cvs 8035 vout
#define REG_ADC_VOUT_L         0x21 //cvs 8035 vout

#define REG_VOUT_SET_H         0x23 //cvs 8035 set vout
#define REG_VOUT_SET_L         0x24 //cvs 8035 set vout

#define REG_ADC_IOUT           0x30 //cvs 8035 Iout
#define REG_OCP_SWITCH         0x31 //cvs 8035 OCP switch
#define REG_ADC_IOCP_SET       0x32 //cvs 8035 Iout setting

#define REG_ENABLE             0x61 //status 00 watting 01 RX 02 TX
#define REG_SYSTEM_STATUS      0x70 //SYSTEM STATUS 00 no error 0xF1 system error 0xF2 LDO set illegal
#define REG_INTR               0x71 //interrupt status
#define REG_TX_MAX_POWER       0x72 

#define CHIP_ID_WRX            0x8035
#define WRX_WATTINT_MODE       0x00

struct cvs_i2c_addr
{
	u8  chipid_h;
	u8  chipid_l;
	u8  fw_ver_h;
	u8  fw_ver_l;
	u8  adc_vout_h;
	u8  adc_vout_l;
    u8  vout_set_h;
	u8  vout_set_l;
	u8  adc_iout;
	u8  ocp_switch;
	u8  iocp_set;
	u8  reg_enable;
	u8  system_status;
	u8  int_flag;
	u8  tx_max_power;
};


typedef enum cvs_ldo_sta_msg
{
	CVS_LDO_OFF = 0,
    CVS_LDO_ON = 1
}tCVS_LDO_Sta_msg;



typedef enum cvs_sta_code_num
{
	CVS_NO_ERR = 0x00,
	CVS_SYS_ERR = 0xF1,
    CVS_VOUT_SET_ERR = 0xF2
}tCVS_Sta_Code_Num;

struct INF_FLAG 
{
	u8 vrect_good: 1;
	u8 ldo_sta: 1;
	u8	rx: 1;
	u8 tx: 1;
	u8 fod: 1;
	u8 ovp: 1;
	u8 ocp: 1;
	u8 otp: 1;
};


typedef union INT_INFO_MSG
{
	u8		val;
	struct INF_FLAG msg;
}tInt_Msg;

typedef struct CVS_SYSSTA		//IIC PAGE15--start address(0x70)  Accessing this register will clear the IIC interrupt request
{
	u8				sys_sta_code;			//RO
	tInt_Msg	    sys_inf_flag;		//RO
	u8 		        tx_maximum_power;	//RO
}tCVS_SysSta;


struct otg_en_ctl {
	struct pinctrl *pinctrl_gpios;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_otg_high, *pins_otg_low;
	bool gpio_otg_prepare;
};

struct dev_info {
	struct i2c_client *client;
    struct delayed_work  eint_work;
	struct device *dev;
	int status_gpio;
	int irq_gpio;
	int max_power;
	int input_current;
	int charge_current;
	struct otg_en_ctl otg_en;
	tCVS_SysSta wpc_sys_sta;
	struct cvs_i2c_addr addr;
	int firmware_updata_feature;
};

#define PRINTK(fmt,args...) printk("CV8035:"fmt,##args)
#define PRINTK_ERR(fmt,args...) printk(KERN_ERR"CV8035:"fmt,##args)
#endif
