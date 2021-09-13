
/************************************************************                            $
*
* file: cvs8035d_wireless_power.c 
*
*************************************************************/

#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/pinctrl/consumer.h>
#include "../mtk_charger_intf.h" //add by sunshuai
#include "cv_8035.h"
#include "cvs_firmware.h"

static struct dev_info dev_info = {
	.input_current = 0,
	.charge_current = 0,
	.max_power = 0,
	.firmware_updata_feature = 0,
};
static struct dev_info *info = &dev_info;

static int cvs_updata_mode = 0;

int set_otg_gpio(int en){

	int ret =0;
	
    if (info->otg_en.gpio_otg_prepare) {
		if (en) {
			pinctrl_select_state(info->otg_en.pinctrl_gpios, info->otg_en.pins_otg_high);
			printk("%s: set w_otg_en PIN to high\n", __func__);
			ret =0;
		}
		else {
			pinctrl_select_state(info->otg_en.pinctrl_gpios, info->otg_en.pins_otg_low);
			printk("%s: set w_otg_en PIN to low\n", __func__);
			ret =0;
		}
	}
	else {
		printk("%s:, error, gpio otg not prepared\n", __func__);
		ret =-1;
	}
	return ret;
}
EXPORT_SYMBOL(set_otg_gpio);



static int cvs_write_reg(u8 *cvbuf,u8 length){
	struct i2c_msg msg[1];
	int ret;
	
	msg[0].addr = info->client->addr;
	msg[0].flags = 0;
	msg[0].len = length;
	msg[0].buf = cvbuf;
	
	ret = i2c_transfer(info->client->adapter,msg,1);
	if(ret == 1)
	{
		return 0;
	}
	else
	{
		PRINTK("%s i2c_transfer error\n",__func__);
		return -EINVAL;
	}
}

static int cvs_read_reg(u8 cvaddr,u8 *cvbuf,u8 length) {
	struct i2c_msg msg[1];
	int ret;
	
	ret = cvs_write_reg(&cvaddr,1);
	mdelay(1);
	if(ret == 0)
	{
		msg[0].addr = info->client->addr;
		msg[0].flags = 1;
		msg[0].len = length;
		msg[0].buf = cvbuf;
		
		ret = i2c_transfer(info->client->adapter,msg,1);
		if(ret == 1)
		{
			return 0;
		}
		else
		{
			PRINTK("%s i2c_transfer error\n",__func__);
			return -EINVAL;
		}
	}
	else
	{
		PRINTK("%s cvs_write_reg error\n",__func__);
		return -EINVAL;
	}
}

static int cvs_read_reg16(u8 cvaddr,u16 *cvbuf) {
	struct i2c_msg msg[1];
	int ret;
	//u8 bytebuf[2];
	u8 *r_buf = NULL;

	r_buf = kzalloc(2, GFP_KERNEL);
	if (r_buf == NULL)
		return -1;
	
	ret = cvs_write_reg(&cvaddr,1);
	mdelay(1);
	
	if(ret == 0)
	{
		msg[0].addr = info->client->addr;
		msg[0].flags = 1;
		msg[0].len = 2;
		msg[0].buf = r_buf;
		
		ret = i2c_transfer(info->client->adapter,msg,1);
		if(ret == 1)
		{
			*cvbuf = ((r_buf[0] << 8) & 0xff00) | r_buf[1];
			//return 0;
		}
		else
		{
			PRINTK("%s i2c_transfer error\n",__func__);
			if (r_buf == NULL)
			   kfree(r_buf);
			return -EINVAL;
		}
	}
	else
	{
		PRINTK("%s cvs_write_reg error\n",__func__);
		if (r_buf == NULL)
		   kfree(r_buf);
		return -EINVAL;
	}
	if (r_buf == NULL)
		kfree(r_buf);
	return 0;
}

// start updata firwmare
int cvs_enter_updata_mode(void)
{
	//Set EN high
	//Input 5V voltage to IC from Vout
	
	return 0;
}

int cvs_if_updata_mode(void)
{
	int ret = 0;
	u8 reg = 0;
	
	ret = cvs_read_reg(0xB0, &reg, 1);
	PRINTK("%s ret =%d\n",__func__,ret);
	if(reg == 0xB0)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

u16 cvs_get_firmware_version(void)
{
	int ret = 0;
	u16 firmware_ver = 0;
	
	if(cvs_if_updata_mode())
	{
		ret = cvs_read_reg16(0xB6, &firmware_ver);
		PRINTK("%s ret =%d\n",__func__,ret);
	}
	else
	{
		ret = cvs_read_reg16(0x04, &firmware_ver);
		PRINTK("%s ret =%d\n",__func__,ret);
	}
	
	PRINTK("cvs_get_firmware_version is 0x%x\n",firmware_ver);
	
	return firmware_ver;
}

u16 cvs_get_firmware_checksum(void)
{
	int ret = 0;
	u16 firmware_checksum = 0;
	
	if(cvs_if_updata_mode())
	{
		ret = cvs_read_reg16(0xB8, &firmware_checksum);
		PRINTK("%s ret =%d\n",__func__,ret);
	}
	else
	{
		ret = cvs_read_reg16(0x06, &firmware_checksum);
		PRINTK("%s ret =%d\n",__func__,ret);
	}
	
	PRINTK("cvs_get_firmware_checksum is 0x%x\n",firmware_checksum);
	
	return firmware_checksum;
}

int cvs_write_mtp(u16 addr,u8 *dat,u8 datlen)
{
	int ret = 0;
	u8 tmpbuf[3] = {0,0,0};
	u8 upstate = 0,overtime_cnt = 0;
	u8 reg = 0xFF;

	tmpbuf[0] = 0xBA;
	tmpbuf[1] = addr >> 8;
	tmpbuf[2] = addr;
	ret = cvs_write_reg(tmpbuf, 3);			//write BA
	PRINTK("%s ret =%d\n",__func__,ret);
	
	ret = cvs_write_reg(dat, datlen);		//write BD
	PRINTK("%s ret =%d\n",__func__,ret);
	
	tmpbuf[0] = 0xBC;
	tmpbuf[1] = 0x0B;
	ret = cvs_write_reg(tmpbuf, 2);			//write BC
	PRINTK("%s ret =%d\n",__func__,ret);
	
	//At this time, the IIC_INT pin becomes low until the IIC_INT pin becomes high again, indicating that the data is successfully burned
	do{
		//Query IIC_INT pin status
		mdelay(10);
		upstate = gpio_get_value(info->irq_gpio); 
		PRINTK("cvs_write_mtp upstate = %d\n",upstate);
		overtime_cnt++;
		if(overtime_cnt > 10) //100mS timeout
		{
			PRINTK("cvs_write_mtp overtime 100msec err");
			return 1;
		}
	}
	while(upstate == 0);
	
	ret = cvs_read_reg(0xB1, &reg, 1);
	PRINTK("%s ret =%d\n",__func__,ret);
	if(reg == 0x00)
	{
		return 0;
	}
	else
	{
		PRINTK("cvs_write_mtp write err");
		return 2;
	}
}

int cvs_parity_updata(u8 *firmware_binfile_data)
{
	int ret = 0;
	u8 tmpbuf[7],i;
	u8 upstate = 0,overtime_cnt = 0;
	u8 reg = 0xFF;

	tmpbuf[0] = 0xBA;
	tmpbuf[1] = 0xFF;
	tmpbuf[2] = 0xFF;
	ret = cvs_write_reg(tmpbuf, 3);			//write BA
	PRINTK("%s ret =%d\n",__func__,ret);
	
	tmpbuf[0] = 0xBD;
	for(i = 0;i < 6;i++)
	{
		tmpbuf[i + 1] = firmware_binfile_data[i];
	}
	ret = cvs_write_reg(tmpbuf, 7);		//write BD
	PRINTK("%s ret =%d\n",__func__,ret);
	
	tmpbuf[0] = 0xBC;
	tmpbuf[1] = 0x0C;
	ret = cvs_write_reg(tmpbuf, 2);			//write BC
	PRINTK("%s ret =%d\n",__func__,ret);
	
	//At this time, the IIC_INT pin becomes low until the IIC_INT pin becomes high again, indicating that the data is successfully burned
	do{
		//Query IIC_INT pin status
		mdelay(10);
		upstate = gpio_get_value(info->irq_gpio);
		PRINTK("cvs_parity_updata upstate = %d\n",upstate);
		overtime_cnt++;
		if(overtime_cnt > 100) //1S timeout
		{
			PRINTK("cvs_parity_updata overtime 1sec err");
			return 1;
		}
	}
	while(upstate == 0);
	
	ret = cvs_read_reg(0xB1, &reg, 1);
	PRINTK("%s ret =%d\n",__func__,ret);
	if(reg == 0x00)
	{
		return 0;
	}
	else
	{
		PRINTK("cvs_parity_updata parity err");
		return 2;
	}
	
	return 0;
}

int cvs_if_firmware_need_updata(u8 *firmware_binfile_data)
{
	u16 local_firmware_version = 0, binfile_firmware_version = 0;
	
	binfile_firmware_version = ((firmware_binfile_data[0] << 8) & 0xff00) | firmware_binfile_data[1];
	local_firmware_version = cvs_get_firmware_version();
	
	if(local_firmware_version != binfile_firmware_version)
	{
		PRINTK("cvs_if_firmware_need_updata = 1");
		return 1;
	}
	else
	{
		PRINTK("cvs_if_firmware_need_updata = 0");
		return 0;
	}
}

int cvs_updata_firmware(u8 *firmware_binfile_data)
{
	int ret = 0;
	u8 firmware_dat_buf[130],i,buflen = 0;
	u16 firmware_length = 0,file_addr = 6,mtp_addr = 0;
	
	firmware_length = ((firmware_binfile_data[4] << 8) & 0xff00) | firmware_binfile_data[5];
	
	cvs_enter_updata_mode();
	if(cvs_if_updata_mode())
	{
		while(firmware_length)
		{
			firmware_dat_buf[0] = 0xBD;
			if(firmware_length > 64)
			{
				buflen = 65;
				firmware_length -= 64;
			}
			else
			{
				buflen = firmware_length + 1;
				firmware_length = 0;
			}
			
			for(i = 1;i < buflen;i++)
			{
				firmware_dat_buf[i] = firmware_binfile_data[file_addr];
				file_addr++;
			}
			
			ret = cvs_write_mtp(mtp_addr,firmware_dat_buf,buflen);
			if(ret == 0)
			{
				mtp_addr += 64;
			}
			else
			{
				PRINTK("cvs_updata_firmware write mtp overtime at mtp_addr = 0x%x!",mtp_addr);
				return 2;
			}
		}
		
		ret = cvs_parity_updata(firmware_binfile_data);
		if(ret)
		{
			PRINTK("cvs_updata_firmware parity updata overtime!");
			return 3;
		}
		
		if(cvs_if_firmware_need_updata(firmware_binfile_data))
		{
			PRINTK("cvs_updata_firmware fail!");
			return 4;
		}
		else
		{
			PRINTK("cvs_updata_firmware success!");
			return 0;
		}
	}
	else
	{
		PRINTK("cvs_updata_firmware mode err!");
		return 1;
	}
}

int cvs_updata_function(void)
{
	int ret = 0;
	
	if(info->firmware_updata_feature == 0 )
       return -1;
	
	cvs_updata_mode = 1;
	
	if(cvs_if_firmware_need_updata(cvs_firmware_data))
	{
		ret = cvs_updata_firmware(cvs_firmware_data);
		if(ret)
		{
			PRINTK("cvs_updata_function updata err!");
		}
		else
		{
			PRINTK("cvs_updata_function updata success!");
		}
	}
	else
	{
		PRINTK("cvs_updata_function cvs no need to updata!");
	}
	
	cvs_updata_mode = 0;
	
	return 0;
}
EXPORT_SYMBOL(cvs_updata_function);
// end   update firwmare

static int cvs_check_output_voltage(void){
	int ret = 0;
	u16 vol =0;
	//output voltage 10mv/unit in reg
	ret = cvs_read_reg16(info->addr.adc_vout_h, &vol);
	vol = vol * 10;
	PRINTK("%s voltage =%d\n",__func__,vol);
	return vol;
}

int get_wireless_charge_current(struct charger_data *pdata){

    PRINTK("enter %s input_current =%d ma charge_current =%d ma\n",__func__,info->input_current,info->charge_current);
	if(info->max_power > 0){
	  if(info->max_power >= 15){
	  	 PRINTK("enter %s max_power > 15 max_power =%d\n",__func__,info->max_power);
	     info->max_power = 15;
	  	}
	  //prize-Solve the problem of 5W over current-pengzhipeng-20210410-start
	  info->input_current = (info->max_power*1000000.0)/(11/1.0);
	  info->charge_current = (info->max_power*1000000.0)/(4/1.0);
	}else {
	  info->input_current = 500000;
	  info->charge_current = 500000;
	}

	if(info->max_power == 5){
	  info->input_current = 900000;
	  info->charge_current = 1000000;
	}
	if(info->max_power == 10){
	  info->input_current = 980000;
	  info->charge_current = info->charge_current;
	}
	
	if(info->max_power > 10){
		  if(cvs_check_output_voltage() > 8000){
	         info->input_current = info->input_current;
		     info->charge_current = info->charge_current;
		  }
	}
	  //prize-Solve the problem of 5W over current-pengzhipeng-20210410-end

	pdata->input_current_limit = info->input_current;
	pdata->charging_current_limit = info->charge_current;
	
	PRINTK("out %s input_current =%d ma charge_current =%d ma\n",__func__,info->input_current,info->charge_current);
    return 0;
}
EXPORT_SYMBOL(get_wireless_charge_current);

int cvs_chk_alive(void){
	int ret = 0;
	u8 reg = 0;
	u16 reg16 = 0;
	
	u16 chiptype = 0,output_cur = 0,output_vol = 0;
 	u8 ver_major = 0,verh = 0,verl = 0,txpower = 0;
	
	ret = cvs_read_reg16(info->addr.chipid_h, &chiptype);
	
	if (ret)
	{
		PRINTK("%s cvs_read_reg16 error\n",__func__);
		return -EINVAL;
	}
	
	if(chiptype == CHIP_ID_WRX)
	{
		//firmware version
		cvs_read_reg(info->addr.fw_ver_h, &ver_major, 1);
		
		cvs_read_reg(info->addr.fw_ver_l, &verh, 1);
		verl = verh & 0x0f;
		verh >>= 4;
		
		//output voltage 10mv/unit in reg 
		cvs_read_reg16(info->addr.adc_vout_h, &reg16);
		output_vol = reg16 * 10;
		
		//output current 10mA/unit in reg
		cvs_read_reg(info->addr.adc_iout, &reg, 1);
		output_cur = reg * 10;
		
		//tx max power
		reg = 0;
		cvs_read_reg(info->addr.tx_max_power, &reg, 1);
		txpower = reg / 2;
		info->max_power = txpower;
		
		PRINTK("cvs_chk_alive chiptype is 0x%x, firmware version is %d.%d.%d, output voltage = %dmV,output current = %dmA, TX max power = %dW\n",
			chiptype,ver_major,verh,verl,output_vol,output_cur,txpower);
		
		return 1;
	}
	else
	{
		PRINTK("cvs_chk_alive chiptype error\n");
		return 0;
	}
}
EXPORT_SYMBOL(cvs_chk_alive);

int cvs_ocp_switch(u8 setting)
{
	int ret = 0;
	u8 reg[2];

	reg[0] = info->addr.ocp_switch;
	reg[1] = setting;
	
	ret = cvs_write_reg(reg,2);
	PRINTK("%s ret =%d\n",__func__,ret);
	return 0;
}

int cvs_set_ocp(u16 cur)
{
	int ret = 0;
	u8 reg[2];
	
	cur = cur / 10;
	reg[0] = info->addr.iocp_set;
	reg[1] = cur;
	
	ret = cvs_write_reg(reg,2);
	PRINTK("%s ret =%d\n",__func__,ret);
	return 0;
}

int cvs_set_output_voltage(u16 vol)
{
	int ret = 0;
	u8 reg[3];
	
	vol = vol / 10;
	reg[0] = info->addr.vout_set_h;
	reg[1] = vol >> 8;
	reg[2] = vol;
	ret = cvs_write_reg(reg,3);
	PRINTK("%s ret =%d\n",__func__,ret);
	return 0;
}

void cvs_info_initialize(void){
  info->wpc_sys_sta.sys_sta_code = 0;
  info->wpc_sys_sta.sys_inf_flag.val = 0;
  info->wpc_sys_sta.tx_maximum_power = 0;
  info->input_current = 0;
  info->charge_current = 0;
  info->max_power = 0;
}
EXPORT_SYMBOL(cvs_info_initialize);

static int cvs_i2c_IRQhandle(void)
{
	int ret = 0;
	u8 r_buf[3] ={0};
	u8 *reg = r_buf; 
	
	ret = cvs_read_reg(info->addr.system_status, &info->wpc_sys_sta.sys_sta_code, 1);
	if (ret)
	{
		PRINTK("%s cvs_read_reg REG_SYSTEM_STATUS  error\n",__func__);
		return -EINVAL;
	}

	ret = cvs_read_reg(info->addr.int_flag, &info->wpc_sys_sta.sys_inf_flag.val,1);
	if (ret)
	{
		PRINTK("%s cvs_read_reg REG_INTR  error\n",__func__);
		return -EINVAL;
	}

	ret = cvs_read_reg(info->addr.tx_max_power, &info->wpc_sys_sta.tx_maximum_power,1);	
	if (ret)
	{
		PRINTK("%s cvs_read_reg REG_TX_MAX_POWER  error\n",__func__);
		return -EINVAL;
	}

	switch(info->wpc_sys_sta.sys_sta_code)
	{
		case CVS_NO_ERR:
			if(info->wpc_sys_sta.sys_inf_flag.msg.ldo_sta ==  CVS_LDO_ON)
			{
				cvs_set_ocp(1800);				
				info->max_power = info->wpc_sys_sta.tx_maximum_power/2;
				PRINTK("CVS_LDO_ON TX max power = %dW  tx_maximum_power =%d\n",info->max_power,info->wpc_sys_sta.tx_maximum_power);
				info->charge_current =500000;
	            info->input_current =500000;
				if(info->max_power > 9)
				 cvs_set_output_voltage(9000);
				
			}
			break;
		
		case CVS_SYS_ERR:		//wireless charger system error
		    if(info->wpc_sys_sta.sys_inf_flag.msg.ovp == 1)
			{
				PRINTK("wireless charger CV8035D over voltage error\n");
			}
			
			if(info->wpc_sys_sta.sys_inf_flag.msg.ocp == 1)
			{
				PRINTK("wireless charger CV8035D over current error\n");
			}
			
            if(info->wpc_sys_sta.sys_inf_flag.msg.otp == 1)
			{
				PRINTK("wireless charger CV8035D over temperature error\n");
			}
			
			//clear system state
			reg[0] = info->addr.system_status;
			reg[1] = 0x00;
			cvs_write_reg(reg,2);
			break;
			
		case CVS_VOUT_SET_ERR:
			PRINTK("wireless charger CV8035D output voltage set error\n");
			
			//clear system state
			reg[0] = info->addr.system_status;
			reg[1] = 0x00;
			cvs_write_reg(reg,2);
			break;
			
		default:
		   PRINTK("sys_sta_code is invalid\n");
		   break;
	}
	
	return 0;
}

/*start prize added by sunshuai, 20190213,open wrx_disable_vout*/
int wrx_disable_vout()
{
	int ret = 0;
	u8 reg[2] ={0};	
	
	reg[0] = info->addr.reg_enable;
	reg[1] = WRX_WATTINT_MODE;
	
	ret = cvs_write_reg(reg,2);
	PRINTK("%s ret =%d\n",__func__,ret);

	return ret;
}
EXPORT_SYMBOL(wrx_disable_vout);
/*end prize added by sunshuai, 20190213,open wrx_disable_vout*/


//prize added by sunshuai Modification did not reach the high temperature 55 standard stop charging problem for ne6153 201900413 start 
bool get_cvs_ischarge_9V(void)
{
       int real_vout =0;  

    if(cvs_chk_alive() == 1)
		pr_err("%s: cv8035d is active\n",__func__);
	else{
		pr_err("%s: cv8035d is no active\n",__func__);
		return false;
	}
    real_vout = cvs_check_output_voltage();
	PRINTK("get_cvs_ischarge_9V   vout = <%d>\n",real_vout);

	if(real_vout <=0)//add by sunshuai for judge 6153 is up 9V
	{
	   	PRINTK("read 6153 voltage is 0\n");
		return false;
	}

	if (real_vout > 8000){
		PRINTK("tune vout success <%d>\n",real_vout);
		return true;
	}else{
		return false;
	}
}
EXPORT_SYMBOL(get_cvs_ischarge_9V);
//prize added by sunshuai Modification did not reach the high temperature 55 standard stop charging problem for ne6153 201900413 end 

int get_cv8035d_status(void){

    pr_err("%s: status_gpio [%d]\n",__func__,gpio_get_value(info->status_gpio));
    if(cvs_chk_alive() == 1){
		pr_err("%s: cv8035d is active\n",__func__);
		return 1;
	}else{
		pr_err("%s: cv8035d is no active\n",__func__);
		return 0;
	}
}
EXPORT_SYMBOL(get_cv8035d_status);

static int cv8035d_parse_dt(struct dev_info *dev_info,struct device_node *np)
{
    int ret = 0;
	u32 val;

    dev_info->status_gpio = of_get_named_gpio(np, "statu-gpio", 0);
    if (dev_info->status_gpio < 0) {
        printk("%s: no status gpio provided\n", __func__);
        return -1;
    } else {
        printk( "%s: status gpio provided ok\n", __func__);
    }
	
	dev_info->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
    if (dev_info->irq_gpio < 0) {
        printk("%s: no irq gpio provided\n", __func__);
        return -1;
    } else {
        printk( "%s: irq gpio provided ok\n", __func__);
    }
	
	if (of_property_read_u32(np, "firmware_updata_feature", &val) >= 0)
		dev_info->firmware_updata_feature = val;
	else {
		dev_info->firmware_updata_feature = 0;
        PRINTK("%s use default val firmware_updata_feature =%d\n",__func__,dev_info->firmware_updata_feature);
	}
	PRINTK("%s firmware_updata_feature =%d\n",__func__,dev_info->firmware_updata_feature);
	
   dev_info->otg_en.pinctrl_gpios = devm_pinctrl_get(dev_info->dev);
   if (IS_ERR(dev_info->otg_en.pinctrl_gpios)) {
		ret = PTR_ERR(dev_info->otg_en.pinctrl_gpios);
		printk("%s can't find chg_data pinctrl\n", __func__);
		return ret;
   }
   
	dev_info->otg_en.pins_default = pinctrl_lookup_state(dev_info->otg_en.pinctrl_gpios, "default");
	if (IS_ERR(dev_info->otg_en.pins_default)) {
		ret = PTR_ERR(dev_info->otg_en.pins_default);
		printk("%s can't find chg_data pinctrl default\n", __func__);
		/* return ret; */
	}

	dev_info->otg_en.pins_otg_high = pinctrl_lookup_state(dev_info->otg_en.pinctrl_gpios, "charger_otg_on");
	if (IS_ERR(dev_info->otg_en.pins_otg_high)) {
		ret = PTR_ERR(dev_info->otg_en.pins_otg_high);
		printk("%s  can't find chg_data pinctrl otg high\n", __func__);
		return ret;
	}

	dev_info->otg_en.pins_otg_low = pinctrl_lookup_state(dev_info->otg_en.pinctrl_gpios, "charger_otg_off");
	if (IS_ERR(dev_info->otg_en.pins_otg_low)) {
		ret = PTR_ERR(dev_info->otg_en.pins_otg_low);
		printk("%s  can't find chg_data pinctrl otg low\n", __func__);
		return ret;
	}

	dev_info->otg_en.gpio_otg_prepare = true;


    return 0;
}

static void cvs8035d_eint_work(struct work_struct *work)
{ 
    PRINTK("enter %s\n",__func__);
	cvs_i2c_IRQhandle();
	PRINTK("out %s\n",__func__);
}

static irqreturn_t cv8035d_irq(int irq,void * data){
    PRINTK("%s\n",__func__);
	schedule_delayed_work(&info->eint_work, 0);
    return IRQ_HANDLED;
}

static int cvs_get_addr(struct cvs_i2c_addr *addr)
{
	if (!addr)
	{
		return -EFAULT;
	}

	addr->chipid_h = REG_CHIP_ID_H;
	addr->chipid_l = REG_CHIP_ID_L;
	addr->fw_ver_h = REG_FW_VER_H;
	addr->fw_ver_l = REG_FW_VER_L;
	addr->adc_vout_h = REG_ADC_VOUT_H;
	addr->adc_vout_l = REG_ADC_VOUT_L;
    addr->vout_set_h = REG_VOUT_SET_H;
	addr->vout_set_l = REG_VOUT_SET_L;
	addr->adc_iout = REG_ADC_IOUT;
	addr->ocp_switch = REG_OCP_SWITCH;
	addr->iocp_set = REG_ADC_IOCP_SET;
	addr->reg_enable = REG_ENABLE;
	addr->system_status = REG_SYSTEM_STATUS;
	addr->int_flag = REG_INTR;
	addr->tx_max_power = REG_TX_MAX_POWER;
	return 0;
}



static int cv8035d_probe(struct i2c_client *client, const struct i2c_device_id *id) {
	int ret =0 ;
	int irq_flags = 0;
	struct device_node *np = client->dev.of_node;
	
	info->client = client;
    info->dev = &client->dev;

    device_init_wakeup(info->dev, true);
    cvs_get_addr(&info->addr);

	if (np) {
	   ret = cv8035d_parse_dt(info,np);
	   if(ret)
		   return ret;
	}
	else
	{
		info->status_gpio = -1;
		info->irq_gpio = -1;
	}

	PRINTK("%s\n",__func__);
	
	irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
    ret = devm_request_threaded_irq(&client->dev, gpio_to_irq(info->irq_gpio),
						NULL, cv8035d_irq, irq_flags, "cv8035d", info);
    if (ret != 0) {
		pr_err("failed to request IRQ %d: %d\n", gpio_to_irq(info->irq_gpio), ret);
		goto err;
    }
	pr_err("sucess to request IRQ %d: %d\n", gpio_to_irq(info->irq_gpio), ret);

	if(!(gpio_get_value(info->irq_gpio))){
		pr_err("%s The interruption has come \n", __func__);
		cvs_i2c_IRQhandle();
	}
		
	INIT_DELAYED_WORK(&info->eint_work, cvs8035d_eint_work);
    cvs_info_initialize();
err:
	return ret;

}

static int cv8035d_remove(struct i2c_client *client) {
	
    if (gpio_is_valid(info->irq_gpio))
        devm_gpio_free(&client->dev, info->irq_gpio);
    if (gpio_is_valid(info->status_gpio))
        devm_gpio_free(&client->dev, info->status_gpio);
    return 0;
}

static const struct of_device_id match_table[] = {
    {.compatible = "cvs,cv8035d",},
    { },
};

static const struct i2c_device_id cv8035d_dev_id[] = {
    {"cv8035d", 0},
    {},
};
MODULE_DEVICE_TABLE(i2c, cv8035d_dev_id);

static struct i2c_driver cv8035d_driver = {
    .driver   = {
        .name           = "cv8035d",
        .owner          = THIS_MODULE,
        .of_match_table = match_table,
    },
    .probe    = cv8035d_probe,
    .remove   = cv8035d_remove,
    .id_table = cv8035d_dev_id,
};
module_i2c_driver(cv8035d_driver);

MODULE_AUTHOR("PRIZE");
MODULE_DESCRIPTION("prize cvs8035d Wireless Power Charger Monitor driver");
MODULE_LICENSE("GPL v2");
