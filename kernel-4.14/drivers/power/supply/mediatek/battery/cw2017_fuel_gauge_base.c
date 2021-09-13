#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/sizes.h>

//prize-add-sunshuai-2017 Multi-Battery Solution Compatible with IIO interface 20200904-start
#if defined(CONFIG_MACH_MT6768)&& defined(CONFIG_MEDIATEK_MT6577_AUXADC)
#include <linux/iio/consumer.h>
#endif
//prize-add-sunshuai-2017 Multi-Battery Solution Compatible with IIO interface 20200904-end

#define CWFG_ENABLE_LOG 1 //CHANGE   Customer need to change this for enable/disable log
#define CWFG_I2C_BUSNUM 5 //CHANGE   Customer need to change this number according to the principle of hardware
#define DOUBLE_SERIES_BATTERY 0

#define CW_PROPERTIES "cw-bat"//prize-Solve the problem that the 8804 does not display the electricity meterpengzhipeng-20210723

#define REG_VERSION             0x00
#define REG_VCELL_H             0x02
#define REG_VCELL_L             0x03
#define REG_SOC_INT             0x04
#define REG_SOC_DECIMAL         0x05
#define REG_TEMP                0x06
#define REG_MODE_CONFIG         0x08
#define REG_GPIO_CONFIG         0x0A
#define REG_SOC_ALERT           0x0B
#define REG_TEMP_MAX            0x0C
#define REG_TEMP_MIN            0x0D
#define REG_VOLT_ID_H           0x0E
#define REG_VOLT_ID_L           0x0F
#define REG_BATINFO             0x10

#define MODE_SLEEP              0x30
#define MODE_NORMAL             0x00
#define MODE_DEFAULT            0xF0
#define CONFIG_UPDATE_FLG       0x80
#define NO_START_VERSION        160

#define GPIO_CONFIG_MIN_TEMP             (0x00 << 4)
#define GPIO_CONFIG_MAX_TEMP             (0x00 << 5)
#define GPIO_CONFIG_SOC_CHANGE           (0x00 << 6)
#define GPIO_CONFIG_MIN_TEMP_MARK        (0x01 << 4)
#define GPIO_CONFIG_MAX_TEMP_MARK        (0x01 << 5)
#define GPIO_CONFIG_SOC_CHANGE_MARK      (0x01 << 6)
#define ATHD                              0x0		 //0x7F
#define DEFINED_MAX_TEMP                          450
#define DEFINED_MIN_TEMP                          0

#define DESIGN_CAPACITY                   4000
#define CWFG_NAME "cw2017"
#define SIZE_BATINFO    80


#define queue_delayed_work_time  8000

#define cw_printk(fmt, arg...)        \
	({                                    \
		if(CWFG_ENABLE_LOG){              \
			printk("FG_CW2017 : %s-%d : " fmt, __FUNCTION__ ,__LINE__,##arg);  \
		}else{}                           \
	})     //need check by Chaman

#if defined(CONFIG_PRIZE_HARDWARE_INFO)
#include "../../../misc/mediatek/hardware_info/hardware_info.h"
extern struct hardware_info current_coulo_info;
#endif

static unsigned char config_info[SIZE_BATINFO] = {
	0x5A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x9E,0xC8,0xD2,0xC5,0xC2,0xCF,0x53,0x25,
	0x10,0xF5,0xEB,0xE1,0xB7,0x93,0x83,0x6E,
	0x5D,0x4D,0x42,0x54,0x94,0xDC,0x76,0xD7,
	0xD7,0xD2,0xD2,0xD0,0xCE,0xCC,0xC4,0xCD,
	0xC3,0xBD,0xCB,0xAE,0x96,0x8A,0x83,0x75,
	0x67,0x61,0x76,0x8C,0xA4,0x96,0x50,0x66,
	0x00,0x00,0x90,0x02,0x00,0x00,0x00,0x00,
	0x00,0x00,0x64,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x22,
};

//static struct power_supply *chrg_usb_psy;
//static struct power_supply *chrg_ac_psy;

struct cw_battery {
    struct i2c_client *client;

    struct workqueue_struct *cwfg_workqueue;
	struct delayed_work battery_delay_work;
	#ifdef CW2017_INTERRUPT
	struct delayed_work interrupt_work;
	#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
	struct power_supply cw_bat;
#else
	struct power_supply *cw_bat;
#endif

	/*User set*/
	unsigned int design_capacity;
	/*IC value*/
	int version;
    int voltage;	
    int capacity;
	int temp;
	
	/*IC config*/
	unsigned char int_config;
	unsigned char soc_alert;
	int temp_max;
	int temp_min;
	
	/*Get before profile write*/	
	int volt_id;
	
	/*Get from charger power supply*/
	//unsigned int charger_mode;
	
	/*Mark for change cw_bat power_supply*/
	//int change;
};
//prize-add-sunshuai-2017 Multi-Battery Solution-20200222-start
#if defined(CONFIG_MTK_CW2017_BATTERY_ID_AUXADC)
struct cw2017batinfo {
   int bat_first_startrange;
   int bat_first_endrange;
   int first_bat_capacity;
   int bat_second_startrange;
   int bat_second_endrange;
   int sec_bat_capacity;
   int bat_third_startrange;
   int bat_third_endrange;
   int third_bat_capacity;
   int bat_channel_num;
   int bat_id;
//prize-add-sunshuai-2017 Multi-Battery Solution Compatible with IIO interface 20200904-start
#if defined(CONFIG_MACH_MT6768)&& defined(CONFIG_MEDIATEK_MT6577_AUXADC)
   bool enable_iio_interface;
#endif
//prize-add-sunshuai-2017 Multi-Battery Solution Compatible with IIO interface 20200904-end
};
static struct cw2017batinfo cw2017fuelguage;
static char *fuelguage_name[] = {
	"batinfo_first", "batinfo_second", "batinfo_third","batinfo_default"
};

//prize-add-sunshuai-2017 Multi-Battery Solution Compatible with IIO interface 20200904-start
#if defined(CONFIG_MACH_MT6768)&& defined(CONFIG_MEDIATEK_MT6577_AUXADC)
struct iio_channel *batid_channel;
#endif
//prize-add-sunshuai-2017 Multi-Battery Solution Compatible with IIO interface 20200904-end

#if !defined(CONFIG_MACH_MT6768)
extern int IMM_GetOneChannelValue_Cali(int Channel, int *voltage);
#endif

#endif
//prize-add-sunshuai-2017 Multi-Battery Solution-20200222-end

int g_cw2017_capacity = 0;
int g_cw2017_vol = 0;
int cw2017_exit_flag=0;
int g_cw2017_bat_temperature_val = 0;
/*Define CW2017 iic read function*/
static int cw_read(struct i2c_client *client, unsigned char reg, unsigned char buf[])
{
	int ret = 0;
	ret = i2c_smbus_read_i2c_block_data( client, reg, 1, buf);
	if(ret < 0){
		printk("IIC error %d\n", ret);
	}
	return ret;
}
/*Define CW2017 iic write function*/		
static int cw_write(struct i2c_client *client, unsigned char reg, unsigned char const buf[])
{
	int ret = 0;
	ret = i2c_smbus_write_i2c_block_data( client, reg, 1, &buf[0] );
	if(ret < 0){
		printk("IIC error %d\n", ret);
	}
	return ret;
}
/*Define CW2017 iic read word function*/	
static int cw_read_word(struct i2c_client *client, unsigned char reg, unsigned char buf[])
{
	int ret = 0;
	ret = i2c_smbus_read_i2c_block_data( client, reg, 2, buf );
	if(ret < 0){
		printk("IIC error %d\n", ret);
	}
	return ret;
}

static int cw2017_enable(struct cw_battery *cw_bat)
{
	int ret;
    unsigned char reg_val = MODE_DEFAULT;
	printk("cw2017_enable!!!\n");

	ret = cw_write(cw_bat->client, REG_MODE_CONFIG, &reg_val);
	if(ret < 0)
		return ret;
	
	msleep(20); 
	reg_val = MODE_SLEEP;
	ret = cw_write(cw_bat->client, REG_MODE_CONFIG, &reg_val);
	if(ret < 0)
		return ret;
	
	msleep(20);
	reg_val = MODE_NORMAL;
	ret = cw_write(cw_bat->client, REG_MODE_CONFIG, &reg_val);
	if(ret < 0)
		return ret;
	
	msleep(20);
	return 0;	
}


static int cw_get_version(struct cw_battery *cw_bat)
{
	int ret = 0;
	unsigned char reg_val = 0;
	int version = 0;
	ret = cw_read(cw_bat->client, REG_VERSION, &reg_val);
	if(ret < 0)
		return INT_MAX;
	
	version = reg_val;	 
	printk("version = %d\n", version);
	return version;	
}

static int cw_get_voltage(struct cw_battery *cw_bat)
{
	int ret = 0;
	unsigned char reg_val[2] = {0 , 0};
	unsigned int voltage = 0;

	ret = cw_read_word(cw_bat->client, REG_VCELL_H, reg_val);
	if(ret < 0)
		return INT_MAX;
	
	voltage = (reg_val[0] << 8) + reg_val[1];
	voltage = voltage  * 5 / 16;

	return(voltage); 
}

#define UI_FULL 100
#define DECIMAL_MAX 80
#define DECIMAL_MIN 20 
static int cw_get_capacity(struct cw_battery *cw_bat)
{
	int ret = 0;
	unsigned char reg_val = 0;
	int soc = 0;
	int soc_decimal = 0;
	int UI_SOC = 0;
	int UI_decimal = 0;
	static int reset_loop = 0;
	ret = cw_read(cw_bat->client, REG_SOC_INT, &reg_val);
	if(ret < 0)
		return INT_MAX;
	soc = reg_val;
	
	ret = cw_read(cw_bat->client, REG_SOC_DECIMAL, &reg_val);
	if(ret < 0)
		return INT_MAX;
	soc_decimal = reg_val;
		
	if(soc > 100){		
		reset_loop++;
		printk("IC error read soc error %d times\n", reset_loop);
		if(reset_loop > 5){
			reset_loop = 0;
			printk("IC error. please reset IC");
			cw2017_enable(cw_bat); //here need modify
		}
		return cw_bat->capacity;
	}
	else{
		reset_loop = 0; 
	}
	
	UI_SOC = ((soc * 256 + soc_decimal) * 100)/ (UI_FULL*256);
	UI_decimal = (((soc * 256 + soc_decimal) * 100 * 100) / (UI_FULL*256)) % 100;
	cw_printk(KERN_INFO "CW2017[%d]: UI_FULL = %d, UI_SOC = %d, UI_decimal = %d soc = %d, soc_decimal = %d\n", __LINE__, UI_FULL, UI_SOC, UI_decimal, soc, soc_decimal);
	
	/* case 1 : aviod swing */
	if(UI_SOC >= 100){
		printk(KERN_INFO "CW2017[%d]: UI_SOC = %d larger 100!!!!\n", __LINE__, UI_SOC);
		UI_SOC = 100;
	}else if((UI_SOC >= (cw_bat->capacity - 1)) && (UI_SOC <= (cw_bat->capacity + 1)) 
		&& ((UI_decimal > DECIMAL_MAX) || (UI_decimal < DECIMAL_MIN)) && (UI_SOC != 100)){
		UI_SOC = cw_bat->capacity;
	}	
	
	return UI_SOC;	
}

static int cw_get_temp(struct cw_battery *cw_bat)
{
	int ret = 0;
	unsigned char reg_val = 0;
	int temp = 0;
	ret = cw_read(cw_bat->client, REG_TEMP, &reg_val);
	if(ret < 0)
		return INT_MAX;
	
	temp = reg_val * 10 / 2 - 400;
	return temp;	
}


static void cw_update_data(struct cw_battery *cw_bat)
{	
	cw_bat->voltage = cw_get_voltage(cw_bat);
	cw_bat->capacity = cw_get_capacity(cw_bat);
	cw_bat->temp = cw_get_temp(cw_bat);
	printk("vol = %d  cap = %d temp = %d\n", 
		cw_bat->voltage, cw_bat->capacity, cw_bat->temp);
}

static int cw_init_data(struct cw_battery *cw_bat)
{
	cw_bat->version = cw_get_version(cw_bat);
	cw_bat->voltage = cw_get_voltage(cw_bat);
	cw_bat->capacity = cw_get_capacity(cw_bat);
	cw_bat->temp = cw_get_temp(cw_bat);
	g_cw2017_capacity = cw_bat->capacity;
    g_cw2017_vol = cw_bat->voltage;
	g_cw2017_bat_temperature_val = cw_bat->temp;
	if(cw_bat->version == INT_MAX){
		return -1;
	}
	printk("ver = %d vol = %d  cap = %d temp = %d\n", 
		cw_bat->version, cw_bat->voltage, cw_bat->capacity, cw_bat->temp);
	return 0;
}

static int cw_init_config(struct cw_battery *cw_bat)
{
	int ret = 0;
	unsigned char reg_gpio_config = 0;
	unsigned char athd = 0;
	unsigned char reg_val = 0;

	cw_bat->design_capacity = DESIGN_CAPACITY;
	/*IC config*/
	cw_bat->int_config = GPIO_CONFIG_MIN_TEMP | GPIO_CONFIG_MAX_TEMP | GPIO_CONFIG_SOC_CHANGE;
	cw_bat->soc_alert = ATHD;
	cw_bat->temp_max = DEFINED_MAX_TEMP;
	cw_bat->temp_min = DEFINED_MIN_TEMP;

	reg_gpio_config = cw_bat->int_config;

	ret = cw_read(cw_bat->client, REG_SOC_ALERT, &reg_val);
	if(ret < 0)
		return ret;
	
	athd = reg_val & CONFIG_UPDATE_FLG; //clear athd
	athd = athd | cw_bat->soc_alert;

	if(reg_gpio_config & GPIO_CONFIG_MAX_TEMP_MARK)
	{
		reg_val = (cw_bat->temp_max + 400) * 2 /10;
		ret = cw_write(cw_bat->client, REG_TEMP_MAX, &reg_val); 
		if(ret < 0)
			return ret;
	}
	if(reg_gpio_config & GPIO_CONFIG_MIN_TEMP_MARK)
	{
		reg_val = (cw_bat->temp_min + 400) * 2 /10;
		ret = cw_write(cw_bat->client, REG_TEMP_MIN, &reg_val); 
		if(ret < 0)
			return ret;
	}
	
	ret = cw_write(cw_bat->client, REG_GPIO_CONFIG, &reg_gpio_config); 
	if(ret < 0)
		return ret;
	
	ret = cw_write(cw_bat->client, REG_SOC_ALERT, &athd);
	if(ret < 0)
		return ret;
	 
	return 0;
}

/*CW2017 update profile function, Often called during initialization*/
static int cw_update_config_info(struct cw_battery *cw_bat)
{
	int ret = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;
	unsigned char reg_val_dig = 0;
	int count = 0;

	/* update new battery info */
	for(i = 0; i < SIZE_BATINFO; i++)
	{
		reg_val = config_info[i];
		ret = cw_write(cw_bat->client, REG_BATINFO + i, &reg_val);
        if(ret < 0) 
			return ret;
		printk("w reg[%02X] = %02X\n", REG_BATINFO +i, reg_val);
	}
	
	ret = cw_read(cw_bat->client, REG_SOC_ALERT, &reg_val);
	if(ret < 0)
		return ret;
	
	reg_val |= CONFIG_UPDATE_FLG;   /* set UPDATE_FLAG */
	ret = cw_write(cw_bat->client, REG_SOC_ALERT, &reg_val);
	if(ret < 0)
		return ret;

	ret = cw2017_enable(cw_bat);
	if(ret < 0) 
		return ret;
	
	while(cw_get_version(cw_bat) == NO_START_VERSION){
		msleep(100);
		count++;
		if(count > 30)
			break;
	}

	for (i = 0; i < 30; i++) {
		msleep(100);
        ret = cw_read(cw_bat->client, REG_SOC_INT, &reg_val);
        ret = cw_read(cw_bat->client, REG_SOC_INT + 1, &reg_val_dig);
		printk("i = %d soc = %d, .soc = %d\n", i, reg_val, reg_val_dig);
        if (ret < 0)
            return ret;
        else if (reg_val <= 100) 
            break;	
    }
	
	return 0;
}

/*CW2017 init function, Often called during initialization*/
static int cw_init(struct cw_battery *cw_bat)
{
    int ret;
    int i;
    unsigned char reg_val = MODE_NORMAL;
	unsigned char config_flg = 0;

	ret = cw_read(cw_bat->client, REG_MODE_CONFIG, &reg_val);
	if(ret < 0)
		return ret;
	
	ret = cw_read(cw_bat->client, REG_SOC_ALERT, &config_flg);
	if(ret < 0)
		return ret;

	if(reg_val != MODE_NORMAL || ((config_flg & CONFIG_UPDATE_FLG) == 0x00)){
		ret = cw_update_config_info(cw_bat);
		if(ret < 0)
			return ret;
	} else {
		for(i = 0; i < SIZE_BATINFO; i++)
		{ 
			ret = cw_read(cw_bat->client, REG_BATINFO +i, &reg_val);
			if(ret < 0)
				return ret;
			
			printk("r reg[%02X] = %02X\n", REG_BATINFO +i, reg_val);
			if(config_info[i] != reg_val)
			{
				break;
			}
		}
		if(i != SIZE_BATINFO)
		{
			//"update flag for new battery info need set"
			ret = cw_update_config_info(cw_bat);
			if(ret < 0)
				return ret;
		}
	}
	cw_printk("cw2017 init success!\n");	
	return 0;
}

static void cw_bat_work(struct work_struct *work)
{
    struct delayed_work *delay_work;
    struct cw_battery *cw_bat;
	
    delay_work = container_of(work, struct delayed_work, work);
    cw_bat = container_of(delay_work, struct cw_battery, battery_delay_work);

	cw_update_data(cw_bat);	
	g_cw2017_capacity = cw_bat->capacity;
    g_cw2017_vol = cw_bat->voltage;
	g_cw2017_bat_temperature_val = cw_bat->temp;
	#ifdef CW_PROPERTIES
	#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
	power_supply_changed(&cw_bat->cw_bat); 
	#else
	power_supply_changed(cw_bat->cw_bat); 
	#endif
	#endif
	
	queue_delayed_work(cw_bat->cwfg_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(queue_delayed_work_time));
}

#ifdef CW_PROPERTIES
static int cw_battery_get_property(struct power_supply *psy,
                enum power_supply_property psp,
                union power_supply_propval *val)
{
    int ret = 0;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
    struct cw_battery *cw_bat;
    cw_bat = container_of(psy, struct cw_battery, cw_bat); 
#else
	struct cw_battery *cw_bat = power_supply_get_drvdata(psy); 
#endif

    switch (psp) {
    case POWER_SUPPLY_PROP_CAPACITY:
            val->intval = cw_bat->capacity;
            break;
	/*
    case POWER_SUPPLY_PROP_STATUS:   //Chaman charger ic will give a real value
            val->intval = cw_bat->status; 
            break;                 
    */      
    case POWER_SUPPLY_PROP_HEALTH:   //Chaman charger ic will give a real value
            val->intval= POWER_SUPPLY_HEALTH_GOOD;
            break;
    case POWER_SUPPLY_PROP_PRESENT:
            val->intval = cw_bat->voltage <= 0 ? 0 : 1;
            break;
            
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
            val->intval = cw_bat->voltage * 1000;
            break;
        
    case POWER_SUPPLY_PROP_TECHNOLOGY:  //Chaman this value no need
            val->intval = POWER_SUPPLY_TECHNOLOGY_LION;	
            break;

    case POWER_SUPPLY_PROP_TEMP: 
            val->intval = cw_bat->temp;	
            break;

/*    case POWER_SUPPLY_PROP_TEMP_ALERT_MIN: 
            val->intval = cw_bat->temp_min;	
            break;

    case POWER_SUPPLY_PROP_TEMP_ALERT_MAX: 
            val->intval = cw_bat->temp_max;	
            break;*/

    case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN: 
            val->intval = cw_bat->design_capacity;	
            break;

    default:
			ret = -EINVAL; 
            break;
    }
    return ret;
}

static enum power_supply_property cw_battery_properties[] = {
    POWER_SUPPLY_PROP_CAPACITY,
    //POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_HEALTH,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_TEMP,
//	POWER_SUPPLY_PROP_TEMP_ALERT_MIN,
//	POWER_SUPPLY_PROP_TEMP_ALERT_MAX,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
};
#endif 

#ifdef CW2017_INTERRUPT

#define WAKE_LOCK_TIMEOUT       (10 * HZ)
static struct wake_lock cw2017_wakelock;

static void interrupt_work_do_wakeup(struct work_struct *work)
{
        struct delayed_work *delay_work;
        struct cw_battery *cw_bat;
		int ret = 0;
		unsigned char reg_val = 0;

        delay_work = container_of(work, struct delayed_work, work);
        cw_bat = container_of(delay_work, struct cw_battery, interrupt_work);
		
		ret = cw_read(cw_bat->client, REG_GPIO_CONFIG, &reg_val); 
		if(ret < 0)
			return ret;	
		/**/
}

static irqreturn_t ops_cw2017_int_handler_int_handler(int irq, void *dev_id)
{
        struct cw_battery *cw_bat = dev_id;
        wake_lock_timeout(&cw2017_wakelock, WAKE_LOCK_TIMEOUT);
        queue_delayed_work(cw_bat->cwfg_workqueue, &cw_bat->interrupt_work, msecs_to_jiffies(20));
        return IRQ_HANDLED;
}

#endif

//prize-add-sunshuai-2017 Multi-Battery Solution-20200222-start
#if defined(CONFIG_MTK_CW2017_BATTERY_ID_AUXADC)
int fgauge2017_get_profile_id(struct device_node *np){
   int Voltiage_cali =0 ;
   int val = 0;
   int ret=0;
	
   if (of_property_read_u32(np, "bat_first_startrange", &val) >= 0)
      cw2017fuelguage.bat_first_startrange = val;
   else {
      cw_printk("[%s] get  bat_first_startrange  fail\n", __func__);
	}

   if (of_property_read_u32(np, "bat_first_endrange", &val) >= 0)
      cw2017fuelguage.bat_first_endrange = val;
   else {
	  cw_printk("[%s] get  bat_first_endrange  fail\n", __func__);
	}

   if (of_property_read_u32(np, "bat_second_startrange", &val) >= 0)
		cw2017fuelguage.bat_second_startrange = val;
   else {
	   cw_printk("[%s] get  bat_second_startrange  fail\n", __func__);
	}

   if (of_property_read_u32(np, "bat_second_endrange", &val) >= 0)
	   cw2017fuelguage.bat_second_endrange = val;
   else {
	   cw_printk("[%s] get  bat_second_endrange  fail\n", __func__);
   }

   if (of_property_read_u32(np, "bat_third_startrange", &val) >= 0)
	   cw2017fuelguage.bat_third_startrange = val;
   else {
	   cw_printk("[%s] get  bat_third_startrange  fail\n", __func__);
   }

   if (of_property_read_u32(np, "bat_third_endrange", &val) >= 0)
	   cw2017fuelguage.bat_third_endrange = val;
   else {
	   cw_printk("[%s] get  bat_third_endrange  fail\n", __func__);
   }

#if !defined(CONFIG_MACH_MT6768)
   if (of_property_read_u32(np, "bat_channel_num", &val) >= 0)
	   cw2017fuelguage.bat_channel_num = val;
   else {
	   cw_printk("[%s] get  bat_channel_num  fail\n", __func__);
   }
	cw_printk(1, "[cw2017] cw2017fuelguage.bat_channel_num =%d \n",cw2017fuelguage.bat_channel_num);
#endif
//prize-add-sunshuai-2017 Multi-Battery Solution Compatible with IIO interface 20200904-end
   
   if (of_property_read_u32(np, "first_bat_capacity", &val) >= 0)
	   cw2017fuelguage.first_bat_capacity = val;
   else {
	   cw_printk("[%s] get  first_bat_capacity  fail\n", __func__);
	   cw2017fuelguage.first_bat_capacity = 1000;
   }

   if (of_property_read_u32(np, "sec_bat_capacity", &val) >= 0)
	   cw2017fuelguage.sec_bat_capacity = val;
   else {
	   cw_printk("[%s] get  sec_bat_capacity  fail\n", __func__);
	   cw2017fuelguage.sec_bat_capacity = 1000;
   }

   if (of_property_read_u32(np, "third_bat_capacity", &val) >= 0)
	   cw2017fuelguage.third_bat_capacity = val;
   else {
	   cw_printk("[%s] get  third_bat_capacity  fail\n", __func__);
	   cw2017fuelguage.third_bat_capacity = 1000;
   }
   
   cw_printk("[cw2017] cw2017fuelguage.bat_first_startrange =%d \n",cw2017fuelguage.bat_first_startrange);
   cw_printk("[cw2017] cw2017fuelguage.bat_first_endrange =%d \n",cw2017fuelguage.bat_first_endrange);
   cw_printk("[cw2017] cw2017fuelguage.bat_second_startrange =%d \n",cw2017fuelguage.bat_second_startrange);
   cw_printk("[cw2017] cw2017fuelguage.bat_second_endrange =%d \n",cw2017fuelguage.bat_second_endrange);
   cw_printk("[cw2017] cw2017fuelguage.bat_third_startrange =%d \n",cw2017fuelguage.bat_third_startrange);
   cw_printk("[cw2017] cw2017fuelguage.bat_third_endrange =%d \n",cw2017fuelguage.bat_third_endrange);
   cw_printk("[cw2017] cw2017fuelguage.bat_channel_num =%d \n",cw2017fuelguage.bat_channel_num);
   cw_printk("[cw2017] cw2017fuelguage.first_bat_capacity =%d \n",cw2017fuelguage.first_bat_capacity);
   cw_printk("[cw2017] cw2017fuelguage.sec_bat_capacity =%d \n",cw2017fuelguage.sec_bat_capacity);
   cw_printk("[cw2017] cw2017fuelguage.third_bat_capacity =%d \n",cw2017fuelguage.third_bat_capacity);
   
//prize-add-sunshuai-2017 Multi-Battery Solution Compatible with IIO interface 20200904-start
#if defined(CONFIG_MACH_MT6768)&& defined(CONFIG_MEDIATEK_MT6577_AUXADC)
   if(cw2017fuelguage.enable_iio_interface){
       ret = iio_read_channel_processed(batid_channel, &val);
	   if (ret < 0) {
		   cw_printk("[%s] Busy/Timeout, IIO ch read failed %d\n",__func__,ret);
		    return ret;
	   }
	   Voltiage_cali = ((val * 1500) >> 12)* 1000;
	   cw_printk("[%s] info id_volt = %d val =%d\n", __func__,Voltiage_cali,val);

    }
#else
   ret= IMM_GetOneChannelValue_Cali(cw2017fuelguage.bat_channel_num, &Voltiage_cali);
   if (ret != 0){
      cw_printk("[%s] channel[%d] info id_volt read fail\n", __func__,cw2017fuelguage.bat_channel_num);
	  return -2;
    } else {
      cw_printk("[%s] channel[%d] info id_volt = %d\n", __func__, cw2017fuelguage.bat_channel_num,Voltiage_cali);
	}
#endif
//prize-add-sunshuai-2017 Multi-Battery Solution Compatible with IIO interface 20200904-end
  
   if(Voltiage_cali > cw2017fuelguage.bat_first_startrange && Voltiage_cali < cw2017fuelguage.bat_first_endrange)
	   cw2017fuelguage.bat_id = 0;
   else if(Voltiage_cali > cw2017fuelguage.bat_second_startrange && Voltiage_cali < cw2017fuelguage.bat_second_endrange)
	   cw2017fuelguage.bat_id = 1;
   else if(Voltiage_cali > cw2017fuelguage.bat_third_startrange && Voltiage_cali < cw2017fuelguage.bat_third_endrange)
	   cw2017fuelguage.bat_id = 2;
   else{
	   cw2017fuelguage.bat_id = 3;
	   cw_printk("[cw2017] cw2017_init did not find Curve corresponding to the battery ,use default Curve");
   }
		
   cw_printk("%s [cw2017]  Curve name %s",__func__,fuelguage_name[cw2017fuelguage.bat_id]);
   cw_printk("%s [cw2017]  cw2017fuelguage.bat_id = %d",__func__,cw2017fuelguage.bat_id);
   
   if(cw2017fuelguage.bat_id > 3 || cw2017fuelguage.bat_id < 0){
	   cw_printk("%s [cw2017] bat_id Invalid value ",__func__);
	   return -3;
   }

   return 0;
}


int get_muilt_bat_capacity(void){
	if(cw2017fuelguage.bat_id == 0){
		cw_printk("[cw2017]  user first_bat_capacity capacity = %d",cw2017fuelguage.first_bat_capacity);
		return cw2017fuelguage.first_bat_capacity;
	}else if(cw2017fuelguage.bat_id == 1){
		cw_printk("[cw2017]  user sec_bat_capacity capacity = %d",cw2017fuelguage.sec_bat_capacity);
		return cw2017fuelguage.sec_bat_capacity;
	}else if(cw2017fuelguage.bat_id == 2){
		cw_printk("[cw2017]  user third_bat_capacity capacity = %d",cw2017fuelguage.third_bat_capacity);
		return cw2017fuelguage.third_bat_capacity;
	}else{
		cw_printk("[cw2017]  default user first capacity = %d",cw2017fuelguage.first_bat_capacity);
		return cw2017fuelguage.first_bat_capacity;
	}
}
EXPORT_SYMBOL(get_muilt_bat_capacity);
#endif
//prize-add-sunshuai-2017 Multi-Battery Solution-20200222-end
static int cw2017_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret;
    int loop = 0;
	struct cw_battery *cw_bat;
#ifdef CW2017_INTERRUPT	
	int irq = 0;
#endif
	//prize add by huarui, support config by dts, 20190612 start
#if defined(CONFIG_MTK_CW2017_SUPPORT_OF)
	struct device_node *np = NULL;
	int size = 0;
	uint8_t buf[SIZE_BATINFO] = {0};
	int i;
#endif
//prize add by huarui, support config by dts, 20190612 end
//prize-add-sunshuai-2017 Multi-Battery Solution Compatible with IIO interface 20200904-start
#if defined(CONFIG_MACH_MT6768)&& defined(CONFIG_MEDIATEK_MT6577_AUXADC)
    const char *iio_name;
#endif
//prize-add-sunshuai-2017 Multi-Battery Solution Compatible with IIO interface 20200904-start
#ifdef CW_PROPERTIES
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0)	
	struct power_supply_desc *psy_desc;
	struct power_supply_config psy_cfg = {0};
#endif
#endif
    //struct device *dev;
	cw_printk("\n");

    cw_bat = devm_kzalloc(&client->dev, sizeof(*cw_bat), GFP_KERNEL);
    if (!cw_bat) {
		cw_printk("cw_bat create fail!\n");
        return -ENOMEM;
    }

    i2c_set_clientdata(client, cw_bat);
	 
    cw_bat->client = client;
	cw_bat->volt_id = 0;
	//prize add by huarui, support config by dts, 20190612 start
#if defined(CONFIG_MTK_CW2017_SUPPORT_OF)
	np = client->dev.of_node;
	if (np){
//prize-add-sunshuai-2017 Multi-Battery Solution Compatible with IIO interface 20200904-start
#if defined(CONFIG_MACH_MT6768)&& defined(CONFIG_MEDIATEK_MT6577_AUXADC)		
	   cw2017fuelguage.enable_iio_interface = of_property_read_bool(np, "enable_iio_interface");	
	   if(cw2017fuelguage.enable_iio_interface){
	      batid_channel = devm_kzalloc(&client->dev, sizeof(*batid_channel),GFP_KERNEL);
	      if (!batid_channel)
		      return -ENOMEM;
	      ret = of_property_read_string(np, "io-channel-names",&iio_name);
	      if (ret < 0)
	        cw_printk("%s no iio_name(%d)\n", __func__, ret);
	      else
	   	    cw_printk("%s iio_name (%s)\n", __func__,iio_name);
	      batid_channel = iio_channel_get(&client->dev, iio_name);
	   }
#endif
//prize-add-sunshuai-2017 Multi-Battery Solution Compatible with IIO interface 20200904-end

// prize-add-sunshuai-2015 Multi-Battery Solution-20200222-start
#if defined(CONFIG_MTK_CW2017_BATTERY_ID_AUXADC)
		if(fgauge2017_get_profile_id(np) == 0){
			size = of_property_count_u8_elems(np,fuelguage_name[cw2017fuelguage.bat_id]);
			cw_printk("cw_bat get %s batinfo size %d!\n",fuelguage_name[cw2017fuelguage.bat_id],size);
			if (size == SIZE_BATINFO){
				ret = of_property_read_u8_array(np,fuelguage_name[cw2017fuelguage.bat_id],buf,size);
				if (!ret){
					memcpy(config_info,buf,size);
					for(i=0;i<size;i++){
						printk("cw2017_probe get %s [%d] %x ",fuelguage_name[cw2017fuelguage.bat_id],i,config_info[i]);
			    }
				cw_printk("cw_bat get %s batinfo sucess size(%d)!\n",fuelguage_name[cw2017fuelguage.bat_id],size);
				}else{
					cw_printk("cw_bat get %s batinfo fail %d!\n",fuelguage_name[cw2017fuelguage.bat_id],ret);
				}
			}else{
				cw_printk("cw_bat get %s batinfo size fail %d!\n",fuelguage_name[cw2017fuelguage.bat_id],size);
			}
		}
#else
		size = of_property_count_u8_elems(np,"batinfo");
		cw_printk("cw_bat get batinfo size %d!\n",size);
		if (size == SIZE_BATINFO){
			ret = of_property_read_u8_array(np,"batinfo",buf,size);
			if (!ret){
				memcpy(config_info,buf,size);
				for(i=0;i<size;i++){
					printk("cw2017_probe[%d] %x ",i,config_info[i]);
			    }
				cw_printk("cw_bat get batinfo sucess size(%d)!\n",size);
			}else{
				cw_printk("cw_bat get batinfo fail %d!\n",ret);
			}
		}else{
			cw_printk("cw_bat get batinfo size fail %d!\n",size);
		}
#endif
// prize-add-sunshuai-2015 Multi-Battery Solution-20200222-end
   	}

#endif
//prize add by huarui, support config by dts, 20190612 end
    ret = cw_init(cw_bat);
    while ((loop++ < 3) && (ret != 0)) {
		msleep(200);
        ret = cw_init(cw_bat);
    }
    if (ret) {
		printk("%s : cw2017 init fail!\n", __func__);
        return ret;	
    }
	
	ret = cw_init_config(cw_bat);
	if (ret) {
		printk("%s : cw2017 init config fail!\n", __func__);
		return ret;
	}
	
	ret = cw_init_data(cw_bat);
    if (ret) {
		printk("%s : cw2017 init data fail!\n", __func__);
        return ret;	
    }

#ifdef CW_PROPERTIES
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
	cw_bat->cw_bat.name = CW_PROPERTIES;
	cw_bat->cw_bat.type = POWER_SUPPLY_TYPE_BATTERY;
	cw_bat->cw_bat.properties = cw_battery_properties;
	cw_bat->cw_bat.num_properties = ARRAY_SIZE(cw_battery_properties);
	cw_bat->cw_bat.get_property = cw_battery_get_property;
	ret = power_supply_register(&client->dev, &cw_bat->cw_bat);
	if(ret < 0) {
	    power_supply_unregister(&cw_bat->cw_bat);
	    return ret;
	}
#else
	psy_desc = devm_kzalloc(&client->dev, sizeof(*psy_desc), GFP_KERNEL);
	if (!psy_desc)
		return -ENOMEM;
	
	psy_cfg.drv_data = cw_bat;
	psy_desc->name = CW_PROPERTIES;
	psy_desc->type = POWER_SUPPLY_TYPE_BATTERY;
	psy_desc->properties = cw_battery_properties;
	psy_desc->num_properties = ARRAY_SIZE(cw_battery_properties);
	psy_desc->get_property = cw_battery_get_property;
	cw_bat->cw_bat = power_supply_register(&client->dev, psy_desc, &psy_cfg);
	if(IS_ERR(cw_bat->cw_bat)) {
		ret = PTR_ERR(cw_bat->cw_bat);
	    printk(KERN_ERR"failed to register battery: %d\n", ret);
	    return ret;
	}
#endif
#endif

	cw_bat->cwfg_workqueue = create_singlethread_workqueue("cwfg_gauge");
	INIT_DELAYED_WORK(&cw_bat->battery_delay_work, cw_bat_work);
	queue_delayed_work(cw_bat->cwfg_workqueue, &cw_bat->battery_delay_work , msecs_to_jiffies(50));

#ifdef CW2017_INTERRUPT
	INIT_DELAYED_WORK(&cw_bat->interrupt_work, interrupt_work_do_wakeup);
	wake_lock_init(&cw2017_wakelock, WAKE_LOCK_SUSPEND, "cw2017_detect");
	if (client->irq > 0) {
			irq = client->irq;
			ret = request_irq(irq, ops_cw2017_int_handler_int_handler, IRQF_TRIGGER_FALLING, "cw2017_detect", cw_bat);
			if (ret < 0) {
					printk(KERN_ERR"fault interrupt registration failed err = %d\n", ret);
			}
			enable_irq_wake(irq);
	}
#endif
	#if defined(CONFIG_PRIZE_HARDWARE_INFO)
		strcpy(current_coulo_info.chip,"cw2017");
		sprintf(current_coulo_info.id,"0x%04x",cw_bat->version);
		strcpy(current_coulo_info.vendor,"weike");
		strcpy(current_coulo_info.more,"coulombmeter");
	#endif
	cw2017_exit_flag = 1;
	cw_printk("cw2017 driver probe success!\n");
    return 0;
}

static int cw2017_remove(struct i2c_client *client)	 
{
	cw_printk("\n");
	return 0;
}

#ifdef CONFIG_PM
static int cw_bat_suspend(struct device *dev)
{
        struct i2c_client *client = to_i2c_client(dev);
        struct cw_battery *cw_bat = i2c_get_clientdata(client);
        cancel_delayed_work(&cw_bat->battery_delay_work);
        return 0;
}

static int cw_bat_resume(struct device *dev)
{	
        struct i2c_client *client = to_i2c_client(dev);
        struct cw_battery *cw_bat = i2c_get_clientdata(client);
        queue_delayed_work(cw_bat->cwfg_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(2));
        return 0;
}

static const struct dev_pm_ops cw_bat_pm_ops = {
        .suspend  = cw_bat_suspend,
        .resume   = cw_bat_resume,
};
#endif

static const struct i2c_device_id cw2017_id_table[] = {
	{CWFG_NAME, 0},
	{}
};

static struct of_device_id cw2017_match_table[] = {
	{ .compatible = "cellwise,cw2017", },
	{ },
};

static struct i2c_driver cw2017_driver = {
	.driver 	  = {
		.name = CWFG_NAME,
#ifdef CONFIG_PM
        .pm     = &cw_bat_pm_ops,
#endif
		.owner	= THIS_MODULE,
		.of_match_table = cw2017_match_table,
	},
	.probe		  = cw2017_probe,
	.remove 	  = cw2017_remove,
	.id_table = cw2017_id_table,
};

/*
static struct i2c_board_info __initdata fgadc_dev = { 
	I2C_BOARD_INFO(CWFG_NAME, 0x63) 
};
*/

static int __init cw2017_init(void)
{
	//struct i2c_client *client;
	//struct i2c_adapter *i2c_adp;
	cw_printk("\n");

    //i2c_register_board_info(CWFG_I2C_BUSNUM, &fgadc_dev, 1);
	//i2c_adp = i2c_get_adapter(CWFG_I2C_BUSNUM);
	//client = i2c_new_device(i2c_adp, &fgadc_dev);
	
    i2c_add_driver(&cw2017_driver);
    return 0; 
}

/*
	//Add to dsti file
	cw2017@63 { 
		compatible = "cellwise,cw2017";
		reg = <0x63>;
	} 
*/

static void __exit cw2017_exit(void)
{
    i2c_del_driver(&cw2017_driver);
}

module_init(cw2017_init);
module_exit(cw2017_exit);

MODULE_AUTHOR("Cellwise FAE");
MODULE_DESCRIPTION("CW2017 FGADC Device Driver V1.2");
MODULE_LICENSE("GPL");

/***代码仅供参考***/
