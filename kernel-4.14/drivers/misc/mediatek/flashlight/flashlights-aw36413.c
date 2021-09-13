/*
* Copyright (C) 2019 MediaTek Inc.
*
* Version: v1.0.0
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/workqueue.h>
#include <linux/list.h>
#include "flashlight.h"
#include "flashlight-dt.h"
#include "flashlight-core.h"


/* device tree should be defined in flashlight-dt.h */
#ifndef AW36413_DTNAME
#define AW36413_DTNAME "mediatek,flashlights_aw36413"
#endif
#ifndef AW36413_DTNAME_I2C
#define AW36413_DTNAME_I2C "awinic,aw36413"
#endif
#define AW36413_NAME "flashlights-aw36413"

#define AW36413_VERSION "v1.0.0"


/* define registers */
#define AW36413_REG_ENABLE           (0x01)
#define AW36413_MASK_ENABLE_LED1     (0x01)
#define AW36413_MASK_ENABLE_LED2     (0x02)
#define AW36413_DISABLE              (0x00)
#define AW36413_ENABLE_LED1          (0x01)
#define AW36413_ENABLE_LED1_TORCH    (0x09)
#define AW36413_ENABLE_LED1_FLASH    (0x0D)
#define AW36413_ENABLE_LED2          (0x02)
#define AW36413_ENABLE_LED2_TORCH    (0x0A)
#define AW36413_ENABLE_LED2_FLASH    (0x0E)

#define AW36413_REG_TORCH_LEVEL_LED1 (0x05)
#define AW36413_REG_FLASH_LEVEL_LED1 (0x03)
#define AW36413_REG_TORCH_LEVEL_LED2 (0x06)
#define AW36413_REG_FLASH_LEVEL_LED2 (0x04)

#define AW36413_REG_TIMING_CONF      (0x08)
#define AW36413_TORCH_RAMP_TIME      (0x10)
#define AW36413_FLASH_TIMEOUT        (0x0F)

/* define channel, level */
#define AW36413_CHANNEL_NUM          2
#define AW36413_CHANNEL_CH1          0
#define AW36413_CHANNEL_CH2          1
#define AW36413_LEVEL_NUM            26
#define AW36413_LEVEL_TORCH          26 //always in torch mode

static int hwen_pin = -1;

static int AW36413_DEBUG_ENABLE = 1;
#define AW36413_DEBUG(format, args...) do { \
	if (AW36413_DEBUG_ENABLE) \
	{\
		printk(KERN_WARNING format, ##args);\
	} \
} while (0)

/* define mutex and work queue */
static DEFINE_MUTEX(aw36413_mutex);
static struct work_struct aw36413_work_ch1;
static struct work_struct aw36413_work_ch2;

struct i2c_client *aw36413_flashlight_client;

/* define usage count */
static int use_count;

/* define i2c */
static struct i2c_client *aw36413_i2c_client;

/* platform data
* torch_pin_enable: TX1/TORCH pin isa hardware TORCH enable
* pam_sync_pin_enable: TX2 Mode The ENVM/TX2 is a PAM Sync. on input
* thermal_comp_mode_enable: LEDI/NTC pin in Thermal Comparator Mode
* strobe_pin_disable: STROBE Input disabled
* vout_mode_enable: Voltage Out Mode enable
*/
struct aw36413_platform_data {
	u8 torch_pin_enable;
	u8 pam_sync_pin_enable;
	u8 thermal_comp_mode_enable;
	u8 strobe_pin_disable;
	u8 vout_mode_enable;
	
	int channel_num;
	struct flashlight_device_id *dev_id;
};

/* aw36413 chip data */
struct aw36413_chip_data {
	struct i2c_client *client;
	struct aw36413_platform_data *pdata;
	struct mutex lock;
	u8 last_flag;
	u8 no_pdata;
};

static struct platform_device *g_aw36413_pdev;
static int led_num = -1;
/******************************************************************************
 * aw36413 operations
 *****************************************************************************/
/******************************************************************************
 * [0]  0x05, 5*2.91+2.55=15
 * [1]  0x0A, 10*2.91+2.55=30
 * [2]  0x0F, 15*2.91+2.55=45
 * [3]  0x14, 20*2.91+2.55=60
 * [4]  0x19, 25*2.91+2.55=75
 * [5]  0x1F, 31*2.91+2.55=90
 * [6]  0x24, 36*2.91+2.55=105
 * [7]  0x2B, 43*2.91+2.55=125
 * [8]  0x33, 51*2.91+2.55=150
 * [9]  0x3C, 60*2.91+2.55=175
 * [10] 0x44, 68*2.91+2.55=200
 * [11] 0x4D, 77*2.91+2.55=225
 * [12] 0x56, 86*2.91+2.55=250
 * [13] 0x5E, 94*2.91+2.55=275
 * [14] 0x67, 103*2.91+2.55=300
 * [15] 0x6F, 111*2.91+2.55=325
 * [16] 0x78, 120*2.91+2.55=350
 *****************************************************************************/
static const unsigned char aw36413_torch_level[AW36413_LEVEL_NUM] = {
	0x05, 0x0A, 0x0F, 0x14, 0x19, 0x1F, 0x24, 0x2B, 0x33, 0x3C,
	0x44, 0x4D, 0x56, 0x5E, 0x67, 0x6F, 0x78, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static const unsigned char aw36413_flash_level[AW36413_LEVEL_NUM] = {
	0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x10, 0x14, 0x19,
	0x1D, 0x21, 0x25, 0x2A, 0x2E, 0x32, 0x37, 0x3B, 0x3F, 0x43,
	0x48, 0x4C, 0x50, 0x54, 0x59, 0x5D};

static volatile unsigned char aw36413_reg_enable;
static volatile int aw36413_level_ch1 = -1;
static volatile int aw36413_level_ch2 = -1;

static int aw36413_is_torch(int level)
{

	if (level >= AW36413_LEVEL_TORCH)
		return -1;

	return 0;
}

static int aw36413_verify_level(int level)
{

	if (level < 0)
		level = 0;
	else if (level >= AW36413_LEVEL_NUM)
		level = AW36413_LEVEL_NUM - 1;

	return level;
}

/* i2c wrapper function */
static int aw36413_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	struct aw36413_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		pr_err("failed writing at 0x%02x\n", reg);

	return ret;
}

static int aw36413_read_reg(struct i2c_client *client, u8 reg)
{
	int val;
	struct aw36413_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	val = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);

	if (val < 0)
		pr_err("failed read at 0x%02x\n", reg);

	return val;
}

/* flashlight enable function */
static int aw36413_enable_ch1(void)
{
	unsigned char reg, val;

	reg = AW36413_REG_ENABLE;
	if (!aw36413_is_torch(aw36413_level_ch1)) {
		/* torch mode */
		aw36413_reg_enable |= AW36413_ENABLE_LED1_TORCH;
	} else {
		/* flash mode */
		aw36413_reg_enable |= AW36413_ENABLE_LED1_FLASH;
	}

	val = aw36413_reg_enable;

	return aw36413_write_reg(aw36413_i2c_client, reg, val);
}

static int aw36413_enable_ch2(void)
{
	unsigned char reg, val;

	reg = AW36413_REG_ENABLE;
	if (!aw36413_is_torch(aw36413_level_ch2)) {
		/* torch mode */
		aw36413_reg_enable |= AW36413_ENABLE_LED2_TORCH;
	} else {
		/* flash mode */
		aw36413_reg_enable |= AW36413_ENABLE_LED2_FLASH;
	}
	val = aw36413_reg_enable;

	return aw36413_write_reg(aw36413_i2c_client, reg, val);
}

static int aw36413_enable(int channel)
{
	gpio_direction_output(hwen_pin, 1);
    gpio_set_value(hwen_pin, 1);
	
	if (led_num == 1) {
		aw36413_enable_ch1();
	} else if (led_num == 2) {
		aw36413_enable_ch2();
	} else if (led_num == 3) {
		aw36413_enable_ch1();
		aw36413_enable_ch2();
	}

	return 0;
}

/* flashlight disable function */
static int aw36413_disable_ch1(void)
{
	unsigned char reg, val;

	reg = AW36413_REG_ENABLE;
	if (aw36413_reg_enable & AW36413_MASK_ENABLE_LED2) {
		/* if LED 2 is enable, disable LED 1 */
		aw36413_reg_enable &= (~AW36413_ENABLE_LED1);
	} else {
		/* if LED 2 is enable, disable LED 1 and clear mode */
		aw36413_reg_enable &= (~AW36413_ENABLE_LED1_FLASH);
	}
	val = aw36413_reg_enable;

	return aw36413_write_reg(aw36413_i2c_client, reg, val);
}

static int aw36413_disable_ch2(void)
{
	unsigned char reg, val;

	reg = AW36413_REG_ENABLE;
	if (aw36413_reg_enable & AW36413_MASK_ENABLE_LED1) {
		/* if LED 1 is enable, disable LED 2 */
		aw36413_reg_enable &= (~AW36413_ENABLE_LED2);
	} else {
		/* if LED 1 is enable, disable LED 2 and clear mode */
		aw36413_reg_enable &= (~AW36413_ENABLE_LED2_FLASH);
	}
	val = aw36413_reg_enable;

	return aw36413_write_reg(aw36413_i2c_client, reg, val);
}

static int aw36413_disable(int channel)
{
	gpio_direction_output(hwen_pin, 0);
	gpio_set_value(hwen_pin, 0);
	
	if (led_num == 1) {
		aw36413_disable_ch1();
	} else if (led_num == 2) {
		aw36413_disable_ch2();
	} else if (led_num == 3) {
		aw36413_disable_ch1();
		aw36413_disable_ch2();
	}

	return 0;
}

/* set flashlight level */
static int aw36413_set_level_ch1(int level)
{
	int ret;
	unsigned char reg, val;

	level = aw36413_verify_level(level);

	/* set torch brightness level */
	reg = AW36413_REG_TORCH_LEVEL_LED1;
	val = aw36413_torch_level[level];
	ret = aw36413_write_reg(aw36413_i2c_client, reg, val);

	aw36413_level_ch1 = level;

	/* set flash brightness level */
	reg = AW36413_REG_FLASH_LEVEL_LED1;
	val = aw36413_flash_level[level];
	ret = aw36413_write_reg(aw36413_i2c_client, reg, val);

	return ret;
}

int aw36413_set_level_ch2(int level)
{
	int ret;
	unsigned char reg, val;

	level = aw36413_verify_level(level);

	/* set torch brightness level */
	reg = AW36413_REG_TORCH_LEVEL_LED2;
	val = aw36413_torch_level[level];
	ret = aw36413_write_reg(aw36413_i2c_client, reg, val);

	aw36413_level_ch2 = level;

	/* set flash brightness level */
	reg = AW36413_REG_FLASH_LEVEL_LED2;
	val = aw36413_flash_level[level];
	ret = aw36413_write_reg(aw36413_i2c_client, reg, val);

	return ret;
}

static int aw36413_set_level(int channel, int level)
{
	if (led_num == 1) {
		aw36413_set_level_ch1(level);
	} else if (led_num == 2) {
		aw36413_set_level_ch2(level);
	} else if (led_num == 3) {
		aw36413_set_level_ch1(level);
		aw36413_set_level_ch2(level);
	}

	return 0;
}

/* flashlight init */
int aw36413_init(void)
{
	int ret;
	unsigned char reg, val;

	usleep_range(2000, 2500);

	/* clear enable register */
	reg = AW36413_REG_ENABLE;
	val = AW36413_DISABLE;
	ret = aw36413_write_reg(aw36413_i2c_client, reg, val);

	aw36413_reg_enable = val;

	/* set torch current ramp time and flash timeout */
	reg = AW36413_REG_TIMING_CONF;
	val = AW36413_TORCH_RAMP_TIME | AW36413_FLASH_TIMEOUT;
	ret = aw36413_write_reg(aw36413_i2c_client, reg, val);

	return ret;
}

/* flashlight uninit */
int aw36413_uninit(void)
{
	aw36413_disable(AW36413_CHANNEL_CH1);
	aw36413_disable(AW36413_CHANNEL_CH2);

	return 0;
}


/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer aw36413_timer_ch1;
static struct hrtimer aw36413_timer_ch2;
static unsigned int aw36413_timeout_ms[AW36413_CHANNEL_NUM];

static void aw36413_work_disable_ch1(struct work_struct *data)
{
	AW36413_DEBUG("ht work queue callback\n");
	aw36413_disable_ch1();
}

static void aw36413_work_disable_ch2(struct work_struct *data)
{
	printk("lt work queue callback\n");
	aw36413_disable_ch2();
}

static enum hrtimer_restart aw36413_timer_func_ch1(struct hrtimer *timer)
{
	schedule_work(&aw36413_work_ch1);
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart aw36413_timer_func_ch2(struct hrtimer *timer)
{
	schedule_work(&aw36413_work_ch2);
	return HRTIMER_NORESTART;
}

int aw36413_timer_start(int channel, ktime_t ktime)
{
	if (channel == AW36413_CHANNEL_CH1)
		hrtimer_start(&aw36413_timer_ch1, ktime, HRTIMER_MODE_REL);
	else if (channel == AW36413_CHANNEL_CH2)
		hrtimer_start(&aw36413_timer_ch2, ktime, HRTIMER_MODE_REL);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

int aw36413_timer_cancel(int channel)
{
	if (channel == AW36413_CHANNEL_CH1)
		hrtimer_cancel(&aw36413_timer_ch1);
	else if (channel == AW36413_CHANNEL_CH2)
		hrtimer_cancel(&aw36413_timer_ch2);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int aw36413_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	ktime_t ktime;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	/* verify channel */
	if (channel < 0 || channel >= AW36413_CHANNEL_NUM) {
		pr_err("Failed with error channel\n");
		return -EINVAL;
	}

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		AW36413_DEBUG("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		aw36413_timeout_ms[channel] = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		AW36413_DEBUG("FLASH_IOC_SET_DUTY(%d): %d\n",
				led_num, (int)fl_arg->arg); //channel -> led_num
		aw36413_set_level(led_num, fl_arg->arg); //channel -> led_num
		break;

	case FLASH_IOC_SET_ONOFF:
		AW36413_DEBUG("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (aw36413_timeout_ms[channel]) {
				ktime =
				ktime_set(aw36413_timeout_ms[channel] / 1000,
				(aw36413_timeout_ms[channel] % 1000) * 1000000);
				aw36413_timer_start(channel, ktime);
			}
			aw36413_enable(led_num); //channel -> led_num
		} else {
			aw36413_disable(led_num); //channel -> led_num
			aw36413_timer_cancel(channel);
		}
		break;

	default:
		AW36413_DEBUG("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int aw36413_open(void)
{
	/* Actual behavior move to set driver function */
	/* since power saving issue */
	return 0;
}

static int aw36413_release(void)
{
	/* uninit chip and clear usage count */
	mutex_lock(&aw36413_mutex);
	use_count--;
	if (!use_count)
		aw36413_uninit();
	if (use_count < 0)
		use_count = 0;
	mutex_unlock(&aw36413_mutex);

	AW36413_DEBUG("Release: %d\n", use_count);

	return 0;
}

static int aw36413_set_driver(int set)
{
	/* init chip and set usage count */
	mutex_lock(&aw36413_mutex);
	if (!use_count)
		aw36413_init();
	use_count++;
	mutex_unlock(&aw36413_mutex);

	AW36413_DEBUG("Set driver: %d\n", use_count);

	return 0;
}

static ssize_t aw36413_strobe_store(struct flashlight_arg arg)
{
	aw36413_set_driver(1);
	aw36413_set_level(arg.ct, arg.level);
	aw36413_enable(arg.ct);
	msleep(arg.dur);
	aw36413_disable(arg.ct);
	aw36413_set_driver(0);

	return 0;
}

static struct flashlight_operations aw36413_ops = {
	aw36413_open,
	aw36413_release,
	aw36413_ioctl,
	aw36413_strobe_store,
	aw36413_set_driver
};


/******************************************************************************
 * I2C device and driver
 *****************************************************************************/
static int aw36413_chip_init(struct aw36413_chip_data *chip)
{
	/* NOTE: Chip initialication move to
	*"set driver" operation for power saving issue.
	* aw36413_init();
	*/

	return 0;
}

/***************************************************************************/
/*AW36413 Debug file */
/***************************************************************************/
static ssize_t
aw36413_get_reg(struct device *cd, struct device_attribute *attr, char *buf)
{
	unsigned char reg_val;
	unsigned char i;
	ssize_t len = 0;

	for (i = 0; i < 0x0E; i++) {
		reg_val = aw36413_read_reg(aw36413_i2c_client, i);
		len += snprintf(buf+len, PAGE_SIZE-len,
			"reg0x%2X = 0x%2X\n", i, reg_val);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\r\n");
	return len;
}

static ssize_t aw36413_set_reg(struct device *cd,
		struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int databuf[2];

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2)
		aw36413_write_reg(aw36413_i2c_client, databuf[0], databuf[1]);
	return len;
}


static ssize_t aw36413_store(struct device *cd,
		struct device_attribute *attr, const char *buf, size_t len)
{
	if(buf[0] == '0')
	{
		aw36413_enable(AW36413_CHANNEL_CH1);
	}
	else if(buf[0] == '1')
	{
		aw36413_disable(AW36413_CHANNEL_CH1);
	}
	else if(buf[0] == '2')
	{
		aw36413_enable(AW36413_CHANNEL_CH2);
	}
	else if(buf[0] == '3')
	{
		aw36413_disable(AW36413_CHANNEL_CH2);
	}

	return len;
}


static DEVICE_ATTR(reg, 0660, aw36413_get_reg, aw36413_set_reg);
static DEVICE_ATTR(set, 0660, NULL, aw36413_store);

static int aw36413_create_sysfs(struct i2c_client *client)
{
	int err;
	struct device *dev = &(client->dev);

	err = device_create_file(dev, &dev_attr_reg);
	
	err = device_create_file(dev, &dev_attr_set);
	
	return err;
}

static int aw36413_parse_dt(struct device *dev,
		struct aw36413_platform_data *pdata)
{
	struct device_node *np, *cnp;
	u32 decouple = 0;
	int i = 0;

	if (!dev || !dev->of_node || !pdata)
		return -ENODEV;

	np = dev->of_node;

	pdata->channel_num = of_get_child_count(np);
	if (!pdata->channel_num) {
		AW36413_DEBUG("Parse no dt, node.\n");
		return 0;
	}
	AW36413_DEBUG("Channel number(%d).\n", pdata->channel_num);

	if (of_property_read_u32(np, "decouple", &decouple))
		AW36413_DEBUG("Parse no dt, decouple.\n");
	
	if (of_property_read_u32(np, "led_num", &led_num))
		AW36413_DEBUG("Parse no dt, led_num.\n");

	pdata->dev_id = devm_kzalloc(dev,
			pdata->channel_num *
			sizeof(struct flashlight_device_id),
			GFP_KERNEL);
	if (!pdata->dev_id)
		return -ENOMEM;

	for_each_child_of_node(np, cnp) {
		if (of_property_read_u32(cnp, "type", &pdata->dev_id[i].type))
			goto err_node_put;
		if (of_property_read_u32(cnp, "ct", &pdata->dev_id[i].ct))
			goto err_node_put;
		if (of_property_read_u32(cnp, "part", &pdata->dev_id[i].part))
			goto err_node_put;
		snprintf(pdata->dev_id[i].name, FLASHLIGHT_NAME_SIZE,
				AW36413_NAME);
		pdata->dev_id[i].channel = i;
		pdata->dev_id[i].decouple = decouple;

		AW36413_DEBUG("Parse dt (type,ct,part,name,channel,decouple)=(%d,%d,%d,%s,%d,%d).\n",
				pdata->dev_id[i].type, pdata->dev_id[i].ct,
				pdata->dev_id[i].part, pdata->dev_id[i].name,
				pdata->dev_id[i].channel,
				pdata->dev_id[i].decouple);
		i++;
	}

	return 0;

err_node_put:
	of_node_put(cnp);
	return -EINVAL;
}

static int
aw36413_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct aw36413_chip_data *chip;
	struct aw36413_platform_data *pdata = client->dev.platform_data;
	int err;
	int i;
	/************************************/
	struct device_node *node = NULL;
    
    node = of_find_compatible_node(NULL, NULL, "mediatek,flashlights_aw36413");
	if (node == NULL) {
		AW36413_DEBUG("aw36413 node is not exist\n");
		return err;
	}	

    hwen_pin = of_get_named_gpio(node, "hwen-gpio", 0);
	if (hwen_pin)
		AW36413_DEBUG("aw36413 hwen_pin is %d\n", hwen_pin);
	
    gpio_request(hwen_pin, "hwen_pin");
	/************************************/

	AW36413_DEBUG("%s Probe start.\n", __func__);

	/* check i2c */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("Failed to check i2c functionality.\n");
		err = -ENODEV;
		goto err_out;
	}

	/* init chip private data */
	chip = kzalloc(sizeof(struct aw36413_chip_data), GFP_KERNEL);
	if (!chip) {
		err = -ENOMEM;
		goto err_out;
	}
	chip->client = client;

	/* init platform data */
	if (!pdata) {
		pr_err("Platform data does not exist\n");
		pdata =
		kzalloc(sizeof(struct aw36413_platform_data), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err_init_pdata;
		}
		chip->no_pdata = 1;
	}
	chip->pdata = pdata;

	err = aw36413_parse_dt(&g_aw36413_pdev->dev, chip->pdata);
	if (err)
		goto err_init_pdata;

	i2c_set_clientdata(client, chip);
	aw36413_i2c_client = client;

	/* init mutex and spinlock */
	mutex_init(&chip->lock);

	/* init work queue */
	INIT_WORK(&aw36413_work_ch1, aw36413_work_disable_ch1);
	INIT_WORK(&aw36413_work_ch2, aw36413_work_disable_ch2);

	/* init timer */
	hrtimer_init(&aw36413_timer_ch1, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw36413_timer_ch1.function = aw36413_timer_func_ch1;
	hrtimer_init(&aw36413_timer_ch2, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw36413_timer_ch2.function = aw36413_timer_func_ch2;
	aw36413_timeout_ms[AW36413_CHANNEL_CH1] = 100;
	aw36413_timeout_ms[AW36413_CHANNEL_CH2] = 100;

	/* init chip hw */
	aw36413_chip_init(chip);

	/* register flashlight device */
	if (pdata->channel_num) {
		for (i = 0; i < pdata->channel_num; i++) {
			if (flashlight_dev_register_by_device_id(&pdata->dev_id[i], &aw36413_ops)) {
				err = -EFAULT;
				goto err_free;
			}
		}
	} else {
		if (flashlight_dev_register(AW36413_NAME, &aw36413_ops)) {
			err = -EFAULT;
			goto err_free;
		}
	}

	/* clear usage count */
	use_count = 0;

	aw36413_create_sysfs(client);

	AW36413_DEBUG("%s Probe done.\n", __func__);

	return 0;

err_free:
	kfree(chip->pdata);
err_init_pdata:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
err_out:
	return err;
}

static int aw36413_i2c_remove(struct i2c_client *client)
{
	struct aw36413_chip_data *chip = i2c_get_clientdata(client);

	AW36413_DEBUG("Remove start.\n");

	/* flush work queue */
	flush_work(&aw36413_work_ch1);
	flush_work(&aw36413_work_ch2);

	/* unregister flashlight operations */
	flashlight_dev_unregister(AW36413_NAME);

	/* free resource */
	if (chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);

	AW36413_DEBUG("Remove done.\n");

	return 0;
}

static const struct i2c_device_id aw36413_i2c_id[] = {
	{AW36413_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id aw36413_i2c_of_match[] = {
	{.compatible = AW36413_DTNAME_I2C},
	{},
};
#endif

static struct i2c_driver aw36413_i2c_driver = {
	.driver = {
		   .name = AW36413_NAME,
#ifdef CONFIG_OF
		   .of_match_table = aw36413_i2c_of_match,
#endif
		   },
	.probe = aw36413_i2c_probe,
	.remove = aw36413_i2c_remove,
	.id_table = aw36413_i2c_id,
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int aw36413_probe(struct platform_device *pdev)
{
	AW36413_DEBUG("%s Probe start.\n", __func__);
	
	g_aw36413_pdev = pdev;

	if (i2c_add_driver(&aw36413_i2c_driver)) {
		pr_err("Failed to add i2c driver.\n");
		return -1;
	}

	AW36413_DEBUG("%s Probe done.\n", __func__);

	return 0;
}

static int aw36413_remove(struct platform_device *pdev)
{
	AW36413_DEBUG("Remove start.\n");

	i2c_del_driver(&aw36413_i2c_driver);

	AW36413_DEBUG("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id aw36413_of_match[] = {
	{.compatible = AW36413_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, aw36413_of_match);
#else
static struct platform_device aw36413_platform_device[] = {
	{
		.name = AW36413_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, aw36413_platform_device);
#endif

static struct platform_driver aw36413_platform_driver = {
	.probe = aw36413_probe,
	.remove = aw36413_remove,
	.driver = {
		.name = AW36413_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = aw36413_of_match,
#endif
	},
};

static int __init flashlight_aw36413_init(void)
{
	int ret = 0;

	AW36413_DEBUG("%s driver version %s.\n", __func__, AW36413_VERSION);
	
	pr_err("flashlight_aw36413_init----------start------\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&aw36413_platform_device);
	if (ret) {
		pr_err("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&aw36413_platform_driver);
	if (ret) {
		pr_err("Failed to register platform driver\n");
		return ret;
	}

	AW36413_DEBUG("flashlight_aw36413 Init done.\n");

	return 0;
}

static void __exit flashlight_aw36413_exit(void)
{
	AW36413_DEBUG("flashlight_aw36413-Exit start.\n");

	platform_driver_unregister(&aw36413_platform_driver);

	AW36413_DEBUG("flashlight_aw36413 Exit done.\n");
}

module_init(flashlight_aw36413_init);
module_exit(flashlight_aw36413_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Joseph <zhangzetao@awinic.com.cn>");
MODULE_DESCRIPTION("AW Flashlight AW36413 Driver");

