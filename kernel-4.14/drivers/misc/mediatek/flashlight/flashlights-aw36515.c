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
#include <linux/workqueue.h>
#include <linux/list.h>
#include "flashlight.h"
#include "flashlight-dt.h"
#include "flashlight-core.h"


/* device tree should be defined in flashlight-dt.h */
#ifndef AW36515_DTNAME
#define AW36515_DTNAME "mediatek,flashlights_aw36515"
#endif
#ifndef AW36515_DTNAME_I2C
#define AW36515_DTNAME_I2C "awinic,aw36515"
#endif
#define AW36515_NAME "flashlights-aw36515"

#define AW36515_VERSION "v1.0.0"


/* define registers */
#define AW36515_REG_ENABLE           (0x01)
#define AW36515_MASK_ENABLE_LED1     (0x01)
#define AW36515_MASK_ENABLE_LED2     (0x02)
#define AW36515_DISABLE              (0x00)
#define AW36515_ENABLE_LED1          (0x01)
#define AW36515_ENABLE_LED1_TORCH    (0x09)
#define AW36515_ENABLE_LED1_FLASH    (0x0D)
#define AW36515_ENABLE_LED2          (0x02)
#define AW36515_ENABLE_LED2_TORCH    (0x0A)
#define AW36515_ENABLE_LED2_FLASH    (0x0E)

#define AW36515_REG_TORCH_LEVEL_LED1 (0x05)
#define AW36515_REG_FLASH_LEVEL_LED1 (0x03)
#define AW36515_REG_TORCH_LEVEL_LED2 (0x06)
#define AW36515_REG_FLASH_LEVEL_LED2 (0x04)

#define AW36515_REG_TIMING_CONF      (0x08)
#define AW36515_TORCH_RAMP_TIME      (0x10)
#define AW36515_FLASH_TIMEOUT        (0x0F)

/* define channel, level */
#define AW36515_CHANNEL_NUM          2
#define AW36515_CHANNEL_CH1          0
#define AW36515_CHANNEL_CH2          1
#define AW36515_LEVEL_NUM            26
#define AW36515_LEVEL_TORCH          26 //always in torch mode



static int AW36515_DEBUG_ENABLE = 1;
#define AW36515_DEBUG(format, args...) do { \
	if (AW36515_DEBUG_ENABLE) \
	{\
		printk(KERN_WARNING format, ##args);\
	} \
} while (0)

/* define mutex and work queue */
static DEFINE_MUTEX(aw36515_mutex);
static struct work_struct aw36515_work_ch1;
static struct work_struct aw36515_work_ch2;

struct i2c_client *aw36515_flashlight_client;

/* define usage count */
static int use_count;

/* define i2c */
static struct i2c_client *aw36515_i2c_client;

/* platform data
* torch_pin_enable: TX1/TORCH pin isa hardware TORCH enable
* pam_sync_pin_enable: TX2 Mode The ENVM/TX2 is a PAM Sync. on input
* thermal_comp_mode_enable: LEDI/NTC pin in Thermal Comparator Mode
* strobe_pin_disable: STROBE Input disabled
* vout_mode_enable: Voltage Out Mode enable
*/
struct aw36515_platform_data {
	u8 torch_pin_enable;
	u8 pam_sync_pin_enable;
	u8 thermal_comp_mode_enable;
	u8 strobe_pin_disable;
	u8 vout_mode_enable;
	
	int channel_num;
	struct flashlight_device_id *dev_id;
};

/* aw36515 chip data */
struct aw36515_chip_data {
	struct i2c_client *client;
	struct aw36515_platform_data *pdata;
	struct mutex lock;
	u8 last_flag;
	u8 no_pdata;
};

static struct platform_device *g_aw36515_pdev;
static int led_num = -1;
/******************************************************************************
 * aw36515 operations
 *****************************************************************************/
/******************************************************************************
 * [0]  0x06, 6*1.96+0.98=12.74
 * [1]  0x0F, 15*1.96+0.98=30.38
 * [2]  0x17, 23*1.96+0.98=46.06
 * [3]  0x1F, 31*1.96+0.98=61.74
 * [4]  0x27, 39*1.96+0.98=77.42
 * [5]  0x2F, 47*1.96+0.98=93.1
 * [6]  0x37, 55*1.96+0.98=108.78
 * [7]  0x40, 64*1.96+0.98=126.42
 * [8]  0x4D, 77*1.96+0.98=151.9
 * [9]  0x59, 89*1.96+0.98=175.42
 * [10] 0x66, 102*1.96+0.98=200.9
 * [11] 0x73, 115*1.96+0.98=226.38
 * [12] 0x7F, 127*1.96+0.98=249.9
 * [13] 0x8C, 140*1.96+0.98=275.38
 * [14] 0x99, 153*1.96+0.98=300.86
 * [15] 0xA6, 166*1.96+0.98=326.34
 * [16] 0xB3, 179*1.96+0.98=351.82
 *****************************************************************************/
static const unsigned char aw36515_torch_level[AW36515_LEVEL_NUM] = {
	0x06, 0x0F, 0x17, 0x1F, 0x27, 0x2F, 0x37, 0x40, 0x4D, 0x59,
	0x66, 0x73, 0x7F, 0x8C, 0x99, 0xA6, 0xB3, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static const unsigned char aw36515_flash_level[AW36515_LEVEL_NUM] = {
	0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x10, 0x14, 0x19,
	0x1D, 0x21, 0x25, 0x2A, 0x2E, 0x32, 0x37, 0x3B, 0x3F, 0x43,
	0x48, 0x4C, 0x50, 0x54, 0x59, 0x5D};

static volatile unsigned char aw36515_reg_enable;
static volatile int aw36515_level_ch1 = -1;
static volatile int aw36515_level_ch2 = -1;

static int aw36515_is_torch(int level)
{

	if (level >= AW36515_LEVEL_TORCH)
		return -1;

	return 0;
}

static int aw36515_verify_level(int level)
{

	if (level < 0)
		level = 0;
	else if (level >= AW36515_LEVEL_NUM)
		level = AW36515_LEVEL_NUM - 1;

	return level;
}

/* i2c wrapper function */
static int aw36515_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	struct aw36515_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		pr_err("failed writing at 0x%02x\n", reg);

	return ret;
}

static int aw36515_read_reg(struct i2c_client *client, u8 reg)
{
	int val;
	struct aw36515_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	val = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);

	if (val < 0)
		pr_err("failed read at 0x%02x\n", reg);

	return val;
}

/* flashlight enable function */
static int aw36515_enable_ch1(void)
{
	unsigned char reg, val;

	reg = AW36515_REG_ENABLE;
	if (!aw36515_is_torch(aw36515_level_ch1)) {
		/* torch mode */
		aw36515_reg_enable |= AW36515_ENABLE_LED1_TORCH;
	} else {
		/* flash mode */
		aw36515_reg_enable |= AW36515_ENABLE_LED1_FLASH;
	}

	val = aw36515_reg_enable;

	return aw36515_write_reg(aw36515_i2c_client, reg, val);
}

static int aw36515_enable_ch2(void)
{
	unsigned char reg, val;

	reg = AW36515_REG_ENABLE;
	if (!aw36515_is_torch(aw36515_level_ch2)) {
		/* torch mode */
		aw36515_reg_enable |= AW36515_ENABLE_LED2_TORCH;
	} else {
		/* flash mode */
		aw36515_reg_enable |= AW36515_ENABLE_LED2_FLASH;
	}
	val = aw36515_reg_enable;

	return aw36515_write_reg(aw36515_i2c_client, reg, val);
}

static int aw36515_enable(int channel)
{
	if (led_num == 1) {
		aw36515_enable_ch1();
	} else if (led_num == 2) {
		aw36515_enable_ch2();
	} else if (led_num == 3) {
		aw36515_enable_ch1();
		aw36515_enable_ch2();
	}

	return 0;
}

/* flashlight disable function */
static int aw36515_disable_ch1(void)
{
	unsigned char reg, val;

	reg = AW36515_REG_ENABLE;
	if (aw36515_reg_enable & AW36515_MASK_ENABLE_LED2) {
		/* if LED 2 is enable, disable LED 1 */
		aw36515_reg_enable &= (~AW36515_ENABLE_LED1);
	} else {
		/* if LED 2 is enable, disable LED 1 and clear mode */
		aw36515_reg_enable &= (~AW36515_ENABLE_LED1_FLASH);
	}
	val = aw36515_reg_enable;

	return aw36515_write_reg(aw36515_i2c_client, reg, val);
}

static int aw36515_disable_ch2(void)
{
	unsigned char reg, val;

	reg = AW36515_REG_ENABLE;
	if (aw36515_reg_enable & AW36515_MASK_ENABLE_LED1) {
		/* if LED 1 is enable, disable LED 2 */
		aw36515_reg_enable &= (~AW36515_ENABLE_LED2);
	} else {
		/* if LED 1 is enable, disable LED 2 and clear mode */
		aw36515_reg_enable &= (~AW36515_ENABLE_LED2_FLASH);
	}
	val = aw36515_reg_enable;

	return aw36515_write_reg(aw36515_i2c_client, reg, val);
}

static int aw36515_disable(int channel)
{
	if (led_num == 1) {
		aw36515_disable_ch1();
	} else if (led_num == 2) {
		aw36515_disable_ch2();
	} else if (led_num == 3) {
		aw36515_disable_ch1();
		aw36515_disable_ch2();
	}

	return 0;
}

/* set flashlight level */
static int aw36515_set_level_ch1(int level)
{
	int ret;
	unsigned char reg, val;

	level = aw36515_verify_level(level);

	/* set torch brightness level */
	reg = AW36515_REG_TORCH_LEVEL_LED1;
	val = aw36515_torch_level[level];
	ret = aw36515_write_reg(aw36515_i2c_client, reg, val);

	aw36515_level_ch1 = level;

	/* set flash brightness level */
	reg = AW36515_REG_FLASH_LEVEL_LED1;
	val = aw36515_flash_level[level];
	ret = aw36515_write_reg(aw36515_i2c_client, reg, val);

	return ret;
}

int aw36515_set_level_ch2(int level)
{
	int ret;
	unsigned char reg, val;

	level = aw36515_verify_level(level);

	/* set torch brightness level */
	reg = AW36515_REG_TORCH_LEVEL_LED2;
	val = aw36515_torch_level[level];
	ret = aw36515_write_reg(aw36515_i2c_client, reg, val);

	aw36515_level_ch2 = level;

	/* set flash brightness level */
	reg = AW36515_REG_FLASH_LEVEL_LED2;
	val = aw36515_flash_level[level];
	ret = aw36515_write_reg(aw36515_i2c_client, reg, val);

	return ret;
}

static int aw36515_set_level(int channel, int level)
{
	if (led_num == 1) {
		aw36515_set_level_ch1(level);
	} else if (led_num == 2) {
		aw36515_set_level_ch2(level);
	} else if (led_num == 3) {
		aw36515_set_level_ch1(level);
		aw36515_set_level_ch2(level);
	}

	return 0;
}

/* flashlight init */
int aw36515_init(void)
{
	int ret;
	unsigned char reg, val;

	usleep_range(2000, 2500);

	/* clear enable register */
	reg = AW36515_REG_ENABLE;
	val = AW36515_DISABLE;
	ret = aw36515_write_reg(aw36515_i2c_client, reg, val);

	aw36515_reg_enable = val;

	/* set torch current ramp time and flash timeout */
	reg = AW36515_REG_TIMING_CONF;
	val = AW36515_TORCH_RAMP_TIME | AW36515_FLASH_TIMEOUT;
	ret = aw36515_write_reg(aw36515_i2c_client, reg, val);

	return ret;
}

/* flashlight uninit */
int aw36515_uninit(void)
{
	aw36515_disable(AW36515_CHANNEL_CH1);
	aw36515_disable(AW36515_CHANNEL_CH2);

	return 0;
}


/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer aw36515_timer_ch1;
static struct hrtimer aw36515_timer_ch2;
static unsigned int aw36515_timeout_ms[AW36515_CHANNEL_NUM];

static void aw36515_work_disable_ch1(struct work_struct *data)
{
	AW36515_DEBUG("ht work queue callback\n");
	aw36515_disable_ch1();
}

static void aw36515_work_disable_ch2(struct work_struct *data)
{
	printk("lt work queue callback\n");
	aw36515_disable_ch2();
}

static enum hrtimer_restart aw36515_timer_func_ch1(struct hrtimer *timer)
{
	schedule_work(&aw36515_work_ch1);
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart aw36515_timer_func_ch2(struct hrtimer *timer)
{
	schedule_work(&aw36515_work_ch2);
	return HRTIMER_NORESTART;
}

int aw36515_timer_start(int channel, ktime_t ktime)
{
	if (channel == AW36515_CHANNEL_CH1)
		hrtimer_start(&aw36515_timer_ch1, ktime, HRTIMER_MODE_REL);
	else if (channel == AW36515_CHANNEL_CH2)
		hrtimer_start(&aw36515_timer_ch2, ktime, HRTIMER_MODE_REL);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

int aw36515_timer_cancel(int channel)
{
	if (channel == AW36515_CHANNEL_CH1)
		hrtimer_cancel(&aw36515_timer_ch1);
	else if (channel == AW36515_CHANNEL_CH2)
		hrtimer_cancel(&aw36515_timer_ch2);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int aw36515_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	ktime_t ktime;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	/* verify channel */
	if (channel < 0 || channel >= AW36515_CHANNEL_NUM) {
		pr_err("Failed with error channel\n");
		return -EINVAL;
	}

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		AW36515_DEBUG("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		aw36515_timeout_ms[channel] = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		AW36515_DEBUG("FLASH_IOC_SET_DUTY(%d): %d\n",
				led_num, (int)fl_arg->arg); //channel -> led_num
		aw36515_set_level(led_num, fl_arg->arg); //channel -> led_num
		break;

	case FLASH_IOC_SET_ONOFF:
		AW36515_DEBUG("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (aw36515_timeout_ms[channel]) {
				ktime =
				ktime_set(aw36515_timeout_ms[channel] / 1000,
				(aw36515_timeout_ms[channel] % 1000) * 1000000);
				aw36515_timer_start(channel, ktime);
			}
			aw36515_enable(led_num); //channel -> led_num
		} else {
			aw36515_disable(led_num); //channel -> led_num
			aw36515_timer_cancel(channel);
		}
		break;

	default:
		AW36515_DEBUG("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int aw36515_open(void)
{
	/* Actual behavior move to set driver function */
	/* since power saving issue */
	return 0;
}

static int aw36515_release(void)
{
	/* uninit chip and clear usage count */
	mutex_lock(&aw36515_mutex);
	use_count--;
	if (!use_count)
		aw36515_uninit();
	if (use_count < 0)
		use_count = 0;
	mutex_unlock(&aw36515_mutex);

	AW36515_DEBUG("Release: %d\n", use_count);

	return 0;
}

static int aw36515_set_driver(int set)
{
	/* init chip and set usage count */
	mutex_lock(&aw36515_mutex);
	if (!use_count)
		aw36515_init();
	use_count++;
	mutex_unlock(&aw36515_mutex);

	AW36515_DEBUG("Set driver: %d\n", use_count);

	return 0;
}

static ssize_t aw36515_strobe_store(struct flashlight_arg arg)
{
	aw36515_set_driver(1);
	aw36515_set_level(arg.ct, arg.level);
	aw36515_enable(arg.ct);
	msleep(arg.dur);
	aw36515_disable(arg.ct);
	aw36515_set_driver(0);

	return 0;
}

static struct flashlight_operations aw36515_ops = {
	aw36515_open,
	aw36515_release,
	aw36515_ioctl,
	aw36515_strobe_store,
	aw36515_set_driver
};


/******************************************************************************
 * I2C device and driver
 *****************************************************************************/
static int aw36515_chip_init(struct aw36515_chip_data *chip)
{
	/* NOTE: Chip initialication move to
	*"set driver" operation for power saving issue.
	* aw36515_init();
	*/

	return 0;
}

/***************************************************************************/
/*AW36515 Debug file */
/***************************************************************************/
static ssize_t
aw36515_get_reg(struct device *cd, struct device_attribute *attr, char *buf)
{
	unsigned char reg_val;
	unsigned char i;
	ssize_t len = 0;

	for (i = 0; i < 0x0E; i++) {
		reg_val = aw36515_read_reg(aw36515_i2c_client, i);
		len += snprintf(buf+len, PAGE_SIZE-len,
			"reg0x%2X = 0x%2X\n", i, reg_val);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\r\n");
	return len;
}

static ssize_t aw36515_set_reg(struct device *cd,
		struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int databuf[2];

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2)
		aw36515_write_reg(aw36515_i2c_client, databuf[0], databuf[1]);
	return len;
}


static ssize_t aw36515_store(struct device *cd,
		struct device_attribute *attr, const char *buf, size_t len)
{
	if(buf[0] == '0')
	{
		aw36515_enable(AW36515_CHANNEL_CH1);
	}
	else if(buf[0] == '1')
	{
		aw36515_disable(AW36515_CHANNEL_CH1);
	}
	else if(buf[0] == '2')
	{
		aw36515_enable(AW36515_CHANNEL_CH2);
	}
	else if(buf[0] == '3')
	{
		aw36515_disable(AW36515_CHANNEL_CH2);
	}

	return len;
}


static DEVICE_ATTR(reg, 0660, aw36515_get_reg, aw36515_set_reg);
static DEVICE_ATTR(set, 0660, NULL, aw36515_store);

static int aw36515_create_sysfs(struct i2c_client *client)
{
	int err;
	struct device *dev = &(client->dev);

	err = device_create_file(dev, &dev_attr_reg);
	
	err = device_create_file(dev, &dev_attr_set);
	
	return err;
}

static int aw36515_parse_dt(struct device *dev,
		struct aw36515_platform_data *pdata)
{
	struct device_node *np, *cnp;
	u32 decouple = 0;
	int i = 0;

	if (!dev || !dev->of_node || !pdata)
		return -ENODEV;

	np = dev->of_node;

	pdata->channel_num = of_get_child_count(np);
	if (!pdata->channel_num) {
		AW36515_DEBUG("Parse no dt, node.\n");
		return 0;
	}
	AW36515_DEBUG("Channel number(%d).\n", pdata->channel_num);

	if (of_property_read_u32(np, "decouple", &decouple))
		AW36515_DEBUG("Parse no dt, decouple.\n");
	
	if (of_property_read_u32(np, "led_num", &led_num))
		AW36515_DEBUG("Parse no dt, led_num.\n");

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
				AW36515_NAME);
		pdata->dev_id[i].channel = i;
		pdata->dev_id[i].decouple = decouple;

		AW36515_DEBUG("Parse dt (type,ct,part,name,channel,decouple)=(%d,%d,%d,%s,%d,%d).\n",
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
aw36515_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct aw36515_chip_data *chip;
	struct aw36515_platform_data *pdata = client->dev.platform_data;
	int err;
	int i;

	AW36515_DEBUG("%s Probe start.\n", __func__);

	/* check i2c */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("Failed to check i2c functionality.\n");
		err = -ENODEV;
		goto err_out;
	}

	/* init chip private data */
	chip = kzalloc(sizeof(struct aw36515_chip_data), GFP_KERNEL);
	if (!chip) {
		err = -ENOMEM;
		goto err_out;
	}
	chip->client = client;

	/* init platform data */
	if (!pdata) {
		pr_err("Platform data does not exist\n");
		pdata =
		kzalloc(sizeof(struct aw36515_platform_data), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err_init_pdata;
		}
		chip->no_pdata = 1;
	}
	chip->pdata = pdata;

	err = aw36515_parse_dt(&g_aw36515_pdev->dev, chip->pdata);
	if (err)
		goto err_init_pdata;

	i2c_set_clientdata(client, chip);
	aw36515_i2c_client = client;

	/* init mutex and spinlock */
	mutex_init(&chip->lock);

	/* init work queue */
	INIT_WORK(&aw36515_work_ch1, aw36515_work_disable_ch1);
	INIT_WORK(&aw36515_work_ch2, aw36515_work_disable_ch2);

	/* init timer */
	hrtimer_init(&aw36515_timer_ch1, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw36515_timer_ch1.function = aw36515_timer_func_ch1;
	hrtimer_init(&aw36515_timer_ch2, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw36515_timer_ch2.function = aw36515_timer_func_ch2;
	aw36515_timeout_ms[AW36515_CHANNEL_CH1] = 100;
	aw36515_timeout_ms[AW36515_CHANNEL_CH2] = 100;

	/* init chip hw */
	aw36515_chip_init(chip);

	/* register flashlight device */
	if (pdata->channel_num) {
		for (i = 0; i < pdata->channel_num; i++) {
			if (flashlight_dev_register_by_device_id(&pdata->dev_id[i], &aw36515_ops)) {
				err = -EFAULT;
				goto err_free;
			}
		}
	} else {
		if (flashlight_dev_register(AW36515_NAME, &aw36515_ops)) {
			err = -EFAULT;
			goto err_free;
		}
	}

	/* clear usage count */
	use_count = 0;

	aw36515_create_sysfs(client);

	AW36515_DEBUG("%s Probe done.\n", __func__);

	return 0;

err_free:
	kfree(chip->pdata);
err_init_pdata:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
err_out:
	return err;
}

static int aw36515_i2c_remove(struct i2c_client *client)
{
	struct aw36515_chip_data *chip = i2c_get_clientdata(client);

	AW36515_DEBUG("Remove start.\n");

	/* flush work queue */
	flush_work(&aw36515_work_ch1);
	flush_work(&aw36515_work_ch2);

	/* unregister flashlight operations */
	flashlight_dev_unregister(AW36515_NAME);

	/* free resource */
	if (chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);

	AW36515_DEBUG("Remove done.\n");

	return 0;
}

static const struct i2c_device_id aw36515_i2c_id[] = {
	{AW36515_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id aw36515_i2c_of_match[] = {
	{.compatible = AW36515_DTNAME_I2C},
	{},
};
#endif

static struct i2c_driver aw36515_i2c_driver = {
	.driver = {
		   .name = AW36515_NAME,
#ifdef CONFIG_OF
		   .of_match_table = aw36515_i2c_of_match,
#endif
		   },
	.probe = aw36515_i2c_probe,
	.remove = aw36515_i2c_remove,
	.id_table = aw36515_i2c_id,
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int aw36515_probe(struct platform_device *pdev)
{
	AW36515_DEBUG("%s Probe start.\n", __func__);
	
	g_aw36515_pdev = pdev;

	if (i2c_add_driver(&aw36515_i2c_driver)) {
		pr_err("Failed to add i2c driver.\n");
		return -1;
	}

	AW36515_DEBUG("%s Probe done.\n", __func__);

	return 0;
}

static int aw36515_remove(struct platform_device *pdev)
{
	AW36515_DEBUG("Remove start.\n");

	i2c_del_driver(&aw36515_i2c_driver);

	AW36515_DEBUG("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id aw36515_of_match[] = {
	{.compatible = AW36515_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, aw36515_of_match);
#else
static struct platform_device aw36515_platform_device[] = {
	{
		.name = AW36515_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, aw36515_platform_device);
#endif

static struct platform_driver aw36515_platform_driver = {
	.probe = aw36515_probe,
	.remove = aw36515_remove,
	.driver = {
		.name = AW36515_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = aw36515_of_match,
#endif
	},
};

static int __init flashlight_aw36515_init(void)
{
	int ret = 0;

	AW36515_DEBUG("%s driver version %s.\n", __func__, AW36515_VERSION);
	
	pr_err("flashlight_aw36515_init----------start------\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&aw36515_platform_device);
	if (ret) {
		pr_err("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&aw36515_platform_driver);
	if (ret) {
		pr_err("Failed to register platform driver\n");
		return ret;
	}

	AW36515_DEBUG("flashlight_aw36515 Init done.\n");

	return 0;
}

static void __exit flashlight_aw36515_exit(void)
{
	AW36515_DEBUG("flashlight_aw36515-Exit start.\n");

	platform_driver_unregister(&aw36515_platform_driver);

	AW36515_DEBUG("flashlight_aw36515 Exit done.\n");
}

module_init(flashlight_aw36515_init);
module_exit(flashlight_aw36515_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Joseph <zhangzetao@awinic.com.cn>");
MODULE_DESCRIPTION("AW Flashlight AW36515 Driver");

