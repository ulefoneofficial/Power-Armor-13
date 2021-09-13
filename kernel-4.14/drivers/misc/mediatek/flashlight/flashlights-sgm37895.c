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

#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/slab.h>

#include "flashlight-core.h"
#include "flashlight-dt.h"

/* device tree should be defined in flashlight-dt.h */
#ifndef SGM37895_DTNAME
#define SGM37895_DTNAME "mediatek,flashlights_sgm37895"
#endif
#ifndef SGM37895_DTNAME_I2C
#define SGM37895_DTNAME_I2C "sgmicro,sgm37895"
#endif

#define SGM37895_NAME "flashlights-sgm37895"

/* define registers */
#define SGM37895_REG_SILICON_REVISION (0x00)

#define SGM37895_REG_FLASH_FEATURE      (0x08)
#define SGM37895_INDUCTOR_CURRENT_LIMIT (0x40)
#define SGM37895_FLASH_RAMP_TIME        (0x00)
#define SGM37895_FLASH_TIMEOUT          (0x07)

#define SGM37895_REG_CURRENT_CONTROL (0x09)

#define SGM37895_REG_ENABLE (0x0A)
#define SGM37895_ENABLE_STANDBY (0x00)
#define SGM37895_ENABLE_TORCH (0x02)
#define SGM37895_ENABLE_FLASH (0x03)

#define SGM37895_REG_FLAG (0x0B)

/* define level */
#define SGM37895_LEVEL_NUM 7
#define SGM37895_LEVEL_TORCH 2
#define SGM37895_HW_TIMEOUT 575 /* ms */

/* define mutex and work queue */
static DEFINE_MUTEX(sgm37895_mutex);
static struct work_struct sgm37895_work;

/* define usage count */
static int use_count;

/* define i2c */
static struct i2c_client *sgm37895_i2c_client;

/* platform data */
struct sgm37895_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};

/* sgm37895 chip data */
struct sgm37895_chip_data {
	struct i2c_client *client;
	struct sgm37895_platform_data *pdata;
	struct mutex lock;
	struct pinctrl *sgm37895_pctrl;
	struct pinctrl_state *sgm37895_en_h;
	struct pinctrl_state *sgm37895_en_l;
	//unsigned char cs;
	//unsigned char current_cfg;
};


/******************************************************************************
 * sgm37895 operations
 *****************************************************************************/
#if 1
 /* this current is single led current,total current is x2 */
static const int sgm37895_current[SGM37895_LEVEL_NUM] = {
	 60,80,  //torch mode is  flash 10%
	 382, 480,  600,  800,  1000
};

static const unsigned char sgm37895_flash_level[SGM37895_LEVEL_NUM] = {
	0x69,0x55,// torch mode is  flash 10%
	0x3d,0x7d, 0x69, 0x55, 0x41	//1a 510 640 800 1000
};
#endif


static int sgm37895_level = -1;

static int sgm37895_is_torch(int level)
{
	if (level >= SGM37895_LEVEL_TORCH)
		return -1;

	return 0;
}

static int sgm37895_verify_level(int level)
{
	if (level < 0)
		level = 0;
	else if (level >= SGM37895_LEVEL_NUM)
		level = SGM37895_LEVEL_NUM - 1;

	return level;
}

/* i2c wrapper function */
static int sgm37895_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	struct sgm37895_chip_data *chip = i2c_get_clientdata(client);

	//mutex_lock(&chip->lock);
	//ret = i2c_smbus_write_byte_data(client, reg, val);
	//mutex_unlock(&chip->lock);

	//if (ret < 0)
	//	pr_err("failed writing at 0x%02x\n", reg);
	
	u8 buffer[1] = {0, };
    int retry = 3;
    struct i2c_msg msg[1];
	
    buffer[0] = val;

    msg->addr = client->addr;
    msg->flags = 0;
    msg->len = 1;
    msg->buf = buffer;

    while (retry--) {
        mutex_lock(&chip->lock);
        ret = i2c_transfer(client->adapter, msg, 1);
        mutex_unlock(&chip->lock);
        if (ret >= 0){
            break;
		}else{
			pr_err("failed writing at 0x%02x\n", reg);
		}
    }

	return ret;
}

static int sgm37895_read_reg(struct i2c_client *client, u8 reg)
{
	//int val = 0;
	struct sgm37895_chip_data *chip = i2c_get_clientdata(client);

	//mutex_lock(&chip->lock);
	//val = i2c_smbus_read_byte_data(client, reg);
	//mutex_unlock(&chip->lock);

	int err = 0;
    int retry = 3;
    struct i2c_msg msg[1];
    u8 buffer[1] = {0,};

    while ( retry-- ) {
        msg[0].addr = client->addr;
        msg[0].flags = I2C_M_RD;
        msg[0].len = 1;
        msg[0].buf = buffer;
		mutex_lock(&chip->lock);
        err = i2c_transfer( client->adapter, msg, 1 );
		mutex_unlock(&chip->lock);
        if ( err >= 0 ) {
            break;
        }else{
			pr_err("failed reading at 0x%02x\n", reg);
		}
    }

	return buffer[0];
}

/* flashlight enable function */
static int sgm37895_enable(void)
{
	unsigned char val;
	int ret = 0;
	struct sgm37895_chip_data *chip = i2c_get_clientdata(sgm37895_i2c_client);

	if (!sgm37895_is_torch(sgm37895_level)) {
		/* torch mode */
		val = 0xE8;
	} else {
		/* flash mode */
		val = 0xC8;
	}
	
	ret = sgm37895_write_reg(sgm37895_i2c_client, 00, val);
	val = sgm37895_flash_level[sgm37895_level];
	ret = sgm37895_write_reg(sgm37895_i2c_client, 00, val);
    pr_debug("%s LPP--00-status %x\n",__func__,sgm37895_read_reg(sgm37895_i2c_client, 0x00));//df
	if (sgm37895_is_torch(sgm37895_level)) {
		pinctrl_select_state(chip->sgm37895_pctrl,chip->sgm37895_en_h);
	}
	
	return ret;
}

/* flashlight disable function */
static int sgm37895_disable(void)
{
	int i=0;
	int val = 0;
	struct sgm37895_chip_data *chip = i2c_get_clientdata(sgm37895_i2c_client);

for(i=0;i<=1;i++){
	pinctrl_select_state(chip->sgm37895_pctrl,chip->sgm37895_en_l);
	pr_debug("%s LPP--11-status %x\n",__func__,sgm37895_read_reg(sgm37895_i2c_client, 0x00));//df
	sgm37895_write_reg(sgm37895_i2c_client, 00, 0x10);
	val = sgm37895_flash_level[sgm37895_level];
	//val = val|chip->cs;
	sgm37895_write_reg(sgm37895_i2c_client, 00, val);
}
	return 0;
}

/* set flashlight level */
static int sgm37895_set_level(int level)
{
	//unsigned char val;

	level = sgm37895_verify_level(level);
	sgm37895_level = level;

	//val = sgm37895_flash_level[level];

	//return sgm37895_write_reg(sgm37895_i2c_client, 00, val);
	return 0;
}

static int sgm37895_get_flag(void)
{
	return sgm37895_read_reg(sgm37895_i2c_client, SGM37895_REG_FLAG);
}

/* flashlight init */
int sgm37895_init(void)
{
	int ret;

	return ret;
}

/* flashlight uninit */
int sgm37895_uninit(void)
{
	sgm37895_disable();

	return 0;
}

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer sgm37895_timer;
static unsigned int sgm37895_timeout_ms;

static void sgm37895_work_disable(struct work_struct *data)
{
	pr_debug("work queue callback\n");
	sgm37895_disable();
}

static enum hrtimer_restart sgm37895_timer_func(struct hrtimer *timer)
{
	schedule_work(&sgm37895_work);
	return HRTIMER_NORESTART;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int sgm37895_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	ktime_t ktime;
	unsigned int s;
	unsigned int ns;

	//struct sgm37895_chip_data *chip = i2c_get_clientdata(sgm37895_i2c_client);
	
	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		pr_debug("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		sgm37895_timeout_ms = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_debug("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		sgm37895_set_level(fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		pr_debug("FLASH_IOC_SET_ONOFF(%d): %d\n",channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (sgm37895_timeout_ms) {
				s = sgm37895_timeout_ms / 1000;
				ns = sgm37895_timeout_ms % 1000 * 1000000;
				ktime = ktime_set(s, ns);
				hrtimer_start(&sgm37895_timer, ktime,
						HRTIMER_MODE_REL);
			}
			sgm37895_enable();
		} else {
			sgm37895_disable();
			hrtimer_cancel(&sgm37895_timer);
		}
		break;

	case FLASH_IOC_GET_DUTY_NUMBER:
		pr_debug("FLASH_IOC_GET_DUTY_NUMBER(%d)\n", channel);
		fl_arg->arg = SGM37895_LEVEL_NUM;
		break;

	case FLASH_IOC_GET_MAX_TORCH_DUTY:
		pr_debug("FLASH_IOC_GET_MAX_TORCH_DUTY(%d)\n", channel);
		fl_arg->arg = SGM37895_LEVEL_TORCH - 1;
		break;

	case FLASH_IOC_GET_DUTY_CURRENT:
		fl_arg->arg = sgm37895_verify_level(fl_arg->arg);
		pr_debug("FLASH_IOC_GET_DUTY_CURRENT(%d): %d\n",
				channel, (int)fl_arg->arg);
		fl_arg->arg = sgm37895_current[fl_arg->arg];
		break;

	case FLASH_IOC_GET_HW_TIMEOUT:
		pr_debug("FLASH_IOC_GET_HW_TIMEOUT(%d)\n", channel);
		fl_arg->arg = SGM37895_HW_TIMEOUT;
		break;

	case FLASH_IOC_GET_HW_FAULT:
		pr_debug("FLASH_IOC_GET_HW_FAULT(%d)\n", channel);
		fl_arg->arg = sgm37895_get_flag();
		break;

	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int sgm37895_open(void)
{
	/* Move to set driver for saving power */
	pr_debug("open driver\n");
	return 0;
}

static int sgm37895_release(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int sgm37895_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&sgm37895_mutex);
	if (set) {
		if (!use_count)
			ret = sgm37895_init();
		use_count++;
		pr_debug("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = sgm37895_uninit();
		if (use_count < 0)
			use_count = 0;
		pr_debug("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&sgm37895_mutex);

	return ret;
}

static ssize_t sgm37895_strobe_store(struct flashlight_arg arg)
{
	sgm37895_set_driver(1);
	sgm37895_set_level(arg.level);
	sgm37895_timeout_ms = 0;
	sgm37895_enable();
	msleep(arg.dur);
	sgm37895_disable();
	sgm37895_set_driver(0);

	return 0;
}

static struct flashlight_operations sgm37895_ops = {
	sgm37895_open,
	sgm37895_release,
	sgm37895_ioctl,
	sgm37895_strobe_store,
	sgm37895_set_driver
};


/******************************************************************************
 * I2C device and driver
 *****************************************************************************/
static int sgm37895_chip_init(struct sgm37895_chip_data *chip)
{
	/* NOTE: Chip initialication move to "set driver" for power saving.
	 * sgm37895_init();
	 */

	return 0;
}

static int sgm37895_parse_dt(struct device *dev,
		struct sgm37895_platform_data *pdata)
{
	struct device_node *np, *cnp;
	u32 decouple = 0;
	int i = 0;

	if (!dev || !dev->of_node || !pdata)
		return -ENODEV;

	np = dev->of_node;

	pdata->channel_num = of_get_child_count(np);
	if (!pdata->channel_num) {
		pr_info("Parse no dt, node.\n");
		return 0;
	}
	pr_info("Channel number(%d).\n", pdata->channel_num);

	if (of_property_read_u32(np, "decouple", &decouple))
		pr_info("Parse no dt, decouple.\n");

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
				SGM37895_NAME);
		pdata->dev_id[i].channel = i;
		pdata->dev_id[i].decouple = decouple;

		pr_info("Parse dt (type,ct,part,name,channel,decouple)=(%d,%d,%d,%s,%d,%d).\n",
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

static int sgm37895_i2c_probe(
		struct i2c_client *client, const struct i2c_device_id *id)
{
	struct sgm37895_chip_data *chip;
	int err,ret;
	struct pinctrl_state *pst_default;

	pr_debug("i2c probe start.\n");

	/* check i2c */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("Failed to check i2c functionality.\n");
		err = -ENODEV;
		goto err_out;
	}

	/* init chip private data */
	chip = kzalloc(sizeof(struct sgm37895_chip_data), GFP_KERNEL);
	if (!chip) {
		err = -ENOMEM;
		goto err_out;
	}
	chip->client = client;

	i2c_set_clientdata(client, chip);
	sgm37895_i2c_client = client;
	
	/* get pinctrl */
	chip->sgm37895_pctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(chip->sgm37895_pctrl)) {
		pr_err("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(chip->sgm37895_pctrl);
		return ret;
	}
	/* TODO: Flashlight XXX pin initialization */
	chip->sgm37895_en_h = pinctrl_lookup_state(chip->sgm37895_pctrl, "sgm37895_en_h");
	if (IS_ERR(chip->sgm37895_en_h)) {
		pr_err("Failed to init (sgm37895_en_h)\n");
		ret = PTR_ERR(chip->sgm37895_en_h);
	}
	chip->sgm37895_en_l = pinctrl_lookup_state(chip->sgm37895_pctrl, "sgm37895_en_l");
	if (IS_ERR(chip->sgm37895_en_l)) {
		pr_err("Failed to init (sgm37895_en_h)\n");
		ret = PTR_ERR(chip->sgm37895_en_l);
	}
	pst_default = pinctrl_lookup_state(chip->sgm37895_pctrl, "sgm37895_default");
	if (IS_ERR(pst_default)) {
		pr_err("Failed to init (sgm37895_default)\n");
		ret = PTR_ERR(pst_default);
	}
	pinctrl_select_state(chip->sgm37895_pctrl,pst_default);


	/* init mutex and spinlock */
	mutex_init(&chip->lock);

	/* init chip hw */
	sgm37895_chip_init(chip);

	pr_debug("i2c probe done.\n");

	return 0;

err_out:
	return err;
}

static int sgm37895_i2c_remove(struct i2c_client *client)
{
	struct sgm37895_chip_data *chip = i2c_get_clientdata(client);

	pr_debug("Remove start.\n");

	client->dev.platform_data = NULL;

	/* free resource */
	kfree(chip);

	pr_debug("Remove done.\n");

	return 0;
}

static const struct i2c_device_id sgm37895_i2c_id[] = {
	{SGM37895_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, sgm37895_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id sgm37895_i2c_of_match[] = {
	{.compatible = SGM37895_DTNAME_I2C},
	{},
};
MODULE_DEVICE_TABLE(of, sgm37895_i2c_of_match);
#endif

static struct i2c_driver sgm37895_i2c_driver = {
	.driver = {
		.name = SGM37895_NAME,
#ifdef CONFIG_OF
		.of_match_table = sgm37895_i2c_of_match,
#endif
	},
	.probe = sgm37895_i2c_probe,
	.remove = sgm37895_i2c_remove,
	.id_table = sgm37895_i2c_id,
};

/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int sgm37895_probe(struct platform_device *pdev)
{
	struct sgm37895_platform_data *pdata = dev_get_platdata(&pdev->dev);
	struct sgm37895_chip_data *chip = NULL;
	int err;
	int i;

	pr_debug("Probe start.\n");

	if (i2c_add_driver(&sgm37895_i2c_driver)) {
		pr_debug("Failed to add i2c driver.\n");
		return -1;
	}

	/* init platform data */
	if (!pdata) {
		pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err_free;
		}
		pdev->dev.platform_data = pdata;
		err = sgm37895_parse_dt(&pdev->dev, pdata);
		if (err)
			goto err_free;
	}

	/* init work queue */
	INIT_WORK(&sgm37895_work, sgm37895_work_disable);

	/* init timer */
	hrtimer_init(&sgm37895_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	sgm37895_timer.function = sgm37895_timer_func;
	sgm37895_timeout_ms = 800;

	/* clear usage count */
	use_count = 0;

	/* register flashlight device */
	if (pdata->channel_num) {
		for (i = 0; i < pdata->channel_num; i++)
			if (flashlight_dev_register_by_device_id(
						&pdata->dev_id[i],
						&sgm37895_ops)) {
				err = -EFAULT;
				goto err_free;
			}
	} else {
		if (flashlight_dev_register(SGM37895_NAME, &sgm37895_ops)) {
			err = -EFAULT;
			goto err_free;
		}
	}

	pr_debug("Probe done.\n");

	return 0;
err_free:
	chip = i2c_get_clientdata(sgm37895_i2c_client);
	i2c_set_clientdata(sgm37895_i2c_client, NULL);
	kfree(chip);
	return err;
}

static int sgm37895_remove(struct platform_device *pdev)
{
	struct sgm37895_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int i;

	pr_debug("Remove start.\n");

	i2c_del_driver(&sgm37895_i2c_driver);

	pdev->dev.platform_data = NULL;

	/* unregister flashlight device */
	if (pdata && pdata->channel_num)
		for (i = 0; i < pdata->channel_num; i++)
			flashlight_dev_unregister_by_device_id(
					&pdata->dev_id[i]);
	else
		flashlight_dev_unregister(SGM37895_NAME);

	/* flush work queue */
	flush_work(&sgm37895_work);

	pr_debug("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id sgm37895_of_match[] = {
	{.compatible = SGM37895_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, sgm37895_of_match);
#else
static struct platform_device sgm37895_platform_device[] = {
	{
		.name = SGM37895_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, sgm37895_platform_device);
#endif

static struct platform_driver sgm37895_platform_driver = {
	.probe = sgm37895_probe,
	.remove = sgm37895_remove,
	.driver = {
		.name = SGM37895_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = sgm37895_of_match,
#endif
	},
};

static int __init flashlight_sgm37895_init(void)
{
	int ret;

	pr_debug("Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&sgm37895_platform_device);
	if (ret) {
		pr_info("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&sgm37895_platform_driver);
	if (ret) {
		pr_info("Failed to register platform driver\n");
		return ret;
	}

	pr_debug("Init done.\n");

	return 0;
}

static void __exit flashlight_sgm37895_exit(void)
{
	pr_debug("Exit start.\n");

	platform_driver_unregister(&sgm37895_platform_driver);

	pr_debug("Exit done.\n");
}

module_init(flashlight_sgm37895_init);
module_exit(flashlight_sgm37895_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Xi Chen <xixi.chen@mediatek.com>");
MODULE_DESCRIPTION("MTK Flashlight SGM37895 Driver");

