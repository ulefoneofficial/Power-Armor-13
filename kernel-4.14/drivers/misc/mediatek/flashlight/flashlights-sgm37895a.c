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
#ifndef S37895A_DTNAME
#define S37895A_DTNAME "mediatek,flashlights_s37895a"
#endif
#ifndef S37895A_DTNAME_I2C
#define S37895A_DTNAME_I2C "sgmicro,s37895a"
#endif

#define S37895A_NAME "flashlights-s37895a"

/* define registers */
#define S37895A_REG_SILICON_REVISION (0x00)

#define S37895A_REG_FLASH_FEATURE      (0x08)
#define S37895A_INDUCTOR_CURRENT_LIMIT (0x40)
#define S37895A_FLASH_RAMP_TIME        (0x00)
#define S37895A_FLASH_TIMEOUT          (0x07)

#define S37895A_REG_CURRENT_CONTROL (0x09)

#define S37895A_REG_ENABLE (0x0A)
#define S37895A_ENABLE_STANDBY (0x00)
#define S37895A_ENABLE_TORCH (0x02)
#define S37895A_ENABLE_FLASH (0x03)

#define S37895A_REG_FLAG (0x0B)

/* define level */
#define S37895A_LEVEL_NUM 7
#define S37895A_LEVEL_TORCH 2
#define S37895A_HW_TIMEOUT 575 /* ms */

/* define mutex and work queue */
static DEFINE_MUTEX(s37895a_mutex);
static struct work_struct s37895a_work;

/* define usage count */
static int use_count;

/* define i2c */
static struct i2c_client *s37895a_i2c_client;

/* platform data */
struct s37895a_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};

/* s37895a chip data */
struct s37895a_chip_data {
	struct i2c_client *client;
	struct s37895a_platform_data *pdata;
	struct mutex lock;
	struct pinctrl *s37895a_pctrl;
	struct pinctrl_state *s37895a_en_h;
	struct pinctrl_state *s37895a_en_l;
	//unsigned char cs;
	//unsigned char current_cfg;
};


/******************************************************************************
 * s37895a operations
 *****************************************************************************/
#if 1
 /* this current is single led current,total current is x2 */
static const int s37895a_current[S37895A_LEVEL_NUM] = {
	 60,80,  //torch mode is  flash 10%
	 382, 480,  600,  800,  1000
};

static const unsigned char s37895a_flash_level[S37895A_LEVEL_NUM] = {
	0x69,0x55,// torch mode is  flash 10%
	0x3d,0x7d, 0x69, 0x55, 0x41	//1a 510 640 800 1000
};
#endif

static int s37895a_level = -1;

static int s37895a_is_torch(int level)
{
	if (level >= S37895A_LEVEL_TORCH)
		return -1;

	return 0;
}

static int s37895a_verify_level(int level)
{
	if (level < 0)
		level = 0;
	else if (level >= S37895A_LEVEL_NUM)
		level = S37895A_LEVEL_NUM - 1;

	return level;
}

/* i2c wrapper function */
static int s37895a_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	struct s37895a_chip_data *chip = i2c_get_clientdata(client);

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

static int s37895a_read_reg(struct i2c_client *client, u8 reg)
{
	//int val = 0;
	struct s37895a_chip_data *chip = i2c_get_clientdata(client);

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
static int s37895a_enable(void)
{
	unsigned char val;
	int ret = 0;
	struct s37895a_chip_data *chip = i2c_get_clientdata(s37895a_i2c_client);

	if (!s37895a_is_torch(s37895a_level)) {
		/* torch mode */
		val = 0xE8;
	} else {
		/* flash mode */
		val = 0xC8;
	}

	ret = s37895a_write_reg(s37895a_i2c_client, 00, val);
	val = s37895a_flash_level[s37895a_level];
	ret = s37895a_write_reg(s37895a_i2c_client, 00, val);
	if (s37895a_is_torch(s37895a_level)) {
		pinctrl_select_state(chip->s37895a_pctrl,chip->s37895a_en_h);
	}
	return ret;
}

/* flashlight disable function */
static int s37895a_disable(void)
{
	struct s37895a_chip_data *chip = i2c_get_clientdata(s37895a_i2c_client);

	pinctrl_select_state(chip->s37895a_pctrl,chip->s37895a_en_l);
	pr_debug("%s status %x\n",__func__,s37895a_read_reg(s37895a_i2c_client, 0x00));
	s37895a_write_reg(s37895a_i2c_client, 00, 0x10);
	return 0;
}

/* set flashlight level */
static int s37895a_set_level(int level)
{
	//unsigned char val;

	level = s37895a_verify_level(level);
	s37895a_level = level;

	//val = s37895a_flash_level[level];

	//return s37895a_write_reg(s37895a_i2c_client, 00, val);
	return 0;
}

static int s37895a_get_flag(void)
{
	return s37895a_read_reg(s37895a_i2c_client, S37895A_REG_FLAG);
}

/* flashlight init */
int s37895a_init(void)
{
	int ret;

	return ret;
}

/* flashlight uninit */
int s37895a_uninit(void)
{
	s37895a_disable();

	return 0;
}

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer s37895a_timer;
static unsigned int s37895a_timeout_ms;

static void s37895a_work_disable(struct work_struct *data)
{
	pr_debug("work queue callback\n");
	s37895a_disable();
}

static enum hrtimer_restart s37895a_timer_func(struct hrtimer *timer)
{
	schedule_work(&s37895a_work);
	return HRTIMER_NORESTART;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int s37895a_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	ktime_t ktime;
	unsigned int s;
	unsigned int ns;

//	struct s37895a_chip_data *chip = i2c_get_clientdata(s37895a_i2c_client);
	
	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		pr_debug("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		s37895a_timeout_ms = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_debug("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		s37895a_set_level(fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		pr_debug("FLASH_IOC_SET_ONOFF(%d): %d\n",channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (s37895a_timeout_ms) {
				s = s37895a_timeout_ms / 1000;
				ns = s37895a_timeout_ms % 1000 * 1000000;
				ktime = ktime_set(s, ns);
				hrtimer_start(&s37895a_timer, ktime,
						HRTIMER_MODE_REL);
			}
			s37895a_enable();
		} else {
			s37895a_disable();
			hrtimer_cancel(&s37895a_timer);
		}
		break;

	case FLASH_IOC_GET_DUTY_NUMBER:
		pr_debug("FLASH_IOC_GET_DUTY_NUMBER(%d)\n", channel);
		fl_arg->arg = S37895A_LEVEL_NUM;
		break;

	case FLASH_IOC_GET_MAX_TORCH_DUTY:
		pr_debug("FLASH_IOC_GET_MAX_TORCH_DUTY(%d)\n", channel);
		fl_arg->arg = S37895A_LEVEL_TORCH - 1;
		break;

	case FLASH_IOC_GET_DUTY_CURRENT:
		fl_arg->arg = s37895a_verify_level(fl_arg->arg);
		pr_debug("FLASH_IOC_GET_DUTY_CURRENT(%d): %d\n",
				channel, (int)fl_arg->arg);
		fl_arg->arg = s37895a_current[fl_arg->arg];
		break;

	case FLASH_IOC_GET_HW_TIMEOUT:
		pr_debug("FLASH_IOC_GET_HW_TIMEOUT(%d)\n", channel);
		fl_arg->arg = S37895A_HW_TIMEOUT;
		break;

	case FLASH_IOC_GET_HW_FAULT:
		pr_debug("FLASH_IOC_GET_HW_FAULT(%d)\n", channel);
		fl_arg->arg = s37895a_get_flag();
		break;

	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int s37895a_open(void)
{
	/* Move to set driver for saving power */
	pr_debug("open driver\n");
	return 0;
}

static int s37895a_release(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int s37895a_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&s37895a_mutex);
	if (set) {
		if (!use_count)
			ret = s37895a_init();
		use_count++;
		pr_debug("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = s37895a_uninit();
		if (use_count < 0)
			use_count = 0;
		pr_debug("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&s37895a_mutex);

	return ret;
}

static ssize_t s37895a_strobe_store(struct flashlight_arg arg)
{
	s37895a_set_driver(1);
	s37895a_set_level(arg.level);
	s37895a_timeout_ms = 0;
	s37895a_enable();
	msleep(arg.dur);
	s37895a_disable();
	s37895a_set_driver(0);

	return 0;
}

static struct flashlight_operations s37895a_ops = {
	s37895a_open,
	s37895a_release,
	s37895a_ioctl,
	s37895a_strobe_store,
	s37895a_set_driver
};


/******************************************************************************
 * I2C device and driver
 *****************************************************************************/
static int s37895a_chip_init(struct s37895a_chip_data *chip)
{
	/* NOTE: Chip initialication move to "set driver" for power saving.
	 * s37895a_init();
	 */

	return 0;
}

static int s37895a_parse_dt(struct device *dev,
		struct s37895a_platform_data *pdata)
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
				S37895A_NAME);
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

static int s37895a_i2c_probe(
		struct i2c_client *client, const struct i2c_device_id *id)
{
	struct s37895a_chip_data *chip;
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
	chip = kzalloc(sizeof(struct s37895a_chip_data), GFP_KERNEL);
	if (!chip) {
		err = -ENOMEM;
		goto err_out;
	}
	chip->client = client;

	i2c_set_clientdata(client, chip);
	s37895a_i2c_client = client;
	
	/* get pinctrl */
	chip->s37895a_pctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(chip->s37895a_pctrl)) {
		pr_err("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(chip->s37895a_pctrl);
		return ret;
	}
	/* TODO: Flashlight XXX pin initialization */
	chip->s37895a_en_h = pinctrl_lookup_state(chip->s37895a_pctrl, "s37895a_en_h");
	if (IS_ERR(chip->s37895a_en_h)) {
		pr_err("Failed to init (s37895a_en_h)\n");
		ret = PTR_ERR(chip->s37895a_en_h);
	}
	chip->s37895a_en_l = pinctrl_lookup_state(chip->s37895a_pctrl, "s37895a_en_l");
	if (IS_ERR(chip->s37895a_en_l)) {
		pr_err("Failed to init (s37895a_en_h)\n");
		ret = PTR_ERR(chip->s37895a_en_l);
	}
	pst_default = pinctrl_lookup_state(chip->s37895a_pctrl, "s37895a_default");
	if (IS_ERR(pst_default)) {
		pr_err("Failed to init (s37895a_default)\n");
		ret = PTR_ERR(pst_default);
	}
	pinctrl_select_state(chip->s37895a_pctrl,pst_default);

	/* init mutex and spinlock */
	mutex_init(&chip->lock);

	/* init chip hw */
	s37895a_chip_init(chip);

	pr_debug("i2c probe done.\n");

	return 0;

err_out:
	return err;
}

static int s37895a_i2c_remove(struct i2c_client *client)
{
	struct s37895a_chip_data *chip = i2c_get_clientdata(client);

	pr_debug("Remove start.\n");

	client->dev.platform_data = NULL;

	/* free resource */
	kfree(chip);

	pr_debug("Remove done.\n");

	return 0;
}

static const struct i2c_device_id s37895a_i2c_id[] = {
	{S37895A_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, s37895a_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id s37895a_i2c_of_match[] = {
	{.compatible = S37895A_DTNAME_I2C},
	{},
};
MODULE_DEVICE_TABLE(of, s37895a_i2c_of_match);
#endif

static struct i2c_driver s37895a_i2c_driver = {
	.driver = {
		.name = S37895A_NAME,
#ifdef CONFIG_OF
		.of_match_table = s37895a_i2c_of_match,
#endif
	},
	.probe = s37895a_i2c_probe,
	.remove = s37895a_i2c_remove,
	.id_table = s37895a_i2c_id,
};

/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int s37895a_probe(struct platform_device *pdev)
{
	struct s37895a_platform_data *pdata = dev_get_platdata(&pdev->dev);
	struct s37895a_chip_data *chip = NULL;
	int err;
	int i;

	pr_debug("Probe start.\n");

	if (i2c_add_driver(&s37895a_i2c_driver)) {
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
		err = s37895a_parse_dt(&pdev->dev, pdata);
		if (err)
			goto err_free;
	}

	/* init work queue */
	INIT_WORK(&s37895a_work, s37895a_work_disable);

	/* init timer */
	hrtimer_init(&s37895a_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	s37895a_timer.function = s37895a_timer_func;
	s37895a_timeout_ms = 800;

	/* clear usage count */
	use_count = 0;

	/* register flashlight device */
	if (pdata->channel_num) {
		for (i = 0; i < pdata->channel_num; i++)
			if (flashlight_dev_register_by_device_id(
						&pdata->dev_id[i],
						&s37895a_ops)) {
				err = -EFAULT;
				goto err_free;
			}
	} else {
		if (flashlight_dev_register(S37895A_NAME, &s37895a_ops)) {
			err = -EFAULT;
			goto err_free;
		}
	}

	pr_debug("Probe done.\n");

	return 0;
err_free:
	chip = i2c_get_clientdata(s37895a_i2c_client);
	i2c_set_clientdata(s37895a_i2c_client, NULL);
	kfree(chip);
	return err;
}

static int s37895a_remove(struct platform_device *pdev)
{
	struct s37895a_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int i;

	pr_debug("Remove start.\n");

	i2c_del_driver(&s37895a_i2c_driver);

	pdev->dev.platform_data = NULL;

	/* unregister flashlight device */
	if (pdata && pdata->channel_num)
		for (i = 0; i < pdata->channel_num; i++)
			flashlight_dev_unregister_by_device_id(
					&pdata->dev_id[i]);
	else
		flashlight_dev_unregister(S37895A_NAME);

	/* flush work queue */
	flush_work(&s37895a_work);

	pr_debug("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id s37895a_of_match[] = {
	{.compatible = S37895A_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, s37895a_of_match);
#else
static struct platform_device s37895a_platform_device[] = {
	{
		.name = S37895A_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, s37895a_platform_device);
#endif

static struct platform_driver s37895a_platform_driver = {
	.probe = s37895a_probe,
	.remove = s37895a_remove,
	.driver = {
		.name = S37895A_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = s37895a_of_match,
#endif
	},
};

static int __init flashlight_s37895a_init(void)
{
	int ret;

	pr_debug("Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&s37895a_platform_device);
	if (ret) {
		pr_info("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&s37895a_platform_driver);
	if (ret) {
		pr_info("Failed to register platform driver\n");
		return ret;
	}

	pr_debug("Init done.\n");

	return 0;
}

static void __exit flashlight_s37895a_exit(void)
{
	pr_debug("Exit start.\n");

	platform_driver_unregister(&s37895a_platform_driver);

	pr_debug("Exit done.\n");
}

module_init(flashlight_s37895a_init);
module_exit(flashlight_s37895a_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Xi Chen <xixi.chen@mediatek.com>");
MODULE_DESCRIPTION("MTK Flashlight S37895A Driver");

