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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include "tps65132.h"

struct lcd_bias {
	struct device *dev;
	struct regulator *disp_bias_pos;
	struct regulator *disp_bias_neg;
	int gpio_lcd_ldo18_pin;
	int gpio_lcd_ldo28_pin;
	int gpio_lcd_fd_pin;
};
static struct lcd_bias *bias;

#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV) 
static int display_bias_regulator_init_dsv(void)
{
	int ret = 0;
	static int regulator_inited = 0;

	if (regulator_inited)
		return ret;

	/* please only get regulator once in a driver */
	bias->disp_bias_pos = regulator_get(NULL, "dsv_pos");
	if (IS_ERR(bias->disp_bias_pos)) { /* handle return value */
		ret = PTR_ERR(bias->disp_bias_pos);
		pr_info("get dsv_pos fail, error: %d\n", ret);
		return ret;
	}

	bias->disp_bias_neg = regulator_get(NULL, "dsv_neg");
	if (IS_ERR(bias->disp_bias_neg)) { /* handle return value */
		ret = PTR_ERR(bias->disp_bias_neg);
		pr_info("get dsv_neg fail, error: %d\n", ret);
		return ret;
	}

	regulator_inited = 1;
	return ret; /* must be 0 */
}
#endif

int disp_late_bias_enable(void){
	return 0;
}

int display_bias_enable(void)
{
	int ret = 0;
#if defined(CONFIG_PRIZE_TPS65132_SUPPORT)
	tps65132_vpos_enable(1);
	udelay(1000);
	tps65132_vneg_enable(1);
	tps65132_set_vpos_volt(5500);
	tps65132_set_vneg_volt(5500);
#elif defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
	int retval = 0;

	display_bias_regulator_init_dsv();

	/* set voltage with min & max*/
	ret = regulator_set_voltage(bias->disp_bias_pos, 5400000, 5400000);
	if (ret < 0)
			pr_info("set voltage disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_set_voltage(bias->disp_bias_neg, 5400000, 5400000);
	if (ret < 0)
			pr_info("set voltage disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;
		ret = regulator_enable(bias->disp_bias_pos);
	if (ret < 0)
			pr_info("enable regulator disp_bias_pos fail, ret = %d\n",
					ret);
	retval |= ret;

	ret = regulator_enable(bias->disp_bias_neg);
	if (ret < 0)
			pr_info("enable regulator disp_bias_neg fail, ret = %d\n",
					ret);
	retval |= ret;

	ret = retval;
#endif
	return ret;
}
//EXPORT_SYMBOL(display_bias_enable);

int display_bias_enable_v(unsigned int mv)
{
	int ret = 0;
#if defined(CONFIG_PRIZE_TPS65132_SUPPORT)
	tps65132_vpos_enable(1);
	udelay(1000);
	tps65132_vneg_enable(1);
	tps65132_set_vpos_volt(mv);
	tps65132_set_vneg_volt(mv);
#elif defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
	int retval = 0;

	display_bias_regulator_init_dsv();

	/* set voltage with min & max*/
	ret = regulator_set_voltage(bias->disp_bias_pos, mv*1000, mv*1000);
	if (ret < 0)
			pr_info("set voltage disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_set_voltage(bias->disp_bias_neg, mv*1000, mv*1000);
	if (ret < 0)
			pr_info("set voltage disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;
		ret = regulator_enable(bias->disp_bias_pos);
	if (ret < 0)
			pr_info("enable regulator disp_bias_pos fail, ret = %d\n",
					ret);
	retval |= ret;

	ret = regulator_enable(bias->disp_bias_neg);
	if (ret < 0)
			pr_info("enable regulator disp_bias_neg fail, ret = %d\n",
					ret);
	retval |= ret;

	ret = retval;
#endif
	return ret;
}
//EXPORT_SYMBOL(display_bias_enable_v);

int display_bias_disable(void)
{
	int ret = 0;
#if defined(CONFIG_PRIZE_TPS65132_SUPPORT)
	tps65132_vneg_enable(0);
	udelay(1000);
	tps65132_vpos_enable(0);
#elif defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
	int retval = 0;

	display_bias_regulator_init_dsv();

	ret = regulator_disable(bias->disp_bias_neg);
	if (ret < 0)
		pr_info("disable regulator disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_disable(bias->disp_bias_pos);
	if (ret < 0)
		pr_info("disable regulator disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	ret = retval;
#endif
	return ret;
}
//EXPORT_SYMBOL(display_bias_disable);

int display_bias_vpos_enable(int enable)
{
	int ret = 0;
#if defined(CONFIG_PRIZE_TPS65132_SUPPORT)
	if (enable){
		ret = tps65132_vpos_enable(1);
	}else{
		ret = tps65132_vpos_enable(0);
	}
#elif defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)

	display_bias_regulator_init_dsv();

	if (enable){
		ret = regulator_enable(bias->disp_bias_pos);
		if (ret < 0)
			pr_info("enable regulator disp_bias_pos fail, ret = %d\n", ret);
	}else{
		ret = regulator_disable(bias->disp_bias_pos);
		if (ret < 0)
			pr_info("disable regulator disp_bias_pos fail, ret = %d\n", ret);
	}
#endif
	return ret;
}
//EXPORT_SYMBOL(display_bias_vpos_enable);

int display_bias_vneg_enable(int enable)
{
	int ret = 0;
#if defined(CONFIG_PRIZE_TPS65132_SUPPORT)
	if (enable){
		ret = tps65132_vneg_enable(1);
	}else{
		ret = tps65132_vneg_enable(0);
	}
#elif defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)

	display_bias_regulator_init_dsv();

	if (enable){
		ret = regulator_enable(bias->disp_bias_neg);
		if (ret < 0)
			pr_info("enable regulator disp_bias_neg fail, ret = %d\n", ret);
	}else{
		ret = regulator_disable(bias->disp_bias_neg);
		if (ret < 0)
			pr_info("disable regulator disp_bias_neg fail, ret = %d\n", ret);
	}
#endif
	return ret;
}
//EXPORT_SYMBOL(display_bias_vneg_enable);

int display_bias_vpos_set(int mv)
{
	int ret = 0;
#if defined(CONFIG_PRIZE_TPS65132_SUPPORT)
	return tps65132_set_vpos_volt(mv);
#elif defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
	display_bias_regulator_init_dsv();

	/* set voltage with min & max*/
	ret = regulator_set_voltage(bias->disp_bias_pos, mv*1000, mv*1000);
	if (ret < 0)
			pr_info("set voltage disp_bias_pos fail, ret = %d\n", ret);
#endif
	return ret;
}
//EXPORT_SYMBOL(display_bias_vpos_set);

int display_bias_vneg_set(int mv)
{
	int ret = 0;
#if defined(CONFIG_PRIZE_TPS65132_SUPPORT)
	return tps65132_set_vneg_volt(mv);
#elif defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
	display_bias_regulator_init_dsv();

	/* set voltage with min & max*/
	ret = regulator_set_voltage(bias->disp_bias_neg, mv*1000, mv*1000);
	if (ret < 0)
			pr_info("set voltage disp_bias_neg fail, ret = %d\n", ret);
#endif
	return ret;
}
//EXPORT_SYMBOL(display_bias_vneg_set);


int display_ldo18_enable(int enable){
	int ret = 0;
	
	if (gpio_is_valid(bias->gpio_lcd_ldo18_pin)){
		gpio_request(bias->gpio_lcd_ldo18_pin,"lcd_v18");
		if (enable){
			gpio_direction_output(bias->gpio_lcd_ldo18_pin,1);
		}else{
			gpio_direction_output(bias->gpio_lcd_ldo18_pin,0);
		}
		gpio_free(bias->gpio_lcd_ldo18_pin);
	}

	return ret;
}
int display_ldo28_enable(int enable){
	int ret = 0;
	
	if (gpio_is_valid(bias->gpio_lcd_ldo28_pin)){
		gpio_request(bias->gpio_lcd_ldo18_pin,"lcd_v28");
		if (enable){
			gpio_direction_output(bias->gpio_lcd_ldo28_pin,1);
		}else{
			gpio_direction_output(bias->gpio_lcd_ldo28_pin,0);
		}
		gpio_free(bias->gpio_lcd_ldo28_pin);
	}
	return ret;
}
int display_fd_enable(int enable){
	int ret = 0;

	if (gpio_is_valid(bias->gpio_lcd_fd_pin)){
		gpio_request(bias->gpio_lcd_fd_pin,"lcd_v28");
		if (enable){
			gpio_direction_output(bias->gpio_lcd_fd_pin,1);
		}else{
			gpio_direction_output(bias->gpio_lcd_fd_pin,0);
		}
		gpio_free(bias->gpio_lcd_fd_pin);
	}
	return ret;
}

static int lcd_bias_parse_dt(struct lcd_bias *bias){

	struct device_node *node;

	node = of_find_compatible_node(NULL,NULL,"prize,lcm_power_gpio");
	if (node){
		bias->gpio_lcd_ldo18_pin = of_get_named_gpio(node,"gpio_lcd_ldo18_gpio",0);
		if (bias->gpio_lcd_ldo18_pin < 0){
			printk(KERN_ERR"lcm_pmic get gpio_lcd_ldo18_pin fail %d\n",bias->gpio_lcd_ldo18_pin);
		}

		bias->gpio_lcd_ldo28_pin = of_get_named_gpio(node,"gpio_lcd_ldo28_gpio",0);
		if (bias->gpio_lcd_ldo28_pin < 0){
			printk(KERN_INFO"lcm_pmic get gpio_lcd_ldo28_pin fail %d\n",bias->gpio_lcd_ldo28_pin);
		}

		bias->gpio_lcd_fd_pin = of_get_named_gpio(node,"gpio_lcd_fd-gpio",0);
		if (bias->gpio_lcd_fd_pin < 0){
			printk(KERN_INFO"lcm_pmic get gpio_lcd_fd_pin fail %d\n",bias->gpio_lcd_fd_pin);
		}

	}else{
		printk(KERN_ERR"lcm_pmic get of_node prize,lcm_power_gpio fail\n");
	}

	return 0;
}


static int lcd_bias_probe(struct platform_device *pdev)
{
	int ret = 0;

	dev_info(&pdev->dev, "Probing...");

	bias = devm_kzalloc(&pdev->dev, sizeof(*bias), GFP_KERNEL);
	if (!bias){
		return -ENOMEM;
	}

	bias->dev = &pdev->dev;
	platform_set_drvdata(pdev, bias);

	ret = lcd_bias_parse_dt(bias);

#if defined(CONFIG_PRIZE_TPS65132_SUPPORT)
	tps65132_init();
#endif

	dev_info(&pdev->dev, "probe OK\n");
	return 0;

}

static int lcd_bias_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_device lcd_bias_dev = {
	.name = "prize_lcd_bias",
	.id = -1,
};

static struct platform_driver lcd_bias_driver = {
	.probe = lcd_bias_probe,
	.remove = lcd_bias_remove,
	.driver = {
		.name = "prize_lcd_bias",
		.owner = THIS_MODULE,
	},
};

static int __init lcd_bias_init(void)
{
	platform_device_register(&lcd_bias_dev);
	return platform_driver_register(&lcd_bias_driver);
}

static void __exit lcd_bias_exit(void)
{
	platform_driver_unregister(&lcd_bias_driver);
}

subsys_initcall(lcd_bias_init);
module_exit(lcd_bias_exit);

MODULE_LICENSE("GPL");
