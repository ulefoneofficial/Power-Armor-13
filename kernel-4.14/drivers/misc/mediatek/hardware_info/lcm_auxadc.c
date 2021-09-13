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

#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/kernel.h>

#include <linux/of_gpio.h>
#include <linux/iio/consumer.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/module.h>

static struct platform_device *plat_dev = NULL;

int lcm_auxadc_get_lcm_v(void){
	
	int ret = 0;
	int val = 0;
	struct iio_channel *lcm_channel = NULL;

	if (plat_dev == NULL){
		return -ENODEV;
	}

	lcm_channel = iio_channel_get(&plat_dev->dev, "lcm-ch");
	if (!IS_ERR_OR_NULL(lcm_channel)){
	#if defined(CONFIG_SC27XX_ADC)
		ret = iio_write_channel_attribute(lcm_channel, 1, 0, IIO_CHAN_INFO_SCALE);//0:0~1.2, 1:0~
		if (ret < 0) {
			printk("%s:set channel attribute big failed! %d\n", __func__, ret);
		}
	#endif
		ret = iio_read_channel_processed(lcm_channel, &val);
		if (ret < 0) {
			printk("%s:Busy/Timeout, IIO ch read failed %d\n", __func__, ret);
			return ret;
		}
		iio_channel_release(lcm_channel);

	#if defined(CONFIG_SC27XX_ADC)
		ret = val;
	#else
		/*val * 1500 / 4096*/
		ret = (val * 1500) >> 12;
	#endif
	}else{
		ret = PTR_ERR(lcm_channel);
		printk("[%s] fail to get auxadc iio ch0: %d, %p\n", __func__, ret, lcm_channel);
	}
	return ret;
}
EXPORT_SYMBOL(lcm_auxadc_get_lcm_v);

static int lcm_auxadc_probe(struct platform_device *pdev){
	int ret = 0;

	plat_dev = pdev;

	return ret;
}

static int lcm_auxadc_remove(struct platform_device *pdev){

	return 0;
}


#ifdef CONFIG_OF
static const struct of_device_id lcm_auxadc_of_match[] = {
        {.compatible = "prize,lcm_auxadc"},
        {},
};
MODULE_DEVICE_TABLE(of, lcm_auxadc_of_match);
#else
	static struct platform_device lcm_auxadc_platform_device = {
	.name = "lcm_auxadc",
	.id = 0,
	.dev = {}
};
MODULE_DEVICE_TABLE(platform, lcm_auxadc_platform_device);
#endif
static struct platform_driver lcm_auxadc_driver = {
	.driver = {
		.name = "lcm_auxadc",
	#ifdef CONFIG_OF
		.of_match_table = lcm_auxadc_of_match,
	#endif

	},
	.probe = lcm_auxadc_probe,
	.remove = lcm_auxadc_remove,
};

static int __init lcm_auxadc_init(void){

	return platform_driver_register(&lcm_auxadc_driver);
}

static void __exit lcm_auxadc_exit(void){
	platform_driver_unregister(&lcm_auxadc_driver);
}

fs_initcall(lcm_auxadc_init);
module_exit(lcm_auxadc_exit);