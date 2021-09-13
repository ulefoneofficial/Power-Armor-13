#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

static int laser_light_pin1 = -1;
static int laser_light_pin2 = -1;

int laser_light_dts(void)
{
	struct device_node *node = NULL;
    
	printk("%s\n", __func__);
    node = of_find_compatible_node(NULL, NULL, "prize,laser_light");
	if (node == NULL) {
		printk("%s get node fail\n", __func__);
        return -1;
	}
	
	laser_light_pin1 = of_get_named_gpio(node, "laser_light_pin1", 0);
	if (laser_light_pin1 < 0) {
		printk("%s get laser_light_pin1 fail\n", __func__);
		return -1;
	}
	
	laser_light_pin2 = of_get_named_gpio(node, "laser_light_pin2", 0);
	if (laser_light_pin2 < 0) {
		printk("%s get laser_light_pin2 fail\n", __func__);
		return -1;
	}
	
	gpio_request(laser_light_pin1, "laser_light_pin1");
	gpio_request(laser_light_pin2, "laser_light_pin2");
	
	gpio_direction_output(laser_light_pin1, 1);
    gpio_set_value(laser_light_pin1, 0);
	gpio_direction_output(laser_light_pin2, 1);
    gpio_set_value(laser_light_pin2, 0);
	
	return 0;
}

static ssize_t laser_light_on_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%d\n", 0);
}

static ssize_t laser_light_on_store(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
	int ret = 0;
	int state = 0;
	
	ret = kstrtoint(buf, 10, &state);
	if (ret < 0)
		return ret;
	
	if (state == 1) {
		gpio_set_value(laser_light_pin1, 1);
		gpio_set_value(laser_light_pin2, 1);
	} else if (state == 0) {
		gpio_set_value(laser_light_pin1, 0);
		gpio_set_value(laser_light_pin2, 0);
	}
	
	return len;
}
static DEVICE_ATTR(on, 0664, laser_light_on_show, laser_light_on_store);

static int laser_light_probe(struct platform_device *pdev)
{
	int ret = 0;
	
    printk("%s\n", __func__);
	
	ret = laser_light_dts();
	if (ret < 0) {
		printk("%s laser_light_dts fail\n", __func__);
		return -1;
	}
	
	device_create_file(&pdev->dev, &dev_attr_on);
	
	printk("%s ok\n", __func__);
    
    return 0;
}

static int laser_light_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_device laser_light_dev = {
	.name = "laser_light",
	.id = -1,
};

static struct platform_driver laser_light_driver = {
	.probe = laser_light_probe,
	.remove = laser_light_remove,
	.driver = {
		.name = "laser_light",
		.owner = THIS_MODULE,
	},
};

static int __init laser_light_init(void) 
{
	int ret;
    
	printk("%s\n", __func__);
	
	ret = platform_device_register(&laser_light_dev);
    if (ret) {
		printk("****[%s] Unable to register device (%d)\n", __func__, ret);
		return ret;
	}
    
	ret = platform_driver_register(&laser_light_driver);
	if (ret) {
		printk("****[%s] Unable to register driver (%d)\n", __func__, ret);
		return ret;
	}
	
	return 0;
}

static void __exit laser_light_exit(void) {
	printk("%s\n", __func__);
	platform_driver_unregister(&laser_light_driver);
}

module_init(laser_light_init);
module_exit(laser_light_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("<chenjiaxi@szprize.com >");
MODULE_DESCRIPTION("Laser Light Driver");