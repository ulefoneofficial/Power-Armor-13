/* prize-lifenfen-20161027 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/tlv.h>

struct i2c_client *tps65132_i2c_client = NULL;

struct pinctrl *pinctrltps65132;


enum tps65132_gpio_type {
	GPIO_DEFAULT = 0,
	GPIO_ENP_HIGH,
	GPIO_ENP_LOW,
	GPIO_ENN_HIGH,
	GPIO_ENN_LOW,
	GPIO_NUM
};

struct tps65132_gpio_attr {
	const char *name;
	bool gpio_prepare;
	struct pinctrl_state *gpioctrl;
};

static struct tps65132_gpio_attr tps65132_gpios[GPIO_NUM] = {
	[GPIO_DEFAULT] = {"default", false, NULL},
	[GPIO_ENP_HIGH] = {"lcdbias_enp_high", false, NULL},
	[GPIO_ENP_LOW] = {"lcdbias_enp_low", false, NULL},
	[GPIO_ENN_HIGH] = {"lcdbias_enn_high", false, NULL},
	[GPIO_ENN_LOW] = {"lcdbias_enn_low", false, NULL},
};

//prize wyq 20200224-start
int lcd_bias_enp_pin_gpio = -1;
int lcd_bias_enn_pin_gpio = -1;
int is_init_lcd_bias_gpio = -1;
//prize wyq 20200224-end

static int tps65132_get_dts_info(void *dev)
{
	int ret = 0;
	int i = 0;

	printk("%s\n", __func__);

	pinctrltps65132 = devm_pinctrl_get(dev);
	if (IS_ERR(pinctrltps65132)) {
		ret = PTR_ERR(pinctrltps65132);
		pr_err("Cannot find pinctrlaud!\n");
		return ret;
	}
	for (i = 1; i < ARRAY_SIZE(tps65132_gpios); i++) {
		tps65132_gpios[i].gpioctrl = pinctrl_lookup_state(pinctrltps65132, tps65132_gpios[i].name);
		if (IS_ERR(tps65132_gpios[i].gpioctrl)) {
			ret = PTR_ERR(tps65132_gpios[i].gpioctrl);
			pr_err("%s pinctrl_lookup_state %s fail %d\n", __func__, tps65132_gpios[i].name,
			       ret);
		} else {
			tps65132_gpios[i].gpio_prepare = true;
 			pr_err("%s pinctrl_lookup_state %s ok\n", __func__, tps65132_gpios[i].name);
		}
	}
	return ret;
}

//prize wyq 20200224-start
static int tps65132_get_dts_info_gpio(void *dev)
{
	struct device_node *node;
	int ret = 0;
	
	printk("%s\n", __func__);

	node = of_find_compatible_node(NULL,NULL,"prize,tps65132");
	if (node){
		lcd_bias_enp_pin_gpio = of_get_named_gpio(node,"lcd_bias_enp_gpio",0);
		if (lcd_bias_enp_pin_gpio < 0){
			printk("tps65132 get lcd_bias_enp_gpio fail %d\n",lcd_bias_enp_pin_gpio);
			return -1;
		}else{
			ret = gpio_request(lcd_bias_enp_pin_gpio,"lcd_bias_enp");
			if (ret) {
				printk("tps65132 request lcd_bias_enp fail %d\n",ret);
				return -1;
			}
			else
				printk("tps65132 request lcd_bias_enp %d ok\n", lcd_bias_enp_pin_gpio);

		}

		lcd_bias_enn_pin_gpio = of_get_named_gpio(node,"lcd_bias_enn_gpio",0);
		if (lcd_bias_enn_pin_gpio < 0){
			printk("tps65132 get lcd_bias_enn_gpio fail %d\n",lcd_bias_enn_pin_gpio);
			return -1;
		}else{
			ret = gpio_request(lcd_bias_enn_pin_gpio,"lcd_bias_enn");
			if (ret){
				printk("tps65132 request lcd_bias_enn fail %d\n",ret);
				return -1;
			}
			else
				printk("tps65132 request lcd_bias_enn %d ok\n", lcd_bias_enn_pin_gpio);
	
		}
		
		is_init_lcd_bias_gpio = 1;
	}else{
		printk("tps65132 try find node \"prize,lcm_power_gpio\"\n");
		node = of_find_compatible_node(NULL,NULL,"prize,lcm_power_gpio");
		if (node){
			lcd_bias_enp_pin_gpio = of_get_named_gpio(node,"gpio_lcd_bias_enp_gpio",0);
			if (lcd_bias_enp_pin_gpio < 0){
				printk("tps65132 get gpio_lcd_bias_enp_gpio fail %d\n",lcd_bias_enp_pin_gpio);
				return -1;
			}else{
				ret = gpio_request(lcd_bias_enp_pin_gpio,"lcd_bias_enp");
				if (ret) {
					printk("tps65132 request lcd_bias_enp fail %d\n",ret);
					return -1;
				}
				else
					printk("tps65132 request lcd_bias_enp %d ok\n", lcd_bias_enp_pin_gpio);

			}

			lcd_bias_enn_pin_gpio = of_get_named_gpio(node,"gpio_lcd_bias_enn_gpio",0);
			if (lcd_bias_enn_pin_gpio < 0){
				printk("tps65132 get gpio_lcd_bias_enn_gpio fail %d\n",lcd_bias_enn_pin_gpio);
				return -1;
			}else{
				ret = gpio_request(lcd_bias_enn_pin_gpio,"lcd_bias_enn");
				if (ret){
					printk("tps65132 request lcd_bias_enn fail %d\n",ret);
					return -1;
				}
				else
					printk("tps65132 request lcd_bias_enn %d ok\n", lcd_bias_enn_pin_gpio);
		
			}
			
			is_init_lcd_bias_gpio = 1;
		}else{
			printk("tps65132 get gpio node fail\n");
			
		}
	}

	return 0;
}
//prize wyq 20200224-end

int tps65132_vpos_enable(bool bEnable)
{
	int retval = 0;
	if (bEnable) {
		if (tps65132_gpios[GPIO_ENP_HIGH].gpio_prepare) {
			retval = pinctrl_select_state(pinctrltps65132, tps65132_gpios[GPIO_ENP_HIGH].gpioctrl);
			if (retval)
				pr_err("could not set tps65132_gpios[GPIO_ENP_HIGH] pins\n");
			else				
				pr_err("set tps65132_gpios[GPIO_ENP_HIGH] pins\n");
			
		}else if(is_init_lcd_bias_gpio == 1 && lcd_bias_enp_pin_gpio > 0) {
			gpio_direction_output(lcd_bias_enp_pin_gpio, 1);
			pr_debug("set lcd_bias_enp_pin_gpio high\n");
		}
		
	} else {
		if (tps65132_gpios[GPIO_ENP_LOW].gpio_prepare) {
			retval = pinctrl_select_state(pinctrltps65132, tps65132_gpios[GPIO_ENP_LOW].gpioctrl);
			if (retval)
				pr_err("could not set tps65132_gpios[GPIO_ENP_LOW] pins\n");			
			else				
				pr_err("set tps65132_gpios[GPIO_ENP_LOW] pins\n");
			
		}else if(is_init_lcd_bias_gpio == 1 && lcd_bias_enp_pin_gpio > 0) {
			gpio_direction_output(lcd_bias_enp_pin_gpio, 0);
			pr_debug("set lcd_bias_enp_pin_gpio low\n");
		}
	}
	
	return retval;
}
EXPORT_SYMBOL(tps65132_vpos_enable);

int tps65132_vneg_enable(bool bEnable)
{
	int retval = 0;
	if (bEnable) {
		if (tps65132_gpios[GPIO_ENN_HIGH].gpio_prepare) {
			retval = pinctrl_select_state(pinctrltps65132, tps65132_gpios[GPIO_ENN_HIGH].gpioctrl);
			if (retval)
				pr_err("could not set tps65132_gpios[GPIO_ENN_HIGH] pins\n");
			else				
				pr_err("set tps65132_gpios[GPIO_ENN_HIGH] pins\n");
		}else if(is_init_lcd_bias_gpio == 1 && lcd_bias_enn_pin_gpio > 0) {
			gpio_direction_output(lcd_bias_enn_pin_gpio, 1);
			pr_debug("set lcd_bias_enn_pin_gpio high\n");
		}
	
	} else {
		if (tps65132_gpios[GPIO_ENN_LOW].gpio_prepare) {
			retval = pinctrl_select_state(pinctrltps65132, tps65132_gpios[GPIO_ENN_LOW].gpioctrl);
			if (retval)
				pr_err("could not set tps65132_gpios[GPIO_ENN_LOW] pins\n");			
			else				
				pr_err("set tps65132_gpios[GPIO_ENN_LOW] pins\n");
			
		}else if(is_init_lcd_bias_gpio == 1 && lcd_bias_enn_pin_gpio > 0) {
			gpio_direction_output(lcd_bias_enn_pin_gpio, 0);
			pr_debug("set lcd_bias_enn_pin_gpio low\n");
		}
		
	}
	
	return retval;
}
EXPORT_SYMBOL(tps65132_vneg_enable);

int tps65132_write_bytes(unsigned char addr, unsigned char value)
{
	int ret = 0;
	struct i2c_client *client = tps65132_i2c_client;
	char write_data[2] = { 0 };
	write_data[0] = addr;
	write_data[1] = value;
	ret = i2c_master_send(client, write_data, 2);
	if (ret < 0)
		printk("tps65132 write data fail !!\n");
	return ret;
}

int __maybe_unused tps65132_read_byte(struct i2c_client *client, unsigned char reg, unsigned char *data)
{
	int ret;
	
	ret = i2c_smbus_read_byte_data(client, reg);
	if(ret < 0){
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}
 
	*data = (unsigned char)ret;
	return ret;
}

int tps65132_set_vpos_volt(int val){
	int ret;
	if (val > 6000){
		val = 6000;
		printk("TPS65132 set vpos %d exceed maximum, set to 6\n",val);
	}
	if (val < 4000){
		val = 4000;
		printk("TPS65132 set vpos %d lower than minimum, set to 4\n",val);
	}
	val = (val-4000)/100;
	if(NULL != tps65132_i2c_client){
		ret = tps65132_write_bytes(0x00, (unsigned char)val);
	}else{
		ret = -1;
	}
	return ret;
}
EXPORT_SYMBOL(tps65132_set_vpos_volt);

int tps65132_set_vneg_volt(int val){
	int ret;
	if (val > 6000){
		val = 6000;
		printk("TPS65132 set vpos %d exceed maximum, set to 6\n",val);
	}
	if (val < 4000){
		val = 4000;
		printk("TPS65132 set vpos %d lower than minimum, set to 4\n",val);
	}
	val = (val-4000)/100;
	if(NULL != tps65132_i2c_client){
		ret = tps65132_write_bytes(0x01, (unsigned char)val);
	}else{
		ret = -1;
	}
	return ret;
}
EXPORT_SYMBOL(tps65132_set_vneg_volt);

static int tps65132_i2c_probe(struct i2c_client *i2c_client,
				       const struct i2c_device_id *id)
{
	printk("%s\n", __func__);

	tps65132_i2c_client = i2c_client;

	tps65132_get_dts_info(&i2c_client->dev);

	tps65132_get_dts_info_gpio(&i2c_client->dev);//prize wyq 20200224

	return 0;
}

static int tps65132_i2c_remove(struct i2c_client *i2c_client)
{
	tps65132_i2c_client = NULL;
	return 0;
}
#if 0
static int tps65132_suspend(struct i2c_client *client, pm_message_t msg)
{
	printk("%s\n", __func__);
	return 0;
}
/*----------------------------------------------------------------------------*/
static int tps65132_resume(struct i2c_client *client)
{
	printk("%s\n", __func__);
	return 0;
}

static const struct dev_pm_ops tps65132_pm_ops = {
	.suspend = tps65132_suspend,
	.resume = tps65132_resume,
};
#endif
static const struct of_device_id tps65132_of_match[] = {
	{ .compatible = "ti,tps65132",},
	{},
};
MODULE_DEVICE_TABLE(of, tps65132_of_match);


static const struct i2c_device_id tps65132_id[] = {
	{"tps65132", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, tps65132_id);

static struct i2c_driver tps65132_i2c_driver = {
	.driver = {
		.name = "tps65132",
		.owner = THIS_MODULE,
		.of_match_table = tps65132_of_match,
	},
	.id_table = tps65132_id,
	.probe = tps65132_i2c_probe,
	.remove = tps65132_i2c_remove,
};

module_i2c_driver(tps65132_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("<prize@szprize.com>");
MODULE_DESCRIPTION("TPS65132 Regulator Driver");