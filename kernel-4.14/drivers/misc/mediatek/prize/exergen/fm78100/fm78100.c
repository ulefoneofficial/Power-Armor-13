#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <uapi/linux/sched/types.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include "fm78100.h"

unsigned int exergen_pdn_pin;
struct i2c_client *exergen_i2c_client;
static struct kobject *fm78100_kobj = NULL;
static char i2c_write_reg(struct i2c_client *client, u8 addr, u8 data)
{
	int err, num = 0;
	char buf[32];

	if (!client)
		return -EINVAL;
	buf[num++] = addr;
	buf[num++] = data;

	err = i2c_master_send(client, buf, num);
	if (err < 0) {
		pr_err("send command error!!\n");
		return -EFAULT;
	}
	err = 0;

	return err;
}

static char i2c_read_reg(struct i2c_client *client, u8 addr)
{
	char ret;
	u8 rdbuf[2] = {0};
	u8 beg = addr;
	struct i2c_msg msgs[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= &beg,
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= rdbuf,
		},
	};
	
	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);
	
    return rdbuf[0];
}
s32 exergen_i2c_read(u8 addr)
{
	char get_byte=0;
   
	get_byte = i2c_read_reg(exergen_i2c_client, addr);
    return get_byte;
}

/**
 * @return: return 0 if success, otherwise return a negative number
 *          which contains the error code.
 */
s32 exergen_i2c_write(u8 addr,u8 para)
{
	return i2c_write_reg(exergen_i2c_client, addr, para);
}

void fm78100_chip_disable(void)
{
	exergen_i2c_write(0x09,0x03);//disable pd
	gpio_direction_output(exergen_pdn_pin,0);
}

void fm78100_chip_enable(void)
{
	gpio_direction_output(exergen_pdn_pin,1);
	fm78100_chip_init();
	mdelay(100);
	exergen_i2c_write(0x09,0x02);//enable pd
	mdelay(100);

}
void fm78100_get_temp(void)
{
	return;
}
int32_t fm78100_read_ch1(void)
{
	uint8_t databuf[3];
	int32_t data_temp;
	databuf[0] = exergen_i2c_read(0xa0);	
	databuf[1] = exergen_i2c_read(0xa1);	 
	databuf[2] = exergen_i2c_read(0xa2);	 	
	data_temp = ((databuf[0])|(databuf[1]<<8)|(databuf[2]<<16));	
	return data_temp;	
}

int32_t fm78100_read_ch2(void)
{
	uint8_t databuf[3];
	int32_t data_temp;
	databuf[0] = exergen_i2c_read(0xa3);	 
	databuf[1] = exergen_i2c_read(0xa4);	 
	databuf[2] = exergen_i2c_read(0xa5); 	
	data_temp = ((databuf[0])|(databuf[1]<<8)|(databuf[2]<<16));		
	return data_temp;	
}

static void exergen_get_dts_info(struct i2c_client * client)
{
	struct device_node *node;
	node = client->dev.of_node;
	if (!IS_ERR(node)){
		exergen_pdn_pin = of_get_named_gpio(node,"exergen_pdn_pin",0);
		EXERGEN_DMESG("EXERGEN exergen_pdn_pin(%d)\n",exergen_pdn_pin);
		gpio_request(exergen_pdn_pin,"exergen_pdn_pin");
		gpio_direction_output(exergen_pdn_pin,1);
	}else{
		EXERGEN_DMESG("EXERGEN get device node fail %s\n",client->name);
	}
}
uint8_t fm78100_chip_id(void)
{
	uint8_t chip_id =0;
	chip_id = exergen_i2c_read(0x00);
	return chip_id;
}
int fm78100_chip_init(void)
{
	s32 chip_id =0;
	int16_t  i =0 ;	
	int ret;
	chip_id = fm78100_chip_id();

	EXERGEN_DMESG("[fm78100]: chip_id = 0x%x\n!!",chip_id);
	if(chip_id != 0x22){
		  return -1;
	}
	
	for(i = 0;i < INIT_ARRAY_SIZE_FM78100; i++)     //AFE_self_first
	{		
		ret = exergen_i2c_write(init_register_array_fm78100[i][0],init_register_array_fm78100[i][1]);
		if(ret < 0)
			return -1;
			
	}
	mdelay(10);

	ret = exergen_i2c_write(0xc2, 0x00);// default:0x00    60 is debug mode turn off input single
	if(ret < 0)
		return -1;
	ret = exergen_i2c_write(0x13, 0x42);   //  0x42  MEMS=16gain,NTC=4gain
	if(ret < 0)
		return -1;
	mdelay(10);

	return 0;
}
static ssize_t device_get_ch1_data_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int32_t data_temp;
	data_temp = fm78100_read_ch1();
	return sprintf(buf, "%d\n", data_temp);
}
static DEVICE_ATTR(ch1, 0444, device_get_ch1_data_show, NULL);
static ssize_t device_get_ch2_data_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int32_t data_temp;
	mdelay(10);
	data_temp = fm78100_read_ch2();
	return sprintf(buf, "%d\n", data_temp);
}
static DEVICE_ATTR(ch2, 0444, device_get_ch2_data_show, NULL);
static int fm78100_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret,i = 10; 
	EXERGEN_DMESG("[fm78100]: fm78100_probe!!");
	exergen_i2c_client =  client;
	exergen_get_dts_info(client);
	while(i != 0){
		ret = fm78100_chip_id();
		i--;
		if(ret == 0x22)
		break;			
	}

	ret = sysfs_create_link(fm78100_kobj,&client->dev.kobj,"rawdata");
	if (ret){
		EXERGEN_DMESG("EXERGEN sysfs_create_link fail\n");
	}
	ret = device_create_file(&client->dev, &dev_attr_ch1);
	if (ret){
		EXERGEN_DMESG("EXERGEN failed device_create_file(dev_attr_open)\n");
	}
	ret = device_create_file(&client->dev, &dev_attr_ch2);
	if (ret){
		EXERGEN_DMESG("EXERGEN failed device_create_file(dev_attr_value)\n");
	}
	
	exergen_load_status = 1;
	return 0;
}
static int fm78100_remove(struct i2c_client *client)
{	
	device_remove_file(&client->dev, &dev_attr_ch1);
	device_remove_file(&client->dev, &dev_attr_ch2);
	sysfs_remove_link(fm78100_kobj,client->name);
	return 0;
}
static void fm78100_resume(struct device *h)
{
	return;
}
static void fm78100_suspend(struct device *h)
{
	return;
}

static const struct i2c_device_id fm78100_fm78100_id[] = {{FM78100_DRIVER_NAME, 0}, {} };
static const struct of_device_id fm78100_dt_match[] = {
    {.compatible = "exergen,fm78100"},
    {},
};
MODULE_DEVICE_TABLE(of, fm78100_dt_match);

static struct i2c_driver fm78100_i2c_driver = {
    .driver = {
        .name = FM78100_DRIVER_NAME,
        .of_match_table = of_match_ptr(fm78100_dt_match),
    },
    .probe = fm78100_probe,
    .remove = fm78100_remove,
    .id_table = fm78100_fm78100_id,
};
static int fm78100_local_init(void)
{
	EXERGEN_DMESG("[fm78100]: fm78100_local_init!!");
    if (i2c_add_driver(&fm78100_i2c_driver) != 0) {
        EXERGEN_DMESG("[fm78100]: Unable to add fm78100 i2c driver!!");
        return -1;
    }

    return 0;
}
static struct exergen_driver_t fm78100_device_driver = {
    .exergen_device_name = FM78100_DRIVER_NAME,
    .exergen_local_init = fm78100_local_init,
    .open = fm78100_chip_enable,
    .getvalue = fm78100_get_temp,
    .close = fm78100_chip_disable,
    .suspend = fm78100_suspend,
    .resume = fm78100_resume,
};
static int __init fm78100_driver_init(void)
{
	EXERGEN_DMESG("[fm78100]: Add FM78100 Touch driver!!");
	fm78100_kobj = kobject_create_and_add("fm78100", kernel_kobj);
	if (!fm78100_kobj){
		EXERGEN_DMESG(" kernel kobject_create_and_add error \r\n"); 
		return -1;
	}
	
    if (exergen_driver_add(&fm78100_device_driver) < 0) {
        EXERGEN_DMESG("[fm78100]: Add FM78100 Touch driver failed!!");
    }
    return 0;
}
static void __exit fm78100_driver_exit(void)
{
    exergen_driver_remove(&fm78100_device_driver);
}

module_init(fm78100_driver_init);
module_exit(fm78100_driver_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Prize exergen driver");
MODULE_AUTHOR("Liao Jie<liaojie@szprize.com>");