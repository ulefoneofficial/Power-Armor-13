#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <uapi/linux/sched/types.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include "fm78100b.h"

static unsigned int exergen_pdn_pin;
static struct i2c_client *exergen_i2c_client;
static struct kobject *fm78100b_kobj = NULL;
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
static s32 exergen_i2c_read(u8 addr)
{
	char get_byte=0;
   
	get_byte = i2c_read_reg(exergen_i2c_client, addr);
    return get_byte;
}

/**
 * @return: return 0 if success, otherwise return a negative number
 *          which contains the error code.
 */
static s32 exergen_i2c_write(u8 addr,u8 para)
{
	return i2c_write_reg(exergen_i2c_client, addr, para);
}

static void fm78100b_chip_disable(void)
{
	gpio_direction_output(exergen_pdn_pin,0);
	return;
}

static void fm78100b_chip_enable(void)
{
	gpio_direction_output(exergen_pdn_pin,1);
	return;

}
static void fm78100b_get_temp(void)
{
	return;
}
static int32_t fm78100b_read_ch1(void)
{
	uint8_t databuf[3],Drdy,convert_finish,wait_count;
	uint32_t ch1_rawdata;
			
	mdelay(5);
	convert_finish = 0;
	exergen_i2c_write(0x30, 0x09);//read mems
	mdelay(5);
	wait_count = 0;	
	while(convert_finish==0)
    {
        Drdy = exergen_i2c_read(0x02);
        if((Drdy&0x01) == 0x01)// 读 0x02值  等待 bit0为1  
		{
        	convert_finish = 1;
		}
		if(wait_count>100)   //  in case no ack & i2c fatel  goto
		{
			convert_finish = 1;
			return 0;
		}
		wait_count++;
		mdelay(5);		
    }
	
	databuf[0] = exergen_i2c_read(0x06);	//H
	databuf[1] = exergen_i2c_read(0x07);	//M 
	databuf[2] = exergen_i2c_read(0x08);	//L 	
	ch1_rawdata = ((databuf[2])|(databuf[1]<<8)|(databuf[0]<<16));	
	return ch1_rawdata;
}

static int32_t fm78100b_read_ch2(void)
{
	uint8_t databuf[3],Drdy,convert_finish,wait_count;
	uint32_t ch2_rawdata;
	
	convert_finish = 0;
	exergen_i2c_write(0x30, 0x08);//read ntc
	mdelay(5);
	wait_count = 0;	
	while(convert_finish==0)
    {
        Drdy = exergen_i2c_read(0x02);
        if((Drdy&0x01) == 0x01)// 读 0x02值  等待 bit0为1  
		{
        	convert_finish = 1;
		}
		if(wait_count>100)   //  in case no ack & i2c fatel  goto
		{
			convert_finish = 1;
			return 0;
		}
		wait_count++;
		mdelay(5);		
    }
	databuf[0] = exergen_i2c_read(0x06);	
	databuf[1] = exergen_i2c_read(0x07);	
	databuf[2] = exergen_i2c_read(0x08);	
	ch2_rawdata = ((databuf[2])|(databuf[1]<<8)|(databuf[0]<<16));	
	return ch2_rawdata;	
}
static uint32_t fm78100b_read_ch0(void)
{
	uint8_t databuf[4];	
	uint32_t ch0_rawdata;
	databuf[0] = exergen_i2c_read(0xb5);	
	databuf[1] = exergen_i2c_read(0xb6);		
	databuf[2] = exergen_i2c_read(0xb7);
	databuf[3] = exergen_i2c_read(0xb8);
	ch0_rawdata = (databuf[3])<<24|(databuf[2]<<16)|(databuf[1]<<8)|(databuf[0]);	
	printk(" 79100  ch0_rawdata = %d \r\n",ch0_rawdata);
	return ch0_rawdata;
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
static uint8_t fm78100b_chip_id(void)
{
	uint8_t chip_id =0;
	chip_id = exergen_i2c_read(0x01);
	EXERGEN_DMESG("EXERGEN fm78100b_chip_id(0x%x)\n",fm78100b_chip_id);
	return chip_id;
}

static ssize_t device_get_ch1_data_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int32_t data_temp;
	data_temp = fm78100b_read_ch1();
	return sprintf(buf, "%d\n", data_temp);
}
static DEVICE_ATTR(ch1, 0644, device_get_ch1_data_show, NULL);
static ssize_t device_get_ch2_data_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int32_t data_temp;
	mdelay(10);
	data_temp = fm78100b_read_ch2();
	return sprintf(buf, "%d\n", data_temp);
}
static DEVICE_ATTR(ch2, 0644, device_get_ch2_data_show, NULL);
static ssize_t device_get_ch0_data_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int32_t data_temp;
	mdelay(10);
	data_temp = fm78100b_read_ch0();
	return sprintf(buf, "%d\n", data_temp);
}
static DEVICE_ATTR(ch0, 0644, device_get_ch0_data_show, NULL);
static int fm78100b_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret,i = 10; 
	EXERGEN_DMESG("[fm78100b]: fm78100b_probe!!");
	exergen_i2c_client =  client;
	exergen_get_dts_info(client);
	while(i != 0){
		ret = fm78100b_chip_id();
		i--;
		if(ret == FM78100B_CHIP_ID)
		break;	
		mdelay(10);
		gpio_direction_output(exergen_pdn_pin,1);
		mdelay(30);
		gpio_direction_output(exergen_pdn_pin,0);
		mdelay(30);
		gpio_direction_output(exergen_pdn_pin,1);
		mdelay(30);		
	}
	if(i == 0)
		return -1;
	fm78100b_kobj = kobject_create_and_add("fm78100b", kernel_kobj);
	if (!fm78100b_kobj){
		EXERGEN_DMESG(" kernel kobject_create_and_add error \r\n"); 
		return -1;
	}
	
	ret = sysfs_create_link(fm78100b_kobj,&client->dev.kobj,"rawdata");
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
	ret = device_create_file(&client->dev, &dev_attr_ch0);
	if (ret){
		EXERGEN_DMESG("EXERGEN failed device_create_file(dev_attr_value)\n");
	}
	exergen_load_status = 1;
	return 0;
}
static int fm78100b_remove(struct i2c_client *client)
{	
	device_remove_file(&client->dev, &dev_attr_ch1);
	device_remove_file(&client->dev, &dev_attr_ch2);
	sysfs_remove_link(fm78100b_kobj,client->name);
	return 0;
}
static void fm78100b_resume(struct device *h)
{
	return;
}
static void fm78100b_suspend(struct device *h)
{
	return;
}

static const struct i2c_device_id fm78100b_fm78100b_id[] = {{FM78100B_DRIVER_NAME, 0}, {} };
static const struct of_device_id fm78100b_dt_match[] = {
    {.compatible = "exergen,fm78100b"},
    {},
};
MODULE_DEVICE_TABLE(of, fm78100b_dt_match);

static struct i2c_driver fm78100b_i2c_driver = {
    .driver = {
        .name = FM78100B_DRIVER_NAME,
        .of_match_table = of_match_ptr(fm78100b_dt_match),
    },
    .probe = fm78100b_probe,
    .remove = fm78100b_remove,
    .id_table = fm78100b_fm78100b_id,
};
static int fm78100b_local_init(void)
{
	EXERGEN_DMESG("[fm78100b]: fm78100b_local_init!!");
    if (i2c_add_driver(&fm78100b_i2c_driver) != 0) {
        EXERGEN_DMESG("[fm78100b]: Unable to add fm78100b i2c driver!!");
        return -1;
    }

    return 0;
}
static struct exergen_driver_t fm78100b_device_driver = {
    .exergen_device_name = FM78100B_DRIVER_NAME,
    .exergen_local_init = fm78100b_local_init,
    .open = fm78100b_chip_enable,
    .getvalue = fm78100b_get_temp,
    .close = fm78100b_chip_disable,
    .suspend = fm78100b_suspend,
    .resume = fm78100b_resume,
};
static int __init fm78100b_driver_init(void)
{
	EXERGEN_DMESG("[fm78100b]: Add FM78100B exergen driver!!");
    if (exergen_driver_add(&fm78100b_device_driver) < 0) {
        EXERGEN_DMESG("[fm78100b]: Add FM78100B exergen driver failed!!");
    }
    return 0;
}
static void __exit fm78100b_driver_exit(void)
{
    exergen_driver_remove(&fm78100b_device_driver);
}

module_init(fm78100b_driver_init);
module_exit(fm78100b_driver_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Prize exergen  fm78100b driver");
MODULE_AUTHOR("Liao Jie<liaojie@szprize.com>");