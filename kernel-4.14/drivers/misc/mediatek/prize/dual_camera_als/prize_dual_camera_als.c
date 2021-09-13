/**************************************************************************
*  dts example
*	spc@47 {
*		compatible = "prize,spc_r";
*		reg = <0x47>;
*        sensor_type = <0x5f>;
*	};
**************************************************************************/

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include "prize_dual_camera_als.h"


static struct spc_data_t spc_r_devtype = {
	.name = "rear",
	.is_enabled = 0,
	.sensor_type = SENSOR_TYPE_UNKNOWN,
};

static struct spc_data_t spc_r_1_devtype = {
	.name = "rear_1",
	.is_enabled = 0,
	.sensor_type = SENSOR_TYPE_UNKNOWN,
};

static struct spc_data_t spc_f_devtype = {
	.name = "front",
	.is_enabled = 0,
	.sensor_type = SENSOR_TYPE_UNKNOWN,
};

extern struct sensor_info_t stk3x3x_info;

static struct kobject *spc_kobj = NULL;


static ssize_t device_open_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct spc_data_t *spc_data = i2c_get_clientdata(client);
	
	int ret = -1;

	//mdelay(100);
	mutex_lock(&spc_data->ops_mutex);
	if (!spc_data->is_enabled){
		spc_data->ops->set_power(client,1);
		ret = spc_data->ops->open(client);
	#if 1//GALAXYCORE_AUTO_DETECT
        camera_als_dbg("%s HH ret %d\n", __func__, ret);
		if (ret){
            camera_als_dbg("%s HH ret in\n", __func__);
			ret = 0;
			spc_data->ops->get_sensor_id(client,&ret);
			if ((ret == 0x51) || (ret == 0x52) || (ret == 0x53) || 
                (ret == 0x58) || (ret == 0x5A) || (ret == 0x5B) || 
                (ret == 0x5C) || (ret == 0x5E) || (ret == 0x5F) || 
                (ret == 0x36)) {
				spc_data->ops = &stk3x3x_info;
				ret = spc_data->ops->open(client);
			}else{
				ret = -EINVAL;
			}
		}
	#endif
		if (ret){
			spc_data->ops->set_power(client,0);
			mutex_unlock(&spc_data->ops_mutex);
            camera_als_dbg("%s rear_open fail %d\n", __func__, ret);
			return sprintf(buf, "%d\n", ret);
		}
		spc_data->is_enabled = 1;
	}
	mutex_unlock(&spc_data->ops_mutex);
    camera_als_dbg("%s rear_open %d\n", __func__, spc_data->is_enabled);
	return sprintf(buf, "%d\n", 0);
}
static DEVICE_ATTR(open, 0444, device_open_show, NULL);           //prize huangjiwu from 0644 to 0444

static ssize_t shutter_value_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct spc_data_t *spc_data = i2c_get_clientdata(client);
	int shutter_value = 0;
	
	if (unlikely(!spc_data->is_enabled)){
		return sprintf(buf, "%d\n", -1);
	}

	shutter_value=spc_data->ops->get_shutter(client);
	return sprintf(buf, "%d\n", shutter_value);
}
static DEVICE_ATTR(value, 0444, shutter_value_show, NULL);    //prize huangjiwu from 0644 to 0444


static ssize_t device_close_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct spc_data_t *spc_data = i2c_get_clientdata(client);

	mutex_lock(&spc_data->ops_mutex);
	if (spc_data->is_enabled){
		spc_data->ops->set_power(client,0);
		spc_data->is_enabled = 0;
	}
	mdelay(10);
	mutex_unlock(&spc_data->ops_mutex);
    camera_als_dbg("%s rear_close %d\n", __func__, spc_data->is_enabled);
	return sprintf(buf, "%d\n", 0);
}
static DEVICE_ATTR(close, 0444, device_close_show, NULL);    //prize huangjiwu from 0644 to 0444

static ssize_t device_type_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct spc_data_t *spc_data = i2c_get_clientdata(client);
	int ret = 0;

	switch(spc_data->sensor_type){
		case SENSOR_TYPE_STK3X3X: ret = sprintf(buf,"%s\n","STK3X3X"); break;
		default: ret = sprintf(buf,"%s\n","UNKNOWN"); break;
	}
	return ret;
}
static DEVICE_ATTR(type, 0444, device_type_show, NULL);      //prize huangjiwu from 0644 to 0444

static const struct of_device_id __maybe_unused spc_of_match[] = {
	{ .compatible = "prize,spc_r", .data = &spc_r_devtype },
	{ .compatible = "prize,spc_f", .data = &spc_f_devtype },
	{ .compatible = "prize,spc_r_1", .data = &spc_r_1_devtype },
};
MODULE_DEVICE_TABLE(of, spc_of_match);

static const struct i2c_device_id spc_id_table[] = {
	{ "SPC_R",	(kernel_ulong_t)&spc_r_devtype, },
	{ "SPC_F",	(kernel_ulong_t)&spc_f_devtype, },
	{ "SPC_R_1",	(kernel_ulong_t)&spc_r_1_devtype, },
	{ }
};
MODULE_DEVICE_TABLE(i2c, spc_id_table);

static int spc_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id){
	
	int ret;
	struct spc_data_t *spc_data;
	const struct of_device_id *of_id = of_match_device(spc_of_match,&client->dev);
	struct device_node *node;
	int sensor_type;
	
    camera_als_dbg("%s %s\n", __func__, client->name);
	
	spc_data = (struct spc_data_t *)of_id->data;
	i2c_set_clientdata(client,spc_data);
	
	mutex_init(&spc_data->ops_mutex);
	
	node = client->dev.of_node;
	//set sensor type
	if (!IS_ERR(node)){
		ret = of_property_read_u32(node,"sensor_type",&sensor_type);
		if (ret){
            camera_als_dbg("%s get sensor_type fail ret(%d)\n", __func__, ret);
			spc_data->sensor_type = SENSOR_TYPE_UNKNOWN;
			spc_data->ops = &stk3x3x_info;
		}else{
            if ((sensor_type == 0x51) || (sensor_type == 0x52) || (sensor_type == 0x53) || 
                (sensor_type == 0x58) || (sensor_type == 0x5A) || (sensor_type == 0x5B) || 
                (sensor_type == 0x5C) || (sensor_type == 0x5E) || (sensor_type == 0x5F) || 
                (sensor_type == 0x36)) {
				spc_data->sensor_type = SENSOR_TYPE_STK3X3X;
				spc_data->ops = &stk3x3x_info;
			}else{
				spc_data->sensor_type = SENSOR_TYPE_UNKNOWN;
				spc_data->ops = &stk3x3x_info;
			}
            camera_als_dbg("%s %s sensor_type(%x)\n", __func__, client->name, sensor_type);
		}
	}
	
	//create sysfs
	ret = sysfs_create_link(spc_kobj,&client->dev.kobj,client->name);
	if (ret){
        camera_als_dbg("%s spc sysfs_create_link fail\n", __func__);
	}
	ret = device_create_file(&client->dev, &dev_attr_open);
	if (ret){
        camera_als_dbg("%s spc device_create_file(dev_attr_open) fail\n", __func__);
	}
	ret = device_create_file(&client->dev, &dev_attr_value);
	if (ret){
        camera_als_dbg("%s spc device_create_file(dev_attr_value) fail\n", __func__);
	}
	ret = device_create_file(&client->dev, &dev_attr_close);
	if (ret){
        camera_als_dbg("%s spc device_create_file(dev_attr_close) fail\n", __func__);
	}
	ret = device_create_file(&client->dev, &dev_attr_type);
	if (ret){
        camera_als_dbg("%s spc device_create_file(dev_attr_type) fail\n", __func__);
	}
	
	return 0;
}

static int  spc_i2c_remove(struct i2c_client *client)
{
	struct spc_data_t *spc_data = i2c_get_clientdata(client);
	
	device_remove_file(&client->dev, &dev_attr_open);
	device_remove_file(&client->dev, &dev_attr_value);
	device_remove_file(&client->dev, &dev_attr_close);
	device_remove_file(&client->dev, &dev_attr_type);
	
	sysfs_remove_link(spc_kobj,client->name);
	
	camera_als_dbg("%s %s\n", __func__, client->name);
	if (spc_data == NULL){
		camera_als_dbg("%s spc_data==NULL err\n");
		return -1;
	}
	
	if (spc_data->is_enabled){
		spc_data->ops->set_power(client,0);
	}
	
	return 0;
}

static int __maybe_unused spc_i2c_suspend(struct device *dev)
{
	return 0;
}

static int __maybe_unused spc_i2c_resume(struct device *dev)
{
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops spc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(spc_i2c_suspend, spc_i2c_resume)
};
#endif

static struct i2c_driver spc_driver = {
	.driver = {
		.name		= "spc",
		.owner		= THIS_MODULE,
		//.of_match_table	= of_match_ptr(sp_cam_of_match),
		.of_match_table	= spc_of_match,
		.pm		= &spc_pm_ops,
	},
	.probe		= spc_i2c_probe,
	.remove		= spc_i2c_remove,
	.id_table	= spc_id_table,
};

//module_i2c_driver(spc_driver);


static int __init spc_init(void){
	int ret = -1;

	spc_kobj = kobject_create_and_add("spc", kernel_kobj);
	if (!spc_kobj){
		camera_als_dbg("%s kernel kobject_create_and_add error\n", __func__); 
		return -1;
	}

	ret = i2c_add_driver(&spc_driver);
	if (ret != 0){
		i2c_del_driver(&spc_driver);
	}
	
	return ret;
}

static void __exit spc_exit(void){
	
	i2c_del_driver(&spc_driver);
	
	if (!spc_kobj){
		return;
	}
	
	if (spc_kobj){
		kobject_put(spc_kobj);
	}
}
module_init(spc_init);
module_exit(spc_exit);

MODULE_LICENSE("GPL");

