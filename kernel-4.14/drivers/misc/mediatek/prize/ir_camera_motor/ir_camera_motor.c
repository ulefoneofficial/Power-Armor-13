#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>

#define IR_CAMERA_MOTOR_DEBUG
#ifdef IR_CAMERA_MOTOR_DEBUG
#define ir_camera_motor_log(fmt,arg...) \
	do{\
		printk("<<ir-camera-motor>>[%d]"fmt"", __LINE__, ##arg);\
    }while(0)
#else
#define camera_als_dbg(fmt,arg...)
#endif

int ir_cut_ldo_en1_pin = -1;
int ir_cut_ldo_en2_pin = -1;
int gpio_icf_en_pin = -1;
int gpio_icf_disable_pin = -1;

static struct proc_dir_entry *ir_camera_motor_proc_entry;

static int ir_camera_motor_read_proc(struct seq_file *m, void *v)
{
    return 0;
}

static int ir_camera_motor_open(struct inode *inode, struct file *file)
{
    return single_open(file, ir_camera_motor_read_proc, inode->i_private);
}

static ssize_t ir_camera_motor_write_proc(struct file *filp, const char __user *buff, size_t count, loff_t *data)
{
    char value[2] = {0};
    
	if (ir_camera_motor_proc_entry == NULL) {
		ir_camera_motor_log("ir_camera_motor_proc_entry pointer null!\n");
		return -EINVAL;
	}
    
    if (count == 0 || count > 2)
		return -EINVAL;
    
	if (copy_from_user(value, buff, 2))
		return -EINVAL;
    
    if (value[0] == '1') {
        ir_camera_motor_log("ir camera motor is 1\n");

        //control +
        gpio_set_value(ir_cut_ldo_en1_pin, 1);
        gpio_set_value(gpio_icf_disable_pin, 1);
        gpio_set_value(ir_cut_ldo_en2_pin, 0);
        gpio_set_value(gpio_icf_en_pin, 0);
        mdelay(1000);
        
        //default
        gpio_set_value(ir_cut_ldo_en1_pin, 0);
        gpio_set_value(gpio_icf_disable_pin, 0);
        gpio_set_value(ir_cut_ldo_en2_pin, 0);
        gpio_set_value(gpio_icf_en_pin, 0);
        mdelay(10);

    } else if (value[0] == '0') {
        ir_camera_motor_log("ir camera motor is 0\n");

        //control -
        gpio_set_value(ir_cut_ldo_en1_pin, 0);
        gpio_set_value(gpio_icf_disable_pin, 0);
        gpio_set_value(ir_cut_ldo_en2_pin, 1);
        gpio_set_value(gpio_icf_en_pin, 1);
        mdelay(1000);
       
        //default
        gpio_set_value(ir_cut_ldo_en1_pin, 0);
        gpio_set_value(gpio_icf_disable_pin, 0);
        gpio_set_value(ir_cut_ldo_en2_pin, 0);
        gpio_set_value(gpio_icf_en_pin, 0);
        mdelay(10);
    }

	return count;
}

static const struct file_operations ir_camera_motor_fops = {
	.owner = THIS_MODULE,
	.open = ir_camera_motor_open,
	.write = ir_camera_motor_write_proc,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

int ir_camera_motor_dts(void)
{
    struct device_node *node;
	
	node = of_find_compatible_node(NULL, NULL, "prize,ir_camera_motor");
	if (!node) {
		ir_camera_motor_log("get of_node prize,ir_camera_motor fail\n");
        return -1;
    } else {
        ir_cut_ldo_en1_pin = of_get_named_gpio(node, "ir_cut_ldo_en1_pin", 0);
        if (ir_cut_ldo_en1_pin < 0){
            ir_camera_motor_log("get ir_cut_ldo_en1_pin fail %d\n", ir_cut_ldo_en1_pin);
            return -1;
        } else {
            gpio_request(ir_cut_ldo_en1_pin, "ir_cut_ldo_en1_pin");
            gpio_direction_output(ir_cut_ldo_en1_pin, 1);
            gpio_set_value(ir_cut_ldo_en1_pin, 0);
        }
        
        ir_cut_ldo_en2_pin = of_get_named_gpio(node, "ir_cut_ldo_en2_pin", 0);
        if (ir_cut_ldo_en2_pin < 0){
            ir_camera_motor_log("get ir_cut_ldo_en2_pin fail %d\n", ir_cut_ldo_en2_pin);
            return -1;
        } else {
            gpio_request(ir_cut_ldo_en2_pin, "ir_cut_ldo_en2_pin");
            gpio_direction_output(ir_cut_ldo_en2_pin, 1);
            gpio_set_value(ir_cut_ldo_en2_pin, 0);
        }
        
        gpio_icf_en_pin = of_get_named_gpio(node, "gpio_icf_en_pin", 0);
        if (gpio_icf_en_pin < 0){
            ir_camera_motor_log("get gpio_icf_en_pin fail %d\n", gpio_icf_en_pin);
            return -1;
        } else {
            gpio_request(gpio_icf_en_pin, "gpio_icf_en_pin");
            gpio_direction_output(gpio_icf_en_pin, 1);
            gpio_set_value(gpio_icf_en_pin, 0);
        }
        
        gpio_icf_disable_pin = of_get_named_gpio(node, "gpio_icf_disable_pin", 0);
        if (gpio_icf_disable_pin < 0){
            ir_camera_motor_log("get gpio_icf_disable_pin fail %d\n", gpio_icf_disable_pin);
            return -1;
        } else {
            gpio_request(gpio_icf_disable_pin, "gpio_icf_disable_pin");
            gpio_direction_output(gpio_icf_disable_pin, 1);
            gpio_set_value(gpio_icf_disable_pin, 0);
        }
    }
	
	return 0;
}

int ir_camera_motor_node_create(void)
{
	ir_camera_motor_proc_entry = proc_mkdir("ir_camera_motor", NULL);
	if (ir_camera_motor_proc_entry == NULL) {
		ir_camera_motor_log("create_proc_entry failed!\n");
		return -1;
	}

	proc_create("ir_camera_motor", 0664, ir_camera_motor_proc_entry, &ir_camera_motor_fops);

	ir_camera_motor_log("create_proc_entry success\n");
	return 0;
}

static int ir_camera_motor_probe(struct platform_device *pdev)
{
    int ret;
    ir_camera_motor_log("%s\n", __func__);
    
    ret = ir_camera_motor_dts();
    if (ret != 0) {
        ir_camera_motor_log("ir_camera_motor_dts failed!\n");
        return -1;
    }
    
    ret = ir_camera_motor_node_create();
    if (ret != 0) {
        ir_camera_motor_log("ir_camera_motor_node_create failed!\n");
        return -1;
    }
	ir_camera_motor_log("%s OK\n", __func__);
    
    return 0;
}

static int ir_camera_motor_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_device ir_camera_motor_dev = {
	.name = "ir_camera_motor",
	.id = -1,
};

static struct platform_driver ir_camera_motor_driver = {
	.probe = ir_camera_motor_probe,
	.remove = ir_camera_motor_remove,
	.driver = {
		.name = "ir_camera_motor",
		.owner = THIS_MODULE,
	},
};

static int __init ir_camera_motor_init(void) {
	int ret;
    
	ir_camera_motor_log("%s\n", __func__);
    
    ret = platform_device_register(&ir_camera_motor_dev);
    if (ret) {
		ir_camera_motor_log("****[%s] Unable to register device (%d)\n", __func__, ret);
		return ret;
	}

	ret = platform_driver_register(&ir_camera_motor_driver);
	if (ret) {
		ir_camera_motor_log("****[%s] Unable to register driver (%d)\n", __func__, ret);
		return ret;
	}
	
	return 0;
}

static void __exit ir_camera_motor_exit(void) {
	ir_camera_motor_log("%s\n", __func__);
	platform_driver_unregister(&ir_camera_motor_driver);
}

module_init(ir_camera_motor_init);
module_exit(ir_camera_motor_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("<chenjiaxi@szprize.com >");
MODULE_DESCRIPTION("IR Camera Motor Driver");

