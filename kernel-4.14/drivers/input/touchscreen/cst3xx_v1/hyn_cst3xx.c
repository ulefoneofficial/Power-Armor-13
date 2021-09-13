/* 
 * drivers/input/touchscreen/hyn_cst3xx.c
 *
 * hynitron TouchScreen driver. 
 *
 * Copyright (c) 2015  hynitron
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * VERSION      	DATE			AUTHOR
 *  1.0		    2015-10-12		    Tim
 *
 * note: only support mulititouch
 */


#include <linux/string.h>
#include <linux/vmalloc.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/byteorder/generic.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/proc_fs.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>

#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>


#include "hyn_cst3xx_fw.h"

/**********************************
config macro define
*********************************/
//#define HYN_GESTURE
// #define HYN_UPDATE_FIRMWARE_ONLINE
//#define CONFIG_TP_ESD_PROTECT
#define ANDROID_TOOL_SURPORT
//#define ICS_SLOT_REPORT
#define REPORT_XY_SWAP
#define SLEEP_CLEAR_POINT
//#define HIGH_SPEED_IIC_TRANSFER
#define TRANSACTION_LENGTH_LIMITED
//#define TPD_HAVE_BUTTON
//#define GTP_HAVE_TOUCH_KEY


#define GTP_DEBUG_FUNC_ON  1

#define GTP_DEBUG_FUNC()               do{\
                                         if(GTP_DEBUG_FUNC_ON)\
                                         printk("<<GTP-FUNC>> Func:%s@Line:%d\n",__func__,__LINE__);\
                                       }while(0)

//#define GTP_RST_PORT    0
//#define GTP_INT_PORT    1


#ifdef HYN_GESTURE
//static int hyn_gesture_flag = 0;
static int hyn_lcd_flag = 0;
//static struct input_dev *hyn_power_idev;
//static char hyn_gesture_c = 0;
static int tpd_halt=0;

#if 0
#define  HYN_TPD_GES_COUNT  14
static int tpd_keys_gesture[HYN_TPD_GES_COUNT] ={KEY_POWER, \
												TPD_GES_KEY_DOUBLECLICK,\
												TPD_GES_KEY_LEFT,\
												TPD_GES_KEY_RIGHT,\
												TPD_GES_KEY_UP,\
												TPD_GES_KEY_DOWN,\
												TPD_GES_KEY_O,\
												TPD_GES_KEY_W,  \
												TPD_GES_KEY_M,\
												TPD_GES_KEY_E,\
												TPD_GES_KEY_C,\
												TPD_GES_KEY_S,\
												TPD_GES_KEY_V,\
												TPD_GES_KEY_Z};
#endif

#endif 



#ifdef CONFIG_TP_ESD_PROTECT
#define SWITCH_ESD_OFF                  0
#define SWITCH_ESD_ON                   1
static struct workqueue_struct *cst3xx_esd_workqueue;
#endif

#ifdef ANDROID_TOOL_SURPORT
static int cst3xx_firmware_info(struct i2c_client * client);
static int cst3xx_update_firmware(struct i2c_client * client, const unsigned char *pdata);
static  unsigned short g_unnormal_mode = 0;
static unsigned short g_cst3xx_tx = 9;
static unsigned short g_cst3xx_rx = 15;
#endif

static unsigned char  report_flag = 0;
//static unsigned char  key_index = 0xFF;
static unsigned int   g_cst3xx_ic_version  = 0;
static unsigned int   g_cst3xx_ic_checksum = 0;

static struct workqueue_struct *cst3xx_wq;
static unsigned int  g_irq_pin = 924;
struct i2c_client *g_i2c_client = NULL;
struct input_dev  *g_input_dev = NULL;



#ifdef 	HYN_UPDATE_FIRMWARE_ONLINE
static int cst3xx_boot_update_fw(struct i2c_client   *client,  unsigned char *pdata);
static unsigned char *pcst3xx_update_firmware = (unsigned char *)cst3_fw ; //the updating firmware

#endif

#ifdef ICS_SLOT_REPORT
#include <linux/input/mt.h> // Protocol B, for input_mt_slot(), input_mt_init_slot()
#endif



struct cst3xx_ts_platform_data{
	int irq_gpio;
	u32 irq_gpio_flags;
	int reset_gpio;
	u32 reset_gpio_flags;
	const char *product1_id;
	const char *product2_id;
	char match_product_id;  //0 no match; 1 match product1-id; 2 match product2-id
	u32 x_max;
	u32 y_max;
	u32 x_min;
	u32 y_min;
	u32 panel_minx;
	u32 panel_miny;
	u32 panel_maxx;
	u32 panel_maxy;
//erobbing add for check if overturn axis of x and y
	bool x_overturn;
	bool y_overturn;
//end
	bool have_touch_key;
	bool i2c_pull_up;
	bool enable_power_off;
	bool enable_slot_report;
	bool enable_esd;

};

struct cst3xx_ts_data{
    struct cst3xx_ts_platform_data *pdata;
    spinlock_t irq_lock;
    struct i2c_client *client;
    struct input_dev  *input_dev;
    struct hrtimer timer;
    struct work_struct  work;
    s32 irq_is_disable;
    s32 use_irq;
    u16 abs_x_max;
    u16 abs_y_max;
    u8  max_touch_num;
    u8  int_trigger_type;
    u8  green_wake_mode;
    u8  enter_update;
 
    spinlock_t esd_lock;
    u8  esd_running;
    s32 clk_tick_cnt;

    u8  sensor_id;
    u16 ic_version;
	u16 fw_version;
	u16 config_version;
	u16 ic_config_version;
	u8	fw_updating;
    //struct ts_func_test_device ts_test_dev;

    bool power_on;
    struct mutex lock;
    // struct regulator *avdd;
    struct regulator *vcc_i2c;


#if defined(CONFIG_FB)
    struct notifier_block fb_notif;
    struct work_struct resume_work;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend early_suspend;
#endif
	struct pinctrl *ts_pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;

	dev_t			devt;//Erobbing add for distinguish open glove mode or not
	bool     in_glove_mode;

};
struct cst3xx_ts_data *g_ts=NULL;


#ifdef TPD_HAVE_BUTTON
#define TPD_KEY_COUNT	2
#define TPD_KEYS		{KEY_MENU,KEY_BACK}
/* {button_center_x, button_center_y, button_width, button_height*/
#define TPD_KEYS_DIM	{{180, 1360, 60, 60},{540, 1360, 60, 60}}
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif



#ifdef GTP_HAVE_TOUCH_KEY
#define TPD_KEY_COUNT	2
//#define TPD_KEYS		{KEY_MENU, KEY_HOMEPAGE, KEY_BACK, KEY_SEARCH}
#define TPD_KEYS		{KEY_MENU,KEY_BACK}
const u16 touch_key_array[] = TPD_KEYS;
//#define GTP_MAX_KEY_NUM ( sizeof( touch_key_array )/sizeof( touch_key_array[0] ) )
#endif


#define INPUT_DEV_PHYS  			"input/ts"
#define TPD_DRIVER_NAME			    "hynitron-ts"
#define TPD_MAX_FINGERS			    5
#define TPD_MAX_X				    1080
#define TPD_MAX_Y				    2160
#define CST3XX_I2C_ADDR				0x1A
#define I2C_BUS_NUMBER              1 //IIC bus num for mtk

#define printk(fmt,arg...)  printk(KERN_ERR" linc HYN:[LINE=%d]"fmt,__LINE__, ##arg)							
//#define printk(fmt,arg...)  printk(" linc HYN:[LINE=%d]"fmt,__LINE__, ##arg)



static int cst3xx_suspend(struct cst3xx_ts_data *ts);
static int cst3xx_resume(struct cst3xx_ts_data *ts);

#if defined(CONFIG_PM)
static int cst3xx_pm_suspend(struct device *dev);
static int cst3xx_pm_resume(struct device *dev);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void cst3xx_ts_early_suspend(struct early_suspend *h);
static void cst3xx_ts_late_resume(struct early_suspend *h);
#endif



void cst3xx_irq_enable(struct cst3xx_ts_data *ts);
void cst3xx_irq_disable(struct cst3xx_ts_data *ts);



#ifdef HYN_GESTURE
static u16 hyn_ges_wakeup_switch = 1;
static unsigned char hyn_gesture_c = 0;

#if 0
static char  hyn_ges_wakeup_flag = 0;
static unsigned char hyn_ges_cmd[48] = {0};
static unsigned int hyn_ges_num = 0;
static struct proc_dir_entry   *tp_proc_file = NULL;

static ssize_t proc_gesture_switch_write(struct file *flip,const char __user *buf, size_t count, loff_t *data)
{	
	if (copy_from_user(&hyn_ges_wakeup_flag, buf, count))
		return -EFAULT;
	printk(" linc -liuhx --%s---%c\n",__func__,hyn_ges_wakeup_flag);
	switch(hyn_ges_wakeup_flag)
	{
		case '0':
			hyn_ges_wakeup_switch = 0;
			break;
		default:
			hyn_ges_wakeup_switch = 1;
			break;
	}
	return count;	
}

static ssize_t proc_gesture_switch_read(struct file *flip, char __user *user_buf, size_t count, loff_t *ppos)
{	
	int len = 0;

	printk(" linc --%s--\n",__func__);
	len = simple_read_from_buffer(user_buf, count, ppos, &hyn_ges_wakeup_flag, 1);
	
	return len;
}

static ssize_t proc_gesture_data_write(struct file *flip,const char __user *buffer, size_t count, loff_t *data)
{

	int i =0;

	if(count > 48)
		count =48;
	
	if (copy_from_user(hyn_ges_cmd, buffer, count))
		return -EFAULT;
	
	hyn_ges_num = 0;
	//memcpy(hyn_ges_cmd, buffer, count);
	for(i = 0; i < count; i++){
		if(buffer[i] != 0){
			//--hyn_ges_cmd[hyn_ges_num ++] = buffer[i];	
			printk(" linc  hyn_ges_cmd[%d] :0x%x\n",i,hyn_ges_cmd[i]);
		}

	//DBG("buffer[%d] = %x\n",buffer[i]);
	hyn_ges_num = count;
	}
	
	return count;

}

static ssize_t proc_gesture_data_read(struct file *flip, char __user *user_buf, size_t count, loff_t *ppos)
{
	int len = 0;

	for (len = 0; len < hyn_ges_num; len++)
		printk(" linc buff[%d] = %c,", len, hyn_ges_cmd[len]);
	printk(" linc \n");
	return len;
}

static ssize_t proc_gesture_write(struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	printk(" linc --%s--\n",__func__);

	sscanf(buffer, "%x", (unsigned int *)&hyn_gesture_c);

	return count;

}

static int proc_gesture_show(struct seq_file *m, void *v)
{
	printk(" linc --%s--\n",__func__);

	seq_printf(m, "%#x\n", hyn_gesture_c);
	return 0;

}



static int proc_gesture_open(struct inode *inode, struct file *file)
{
        printk(" linc --%s--\n",__func__);
	return single_open(file, proc_gesture_show, NULL);
}

static const struct file_operations proc_gesture_fops = {
	.owner		= THIS_MODULE,
	.open		= proc_gesture_open,
	.read		= seq_read,
	.write		= proc_gesture_write, 
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations proc_gesture_switch_fops = {
	.write	=	proc_gesture_switch_write,
	.read	=	proc_gesture_switch_read,
};
static const struct file_operations proc_gesture_data_fops = {
	.write	=	proc_gesture_data_write,
	.read	=	proc_gesture_data_read,
};



static int gesture_proc_init(void)
{

   int rc;

   tp_proc_file = proc_create("gesture", 0777, NULL,&proc_gesture_fops);
   if (tp_proc_file == NULL)
   {
      rc = -ENOMEM;
      goto err_del_gesture;
   }

   tp_proc_file = proc_create_data("gesture_switch", 0777, NULL,&proc_gesture_switch_fops,NULL);
   if (tp_proc_file == NULL)
   {
      rc = -ENOMEM;
      goto err_del_gesture_switch;
   }

   tp_proc_file = proc_create_data("gesture_data", 0777, NULL,&proc_gesture_data_fops,NULL);
   if (tp_proc_file == NULL)
   {
      rc = -ENOMEM;
      goto err_del_gesture_data;
   }

   return 0;

err_del_gesture_data:
   remove_proc_entry("gesture_data", NULL);
err_del_gesture_switch:
   remove_proc_entry("gesture_switch", NULL);
err_del_gesture:
   remove_proc_entry("gesture", NULL);
   return rc;
}


#endif
#endif


#ifdef HIGH_SPEED_IIC_TRANSFER
static int cst3xx_i2c_read(struct i2c_client *client, unsigned char *buf, int len) 
{ 
	struct i2c_msg msg; 
	int ret = -1; 
	int retries = 0; 
	
	msg.flags |= I2C_M_RD; 
	msg.addr   = client->addr;
	msg.len    = len; 
	msg.buf    = buf;	

	while (retries < 2) { 
		ret = i2c_transfer(client->adapter, &msg, 1); 
		if(ret == 1)
			break; 
		retries++; 
	} 
	
	return ret; 
} 


/*******************************************************
Function:
    read data from register.
Input:
    buf: first two byte is register addr, then read data store into buf
    len: length of data that to read
Output:
    success: number of messages
    fail:	negative errno
*******************************************************/
static int cst3xx_i2c_read_register(struct i2c_client *client, unsigned char *buf, int len) 
{ 
	struct i2c_msg msgs[2]; 
	int ret = -1; 
	int retries = 0; 
	
	msgs[0].flags = client->flags & I2C_M_TEN;
	msgs[0].addr  = client->addr;  
	msgs[0].len   = 2;
	msgs[0].buf   = buf; 

	msgs[1].flags |= I2C_M_RD;
	msgs[1].addr   = client->addr; 
	msgs[1].len    = len; 
	msgs[1].buf    = buf;

	while (retries < 2) { 
		ret = i2c_transfer(client->adapter, msgs, 2); 
		if(ret == 2)
			break; 
		retries++; 
	} 
	
	return ret; 
} 

static int cst3xx_i2c_write(struct i2c_client *client, unsigned char *buf, int len) 
{ 
	struct i2c_msg msg; 
	int ret = -1; 
	int retries = 0;

	msg.flags = client->flags & I2C_M_TEN; 
	msg.addr  = client->addr; 
	msg.len   = len; 
	msg.buf   = buf;		  
	  
	while (retries < 2) { 
		ret = i2c_transfer(client->adapter, &msg, 1); 
		if(ret == 1)
			break; 
		retries++; 
	} 	
	
	return ret; 
}

#else
static int cst3xx_i2c_read(struct i2c_client *client, unsigned char *buf, int len) 
{ 
	int ret = -1; 
	int retries = 0; 

	while (retries < 2) { 
		ret = i2c_master_recv(client, buf, len); 
		if(ret<=0) 
		    retries++;
        else
            break; 
	} 
	
	return ret; 
} 

static int cst3xx_i2c_write(struct i2c_client *client, unsigned char *buf, int len) 
{ 
	int ret = -1; 
	int retries = 0; 

	while (retries < 2) { 
		ret = i2c_master_send(client, buf, len); 
		if(ret<=0) 
		    retries++;
        else
            break; 
	} 
	
	return ret; 
}

static int cst3xx_i2c_read_register(struct i2c_client *client, unsigned char *buf, int len) 
{ 
	int ret = -1; 
    
    ret = cst3xx_i2c_write(client, buf, 2);

    ret = cst3xx_i2c_read(client, buf, len);
	
    return ret; 
} 
#endif


static void cst3xx_reset_ic(unsigned int ms)
{
#if 1
	unsigned char buf[4];	
    buf[0] = 0xD1;	
	buf[1] = 0x0E;	 	
	cst3xx_i2c_write(g_i2c_client, buf, 2);
//#else	
	struct cst3xx_ts_data *ts=NULL;	
	ts=i2c_get_clientdata(g_i2c_client);
	if(!ts);{		
		printk("%s failed\n",__func__);	
	    return;
    }	    
	gpio_direction_output(ts->pdata->reset_gpio, 0);   
	msleep(50);  
	gpio_direction_output(ts->pdata->reset_gpio,1);
#endif
	mdelay(ms);
}
/*******************************************************
Function:
    test i2c communication
Input:
    client: i2c client
Output:

    success: big than 0
    fail:	negative
*******************************************************/
static int cst3xx_i2c_test(struct i2c_client *client)
{
	int retry = 0;
	int ret;
	unsigned char buf[4];

	buf[0] = 0xD1;
	buf[1] = 0x06;
	while (retry++ < 5) {
		ret = cst3xx_i2c_write(client, buf, 2);
		if (ret > 0)
			return ret;
		
		mdelay(2);		
	}

    if(retry==5) printk("linc cst3xx hyn I2C TEST error.ret:%d;\n", ret);
	
	return ret;
}

static s32 irq_is_disable = 0;

void cst3xx_irq_enable(struct cst3xx_ts_data *ts)
{
	unsigned long irqflags = 0;

	GTP_DEBUG_FUNC(); 
	
	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (irq_is_disable) {
		enable_irq(ts->client->irq);
		irq_is_disable = 0;
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

void cst3xx_irq_disable(struct cst3xx_ts_data *ts)
{
	unsigned long irqflags;
	
	GTP_DEBUG_FUNC(); 
	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (!irq_is_disable) {
		irq_is_disable = 1;
		disable_irq_nosync(ts->client->irq);
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}


#ifdef HYN_GESTURE
void hyn_key_report(int key_value)
{
#if 1
	input_report_key(g_input_dev, key_value, 1);
	input_sync(g_input_dev);	
	input_report_key(g_input_dev, key_value, 0);
	input_sync(g_input_dev);
#else
	input_report_key(g_input_dev, KEY_POWER, 1);
	input_sync(g_input_dev);
	input_report_key(g_input_dev, KEY_POWER, 0);
	input_sync(g_input_dev);
#endif
}

#if 0
static ssize_t hyn_sysfs_tpgesture_show(struct device *dev,struct device_attribute *attr, char *buf)
{
    ssize_t len=0;
    sprintf(&buf[len],"%s\n","tp gesture is on/off:");
    len += (strlen("tp gesture is on/off:")+1);
    if(hyn_ges_wakeup_switch == 1){
        sprintf(&buf[len],"%s\n","  on  ");
        len += (strlen("  on  ")+1);
    }else if(hyn_ges_wakeup_switch == 0){
        sprintf(&buf[len],"%s\n","  off  ");
        len += (strlen("  off  ")+1);
    }

    sprintf(&buf[len],"%s\n","tp gesture:");
    len += (strlen("tp gesture:")+1);
    sprintf(&buf[len],"%c\n",hyn_gesture_c);
    len += 2;	
    return len;
}
static ssize_t hyn_sysfs_tpgesturet_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
    char tmp_buf[16];

    if(copy_from_user(tmp_buf, buf, (count>16?16:count))){
        return -1;
    }
    if(buf[0] == '0'){
       	hyn_ges_wakeup_switch = 0;
        printk("linc cst3xx[HYN_GESTURE] hyn_sysfs_tpgesturet_store off.\n");
    }else if(buf[0] == '1'){
        hyn_ges_wakeup_switch = 1;
        printk("linc cst3xx[HYN_GESTURE] hyn_sysfs_tpgesturet_store on.\n");
    }

    return count;
}

static DEVICE_ATTR(tpgesture, S_IRUGO|S_IWUSR, hyn_sysfs_tpgesture_show, hyn_sysfs_tpgesturet_store);
static void hyn_request_power_idev(void)
{
    struct input_dev *idev;
    int rc = 0;
    idev = input_allocate_device();


    if(!idev){
        return;
    }
    hyn_power_idev = idev; 
    idev->name = "hyn_gesture";  
    idev->id.bustype = BUS_I2C;
    input_set_capability(idev,EV_KEY,KEY_POWER);
    input_set_capability(idev,EV_KEY,KEY_END);
	
    rc = input_register_device(idev);
    if(rc){
        input_free_device(idev);
        hyn_power_idev = NULL;
    }
}
static unsigned int hyn_gesture_init(void)
{
    int ret;
    struct kobject *hyn_debug_kobj;
	
    hyn_debug_kobj = kobject_create_and_add("hyn_gesture", NULL) ;
    if (hyn_debug_kobj == NULL)
    {
        printk("linc cst3xx%s: subsystem_register failed\n", __func__);
        return -ENOMEM;
    }
    ret = sysfs_create_file(hyn_debug_kobj, &dev_attr_tpgesture.attr);
    if (ret)
    {
        printk("linc cst3xx%s: sysfs_create_version_file failed\n", __func__);
        return ret;
    }
    hyn_request_power_idev();
    printk("linc cst3xx hyn_gesture_init success.\n");
    return 1;
}
#endif

#endif

static irqreturn_t cst3xx_ts_irq_handler(int irq, void *dev_id)
{
	struct cst3xx_ts_data *ts=NULL;
	GTP_DEBUG_FUNC(); 
	ts=dev_id;
	if(!ts)
	{
		return IRQ_NONE;
	}
	queue_work(cst3xx_wq, &ts->work);
    cst3xx_irq_disable(ts);
	return IRQ_HANDLED;
}

static void cst3xx_touch_down(struct input_dev *input_dev,s32 id,s32 x,s32 y,s32 w)
{
	s32 temp_w = (w>>2);
#ifdef ICS_SLOT_REPORT
	input_mt_slot(input_dev, id);
	input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 1);
	input_report_abs(input_dev, ABS_MT_TRACKING_ID, id);
	input_report_abs(input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
	input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, temp_w);
	input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, temp_w);
	input_report_abs(input_dev, ABS_MT_PRESSURE, temp_w);
#else    
	input_report_key(input_dev, BTN_TOUCH, 1);
	input_report_abs(input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
	input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, temp_w);
	input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, temp_w);
	input_report_abs(input_dev, ABS_MT_PRESSURE, temp_w);
	input_report_abs(input_dev, ABS_MT_TRACKING_ID, id);
	input_mt_sync(input_dev);
#endif
	#if 0	
	if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode()) 
      {   		
          tpd_button(x, y, 1);
	}
    #endif
}
static void cst3xx_touch_up(struct input_dev *input_dev, int id)
{
#ifdef ICS_SLOT_REPORT
    input_mt_slot(input_dev, id);
    input_report_abs(input_dev, ABS_MT_TRACKING_ID, -1);
    input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
#else
    input_report_key(input_dev, BTN_TOUCH, 0);
    input_mt_sync(input_dev);
#endif

#if 0
	if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode()) {   
	    tpd_button(0, 0, 0);  
	}
#endif

}

#ifdef ANDROID_TOOL_SURPORT   //debug tool support
#define CST3XX_PROC_DIR_NAME	"cst1xx_ts"
#define CST3XX_PROC_FILE_NAME	"cst1xx-update"
static struct proc_dir_entry *g_proc_dir, *g_update_file;
static int CMDIndex = 0;
#if 0
static struct file *cst3xx_open_fw_file(char *path)
{
	struct file * filp = NULL;
	int ret;
	
	filp = filp_open(path, O_RDONLY, 0);
	if (IS_ERR(filp)) {
        ret = PTR_ERR(filp);
        return NULL;
    }
    filp->f_op->llseek(filp, 0, 0);
	
    return filp;
}

static void cst3xx_close_fw_file(struct file * filp)
{
	 if(filp)
	 filp_close(filp,NULL);
}

static int cst3xx_read_fw_file(unsigned char *filename, unsigned char *pdata, int *plen)
{
	struct file *fp;
	int size;
	int length;
	int ret = -1;

	if((pdata == NULL) || (strlen(filename) == 0)) {
		printk("linc cst3xxfile name is null.\n");
		return ret;
	}
	fp = cst3xx_open_fw_file(filename);
	if(fp == NULL) {		
        printk("linc cst3xxOpen bin file faild.path:%s.\n", filename);
		goto clean;
	}
	
	length = fp->f_op->llseek(fp, 0, SEEK_END); 
	fp->f_op->llseek(fp, 0, 0);	
	size = fp->f_op->read(fp, pdata, length, &fp->f_pos);
	if(size == length) {
    	ret = 0;
    	*plen = length;
	} 	

clean:
	cst3xx_close_fw_file(fp);
	return ret;
}

#else 

static struct file *cst3xx_open_fw_file(char *path, mm_segment_t * old_fs_p)
{
	struct file * filp;
	int ret;
	
	*old_fs_p = get_fs();
	set_fs(KERNEL_DS);
	filp = filp_open(path, O_RDONLY, 0);
	if (IS_ERR(filp)) 
	{
        ret = PTR_ERR(filp);
        return NULL;
    }
    filp->f_op->llseek(filp, 0, 0);
	
    return filp;
}

static void cst3xx_close_fw_file(struct file * filp,mm_segment_t old_fs)
{
	set_fs(old_fs);
	if(filp)
	    filp_close(filp,NULL);
}

static int cst3xx_read_fw_file(unsigned char *filename, unsigned char *pdata, int *plen)
{
	struct file *fp;
	mm_segment_t old_fs;
	int size;
	int length;
	int ret = -1;

	if((pdata == NULL) || (strlen(filename) == 0)) 
		return ret;
		
	fp = cst3xx_open_fw_file(filename, &old_fs);
	if(fp == NULL) 
	{		
        printk("Open bin file faild.path:%s.\n", filename);
		goto clean;
	}
	
	length = fp->f_op->llseek(fp, 0, SEEK_END); 
	fp->f_op->llseek(fp, 0, 0);	
	size = fp->f_op->read(fp, pdata, length, &fp->f_pos);
	if(size == length) 
	{
    	ret = 0;
    	*plen = length;
	} 	
	else
	{
		printk("read bin file length fail****size:%d*******length:%d .\n", size,length);

	}

clean:
	cst3xx_close_fw_file(fp, old_fs);
	return ret;
}


#endif

static int cst3xx_apk_fw_dowmload(struct i2c_client *client,
		unsigned char *pdata, int length) 
{ 

#ifdef 	HYN_UPDATE_FIRMWARE_ONLINE
{	
		int ret=-1;	
		pcst3xx_update_firmware=(unsigned char *)pdata;
		ret = cst3xx_update_firmware(g_i2c_client, pdata);
		return ret;
}	
#endif

	return 0;
}

static ssize_t cst3xx_proc_read_foobar(struct file *page,char __user *user_buf, size_t count, loff_t *data)
{	
	unsigned char buf[512];
	int len = 0;	
	int ret,read_count,time_out;

	printk(" linc cst3xx is entering cst3xx_proc_read_foobar.\n");
	cst3xx_irq_disable(g_ts);
		
	if (CMDIndex == 0) {
		sprintf(buf,"Hynitron touchscreen driver 1.0\n");
		//strcpy(page,buf);	
		len = strlen(buf);
		ret = copy_to_user(user_buf,buf,len);
		
	}
	else if (CMDIndex == 1) {
		buf[0] = g_cst3xx_rx;
		buf[1] = g_cst3xx_tx;
		ret = copy_to_user(user_buf,buf,2);
    	len = 2;
	}
	if((CMDIndex == 2 )|| (CMDIndex == 3)||(CMDIndex == 4))
	{		
		unsigned short rx,tx;
		int data_len;
		
		rx = g_cst3xx_rx;
		tx = g_cst3xx_tx;
		data_len = rx*tx*2 + 4 + (tx+rx)*2 + rx + rx; //374


		buf[0] = 0xD1;
		buf[1] = 0x18;  
		ret = cst3xx_i2c_write(g_i2c_client, buf, 2);
		if(ret < 0)
		{				
			printk("Write command(0x8001) failed.error:%d.\n", ret);
			goto END;
		}	


		
		if(CMDIndex == 2)  //read diff
		{
			buf[0] = 0xD1;
			buf[1] = 0x0D;
		}
		else  if(CMDIndex == 3)        //rawdata
		{  
			buf[0] = 0xD1;
			buf[1] = 0x0A;
		}
		else if(CMDIndex == 4)          //factory test
		{  
			buf[0] = 0xD1;
			buf[1] = 0x19;
			data_len = rx*tx*4 +(4 + tx + rx)*2;
		}
				
		ret = i2c_master_send(g_i2c_client, buf, 2);  
		if(ret < 0) 
		{			
			printk("Write command raw/diff mode failed:%d.\n", ret);
			goto END;
		}

    	

		g_unnormal_mode = 1;
		mdelay(14);

		time_out=60;
		while(!gpio_get_value(g_irq_pin)){
	
			mdelay(1);
			time_out--;
			if(time_out==0){
				goto END;
			}
		};
		
 			

#ifdef TRANSACTION_LENGTH_LIMITED
		
	    buf[0] = 0x80;
		buf[1] = 0x01;		
		ret = cst3xx_i2c_write(g_i2c_client, buf, 2);
		if(ret < 0)
		{				
			printk("Write command(0x8001) failed.error:%d.\n", ret);
			goto END;
		}
				

		for(read_count=0;read_count<data_len;read_count+=6){

			ret = cst3xx_i2c_read(g_i2c_client, &buf[2+read_count], 6);
			if(ret < 0) 		
			{
				printk("Read raw/diff data failed.error:%d.\n", ret);
				goto END;
			}	
		}	

		
		buf[0] = 0xD1;
		buf[1] = 0x17;  
		ret = cst3xx_i2c_write(g_i2c_client, buf, 2);
		if(ret < 0)
		{				
			printk("Write command(0x8001) failed.error:%d.\n", ret);
			goto END;
		}	

#else


    	buf[0] = 0x80;
		buf[1] = 0x01;		
		ret = cst3xx_i2c_write(g_i2c_client, buf, 2);
		if(ret < 0)
		{				
			printk("Write command(0x8001) failed.error:%d.\n", ret);
			goto END;
		}	
		ret = cst3xx_i2c_read(g_i2c_client, &buf[2], data_len);
		if(ret < 0) 		
		{
			printk("Read raw/diff data failed.error:%d.\n", ret);
			goto END;
		}	


#endif
		
		
		mdelay(2);
		
		buf[0] = 0xD1;
		buf[1] = 0x08;		
		ret = cst3xx_i2c_write(g_i2c_client, buf, 2); 		
		if(ret < 0) 
		{				
			printk("Write command normal mode failed.error:%d.\n", ret);
			goto END;
		}	
		
		buf[0] = rx;
		buf[1] = tx;	
    	ret = copy_to_user(user_buf,buf,data_len + 2);
    	len = data_len + 2;

		mdelay(2);
	}	

END:	
	g_unnormal_mode = 0;	
	CMDIndex = 0;	
	cst3xx_irq_enable(g_ts);
	return len;
}

static ssize_t cst3xx_proc_write_foobar(struct file *file, const char __user *buffer,size_t count, loff_t *data)
{
    unsigned char cmd[128];
    unsigned char *pdata = NULL;
	int len;
	int ret;
    int length = 24*1024;

	if (count > 128) 
		len = 128;
	else 
		len = count;

   printk(" linc cst3xx is entering cst3xx_proc_write_foobar .\n");
    
	if (copy_from_user(cmd, buffer, len))  {
		printk("linc cst3xxcopy data from user space failed.\n");
		return -EFAULT;
	}
	
	 printk(" linc cmd:%d......%d.......len:%d\r\n", cmd[0], cmd[1], len);
	
	if (cmd[0] == 0) {
	    pdata = kzalloc(sizeof(char)*length, GFP_KERNEL);
	    if(pdata == NULL) {
	        printk("linc cst3xxzalloc GFP_KERNEL memory fail.\n");
	        return -ENOMEM;
	    }
		ret = cst3xx_read_fw_file(&cmd[1], pdata, &length);
	  	if(ret < 0) {
			if(pdata != NULL) {
				kfree(pdata);
				pdata = NULL;	
			}				
			return -EPERM;
	  	}
		
		ret = cst3xx_apk_fw_dowmload(g_i2c_client, pdata, length);
	  	if(ret < 0){
	        printk("linc cst3xxupdate firmware failed.\n");
			if(pdata != NULL) {
				kfree(pdata);
				pdata = NULL;	
			}	
	        return -EPERM;
		}
        mdelay(50);
		
		cst3xx_firmware_info(g_i2c_client);    
		
		if(pdata != NULL) {
			kfree(pdata);
			pdata = NULL;	
		}
	}
	else if (cmd[0] == 2) {					
		//cst3xx_touch_release();		
		CMDIndex = cmd[1];			
	}			
	else if (cmd[0] == 3) {				
		CMDIndex = 0;		
	}	
			
	return count;
}

static const struct file_operations proc_tool_debug_fops = {

	.owner		= THIS_MODULE,

	.read	    = cst3xx_proc_read_foobar,

	.write		= cst3xx_proc_write_foobar, 

	

};



static int  cst3xx_proc_fs_init(void)

{

	int ret;	

	g_proc_dir = proc_mkdir(CST3XX_PROC_DIR_NAME, NULL);

	if (g_proc_dir == NULL) {

		ret = -ENOMEM;

		goto out;

	}

   g_update_file = proc_create(CST3XX_PROC_FILE_NAME, 0777, g_proc_dir,&proc_tool_debug_fops);

   if (g_update_file == NULL)

   {

      ret = -ENOMEM;

      goto no_foo;

   }
	return 0;

no_foo:

	remove_proc_entry(CST3XX_PROC_FILE_NAME, g_proc_dir);

out:

	return ret;

}


#define PROC_TP_VERSION "driver/tp_version"
static char mtk_tp_version[128] = {0};

static int tp_info_version_read(struct seq_file *m, void *v)
{
    struct input_dev *input_dev =g_input_dev;
    struct i2c_client *client = g_i2c_client;
    unsigned int fwver = 0;

    mutex_lock(&input_dev->mutex);

	fwver=g_cst3xx_ic_version;
	
    if ((fwver == 0xFF) || (fwver == 0x00))
              printk("get tp fw version fail!!\n"); 
    else
      sprintf(mtk_tp_version,"0x%02x",fwver);

     seq_printf(m, "%s\n",mtk_tp_version);

     mutex_unlock(&input_dev->mutex);
	
   return 0;
};

static int proc_tp_version_open_cst3xx(struct inode *inode, struct file *file)
{
    return single_open(file, tp_info_version_read, NULL);
};

static  struct file_operations tp_version_proc_fops1 = {
    .owner = THIS_MODULE,
    .open  = proc_tp_version_open_cst3xx,
    .read  = seq_read,
};
////steven add end 20181024	 

#endif


#ifdef CONFIG_TP_ESD_PROTECT

static int esd_work_cycle = 200;
static struct delayed_work esd_check_work;
static int esd_running;
struct mutex esd_lock;
static void cst3xx_esd_check_func(struct work_struct *);


void cst3xx_init_esd_protect(void)
{
    esd_work_cycle = 2 * HZ;	/*HZ: clock ticks in 1 second generated by system*/
	printk(" linc Clock ticks for an esd cycle: %d", esd_work_cycle);
	INIT_DELAYED_WORK(&esd_check_work, cst3xx_esd_check_func);
	mutex_init(&esd_lock);


}
	
void cst3xx_esd_switch(s32 on)
{
	mutex_lock(&esd_lock);
	if (SWITCH_ESD_ON == on) {	/* switch on esd check */
		if (!esd_running) {
			esd_running = 1;
			printk(" linc Esd protector started!");
			queue_delayed_work(cst3xx_esd_workqueue, &esd_check_work, esd_work_cycle);
		}
	} else {		/* switch off esd check */
		if (esd_running) {
			esd_running = 0;
			printk(" linc Esd protector stopped!");
			cancel_delayed_work(&esd_check_work);
		}
	}
	mutex_unlock(&esd_lock);

}


static void cst3xx_esd_check_func(struct work_struct *work)
{
	
    int retry = 0;
	int ret;
	unsigned char buf[4];

	if (!esd_running) {
	printk(" linc Esd protector suspended!");
	return;
	}

	buf[0] = 0xD0;
	buf[1] = 0x4C;
	
	while(retry++ < 3) {
		ret = cst3xx_i2c_read_register(g_i2c_client, buf, 1);
		if (ret > 0) break;
		
		mdelay(2);		
	}

    if((retry>3) || ((buf[0]!=128)&&(buf[0]!=140)&&(buf[0]!=148))) {
		
		//cst2xx_chip_init();
		
		//cst2xx_check_code(cst2xx_i2c);

		cst3xx_reset_ic(10);
    }
	
	mutex_lock(&esd_lock);
	if (esd_running)
		queue_delayed_work(cst3xx_esd_workqueue, &esd_check_work, esd_work_cycle);
	else
		printk(" linc Esd protector suspended!");
	mutex_unlock(&esd_lock);
}

#endif



/*******************************************************
Function:
    get firmware version, ic type...
Input:
    client: i2c client
Output:
    success: 0
    fail:	-1
*******************************************************/
static int cst3xx_firmware_info(struct i2c_client * client)
{
	int ret;
	unsigned char buf[28];
//	unsigned short ic_type, project_id;

	
	buf[0] = 0xD1;
	buf[1] = 0x01;
	ret = cst3xx_i2c_write(client, buf, 2);
	if (ret < 0) return -1;
	
	mdelay(10);

	buf[0] = 0xD2;
	buf[1] = 0x08;
	ret = cst3xx_i2c_read_register(client, buf, 8);
	if (ret < 0) return -1;	

	g_cst3xx_ic_version = buf[3];
	g_cst3xx_ic_version <<= 8;
	g_cst3xx_ic_version |= buf[2];
	g_cst3xx_ic_version <<= 8;
	g_cst3xx_ic_version |= buf[1];
	g_cst3xx_ic_version <<= 8;
	g_cst3xx_ic_version |= buf[0];

	g_cst3xx_ic_checksum = buf[7];
	g_cst3xx_ic_checksum <<= 8;
	g_cst3xx_ic_checksum |= buf[6];
	g_cst3xx_ic_checksum <<= 8;
	g_cst3xx_ic_checksum |= buf[5];
	g_cst3xx_ic_checksum <<= 8;
	g_cst3xx_ic_checksum |= buf[4];	

	printk("linc cst3xx [cst3xx] the chip ic version:0x%x, checksum:0x%x\r\n",
		g_cst3xx_ic_version, g_cst3xx_ic_checksum);

	if(g_cst3xx_ic_version==0xA5A5A5A5)
	{
		printk("linc cst3xx [cst3xx] the chip ic don't have firmware. \n");
		return -1;
	}

    buf[0] = 0xD1;
	buf[1] = 0x09;
	ret = cst3xx_i2c_write(client, buf, 2);
	if (ret < 0) return -1;
    mdelay(5);
	
	
	return 0;
}


#ifdef 	HYN_UPDATE_FIRMWARE_ONLINE

/*******************************************************
Function:
    read checksum in bootloader mode
Input:
    client: i2c client
    strict: check checksum value
Output:
    success: 0
    fail:	-1
*******************************************************/

#define CST3XX_BIN_SIZE    (24*1024 + 24)

static int cst3xx_check_checksum(struct i2c_client * client)
{
	int ret;
	int i;
	unsigned int  checksum;
	unsigned int  bin_checksum;
	unsigned char buf[4];
	const unsigned char *pData;

	for(i=0; i<5; i++)
	{
		buf[0] = 0xA0;
		buf[1] = 0x00;
		ret = cst3xx_i2c_read_register(client, buf, 1);
		if(ret < 0)
		{
			mdelay(2);
			continue;
		}

		if(buf[0]!=0)
			break;
		else
		mdelay(2);
	}
    mdelay(2);


    if(buf[0]==0x01)
	{
		buf[0] = 0xA0;
		buf[1] = 0x08;
		ret = cst3xx_i2c_read_register(client, buf, 4);
		
		if(ret < 0)	return -1;
		
		// read chip checksum
		checksum = buf[0] + (buf[1]<<8) + (buf[2]<<16) + (buf[3]<<24);

        pData=(unsigned char  *)pcst3xx_update_firmware +24*1024+16;   //7*1024 +512
		bin_checksum = pData[0] + (pData[1]<<8) + (pData[2]<<16) + (pData[3]<<24);

        printk(" linc hyn the updated ic checksum is :0x%x. the updating firmware checksum is:0x%x------\n", checksum, bin_checksum);
    
        if(checksum!=bin_checksum)
		{
			printk("linc cst3xx hyn check sum error.\n");		
			return -1;
			
		}
		
	}
	else
	{
		printk("linc cst3xx hyn No checksum.\n");
		return -1;
	}	
	return 0;
}
static int cst3xx_into_program_mode(struct i2c_client * client)
{
	int ret;
	unsigned char buf[4];
	
	buf[0] = 0xA0;
	buf[1] = 0x01;	
	buf[2] = 0xAA;	//set cmd to enter program mode		
	ret = cst3xx_i2c_write(client, buf, 3);
	if (ret < 0)  return -1;

	mdelay(2);
	
	buf[0] = 0xA0;
	buf[1] = 0x02;	//check whether into program mode
	ret = cst3xx_i2c_read_register(client, buf, 1);
	if (ret < 0)  return -1;
	
	if (buf[0] != 0x55) return -1;
	
	return 0;
}

static int cst3xx_exit_program_mode(struct i2c_client * client)
{
	int ret;
	unsigned char buf[3];
	
	buf[0] = 0xA0;
	buf[1] = 0x06;
	buf[2] = 0xEE;
	ret = cst3xx_i2c_write(client, buf, 3);
	if (ret < 0)
		return -1;
	
	mdelay(10);	//wait for restart

	
	return 0;
}

static int cst3xx_erase_program_area(struct i2c_client * client)
{
	int ret;
	unsigned char buf[3];
	
	buf[0] = 0xA0;
	buf[1] = 0x02;	
	buf[2] = 0x00;		//set cmd to erase main area		
	ret = cst3xx_i2c_write(client, buf, 3);
	if (ret < 0) return -1;
	
	mdelay(5);
	
	buf[0] = 0xA0;
	buf[1] = 0x03;
	ret = cst3xx_i2c_read_register(client, buf, 1);
	if (ret < 0)  return -1;
	
	if (buf[0] != 0x55) return -1;

	return 0;
}

static int cst3xx_write_program_data(struct i2c_client * client,
		const unsigned char *pdata)
{
	int i, ret;
	unsigned char *i2c_buf;
	unsigned short eep_addr;
	int total_kbyte;
#ifdef TRANSACTION_LENGTH_LIMITED
	unsigned char temp_buf[8];
	unsigned short iic_addr;
	int  j;

#endif
	

	i2c_buf = kmalloc(sizeof(unsigned char)*(1024 + 2), GFP_KERNEL);
	if (i2c_buf == NULL) 
		return -1;
	
	//make sure fwbin len is N*1K
	//total_kbyte = len / 1024;
	total_kbyte = 24;
	for (i=0; i<total_kbyte; i++) {
		i2c_buf[0] = 0xA0;
		i2c_buf[1] = 0x14;
		eep_addr = i << 10;		//i * 1024
		i2c_buf[2] = eep_addr;
		i2c_buf[3] = eep_addr>>8;
		ret = cst3xx_i2c_write(client, i2c_buf, 4);
		if (ret < 0)
			goto error_out;




	
#ifdef TRANSACTION_LENGTH_LIMITED
		memcpy(i2c_buf, pdata + eep_addr, 1024);
		for(j=0; j<256; j++) {
			iic_addr = (j<<2);
    	temp_buf[0] = (iic_addr+0xA018)>>8;
    	temp_buf[1] = (iic_addr+0xA018)&0xFF;
		temp_buf[2] = i2c_buf[iic_addr+0];
		temp_buf[3] = i2c_buf[iic_addr+1];
		temp_buf[4] = i2c_buf[iic_addr+2];
		temp_buf[5] = i2c_buf[iic_addr+3];
    	ret = cst3xx_i2c_write(client, temp_buf, 6);
    		if (ret < 0)
    			goto error_out;		
		}
#else
		
		i2c_buf[0] = 0xA0;
		i2c_buf[1] = 0x18;
		memcpy(i2c_buf + 2, pdata + eep_addr, 1024);
		ret = cst3xx_i2c_write(client, i2c_buf, 1026);
		if (ret < 0)
			goto error_out;
#endif
		
		i2c_buf[0] = 0xA0;
		i2c_buf[1] = 0x04;
		i2c_buf[2] = 0xEE;
		ret = cst3xx_i2c_write(client, i2c_buf, 3);
		if (ret < 0)
			goto error_out;
		
		mdelay(60);	
		
		i2c_buf[0] = 0xA0;
		i2c_buf[1] = 0x05;
		ret = cst3xx_i2c_read_register(client, i2c_buf, 1);
		if (ret < 0)
			goto error_out;
		
		if (i2c_buf[0] != 0x55)
			goto error_out;

	}
	
	i2c_buf[0] = 0xA0;
	i2c_buf[1] = 0x03;
	i2c_buf[2] = 0x00;
	ret = cst3xx_i2c_write(client, i2c_buf, 3);
	if (ret < 0)
		goto error_out;
	
	mdelay(8);	
	
	if (i2c_buf != NULL) {
		kfree(i2c_buf);
		i2c_buf = NULL;
	}

	return 0;
	
error_out:
	if (i2c_buf != NULL) {
		kfree(i2c_buf);
		i2c_buf = NULL;
	}
	return -1;
}

static int cst3xx_update_firmware(struct i2c_client * client, const unsigned char *pdata)
{
	int ret;
	int retry = 0;
	int retry_time = 0;
	
	printk("linc cst3xx----------upgrade cst3xx begain------------\n");
	
START_FLOW:	
	
	cst3xx_reset_ic(5+retry_time);

	ret = cst3xx_into_program_mode(client);
	if (ret < 0) {
		printk("linc cst3xx[cst3xx]into program mode failed.\n");
		retry_time++;
		goto err_out;
	}

	ret = cst3xx_erase_program_area(client);
	if (ret) {
		printk("linc cst3xx[cst3xx]erase main area failed.\n");
		goto err_out;
	}

	ret = cst3xx_write_program_data(client, pdata);
	if (ret < 0) {
		printk("linc cst3xx[cst3xx]write program data into cstxxx failed.\n");
		goto err_out;
	}

    ret =cst3xx_check_checksum(client);
	if (ret < 0) {
		printk("linc cst3xx[cst3xx] after write program cst3xx_check_checksum failed.\n");
		goto err_out;
	}

	ret = cst3xx_exit_program_mode(client);
	if (ret < 0) {
		printk("linc cst3xx[cst3xx]exit program mode failed.\n");
		goto err_out;
	}

	cst3xx_reset_ic(10);
	
	printk("linc cst3xx hyn----------cst3xx_update_firmware  end------------\n");
	
	return 0;
	
err_out:
	if (retry < 25) {
		retry++;
		goto START_FLOW;
	} 
	else {	
		return -1;
	}
}

static int cst3xx_update_judge( unsigned char *pdata, int strict)
{
	unsigned short ic_type, project_id;
	unsigned int fw_checksum, fw_version;
	const unsigned int *p;
	int i;
	unsigned char *pBuf;
		
	fw_checksum = 0x55;
	p = (const unsigned int *)pdata;
	for (i=0; i<(CST3XX_BIN_SIZE-4); i+=4) {
		fw_checksum += (*p);
		p++;
	}
	
	if (fw_checksum != (*p)) {
		printk("linc cst3xx[cst3xx]calculated checksum error:0x%x not equal 0x%x.\n", fw_checksum, *p);
		return -1;	//bad fw, so do not update
	}
	
	pBuf = &pdata[CST3XX_BIN_SIZE-16];
	
	project_id = pBuf[1];
	project_id <<= 8;
	project_id |= pBuf[0];

	ic_type = pBuf[3];
	ic_type <<= 8;
	ic_type |= pBuf[2];

	fw_version = pBuf[7];
	fw_version <<= 8;
	fw_version |= pBuf[6];
	fw_version <<= 8;
	fw_version |= pBuf[5];
	fw_version <<= 8;
	fw_version |= pBuf[4];

	fw_checksum = pBuf[11];
	fw_checksum <<= 8;
	fw_checksum |= pBuf[10];
	fw_checksum <<= 8;
	fw_checksum |= pBuf[9];
	fw_checksum <<= 8;
	fw_checksum |= pBuf[8];	
	
	printk("linc cst3xx[cst3xx]the updating firmware:project_id:0x%04x,ic type:0x%04x,version:0x%x,checksum:0x%x\n",
			project_id, ic_type, fw_version, fw_checksum);

	if (strict > 0) {
		
		printk("linc cst3xx[cst3xx]the updating firmware:fw version(0x%x), teh chip firmware:ic version(0x%x).\n",
				fw_version, g_cst3xx_ic_version);
		
		if (g_cst3xx_ic_version > fw_version) {
			return -1;	
		}
		
	}	
	
	return 0;
}

static int cst3xx_boot_update_fw(struct i2c_client   *client,  unsigned char *pdata)
{
	int ret;
	int retry = 0;
	int flag = 0;

	while (retry++ < 3) {
		ret = cst3xx_firmware_info(client);
		if (ret == 0) {
			flag = 1;
			break;
		}
	}
	if (flag == 1) {
		ret = cst3xx_update_judge(pdata, 1);
		if (ret < 0) {
			printk("linc cst3xx[cst3xx] no need to update firmware.\n");
			return 0;
		}
	}
	
	ret = cst3xx_update_firmware(client, pdata);
	if (ret < 0){
		printk("linc cst3xx [cst3xx] update firmware failed.\n");
		return -1;
	}

    mdelay(50);

	ret = cst3xx_firmware_info(client);
	if (ret < 0) {
		printk("linc cst3xx [cst3xx] after update read version and checksum fail.\n");
		return -1;
	}

	

	return 0;
}


static int hyn_boot_update_fw(struct i2c_client * client)
{
	unsigned char *ptr_fw;
	int ret;
	ptr_fw = pcst3xx_update_firmware;
	
	ret = cst3xx_boot_update_fw(client, ptr_fw);
    return ret;
	
}


#endif


static void cst3xx_touch_report(struct work_struct *work)
{
	unsigned char buf[30];
	unsigned char i2c_buf[8];
	unsigned char key_status, key_id = 0, finger_id, sw;
	unsigned int  input_x = 0; 
	unsigned int  input_y = 0; 
	unsigned int  input_w = 0;
    unsigned char cnt_up, cnt_down;
	int   i, ret, idx; 
	int cnt, i2c_len;

	struct cst3xx_ts_data *ts=NULL;
#ifdef TRANSACTION_LENGTH_LIMITED
	int  len_1, len_2;
#endif
	ts=container_of(work,struct cst3xx_ts_data,work);	

	GTP_DEBUG_FUNC(); 

#ifdef HYN_GESTURE
    if((hyn_ges_wakeup_switch == 1)&&(tpd_halt == 1)){
        int tmp_c;
		int err;
      // unsigned char hyn_gesture_tmp = 0;	
        buf[0] = 0xD0;
		buf[1] = 0x4C;
		err = cst3xx_i2c_read_register(g_i2c_client, buf, 1);
        if(err < 0)
        {
        	printk("linc cst3xxiic read gesture flag failed.\n");
        	goto OUT_PROCESS;
        }
		tmp_c=buf[0]&0x7F;

		printk(" linc [HYN_GESTURE] tmp_c =%d \n",tmp_c);
        if(1)
        {
           			
			if(0 == hyn_lcd_flag){
				
                #if 1

				switch(tmp_c){
	                case 0x20:    //double
						 hyn_key_report(KEY_I);
						 hyn_gesture_c = (char)'*';
						 break;
	                case 0x3:     //left
	                     hyn_key_report(KEY_F);
						 hyn_gesture_c = (char)0xa1fb;
						 break;
	                case 0x1:     //right
	                     hyn_key_report(KEY_R);
						 hyn_gesture_c = (char)0xa1fa;
						 break;
	                case 0x2:     //up
	                     hyn_key_report(KEY_K);
						 hyn_gesture_c = (char)0xa1fc;
						 break;
					case 0x4:     //down
						 hyn_key_report(KEY_L);
						 hyn_gesture_c = (char)0xa1fd;
						 break;
	                case 0x5:     //O
	                     hyn_key_report(KEY_O);
						 hyn_gesture_c = (char)'O';
						 break;
	                case 0x0A:    //W
	                     hyn_key_report(KEY_W);
						 hyn_gesture_c = (char)'W';
						 break;
	                case 0x8: //M
	                case 0x09:
					case 0x0f:
			  	    case 0x10:
					case 0x15:
						 hyn_key_report(KEY_M);
						 hyn_gesture_c = (char)'M';
						 break;
	                case 0x07:            //E
	                     hyn_key_report(KEY_E);
						 hyn_gesture_c = (char)'E';
						 break;
	                case 0x6:       
	                case 0x0e:      //C
	                     hyn_key_report(KEY_C); 
						 hyn_gesture_c = (char)'C';
						 break;
	                case 0x0C:            //S
	                     hyn_key_report(KEY_S);
						 hyn_gesture_c = (char)'S';
						 break;
	                case 0x0B:            //V
	                     hyn_key_report(KEY_V);
						 hyn_gesture_c = (char)'V';
						 break;
	                case 0x0D:            //Z
	                     hyn_key_report(KEY_Z);
						 hyn_gesture_c = (char)'Z';
				         break;
	                default:
	                     break;
					}
                #else
					switch(tmp_c){
	                case 0x20:    //double					
						 hyn_gesture_tmp = (char)'*';
						 break;
	                case 0x3:     //left	                     
						 hyn_gesture_tmp = 0xBB;
						 break;
	                case 0x1:     //right
						 hyn_gesture_tmp = 0xAA;
						 break;
	                case 0x2:     //up
						 hyn_gesture_tmp = 0xBA;
						 break;
		      		case 0x4:     //down
						 hyn_gesture_tmp =0xAB;
						 break;
	                case 0x5:     //O
	                case 0x14:
					case 0x16:
							
						 hyn_gesture_tmp = (char)'o';
						 break;
	                case 0x0A:    //W
						 hyn_gesture_tmp = (char)'w';
						 break;
	                case 0x8: //M
	                case 0x09:
					case 0x0f:
			  	    case 0x10:
					case 0x15:
						 hyn_gesture_tmp = (char)'m';
						 break;
	                case 0x07:            //E
	                case 0x11:
		      case 0x13:
						 hyn_gesture_tmp = 'e';
						 break;
	                case 0x6:       //C
	                case 0x0e:					
						 hyn_gesture_tmp = (char)'c';
						 break;
	                case 0x0C:            //S
	                case 0x12:	
						 hyn_gesture_tmp = (char)'s';
						 break;
	                case 0x0B:            //V
						 hyn_gesture_tmp = (char)'v';
						 break;
	                case 0x0D:            //Z
						 hyn_gesture_tmp = (char)'z';
				         break;
	                default:
	                     break;
						}
				
				
				printk(" linc [GSL_GESTURE] input report KEY_POWER.hyn_gesture_c = %c ,%d\n",hyn_gesture_c,hyn_gesture_c);
				for(i = 0; i < hyn_ges_num; i++){
						if(hyn_gesture_tmp == hyn_ges_cmd[i]){
			 				hyn_gesture_c = hyn_gesture_tmp;
							input_report_key(hyn_power_idev,KEY_POWER,1);
							input_sync(hyn_power_idev);
							input_report_key(hyn_power_idev,KEY_POWER,0);
							input_sync(hyn_power_idev);    
							}
					}
                #endif
			//hyn_lcd_flag = 1;		

			}
            goto i2c_lock; 
        }
        
    }
#endif


    key_status = 0;


	buf[0] = 0xD0;
	buf[1] = 0x00;
	ret = cst3xx_i2c_read_register(g_i2c_client, buf, 7);
	if(ret < 0) {
		printk(" linc iic read touch point data failed.\n");
		goto OUT_PROCESS;
	}
		
	if(buf[6] != 0xAB) {
		//printk(" linc data is not valid..\r\n");
		goto OUT_PROCESS;
	}

	if(buf[5] == 0x80) {
		key_status = buf[0];
		key_id = buf[1];		
		goto KEY_PROCESS;
	} 
	
	cnt = buf[5] & 0x7F;
	if(cnt > TPD_MAX_FINGERS) goto OUT_PROCESS;
	else if(cnt==0)     goto CLR_POINT;
	
	if(cnt == 0x01) {
		goto FINGER_PROCESS;
	} 
	else {
		#ifdef TRANSACTION_LENGTH_LIMITED
		if((buf[5]&0x80) == 0x80) //key
		{
			i2c_len = (cnt - 1)*5 + 3;
			len_1   = i2c_len;
			for(idx=0; idx<i2c_len; idx+=6) {
			    i2c_buf[0] = 0xD0;
				i2c_buf[1] = 0x07+idx;
				
				if(len_1>=6) {
					len_2  = 6;
					len_1 -= 6;
				}
				else {
					len_2 = len_1;
					len_1 = 0;
				}
				
    			ret = cst3xx_i2c_read_register(g_i2c_client, i2c_buf, len_2);
    			if(ret < 0) goto OUT_PROCESS;

				for(i=0; i<len_2; i++) {
                   buf[5+idx+i] = i2c_buf[i];
				}
			}
			
			i2c_len   += 5;
			key_status = buf[i2c_len - 3];
			key_id     = buf[i2c_len - 2];
		} 
		else {			
			i2c_len = (cnt - 1)*5 + 1;
			len_1   = i2c_len;
			
			for(idx=0; idx<i2c_len; idx+=6) {
			    i2c_buf[0] = 0xD0;
				i2c_buf[1] = 0x07+idx;
				
				if(len_1>=6) {
					len_2  = 6;
					len_1 -= 6;
				}
				else {
					len_2 = len_1;
					len_1 = 0;
				}
				
    			ret = cst3xx_i2c_read_register(g_i2c_client, i2c_buf, len_2);
    			if (ret < 0) goto OUT_PROCESS;

				for(i=0; i<len_2; i++) {
                   buf[5+idx+i] = i2c_buf[i];
				}
			}			
			i2c_len += 5;
		}
		#else
		if ((buf[5]&0x80) == 0x80) {
			buf[5] = 0xD0;
			buf[6] = 0x07;
			i2c_len = (cnt - 1)*5 + 3;
			ret = cst3xx_i2c_read_register(g_i2c_client, &buf[5], i2c_len);
			if (ret < 0)
				goto OUT_PROCESS;
			i2c_len += 5;
			key_status = buf[i2c_len - 3];
			key_id = buf[i2c_len - 2];
		} 
		else {			
			buf[5] = 0xD0;
			buf[6] = 0x07;			
			i2c_len = (cnt - 1)*5 + 1;
			ret = cst3xx_i2c_read_register(g_i2c_client, &buf[5], i2c_len);
			if (ret < 0)
				goto OUT_PROCESS;
			i2c_len += 5;
		}
		#endif

		if (buf[i2c_len - 1] != 0xAB) {
			goto OUT_PROCESS;
		}
	}	

    //both key and point
	if((cnt>0)&&(key_status&0x80))  {
        if(report_flag==0xA5) goto KEY_PROCESS; 
	}
	
FINGER_PROCESS:

	i2c_buf[0] = 0xD0;
	i2c_buf[1] = 0x00;
	i2c_buf[2] = 0xAB;
	ret = cst3xx_i2c_write(g_i2c_client, i2c_buf, 3);
	if(ret < 0) {
		printk("linc cst3xx hyn send read touch info ending failed.\r\n"); 
		cst3xx_reset_ic(20);
	}
	
	idx = 0;
    cnt_up = 0;
    cnt_down = 0;
	for (i = 0; i < cnt; i++) {
		
		input_x = (unsigned int)((buf[idx + 1] << 4) | ((buf[idx + 3] >> 4) & 0x0F));
		input_y = (unsigned int)((buf[idx + 2] << 4) | (buf[idx + 3] & 0x0F));	
		input_w = (unsigned int)(buf[idx + 4]);
		sw = (buf[idx] & 0x0F) >> 1;
		finger_id = (buf[idx] >> 4) & 0x0F;

        #if 0        	
		input_x = TPD_MAX_X - input_x;
		input_y = TPD_MAX_Y - input_y;

		#endif

	   
        //printk("linc cst3xxPoint x:%d, y:%d, id:%d, sw:%d. \n", input_x, input_y, finger_id, sw);

		if (sw == 0x03) {
			cst3xx_touch_down(ts->input_dev, finger_id, input_x, input_y, input_w);
            cnt_down++;
        }
		else {
            cnt_up++;
            #ifdef ICS_SLOT_REPORT
			cst3xx_touch_up(ts->input_dev, finger_id);
            #endif
        }
		idx += 5;
	}
    
    #ifndef ICS_SLOT_REPORT
    if((cnt_up>0) && (cnt_down==0))
        cst3xx_touch_up(ts->input_dev, 0);
    #endif

	if(cnt_down==0)  report_flag = 0;
	else report_flag = 0xCA;

    input_sync(ts->input_dev);
	goto END;

KEY_PROCESS:

	i2c_buf[0] = 0xD0;
	i2c_buf[1] = 0x00;
	i2c_buf[2] = 0xAB;
	ret = cst3xx_i2c_write(g_i2c_client, i2c_buf, 3);
	if (ret < 0) {
		printk("linc cst3xx hyn send read touch info ending failed.\r\n"); 
		cst3xx_reset_ic(20);
	}
	
    #ifdef GTP_HAVE_TOUCH_KEY
	if(key_status&0x80) {
		i = (key_id>>4)-1;
        if((key_status&0x7F)==0x03) {
			if((i==key_index)||(key_index==0xFF)) {
        		//cst3xx_touch_down(input_dev, 0, tpd_keys_dim_local[i][0], tpd_keys_dim_local[i][1], 50);
                input_report_key(ts->input_dev, touch_key_array[i], 1);
    			report_flag = 0xA5;
				key_index   = i;
			}
			else {
                input_report_key(ts->input_dev, touch_key_array[key_index], 0);
				key_index = 0xFF;
			}
		}
    	else {
			input_report_key(ts->input_dev, touch_key_array[i], 0);
            cst3xx_touch_up(ts->input_dev, 0);
			report_flag = 0;	
			key_index = 0xFF;
    	}
	}
	#endif	

    #ifdef TPD_HAVE_BUTTON
	if(key_status&0x80) {
		i = (key_id>>4)-1;
        if((key_status&0x7F)==0x03) {
			if((i==key_index)||(key_index==0xFF)) {
        		cst3xx_touch_down(ts->input_dev, 0, tpd_keys_dim_local[i][0], tpd_keys_dim_local[i][1], 50);
    			report_flag = 0xA5;
				key_index   = i;
			}
			else {
				
				key_index = 0xFF;
			}
		}
    	else {
            cst3xx_touch_up(ts->input_dev, 0);
			report_flag = 0;	
			key_index = 0xFF;
    	}
	}
    

	#endif	
	
	input_sync(ts->input_dev);
    goto END;

CLR_POINT:
#ifdef SLEEP_CLEAR_POINT
	#ifdef ICS_SLOT_REPORT
		for(i=0; i<=10; i++) {	
			input_mt_slot(ts->input_dev, i);
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, -1);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
		}
	#else
	    input_report_key(ts->input_dev, BTN_TOUCH, 0);
		input_mt_sync(ts->input_dev);
	#endif
		input_sync(ts->input_dev);	
#endif		
		
OUT_PROCESS:
	buf[0] = 0xD0;
	buf[1] = 0x00;
	buf[2] = 0xAB;
	ret = cst3xx_i2c_write(g_i2c_client, buf, 3);
	if (ret < 0) {
		printk(" linc send read touch info ending failed.\n"); 
		cst3xx_reset_ic(20);
	}

#ifdef HYN_GESTURE	
i2c_lock:
     key_status = 0;
	;
#endif	


	
END:	

    cst3xx_irq_enable(ts);
	return;
}

#if   defined(CONFIG_FB)
static void cst3xx_ts_resume_work(struct work_struct *work)
{
    struct cst3xx_ts_data *ts_data = container_of(work,struct cst3xx_ts_data, resume_work);

    cst3xx_resume(ts_data);
}
/* frame buffer notifier block control the suspend/resume procedure */
struct notifier_block cst3xx_fb_notifier;
static int gtp_fb_notifier_callback(struct notifier_block *noti, unsigned long event, void *data)
{
	struct fb_event *ev_data = data;
	int *blank;
	struct cst3xx_ts_data *ts=NULL;
	ts=i2c_get_clientdata(g_i2c_client);
	if(!ts)
	{
		printk("cst3xx data pointor is null,no the device\n");
		return -ENODEV;
	}

    #ifndef FB_EARLY_EVENT_BLANK
        #error Need add FB_EARLY_EVENT_BLANK to fbmem.c
    #endif


	if (ev_data && ev_data->data && event == FB_EVENT_BLANK) {
		blank = ev_data->data;
        if (*blank == FB_BLANK_UNBLANK) {
			printk("Resume by fb notifier.");
			if(ts)
			{
				cst3xx_resume(ts);
			}
			else
			{
				printk("cst3xx ts pointer is null, resume fail\n");
			}
        }
    }

	if (ev_data && ev_data->data && event == FB_EVENT_BLANK) {
		blank = ev_data->data;
        if (*blank == FB_BLANK_POWERDOWN) {
			printk("Suspend by fb notifier.");
			if(ts)
			{
				cst3xx_suspend(ts);
			}
			else
			{
				printk("cst3xx ts pointer is null, suspend fail\n");
			}
		}
	}

	return 0;
}
#endif


#if defined(CONFIG_PM)

static int cst3xx_pm_suspend(struct device *dev)
{
	struct cst3xx_ts_data *ts=NULL;
	ts=i2c_get_clientdata(g_i2c_client);
	if(!ts);
	{
		printk("%s failed\n",__func__);
		return -1;
	}
    return cst3xx_suspend(ts);
}
static int cst3xx_pm_resume(struct device *dev)
{
	struct cst3xx_ts_data *ts=NULL;
	ts=i2c_get_clientdata(g_i2c_client);
	if(!ts)
	{
		return -1;
	}
	return cst3xx_resume(ts);
}

/* bus control the suspend/resume procedure */
static const struct dev_pm_ops cst3xx_ts_pm_ops = {
	.suspend = cst3xx_pm_suspend,
	.resume = cst3xx_pm_resume,
};

#elif defined(CONFIG_HAS_EARLYSUSPEND)
/* earlysuspend module the suspend/resume procedure */
static void cst3xx_ts_early_suspend(struct early_suspend *h)
{
	struct cst3xx_ts_data *ts=NULL;
	ts=i2c_get_clientdata(g_i2c_client);
	if(!ts);
	{
		printk("%s failed\n",__func__);
		return;
	}
	cst3xx_suspend(ts);
}

static void cst3xx_ts_late_resume(struct early_suspend *h)
{
	struct cst3xx_ts_data *ts=NULL;
	ts=i2c_get_clientdata(g_i2c_client);
	if(!ts);
	{
		printk("%s failed\n",__func__);
		return;
	}

	cst3xx_resume(ts);
}
static struct early_suspend cst3xx_early_suspend = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1,
	.suspend = cst3xx_ts_early_suspend,
	.resume = cst3xx_ts_late_resume,
};
#endif


static s32 cst3xx_request_irq(struct cst3xx_ts_data *ts)
{
	s32 ret = -1;
	ret = request_irq(ts->client->irq, cst3xx_ts_irq_handler, IRQ_TYPE_EDGE_RISING, ts->client->name, ts);
	if (ret) {
		printk("Request IRQ failed!ERRNO:%d.", ret);
		gpio_direction_input(ts->pdata->irq_gpio);
		gpio_free(ts->pdata->irq_gpio);

		return -1;
	} else {
		cst3xx_irq_disable(ts);
		return 0;
	}
}
static s8 cst3xx_request_input_dev(struct cst3xx_ts_data *ts)
{
	s8 ret = -1;
//	u8 index = 0;

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		printk("Failed to allocate input device.");
		return -ENOMEM;
	}

	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
#ifdef ICS_SLOT_REPORT 
#if 1
    input_mt_init_slots(ts->input_dev, 16, INPUT_MT_DIRECT);
#else
    input_mt_init_slots(ts->input_dev, 16);
#endif
#else
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#endif
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);


#ifdef HYN_GESTURE
	input_set_capability(ts->input_dev, EV_KEY, KEY_GES_REGULAR);
    input_set_capability(ts->input_dev, EV_KEY, KEY_GES_CUSTOM);
#endif

	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE,    0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0,(5+1),0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, TPD_MAX_X,0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, TPD_MAX_Y,0, 0);

	ts->input_dev->name = TPD_DRIVER_NAME;
	ts->input_dev->phys = INPUT_DEV_PHYS;
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0xDEAD;
	ts->input_dev->id.product = 0xBEEF;
	ts->input_dev->id.version = 10427;

	ret = input_register_device(ts->input_dev);
	if (ret) {
		printk("Register %s input device failed", ts->input_dev->name);
		return -ENODEV;
	}

#if   defined(CONFIG_FB)
		INIT_WORK(&ts->resume_work,cst3xx_ts_resume_work);
		ts->fb_notif.notifier_call = gtp_fb_notifier_callback;
		fb_register_client(&ts->fb_notif);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
		ts->early_suspend.suspend=cst3xx_ts_early_suspend;
		ts->early_suspend.resume=cst3xx_ts_late_resume;
		register_early_suspend(&ts->early_suspend);
#endif

	return 0;
}



static s32 cst3xx_request_io_port(struct cst3xx_ts_data *ts)
{
	s32 ret = 0;

	if(!ts)
		return -ENODEV;
	
	ret = gpio_request(ts->pdata->irq_gpio, "HYN_INT_IRQ");
	if (ret < 0) {
		printk("Failed to request GPIO:%d, ERRNO:%d", ts->pdata->irq_gpio, ret);
		ret = -ENODEV;
		return ret;
	}
	ts->client->irq = gpio_to_irq(ts->pdata->irq_gpio);
	//g_irq_pin=ts->pdata->irq_gpio;

	ret = gpio_request(ts->pdata->reset_gpio, "HYN_RST_PORT");
	if (ret < 0) {
		printk("Failed to request GPIO:%d, ERRNO:%d", ts->pdata->reset_gpio, ret);
		ret = -ENODEV;
	}
	gpio_direction_input(ts->pdata->reset_gpio);
	if (ret < 0) {
		gpio_free(ts->pdata->reset_gpio);
		gpio_free(ts->pdata->irq_gpio);
	}

	return ret;
}


static int cst3xx_ts_get_dt_coords(struct device *dev, char *name,struct cst3xx_ts_platform_data *pdata)
{
	struct property *prop=NULL;
	struct device_node *np = dev->of_node;
	int rc;
	u32 coords[4];

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	rc = of_property_read_u32_array(np, name, coords,4);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read %s\n", name);
		return rc;
	}

	if (!strcmp(name, "hynitron,panel-coords")) {
		pdata->panel_minx = coords[0];
		pdata->panel_miny = coords[1];
		pdata->panel_maxx = coords[2];
		pdata->panel_maxy = coords[3];
	} else if (!strcmp(name, "hynitron,display-coords")) {
		pdata->x_min = coords[0];
		pdata->y_min = coords[1];
		pdata->x_max = coords[2];
		pdata->y_max = coords[3];
	} else {
		dev_err(dev, "unsupported property %s\n", name);
		return -EINVAL;
	}

	return 0;
}
static int cst3xx_parse_dt(struct device *dev,struct cst3xx_ts_platform_data *pdata)
{
	struct device_node *np;
	//struct property *prop;
	int ret = 0;
	//int i, read_cfg_num, tch_key_nu;
	//u32 temp_val,button_map[3];
	//u32 gesture_map[15];
	//char prop_name[24];
	

    if (!dev)
        return -ENODEV;
    
    np = dev->of_node;
	ret = cst3xx_ts_get_dt_coords(dev, "hynitron,panel-coords", pdata);
	if (ret && (ret != -EINVAL))
		return ret;
	ret = cst3xx_ts_get_dt_coords(dev, "hynitron,display-coords", pdata);
	if (ret)
		return ret;

	pdata->reset_gpio= of_get_named_gpio_flags(np, "hynitron,rst-gpio",0, &pdata->reset_gpio_flags);
	if (pdata->reset_gpio < 0)
	{
		printk("CST3XX Invalid GPIO, get reset-gpio failed\n");
		return pdata->reset_gpio;
	}
	pdata->irq_gpio = of_get_named_gpio_flags(np, "hynitron,irq-gpio",0, &pdata->irq_gpio_flags);
	if (pdata->irq_gpio < 0)
	{
		printk("CST3XX Invalid GPIO, get irs-gpio failed\n");
		return pdata->irq_gpio;
	}
    if (!gpio_is_valid(pdata->irq_gpio) || !gpio_is_valid(pdata->reset_gpio)) {
        printk("Invalid GPIO, irq-gpio:%d, rst-gpio:%d",
            pdata->irq_gpio, pdata->reset_gpio);
        return -EINVAL;
    }
    /*
	
//for reading touch key value from device tree
	pdata->have_touch_key=0;
	tch_key_nu=0;
	prop = of_find_property(np, "hynitron,button-map", NULL);
	if (prop) {
		tch_key_nu= prop->length / sizeof(temp_val);
		if (tch_key_nu > 3)
			return -EINVAL;
		pdata->button_map= devm_kzalloc(dev,sizeof(*pdata->button_map),GFP_KERNEL);
		if (!pdata->button_map)
			return -ENOMEM;
		pdata->button_map->map = devm_kzalloc(dev,sizeof(*pdata->button_map->map) *3, GFP_KERNEL);
		if (!pdata->button_map->map)
			return -ENOMEM;
		ret = of_property_read_u32_array(np,"hynitron,button-map", button_map,tch_key_nu);
		if (ret) {
			dev_err(dev, "Unable to read key codes\n");
			return ret;
		}
		for (i = 0; i < tch_key_nu; i++)
			pdata->button_map->map[i] = button_map[i];
		pdata->button_map->nbuttons = tch_key_nu;
		pdata->have_touch_key=1;
	}
	else
	{
		printk("GTP1x there is no touch key\n");
		//pdata->have_touch_key=0;
	}
//end
	prop = of_find_property(np, "hynitron,gesture-map", NULL);
	if (prop) {
		tch_key_nu = prop->length / sizeof(temp_val);
		if (tch_key_nu > 15)
			return -EINVAL;
		pdata->gesture_map= devm_kzalloc(dev,sizeof(*pdata->gesture_map),GFP_KERNEL);
		if (!pdata->gesture_map)
			return -ENOMEM;
		pdata->gesture_map->map = devm_kzalloc(dev,	sizeof(*pdata->gesture_map->map) *15, GFP_KERNEL);
		if (!pdata->gesture_map->map)
			return -ENOMEM;
		ret = of_property_read_u32_array(np,"hynitron,gesture-map", gesture_map,tch_key_nu);
		if (ret) {
			dev_err(dev, "Unable to read gesture codes\n");
			return ret;
		}
		for (i = 0; i < tch_key_nu; i++)
			pdata->gesture_map->map[i] = gesture_map[i];
		pdata->gesture_map->nbuttons = tch_key_nu;
	}

	prop = of_find_property(np, "hynitron,gesture-key", NULL);
	if (prop) {
		tch_key_nu = prop->length / sizeof(temp_val);
		if (tch_key_nu > 15)
			return -EINVAL;
		pdata->gesture_key= devm_kzalloc(dev,sizeof(*pdata->gesture_key),GFP_KERNEL);
		if (!pdata->gesture_key)
			return -ENOMEM;
		pdata->gesture_key->map = devm_kzalloc(dev,	sizeof(*pdata->gesture_key->map) *15, GFP_KERNEL);
		if (!pdata->gesture_key->map)
			return -ENOMEM;
		ret = of_property_read_u32_array(np,"hynitron,gesture-key", gesture_map,tch_key_nu);
		if (ret) {
			dev_err(dev, "Unable to read gesture codes\n");
			return ret;
		}
		for (i = 0; i < tch_key_nu; i++)
			pdata->gesture_key->map[i] = gesture_map[i];
		pdata->gesture_key->nbuttons = tch_key_nu;
	}
	dev_dbg(dev, "%d config data read from device tree.\n", read_cfg_num);
	*/
	
    return 0;

}

static int cst3xx_tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{	
	int ret;

	struct cst3xx_ts_platform_data *pdata;
    struct cst3xx_ts_data *ts;
	
#ifdef GTP_HAVE_TOUCH_KEY
    s32 idx = 0;
#endif
	
    printk(" linc hyn is entering tpd_i2c_probe. \n");

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,sizeof(struct cst3xx_ts_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev,"cst3xx Failed to allocate memory for pdata\n");
			return -ENOMEM;
		}

		ret = cst3xx_parse_dt(&client->dev,pdata);
		if (ret)
			return ret;
	} else {
		pdata = client->dev.platform_data;
	}
	if (!pdata) {
		dev_err(&client->dev, "cst3xx invalid pdata\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        printk("I2C check functionality failed.");
        return -ENODEV;
    }
    ts = kzalloc(sizeof(*ts), GFP_KERNEL);
    if (ts == NULL)
    {
        printk("Alloc GFP_KERNEL memory failed.");
        return -ENOMEM;
    }

	if(client->addr != 0x1A)
	{
		client->addr = 0x1A;
		printk("i2c_client_HYN->addr=%d.\n",client->addr);
	}

 	memset(ts, 0, sizeof(*ts));
	INIT_WORK(&ts->work, cst3xx_touch_report);
	ts->client = client;
    ts->pdata = pdata;
	i2c_set_clientdata(client,ts);

	spin_lock_init(&ts->irq_lock);//(&irq_lock);
	mutex_init(&ts->lock);
	g_i2c_client = client;
    
    /***********/
	gpio_direction_output(ts->pdata->reset_gpio,1);
    msleep(50);  
    /***********/

    ts->vcc_i2c = regulator_get(&ts->client->dev, "vcc_i2c");
    if (IS_ERR(ts->vcc_i2c)) {
        ret = PTR_ERR(ts->vcc_i2c);
        ts->vcc_i2c = NULL;
        dev_info(&ts->client->dev,"Regulator get failed vcc_i2c ret=%d\n", ret);
    } else {
        dev_info(&ts->client->dev,"Regulator get vcc_i2c success\n");
    }
    
    ret = regulator_set_voltage(ts->vcc_i2c, 3300000,3300000);
    if (ret) {
        regulator_put(ts->vcc_i2c);
        ts->vcc_i2c = NULL;
        dev_info(&ts->client->dev,"ktf Regulator set 2.8V failed vcc_i2c ret=%d\n", ret);
        return ret;
    } else {
        dev_info(&ts->client->dev,"ktf Regulator set 2.8V success\n");
    }

	ret = regulator_enable(ts->vcc_i2c);
	if (ret) {
		printk("GTP enable vcc_i2c failed\n");
		return ret;
	} else {
        printk("GTP enable vcc_i2c success\n");
    }
	
	mdelay(40);
	
	ret = cst3xx_i2c_test(client);
	if (ret < 0) {
		printk("linc cst3xx hyn i2c communication failed.\n");
		return -1;
	}
	
	mdelay(20);

	ret = cst3xx_request_io_port(ts);
	if (ret < 0) {
		printk("cst3xx request IO port failed.");
		return ret;
	}


	ret = cst3xx_request_input_dev(ts);
	if (ret < 0) {
		printk("cst3xxrequest input dev failed");
	}

	g_input_dev=ts->input_dev;

	ret = cst3xx_request_irq(ts);
	if (ret < 0) {
		printk("cst3xx works in polling mode.");
	} else {
		printk("cst3xx works in interrupt mode.");
	}


	cst3xx_irq_enable(ts);


#ifdef 	HYN_UPDATE_FIRMWARE_ONLINE
	ret = hyn_boot_update_fw(client);
	if(ret < 0){
		printk("linc cst3xx hyn_boot_update_fw failed.\n");
		return -1;
	}
#else
	ret = cst3xx_firmware_info(client);
	if(ret < 0){
		printk("linc cst3xx cst3xx_firmware_info failed.\n");
		return -1;
	}
#endif	
	
	
#ifdef ANDROID_TOOL_SURPORT
	ret = cst3xx_proc_fs_init();
	if(ret < 0) {
		printk("linc cst3xx hyn create cst3xx proc fs failed.\n");
	}

	memset(mtk_tp_version,0,128);
    proc_create(PROC_TP_VERSION, 0, NULL, &tp_version_proc_fops1);
#endif


#ifdef HYN_GESTURE
//	for(Hyn_key_set=0;Hyn_key_set<HYN_TPD_GES_COUNT;Hyn_key_set++)	
//	input_set_capability(g_input_dev, EV_KEY,  tpd_keys_gesture[Hyn_key_set]); 
	enable_irq_wake(client->irq);
#endif

		
#ifdef CONFIG_TP_ESD_PROTECT

	cst3xx_esd_workqueue = create_singlethread_workqueue("cst2xx_esd_workqueue");
	if (cst3xx_esd_workqueue == NULL)
		printk("linc cst3xxcreate cst2xx_esd_workqueue failed!");

#endif


#ifdef GTP_HAVE_TOUCH_KEY
    for (idx=0; idx<TPD_KEY_COUNT; idx++) {
        input_set_capability(g_input_dev, EV_KEY, touch_key_array[idx]);
    }
#endif	


#ifdef CONFIG_TP_ESD_PROTECT

	cst3xx_init_esd_protect();
	cst3xx_esd_switch(SWITCH_ESD_ON);

#endif

#ifdef HYN_GESTURE

#if 0

	retval = gesture_proc_init();
	if (retval)
	{
		printk("linc cst3xxCreate gesture_proc failed!\n.");
	}

    hyn_gesture_init();

#endif


input_set_capability(g_input_dev, EV_KEY, KEY_POWER);
input_set_capability(g_input_dev, EV_KEY, KEY_C);
input_set_capability(g_input_dev, EV_KEY, KEY_M);
input_set_capability(g_input_dev, EV_KEY, KEY_E);
input_set_capability(g_input_dev, EV_KEY, KEY_O);
input_set_capability(g_input_dev, EV_KEY, KEY_W);
input_set_capability(g_input_dev, EV_KEY, KEY_S);
input_set_capability(g_input_dev, EV_KEY, KEY_UP);
input_set_capability(g_input_dev, EV_KEY, KEY_LEFT);
input_set_capability(g_input_dev, EV_KEY, KEY_RIGHT);
input_set_capability(g_input_dev, EV_KEY, KEY_DOWN);
input_set_capability(g_input_dev, EV_KEY, KEY_U);


	
#endif

	cst3xx_reset_ic(13);

	g_ts=ts;

	printk(" linc hyn is endding tpd_i2c_probe .\n");

	return 0;
}

static int cst3xx_tpd_remove(struct i2c_client *client)
{	
	struct cst3xx_ts_data *ts;
	printk(" linc cst3xx removed.\n");	
	ts=i2c_get_clientdata(client);
#ifdef HYN_GESTURE
	disable_irq_wake(client->irq);
#endif  

#if defined(CONFIG_FB)
	fb_unregister_client(&ts->fb_notif);
#endif

	if (cst3xx_wq) {
		destroy_workqueue(cst3xx_wq);
		}	
	input_unregister_device(ts->input_dev);
	if (gpio_is_valid(ts->pdata->irq_gpio))
		gpio_free(ts->pdata->irq_gpio);
	if (gpio_is_valid(ts->pdata->reset_gpio))
		gpio_free(ts->pdata->reset_gpio);
	if (ts->client&& ts->client->irq)
		free_irq(ts->client->irq, ts->client);
	//Erobbing add for distinguish resistance TP or not
	return 0;
	}
/*
static int cst3xx_tpd_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, TPD_DEVICE_NAME);
	return 0;
}
*/




static const struct i2c_device_id cst3xx_tpd_id[] = {{TPD_DRIVER_NAME,0},{}};



//static unsigned short force[] = { 0, CST3XX_I2C_ADDR, I2C_CLIENT_END, I2C_CLIENT_END };
//static const unsigned short *const forces[] = { force, NULL };


/*
	static struct i2c_board_info __initdata cst3xx_i2c_tpd = { 
		I2C_BOARD_INFO(TPD_DEVICE_NAME, CST3XX_I2C_ADDR)
	};
*/



static void cst3xx_enter_sleep(struct i2c_client *client)
{
	int ret;
	int retry = 0;
	unsigned char buf[2];

    buf[0] = 0xD1;
	buf[1] = 0x05;
	while (retry++ < 5) {
		ret = cst3xx_i2c_write(client, buf, 2);
		if (ret > 0)
			return;
		mdelay(2);
	}
	
	return;
}



static int cst3xx_resume(struct cst3xx_ts_data *ts)
{	
#ifdef ICS_SLOT_REPORT
	int idx;
#endif	

	printk(" linc cst3xx wake up.\n");	
    gpio_direction_output(ts->pdata->reset_gpio,1);
    msleep(50);  

#ifdef SLEEP_CLEAR_POINT
#ifdef ICS_SLOT_REPORT
	for(idx=0; idx<=10; idx++) {	
		input_mt_slot(g_input_dev, idx);
		input_report_abs(g_input_dev, ABS_MT_TRACKING_ID, -1);
		input_mt_report_slot_state(g_input_dev, MT_TOOL_FINGER, false);
	}
#else
    input_report_key(g_input_dev, BTN_TOUCH, 0);
	input_mt_sync(g_input_dev);
#endif
	input_sync(g_input_dev);	
#endif	


#ifdef HYN_GESTURE

    if(hyn_ges_wakeup_switch == 1){
        u8 buf[4];
        //close gesture detect
        buf[0] = 0xD0;
		buf[1] = 0x4C;
		buf[2] = 0x00;
		cst3xx_i2c_write(g_i2c_client, buf, 3);
        tpd_halt=0;
		hyn_lcd_flag = 1;	
		
		printk(" linc tpd-gesture detect is closed .\n");
       	
    }
	else
#endif
	
	cst3xx_irq_enable(ts);
	
    cst3xx_firmware_info(g_i2c_client);


#ifdef CONFIG_TP_ESD_PROTECT

	cst3xx_esd_switch(SWITCH_ESD_ON);

#endif
	
	cst3xx_reset_ic(30);

	mdelay(200);
	
	printk(" linc cst3xx wake up done.\n");

	return 0;

}
static int cst3xx_suspend(struct cst3xx_ts_data *ts)
{ 

 #ifdef ICS_SLOT_REPORT
	int idx;
#endif

	printk(" linc cst3xx enter sleep.\n");

#ifdef HYN_GESTURE
    if(hyn_ges_wakeup_switch == 1){  
        
//		int err;
		u8 buf[4];
        hyn_lcd_flag = 0;
        mdelay(10);			
		tpd_halt=1;
    
        buf[0] = 0xD0;
		buf[1] = 0x4C;
		buf[2] = 0x80;
		cst3xx_i2c_write(g_i2c_client, buf, 3);		
		printk(" linc tpd-gesture detect is opened \n");
		
        mdelay(10);
        return -1;
    }
#endif

#ifdef CONFIG_TP_ESD_PROTECT

	cst3xx_esd_switch(SWITCH_ESD_OFF);

#endif

    cst3xx_irq_disable(ts);


#ifdef SLEEP_CLEAR_POINT
	#ifdef ICS_SLOT_REPORT
		for(idx=0; idx<=10; idx++) {	
			input_mt_slot(g_input_dev, idx);
			input_report_abs(g_input_dev, ABS_MT_TRACKING_ID, -1);
			input_mt_report_slot_state(g_input_dev, MT_TOOL_FINGER, false);
		}
	#else
	    input_report_key(g_input_dev, BTN_TOUCH, 0);
		input_mt_sync(g_input_dev);
	#endif
		input_sync(g_input_dev);	
#endif	

    cst3xx_enter_sleep(g_i2c_client);
	
    gpio_direction_output(ts->pdata->reset_gpio,0);
    msleep(50);  
	printk(" linc cst3xx enter sleep done.\n");

	return 0;

}

static const struct of_device_id tpd_of_match[] = {
	{.compatible = "hynitron,cst1xx"},
	{},
};

MODULE_DEVICE_TABLE(of, tpd_of_match);


static struct i2c_driver cst3xx_ts_driver = {

  .driver = {
    		.name = TPD_DRIVER_NAME,		
#ifdef CONFIG_OF
		   .of_match_table = of_match_ptr(tpd_of_match),
#endif
#if !defined(CONFIG_FB) && defined(CONFIG_PM)
		   .pm = &cst3xx_ts_pm_ops,
#endif
  },
  .probe    = cst3xx_tpd_probe,
  .remove   = cst3xx_tpd_remove,
  .id_table = cst3xx_tpd_id,
//  .detect   = cst3xx_tpd_detect,
  //.address_list = (const unsigned short *)forces,
};
/* called when loaded into kernel */
static int __init cst3xx_ts_init(void)
{
	printk(" linc hyn is entering cst3xx_ts_init.\n");

	cst3xx_wq = create_singlethread_workqueue("cst3xx_wq");
	if (!cst3xx_wq) {
		printk("Creat workqueue failed.");
		return -ENOMEM;
	}
	return i2c_add_driver(&cst3xx_ts_driver);
}

/* should never be called */
static void __exit cst3xx_ts_exit(void)
{
	printk(" linc hyn is entering cst3xx_ts_exit.\n");
	i2c_del_driver(&cst3xx_ts_driver);
	if (cst3xx_wq) {
		destroy_workqueue(cst3xx_wq);
	}
}

late_initcall(cst3xx_ts_init);
module_exit(cst3xx_ts_exit);

