/*
Z6 Digital Lcd Driver
gezi
2021-06-30
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/timer.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/sysfs.h>
#include <linux/power_supply.h>

#include "tm1621d.h"

int TM1621D_DEBUG_ENABLE = 1;

static spinlock_t wake_tm1621d_lock;

static struct hrtimer tm1621d_timer;

static DECLARE_WAIT_QUEUE_HEAD(tm1621d_thread_wq);

static unsigned int tm1621d_thread_wq_flag = false;

//CHARGE_STATE g_curent_charge_state = STATE_HOST_BATTERY_DISCHARGE;

//static ktime_t g_ktime;

/*
--------tm1621d ctrl gpio---------
sgm4564_oe
slg_ldo_3v3
tm1621d_cs
tm1621d_wr
tm1621d_data
*/
static int sgm4564_oe;
static int slg_ldo_3v3;
static int gpio_cs;
static int gpio_wr;
static int gpio_data;


const struct of_device_id tm1612d_id[] = {
	{.compatible = "mediatek,tm1612d"},
	{},
};
MODULE_DEVICE_TABLE(of, tm1612d_id);
			//		0	 1	  2    3    4    5    6    7    8    9    A    b    c    d    E    F
//u8 Smg[16]   = {0xeb,0x60,0xc7,0xE5,0x6C,0xAD,0xaf,0xE0,0xef,0xed,0xee,0x2f,0x8b,0x67,0x8f,0x8e};  //0~F字型码
u8 Smg_h[16]   = {0x5f,0x06,0x3d,0x2f,0x66,0x6b,0x7b,0x0e,0x7f,0x6f,0x7e,0x73,0x59,0x47,0x79,0x78};  //0~F字型码
u8 Smg_l[16]   = {0xf5,0x60,0xd3,0xf2,0x66,0xb6,0xb7,0xe0,0xf7,0xf6,0xe7,0x37,0x95,0x74,0x97,0x87};  //0~F字型码
u8 Tab0[16]    = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //清屏
//u8 test[7]     = {0x00,0xff,0xff,0xff,0x7e,0xf5,0x0d};
			    //NULL T4&2 T5&1 T611 COL3 T3&4 T2FH */
u8 out_data[7]  ={0x00,0x7f,0x7f,0x00,0xff,0xf7,0x00};
u8 fulldata[7]  ={0xff,0xff,0xff,0xff,0xff,0xff,0xff};
u8 time_mask[7] ={0x00,0x7f,0x7f,0x00,0xf7,0xf7,0x00};
/*                NULL T4&2 T5&1 T611 COL3 T3&4 T2FH */
			//	   COL 	T1	 T2   T3   T4   T5   T6   T7   T8   T9  T10  T11   HD  FHD
u8 Smg_tmask[14]={0x08,0x04,0x08,0x08,0x80,0x80,0x80,0x40,0x10,0x20,0x08,0x04,0x02,0x01};  //状态栏
u8 Smg_t_bit[14]={0x04,0x06,0x06,0x05,0x01,0x02,0x03,0x03,0x03,0x03,0x03,0x03,0x06,0x06};  //状态栏
/********************从高位写入数据*************************/
static void Write_Data_H(u8 Data, u8 Cnt)	   //Data的高cnt位写入TM1621D，高位在前
{
	u8 i;
	for(i=0;i<Cnt;i++)
	{
		gpio_direction_output(gpio_wr,0);
		if(Data&0x80){                          //从最高位发送
			gpio_direction_output(gpio_data,1);	
		}
		else{
			gpio_direction_output(gpio_data,0);
		}
		
		udelay(20);
		
		gpio_direction_output(gpio_wr,1);
		Data<<=1;
	}
	gpio_direction_output(gpio_wr,0);
	gpio_direction_output(gpio_data,0);
}

/********************从低位写入数据*************************/
static void Write_Data_L(u8 Data,u8 Cnt) //Data 的低cnt位写入TM1621D，低位在前 
{ 
	unsigned char i; 
	for(i=0;i<Cnt;i++) 
	{ 
		gpio_direction_output(gpio_wr,0);
		if(Data&0x01){ 	                        //从低位发送
			gpio_direction_output(gpio_data,1);	
		}			
		else {
			gpio_direction_output(gpio_data,0); 
		}
		
		udelay(20);
		
		gpio_direction_output(gpio_wr,1);
		Data>>=1;
	}
	gpio_direction_output(gpio_wr,0);
	gpio_direction_output(gpio_data,0); 
} 

/********************写入控制命令*************************/
void WriteCmd(u8 Cmd)
{
	gpio_direction_output(gpio_cs,0);
	udelay(20);
	Write_Data_H(0x80,4);     //写入命令标志100
	Write_Data_H(Cmd,8);      //写入命令数据
	gpio_direction_output(gpio_cs,1);
	udelay(20);
}

/*********指定地址写入数据，实际写入后4位************/
void WriteOneData(u8 Addr, u8 Data)
{
	gpio_direction_output(gpio_cs,0);
	Write_Data_H(0xa0,3);     //写入数据标志101
	Write_Data_H(Addr<<2,6);  //写入地址数据
	Write_Data_L(Data,4);     //写入数据
	gpio_direction_output(gpio_cs,1);
	udelay(20);
}

/*********连续写入方式，每次数据为8位，写入数据************/
void WriteAllData(u8 Addr,u8 *p,u8 cnt)
{
	u8 i;
	gpio_direction_output(gpio_cs,0);
	Write_Data_H(0xa0,3);          //写入数据标志101
	Write_Data_H(Addr<<2,6);	//写入地址数据
	for(i=0;i<cnt;i++)		//写入数据
	{
		Write_Data_L(*p,8);	    
		p++;
	} 
	gpio_direction_output(gpio_cs,1);
	udelay(20);
}

/*******************TM1621D初始化**********************/
void tm1621d_init()
{
	gpio_direction_output(gpio_cs,1);
	gpio_direction_output(gpio_wr,1);
	gpio_direction_output(gpio_data,1);
	udelay(1000);		
	WriteCmd(BIAS);		 	//1/3偏压 4公共口
	WriteCmd(RC);			 //内部RC振荡
	WriteCmd(SYSDIS);		 //关系统振荡器和LCD偏压发生器
	WriteCmd(SYSEN);		 //打开系统振荡器
	WriteCmd(LCDON);		 //开LCD偏压
}

static void tm1621d_power_set(int on)
{
	TM_DBG("gezi ----%s----%d\n",__func__,on);
	if(on){
		gpio_direction_output(sgm4564_oe,1);
		gpio_direction_output(slg_ldo_3v3,1);
	}
	else{
		gpio_direction_output(slg_ldo_3v3,0);
		gpio_direction_output(sgm4564_oe,0);
	}
}

void tm1621d_test()
{
	//static u8 init_done = 0;
	static u8 i = 1;
	if(i){
		tm1621d_power_set(1);
		udelay(500);
		//TM_DBG("gezi ----[gezi][%s]\n",__func__);
		tm1621d_init();		//开机初始化
		udelay(1000);	
		WriteAllData(0,Tab0,16); 	//LCD清显存
		WriteAllData(0x0a,out_data,16); 	 //LCD SEG10到SEG21显示0~5
		udelay(500);
		i = 1;
	}
	else{
		WriteAllData(0,Tab0,16); 	//LCD清显存
		mdelay(500);
		tm1621d_power_set(0);
		i = 1;
	}
	
}


void wake_up_tm1621d_thread_wq(void)
{
	
	TM_DBG("gezi ----[gezi][%s]\n",__func__);
	
	spin_lock(&wake_tm1621d_lock);
	
	tm1621d_thread_wq_flag = true;

	wake_up(&tm1621d_thread_wq);
	
	spin_unlock(&wake_tm1621d_lock);
}

//EXPORT_SYMBOL(wake_up_tm1621d_thread_wq);

int tm1612d_thread_routine(void *x)
{
	while (true) 
	{
		wait_event(tm1621d_thread_wq, (tm1621d_thread_wq_flag == true));
		
		tm1621d_thread_wq_flag = false;
		
		TM_DBG("gezi tm1612d_thread_routine......wake up...........\n");
		
		tm1621d_test();
	}
}

static ssize_t tm1621d_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	TM_DBG("tm1621d_show=======");
	return 0;
}

static ssize_t tm1621d_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t size)
{
	int num;
	int i=0;
	TM_DBG("tm1621d_show buf:%s,size:%d=======",*buf,(int)size);
	sscanf(buf,"%d",&num);
	if( (num > 9999) || (num < 0) )
	{
		if(num == 88888)
			for(i=0;i<7;i++)
				out_data[i]=fulldata[i];
		else if{num == 99999}
			for(i=0;i<7;i++)
				out_data[i]=Tab0[i];
	}else{
		out_data[1] = (out_data[1]&(~time_mask[1])) | (Smg_h[num%1000/100]&time_mask[1]);
		out_data[2] = (out_data[2]&(~time_mask[2])) | (Smg_h[num/1000]&time_mask[2]);
		out_data[4] = (out_data[4]&(~time_mask[4])) | (Smg_l[num%100/10]&time_mask[4]);
		out_data[5] = (out_data[5]&(~time_mask[5])) | (Smg_l[num%10]&time_mask[5]);
	}
	return size;
	
}
static ssize_t tm1621d_t_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	TM_DBG("tm1621d_t_show=======");
	return 0;
}
static ssize_t tm1621d_t_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t size)
{
	int t_num;
	int ret = 0;
	sscanf(buf,"%d",&t_num);
	if( (t_num >= 0) && (t_num < 15) ){
		TM_DBG("tm1621d_t_show buf:%s,size:%d=======",*buf,(int)size);
		ret = out_data[Smg_t_bit[t_num]]&Smg_tmask[t_num];
		out_data[Smg_t_bit[t_num]] = (out_data[Smg_t_bit[t_num]]&(~Smg_tmask[t_num]))  |  ((~ret)&Smg_tmask[t_num]);
		TM_DBG("tm1621d_t_show wmd buf:0x%x=======",out_data[Smg_t_bit[t_num]]);
	}
	   
	return size;
}
static DEVICE_ATTR(tm1621d_time, 0664, tm1621d_show, tm1621d_store);
static DEVICE_ATTR(tm1621d_status, 0664, tm1621d_t_show, tm1621d_t_store);
struct class *tm1621d_class;
static int gpio_init(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;
	
	int ret = 0;
		
	struct device *tm1621d_dev;
	tm1621d_class = class_create(THIS_MODULE, "tm1621d");
	
	if (IS_ERR(tm1621d_class)) {
		TM_DBG("Failed to create class(tm1621d_class)!");
		return PTR_ERR(tm1621d_class);
	}

	tm1621d_dev = device_create(tm1621d_class, NULL, 0, NULL, "tm1621d_fun");
	if (IS_ERR(tm1621d_dev))
		TM_DBG("Failed to create tm1621d_dev device");
	
	if (device_create_file(tm1621d_dev, &dev_attr_tm1621d_time) < 0)
		TM_DBG("Failed to create device file(%s)!",dev_attr_tm1621d_time.attr.name);	
	if (device_create_file(tm1621d_dev, &dev_attr_tm1621d_status) < 0)
		TM_DBG("Failed to create device file(%s)!",dev_attr_tm1621d_status.attr.name);	
	
	TM_DBG("tm1612d request GPIO start!!!\n");
	
	if (dev->of_node) {
		match = of_match_device(of_match_ptr(tm1612d_id), dev);
		if (!match) {
			pr_err("[tm1612d][ERROR] No device match found\n");
			return -ENODEV;
		}
	}
	
	sgm4564_oe = of_get_named_gpio(dev->of_node, "sgm4564-oe", 0);
	ret = gpio_request(sgm4564_oe, "sgm4564_oe");
	if (ret < 0)
	{
		TM_DBG("[ERROR] Unable to request sgm4564-oe\n");
		goto err;
	}
	else
		TM_DBG("SUCCESS] success to request sgm4564-oe = %d\n",sgm4564_oe);
	
	slg_ldo_3v3 = of_get_named_gpio(dev->of_node, "slg-ldo-3v3", 0);
	ret = gpio_request(slg_ldo_3v3, "slg_ldo_3v3");
	if (ret < 0)
	{
		TM_DBG("[ERROR] Unable to request slg_ldo_3v3\n");
		goto err;
	}
	else
		TM_DBG("SUCCESS] success to request slg_ldo_3v3 = %d\n",slg_ldo_3v3);
	
	gpio_cs = of_get_named_gpio(dev->of_node, "tm1621d-cs", 0);
	ret = gpio_request(gpio_cs, "gpio_cs");
	if (ret < 0)
	{
		TM_DBG("[ERROR] Unable to request gpio-cs\n");
		goto err;
	}
	else
		TM_DBG("SUCCESS] success to request gpio_cs = %d\n",gpio_cs);
	
	gpio_wr = of_get_named_gpio(dev->of_node, "tm1621d-wr", 0);
	ret = gpio_request(gpio_wr, "gpio_wr");
	if (ret < 0)
	{
		TM_DBG("[ERROR] Unable to request gpio_wr\n");
		goto err;
	}
	else
		TM_DBG("SUCCESS] success to request gpio_wr = %d\n",gpio_wr);
	
	gpio_data = of_get_named_gpio(dev->of_node, "tm1621d-data", 0);
	ret = gpio_request(gpio_data, "gpio_data");
	if (ret < 0)
	{
		TM_DBG("[ERROR] Unable to request gpio_data\n");
		goto err;
	}
	else
		TM_DBG("SUCCESS] success to request gpio_data = %d\n",gpio_data);
	
	
	return 0;
	
err:
	return -1;
}
enum hrtimer_restart tm1621d_hrtimer_func(struct hrtimer *timer)
{
	ktime_t ktime;
	
	TM_DBG("gezi----- charge_state_hrtimer_func....\n ");
	
	ktime = ktime_set(3, 0);
	
	wake_up_tm1621d_thread_wq();
	
	hrtimer_start(&tm1621d_timer, ktime, HRTIMER_MODE_REL);
	
	return HRTIMER_NORESTART;
}

void tm1612d_hrtimer_init(void)
{
	ktime_t ktime;
	
	ktime = ktime_set(5, 0); 
	
	TM_DBG("tm1621d_timer init..\n");

	hrtimer_init(&tm1621d_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	tm1621d_timer.function = tm1621d_hrtimer_func;
	hrtimer_start(&tm1621d_timer, ktime, HRTIMER_MODE_REL);

}


/*
static int sgm4564_oe;
static int slg_ldo_3v3;
static int gpio_cs;
static int gpio_wr;
static int gpio_data;
*/



static int tm1612d_probe(struct platform_device *dev)
{
	
	spin_lock_init(&wake_tm1621d_lock);
	
	//g_ktime = ktime_set(30, 0);
	
	if(gpio_init(dev) < 0)
	{
		TM_DBG("gezi ----%s-----error...\n",__func__);
		return -1;
	}
	
	//tm1621d_power_set(1);
	
	kthread_run(tm1612d_thread_routine, NULL, "tm1612d_thread_routine");
	
	tm1612d_hrtimer_init();
	
	//wake_up_tm1621d_thread_wq();
	

	TM_DBG("gezi ----%s-----success...\n",__func__);
	
	return 0;
}




static int tm1612d_remove(struct platform_device *dev)
{
	
	return 0;
}

static struct platform_driver tm1612d_driver = {
	.probe = tm1612d_probe,
	//.suspend = tm1612d_suspend, 
	//.resume = tm1612d_resume,
	.remove = tm1612d_remove,
	.driver = {
		   .name = "tm1612d",
		   .of_match_table = of_match_ptr(tm1612d_id),	
	},
};


static int tm1612d_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&tm1612d_driver);
	if (ret) {
		TM_DBG("gezi ---- tm1612d driver register error:(%d)\n", ret);
		return ret;
	} else {
		TM_DBG("gezi ----tm1612d platform driver register done!\n");
	}

	return 0;

}

static void tm1612d_exit(void)
{
	TM_DBG("gezi ----tm1612d_exit\n");
	platform_driver_unregister(&tm1612d_driver);

}

late_initcall_sync(tm1612d_init);
module_exit(tm1612d_exit);

MODULE_DESCRIPTION("Prize TM1621D Driver");
MODULE_AUTHOR("zhaopengge <zhaopengge@szprize.com>");
MODULE_LICENSE("GPL");
