/***********************************************************
 *  版权所有 (C) 2015-2020, 深圳市铂睿智恒科技有限公司 
 *
 *  文件名称: hall_device.c
 *  内容摘要: hall driver for hall device
 *  当前版本: V1.0
 *  作    者: 丁俊
 *  完成日期: 2015-04-10
 *  修改记录: 
 *  修改日期: 
 *  版本号  :
 *  修改人  :
 ***********************************************************/
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/ctype.h>
#include <linux/semaphore.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/workqueue.h>
#include <linux/delay.h>

#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/kthread.h>
#include <linux/input.h>
#if defined(CONFIG_PM_WAKELOCKS)
#include <linux/pm_wakeup.h>
#else
#include <linux/wakelock.h>
#endif
#include <linux/time.h>

#include <linux/string.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <linux/input.h>

#include <linux/of.h>
#include <linux/iio/consumer.h>
#include <linux/iio/iio.h>

//prize add by lipengpeng 20210312 start 
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/uaccess.h>
#include <linux/init.h>
#include <linux/types.h>

//prize add by lipengpeng 20210312 end  

/*----------------------------------------------------------------------
static variable defination
----------------------------------------------------------------------*/
#define air_quality_DEVNAME    "air_quality_dev"

#define EN_DEBUG

#if defined(EN_DEBUG)
		
#define TRACE_FUNC 	printk("[air_quality_dev] function: %s, line: %d \n", __func__, __LINE__);

#define air_quality_DEBUG  printk
#else

#define TRACE_FUNC(x,...)

#define air_quality_DEBUG(x,...)
#endif

int adc_status=0;
struct iio_channel *air_channel = NULL;
/****************************************************************/
/*******static function defination                             **/
/****************************************************************/

static struct pinctrl *air_quality_pinctrl;
static struct pinctrl_state *air_quality_default;
static struct pinctrl_state *air_quality_lna_en_high;
static struct pinctrl_state *air_quality_lna_en_low;

//prize add by lipengpeng 20210312 start 
static struct hrtimer air_quality_timer;
int close_airquality_value=0;
int air_quality_getadc_v(void);
int air_quality_getadc(void);
static int air_quality_timer_start(ktime_t ktime);
static int air_quality_timer_cancel(void);
static void air_quality_polling(struct work_struct *data);

static struct workqueue_struct *air_workqueue;
static struct work_struct air_work;
static DEFINE_MUTEX(air_mutex);

#define VS0_NORMAL_CYCLE_CNT   		    600//一般的基准更新周期     
#define VS_FILTER_CNT      		        VS0_NORMAL_CYCLE_CNT/6//响应电压的滤波周期
//#define USE_RS_ENABLE                   1//使用电阻变化率
#define PPM_MAX                         99
#define PPM_MIN                         1
#define ADC_MAX                         4095
#define NORMAL_ONE_CYCLE_CNT          	100//
#define WARMING_UP_DELAY                30//传感器的预热稳定时间

#define VS0_CHANGE_UP_COEF   			0.8f//基准往上更新调整系数
#define VS0_CHANGE_DOWN_COEF   			0.6f//基准往下更新调整系数
//typedef  unsigned char uint8_t ;
//typedef  unsigned int uint16_t;

//prize add by lipengpeng 20210327 start 
//#define TvocGain			30		//TVOC 额外增益
//#define BasicsGain			500		//基础增益
//#define HchoUpperLimit		2*1000		//HCHC量程上限
//#define TvocUpperLimit		10*1000		//TVOC量程上限

#define HchoUpperLimit		0.2		//HCHC量程上限
#define TvocUpperLimit		30		//TVOC量程上限
#define ERR_DATA_UP			1600	//数据范围筛选上限  
#define ERR_DATA_DW			200		//数据范围筛选下限 

//typedef struct{
	
//	float hcho_f;		//HCHO输出值
	
//	float tvoc_f;		//TVOC 输出值
	
//	float filter_value;	//对输入数据进行低通滤波
	
//}test_t;

//prize add by lipengpeng 20210327 end 

typedef struct
{
	unsigned int VC_ADC_VOL;
	unsigned int VL_ADC_VOL;
	unsigned int VS_ADC_VOL;

	float   SensorVS0TempFloat;
	float   SensorCycleAvgValueTempFloat;

	unsigned int  normalCycleCnt; //传感器每一个周期的计数，范围：1-VS0_NORMAL_CYCLE_CNT 初始化为1

	unsigned int SensorVS0;

	unsigned int VsRealTimeCycleCnt;//用于计算等级的实时电压滤波计数
	unsigned int SensorRealTimeAvgValue;
	float        RealTimeAvgFloat;//实时电压滤波的均值

	unsigned char degree;
	unsigned char ppm_out;

	unsigned char warming_up_flag;
	
	
}SensorModuleData;
SensorModuleData sm;//传感器参数的声明


//static void sensorParameterInit(SensorModuleData* sm);
//static void updateVS0(SensorModuleData* sm);
//static void updateVS_filter(SensorModuleData* sm);
//static unsigned char  HGM_SENSOR_GET_PPM(unsigned int vs);
//void getDegree(SensorModuleData* sm);


/*
**传感器参数初始化
*/
void sensorParameterInit(SensorModuleData* sm)
{

	sm->VC_ADC_VOL = 0;
	sm->VL_ADC_VOL = 0;
	sm->VS_ADC_VOL = 0;

	sm->normalCycleCnt = 1;

	sm->VsRealTimeCycleCnt = 1;
	sm->degree = PPM_MAX;
	sm->ppm_out = PPM_MIN;
	sm->warming_up_flag = 0;

}

/*
**外设初始化需要调用一次（only once）
*/
void SENSOR_INIT(void)
{
	sensorParameterInit(&sm);
}

/*unsigned char HGM_SENSOR_GET_PPM(uint32_t vs)
{
	sm.VS_ADC_VOL = vs;
	updateVS0(&sm);
	updateVS_filter(&sm);
	getDegree(&sm);
	return sm.ppm_out;

}


void updateVS_filter(SensorModuleData* sm)
{

	if (sm->VsRealTimeCycleCnt == VS_FILTER_CNT + 1)
	{
		sm->VsRealTimeCycleCnt = 3;	   //可以有一定的初始计数，防止数据波动过大。
	}

	//新的一个小周期的累积开始
	sm->RealTimeAvgFloat = ((float)sm->RealTimeAvgFloat / (float)sm->VsRealTimeCycleCnt) * (sm->VsRealTimeCycleCnt - 1) \
		+ (float)sm->VS_ADC_VOL / (float)sm->VsRealTimeCycleCnt;

	sm->SensorRealTimeAvgValue = sm->RealTimeAvgFloat;
	sm->VsRealTimeCycleCnt++;


}

void updateVS0(SensorModuleData* sm)
{
	static uint8_t warmingup_cnt;
	float temp_change;
	if (warmingup_cnt++ > WARMING_UP_DELAY) {
		sm->warming_up_flag = 1;
	}
	if (!sm->warming_up_flag) {
		return;
	}

	if (sm->normalCycleCnt == VS0_NORMAL_CYCLE_CNT + 1)
	{
		sm->normalCycleCnt = 1;
		//		sm->vs0_lock_flag=1;//电池包 手持仪器 冰箱，汽车AQS，传感器是可量产，稳定可靠

		if (sm->SensorVS0 < sm->SensorCycleAvgValueTempFloat) {//基准往上更新
			temp_change = sm->SensorCycleAvgValueTempFloat - sm->SensorVS0;
			sm->SensorVS0 += temp_change * VS0_CHANGE_UP_COEF;

		}
		else if (sm->SensorVS0 > sm->SensorCycleAvgValueTempFloat) {//基准向下更新

			temp_change = sm->SensorVS0 - sm->SensorCycleAvgValueTempFloat;
			sm->SensorVS0 -= temp_change * VS0_CHANGE_DOWN_COEF;

		}
	}
	//新的一个小周期的累积开始

	sm->SensorCycleAvgValueTempFloat = ((float)sm->SensorCycleAvgValueTempFloat / (float)sm->normalCycleCnt) \
		* (sm->normalCycleCnt - 1) + (float)sm->VS_ADC_VOL / (float)sm->normalCycleCnt;


	if (sm->normalCycleCnt < VS0_NORMAL_CYCLE_CNT)//不满一个周期则保持跟随
	{
		if (sm->degree > 90) {
			sm->SensorVS0TempFloat = ((float)sm->SensorVS0TempFloat / (float)sm->normalCycleCnt) * (sm->normalCycleCnt - 1) + (float)sm->VS_ADC_VOL / (float)sm->normalCycleCnt;
		}
		else {
			sm->SensorVS0TempFloat = sm->SensorVS0TempFloat;
		}
		sm->SensorVS0 = sm->SensorVS0TempFloat;
	}
	sm->normalCycleCnt++;
}

void getDegree(SensorModuleData* sm)
{

	float rs = 0, rs_filter, rs0;

	float rs_filter_temp1, rs_filter_temp2;
	float rs_temp1 = 0, rs_temp2 = 0;

	float degree_r;
	float degree_filter_r;

	float degree_temp;

	rs = (float)(sm->VS_ADC_VOL);
	rs_filter = (float)sm->RealTimeAvgFloat;
	rs0 = (float)(sm->SensorVS0);
	if (0 == rs0) { return; }

	rs_temp1 = ((double)(sm->VS_ADC_VOL) / (double)(sm->VL_ADC_VOL));
	rs_temp2 = ((double)(sm->SensorVS0) / (double)(sm->VC_ADC_VOL - sm->SensorVS0));

	rs_filter_temp1 = ((double)(sm->SensorRealTimeAvgValue) / (double)(sm->VL_ADC_VOL));
	rs_filter_temp2 = ((double)(sm->SensorVS0) / (double)(sm->VC_ADC_VOL - sm->SensorVS0));
#ifdef 	USE_RS_ENABLE
	degree_r = (rs_temp1 / rs_temp2) * 100.0f;//电阻变化率
	degree_filter_r = (rs_filter_temp1 / rs_filter_temp2) * 100.0f;

#else
	degree_r = (rs / rs0) * 100.0f;  //电压变化率
	degree_filter_r = (rs_filter / rs0) * 100.0f;   //电压变化率

#endif


	degree_temp = degree_filter_r;

	if (degree_temp > 99)
	{
		degree_temp = 99;
	}
	else if (degree_temp < PPM_MIN) {
		degree_temp = PPM_MIN;
	}

	sm->degree = degree_temp;
	if (sm->degree > PPM_MAX) {
		sm->degree = PPM_MAX;
	}

	sm->ppm_out = 100 - sm->degree;
}
*/

//prize add by lipengpeng 20210312 end  

//prize add by lipengpeng 20210327 start 
/*test_t test_fun(float	vol, int update_flag)		//每秒钟调用一次，输入ADC得到的原始值
{
	static	float 	w_x[3]={0,0,0},	w_y[3]={0,0,0};
	static	float		Gain = 0.0026030080977003916;			//0.167HZ
	static	float		B[3] = {1,2,1};
	static	float		A[3] = {1,-1.8902512609862294,0.90066329337703088};
	static	float 	max_voltage  	= 0;		//最大电压，即洁净空气的电压
	static	uint8_t	frist_flag			= 0;		//初次上电标志
	
	test_t	temp;
	
	if(frist_flag == 0)							//初次上电更新滤波参数初值
	{
		w_x[2]=w_x[1]=w_x[1]=w_x[0]	= vol; 
		w_y[2]=w_y[1]=w_y[1]=w_y[0]	= vol; 
		frist_flag = 1;
	}
	///////////低通滤波器开始/////////////
w_x[0] = vol;
	w_y[0] = (B[0]*w_x[0]+B[1]*w_x[1]+B[2]*w_x[2])*Gain-w_y[1]*A[1]-w_y[2]*A[2];
	
	temp.filter_value = w_y[0]/A[0];
	
	w_x[2] = w_x[1]; w_x[1] = w_x[0];
	w_y[2] = w_y[1]; w_y[1] = w_y[0];
	///////////低通滤波器结束/////////////
	if(temp.filter_value > max_voltage || update_flag)		//更新最大电压，即最干净空气
	{
		max_voltage = temp.filter_value;
	}
	temp.ratio =  (max_voltage - (temp.filter_value)) / (max_voltage);	
//当前电压与最大电压的比值
	temp.hcho_f = temp.ratio * temp.ratio * BasicsGain;		//拟合出数据
	
	temp.tvoc_f = temp.ratio * temp.ratio	* BasicsGain * TvocGain;		//拟合出数据
	
	if(temp.hcho_f > HchoUpperLimit)		//输出限幅
	{
		temp.hcho_f = HchoUpperLimit;
	}
	if(temp.tvoc_f > TvocUpperLimit)
	{
		temp.tvoc_f = TvocUpperLimit;
	}
	
	temp.hcho_f=temp.hcho_f*1000;
	temp.tvoc_f=temp.tvoc_f*1000;
	
	return temp;
}
*/


float fabs_m(float vol)
{
	return vol>0?vol:-vol;
}

float test_fun(float  vol,int update_flag)
{
	static	float 	w_x[3]={0,0,0},	w_y[3]={0,0,0};
	static	float	Gain = 0.0026030080977003916;			//0.167HZ
	static	float	B[3] = {1,2,1};
	static	float	A[3] = {1,-1.8902512609862294,0.90066329337703088};
	static	float	vol_buff[50];
	
	static	float	filter_value = 0;
	static  float	output_value = 0;
	static	long	run_count	 = 0;						//运行计数 
	
	int i=0, j=0;
	
	float	aver_vol = 0;
	
	if(vol > ERR_DATA_UP || vol < ERR_DATA_DW)
	{
		return output_value;
	}
	
	if(update_flag)
	{
		run_count = 0;
		output_value = 0;
	}
	
	if(run_count == 0)							//初次上电更新滤波参数初值
	{
		w_x[2]=w_x[1]=w_x[1]=w_x[0]	= vol; 
		w_y[2]=w_y[1]=w_y[1]=w_y[0]	= vol; 
	}
	///////////低通滤波器开始/////////////
	w_x[0] = vol;
	w_y[0] = (B[0]*w_x[0]+B[1]*w_x[1]+B[2]*w_x[2])*Gain-w_y[1]*A[1]-w_y[2]*A[2];
	
	filter_value = w_y[0]/A[0];
	
	w_x[2] = w_x[1]; w_x[1] = w_x[0];
	w_y[2] = w_y[1]; w_y[1] = w_y[0];
	///////////低通滤波器结束/////////////
	if(run_count>=120)
	{
		for(i=0;i<49;i++)
		{
			vol_buff[i]=vol_buff[i+1];
		}
		vol_buff[49] = filter_value;
	}

	if(run_count>=180)
	{
		aver_vol = 0;
		
		for(i=0;i<40;i++)
		{
			aver_vol += vol_buff[i];
		}
		aver_vol/=40;
		
		if(fabs_m(aver_vol-filter_value)>20)
		{
			for(j=40;j<50;j++)
			{
				vol_buff[j] = aver_vol;
			}
		}
		
		output_value = aver_vol-filter_value;
		
		run_count++;
	
		return output_value*1000;
	}
	else
	{
		run_count++;
		
		return 0;
	}
}
//prize add by lipengpeng 20210327 end 


//prize add by lipengpeng 20210229 start 
int air_quality_getadc_v(void){
	
	int ret = 0;
	int val = 0;
	
	if (!IS_ERR_OR_NULL(air_channel)){
		ret = iio_read_channel_processed(air_channel, &val);
		if (ret < 0) {
			printk("%s:Busy/Timeout, IIO ch read failed %d\n", __func__, ret);
			return ret;
		}
         printk("lpp-----get air quality vol xx=%d\n", val);
		/*val * 1500 / 4096*/
		ret = (val * 1450) >> 12;  //max 1.45V
	}
	return ret;
}

int air_quality_getadc(void){
	
	int ret = 0;
	int val = 0;
	
	if (!IS_ERR_OR_NULL(air_channel)){
		ret = iio_read_channel_processed(air_channel, &val);
		if (ret < 0) {
			printk("%s:Busy/Timeout, IIO ch read failed %d\n", __func__, ret);
			return ret;
		}
		//1764  0.64 
         printk("lpp-----get air quality vol=%d\n", val);
		ret = (val * 1450) >> 12;  //max 1.45V
	}
	return 1800-ret;
}

//prize add by lipengpeng  20210220 end 

//prize add by lipengpeng 20210312 start
static int air_quality_timer_start(ktime_t ktime)
{
	printk("lipengpeng start air quality\n");
	hrtimer_start(&air_quality_timer, ktime, HRTIMER_MODE_REL);

	return 0;
}

static int air_quality_timer_cancel(void)
{

	hrtimer_cancel(&air_quality_timer);
		/* flush work queue */
	//flush_work(&air_work);

	//if (air_workqueue) {
	//	flush_workqueue(air_workqueue);
	//	destroy_workqueue(air_workqueue);
		//air_workqueue = NULL;
	//}
	return 0;
}


int air_val_tvoc=0,air_val_hcho=0;

static int update_flag=1;
 int i=0;

static void air_quality_polling(struct work_struct *data)
{
	uint32_t adc_val;
	//unsigned char air_val;
	float	air_val;

	
	mutex_lock(&air_mutex);
	
	printk("lipengpeng open air quality power\n");
	pinctrl_select_state(air_quality_pinctrl, air_quality_lna_en_high);
	//mdelay(500);
	adc_val=air_quality_getadc();

    i++;
    if(i>3) update_flag=0;	
	printk("lpp---adc val=%d i=%d update_flag=%d\n",adc_val,i,update_flag);
	
	air_val=test_fun(adc_val,update_flag);
	air_val_tvoc=(int)air_val;
	air_val_hcho=(int)air_val;
	
	printk("lpp---air_val_tvoc=%d air_val_hcho=%d\n",air_val_tvoc,air_val_hcho);
    //HGM_SENSOR_GET_PPM(adc_val);
	
	mutex_unlock(&air_mutex);
}

static enum hrtimer_restart air_quality_timer_func(struct hrtimer *timer)
{
//	schedule_work(&air_quality_timer_func);
    //uint32_t adc_val;
	ktime_t ktime;
	
	if (air_workqueue != NULL)
		queue_work(air_workqueue, &air_work);
	
	//printk("lipengpeng open air quality power\n");
	//pinctrl_select_state(air_quality_pinctrl, air_quality_lna_en_high);
	//mdelay(1000);
	//adc_val=air_quality_getadc();
	//printk("lpp---adc val=%d\n",adc_val);
	
	//air_val=HGM_SENSOR_GET_PPM(adc_val);
	//printk("lpp---air val=%d\n",air_val);
	
	ktime =ktime_set(1, 100*1000*1000);// ktime_set(1,200*1000*1000);
	air_quality_timer_start(ktime);

	return HRTIMER_NORESTART;
}

//prize add by lipengpeng 20210312 end 
static int air_mode=0;
static ssize_t device_air_quality_data_show(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
	printk("[air_quality_dev] air_val_tvoc=%d\n", air_val_tvoc);
	return sprintf(buf, "%d\n", air_val_tvoc);
}
static ssize_t device_air_quality_data_store(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t size)
{
	//ktime_t ktime;
	
	printk("[air_quality_dev] %s ON/OFF value = %d:\n ", __func__, adc_status);
	
	if(sscanf(buf, "%u", &adc_status) != 1)
	{
		air_quality_DEBUG("[air_quality_dev]: Invalid values\n");
		return -EINVAL;
	}
	switch(adc_status)
	{
		case 0:
		//air  quality  diable
		printk("lpp close air quality power && stop timer\n");
		air_quality_timer_cancel();
		//pinctrl_select_state(air_quality_pinctrl, air_quality_lna_en_low);
		//air_mode=0;
		break;
		
		case 1:
		//air  quality  enable
		printk("lpp close air quality power && start timer\n");
		//pinctrl_select_state(air_quality_pinctrl, air_quality_lna_en_high);
	    //ktime = ktime_set(1,700*1000*1000);
	    //air_quality_timer_start(ktime);
		//air_mode=1;
		break;
		
		default:
		air_quality_DEBUG("[air_quality_dev]: Invalid values_%d\n",adc_status);
		break;
	}
	return size;
}

static ssize_t device_air_quality_hcho_show(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
	printk("[air_quality_dev] air_val_hcho=%d\n", air_val_hcho);
	return sprintf(buf, "%d\n", air_val_hcho);
}
static ssize_t device_air_quality_hcho_store(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t size)
{
	//ktime_t ktime;
	
	printk("[air_quality_dev] %s ON/OFF value = %d:\n ", __func__, adc_status);
	
	//if(sscanf(buf, "%u", &adc_status) != 1)
	//{
	//	air_quality_DEBUG("[air_quality_dev]: Invalid values\n");
	//	return -EINVAL;
	//}
	//switch(adc_status)
	//{
	//	case 0:
		//air  quality  diable
	//	printk("lpp close air quality power && stop timer\n");
		//air_quality_timer_cancel();
		//pinctrl_select_state(air_quality_pinctrl, air_quality_lna_en_low);
		//air_mode=0;
	//	break;
		
	//	case 1:
		//air  quality  enable
	//	printk("lpp close air quality power && start timer\n");
		//pinctrl_select_state(air_quality_pinctrl, air_quality_lna_en_high);
	    //ktime = ktime_set(1,700*1000*1000);
	    //air_quality_timer_start(ktime);
		//air_mode=1;
	//	break;
		
	//	default:
	//	air_quality_DEBUG("[air_quality_dev]: Invalid values_%d\n",adc_status);
	//	break;
	//}
	return size;
}

//prize add by lipengpeng 20210312 start 
static ssize_t device_air_quality_open_show(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
	printk("[air_quality_dev] air_quality_getadc_v()=%d\n", air_mode);
	return sprintf(buf, "%d\n", air_mode);
}
static ssize_t device_air_quality_open_store(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t size)
{
	
	ktime_t ktime;
	
	printk("[air_quality_dev] %s ON/OFF value = %d:\n ", __func__, adc_status);
	
	if(sscanf(buf, "%u", &adc_status) != 1)
	{
		air_quality_DEBUG("[air_quality_dev]: Invalid values\n");
		return -EINVAL;
	}
	switch(adc_status)
	{
		case 0:
		//air  quality  diable
	//prize add by lipengpeng  20210420 start  Clearing 0
		air_val_tvoc=0;
		air_val_hcho=0;
	//prize add by lipengpeng  20210420 end  Clearing 0
		printk("lpp close air quality power && stop xxx timer\n");
		air_quality_timer_cancel();
		pinctrl_select_state(air_quality_pinctrl, air_quality_lna_en_low);
		air_mode=0;
		break;
		
		case 1:
		//air  quality  enable
		printk("lpp close air quality power && start xxx timer\n");
		pinctrl_select_state(air_quality_pinctrl, air_quality_lna_en_high);
	    ktime = ktime_set(1, 100*1000*1000);
	    air_quality_timer_start(ktime);
		update_flag=1;//空气质量最大值清零处理
		air_mode=1;//空气质量模式
		i=0; //重新计时.
		break;
		
		default:
		air_quality_DEBUG("[air_quality_dev]: Invalid values_%d\n",adc_status);
		break;
	}
	return size;
}

//prize add by lipengpeng 20210312 end 
static DEVICE_ATTR(air_quality_data, S_IRUGO|S_IWUSR|S_IWGRP, device_air_quality_data_show, device_air_quality_data_store);
static DEVICE_ATTR(air_quality_open, S_IRUGO|S_IWUSR|S_IWGRP, device_air_quality_open_show, device_air_quality_open_store);
static DEVICE_ATTR(air_quality_hcho, S_IRUGO|S_IWUSR|S_IWGRP, device_air_quality_hcho_show, device_air_quality_hcho_store);

static int air_quality_get_dts_fun(struct platform_device *pdev)
{
	int ret = 0;
	air_quality_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(air_quality_pinctrl)) {
		air_quality_DEBUG("Cannot find fm ant pinctrl!");
		ret = PTR_ERR(air_quality_pinctrl);
	}
	//fm ant eint pin initialization 
	air_quality_default= pinctrl_lookup_state(air_quality_pinctrl, "default");
	if (IS_ERR(air_quality_default)) {
		ret = PTR_ERR(air_quality_default);
		air_quality_DEBUG("%s : init err, air_quality_default\n", __func__);
	}

	air_quality_lna_en_high = pinctrl_lookup_state(air_quality_pinctrl, "air_en_high");
	if (IS_ERR(air_quality_lna_en_high)) {
		ret = PTR_ERR(air_quality_lna_en_high);
		air_quality_DEBUG("%s : init err, air_quality_lna_en_high\n", __func__);
	}

	air_quality_lna_en_low = pinctrl_lookup_state(air_quality_pinctrl, "air_en_low");
	if (IS_ERR(air_quality_lna_en_low)) {
		ret = PTR_ERR(air_quality_lna_en_low);
		air_quality_DEBUG("%s : init err, air_quality_lna_en_low\n", __func__);
	}
	//设置暂时open enable
	pinctrl_select_state(air_quality_pinctrl, air_quality_lna_en_low);
	return ret;
}
/****************************************************************/
/*******export function defination                             **/
/****************************************************************/
struct class *air_quality_class;

static struct device *air_quality_dev;
static int air_quality_probe(struct platform_device *pdev)
{
	   int ret = 0;
	 //  ktime_t ktime;
	 //  struct device *air_quality_dev;
	   TRACE_FUNC;
       air_quality_get_dts_fun(pdev);
	  
	   air_quality_class = class_create(THIS_MODULE, "air_quality");
	
	if (IS_ERR(air_quality_class)) {
		air_quality_DEBUG("Failed to create class(air_quality_class)!");
		return PTR_ERR(air_quality_class);
	}

	air_quality_dev = device_create(air_quality_class, NULL, 0, NULL, "air_quality_data");
	if (IS_ERR(air_quality_dev))
		air_quality_DEBUG("Failed to create air_quality_dev device");
	
	if (device_create_file(air_quality_dev, &dev_attr_air_quality_data) < 0)
		air_quality_DEBUG("Failed to create device file(%s)!",dev_attr_air_quality_data.attr.name);	
	
	if (device_create_file(air_quality_dev, &dev_attr_air_quality_open) < 0)
		air_quality_DEBUG("Failed to create device file(%s)!",dev_attr_air_quality_open.attr.name);	

	if (device_create_file(air_quality_dev, &dev_attr_air_quality_hcho) < 0)
		air_quality_DEBUG("Failed to create device file(%s)!",dev_attr_air_quality_hcho.attr.name);	


	air_channel = iio_channel_get(&pdev->dev, "air-ch");
	if (IS_ERR(air_channel)) {
		ret = PTR_ERR(air_channel);
		printk("[%s] lpp fail to get auxadc iio ch4: %d, %p\n", __func__, ret, air_channel);
		return ret;
	}
	//init air quality 
	SENSOR_INIT();
//prize add by lipengpeng 20210312 start 
	INIT_WORK(&air_work, air_quality_polling);
	if (air_workqueue == NULL)
		air_workqueue = create_workqueue("air_quality_polling");
	
	hrtimer_init(&air_quality_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	air_quality_timer.function = air_quality_timer_func;
	//ktime = ktime_set(1,700*1000*1000);
	//hrtimer_start(&air_quality_timer, ktime, HRTIMER_MODE_REL);
//prize add by lipengpeng 20210312 end 
		return 0;
}

static int air_quality_remove(struct platform_device *dev)	
{
	air_quality_DEBUG("[air_quality_dev]:air_quality_remove begin!\n");
	class_destroy(air_quality_class);
	device_remove_file(air_quality_dev, &dev_attr_air_quality_data);
	device_remove_file(air_quality_dev, &dev_attr_air_quality_open);
	device_remove_file(air_quality_dev, &dev_attr_air_quality_hcho);
	air_quality_DEBUG("[air_quality_dev]:air_quality_remove Done!\n");
	return 0;
}
static const struct of_device_id air_quality_dt_match[] = {
	{.compatible = "prize,air_quality"},
	{},
};

static struct platform_driver air_quality_driver = {
	.probe	= air_quality_probe,
	.remove  = air_quality_remove,
	.driver    = {
		.name       = "Air_Quality_driver",
		.of_match_table = of_match_ptr(air_quality_dt_match),
	},
};

static int __init air_quality_init(void)
{
    int retval = 0;
    TRACE_FUNC;
    printk("[%s]: air_quality_driver, retval=%d \n!", __func__, retval);
	  if (retval != 0) {
		  return retval;
	  }
    platform_driver_register(&air_quality_driver);
    return 0;
}

static void __exit air_quality_exit(void)
{
    TRACE_FUNC;
    platform_driver_unregister(&air_quality_driver);
}

module_init(air_quality_init);
module_exit(air_quality_exit);
MODULE_DESCRIPTION("AIR QUALITY driver");
MODULE_AUTHOR("lipengpeng <lipengpeng@szprize.com>");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("AIRQUALITYDEVICE");

