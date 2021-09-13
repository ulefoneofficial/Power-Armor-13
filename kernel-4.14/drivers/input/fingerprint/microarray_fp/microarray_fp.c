/**********************************************************************************************************
*							Copyright ( C ) , x-2021
*						ShenZhen PRIZE TECHNOLOGY CO.,LTD
*								All Rights Reserved
* --------------------------------------------------------------------------------------------							
* File Name: microarray_fp.c
* Author   : Chendewen
* Time     : 2021/5/19
* Version  : V1.0.0
**********************************************************************************************************/



/*========================================================================================================
 *                                          include files
 ========================================================================================================*/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spidev.h>
#include <linux/spi/spi.h>

#define MICROAARY_FP_MODULE
#include "microarray_fp.h"



/*========================================================================================================
 *                            global static variable (全局静态变量，只有本源文件可使用)
 ========================================================================================================*/

u8 read_id_flag=0;
static u8* stxb;
static u8* srxb;


static struct microarray_fp_priv *gpriv ;

static DECLARE_WAIT_QUEUE_HEAD(U1_Waitq);
static DECLARE_WAIT_QUEUE_HEAD(U2_Waitq);
static DECLARE_WAIT_QUEUE_HEAD(gWaitq);
static DECLARE_WAIT_QUEUE_HEAD(screenwaitq);


static DEFINE_MUTEX(dev_lock);
static DEFINE_MUTEX(drv_lock);
static DEFINE_MUTEX(ioctl_lock);

static unsigned int ma_drv_reg;
static unsigned int ma_speed;
static unsigned int is_screen_on;
static unsigned int screen_flag;
static unsigned int int_pin_state;

/*========================================================================================================
 *                          global dynamic function (全局动态函数，该源文件和其他源文件均可调用)
 ========================================================================================================*/
 
 
 

/*========================================================================================================
 *                          global static function (全局静态函数，只有该源文件才可以调用)
 ========================================================================================================*/
 
//=========================================SPI ===========================//
static int spi_trans(u8 *tx, u8 *rx, u32 spilen)
{
	struct spi_message m;
	struct spi_transfer t = {
		.tx_buf = tx,
		.rx_buf = rx,
		.len = spilen,
	};

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spi_sync(gpriv->spi, &m);
}


static int init_spi(struct spi_device *spi,struct microarray_fp_priv *priv)
{
	int ret;
	
	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;
    spi->max_speed_hz = 6 * 1000 * 1000;
	
	ret = spi_setup(spi);
	if(ret != 0)
	{
		MICROARRAY_FP_ERR("spi_setup is fail !\n");
		return -1;
	}
	
	priv->spi = spi;		
	priv->smas.spi = spi;
	spi_set_drvdata(spi, priv);
	
    
	return 0;

}

void mas_enable_spi_clock(struct spi_device *spi)
{
#if defined(TEE_ID_COMPATIBLE_TRUSTKERNEL) || defined(TEE_ID_COMPATIBLE_MICROTRUST)  
#ifdef CONFIG_MTK_CLKMGR         
	enable_clock(MT_CG_PERI_SPI0, "spi");        
#elif defined(MT6797)            
	mt_spi_enable_clk(spi_master_get_devdata(spi->master)); 
#else
	mt_spi_enable_master_clk(spi);
	//enable_clk();
#endif
#endif
}

void mas_disable_spi_clock(struct spi_device *spi){
    
#if defined(TEE_ID_COMPATIBLE_TRUSTKERNEL) || defined(TEE_ID_COMPATIBLE_MICROTRUST)
#ifdef CONFIG_MTK_CLKMGR          
    disable_clock(MT_CG_PERI_SPI0, "spi");       
#elif defined(MT6797)             
    mt_spi_disable_clk(spi_master_get_devdata(spi->master)); 
#else
    mt_spi_disable_master_clk(spi);
    //disable_clk();
#endif
#endif

}  

//====================================hw==========================//
#if 0
static int microarray_fp_hw_rest(struct microarray_fp_priv *priv)
{	
	if (!gpio_is_valid(priv->pin_rst))
	{
		MICROARRAY_FP_ERR("microarray_fp_priv-->pin_rst is err !");
		return -1;
	}
	
	gpio_set_value(priv->pin_rst, priv->flags_rst);
	udelay(5000);
	gpio_set_value(priv->pin_rst, !priv->flags_rst);
	
	return 0;
}

#endif

static int microarray_fp_hw_power_on(struct microarray_fp_priv *priv)
{	
	if(IS_ERR_OR_NULL(priv->vdd_reg) && !gpio_is_valid(priv->pin_pwr))
	{
		MICROARRAY_FP_ERR("microarray_fp_priv-->vdd_reg && pin_pwr is err !\n");
		return -1;
	}
	
	// regulator
	if (!IS_ERR_OR_NULL(priv->vdd_reg))
	{
		regulator_enable(priv->vdd_reg);
    }
	else
	{
		MICROARRAY_FP_INFO("microarray_fp_priv-->vdd_reg is err but pin_pwr is not err !");
	}
		
	// gpio
	if (gpio_is_valid(priv->pin_pwr))
	{
		gpio_set_value(priv->pin_pwr, priv->flags_pwr);
	}
	else
	{
		MICROARRAY_FP_INFO("microarray_fp_priv-->pin_pwr is err but vdd_reg is not err !");
	}
	
	return 0;
}


static int microarray_fp_hw_power_off(struct microarray_fp_priv *priv)
{	
	if(IS_ERR_OR_NULL(priv->vdd_reg) && !gpio_is_valid(priv->pin_pwr))
	{
		MICROARRAY_FP_ERR("microarray_fp_priv-->vdd_reg && pin_pwr is err !\n");
		return -1;
	}
	
	// regulator
	if (!IS_ERR_OR_NULL(priv->vdd_reg))
	{
		regulator_disable(priv->vdd_reg);
    }
	else
	{
		MICROARRAY_FP_INFO("microarray_fp_priv-->vdd_reg is err but pin_pwr is not err !");
	}
		
	// gpio
	if (gpio_is_valid(priv->pin_pwr))
	{
		gpio_set_value(priv->pin_pwr, !priv->flags_pwr);
	}
	else
	{
		MICROARRAY_FP_INFO("microarray_fp_priv-->pin_pwr is err but vdd_reg is not err !");
	}
	
	return 0;

}

static int microarray_fp_rest(struct microarray_fp_priv *priv)
{	
	u8 buf_rx[4]= {0};
	u8 buf_tx[4] = {0x8C,0xFF,0xFF,0xFF};
    int ret;
	
	ret =spi_trans(buf_tx,buf_rx,4);
	if(ret != 0)
	{
		return -1;
	}
	
	udelay(5000);

	return 0;
}

static int microarray_fp_hw_test(struct microarray_fp_priv *priv)
{	
	u8 buf_rx[4]= {0};
	u8 buf_tx[4] = {0x00,0xFF,0xFF,0xFF};
    int ret;

	ret=spi_trans(buf_tx,buf_rx,4);
	if(ret != 0)
	{
		return -1;
	}

    
    if(buf_rx[3] == 0x41 || buf_rx[3] == 0x45) 
    {
        read_id_flag = 1;
        return 0;
    }

	return -1;
}

static int init_microarray_fp_hw(struct microarray_fp_priv *priv)
{
	int ret,i;
	
	ret=microarray_fp_hw_power_on(priv);
	if(ret<0)
	{
		MICROARRAY_FP_ERR("microarray_fp_hw_power_on  is fail !\n");
		return -1;
	}
	
	// 测试读id
    for(i =0;i<5;i++)
    {
		ret=microarray_fp_rest(priv);
		if(ret<0)
		{
			MICROARRAY_FP_ERR("microarray_fp_rest  is fail !\n");
			return -1;
		}
	
        ret= microarray_fp_hw_test(priv);
        if(ret == 0)
        {
            break;
        }
    }
    if(ret<0)
    {
        MICROARRAY_FP_ERR("microarray_fp_hw_test  is fail !\n");
        return -1;
    }
	
	
	return 0;
	
}

static int mas_switch_power(int cmd)
{    
	int ret = 0;
	
	if(cmd)
	{
		ret = microarray_fp_hw_power_on(gpriv);
	}
	else
	{
		ret = microarray_fp_hw_power_off(gpriv);
	}                                                                                                                     

	return ret;
}  


//==============================================interrupt=====================//
static irqreturn_t mas_interrupt(int irq, void *dev_id) {

    queue_work(gpriv->Workq, &gpriv->Work);

	return IRQ_HANDLED;
}

static void mas_work(struct work_struct *pws) {
   gpriv->smas.f_irq = 1;
   wake_up(&gWaitq);

}

int mas_get_interrupt_gpio(unsigned int index)
{
	int val;
	
	val = gpio_get_value(gpriv->pin_irq);
	
	return val;
}


//===================================file f_pos==========================//

/* 写数据
 * @return 成功:count, -1count太大，-2拷贝失败
 */
static ssize_t mas_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos) {
    int val = 0;
    int ret ;
    //MALOGD("start");
    //printk("cdw--->mas_write\n");
    if(count==6) {                                                                              //cmd ioctl, old version used the write interface to do ioctl, this is only for the old version
        int cmd, arg;
        u8 tmp[6];
        ret = copy_from_user(tmp, buf, count);
        cmd = tmp[0];
        cmd <<= 8;
        cmd += tmp[1];
        arg = tmp[2];
        arg <<= 8;
        arg += tmp[3];
        arg <<= 8;
        arg += tmp[4];
        arg <<= 8;
        arg += tmp[5];

	} else {        
        //memset(stxb, 0, FBUF);
        memset(stxb, 0xff, IMAGE_DMA_SIZE);
        ret = copy_from_user(stxb, buf, count);     
        if(ret) {
            MICROARRAY_FP_ERR("copy form user failed\n");
            val = -2;
        } else {
            val = count;
        }       
    }
    return val;
}

/* 读数据
 * @return 成功:count, -1count太大，-2通讯失败, -3拷贝失败
 */
static ssize_t mas_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos) {
    int val, ret = 0;
    ret = spi_trans(stxb, srxb, count);
    if(ret) {
        MICROARRAY_FP_ERR("mas_sync failed.\n");
        return -2;
    }
    ret = copy_to_user(buf, srxb, count);
    if(!ret) val = count;
    else {
        val = -3;
        MICROARRAY_FP_ERR("copy_to_user failed.\n");
    }       

    return val;
}

static long mas_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
    //MALOGF("start");
	int tmp;
    int ret;
	
    //printk("cdw-->mas_ioctl-->cmd=[%d]\n",cmd);
    switch(cmd){
        case TIMEOUT_WAKELOCK:                                                       //延时锁    timeout lock
#ifdef CONFIG_PM_WAKELOCKS
            __pm_wakeup_event(&gpriv->ProcessWakeLock, 5000);
#else
	    wake_lock_timeout(&gpriv->ProcessWakeLock, 5 * HZ);
#endif
            break;
        case SLEEP:                                                       //remove the process out of the runqueue
            if(mas_get_interrupt_gpio(0) == 1) {
                MICROARRAY_FP_ERR("int pull down failed!");
                return 0;
			}
            gpriv->smas.f_irq = 0;
			ret = wait_event_freezable(gWaitq, gpriv->smas.f_irq != 0);
			break;
        case WAKEUP:                                                       //wake up, schedule the process into the runqueue
            gpriv->smas.f_irq = 1;
            wake_up(&gWaitq);
            break;
        case ENABLE_CLK:
            mas_enable_spi_clock(gpriv->smas.spi);                                    //if the spi clock is not opening always, do this methods
            break;
        case DISABLE_CLK:
            mas_disable_spi_clock(gpriv->smas.spi);                                   //disable the spi clock
            break;
        case ENABLE_INTERRUPT:
            enable_irq(gpio_to_irq(gpriv->pin_irq));                                                    //enable the irq,in fact, you can make irq enable always
            break;
        case DISABLE_INTERRUPT:
            disable_irq(gpio_to_irq(gpriv->pin_irq));                                                    //disable the irq
            break;
        case TAP_DOWN:
            input_report_key(gpriv->smas.input, FINGERPRINT_TAP, 1);
            input_sync(gpriv->smas.input);                                                 //tap down
            break;
        case TAP_UP:
            input_report_key(gpriv->smas.input, FINGERPRINT_TAP, 0);
            input_sync(gpriv->smas.input);                                                     //tap up
            break;
        case SINGLE_TAP:
            input_report_key(gpriv->smas.input, FINGERPRINT_TAP, 1);
            input_sync(gpriv->smas.input);
            input_report_key(gpriv->smas.input, FINGERPRINT_TAP, 0);
            input_sync(gpriv->smas.input);                                                       //single tap
            break;
        case DOUBLE_TAP:
            input_report_key(gpriv->smas.input, FINGERPRINT_DTAP, 1);
            input_sync(gpriv->smas.input);
            input_report_key(gpriv->smas.input, FINGERPRINT_DTAP, 0);
            input_sync(gpriv->smas.input);                                              //double tap
            break;
        case LONG_TAP:
            input_report_key(gpriv->smas.input, FINGERPRINT_LONGPRESS, 1);
            input_sync(gpriv->smas.input);
            input_report_key(gpriv->smas.input, FINGERPRINT_LONGPRESS, 0);
            input_sync(gpriv->smas.input);                                               //long tap
            break;
        case MA_KEY_UP:
            input_report_key(gpriv->smas.input, FINGERPRINT_SWIPE_UP, 1);
            input_sync(gpriv->smas.input);
            input_report_key(gpriv->smas.input, FINGERPRINT_SWIPE_UP, 0);
            input_sync(gpriv->smas.input);
            break;
        case MA_KEY_LEFT:
            input_report_key(gpriv->smas.input, FINGERPRINT_SWIPE_LEFT, 1);
            input_sync(gpriv->smas.input);
            input_report_key(gpriv->smas.input, FINGERPRINT_SWIPE_LEFT, 0);
            input_sync(gpriv->smas.input);
            break;
        case MA_KEY_DOWN:
            input_report_key(gpriv->smas.input, FINGERPRINT_SWIPE_DOWN, 1);
            input_sync(gpriv->smas.input);
            input_report_key(gpriv->smas.input, FINGERPRINT_SWIPE_DOWN, 0);
            input_sync(gpriv->smas.input);
            break;
        case MA_KEY_RIGHT:
            input_report_key(gpriv->smas.input, FINGERPRINT_SWIPE_RIGHT, 1);
            input_sync(gpriv->smas.input);
            input_report_key(gpriv->smas.input, FINGERPRINT_SWIPE_RIGHT, 0);
            input_sync(gpriv->smas.input);
            break;
        case SET_MODE:
            mutex_lock(&ioctl_lock);
            ret = copy_from_user(&ma_drv_reg, (unsigned int*)arg, sizeof(unsigned int));
            mutex_unlock(&ioctl_lock);
            break;
        case GET_MODE:
            mutex_lock(&ioctl_lock);
            ret = copy_to_user((unsigned int*)arg, &ma_drv_reg, sizeof(unsigned int));
            mutex_unlock(&ioctl_lock);
            break;
        case MA_IOC_GVER:
            {            
                unsigned int ma_drv_version = MA_DRV_VERSION;
                mutex_lock(&ioctl_lock);
                if (copy_to_user((void *)arg, &ma_drv_version, sizeof(unsigned int))) {
                    MICROARRAY_FP_ERR("copy_to_user(..) failed.\n");
                    ret = (-EFAULT);
                }            
                mutex_unlock(&ioctl_lock);
            }
            break;
		case SCREEN_ON:
			mas_switch_power(1);
			break;
		case SCREEN_OFF:
			mas_switch_power(0);
			break;
        case SET_SPI_SPEED:
            ret = copy_from_user(&ma_speed, (unsigned int*)arg, sizeof(unsigned int));
            //ma_spi_change(smas->spi, ma_speed, 0);
            break;
        case WAIT_FACTORY_CMD:
            gpriv->smas.u2_flag = 0;
           	ret = wait_event_freezable(U2_Waitq, gpriv->smas.u2_flag != 0);
			break;
        case WAKEUP_FINGERPRINTD:
            gpriv->smas.u2_flag = 1;
            wake_up(&U2_Waitq);
            break;
        case WAIT_FINGERPRINTD_RESPONSE:
            gpriv->smas.u1_flag = 0;
            ret = wait_event_freezable(U1_Waitq,  gpriv->smas.u1_flag != 0);
            mutex_lock(&ioctl_lock);
            tmp = copy_to_user((unsigned int*)arg, &ma_drv_reg, sizeof(unsigned int));
            mutex_unlock(&ioctl_lock);
			break;
        case WAKEUP_FACTORY_TEST_SEND_FINGERPRINTD_RESPONSE:
            mutex_lock(&ioctl_lock);
            ret = copy_from_user(&ma_drv_reg, (unsigned int*)arg, sizeof(unsigned int));
            mutex_unlock(&ioctl_lock);
            msleep(4);
            gpriv->smas.u1_flag = 1;
            wake_up(&U1_Waitq);
            break;
		case WAIT_SCREEN_STATUS_CHANGE:
			screen_flag = 0;
			ret = wait_event_freezable(screenwaitq, screen_flag != 0);
			mutex_lock(&ioctl_lock);
			tmp = copy_to_user((unsigned int*)arg, &is_screen_on, sizeof(unsigned int));
			mutex_unlock(&ioctl_lock);
			break;
		case GET_INTERRUPT_STATUS:
			int_pin_state = mas_get_interrupt_gpio(0);
			mutex_lock(&ioctl_lock);
			tmp = copy_to_user((unsigned int*)arg, &int_pin_state, sizeof(unsigned int));
			mutex_unlock(&ioctl_lock);
			break;
		case GET_SCREEN_STATUS:
			mutex_lock(&ioctl_lock);
			ret = copy_to_user((unsigned int*)arg, &is_screen_on, sizeof(unsigned int));
			mutex_unlock(&ioctl_lock);
			break;
        default:
            ret = -EINVAL;
            MICROARRAY_FP_ERR("mas_ioctl no such cmd");
    }

    return ret;
}

#ifdef CONFIG_COMPAT
static long mas_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    //printk("cdw-->mas_compat_ioctl-->cmd=[%d]\n",cmd);
    int retval = 0;
    //printk("cdw-->mas_compat_ioctl-->cmd=[%d]\n",cmd);
    retval = filp->f_op->unlocked_ioctl(filp, cmd, arg);
    return retval;
}
#endif

void * kernel_memaddr = NULL;
unsigned long kernel_memesize = 0;

int mas_mmap(struct file *filp, struct vm_area_struct *vma)
{
	unsigned long page;
	if ( !kernel_memaddr ) { 
			kernel_memaddr = kmalloc(128*1024, GFP_KERNEL);
			if( !kernel_memaddr ) { 
					return -1; 
			}   
	}  
    page = virt_to_phys((void *)kernel_memaddr) >> PAGE_SHIFT;
    vma->vm_page_prot=pgprot_noncached(vma->vm_page_prot);
    if( remap_pfn_range(vma, vma->vm_start, page, (vma->vm_end - vma->vm_start), 
                vma->vm_page_prot) )
        return -1;
    //vma->vm_flags |= VM_RESERVED;
    vma->vm_flags |=  VM_DONTEXPAND | VM_DONTDUMP;
    printk("remap_pfn_rang page:[%lu] ok.\n", page);
    return 0;
}



static const struct file_operations sfops = {
    .owner = THIS_MODULE,
    .write = mas_write,
    .read = mas_read,
    .unlocked_ioctl = mas_ioctl,
    .mmap = mas_mmap,
    //.ioctl = mas_ioctl,       
    //using the previous line replacing the unlock_ioctl while the linux kernel under version2.6.36
#ifdef CONFIG_COMPAT
    .compat_ioctl = mas_compat_ioctl,
#endif

};

static int init_file_node(struct microarray_fp_priv *priv)
{
    int ret;

    ret = alloc_chrdev_region(&(priv->sdev.idd), 0, 1, MA_CHR_DEV_NAME);
    if(ret < 0)
    {
        MICROARRAY_FP_ERR("alloc_chrdev_region error!\n");
        return -1;
    }
    
	priv->sdev.chd = cdev_alloc();
	if (!(priv->sdev.chd))
    {
        MICROARRAY_FP_ERR("cdev_alloc error!\n");
        return -1;
    }
	
    priv->sdev.chd->owner = THIS_MODULE;
    priv->sdev.chd->ops = &sfops;
	cdev_add(priv->sdev.chd, priv->sdev.idd, 1); 
	priv->sdev.cls = class_create(THIS_MODULE, MA_CHR_DEV_NAME);
	if (IS_ERR(priv->sdev.cls)) 
	{
		MICROARRAY_FP_ERR("class_create err !!! \n");
		return -1;
	}
	
	priv->sdev.dev = device_create(priv->sdev.cls, NULL, priv->sdev.idd, NULL, MA_CHR_FILE_NAME);
	ret = IS_ERR(priv->sdev.dev) ? PTR_ERR(priv->sdev.dev) : 0;
	if(ret)
	{
	    MICROARRAY_FP_ERR("device_create err\n");
		return -1;
	}

	return 0;
}

//======================================notifier ==============================//
static int mas_fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	unsigned int blank;
	if(event != FB_EVENT_BLANK) {
		return 0;
	}
	blank = *(int *)evdata->data;
	switch(blank){
		case FB_BLANK_UNBLANK:
			is_screen_on = 1;
			break;
		case FB_BLANK_POWERDOWN:
			is_screen_on = 0;
			break;
		default:
			break;
	}
	screen_flag = 1;
	wake_up(&screenwaitq);
	return 0;
}


static int init_notifier_call(struct microarray_fp_priv  *priv)
{
	priv->notifier.notifier_call = mas_fb_notifier_callback;
	fb_register_client(&priv->notifier);
	is_screen_on = 1;
	return 0;
}




//=================================================input =====================//
static int init_input(struct microarray_fp_priv  *priv) 
{
    struct input_dev *input = NULL;
	int ret;
    
	input = input_allocate_device();
    if (!input) 
	{
        MICROARRAY_FP_ERR("input_allocate_device failed.");
        return -1 ;
    }
	
    set_bit(EV_KEY, input->evbit);
    //set_bit(EV_ABS, input->evbit);
    set_bit(EV_SYN, input->evbit);
    set_bit(FINGERPRINT_SWIPE_UP, input->keybit); //单触
    set_bit(FINGERPRINT_SWIPE_DOWN, input->keybit);
    set_bit(FINGERPRINT_SWIPE_LEFT, input->keybit);
    set_bit(FINGERPRINT_SWIPE_RIGHT, input->keybit);
    set_bit(FINGERPRINT_TAP, input->keybit);
    set_bit(FINGERPRINT_DTAP, input->keybit);
    set_bit(FINGERPRINT_LONGPRESS, input->keybit);
    
    set_bit(KEY_POWER, input->keybit);

    input->name = MA_CHR_DEV_NAME;
    input->id.bustype = BUS_SPI;
    ret = input_register_device(input);
    if (ret) 
	{
        input_free_device(input);
        MICROARRAY_FP_ERR("failed to register input device.");
        return -1;
    }
	
	priv->smas.input  = input;
	
	return 0;
}

//==================================dts===============================//
static int microarray_fp_parse_dts(struct device_node *np,struct microarray_fp_priv  *priv)
{
	int ret ;
	
	// power
	priv->pin_pwr =  of_get_named_gpio_flags(np, "fingerprint,power-gpio",0, &priv->flags_pwr);
	if (gpio_is_valid(priv->pin_pwr))
	{
		if (gpio_request(priv->pin_pwr,"microarray_fp_gpio_power" )) 
		{
			MICROARRAY_FP_ERR("gpio_request microarray_fp_gpio_power is err !!!\n");
			goto err_request_gpio_power;
		}
		else
		{
			gpio_direction_output(priv->pin_pwr,!priv->flags_pwr);
		}
		
	}

	
	priv->vdd_reg = regulator_get(/*&(priv->spi->dev)*/NULL, "VFP");
    if (!IS_ERR(priv->vdd_reg))
	{
		regulator_set_voltage(priv->vdd_reg, 2800000, 2800000);
    }
	
	// rst
	priv->pin_rst =  of_get_named_gpio_flags(np, "fingerprint,rest-gpio",0, &priv->flags_rst);
	if (gpio_is_valid(priv->pin_rst))
	{
		if (gpio_request(priv->pin_rst,"microarray_fp_gpio_rest" )) 
		{
			MICROARRAY_FP_ERR("gpio_request microarray_fp_gpio_rest is err !!!\n");
			goto err_request_gpio_rest;
		}
		else
		{
			gpio_direction_output(priv->pin_rst,priv->flags_rst);
		}	
		
	}

	// irq                                       
	priv->pin_irq =  of_get_named_gpio_flags(np, "fingerprint,touch-int-gpio",0, &priv->flags_irq);
	if(!gpio_is_valid(priv->pin_irq))
	{
		MICROARRAY_FP_ERR("of_get_named_gpio_flags fingerprint,touch-int-gpio is err !!!\n");
		goto err_of_get_named_gpio_flags_irq;
	}
	
	if (gpio_request(priv->pin_irq,"microarray_fp_gpio_irq" )) 
	{
		MICROARRAY_FP_ERR("gpio_request microarray_fp_gpio_irq is err !!!\n");
		goto err_request_gpio_irq;
	}

	gpio_direction_input(priv->pin_irq);
	ret = request_irq(gpio_to_irq(priv->pin_irq), mas_interrupt, IRQF_TRIGGER_RISING | IRQF_ONESHOT, "microarray_fp_irq", NULL);
	if(ret<0)
	{
		MICROARRAY_FP_ERR("request_irq is err !!!");
		goto err_gpio_request_irq;
	}
    enable_irq_wake(gpio_to_irq(priv->pin_irq));
		
	
	
	
	MICROARRAY_FP_DEBUG("pin_pwr=[%d],vdd_reg=[%d],pin_rst=[%d],pin_irq=[%d]\n",priv->pin_pwr,priv->vdd_reg,priv->pin_rst,priv->pin_irq);


	
	return 0;
	
err_gpio_request_irq:
	gpio_free(priv->pin_irq);
	
err_request_gpio_irq:


err_of_get_named_gpio_flags_irq:
	if (gpio_is_valid(priv->pin_rst))
	{
		gpio_free(priv->pin_rst);
	}

err_request_gpio_rest:
	if (gpio_is_valid(priv->pin_pwr))
	{
		gpio_free(priv->pin_pwr);
	}
	
	if (!IS_ERR(priv->vdd_reg))
	{
		regulator_put(priv->vdd_reg);
	}
	
	
err_request_gpio_power:
	return -1;
	
	
}

static int microarray_fp_probe(struct spi_device *spi)
{
	struct microarray_fp_priv *priv = NULL;
	struct device_node *np = spi->dev.of_node;
	int ret=-1;
	
	MICROARRAY_FP_DEBUG("microarray_fp driver v3.6 init is startting ...\n");
	
	/*-- 校验 --*/
	if (!np)
	{
		MICROARRAY_FP_ERR ("microarray_fp driver init is fail of  NULL pointer\n");
		goto err_check;
	}
	
	/*-- 申请microarray_fp 空间 --*/
	priv = kzalloc(sizeof(struct microarray_fp_priv), GFP_KERNEL);
	if (!priv)
	{
		MICROARRAY_FP_ERR("microarray_fp request memory is fail ,memory space is not enough!\n");
		goto err_kzalloc;
	}

	/*--解析dts --*/
	ret = microarray_fp_parse_dts(np,priv);
	if(ret<0)
	{
		MICROARRAY_FP_ERR("microarray_fp_parse_dts  is fail !\n");
		goto err_microarray_fp_parse_dts;
	}
	
	/*--初始化唤醒锁和队列--*/
#ifdef CONFIG_PM_WAKELOCKS
    wakeup_source_init(&priv->ProcessWakeLock, "microarray_process_wakelock");
#else
    wake_lock_init(&priv->ProcessWakeLock, WAKE_LOCK_SUSPEND, "microarray_process_wakelock");
#endif

    INIT_WORK(&priv->Work, mas_work);
    priv->Workq = create_singlethread_workqueue("mas_workqueue");
    if (!priv->Workq) 
	{
		MICROARRAY_FP_ERR("create_singlethread_workqueue is err\n");
        goto err_create_singlethread_workqueue;
    }
	
	
	/*--初始化全局变量--*/
	gpriv = priv;
    stxb = (u8*)__get_free_pages(GFP_KERNEL, get_order(IMAGE_DMA_SIZE));
    srxb = (u8*)__get_free_pages(GFP_KERNEL, get_order(IMAGE_DMA_SIZE));
    if ( stxb==NULL || srxb==NULL) 
	{
		MICROARRAY_FP_ERR("stxb or srxb __get_free_pages is err\n");
		goto err_microarray_fp_parse_dts;
	}
    memset(stxb,0x00,get_order(IMAGE_DMA_SIZE));
    memset(srxb,0x00,get_order(IMAGE_DMA_SIZE));

	/*--初始化spi--*/
	ret = init_spi(spi,priv);
	if(0 != ret)
	{
		MICROARRAY_FP_ERR("init_spi  is fail !\n");
		goto err_init_spi;
	}
	
	/*--初始化文件节点--*/
	ret= init_file_node(priv);
	if(0 != ret)
	{
		MICROARRAY_FP_ERR("init_spi  is fail !\n");
		goto err_init_file_node;
	}
	
	/*--初始化input--*/
	ret=init_input(priv);
	if(ret<0)
	{
		MICROARRAY_FP_ERR("init_input  is fail !\n");
		goto err_init_input;
	}
	
	/*--初始化通知--*/
	ret = init_notifier_call(priv);
	if(ret != 0)
	{
		MICROARRAY_FP_ERR("init_notifier_call  is fail !\n");
		goto err_init_notifier_call;
	}
	
	/*--初始化microarray hw--*/
	ret=init_microarray_fp_hw(priv);
	if(ret<0)
	{
		MICROARRAY_FP_ERR("init_microarray_fp_hw  is fail !\n");
		goto err_init_microarray_fp_hw;
	}
	


	MICROARRAY_FP_DEBUG("microarray_fp driver init is sucess !!!\n");
	
	return 0;
	
err_init_microarray_fp_hw:
	fb_unregister_client(&priv->notifier);
	
err_init_notifier_call:
	input_free_device(priv->smas.input);

err_init_input:
    cdev_del(priv->sdev.chd);
    device_destroy(priv->sdev.cls, priv->sdev.idd);
	unregister_chrdev_region(priv->sdev.idd, 1);

err_init_file_node:
	
err_init_spi:
	destroy_workqueue(priv->Workq);

err_create_singlethread_workqueue:
	free_irq(gpio_to_irq(priv->pin_irq), NULL);

	gpio_free(priv->pin_irq);
	

	if (gpio_is_valid(priv->pin_rst))
	{
		gpio_free(priv->pin_rst);
	}
	

	if (gpio_is_valid(priv->pin_pwr))
	{
		gpio_free(priv->pin_pwr);
	}
	
	if (!IS_ERR(priv->vdd_reg))
	{
		regulator_put(priv->vdd_reg);
	}
	
err_microarray_fp_parse_dts:
	
err_kzalloc:
	
err_check:
	return -1;


}



static int microarray_fp_suspend(struct device *dev)
{

	MICROARRAY_FP_INFO("microarray_fp_suspend !!!\n");
	return 0;
}

static int microarray_fp_resume(struct device *dev)
{
	MICROARRAY_FP_INFO("microarray_fp_resume !!!\n");
	return 0;
}

static int microarray_fp_remove(struct spi_device *spi)
{
	struct microarray_fp_priv *priv = spi_get_drvdata(spi);
	
	fb_unregister_client(&priv->notifier);
	input_free_device(priv->smas.input);
	
	cdev_del(priv->sdev.chd);
    device_destroy(priv->sdev.cls, priv->sdev.idd);
	unregister_chrdev_region(priv->sdev.idd, 1);
	
	destroy_workqueue(priv->Workq);
	
	free_irq(gpio_to_irq(priv->pin_irq), NULL);

	gpio_free(priv->pin_irq);
	

	if (gpio_is_valid(priv->pin_rst))
	{
		gpio_free(priv->pin_rst);
	}
	

	if (gpio_is_valid(priv->pin_pwr))
	{
		gpio_free(priv->pin_pwr);
	}
	
	if (!IS_ERR(priv->vdd_reg))
	{
		regulator_put(priv->vdd_reg);
	}


	
	return 0;
}

static const struct dev_pm_ops microarray_fp_pm = {
	.suspend = microarray_fp_suspend,
	.resume = microarray_fp_resume};

struct of_device_id microarray_fp_of_match[] = {
	{ .compatible = "microarray_fp,e064n", },
    { .compatible = "prize,fingerprint",  },
	{},
};
MODULE_DEVICE_TABLE(of, microarray_fp_of_match);

static const struct spi_device_id microarray_fp_id[] = {
	{MICROARRY_FP_DRV_NAME, 0},
	{}};
MODULE_DEVICE_TABLE(spi, microarray_fp_id);

static struct spi_driver microarray_fp_driver = {
	.driver = {
		.name = MICROARRY_FP_DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &microarray_fp_pm,
		.of_match_table = of_match_ptr(microarray_fp_of_match),
	},
	.id_table = microarray_fp_id,
	.probe = microarray_fp_probe,
	.remove = microarray_fp_remove,
};


static int __init microarray_fp_init(void)
{
	return spi_register_driver(&microarray_fp_driver);
}

static void __exit microarray_fp_exit(void)
{
	spi_unregister_driver(&microarray_fp_driver);
}

late_initcall_sync(microarray_fp_init);
module_exit(microarray_fp_exit);

MODULE_DESCRIPTION("microarray_fp spi Driver");
MODULE_AUTHOR("chendewen@szprize.com");
MODULE_LICENSE("GPL");
MODULE_ALIAS("microarray_fp");
