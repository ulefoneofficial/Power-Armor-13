
/**********************************************************************************************************
*							Copyright ( C ) , x-2021
*						ShenZhen PRIZE TECHNOLOGY CO.,LTD
*								All Rights Reserved
* --------------------------------------------------------------------------------------------							
* File Name: microarray_fp.h
* Author   : Chendewen
* Time     : 2021/5/19
* Version  : V1.0.0
**********************************************************************************************************/


#ifndef  _MICROARRAY_FP_H_                                     
#define  _MICROARRAY_FP_H_


/*========================================================================================================
 *                                            include files
 =======================================================================================================*/
#include <linux/poll.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/freezer.h>
#include <asm/ioctl.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/ioctl.h>
#include <linux/regulator/consumer.h>
#include <linux/power_supply.h>
#include <linux/pinctrl/consumer.h>






/*========================================================================================================
 *                                               define
 =======================================================================================================*/
#define DEBUG_MICROARRAY_FP
#ifdef DEBUG_MICROARRAY_FP
    #define MICROARRAY_FP_DEBUG(fmt,arg...)  printk(KERN_INFO "cdw_microarray_fp_debug=======Function:[%s]," fmt,__func__,##arg);
#else
	#define MICROARRAY_FP_DEBUG(fmt,arg...) 
#endif


#define MICROARRAY_FP_INFO(fmt,arg...) 	printk(KERN_INFO  "cdw_microarray_fp_info========Function:[%s]," fmt ,__func__,##arg);

#define MICROARRAY_FP_ERR(fmt,arg...) 	printk(KERN_ERR   "cdw_microarray_fp_err =========Function:[%s]," fmt ,__func__,##arg);

#define MICROARRY_FP_DRV_NAME  "microarray_fp" 
#define MA_CHR_FILE_NAME 	"madev0"  //do not neeed modify usually 
#define MA_CHR_DEV_NAME 	"madev"	  //do not neeed modify usually 

#define IMAGE_SIZE 13312
#define IMAGE_DMA_SIZE 32*1024

#define FINGERPRINT_SWIPE_UP 			KEY_FN_F1//827
#define FINGERPRINT_SWIPE_DOWN 			KEY_FN_F2//828
#define FINGERPRINT_SWIPE_LEFT 			KEY_FN_F3//829
#define FINGERPRINT_SWIPE_RIGHT 		KEY_FN_F4//830
#define FINGERPRINT_TAP 				KEY_FN_F9//	831
#define FINGERPRINT_DTAP				KEY_FN_F6// 	832
#define FINGERPRINT_LONGPRESS 			KEY_FN_F7//833


// ioctl
#define MA_DRV_VERSION	    (0x00004006)

#define MA_IOC_MAGIC            'M'
//#define MA_IOC_INIT           _IOR(MA_IOC_MAGIC, 0, unsigned char)
#define TIMEOUT_WAKELOCK        _IO(MA_IOC_MAGIC, 1)
#define SLEEP                   _IO(MA_IOC_MAGIC, 2)    //陷入内核
#define WAKEUP                  _IO(MA_IOC_MAGIC, 3)    //唤醒
#define ENABLE_CLK              _IO(MA_IOC_MAGIC, 4)    //打开spi时钟
#define DISABLE_CLK             _IO(MA_IOC_MAGIC, 5)    //关闭spi时钟
#define ENABLE_INTERRUPT        _IO(MA_IOC_MAGIC, 6)    //开启中断上报
#define DISABLE_INTERRUPT       _IO(MA_IOC_MAGIC, 7)    //关闭中断上报
#define TAP_DOWN                _IO(MA_IOC_MAGIC, 8)
#define TAP_UP                  _IO(MA_IOC_MAGIC, 9)
#define SINGLE_TAP              _IO(MA_IOC_MAGIC, 11)
#define DOUBLE_TAP              _IO(MA_IOC_MAGIC, 12)
#define LONG_TAP                _IO(MA_IOC_MAGIC, 13)

#define MA_IOC_VTIM             _IOR(MA_IOC_MAGIC,  14, unsigned char)     //version time
#define MA_IOC_CNUM             _IOR(MA_IOC_MAGIC,  15, unsigned char)     //cover num
#define MA_IOC_SNUM             _IOR(MA_IOC_MAGIC,  16, unsigned char)     //sensor type
#define MA_IOC_UKRP             _IOW(MA_IOC_MAGIC,  17, unsigned char)     //user define the report key

#define MA_KEY_UP/*KEY_UP*/                  _IO(MA_IOC_MAGIC,  18)                  //nav up
#define MA_KEY_LEFT/*KEY_LEFT*/                _IO(MA_IOC_MAGIC,  19)                  //nav left
#define MA_KEY_DOWN/*KEY_DOWN*/                _IO(MA_IOC_MAGIC,  20)                  //nav down
#define MA_KEY_RIGHT/*KEY_RIGHT*/               _IO(MA_IOC_MAGIC,  21)                  //nav right

#define MA_KEY_F14/*KEY_F14*/                 _IO(MA_IOC_MAGIC,  23)  //for chuanyin
#define SET_MODE                _IOW(MA_IOC_MAGIC, 33, unsigned int)    //for yude
#define GET_MODE                _IOR(MA_IOC_MAGIC, 34, unsigned int)    //for yude


#define ENABLE_IRQ/*ENABLE_IQ*/               _IO(MA_IOC_MAGIC, 31)
#define DISABLE_IRQ/*DISABLE_IQ*/              _IO(MA_IOC_MAGIC, 32)

#define MA_IOC_GVER             _IOR(MA_IOC_MAGIC,   35, unsigned int)      //get the driver version,the version mapping in the u32 is the final  4+4+8,as ******** ******* ****(major verson number) ****(minor version number) ********(revised version number), the front 16 byte is reserved.
#define SCREEN_OFF              _IO(MA_IOC_MAGIC,    36)
#define SCREEN_ON               _IO(MA_IOC_MAGIC,    37)
#define SET_SPI_SPEED           _IOW(MA_IOC_MAGIC,   38, unsigned int)


#define WAIT_FACTORY_CMD        _IO(MA_IOC_MAGIC,    39)//for fingerprintd
#define WAKEUP_FINGERPRINTD     _IO(MA_IOC_MAGIC,    40)//for factory test
#define WAIT_FINGERPRINTD_RESPONSE                                  _IOR(MA_IOC_MAGIC,    41, unsigned int)//for factory test
#define WAKEUP_FACTORY_TEST_SEND_FINGERPRINTD_RESPONSE              _IOW(MA_IOC_MAGIC,    42, unsigned int)//for fingerprintd
#define WAIT_SCREEN_STATUS_CHANGE                                   _IOR(MA_IOC_MAGIC,    43, unsigned int)
#define GET_INTERRUPT_STATUS                                        _IOR(MA_IOC_MAGIC,    44, unsigned int)
#define SYNC					_IO(MA_IOC_MAGIC, 45)
#define SYNC2					_IO(MA_IOC_MAGIC, 46)
#define GET_SCREEN_STATUS		_IOR(MA_IOC_MAGIC, 47, unsigned int)


/*--extern define --*/
#ifdef   MICROAARY_FP_MODULE
#define  MICROAARY_FP_EXT
#else
#define  MICROAARY_FP_EXT  extern
#endif



/*========================================================================================================
 *                                               struct or enum
 ========================================================================================================*/
struct fprint_spi {
    u8 do_what;             //工作内容
    u8 f_wake;              //唤醒标志  
    int value;              
    volatile u8 f_irq;      //中断标志
    volatile u8 u1_flag;    //reserve for ours thread interrupt
    volatile u8 u2_flag;    //reserve for ours thread interrupt
    volatile u8 f_repo;     //上报开关  
    spinlock_t spi_lock;
    struct spi_device *spi;
    struct list_head dev_entry;
    struct spi_message msg;
    struct spi_transfer xfer;
    struct input_dev *input;
    struct work_struct work;
    struct workqueue_struct *workq;
#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend suspend;
#endif

#ifdef CONFIG_PM_WAKELOCKS
	struct wakeup_source wl;
#else
	struct wake_lock wl;
#endif
};


struct fprint_dev {
    dev_t idd;          
    int major;          
    int minor;
    struct cdev *chd;
    struct class *cls;
    struct device *dev;
};

struct microarray_fp_priv
{
	struct spi_device *spi;
	struct regulator *vdd_reg;
	struct fprint_spi smas;
	struct fprint_dev sdev;
	
	
	int pin_pwr;
	int pin_irq;
	int pin_rst;
	
	enum of_gpio_flags flags_pwr;
	enum of_gpio_flags flags_irq;
	enum of_gpio_flags flags_rst;
	
	struct work_struct Work;
	struct workqueue_struct *Workq;
	struct notifier_block notifier;

#ifdef CONFIG_PM_WAKELOCKS
	struct wakeup_source ProcessWakeLock;
#else
	struct wake_lock ProcessWakeLock;
#endif
	
};




/*========================================================================================================
 *                    global variable declaration(全局动态变量声明，供其他源文件使用)
 ========================================================================================================*/



/*========================================================================================================
 *                    global function declaration(全局动态函数声明，供其他源文件调用)
 =======================================================================================================*/








#endif                                                      
