#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
//#include <linux/wakelock.h>
#include <net/sock.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>

/* MTK header */
//#include "mt_spi.h"
//#include "mt_spi_hal.h"
//#include "mt_gpio.h"
//#include "mach/gpio_const.h"

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#include <linux/ioctl.h>
#include <linux/poll.h>
#include <linux/workqueue.h>

#include <linux/irq.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include "fpsensor_spi_tee.h"
#include "fpsensor_wakelock.h"
#if defined(CHIPONE_COMPATIBLE_BEANPOD_V2_5_1)
/**
Maybe this path is changed by ODM. And you should modify makefile of chipone.
**/
#include "../../../misc/mediatek/teei/V1.0/tz_vfs/fp_vendor.h" 
#elif defined(CHIPONE_COMPATIBLE_BEANPOD_V2_8_0)
/**
Maybe following paths are changed by ODM. And you should modify makefile of chipone.
chipone_vendor_uuid must be same to TA name.
**/




#include <teei_fp.h>
#include <tee_client_api.h>
static struct TEEC_UUID chipone_vendor_uuid = { 0x7778c03f, 0xc30c, 0x4dd0,
	{0xa3, 0x19, 0xea, 0x29, 0x64, 0x3d, 0x4d, 0x4b }};
#endif

#define FPSENSOR_SPI_VERSION              "fpsensor_spi_tee_mtk_v1.23.1"
#define FP_NOTIFY                         1

/* global variables */
static fpsensor_data_t *g_fpsensor = NULL;
volatile static int fpsensor_balance_spi_clk = 0;
extern void mt_spi_enable_master_clk(struct spi_device *spidev);
extern void mt_spi_disable_master_clk(struct spi_device *spidev);

//prize add by lipengpeng 20200811 start
#if defined(CONFIG_PRIZE_HARDWARE_INFO)
#include "../../../../misc/mediatek/hardware_info/hardware_info.h"
extern struct hardware_info current_fingerprint_info;
#endif
//prize add by lipengpeng 20200811 end
/* -------------------------------------------------------------------- */
/* fingerprint chip hardware configuration                              */
/* -------------------------------------------------------------------- */
static DEFINE_MUTEX(spidev_set_gpio_mutex);
static void spidev_gpio_as_int(fpsensor_data_t *fpsensor)
{
    fpsensor_debug(DEBUG_LOG, "[fpsensor]----%s---\n", __func__);
    mutex_lock(&spidev_set_gpio_mutex);
    fpsensor_debug(DEBUG_LOG,"[fpsensor]spidev_gpio_as_int\n");
    pinctrl_select_state(fpsensor->pinctrl1, fpsensor->eint_as_int);
    mutex_unlock(&spidev_set_gpio_mutex);
}
static int fpsensor_irq_gpio_cfg(fpsensor_data_t *fpsensor)
{
    struct device_node *node;
    //fpsensor_data_t *fpsensor;
    u32 ints[2] = {0, 0};
    fpsensor_debug(DEBUG_LOG,"%s\n", __func__);

    //fpsensor = g_fpsensor;

    spidev_gpio_as_int(fpsensor);

    node = of_find_compatible_node(NULL, NULL, "mediatek,icnfpsensor");
    if ( node) {
        of_property_read_u32_array( node, "debounce", ints, ARRAY_SIZE(ints));
        // gpio_request(ints[0], "fpsensor-irq");
        // gpio_set_debounce(ints[0], ints[1]);
        fpsensor_debug(DEBUG_LOG,"[fpsensor]ints[0] = %d,is irq_gpio , ints[1] = %d!!\n", ints[0], ints[1]);
        fpsensor->irq_gpio = ints[0];
        fpsensor->irq = irq_of_parse_and_map(node, 0);  // get irq number
        if (!fpsensor->irq) {
            fpsensor_debug(ERR_LOG,"fpsensor irq_of_parse_and_map fail!!\n");
            return -EINVAL;
        }
        fpsensor_debug(DEBUG_LOG," [fpsensor]fpsensor->irq= %d,fpsensor>irq_gpio = %d\n", fpsensor->irq,
                       fpsensor->irq_gpio);
    } else {
        fpsensor_debug(ERR_LOG,"fpsensor null irq node!!\n");
        return -EINVAL;
    }

    return 0 ;
}

void fpsensor_gpio_output_dts(int gpio, int level)
{
    mutex_lock(&spidev_set_gpio_mutex);
    fpsensor_debug(DEBUG_LOG, "[fpsensor]fpsensor_gpio_output_dts: gpio= %d, level = %d\n", gpio, level);
    if (gpio == FPSENSOR_RST_PIN) {
        if (level) {
            pinctrl_select_state(g_fpsensor->pinctrl1, g_fpsensor->fp_rst_high);
        } else {
            pinctrl_select_state(g_fpsensor->pinctrl1, g_fpsensor->fp_rst_low);
        }
    } else if (gpio == FPSENSOR_SPI_CS_PIN) {
        pinctrl_select_state(g_fpsensor->pinctrl1, g_fpsensor->fp_cs_low);
    } else if (gpio == FPSENSOR_SPI_MO_PIN) {
        pinctrl_select_state(g_fpsensor->pinctrl1, g_fpsensor->fp_mo_low);
    } else if (gpio == FPSENSOR_SPI_CK_PIN) {
        pinctrl_select_state(g_fpsensor->pinctrl1, g_fpsensor->fp_ck_low);
    } else if (gpio == FPSENSOR_SPI_MI_PIN) {
        pinctrl_select_state(g_fpsensor->pinctrl1, g_fpsensor->fp_mi_low);
    }
    mutex_unlock(&spidev_set_gpio_mutex);
}
int fpsensor_gpio_wirte(int gpio, int value)
{
    fpsensor_gpio_output_dts(gpio, value);
    return 0;
}
int fpsensor_gpio_read(int gpio)
{
    return gpio_get_value(gpio);
}

int fpsensor_spidev_dts_init(fpsensor_data_t *fpsensor)
{
    struct device_node *node = NULL;
    int ret = 0;
    fpsensor_debug(DEBUG_LOG, "%s\n", __func__);

    node = of_find_compatible_node(NULL, NULL, "mediatek,icnfpsensor");
    if (node) {
      
        fpsensor->eint_as_int = pinctrl_lookup_state(fpsensor->pinctrl1, "finger_eint");
        if (IS_ERR(fpsensor->eint_as_int)) {
            ret = PTR_ERR(fpsensor->eint_as_int);
            fpsensor_debug(ERR_LOG, "fpsensor Cannot find fp pinctrl eint_as_int!\n");
            return ret;
        }
        fpsensor->fp_rst_low = pinctrl_lookup_state(fpsensor->pinctrl1, "finger_rst_low");
        if (IS_ERR(fpsensor->fp_rst_low)) {
            ret = PTR_ERR(fpsensor->fp_rst_low);
            fpsensor_debug(ERR_LOG,"fpsensor Cannot find fp pinctrl fp_rst_low!\n");
            return ret;
        }
        fpsensor->fp_rst_high = pinctrl_lookup_state(fpsensor->pinctrl1, "finger_rst_high");
        if (IS_ERR(fpsensor->fp_rst_high)) {
            ret = PTR_ERR(fpsensor->fp_rst_high);
            fpsensor_debug(ERR_LOG, "fpsensor Cannot find fp pinctrl fp_rst_high!\n");
            return ret;
        }
#if 0

        fpsensor->fp_cs_low = pinctrl_lookup_state(fpsensor->pinctrl1, "fpsensor_spi_cs_low");
        if (IS_ERR(fpsensor->fp_cs_low)) {
            ret = PTR_ERR(fpsensor->fp_cs_low);
            fpsensor_debug(ERR_LOG,"fpsensor Cannot find fp pinctrl fp_cs_low!\n");
            return ret;
        }

        fpsensor->fp_mo_low = pinctrl_lookup_state(fpsensor->pinctrl1, "fpsensor_spi_mo_low");
        if (IS_ERR(fpsensor->fp_mo_low)) {
            ret = PTR_ERR(fpsensor->fp_mo_low);
            fpsensor_debug(ERR_LOG,"fpsensor Cannot find fp pinctrl fp_mo_low!\n");
            return ret;
        }

        fpsensor->fp_mi_low = pinctrl_lookup_state(fpsensor->pinctrl1, "fpsensor_spi_mi_low");
        if (IS_ERR(fpsensor->fp_mi_low)) {
            ret = PTR_ERR(fpsensor->fp_mi_low);
            fpsensor_debug(ERR_LOG,"fpsensor Cannot find fp pinctrl fp_mi_low!\n");
            return ret;
        }

        fpsensor->fp_ck_low = pinctrl_lookup_state(fpsensor->pinctrl1, "fpsensor_spi_mclk_low");
        if (IS_ERR(fpsensor->fp_ck_low)) {
            ret = PTR_ERR(fpsensor->fp_ck_low);
            fpsensor_debug(ERR_LOG,"fpsensor Cannot find fp pinctrl fp_ck_low!\n");
            return ret;
        }

        fpsensor_gpio_output_dts(FPSENSOR_SPI_MO_PIN, 0);
        fpsensor_gpio_output_dts(FPSENSOR_SPI_MI_PIN, 0);
        fpsensor_gpio_output_dts(FPSENSOR_SPI_CK_PIN, 0);
        fpsensor_gpio_output_dts(FPSENSOR_SPI_CS_PIN, 0);
#endif
    } else {
        fpsensor_debug(ERR_LOG,"fpsensor Cannot find node!\n");
        return -ENODEV;
    }
    return 0;
}
/* delay us after reset */
static void fpsensor_hw_reset(int delay)
{
    FUNC_ENTRY();

    fpsensor_gpio_wirte(FPSENSOR_RST_PIN,  1);
    udelay(100);
    fpsensor_gpio_wirte(FPSENSOR_RST_PIN,  0);
    udelay(1000);
    fpsensor_gpio_wirte(FPSENSOR_RST_PIN,  1);
    if (delay) {
        /* delay is configurable */
        udelay(delay);
    }

    FUNC_EXIT();
    return;
}

static void fpsensor_spi_clk_enable(u8 bonoff)
{
#if defined(USE_SPI_BUS)
    if (bonoff == 0 &&(fpsensor_balance_spi_clk == 1)) {
        mt_spi_disable_master_clk(g_fpsensor->spi);
		fpsensor_balance_spi_clk = 0;
    }
	else if(bonoff == 1&& (fpsensor_balance_spi_clk == 0)) {
		mt_spi_enable_master_clk(g_fpsensor->spi);
		fpsensor_balance_spi_clk = 1;
    }
#elif defined(USE_PLATFORM_BUS)
    if (bonoff == 0) {
        //enable_clock(MT_CG_PERI_SPI0, "spi");
    } else {
        //disable_clock(MT_CG_PERI_SPI0, "spi");
    }
#endif
}

static void setRcvIRQ(int val)
{
    fpsensor_data_t *fpsensor_dev = g_fpsensor;
    fpsensor_dev->RcvIRQ = val;
}

static void fpsensor_enable_irq(fpsensor_data_t *fpsensor_dev)
{
    FUNC_ENTRY();
    setRcvIRQ(0);
    /* Request that the interrupt should be wakeable */
    if (fpsensor_dev->irq_enabled == 0) {
        enable_irq(fpsensor_dev->irq);
        fpsensor_dev->irq_enabled = 1;
    }
    FUNC_EXIT();
    return;
}

static void fpsensor_disable_irq(fpsensor_data_t *fpsensor_dev)
{
    FUNC_ENTRY();

    if (0 == fpsensor_dev->device_available) {
        fpsensor_debug(ERR_LOG, "%s, devices not available\n", __func__);
        goto out;
    }

    if (0 == fpsensor_dev->irq_enabled) {
        fpsensor_debug(ERR_LOG, "%s, irq already disabled\n", __func__);
        goto out;
    }

    if (fpsensor_dev->irq) {
        disable_irq_nosync(fpsensor_dev->irq);
        fpsensor_debug(DEBUG_LOG, "%s disable interrupt!\n", __func__);
    }
    fpsensor_dev->irq_enabled = 0;

out:
    setRcvIRQ(0);
    FUNC_EXIT();
    return;
}

static irqreturn_t fpsensor_irq(int irq, void *handle)
{
    fpsensor_data_t *fpsensor_dev = (fpsensor_data_t *)handle;

    /* Make sure 'wakeup_enabled' is updated before using it
    ** since this is interrupt context (other thread...) */
    smp_rmb();
    wake_lock_timeout(&fpsensor_dev->ttw_wl, msecs_to_jiffies(1000));
    setRcvIRQ(1);
    wake_up_interruptible(&fpsensor_dev->wq_irq_return);

    return IRQ_HANDLED;
}

// release and cleanup fpsensor char device
static void fpsensor_dev_cleanup(fpsensor_data_t *fpsensor)
{
    FUNC_ENTRY();

    cdev_del(&fpsensor->cdev);
    unregister_chrdev_region(fpsensor->devno, FPSENSOR_NR_DEVS);
    device_destroy(fpsensor->class, fpsensor->devno);
    class_destroy(fpsensor->class);

    FUNC_EXIT();
}

static long fpsensor_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    fpsensor_data_t *fpsensor_dev = NULL;
    int retval = 0;
    unsigned int val = 0;
    int irqf;

    fpsensor_debug(INFO_LOG, "[rickon]: fpsensor ioctl cmd : 0x%x \n", cmd );
    fpsensor_dev = (fpsensor_data_t *)filp->private_data;
    fpsensor_dev->cancel = 0 ;
    switch (cmd) {
    case FPSENSOR_IOC_INIT:
        fpsensor_debug(INFO_LOG, "%s: fpsensor init started======\n", __func__);
        if(fpsensor_irq_gpio_cfg(fpsensor_dev) != 0) {
            break;
        }
        //regist irq
        irqf = IRQF_TRIGGER_RISING | IRQF_ONESHOT;
        retval = request_threaded_irq(fpsensor_dev->irq, fpsensor_irq, NULL,
                                      irqf, FPSENSOR_DEV_NAME, fpsensor_dev);
        if (retval == 0) {
            fpsensor_debug(INFO_LOG, " irq thread reqquest success!\n");
        } else {
            fpsensor_debug(ERR_LOG, " irq thread request failed , retval =%d \n", retval);
            break;
        }
        enable_irq_wake(g_fpsensor->irq);
	//	fpsensor_balance_spi_clk = 0;
        fpsensor_dev->device_available = 1;
        // fix Unbalanced enable for IRQ, disable irq at first
        fpsensor_dev->irq_enabled = 1;
        fpsensor_disable_irq(fpsensor_dev);
        fpsensor_debug(INFO_LOG, "%s: fpsensor init finished======\n", __func__);
        break;

    case FPSENSOR_IOC_EXIT:
        fpsensor_disable_irq(fpsensor_dev);
        if (fpsensor_dev->irq) {
            free_irq(fpsensor_dev->irq, fpsensor_dev);
            fpsensor_dev->irq_enabled = 0;
        }
        fpsensor_dev->device_available = 0;
        fpsensor_debug(INFO_LOG, "%s: fpsensor exit finished======\n", __func__);
        break;

    case FPSENSOR_IOC_RESET:
        fpsensor_debug(INFO_LOG, "%s: chip reset command\n", __func__);
        fpsensor_hw_reset(1250);
        break;

    case FPSENSOR_IOC_ENABLE_IRQ:
        fpsensor_debug(INFO_LOG, "%s: chip ENable IRQ command\n", __func__);
        fpsensor_enable_irq(fpsensor_dev);
        break;

    case FPSENSOR_IOC_DISABLE_IRQ:
        fpsensor_debug(INFO_LOG, "%s: chip disable IRQ command\n", __func__);
        fpsensor_disable_irq(fpsensor_dev);
        break;
    case FPSENSOR_IOC_GET_INT_VAL:
        val = gpio_get_value(fpsensor_dev->irq_gpio);
        if (copy_to_user((void __user *)arg, (void *)&val, sizeof(uint32_t))) {
            fpsensor_debug(ERR_LOG, "Failed to copy data to user\n");
            retval = -EFAULT;
            break;
        }
        retval = 0;
        break;
    case FPSENSOR_IOC_ENABLE_SPI_CLK:
        fpsensor_debug(INFO_LOG, "%s: ENABLE_SPI_CLK ======\n", __func__);
        fpsensor_spi_clk_enable(1);
        break;
    case FPSENSOR_IOC_DISABLE_SPI_CLK:
        fpsensor_debug(INFO_LOG, "%s: DISABLE_SPI_CLK ======\n", __func__);
        fpsensor_spi_clk_enable(0);
        break;
    case FPSENSOR_IOC_ENABLE_POWER:
        fpsensor_debug(INFO_LOG, "%s: FPSENSOR_IOC_ENABLE_POWER ======\n", __func__);
        break;
    case FPSENSOR_IOC_DISABLE_POWER:
        fpsensor_debug(INFO_LOG, "%s: FPSENSOR_IOC_DISABLE_POWER ======\n", __func__);
        break;
    case FPSENSOR_IOC_REMOVE:
	
		if(fpsensor_dev->pinctrl1 != NULL){
			devm_pinctrl_put(fpsensor_dev->pinctrl1);
			fpsensor_dev->pinctrl1 = NULL;
		}
        fpsensor_disable_irq(fpsensor_dev);
        if (fpsensor_dev->irq) {
            free_irq(fpsensor_dev->irq, fpsensor_dev);
            fpsensor_dev->irq_enabled = 0;
        }
		if(gpio_is_valid(fpsensor_dev->irq_gpio))
		{
			gpio_free(fpsensor_dev->irq_gpio);
		}		
        fpsensor_dev->device_available = 0;
        fpsensor_dev_cleanup(fpsensor_dev);
#if FP_NOTIFY
        fb_unregister_client(&fpsensor_dev->notifier);
#endif
        fpsensor_dev->free_flag = 1;
		wake_lock_destroy(&fpsensor_dev->ttw_wl);
		if(fpsensor_dev!=NULL){
			//kfree(fpsensor_dev);
			//fpsensor_dev = NULL;
		}
        fpsensor_debug(INFO_LOG, "%s remove finished\n", __func__);
        break;
    case FPSENSOR_IOC_CANCEL_WAIT:
        fpsensor_debug(INFO_LOG, "%s: FPSENSOR CANCEL WAIT\n", __func__);
        wake_up_interruptible(&fpsensor_dev->wq_irq_return);
        fpsensor_dev->cancel = 1;
        break;
#if FP_NOTIFY
    case FPSENSOR_IOC_GET_FP_STATUS :
        val = fpsensor_dev->fb_status;
        fpsensor_debug(INFO_LOG, "%s: FPSENSOR_IOC_GET_FP_STATUS  %d \n",__func__, fpsensor_dev->fb_status);
        if (copy_to_user((void __user *)arg, (void *)&val, sizeof(uint32_t))) {
            fpsensor_debug(ERR_LOG, "Failed to copy data to user\n");
            retval = -EFAULT;
            break;
        }
        retval = 0;
        break;
#endif
    case FPSENSOR_IOC_ENABLE_REPORT_BLANKON:
        if (copy_from_user(&val, (void __user *)arg, sizeof(uint32_t))) {
            retval = -EFAULT;
            break;
        }
        fpsensor_dev->enable_report_blankon = val;
        fpsensor_debug(INFO_LOG, "%s: FPSENSOR_IOC_ENABLE_REPORT_BLANKON: %d\n", __func__, val);
        break;
    case FPSENSOR_IOC_UPDATE_DRIVER_SN:
        if (copy_from_user(&g_cmd_sn, (void __user *)arg, sizeof(uint32_t))) {
            fpsensor_debug(ERR_LOG, "Failed to copy g_cmd_sn from user to kernel\n");
            retval = -EFAULT;
            break;
        }
        //fpsensor_debug(INFO_LOG, "%s: FPSENSOR_IOC_UPDATE_DRIVER_SN: %d\n", __func__, g_cmd_sn);
        break;
    default:
        fpsensor_debug(ERR_LOG, "fpsensor doesn't support this command(0x%x)\n", cmd);
        break;
    }

    //FUNC_EXIT();
    return retval;
}

static long fpsensor_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    return fpsensor_ioctl(filp, cmd, (unsigned long)(arg));
}

static unsigned int fpsensor_poll(struct file *filp, struct poll_table_struct *wait)
{
    unsigned int ret = 0;

    ret |= POLLIN;
    poll_wait(filp, &g_fpsensor->wq_irq_return, wait);
    if (g_fpsensor->cancel == 1) {
        fpsensor_debug(ERR_LOG, " cancle\n");
        ret =  POLLERR;
        g_fpsensor->cancel = 0;
        return ret;
    }

    if ( g_fpsensor->RcvIRQ) {
        if (g_fpsensor->RcvIRQ == 2) {
            fpsensor_debug(ERR_LOG, " get fp on notify\n");
            ret |= POLLHUP;
        } else {
            fpsensor_debug(ERR_LOG, " get irq\n");
            ret |= POLLRDNORM;
        }
    } else {
        ret = 0;
    }
    return ret;
}

static int fpsensor_open(struct inode *inode, struct file *filp)
{
    fpsensor_data_t *fpsensor_dev;

    FUNC_ENTRY();
    fpsensor_dev = container_of(inode->i_cdev, fpsensor_data_t, cdev);
    fpsensor_dev->users++;
    fpsensor_dev->device_available = 1;
    filp->private_data = fpsensor_dev;
    FUNC_EXIT();
    return 0;
}

static int fpsensor_release(struct inode *inode, struct file *filp)
{
    fpsensor_data_t *fpsensor_dev;
    int    status = 0;

    FUNC_ENTRY();
    fpsensor_dev = filp->private_data;
    filp->private_data = NULL;

    /*last close??*/
    fpsensor_dev->users--;
    if (fpsensor_dev->users <= 0) {
        fpsensor_debug(INFO_LOG, "%s, disble_irq. irq = %d\n", __func__, fpsensor_dev->irq);
        fpsensor_disable_irq(fpsensor_dev);
    }
    fpsensor_dev->device_available = 0;
    FUNC_EXIT();
    return status;
}

static ssize_t fpsensor_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    fpsensor_debug(ERR_LOG, "Not support read opertion in TEE version\n");
    return -EFAULT;
}

static ssize_t fpsensor_write(struct file *filp, const char __user *buf, size_t count,
                              loff_t *f_pos)
{
    fpsensor_debug(ERR_LOG, "Not support write opertion in TEE version\n");
    return -EFAULT;
}

static const struct file_operations fpsensor_fops = {
    .owner          = THIS_MODULE,
    .write          = fpsensor_write,
    .read           = fpsensor_read,
    .unlocked_ioctl = fpsensor_ioctl,
    .compat_ioctl   = fpsensor_compat_ioctl,
    .open           = fpsensor_open,
    .release        = fpsensor_release,
    .poll           = fpsensor_poll,

};

// create and register a char device for fpsensor
static int fpsensor_dev_setup(fpsensor_data_t *fpsensor)
{
    int ret = 0;
    dev_t dev_no = 0;
    struct device *dev = NULL;
    int fpsensor_dev_major = FPSENSOR_DEV_MAJOR;
    int fpsensor_dev_minor = 0;

    FUNC_ENTRY();

    if (fpsensor_dev_major) {
        dev_no = MKDEV(fpsensor_dev_major, fpsensor_dev_minor);
        ret = register_chrdev_region(dev_no, FPSENSOR_NR_DEVS, FPSENSOR_DEV_NAME);
    } else {
        ret = alloc_chrdev_region(&dev_no, fpsensor_dev_minor, FPSENSOR_NR_DEVS, FPSENSOR_DEV_NAME);
        fpsensor_dev_major = MAJOR(dev_no);
        fpsensor_dev_minor = MINOR(dev_no);
        fpsensor_debug(INFO_LOG, "fpsensor device major is %d, minor is %d\n",
                       fpsensor_dev_major, fpsensor_dev_minor);
    }

    if (ret < 0) {
        fpsensor_debug(ERR_LOG, "can not get device major number %d\n", fpsensor_dev_major);
        goto out;
    }

    cdev_init(&fpsensor->cdev, &fpsensor_fops);
    fpsensor->cdev.owner = THIS_MODULE;
    fpsensor->cdev.ops   = &fpsensor_fops;
    fpsensor->devno      = dev_no;
    ret = cdev_add(&fpsensor->cdev, dev_no, FPSENSOR_NR_DEVS);
    if (ret) {
        fpsensor_debug(ERR_LOG, "add char dev for fpsensor failed\n");
        goto release_region;
    }

    fpsensor->class = class_create(THIS_MODULE, FPSENSOR_CLASS_NAME);
    if (IS_ERR(fpsensor->class)) {
        fpsensor_debug(ERR_LOG, "create fpsensor class failed\n");
        ret = PTR_ERR(fpsensor->class);
        goto release_cdev;
    }

    dev = device_create(fpsensor->class, &fpsensor->spi->dev, dev_no, fpsensor, FPSENSOR_DEV_NAME);
    if (IS_ERR(dev)) {
        fpsensor_debug(ERR_LOG, "create device for fpsensor failed\n");
        ret = PTR_ERR(dev);
        goto release_class;
    }
    FUNC_EXIT();
    return ret;

release_class:
    class_destroy(fpsensor->class);
    fpsensor->class = NULL;
release_cdev:
    cdev_del(&fpsensor->cdev);
release_region:
    unregister_chrdev_region(dev_no, FPSENSOR_NR_DEVS);
out:
    FUNC_EXIT();
    return ret;
}

#if FP_NOTIFY
static int fpsensor_fb_notifier_callback(struct notifier_block* self, unsigned long event, void* data)
{
    int retval = 0;
    static char screen_status[64] = { '\0' };
    struct fb_event* evdata = data;
    unsigned int blank;
    fpsensor_data_t *fpsensor_dev = g_fpsensor;

    fpsensor_debug(INFO_LOG,"%s enter.  event : 0x%x\n", __func__, (unsigned)event);
    if (event != FB_EVENT_BLANK /* FB_EARLY_EVENT_BLANK */) {
        return 0;
    }

    blank = *(int*)evdata->data;
    fpsensor_debug(INFO_LOG,"%s enter, blank=0x%x\n", __func__, blank);

    switch (blank) {
    case FB_BLANK_UNBLANK:
        fpsensor_debug(INFO_LOG,"%s: lcd on notify\n", __func__);
        sprintf(screen_status, "SCREEN_STATUS=%s", "ON");
        fpsensor_dev->fb_status = 1;
        if( fpsensor_dev->enable_report_blankon) {
            fpsensor_dev->RcvIRQ = 2;
            wake_up_interruptible(&fpsensor_dev->wq_irq_return);
        }
        break;

    case FB_BLANK_POWERDOWN:
        fpsensor_debug(INFO_LOG,"%s: lcd off notify\n", __func__);
        sprintf(screen_status, "SCREEN_STATUS=%s", "OFF");
        fpsensor_dev->fb_status = 0;
        break;

    default:
        fpsensor_debug(INFO_LOG,"%s: other notifier, ignore\n", __func__);
        break;
    }

    fpsensor_debug(INFO_LOG,"%s %s leave.\n", screen_status, __func__);
    return retval;
}
#endif

#if defined(CHIPONE_COMPATIBLE_BEANPOD_V2_5_1) || defined(CHIPONE_COMPATIBLE_BEANPOD_V2_8_0)
#if 0
// There is not mt_chip_conf struct in MTK6739.
struct mt_chip_conf fpsensor_spi_conf_mt65xx = {
    .setuptime = 15,
    .holdtime = 15,
    .high_time = 21, //for mt6582, 104000khz/(4+4) = 130000khz
    .low_time = 21,
    .cs_idletime = 20,
    .ulthgh_thrsh = 0,

    .cpol = 0,
    .cpha = 0,

    .rx_mlsb = 1,
    .tx_mlsb = 1,

    .tx_endian = 0,
    .rx_endian = 0,

    .com_mod = FIFO_TRANSFER,
    .pause = 1,
    .finish_intr = 1,
    .deassert = 0,
    .ulthigh = 0,
    .tckdly = 0,
};
#endif

typedef enum {
    SPEED_500KHZ = 500,
    SPEED_1MHZ = 1000,
    SPEED_2MHZ = 2000,
    SPEED_3MHZ = 3000,
    SPEED_4MHZ = 4000,
    SPEED_6MHZ = 6000,
    SPEED_8MHZ = 8000,
    SPEED_KEEP,
    SPEED_UNSUPPORTED
} SPI_SPEED;

void fpsensor_spi_set_mode(struct spi_device *spi, SPI_SPEED speed, int flag)
{
#if 0
    struct mt_chip_conf *mcc = &fpsensor_spi_conf_mt65xx;
    if (flag == 0) {
        mcc->com_mod = FIFO_TRANSFER;
    } else {
        mcc->com_mod = DMA_TRANSFER;
    }

    switch (speed) {
    case SPEED_500KHZ:
        mcc->high_time = 120;
        mcc->low_time = 120;
        break;
    case SPEED_1MHZ:
        mcc->high_time = 60;
        mcc->low_time = 60;
        break;
    case SPEED_2MHZ:
        mcc->high_time = 30;
        mcc->low_time = 30;
        break;
    case SPEED_3MHZ:
        mcc->high_time = 20;
        mcc->low_time = 20;
        break;
    case SPEED_4MHZ:
        mcc->high_time = 15;
        mcc->low_time = 15;
        break;
    case SPEED_6MHZ:
        mcc->high_time = 10;
        mcc->low_time = 10;
        break;
    case SPEED_8MHZ:
        mcc->high_time = 8;
        mcc->low_time = 8;
        break;
    case SPEED_KEEP:
    case SPEED_UNSUPPORTED:
        break;
    }
#endif
    if (spi_setup(spi) < 0) {
        fpsensor_debug(ERR_LOG, "fpsensor:Failed to set spi.\n");
    }
}

/* -------------------------------------------------------------------- */
int fpsensor_spi_setup(fpsensor_data_t *fpsensor)
{
    int error = 0;

    FUNC_ENTRY();
    fpsensor->spi->mode = SPI_MODE_0;
    fpsensor->spi->bits_per_word = 8;
    //fpsensor->spi->chip_select = 0;
    //fpsensor->spi->controller_data = (void *)&fpsensor_spi_conf_mt65xx;
    spi_setup(fpsensor->spi);
    if (error) {
        fpsensor_debug(ERR_LOG, "spi_setup failed\n");
        goto out_err;
    }
    fpsensor_spi_set_mode(fpsensor->spi, fpsensor->spi_freq_khz, 0);

out_err:
    FUNC_EXIT() ;
    return error;
}

int fpsensor_spi_send_recv(fpsensor_data_t *fpsensor, size_t len , u8 *tx_buffer, u8 *rx_buffer)
{
    struct spi_message msg;
    struct spi_transfer cmd = {
        .cs_change = 0,
        .delay_usecs = 0,
        .speed_hz = (u32)fpsensor->spi_freq_khz * 1000u,
        .tx_buf = tx_buffer,
        .rx_buf = rx_buffer,
        .len    = len,
        .tx_dma = 0,
        .rx_dma = 0,
        .bits_per_word = 0,
    };
    int error = 0 ;
    FUNC_ENTRY();
#if FPSENSOR_MANUAL_CS
    fpsensor_gpio_wirte(FPSENSOR_SPI_CS_PIN, 0);
#endif
    fpsensor_spi_set_mode(fpsensor->spi, (u32)fpsensor->spi_freq_khz, 0);
    spi_message_init(&msg);
    spi_message_add_tail(&cmd,  &msg);
    spi_sync(fpsensor->spi, &msg);
    if (error) {
        fpsensor_debug(ERR_LOG, "spi_sync failed.\n");
    }
    fpsensor_debug(DEBUG_LOG, "tx_len : %d \n", (int)len);
    fpsensor_debug(DEBUG_LOG, "tx_buf : %x  %x  %x  %x  %x  %x\n",
                   (len > 0) ? tx_buffer[0] : 0,
                   (len > 1) ? tx_buffer[1] : 0,
                   (len > 2) ? tx_buffer[2] : 0,
                   (len > 3) ? tx_buffer[3] : 0,
                   (len > 4) ? tx_buffer[4] : 0,
                   (len > 5) ? tx_buffer[5] : 0);
    fpsensor_debug(DEBUG_LOG, "rx_buf : %x  %x  %x  %x  %x  %x\n",
                   (len > 0) ? rx_buffer[0] : 0,
                   (len > 1) ? rx_buffer[1] : 0,
                   (len > 2) ? rx_buffer[2] : 0,
                   (len > 3) ? rx_buffer[3] : 0,
                   (len > 4) ? rx_buffer[4] : 0,
                   (len > 5) ? rx_buffer[5] : 0);
#if FPSENSOR_MANUAL_CS
    fpsensor_gpio_wirte(FPSENSOR_SPI_CS_PIN, 1);
#endif
    FUNC_EXIT() ;
    return error;
}


int fpsensor_check_HWID(fpsensor_data_t *fpsensor)
{
    unsigned int hwid = 0;
    unsigned char  tx[4] = {0};
    unsigned char  rx[4] = {0};
    int match = 0;
    tx[0] = 0x08;
    tx[1] = 0x55;
    fpsensor_spi_send_recv(fpsensor, 2, tx, rx);

    tx[0] = 0x00;
    rx[1] = 0x00;
    rx[2] = 0x00;
    fpsensor_spi_send_recv(fpsensor, 3, tx, rx);
    hwid = ((rx[1] << 8) | (rx[2]));
    fpsensor_debug(ERR_LOG,"HWID 0x%x .\n",hwid);
    if ((hwid == 0x7332) || (hwid == 0x7153) || (hwid == 0x7230) ||(hwid == 0x7222)) {
        match = 1;
    }
    return match;
}
#endif

#if defined(USE_SPI_BUS)
static int fpsensor_probe(struct spi_device *spi)
#elif defined(USE_PLATFORM_BUS)
static int fpsensor_probe(struct platform_device *spi)
#endif
{
    int status = 0;
    fpsensor_data_t *fpsensor_dev = NULL;
#if defined(CHIPONE_COMPATIBLE_BEANPOD_V2_5_1)
	char fp_name[]="fp_server_chipone";
	uint8_t len_name = sizeof(fp_name)/sizeof(char);
	int id_match = 0,count = 0,error = 0;
#elif defined(CHIPONE_COMPATIBLE_BEANPOD_V2_8_0)
	int id_match = 0,count = 0;
#endif

    FUNC_ENTRY();

    /* Allocate driver data */
    fpsensor_dev = kzalloc(sizeof(*fpsensor_dev), GFP_KERNEL);
    if (!fpsensor_dev) {
        status = -ENOMEM;
        fpsensor_debug(ERR_LOG, "%s, Failed to alloc memory for fpsensor device.\n", __func__);
        goto out;
    }

    /* Initialize the driver data */
    g_fpsensor = fpsensor_dev;
    fpsensor_dev->spi               = spi ;
    fpsensor_dev->device_available  = 0;
    fpsensor_dev->users             = 0;
    fpsensor_dev->irq               = 0;
    fpsensor_dev->irq_gpio          = 0;
    fpsensor_dev->irq_enabled       = 0;
    fpsensor_dev->free_flag         = 0;
	
    spi->dev.of_node = of_find_compatible_node(NULL, NULL, "mediatek,icnfpsensor");
    fpsensor_dev->pinctrl1 = devm_pinctrl_get(&spi->dev);
    if (IS_ERR(fpsensor_dev->pinctrl1)) {
        status = PTR_ERR(fpsensor_dev->pinctrl1);
        fpsensor_debug(ERR_LOG, "fpsensor Cannot find fp pinctrl1.\n");
        goto release_drv_data;
    }
	fpsensor_spidev_dts_init(fpsensor_dev);

#if defined(CHIPONE_COMPATIBLE_BEANPOD_V2_5_1)
    fpsensor_dev->spi_freq_khz = 6000u;
    fpsensor_spi_clk_enable(1);
	fpsensor_spi_setup(fpsensor_dev);
    error = get_fp_spi_enable();
    if(error == 1){
        fpsensor_hw_reset(1250);
        for(count = 0; count < 5;count++ ){
            id_match = fpsensor_check_HWID(fpsensor_dev);
            if(1 == id_match)
                break;
            udelay(300);    
        }
        if (0 == id_match) {
            fpsensor_debug(ERR_LOG, "fpsensor faild to get chip id in linux driver.\n");
            fpsensor_spi_clk_enable(0);
            goto release_drv_data;
        }
        
        fpsensor_debug(DEBUG_LOG,"fpsensor succeeded in matching chip id in linux driver --FP_VENDOR_CHIPONE--.\n");
        //set_fp_vendor(FP_VENDOR_CHIPONE);    
         set_fp_ta_name(fp_name,len_name);
    } 
	else {
        fpsensor_debug(ERR_LOG, "fpsensor failed to get_fp_spi_enable from beanpod.\n");
        fpsensor_spi_clk_enable(0);
        goto release_drv_data;  
    }
    fpsensor_spi_clk_enable(0);
#elif defined(CHIPONE_COMPATIBLE_BEANPOD_V2_8_0)
	fpsensor_dev->spi_freq_khz = 6000u;
    fpsensor_spi_clk_enable(1);
    fpsensor_spi_setup(fpsensor_dev);
	fpsensor_hw_reset(1250);
	for(count = 0; count < 5;count++ ){
		id_match = fpsensor_check_HWID(fpsensor_dev);
		if(1 == id_match)
			break;
		udelay(300);	
	}
	if (0 == id_match) {
		fpsensor_debug(ERR_LOG, "fpsensor faild to get chip id in linux driver.\n");
        fpsensor_spi_clk_enable(0);
		goto release_drv_data;
	}
		
	memcpy(&uuid_fp , &chipone_vendor_uuid , sizeof(struct TEEC_UUID));
	fpsensor_debug(DEBUG_LOG,"fpsensor succeeded in matching chip id in linux driver --FP_VENDOR_CHIPONE--.\n");
	fpsensor_spi_clk_enable(0);
#endif

    /* setup a char device for fpsensor */
    status = fpsensor_dev_setup(fpsensor_dev);
    if (status) {
        fpsensor_debug(ERR_LOG, "fpsensor setup char device failed, %d", status);
        goto release_drv_data;
    }

    fpsensor_spi_clk_enable(1);
    init_waitqueue_head(&fpsensor_dev->wq_irq_return);
    wake_lock_init(&g_fpsensor->ttw_wl, WAKE_LOCK_SUSPEND, "fpsensor_ttw_wl");
    fpsensor_dev->device_available = 1;
#if FP_NOTIFY
    fpsensor_dev->notifier.notifier_call = fpsensor_fb_notifier_callback;
    fb_register_client(&fpsensor_dev->notifier);
#endif

//prize add by lipengpeng 20200811 start
#if defined(CONFIG_PRIZE_HARDWARE_INFO)
        sprintf(current_fingerprint_info.chip,"ICNF7232AL");
        //strcpy(current_fingerprint_info.id, GF_LINUX_VERSION);
        strcpy(current_fingerprint_info.vendor,"chipone");
        strcpy(current_fingerprint_info.more,"fingerprint");
#endif
//prize add by lipengpeng 20200811 end

    fpsensor_debug(INFO_LOG, "%s finished, driver version: %s\n", __func__, FPSENSOR_SPI_VERSION);
    goto out;

release_drv_data:
	if(fpsensor_dev!=NULL){
	    kfree(fpsensor_dev);
	    fpsensor_dev = NULL;
	}
  if(fpsensor_dev->pinctrl1 != NULL){
      devm_pinctrl_put(fpsensor_dev->pinctrl1);
      fpsensor_dev->pinctrl1 = NULL;
  }
  fpsensor_debug(DEBUG_LOG,"fpsensor goto release_drv_data in driver\n");

out:
    FUNC_EXIT();
    return status;
}

#if defined(USE_SPI_BUS)
static int fpsensor_remove(struct spi_device *spi)
#elif defined(USE_PLATFORM_BUS)
static int fpsensor_remove(struct platform_device *spi)
#endif
{
    fpsensor_data_t *fpsensor_dev = g_fpsensor;

    FUNC_ENTRY();
    fpsensor_disable_irq(fpsensor_dev);
    if (fpsensor_dev->irq)
        free_irq(fpsensor_dev->irq, fpsensor_dev);
#if FP_NOTIFY
    fb_unregister_client(&fpsensor_dev->notifier);
#endif
    fpsensor_dev_cleanup(fpsensor_dev);
    wake_lock_destroy(&fpsensor_dev->ttw_wl);
	if(fpsensor_dev!=NULL){
		//kfree(fpsensor_dev);
		//fpsensor_dev = NULL;
		//g_fpsensor = NULL;
	}
    FUNC_EXIT();
    return 0;
}

#ifdef CONFIG_OF
static struct of_device_id fpsensor_of_match[] = {
    { .compatible = "prize,fingerprint", },
    {}
};
MODULE_DEVICE_TABLE(of, fpsensor_of_match);
#endif

#if defined(FPSENSOR_SPI_BUS_DYNAMIC)
static struct spi_board_info spi_board_devs[] __initdata = {
    [0] = {
        .modalias = FPSENSOR_DEV_NAME,
        .bus_num = 0,
        .chip_select = 0,
        .mode = SPI_MODE_0,
//		#ifndef CONFIG_SPI_MT65XX
//        .controller_data = &fpsensor_spi_conf_mt65xx, //&spi_conf
//	    #endif
    },
};
#endif

#if defined(USE_SPI_BUS)
static struct spi_driver fpsensor_spi_driver = {
    .driver = {
        .name = FPSENSOR_DEV_NAME,
        .bus = &spi_bus_type,
        .owner = THIS_MODULE,
#ifdef CONFIG_OF
        .of_match_table = of_match_ptr(fpsensor_of_match),
#endif
    },
    .probe = fpsensor_probe,
    .remove = fpsensor_remove,
};
#elif defined(USE_PLATFORM_BUS)
static struct platform_driver fpsensor_plat_driver = {
    .driver = {
        .name = FPSENSOR_DEV_NAME,
        .bus    = &platform_bus_type,
        .owner = THIS_MODULE,
#ifdef CONFIG_OF
        .of_match_table = of_match_ptr(fpsensor_of_match),
#endif
    },
    .probe = fpsensor_probe,
    .remove = fpsensor_remove,

};
#endif

static int __init fpsensor_init(void)
{
    int status;
#if defined(FPSENSOR_SPI_BUS_DYNAMIC)
	spi_register_board_info(spi_board_devs, ARRAY_SIZE(spi_board_devs));
#endif
#if defined(USE_PLATFORM_BUS)
    status = platform_driver_register(&fpsensor_plat_driver);
#elif defined(USE_SPI_BUS)
    status = spi_register_driver(&fpsensor_spi_driver);
#endif
    if (status < 0) {
        fpsensor_debug(ERR_LOG, "%s, Failed to register TEE driver.\n", __func__);
    }

    return status;
}
module_init(fpsensor_init);

static void __exit fpsensor_exit(void)
{
#if defined(USE_PLATFORM_BUS)
    platform_driver_unregister(&fpsensor_plat_driver);
#elif defined(USE_SPI_BUS)
    spi_unregister_driver(&fpsensor_spi_driver);
#endif
}
module_exit(fpsensor_exit);

MODULE_AUTHOR("xhli");
MODULE_DESCRIPTION(" Fingerprint chip TEE driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:fpsensor-drivers");
