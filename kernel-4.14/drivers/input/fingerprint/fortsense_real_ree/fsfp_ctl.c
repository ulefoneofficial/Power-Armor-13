/**
 * The device control driver for Fortsense's fingerprint sensor.
 *
 * Copyright (C) 2018 Fortsense Corporation. <http://www.fortsense.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 * Public License for more details.
**/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/spi/spi.h>

#include "fsfp_ctl.h"

#if SF_BEANPOD_COMPATIBLE_V1
#include "nt_smc_call.h"
#endif

#if SF_INT_TRIG_HIGH
#include <linux/irq.h>
#endif

#ifdef CONFIG_RSEE
#include <linux/tee_drv.h>
#endif

/* prize added by chenjiaxi, fortsense hardware info, 20210420-start */
#if defined(CONFIG_PRIZE_HARDWARE_INFO)
#include "../../../misc/mediatek/prize/hardware_info/hardware_info.h"
extern struct hardware_info current_fingerprint_info;
#endif
/* prize added by chenjiaxi, fortsense hardware info, 20210420-end */

//---------------------------------------------------------------------------------
#define SF_DRV_VERSION "v2.3.2-2020-03-19"

#define MODULE_NAME "fortsense-fsfp_ctl"
#define xprintk(level, fmt, args...) printk(level MODULE_NAME"-%d: "fmt, __LINE__, ##args)

#if SF_TRUSTKERNEL_COMPAT_SPI_MT65XX
#define SPI_MODULE_CLOCK      (120 * 1000 * 1000)
#elif defined(CONFIG_MTK_SPI)
#define SPI_MODULE_CLOCK      (100 * 1000 * 1000)
#endif

#define SF_DEFAULT_SPI_SPEED  (1 * 1000 * 1000)
//---------------------------------------------------------------------------------

//---------------------------------------------------------------------------------
static long fsfp_ctl_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static int fsfp_ctl_init_irq(void);
static int fsfp_ctl_init_input(void);
#ifdef CONFIG_COMPAT
static long fsfp_ctl_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
#endif
#if SF_REE_PLATFORM
static ssize_t fsfp_ctl_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
static ssize_t fsfp_ctl_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos);
#endif
#if (SF_PROBE_ID_EN && !SF_RONGCARD_COMPATIBLE)
static int fsfp_read_sensor_id(void);
#endif

extern int fsfp_platform_init(struct fsfp_ctl_device *ctl_dev);
extern void fsfp_platform_exit(struct fsfp_ctl_device *ctl_dev);

extern int fb_blank(struct fb_info *info, int blank);

//---------------------------------------------------------------------------------

#if QUALCOMM_REE_DEASSERT
static int qualcomm_deassert = 0;
#endif

#ifdef CONFIG_RSEE
int rsee_client_get_fpid(int *vendor_id);
#endif
static struct file_operations fsfp_ctl_fops = {
    .owner          = THIS_MODULE,
    .unlocked_ioctl = fsfp_ctl_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl   = fsfp_ctl_compat_ioctl,
#endif
#if SF_REE_PLATFORM
    .read           = fsfp_ctl_read,
    .write          = fsfp_ctl_write,
#endif
};

static struct fsfp_ctl_device fsfp_ctl_dev = {
    .miscdev = {
        .minor  = MISC_DYNAMIC_MINOR,
        .name   = "fortsense_fp",
        .fops   = &fsfp_ctl_fops,
    },
    .rst_num = 0,
    .irq_pin = 0,
    .irq_num = 0,
    .spi_buf_size = 25 * 1024,
};

#if SF_REG_DEVICE_BY_DRIVER
static struct platform_device *fsfp_device = NULL;
#endif

#if (defined(CONFIG_MTK_SPI) || SF_TRUSTKERNEL_COMPAT_SPI_MT65XX)
#define SF_DEFAULT_SPI_HALF_CYCLE_TIME  ((SPI_MODULE_CLOCK / SF_DEFAULT_SPI_SPEED) / 2)
#define SF_DEFAULT_SPI_HIGH_TIME   (((SPI_MODULE_CLOCK / SF_DEFAULT_SPI_SPEED) % 2 == 0) ? \
                                   SF_DEFAULT_SPI_HALF_CYCLE_TIME : (SF_DEFAULT_SPI_HALF_CYCLE_TIME + 1))
#define SF_DEFAULT_SPI_LOW_TIME    SF_DEFAULT_SPI_HALF_CYCLE_TIME

static struct mt_chip_conf smt_conf = {
    .setuptime = 15,
    .holdtime = 15,
    .high_time = SF_DEFAULT_SPI_HIGH_TIME, // 10--6m 15--4m 20--3m 30--2m [ 60--1m 120--0.5m  300--0.2m]
    .low_time  = SF_DEFAULT_SPI_LOW_TIME,
    .cs_idletime = 20,
    .ulthgh_thrsh = 0,
    .cpol = SPI_CPOL_0,
    .cpha = SPI_CPHA_0,
    .rx_mlsb = SPI_MSB,
    .tx_mlsb = SPI_MSB,
    .tx_endian = 0,
    .rx_endian = 0,
    .com_mod = FIFO_TRANSFER,
    .pause = 0,
    .finish_intr = 1,
    .deassert = 0,
    .ulthigh = 0,
    .tckdly = 0,
};
#endif

static int fsfp_remove(fsfp_device_t *pdev);
static int fsfp_probe(fsfp_device_t *pdev);

#if SF_SPI_RW_EN
static struct of_device_id  fsfp_of_match[] = {
    { .compatible = COMPATIBLE_SW_FP, },
	{ .compatible = "prize,fingerprint", }, //clw add
    // {},
};

static struct spi_board_info spi_board_devs[] __initdata = {
    [0] = {
        .modalias = "fortsense-fp",
        .bus_num = 0,
        .chip_select = 0,
        .mode = SPI_MODE_0,
    },
};

static int fsfp_ctl_spi_speed(unsigned int speed)
{
#ifdef CONFIG_MTK_SPI
    unsigned int time;
    time = SPI_MODULE_CLOCK / speed;
    fsfp_ctl_dev.mt_conf.high_time = time / 2;
    fsfp_ctl_dev.mt_conf.low_time  = time / 2;

    if ((time % 2) != 0) {
        fsfp_ctl_dev.mt_conf.high_time += 1;
    }

#elif (SF_REE_PLATFORM && QUALCOMM_REE_DEASSERT)
    double delay_ns = 0;

    if (speed <= 1000 * 1000) {
        speed = 0.96 * 1000 * 1000; //0.96M
        qualcomm_deassert = 0;
    }
    else if (speed <= 4800 * 1000) {
        delay_ns = (1.0 / (((double)speed) / (1.0 * 1000 * 1000)) - 1.0 / 4.8) * 8 * 1000;
        speed = 4.8 * 1000 * 1000; //4.8M
        qualcomm_deassert = 3;
    }
    else {
        delay_ns = (1.0 / (((double)speed) / (1.0 * 1000 * 1000)) - 1.0 / 9.6) * 8 * 1000;
        speed = 9.6 * 1000 * 1000; //9.6M
        qualcomm_deassert = 10;
    }

    xprintk(KERN_INFO, "need delay_ns = xxx, qualcomm_deassert = %d(maybe custom).\n", qualcomm_deassert);
#endif
    fsfp_ctl_dev.pdev->max_speed_hz = speed;
    spi_setup(fsfp_ctl_dev.pdev);
    return 0;
}
#endif

static fsfp_driver_t fsfp_driver = {
    .driver = {
        .name = "fortsense-fp",
#if SF_SPI_RW_EN
        .bus = &spi_bus_type,
#endif
        .owner = THIS_MODULE,
#ifdef CONFIG_OF
#if SF_SPI_RW_EN
        .of_match_table = fsfp_of_match,
#endif
#endif
    },
    .probe  = fsfp_probe,
    .remove = fsfp_remove,
};

static fsfp_version_info_t fsfp_hw_ver;


//---------------------------------------------------------------------------------
#if SF_INT_TRIG_HIGH
static int fsfp_ctl_set_irq_type(unsigned long type)
{
    int err = 0;

    if (fsfp_ctl_dev.irq_num > 0) {
        err = irq_set_irq_type(fsfp_ctl_dev.irq_num, type | IRQF_NO_SUSPEND | IRQF_ONESHOT);
    }

    return err;
}
#endif

static void fsfp_ctl_device_event(struct work_struct *ws)
{
    char *uevent_env[2] = { SF_INT_EVENT_NAME, NULL };
    xprintk(SF_LOG_LEVEL, "%s(..) enter.\n", __FUNCTION__);
    kobject_uevent_env(&fsfp_ctl_dev.miscdev.this_device->kobj,
                       KOBJ_CHANGE, uevent_env);
}

static irqreturn_t fsfp_ctl_device_irq(int irq, void *dev_id)
{
    disable_irq_nosync(irq);
    xprintk(SF_LOG_LEVEL, "%s(irq = %d, ..) toggled.\n", __FUNCTION__, irq);
    schedule_work(&fsfp_ctl_dev.work_queue);
#ifdef CONFIG_PM_WAKELOCKS
    __pm_wakeup_event(&fsfp_ctl_dev.wakelock, msecs_to_jiffies(5000));
#else
    wake_lock_timeout(&fsfp_ctl_dev.wakelock, msecs_to_jiffies(5000));
#endif
#if SF_INT_TRIG_HIGH
    fsfp_ctl_set_irq_type(IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND | IRQF_ONESHOT);
#endif
    enable_irq(irq);
    return IRQ_HANDLED;
}

static int fsfp_ctl_report_key_event(struct input_dev *input, fsfp_key_event_t *kevent)
{
    int err = 0;
    unsigned int key_code = KEY_UNKNOWN;
    xprintk(SF_LOG_LEVEL, "%s(..) enter.\n", __FUNCTION__);

    switch (kevent->key) {
        case SF_KEY_HOME:
            key_code = KEY_HOME;
            break;

        case SF_KEY_MENU:
            key_code = KEY_MENU;
            break;

        case SF_KEY_BACK:
            key_code = KEY_BACK;
            break;

        case SF_KEY_F11:
            key_code = KEY_F11;
            break;

        case SF_KEY_ENTER:
            key_code = KEY_ENTER;
            break;

        case SF_KEY_UP:
            key_code = KEY_UP;
            break;

        case SF_KEY_LEFT:
            key_code = KEY_LEFT;
            break;

        case SF_KEY_RIGHT:
            key_code = KEY_RIGHT;
            break;

        case SF_KEY_DOWN:
            key_code = KEY_DOWN;
            break;

        case SF_KEY_WAKEUP:
            key_code = KEY_WAKEUP;
            break;

        default:
            break;
    }

    input_report_key(input, key_code, kevent->value);
    input_sync(input);
    xprintk(SF_LOG_LEVEL, "%s(..) leave.\n", __FUNCTION__);
    return err;
}

static const char *fsfp_ctl_get_version(void)
{
    static char version[SF_DRV_VERSION_LEN] = {'\0', };
    strncpy(version, SF_DRV_VERSION, SF_DRV_VERSION_LEN);
    version[SF_DRV_VERSION_LEN - 1] = '\0';
    return (const char *)version;
}

////////////////////////////////////////////////////////////////////////////////
// struct file_operations fields.

static long fsfp_ctl_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int err = 0;
    fsfp_key_event_t kevent;
    xprintk(SF_LOG_LEVEL, "%s(_IO(type,nr) nr= 0x%08x, ..)\n", __FUNCTION__, _IOC_NR(cmd));

    switch (cmd) {
        case SF_IOC_INIT_DRIVER: {
#if MULTI_HAL_COMPATIBLE
            fsfp_ctl_dev.gpio_init(&fsfp_ctl_dev);
#endif
            break;
        }

        case SF_IOC_DEINIT_DRIVER: {
#if MULTI_HAL_COMPATIBLE
            fsfp_ctl_dev.free_gpio(&fsfp_ctl_dev);
#endif
            break;
        }

        case SPI_IOC_RST:
        case SF_IOC_RESET_DEVICE: {
            fsfp_ctl_dev.reset(false);
            msleep(1);
            fsfp_ctl_dev.reset(true);
            msleep(10);
            break;
        }

        case SF_IOC_ENABLE_IRQ: {
            // TODO:
            break;
        }

        case SF_IOC_DISABLE_IRQ: {
            // TODO:
            break;
        }

        case SF_IOC_REQUEST_IRQ: {
#if MULTI_HAL_COMPATIBLE
            fsfp_ctl_init_irq();
#endif
            break;
        }

        case SF_IOC_ENABLE_SPI_CLK: {
            fsfp_ctl_dev.spi_clk_on(true);
            break;
        }

        case SF_IOC_DISABLE_SPI_CLK: {
            fsfp_ctl_dev.spi_clk_on(false);
            break;
        }

        case SF_IOC_ENABLE_POWER: {
            fsfp_ctl_dev.power_on(true);
            break;
        }

        case SF_IOC_DISABLE_POWER: {
            fsfp_ctl_dev.power_on(false);
            break;
        }

        case SF_IOC_REPORT_KEY_EVENT: {
            if (copy_from_user(&kevent, (fsfp_key_event_t *)arg, sizeof(fsfp_key_event_t))) {
                xprintk(KERN_ERR, "copy_from_user(..) failed.\n");
                err = (-EFAULT);
                break;
            }

            err = fsfp_ctl_report_key_event(fsfp_ctl_dev.input, &kevent);
            break;
        }

        case SF_IOC_SYNC_CONFIG: {
            // TODO:
            break;
        }

        case SPI_IOC_WR_MAX_SPEED_HZ:
        case SF_IOC_SPI_SPEED: {
#if SF_SPI_RW_EN
            fsfp_ctl_spi_speed(arg);
#endif
            break;
        }

        case SPI_IOC_RD_MAX_SPEED_HZ: {
            // TODO:
            break;
        }

        case FORTSENSE_IOC_ATTRIBUTE:
        case SF_IOC_ATTRIBUTE: {
            err = __put_user(fsfp_ctl_dev.attribute, (__u32 __user *)arg);
            break;
        }

        case SF_IOC_GET_VERSION: {
            if (copy_to_user((void *)arg, fsfp_ctl_get_version(), SF_DRV_VERSION_LEN)) {
                xprintk(KERN_ERR, "copy_to_user(..) failed.\n");
                err = (-EFAULT);
                break;
            }

            break;
        }

        case SF_IOC_SET_LIB_VERSION: {
            if (copy_from_user((void *)&fsfp_hw_ver, (void *)arg, sizeof(fsfp_version_info_t))) {
                xprintk(KERN_ERR, "fsfp_hw_info_t copy_from_user(..) failed.\n");
                err = (-EFAULT);
                break;
            }

			/* prize-wyq, sunwave fingerprint info, 20190420-start */
			#if defined(CONFIG_PRIZE_HARDWARE_INFO)
				snprintf(current_fingerprint_info.chip,32,"%s %s",fsfp_hw_ver.fortsense_id,fsfp_hw_ver.firmware);
				snprintf(current_fingerprint_info.id,32,"ca:%s",fsfp_hw_ver.ca_version);// sf_hw_ver.ta_version
				snprintf(current_fingerprint_info.vendor,32,"vendor_id:%s",fsfp_hw_ver.vendor_id);
				strcpy(current_fingerprint_info.more,"fortsense");
			#endif
			/* prize-wyq, sunwave fingerprint info, 20190420-end */
            break;
        }

        case SF_IOC_GET_LIB_VERSION: {
            if (copy_to_user((void *)arg, (void *)&fsfp_hw_ver, sizeof(fsfp_version_info_t))) {
                xprintk(KERN_ERR, "fsfp_hw_info_t copy_to_user(..) failed.\n");
                err = (-EFAULT);
                break;
            }

            break;
        }

        case SF_IOC_SET_SPI_BUF_SIZE: {
            fsfp_ctl_dev.spi_buf_size = arg;
            break;
        }

        case SF_IOC_SET_RESET_OUTPUT: {
            if (arg) {
                fsfp_ctl_dev.reset(true);
            }
            else {
                fsfp_ctl_dev.reset(false);
            }

            break;
        }

        default:
            err = (-EINVAL);
            break;
    }

    return err;
}

static ssize_t fortsense_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int len = 0;
    len += sprintf(buf, "%s\n", fsfp_hw_ver.driver);
    return len;
}
static DEVICE_ATTR(version, S_IRUGO | S_IWUSR, fortsense_version_show, NULL);

static ssize_t fortsense_chip_info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int len = 0;
    len += sprintf((char *)buf, "chip   : %s %s\nid     : 0x0 lib:%s\nvendor : fw:%s\nmore   : fingerprint\n",
                   fsfp_hw_ver.fortsense_id, fsfp_hw_ver.ca_version,
                   fsfp_hw_ver.algorithm,
                   fsfp_hw_ver.firmware);
    return len;
}
static DEVICE_ATTR(chip_info, S_IRUGO | S_IWUSR, fortsense_chip_info_show, NULL);

static ssize_t fsfp_show_version(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret = 0;
    ret += sprintf(buf + ret, "solution:%s\n", fsfp_hw_ver.tee_solution);
    ret += sprintf(buf + ret, "ca      :%s\n", fsfp_hw_ver.ca_version);
    ret += sprintf(buf + ret, "ta      :%s\n", fsfp_hw_ver.ta_version);
    ret += sprintf(buf + ret, "alg     :%s\n", fsfp_hw_ver.algorithm);
    ret += sprintf(buf + ret, "nav     :%s\n", fsfp_hw_ver.algo_nav);
    ret += sprintf(buf + ret, "driver  :%s\n", fsfp_hw_ver.driver);
    ret += sprintf(buf + ret, "firmware:%s\n", fsfp_hw_ver.firmware);
    ret += sprintf(buf + ret, "sensor  :%s\n", fsfp_hw_ver.fortsense_id);
    ret += sprintf(buf + ret, "vendor  :%s\n", fsfp_hw_ver.vendor_id);
    return ret;
}

static DEVICE_ATTR(tee_version, S_IWUSR | S_IRUGO, fsfp_show_version, NULL);

static ssize_t
fsfp_store_set_fun(struct device *d, struct device_attribute *attr,
                   const char *buf, size_t count)
{
    int blank;
    int ret;
    ret = sscanf(buf, "%d", &blank);
    xprintk(KERN_INFO, "fsfp_store_set_fun (..) blank = %d.\n", blank);
    fb_blank(NULL, blank);
    xprintk(KERN_INFO, "fsfp_store_set_fun (..) end.\n");
    return count;
}
static DEVICE_ATTR(set_fun, S_IWUSR | S_IRUGO, NULL, fsfp_store_set_fun);

static struct attribute *fsfp_sysfs_entries[] = {
    &dev_attr_set_fun.attr,
    &dev_attr_tee_version.attr,
    &dev_attr_chip_info.attr,
    &dev_attr_version.attr,
    NULL
};

static struct attribute_group fsfp_attribute_group = {
    .attrs = fsfp_sysfs_entries,
};

#ifdef CONFIG_COMPAT
static long fsfp_ctl_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    return fsfp_ctl_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static int fsfp_suspend(void)
{
    char *screen[2] = { "SCREEN_STATUS=OFF", NULL };
    kobject_uevent_env(&fsfp_ctl_dev.miscdev.this_device->kobj, KOBJ_CHANGE, screen);
#if SF_INT_TRIG_HIGH
    fsfp_ctl_set_irq_type(IRQF_TRIGGER_HIGH);
#endif
    return 0;
}

static int fsfp_resume(void)
{
    char *screen[2] = { "SCREEN_STATUS=ON", NULL };
    kobject_uevent_env(&fsfp_ctl_dev.miscdev.this_device->kobj, KOBJ_CHANGE, screen);
#if SF_INT_TRIG_HIGH
    fsfp_ctl_set_irq_type(IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND | IRQF_ONESHOT);
#endif
    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void fsfp_early_suspend(struct early_suspend *handler)
{
    fsfp_suspend();
}

static void fsfp_late_resume(struct early_suspend *handler)
{
    fsfp_resume();
}

#elif defined(CONFIG_ADF_SPRD)
/*
 * touchscreen's suspend and resume state should rely on screen state,
 * as fb_notifier and early_suspend are all disabled on our platform,
 * we can only use adf_event now
 */
static int fsfp_adf_event_handler( \
                                   struct notifier_block *nb, unsigned long action, void *data)
{
    struct adf_notifier_event *event = data;
    int adf_event_data;

    if (action != ADF_EVENT_BLANK) {
        return NOTIFY_DONE;
    }

    adf_event_data = *(int *)event->data;
    xprintk(KERN_INFO, "receive adf event with adf_event_data=%d \n", adf_event_data);

    switch (adf_event_data) {
        case DRM_MODE_DPMS_ON:
            fsfp_resume();
            break;

        case DRM_MODE_DPMS_OFF:
            fsfp_suspend();
            break;

        default:
            xprintk(KERN_ERR, "receive adf event with error data, adf_event_data=%d \n",
                    adf_event_data);
            break;
    }

    return NOTIFY_OK;
}

#else

static int fsfp_fb_notifier_callback(struct notifier_block *self,
                                     unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    unsigned int blank;
    int retval = 0;

    if (event != FB_EVENT_BLANK /* FB_EARLY_EVENT_BLANK */) {
        return 0;
    }

    blank = *(int *)evdata->data;

    switch (blank) {
        case FB_BLANK_UNBLANK:
            fsfp_resume();
            break;

        case FB_BLANK_POWERDOWN:
            fsfp_suspend();
            break;

        default:
            break;
    }

    return retval;
}
#endif //SF_CFG_HAS_EARLYSUSPEND
////////////////////////////////////////////////////////////////////////////////
static int fsfp_remove(fsfp_device_t *spi)
{
    int err = 0;

    if (fsfp_ctl_dev.pdev != NULL && fsfp_ctl_dev.gpio_init != NULL) {
#ifdef CONFIG_HAS_EARLYSUSPEND
        unregister_early_suspend(&fsfp_ctl_dev.early_suspend);
#elif defined(CONFIG_ADF_SPRD)
        // TODO: is it name adf_unregister_client? Unverified it.
        adf_unregister_client(&fsfp_ctl_dev.adf_event_block);
#else
        fb_unregister_client(&fsfp_ctl_dev.notifier);
#endif

        if (fsfp_ctl_dev.input) {
            input_unregister_device(fsfp_ctl_dev.input);
        }

        if (fsfp_ctl_dev.irq_num >= 0) {
            free_irq(fsfp_ctl_dev.irq_num, (void *)&fsfp_ctl_dev);
            fsfp_ctl_dev.irq_num = 0;
        }

        misc_deregister(&fsfp_ctl_dev.miscdev);
#ifdef CONFIG_PM_WAKELOCKS
        wakeup_source_trash(&fsfp_ctl_dev.wakelock);
#else
        wake_lock_destroy(&fsfp_ctl_dev.wakelock);
#endif
        fsfp_ctl_dev.free_gpio(&fsfp_ctl_dev);
        fsfp_platform_exit(&fsfp_ctl_dev);
        fsfp_ctl_dev.pdev = NULL;
    }

    return err;
}

static int fsfp_probe(fsfp_device_t *dev)
{
    int err = 0;
    xprintk(KERN_INFO, "fortsense %s enter\n", __FUNCTION__);
    fsfp_ctl_dev.pdev = dev;
    /* setup spi config */
#ifdef CONFIG_MTK_SPI
    memcpy(&fsfp_ctl_dev.mt_conf, &smt_conf, sizeof(struct mt_chip_conf));
    fsfp_ctl_dev.pdev->controller_data = (void *)&fsfp_ctl_dev.mt_conf;
    xprintk(KERN_INFO, "fortsense %s old MTK SPI.\n", __FUNCTION__);
#endif
#if SF_SPI_RW_EN
    fsfp_ctl_dev.pdev->mode            = SPI_MODE_0;
    fsfp_ctl_dev.pdev->bits_per_word   = 8;
    fsfp_ctl_dev.pdev->max_speed_hz    = SF_DEFAULT_SPI_SPEED;
    spi_setup(fsfp_ctl_dev.pdev);
#endif
    /* Initialize the platform config. */
    err = fsfp_platform_init(&fsfp_ctl_dev);

    if (err) {
        fsfp_ctl_dev.pdev = NULL;
        xprintk(KERN_ERR, "fsfp_platform_init failed with %d.\n", err);
        return err;
    }

#ifdef CONFIG_PM_WAKELOCKS
    wakeup_source_init(&fsfp_ctl_dev.wakelock , "fsfp_wakelock");
#else
    wake_lock_init(&fsfp_ctl_dev.wakelock, WAKE_LOCK_SUSPEND, "fsfp_wakelock");
#endif
    /* Initialize the GPIO pins. */
#if MULTI_HAL_COMPATIBLE
    xprintk(KERN_INFO, " do not initialize the GPIO pins. \n");
#else
    err = fsfp_ctl_dev.gpio_init(&fsfp_ctl_dev);

    if (err) {
        xprintk(KERN_ERR, "gpio_init failed with %d.\n", err);
        fsfp_ctl_dev.free_gpio(&fsfp_ctl_dev);
        fsfp_platform_exit(&fsfp_ctl_dev);
        fsfp_ctl_dev.pdev = NULL;
        return err;
    }

    fsfp_ctl_dev.reset(false);
    msleep(1);
    fsfp_ctl_dev.reset(true);
    msleep(10);
#endif
#if SF_PROBE_ID_EN
#if SF_BEANPOD_COMPATIBLE_V2
    err = get_fp_spi_enable();

    if (err != 1) {
        xprintk(KERN_ERR, "get_fp_spi_enable ret=%d\n", err);
        fsfp_ctl_dev.free_gpio(&fsfp_ctl_dev);
        fsfp_platform_exit(&fsfp_ctl_dev);
        fsfp_ctl_dev.pdev = NULL;
        return -1;
    }

#endif
#if SF_RONGCARD_COMPATIBLE
#ifdef CONFIG_RSEE
    uint64_t vendor_id = 0x00;
    fsfp_ctl_dev.spi_clk_on(true);
    err = rsee_client_get_fpid(&vendor_id);
    fsfp_ctl_dev.spi_clk_on(false);
    xprintk(KERN_INFO, "rsee_client_get_fpid vendor id is 0x%x\n", vendor_id);

    if (err || !((vendor_id >> 8) == 0x82)) {
        xprintk(KERN_ERR, "rsee_client_get_fpid failed !\n");
        err = -1;
    }

#else
    err = -1;
    xprintk(KERN_INFO, "CONFIG_RSEE not define, skip rsee_client_get_fpid!\n");
#endif
#else
    fsfp_ctl_dev.spi_clk_on(true);
    err = fsfp_read_sensor_id();
    fsfp_ctl_dev.spi_clk_on(false);
#endif

    if (err < 0) {
        xprintk(KERN_ERR, "fortsense probe read chip id is failed\n");
        fsfp_ctl_dev.free_gpio(&fsfp_ctl_dev);
        fsfp_platform_exit(&fsfp_ctl_dev);
        fsfp_ctl_dev.pdev = NULL;
        return -1;
    }

#if SF_BEANPOD_COMPATIBLE_V2
#ifndef MAX_TA_NAME_LEN
    set_fp_vendor(FP_VENDOR_FORTSENSE);
#else
    set_fp_ta_name("fp_server_fortsense", MAX_TA_NAME_LEN);
#endif //MAX_TA_NAME_LEN
#endif //SF_BEANPOD_COMPATIBLE_V2
#endif //SF_PROBE_ID_EN
    /* reset spi dma mode in old MTK. */
#if (SF_REE_PLATFORM && defined(CONFIG_MTK_SPI))
    {
        fsfp_ctl_dev.mt_conf.com_mod = DMA_TRANSFER;
        spi_setup(fsfp_ctl_dev.pdev);
    }
#endif
    /* Initialize the input subsystem. */
    err = fsfp_ctl_init_input();

    if (err) {
        fsfp_ctl_dev.free_gpio(&fsfp_ctl_dev);
        fsfp_platform_exit(&fsfp_ctl_dev);
        fsfp_ctl_dev.pdev = NULL;
        xprintk(KERN_ERR, "fsfp_ctl_init_input failed with %d.\n", err);
        return err;
    }

    /* Register as a miscellaneous device. */
    err = misc_register(&fsfp_ctl_dev.miscdev);

    if (err) {
        fsfp_ctl_dev.free_gpio(&fsfp_ctl_dev);
        fsfp_platform_exit(&fsfp_ctl_dev);
        xprintk(KERN_ERR, "misc_register(..) = %d.\n", err);
        input_unregister_device(fsfp_ctl_dev.input);
        fsfp_ctl_dev.pdev = NULL;
        return err;
    }

    err = sysfs_create_group(&fsfp_ctl_dev.miscdev.this_device->kobj, &fsfp_attribute_group);
    /* Initialize the interrupt callback. */
    INIT_WORK(&fsfp_ctl_dev.work_queue, fsfp_ctl_device_event);
#if MULTI_HAL_COMPATIBLE
    xprintk(KERN_INFO, " do not initialize the fingerprint interrupt. \n");
#else
    err = fsfp_ctl_init_irq();

    if (err) {
        xprintk(KERN_ERR, "fsfp_ctl_init_irq failed with %d.\n", err);
        input_unregister_device(fsfp_ctl_dev.input);
        misc_deregister(&fsfp_ctl_dev.miscdev);
        fsfp_ctl_dev.free_gpio(&fsfp_ctl_dev);
        fsfp_platform_exit(&fsfp_ctl_dev);
        fsfp_ctl_dev.pdev = NULL;
        return err;
    }

#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
    fsfp_ctl_dev.early_suspend.level = (EARLY_SUSPEND_LEVEL_DISABLE_FB - 1);
    fsfp_ctl_dev.early_suspend.suspend = fsfp_early_suspend;
    fsfp_ctl_dev.early_suspend.resume = fsfp_late_resume;
    register_early_suspend(&fsfp_ctl_dev.early_suspend);
#elif defined(CONFIG_ADF_SPRD)
    fsfp_ctl_dev.adf_event_block.notifier_call = fsfp_adf_event_handler;
    err = adf_register_client(&fsfp_ctl_dev.adf_event_block);

    if (err < 0) {
        xprintk(KERN_ERR, "register adf notifier fail, cannot sleep when screen off");
    }
    else {
        xprintk(KERN_ERR, "register adf notifier succeed");
    }

#else
    fsfp_ctl_dev.notifier.notifier_call = fsfp_fb_notifier_callback;
    fb_register_client(&fsfp_ctl_dev.notifier);
#endif
    /* beanpod ISEE2.7 */
#if SF_BEANPOD_COMPATIBLE_V2_7
    {
        /* fortsense define, flow by trustonic */
        struct TEEC_UUID vendor_uuid = {0x0401c03f, 0xc30c, 0x4dd0, \
            {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04} \
        };
        memcpy(&uuid_fp, &vendor_uuid, sizeof(struct TEEC_UUID));
    }
    xprintk(KERN_ERR, "%s set beanpod isee2.7.0 uuid ok \n", __FUNCTION__);
#endif
    xprintk(KERN_ERR, "%s leave\n", __FUNCTION__);
    return err;
}

#if SF_SPI_TRANSFER
static int tee_spi_transfer(void *fsfp_conf, int cfg_len, const char *txbuf, char *rxbuf, int len)
{
    struct spi_transfer t;
    struct spi_message m;
    memset(&t, 0, sizeof(t));
    spi_message_init(&m);
    t.tx_buf = txbuf;
    t.rx_buf = rxbuf;
    t.bits_per_word = 8;
    t.len = len;
#if QUALCOMM_REE_DEASSERT
    t.speed_hz = qualcomm_deassert;
#else
    t.speed_hz = fsfp_ctl_dev.pdev->max_speed_hz;
#endif
    spi_message_add_tail(&t, &m);
    return spi_sync(fsfp_ctl_dev.pdev, &m);
}
#endif

#if SF_REE_PLATFORM
static ssize_t fsfp_ctl_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    const size_t bufsiz = fsfp_ctl_dev.spi_buf_size;
    ssize_t status = 0;

    /* chipselect only toggles at start or end of operation */
    if (count > bufsiz) {
        return (-EMSGSIZE);
    }

    if (fsfp_ctl_dev.spi_buffer == NULL) {
        fsfp_ctl_dev.spi_buffer = kmalloc(bufsiz, GFP_KERNEL);

        if (fsfp_ctl_dev.spi_buffer == NULL) {
            xprintk(KERN_ERR, " %s malloc spi_buffer failed.\n", __FUNCTION__);
            return (-ENOMEM);
        }
    }

    memset(fsfp_ctl_dev.spi_buffer, 0, bufsiz);

    if (copy_from_user(fsfp_ctl_dev.spi_buffer, buf, count)) {
        xprintk(KERN_ERR, "%s copy_from_user(..) failed.\n", __FUNCTION__);
        return (-EMSGSIZE);
    }

    {
        /* not used */
        void *fsfp_conf;
        int cfg_len = 0;
        status = tee_spi_transfer(fsfp_conf, cfg_len, fsfp_ctl_dev.spi_buffer, fsfp_ctl_dev.spi_buffer, count);
    }

    if (status == 0) {
        status = copy_to_user(buf, fsfp_ctl_dev.spi_buffer, count);

        if (status != 0) {
            status = -EFAULT;
        }
        else {
            status = count;
        }
    }
    else {
        xprintk(KERN_ERR, " %s spi_transfer failed.\n", __FUNCTION__);
    }

    return status;
}

static ssize_t fsfp_ctl_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    const size_t bufsiz = fsfp_ctl_dev.spi_buf_size;
    ssize_t status = 0;

    /* chipselect only toggles at start or end of operation */
    if (count > bufsiz) {
        return (-EMSGSIZE);
    }

    if (fsfp_ctl_dev.spi_buffer == NULL) {
        fsfp_ctl_dev.spi_buffer = kmalloc(bufsiz, GFP_KERNEL);

        if (fsfp_ctl_dev.spi_buffer == NULL) {
            xprintk(KERN_ERR, " %s malloc spi_buffer failed.\n", __FUNCTION__);
            return (-ENOMEM);
        }
    }

    memset(fsfp_ctl_dev.spi_buffer, 0, bufsiz);

    if (copy_from_user(fsfp_ctl_dev.spi_buffer, buf, count)) {
        xprintk(KERN_ERR, "%s copy_from_user(..) failed.\n", __FUNCTION__);
        return (-EMSGSIZE);
    }

    {
        /* not used */
        void *fsfp_conf;
        int cfg_len = 0;
        status = tee_spi_transfer(fsfp_conf, cfg_len, fsfp_ctl_dev.spi_buffer, fsfp_ctl_dev.spi_buffer, count);
    }

    if (status == 0) {
        status = count;
    }
    else {
        xprintk(KERN_ERR, " %s spi_transfer failed.\n", __FUNCTION__);
    }

    return status;
}
#endif

#if (SF_PROBE_ID_EN && !SF_RONGCARD_COMPATIBLE)
static int fsfp_read_sensor_id(void)
{
    int ret = -1;
    int trytimes = 3;
    char readbuf[16]  = {0};
    char writebuf[16] = {0};
    int cfg_len = 0;
    //默认速度设置为1M, 不然8201/8211系列有可能读不到ID
#if (defined(CONFIG_MTK_SPI) || SF_TRUSTKERNEL_COMPAT_SPI_MT65XX)
    smt_conf.high_time = SF_DEFAULT_SPI_HIGH_TIME;
    smt_conf.low_time  = SF_DEFAULT_SPI_LOW_TIME;
    smt_conf.com_mod   = FIFO_TRANSFER;
    smt_conf.cpol      = SPI_CPOL_0; // SPI_MODE_0: cpol = 0 and cpha = 0
    smt_conf.cpha      = SPI_CPHA_0;
#if (SF_COMPATIBLE_SEL == SF_COMPATIBLE_TRUSTKERNEL)
    cfg_len = sizeof(struct mt_chip_conf);
#else
    memcpy(&fsfp_ctl_dev.mt_conf, &smt_conf, sizeof(struct mt_chip_conf));
#endif
#else
    /* not used */
    int smt_conf;
#endif
    fsfp_ctl_dev.pdev->max_speed_hz = SF_DEFAULT_SPI_SPEED;
    fsfp_ctl_dev.pdev->bits_per_word = 8;
    fsfp_ctl_dev.pdev->mode = SPI_MODE_0;
    spi_setup(fsfp_ctl_dev.pdev);
    xprintk(SF_LOG_LEVEL, "%s(..) enter.\n", __FUNCTION__);
    msleep(10);

    do {
        /* 1.detect Fortsense ID */
        fsfp_ctl_dev.reset(false);
        msleep(1);
        fsfp_ctl_dev.reset(true);
        msleep(10);
        memset(readbuf,  0, sizeof(readbuf));
        memset(writebuf, 0, sizeof(writebuf));
        writebuf[0] = 0x65;
        writebuf[1] = (uint8_t)(~0x65);
        writebuf[2] = 0x00;
        writebuf[3] = 0x04;
        writebuf[4] = 0x20;
        writebuf[5] = 0x04;
        writebuf[6] = 0x20;
        ret = tee_spi_transfer(&smt_conf, cfg_len, writebuf, readbuf, 11);

        if (ret != 0) {
            xprintk(KERN_ERR, "SPI transfer failed\n");
            continue;
        }

        if ((0x53 == readbuf[7]) && (0x75 == readbuf[8]) && (0x6e == readbuf[9]) && (0x57 == readbuf[10])) {
            xprintk(KERN_INFO, "read 4bytes id is ok\n");
            // readid 2 bytes
            memset(readbuf,  0, sizeof(readbuf));
            memset(writebuf, 0, sizeof(writebuf));
            writebuf[0] = 0x65;
            writebuf[1] = (uint8_t)(~0x65);
            writebuf[2] = 0x00;
            writebuf[3] = 0x02;
            writebuf[4] = 0x20;
            writebuf[5] = 0x01;
            writebuf[6] = 0x41;
            ret = tee_spi_transfer(&smt_conf, cfg_len, writebuf, readbuf, 8);

            if (ret != 0) {
                xprintk(KERN_ERR, "SPI transfer failed\n");
                continue;
            }

            xprintk(KERN_INFO, " tee_spi_transfer.readbuf = 0x%02x-%02x\n", readbuf[7], readbuf[8]);

            if (0x95 == readbuf[7]) {
                xprintk(KERN_INFO, "read id ok\n");
                return 0;
            }
            else {
                xprintk(KERN_INFO, "read id fail\n");
            }
        }
        else {
            xprintk(KERN_INFO, " tee_spi_transfer.readbuf = 0x%02x-%02x-%02x-%02x\n", readbuf[7], readbuf[8], readbuf[9],
                    readbuf[10]);
        }
    }
    while (trytimes--);

    return -1;
}

#endif

////////////////////////////////////////////////////////////////////////////////
static int fsfp_ctl_init_irq(void)
{
    int err = 0;
    unsigned long flags = IRQF_TRIGGER_FALLING; // IRQF_TRIGGER_FALLING or IRQF_TRIGGER_RISING
#if !SF_MTK_CPU
    flags |= IRQF_ONESHOT;
#if SF_INT_TRIG_HIGH
    flags |= IRQF_NO_SUSPEND;
#endif
#endif
    xprintk(SF_LOG_LEVEL, "%s(..) enter.\n", __FUNCTION__);
    /* Register interrupt callback. */
    err = request_irq(fsfp_ctl_dev.irq_num, fsfp_ctl_device_irq,
                      flags, "sf-irq", NULL);

    if (err) {
        xprintk(KERN_ERR, "request_irq(..) = %d.\n", err);
    }

    enable_irq_wake(fsfp_ctl_dev.irq_num);
    xprintk(SF_LOG_LEVEL, "%s(..) leave.\n", __FUNCTION__);
    return err;
}

static int fsfp_ctl_init_input(void)
{
    int err = 0;
    xprintk(SF_LOG_LEVEL, "%s(..) enter.\n", __FUNCTION__);
    fsfp_ctl_dev.input = input_allocate_device();

    if (!fsfp_ctl_dev.input) {
        xprintk(KERN_ERR, "input_allocate_device(..) failed.\n");
        return (-ENOMEM);
    }

    fsfp_ctl_dev.input->name = "sf-keys";
    __set_bit(EV_KEY,     fsfp_ctl_dev.input->evbit );
    __set_bit(KEY_HOME,   fsfp_ctl_dev.input->keybit);
    __set_bit(KEY_MENU,   fsfp_ctl_dev.input->keybit);
    __set_bit(KEY_BACK,   fsfp_ctl_dev.input->keybit);
    __set_bit(KEY_F11,    fsfp_ctl_dev.input->keybit);
    __set_bit(KEY_ENTER,  fsfp_ctl_dev.input->keybit);
    __set_bit(KEY_UP,     fsfp_ctl_dev.input->keybit);
    __set_bit(KEY_LEFT,   fsfp_ctl_dev.input->keybit);
    __set_bit(KEY_RIGHT,  fsfp_ctl_dev.input->keybit);
    __set_bit(KEY_DOWN,   fsfp_ctl_dev.input->keybit);
    __set_bit(KEY_WAKEUP, fsfp_ctl_dev.input->keybit);
    err = input_register_device(fsfp_ctl_dev.input);

    if (err) {
        xprintk(KERN_ERR, "input_register_device(..) = %d.\n", err);
        input_free_device(fsfp_ctl_dev.input);
        fsfp_ctl_dev.input = NULL;
        return (-ENODEV);
    }

    xprintk(SF_LOG_LEVEL, "%s(..) leave.\n", __FUNCTION__);
    return err;
}

static int __init fsfp_ctl_driver_init(void)
{
    int err = 0;
    xprintk(KERN_INFO, "'%s' SW_BUS_NAME = %s\n", __FUNCTION__, SW_BUS_NAME);
#if SF_BEANPOD_COMPATIBLE_V1
    uint64_t fp_vendor_id = 0x00;
    get_t_device_id(&fp_vendor_id);
    xprintk(KERN_INFO, "'%s' fp_vendor_id = 0x%x\n", __FUNCTION__, fp_vendor_id);

    if (fp_vendor_id != 0x02) {
        return 0;
    }

#endif
#if SF_SPI_RW_EN
    /**register SPI device、driver***/
    // spi_register_board_info(spi_board_devs, ARRAY_SIZE(spi_board_devs));
    err = spi_register_driver(&fsfp_driver);

    if (err < 0) {
        xprintk(KERN_ERR, "%s, Failed to register SPI driver.\n", __FUNCTION__);
    }

#else
#if SF_REG_DEVICE_BY_DRIVER
    fsfp_device = platform_device_alloc("fortsense-fp", 0);

    if (fsfp_device) {
        err = platform_device_add(fsfp_device);

        if (err) {
            platform_device_put(fsfp_device);
            fsfp_device = NULL;
        }
    }
    else {
        err = -ENOMEM;
    }

#endif

    if (err) {
        xprintk(KERN_ERR, "%s, Failed to register platform device.\n", __FUNCTION__);
    }

    err = platform_driver_register(&fsfp_driver);

    if (err) {
        xprintk(KERN_ERR, "%s, Failed to register platform driver.\n", __FUNCTION__);
        return -EINVAL;
    }

#endif
    xprintk(KERN_INFO, "fortsense fingerprint device control driver registered.\n");
    xprintk(KERN_INFO, "driver version: '%s'.\n", fsfp_ctl_get_version());
    return err;
}

static void __exit fsfp_ctl_driver_exit(void)
{
#if SF_SPI_RW_EN
    spi_unregister_driver(&fsfp_driver);
#else
    platform_driver_unregister(&fsfp_driver);
#endif
#if SF_REG_DEVICE_BY_DRIVER

    if (fsfp_device) {
        platform_device_unregister(fsfp_device);
    }

#endif
    xprintk(KERN_INFO, "fortsense fingerprint device control driver released.\n");
}

late_initcall(fsfp_ctl_driver_init);
module_exit(fsfp_ctl_driver_exit);

MODULE_DESCRIPTION("The device control driver for Fortsense's fingerprint sensor.");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Fortsense");

