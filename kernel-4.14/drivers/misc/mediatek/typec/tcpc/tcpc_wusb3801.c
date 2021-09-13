/************************************************************************
 *
 *  WILLSEMI TypeC Chipset Driver for Linux & Android.  
 *
 *
 * ######################################################################
 *
 *  Author: lei.huang (lhuang@sh-willsemi.com)
 *
 * Copyright (c) 2021, WillSemi Inc. All rights reserved.
 *
 ************************************************************************/


#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/semaphore.h>
#include <linux/pm_runtime.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/cpu.h>
#include <linux/version.h>
#include <linux/pm_wakeup.h>
#include <linux/sched/clock.h>
#include <uapi/linux/sched/types.h>

#include "inc/pd_dbg_info.h"
#include "inc/tcpci.h"
#include "inc/tcpci_typec.h"

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0))
#include <linux/sched/rt.h>
#endif

#define WUSB3801_IRQ_WAKE_TIME	(500) /* ms */
/* Register Map */
#define WUSB3801_DRV_VERSION	"3.1.0_MTK"

/******************************************************************************
* Register addresses
******************************************************************************/
#define WUSB3801_REG_VERSION_ID         0x01
#define WUSB3801_REG_CONTROL0           0x02
#define WUSB3801_REG_INTERRUPT          0x03
#define WUSB3801_REG_STATUS             0x04
#define WUSB3801_REG_CONTROL1           0x05
#define WUSB3801_REG_TEST0              0x06
#define WUSB3801_REG_TEST_01            0x07
#define WUSB3801_REG_TEST_02            0x08
#define WUSB3801_REG_TEST_03            0x09
#define WUSB3801_REG_TEST_04            0x0A
#define WUSB3801_REG_TEST_05            0x0B
#define WUSB3801_REG_TEST_06            0x0C
#define WUSB3801_REG_TEST_07            0x0D
#define WUSB3801_REG_TEST_08            0x0E
#define WUSB3801_REG_TEST_09            0x0F
#define WUSB3801_REG_TEST_0A            0x10
#define WUSB3801_REG_TEST_0B            0x11
#define WUSB3801_REG_TEST_0C            0x12
#define WUSB3801_REG_TEST_0D            0x13
#define WUSB3801_REG_TEST_0E            0x14
#define WUSB3801_REG_TEST_0F            0x15
#define WUSB3801_REG_TEST_10            0x16
#define WUSB3801_REG_TEST_11            0x17
#define WUSB3801_REG_TEST_12            0x18


#define WUSB3801_SLAVE_ADDR0            0xc0
#define WUSB3801_SLAVE_ADDR1            0xd0
/******************************************************************************
* Register bits
******************************************************************************/
/* WUSB3801_REG_VERSION_ID (0x01)  RU*/
/* WUSB3801_REG_CONTROL0 (0x02) */
#define CTRL_INT_SHIFT    0
#define CTRL_INT          (0x01 << CTRL_INT_SHIFT)      /*RW*/
#define CTRL_WORK_MODE_SHIFT  1
#define CTRL_WORK_MODE        (0x03 << CTRL_WORK_MODE_SHIFT)    /*RCU*/
#define CTRL_CURRENT_MODE_SHIFT         3
#define CTRL_CURRENT_MODE               (0x03 << CTRL_CURRENT_MODE_SHIFT)           /*RU*/
#define CTRL_TRY_SNK_SRC_SHIFT    5
#define CTRL_TRY_SNK_SRC           (0x03 << CTRL_TRY_SNK_SRC_SHIFT)      /*RU*/
#define CTRL_ACC_SUPPORT_SHIFT    7
#define CTRL_ACC_SUPPORT          (0x01 << CTRL_ACC_SUPPORT_SHIFT)      /*RU*/

/* WUSB3801_REG_INTERRUPT (0x03) */
#define INT_ATTACH_DETACH_SHIFT        0x00    /*RW*/
#define INT_ATTACH_DETACH          (0x03 << INT_ATTACH_DETACH_SHIFT)      /*RU*/

/* WUSB3801_REG_STATUS (0x04) */
#define CC_STATUS_PLUG_ORIENTATION_SHIFT    0
#define CC_STATUS_PLUG_ORIENTATION          (0x03 << CC_STATUS_PLUG_ORIENTATION_SHIFT)      /*RW*/
#define CC_PLUG_PORT_STATUS_SHIFT  2
#define CC_PLUG_PORT_STATUS      (0x07 << CC_PLUG_PORT_STATUS_SHIFT)    /*RCU*/
#define CC_STATUS_CHARGING_CURRENT_SNK_SHIFT         5
#define CC_STATUS_CHARGING_CURRENT_SNK              (0x03 << CC_STATUS_CHARGING_CURRENT_SNK_SHIFT)           /*RU*/
#define CC_STATUS_VBUS_DETECTION_SRC_SHIFT    7
#define CC_STATUS_VBUS_DETECTION_SRC         (0x01 << CC_STATUS_VBUS_DETECTION_SRC_SHIFT)      /*RU*/


/* SET_MODE_SELECT */
//#define SET_MODE_SELECT_DEFAULT  0x00
#define SET_MODE_SELECT_SNK       0x00
#define SET_MODE_SELECT_SRC       0x01
#define SET_MODE_SELECT_DRP       0x02

/* MOD_CURRENT_MODE_ADVERTISE */
#define MOD_CURRENT_MODE_ADVERTISE_DEFAULT      0x00
#define MOD_CURRENT_MODE_ADVERTISE_MID          0x01
#define MOD_CURRENT_MODE_ADVERTISE_HIGH         0x02
/* MOD_CURRENT_MODE_DETECT */
#define MOD_CURRENT_MODE_DETECT_DEFAULT      0x00
#define MOD_CURRENT_MODE_DETECT_MID          0x01
#define MOD_CURRENT_MODE_DETECT_ACCESSARY    0x02
#define MOD_CURRENT_MODE_DETECT_HIGH         0x03

#define CC_STATUS_VBUS_DETECTION       0x01


#define  IC_TEST_DEV
/******************************************************************************
 * Constants
 ******************************************************************************/
enum current_adv_type {
	HOST_CUR_USB = 0,   /*default 500mA or 900mA*/
	HOST_CUR_1P5,      /*1.5A*/
	HOST_CUR_3A       /*3A*/
};

enum current_det_type {
	DET_CUR_ACCESSORY = 0,  /*charg through accessory 500mA*/
	DET_CUR_USB,    /*default 500mA or 900mA*/
	DET_CUR_1P5,
	DET_CUR_3A
};

enum cable_attach_type {
	CABLE_NOT_INT = 0,
	CABLE_ATTACHED,
	CABLE_NOT_ATTACHED
};

enum cable_state_type {
	CABLE_STATE_NOT_ATTACHED = 0,
	CABLE_STATE_AS_DFP,
	CABLE_STATE_AS_UFP,
	CABLE_STATE_TO_ACCESSORY,
	CABLE_STATE_TO_DEBUG
};

enum cable_dir_type {
	ORIENT_DEFAULT,
	ORIENT_CC1,
	ORIENT_CC2,
	ORIENT_CC1_CC2
};

enum cc_modes_type {
	MODE_UFP,
	MODE_DFP,
	MODE_DRP
};

enum int_attach_type {
	INT_NOT_INT = 0,
	INT_ATTACHED,
	INT_NOT_ATTACHED
};

/* Type-C Attrs */
struct type_c_parameters {
	enum current_det_type current_det;         /*charging current on UFP*/
	enum int_attach_type int_attach;     /*if an accessory is attached*/
	//enum cable_attach_type active_cable_attach;         /*if an active_cable is attached*/
	enum cable_state_type attach_state;        /*DFP->UFP or UFP->DFP*/
	enum cable_dir_type cable_dir;           /*cc1 or cc2*/
};

/*Working context structure*/
struct wusb3801_chip {
	struct i2c_client *client;
	struct device *dev;
	struct semaphore io_lock;
	struct semaphore suspend_lock;
	struct tcpc_desc *tcpc_desc;
	struct tcpc_device *tcpc;
	struct kthread_worker irq_worker;
	struct kthread_work irq_work;
	struct task_struct *irq_worker_task;
	struct wakeup_source irq_wake_lock;
	struct mutex  mutex;

	atomic_t poll_count;
	struct delayed_work	poll_work;

	struct type_c_parameters type_c_param;
	struct type_c_parameters type_c_param_old;
	int irq_gpio;
	int irq;
	int chip_id;
};

/* i2c operate interfaces */
static int wusb3801_read_reg(struct i2c_client *i2c, u8 reg, u8 *dest)
{
	struct wusb3801_chip *info = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&info->mutex);
	ret = i2c_smbus_read_byte_data(i2c, reg);
	mutex_unlock(&info->mutex);
	if (ret < 0) {
		pr_err("%s: (0x%x) error, ret(%d)\n", __func__, reg, ret);
		return ret;
	}

	ret &= 0xff;
	*dest = ret;
	return 0;
}
#if 1//def __TEST_CC_PATCH__
static int wusb3801_write_reg(struct i2c_client *i2c, u8 reg, u8 value)
{
	struct wusb3801_chip *info = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&info->mutex);
	ret = i2c_smbus_write_byte_data(i2c, reg, value);
	mutex_unlock(&info->mutex);
	if (ret < 0)
		pr_err("%s: (0x%x) error, ret(%d)\n", __func__, reg, ret);

	return ret;
}
#endif
static int wusb3801_update_reg(struct i2c_client *i2c, u8 reg, u8 val, u8 mask)
{
	struct wusb3801_chip *info = i2c_get_clientdata(i2c);
	int ret;
	u8 old_val, new_val;

	mutex_lock(&info->mutex);
	ret = i2c_smbus_read_byte_data(i2c, reg);

	if (ret >= 0) {
		old_val = ret & 0xff;
		new_val = (val & mask) | (old_val & (~mask));
		ret = i2c_smbus_write_byte_data(i2c, reg, new_val);
	}
	mutex_unlock(&info->mutex);
	return ret;
}

//
#ifdef __TEST_CC_PATCH__
#undef  __CONST_FFS
#define __CONST_FFS(_x) \
        ((_x) & 0x0F ? ((_x) & 0x03 ? ((_x) & 0x01 ? 0 : 1) :\
                                      ((_x) & 0x04 ? 2 : 3)) :\
                       ((_x) & 0x30 ? ((_x) & 0x10 ? 4 : 5) :\
                                      ((_x) & 0x40 ? 6 : 7)))

#undef  FFS
#define FFS(_x) \
        ((_x) ? __CONST_FFS(_x) : 0)

#undef  BITS
#define BITS(_end, _start) \
        ((BIT(_end) - BIT(_start)) + BIT(_end))

#undef  __BITS_GET
#define __BITS_GET(_byte, _mask, _shift) \
        (((_byte) & (_mask)) >> (_shift))

#undef  BITS_GET
#define BITS_GET(_byte, _bit) \
        __BITS_GET(_byte, _bit, FFS(_bit))

static int test_cc_patch(struct wusb3801_chip *chip)
{
	int rc;
	int rc_reg_08;
	int i = 0;
	
	struct device *cdev = &chip->client->dev;
	dev_err(cdev, "%s \n",__func__);

	wusb3801_write_reg(chip->tcpc,
			WUSB3801_REG_TEST_02, 0x82);
	msleep(100);
	wusb3801_write_reg(chip->tcpc,
			WUSB3801_REG_TEST_09, 0xC0);
	msleep(100);
	rc = wusb3801_write_reg(chip->tcpc, WUSB3801_REG_TEST0);
	msleep(10);
	wusb3801_write_reg(chip->tcpc,
			WUSB3801_REG_TEST_09, 0x00);
	msleep(10);
	wusb3801_write_reg(chip->tcpc,
			WUSB3801_REG_TEST_02, 0x80);
//huanglei add for reg 0x08 write zero fail begin
	do{
    		msleep(100);
        	wusb3801_write_reg(chip->tcpc,
        		WUSB3801_REG_TEST_02, 0x00);
	    	msleep(100);
	    	rc = wusb3801_read_reg(chip->tcpc, WUSB3801_REG_TEST_02,&rc_reg_08);
			i++;		
	}while(rc_reg_08 != 0 && i < 5);
//end	
	dev_err(cdev, "%s rc = [0x%02x] \n",__func__, rc);
    return BITS_GET(rc, 0x40);
}
#endif /* __TEST_CC_PATCH__ */


/************************************************************************
 *
 *       fregdump_show
 *
 *  Description :
 *  -------------
 *  Dump registers to user space. there is side-effects for Read/Clear 
 *  registers. For example interrupt status. 
 *
 ************************************************************************/
 #ifdef IC_TEST_DEV
static ssize_t fregdump_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wusb3801_chip *chip = i2c_get_clientdata(client);
	int i, rc, ret = 0;

	mutex_lock(&chip->mutex);
	for (i = WUSB3801_REG_VERSION_ID ; i <= WUSB3801_REG_TEST_12; i++) {
		rc = i2c_smbus_read_byte_data(chip->client, (uint8_t)i);
		if (rc < 0) {
			pr_err("cannot read 0x%02x\n", i);
			rc = 0;
		}
		ret += snprintf(buf + ret, 1024 - ret, "from 0x%02x read 0x%02x\n", (uint8_t)i, rc);
	}
	mutex_unlock(&chip->mutex);
	return ret;
}

DEVICE_ATTR(fregdump, S_IRUGO, fregdump_show, NULL);
#endif
static int wusb3801_init_alert_mask(struct tcpc_device *tcpc)
{
	return 0;
}

static int wusb3801_init_power_status_mask(struct tcpc_device *tcpc)
{
	return 0;
}

static int wusb3801_init_fault_mask(struct tcpc_device *tcpc)
{
	return 0;
}

static int wusb3801_init_rt_mask(struct tcpc_device *tcpc)
{
	return 0;
}

static inline void wusb3801_poll_ctrl(struct wusb3801_chip *chip)
{
	cancel_delayed_work_sync(&chip->poll_work);

	if (atomic_read(&chip->poll_count) == 0) {
		atomic_inc(&chip->poll_count);
		cpu_idle_poll_ctrl(true);
	}

	schedule_delayed_work(
		&chip->poll_work, msecs_to_jiffies(40));
}

/***********************************************************
 * read registers in irq process
 ***********************************************************/
static void process_mode_register(struct wusb3801_chip *info)
{
	#if 0
	u8 val, tmp, reg_val;
	int ret;
	ret = wusb3801_read_reg(info->client, WUSB3801_REG_STATUS, &reg_val);
	if (ret < 0) {
		pr_err("%s err\n", __func__);
		return;
	}
	tmp = reg_val;
	/* check current_detect */
	val = ((tmp & CC_STATUS_CHARGING_CURRENT_SNK) >> CC_STATUS_CHARGING_CURRENT_SNK_SHIFT);
	info->type_c_param.current_det = val;
	/* check accessory attch */
	//tmp = reg_val;
	//val = ((tmp & MOD_ACCESSORY_CONNECTED) >> MOD_ACCESSORY_CONNECTED_SHIFT);
	//info->type_c_param.accessory_attach = val;

	/* check cable attach */
	//tmp = reg_val;
	//val = (tmp & CC_PLUG_PORT_STATUS) >> CC_PLUG_PORT_STATUS_SHIFT;
	//info->type_c_param.active_cable_attach = val;
	/* check attach state */
	val = ((tmp & CC_PLUG_PORT_STATUS) >> CC_PLUG_PORT_STATUS_SHIFT);
	info->type_c_param.attach_state = val;
	/* update current adv when act as DFP */
	if (info->type_c_param.attach_state == CABLE_STATE_AS_DFP ||
	    info->type_c_param.attach_state == CABLE_STATE_TO_ACCESSORY) {
		val = (HOST_CUR_USB << CTRL_CURRENT_MODE_SHIFT);
	} else {
		val = (HOST_CUR_3A << CTRL_CURRENT_MODE_SHIFT);
	}
	wusb3801_update_reg(info->client, WUSB3801_REG_CONTROL0, val, CTRL_CURRENT_MODE);
	/* check cable dir */
	//tmp = reg_val;
	//val = ((tmp & CC_STATUS_PLUG_ORIENTATION) >> CC_STATUS_PLUG_ORIENTATION_SHIFT);
	//info->type_c_param.cable_dir = val;
	//printk("wusb3801 process_mode_register reg_val=%d,val=%d\n",reg_val,val);
	printk("wusb3801 process_mode_register attach_state=%d,current_det =%d\n",info->type_c_param.attach_state,info->type_c_param.current_det);
	#endif
}

static void process_interrupt_register(struct wusb3801_chip *info)
{
	#if 0
	u8 val, tmp, reg_val;
	int ret;
	/* get interrupt */
	ret = wusb3801_read_reg(info->client, WUSB3801_REG_INTERRUPT, &reg_val);
	if (ret < 0) {
		pr_err("%s err\n", __func__);
		return;
	}
	tmp = reg_val;
	val = (tmp & INT_ATTACH_DETACH) >> INT_ATTACH_DETACH_SHIFT;
	printk("wusb3801interrupt_register ATTACH=%d\n",val);
	#endif
}

static void wusb3801_irq_work_handler(struct kthread_work *work)
{
	struct wusb3801_chip *chip =
			container_of(work, struct wusb3801_chip, irq_work);
	int regval = 0;
	int gpio_val;
	#ifdef IC_TEST_DEV
	int i = 0;
	int rc = 0;
	#endif
	wusb3801_poll_ctrl(chip);
	/* make sure I2C bus had resumed */
	down(&chip->suspend_lock);
	tcpci_lock_typec(chip->tcpc);

	process_mode_register(chip);
	process_interrupt_register(chip);
	printk("wusb3801_irq_work_handler\n");
	do {
		regval = tcpci_alert(chip->tcpc);
		#ifdef IC_TEST_DEV
		if(regval)
		{
			printk("wusb3801_tcpci_alert\n");
		}
		#endif
		if (regval)
			break;
		gpio_val = gpio_get_value(chip->irq_gpio);
		#ifdef IC_TEST_DEV
		//异常情况 出现清除中断，中断脚应该置高，现在还是置低打印所有寄存器信息到FAE分析
		for (i = WUSB3801_REG_VERSION_ID ; i <= WUSB3801_REG_TEST_12; i++) {
			rc = i2c_smbus_read_byte_data(chip->client, (uint8_t)i);
			if (rc < 0) {
				printk("wusb3801 cannot read 0x%02x\n", i);
				rc = 0;
			}
			//ret += snprintf(buf + ret, 1024 - ret, "from 0x%02x read 0x%02x\n", (uint8_t)i, rc);
			printk("wusb3801_0x%02x_0x%02x\n", (uint8_t)i, rc);
		}
		#endif
	} while (gpio_val == 0);

	tcpci_unlock_typec(chip->tcpc);
	up(&chip->suspend_lock);
}
static void wusb3801_poll_work(struct work_struct *work)
{
	struct wusb3801_chip *chip = container_of(
		work, struct wusb3801_chip, poll_work.work);

	if (atomic_dec_and_test(&chip->poll_count))
		cpu_idle_poll_ctrl(false);
}


static irqreturn_t wusb3801_intr_handler(int irq, void *data)
{
	struct wusb3801_chip *chip = data;

	__pm_wakeup_event(&chip->irq_wake_lock, WUSB3801_IRQ_WAKE_TIME);

	kthread_queue_work(&chip->irq_worker, &chip->irq_work);
	return IRQ_HANDLED;
}

static int wusb3801_init_alert(struct tcpc_device *tcpc)
{
	struct wusb3801_chip *chip = tcpc_get_dev_data(tcpc);
	struct sched_param param = { .sched_priority = MAX_RT_PRIO - 1 };
	int ret;
	char *name;
	int len;
	int reg_val = 0;
	//u8 val = 0;
	len = strlen(chip->tcpc_desc->name);
	name = devm_kzalloc(chip->dev, len+5, GFP_KERNEL);
	if (!name)
		return -ENOMEM;

	snprintf(name, PAGE_SIZE, "%s-IRQ", chip->tcpc_desc->name);

	pr_info("%s name = %s, gpio = %d\n", __func__,
				chip->tcpc_desc->name, chip->irq_gpio);

	ret = devm_gpio_request(chip->dev, chip->irq_gpio, name);
	if (ret < 0) {
		pr_err("Error: failed to request GPIO%d (ret = %d)\n",
		chip->irq_gpio, ret);
		goto init_alert_err;
	}

	ret = gpio_direction_input(chip->irq_gpio);
	if (ret < 0) {
		pr_err("Error: failed to set GPIO%d as input pin(ret = %d)\n",
		chip->irq_gpio, ret);
		goto init_alert_err;
	}

	chip->irq = gpio_to_irq(chip->irq_gpio);
	if (chip->irq <= 0) {
		pr_err("%s gpio to irq fail, chip->irq(%d)\n",
						__func__, chip->irq);
		goto init_alert_err;
	}

	pr_info("%s : IRQ number = %d\n", __func__, chip->irq);

	kthread_init_worker(&chip->irq_worker);
	chip->irq_worker_task = kthread_run(kthread_worker_fn,
			&chip->irq_worker, "%s", chip->tcpc_desc->name);
	if (IS_ERR(chip->irq_worker_task)) {
		pr_err("Error: Could not create tcpc task\n");
		goto init_alert_err;
	}

	sched_setscheduler(chip->irq_worker_task, SCHED_FIFO, &param);
	kthread_init_work(&chip->irq_work, wusb3801_irq_work_handler);

	pr_info("IRQF_NO_THREAD Test\r\n");
	ret = request_irq(chip->irq, wusb3801_intr_handler,
		IRQF_TRIGGER_LOW | IRQF_NO_THREAD, name, chip);//IRQF_TRIGGER_FALLING IRQF_TRIGGER_LOW  modify EINT 41 is pending suppend die
	if (ret < 0) {
		pr_err("Error: failed to request irq%d (gpio = %d, ret = %d)\n",
			chip->irq, chip->irq_gpio, ret);
		goto init_alert_err;
	}
	reg_val = (1 << CTRL_INT_SHIFT);
	ret = wusb3801_update_reg(chip->client, WUSB3801_REG_CONTROL0, reg_val, CTRL_INT);
	if (ret < 0) {
		pr_err("%s: init WUSB3801_REG_CONTROL0 fail!\n", __func__);
		return ret;
	}
	/* get interrupt */
	//ret = wusb3801_read_reg(chip->client, WUSB3801_REG_INTERRUPT, &val);
	//if (ret < 0) {
	//	pr_err("%s: failed to read interrupt\n", __func__);
	//	return ret;
	//}
	//reg_val = (0 << CTRL_INT_SHIFT);
	//ret = wusb3801_update_reg(chip->client, WUSB3801_REG_CONTROL0, reg_val, CTRL_INT);
	//if (ret < 0) {
	//	pr_err("%s: init WUSB3801_REG_CONTROL0 fail!\n", __func__);
	//	return ret;
	//}
	//wusb3801_write_reg(chip->client,WUSB3801_REG_CONTROL1, 0x0);
	enable_irq_wake(chip->irq);
	//disable_irq(chip->irq);
	//wusb3801_write_reg(chip->client,WUSB3801_REG_CONTROL1, 0x1);
	return 0;
init_alert_err:
	return -EINVAL;
}

int wusb3801_alert_status_clear(struct tcpc_device *tcpc, uint32_t mask)
{
	//
	int ret = 0;
	if (ret < 0) {
		pr_err("%s: update reg fail!\n", __func__);
	}
        pr_info("%s enter \n",__func__);
	return 0;
}

static int wusb3801_tcpc_init(struct tcpc_device *tcpc, bool sw_reset)
{
    struct wusb3801_chip *chip = tcpc_get_dev_data(tcpc);   
	//struct wusb3801_chip *info = i2c_get_clientdata(chip->client);
	int ret = 0;
	u8 reg_val;
	//u8 val, reg_val;
	pr_info("%s enter \n",__func__);
	//int_sts = rc & WUSB3801_INT_STS_MASK;
	//pr_info("%s WUSB3801_REG_INTERRUPT : 0x%02x\n", __func__, reg_val); 
	//msleep(100);
	ret = wusb3801_read_reg(chip->client, WUSB3801_REG_STATUS, &reg_val);
	if (ret < 0) {
		pr_err("%s: failed to read reg status\n", __func__);
		return ret;
	}
	pr_info("%s WUSB3801_REG_STATUS : 0x%02x\n", __func__, reg_val);
	/* check current_detect */
	//val = ((reg_val & CC_STATUS_CHARGING_CURRENT_SNK) >> CC_STATUS_CHARGING_CURRENT_SNK_SHIFT);
	//chip->type_c_param.current_det = val;
	/* check attach state */
	//val = ((reg_val & CC_PLUG_PORT_STATUS) >> CC_PLUG_PORT_STATUS_SHIFT);
	//chip->type_c_param.attach_state = val;
	/* Try SNK/SRC [bit 7:6] */
	reg_val = (1 << CTRL_TRY_SNK_SRC_SHIFT);
	ret = wusb3801_update_reg(chip->client, WUSB3801_REG_CONTROL0, reg_val, CTRL_TRY_SNK_SRC);
	if (ret < 0) {
		pr_err("%s: init WUSB3801_REG_CONTROL0 fail!\n", __func__);
		return ret;
	}
	/* CURRENT MODE ADVERTISE 3A [bit 7:6] */
	reg_val = (HOST_CUR_3A << CTRL_CURRENT_MODE_SHIFT);
	ret = wusb3801_update_reg(chip->client, WUSB3801_REG_CONTROL0, reg_val, CTRL_CURRENT_MODE);
	if (ret < 0) {
		pr_err("%s: init WUSB3801_REG_CONTROL0 fail!\n", __func__);
		return ret;
	}
	tcpci_alert_status_clear(tcpc, 0xffffffff);
	wusb3801_init_power_status_mask(tcpc);
	wusb3801_init_alert_mask(tcpc);
	wusb3801_init_fault_mask(tcpc);
	wusb3801_init_rt_mask(tcpc);
	/* INT ENABLE [bit 0] */
    reg_val = (0 << CTRL_INT_SHIFT);
	ret = wusb3801_update_reg(chip->client, WUSB3801_REG_CONTROL0, reg_val, CTRL_INT);
	if (ret < 0) {
		pr_err("%s: init WUSB3801_REG_CONTROL0 fail!\n", __func__);
		return ret;
	}
	//RST int
	wusb3801_write_reg(chip->client,WUSB3801_REG_CONTROL1, 0x1);   
	msleep(5);
	return 0;
}

int wusb3801_fault_status_clear(struct tcpc_device *tcpc, uint8_t status)
{
    pr_info("%s enter \n",__func__);
	return 0;
}

int wusb3801_get_alert_mask(struct tcpc_device *tcpc, uint32_t *mask)
{
	*mask = 0;
	*mask |= (TCPC_REG_ALERT_CC_STATUS |
				TCPC_REG_ALERT_POWER_STATUS |
				TCPC_REG_ALERT_EXT_RA_DETACH);
	return 0;
}

/*****************************************************************
 * wusb3801 does not have alert register, compare type_c_param_old
 * with type_c_param for figuring out what has been changed.
 ****************************************************************/
int wusb3801_get_alert_status(struct tcpc_device *tcpc, uint32_t *alert)
{
	struct wusb3801_chip *chip = tcpc_get_dev_data(tcpc);
	#if 1
	u8 val, tmp, reg_val;
	int ret;
	#endif
	*alert = 0;
	//log上看出中断后产生的状态在*alert值不正确,直接改为寄存器中读取判断状态
	#if 1
	ret = wusb3801_read_reg(chip->client, WUSB3801_REG_INTERRUPT, &reg_val);
	if (ret < 0) {
		pr_err("%s err\n", __func__);
		//return;
	}
	tmp = reg_val;
	val = (tmp & INT_ATTACH_DETACH) >> INT_ATTACH_DETACH_SHIFT;
	chip->type_c_param.int_attach = val;
	ret = wusb3801_read_reg(chip->client, WUSB3801_REG_STATUS, &reg_val);
	if (ret < 0) {
		pr_err("%s err\n", __func__);
		//return 0;
	}
	tmp = reg_val;
	/* check current_detect */
	val = ((tmp & CC_STATUS_CHARGING_CURRENT_SNK) >> CC_STATUS_CHARGING_CURRENT_SNK_SHIFT);
	chip->type_c_param.current_det = val;
	val = ((tmp & CC_PLUG_PORT_STATUS) >> CC_PLUG_PORT_STATUS_SHIFT);
	chip->type_c_param.attach_state = val;
	if(chip->type_c_param.int_attach != INT_NOT_INT)
	{
		if(chip->type_c_param.attach_state != CABLE_STATE_NOT_ATTACHED)
		{
			*alert |= TCPC_REG_ALERT_CC_STATUS;
		}
		if(chip->type_c_param.attach_state == CABLE_STATE_AS_UFP)
		{
			*alert |= TCPC_REG_ALERT_POWER_STATUS;
			*alert |= TCPC_REG_ALERT_EXT_RA_DETACH;
		}
	}
	if(chip->type_c_param.int_attach == INT_NOT_ATTACHED)
	{
		*alert |= TCPC_REG_ALERT_CC_STATUS;
		*alert |= TCPC_REG_ALERT_POWER_STATUS;
		*alert |= TCPC_REG_ALERT_EXT_RA_DETACH;
	}
	
	/* update current adv when act as DFP */
	if (chip->type_c_param.attach_state == CABLE_STATE_AS_DFP ||
	    chip->type_c_param.attach_state == CABLE_STATE_TO_ACCESSORY) {
		val = (HOST_CUR_USB << CTRL_CURRENT_MODE_SHIFT);
	} else {
		val = (HOST_CUR_3A << CTRL_CURRENT_MODE_SHIFT);
	}
	wusb3801_update_reg(chip->client, WUSB3801_REG_CONTROL0, val, CTRL_CURRENT_MODE);
	#else
	/* cc status change */
	if ((chip->type_c_param.current_det != chip->type_c_param_old.current_det) ||
		//(chip->type_c_param.active_cable_attach != chip->type_c_param_old.active_cable_attach) ||
		(chip->type_c_param.attach_state != chip->type_c_param_old.attach_state) ||
		(chip->type_c_param.cable_dir != chip->type_c_param_old.cable_dir)) {
		*alert |= TCPC_REG_ALERT_CC_STATUS;
	}
	/* UFP <-> Non-UFP = Power present change */
	if (((chip->type_c_param.attach_state == CABLE_STATE_AS_UFP) &&
		(chip->type_c_param_old.attach_state != CABLE_STATE_AS_UFP)) ||
		((chip->type_c_param.attach_state != CABLE_STATE_AS_UFP) &&
		(chip->type_c_param_old.attach_state == CABLE_STATE_AS_UFP))) {
		*alert |= TCPC_REG_ALERT_POWER_STATUS;
		*alert |= TCPC_REG_ALERT_EXT_RA_DETACH;
	}
	/* Add more alert bits here if need */
	/* sync status to type_c_param_old */
	chip->type_c_param_old.current_det = chip->type_c_param.current_det;
	//chip->type_c_param_old.active_cable_attach = chip->type_c_param.active_cable_attach;
	chip->type_c_param_old.attach_state = chip->type_c_param.attach_state;
	chip->type_c_param_old.cable_dir = chip->type_c_param.cable_dir;
	//chip->type_c_param_old.accessory_attach = chip->type_c_param.accessory_attach;
	#endif
	printk("wusb3801_get_alert_status *alert=%x,attach_state=%d,int_attach=%d\n",*alert,chip->type_c_param.attach_state,chip->type_c_param.int_attach);
	return 0;
}

static int wusb3801_get_power_status(
		struct tcpc_device *tcpc, uint16_t *pwr_status)
{
	struct wusb3801_chip *chip = tcpc_get_dev_data(tcpc);
	u8 reg_val;
	u8 val;
	int ret;

	ret = wusb3801_read_reg(chip->client, WUSB3801_REG_STATUS, &reg_val);
	if (ret < 0) {
		pr_err("%s err\n", __func__);
		return ret;
	}
	val = ((reg_val & CC_STATUS_VBUS_DETECTION_SRC) >> CC_STATUS_VBUS_DETECTION_SRC_SHIFT);
	*pwr_status = 0;
	if (val & CC_STATUS_VBUS_DETECTION) {
		*pwr_status |= TCPC_REG_POWER_STATUS_VBUS_PRES;
	}
	//if(chip->type_c_param.attach_state & CABLE_STATE_AS_UFP)
	//{
	//	*pwr_status |= TCPC_REG_POWER_STATUS_VBUS_PRES;
	//}
	printk("wusb3801_get_power_status pwr_status=%d,val=%d\n",*pwr_status,reg_val);
	return 0;
}

int wusb3801_get_fault_status(struct tcpc_device *tcpc, uint8_t *status)
{

    pr_info("%s enter \n",__func__);
	return 0;
}

/*******************************************************************
 * Translate wusb3801 cc register's value to tcpc_cc_voltage_status.
 *******************************************************************/
static int wusb3801_get_cc(struct tcpc_device *tcpc, int *cc1, int *cc2)
{
	struct wusb3801_chip *chip = tcpc_get_dev_data(tcpc);
	int *active_cc, *secondary_cc;

	*cc1 = TYPEC_CC_VOLT_OPEN;
	*cc2 = TYPEC_CC_VOLT_OPEN;

	if (chip->type_c_param.attach_state == CABLE_STATE_NOT_ATTACHED) {
		pr_debug("cc not attached\n");
		return 0;
	}

	switch(chip->type_c_param.cable_dir)
	{
		case 1:
			active_cc = cc1;
			secondary_cc = cc2;
			break;
		case 2:
			active_cc = cc2;
			secondary_cc = cc1;
			break;
		default:
			active_cc = cc1;
			secondary_cc = cc2;
			break;
	}
	if (chip->type_c_param.attach_state == CABLE_STATE_AS_UFP) {
		switch (chip->type_c_param.current_det) {
		case 0: /* RP Default */
			*active_cc |= TYPEC_CC_VOLT_SNK_DFT;
			break;
		case 1: /* RP 1.5V */
			*active_cc |= TYPEC_CC_VOLT_SNK_1_5;
			break;
		case 3: /* RP 3.0V */
			*active_cc |= TYPEC_CC_VOLT_SNK_3_0;
			break;
		default:
			*active_cc |= TYPEC_CC_VOLT_SNK_DFT;
			break;
		}
 	} else if (chip->type_c_param.attach_state == CABLE_STATE_TO_ACCESSORY) {
			*active_cc |= TYPEC_CC_VOLT_RA;
			*secondary_cc |= TYPEC_CC_VOLT_RA;
 	} else if (chip->type_c_param.attach_state == CABLE_STATE_TO_DEBUG) {
			*active_cc |= TYPEC_CC_VOLT_RD;
			*secondary_cc |= TYPEC_CC_VOLT_RD;
	}
	else if (chip->type_c_param.attach_state == CABLE_STATE_AS_DFP) {
			*active_cc |= TYPEC_CC_VOLT_RD;
 	}
	printk("wusb3801_get_cc active_cc=%d,secondary_cc=%d\n",*active_cc,*secondary_cc);
	return 0;
}

static int wusb3801_set_cc(struct tcpc_device *tcpc, int pull)
{
	struct wusb3801_chip *chip = tcpc_get_dev_data(tcpc);
	int ret;
	u8 value = SET_MODE_SELECT_SNK;
	
	if (pull == TYPEC_CC_RP)
		value = SET_MODE_SELECT_SRC;
	else if (pull == TYPEC_CC_RD)
		value = SET_MODE_SELECT_SNK;
	else if (pull == TYPEC_CC_DRP)
		value = SET_MODE_SELECT_DRP;
	printk("wusb3801_set_cc pull=%d,%d\n",pull,value);
	value = value << CTRL_WORK_MODE_SHIFT;
	ret = wusb3801_update_reg(chip->client, WUSB3801_REG_CONTROL0, value, CTRL_WORK_MODE_SHIFT);
	if (ret < 0) {
		pr_err("%s: update reg fail!\n", __func__);
	}

	return 0;
}

static int wusb3801_set_polarity(struct tcpc_device *tcpc, int polarity)
{
        pr_info("%s enter \n",__func__);
	return 0;
}

static int wusb3801_set_low_rp_duty(struct tcpc_device *tcpc, bool low_rp)
{
        pr_info("%s enter \n",__func__);
	return 0;
}

static int wusb3801_set_vconn(struct tcpc_device *tcpc, int enable)
{
        pr_info("%s enter \n",__func__);
	return 0;
}

static int wusb3801_tcpc_deinit(struct tcpc_device *tcpc_dev)
{
        pr_info("%s enter \n",__func__);
	return 0;
}


static struct tcpc_ops wusb3801_tcpc_ops = {
	.init = wusb3801_tcpc_init,
	.alert_status_clear = wusb3801_alert_status_clear,
	.fault_status_clear = wusb3801_fault_status_clear,
	.get_alert_mask = wusb3801_get_alert_mask,
	.get_alert_status = wusb3801_get_alert_status,
	.get_power_status = wusb3801_get_power_status,
	.get_fault_status = wusb3801_get_fault_status,
	.get_cc = wusb3801_get_cc,
	.set_cc = wusb3801_set_cc,
	.set_polarity = wusb3801_set_polarity,
	.set_low_rp_duty = wusb3801_set_low_rp_duty,
	.set_vconn = wusb3801_set_vconn,
	.deinit = wusb3801_tcpc_deinit,
};


static int mt_parse_dt(struct wusb3801_chip *chip, struct device *dev)
{
	struct device_node *np = dev->of_node;
	int ret;

	if (!np)
		return -EINVAL;

	pr_info("%s\n", __func__);

	np = of_find_node_by_name(NULL, "wusb3801_type_c_port0");
	if (!np) {
		pr_err("%s find node type_c_port0 fail\n", __func__);
		return -ENODEV;
	}
	dev->of_node = np;

#if (!defined(CONFIG_MTK_GPIO) || defined(CONFIG_MTK_GPIOLIB_STAND))
	ret = of_get_named_gpio(np, "wusb3801,intr_gpio", 0);
	if (ret < 0) {
		pr_err("%s no intr_gpio info\n", __func__);
		return ret;
	}
	chip->irq_gpio = ret;
#else
	ret = of_property_read_u32(
		np, "wusb3801,intr_gpio_num", &chip->irq_gpio);
	if (ret < 0)
		pr_err("%s no intr_gpio info\n", __func__);
#endif

	return ret;
}
static int wusb3801_tcpcdev_init(struct wusb3801_chip *chip, struct device *dev)
{
	struct tcpc_desc *desc;
	struct device_node *np = dev->of_node;
	u32 val, len;
   // int ret;

	const char *name = "default";

	np = of_find_node_by_name(NULL, "wusb3801_type_c_port0");
	if (!np) {
		pr_err("%s find node mt6370 fail\n", __func__);
		return -ENODEV;
	}

	desc = devm_kzalloc(dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;
	if (of_property_read_u32(np, "wusb3801-tcpc,role_def", &val) >= 0) {
		if (val >= TYPEC_ROLE_NR)
			desc->role_def = TYPEC_ROLE_DRP;
		else
			desc->role_def = val;
	} else {
		dev_info(dev, "use default Role DRP\n");
		desc->role_def = TYPEC_ROLE_DRP;
	}

	if (of_property_read_u32(
		np, "wusb3801-tcpc,notifier_supply_num", &val) >= 0) {
		if (val < 0)
			desc->notifier_supply_num = 0;
		else
			desc->notifier_supply_num = val;
	} else
		desc->notifier_supply_num = 0;

	if (of_property_read_u32(np, "wusb3801-tcpc,rp_level", &val) >= 0) {
		switch (val) {
		case 0: /* RP Default */
			desc->rp_lvl = TYPEC_CC_RP_DFT;
			break;
		case 1: /* RP 1.5V */
			desc->rp_lvl = TYPEC_CC_RP_1_5;
			break;
		case 2: /* RP 3.0V */
			desc->rp_lvl = TYPEC_CC_RP_3_0;
			break;
		default:
			break;
		}
	}


	of_property_read_string(np, "wusb3801-tcpc,name", (char const **)&name);

	len = strlen(name);
	desc->name = kzalloc(len+1, GFP_KERNEL);
	if (!desc->name)
		return -ENOMEM;

	strlcpy((char *)desc->name, name, len+1);

	chip->tcpc_desc = desc;

	chip->tcpc = tcpc_device_register(dev,
			desc, &wusb3801_tcpc_ops, chip);
	if (IS_ERR(chip->tcpc))
		return -EINVAL;

	chip->tcpc->tcpc_flags = TCPC_FLAGS_LPM_WAKEUP_WATCHDOG;
	return 0;
}

static inline int wusb3801_check_revision(struct i2c_client *client)
{
	int rc;
	rc = i2c_smbus_read_byte_data(client, WUSB3801_REG_VERSION_ID);
	if (rc < 0)
		return rc;

	pr_info("VendorID register: 0x%02x\n", rc );

	return rc;
}


static int wusb3801_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct wusb3801_chip *chip;
	int ret = 0, chip_id;
	bool use_dt = client->dev.of_node;
	#ifdef IC_TEST_DEV
	struct device *cdev = &client->dev;
	#endif
	pr_info("%s\n", __func__);
	client->addr = 0x60;
	if (i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_I2C_BLOCK | I2C_FUNC_SMBUS_BYTE_DATA))
		pr_info("I2C functionality : OK...\n");
	else
		pr_info("I2C functionality check : failuare...\n");

	chip_id = wusb3801_check_revision(client);
	if (chip_id < 0)
		return chip_id;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	if (use_dt)
		mt_parse_dt(chip, &client->dev);
	else {
		dev_err(&client->dev, "no dts node\n");
		return -ENODEV;
	}
	chip->dev = &client->dev;
	chip->client = client;
	sema_init(&chip->io_lock, 1);
	sema_init(&chip->suspend_lock, 1);
	mutex_init(&chip->mutex);
	i2c_set_clientdata(client, chip);

	INIT_DELAYED_WORK(&chip->poll_work, wusb3801_poll_work);
	wakeup_source_init(&chip->irq_wake_lock,
		"wusb3801_irq_wakelock");

	chip->chip_id = chip_id;
	pr_info("wusb3801_chipID = 0x%0x\n", chip_id);

	ret = wusb3801_tcpcdev_init(chip, &client->dev);
	if (ret < 0) {
		dev_err(&client->dev, "wusb3801 tcpc dev init fail\n");
		return -EINVAL;
	}

	ret = wusb3801_init_alert(chip->tcpc);
	if (ret < 0) {
		pr_err("wusb3801 init alert fail\n");
		goto err_irq_init;
	}

	tcpc_schedule_init_work(chip->tcpc);
	pr_info("%s probe OK!\n", __func__);
#ifdef __TEST_CC_PATCH__
	//huanglei add for reg 0x08& 0x0F write zero fail begin
    wusb3801_i2c_write8(chip->tcpc,
        WUSB3801_REG_TEST_02, 0x00);
    wusb3801_i2c_write8(chip->tcpc,
        WUSB3801_REG_TEST_09, 0x00);
//huanglei add for reg 0x08& 0x0F write zero fail end  
#endif
#ifdef IC_TEST_DEV
	ret = device_create_file(cdev, &dev_attr_fregdump);
	if (ret < 0) {
		dev_err(cdev, "failed to create dev_attr_fregdump\n");
		device_remove_file(cdev, &dev_attr_fregdump);
		return -ENODEV;
	}
#endif


	return 0;

err_irq_init:
	tcpc_device_unregister(chip->dev, chip->tcpc);
	mutex_destroy(&chip->mutex);
	wakeup_source_trash(&chip->irq_wake_lock);
	return ret;
}

static int wusb3801_i2c_remove(struct i2c_client *client)
{
	struct wusb3801_chip *chip = i2c_get_clientdata(client);

	if (chip) {
		cancel_delayed_work_sync(&chip->poll_work);
		tcpc_device_unregister(chip->dev, chip->tcpc);
		#ifdef IC_TEST_DEV
		device_remove_file(chip->dev, &dev_attr_fregdump);
		#endif
	}
	mutex_destroy(&chip->mutex);
	return 0;
}

#ifdef CONFIG_PM
static int wusb3801_i2c_suspend(struct device *dev)
{
	struct wusb3801_chip *chip;
	struct i2c_client *client = to_i2c_client(dev);

	if (client) {
		chip = i2c_get_clientdata(client);
		if (chip) {
			down(&chip->suspend_lock);
		}
	}

	return 0;
}

static int wusb3801_i2c_resume(struct device *dev)
{
	struct wusb3801_chip *chip;
	struct i2c_client *client = to_i2c_client(dev);

	if (client) {
		chip = i2c_get_clientdata(client);
		if (chip)
			up(&chip->suspend_lock);
	}

	return 0;
}

static void wusb3801_shutdown(struct i2c_client *client)
{
	struct wusb3801_chip *chip = i2c_get_clientdata(client);

	/* Please reset IC here */
	if (chip != NULL) {
		if (chip->irq)
			disable_irq(chip->irq);
		tcpm_shutdown(chip->tcpc);
	}
}

#ifdef CONFIG_PM_RUNTIME
static int wusb3801_pm_suspend_runtime(struct device *device)
{
	dev_dbg(device, "pm_runtime: suspending...\n");
	return 0;
}

static int wusb3801_pm_resume_runtime(struct device *device)
{
	dev_dbg(device, "pm_runtime: resuming...\n");
	return 0;
}
#endif /* CONFIG_PM_RUNTIME */


static const struct dev_pm_ops wusb3801_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(
			wusb3801_i2c_suspend,
			wusb3801_i2c_resume)
#ifdef CONFIG_PM_RUNTIME
	SET_RUNTIME_PM_OPS(
		wusb3801_pm_suspend_runtime,
		wusb3801_pm_resume_runtime,
		NULL
	)
#endif /* CONFIG_PM_RUNTIME */
};
#define wusb3801_PM_OPS	(&wusb3801_pm_ops)
#else
#define wusb3801_PM_OPS	(NULL)
#endif /* CONFIG_PM */

static const struct i2c_device_id wusb3801_id_table[] = {
	{"wusb3801", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, wusb3801_id_table);

static const struct of_device_id rt_match_table[] = {
	{.compatible = "wusb3801,usb_type_c",},
	{},
};

static struct i2c_driver wusb3801_driver = {
	.driver = {
		.name = "usb_type_c0",
		.owner = THIS_MODULE,
		.of_match_table = rt_match_table,
		.pm = wusb3801_PM_OPS,
	},
	.probe = wusb3801_i2c_probe,
	.remove = wusb3801_i2c_remove,
	.shutdown = wusb3801_shutdown,
	.id_table = wusb3801_id_table,
};

static int __init wusb3801_init(void)
{
	struct device_node *np;

	pr_info("%s (%s): initializing...\n", __func__, WUSB3801_DRV_VERSION);
	np = of_find_node_by_name(NULL, "usb_type_c");
	if (np != NULL)
		pr_info("usb_type_c node found...\n");
	else
		pr_info("usb_type_c node not found...\n");

	return i2c_add_driver(&wusb3801_driver);
}
subsys_initcall(wusb3801_init);

static void __exit wusb3801_exit(void)
{
	i2c_del_driver(&wusb3801_driver);
}
module_exit(wusb3801_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("wusb3801 TCPC Driver");
MODULE_VERSION(WUSB3801_DRV_VERSION);
