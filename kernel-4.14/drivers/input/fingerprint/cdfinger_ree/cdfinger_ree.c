#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/ioport.h>
#include <linux/errno.h>
#include <linux/spi/spi.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#ifdef CONFIG_PM_WAKELOCKS
#include <linux/pm_wakeup.h>
#else
#include <linux/wakelock.h>
#endif
#include <linux/kthread.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/spi/spidev.h>
#include <linux/semaphore.h>
#include <linux/poll.h>
#include <linux/fcntl.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/input.h>
#include <linux/signal.h>
#include <linux/gpio.h>
#include <linux/mm.h>
#include <linux/of_gpio.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
//#include <mtk_spi.h>
#include <linux/of_irq.h>
#include <linux/regulator/consumer.h>
#include <linux/fb.h>
#include <linux/notifier.h>
#include <linux/semaphore.h>

typedef struct key_report
{
	int key;
	int value;
} key_report_t;

struct WRITE_THEN_READ_STR
{
	unsigned char *Prxbuf;
	unsigned char *Ptxbuf;
	unsigned int rxlen;
	unsigned int txlen;
};

#define CDFINGER_DBG(fmt, args...)                                                      \
	do                                                                                  \
	{                                                                                   \
		if (cdfinger_debug & 0x01)                                                      \
			printk("fingertech [DBG][cdfinger]:%5d: <%s>" fmt, __LINE__, __func__, ##args); \
	} while (0)
#define CDFINGER_FUNCTION(fmt, args...)                                                 \
	do                                                                                  \
	{                                                                                   \
		if (cdfinger_debug & 0x02)                                                      \
			printk("fingertech [DBG][cdfinger]:%5d: <%s>" fmt, __LINE__, __func__, ##args); \
	} while (0)
#define CDFINGER_REG(fmt, args...)                                                      \
	do                                                                                  \
	{                                                                                   \
		if (cdfinger_debug & 0x04)                                                      \
			printk("fingertech [DBG][cdfinger]:%5d: <%s>" fmt, __LINE__, __func__, ##args); \
	} while (0)
#define CDFINGER_ERR(fmt, args...)                                                  \
	do                                                                              \
	{                                                                               \
		printk("fingertech [DBG][cdfinger]:%5d: <%s>" fmt, __LINE__, __func__, ##args); \
	} while (0)

#define HAS_RESET_PIN
#define DTS_PROBE

#define VERSION "cdfinger version 3.1"
#define DEVICE_NAME "fpsdev0"
#define SPI_DRV_NAME "cdfinger"

#define FPS998                             0x70
#define FPS958E                            0x56
#define FPS988E                            0x80
#define FPS1356E                           0x80
#define FPS1256E                           0x56
#define FPS1256S                           0x54
#define FPS998E                            0x98
#define FPS998EAS                          0x96
#define FPS980S                            0x88
#define FPS1735							   0x35




#define CDFINGER_IOCTL_MAGIC_NO 0xFB

#define CDFINGER_INITERRUPT_MODE _IOW(CDFINGER_IOCTL_MAGIC_NO, 2, uint8_t)
#define CDFINGER_GETID           _IO(CDFINGER_IOCTL_MAGIC_NO,12)
#define CDFINGER_HW_RESET _IOW(CDFINGER_IOCTL_MAGIC_NO, 14, uint8_t)
#define CDFINGER_GET_STATUS _IO(CDFINGER_IOCTL_MAGIC_NO, 15)
#define CDFINGER_SPI_WRITE_AND_READ _IOWR(CDFINGER_IOCTL_MAGIC_NO, 18, struct WRITE_THEN_READ_STR)
#define CDFINGER_REPORT_KEY _IOW(CDFINGER_IOCTL_MAGIC_NO, 19, key_report_t)
#define CDFINGER_INIT_GPIO _IO(CDFINGER_IOCTL_MAGIC_NO, 20)
#define CDFINGER_INIT_IRQ _IO(CDFINGER_IOCTL_MAGIC_NO, 21)
#define CDFINGER_POWER_ON _IO(CDFINGER_IOCTL_MAGIC_NO, 22)
#define CDFINGER_RESET _IO(CDFINGER_IOCTL_MAGIC_NO, 23)
#define CDFINGER_RELEASE_DEVICE _IO(CDFINGER_IOCTL_MAGIC_NO, 25)
#define CDFINGER_WAKE_LOCK _IOW(CDFINGER_IOCTL_MAGIC_NO, 26, uint8_t)
#define CDFINGER_POLL_TRIGGER _IO(CDFINGER_IOCTL_MAGIC_NO, 31)
#define CDFINGER_NEW_KEYMODE _IOW(CDFINGER_IOCTL_MAGIC_NO, 37, uint8_t)
#define CDFINGER_CONTROL_IRQ _IOW(CDFINGER_IOCTL_MAGIC_NO, 38, uint8_t)
#define CDFINGER_CHANGER_CLK_FREQUENCY _IOW(CDFINGER_IOCTL_MAGIC_NO, 39, uint32_t)

static u8 cdfinger_debug = 0x01;
static int fb_status = 1;   // screen on
static int isInKeyMode = 1; // not key mode
static int isInit = 0;
static int sign_sync = 0; // for poll
static DECLARE_WAIT_QUEUE_HEAD(cdfinger_waitqueue);

/* prize added by chenjiaxi, cdfinger ree info, 20200111-start */
#if defined(CONFIG_PRIZE_HARDWARE_INFO)
#include "../../../misc/mediatek/hardware_info/hardware_info.h"
extern struct hardware_info current_fingerprint_info;
#endif
/* prize added by chenjiaxi, cdfinger ree info, 20200111-end */

enum spi_speed
{
	CDFINGER_SPI_4M1 = 1,
	CDFINGER_SPI_4M4,
	CDFINGER_SPI_4M7,
	CDFINGER_SPI_5M1,
	CDFINGER_SPI_5M5,
	CDFINGER_SPI_6M1,
	CDFINGER_SPI_6M7,
	CDFINGER_SPI_7M4,
	CDFINGER_SPI_8M
};

static struct cdfinger_data
{
	struct spi_device *spi;
	unsigned int spi_frequence;
	unsigned int last_spi_frequence;
	struct mutex buf_lock;
	struct miscdevice *miscdev;
	struct mutex transfer_lock;
	unsigned int irq;
	int irq_enabled;
	u8  sensor_type;

	u32 vdd_ldo_enable;
	u32 vio_ldo_enable;
	u32 config_spi_pin;
#if defined(CONFIG_PRIZE_FP_USE_VFP)
	struct regulator *vdd_reg;
#endif
	int pwr_gpio; //power gpio number
	int key_report;

	struct pinctrl *fps_pinctrl;
	struct pinctrl_state *fps_reset_high;
	struct pinctrl_state *fps_reset_low;
	struct pinctrl_state *fps_power_on;
	struct pinctrl_state *fps_power_off;
	struct pinctrl_state *fps_vio_on;
	struct pinctrl_state *fps_vio_off;
	struct pinctrl_state *cdfinger_spi_miso;
	struct pinctrl_state *cdfinger_spi_mosi;
	struct pinctrl_state *cdfinger_spi_sck;
	struct pinctrl_state *cdfinger_spi_cs;
	struct pinctrl_state *cdfinger_irq;

	struct input_dev *cdfinger_inputdev;
#ifdef CONFIG_PM_WAKELOCKS
 struct wakeup_source cdfinger_lock;
#else
	struct wake_lock cdfinger_lock;
#endif
	struct fasync_struct *async_queue;
	u8 last_transfer;
	struct notifier_block notifier;
} * g_cdfinger;

static DECLARE_WAIT_QUEUE_HEAD(waiter);
/*static struct mt_chip_conf spi_conf = {
	.setuptime = 7,
	.holdtime = 7,
	.high_time = 13,
	.low_time = 13,
	.cs_idletime = 6,
	.cpol = 0,
	.cpha = 0,
	.rx_mlsb = 1,
	.tx_mlsb = 1,
	.tx_endian = 0,
	.rx_endian = 0,
	.com_mod = DMA_TRANSFER,
	.pause = 1,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};
*/
static void cdfinger_disable_irq(struct cdfinger_data *cdfinger)
{
	if (cdfinger->irq_enabled == 1)
	{
		//disable_irq_nosync(cdfinger->irq);
		cdfinger->irq_enabled = 0;
		CDFINGER_DBG("irq disable\n");
	}
}

static void cdfinger_enable_irq(struct cdfinger_data *cdfinger)
{
	if (cdfinger->irq_enabled == 0)
	{
		//enable_irq(cdfinger->irq);
		cdfinger->irq_enabled = 1;
		CDFINGER_DBG("irq enable\n");
	}
}

static int cdfinger_parse_dts(struct cdfinger_data *cdfinger)
{
	int ret = -1;

	if (cdfinger->spi == NULL)
	{
		CDFINGER_ERR("spi is NULL !\n");
		goto parse_err;
	}

#ifdef DTS_PROBE
	cdfinger->spi->dev.of_node = of_find_compatible_node(NULL, NULL, "mediatek,mt6765-fpc");//prize "mediatek,fps1098");
#endif

	if (!(cdfinger->spi->dev.of_node))
	{
		CDFINGER_ERR("of node not exist!\n");
		goto parse_err;
	}

	cdfinger->irq = irq_of_parse_and_map(cdfinger->spi->dev.of_node, 0);
	if (cdfinger->irq < 0)
	{
		CDFINGER_ERR("parse irq failed! irq[%d]\n", cdfinger->irq);
		goto parse_err;
	}

	of_property_read_u32(cdfinger->spi->dev.of_node, "vdd_ldo_enable", &cdfinger->vdd_ldo_enable);
	of_property_read_u32(cdfinger->spi->dev.of_node, "vio_ldo_enable", &cdfinger->vio_ldo_enable);
	of_property_read_u32(cdfinger->spi->dev.of_node, "config_spi_pin", &cdfinger->config_spi_pin);

	CDFINGER_DBG("irq[%d], vdd_ldo_enable[%d], vio_ldo_enable[%d], config_spi_pin[%d]\n",
				 cdfinger->irq, cdfinger->vdd_ldo_enable, cdfinger->vio_ldo_enable, cdfinger->config_spi_pin);

	cdfinger->fps_pinctrl = devm_pinctrl_get(&cdfinger->spi->dev);
	if (IS_ERR(cdfinger->fps_pinctrl))
	{
		ret = PTR_ERR(cdfinger->fps_pinctrl);
		CDFINGER_ERR("Cannot find fingerprint cdfinger->fps_pinctrl! ret=%d\n", ret);
		goto parse_err;
	}

	cdfinger->cdfinger_irq = pinctrl_lookup_state(cdfinger->fps_pinctrl, "fpc_eint_as_int");//prize "fingerprint_irq");
	if (IS_ERR(cdfinger->cdfinger_irq))
	{
		ret = PTR_ERR(cdfinger->cdfinger_irq);
		CDFINGER_ERR("cdfinger->cdfinger_irq ret = %d\n", ret);
		goto parse_err;
	}
	cdfinger->fps_reset_low = pinctrl_lookup_state(cdfinger->fps_pinctrl, "fpc_pins_rst_low");//prize "fingerprint_reset_low");
	if (IS_ERR(cdfinger->fps_reset_low))
	{
		ret = PTR_ERR(cdfinger->fps_reset_low);
		CDFINGER_ERR("cdfinger->fps_reset_low ret = %d\n", ret);
		goto parse_err;
	}
	cdfinger->fps_reset_high = pinctrl_lookup_state(cdfinger->fps_pinctrl, "fpc_pins_rst_high");//prize "fingerprint_reset_high");
	if (IS_ERR(cdfinger->fps_reset_high))
	{
		ret = PTR_ERR(cdfinger->fps_reset_high);
		CDFINGER_ERR("cdfinger->fps_reset_high ret = %d\n", ret);
		goto parse_err;
	}

	if (cdfinger->config_spi_pin == 1)
	{
		cdfinger->cdfinger_spi_miso = pinctrl_lookup_state(cdfinger->fps_pinctrl, "fpc_mode_as_mi");//prize "fingerprint_spi_miso");
		if (IS_ERR(cdfinger->cdfinger_spi_miso))
		{
			ret = PTR_ERR(cdfinger->cdfinger_spi_miso);
			CDFINGER_ERR("cdfinger->cdfinger_spi_miso ret = %d\n", ret);
			goto parse_err;
		}
		cdfinger->cdfinger_spi_mosi = pinctrl_lookup_state(cdfinger->fps_pinctrl, "fpc_mode_as_mo");//prize "fingerprint_spi_mosi");
		if (IS_ERR(cdfinger->cdfinger_spi_mosi))
		{
			ret = PTR_ERR(cdfinger->cdfinger_spi_mosi);
			CDFINGER_ERR("cdfinger->cdfinger_spi_mosi ret = %d\n", ret);
			goto parse_err;
		}
		cdfinger->cdfinger_spi_sck = pinctrl_lookup_state(cdfinger->fps_pinctrl, "fpc_mode_as_ck");//prize "fingerprint_spi_sck");
		if (IS_ERR(cdfinger->cdfinger_spi_sck))
		{
			ret = PTR_ERR(cdfinger->cdfinger_spi_sck);
			CDFINGER_ERR("cdfinger->cdfinger_spi_sck ret = %d\n", ret);
			goto parse_err;
		}
		cdfinger->cdfinger_spi_cs = pinctrl_lookup_state(cdfinger->fps_pinctrl, "fpc_mode_as_cs");//prize "fingerprint_spi_cs");
		if (IS_ERR(cdfinger->cdfinger_spi_cs))
		{
			ret = PTR_ERR(cdfinger->cdfinger_spi_cs);
			CDFINGER_ERR("cdfinger->cdfinger_spi_cs ret = %d\n", ret);
			goto parse_err;
		}
	}

	if (cdfinger->vdd_ldo_enable == 1)
	{
		cdfinger->fps_power_on = pinctrl_lookup_state(cdfinger->fps_pinctrl,"fingerprint_power_high");
		if (IS_ERR(cdfinger->fps_power_on))
		{
			ret = PTR_ERR(cdfinger->fps_power_on);
			CDFINGER_ERR("cdfinger->fps_power_on ret = %d\n",ret);
			goto parse_err;
		}

		cdfinger->fps_power_off = pinctrl_lookup_state(cdfinger->fps_pinctrl,"fingerprint_power_low");
		if (IS_ERR(cdfinger->fps_power_off))
		{
			ret = PTR_ERR(cdfinger->fps_power_off);
			CDFINGER_ERR("cdfinger->fps_power_off ret = %d\n",ret);
			goto parse_err;
		}
	}

	if (cdfinger->vio_ldo_enable == 1)
	{
		cdfinger->fps_vio_on = pinctrl_lookup_state(cdfinger->fps_pinctrl, "fingerprint_vio_high");
		if (IS_ERR(cdfinger->fps_vio_on))
		{
			ret = PTR_ERR(cdfinger->fps_vio_on);
			CDFINGER_ERR("cdfinger->fps_vio_on ret = %d\n", ret);
			goto parse_err;
		}

		cdfinger->fps_vio_off = pinctrl_lookup_state(cdfinger->fps_pinctrl, "fingerprint_vio_low");
		if (IS_ERR(cdfinger->fps_vio_off))
		{
			ret = PTR_ERR(cdfinger->fps_vio_off);
			CDFINGER_ERR("cdfinger->fps_vio_off ret = %d\n", ret);
			goto parse_err;
		}
	}

	return 0;
parse_err:
	CDFINGER_ERR("parse dts failed!\n");

	return ret;
}

static int spi_send_cmd(struct cdfinger_data *cdfinger, u8 *tx, u8 *rx, u32 spilen)
{
	struct spi_message m;
	//struct mt_chip_conf *spiconf = &spi_conf;
	struct spi_transfer t = {
		.tx_buf = tx,
		.rx_buf = rx,
		.len = spilen,
	};

	//CDFINGER_DBG("transfer msg[0x%x]\n", tx[0]);

	if (tx[0] == 0x14 && 0x14 == cdfinger->last_transfer)
	{
		CDFINGER_DBG("Warning: transfer is same as last transfer.now[0x%x], last[0x%x]\n", tx[0], cdfinger->last_transfer);
		return 0;
	}
	cdfinger->last_transfer = tx[0];
	
/*	if(spilen > 8)
	{
		if(spiconf->com_mod != DMA_TRANSFER)
		{
			spiconf->com_mod = DMA_TRANSFER;
			spi_setup(cdfinger->spi);
		}
	}
	else
	{
		if(spiconf->com_mod != FIFO_TRANSFER)
		{
			spiconf->com_mod = FIFO_TRANSFER;
			spi_setup(cdfinger->spi);
		}
	}*/

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spi_sync(cdfinger->spi, &m);
}

static int spi_send_cmd_fifo(struct cdfinger_data *cdfinger, u8 *tx, u8 *rx, u32 spilen)
{
	int ret = 0;

	mutex_lock(&cdfinger->transfer_lock);
	ret = spi_send_cmd(cdfinger, tx, rx, spilen);
	mutex_unlock(&cdfinger->transfer_lock);
	udelay(100);

	return ret;
}

static int cdfinger_power_on(struct cdfinger_data *cdfinger)
{
	int ret = 0;
	
#if defined(CONFIG_PRIZE_FP_USE_VFP)
	if (!IS_ERR_OR_NULL(cdfinger->vdd_reg)){
		ret = regulator_enable(cdfinger->vdd_reg);
        if (ret) {
            CDFINGER_ERR("Regulator vdd enable failed ret = %d\n", ret);
            return ret;
        }
	};
#else
	if (cdfinger->vdd_ldo_enable == 1)
	{
		pinctrl_select_state(cdfinger->fps_pinctrl, cdfinger->fps_power_on);
	}
#endif

	if (cdfinger->vio_ldo_enable == 1)
	{
		pinctrl_select_state(cdfinger->fps_pinctrl, cdfinger->fps_vio_on);
	}
	msleep(10);
	return ret;
}

static void cdfinger_power_off(struct cdfinger_data *cdfinger)
{
#if defined(CONFIG_PRIZE_FP_USE_VFP)
	int ret = 0;
	
	ret = regulator_disable(cdfinger->vdd_reg);
	if (ret) {
		CDFINGER_ERR("Regulator vdd disable failed ret = %d\n", ret);
		return;
	}
#else
	if (cdfinger->vdd_ldo_enable == 1)
	{
		pinctrl_select_state(cdfinger->fps_pinctrl, cdfinger->fps_power_off);
		//gpio_direction_output(cdfinger->pwr_gpio, 0);
	}
#endif

	if (cdfinger->vio_ldo_enable == 1)
	{
		pinctrl_select_state(cdfinger->fps_pinctrl, cdfinger->fps_vio_off);
	}
}

static void cdfinger_reset(struct cdfinger_data *cdfinger, int count)
{
#ifdef HAS_RESET_PIN
	pinctrl_select_state(cdfinger->fps_pinctrl, cdfinger->fps_reset_high);
	mdelay(count);
	pinctrl_select_state(cdfinger->fps_pinctrl, cdfinger->fps_reset_low);
	mdelay(count);
	pinctrl_select_state(cdfinger->fps_pinctrl, cdfinger->fps_reset_high);
	mdelay(count);
#endif

	return;
}

int cdfinger_report_key(struct cdfinger_data *cdfinger, unsigned long arg)
{
	key_report_t report;

	CDFINGER_FUNCTION("enter\n");
	if (copy_from_user(&report, (key_report_t *)arg, sizeof(key_report_t)))
	{
		CDFINGER_ERR("%s err\n", __func__);
		return -1;
	}
	input_report_key(cdfinger->cdfinger_inputdev, report.key, !!report.value);
	input_sync(cdfinger->cdfinger_inputdev);

	CDFINGER_FUNCTION("exit\n");
	return 0;
}

static int cdfinger_init_gpio(struct cdfinger_data *cdfinger)
{
	if (cdfinger->config_spi_pin == 1)
	{
		pinctrl_select_state(cdfinger->fps_pinctrl, cdfinger->cdfinger_spi_miso);
		pinctrl_select_state(cdfinger->fps_pinctrl, cdfinger->cdfinger_spi_mosi);
		pinctrl_select_state(cdfinger->fps_pinctrl, cdfinger->cdfinger_spi_sck);
		pinctrl_select_state(cdfinger->fps_pinctrl, cdfinger->cdfinger_spi_cs);
	}

	return 0;
}

static int cdfinger_free_gpio(struct cdfinger_data *cdfinger)
{
	int ret = 0;

	CDFINGER_FUNCTION("enter\n");
	devm_pinctrl_put(cdfinger->fps_pinctrl);
	CDFINGER_FUNCTION("exit\n");

	return ret;
}

static void cdfinger_async_report(void)
{
	struct cdfinger_data *cdfinger = g_cdfinger;

	CDFINGER_FUNCTION("enter\n");
#ifdef CONFIG_PM_WAKELOCKS
	__pm_wakeup_event(&cdfinger->cdfinger_lock, jiffies_to_msecs(3*HZ));
#else
	wake_lock_timeout(&cdfinger->cdfinger_lock,3*HZ);
#endif
	sign_sync = 1;
	wake_up_interruptible(&cdfinger_waitqueue);
	kill_fasync(&cdfinger->async_queue, SIGIO, POLL_IN);
	CDFINGER_FUNCTION("exit\n");

	return;
}

static irqreturn_t cdfinger_interrupt_handler(int irq, void *arg)
{
	struct cdfinger_data *cdfinger = g_cdfinger;

	if (1 == cdfinger->irq_enabled)
	{
		cdfinger_async_report();
	}

	return IRQ_HANDLED;
}

static int cdfinger_init_irq(struct cdfinger_data *cdfinger)
{
	unsigned int status = 0;
	if (isInit == 1)
		return 0;
	pinctrl_select_state(cdfinger->fps_pinctrl, cdfinger->cdfinger_irq);

	status = request_threaded_irq(cdfinger->irq, cdfinger_interrupt_handler, NULL,
								  IRQF_TRIGGER_RISING | IRQF_ONESHOT, "cdfinger-irq", cdfinger);
	if (status)
	{
		CDFINGER_ERR("request_irq error\n");
	}

	enable_irq_wake(cdfinger->irq);
	cdfinger->irq_enabled = 1;
	isInit = 1;
	return status;
}

static int cdfinger_spi_transmission(struct cdfinger_data *cdfinger, unsigned long arg)
{
	int ret = 0;

	struct WRITE_THEN_READ_STR spi_transmission;
	unsigned char *tx = NULL;
	unsigned char *rx = NULL;

	if (copy_from_user(&spi_transmission, (struct WRITE_THEN_READ_STR *)arg, sizeof(struct WRITE_THEN_READ_STR)))
	{
		CDFINGER_ERR("%s  copy_from_user err\n", __func__);
		return -1;
	}
	//CDFINGER_ERR("%s  spi_transmission.txbuf[0]=0x%x, spi_transmission.txlen=%d\n", __func__, spi_transmission.Ptxbuf[0], spi_transmission.txlen);

	tx = kzalloc(spi_transmission.txlen, GFP_KERNEL);
	if (NULL == tx)
	{
		CDFINGER_ERR("%s  tx kzalloc err\n", __func__);
		ret = -1;
	}
	rx = kzalloc(spi_transmission.rxlen, GFP_KERNEL);
	if (NULL == rx)
	{
		CDFINGER_ERR("%s  rx kzalloc err\n", __func__);
		ret = -1;
		goto free_tx;
	}

	if (copy_from_user(tx, spi_transmission.Ptxbuf, spi_transmission.txlen))
	{
		CDFINGER_ERR("%s  copy_from_user err\n", __func__);
		ret = -1;
		goto free_rx;
	}
	ret = spi_send_cmd_fifo(cdfinger, tx, rx, spi_transmission.txlen);
	if (ret != 0)
	{
		CDFINGER_ERR(" cdfinger_spi_transmission spi transfer fail\n");
		ret = -1;
		goto free_rx;
	}
	//CDFINGER_ERR("spi_transmission.rx[0]=0x%x",rx[0]);

	if (copy_to_user(spi_transmission.Prxbuf, rx, spi_transmission.txlen))
	{
		CDFINGER_ERR("%s  copy_from_user err\n", __func__);
		ret = -1;
	}
	//CDFINGER_ERR("spi_transmission.rxbuf[0]=0x%x",spi_transmission.Prxbuf[0]);

free_rx:
	kfree(rx);
free_tx:
	kfree(tx);
	return ret;
}

static void cdfinger_wake_lock(struct cdfinger_data *cdfinger, int arg)
{
	if (arg)
	{
#ifdef CONFIG_PM_WAKELOCKS
		__pm_stay_awake(&cdfinger->cdfinger_lock);
#else
		wake_lock(&cdfinger->cdfinger_lock);
#endif
	}
	else
	{
#ifdef CONFIG_PM_WAKELOCKS
		__pm_relax(&cdfinger->cdfinger_lock);
		__pm_wakeup_event(&cdfinger->cdfinger_lock, jiffies_to_msecs(3*HZ));
#else
		wake_unlock(&cdfinger->cdfinger_lock);
		//wake_lock_timeout(&cdfinger->cdfinger_lock,3*HZ);
#endif
	}
}

static unsigned int cdfinger_changer_clk_frequency(struct cdfinger_data *cdfinger, unsigned int spi_frequence)
{
	if (cdfinger == NULL || spi_frequence <= 0 || cdfinger->last_spi_frequence == spi_frequence)
		return -1;

	CDFINGER_DBG("set spi frequence: %d\n", spi_frequence);
	cdfinger->spi_frequence = spi_frequence;

	cdfinger->spi->max_speed_hz = spi_frequence;
	if (spi_setup(cdfinger->spi) != 0)
	{
		CDFINGER_ERR("set spi frequence failed\n");
		return -1;
	}
	else
	{
		CDFINGER_DBG("set spi frequence success\n");
		cdfinger->last_spi_frequence = spi_frequence;
	}
	return 0;
}

static unsigned int cdfinger_poll(struct file *filp, struct poll_table_struct *wait)
{
	int mask = 0;
	poll_wait(filp, &cdfinger_waitqueue, wait);
	if (sign_sync == 1)
	{
		mask |= POLLIN | POLLPRI;
	}
	else if (sign_sync == 2)
	{
		mask |= POLLOUT;
	}
	sign_sync = 0;
	CDFINGER_DBG("mask %u\n", mask);
	return mask;
}

static long cdfinger_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct cdfinger_data *cdfinger = filp->private_data;
	int ret = 0;

	CDFINGER_FUNCTION("enter\n");
	if (cdfinger == NULL)
	{
		CDFINGER_ERR("%s: fingerprint please open device first!\n", __func__);
		return -EIO;
	}

	mutex_lock(&cdfinger->buf_lock);
	switch (cmd)
	{
	case CDFINGER_INIT_GPIO:
		CDFINGER_FUNCTION("cmd=CDFINGER_INIT_GPIO\n");
		ret = cdfinger_init_gpio(cdfinger);
		break;

	case CDFINGER_POWER_ON:
		CDFINGER_FUNCTION("cmd=CDFINGER_POWER_ON\n");
		ret = cdfinger_power_on(cdfinger);
		break;

	case CDFINGER_RESET:
		CDFINGER_FUNCTION("cmd=CDFINGER_RESET\n");
		cdfinger_reset(cdfinger, 1);
		break;

	case CDFINGER_HW_RESET:
		CDFINGER_FUNCTION("cmd=CDFINGER_HW_RESET\n");
		cdfinger_reset(cdfinger, arg);
		break;

	case CDFINGER_RELEASE_DEVICE:
		CDFINGER_FUNCTION("cmd=CDFINGER_RELEASE_DEVICE\n");
		cdfinger_free_gpio(cdfinger);
		misc_deregister(cdfinger->miscdev);
		isInit = 0;
		break;

	case CDFINGER_INIT_IRQ:
		CDFINGER_FUNCTION("cmd=CDFINGER_INIT_IRQ\n");
		ret = cdfinger_init_irq(cdfinger);
		break;

	case CDFINGER_SPI_WRITE_AND_READ:
		CDFINGER_FUNCTION("cmd=CDFINGER_SPI_WRITE_AND_READ\n");
		ret = cdfinger_spi_transmission(cdfinger, arg);
		break;

	case CDFINGER_NEW_KEYMODE:
		CDFINGER_FUNCTION("cmd=CDFINGER_NEW_KEYMODE\n");
		isInKeyMode = 0;
		break;

	case CDFINGER_INITERRUPT_MODE:
		CDFINGER_FUNCTION("cmd=CDFINGER_INITERRUPT_MODE\n");
		isInKeyMode = 1; // not key mode
		break;

	case CDFINGER_GET_STATUS:
		CDFINGER_FUNCTION("cmd=CDFINGER_GET_STATUS\n");
		ret = fb_status;
		break;

	case CDFINGER_WAKE_LOCK:
		CDFINGER_FUNCTION("cmd=CDFINGER_WAKE_LOCK\n");
		cdfinger_wake_lock(cdfinger, arg);
		break;

	case CDFINGER_REPORT_KEY:
		CDFINGER_FUNCTION("cmd=CDFINGER_REPORT_KEY\n");
		ret = cdfinger_report_key(cdfinger, arg);
		break;

    case CDFINGER_GETID:
        ret = cdfinger->sensor_type;
        break;
	case CDFINGER_CONTROL_IRQ:
		if (1 == arg)
			cdfinger_enable_irq(cdfinger);
		else
			cdfinger_disable_irq(cdfinger);
		break;

	case CDFINGER_CHANGER_CLK_FREQUENCY:
		CDFINGER_FUNCTION("cmd=CDFINGER_CHANGER_CLK_FREQUENCY\n");
		ret = cdfinger_changer_clk_frequency(cdfinger, arg);
		break;
	case CDFINGER_POLL_TRIGGER:
		sign_sync = 2;
		wake_up_interruptible(&cdfinger_waitqueue);
		ret = 0;
		break;
	default:
		ret = -ENOTTY;
		break;
	}
	mutex_unlock(&cdfinger->buf_lock);
	CDFINGER_FUNCTION("exit\n");

	return ret;
}

static int cdfinger_open(struct inode *inode, struct file *file)
{
	CDFINGER_FUNCTION("enter\n");
	file->private_data = g_cdfinger;
	CDFINGER_FUNCTION("exit\n");

	return 0;
}

static ssize_t cdfinger_write(struct file *file, const char *buff, size_t count, loff_t *ppos)
{
	return -ENOMEM;
}

static int cdfinger_async_fasync(int fd, struct file *filp, int mode)
{
	struct cdfinger_data *cdfinger = g_cdfinger;

	CDFINGER_FUNCTION("enter\n");
	return fasync_helper(fd, filp, mode, &cdfinger->async_queue);
}

static ssize_t cdfinger_read(struct file *file, char *buff, size_t count, loff_t *ppos)
{
	int ret = 0;

	return ret;
}

static int cdfinger_release(struct inode *inode, struct file *file)
{
	struct cdfinger_data *cdfinger = file->private_data;

	CDFINGER_FUNCTION("enter\n");
	if (cdfinger == NULL)
	{
		CDFINGER_ERR("%s: fingerprint please open device first!\n", __func__);
		return -EIO;
	}
	file->private_data = NULL;
	CDFINGER_FUNCTION("exit\n");

	return 0;
}

static const struct file_operations cdfinger_fops = {
	.owner = THIS_MODULE,
	.open = cdfinger_open,
	.write = cdfinger_write,
	.read = cdfinger_read,
	.release = cdfinger_release,
	.fasync = cdfinger_async_fasync,
	.unlocked_ioctl = cdfinger_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = cdfinger_ioctl,	
#endif
        .poll = cdfinger_poll,
};

static struct miscdevice cdfinger_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &cdfinger_fops,
};

static ssize_t cdfinger_debug_show(struct device *dev,
								   struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "cdfinger_debug = %x\n", cdfinger_debug);
}

static ssize_t cdfinger_version_show(struct device *dev,
									 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", VERSION);
}

static ssize_t cdfinger_debug_store(struct device *dev,
									struct device_attribute *attr, const char *buf, size_t size)
{
	int data;

	if (buf != NULL)
		sscanf(buf, "%x", &data);

	cdfinger_debug = (u8)data;

	return size;
}

static ssize_t cdfinger_spi_freq_show(struct device *dev,
									  struct device_attribute *attr, char *buf)
{
	struct cdfinger_data *cdfinger = g_cdfinger;
	if (cdfinger == NULL)
		return -1;
	return sprintf(buf, "current spi frequence[%d]\n", cdfinger->spi_frequence);
}

static ssize_t cdfinger_spi_freq_store(struct device *dev,
									   struct device_attribute *attr, const char *buf, size_t size)
{
	struct cdfinger_data *cdfinger = g_cdfinger;
	unsigned int spi_frequence = 0;

	if (buf != NULL)
		sscanf(buf, "%d", &spi_frequence);

	if (cdfinger == NULL || spi_frequence <= 0 || cdfinger->last_spi_frequence == spi_frequence)
		return size;

	CDFINGER_DBG("set spi frequence: %d\n", spi_frequence);
	cdfinger->spi_frequence = spi_frequence;
	cdfinger->spi->max_speed_hz = spi_frequence;
	if (spi_setup(cdfinger->spi) != 0)
	{
		CDFINGER_ERR("set spi frequence failed\n");
	}
	else
	{
		CDFINGER_DBG("set spi frequence success\n");
		cdfinger->last_spi_frequence = spi_frequence;
	}

	return size;
}

static ssize_t cdfinger_config_spi_store(struct device *dev,
										 struct device_attribute *attr, const char *buf, size_t size)
{
	struct cdfinger_data *cdfinger = g_cdfinger;

	CDFINGER_DBG("cdfinger->config_spi_pin = %d\n", cdfinger->config_spi_pin);
	if (cdfinger->config_spi_pin == 1)
	{
		pinctrl_select_state(cdfinger->fps_pinctrl, cdfinger->cdfinger_spi_miso);
		pinctrl_select_state(cdfinger->fps_pinctrl, cdfinger->cdfinger_spi_mosi);
		pinctrl_select_state(cdfinger->fps_pinctrl, cdfinger->cdfinger_spi_sck);
		pinctrl_select_state(cdfinger->fps_pinctrl, cdfinger->cdfinger_spi_cs);
	}

	return size;
}

static ssize_t cdfinger_reset_show(struct device *dev,
								   struct device_attribute *attr, char *buf)
{
#ifdef HAS_RESET_PIN
	cdfinger_reset(g_cdfinger, 1);
	return sprintf(buf, "reset success!\n");
#else
	return sprintf(buf, "no reset pin!\n");
#endif
}

static DEVICE_ATTR(debug, S_IRUGO | S_IWUSR, cdfinger_debug_show, cdfinger_debug_store);
static DEVICE_ATTR(version, S_IRUGO, cdfinger_version_show, NULL);
static DEVICE_ATTR(spi_freq, S_IRUGO | S_IWUSR, cdfinger_spi_freq_show, cdfinger_spi_freq_store);
static DEVICE_ATTR(config_spi, S_IWUSR, NULL, cdfinger_config_spi_store);
static DEVICE_ATTR(reset, S_IRUGO, cdfinger_reset_show, NULL);

static struct attribute *cdfinger_attrs[] =
	{
		&dev_attr_debug.attr,
		&dev_attr_version.attr,
		&dev_attr_spi_freq.attr,
		&dev_attr_config_spi.attr,
		&dev_attr_reset.attr,
		NULL};

static struct attribute_group cdfinger_attribute_group = {
	.name = "cdfinger",
	.attrs = cdfinger_attrs,
};

static int cdfinger_create_inputdev(struct cdfinger_data *cdfinger)
{
	cdfinger->cdfinger_inputdev = input_allocate_device();
	if (!cdfinger->cdfinger_inputdev)
	{
		CDFINGER_ERR("cdfinger->cdfinger_inputdev create faile!\n");
		return -ENOMEM;
	}
	__set_bit(EV_KEY, cdfinger->cdfinger_inputdev->evbit);
	__set_bit(KEY_F11, cdfinger->cdfinger_inputdev->keybit);
	__set_bit(KEY_F1, cdfinger->cdfinger_inputdev->keybit);
	__set_bit(KEY_F2, cdfinger->cdfinger_inputdev->keybit);
	__set_bit(KEY_F3, cdfinger->cdfinger_inputdev->keybit);
	__set_bit(KEY_F4, cdfinger->cdfinger_inputdev->keybit);
	__set_bit(KEY_VOLUMEUP, cdfinger->cdfinger_inputdev->keybit);
	__set_bit(KEY_VOLUMEDOWN, cdfinger->cdfinger_inputdev->keybit);
	__set_bit(KEY_PAGEUP, cdfinger->cdfinger_inputdev->keybit);
	__set_bit(KEY_PAGEDOWN, cdfinger->cdfinger_inputdev->keybit);
	__set_bit(KEY_UP, cdfinger->cdfinger_inputdev->keybit);
	__set_bit(KEY_LEFT, cdfinger->cdfinger_inputdev->keybit);
	__set_bit(KEY_RIGHT, cdfinger->cdfinger_inputdev->keybit);
	__set_bit(KEY_DOWN, cdfinger->cdfinger_inputdev->keybit);
	__set_bit(KEY_ENTER, cdfinger->cdfinger_inputdev->keybit);

	cdfinger->cdfinger_inputdev->id.bustype = BUS_HOST;
	cdfinger->cdfinger_inputdev->name = "cdfinger_inputdev";
	if (input_register_device(cdfinger->cdfinger_inputdev))
	{
		CDFINGER_ERR("register inputdev failed\n");
		input_free_device(cdfinger->cdfinger_inputdev);
		return -1;
	}

	if (sysfs_create_group(&cdfinger->cdfinger_inputdev->dev.kobj, &cdfinger_attribute_group) < 0)
	{
		CDFINGER_ERR("sysfs create group failed\n");
		input_unregister_device(cdfinger->cdfinger_inputdev);
		cdfinger->cdfinger_inputdev = NULL;
		input_free_device(cdfinger->cdfinger_inputdev);
		return -2;
	}

	return 0;
}

static int cdfinger_check_id(struct cdfinger_data *cdfinger)
{
    u8 id_rx[7]= {0};
    //u8 id_cmd[7] = {0x21,0x66,0x66,0xb5,0x00,0x43,0x44};
	u8 id_cmd[7] = {0};
    static u8 reset = 0x0c;
    static u8 start = 0x18;
    u8 read;

    spi_send_cmd_fifo(cdfinger, &start, &read, 1);
    spi_send_cmd_fifo(cdfinger, &reset, &read, 1);
    spi_send_cmd_fifo(cdfinger, &start, &read, 1);


    /*spi_send_cmd_fifo(cdfinger,id_cmd,id_rx,sizeof(id_cmd)/sizeof(id_cmd[0]));
    CDFINGER_DBG("reg[5] = 0x%x reg[6] = 0x%x\n",id_rx[5], id_rx[6]);
    if((id_rx[5]==0x70)&&(id_rx[6]==0x70))
    {
        cdfinger->sensor_type = FPS998;
        return 0;
    }*/

    memset(id_cmd,0x66,sizeof(id_cmd)/sizeof(id_cmd[0]));
	udelay(1000);
    id_cmd[0] = 0x74;
    spi_send_cmd_fifo(cdfinger,id_cmd,id_rx,sizeof(id_cmd)/sizeof(id_cmd[0]));
    CDFINGER_DBG("reg[3] = 0x%x ",id_rx[3]);
    switch(id_rx[3])
    {
    case FPS958E:
    case FPS988E:
    case FPS1256S:
    case FPS998E:
    case FPS998EAS:
    case FPS980S:
	case FPS1735:
        cdfinger->sensor_type = id_rx[3];
        return 0;
    default:
        return -1;
    }
}

static int cdfinger_fb_notifier_callback(struct notifier_block *self,
										 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	unsigned int blank;
	int retval = 0;

	if (event != FB_EVENT_BLANK /* FB_EARLY_EVENT_BLANK */)
	{
		return 0;
	}
	blank = *(int *)evdata->data;
	switch (blank)
	{
	case FB_BLANK_UNBLANK:
		//CDFINGER_DBG("sunlin==FB_BLANK_UNBLANK==\n");
		mutex_lock(&g_cdfinger->buf_lock);
		fb_status = 1;
		if (isInKeyMode == 0)
			cdfinger_async_report();
		mutex_unlock(&g_cdfinger->buf_lock);
		break;

	case FB_BLANK_POWERDOWN:
		//CDFINGER_DBG("sunlin==FB_BLANK_POWERDOWN==\n");
		mutex_lock(&g_cdfinger->buf_lock);
		fb_status = 0;
		if (isInKeyMode == 0)
			cdfinger_async_report();
		mutex_unlock(&g_cdfinger->buf_lock);
		break;

	default:
		break;
	}
	return retval;
}
static int cdfinger_probe(struct spi_device *spi)
{
	struct cdfinger_data *cdfinger = NULL;
	int status = -ENODEV;
	int checkid_count;
	int ret = 0;
	//CDFINGER_DBG("enter\n");
	printk("fingertech cdfinger cdfinger_probe enter\n");
	cdfinger = kzalloc(sizeof(struct cdfinger_data), GFP_KERNEL);
	if (!cdfinger)
	{
		CDFINGER_ERR("alloc cdfinger failed!\n");
		return -ENOMEM;
	}

	mutex_init(&cdfinger->transfer_lock);
	g_cdfinger = cdfinger;
	cdfinger->spi = spi;
	if (cdfinger_parse_dts(cdfinger))
	{
		CDFINGER_ERR("%s: parse dts failed!\n", __func__);
		goto free_cdfinger;
	}

	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;
	//spi->controller_data = (void *)&spi_conf;
	if (spi_setup(spi) != 0)
	{
		CDFINGER_ERR("%s: spi setup failed!\n", __func__);
		goto free_cdfinger;
	}
#if defined(CONFIG_PRIZE_FP_USE_VFP)
	cdfinger->vdd_reg = regulator_get(&cdfinger->spi->dev, "VFP");
    if (IS_ERR(cdfinger->vdd_reg)) {
        ret = PTR_ERR(cdfinger->vdd_reg);
        CDFINGER_ERR("%s: Regulator get failed vdd ret = %d\n",__func__,ret);
        //return ret;
		goto free_cdfinger;
    }
	ret = regulator_set_voltage(cdfinger->vdd_reg, 2800000, 2800000);
	if (ret) {
		CDFINGER_ERR("%s: Regulator set vdd val fail ret = %d\n",__func__,ret);
		//return ret;
		goto free_regulator;
	}
#endif
	ret = cdfinger_power_on(cdfinger);
	if (ret){
		CDFINGER_ERR("%s: power on fail ret = %d\n",__func__,ret);
		//goto free_regulator;
	};
    for(checkid_count=0; checkid_count<5; checkid_count++)
    {
        status = cdfinger_check_id(cdfinger);
        if ((status != 0)&&(checkid_count == 4))
        {
            CDFINGER_ERR("cdfinger: check id failed! status=%d\n",status);
            goto power_off;
        }
        else if (status == 0)
        {
            break;
        }
        cdfinger_reset(cdfinger,10);
    }
	mutex_init(&cdfinger->buf_lock);
#ifdef CONFIG_PM_WAKELOCKS
	wakeup_source_init(&cdfinger->cdfinger_lock, "cdfinger wakelock");
#else
	wake_lock_init(&cdfinger->cdfinger_lock, WAKE_LOCK_SUSPEND, "cdfinger wakelock");
#endif

	status = misc_register(&cdfinger_dev);
	if (status < 0)
	{
		CDFINGER_ERR("%s: cdev register failed!\n", __func__);
		goto free_wakelock;
	}
	cdfinger->miscdev = &cdfinger_dev;
	if (cdfinger_create_inputdev(cdfinger) < 0)
	{
		CDFINGER_ERR("%s: inputdev register failed!\n", __func__);
		goto free_miscDevice;
	}

	spi_set_drvdata(spi, cdfinger);

	cdfinger->notifier.notifier_call = cdfinger_fb_notifier_callback;
	fb_register_client(&cdfinger->notifier);
    
/* prize added by chenjiaxi, cdfinger ree info, 20200111-start */
#if defined(CONFIG_PRIZE_HARDWARE_INFO)
	if (cdfinger->sensor_type == 0x98)
		sprintf(current_fingerprint_info.chip,"FPS998E");
	else if (cdfinger->sensor_type == 0x70)
		sprintf(current_fingerprint_info.chip,"FPS998");
	else if (cdfinger->sensor_type == 0x80)
		sprintf(current_fingerprint_info.chip,"FPS988E/FPS1356E");
	else if (cdfinger->sensor_type == 0x56)
		sprintf(current_fingerprint_info.chip,"FPS958E/FPS1256E");
	else if (cdfinger->sensor_type == 0x88)
		sprintf(current_fingerprint_info.chip,"FPS980S");
	else if (cdfinger->sensor_type == 0x54)
		sprintf(current_fingerprint_info.chip,"FPS1256S");
	else if (cdfinger->sensor_type == 0x96)
		sprintf(current_fingerprint_info.chip,"FPS998EAS");
	else if (cdfinger->sensor_type == 0x35)
		sprintf(current_fingerprint_info.chip,"FPS1735");
	else
		sprintf(current_fingerprint_info.chip,"UNKNOW");
	sprintf(current_fingerprint_info.id,"0x%x",cdfinger->sensor_type);
	strcpy(current_fingerprint_info.vendor,"cdfinger");
	strcpy(current_fingerprint_info.more,"fingerprint");
#endif
/* prize added by chenjiaxi, cdfinger ree info, 20200111-end */

	CDFINGER_DBG("exit\n");
	return 0;

free_miscDevice:
	misc_deregister(cdfinger->miscdev);
free_wakelock:
#ifdef CONFIG_PM_WAKELOCKS
	wakeup_source_trash(&cdfinger->cdfinger_lock);
#else
	wake_lock_destroy(&cdfinger->cdfinger_lock);
#endif
	mutex_destroy(&cdfinger->buf_lock);
power_off:
	cdfinger_power_off(cdfinger);
#if defined(CONFIG_PRIZE_FP_USE_VFP)
free_regulator:
	regulator_put(cdfinger->vdd_reg);
#endif
free_cdfinger:
	mutex_destroy(&cdfinger->transfer_lock);
	kfree(cdfinger);
	cdfinger = NULL;

	return -1;
}

static int cdfinger_suspend(struct device *dev)
{
	return 0;
}

static int cdfinger_resume(struct device *dev)
{
	return 0;
}

static int cdfinger_remove(struct spi_device *spi)
{
	struct cdfinger_data *cdfinger = spi_get_drvdata(spi);

	free_irq(cdfinger->irq, cdfinger);
	sysfs_remove_group(&cdfinger->cdfinger_inputdev->dev.kobj, &cdfinger_attribute_group);
	input_unregister_device(cdfinger->cdfinger_inputdev);
	cdfinger->cdfinger_inputdev = NULL;
	input_free_device(cdfinger->cdfinger_inputdev);
	misc_deregister(cdfinger->miscdev);
#ifdef CONFIG_PM_WAKELOCKS
	wakeup_source_trash(&cdfinger->cdfinger_lock);
#else
	wake_lock_destroy(&cdfinger->cdfinger_lock);
#endif
	mutex_destroy(&cdfinger->buf_lock);
	mutex_destroy(&cdfinger->transfer_lock);
	kfree(cdfinger);
	cdfinger = NULL;
	g_cdfinger = NULL;
	cdfinger_power_off(cdfinger);

	return 0;
}

static const struct dev_pm_ops cdfinger_pm = {
	.suspend = cdfinger_suspend,
	.resume = cdfinger_resume};

struct of_device_id cdfinger_of_match[] = {
	{ .compatible = "cdfinger,fps998e", },
	{ .compatible = "cdfinger,fps1098", },
	{ .compatible = "cdfinger,fps998", },
	{ .compatible = "mediatek,cdfinger-finger", },
	{ .compatible = "mediatek,finger-spi", },
	{ .compatible = "prize,fingerprint", },

	{},
};
MODULE_DEVICE_TABLE(of, cdfinger_of_match);

static const struct spi_device_id cdfinger_id[] = {
	{SPI_DRV_NAME, 0},
	{}};
MODULE_DEVICE_TABLE(spi, cdfinger_id);

static struct spi_driver cdfinger_driver = {
	.driver = {
		.name = SPI_DRV_NAME,
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
		.pm = &cdfinger_pm,
		.of_match_table = of_match_ptr(cdfinger_of_match),
	},
	.id_table = cdfinger_id,
	.probe = cdfinger_probe,
	.remove = cdfinger_remove,
};

#ifndef DTS_PROBE
static struct spi_board_info spi_board_cdfinger[] __initdata = {
	[0] = {
		.modalias = "cdfinger",
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_0,
		.max_speed_hz = 6000000,
	},
};
#endif

static int __init cdfinger_spi_init(void)
{
#ifndef DTS_PROBE
	spi_register_board_info(spi_board_cdfinger, ARRAY_SIZE(spi_board_cdfinger));
#endif
	printk("fingertech cdfinger cdfinger_spi_init enter");
	return spi_register_driver(&cdfinger_driver);
}

static void __exit cdfinger_spi_exit(void)
{
	spi_unregister_driver(&cdfinger_driver);
}

late_initcall_sync(cdfinger_spi_init);
module_exit(cdfinger_spi_exit);

MODULE_DESCRIPTION("cdfinger spi Driver");
MODULE_AUTHOR("shuaitao@cdfinger.com");
MODULE_LICENSE("GPL");
MODULE_ALIAS("cdfinger");
