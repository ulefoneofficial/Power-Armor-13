/*
 * Copyright (C) 2010 Trusted Logic S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/dma-mapping.h>
#include <linux/proc_fs.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include "nxp533.h"

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif

/*  for clock buffer */
#include "../../misc/mediatek/base/power/include/mtk_clkbuf_ctl.h"
/* begin, prize-lifenfen-20181126, add for nfc hardware info */
#if defined(CONFIG_PRIZE_HARDWARE_INFO)
#include "../../misc/mediatek/hardware_info/hardware_info.h"
extern struct hardware_info current_nfc_info;

#endif
/* end, prize-lifenfen-20181126, add for nfc hardware info */

#define PN544_DRVNAME		"pn544"

#define MAX_BUFFER_SIZE		512
#define I2C_ID_NAME		"pn547"

struct pn547_dev
{
	wait_queue_head_t	read_wq;
	struct mutex		read_mutex;
	struct i2c_client	*client;
	union  nqx_uinfo	nqx_info;
	struct miscdevice	pn547_device;
	unsigned int irq_gpio;
	bool			irq_enabled;
	spinlock_t		irq_enabled_lock;
	struct regulator	*reg;
};

//static  struct pn547_i2c_platform_data pn547_platform_data;

static  struct pinctrl *gpctrl = NULL;
static  struct pinctrl_state *st_ven_h = NULL;
static  struct pinctrl_state *st_ven_l = NULL;
static  struct pinctrl_state *st_dwn_h = NULL;
static  struct pinctrl_state *st_dwn_l = NULL;
static  struct pinctrl_state *st_eint_int = NULL;

/*****************************************************************************
 * Function
 *****************************************************************************/

static void pn547_enable_irq(struct pn547_dev *pn547_dev)
{
	unsigned long flags;
	//printk("%s\n", __func__);
	spin_lock_irqsave(&pn547_dev->irq_enabled_lock, flags);
	if (!pn547_dev->irq_enabled)
	{
		pn547_dev->irq_enabled = true;
		enable_irq(pn547_dev->client->irq);
	}
	spin_unlock_irqrestore(&pn547_dev->irq_enabled_lock, flags);
}

static void pn547_disable_irq(struct pn547_dev *pn547_dev)
{
	unsigned long flags;
	//printk("%s\n", __func__);
	spin_lock_irqsave(&pn547_dev->irq_enabled_lock, flags);
	if (pn547_dev->irq_enabled)
	{
		disable_irq_nosync(pn547_dev->client->irq);
		pn547_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&pn547_dev->irq_enabled_lock, flags);
}

static irqreturn_t pn547_dev_irq_handler(int irq, void *dev)
{
	struct pn547_dev *pn547_dev = dev;
	//printk("pn547_dev_irq_handler()\n");
	pn547_disable_irq(pn547_dev);
	/* Wake up waiting readers */
	wake_up(&pn547_dev->read_wq);
	return IRQ_HANDLED;
}

static int pn547_platform_pinctrl_select(struct pinctrl *p, struct pinctrl_state *s)
{
	int ret = 0;
	//printk("%s\n", __func__);
	if (p != NULL && s != NULL) {
		ret = pinctrl_select_state(p, s);
	} else {
		printk("%s: pinctrl_select err\n", __func__);
		ret = -1;
	}
	return ret;
}

static ssize_t pn547_dev_read(struct file *filp, char __user *buf,
			       size_t count, loff_t *offset)
{
	struct pn547_dev *pn547_dev = filp->private_data;
	unsigned char *I2CDMAReadBuf = NULL;
	int ret=0;
	//printk("%s\n", __func__);

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	//printk("%s : reading %zu bytes.\n", __func__, count);

	I2CDMAReadBuf = kmalloc(MAX_BUFFER_SIZE, GFP_KERNEL);
	if (!I2CDMAReadBuf) {
		printk("%s: failed to allocate memory for pn547_dev->read\n", __func__);
		ret = -ENOMEM;
		return ret;
	}

	mutex_lock(&pn547_dev->read_mutex);
	//printk(" %s : mutex_lock.\n", __func__);
	if (!gpio_get_value(pn547_dev->irq_gpio)) {
		//printk("pn547 read no event\n");
		if (filp->f_flags & O_NONBLOCK)
		{
			ret = -EAGAIN;
			goto fail;
		}

		//printk("pn547 read wait event\n");
		pn547_enable_irq(pn547_dev);

		//printk("pn547 read1  pn547_dev->irq_gpio=%d\n", pn547_dev->irq_gpio);
		//printk("pn547 read2  gpio_get_value(pn547_dev->irq_gpio)=%d\n", gpio_get_value(pn547_dev->irq_gpio));
		ret = wait_event_interruptible(pn547_dev->read_wq, gpio_get_value(pn547_dev->irq_gpio));
		//printk("pn547 read2  gpio_get_value(pn547_dev->irq_gpio)=%d\n", gpio_get_value(pn547_dev->irq_gpio));

		pn547_disable_irq(pn547_dev);

		if (ret)
		{
			printk("pn547 read wait event error\n");
			goto fail;
		}
	}
	//printk("%s : i2c_master_recv start.\n", __func__);

	ret = i2c_master_recv(pn547_dev->client, I2CDMAReadBuf, count);
	mutex_unlock(&pn547_dev->read_mutex);

	if (ret < 0){
		printk("pn547 %s: i2c_master_recv returned %d\n", __func__, ret);
		goto err;
	}else if( ret > count){
	       printk("pn547 %s: received too many bytes from i2c (%d)\n", __func__, ret);
		ret = -EIO;
		goto err;
	}

	if (copy_to_user(buf, I2CDMAReadBuf, ret)) {
		printk("%s : failed to copy to user space\n", __func__);
		ret = -EFAULT;
		goto err;
	}

	kfree(I2CDMAReadBuf);
	return ret;

fail:
	mutex_unlock(&pn547_dev->read_mutex);
err:
	kfree(I2CDMAReadBuf);
	return ret;
}


static ssize_t pn547_dev_write(struct file *filp, const char __user *buf,
				size_t count, loff_t *offset)
{
	struct pn547_dev *pn547_dev;
	int ret=0; //baker_mod_w=0;
	unsigned char *I2CDMAWriteBuf = NULL;

	//printk("%s\n", __func__);

	pn547_dev = filp->private_data;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	I2CDMAWriteBuf = kmalloc(MAX_BUFFER_SIZE, GFP_KERNEL);
	if (!I2CDMAWriteBuf) {
		printk("%s: failed to allocate memory for pn547_dev->write\n", __func__);
		ret = -ENOMEM;
		return ret;
	}

	if (copy_from_user(I2CDMAWriteBuf, buf, count))
	{
		printk("%s : failed to copy from user space\n", __func__);
		ret = -EFAULT;
		goto err;
	}

	msleep(15);

	//printk("pn547 %s : writing %zu bytes.\n", __func__, count);
	ret = i2c_master_send(pn547_dev->client, I2CDMAWriteBuf, count);

	if (ret != count)
	{
		printk("pn547 %s : i2c_master_send returned %d\n", __func__, ret);
		ret = -EIO;
		goto err;
	}

err:
	kfree(I2CDMAWriteBuf);
	return ret;
}

static int pn547_dev_open(struct inode *inode, struct file *filp)
{
	int ret = 0;
	struct pn547_dev *pn547_dev = container_of(filp->private_data, struct pn547_dev, pn547_device);

	//printk("%s:pn547_dev=%p\n", __func__, pn547_dev);

	filp->private_data = pn547_dev;

	//printk("pn547 %s : %d,%d\n", __func__, imajor(inode), iminor(inode));

	return ret;
}

static long pn547_dev_unlocked_ioctl(struct file *filp, unsigned int cmd,
				      unsigned long arg)
{
	int ret;
	struct pn547_dev *pn547_dev = filp->private_data;

	printk("%s:cmd=%d, arg=%ld, pn547_dev=%p\n", __func__, cmd, arg, pn547_dev);

	switch (cmd)
	{
		case PN547_SET_PWR:
			if (arg == 2) {
				//clk_buf_ctrl(CLK_BUF_NFC, 1);
				/* power on with firmware download (requires hw reset) */
				printk("pn547 %s power on with firmware\n", __func__);
				ret = pn547_platform_pinctrl_select(gpctrl, st_ven_h);
				ret = pn547_platform_pinctrl_select(gpctrl, st_dwn_h);
				msleep(10);
				ret = pn547_platform_pinctrl_select(gpctrl, st_ven_l);
				msleep(50);
				ret = pn547_platform_pinctrl_select(gpctrl, st_ven_h);
				msleep(10);
			} else if (arg == 1) {
				//clk_buf_ctrl(CLK_BUF_NFC, 1);
				/* power on */
				printk("pn547 %s power on\n", __func__);
				ret = pn547_platform_pinctrl_select(gpctrl, st_dwn_l);
				ret = pn547_platform_pinctrl_select(gpctrl, st_ven_h);
				msleep(10);
			} else  if (arg == 0) {
				//clk_buf_ctrl(CLK_BUF_NFC, 0);
				/* power off */
				printk("pn547 %s power off\n", __func__);
				ret = pn547_platform_pinctrl_select(gpctrl, st_dwn_l);
				ret = pn547_platform_pinctrl_select(gpctrl, st_ven_l);
				msleep(50);
			} else {
				printk("pn547 %s bad arg %lu\n", __func__, arg);
				return -EINVAL;
			}
			break;
		default:
			printk("pn547 %s bad ioctl %u\n", __func__, cmd);
			return -EINVAL;
	}

	return ret;
}


static const struct file_operations pn547_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = pn547_dev_read,
	.write = pn547_dev_write,
	.open = pn547_dev_open,
#ifdef CONFIG_COMPAT
	.compat_ioctl = pn547_dev_unlocked_ioctl,
#endif
	.unlocked_ioctl = pn547_dev_unlocked_ioctl,
};

static int pn547_remove(struct i2c_client *client)
{
	struct pn547_dev *pn547_dev;

	pn547_dev = i2c_get_clientdata(client);
	free_irq(client->irq, pn547_dev);
	misc_deregister(&pn547_dev->pn547_device);
	mutex_destroy(&pn547_dev->read_mutex);
	kfree(pn547_dev);
	return 0;
}

/* Check for availability of NQ_ NFC controller hardware */
static int nfcc_hw_check(struct i2c_client *client, struct pn547_dev *pn547_dev)
{
	int ret = 0;

	unsigned char raw_nci_reset_cmd[] =  {0x20, 0x00, 0x01, 0x00};
	unsigned char raw_nci_init_cmd[] =   {0x20, 0x01, 0x00};
	unsigned char raw_nci_getversion_cmd[] = {0x00, 0x04, 0xF1,
					0x00, 0x00, 0x00, 0x6E, 0xEF};

	unsigned char nci_init_rsp[28];
	unsigned char nci_reset_rsp[6];
	unsigned char nci_getversion_rsp[20];
	unsigned char init_rsp_len = 0;

	ret = pn547_platform_pinctrl_select(gpctrl, st_ven_h);/* HPD : Enable*/
	/* hardware dependent delay */
	usleep_range(10000, 10100);
	ret = pn547_platform_pinctrl_select(gpctrl, st_ven_l);/* ULPM: Disable */
	/* hardware dependent delay */
	usleep_range(10000, 10100);
	ret = pn547_platform_pinctrl_select(gpctrl, st_ven_h);/* HPD : Enable*/
	/* hardware dependent delay */
	usleep_range(10000, 10100);

	/* send NCI CORE RESET CMD with Keep Config parameters */
	ret = i2c_master_send(client, raw_nci_reset_cmd,
						sizeof(raw_nci_reset_cmd));

	if (ret < 0) {
		dev_err(&client->dev,
		"%s: - i2c_master_send core reset Error\n", __func__);

		ret = pn547_platform_pinctrl_select(gpctrl, st_ven_h);/* HPD : Enable*/
		usleep_range(10000, 10100);
		ret = pn547_platform_pinctrl_select(gpctrl, st_dwn_h);
		usleep_range(10000, 10100);
		ret = pn547_platform_pinctrl_select(gpctrl, st_ven_l);
		usleep_range(10000, 10100);
		ret = pn547_platform_pinctrl_select(gpctrl, st_ven_h);
		usleep_range(10000, 10100);

		ret = i2c_master_send(client, raw_nci_getversion_cmd,
						sizeof(raw_nci_getversion_cmd));

		if (ret < 0) {
			dev_err(&client->dev,
			"%s: - i2c_master_send get version Error\n", __func__);
			ret = pn547_platform_pinctrl_select(gpctrl, st_dwn_l);
			goto err_nfcc_hw_check;
		}
		/* hardware dependent delay */
		msleep(30);

		ret = i2c_master_recv(client, nci_getversion_rsp,
						sizeof(nci_getversion_rsp));
		if (ret < 0) {
			dev_err(&client->dev,
			"%s: - i2c_master_recv get version Error\n", __func__);
			ret = pn547_platform_pinctrl_select(gpctrl, st_dwn_l);
			goto err_nfcc_hw_check;
		} else {
			pn547_dev->nqx_info.info.chip_type =
				nci_getversion_rsp[3];
			pn547_dev->nqx_info.info.rom_version =
				nci_getversion_rsp[4];
			pn547_dev->nqx_info.info.fw_minor =
				nci_getversion_rsp[10];
			pn547_dev->nqx_info.info.fw_major =
				nci_getversion_rsp[11];
		}
		ret = pn547_platform_pinctrl_select(gpctrl, st_dwn_l);
		goto err_nfcc_reset_failed;
	}
	/* hardware dependent delay */
	msleep(30);

	/* Read Response of RESET command */
	ret = i2c_master_recv(client, nci_reset_rsp,
		sizeof(nci_reset_rsp));
	if (ret < 0) {
		dev_err(&client->dev,
		"%s: - i2c_master_recv core reset Error\n", __func__);
		goto err_nfcc_hw_check;
	}
	dev_info(&client->dev,
	"%s: - nq - reset cmd answer : NfcNciRx %02x %02x %02x %02x %02x %02x\n",
	__func__, nci_reset_rsp[0], nci_reset_rsp[1],
	nci_reset_rsp[2], nci_reset_rsp[3],
	nci_reset_rsp[4], nci_reset_rsp[5]);
	ret = i2c_master_send(client, raw_nci_init_cmd,
		sizeof(raw_nci_init_cmd));
	if (ret < 0) {
		dev_err(&client->dev,
		"%s: - i2c_master_send core init Error\n", __func__);
		goto err_nfcc_hw_check;
	}
	/* hardware dependent delay */
	msleep(30);
	/* Read Response of INIT command */
	ret = i2c_master_recv(client, nci_init_rsp,
		sizeof(nci_init_rsp));
	if (ret < 0) {
		dev_err(&client->dev,
		"%s: - i2c_master_recv core init Error\n", __func__);
		goto err_nfcc_hw_check;
	}
	init_rsp_len = 2 + nci_init_rsp[2]; /*payload + len*/
	if (init_rsp_len > PAYLOAD_HEADER_LENGTH) {
		pn547_dev->nqx_info.info.chip_type =
				nci_init_rsp[init_rsp_len - 3];
		pn547_dev->nqx_info.info.rom_version =
				nci_init_rsp[init_rsp_len - 2];
		pn547_dev->nqx_info.info.fw_major =
				nci_init_rsp[init_rsp_len - 1];
		pn547_dev->nqx_info.info.fw_minor =
				nci_init_rsp[init_rsp_len];
	}
err_nfcc_reset_failed:
	dev_info(&pn547_dev->client->dev, "NQ NFCC chip_type = 0x%x\n",
		pn547_dev->nqx_info.info.chip_type);
	dev_info(&pn547_dev->client->dev, "NQ fw version = %x.%x.%x\n",
		pn547_dev->nqx_info.info.rom_version,
		pn547_dev->nqx_info.info.fw_major,
		pn547_dev->nqx_info.info.fw_minor);

	switch (pn547_dev->nqx_info.info.chip_type) {
	case NFCC_NQ_210:
		dev_info(&client->dev,
		"%s: ## NFCC == NQ210 ##\n", __func__);
		break;
	case NFCC_NQ_220:
		dev_info(&client->dev,
		"%s: ## NFCC == NQ220 ##\n", __func__);
		break;
	case NFCC_NQ_310:
		dev_info(&client->dev,
		"%s: ## NFCC == NQ310 ##\n", __func__);
		break;
	case NFCC_NQ_330:
		dev_info(&client->dev,
		"%s: ## NFCC == NQ330 ##\n", __func__);
		break;
	case NFCC_PN66T:
		dev_info(&client->dev,
		"%s: ## NFCC == PN66T ##\n", __func__);
		break;
	case NFCC_PN553:
		dev_info(&client->dev,
		"%s: ## NFCC == PN553 ##\n", __func__);
		break;
	default:
		dev_err(&client->dev,
		"%s: - NFCC ChipID can't be identified\n", __func__);
		break;
	}

	/*Disable NFC by default to save power on boot*/
	ret = pn547_platform_pinctrl_select(gpctrl, st_ven_l);/* ULPM: Disable */
	ret = 0;
	goto done;

err_nfcc_hw_check:
	ret = -ENXIO;
	dev_err(&client->dev,
		"%s: - NFCC HW not available\n", __func__);
done:
	return ret;
}

static int pn547_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{

	int ret=0;
	struct pn547_dev *pn547_dev;
	struct device_node *node;

	printk("pn547_probe\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		printk("%s: need I2C_FUNC_I2C\n", __func__);
		ret = -ENODEV;
	}

	pn547_dev = kzalloc(sizeof(*pn547_dev), GFP_KERNEL);
	//printk("pn547_dev=%p\n", pn547_dev);

	if (pn547_dev == NULL)
	{
		printk("%s: pn547 failed to allocate memory for module data\n", __func__);
		ret = -ENOMEM;
	}

	memset(pn547_dev, 0, sizeof(struct pn547_dev));
	pn547_dev->client = client;

	/* init mutex and queues */
	init_waitqueue_head(&pn547_dev->read_wq);
	mutex_init(&pn547_dev->read_mutex);
	spin_lock_init(&pn547_dev->irq_enabled_lock);
#if 1
	pn547_dev->pn547_device.minor = MISC_DYNAMIC_MINOR;
	pn547_dev->pn547_device.name = PN544_DRVNAME;
	pn547_dev->pn547_device.fops = &pn547_dev_fops;

	ret = misc_register(&pn547_dev->pn547_device);
	if (ret)
	{
		printk("%s: misc_register failed\n", __func__);
		goto err_misc_register;
	}
#endif

	/* request irq.  the irq is set whenever the chip has data available
	* for reading.  it is cleared when all data has been read.
	*/

	/*  NFC IRQ settings     */
	node = of_find_compatible_node(NULL, NULL, "mediatek,nfc-gpio-v2");

	if (node) {
#if (!defined(CONFIG_MTK_GPIO) || defined(CONFIG_MTK_GPIOLIB_STAND))
		ret = of_get_named_gpio(node, "gpio-irq-std", 0);
		if (ret < 0)
		    printk("%s : get NFC IRQ GPIO failed (%d).\n", __func__, ret);
		else
		    pn547_dev->irq_gpio = ret;
#else
		of_property_read_u32_array(node, "gpio-irq",
					   &(pn547_dev->irq_gpio), 1);
#endif
		printk("pn547_dev->irq_gpio = %d\n", pn547_dev->irq_gpio);
	} else {
		printk("%s : get gpio num err.\n", __func__);
		goto err_irq_request;
	}

	node = of_find_compatible_node(NULL, NULL, "mediatek,irq_nfc-eint");
	if (node) {
		client->irq = irq_of_parse_and_map(node, 0);
		printk("pn547 client->irq = %d\n", client->irq);

		ret =
		    request_irq(client->irq, pn547_dev_irq_handler,
				IRQF_TRIGGER_NONE, "nfc_eint_as_int", pn547_dev);

		if (ret) {
			printk("%s: EINT IRQ LINE NOT AVAILABLE, ret = %d\n", __func__, ret);
			goto err_irq_request;
		} else {

			printk("%s: set EINT finished, client->irq=%d\n", __func__,
				 client->irq);

			pn547_dev->irq_enabled = true;
			pn547_disable_irq(pn547_dev);
		}

	} else {
		printk("%s: can not find NFC eint compatible node\n",
		       __func__);
		goto err_irq_request;
	}

	ret= nfcc_hw_check(client, pn547_dev);
	if (ret) {
		/* make sure NFCC is not enabled */
		ret = pn547_platform_pinctrl_select(gpctrl, st_ven_l);
		/* We don't think there is hardware switch NFC OFF */
		#ifdef CONFIG_REGIGISTER_DYNAMIC
		goto err_request_hw_check_failed;
		#endif /* CONFIG_REGIGISTER_DYNAMIC */
	}

	i2c_set_clientdata(client, pn547_dev);
		/*prize add by wangyongsheng 20210331--------start*/	
#if defined(CONFIG_PRIZE_HARDWARE_INFO)
		sprintf(current_nfc_info.chip,"NQ310 :fw_version:0x%02x",pn547_dev->nqx_info.info.rom_version);
		sprintf(current_nfc_info.id,"0x%04x",pn547_dev->nqx_info.info.fw_major);
		strcpy(current_nfc_info.vendor,"NXP");
		sprintf(current_nfc_info.more,"nfc");

#endif
	//prize-wangyongsheng-20210331-end	

       printk("%s, probing pn547 driver exited successfully\n", __func__);
	return 0;

#ifdef CONFIG_REGIGISTER_DYNAMIC
err_request_hw_check_failed:
	free_irq(client->irq, pn547_dev);
#endif /* CONFIG_REGIGISTER_DYNAMIC */
err_irq_request:
	misc_deregister(&pn547_dev->pn547_device);
err_misc_register:
	mutex_destroy(&pn547_dev->read_mutex);
	kfree(pn547_dev);
	return ret;
}

static const struct i2c_device_id pn547_id[] = {
	{I2C_ID_NAME, 0},
	{}
};

static const struct of_device_id nfc_i2c_of_match[] = {
	{.compatible = "mediatek,nfc"},
	{},
};

static struct i2c_driver pn547_driver =
{
	.id_table	= pn547_id,
	.probe		= pn547_probe,
	.remove		= pn547_remove,
	.driver		= {
		.name = "pn547",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = nfc_i2c_of_match,
#endif
	},
};


static int pn547_platform_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;

	printk("%s\n", __func__);

	gpctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(gpctrl)) {
		printk( "%s: Cannot find pinctrl!", __func__);
		ret = PTR_ERR(gpctrl);
		goto end;
	}

	st_ven_h = pinctrl_lookup_state(gpctrl, "ven_high");
	if (IS_ERR(st_ven_h)) {
		ret = PTR_ERR(st_ven_h);
		printk("%s: pinctrl err, st_ven_h\n", __func__);
		goto end;
	}

	st_ven_l = pinctrl_lookup_state(gpctrl, "ven_low");
	if (IS_ERR(st_ven_l)) {
		ret = PTR_ERR(st_ven_l);
		printk("%s: pinctrl err, st_ven_l\n", __func__);
		goto end;
	}
	//modify by tzf@meitu.begin
	st_dwn_h = pinctrl_lookup_state(gpctrl, "dwn_high");
	if (IS_ERR(st_dwn_h)) {
		ret = PTR_ERR(st_dwn_h);
		printk("%s: pinctrl err, st_dwn_h\n", __func__);
		goto end;
	}

	st_dwn_l = pinctrl_lookup_state(gpctrl, "dwn_low");
	if (IS_ERR(st_dwn_l)) {
		ret = PTR_ERR(st_dwn_l);
		printk("%s: pinctrl err, st_dwn_l\n", __func__);
		goto end;
	}

	st_eint_int = pinctrl_lookup_state(gpctrl, "eint_as_int");
	if (IS_ERR(st_eint_int)) {
		ret = PTR_ERR(st_eint_int);
		printk("%s: pinctrl err, st_eint_int\n", __func__);
		goto end;
	}

	/* select state */
	ret = pn547_platform_pinctrl_select(gpctrl, st_ven_l);
	usleep_range(900, 1000);

	ret = pn547_platform_pinctrl_select(gpctrl, st_dwn_l);
	usleep_range(900, 1000);

      ret = pn547_platform_pinctrl_select(gpctrl, st_eint_int);
	usleep_range(900, 1000);

end:
	return ret;
}

static int pn547_platform_probe(struct platform_device *pdev)
{
	int ret = 0;

	//printk("%s: &pdev=%p\n", __func__, pdev);

	/* pinctrl init */
	ret = pn547_platform_pinctrl_init(pdev);

	return ret;
}

static int pn547_platform_remove(struct platform_device *pdev)
{
	printk("%s: &pdev=%p\n", __func__, pdev);

	return 0;
}

/*  platform driver */
static const struct of_device_id pn547_platform_of_match[] = {
	{.compatible = "mediatek,nfc-gpio-v2",},
	{},
};

static struct platform_driver pn547_platform_driver = {
	.probe		= pn547_platform_probe,
	.remove		= pn547_platform_remove,
	.driver		= {
		.name = PN544_DRVNAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = pn547_platform_of_match,
#endif
	},
};

/*
 * module load/unload record keeping
 */
static int __init pn547_dev_init(void)
{
	int ret;
	printk("pn547_dev_init\n");

	platform_driver_register(&pn547_platform_driver);
	printk("platform_driver_register success\n");
	ret = i2c_add_driver(&pn547_driver);
	printk("pn547_dev_init success\n");

	return 0;
}
module_init(pn547_dev_init);

static void __exit pn547_dev_exit(void)
{
	printk("Unloading pn547 driver\n");

	i2c_del_driver(&pn547_driver);
}
module_exit(pn547_dev_exit);

MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("NFC PN547 driver");
MODULE_LICENSE("GPL");

