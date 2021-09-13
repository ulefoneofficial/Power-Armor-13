#include <linux/module.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_DRM)
#if defined(CONFIG_DRM_PANEL)
#include <drm/drm_panel.h>
#else
#include <linux/msm_drm_notify.h>
#endif
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#define FTS_SUSPEND_LEVEL 1     /* Early-suspend level */
#endif
#include "focaltech_core.h"
	
	#define ENTER_WORK_FACTORY_RETRIES              5
	#define FACTORY_TEST_DELAY                      18
	#define FACTORY_TEST_RETRY_DELAY                100

	#define FACTORY_REG_CHX_NUM                     0x02
	#define FACTORY_REG_CHY_NUM                     0x03
	#define DEVICE_MODE_ADDR                        0x00
	#define FACTORY_REG_LINE_ADDR                   0x01
	#define FACTORY_REG_RAWDATA_ADDR                0x6A
	#define FACTORY_REG_DATA_SELECT                 0x06
	
	/****************************************************
	分包读包长度宏，默认128，最大256，如果客户系统读受限制，可以修改，建议4的倍数
	***************************************************/
	#define packet                                  128

	int fts_raw_enter_factory_mode(void)
	{
		int ret = 0;
		u8 mode = 0;
		int i = 0;
		int j = 0;
		u8 addr = DEVICE_MODE_ADDR;
		

		ret = fts_read(&addr,1, &mode,1);
		if ((ret >= 0) && (0x40 == mode))
			return 0;

		for (i = 0; i < 5; i++) {
			ret = fts_write_reg(addr,0x40);
			if (ret >= 0) {
				msleep(FACTORY_TEST_DELAY);
				for (j = 0; j < 20; j++) {
					ret = fts_read(&addr,1, &mode,1);
					if ((ret >= 0) && (0x40 == mode)) {
						FTS_INFO("enter factory mode success");
						msleep(200);
						return 0;
					} else
						msleep(FACTORY_TEST_DELAY);
				}
			}

			msleep(50);
		}

		if (i >= 5) {
			FTS_INFO("Enter factory mode fail");
			return -EIO;
		}

		return 0;
	}


	static int fts_raw_get_channel_num(u8 tx_rx_reg, u8 *ch_num, u8 ch_num_max)
	{
		int ret = 0;
		int i = 0;

		for (i = 0; i < 3; i++) {
			ret = fts_read(&tx_rx_reg,1, ch_num,1);
			if ((ret < 0) || (*ch_num > ch_num_max)) {
				msleep(50);
			} else
				break;
		}

		if (i >= 3) {
			FTS_INFO("get channel num fail");
			return -EIO;
		}

		return 0;
	}


	int fts_raw_start_scan(void)
	{
		int ret = 0;
		u8 addr = 0;
		u8 val = 0;
		u8 finish_val = 0;
		int times = 0;
		addr = DEVICE_MODE_ADDR;
		val = 0xC0;
		finish_val = 0x40;
		
		fts_raw_enter_factory_mode();
		ret = fts_write_reg(addr, val);
		if (ret < 0) {
			FTS_INFO("write start scan mode fail\n");
			return ret;
		}
		
		while (times++ < 50) {
			msleep(18);

			ret = fts_read(&addr,1, &val,1);
			if ((ret >= 0) && (val == finish_val)) {
				break;
			} else
				FTS_INFO("reg%x=%x,retry:%d", addr, val, times);
		}

		if (times >= 50) {
			FTS_INFO("scan timeout\n");
			return -EIO;
		}

		return 0;
		
	}


	int fts_raw_read_rawdata(u8 addr, u8 *readbuf, int readlen)
	{
		int ret = 0;
		int i = 0;
		int packet_length = 0;
		int packet_num = 0;
		int packet_remainder = 0;
		int offset = 0;
		int byte_num = readlen;

		packet_num = byte_num / packet;
		packet_remainder = byte_num % packet;
		if (packet_remainder)
			packet_num++;

		if (byte_num < packet) {
			packet_length = byte_num;
		} else {
			packet_length = packet;
		}
		/* FTS_TEST_DBG("packet num:%d, remainder:%d", packet_num, packet_remainder); */
		ret = fts_read(&addr, 1, &readbuf[offset], packet_length);
		if (ret < 0) {
			FTS_INFO("read buffer fail");
			return ret;
		}
		for (i = 1; i < packet_num; i++) {
			offset += packet_length;
			if ((i == (packet_num - 1)) && packet_remainder) {
				packet_length = packet_remainder;
			}

			ret = fts_read(&addr, 1, &readbuf[offset],
									packet_length);

			if (ret < 0) {
				FTS_INFO("read buffer fail");
				return ret;
			}
		}

		return 0;
	}


	static int fts_raw_read(
		u8 off_addr,
		u8 off_val,
		u8 rawdata_addr,
		int byte_num,
		int *buf)
		{
			int ret = 0;
			int i = 0;
			u8 *data = NULL;
			
			/* set line addr or rawdata start addr */
			ret = fts_write_reg(off_addr, off_val);
			if (ret < 0) {
				FTS_INFO("wirte line/start addr fail\n");
				return ret;
			}

			data = (u8 *)kzalloc(byte_num * sizeof(u8), GFP_KERNEL);
			if (NULL == data) {
				FTS_INFO("mass data buffer malloc fail\n");
				return -ENOMEM;
			}

			/* read rawdata buffer */
			FTS_INFO("mass data len:%d", byte_num);
			ret = fts_raw_read_rawdata(rawdata_addr, data, byte_num);
			if (ret < 0) {
				FTS_INFO("read mass data fail\n");
				goto read_massdata_err;
			}

			for (i = 0; i < byte_num; i = i + 2) {
				buf[i >> 1] = (int)(short)((data[i] << 8) + data[i + 1]);
			}

			ret = 0;
		read_massdata_err:
			kfree(data);
			return ret;
			
		}
	
	
	int fts_rawdata_differ(u8 raw_diff,char *buf)
	{
		
		u8 val = 0xAD;
        u8 addr = FACTORY_REG_LINE_ADDR;
        u8 rawdata_addr = FACTORY_REG_RAWDATA_ADDR;
		int count = 0;
		int i = 0;
		int j = 0;
		int k = 0;
		int ret = 0;
		u8 cmd[2] = {0};
		
		u8 tx_num = 0;
		u8 rx_num = 0;
		int byte_num = 0;
		int buffer_length = 0;
		int *buffer = NULL;
		struct input_dev *input_dev = fts_data->input_dev;
		
		fts_irq_disable();
		mutex_lock(&input_dev->mutex);
		
		cmd[0] = 0xEE;
		cmd[1] = 1;
		fts_write(cmd, 2);
		fts_raw_enter_factory_mode();
		
		fts_raw_get_channel_num(FACTORY_REG_CHX_NUM, &tx_num, 60);
		fts_raw_get_channel_num(FACTORY_REG_CHY_NUM, &rx_num, 60);
		FTS_ERROR("tx_num = %d  rx_num = %d",tx_num,rx_num);
		cmd[0] = FACTORY_REG_DATA_SELECT;
		cmd[1] = raw_diff;
		fts_write(cmd, 2);
	
		fts_raw_start_scan();
		byte_num = tx_num * rx_num *2;
		buffer_length = (tx_num + 1) * rx_num;
		buffer_length *= sizeof(int) * 2;
		FTS_INFO("test buffer length:%d", buffer_length);
		buffer = (int *)kzalloc(buffer_length, GFP_KERNEL);
		if (NULL == buffer) {
			FTS_INFO("test buffer(%d) malloc fail", buffer_length);
			goto read_err;
		}
		memset(buffer, 0, buffer_length);
		
		ret = fts_raw_read(addr, val, rawdata_addr, byte_num, buffer);
		if (ret < 0) {
			FTS_INFO("read rawdata fail\n");
			goto read_err;
		}
		for (i = 0; i < tx_num; i++) {
			for(j = 0;j<rx_num;j++)
			{
				count += snprintf(buf + count, PAGE_SIZE-count, "%4d\t", buffer[k++]);
			}
			count += snprintf(buf + count, PAGE_SIZE-count, "\n");
		}
		
		
	read_err:
			cmd[0] = DEVICE_MODE_ADDR;
			cmd[1] = 0x00;
			ret = fts_write(cmd, 2);
			cmd[0] = 0xEE;
			cmd[1] = 0x00;
			ret = fts_write(cmd, 2);
			kfree(buffer);
			mutex_unlock(&input_dev->mutex);		
			fts_irq_enable();
			return count;
		
	}
	
	ssize_t fts_rawdata_show(
    struct device *dev, struct device_attribute *attr, char *buf)
	{
		int count = 0;
		count = fts_rawdata_differ(0,buf);
		return count;
		
	}
	
	
	ssize_t fts_differ_show(
    struct device *dev, struct device_attribute *attr, char *buf)
	{
		int count = 0;
		count = fts_rawdata_differ(1,buf);
		return count;
	}

