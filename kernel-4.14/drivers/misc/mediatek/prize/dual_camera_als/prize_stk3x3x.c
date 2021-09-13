/**************************************************************************
*  double_camera.c
* 
*  Create Date : 
* 
*  Modify Date : 
*
*  Create by   : AWINIC Technology CO., LTD
*
*  Version     : 0.9, 2016/02/15
**************************************************************************/

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <asm/io.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include "prize_dual_camera_als.h"
#include "prize_stk3x3x.h"

#ifndef C_I2C_FIFO_SIZE
#define C_I2C_FIFO_SIZE     10
#endif
static DEFINE_MUTEX(stk3x3x_i2c_mutex);
/*----------------------------------------------------------------------------*/
static int stk3x3x_hwmsen_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
    int err;
    u8 beg = addr;
    struct i2c_msg msgs[2] = {
        {
            .addr = client->addr,
            .flags = 0,
            .len = 1,
            .buf = &beg
        },
        {
            .addr = client->addr,
            .flags = I2C_M_RD,
            .len = len,
            .buf = data,
        }
    };
    mutex_lock(&stk3x3x_i2c_mutex);

    if (!client) {
        mutex_unlock(&stk3x3x_i2c_mutex);
        return -EINVAL;
    } else if (len > C_I2C_FIFO_SIZE) {
        mutex_unlock(&stk3x3x_i2c_mutex);
        camera_als_dbg("%s length %d exceeds %d\n", __func__, len, C_I2C_FIFO_SIZE);
        return -EINVAL;
    }

    err = i2c_transfer(client->adapter, msgs, sizeof(msgs) / sizeof(msgs[0]));
    mutex_unlock(&stk3x3x_i2c_mutex);

    if (err != 2) {
        camera_als_dbg("%s i2c_transfer error: (%d %p %d) %d\n", __func__, addr, data, len, err);
        err = -EIO;
    } else {
        err = 0; /* no error */
    }

    return err;
}

static int stk3x3x_master_recv(struct i2c_client *client, u16 addr, u8 *buf, int count)
{
	int ret = 0, retry = 0;

	while (retry++ < 3) {
		ret = stk3x3x_hwmsen_read_block(client, addr, buf, count);
		if (ret == 0)
			break;
		udelay(100);
	}

	return (ret == 0) ? count : ret;
}
/*----------------------------------------------------------------------------*/
static int hwmsen_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	int err, idx, num;
	char buf[C_I2C_FIFO_SIZE];

	if (!client)
		return -EINVAL;
	else if (len >= C_I2C_FIFO_SIZE) {
		camera_als_dbg("%s length %d exceeds %d\n", __func__, len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	num = 0;
	buf[num++] = addr;
	for (idx = 0; idx < len; idx++)
		buf[num++] = data[idx];

	err = i2c_master_send(client, buf, num);
	if (err < 0) {
		camera_als_dbg("%s send command error!!\n", __func__);
		return -EFAULT;
	}
	err = 0;	/*no error*/

	return err;
}

static int stk3x3x_master_send(struct i2c_client *client, u16 addr, u8 *buf, int count)
{
	int ret = 0, retry = 0;

	while (retry++ < 3) {
		ret = hwmsen_write_block(client, addr, buf, count);
		if (ret == 0)
			break;
		udelay(100);
	}

	return (ret == 0) ? count : ret;
}
/*----------------------------------------------------------------------------*/

static void stk3x3x_sensor_init(struct i2c_client *client)
{
    u8 buf[8] = {0};
    
	// check if sensor ID correct
    if (stk3x3x_master_recv(client, STK_PDT_ID_REG, buf, 0x01) < 0) {
        camera_als_dbg("%s stk3x3x_master_recv STK_PDT_ID_REG error\n", __func__);
    }
	camera_als_dbg("%s STK3X3X Read ID %x\n", __func__, buf[0]);
	if(buf[0] == 0x51){	
		/* als default init */
		buf[0] =  ((STK_ALS_PRS1 << STK_ALS_PRST_SHIFT) |(STK_ALS_GAIN64 << STK_ALS_GAIN_SHIFT) |
				(STK_ALS_IT50 << STK_ALS_IT_SHIFT));
		stk3x3x_master_send(client, STK_ALSCTRL_REG, buf, 1);
		
		buf[0] = 0x30;
		stk3x3x_master_send(client, STK_ALSCTRL2_REG, buf, 1);
		
		buf[0] = 0x10;
		stk3x3x_master_send(client, 0xA0, buf, 1);
		
		buf[0] = 0x7F;
		stk3x3x_master_send(client, 0xA1, buf, 1);
	
		buf[0] = 0x64;
		stk3x3x_master_send(client, 0xAA, buf, 1);
		
		buf[0] = 0x01;
		stk3x3x_master_send(client, 0xFB, buf, 1);
		
		buf[0] = 0x82;
		stk3x3x_master_send(client, 0xF6, buf, 1);
    }else{
		   /* als default init */
		buf[0] =  ((STK_ALS_PRS1 << STK_ALS_PRST_SHIFT) |(STK_ALS_GAIN64 << STK_ALS_GAIN_SHIFT) |
				(STK_ALS_IT100 << STK_ALS_IT_SHIFT));
		stk3x3x_master_send(client, STK_ALSCTRL_REG, buf, 1);
		
		buf[0] = 0x30;
		stk3x3x_master_send(client, STK_ALSCTRL2_REG, buf, 1);
		
		buf[0] = 0x0F;
		stk3x3x_master_send(client, 0xA1, buf, 1);
		
		buf[0] = 0x15;
		stk3x3x_master_send(client, 0xDB, buf, 1);
    }
	
    buf[0] = STK_STATE_EN_ALS_MASK;
    stk3x3x_master_send(client, STK_STATE_REG, buf, 1);
    
    camera_als_dbg("%s success\n", __func__);
}

static unsigned short stk3x3x_read_shutter(struct i2c_client *client)
{
    unsigned short als_data;
    u8 buf[8] = {0};
    
    if (stk3x3x_master_recv(client, STK_DATA1_ALS_REG, buf, 0x02) < 0) {
        camera_als_dbg("%s stk3x3x_master_recv STK_DATA1_ALS_REG error\n", __func__);
    }
    als_data = (buf[0] << 8) | (buf[1]);
    camera_als_dbg("%s %d\n", __func__, als_data);

	return als_data;
}

static unsigned int stk3x3x_get_sensor_id(struct i2c_client *client,unsigned int *sensorID)
{
    u8 buf[2] = {0};
    
    // check if sensor ID correct
    if (stk3x3x_master_recv(client, STK_PDT_ID_REG, buf, 0x01) < 0) {
        camera_als_dbg("%s stk3x3x_master_recv STK_PDT_ID_REG error\n", __func__);
    }
    *sensorID = buf[0];
	camera_als_dbg("%s STK3X3X Read ID %x, PID=0x%x\n", __func__, *sensorID, buf[0]);

    return 0;
}

static int stk3x3x_set_power(struct i2c_client *client,unsigned int enable)
{
    u8 old_state = 0, new_state = 0;
    u8 buf[8] = {0};
    if (stk3x3x_master_recv(client, STK_STATE_REG, buf, 1) < 0) {
        camera_als_dbg("%s stk3x3x_master_recv STK_STATE_REG error\n", __func__);
    }
    old_state = buf[0];

    if (enable)
    {
        new_state = old_state & (~(STK_STATE_EN_ALS_MASK | STK_STATE_EN_WAIT_MASK));
        new_state |= STK_STATE_EN_ALS_MASK;
        new_state &= 0xF7;
    } else {
        new_state = old_state & (~(STK_STATE_EN_ALS_MASK | STK_STATE_EN_WAIT_MASK));
    }
    
    /* write new state*/
    buf[0] = new_state;
    if (stk3x3x_master_send(client, STK_STATE_REG, buf, 1) < 0) {
        camera_als_dbg("%s stk3x3x_master_send STK_STATE_REG error\n", __func__);
    }

    return 0;
}

static int stk3x3x_open(struct i2c_client *client)
{
    int i;
    u8 buf[8] = {0}, r_buf = 0;

	//  Read sensor ID to adjust I2C is OK?
	for(i=0;i<3;i++)
	{
        buf[0] = 0x7F;
        if (stk3x3x_master_send(client, STK_WAIT_REG, buf, 1) < 0) {
            camera_als_dbg("%s stk3x3x_master_send STK_WAIT_REG error\n", __func__);
            return -EINVAL;
        }
        if (stk3x3x_master_recv(client, STK_WAIT_REG, &r_buf, 1) < 0) {
            camera_als_dbg("%s stk3x3x_master_recv STK_WAIT_REG error\n", __func__);
            return -EINVAL;
        }
        if (buf[0] != r_buf) {
            camera_als_dbg("%s i2c r/w test error, read-back value is not the same, write=0x%x, read=0x%x\n", __func__, buf, r_buf);
            return -EINVAL;
        }
	}
	
	camera_als_dbg("%s stk3x3x Read ID OK\n", __func__);
	stk3x3x_sensor_init(client);

	return 0;
}

const struct sensor_info_t stk3x3x_info = {
	.sensor_type = SENSOR_TYPE_STK3X3X,
	.open = stk3x3x_open,
	.init = stk3x3x_sensor_init,
	.get_shutter = stk3x3x_read_shutter,
	.get_sensor_id = stk3x3x_get_sensor_id,
	.set_power = stk3x3x_set_power,
};
