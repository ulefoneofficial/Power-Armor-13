#ifndef __PRIZE_DUAL_CAM__
#define __PRIZE_DUAL_CAM__

#define CAMERA_ALS_DEBUG
#ifdef CAMERA_ALS_DEBUG
#define camera_als_dbg(fmt,arg...) \
	do{\
		printk("<<SPC-DBG>>[%d]"fmt"", __LINE__, ##arg);\
    }while(0)
#else
#define camera_als_dbg(fmt,arg...)
#endif

enum SENSOR_TYPE {
	SENSOR_TYPE_STK3X3X = 0,
	SENSOR_TYPE_UNKNOWN,
};

struct sensor_info_t {
	enum SENSOR_TYPE sensor_type;
	int (*open)(struct i2c_client *);
	void (*init)(struct i2c_client *);
	unsigned short (*get_shutter)(struct i2c_client *);
	unsigned int (*get_sensor_id)(struct i2c_client *,unsigned int *id);
	int (*set_power)(struct i2c_client *,unsigned int enable);
};

struct spc_data_t{
	char name[8];
	char is_enabled;
	enum SENSOR_TYPE sensor_type;
	struct sensor_info_t *ops;
	struct mutex ops_mutex;
};

#endif