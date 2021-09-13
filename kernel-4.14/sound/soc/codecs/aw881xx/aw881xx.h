#ifndef __AW881XX_H__
#define __AW881XX_H__

/*
 * i2c transaction on Linux limited to 64k
 * (See Linux kernel documentation: Documentation/i2c/writing-clients)
*/
#define MAX_I2C_BUFFER_SIZE			(65536)

#define AW881XX_FLAG_START_ON_MUTE		(1 << 0)
#define AW881XX_FLAG_SKIP_INTERRUPTS		(1 << 1)

#define AW881XX_NUM_RATES			(9)
#define AW881XX_SYSST_CHECK_MAX			(10)

#define AW881XX_DFT_CALI_RE			(0x8000)

#define AW881XX_MONITOR_DFT_FLAG		(0)
#define AW881XX_MONITOR_TIMER_DFT_VAL		(30000)

#define AW881XX_VBAT_COEFF_INT_10BIT		(1023)

/********************************************
 *
 * DSP I2C WRITES
 *
 *******************************************/
#define AW881XX_DSP_I2C_WRITES
#define AW881XX_MAX_RAM_WRITE_BYTE_SIZE		(128)


/********************************************
 *
 * Register List
 *
 *******************************************/
#define AW881XX_REG_MAX				(0x7F)
#define AW881XX_REG_ID				(0x00)
#define AW881XX_REG_SYSST			(0x01)
#define AW881XX_REG_SYSINT			(0x02)
#define AW881XX_REG_SYSINTM			(0x03)
#define AW881XX_REG_SYSST			(0x01)
#define AW881XX_REG_SYSST			(0x01)
#define AW881XX_REG_HAGCCFG7			(0x0F)
#define AW881XX_REG_SYSINTM_MASK		(0xffff)
#define AW881XX_VOL_REG_SHIFT			(8)
#define AW881XX_VOLUME_MIN			(-192)
#define AW881XX_VOLUME_MAX			(0)
#define AW881XX_REG_PRODUCT_ID			(0x79)
#define AW881XX_BIT_PRODUCT_ID_MASK		(~(15 << 0))
#define AW881XX_BIT_SYSST_PLLS			(1 << 0)

#define REG_NONE_ACCESS				(0)
#define REG_RD_ACCESS				(1 << 0)
#define REG_WR_ACCESS				(1 << 1)

#define AW881XX_REG_DSP_PID			(0x1F80)

#define AW881XX_DSP_RE_SHIFT			(12)

/********************************************
 *
 * enum
 *
 *******************************************/
enum aw881xx_scene_mode {
	AW881XX_SPK_MODE,
	AW881XX_VOICE_MODE,
	AW881XX_FM_MODE,
	AW881XX_RCV_MODE,
	AW881XX_MODE_MAX,
};

#define AW881XX_MODE_CFG_NUM_MAX		(3)
#define AW881XX_CFG_NUM_MAX			(AW881XX_MODE_MAX * AW881XX_MODE_CFG_NUM_MAX)
#define AW881XX_CFG_NAME_MAX			(64)


enum aw881xx_id {
	AW881XX_CHIPID = 0x1806,
	AW881XX_PID_01 = 0x01,
	AW881XX_PID_02 = 0x02,
	AW881XX_PID_03 = 0x03,
};

enum aw881xx_dsp_pid {
	AW881XX_DSP_PID_01 = 0x0000,
	AW881XX_DSP_PID_02 = 0x0001,
	AW881XX_DSP_PID_03 = 0x6E90,
};

enum aw881xx_init {
	AW881XX_INIT_ST = 0,
	AW881XX_INIT_OK = 1,
	AW881XX_INIT_NG = 2,
};

enum aw881xx_dsp_cfg {
	AW881XX_DSP_WORK = 0,
	AW881XX_DSP_BYPASS = 1,
};


enum aw881xx_baseaddr {
	AW881XX_SPK_REG_ADDR = 0x00,
	AW881XX_SPK_DSP_FW_ADDR = 0x8c00,
	AW881XX_SPK_DSP_CFG_ADDR = 0x8600,
	AW881XX_RCV_REG_ADDR = 0x00,
	AW881XX_RCV_DSP_FW_ADDR = 0x8c00,
	AW881XX_RCV_DSP_CFG_ADDR = 0x8600,
	AW881XX_BASE_ADDR_MAX = 6,
};

/********************************************
 *
 * device ops interface
 *
 *******************************************/
/* asoc interface */
typedef int (*update_hw_params_t)(void *);
typedef void (*get_volume_t)(void *, unsigned int *);
typedef void (*set_volume_t)(void *, unsigned int);

/* aw881xx samrtpa control */
typedef int (*start_t)(void *);
typedef void (*stop_t)(void *);
typedef void (*update_dsp_t)(void *);
typedef void (*run_pwd_t)(void *, bool);
typedef void (*monitor_t)(void *);
typedef void (*set_cali_re_t)(void *);

/* aw881xx interrupt */
typedef void (*interrupt_setup_t)(void *);
typedef void (*interrupt_handle_t)(void *);

/* get aw881xx config and status */
typedef int (*get_hmute_t)(void *);
typedef int (*get_dsp_config_t)(void *);
typedef int (*get_iis_status_t)(void *);
typedef unsigned char (*get_reg_access_t)(uint8_t);
typedef int (*get_dsp_mem_reg_t)(void *, uint8_t *, uint8_t *);

struct device_ops {
	update_hw_params_t update_hw_params;
	get_volume_t get_volume;
	set_volume_t set_volume;

	start_t start;
	stop_t stop;
	update_dsp_t update_dsp;
	run_pwd_t run_pwd;
	monitor_t monitor;
	set_cali_re_t set_cali_re;

	interrupt_setup_t interrupt_setup;
	interrupt_handle_t interrupt_handle;

	get_hmute_t get_hmute;
	get_dsp_config_t get_dsp_config;
	get_iis_status_t get_iis_status;
	get_reg_access_t get_reg_access;
	get_dsp_mem_reg_t get_dsp_mem_reg;
};

/********************************************
 *
 * aw881xx struct
 *
 *******************************************/
struct aw881xx {
	struct regmap *regmap;
	struct i2c_client *i2c;
	struct snd_soc_codec *codec;
	struct device *dev;
	struct mutex lock;
	struct mutex i2c_lock;
	struct hrtimer monitor_timer;
	struct work_struct monitor_work;
	struct device_ops dev_ops;

	int sysclk;
	int rate;
	int width;
	int pstream;
	int cstream;

	int reset_gpio;
	int irq_gpio;

	uint8_t flags;
	bool work_flag;
	uint8_t init;
	uint8_t scene_mode;

	uint16_t chipid;
	uint8_t pid;

	uint8_t reg_addr;
	uint16_t dsp_addr;

	uint8_t dsp_cfg;
	char cfg_name[AW881XX_CFG_NUM_MAX][AW881XX_CFG_NAME_MAX];
	uint16_t cfg_num;
	uint32_t dsp_fw_len;
	uint32_t dsp_cfg_len;

	uint16_t cali_re;
	uint16_t re;
	uint16_t f0;

	uint32_t monitor_flag;
	uint32_t monitor_timer_val;

	int32_t pre_vol_bst_ipeak;
	int32_t pre_vol_gain_db;
	int32_t pre_temp;
	int32_t pre_temp_bst_ipeak;
	int32_t pre_temp_gain_db;
	int32_t pre_temp_vmax;

	uint16_t intmask;
};

struct aw881xx_container {
	int len;
	unsigned char data[];
};

/******************************************************
 *
 * aw881xx i2c write/read
 *
 ******************************************************/
int aw881xx_reg_writes(struct aw881xx *aw881xx,
	uint8_t reg_addr, uint8_t *buf, uint16_t len);
int aw881xx_reg_write(struct aw881xx *aw881xx,
	uint8_t reg_addr, uint16_t reg_data);
int aw881xx_reg_read(struct aw881xx *aw881xx,
	uint8_t reg_addr, uint16_t *reg_data);
int aw881xx_reg_write_bits(struct aw881xx *aw881xx,
	uint8_t reg_addr, uint16_t mask, uint16_t reg_data);
int aw881xx_dsp_write(struct aw881xx *aw881xx,
	uint16_t dsp_addr, uint16_t dsp_data);
int aw881xx_dsp_read(struct aw881xx *aw881xx,
	uint16_t dsp_addr, uint16_t *dsp_data);


void aw881xx_get_cfg_shift(struct aw881xx *aw881xx);
int aw881xx_dsp_update_cali_re(struct aw881xx *aw881x);
int aw881xx_read_dsp_pid(struct aw881xx *aw881xx);

/******************************************************
 *
 * aw881xx i2c monitor
 *
 ******************************************************/
int aw881xx_monitor_start(struct aw881xx *aw881xx);
int aw881xx_monitor_stop(struct aw881xx *aw881xx);

extern int aw881xx_i2c_probe(struct i2c_client *i2c,
	const struct i2c_device_id *id);
extern int aw881xx_i2c_remove(struct i2c_client *i2c);
#endif
