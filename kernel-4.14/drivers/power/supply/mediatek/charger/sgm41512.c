/*
 * SGM SGM41512 charger driver by SUYI
 *
 * Copyright (C) 2021 SGMicro Corporation
 *
 * This driver is for Linux kernel 4.4
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/i2c.h>
#include <linux/of_device.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/reboot.h>

#include <mt-plat/upmu_common.h>
#include <mt-plat/charger_class.h>
#include <mt-plat/charger_type.h>
#ifdef CONFIG_RT_REGMAP
#include <mt-plat/rt-regmap.h>
#endif /* CONFIG_RT_REGMAP */
#include <mt-plat/mtk_boot_common.h>

#include "mtk_charger_intf.h"
#include "sgm41512.h"
#define SGM41512_DRV_VERSION	"1.0.0_SGM"

enum SGM41512_streg_idx {
	SGM41512_ST_STATUS = 0,
	SGM41512_ST_FAULT,
	SGM41512_ST_STATUS2,
	SGM41512_STREG_MAX,
};

enum sgm41512_ic_stat {
	SGM41512_ICSTAT_SLEEP = 0,
	SGM41512_ICSTAT_PRECHG,
	SGM41512_ICSTAT_FASTCHG,
	SGM41512_ICSTAT_CHGDONE,
	SGM41512_ICSTAT_MAX,
};

enum sgm41512_mivr_track {
	SGM41512_MIVRTRACK_REG = 0,
	SGM41512_MIVRTRACK_VBAT_200MV,
	SGM41512_MIVRTRACK_VBAT_250MV,
	SGM41512_MIVRTRACK_VBAT_300MV,
	SGM41512_MIVRTRACK_MAX,
};

enum sgm41512_port_stat {
	SGM41512_PORTSTAT_NOINPUT = 0,
	SGM41512_PORTSTAT_SDP,
	SGM41512_PORTSTAT_CDP,
	SGM41512_PORTSTAT_DCP,
	SGM41512_PORTSTAT_RESERVED_1,
	SGM41512_PORTSTAT_UKNADPT,
	SGM41512_PORTSTAT_NSADPT,
	SGM41512_PORTSTAT_OTG,
	SGM41512_PORTSTAT_MAX,
};

enum sgm41512_usbsw_state {
	SGM41512_USBSW_CHG = 0,
	SGM41512_USBSW_USB,
};

struct sgm41512_desc {
	const char *rm_name;
	u8 rm_slave_addr;
	u32 vac_ovp;
	u32 mivr;
	u32 aicr;
	u32 cv;
	u32 ichg;
	u32 ieoc;
	u32 safe_tmr;
	u32 wdt;
	u32 mivr_track;
	bool en_safe_tmr;
	bool en_te;
	bool en_jeita;
	bool ceb_invert;
	bool dis_i2c_tout;
	bool en_qon_rst;
	bool auto_aicr;
	const char *chg_name;
};

/* These default values will be applied if there's no property in dts */
static struct sgm41512_desc sgm41512_default_desc = {
	.rm_name = "sgm41512",
	.rm_slave_addr = SGM41512_SLAVE_ADDR,
	.vac_ovp = 6500000,
	.mivr = 4400000,
	.aicr = 500000,
	.cv = 4400000,
	.ichg = 2000000,
	.ieoc = 200000,
	.safe_tmr = 10,
	.wdt = 40,
	.mivr_track = SGM41512_MIVRTRACK_REG,
	.en_safe_tmr = true,
	.en_te = true,
	.en_jeita = true,
	.ceb_invert = false,
	.dis_i2c_tout = false,
	.en_qon_rst = true,
	.auto_aicr = true,
	.chg_name = "primary_chg",
};

static const u8 sgm41512_irq_maskall[SGM41512_STREG_MAX] = {
	0xFF, 0xFF, 0xFF,
};

static const u32 sgm41512_vac_ovp[] = {
	5500000, 6500000, 10500000, 14000000,
};

static const u32 sgm41512_wdt[] = {
	0, 40, 80, 160,
};

static const u32 sgm41512_otgcc[] = {
	500000, 1200000,
};

struct sgm41512_chip {
	struct i2c_client *client;
	struct device *dev;
	struct charger_device *chg_dev;
	struct charger_properties chg_props;
	struct mutex io_lock;
#ifdef CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT
	struct mutex bc12_lock;
	struct mutex bc12_en_lock;
#endif /* CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT */
	struct mutex hidden_mode_lock;
	int hidden_mode_cnt;
	u8 dev_id;
	u8 dev_rev;
	u8 chip_rev;
	struct sgm41512_desc *desc;
	u32 intr_gpio;
	u32 ceb_gpio;
	int irq;
	u8 irq_mask[SGM41512_STREG_MAX];
#ifdef CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT
	struct delayed_work psy_dwork;
	atomic_t vbus_gd;
	bool attach;
	enum sgm41512_port_stat port;
	enum charger_type chg_type;
	struct power_supply *psy;
	struct wakeup_source bc12_en_ws;
	int bc12_en_buf[2];
	int bc12_en_buf_idx;
	atomic_t bc12_en_req_cnt;
	wait_queue_head_t bc12_en_req;
	struct task_struct *bc12_en_kthread;
#endif /* CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT */
	bool chg_done_once;
	struct wakeup_source buck_dwork_ws;
	struct delayed_work buck_dwork;
#ifdef CONFIG_RT_REGMAP
	struct rt_regmap_device *rm_dev;
	struct rt_regmap_properties *rm_prop;
#endif /* CONFIG_RT_REGMAP */
	bool enter_shipping_mode;
	//struct completion aicc_done;
	struct completion pe_done;
	bool bc12_enable_done;
};

static const u8 sgm41512_reg_addr[] = {
	SGM41512_REG_00,
	SGM41512_REG_01,
	SGM41512_REG_02,
	SGM41512_REG_03,
	SGM41512_REG_04,
	SGM41512_REG_05,
	SGM41512_REG_06,
	SGM41512_REG_07,
	SGM41512_REG_08,
	SGM41512_REG_09,
	SGM41512_REG_0A,
	SGM41512_REG_0B,
};

static int sgm41512_read_device(void *client, u32 addr, int len, void *dst)
{
	return i2c_smbus_read_i2c_block_data(client, addr, len, dst);
}

static int sgm41512_write_device(void *client, u32 addr, int len,
			       const void *src)
{
	return i2c_smbus_write_i2c_block_data(client, addr, len, src);
}

#ifdef CONFIG_RT_REGMAP
RT_REG_DECL(SGM41512_REG_00, 1, RT_VOLATILE, {});
RT_REG_DECL(SGM41512_REG_01, 1, RT_VOLATILE, {});
RT_REG_DECL(SGM41512_REG_02, 1, RT_VOLATILE, {});
RT_REG_DECL(SGM41512_REG_03, 1, RT_VOLATILE, {});
RT_REG_DECL(SGM41512_REG_04, 1, RT_VOLATILE, {});
RT_REG_DECL(SGM41512_REG_05, 1, RT_VOLATILE, {});
RT_REG_DECL(SGM41512_REG_06, 1, RT_VOLATILE, {});
RT_REG_DECL(SGM41512_REG_07, 1, RT_VOLATILE, {});
RT_REG_DECL(SGM41512_REG_08, 1, RT_VOLATILE, {});
RT_REG_DECL(SGM41512_REG_09, 1, RT_VOLATILE, {});
RT_REG_DECL(SGM41512_REG_0A, 1, RT_VOLATILE, {});
RT_REG_DECL(SGM41512_REG_0B, 1, RT_VOLATILE, {});

static const rt_register_map_t sgm41512_rm_map[] = {
	RT_REG(SGM41512_REG_00),
	RT_REG(SGM41512_REG_01),
	RT_REG(SGM41512_REG_02),
	RT_REG(SGM41512_REG_03),
	RT_REG(SGM41512_REG_04),
	RT_REG(SGM41512_REG_05),
	RT_REG(SGM41512_REG_06),
	RT_REG(SGM41512_REG_07),
	RT_REG(SGM41512_REG_08),
	RT_REG(SGM41512_REG_09),
	RT_REG(SGM41512_REG_0A),
	RT_REG(SGM41512_REG_0B),
};

static struct rt_regmap_fops sgm41512_rm_fops = {
	.read_device = sgm41512_read_device,
	.write_device = sgm41512_write_device,
};

static int sgm41512_register_rt_regmap(struct sgm41512_chip *chip)
{
	struct rt_regmap_properties *prop = NULL;

	dev_info(chip->dev, "%s\n", __func__);

	prop = devm_kzalloc(chip->dev, sizeof(*prop), GFP_KERNEL);
	if (!prop)
		return -ENOMEM;

	prop->name = chip->desc->rm_name;
	prop->aliases = chip->desc->rm_name;
	prop->register_num = ARRAY_SIZE(sgm41512_rm_map);
	prop->rm = sgm41512_rm_map;
	prop->rt_regmap_mode = RT_SINGLE_BYTE | RT_CACHE_DISABLE |
			       RT_IO_PASS_THROUGH | RT_DBG_SPECIAL;
	prop->io_log_en = 0;

	chip->rm_prop = prop;
	chip->rm_dev = rt_regmap_device_register_ex(chip->rm_prop,
						    &sgm41512_rm_fops, chip->dev,
						    chip->client,
						    chip->desc->rm_slave_addr,
						    chip);
	if (!chip->rm_dev) {
		dev_notice(chip->dev, "%s fail\n", __func__);
		return -EIO;
	}

	return 0;
}
#endif /* CONFIG_RT_REGMAP */

static inline int __sgm41512_i2c_read_byte(struct sgm41512_chip *chip, u8 cmd,
					 u8 *data)
{
	int ret = 0;
	u8 regval = 0;

#ifdef CONFIG_RT_REGMAP
	ret = rt_regmap_block_read(chip->rm_dev, cmd, 1, &regval);
#else
	ret = sgm41512_read_device(chip->client, cmd, 1, &regval);
#endif /* CONFIG_RT_REGMAP */

	if (ret < 0)
		dev_notice(chip->dev, "%s reg0x%02X fail(%d)\n",
				      __func__, cmd, ret);
	else {
		/*dev_dbg(chip->dev, "%s reg0x%02X = 0x%02X\n",
				   __func__, cmd, regval);*/
		*data = regval;
	}

	return ret;
}

static int sgm41512_i2c_read_byte(struct sgm41512_chip *chip, u8 cmd, u8 *data)
{
	int ret = 0;

	mutex_lock(&chip->io_lock);
	ret = __sgm41512_i2c_read_byte(chip, cmd, data);
	mutex_unlock(&chip->io_lock);

	return ret;
}

static inline int __sgm41512_i2c_write_byte(struct sgm41512_chip *chip, u8 cmd,
					  u8 data)
{
	int ret = 0;

#ifdef CONFIG_RT_REGMAP
	ret = rt_regmap_block_write(chip->rm_dev, cmd, 1, &data);
#else
	ret = sgm41512_write_device(chip->client, cmd, 1, &data);
#endif /* CONFIG_RT_REGMAP */

	if (ret < 0)
		dev_notice(chip->dev, "%s reg0x%02X = 0x%02X fail(%d)\n",
				      __func__, cmd, data, ret);
	/*else
		dev_dbg(chip->dev, "%s reg0x%02X = 0x%02X\n",
				   __func__, cmd, data);*/

	return ret;
}
/*
static int sgm41512_i2c_write_byte(struct sgm41512_chip *chip, u8 cmd, u8 data)
{
	int ret = 0;

	mutex_lock(&chip->io_lock);
	ret = __sgm41512_i2c_write_byte(chip, cmd, data);
	mutex_unlock(&chip->io_lock);

	return ret;
}
*/
static inline int __sgm41512_i2c_block_read(struct sgm41512_chip *chip, u8 cmd,
					  u32 len, u8 *data)
{
	int ret = 0;/* i = 0;*/

#ifdef CONFIG_RT_REGMAP
	ret = rt_regmap_block_read(chip->rm_dev, cmd, len, data);
#else
	ret = sgm41512_read_device(chip->client, cmd, len, data);
#endif /* CONFIG_RT_REGMAP */

	if (ret < 0)
		dev_notice(chip->dev, "%s reg0x%02X..reg0x%02X fail(%d)\n",
				      __func__, cmd, cmd + len - 1, ret);
	/*else
		for (i = 0; i <= len - 1; i++)
			dev_dbg(chip->dev, "%s reg0x%02X = 0x%02X\n",
					   __func__, cmd + i, data[i]);*/

	return ret;
}

static int sgm41512_i2c_block_read(struct sgm41512_chip *chip, u8 cmd, u32 len,
				 u8 *data)
{
	int ret = 0;

	mutex_lock(&chip->io_lock);
	ret = __sgm41512_i2c_block_read(chip, cmd, len, data);
	mutex_unlock(&chip->io_lock);

	return ret;
}

static inline int __sgm41512_i2c_block_write(struct sgm41512_chip *chip, u8 cmd,
					   u32 len, const u8 *data)
{
	int ret = 0,i = 0;

#ifdef CONFIG_RT_REGMAP
	ret = rt_regmap_block_write(chip->rm_dev, cmd, len, data);
#else
	ret = sgm41512_write_device(chip->client, cmd, len, data);
#endif /* CONFIG_RT_REGMAP */

	if (ret < 0) {
		dev_notice(chip->dev, "%s fail(%d)\n", __func__, ret);
		for (i = 0; i <= len - 1; i++)
			dev_notice(chip->dev, "%s reg0x%02X = 0x%02X\n",
					      __func__, cmd + i, data[i]);
	} 
	/*else
		for (i = 0; i <= len - 1; i++)
			dev_dbg(chip->dev, "%s reg0x%02X = 0x%02X\n",
					   __func__, cmd + i, data[i]);*/

	return ret;
}
/*
static int sgm41512_i2c_block_write(struct sgm41512_chip *chip, u8 cmd, u32 len,
				  const u8 *data)
{
	int ret = 0;

	mutex_lock(&chip->io_lock);
	ret = __sgm41512_i2c_block_write(chip, cmd, len, data);
	mutex_unlock(&chip->io_lock);

	return ret;
}
*/
static int sgm41512_i2c_test_bit(struct sgm41512_chip *chip, u8 cmd, u8 shift,
			       bool *is_one)
{
	int ret = 0;
	u8 regval = 0;

	ret = sgm41512_i2c_read_byte(chip, cmd, &regval);
	if (ret < 0) {
		*is_one = false;
		return ret;
	}

	regval &= 1 << shift;
	*is_one = (regval ? true : false);

	return ret;
}

static int sgm41512_i2c_update_bits(struct sgm41512_chip *chip, u8 cmd, u8 data,
				  u8 mask)
{
	int ret = 0;
	u8 regval = 0;

	mutex_lock(&chip->io_lock);
	ret = __sgm41512_i2c_read_byte(chip, cmd, &regval);
	if (ret < 0)
		goto out;

	regval &= ~mask;
	regval |= (data & mask);

	ret = __sgm41512_i2c_write_byte(chip, cmd, regval);
out:
	mutex_unlock(&chip->io_lock);
	return ret;
}

static inline int sgm41512_set_bit(struct sgm41512_chip *chip, u8 cmd, u8 mask)
{
	return sgm41512_i2c_update_bits(chip, cmd, mask, mask);
}

static inline int sgm41512_clr_bit(struct sgm41512_chip *chip, u8 cmd, u8 mask)
{
	return sgm41512_i2c_update_bits(chip, cmd, 0x00, mask);
}

static inline u8 sgm41512_closest_reg(u32 min, u32 max, u32 step, u32 target)
{
	if (target < min)
		return 0;

	if (target >= max)
		target = max;

	return (target - min) / step;
}

static inline u8 sgm41512_closest_reg_via_tbl(const u32 *tbl, u32 tbl_size,
					    u32 target)
{
	u32 i = 0;

	if (target < tbl[0])
		return 0;

	for (i = 0; i < tbl_size - 1; i++) {
		if (target >= tbl[i] && target < tbl[i + 1])
			return i;
	}

	return tbl_size - 1;
}

static inline u32 sgm41512_closest_value(u32 min, u32 max, u32 step, u8 regval)
{
	u32 val = 0;

	val = min + regval * step;
	if (val > max)
		val = max;

	return val;
}

static int __sgm41512_get_vbus_stat(struct sgm41512_chip *chip,
				u8 *stat)
{
	int ret = 0;
	u8 regval = 0;

	ret = sgm41512_i2c_read_byte(chip, SGM41512_REG_ICSTAT, &regval);
	if (ret < 0)
		return ret;
	*stat = (regval & SGM41512_VBUSSTAT_MASK) >> SGM41512_VBUSSTAT_SHIFT;

	return ret;
}

static int __sgm41512_get_chrg_stat(struct sgm41512_chip *chip,
				u8 *stat)
{
	int ret = 0;
	u8 regval = 0;

	ret = sgm41512_i2c_read_byte(chip, SGM41512_REG_ICSTAT, &regval);
	if (ret < 0)
		return ret;
	*stat = (regval & SGM41512_CHRGSTAT_MASK) >> SGM41512_CHRGSTAT_SHIFT;

	return ret;
}

static int __sgm41512_is_fault(struct sgm41512_chip *chip, bool *normal)
{
	int ret = 0;
	u8 regval = 0;

	ret = sgm41512_i2c_read_byte(chip, SGM41512_REG_FAULT, &regval);
	if (ret < 0)
		return ret;
	*normal = (regval == 0);

	return ret;
}

static int __sgm41512_get_ic_stat(struct sgm41512_chip *chip,
				enum sgm41512_ic_stat *stat)
{
	int ret = 0;
	u8 regval = 0;

	ret = sgm41512_i2c_read_byte(chip, SGM41512_REG_ICSTAT, &regval);
	if (ret < 0)
		return ret;
	*stat = (regval & SGM41512_ICSTAT_MASK) >> SGM41512_ICSTAT_SHIFT;

	return ret;
}

static int __sgm41512_get_mivr(struct sgm41512_chip *chip, u32 *mivr)
{
	int ret = 0;
	u8 regval = 0;

	ret = sgm41512_i2c_read_byte(chip, SGM41512_REG_MIVR, &regval);
	if (ret < 0)
		return ret;

	regval = (regval & SGM41512_MIVR_MASK) >> SGM41512_MIVR_SHIFT;
	*mivr = sgm41512_closest_value(SGM41512_MIVR_MIN, SGM41512_MIVR_MAX,
				     SGM41512_MIVR_STEP, regval);

	return ret;
}

static int __sgm41512_get_aicr(struct sgm41512_chip *chip, u32 *aicr)
{
	int ret = 0;
	u8 regval = 0;

	ret = sgm41512_i2c_read_byte(chip, SGM41512_REG_AICR, &regval);
	if (ret < 0)
		return ret;

	regval = (regval & SGM41512_AICR_MASK) >> SGM41512_AICR_SHIFT;
	*aicr = sgm41512_closest_value(SGM41512_AICR_MIN, SGM41512_AICR_MAX,
				     SGM41512_AICR_STEP, regval);
	if (*aicr > SGM41512_AICR_MIN && *aicr < SGM41512_AICR_MAX)
		*aicr -= SGM41512_AICR_STEP;

	return ret;
}

static int __sgm41512_get_cv(struct sgm41512_chip *chip, u32 *cv)
{
	int ret = 0;
	u8 regval = 0;

	ret = sgm41512_i2c_read_byte(chip, SGM41512_REG_CV, &regval);
	if (ret < 0)
		return ret;

	regval = (regval & SGM41512_CV_MASK) >> SGM41512_CV_SHIFT;
	*cv = sgm41512_closest_value(SGM41512_CV_MIN, SGM41512_CV_MAX, SGM41512_CV_STEP,
				   regval);

	return ret;
}

static int __sgm41512_get_ichg(struct sgm41512_chip *chip, u32 *ichg)
{
	int ret = 0;
	u8 regval = 0;

	ret = sgm41512_i2c_read_byte(chip, SGM41512_REG_ICHG, &regval);
	if (ret < 0)
		return ret;

	regval = (regval & SGM41512_ICHG_MASK) >> SGM41512_ICHG_SHIFT;
	*ichg = sgm41512_closest_value(SGM41512_ICHG_MIN, SGM41512_ICHG_MAX,
				     SGM41512_ICHG_STEP, regval);

	return ret;
}

static int __sgm41512_get_ieoc(struct sgm41512_chip *chip, u32 *ieoc)
{
	int ret = 0;
	u8 regval = 0;

	ret = sgm41512_i2c_read_byte(chip, SGM41512_REG_IEOC, &regval);
	if (ret < 0)
		return ret;

	regval = (regval & SGM41512_IEOC_MASK) >> SGM41512_IEOC_SHIFT;
	*ieoc = sgm41512_closest_value(SGM41512_IEOC_MIN, SGM41512_IEOC_MAX,
				     SGM41512_IEOC_STEP, regval);

	return ret;
}

static int __sgm41512_is_hz_enabled(struct sgm41512_chip *chip, bool *en)
{
	return sgm41512_i2c_test_bit(chip, SGM41512_REG_HZ,
				   SGM41512_FORCE_HZ_SHIFT, en);
}

static int __sgm41512_is_chg_enabled(struct sgm41512_chip *chip, bool *en)
{
	return sgm41512_i2c_test_bit(chip, SGM41512_REG_CHG_EN,
				   SGM41512_CHG_EN_SHIFT, en);
}

static int __sgm41512_enable_shipmode(struct sgm41512_chip *chip, bool en)
{
	dev_info(chip->dev, "%s en = %d\n", __func__, en);
	return (en ? sgm41512_set_bit : sgm41512_clr_bit)
		(chip, SGM41512_REG_BATFETDIS, SGM41512_BATFETDIS_MASK);
}

static int __sgm41512_enable_safe_tmr(struct sgm41512_chip *chip, bool en)
{
	dev_info(chip->dev, "%s en = %d\n", __func__, en);
	return (en ? sgm41512_set_bit : sgm41512_clr_bit)
		(chip, SGM41512_REG_SAFETMR_EN, SGM41512_SAFETMR_EN_MASK);
}

static int __sgm41512_enable_te(struct sgm41512_chip *chip, bool en)
{
	dev_info(chip->dev, "%s en = %d\n", __func__, en);
	return (en ? sgm41512_set_bit : sgm41512_clr_bit)
		(chip, SGM41512_REG_TE, SGM41512_TE_MASK);
}

static int __sgm41512_enable_hz(struct sgm41512_chip *chip, bool en)
{
	int ret = 0;

	dev_info(chip->dev, "%s en = %d\n", __func__, en);

	if (ret < 0)
		return ret;

	/* Use force HZ */
	ret = (en ? sgm41512_set_bit : sgm41512_clr_bit)
		(chip, SGM41512_REG_HZ, SGM41512_FORCE_HZ_MASK);

	return ret;
}

static int __sgm41512_enable_otg(struct sgm41512_chip *chip, bool en)
{
	dev_info(chip->dev, "%s en = %d\n", __func__, en);
	return (en ? sgm41512_set_bit : sgm41512_clr_bit)
		(chip, SGM41512_REG_OTG_EN, SGM41512_OTG_EN_MASK);
}

static int __sgm41512_set_otgcc(struct sgm41512_chip *chip, u32 cc)
{
	dev_info(chip->dev, "%s cc = %d\n", __func__, cc);
	return (cc <= sgm41512_otgcc[0] ? sgm41512_clr_bit : sgm41512_set_bit)
		(chip, SGM41512_REG_OTGCC, SGM41512_OTGCC_MASK);
}

static int __sgm41512_enable_chg(struct sgm41512_chip *chip, bool en)
{
	int ret = 0;
	struct sgm41512_desc *desc = chip->desc;

	dev_info(chip->dev, "%s en = %d, chip_rev = %d\n",
			    __func__, en, chip->chip_rev);

	if (chip->ceb_gpio != U32_MAX)
		gpio_set_value(chip->ceb_gpio, desc->ceb_invert ? en : !en);

	ret = (en ? sgm41512_set_bit : sgm41512_clr_bit)
		(chip, SGM41512_REG_CHG_EN, SGM41512_CHG_EN_MASK);
	if (ret >= 0 )//&& chip->chip_rev <= 4)
		mod_delayed_work(system_wq, &chip->buck_dwork,
				 msecs_to_jiffies(100));

	return ret;
}

static int __sgm41512_set_vac_ovp(struct sgm41512_chip *chip, u32 vac_ovp)
{
	u8 regval = 0;

	regval = sgm41512_closest_reg_via_tbl(sgm41512_vac_ovp,
					    ARRAY_SIZE(sgm41512_vac_ovp),
					    vac_ovp);

	dev_info(chip->dev, "%s vac_ovp = %d(0x%02X)\n",
			    __func__, vac_ovp, regval);

	return sgm41512_i2c_update_bits(chip, SGM41512_REG_VAC_OVP,
				      regval << SGM41512_VAC_OVP_SHIFT,
				      SGM41512_VAC_OVP_MASK);
}

static int __sgm41512_set_mivr(struct sgm41512_chip *chip, u32 mivr)
{
	u8 regval = 0;

	regval = sgm41512_closest_reg(SGM41512_MIVR_MIN, SGM41512_MIVR_MAX,
				    SGM41512_MIVR_STEP, mivr);

	dev_info(chip->dev, "%s mivr = %d(0x%02X)\n", __func__, mivr, regval);

	return sgm41512_i2c_update_bits(chip, SGM41512_REG_MIVR,
				      regval << SGM41512_MIVR_SHIFT,
				      SGM41512_MIVR_MASK);
}

static int __sgm41512_set_aicr(struct sgm41512_chip *chip, u32 aicr)
{
	u8 regval = 0;

	regval = sgm41512_closest_reg(SGM41512_AICR_MIN, SGM41512_AICR_MAX,
				    SGM41512_AICR_STEP, aicr);
	/* 0 & 1 are both 50mA */
	if (aicr < SGM41512_AICR_MAX)
		regval += 1;

	dev_info(chip->dev, "%s aicr = %d(0x%02X)\n", __func__, aicr, regval);

	return sgm41512_i2c_update_bits(chip, SGM41512_REG_AICR,
				      regval << SGM41512_AICR_SHIFT,
				      SGM41512_AICR_MASK);
}

static int __sgm41512_set_cv(struct sgm41512_chip *chip, u32 cv)
{
	u8 regval = 0;

	regval = sgm41512_closest_reg(SGM41512_CV_MIN, SGM41512_CV_MAX,
				    SGM41512_CV_STEP, cv);

	dev_info(chip->dev, "%s cv = %d(0x%02X)\n", __func__, cv, regval);

	return sgm41512_i2c_update_bits(chip, SGM41512_REG_CV,
				      regval << SGM41512_CV_SHIFT,
				      SGM41512_CV_MASK);
}

static int __sgm41512_set_ichg(struct sgm41512_chip *chip, u32 ichg)
{
	u8 regval = 0;

	regval = sgm41512_closest_reg(SGM41512_ICHG_MIN, SGM41512_ICHG_MAX,
				    SGM41512_ICHG_STEP, ichg);

	dev_info(chip->dev, "%s ichg = %d(0x%02X)\n", __func__, ichg, regval);

	return sgm41512_i2c_update_bits(chip, SGM41512_REG_ICHG, \
				      regval << SGM41512_ICHG_SHIFT, \
				      SGM41512_ICHG_MASK);
}

static int __sgm41512_set_ieoc(struct sgm41512_chip *chip, u32 ieoc)
{
	u8 regval = 0;

	regval = sgm41512_closest_reg(SGM41512_IEOC_MIN, SGM41512_IEOC_MAX,
				    SGM41512_IEOC_STEP, ieoc);

	dev_info(chip->dev, "%s ieoc = %d(0x%02X)\n", __func__, ieoc, regval);

	return sgm41512_i2c_update_bits(chip, SGM41512_REG_IEOC,
				      regval << SGM41512_IEOC_SHIFT,
				      SGM41512_IEOC_MASK);
}

static int __sgm41512_set_safe_tmr(struct sgm41512_chip *chip, u32 hr)
{
	u8 regval = 0;

	regval = sgm41512_closest_reg(SGM41512_SAFETMR_MIN, SGM41512_SAFETMR_MAX,
				    SGM41512_SAFETMR_STEP, hr);

	dev_info(chip->dev, "%s time = %d(0x%02X)\n", __func__, hr, regval);

	return sgm41512_i2c_update_bits(chip, SGM41512_REG_SAFETMR,
				      regval << SGM41512_SAFETMR_SHIFT,
				      SGM41512_SAFETMR_MASK);
}

static int __sgm41512_set_wdt(struct sgm41512_chip *chip, u32 sec)
{
	u8 regval = 0;

	/* 40s is the minimum, set to 40 except sec == 0 */
	if (sec <= 40 && sec > 0)
		sec = 40;
	regval = sgm41512_closest_reg_via_tbl(sgm41512_wdt, ARRAY_SIZE(sgm41512_wdt),
					    sec);

	dev_info(chip->dev, "%s time = %d(0x%02X)\n", __func__, sec, regval);

	return sgm41512_i2c_update_bits(chip, SGM41512_REG_WDT,
				      regval << SGM41512_WDT_SHIFT,
				      SGM41512_WDT_MASK);
}

static int __sgm41512_set_mivrtrack(struct sgm41512_chip *chip, u32 mivr_track)
{
	if (mivr_track >= SGM41512_MIVRTRACK_MAX)
		mivr_track = SGM41512_MIVRTRACK_VBAT_300MV;

	dev_info(chip->dev, "%s mivrtrack = %d\n", __func__, mivr_track);

	return sgm41512_i2c_update_bits(chip, SGM41512_REG_MIVRTRACK,
				      mivr_track << SGM41512_MIVRTRACK_SHIFT,
				      SGM41512_MIVRTRACK_MASK);
}

static int __sgm41512_kick_wdt(struct sgm41512_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return sgm41512_set_bit(chip, SGM41512_REG_WDTCNTRST, SGM41512_WDTCNTRST_MASK);
}
/*
static int __sgm41512_set_jeita_cool_iset(struct sgm41512_chip *chip, bool en)
{
	dev_info(chip->dev, "%s en = %d\n", __func__, en);
	return (en ? sgm41512_set_bit : sgm41512_clr_bit)
		(chip, SGM41512_REG_JEITA_COOL_ISET, SGM41512_JEITA_COOL_ISET_MASK);
}

static int __sgm41512_set_jeita_cool_vset(struct sgm41512_chip *chip, bool en)
{
	dev_info(chip->dev, "%s en = %d\n", __func__, en);
	return (en ? sgm41512_set_bit : sgm41512_clr_bit)
		(chip, SGM41512_REG_JEITA_COOL_VSET, SGM41512_JEITA_COOL_VSET_MASK);
}
*/
static int __sgm41512_dump_registers(struct sgm41512_chip *chip)
{
	int ret = 0, i = 0;
	u32 mivr = 0, aicr = 0, cv = 0, ichg = 0, ieoc = 0;
	u8 chrg_stat = 0, vbus_stat = 0;
	bool chg_en = 0;
	u8 regval = 0;
	bool ic_normal = false;
	enum sgm41512_ic_stat ic_stat = SGM41512_ICSTAT_SLEEP;
	
	ret = __sgm41512_kick_wdt(chip);

	ret = __sgm41512_get_mivr(chip, &mivr);
	ret = __sgm41512_get_aicr(chip, &aicr);
	ret = __sgm41512_get_cv(chip, &cv);
	ret = __sgm41512_get_ichg(chip, &ichg);
	ret = __sgm41512_get_ieoc(chip, &ieoc);
	ret = __sgm41512_is_chg_enabled(chip, &chg_en);

	ret = __sgm41512_get_chrg_stat(chip, &chrg_stat);
	ret = __sgm41512_get_vbus_stat(chip, &vbus_stat);
	ret = __sgm41512_get_ic_stat(chip, &ic_stat);

	ret = __sgm41512_is_fault(chip, &ic_normal);
	if (ret < 0)
		dev_notice(chip->dev, "%s check charger fault fail(%d)\n",
					      __func__, ret);

	if (1){// ic_normal == false) {
		for (i = 0; i < ARRAY_SIZE(sgm41512_reg_addr); i++) {
			ret = sgm41512_i2c_read_byte(chip, sgm41512_reg_addr[i],
						   &regval);
			if (ret < 0)
				continue;
			dev_notice(chip->dev, "%s reg0x%02X = 0x%02X\n",
					      __func__, sgm41512_reg_addr[i],
					      regval);
		}
	}

	dev_info(chip->dev, "%s MIVR = %dmV, AICR = %dmA\n",
		 __func__, mivr / 1000, aicr / 1000);

	dev_info(chip->dev, "%s chg_en = %d, chrg_stat = %d, vbus_stat = %d ic_stat = %d\n",
		 __func__, chg_en, chrg_stat, vbus_stat,ic_stat);
		 
	dev_info(chip->dev, "%s CV = %dmV, ICHG = %dmA, IEOC = %dmA\n",
		 __func__, cv / 1000, ichg / 1000, ieoc / 1000);

	return 0;
}

static void sgm41512_buck_dwork_handler(struct work_struct *work)
{
	int ret = 0;//, i = 0;
	struct sgm41512_chip *chip =
		container_of(work, struct sgm41512_chip, buck_dwork.work);
	bool chg_rdy = false, chg_done = false, sys_min = false;
	u8 regval = 0;

	dev_info(chip->dev, "%s\n", __func__);

	__pm_stay_awake(&chip->buck_dwork_ws);
	ret = sgm41512_i2c_read_byte(chip, SGM41512_REG_CHGDONE, &regval);
	if (ret < 0)
		goto out;

	if ((regval & SGM41512_ST_CHGDONE_MASK) == (SGM41512_ICSTAT_CHGDONE << SGM41512_ST_CHGDONE_SHIFT))
		chg_done = true;
	else
		chg_done = false;

	ret = sgm41512_i2c_read_byte(chip, SGM41512_REG_CHGRDY, &regval);
	if (ret < 0)
		goto out;

	if (regval & SGM41512_ST_CHGRDY_MASK)
		chg_rdy = true;
	else
		chg_rdy = false;

	dev_info(chip->dev, "%s chg_rdy = %d\n", __func__, chg_rdy);
	dev_info(chip->dev, "%s chg_done = %d, chg_done_once = %d\n",
			    __func__, chg_done, chip->chg_done_once);
	if (!chg_rdy)
		goto out;

	ret = sgm41512_i2c_test_bit(chip, SGM41512_REG_SYSMIN,
				  SGM41512_ST_SYSMIN_SHIFT, &sys_min);
	if (ret < 0)
		goto out;
	dev_info(chip->dev, "%s sys_min = %d\n", __func__, sys_min);

out:
	__pm_relax(&chip->buck_dwork_ws);
}

#ifdef CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT
#ifdef CONFIG_TCPC_CLASS
static bool sgm41512_is_vbusgd(struct sgm41512_chip *chip)
{
	int ret = 0;
	bool vbus_gd = false;

	ret = sgm41512_i2c_test_bit(chip, SGM41512_REG_VBUSGD,
				  SGM41512_ST_VBUSGD_SHIFT, &vbus_gd);
	if (ret < 0)
		dev_notice(chip->dev, "%s check stat fail(%d)\n",
				      __func__, ret);
	dev_dbg(chip->dev, "%s vbus_gd = %d\n", __func__, vbus_gd);

	return vbus_gd;
}
/*
static bool sgm41512_is_chgrdy(struct sgm41512_chip *chip)
{
	int ret = 0;
	bool chg_rdy = false;

	ret = sgm41512_i2c_test_bit(chip, SGM41512_REG_CHGRDY,
				  SGM41512_ST_CHGRDY_SHIFT, &chg_rdy);
	if (ret < 0)
		dev_notice(chip->dev, "%s check stat fail(%d)\n",
				      __func__, ret);
	dev_dbg(chip->dev, "%s chg_rdy = %d\n", __func__, chg_rdy);

	return chg_rdy;
}

static bool sgm41512_is_chgdone(struct sgm41512_chip *chip)
{
	int ret = 0;
	bool chg_done = false;

	//sgm41512_is_charging_done(struct charger_device *chg_dev, bool *done)

	ret = sgm41512_i2c_test_bit(chip, SGM41512_REG_CHGRDY,
				  SGM41512_ST_CHGRDY_SHIFT, &chg_done);
	if (ret < 0)
		dev_notice(chip->dev, "%s check stat fail(%d)\n",
				      __func__, ret);
	dev_dbg(chip->dev, "%s chg_rdy = %d\n", __func__, chg_done);

	return chg_done;
}
*/
#endif /* CONFIG_TCPC_CLASS */

static void sgm41512_enable_bc12(struct sgm41512_chip *chip, bool en)
{
	dev_info(chip->dev, "%s en = %d\n", __func__, en);

	mutex_lock(&chip->bc12_en_lock);
	chip->bc12_en_buf[chip->bc12_en_buf_idx] = en;
	//chip->bc12_en_buf_idx = 1 - chip->bc12_en_buf_idx;
	atomic_inc(&chip->bc12_en_req_cnt);
	wake_up(&chip->bc12_en_req);
	mutex_unlock(&chip->bc12_en_lock);
}

static void sgm41512_force_iildet_en(struct sgm41512_chip *chip,int en)
{
	u8 val = 0;
	int ret = 0;
	
	if(chip == NULL)
		return;
	
	pr_err("[%s]-----------en = %d\n",__func__,en);
	
	ret = sgm41512_i2c_read_byte(chip, SGM41512_REG_IINDET_EN, &val);
	if (ret < 0)
	{
		dev_notice(chip->dev, "%s read err(%d)\n", __func__, ret);
		return;
	}
	
	if(en)
	{
		val |= SGM41512_REG_IINDET_EN_MASK;
	}
	else
	{
		val &= ~SGM41512_REG_IINDET_EN_MASK;
	}
	
	__sgm41512_i2c_write_byte(chip,SGM41512_REG_IINDET_EN,val);
	
	//msleep(500);
}

static void sgm41512_set_usbsw_state(struct sgm41512_chip *chip, int state)
{
	dev_info(chip->dev, "%s state = %d\n", __func__, state);

	if (state == SGM41512_USBSW_CHG)
		Charger_Detect_Init();
	else
		Charger_Detect_Release();
}

static bool sgm41512_chg_rdy(struct sgm41512_chip *chip)
{
	int ret = 0;
	bool chg_rdy = false;
	u8 regval = 0;
	
	dev_info(chip->dev, "%s\n", __func__);
	
	ret = sgm41512_i2c_read_byte(chip, SGM41512_ST_STATUS, &regval);
	if (ret < 0)
		return -SGM41512_CHR_DET_IIC_ERROR;

	chg_rdy = (regval & SGM41512_PGSTAT_MASK ? true : false);
	
	return chg_rdy;
}
static int sgm41512_charge_type_detect_second(struct sgm41512_chip *chip)
{
#ifdef CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT
	int ret = 0;
	int retry_cnt = 0;
	u8 regval = 0;
	bool chg_rdy = false;
	bool attach = false, inform_psy = true;
	u8 port = SGM41512_PORTSTAT_NOINPUT;
	unsigned int boot_mode = get_boot_mode();
	
/*prize add by zhaopengge 20210623 start*/
	if ((chip->dev_id != SGM41512_DEVID) && (chip->dev_id != ETA6963_DEVID) && (chip->dev_id != ETA6953_DEVID)){
		return -SGM41512_CHR_DET_IIC_ERROR;
	}
/*prize add by zhaopengge 20210623 end*/
	dev_info(chip->dev, "%s\n", __func__);
	
	ret = sgm41512_i2c_read_byte(chip, SGM41512_ST_STATUS, &regval);
	if (ret < 0)
		return -SGM41512_CHR_DET_IIC_ERROR;

	chg_rdy = (regval & SGM41512_PGSTAT_MASK ? true : false);
	
	attach = atomic_read(&chip->vbus_gd);
	
//	if((!chg_rdy) && attach)
//	{
//	dev_info(chip->dev, "%s  chg_rdy = %d,attach = %d\n", __func__,chg_rdy,attach);
//		msleep(100);
//		return -SGM41512_CHR_DET_VBUS_NOT_READY;
//	}
	
	dev_info(chip->dev, "%s chg_rdy = %d, attach = %d\n",__func__, chg_rdy, attach);

	//dev_info(chip->dev, "%s chip->attach (%d) \n",__func__,chip->attach);

#if 1
	if (chip->attach == attach) 
	{
		dev_info(chip->dev, "%s attach(%d) is the same\n", __func__, attach);
		inform_psy = !attach;
		goto out;
	}
#endif

	chip->attach = attach;
	
	dev_info(chip->dev, "%s attach = %d,chip->attach = %d\n", __func__, attach,chip->attach);

	if (!attach) {
		chip->port = SGM41512_PORTSTAT_NOINPUT;
		chip->chg_type = CHARGER_UNKNOWN;
		chip->bc12_enable_done = false;
		goto out;
	}
retry:
	ret = sgm41512_i2c_read_byte(chip, SGM41512_REG_ICSTAT, &port);
		
	dev_info(chip->dev, "%s chip->port = %x\n", __func__,port);
		
	if (ret < 0)
		chip->port = SGM41512_PORTSTAT_NOINPUT;
	else
		chip->port = (port & SGM41512_PORTSTAT_MASK) >> SGM41512_PORTSTAT_SHIFT;
		
	dev_info(chip->dev, "%s chip->port = %x\n", __func__,chip->port);

	switch (chip->port) {
	case SGM41512_PORTSTAT_NOINPUT:
		chip->chg_type = CHARGER_UNKNOWN;
		break;
	case SGM41512_PORTSTAT_SDP:
		chip->chg_type = STANDARD_HOST;
		break;
	case SGM41512_PORTSTAT_CDP:
		chip->chg_type = CHARGING_HOST;
		break;
	case SGM41512_PORTSTAT_DCP:
		chip->chg_type = STANDARD_CHARGER;
		break;
	case SGM41512_PORTSTAT_UKNADPT:
	case SGM41512_PORTSTAT_NSADPT:
	default:
		chip->chg_type = NONSTANDARD_CHARGER;
		break;
	}
	
	if(chip->chg_type == CHARGER_UNKNOWN && attach)
	{
		//sgm41512_enable_bc12(chip,true);
		//return 0;
		dev_info(chip->dev, "%s sgm41512_chg_type detect err %d\n", __func__,retry_cnt);
		sgm41512_force_iildet_en(chip,true);
		msleep(200);
		retry_cnt++;
		
		if(retry_cnt < 20)
		{
			goto retry;
		}
		else
		{
			chip->port = SGM41512_PORTSTAT_NSADPT;
			chip->chg_type = NONSTANDARD_CHARGER;
		}
	}

out:
	if(chip->chg_type != STANDARD_CHARGER)
	{
		sgm41512_enable_bc12(chip,false);
		if((boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT) ||(boot_mode == LOW_POWER_OFF_CHARGING_BOOT))
		{
			dev_info(chip->dev, " boot mode -- power off charger --,%s sgm41512_enable_bc12 = false\n", __func__);
		}
		dev_info(chip->dev, "%s sgm41512_enable_bc12 = false\n", __func__);
	}
	if (inform_psy)
	{
		dev_info(chip->dev, "%s sgm41512 start schedule_delayed_work\n", __func__);
		schedule_delayed_work(&chip->psy_dwork, 0);
	}
	
	return SGM41512_CHR_DET_OK;
	
#endif /* CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT */
}

static int sgm41512_bc12_en_kthread(void *data)
{
	int ret = 0,i = 0, en = 0;
	struct sgm41512_chip *chip = data;
	const int max_wait_cnt = 200;
	//const int max_retry_cnt = 10;
	dev_info(chip->dev, "%s\n", __func__);
wait:
	wait_event(chip->bc12_en_req, atomic_read(&chip->bc12_en_req_cnt) > 0 ||
				      kthread_should_stop());
	if (atomic_read(&chip->bc12_en_req_cnt) <= 0 &&
	    kthread_should_stop()) {
		dev_info(chip->dev, "%s bye bye\n", __func__);
		return 0;
	}
	atomic_dec(&chip->bc12_en_req_cnt);
/*
	mutex_lock(&chip->bc12_en_lock);
	dev_info(chip->dev, "%s chip->bc12_en_buf_idx = %d\n", __func__, chip->bc12_en_buf_idx);
	chip->bc12_en_buf[chip->bc12_en_buf_idx] = -1;
	if (en == -1) {
		chip->bc12_en_buf_idx = 1 - chip->bc12_en_buf_idx;
		en = chip->bc12_en_buf[chip->bc12_en_buf_idx];
		chip->bc12_en_buf[chip->bc12_en_buf_idx] = -1;
	}
	mutex_unlock(&chip->bc12_en_lock);
*/
	en = chip->bc12_en_buf[chip->bc12_en_buf_idx];
	
	dev_info(chip->dev, "%s en = %d\n", __func__, en);
	//if (en == -1)
	//	goto wait;



	__pm_stay_awake(&chip->bc12_en_ws);
	
	sgm41512_set_usbsw_state(chip, en ? SGM41512_USBSW_CHG : SGM41512_USBSW_USB);

	if (en) {
		
		/* Workaround for CDP port */
		sgm41512_force_iildet_en(chip,true);
		
		msleep(200);
		
		for (i = 0; i < max_wait_cnt; i++) 
		{
			if (sgm41512_chg_rdy(chip))
			{
				break;
			}
			else
			{
				dev_info(chip->dev, "%s sgm41512 chg is not rdy,wait...%d\n", __func__,max_wait_cnt);
				msleep(200);
			}
		}
		if (i == max_wait_cnt)
			dev_notice(chip->dev, "%s CDP timeout\n", __func__);
		else
			dev_info(chip->dev, "%s CDP free\n", __func__);
		
		//chip->bc12_enable_done = true;
		
		ret = sgm41512_charge_type_detect_second(chip);
		
		if(ret != SGM41512_CHR_DET_OK)
		{
			dev_info(chip->dev, "%s sgm41512 chg det err ret = %d\n", __func__,ret);
		}
	}
	
	__pm_relax(&chip->bc12_en_ws);
	goto wait;

	return 0;
}



static int sgm41512_charge_type_detect_first(struct sgm41512_chip *chip)
{
/*
	bool vbus_gd = false;
	enum charger_type chg_type = CHARGER_UNKNOWN;

	mutex_lock(&chip->bc12_lock);
	vbus_gd = atomic_read(&chip->vbus_gd);
	chg_type = chip->chg_type;
	mutex_unlock(&chip->bc12_lock);
*/	
/*prize add by zhaopengge 20210623 start*/
	if((chip->dev_id != SGM41512_DEVID) && (chip->dev_id != ETA6963_DEVID) && (chip->dev_id != ETA6953_DEVID)){
		return -ENOTSUPP;
	}
/*prize add by zhaopengge 20210623 end*/	

	dev_info(chip->dev, "%s\n", __func__);
	
	//if (atomic_read(&chip->vbus_gd)) {
		//sgm41512_enable_bc12(chip, false);
	sgm41512_enable_bc12(chip, true);
	//}

	return 0;
}

static void sgm41512_switch_dpdm_event_detection(struct sgm41512_chip *chip,int en)
{
	u8 val = 0;
	int ret = 0;
	
	if(chip == NULL)
		return;
	
	pr_err("[%s]-----------en = %d\n",__func__,en);
	
	ret = sgm41512_i2c_read_byte(chip, SGM41512_REG_VINDPM, &val);
	if (ret < 0)
	{
		dev_notice(chip->dev, "%s read err(%d)\n", __func__, ret);
		return;
	}
	if(en)
	{
		val &= ~SGM41512_VINDPM_IINDPM_MASK;
	}
	else
	{
		val |= SGM41512_VINDPM_IINDPM_MASK;
	}
	
	__sgm41512_i2c_write_byte(chip,SGM41512_REG_VINDPM,val);
}

static void sgm41512_inform_psy_dwork_handler(struct work_struct *work)
{
	int ret = 0;
	union power_supply_propval propval = {.intval = 0};
	struct sgm41512_chip *chip = container_of(work, struct sgm41512_chip,
						psy_dwork.work);
	bool vbus_gd = false;
	enum charger_type chg_type = CHARGER_UNKNOWN;

	mutex_lock(&chip->bc12_lock);
	vbus_gd = atomic_read(&chip->vbus_gd);
	chg_type = chip->chg_type;
	mutex_unlock(&chip->bc12_lock);

	dev_info(chip->dev, "%s vbus_gd = %d, type = %d\n", __func__,
			    vbus_gd, chg_type);

	/* Get chg type det power supply */
	if (!chip->psy)
		chip->psy = power_supply_get_by_name("charger");
	if (!chip->psy) {
		dev_notice(chip->dev, "%s get power supply fail\n", __func__);
		schedule_delayed_work(&chip->psy_dwork, msecs_to_jiffies(1000));
		return;
	}

	propval.intval = vbus_gd;
	ret = power_supply_set_property(chip->psy, POWER_SUPPLY_PROP_ONLINE,
					&propval);
	if (ret < 0)
		dev_notice(chip->dev, "%s psy online fail(%d)\n",
				      __func__, ret);

	propval.intval = chg_type;
	ret = power_supply_set_property(chip->psy,
					POWER_SUPPLY_PROP_CHARGE_TYPE,
					&propval);
	if (ret < 0)
		dev_notice(chip->dev, "%s psy type fail(%d)\n", __func__, ret);
	
	pr_err("sgm41512 : gezi--NOTICE--- report charge type = %d--\n",chg_type);
	
	if(chg_type != CHARGER_UNKNOWN)
	{
		//sgm41512_switch_dpdm_event_detection(chip,false);
	}
	else
	{
		//sgm41512_switch_dpdm_event_detection(chip,true);
		sgm41512_force_iildet_en(chip,false);
	}
}
/*
static int sgm41512_bc12_postprocess(struct sgm41512_chip *chip)
{
	//int ret = 0;
	bool attach = false; //inform_psy = true;
	//u8 port = SGM41512_PORTSTAT_NOINPUT;

	if (chip->dev_id != SGM41512_DEVID)
		return -ENOTSUPP;
	
	dev_info(chip->dev, "%s %d attach (%d)  chip->attach (%d)\n",
		__func__,__LINE__, attach , chip->attach);
	attach = atomic_read(&chip->vbus_gd);
	dev_info(chip->dev, "%s %d attach (%d)  chip->attach (%d) \n",__func__,__LINE__,attach,chip->attach);
	
	sgm41512_enable_bc12(chip, false);
	
	return 0;
}
*/
#endif /* CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT */

static int sgm41512_detach_irq_handler(struct sgm41512_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
#ifdef CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT
#ifndef CONFIG_TCPC_CLASS
	//pr_err("gezi--------------------\n");
	mutex_lock(&chip->bc12_lock);
	atomic_set(&chip->vbus_gd, sgm41512_is_vbusgd(chip));
	//sgm41512_bc12_postprocess(chip);
	mutex_unlock(&chip->bc12_lock);
#endif /* CONFIG_TCPC_CLASS */
#endif /* CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT */
	return 0;
}

static int sgm41512_rechg_irq_handler(struct sgm41512_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	charger_dev_notify(chip->chg_dev, CHARGER_DEV_NOTIFY_RECHG);
	return 0;
}

static int sgm41512_chg_done_irq_handler(struct sgm41512_chip *chip)
{
	dev_info(chip->dev, "%s chip_rev = %d\n", __func__, chip->chip_rev);

	//if (chip->chip_rev > 4)
		//return 0;
	cancel_delayed_work_sync(&chip->buck_dwork);
	chip->chg_done_once = false;
	mod_delayed_work(system_wq, &chip->buck_dwork, msecs_to_jiffies(100));

	return 0;
}

static int sgm41512_bg_chg_irq_handler(struct sgm41512_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int sgm41512_ieoc_irq_handler(struct sgm41512_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	charger_dev_notify(chip->chg_dev, CHARGER_DEV_NOTIFY_EOC);
	return 0;
}

static int sgm41512_chg_rdy_irq_handler(struct sgm41512_chip *chip)
{
	struct chgdev_notify *noti = &(chip->chg_dev->noti);

	dev_info(chip->dev, "%s chip_rev = %d\n", __func__, chip->chip_rev);
	
	if(chip->chip_rev >= 0)
		return 0;

	noti->vbusov_stat = false;
	charger_dev_notify(chip->chg_dev, CHARGER_DEV_NOTIFY_VBUS_OVP);

	mod_delayed_work(system_wq, &chip->buck_dwork, msecs_to_jiffies(100));

	return 0;
}



static int sgm41512_bc12_done_irq_handler(struct sgm41512_chip *chip)
{/*
	struct chgdev_notify *noti = &(chip->chg_dev->noti);

	dev_info(chip->dev, "%s chip_rev = %d\n", __func__, chip->chip_rev);

	noti->vbusov_stat = false;
	charger_dev_notify(chip->chg_dev, CHARGER_DEV_NOTIFY_VBUS_OVP);

	mod_delayed_work(system_wq, &chip->buck_dwork, msecs_to_jiffies(100));
*/
	dev_info(chip->dev, "%s line = %d\n", __func__,__LINE__);
	if(chip->bc12_enable_done)	
	{
		//sgm41512_charge_type_detect_second(chip);
	}
	else
	{
		dev_info(chip->dev, "%s line = %d,bc12_enable_done = %d\n", __func__,__LINE__,chip->bc12_enable_done);
	}
		
	return 0;
}

static int sgm41512_vbus_gd_irq_handler(struct sgm41512_chip *chip)
{
	
#ifdef CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT
#ifndef CONFIG_TCPC_CLASS
	//mutex_lock(&chip->bc12_lock);
	int vbus_gd = sgm41512_is_vbusgd(chip);
	
	atomic_set(&chip->vbus_gd,vbus_gd);
	dev_info(chip->dev, "%s vbus_gd = %d\n", __func__,vbus_gd);
	//sgm41512_charge_type_detect_first(chip);
	//mutex_unlock(&chip->bc12_lock);
#endif /* CONFIG_TCPC_CLASS */
#endif /* CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT */
	return 0;
}

static int sgm41512_chg_batov_irq_handler(struct sgm41512_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	charger_dev_notify(chip->chg_dev, CHARGER_DEV_NOTIFY_BAT_OVP);
	return 0;
}

static int sgm41512_chg_sysov_irq_handler(struct sgm41512_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int sgm41512_chg_tout_irq_handler(struct sgm41512_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	charger_dev_notify(chip->chg_dev, CHARGER_DEV_NOTIFY_SAFETY_TIMEOUT);
	return 0;
}

static int sgm41512_chg_busuv_irq_handler(struct sgm41512_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int sgm41512_chg_threg_irq_handler(struct sgm41512_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int sgm41512_chg_aicr_irq_handler(struct sgm41512_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int sgm41512_chg_mivr_irq_handler(struct sgm41512_chip *chip)
{
	int ret = 0;
	bool mivr = false;

	ret = sgm41512_i2c_test_bit(chip, SGM41512_REG_ST_MIVR, SGM41512_ST_MIVR_SHIFT,
				  &mivr);
	if (ret < 0)
		return ret;

	dev_info(chip->dev, "%s mivr = %d\n", __func__, mivr);

	return 0;
}

static int sgm41512_sys_short_irq_handler(struct sgm41512_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int sgm41512_sys_min_irq_handler(struct sgm41512_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}
/*
static int sgm41512_aicc_done_irq_handler(struct sgm41512_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	complete(&chip->aicc_done);
	return 0;
}
*/

static int sgm41512_pe_done_irq_handler(struct sgm41512_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	complete(&chip->pe_done);
	return 0;
}

static int sgm41512_jeita_cold_irq_handler(struct sgm41512_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int sgm41512_jeita_cool_irq_handler(struct sgm41512_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int sgm41512_jeita_warm_irq_handler(struct sgm41512_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int sgm41512_jeita_hot_irq_handler(struct sgm41512_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int sgm41512_otg_fault_irq_handler(struct sgm41512_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int sgm41512_otg_lbp_irq_handler(struct sgm41512_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int sgm41512_otg_cc_irq_handler(struct sgm41512_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int sgm41512_wdt_irq_handler(struct sgm41512_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return __sgm41512_kick_wdt(chip);
}

static int sgm41512_vac_ov_irq_handler(struct sgm41512_chip *chip)
{
	int ret = 0;
	bool vac_ov = false;
	struct chgdev_notify *noti = &(chip->chg_dev->noti);

	ret = sgm41512_i2c_test_bit(chip, SGM41512_REG_VACOV, SGM41512_ST_VACOV_SHIFT,
				  &vac_ov);
	if (ret < 0)
		return ret;

	dev_info(chip->dev, "%s vac_ov = %d\n", __func__, vac_ov);
	noti->vbusov_stat = vac_ov;
	charger_dev_notify(chip->chg_dev, CHARGER_DEV_NOTIFY_VBUS_OVP);

	return 0;
}

static int sgm41512_otp_irq_handler(struct sgm41512_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

struct irq_mapping_tbl {
	const char *name;
	int (*hdlr)(struct sgm41512_chip *chip);
	int num;
};

#define SGM41512_IRQ_MAPPING(_name, _num) \
	{.name = #_name, .hdlr = sgm41512_##_name##_irq_handler, .num = _num}

static const struct irq_mapping_tbl sgm41512_irq_mapping_tbl[] = {
	SGM41512_IRQ_MAPPING(wdt, 29),
	SGM41512_IRQ_MAPPING(vbus_gd, 7),
	SGM41512_IRQ_MAPPING(chg_rdy, 6),
	SGM41512_IRQ_MAPPING(bc12_done, 0),
	SGM41512_IRQ_MAPPING(detach, 1),
	SGM41512_IRQ_MAPPING(rechg, 2),
	SGM41512_IRQ_MAPPING(chg_done, 3),
	SGM41512_IRQ_MAPPING(bg_chg, 4),
	SGM41512_IRQ_MAPPING(ieoc, 5),
	SGM41512_IRQ_MAPPING(chg_batov, 9),
	SGM41512_IRQ_MAPPING(chg_sysov, 10),
	SGM41512_IRQ_MAPPING(chg_tout, 11),
	SGM41512_IRQ_MAPPING(chg_busuv, 12),
	SGM41512_IRQ_MAPPING(chg_threg, 13),
	SGM41512_IRQ_MAPPING(chg_aicr, 14),
	SGM41512_IRQ_MAPPING(chg_mivr, 15),
	SGM41512_IRQ_MAPPING(sys_short, 16),
	SGM41512_IRQ_MAPPING(sys_min, 17),
//	SGM41512_IRQ_MAPPING(aicc_done, 18),
	SGM41512_IRQ_MAPPING(pe_done, 19),
	SGM41512_IRQ_MAPPING(jeita_cold, 20),
	SGM41512_IRQ_MAPPING(jeita_cool, 21),
	SGM41512_IRQ_MAPPING(jeita_warm, 22),
	SGM41512_IRQ_MAPPING(jeita_hot, 23),
	SGM41512_IRQ_MAPPING(otg_fault, 24),
	SGM41512_IRQ_MAPPING(otg_lbp, 25),
	SGM41512_IRQ_MAPPING(otg_cc, 26),
	SGM41512_IRQ_MAPPING(vac_ov, 30),
	SGM41512_IRQ_MAPPING(otp, 31),
};
static irqreturn_t sgm41512_irq_handler(int irq, void *data)
{
	int ret = 0;//, i = 0, irqnum = 0, irqbit = 0;
	u8 evt[SGM41512_STREG_MAX] = {0};
	struct sgm41512_chip *chip = (struct sgm41512_chip *)data;

	dev_info(chip->dev, "%s\n", __func__);

	pm_stay_awake(chip->dev);

	ret = sgm41512_i2c_block_read(chip, SGM41512_REG_STATUS, SGM41512_STREG_MAX,
				    evt);
	if (ret < 0) {
		dev_notice(chip->dev, "%s read evt fail(%d)\n", __func__, ret);
		goto out;
	}
	
	switch ((evt[SGM41512_ST_STATUS] & SGM41512_CHRGSTAT_MASK) >> SGM41512_CHRGSTAT_SHIFT)
	{
		case 3:
			sgm41512_chg_done_irq_handler(chip);

			break;
		default:

			break;
	}

	if ( evt[SGM41512_ST_STATUS] & SGM41512_PGSTAT_MASK )
	{
		dev_info(chip->dev, "%s %d\n", __func__,__LINE__);
		//sgm41512_chg_rdy_irq_handler(chip);
		//sgm41512_bc12_done_irq_handler(chip);
	}

	switch ((evt[SGM41512_ST_FAULT] & SGM41512_CHRGFAULT_MASK) >> SGM41512_CHRGFAULT_SHIFT)
	{
		case 1:
			sgm41512_chg_busuv_irq_handler(chip);

			break;
		case 3:
			sgm41512_chg_tout_irq_handler(chip);

			break;
		default:

			break;
	}

	if ( evt[SGM41512_ST_FAULT] & SGM41512_BATFAULT_MASK )
	{
		sgm41512_chg_batov_irq_handler(chip);
	}
	
#if 0
	if ( evt[SGM41512_ST_STATUS2] & SGM41512_ST_VBUSGD_MASK )
	{
		sgm41512_detach_irq_handler(chip);
	}

	if ( evt[SGM41512_ST_STATUS2] & SGM41512_ST_MIVR_MASK )
	{
		sgm41512_chg_mivr_irq_handler(chip);
	}

	if ( evt[SGM41512_ST_STATUS2] & SGM41512_ST_VACOV_MASK )
	{
		sgm41512_vac_ov_irq_handler(chip);
	}
#endif
out:
	pm_relax(chip->dev);
	return IRQ_HANDLED;
}

static int sgm41512_register_irq(struct sgm41512_chip *chip)
{
	int ret = 0;

	dev_info(chip->dev, "%s\n", __func__);

	ret = devm_gpio_request_one(chip->dev, chip->intr_gpio, GPIOF_DIR_IN,
			devm_kasprintf(chip->dev, GFP_KERNEL,
			"sgm41512_intr_gpio.%s", dev_name(chip->dev)));
	if (ret < 0) {
		dev_notice(chip->dev, "%s gpio request fail(%d)\n",
				      __func__, ret);
		return ret;
	}
	chip->irq = gpio_to_irq(chip->intr_gpio);
	if (chip->irq < 0) {
		dev_notice(chip->dev, "%s gpio2irq fail(%d)\n",
				      __func__, chip->irq);
		return chip->irq;
	}
	dev_info(chip->dev, "%s irq = %d\n", __func__, chip->irq);

	/* Request threaded IRQ */
	ret = devm_request_threaded_irq(chip->dev, chip->irq, NULL,
					sgm41512_irq_handler,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,//IRQF_TRIGGER_FALLING //IRQF_TRIGGER_RISING
					devm_kasprintf(chip->dev, GFP_KERNEL,
					"sgm41512_irq.%s", dev_name(chip->dev)),
					chip);
	if (ret < 0) {
		dev_notice(chip->dev, "%s request threaded irq fail(%d)\n",
				      __func__, ret);
		return ret;
	}
	device_init_wakeup(chip->dev, true);

	return ret;
}

static inline int sgm41512_get_irq_number(struct sgm41512_chip *chip,
					const char *name)
{
	int i = 0;

	if (!name) {
		dev_notice(chip->dev, "%s null name\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(sgm41512_irq_mapping_tbl); i++) {
		if (!strcmp(name, sgm41512_irq_mapping_tbl[i].name))
			return sgm41512_irq_mapping_tbl[i].num;
	}

	return -EINVAL;
}

static inline const char *sgm41512_get_irq_name(int irqnum)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(sgm41512_irq_mapping_tbl); i++) {
		if (sgm41512_irq_mapping_tbl[i].num == irqnum)
			return sgm41512_irq_mapping_tbl[i].name;
	}

	return "not found";
}

static inline void sgm41512_irq_mask(struct sgm41512_chip *chip, int irqnum)
{
	dev_dbg(chip->dev, "%s irq(%d, %s)\n", __func__, irqnum,
		sgm41512_get_irq_name(irqnum));
	chip->irq_mask[irqnum / 8] |= (1 << (irqnum % 8));
}

static inline void sgm41512_irq_unmask(struct sgm41512_chip *chip, int irqnum)
{
	dev_info(chip->dev, "%s irq(%d, %s)\n", __func__, irqnum,
		 sgm41512_get_irq_name(irqnum));
	chip->irq_mask[irqnum / 8] &= ~(1 << (irqnum % 8));
}

static int sgm41512_parse_dt(struct sgm41512_chip *chip)
{
	int ret = 0, irqcnt = 0;//, irqnum = 0;
	struct device_node *parent_np = chip->dev->of_node, *np = NULL;
	struct sgm41512_desc *desc = NULL;
	const char *name = NULL;

	dev_info(chip->dev, "%s\n", __func__);

	chip->desc = &sgm41512_default_desc;

	if (!parent_np) {
		dev_notice(chip->dev, "%s no device node\n", __func__);
		return -EINVAL;
	}
	np = of_get_child_by_name(parent_np, "sgm41512");
	if (!np) {
		dev_info(chip->dev, "%s no sgm41512 device node\n", __func__);
		np = parent_np;
	}

	desc = devm_kzalloc(chip->dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;
	memcpy(desc, &sgm41512_default_desc, sizeof(*desc));

	ret = of_property_read_string(np, "chg_name", &desc->chg_name);
	if (ret < 0)
		dev_info(chip->dev, "%s no chg_name(%d)\n", __func__, ret);

	ret = of_property_read_string(np, "chg_alias_name",
				      &chip->chg_props.alias_name);
	if (ret < 0) {
		dev_info(chip->dev, "%s no chg_alias_name(%d)\n",
				    __func__, ret);
		chip->chg_props.alias_name = "sgm41512_chg";
	}
	dev_info(chip->dev, "%s name = %s, alias name = %s\n", __func__,
			    desc->chg_name, chip->chg_props.alias_name);

	if (strcmp(desc->chg_name, "secondary_chg") == 0)
		chip->enter_shipping_mode = true;

#if !defined(CONFIG_MTK_GPIO) || defined(CONFIG_MTK_GPIOLIB_STAND)
	ret = of_get_named_gpio(parent_np, "rt,intr_gpio", 0);
	if (ret < 0) {
		dev_notice(chip->dev, "%s no rt,intr_gpio(%d)\n",
				      __func__, ret);
		return ret;
	} else
		chip->intr_gpio = ret;

	ret = of_get_named_gpio(parent_np, "rt,ceb_gpio", 0);
	if (ret < 0) {
		dev_info(chip->dev, "%s no rt,ceb_gpio(%d)\n",
				    __func__, ret);
		chip->ceb_gpio = U32_MAX;
	} else
		chip->ceb_gpio = ret;
#else
	ret = of_property_read_u32(parent_np, "rt,intr_gpio_num",
				   &chip->intr_gpio);
	if (ret < 0) {
		dev_notice(chip->dev, "%s no rt,intr_gpio_num(%d)\n",
				      __func__, ret);
		return ret;
	}

	ret = of_property_read_u32(parent_np, "rt,ceb_gpio_num",
				   &chip->ceb_gpio);
	if (ret < 0) {
		dev_info(chip->dev, "%s no rt,ceb_gpio_num(%d)\n",
				    __func__, ret);
		chip->ceb_gpio = U32_MAX;
	}
#endif
	dev_info(chip->dev, "%s intr_gpio = %u, ceb_gpio = %u\n",
			    __func__, chip->intr_gpio, chip->ceb_gpio);

	if (chip->ceb_gpio != U32_MAX) {
		ret = devm_gpio_request_one(
				chip->dev, chip->ceb_gpio, GPIOF_DIR_OUT,
				devm_kasprintf(chip->dev, GFP_KERNEL,
				"sgm41512_ceb_gpio.%s", dev_name(chip->dev)));
		if (ret < 0) {
			dev_notice(chip->dev, "%s gpio request fail(%d)\n",
					      __func__, ret);
			return ret;
		}
	}

	/* Register map */
	ret = of_property_read_u8(np, "rm-slave-addr", &desc->rm_slave_addr);
	if (ret < 0)
		dev_info(chip->dev, "%s no rm-slave-addr(%d)\n", __func__, ret);
	ret = of_property_read_string(np, "rm-name", &desc->rm_name);
	if (ret < 0)
		dev_info(chip->dev, "%s no rm-name(%d)\n", __func__, ret);

	/* Charger parameter */
	ret = of_property_read_u32(np, "vac_ovp", &desc->vac_ovp);
	if (ret < 0)
		dev_info(chip->dev, "%s no vac_ovp(%d)\n", __func__, ret);

	ret = of_property_read_u32(np, "mivr", &desc->mivr);
	if (ret < 0)
		dev_info(chip->dev, "%s no mivr(%d)\n", __func__, ret);

	ret = of_property_read_u32(np, "aicr", &desc->aicr);
	if (ret < 0)
		dev_info(chip->dev, "%s no aicr(%d)\n", __func__, ret);

	ret = of_property_read_u32(np, "cv", &desc->cv);
	if (ret < 0)
		dev_info(chip->dev, "%s no cv(%d)\n", __func__, ret);

	ret = of_property_read_u32(np, "ichg", &desc->ichg);
	if (ret < 0)
		dev_info(chip->dev, "%s no ichg(%d)\n", __func__, ret);

	ret = of_property_read_u32(np, "ieoc", &desc->ieoc) < 0;
	if (ret < 0)
		dev_info(chip->dev, "%s no ieoc(%d)\n", __func__, ret);

	ret = of_property_read_u32(np, "safe_tmr", &desc->safe_tmr);
	if (ret < 0)
		dev_info(chip->dev, "%s no safe_tmr(%d)\n", __func__, ret);

	ret = of_property_read_u32(np, "wdt", &desc->wdt);
	if (ret < 0)
		dev_info(chip->dev, "%s no wdt(%d)\n", __func__, ret);

	ret = of_property_read_u32(np, "mivr_track", &desc->mivr_track);
	if (ret < 0)
		dev_info(chip->dev, "%s no mivr_track(%d)\n", __func__, ret);
	if (desc->mivr_track >= SGM41512_MIVRTRACK_MAX)
		desc->mivr_track = SGM41512_MIVRTRACK_VBAT_300MV;

	desc->en_safe_tmr = of_property_read_bool(np, "en_safe_tmr");
	desc->en_te = of_property_read_bool(np, "en_te");
	desc->en_jeita = of_property_read_bool(np, "en_jeita");
	desc->ceb_invert = of_property_read_bool(np, "ceb_invert");
	desc->dis_i2c_tout = of_property_read_bool(np, "dis_i2c_tout");
	desc->en_qon_rst = of_property_read_bool(np, "en_qon_rst");
	desc->auto_aicr = of_property_read_bool(np, "auto_aicr");

	chip->desc = desc;
	
	desc->en_te = 1;

	memcpy(chip->irq_mask, sgm41512_irq_maskall, SGM41512_STREG_MAX);
	while (true) {
		ret = of_property_read_string_index(np, "interrupt-names",
						    irqcnt, &name);
		if (ret < 0)
			break;
		irqcnt++;
	}

	return 0;
}

static int sgm41512_check_chg(struct sgm41512_chip *chip)
{
	int ret = 0;
	u8 regval = 0;

	dev_info(chip->dev, "%s\n", __func__);

	ret = sgm41512_i2c_read_byte(chip, SGM41512_REG_VBUSGD, &regval);
	if (ret < 0)
		return ret;

	if (regval & SGM41512_ST_VBUSGD_MASK)
		sgm41512_vbus_gd_irq_handler(chip);


	ret = sgm41512_i2c_read_byte(chip, SGM41512_REG_CHGDONE, &regval);
	if (ret < 0)
		return ret;

	if ((regval & SGM41512_ST_CHGDONE_MASK) == (SGM41512_ICSTAT_CHGDONE << SGM41512_ST_CHGDONE_SHIFT))
		sgm41512_chg_done_irq_handler(chip);
	else
	{
		ret = sgm41512_i2c_read_byte(chip, SGM41512_REG_CHGRDY, &regval);
		if (ret < 0)
			return ret;

		if (regval & SGM41512_ST_CHGRDY_MASK)
			sgm41512_chg_rdy_irq_handler(chip);
	}

	return ret;
}


static int sgm41512_init_setting(struct sgm41512_chip *chip)
{
	int ret = 0;
	struct sgm41512_desc *desc = chip->desc;
	u8 evt[SGM41512_STREG_MAX] = {0};
	unsigned int boot_mode = get_boot_mode();

	dev_info(chip->dev, "%s\n", __func__);

	/* Clear all IRQs */
	ret = sgm41512_i2c_block_read(chip, SGM41512_REG_STATUS, SGM41512_STREG_MAX,
				    evt);
	if (ret < 0)
		dev_notice(chip->dev, "%s clear irq fail(%d)\n", __func__, ret);

	ret = __sgm41512_set_vac_ovp(chip, desc->vac_ovp);
	if (ret < 0)
		dev_notice(chip->dev, "%s set vac ovp fail(%d)\n",
				      __func__, ret);

	ret = __sgm41512_set_mivr(chip, desc->mivr);
	if (ret < 0)
		dev_notice(chip->dev, "%s set mivr fail(%d)\n", __func__, ret);

	ret = __sgm41512_set_aicr(chip, desc->aicr);
	if (ret < 0)
		dev_notice(chip->dev, "%s set aicr fail(%d)\n", __func__, ret);

	ret = __sgm41512_set_cv(chip, desc->cv);
	if (ret < 0)
		dev_notice(chip->dev, "%s set cv fail(%d)\n", __func__, ret);

	ret = __sgm41512_set_ichg(chip, desc->ichg);
	if (ret < 0)
		dev_notice(chip->dev, "%s set ichg fail(%d)\n", __func__, ret);

	ret = __sgm41512_set_ieoc(chip, desc->ieoc);
	if (ret < 0)
		dev_notice(chip->dev, "%s set ieoc fail(%d)\n", __func__, ret);

	ret = __sgm41512_set_safe_tmr(chip, desc->safe_tmr);
	if (ret < 0)
		dev_notice(chip->dev, "%s set safe tmr fail(%d)\n",
				      __func__, ret);

	ret = __sgm41512_set_mivrtrack(chip, desc->mivr_track);
	if (ret < 0)
		dev_notice(chip->dev, "%s set mivrtrack fail(%d)\n",
				      __func__, ret);

	ret = __sgm41512_enable_safe_tmr(chip, desc->en_safe_tmr);
	if (ret < 0)
		dev_notice(chip->dev, "%s en safe tmr fail(%d)\n",
				      __func__, ret);

	ret = __sgm41512_enable_te(chip, desc->en_te);
	if (ret < 0)
		dev_notice(chip->dev, "%s en te fail(%d)\n", __func__, ret);
	
	sgm41512_switch_dpdm_event_detection(chip,false);

	if ((boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT ||
	     boot_mode == LOW_POWER_OFF_CHARGING_BOOT) &&
	     strcmp(desc->chg_name, "primary_chg") == 0) {
		dev_info(chip->dev,
		"%s do not HZ=1 and CHG_EN=0 for primary_chg when KPOC\n",
		__func__);
		return 0;
	}

	/*
	 * Customization for MTK platform
	 * Primary charger: CHG_EN=1 at rt9471_plug_in()
	 * Secondary charger: CHG_EN=1 at needed, e.x.: PE10, PE20, etc...
	 */
	ret = __sgm41512_enable_chg(chip, false);
	if (ret < 0)
		dev_notice(chip->dev, "%s dis chg fail(%d)\n", __func__, ret);

	/*
	 * Customization for MTK platform
	 * Primary charger: HZ=0 at sink vbus 5V with TCPC enabled
	 * Secondary charger: HZ=0 at needed, e.x.: PE10, PE20, etc...
	 */
#ifndef CONFIG_TCPC_CLASS
	if (strcmp(desc->chg_name, "secondary_chg") == 0) {
#endif /* CONFIG_TCPC_CLASS */
		ret = __sgm41512_enable_hz(chip, true);
		if (ret < 0)
			dev_notice(chip->dev, "%s en hz fail(%d)\n",
					      __func__, ret);
#ifndef CONFIG_TCPC_CLASS
	}
#endif /* CONFIG_TCPC_CLASS */

	return 0;
}

static int sgm41512_reset_register(struct sgm41512_chip *chip)
{
	int ret = 0;

	dev_info(chip->dev, "%s\n", __func__);

	ret = sgm41512_set_bit(chip, SGM41512_REG_REGRST, SGM41512_REGRST_MASK);
	if (ret < 0)
		return ret;
#ifdef CONFIG_RT_REGMAP
	rt_regmap_cache_reload(chip->rm_dev);
#endif /* CONFIG_RT_REGMAP */

	return __sgm41512_set_wdt(chip, 0);
}

static bool sgm41512_check_devinfo(struct sgm41512_chip *chip)
{
	int ret = 0;

	ret = i2c_smbus_read_byte_data(chip->client, SGM41512_REG_DEVID);
	if (ret < 0) {
		dev_notice(chip->dev, "%s get devinfo fail(%d)\n",
				      __func__, ret);
		return false;
	}
	chip->dev_id = (ret & SGM41512_DEVID_MASK) >> SGM41512_DEVID_SHIFT;
	chip->dev_rev = (ret & SGM41512_DEVREV_MASK) >> SGM41512_DEVREV_SHIFT;
	dev_info(chip->dev, "%s id = 0x%02X, devid rev = 0x%02X\n",
			    __func__, chip->dev_id, chip->dev_rev);
	switch (chip->dev_id) {
/*prize add by zhaopengge 20210623 start*/
	case ETA6963_DEVID:
	case ETA6953_DEVID:
/*prize add by zhaopengge 20210623 end*/
	case SGM41512_DEVID:
		break;
	default:
		dev_notice(chip->dev, "%s incorrect devid 0x%02X\n",
				      __func__, chip->dev_id);
		return false;
	}

	return true;
}

static int sgm41512_enable_charging(struct charger_device *chg_dev, bool en);
static int sgm41512_plug_in(struct charger_device *chg_dev)
{
	struct sgm41512_chip *chip = dev_get_drvdata(&chg_dev->dev);

	dev_info(chip->dev, "%s\n", __func__);

	/* Enable charging */
	return sgm41512_enable_charging(chg_dev, true);
}

static int sgm41512_plug_out(struct charger_device *chg_dev)
{
	struct sgm41512_chip *chip = dev_get_drvdata(&chg_dev->dev);

	dev_info(chip->dev, "%s\n", __func__);

	/* Disable charging */
	return sgm41512_enable_charging(chg_dev, false);
}

static int sgm41512_enable_charging(struct charger_device *chg_dev, bool en)
{
	int ret = 0;
	struct sgm41512_chip *chip = dev_get_drvdata(&chg_dev->dev);

	dev_info(chip->dev, "%s en = %d\n", __func__, en);

	if (en) {
		ret = __sgm41512_set_wdt(chip, chip->desc->wdt);
		if (ret < 0) {
			dev_notice(chip->dev, "%s set wdt fail(%d)\n",
					      __func__, ret);
			return ret;
		}
	}

	ret = __sgm41512_enable_chg(chip, en);
	if (ret < 0) {
		dev_notice(chip->dev, "%s en chg fail(%d)\n", __func__, ret);
		return ret;
	}

	if (!en) {
		ret = __sgm41512_set_wdt(chip, 0);
		if (ret < 0)
			dev_notice(chip->dev, "%s set wdt fail(%d)\n",
					      __func__, ret);
	}

	return ret;
}

static int sgm41512_is_charging_enabled(struct charger_device *chg_dev, bool *en)
{
	struct sgm41512_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return __sgm41512_is_chg_enabled(chip, en);
}

static int sgm41512_is_charging_done(struct charger_device *chg_dev, bool *done)
{
	int ret = 0;
	struct sgm41512_chip *chip = dev_get_drvdata(&chg_dev->dev);
	enum sgm41512_ic_stat ic_stat = SGM41512_ICSTAT_SLEEP;

	ret = __sgm41512_get_ic_stat(chip, &ic_stat);
	if (ret < 0)
		return ret;
	*done = (ic_stat == SGM41512_ICSTAT_CHGDONE);

	return ret;
}

static int sgm41512_set_vindpm(struct charger_device *chg_dev, u32 uV)
{
	struct sgm41512_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return __sgm41512_set_mivr(chip, uV);
}

static int sgm41512_get_vindpm_state(struct charger_device *chg_dev, bool *in_loop)
{
	struct sgm41512_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return sgm41512_i2c_test_bit(chip, SGM41512_REG_ST_MIVR,
				   SGM41512_ST_MIVR_SHIFT, in_loop);
}

static int sgm41512_get_iindpm(struct charger_device *chg_dev, u32 *uA)
{
	struct sgm41512_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return __sgm41512_get_aicr(chip, uA);
}

static int sgm41512_set_iindpm(struct charger_device *chg_dev, u32 uA)
{
	struct sgm41512_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return __sgm41512_set_aicr(chip, uA);
}

static int sgm41512_get_min_iindpm(struct charger_device *chg_dev, u32 *uA)
{
	*uA = sgm41512_closest_value(SGM41512_AICR_MIN, SGM41512_AICR_MAX,
				   SGM41512_AICR_STEP, 0);
	return 0;
}

static int sgm41512_get_vreg(struct charger_device *chg_dev, u32 *uV)
{
	struct sgm41512_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return __sgm41512_get_cv(chip, uV);
}

static int sgm41512_set_vreg(struct charger_device *chg_dev, u32 uV)
{
	struct sgm41512_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return __sgm41512_set_cv(chip, uV);
}

static int sgm41512_get_ichg(struct charger_device *chg_dev, u32 *uA)
{
	struct sgm41512_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return __sgm41512_get_ichg(chip, uA);
}

static int sgm41512_set_ichg(struct charger_device *chg_dev, u32 uA)
{
	struct sgm41512_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return __sgm41512_set_ichg(chip, uA);
}

static int sgm41512_get_min_ichg(struct charger_device *chg_dev, u32 *uA)
{
	*uA = sgm41512_closest_value(SGM41512_ICHG_MIN, SGM41512_ICHG_MAX,
				   SGM41512_ICHG_STEP, 0);
	return 0;
}

static int sgm41512_get_iterm(struct charger_device *chg_dev, u32 *uA)
{
	struct sgm41512_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return __sgm41512_get_ieoc(chip, uA);
}

static int sgm41512_set_iterm(struct charger_device *chg_dev, u32 uA)
{
	struct sgm41512_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return __sgm41512_set_ieoc(chip, uA);
}

static int sgm41512_enable_term(struct charger_device *chg_dev, bool en)
{
	struct sgm41512_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return __sgm41512_enable_te(chip, en);
}

static int sgm41512_kick_wdt(struct charger_device *chg_dev)
{
	struct sgm41512_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return __sgm41512_kick_wdt(chip);
}

static int sgm41512_event(struct charger_device *chg_dev, u32 event, u32 args)
{
	struct sgm41512_chip *chip = dev_get_drvdata(&chg_dev->dev);

	dev_info(chip->dev, "%s event = %d\n", __func__, event);

	switch (event) {
	case EVENT_EOC:
		charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_EOC);
		break;
	case EVENT_RECHARGE:
		charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_RECHG);
		break;
	default:
		break;
	}

	return 0;
}

static int sgm41512_enable_hz(struct charger_device *chg_dev, bool en)
{
	struct sgm41512_chip *chip = dev_get_drvdata(&chg_dev->dev);

	dev_info(chip->dev, "%s en = %d\n", __func__, en);

	return __sgm41512_enable_hz(chip, !en);
}

static int sgm41512_is_hz_enabled(struct charger_device *chg_dev, bool *en)
{
	int ret = 0;
	struct sgm41512_chip *chip = dev_get_drvdata(&chg_dev->dev);

	ret = __sgm41512_is_hz_enabled(chip, en);
	*en = !*en;

	return ret;
}

static int sgm41512_enable_safety_timer(struct charger_device *chg_dev, bool en)
{
	struct sgm41512_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return __sgm41512_enable_safe_tmr(chip, en);
}

static int sgm41512_is_safety_timer_enabled(struct charger_device *chg_dev,
					  bool *en)
{
	struct sgm41512_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return sgm41512_i2c_test_bit(chip, SGM41512_REG_SAFETMR_EN,
				   SGM41512_SAFETMR_EN_SHIFT, en);
}

static int sgm41512_enable_otg(struct charger_device *chg_dev, bool en)
{
	int ret = 0;
	struct sgm41512_chip *chip = dev_get_drvdata(&chg_dev->dev);

	if (en) {
		//ret = __sgm41512_set_wdt(chip, chip->desc->wdt);
		ret = __sgm41512_set_wdt(chip, 0);
		if (ret < 0) {
			dev_notice(chip->dev, "%s set wdt fail(%d)\n",
					      __func__, ret);
			return ret;
		}

#ifdef CONFIG_TCPC_CLASS
		ret = __sgm41512_enable_hz(chip, false);
		if (ret < 0)
			dev_notice(chip->dev, "%s dis hz fail(%d)\n",
					      __func__, ret);
#endif /* CONFIG_TCPC_CLASS */
	}

	ret = __sgm41512_enable_otg(chip, en);
	if (ret < 0) {
		dev_notice(chip->dev, "%s en otg fail(%d)\n", __func__, ret);
		return ret;
	}

	if (!en) {
#ifdef CONFIG_TCPC_CLASS
		ret = __sgm41512_enable_hz(chip, true);
		if (ret < 0)
			dev_notice(chip->dev, "%s en hz fail(%d)\n",
					      __func__, ret);
#endif /* CONFIG_TCPC_CLASS */

		ret = __sgm41512_set_wdt(chip, 0);
		if (ret < 0)
			dev_notice(chip->dev, "%s set wdt fail(%d)\n",
					      __func__, ret);
	}

	return ret;
}

static int sgm41512_set_boost_ilim(struct charger_device *chg_dev,
					  u32 uA)
{
	struct sgm41512_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return __sgm41512_set_otgcc(chip, uA);
}

static int sgm41512_enable_chg_type_det(struct charger_device *chg_dev, bool en)
{
	int ret = 0;
#ifdef CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT
#ifdef CONFIG_TCPC_CLASS
	int vbusgd = 0;
	struct sgm41512_chip *chip = dev_get_drvdata(&chg_dev->dev);
	vbusgd = sgm41512_is_vbusgd(chip);
	
	dev_info(chip->dev, "%s en = %d,sgm41512_is_vbusgd = %d\n", __func__, en,vbusgd);

	mutex_lock(&chip->bc12_lock);
	
	atomic_set(&chip->vbus_gd,en);
	
	ret = (en ? sgm41512_charge_type_detect_first : sgm41512_charge_type_detect_second)(chip);
	mutex_unlock(&chip->bc12_lock);
	if (ret < 0)
		dev_notice(chip->dev, "%s en bc12 fail(%d)\n", __func__, ret);
#endif /* CONFIG_TCPC_CLASS */
#endif /* CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT */
	return ret;
}

static int sgm41512_dump_registers(struct charger_device *chg_dev)
{
	struct sgm41512_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return __sgm41512_dump_registers(chip);
}

static int sgm41512_run_iind(struct charger_device *chg_dev, u32 *uA)
{
	int ret = 0;
	struct sgm41512_chip *chip = dev_get_drvdata(&chg_dev->dev);
	bool chg_mivr = false;
	u32 aicr = 0;

	dev_info(chip->dev, "%s chip_rev = %d\n", __func__, chip->chip_rev);

	ret = sgm41512_i2c_test_bit(chip, SGM41512_REG_ST_MIVR, SGM41512_ST_MIVR_SHIFT,
				  &chg_mivr);
	if (ret < 0)
		return ret;
	if (!chg_mivr) {
		dev_info(chip->dev, "%s mivr stat not act\n", __func__);
		return ret;
	}

	/* Backup the aicr */
	ret = __sgm41512_get_aicr(chip, &aicr);
	if (ret < 0)
		return ret;
	
	


	return ret;
}

static struct charger_ops sgm41512_chg_ops = {
	/* cable plug in/out for primary charger */
	.plug_in = sgm41512_plug_in,
	.plug_out = sgm41512_plug_out,

	/* enable/disable charger */
	.enable = sgm41512_enable_charging,
	.is_enabled = sgm41512_is_charging_enabled,
	.is_charging_done = sgm41512_is_charging_done,

	/* get/set minimun input voltage regulation */
	.set_mivr = sgm41512_set_vindpm,
	.get_mivr_state = sgm41512_get_vindpm_state,

	/* get/set input current */
	.get_input_current = sgm41512_get_iindpm,
	.set_input_current = sgm41512_set_iindpm,
	.get_min_input_current = sgm41512_get_min_iindpm,

	/* get/set charging voltage */
	.get_constant_voltage = sgm41512_get_vreg,
	.set_constant_voltage = sgm41512_set_vreg,

	/* get/set charging current*/
	.get_charging_current = sgm41512_get_ichg,
	.set_charging_current = sgm41512_set_ichg,
	.get_min_charging_current = sgm41512_get_min_ichg,

	/* get/set termination current */
	.get_eoc_current = sgm41512_get_iterm,
	.set_eoc_current = sgm41512_set_iterm,

	/* enable te */
	.enable_termination = sgm41512_enable_term,

	/* kick wdt */
	.kick_wdt = sgm41512_kick_wdt,

	.event = sgm41512_event,

	/* enable/disable powerpath for primary charger */
	.enable_powerpath = sgm41512_enable_hz,
	.is_powerpath_enabled = sgm41512_is_hz_enabled,

	/* enable/disable chip for secondary charger */
	.enable_chip = sgm41512_enable_hz,
	.is_chip_enabled = sgm41512_is_hz_enabled,

	/* enable/disable charging safety timer */
	.enable_safety_timer = sgm41512_enable_safety_timer,
	.is_safety_timer_enabled = sgm41512_is_safety_timer_enabled,

	/* OTG */
	.enable_otg = sgm41512_enable_otg,
	.set_boost_current_limit = sgm41512_set_boost_ilim,

	/* charger type detection */
	.enable_chg_type_det = sgm41512_enable_chg_type_det,

	.dump_registers = sgm41512_dump_registers,

	/* new features for chip_rev >= 4, AICC */
	.run_aicl = sgm41512_run_iind,
};
#if 1
static ssize_t shipping_mode_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int ret = 0, tmp = 0;
	struct sgm41512_chip *chip = dev_get_drvdata(dev);

	ret = kstrtoint(buf, 10, &tmp);
	if (ret < 0) {
		dev_notice(dev, "%s parsing number fail(%d)\n", __func__, ret);
		return -EINVAL;
	}
	if (tmp != 5526789)
		return -EINVAL;
	chip->enter_shipping_mode = true;
	/*
	 * Use kernel_halt() instead of kernel_power_off() to prevent
	 * the system from booting again while cable still plugged-in.
	 * But plug-out cable before AP WDT timeout, please.
	 */
	kernel_halt();

	return count;
}
static const DEVICE_ATTR_WO(shipping_mode);
#endif

static int sgm41512_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret = 0;
	struct sgm41512_chip *chip = NULL;

	dev_info(&client->dev, "%s (%s)\n", __func__, SGM41512_DRV_VERSION);

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	chip->client = client;
	chip->dev = &client->dev;
	mutex_init(&chip->io_lock);
#ifdef CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT
	mutex_init(&chip->bc12_lock);
	mutex_init(&chip->bc12_en_lock);
#endif /* CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT */
	mutex_init(&chip->hidden_mode_lock);
	chip->hidden_mode_cnt = 0;
#ifdef CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT
	INIT_DELAYED_WORK(&chip->psy_dwork, sgm41512_inform_psy_dwork_handler);
	atomic_set(&chip->vbus_gd, 0);
	chip->attach = false;
	chip->port = SGM41512_PORTSTAT_NOINPUT;
	chip->chg_type = CHARGER_UNKNOWN;
#endif /* CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT */
	chip->chg_done_once = false;
	wakeup_source_init(&chip->buck_dwork_ws,
			   devm_kasprintf(chip->dev, GFP_KERNEL,
			   "sgm41512_buck_dwork_ws.%s", dev_name(chip->dev)));
	INIT_DELAYED_WORK(&chip->buck_dwork, sgm41512_buck_dwork_handler);
	chip->enter_shipping_mode = false;
	chip->bc12_enable_done = false;
	//init_completion(&chip->aicc_done);
	init_completion(&chip->pe_done);
	i2c_set_clientdata(client, chip);

	if (!sgm41512_check_devinfo(chip)) {
		ret = -ENODEV;
		goto err_nodev;
	}

	ret = sgm41512_parse_dt(chip);
	if (ret < 0) {
		dev_notice(chip->dev, "%s parse dt fail(%d)\n", __func__, ret);
		goto err_parse_dt;
	}

#ifdef CONFIG_RT_REGMAP
	ret = sgm41512_register_rt_regmap(chip);
	if (ret < 0) {
		dev_notice(chip->dev, "%s register rt regmap fail(%d)\n",
				      __func__, ret);
		goto err_register_rm;
	}
#endif /* CONFIG_RT_REGMAP */

	ret = sgm41512_reset_register(chip);
	if (ret < 0)
		dev_notice(chip->dev, "%s reset register fail(%d)\n",
				      __func__, ret);

	ret = sgm41512_init_setting(chip);
	if (ret < 0) {
		dev_notice(chip->dev, "%s init fail(%d)\n", __func__, ret);
		goto err_init;
	}

#ifdef CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT
/*prize add by zhaopengge 20210623 start*/
	if (chip->dev_id == SGM41512_DEVID || chip->dev_id == ETA6963_DEVID || chip->dev_id == ETA6953_DEVID) {
/*prize add by zhaopengge 20210623 end*/
		wakeup_source_init(&chip->bc12_en_ws, devm_kasprintf(chip->dev,
				   GFP_KERNEL, "sgm41512_bc12_en_ws.%s",
				   dev_name(chip->dev)));
		//chip->bc12_en_buf[0] = chip->bc12_en_buf[1] = -1;
		chip->bc12_en_buf_idx = 0;
		atomic_set(&chip->bc12_en_req_cnt, 0);
		init_waitqueue_head(&chip->bc12_en_req);
		chip->bc12_en_kthread =
			kthread_run(sgm41512_bc12_en_kthread, chip,
				    devm_kasprintf(chip->dev, GFP_KERNEL,
				    "sgm41512_bc12_en_kthread.%s",
				    dev_name(chip->dev)));
		if (IS_ERR_OR_NULL(chip->bc12_en_kthread)) {
			ret = PTR_ERR(chip->bc12_en_kthread);
			dev_notice(chip->dev, "%s kthread run fail(%d)\n",
					      __func__, ret);
			goto err_kthread_run;
		}
	}
#endif /* CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT */

	chip->chg_dev = charger_device_register(chip->desc->chg_name,
			chip->dev, chip, &sgm41512_chg_ops, &chip->chg_props);
	if (IS_ERR_OR_NULL(chip->chg_dev)) {
		ret = PTR_ERR(chip->chg_dev);
		dev_notice(chip->dev, "%s register chg dev fail(%d)\n",
				      __func__, ret);
		goto err_register_chg_dev;
	}

	ret = sgm41512_check_chg(chip);
	if (ret < 0) {
		dev_notice(chip->dev, "%s check chg(%d)\n", __func__, ret);
		goto err_check_chg;
	}

	ret = sgm41512_register_irq(chip);
	if (ret < 0) {
		dev_notice(chip->dev, "%s register irq fail(%d)\n",
				      __func__, ret);
		goto err_register_irq;
	}

	if (ret < 0) {
		dev_notice(chip->dev, "%s init irq fail(%d)\n", __func__, ret);
		goto err_init_irq;
	}

	ret = device_create_file(chip->dev, &dev_attr_shipping_mode);
	if (ret < 0) {
		dev_notice(chip->dev, "%s create file fail(%d)\n",
				      __func__, ret);
		goto err_create_file;
	}

	__sgm41512_dump_registers(chip);
	dev_info(chip->dev, "%s successfully\n", __func__);
	return 0;

err_create_file:
err_init_irq:
err_register_irq:
err_check_chg:
	charger_device_unregister(chip->chg_dev);
err_register_chg_dev:
#ifdef CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT
	if (chip->bc12_en_kthread) {
		kthread_stop(chip->bc12_en_kthread);
		wakeup_source_trash(&chip->bc12_en_ws);
	}
err_kthread_run:
#endif /* CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT */
err_init:
#ifdef CONFIG_RT_REGMAP
	rt_regmap_device_unregister(chip->rm_dev);
err_register_rm:
#endif /* CONFIG_RT_REGMAP */
err_parse_dt:
err_nodev:
	mutex_destroy(&chip->io_lock);
#ifdef CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT
	mutex_destroy(&chip->bc12_lock);
	mutex_destroy(&chip->bc12_en_lock);
#endif /* CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT */
	mutex_destroy(&chip->hidden_mode_lock);
	wakeup_source_trash(&chip->buck_dwork_ws);
	devm_kfree(chip->dev, chip);
	return ret;
}

static int sgm41512_remove(struct i2c_client *client)
{
	struct sgm41512_chip *chip = i2c_get_clientdata(client);

	dev_info(chip->dev, "%s\n", __func__);

	device_remove_file(chip->dev, &dev_attr_shipping_mode);
	charger_device_unregister(chip->chg_dev);
	disable_irq(chip->irq);
#ifdef CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT
	cancel_delayed_work_sync(&chip->psy_dwork);
	if (chip->psy)
		power_supply_put(chip->psy);
	if (chip->bc12_en_kthread) {
		kthread_stop(chip->bc12_en_kthread);
		wakeup_source_trash(&chip->bc12_en_ws);
	}
#endif /* CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT */
	cancel_delayed_work_sync(&chip->buck_dwork);
#ifdef CONFIG_RT_REGMAP
	rt_regmap_device_unregister(chip->rm_dev);
#endif /* CONFIG_RT_REGMAP */
	mutex_destroy(&chip->io_lock);
#ifdef CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT
	mutex_destroy(&chip->bc12_lock);
	mutex_destroy(&chip->bc12_en_lock);
#endif /* CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT */
	mutex_destroy(&chip->hidden_mode_lock);
	wakeup_source_trash(&chip->buck_dwork_ws);

	return 0;
}

static void sgm41512_shutdown(struct i2c_client *client)
{
	int ret = 0;
	struct sgm41512_chip *chip = i2c_get_clientdata(client);

	dev_info(chip->dev, "%s\n", __func__);

	charger_device_unregister(chip->chg_dev);
	disable_irq(chip->irq);
#ifdef CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT
	if (chip->bc12_en_kthread)
		kthread_stop(chip->bc12_en_kthread);
#endif /* CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT */
	cancel_delayed_work_sync(&chip->buck_dwork);
	sgm41512_reset_register(chip);

	if (!chip->enter_shipping_mode)
		return;

	ret = __sgm41512_enable_shipmode(chip, true);
	if (ret < 0)
		dev_notice(chip->dev, "%s enter shipping mode fail(%d)\n",
				      __func__, ret);
}

static int sgm41512_suspend(struct device *dev)
{
	struct sgm41512_chip *chip = dev_get_drvdata(dev);

	dev_info(dev, "%s\n", __func__);
	if (device_may_wakeup(dev))
		enable_irq_wake(chip->irq);
	//disable_irq(chip->irq);

	return 0;
}

static int sgm41512_resume(struct device *dev)
{
	struct sgm41512_chip *chip = dev_get_drvdata(dev);

	dev_info(dev, "%s\n", __func__);
	enable_irq(chip->irq);
	if (device_may_wakeup(dev))
		disable_irq_wake(chip->irq);

	return 0;
}

static SIMPLE_DEV_PM_OPS(sgm41512_pm_ops, sgm41512_suspend, sgm41512_resume);

static const struct of_device_id sgm41512_of_device_id[] = {
	{ .compatible = "sgm,swchg", },
	{ },
};
MODULE_DEVICE_TABLE(of, sgm41512_of_device_id);

static const struct i2c_device_id sgm41512_i2c_device_id[] = {
	{ "sgm41512", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, sgm41512_i2c_device_id);

static struct i2c_driver sgm41512_i2c_driver = {
	.driver = {
		.name = "sgm41512",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(sgm41512_of_device_id),
		//.pm = &sgm41512_pm_ops,
	},
	.probe = sgm41512_probe,
	.remove = sgm41512_remove,
	.shutdown = sgm41512_shutdown,
	.id_table = sgm41512_i2c_device_id,
};
module_i2c_driver(sgm41512_i2c_driver);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Stuart Su <stuart_su@sg-micro.com>");
MODULE_DESCRIPTION("SGM41512 Charger Driver");
MODULE_VERSION(SGM41512_DRV_VERSION);

/*
 * Release Note
 * 1.0.0
 * (1) Initial released
 */
