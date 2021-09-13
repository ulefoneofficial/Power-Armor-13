/*
 * aw881xx_monitor.c monitor_module
 *
 * Version: v0.2.0
 *
 * Copyright (c) 2019 AWINIC Technology CO., LTD
 *
 *  Author: Nick Li <liweilei@awinic.com.cn>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/module.h>
#include <linux/debugfs.h>
#include <asm/ioctls.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/firmware.h>
#include "aw881xx.h"
#include "aw881xx_reg.h"
#include "aw881xx_monitor.h"


static DEFINE_MUTEX(g_aw_monitor_lock);
static LIST_HEAD(g_aw_monitor_list);
uint8_t g_monitor_dev_num;
struct aw_monitor_cfg g_monitor_cfg;
#define AW_MONITOR_FILE "aw881xx_monitor.bin"

static void aw881xx_monitor_set_gain_value(struct aw881xx *aw881xx,
					uint16_t gain_value)
{
	struct aw_monitor_cfg *monitor_cfg = aw881xx->monitor.monitor_cfg;
	uint32_t read_volume;
	uint32_t set_volume;
	uint32_t gain_db;

	if (gain_value == AW_GAIN_NONE || (!monitor_cfg->gain_switch))
		return;

	aw881xx_get_volume(aw881xx, &read_volume);

	gain_db = aw881xx_reg_val_to_db(gain_value);

	/*add offset*/
	set_volume = gain_db + aw881xx->db_offset;

	if (read_volume == set_volume) {
		aw_dev_info(aw881xx->dev, "%s: db_offset:%d + gain_db:%d = %d, no change\n",
			__func__, aw881xx->db_offset, gain_db, set_volume);
		return;
	}

	aw881xx_set_volume(aw881xx, set_volume);

	aw_dev_info(aw881xx->dev, "%s: db_offset:%d + gain_db:%d = %d\n",
		__func__, aw881xx->db_offset, gain_db, set_volume);
}

static void aw881xx_monitor_set_bst_ipeak(struct aw881xx *aw881xx,
					uint16_t bst_ipeak)
{
	struct aw_monitor_cfg *monitor_cfg = aw881xx->monitor.monitor_cfg;
	int ret = -1;
	uint16_t reg_val = 0;
	uint16_t read_reg_val;

	if (bst_ipeak == AW_IPEAK_NONE || (!monitor_cfg->ipeak_switch))
		return;

	ret = aw881xx_reg_read(aw881xx, AW881XX_REG_SYSCTRL2, &reg_val);
	if (ret < 0)
		return;

	/* check ipeak */
	read_reg_val = reg_val;
	read_reg_val = read_reg_val & (~AW881XX_BIT_SYSCTRL2_BST_IPEAK_MASK);
	if (read_reg_val == bst_ipeak) {
		aw_dev_info(aw881xx->dev, "%s: ipeak=0x%x, no change\n",
			__func__, bst_ipeak);
		return;
	}

	reg_val &= AW881XX_BIT_SYSCTRL2_BST_IPEAK_MASK;
	reg_val |= bst_ipeak;

	ret = aw881xx_reg_write(aw881xx, AW881XX_REG_SYSCTRL2, reg_val);
	if (ret < 0)
		return;

	aw_dev_info(aw881xx->dev, "%s: set reg_val=0x%x, bst_ipeak=0x%x\n",
		__func__, reg_val, bst_ipeak);
}

static int aw881xx_monitor_vmax_check(struct aw881xx *aw881xx)
{
	int ret = -1;

	ret = aw881xx_get_iis_status(aw881xx);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: no iis signal\n", __func__);
		return ret;
	}

	ret = aw881xx_get_dsp_status(aw881xx);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: dsp not work\n", __func__);
		return ret;
	}

	return 0;
}

static void aw881xx_monitor_set_vmax(struct aw881xx *aw881xx, uint32_t vmax)
{
	struct aw_monitor_cfg *monitor_cfg = aw881xx->monitor.monitor_cfg;
	int ret = -1;
	uint16_t vmax_set;

	if (vmax == AW_VMAX_NONE || (!monitor_cfg->vmax_switch))
		return;

	ret = aw881xx_monitor_vmax_check(aw881xx);
	if (ret) {
		aw_dev_err(aw881xx->dev, "%s: vmax_check fail, ret=%d\n",
			__func__, ret);
		return;
	}

	vmax_set = (uint16_t)((int16_t)vmax +
			(int16_t)aw881xx->monitor.vmax_init_val);
	aw_dev_info(aw881xx->dev, "%s: vmax_set:0x%x, vmax:0x%x\n",
			__func__, vmax_set, vmax);

	ret = aw881xx_dsp_write(aw881xx, AW881XX_DSP_REG_VMAX, vmax_set);
	if (ret)
		return;
}

static int aw881xx_monitor_get_voltage(struct aw881xx *aw881xx,
					uint16_t *voltage)
{
	int ret = -1;

	if (voltage == NULL) {
		aw_dev_err(aw881xx->dev, "%s: voltage=%p\n",
			__func__, voltage);
		return ret;
	}

	ret = aw881xx_reg_read(aw881xx, AW881XX_REG_VBAT, voltage);
	if (ret < 0)
		return ret;

	*voltage = (*voltage) * AW881XX_VBAT_RANGE /
		AW881XX_VBAT_COEFF_INT_10BIT;

	aw_dev_dbg(aw881xx->dev, "%s: chip voltage=%dmV\n", __func__, *voltage);

	return ret;
}

static int aw881xx_monitor_get_temperature(struct aw881xx *aw881xx,
					int16_t *temp)
{
	int ret = -1;
	uint16_t reg_val = 0;

	if (temp == NULL) {
		aw_dev_err(aw881xx->dev, "%s:temp=%p\n",
			__func__, temp);
		return ret;
	}

	ret = aw881xx_reg_read(aw881xx, AW881XX_REG_TEMP, &reg_val);
	if (ret < 0)
		return ret;

	if (reg_val & AW881XX_TEMP_SIGN_MASK)
		reg_val |= AW881XX_TEMP_NEG_MASK;

	*temp = (int)reg_val;

	aw_dev_dbg(aw881xx->dev, "%s: chip_temperature=%d\n", __func__, *temp);

	return ret;
}

static int aw881xx_monitor_get_temp_and_vol(struct aw881xx *aw881xx)
{
	struct aw881xx_monitor *monitor = &aw881xx->monitor;
	uint16_t voltage = 0;
	int16_t current_temp = 0;
	int ret = -1;

#ifdef AW_DEBUG
	if (monitor->test_vol == 0) {
		ret = aw881xx_monitor_get_voltage(aw881xx, &voltage);
		if (ret < 0)
			return ret;
	} else {
		voltage = monitor->test_vol;
	}

	if (monitor->test_temp == 0) {
		ret = aw881xx_monitor_get_temperature(aw881xx, &current_temp);
		if (ret)
			return ret;
	} else {
		current_temp = monitor->test_temp;
	}
#else
	ret = aw881xx_monitor_get_voltage(aw881xx, &voltage);
	if (ret < 0)
		return ret;

	ret = aw881xx_monitor_get_temperature(aw881xx, &current_temp);
	if (ret < 0)
		return ret;
#endif

	monitor->vol_trace.sum_val += voltage;
	monitor->temp_trace.sum_val += current_temp;
	monitor->samp_count++;

	return 0;
}

static int aw881xx_monitor_trace_data_from_table(struct aw881xx *aw881xx,
			struct aw_table_info table_info,
			struct aw_monitor_trace *data_trace)
{
	int i;

	if (table_info.aw_table == NULL) {
		aw_dev_err(aw881xx->dev, "%s: table_info.aw_table is null\n",
			__func__);
		return -EINVAL;
	}

	for (i = 0; i < table_info.table_num; i++) {
		if (data_trace->sum_val >= table_info.aw_table[i].min_val &&
			data_trace->sum_val <= table_info.aw_table[i].max_val) {
			memcpy(&data_trace->aw_table, &table_info.aw_table[i],
				sizeof(struct aw_table));
			break;
		}
	}

	return 0;
}

static int aw881xx_monitor_first_get_data_form_table(struct aw881xx *aw881xx,
				struct aw_table_info table_info,
			struct aw_monitor_trace *data_trace)
{
	int i;

	if (table_info.aw_table == NULL) {
		aw_dev_err(aw881xx->dev, "%s: table_info.aw_table is null\n",
			__func__);
		return -EINVAL;
	}

	for (i = 0; i < table_info.table_num; i++) {
		if (data_trace->sum_val >= table_info.aw_table[i].min_val) {
			memcpy(&data_trace->aw_table, &table_info.aw_table[i],
				sizeof(struct aw_table));
			break;
		}
	}

	return 0;
}

static int aw881xx_monitor_get_data_from_table(struct aw881xx *aw881xx,
					struct aw_table_info table_info,
					struct aw_monitor_trace *data_trace,
					uint32_t aplha)
{
	struct aw881xx_monitor *monitor = &aw881xx->monitor;

	if (monitor->first_entry == AW_FIRST_ENTRY) {
		return aw881xx_monitor_first_get_data_form_table(aw881xx,
						table_info, data_trace);
	} else {
		data_trace->sum_val = data_trace->sum_val / monitor->samp_count;
		data_trace->sum_val = ((int32_t)aplha * data_trace->sum_val +
			(1000 - (int32_t)aplha) * data_trace->pre_val) / 1000;
		return aw881xx_monitor_trace_data_from_table(aw881xx,
						table_info, data_trace);
	}

	return 0;
}

static int aw881xx_monitor_get_data(struct aw881xx *aw881xx)
{
	struct aw881xx_monitor *monitor = &aw881xx->monitor;
	struct aw_monitor_cfg *monitor_cfg = monitor->monitor_cfg;
	struct aw_monitor_trace *vol_trace = &monitor->vol_trace;
	struct aw_monitor_trace *temp_trace = &monitor->temp_trace;
	int ret;

	if (monitor_cfg->vol_switch) {
		ret = aw881xx_monitor_get_data_from_table(aw881xx,
			monitor_cfg->vol_info, vol_trace,
			monitor_cfg->vol_aplha);
		if (ret < 0)
			return ret;
	} else {
		vol_trace->aw_table.ipeak = AW_IPEAK_NONE;
		vol_trace->aw_table.gain = AW_GAIN_NONE;
		vol_trace->aw_table.vmax = AW_VMAX_NONE;
	}

	if (monitor_cfg->temp_switch) {
		ret = aw881xx_monitor_get_data_from_table(aw881xx,
			monitor_cfg->temp_info, temp_trace,
			monitor_cfg->temp_aplha);
		if (ret < 0)
			return ret;
	} else {
		temp_trace->aw_table.ipeak = AW_IPEAK_NONE;
		temp_trace->aw_table.gain = AW_GAIN_NONE;
		temp_trace->aw_table.vmax = AW_VMAX_NONE;
	}

	aw_dev_dbg(aw881xx->dev, "%s: filter_vol:%d, ipeak = 0x%x, gain = 0x%x, vmax = 0x%x\n",
			__func__, monitor->vol_trace.sum_val,
			vol_trace->aw_table.ipeak,
			vol_trace->aw_table.gain,
			vol_trace->aw_table.vmax);

	aw_dev_dbg(aw881xx->dev, "%s: filter_temp:%d, temp: ipeak = 0x%x, gain = 0x%x, vmax = 0x%x\n",
			__func__, monitor->temp_trace.sum_val,
			temp_trace->aw_table.ipeak,
			temp_trace->aw_table.gain,
			temp_trace->aw_table.vmax);

	return 0;
}

static void aw881xx_monitor_get_cfg(struct aw881xx *aw881xx,
					struct aw_table *set_table)
{
	struct aw881xx_monitor *monitor = &aw881xx->monitor;
	struct aw_table *temp_data = &monitor->temp_trace.aw_table;
	struct aw_table *vol_data = &monitor->vol_trace.aw_table;

	if (temp_data->ipeak == AW_IPEAK_NONE &&
		vol_data->ipeak == AW_IPEAK_NONE) {
		memcpy(set_table, temp_data, sizeof(struct aw_table));
	} else if (temp_data->ipeak == AW_IPEAK_NONE) {
		memcpy(set_table, vol_data, sizeof(struct aw_table));
	} else if (vol_data->ipeak == AW_IPEAK_NONE) {
		memcpy(set_table, temp_data, sizeof(struct aw_table));
	} else {
		set_table->ipeak = (temp_data->ipeak < vol_data->ipeak ?
				temp_data->ipeak : vol_data->ipeak);
		set_table->gain = (temp_data->gain < vol_data->gain ?
				vol_data->gain : temp_data->gain);
		set_table->vmax = (temp_data->vmax < vol_data->vmax ?
				vol_data->vmax : temp_data->vmax);
	}
}

static int aw881xx_monitor_check_sysint(struct aw881xx *aw881xx)
{
	int ret = -1;
	uint16_t sysint = 0;

	ret = aw881xx_get_sysint(aw881xx, &sysint);
	if (ret < 0)
		aw_dev_err(aw881xx->dev, "%s: get_sysint fail, ret=%d\n",
			__func__, ret);
	else
		aw_dev_info(aw881xx->dev, "%s: get_sysint=0x%04x\n",
			__func__, sysint);

	return ret;
}

static int aw881xx_monitor_work(struct aw881xx *aw881xx)
{
	int ret = -1;
	struct aw881xx_monitor *monitor = &aw881xx->monitor;
	struct aw_monitor_cfg *monitor_cfg = monitor->monitor_cfg;
	struct aw_table set_table;

	ret = aw881xx_monitor_get_temp_and_vol(aw881xx);
	if (ret < 0)
		return ret;

	if (monitor->samp_count < monitor_cfg->monitor_count &&
		(monitor->first_entry == AW_NOT_FIRST_ENTRY))
		return 0;

	ret = aw881xx_monitor_get_data(aw881xx);
	if (ret < 0)
		return ret;

	aw881xx_monitor_get_cfg(aw881xx, &set_table);

	aw_dev_dbg(aw881xx->dev, "%s: set_ipeak = 0x%x, set_gain = 0x%x, set_vmax = 0x%x\n",
		__func__, set_table.ipeak, set_table.gain, set_table.vmax);

	aw881xx_monitor_set_bst_ipeak(aw881xx, set_table.ipeak);

	aw881xx_monitor_set_gain_value(aw881xx, set_table.gain);

	aw881xx_monitor_set_vmax(aw881xx, set_table.vmax);

	aw881xx_monitor_check_sysint(aw881xx);

	monitor->samp_count = 0;

	monitor->temp_trace.pre_val = monitor->temp_trace.sum_val;
	monitor->temp_trace.sum_val = 0;

	monitor->vol_trace.pre_val = monitor->vol_trace.sum_val;
	monitor->vol_trace.sum_val = 0;

	if (monitor->first_entry == AW_FIRST_ENTRY)
		monitor->first_entry = AW_NOT_FIRST_ENTRY;

	return 0;
}

static void aw881xx_monitor_work_func(struct work_struct *work)
{
	struct aw881xx *aw881xx = container_of(work,
			struct aw881xx, monitor.delay_work.work);
	struct aw_monitor_cfg *monitor_cfg = aw881xx->monitor.monitor_cfg;

	aw_dev_dbg(aw881xx->dev, "%s: enter\n", __func__);

	if ((aw881xx->monitor.is_enable) &&
		monitor_cfg->monitor_switch &&
		(aw881xx->prof_info.prof_desc[aw881xx->cur_prof].id != AW_PROFILE_RECEIVER) &&
		(monitor_cfg->monitor_status == AW_MON_CFG_OK)) {
		if (!aw881xx_get_hmute(aw881xx)) {
			aw881xx_monitor_work(aw881xx);
			schedule_delayed_work(&aw881xx->monitor.delay_work,
				msecs_to_jiffies(monitor_cfg->monitor_time));
		}
	}
}

void aw881xx_monitor_start(struct aw881xx_monitor *monitor)
{
	struct aw881xx *aw881xx = container_of(monitor,
			struct aw881xx, monitor);

	aw_dev_info(aw881xx->dev, "%s: enter\n", __func__);

	monitor->first_entry = AW_FIRST_ENTRY;
	monitor->samp_count = 0;
	monitor->vol_trace.sum_val = 0;
	monitor->temp_trace.sum_val = 0;

	schedule_delayed_work(&monitor->delay_work,
				msecs_to_jiffies(0));
}

void aw881xx_monitor_stop(struct aw881xx_monitor *monitor)
{
	struct aw881xx *aw881xx = container_of(monitor,
				struct aw881xx, monitor);

	aw_dev_info(aw881xx->dev, "%s: enter\n", __func__);

	if (delayed_work_pending(&monitor->delay_work))
		cancel_delayed_work_sync(&monitor->delay_work);
}


/*****************************************************
 * load monitor config
 *****************************************************/
static int aw_check_monitor_profile(struct aw881xx *aw881xx,
					const struct firmware *cont)
{
	struct aw_monitor_hdr *monitor_hdr =
		(struct aw_monitor_hdr *)cont->data;
	int temp_size, vol_size;

	if (cont->size < sizeof(struct aw_monitor_hdr)) {
		aw_dev_err(aw881xx->dev,
			"%s:params size[%d] < struct aw_monitor_hdr size[%d]!\n",
			__func__, (int)cont->size,
			(int)sizeof(struct aw_monitor_hdr));
		return -ENOMEM;
	}

	if (monitor_hdr->temp_offset > cont->size) {
		aw_dev_err(aw881xx->dev,
			"%s:temp_offset[%d] overflow file size[%d]!\n",
			__func__, monitor_hdr->temp_offset, (int)cont->size);
		return -ENOMEM;
	}

	if (monitor_hdr->vol_offset > cont->size) {
		aw_dev_err(aw881xx->dev,
			"%s:vol_offset[%d] overflow file size[%d]!\n",
			__func__, monitor_hdr->vol_offset, (int)cont->size);
		return -ENOMEM;
	}

	temp_size = monitor_hdr->temp_num * monitor_hdr->single_temp_size;
	if (temp_size > cont->size) {
		aw_dev_err(aw881xx->dev,
			"%s:temp_size:[%d] overflow file size[%d]!!\n",
			__func__, temp_size, (int)cont->size);
		return -ENOMEM;
	}

	vol_size = monitor_hdr->vol_num * monitor_hdr->single_vol_size;
	if (vol_size > cont->size) {
		aw_dev_err(aw881xx->dev,
			"%s:vol_size:[%d] overflow file size[%d]!\n",
			__func__, vol_size, (int)cont->size);
		return -ENOMEM;
	}

	return 0;
}

static void aw_parse_monitor_hdr(struct aw881xx *aw881xx,
					const struct firmware *cont)
{
	struct aw_monitor_hdr *monitor_hdr =
			(struct aw_monitor_hdr *)cont->data;
	struct aw_monitor_cfg *monitor_cfg = aw881xx->monitor.monitor_cfg;

	monitor_cfg->monitor_switch = monitor_hdr->monitor_switch;
	monitor_cfg->monitor_time = monitor_hdr->monitor_time;
	monitor_cfg->monitor_count = monitor_hdr->monitor_count;
	monitor_cfg->ipeak_switch = monitor_hdr->ipeak_switch;
	monitor_cfg->gain_switch = monitor_hdr->gain_switch;
	monitor_cfg->vmax_switch = monitor_hdr->vmax_switch;
	monitor_cfg->temp_switch = monitor_hdr->temp_switch;
	monitor_cfg->temp_aplha = monitor_hdr->temp_aplha;
	monitor_cfg->vol_switch = monitor_hdr->vol_switch;
	monitor_cfg->vol_aplha = monitor_hdr->vol_aplha;

	aw_dev_dbg(aw881xx->dev, "%s: chip name:%s\n",
		__func__, monitor_hdr->chip_type);
	aw_dev_dbg(aw881xx->dev, "%s: ui ver:0x%x\n",
		__func__, monitor_hdr->ui_ver);

	aw_dev_info(aw881xx->dev,
		"%s:monitor_switch:%d, monitor_time:%d (ms), monitor_count:%d\n",
		__func__, monitor_cfg->monitor_switch,
		monitor_cfg->monitor_time, monitor_cfg->monitor_count);

	aw_dev_info(aw881xx->dev,
		"%s:ipeak_switch:%d, gain_switch:%d, vmax_switch:%d\n",
		__func__, monitor_cfg->ipeak_switch,
		monitor_cfg->gain_switch, monitor_cfg->vmax_switch);

	aw_dev_info(aw881xx->dev,
		"%s:temp_switch:%d, temp_aplha:%d, vol_switch:%d, vol_aplha:%d\n",
		__func__, monitor_cfg->temp_switch,
		monitor_cfg->temp_aplha, monitor_cfg->vol_switch,
		monitor_cfg->vol_aplha);
}

static void aw_populate_data_to_table(struct aw881xx *aw881xx,
		struct aw_table_info *table_info, const char *offset_ptr)
{
	int i;

	for (i = 0; i < table_info->table_num * AW_TABLE_SIZE;
		i += AW_TABLE_SIZE) {
		table_info->aw_table[i / AW_TABLE_SIZE].min_val =
			GET_16_DATA(offset_ptr[1 + i], offset_ptr[i]);
		table_info->aw_table[i / AW_TABLE_SIZE].max_val =
			GET_16_DATA(offset_ptr[3 + i], offset_ptr[2 + i]);
		table_info->aw_table[i / AW_TABLE_SIZE].ipeak =
			GET_16_DATA(offset_ptr[5 + i], offset_ptr[4 + i]);
		table_info->aw_table[i / AW_TABLE_SIZE].gain =
			GET_16_DATA(offset_ptr[7 + i], offset_ptr[6 + i]);
		table_info->aw_table[i / AW_TABLE_SIZE].vmax =
			GET_32_DATA(offset_ptr[11 + i], offset_ptr[10 + i],
				offset_ptr[9 + i], offset_ptr[8 + i]);
	}

	for (i = 0; i < table_info->table_num; i++)
		aw_dev_info(aw881xx->dev,
			"%s: min_val:%d, max_val:%d, ipeak:0x%x, gain:0x%x, vmax:0x%x\n",
			__func__,
			table_info->aw_table[i].min_val,
			table_info->aw_table[i].max_val,
			table_info->aw_table[i].ipeak,
			table_info->aw_table[i].gain,
			table_info->aw_table[i].vmax);

}

static int aw_parse_temp_data(struct aw881xx *aw881xx,
			const struct firmware *cont)
{
	struct aw_monitor_hdr *monitor_hdr =
			(struct aw_monitor_hdr *)cont->data;
	struct aw_table_info *temp_info =
		&aw881xx->monitor.monitor_cfg->temp_info;

	aw_dev_info(aw881xx->dev, "%s: ===parse temp start ===\n",
		__func__);

	if (temp_info->aw_table != NULL) {
		kfree(temp_info->aw_table);
		temp_info->aw_table = NULL;
	}

	temp_info->aw_table = kzalloc((monitor_hdr->temp_num * AW_TABLE_SIZE),
			GFP_KERNEL);
	if (!temp_info->aw_table)
		return -ENOMEM;

	temp_info->table_num = monitor_hdr->temp_num;
	aw_populate_data_to_table(aw881xx, temp_info,
		&cont->data[monitor_hdr->temp_offset]);
	aw_dev_info(aw881xx->dev, "%s: ===parse temp end ===\n",
		__func__);
	return 0;
}

static int aw_parse_vol_data(struct aw881xx *aw881xx,
			const struct firmware *cont)
{
	struct aw_monitor_hdr *monitor_hdr =
			(struct aw_monitor_hdr *)cont->data;
	struct aw_table_info *vol_info =
		&aw881xx->monitor.monitor_cfg->vol_info;

	aw_dev_info(aw881xx->dev, "%s: ===parse vol start ===\n",
		__func__);

	if (vol_info->aw_table != NULL) {
		kfree(vol_info->aw_table);
		vol_info->aw_table = NULL;
	}

	vol_info->aw_table = kzalloc((monitor_hdr->vol_num * AW_TABLE_SIZE),
			GFP_KERNEL);
	if (!vol_info->aw_table)
		return -ENOMEM;

	vol_info->table_num = monitor_hdr->vol_num;
	aw_populate_data_to_table(aw881xx, vol_info,
		&cont->data[monitor_hdr->vol_offset]);
	aw_dev_info(aw881xx->dev, "%s: ===parse vol end ===\n",
		__func__);
	return 0;
}

static int aw_parse_monitor_data(struct aw881xx *aw881xx,
					const struct firmware *cont)
{
	int ret;
	struct aw_monitor_cfg *monitor_cfg = aw881xx->monitor.monitor_cfg;

	ret = aw_check_monitor_profile(aw881xx, cont);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s:check %s failed\n",
				__func__, AW_MONITOR_FILE);
		return ret;
	}

	aw_parse_monitor_hdr(aw881xx, cont);

	ret = aw_parse_temp_data(aw881xx, cont);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s:parse temp failed\n",
				__func__);
		return ret;
	}

	ret = aw_parse_vol_data(aw881xx, cont);
	if (ret < 0) {
		if (monitor_cfg->temp_info.aw_table != NULL) {
			kfree(monitor_cfg->temp_info.aw_table);
			monitor_cfg->temp_info.aw_table = NULL;
			monitor_cfg->temp_info.table_num = 0;
		}
		return ret;
	}

	monitor_cfg->monitor_status = AW_MON_CFG_OK;
	return 0;
}

static int aw_monitor_param_check_sum(struct aw881xx *aw881xx,
					const struct firmware *cont)
{
	int i, check_sum = 0;
	struct aw_monitor_hdr *monitor_hdr =
		(struct aw_monitor_hdr *)cont->data;

	for (i = 4; i < cont->size; i++)
		check_sum += (uint8_t)cont->data[i];

	if (monitor_hdr->check_sum != check_sum) {
		aw_dev_err(aw881xx->dev,
			"%s:check_sum[%d] is not equal to actual check_sum[%d]\n",
			__func__, monitor_hdr->check_sum, check_sum);
		return -ENOMEM;
	}

	return 0;
}

static int aw_parse_monitor_profile(struct aw881xx *aw881xx,
					const struct firmware *cont)
{
	struct aw_monitor_hdr *monitor_hdr =
			(struct aw_monitor_hdr *)cont->data;
	int ret;

	ret = aw_monitor_param_check_sum(aw881xx, cont);
	if (ret < 0)
		return ret;

	switch (monitor_hdr->monitor_ver) {
	case AW_MONITOR_HDR_VER_0_1_0:
		return aw_parse_monitor_data(aw881xx, cont);
	default:
		aw_dev_err(aw881xx->dev, "%s:cfg version:0x%x unsupported\n",
				__func__, monitor_hdr->monitor_ver);
		return -EINVAL;
	}
}

static void aw_monitor_profile_loaded(const struct firmware *cont,
					void *context)
{
	struct aw881xx *aw881xx = context;
	int ret;

	mutex_lock(&g_aw_monitor_lock);
	if (aw881xx->monitor.monitor_cfg->monitor_status == AW_MON_CFG_ST) {
		if (!cont) {
			aw_dev_err(aw881xx->dev, "%s:failed to read %s\n",
					__func__, AW_MONITOR_FILE);
			goto exit;
		}

		aw_dev_info(aw881xx->dev, "%s: loaded %s - size: %zu\n",
			__func__, AW_MONITOR_FILE, cont ? cont->size : 0);

		ret = aw_parse_monitor_profile(aw881xx, cont);
		if (ret < 0)
			aw_dev_err(aw881xx->dev, "%s:parse monitor cfg failed\n",
				__func__);
	}

exit:
	release_firmware(cont);
	mutex_unlock(&g_aw_monitor_lock);
}

int aw881xx_load_monitor_profile(struct aw881xx_monitor *monitor)
{
	int ret;
	struct aw881xx *aw881xx = container_of(monitor,
					struct aw881xx, monitor);

	if (!monitor->is_enable) {
		aw_dev_info(aw881xx->dev, "%s: monitor flag:%d, monitor bin noload\n",
			__func__, monitor->is_enable);
		ret = 0;
	} else {
		ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				AW_MONITOR_FILE,
				aw881xx->dev, GFP_KERNEL, aw881xx,
				aw_monitor_profile_loaded);
	}

	return ret;
}

static void aw881xx_deinit_monitor_profile(struct aw881xx_monitor *monitor)
{
	struct aw_monitor_cfg *monitor_cfg = monitor->monitor_cfg;
	struct aw881xx *aw881xx = container_of(monitor,
				struct aw881xx, monitor);

	aw_dev_dbg(aw881xx->dev, "%s: enter\n", __func__);

	monitor_cfg->monitor_status = AW_MON_CFG_ST;

	if (monitor_cfg->temp_info.aw_table != NULL) {
		kfree(monitor_cfg->temp_info.aw_table);
		monitor_cfg->temp_info.aw_table = NULL;
	}

	if (monitor_cfg->vol_info.aw_table != NULL) {
		kfree(monitor_cfg->vol_info.aw_table);
		monitor_cfg->vol_info.aw_table = NULL;
	}
	memset(monitor_cfg, 0, sizeof(struct aw_monitor_cfg));
}


#ifdef AW_DEBUG
static ssize_t aw881xx_vol_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	uint32_t vol = 0;
	int ret = -1;

	if (count == 0)
		return 0;

	ret = kstrtouint(buf, 0, &vol);
	if (ret < 0)
		return ret;

	aw_dev_info(aw881xx->dev, "%s: vol set =%d\n", __func__, vol);
	aw881xx->monitor.test_vol = vol;

	return count;
}

static ssize_t aw881xx_vol_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	ssize_t len = 0;
	uint32_t local_vol = aw881xx->monitor.test_vol;

	len += snprintf(buf+len, PAGE_SIZE-len,
		"aw881xx vol: %u\n", local_vol);
	return len;
}

static ssize_t aw881xx_temp_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	int32_t temp = 0;
	int ret = -1;

	if (count == 0)
		return 0;

	ret = kstrtoint(buf, 0, &temp);
	if (ret < 0)
		return ret;

	aw_dev_info(aw881xx->dev, "%s: temp set =%d\n", __func__, temp);
	aw881xx->monitor.test_temp = temp;

	return count;
}

static ssize_t aw881xx_temp_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	ssize_t len = 0;
	int32_t local_temp = aw881xx->monitor.test_temp;

	len += snprintf(buf+len, PAGE_SIZE-len,
		"aw881xx temp: %d\n", local_temp);
	return len;
}

static DEVICE_ATTR(vol, S_IWUSR | S_IRUGO,
	aw881xx_vol_show, aw881xx_vol_store);
static DEVICE_ATTR(temp, S_IWUSR | S_IRUGO,
	aw881xx_temp_show, aw881xx_temp_store);
#endif

static ssize_t aw881xx_monitor_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	int ret = -1;
	unsigned int databuf[2] = { 0 };

	ret = kstrtouint(buf, 0, &databuf[0]);
	if (ret < 0)
		return ret;

	mutex_lock(&aw881xx->lock);
	if (aw881xx->monitor.is_enable == databuf[0]) {
		mutex_unlock(&aw881xx->lock);
		return count;
	}
	aw881xx->monitor.is_enable = databuf[0];
	if (databuf[0])
		aw881xx_monitor_start(&aw881xx->monitor);
	mutex_unlock(&aw881xx->lock);

	return count;
}

static ssize_t aw881xx_monitor_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"aw881xx monitor_flag=%u\n",
			aw881xx->monitor.is_enable);

	return len;
}

static ssize_t aw_monitor_update_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	struct list_head *list = NULL;
	struct aw881xx_monitor *aw_monitor = NULL;
	uint32_t update = 0;
	int ret = -1;

	if (count == 0)
		return 0;

	ret = kstrtouint(buf, 0, &update);
	if (ret < 0)
		return ret;

	aw_dev_info(aw881xx->dev, "%s:monitor update = %d\n",
		__func__, update);

	if (update) {
		/*stop all monitor*/
		list_for_each(list, &g_aw_monitor_list) {
			aw_monitor = container_of(list,
				struct aw881xx_monitor, list);
			aw881xx_monitor_stop(aw_monitor);
		}

		aw881xx_deinit_monitor_profile(&aw881xx->monitor);
		aw881xx_load_monitor_profile(&aw881xx->monitor);
		msleep(50);
		/*start all monitor*/
		list_for_each(list, &g_aw_monitor_list) {
			aw_monitor = container_of(list,
				struct aw881xx_monitor, list);
			aw881xx_monitor_start(aw_monitor);
		}
	}

	return count;
}


static DEVICE_ATTR(monitor, S_IWUSR | S_IRUGO,
		aw881xx_monitor_show, aw881xx_monitor_store);
static DEVICE_ATTR(monitor_update, S_IWUSR,
		NULL, aw_monitor_update_store);


static struct attribute *aw881xx_monitor_attr[] = {
#ifdef AW_DEBUG
	&dev_attr_vol.attr,
	&dev_attr_temp.attr,
#endif
	&dev_attr_monitor_update.attr,
	&dev_attr_monitor.attr,
	NULL
};

static struct attribute_group aw881xx_monitor_attr_group = {
	.attrs = aw881xx_monitor_attr
};

int aw881xx_monitor_init(struct aw881xx_monitor *monitor)
{
	int ret = -1;
	struct aw881xx *aw881xx =
		container_of(monitor, struct aw881xx, monitor);

	aw_dev_info(aw881xx->dev, "%s: enter\n", __func__);

#ifdef AW_DEBUG
	monitor->test_vol = 0;
	monitor->test_temp = 0;
#endif

	mutex_lock(&g_aw_monitor_lock);
	if (g_monitor_dev_num == 0)
		memset(&g_monitor_cfg, 0, sizeof(struct aw_monitor_cfg));
	monitor->monitor_cfg = &g_monitor_cfg;
	g_monitor_dev_num++;
	mutex_unlock(&g_aw_monitor_lock);

	INIT_LIST_HEAD(&monitor->list);
	list_add(&monitor->list, &g_aw_monitor_list);

	INIT_DELAYED_WORK(&monitor->delay_work, aw881xx_monitor_work_func);

	ret = sysfs_create_group(&aw881xx->dev->kobj,
				 &aw881xx_monitor_attr_group);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev,
			"%s error creating sysfs attr files\n", __func__);
		return ret;
	}

	return 0;
}

void aw881xx_monitor_deinit(struct aw881xx_monitor *monitor)
{
	struct aw881xx *aw881xx = container_of(monitor,
			struct aw881xx, monitor);

	mutex_lock(&g_aw_monitor_lock);
	aw881xx_monitor_stop(monitor);
	g_monitor_dev_num--;
	if (g_monitor_dev_num == 0)
		aw881xx_deinit_monitor_profile(monitor);
	mutex_unlock(&g_aw_monitor_lock);

	sysfs_remove_group(&aw881xx->dev->kobj, &aw881xx_monitor_attr_group);
}

void aw881xx_parse_monitor_dt(struct aw881xx_monitor *monitor)
{
	int ret;
	struct aw881xx *aw881xx = container_of(monitor,
				struct aw881xx, monitor);
	struct device_node *np = aw881xx->dev->of_node;

	ret = of_property_read_u32(np, "monitor-flag", &monitor->is_enable);
	if (ret) {
		monitor->is_enable = AW_MONITOR_DFT_FLAG;
		aw_dev_info(aw881xx->dev,
			"%s: monitor-flag get failed ,user default value!\n",
			__func__);
	} else {
		aw_dev_info(aw881xx->dev, "%s: monitor-flag = %d\n",
			__func__, monitor->is_enable);
	}

}

