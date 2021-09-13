/*******************************************************************************
 * AW86014AF.c
 *
 * Copyright (c) 2020 AWINIC Technology CO., LTD
 *
 * Author: liangqing <liangqing@awinic.com>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of  the GNU General  Public License as published by the Free
 * Software Foundation; either version 2 of the  License, or (at your option)
 * any later version.
 ******************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/printk.h>
#include <linux/device.h>

#include "lens_info.h"
#include "AW86014AF.h"

#define AW86014_SLAVE_ADDR		0x0c
#define AW86014_DRIVER_VERSION		"v0.1.1"

static struct i2c_client *g_pstAF_I2Cclient;
static int *g_pAF_Opened;
static spinlock_t *g_pAF_SpinLock;

static unsigned long g_u4CurrPosition;
static unsigned long g_u4AF_MACRO = 1023;
static unsigned long g_u4AF_INF;

/*******************************************************************************
 * I2c read/write
 ******************************************************************************/
static int AW86014AF_WriteReg(unsigned char a_uAddr, unsigned char a_uData)
{
	int ret = 0;

	unsigned char puSendCmd[2] = { a_uAddr, a_uData };

	g_pstAF_I2Cclient->addr = AW86014_SLAVE_ADDR;
	ret = i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 2);
	if (ret < 0) {
		AW_LOG_ERR("Send data err, ret = %d\n", ret);
		return ret;
	}

	return 0;
}

static int AW86014AF_ReadReg(unsigned char a_uAddr, unsigned char *a_puData)
{
	int ret = 0;
	char a_uResult;
	unsigned char puSendCmd[1] = { a_uAddr };

	g_pstAF_I2Cclient->addr = AW86014_SLAVE_ADDR;

	ret = i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 1);
	if (ret < 0) {
		AW_LOG_ERR("Send address err, ret = %d\n", ret);
		return ret;
	}

	ret = i2c_master_recv(g_pstAF_I2Cclient, &a_uResult, 1);
	if (ret < 0) {
		AW_LOG_ERR("Recv data err, ret = %d\n", ret);
		return ret;
	}

	*a_puData = a_uResult;

	return 0;
}

static inline void AW86014AF_SetAdvance(void)
{
	AW_LOG_INF("Start\n");

	AW86014AF_WriteReg(0xed, 0xab); /* advanced */

#ifdef AW_ADVANCE_DIRECT
	AW86014AF_WriteReg(0x06, 0x00);
	AW_LOG_INF("Set AW_ADVANCE_DIRECT\n");
#elif defined(AW_ADVANCE_VRC)
	AW86014AF_WriteReg(0x06, AW_ADVANCE_VRC_MODE);
	AW86014AF_WriteReg(0x07, AW_ADVANCE_RPESC);
	AW86014AF_WriteReg(0x08, AW_ADVANCE_VRCT);
	AW_LOG_INF("Set AW_ADVANCE_VRC = 0x%02x\n", AW_ADVANCE_VRC_MODE);
#endif

	AW_LOG_INF("End\n");
}

static inline void AW86014AF_InitDrv(void)
{
	AW_LOG_INF("Start\n");

	if (*g_pAF_Opened == 1) {
		AW86014AF_SetAdvance();

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 2;
		spin_unlock(g_pAF_SpinLock);
	} else if (*g_pAF_Opened == 0) {
		AW_LOG_ERR("Please open device\n");
		return;
	}

	AW_LOG_INF("Init drv OK!\n");
}

static int AW86014AF_SetPos(unsigned long a_u4Position)
{
	int ret = -1;
	unsigned char pos[3] = { 0 };

	AW_LOG_INF("Start\n");

	pos[0] = REG_VCM_MSB;
	pos[1] = (unsigned char)((a_u4Position >> 8) & 0x03);
	pos[2] = (unsigned char)(a_u4Position & 0xff);

	AW_LOG_INF("Target Position = 0x%04lx\n", a_u4Position);

	g_pstAF_I2Cclient->addr = AW86014_SLAVE_ADDR;
	ret = i2c_master_send(g_pstAF_I2Cclient, &pos[0], 3);
	if (ret < 0) {
		AW_LOG_ERR("Set position err, ret = %d\n", ret);
		return ret;
	}

	AW_LOG_INF("End\n");

	return 0;
}

static inline int AW86014AF_MoveVCM(unsigned long a_u4Position)
{
	AW_LOG_INF("Start\n");

	if ((a_u4Position > g_u4AF_MACRO) || (a_u4Position < g_u4AF_INF)) {
		AW_LOG_ERR("Target position out of range\n");
		return -EINVAL;
	}

	if (*g_pAF_Opened == 1) {
		*g_pAF_Opened = 2;
	} else if (*g_pAF_Opened == 0) {
		AW_LOG_ERR("Please open device\n");
		return -EIO;
	}

	AW_LOG_INF("*g_pAF_Opened = %d\n", *g_pAF_Opened);

	if (*g_pAF_Opened == 2) {
		if (AW86014AF_SetPos(a_u4Position) == 0) {
			spin_lock(g_pAF_SpinLock);
			g_u4CurrPosition = a_u4Position;
			spin_unlock(g_pAF_SpinLock);
		} else {
			AW_LOG_ERR("Move vcm error!\n");
			return -EAGAIN;
		}
	}

	AW_LOG_INF("End\n");

	return 0;
}

static inline int AW86014AF_SetMacro(unsigned long a_u4Param)
{
	AW_LOG_INF("Start\n");

	spin_lock(g_pAF_SpinLock);
	g_u4AF_MACRO = a_u4Param;
	spin_unlock(g_pAF_SpinLock);

	AW_LOG_INF("g_u4AF_MACRO = 0x%04lx\n", g_u4AF_MACRO);

	return 0;
}

static inline int AW86014AF_SetInf(unsigned long a_u4Param)
{
	AW_LOG_INF("Start\n");

	spin_lock(g_pAF_SpinLock);
	g_u4AF_INF = a_u4Param;
	spin_unlock(g_pAF_SpinLock);

	AW_LOG_INF("g_u4AF_INF = 0x%04lx\n", g_u4AF_INF);

	return 0;
}

static inline int AW86014AF_GetVCMInfo(
				__user struct stAF_MotorInfo *pstMotorInfo)
{
	struct stAF_MotorInfo stMotorInfo;

	AW_LOG_INF("Start\n");

	stMotorInfo.u4MacroPosition = g_u4AF_MACRO;
	stMotorInfo.u4InfPosition = g_u4AF_INF;
	stMotorInfo.u4CurrentPosition = g_u4CurrPosition;
	stMotorInfo.bIsSupportSR = 1;

	stMotorInfo.bIsMotorMoving = 1;

	if (*g_pAF_Opened >= 1)
		stMotorInfo.bIsMotorOpen = 1;
	else
		stMotorInfo.bIsMotorOpen = 0;

	if (copy_to_user(pstMotorInfo, &stMotorInfo,
						sizeof(struct stAF_MotorInfo)))
		AW_LOG_ERR("Get VCM information error!\n");

	AW_LOG_INF("u4MacroPosition=0x%04x\n", stMotorInfo.u4MacroPosition);
	AW_LOG_INF("u4InfPosition=0x%04x\n", stMotorInfo.u4InfPosition);
	AW_LOG_INF("u4CurrentPosition=0x%04x\n", stMotorInfo.u4CurrentPosition);
	AW_LOG_INF("bIsSupportSR=%d\n", stMotorInfo.bIsSupportSR);
	AW_LOG_INF("bIsMotorMoving=%d\n", stMotorInfo.bIsMotorMoving);
	AW_LOG_INF("bIsMotorOpen=%d\n", stMotorInfo.bIsMotorOpen);

	return 0;
}

long AW86014AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
							unsigned long a_u4Param)
{
	long ret = -1;

	AW_LOG_INF("Start\n");

	switch (a_u4Command) {
	case AFIOC_G_MOTORINFO:
		ret = AW86014AF_GetVCMInfo(
				(__user struct stAF_MotorInfo *) (a_u4Param));
		break;
	case AFIOC_T_MOVETO:
		ret = AW86014AF_MoveVCM(a_u4Param);
		break;
	case AFIOC_T_SETMACROPOS:
		ret = AW86014AF_SetMacro(a_u4Param);
		break;
	case AFIOC_T_SETINFPOS:
		ret = AW86014AF_SetInf(a_u4Param);
		break;
	default:
		AW_LOG_ERR("NO CMD!\n");
		ret = -EPERM;
		break;
	}

	AW_LOG_INF("End\n");

	return ret;
}

int AW86014AF_Release(struct inode *a_pInode, struct file *a_pstFile)
{
	AW_LOG_INF("Start\n");

	if (*g_pAF_Opened == 2) {
		unsigned long pos;

		pos = (unsigned long)((AW_INIT_POS_H << 8) | AW_INIT_POS_L);
		AW86014AF_SetPos(pos);
	}

	if (*g_pAF_Opened) {
		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 0;
		spin_unlock(g_pAF_SpinLock);

		AW_LOG_INF("VCM free\n");
	}

	AW_LOG_INF("End\n");

	return 0;
}
int AW86014AF_PowerDown(struct i2c_client *pstAF_I2Cclient, int *pAF_Opened)
{
	unsigned char data;

	AW_LOG_INF("Start\n");

	g_pstAF_I2Cclient = pstAF_I2Cclient;
	g_pAF_Opened = pAF_Opened;

	if (*g_pAF_Opened > 0)
		*g_pAF_Opened = 0;

	AW86014AF_WriteReg(0xed, 0xab);

	AW86014AF_ReadReg(0x02, &data);
	data |= 0x01;
	AW86014AF_WriteReg(0x02, data); /* enter PD mode, PD = 1; */
	usleep_range(5000, 5500);

	AW_LOG_INF("End\n");

	return 0;
}

int AW86014AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				spinlock_t *pAF_SpinLock, int *pAF_Opened)
{
	AW_LOG_INF("Start\n");

	g_pstAF_I2Cclient = pstAF_I2Cclient;
	g_pAF_SpinLock = pAF_SpinLock;
	g_pAF_Opened = pAF_Opened;

	spin_lock(g_pAF_SpinLock);
	*g_pAF_Opened = 1;
	spin_unlock(g_pAF_SpinLock);

	AW86014AF_InitDrv();

	AW_LOG_INF("End\n");

	return 1;
}

int AW86014AF_GetFileName(unsigned char *pFileName)
{
	#if SUPPORT_GETTING_LENS_FOLDER_NAME
	char FilePath[256];
	char *FileString;

	sprintf(FilePath, "%s", __FILE__);
	FileString = strrchr(FilePath, '/');
	*FileString = '\0';
	FileString = (strrchr(FilePath, '/') + 1);
	strncpy(pFileName, FileString, AF_MOTOR_NAME);
	AW_LOG_INF("FileName : %s\n", pFileName);
	#else
	pFileName[0] = '\0';
	#endif
	return 1;
}

MODULE_AUTHOR("<liangqing@awinic.com>");
MODULE_DESCRIPTION("AW86014AF VCM Driver");
MODULE_LICENSE("GPL v2");

