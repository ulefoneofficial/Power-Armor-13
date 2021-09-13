/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef _LENS_LIST_H

#define _LENS_LIST_H

#if 0
#define AK7371AF_SetI2Cclient AK7371AF_SetI2Cclient_Sub2
#define AK7371AF_Ioctl AK7371AF_Ioctl_Sub2
#define AK7371AF_Release AK7371AF_Release_Sub2
#define AK7371AF_PowerDown AK7371AF_PowerDown_Sub2
#define AK7371AF_GetFileName AK7371AF_GetFileName_Sub2
extern int AK7371AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long AK7371AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int AK7371AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int AK7371AF_PowerDown(struct i2c_client *pstAF_I2Cclient,
				int *pAF_Opened);
extern int AK7371AF_GetFileName(unsigned char *pFileName);
#endif
//prize add by lipengpeng 20210511 start 
#define DW9718SAF_SetI2Cclient DW9718SAF_SetI2Cclient_Sub2
#define DW9718SAF_Ioctl DW9718SAF_Ioctl_Sub2
#define DW9718SAF_Release DW9718SAF_Release_Sub2
extern int DW9718SAF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long DW9718SAF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param);
extern int DW9718SAF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int DW9718SAF_GetFileName(unsigned char *pFileName);
//prize add by lipengpeng 20210511 end 

extern void AFRegulatorCtrl(int Stage);
#endif
