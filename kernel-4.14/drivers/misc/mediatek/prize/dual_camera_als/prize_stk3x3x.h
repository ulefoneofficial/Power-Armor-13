/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*
 * Definitions for stk3x3x als/ps sensor chip.
 */
#ifndef __STK3X3X_H__
#define __STK3X3X_H__

/*ALSPS REGS*/
#define STK_STATE_REG                   0x00
#define STK_PSCTRL_REG                  0x01
#define STK_ALSCTRL_REG                 0x02
#define STK_LEDCTRL_REG                 0x03
#define STK_INTCTRL1_REG                0x04
#define STK_WAIT_REG                    0x05
#define STK_THDH1_PS_REG                0x06
#define STK_THDH2_PS_REG                0x07
#define STK_THDL1_PS_REG                0x08
#define STK_THDL2_PS_REG                0x09
#define STK_THDH1_ALS_REG               0x0A
#define STK_THDH2_ALS_REG               0x0B
#define STK_THDL1_ALS_REG               0x0C
#define STK_THDL2_ALS_REG               0x0D
#define STK_FLAG_REG                    0x10
#define STK_DATA1_PS_REG                0x11
#define STK_DATA2_PS_REG                0x12
#define STK_DATA1_ALS_REG               0x13
#define STK_DATA2_ALS_REG               0x14
#define STK_DATA1_C                     0x1B
#define STK_DATA2_C                     0x1C
#define STK_DATA1_PS_OFF_REG            0x1D
#define STK_DATA2_PS_OFF_REG            0x1E
#define STK_PDT_ID_REG                  0x3E
#define STK_RESERVED_REG                0x3F
#define STK_ALSCTRL2_REG                0x4E
#define STK_SW_RESET_REG                0x80
#define STK_SEG_CTRL					0x81
#define STK_SYS_CTRL1                   0xA0
#define STK_PDCTRL_REG                  0xA1
#define STK_INT_CTRL2_REG               0xA5
#define STK_THD_BGIR_REG                0xAA
#define STK_AGCTRL_REG                  0xDB

/********************************************* 
* Register control                           *
*********************************************/

/* Define state reg */
#define STK_STATE_EN_WAIT_SHIFT                         2
#define STK_STATE_EN_ALS_SHIFT                          1
#define STK_STATE_EN_PS_SHIFT                           0
#define STK_STATE_EN_WAIT_MASK                          0x04
#define STK_STATE_EN_ALS_MASK                           0x02
#define STK_STATE_EN_PS_MASK                            0x01

/* Define PS ctrl reg */
#define STK_PS_PRST_SHIFT               6
#define STK_PS_GAIN_SHIFT               4
#define STK_PS_IT_SHIFT                 0

#define STK_PS_PRS_MASK                 0xC0
#define STK_PS_GAIN_MASK                0x30
#define STK_PS_IT_MASK                  0x0F

/* Define ALS ctrl-1 reg */
#define STK_ALS_PRST_SHIFT              6
#define STK_ALS_GAIN_SHIFT              4
#define STK_ALS_IT_SHIFT                0

#define STK_ALS_PRS_MASK                0xC0
#define STK_ALS_GAIN_MASK               0x30
#define STK_ALS_IT_MASK                 0x0F

/* Define LED ctrl reg */
#define STK_LED_IRDR_SHIFT              5
#define STK_LED_IRDR_MASK               0xE0

/* Define interrupt-1 reg */
#define STK_INT_CTRL_SHIFT              7
#define STK_INT_INVALID_PS_SHIFT        5
#define STK_INT_ALS_SHIFT               3
#define STK_INT_PS_SHIFT                0

#define STK_INT_CTRL_MASK               0x80
#define STK_INT_INVALID_PS_MASK         0x20
#define STK_INT_ALS_MASK                0x08
#define STK_INT_PS_MASK                 0x07

#define STK_INT_PS_MODE1                0x01
#define STK_INT_PS_MODE3                0x03
#define STK_INT_PS_MODE4                0x04
#define STK_INT_ALS                     0x08

/* Define flag reg */
#define STK_FLG_ALSDR_SHIFT             7
#define STK_FLG_PSDR_SHIFT              6
#define STK_FLG_ALSINT_SHIFT            5
#define STK_FLG_PSINT_SHIFT             4
#define STK_FLG_ALSSAT_SHIFT            2
#define STK_FLG_INVALID_PS_SHIFT        1
#define STK_FLG_NF_SHIFT                0

#define STK_FLG_ALSDR_MASK              0x80
#define STK_FLG_PSDR_MASK               0x40
#define STK_FLG_ALSINT_MASK             0x20
#define STK_FLG_PSINT_MASK              0x10
#define STK_FLG_ALSSAR_MASK             0x04
#define STK_FLG_INVALID_PS_MASK         0x02
#define STK_FLG_NF_MASK                 0x01

/* Define alsctrl2 reg */
#define STK_GAIN_C_SHIFT                4
#define STK_GAIN_C_MASK                 0x30

/* Define pdctrl reg */
#define STK_PDCTRL_ALS_SEL_SHIFT        4
#define STK_PDCTRL_PS_SEL_SHIFT         0
#define STK_PDCTRL_ALS_SEL_MASK         0x70
#define STK_PDCTRL_PS_SEL_MASK          0x0F

/* Define interrupt-2 reg */
#define STK_INT_ALS_DR_SHIFT            1
#define STK_INT_PS_DR_SHIFT             0
#define STK_INT_ALS_DR_MASK             0x02
#define STK_INT_PS_DR_MASK              0x01

/* Define THD BGIR reg */
#define STK_THD_BGIR_SHIFT              0
#define STK_THD_BGIR_MASK               0x7F

/* Define AGCTRL reg */
#define STK_AGCTRL_PS_SHIFT             0
#define STK_AGCTRL_PS_MASK              0x03

/*********************************************
* Parameters control                         *
*********************************************/
#define STK_PS_PRS1                     0x00
#define STK_PS_PRS2                     0x01
#define STK_PS_PRS4                     0x02
#define STK_PS_PRS16                    0x03
#define STK_PS_GAIN1                    0x00
#define STK_PS_GAIN2                    0x01
#define STK_PS_GAIN4                    0x02
#define STK_PS_GAIN8                    0x03
#define STK_PS_IT100                    0x00
#define STK_PS_IT200                    0x01
#define STK_PS_IT400                    0x02
#define STK_PS_IT800                    0x03
#define STK_PS_IT1600                   0x04
#define STK_PS_IT3000                   0x05
#define STK_PS_IT6200                   0x06

#define STK_ALS_PRS1                    0x00
#define STK_ALS_PRS2                    0x01
#define STK_ALS_PRS4                    0x02
#define STK_ALS_PRS8                    0x03
#define STK_ALS_GAIN1                   0x00
#define STK_ALS_GAIN4                   0x01
#define STK_ALS_GAIN16                  0x02
#define STK_ALS_GAIN64                  0x03
#define STK_ALS_IT25                    0x00
#define STK_ALS_IT50                    0x01
#define STK_ALS_IT100                   0x02
#define STK_ALS_IT200                   0x03
#define STK_ALS_IT400                   0x04
#define STK_ALS_IT800                   0x05
#define STK_ALS_IT1600                  0x06

#define STK_IRDR_LED_3_125mA            0x00
#define STK_IRDR_LED_6_25mA             0x01
#define STK_IRDR_LED_12_5mA             0x02
#define STK_IRDR_LED_25mA               0x03
#define STK_IRDR_LED_50mA               0x04
#define STK_IRDR_LED_100mA              0x05
#define STK_IRDR_LED_150mA              0x06

#define STK_WAIT_20                     0x0C
#define STK_WAIT_50                     0x20
#define STK_WAIT_100                    0x40

#define STK_C_GAIN1                     0x00
#define STK_C_GAIN4                     0x01
#define STK_C_GAIN16                    0x02
#define STK_C_GAIN64                    0x03

/* Register  default Value define */
#define STK_PSCTRL_0X_64X               0x32
#define STK_ALSCTRL_0X_64X              0x31
#define STK_WAIT_50MS                   0x20
#define STK_WAIT_RST                    0x7F
#define STK_INT_CTRL                    0x01
#define STK_INT_CTRL2                   0x00
#define STK_SPEC1_ADDR                  0x10
#define STK_SPEC2_ADDR                  0x64

#endif
