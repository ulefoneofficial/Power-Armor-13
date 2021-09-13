#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See http://www.gnu.org/licenses/gpl-2.0.html for more details.
#

MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
subdir-ccflags-y += -Werror
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include
subdir-ccflags-$(CONFIG_USB_MU3D_DRV) += -I$(srctree)/drivers/misc/mediatek/mu3d/drv
subdir-ccflags-$(CONFIG_USB_MTK_HDRC) += -I$(srctree)/drivers/misc/mediatek/usb20/
subdir-ccflags-$(CONFIG_MTK_PMIC_CHIP_MT6355) += -I$(srctree)/drivers/misc/mediatek/pmic/include/mt6355
subdir-ccflags-$(CONFIG_MTK_PMIC_CHIP_MT6356) += -I$(srctree)/drivers/misc/mediatek/pmic/include/mt6356
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/typec/tcpc/inc
subdir-ccflags-y += -I$(srctree)/drivers/power/supply/mediatek/misc/
subdir-ccflags-y += -I$(srctree)/drivers/power/supply/mediatek/battery/

obj-y += charger_class.o mtk_charger.o mtk_switch_charging.o mtk_pdc_intf.o\
	 mtk_chg_type_det.o mtk_pe40_intf.o mtk_linear_charging.o\
	 adapter_class.o mtk_pd_adapter.o\
	 mtk_switch_charging2.o mtk_intf.o mtk_pe40.o mtk_pdc.o

obj-$(CONFIG_MTK_DUAL_CHARGER_SUPPORT) += mtk_dual_switch_charging.o
obj-$(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT) += mtk_pe_intf.o
obj-$(CONFIG_MTK_PUMP_EXPRESS_PLUS_20_SUPPORT) += mtk_pe20_intf.o
obj-$(CONFIG_MTK_PUMP_EXPRESS_50_SUPPORT)	+= mtk_pe50_intf.o mtk_pe50.o
obj-$(CONFIG_RT_PROP_CHGALGO) += prop_chgalgo_class.o pca_pps_ta.o \
				 pca_mtk_chg.o pca_dv2_algo.o
obj-$(CONFIG_CHARGER_RT9465) += rt9465.o
obj-$(CONFIG_CHARGER_RT9466) += rt9466.o
obj-$(CONFIG_CHARGER_RT9458) += rt9458.o
obj-$(CONFIG_CHARGER_RT9467) += rt9467.o
obj-$(CONFIG_CHARGER_RT9471) += rt9471.o
obj-$(CONFIG_CHARGER_RT9759) += rt9759.o
#add by liaojie 20210305  for sc8551 30w start
obj-$(CONFIG_CHARGER_SC8551) += sc8551.o
#add by liaojie 20210305  for sc8551 30w end
obj-$(CONFIG_CHARGER_HL7005) += hl7005.o
obj-$(CONFIG_CHARGER_BQ25601) += bq25601.o
obj-$(CONFIG_LOAD_SWITCH_RT9750) += rt9750.o
obj-$(CONFIG_CHARGER_MT6362) += mt6362-chg.o
# begin, prize-sunshuai-20200111, add for wireless power receiver mt5715
obj-$(CONFIG_PRIZE_WIRELESS_RECEIVER_MAXIC_MT5715) += MT5715_wireless_driver.o
# end, prize-sunshuai-20200111, add for wireless power receiver mt5715

#//prize added by huarui, eta6937 support, 20190111-start
obj-$(CONFIG_HL7005ALL_CHARGER_SUPPORT) += hl7005all.o
#//prize added by huarui, eta6937 support, 20190111-end

#//prize added by sunshuai, charge support, 20200507-start
obj-$(CONFIG_PRIZE_MT5725_SUPPORT_15W) += mt5725_wireless_driver-15w.o
#//prize added by sunshuai, charge support, 20200507-end

#//prize added by sunshuai, cvs8035 support, 20200701-start
obj-$(CONFIG_PRIZE_CV8035D_SUPPORT) += cvs8035/
#//prize added by sunshuai, cvs8035 support, 20200701-end

#//prize added by huarui, sgm41512 support, 20190111-start
obj-$(CONFIG_CHARGER_SGM41512) += sgm41512.o
#//prize added by huarui, sgm41512 support, 20190111-end