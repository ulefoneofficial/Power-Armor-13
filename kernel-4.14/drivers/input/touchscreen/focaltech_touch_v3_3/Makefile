# Makefile for the focaltech touchscreen drivers.

ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/$(CONFIG_TOUCHSCREEN_MTK_FTS_DIRECTORY)
ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
ccflags-y += -I$(srctree)/drivers/base
ccflags-y += -I$(srctree)/drivers/misc/mediatek/hwmon/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/alsps/inc/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/alsps/inc/

obj-$(CONFIG_TOUCHSCREEN_FTS_V3_3)	+=  focaltech_core.o
obj-$(CONFIG_TOUCHSCREEN_FTS_V3_3)	+=  focaltech_ex_fun.o
obj-$(CONFIG_TOUCHSCREEN_FTS_V3_3)	+=  focaltech_ex_mode.o
obj-$(CONFIG_TOUCHSCREEN_FTS_V3_3)	+=  focaltech_gesture.o
obj-$(CONFIG_TOUCHSCREEN_FTS_V3_3)	+=  focaltech_esdcheck.o
obj-$(CONFIG_TOUCHSCREEN_FTS_V3_3)   +=  focaltech_point_report_check.o
obj-$(CONFIG_TOUCHSCREEN_FTS_V3_3)   +=  focaltech_proximity.o
obj-$(CONFIG_TOUCHSCREEN_FTS_V3_3)	+=  focaltech_flash.o
obj-$(CONFIG_TOUCHSCREEN_FTS_V3_3)   +=  focaltech_spi.o

obj-y   +=  mtk_tpd.o
obj-y   +=  tpd_button.o
obj-y   +=  tpd_calibrate.o
obj-y   +=  tpd_debug.o
obj-y   +=  tpd_default.o
obj-y   +=  tpd_misc.o
obj-y   +=  tpd_setting.o