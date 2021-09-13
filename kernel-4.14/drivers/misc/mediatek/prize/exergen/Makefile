subdir-ccflags-y += -Werror
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/prize/exergen/
# In case the platform does NOT support this type of sensors
obj-y   +=  prize_exergen.o
obj-$(CONFIG_EXERGEN_FM78100)   +=  fm78100/