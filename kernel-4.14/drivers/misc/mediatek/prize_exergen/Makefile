subdir-ccflags-y += -Werror
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/prize_exergen/
# In case the platform does NOT support this type of sensors
obj-y   +=  prize_exergen.o
PRIZE_EXERGEN_SELECT_LIST := $(subst ",,$(CONFIG_PRIZE_EXERGEN_SELECT_LIST))
$(info, "prize_exergen")
$(info, $(PRIZE_EXERGEN_SELECT_LIST))
obj-y += $(foreach EXERGEN,$(PRIZE_EXERGEN_SELECT_LIST),$(EXERGEN)/)
