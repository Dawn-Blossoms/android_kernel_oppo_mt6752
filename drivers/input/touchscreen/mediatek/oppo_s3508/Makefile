include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Linux driver folder
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/oppo_s3508/
ccflags-y += -I$(srctree)/arch/arm/mach-$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/oppo_s3508/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/oppo_s3508/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/

ifeq ($(CONFIG_TOUCHSCREEN_MTK_OPPO_S3508_15021),y)
obj-y	+=  synaptics_s3508_15021.o
endif
ifeq ($(CONFIG_TOUCHSCREEN_MTK_OPPO_S3508_15011),y)
obj-y	+=  synaptics_s3508_15011.o
endif
obj-y	+=  synaptics_redremote.o


