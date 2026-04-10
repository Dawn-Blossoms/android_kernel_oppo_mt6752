
include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Module project related part

ifeq ($(CONFIG_MTK_YAS532),y)
obj-y	+=  yamaha532/
endif

#ifdef VENDOR_EDIT 	
#ziqing.guo@BasicDrv.Sensor,2015/10/19, add for add msensor
ifeq ($(CONFIG_MTK_MMC3416X),y)
obj-y   +=  mmc3416x-new/
endif
#endif