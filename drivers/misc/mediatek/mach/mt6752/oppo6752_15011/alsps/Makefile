
include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Module project related part

ifeq ($(CONFIG_MTK_CM36652_NEW),y)
obj-y	+=  cm36652-new/
endif

#ifdef VENDOR_EDIT 	
#ziqing.guo@BasicDrv.Sensor,2015/10/20, add for add alsps
ifeq ($(CONFIG_MTK_CM36686_NEW),y)
obj-y	+=  cm36686-new/
endif

ifeq ($(CONFIG_MTK_APDS9921_NEW),y)
obj-y	+=  apds9921-new/
endif

ifeq ($(CONFIG_MTK_STK3310_SA),y)
obj-y	+=  stk3310-sa/
endif
#endif

#ifdef VENDOR_EDIT
#zhihong.lu@BasicDrv.Sensor,2015/10/20, add for add alsps
ifeq ($(CONFIG_MTK_TMD27723_NEW),y)
obj-y	+=  tmd27723-new/
endif
#endif
