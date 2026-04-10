
include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Module project related part

ifeq ($(CONFIG_MTK_MPU6515A),y)
obj-y	+=  mpu6515/
endif

#ifdef VENDOR_EDIT 	
#ziqing.guo@BasicDrv.Sensor,2015/10/19, add for add gsensor lis3dh
ifeq ($(CONFIG_MTK_LIS3DH),y)
obj-y   +=  lis3dh-new/
endif
#endif

