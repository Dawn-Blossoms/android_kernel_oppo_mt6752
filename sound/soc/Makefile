#ifdef  VENDOR_EDIT
#xiang.fei@Multimedia, 2015/10/16, Add for audio*/
ifneq ($(filter "oppo6752_15021",$(CONFIG_ARCH_MTK_PROJECT)),)
KBUILD_CFLAGS +=   -DOPPO_MTK_15021 
KBUILD_CPPFLAGS += -DOPPO_MTK_15021
CFLAGS_KERNEL +=  -DOPPO_MTK_15021
CFLAGS_MODULE +=  -DOPPO_MTK_15021
else ifneq ($(filter "oppo6752_15011",$(CONFIG_ARCH_MTK_PROJECT)),)
KBUILD_CFLAGS +=   -DOPPO_MTK_15011 
KBUILD_CPPFLAGS += -DOPPO_MTK_15011
CFLAGS_KERNEL +=  -DOPPO_MTK_15011
CFLAGS_MODULE +=  -DOPPO_MTK_15011
endif
#endif  
snd-soc-core-objs := soc-core.o soc-dapm.o soc-jack.o soc-cache.o soc-utils.o
snd-soc-core-objs += soc-pcm.o soc-compress.o soc-io.o

ifneq ($(CONFIG_SND_SOC_DMAENGINE_PCM),)
snd-soc-core-objs += soc-dmaengine-pcm.o
endif

ifneq ($(CONFIG_SND_SOC_GENERIC_DMAENGINE_PCM),)
snd-soc-core-objs += soc-generic-dmaengine-pcm.o
endif

obj-$(CONFIG_SND_SOC)	+= snd-soc-core.o
obj-$(CONFIG_SND_SOC)	+= codecs/
obj-$(CONFIG_SND_SOC)	+= generic/
obj-$(CONFIG_SND_SOC)	+= atmel/
obj-$(CONFIG_SND_SOC)	+= au1x/
obj-$(CONFIG_SND_SOC)	+= blackfin/
obj-$(CONFIG_SND_SOC)	+= cirrus/
obj-$(CONFIG_SND_SOC)	+= davinci/
obj-$(CONFIG_SND_SOC)	+= dwc/
obj-$(CONFIG_SND_SOC)	+= fsl/
obj-$(CONFIG_SND_SOC)	+= jz4740/
obj-$(CONFIG_SND_SOC)	+= mid-x86/
obj-$(CONFIG_SND_SOC)	+= mxs/
obj-$(CONFIG_SND_SOC)	+= nuc900/
obj-$(CONFIG_SND_SOC)	+= omap/
obj-$(CONFIG_SND_SOC)	+= kirkwood/
obj-$(CONFIG_SND_SOC)	+= pxa/
obj-$(CONFIG_SND_SOC)	+= samsung/
obj-$(CONFIG_SND_SOC)	+= s6000/
obj-$(CONFIG_SND_SOC)	+= sh/
obj-$(CONFIG_SND_SOC)	+= tegra/
obj-$(CONFIG_SND_SOC)	+= txx9/
obj-$(CONFIG_SND_SOC)	+= ux500/
obj-$(CONFIG_SND_SOC)	+= mediatek/
