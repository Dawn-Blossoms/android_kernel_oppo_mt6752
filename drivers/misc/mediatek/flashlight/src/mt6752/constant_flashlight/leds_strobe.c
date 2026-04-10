#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_hw.h"
#include <cust_gpio_usage.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/xlog.h>
#include <linux/version.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
#include <linux/mutex.h>
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif
#endif



/******************************************************************************
 * Debug configuration
******************************************************************************/
// availible parameter
// ANDROID_LOG_ASSERT
// ANDROID_LOG_ERROR
// ANDROID_LOG_WARNING
// ANDROID_LOG_INFO
// ANDROID_LOG_DEBUG
// ANDROID_LOG_VERBOSE
#define TAG_NAME "leds_strobe.c"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_INFO  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_WARN(fmt, arg...)        xlog_printk(ANDROID_LOG_WARNING, TAG_NAME, KERN_WARNING  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_NOTICE(fmt, arg...)      xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_NOTICE  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_INFO(fmt, arg...)        xlog_printk(ANDROID_LOG_INFO   , TAG_NAME, KERN_INFO  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_TRC_FUNC(f)              xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME,  "<%s>\n", __FUNCTION__);
#define PK_TRC_VERBOSE(fmt, arg...) xlog_printk(ANDROID_LOG_VERBOSE, TAG_NAME,  fmt, ##arg)
#define PK_ERROR(fmt, arg...)       xlog_printk(ANDROID_LOG_ERROR  , TAG_NAME, KERN_ERR "%s: " fmt, __FUNCTION__ ,##arg)


#define DEBUG_LEDS_STROBE
#ifdef  DEBUG_LEDS_STROBE
	#define PK_DBG PK_DBG_FUNC
	#define PK_VER PK_TRC_VERBOSE
	#define PK_ERR PK_ERROR
#else
	#define PK_DBG(a,...)
	#define PK_VER(a,...)
	#define PK_ERR(a,...)
#endif

#define FL_GPIO_CONTROL

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock); /* cotta-- SMP proection */


static u32 strobe_Res = 0;
static u32 strobe_Timeus = 0;
static BOOL g_strobe_On = 0;

static int g_duty=-1;
static int g_timeOutTimeMs=0;

//static int g_is_MainSpark = 0;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
static DEFINE_MUTEX(g_strobeSem);
#else
static DECLARE_MUTEX(g_strobeSem);
#endif

static struct work_struct workTimeOut;

extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
static void work_timeOutFunc(struct work_struct *data);

//#define GPIO_CAMERA_FLASH_EN   			GPIO125
//#define GPIO_CAMERA_FLASH_EN_M_GPIO		GPIO_MODE_00

#define GPIO_CAMERA_TORCH_EN			(GPIO118 | 0x80000000)
#define GPIO_CAMERA_TORCH_EN_M_GPIO		GPIO_MODE_00

extern int iWriteStrobeReg(char reg, char data);
extern int iReadStrobeReg(char reg, char * data);

#ifdef FL_GPIO_CONTROL
static unsigned long g_flash_gpio = (GPIO125 | 0x80000000); 
#define GPIO_74  (GPIO74 | 0x80000000)
#define GPIO_75  (GPIO75 | 0x80000000)
#define GPIO_76  (GPIO76 | 0x80000000) 
static void hw_version_identify(void)
{
    int id74;
    int id75;
    int id76;
    int hw_version = 0;

    mt_set_gpio_mode(GPIO_74, 0);
    mt_set_gpio_dir(GPIO_74, GPIO_DIR_IN);
    mt_set_gpio_pull_select(GPIO_74, GPIO_PULL_DOWN);
    mt_set_gpio_pull_enable(GPIO_74, GPIO_PULL_ENABLE);

    mt_set_gpio_mode(GPIO_75, 0);
    mt_set_gpio_dir(GPIO_75, GPIO_DIR_IN);
    mt_set_gpio_pull_select(GPIO_75, GPIO_PULL_DOWN);
    mt_set_gpio_pull_enable(GPIO_75, GPIO_PULL_ENABLE);

    mt_set_gpio_mode(GPIO_76, 0);
    mt_set_gpio_dir(GPIO_76, GPIO_DIR_IN);
    mt_set_gpio_pull_select(GPIO_76, GPIO_PULL_DOWN);
    mt_set_gpio_pull_enable(GPIO_76, GPIO_PULL_ENABLE);

    //MDELAY(100);
    
    id74 = mt_get_gpio_in(GPIO_74);
    id75 = mt_get_gpio_in(GPIO_75);
    id76 = mt_get_gpio_in(GPIO_76);

    hw_version |= (id74&0x01) << 2;
    hw_version |= (id75&0x01) << 1;
    hw_version |= (id76&0x01) << 0;

    if (hw_version == 0) {
        PK_DBG("Flash gpio is GPIO130\n");
        g_flash_gpio = (GPIO130 | 0x80000000);
    } else {
        PK_DBG("Flash gpio is GPIO125\n");
        g_flash_gpio = (GPIO125 | 0x80000000);
    }
}

ssize_t gpio_FL_Strb_Enable(void) 
{
	PK_DBG(" E\n");
	if(mt_set_gpio_mode(g_flash_gpio, GPIO_MODE_00)){PK_DBG(" set gpio mode failed!! \n");}
	if(mt_set_gpio_dir(g_flash_gpio, GPIO_DIR_OUT)){PK_DBG(" set gpio dir failed!! \n");}
	if(mt_set_gpio_out(g_flash_gpio, GPIO_OUT_ONE)){PK_DBG(" set gpio failed!! \n");}
	return 0;
}

ssize_t gpio_FL_Strb_Disable(void) 
{
	PK_DBG(" E\n");
	if(mt_set_gpio_mode(g_flash_gpio, GPIO_MODE_00)){PK_DBG(" set gpio mode failed!! \n");}
	if(mt_set_gpio_dir(g_flash_gpio, GPIO_DIR_OUT)){PK_DBG(" set gpio dir failed!! \n");}
	if(mt_set_gpio_out(g_flash_gpio, GPIO_OUT_ZERO)){PK_DBG(" set gpio failed!! \n");}
	return 0;
}

ssize_t gpio_FL_Torch_Enable(void) 
{
	PK_DBG(" E\n");
	if(mt_set_gpio_mode(GPIO_CAMERA_TORCH_EN, GPIO_CAMERA_TORCH_EN_M_GPIO)){PK_DBG(" set gpio mode failed!! \n");}
	if(mt_set_gpio_dir(GPIO_CAMERA_TORCH_EN, GPIO_DIR_OUT)){PK_DBG(" set gpio dir failed!! \n");}
	if(mt_set_gpio_out(GPIO_CAMERA_TORCH_EN, GPIO_OUT_ONE)){PK_DBG(" set gpio failed!! \n");}
	return 0;
}

ssize_t gpio_FL_Torch_Disable(void) 
{
	PK_DBG(" E\n");
	if(mt_set_gpio_mode(GPIO_CAMERA_TORCH_EN, GPIO_CAMERA_TORCH_EN_M_GPIO)){PK_DBG(" set gpio mode failed!! \n");}
	if(mt_set_gpio_dir(GPIO_CAMERA_TORCH_EN, GPIO_DIR_OUT)){PK_DBG(" set gpio dir failed!! \n");}
	if(mt_set_gpio_out(GPIO_CAMERA_TORCH_EN, GPIO_OUT_ZERO)){PK_DBG(" set gpio failed!! \n");}
	return 0;
}
#endif

int FL_Enable(void)
{
	char reg_value = 0;
	int i = 3;
	PK_DBG("%s g_duty:%d \n", __FUNCTION__, g_duty);
	
#ifdef VENDOR_EDIT
	//LiuBin@Camera, 2015/03/21, Add for clear the error flag
	iReadStrobeReg(0x0B, &reg_value);
    if(reg_value != 0)
    {
       int j=0;
       for(j=0;j<=3;j++)
       	{
       	   iReadStrobeReg(0x0B, &reg_value);
		   if(reg_value == 0)
		   	  break;
		   PK_DBG(" flag 0x%x j=%d\n",reg_value,j);	
       	}
    }

#endif /* VENDOR_EDIT */
    
	if(g_duty==0)
	{
		//torch mode
		#ifndef FL_GPIO_CONTROL
		iWriteStrobeReg(0x09, 0x10);  //torch current 93.74mA
		iWriteStrobeReg(0x0A, 0x02);  ////torch mode enable, disable hardware pin
		#else
		PK_DBG(" turn to 375.74 ma\n");
		gpio_FL_Strb_Disable();
		gpio_FL_Torch_Enable();		
		iWriteStrobeReg(0x09, 0x70);  //torch current 375.74mA
		iWriteStrobeReg(0x0A, 0x12);  ////torch mode enable, disable hardware pin
		for(;i>=0;i--)
		{
			udelay(750);
		}
		iWriteStrobeReg(0x09, 0x10);  //torch current 93.74mA
		PK_DBG(" turn to 93.74 ma\n");
		#endif
	}
	else if (g_duty==12)
	{
		//engine mode torch for test
		#ifndef FL_GPIO_CONTROL
		iWriteStrobeReg(0x09, 0x00);  //torch current 48.4mA
		iWriteStrobeReg(0x0A, 0x02);  ////torch mode enable, disable hardware pin
		#else
		gpio_FL_Strb_Disable();
		gpio_FL_Torch_Enable();
		
		PK_DBG("enginer mode torch current is 48.4mA \n");
		iWriteStrobeReg(0x09, 0x00);  //torch current 48.4mA
		iWriteStrobeReg(0x0A, 0x12);  ////torch mode enable, disable hardware pin
		#endif
	}
	else if (g_duty==13)
	{
		//torch mode for status bar and flashlight in tools
		#ifndef FL_GPIO_CONTROL
		iWriteStrobeReg(0x09, 0x00);  //torch current 48.4mA
		iWriteStrobeReg(0x0A, 0x02);  ////torch mode enable, disable hardware pin
		#else
		PK_DBG(" turn to 375.74 ma\n");
		gpio_FL_Strb_Disable();
		gpio_FL_Torch_Enable();
		iWriteStrobeReg(0x09, 0x70);  //torch current 375.74mA
		iWriteStrobeReg(0x0A, 0x12);  ////torch mode enable, disable hardware pin
		for(;i>=0;i--)
		{
			udelay(750);
		}
		iWriteStrobeReg(0x09, 0x00);  //torch current 48.4mA
		PK_DBG("torch current is 48.4mA, for flashlight of status bar and tools \n");
		#endif
	}
	else
	{
		#ifndef FL_GPIO_CONTROL
		iWriteStrobeReg(0x09, g_duty);  //main flash current 187.5~1031.25mA
		iWriteStrobeReg(0x0A, 0x03);  //flash mode enable
		#else
		gpio_FL_Torch_Disable();
		gpio_FL_Strb_Enable();		
		iWriteStrobeReg(0x09, g_duty);  //main flash current 187.5~1031.25mA
		iWriteStrobeReg(0x0A, 0x23);  //flash mode enable
		PK_DBG("flash strobe g_duty:%d \n", g_duty);
		#endif
	}

	PK_DBG(" flag 0x%x line=%d\n",reg_value,__LINE__);	
    return 0;
}

int FL_Disable(void)
{
	char reg_value = 0;
	int i = 3;

	if (g_duty == 0)
	{
		PK_DBG("set current to 375.74mA \n");
		iWriteStrobeReg(0x09, 0x70);  //torch current 375.74mA
	}
	iWriteStrobeReg(0x0A, 0x00);

	#ifdef FL_GPIO_CONTROL
	gpio_FL_Strb_Disable();
	gpio_FL_Torch_Disable();
	#endif

	if (g_duty == 0)
	{
		PK_DBG("torch disable not engine mode \n");
		for(;i>=0;i--)
		{
			udelay(750);
		}
	}
#ifdef VENDOR_EDIT
//LiuBin@Camera, 2015/03/21, Add for clear the error flag
	iReadStrobeReg(0x0B, &reg_value);
#endif /* VENDOR_EDIT */
	PK_DBG(" flag 0X%x line=%d\n",reg_value,__LINE__);
    return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
	PK_DBG(" duty = %d \n", duty);
    g_duty = duty;
    return 0;
}

int FL_Init(void)
{
	char reg_value = 0;
	
	#ifdef FL_GPIO_CONTROL
	hw_version_identify();
	gpio_FL_Strb_Disable();
	gpio_FL_Torch_Disable();
	#endif
	
    iWriteStrobeReg(0x0A, 0x00);
    iWriteStrobeReg(0x08, 0x47);

#ifdef VENDOR_EDIT
//LiuBin@Camera, 2015/03/21, Add for clear the error flag
    iReadStrobeReg(0x0B, &reg_value);
#endif /* VENDOR_EDIT */

    INIT_WORK(&workTimeOut, work_timeOutFunc);
    PK_DBG(" line=%d\n",__LINE__);
    return 0;
}

int FL_Uninit(void)
{
	FL_Disable();

	#ifdef FL_GPIO_CONTROL
	g_flash_gpio = (GPIO125 | 0x80000000);
	#endif
	PK_DBG(" line=%d\n",__LINE__);
    return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
    FL_Disable();
    PK_DBG("ledTimeOut_callback\n");
    //printk(KERN_ALERT "work handler function./n");
}

enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
    schedule_work(&workTimeOut);
    return HRTIMER_NORESTART;
}
static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
	static int init_flag;
	if (init_flag==0){
		init_flag=1;
		INIT_WORK(&workTimeOut, work_timeOutFunc);
		g_timeOutTimeMs=1000; //1s
		hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
		g_timeOutTimer.function=ledTimeOutCallback;
	}
}

static int constant_flashlight_ioctl(MUINT32 cmd, MUINT32 arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	char value;
	char* ret;
	ret = &value;
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
	PK_DBG(" line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",__LINE__, ior_shift, iow_shift, iowr_shift, arg);
    switch(cmd)
    {

		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",arg);
			g_timeOutTimeMs=arg;
		break;

    	case FLASH_IOC_SET_DUTY :
    		PK_DBG("FLASHLIGHT_DUTY: %d\n",arg);
    		FL_dim_duty(arg);
    		break;


    	case FLASH_IOC_SET_STEP:
    		PK_DBG("FLASH_IOC_SET_STEP: %d\n",arg);

    		break;

    	case FLASH_IOC_SET_ONOFF :
    		PK_DBG("FLASHLIGHT_ONOFF: %d\n",arg);
    		if(arg==1)
    		{
				if(g_timeOutTimeMs!=0)
	            {
	            	ktime_t ktime;
					ktime = ktime_set( 0, g_timeOutTimeMs*1000000 );
					hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL );
	            }
    			FL_Enable();
    		}
    		else
    		{
    			FL_Disable();
				hrtimer_cancel( &g_timeOutTimer );
    		}
    		break;
		#ifdef VENDOR_EDIT
		//LiuBin@Camera, 2015/03/21, Add for clear flash IC error flag
		case FLASH_IOC_READ_ERROR_FLAG_REG:
			PK_DBG("clear flash IC error flag \n");
			iReadStrobeReg(0x0B, &value);
			break;
    	#endif /* VENDOR_EDIT */
	default :
    		PK_DBG(" No such command \n");
    		i4RetValue = -EPERM;
    		break;
    }
    return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
    int i4RetValue = 0;
    PK_DBG(" line=%d\n", __LINE__);

	if (0 == strobe_Res)
	{
		FL_Init();
		timerInit();
	}
	PK_DBG(" line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


    if(strobe_Res)
    {
        PK_ERR(" busy!\n");
        i4RetValue = -EBUSY;
    }
    else
    {
        strobe_Res += 1;
    }


    spin_unlock_irq(&g_strobeSMPLock);
    PK_DBG(" line=%d\n", __LINE__);

    return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
    PK_DBG(" E \n");

    if (strobe_Res)
    {
        spin_lock_irq(&g_strobeSMPLock);

        strobe_Res = 0;
        strobe_Timeus = 0;

        /* LED On Status */
        g_strobe_On = FALSE;

        spin_unlock_irq(&g_strobeSMPLock);

    	FL_Uninit();
    }

    PK_DBG(" Done\n");

    return 0;

}


FLASHLIGHT_FUNCTION_STRUCT	constantFlashlightFunc=
{
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc != NULL)
    {
        *pfFunc = &constantFlashlightFunc;
    }
    return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

    return 0;
}

EXPORT_SYMBOL(strobe_VDIrq);

