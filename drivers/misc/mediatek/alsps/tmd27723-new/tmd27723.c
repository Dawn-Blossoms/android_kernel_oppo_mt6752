/* 
 * Author: yucong xiong <yucong.xion@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>

#define POWER_NONE_MACRO MT65XX_POWER_NONE

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps.h>
#include "tmd27723.h"
#include <linux/sched.h>
#include <alsps.h>
#include <linux/batch.h>
#include <linux/oppo_devices_list.h>

#ifdef CUSTOM_KERNEL_SENSORHUB
#include <SCP_sensorHub.h>
#endif
/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/

#define TMD2772_DEV_NAME     "TMD2772"
/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               printk(KERN_INFO 	APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  	APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(KERN_ERR	APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_INFO 	APS_TAG fmt, ##args)    

#define I2C_FLAG_WRITE	0
#define I2C_FLAG_READ	1

#ifdef VENDOR_EDIT 	
//ziqing.guo@BasicDrv.Sensor,2015/04/12, add for PSD
static DECLARE_WAIT_QUEUE_HEAD(enable_ps);
#endif
static int set_psensor_threshold(struct i2c_client *client);
long TMD2772_read_ps(struct i2c_client *client, u16 *data);

/******************************************************************************
 * extern functions
*******************************************************************************/
#ifdef CUST_EINT_ALS_TYPE
extern void mt_eint_mask(unsigned int eint_num);
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
extern void mt_eint_print_status(void);
#else
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt65xx_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
#endif
/*----------------------------------------------------------------------------*/
static int TMD2772_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int TMD2772_i2c_remove(struct i2c_client *client);
static int TMD2772_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int TMD2772_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int TMD2772_i2c_resume(struct i2c_client *client);

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id TMD2772_i2c_id[] = {{TMD2772_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_TMD2772={ I2C_BOARD_INFO(TMD2772_DEV_NAME, 0x39)};
static unsigned long long int_top_time = 0;
extern ALSPS_DEV alsps_dev;
/*----------------------------------------------------------------------------*/
struct TMD2772_priv {
	struct alsps_hw  *hw;
	struct i2c_client *client;
	struct work_struct	eint_work;
#ifdef CUSTOM_KERNEL_SENSORHUB
    struct work_struct init_done_work;
#endif

	/*misc*/
	u16 		als_modulus;
	atomic_t	i2c_retry;
	atomic_t	als_suspend;
	atomic_t	als_debounce;	/*debounce time after enabling als*/
	atomic_t	als_deb_on; 	/*indicates if the debounce is on*/
	atomic_t	als_deb_end;	/*the jiffies representing the end of debounce*/
	atomic_t	ps_mask;		/*mask ps: always return far away*/
	atomic_t	ps_debounce;	/*debounce time after enabling ps*/
	atomic_t	ps_deb_on;		/*indicates if the debounce is on*/
	atomic_t	ps_deb_end; 	/*the jiffies representing the end of debounce*/
	atomic_t	ps_suspend;
	atomic_t 	trace;
	
	
	/*data*/
	u16			als;
	u16 		ps;
	u8			_align;
	u16			als_level_num;
	u16			als_value_num;
	u32			als_level[C_CUST_ALS_LEVEL-1];
	u32			als_value[C_CUST_ALS_LEVEL];
	int			ps_cali;
	
	atomic_t	als_cmd_val;	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_cmd_val; 	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_thd_val_high;	 /*the cmd value can't be read, stored in ram*/
	atomic_t	ps_thd_val_low; 	/*the cmd value can't be read, stored in ram*/
	atomic_t	als_thd_val_high;	 /*the cmd value can't be read, stored in ram*/
	atomic_t	als_thd_val_low; 	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_thd_val;
	ulong		enable; 		/*enable mask*/
	ulong		pending_intr;	/*pending interrupt*/
	
	/*early suspend*/
	#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend	early_drv;
	#endif     
};

static int TMD2772_get_ps_value(struct TMD2772_priv *obj, u16 ps);

/*----------------------------------------------------------------------------*/

static struct i2c_driver TMD2772_i2c_driver = {	
	.probe      = TMD2772_i2c_probe,
	.remove     = TMD2772_i2c_remove,
	.detect     = TMD2772_i2c_detect,
	.suspend    = TMD2772_i2c_suspend,
	.resume     = TMD2772_i2c_resume,
	.id_table   = TMD2772_i2c_id,
	.driver = {
		.name = TMD2772_DEV_NAME,
	},
};

/*----------------------------------------------------------------------------*/
struct PS_CALI_DATA_STRUCT
{
	int close;
	int far_away;
	int valid;
};

/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static struct i2c_client *TMD2772_i2c_client = NULL;
static struct TMD2772_priv *TMD2772_obj = NULL;
//static struct PS_CALI_DATA_STRUCT ps_cali={0,0,0};
static int intr_flag = 0;
static int TMD2772_local_init(void);
static int TMD2772_remove(void);
static int TMD2772_init_flag =-1; // 0<==>OK -1 <==> fail
static struct alsps_init_info TMD2772_init_info = {
		.name = "TMD2772",
		.init = TMD2772_local_init,
		.uninit = TMD2772_remove,
	
};

/*----------------------------------------------------------------------------*/

static DEFINE_MUTEX(TMD2772_mutex);

static struct PS_CALI_DATA_STRUCT ps_cali={0,0,0};
static int intr_flag_value = 0;
//static int prox_last_value = 0;


/*----------------------------------------------------------------------------*/
typedef enum {
	CMC_BIT_ALS    = 1,
	CMC_BIT_PS	   = 2,
}CMC_BIT;
/*-----------------------------CMC for debugging-------------------------------*/
typedef enum {
    CMC_TRC_ALS_DATA= 0x0001,
    CMC_TRC_PS_DATA = 0x0002,
    CMC_TRC_EINT    = 0x0004,
    CMC_TRC_IOCTL   = 0x0008,
    CMC_TRC_I2C     = 0x0010,
    CMC_TRC_CVT_ALS = 0x0020,
    CMC_TRC_CVT_PS  = 0x0040,
    CMC_TRC_DEBUG   = 0x8000,
} CMC_TRC;
/*-----------------------------------------------------------------------------*/

int TMD2772_i2c_master_operate(struct i2c_client *client, const char *buf, int count, int i2c_flag)
{
	int res = 0;
	mutex_lock(&TMD2772_mutex);
	switch(i2c_flag){	
	case I2C_FLAG_WRITE:
	client->addr &=I2C_MASK_FLAG;
	res = i2c_master_send(client, buf, count);
	client->addr &=I2C_MASK_FLAG;
	break;
	
	case I2C_FLAG_READ:
	client->addr &=I2C_MASK_FLAG;
	client->addr |=I2C_WR_FLAG;
	client->addr |=I2C_RS_FLAG;
	res = i2c_master_send(client, buf, count);
	client->addr &=I2C_MASK_FLAG;
	break;
	default:
	APS_LOG("TMD2772_i2c_master_operate i2c_flag command not support!\n");
	break;
	}
	if(res < 0)
	{
		goto EXIT_ERR;
	}
	mutex_unlock(&TMD2772_mutex);
	return res;
	EXIT_ERR:
	mutex_unlock(&TMD2772_mutex);
	APS_ERR("TMD2772_i2c_master_operate fail\n");
	return res;
}
/*----------------------------------------------------------------------------*/
static void TMD2772_power(struct alsps_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;

	APS_LOG("power %s\n", on ? "on" : "off");

	if(hw->power_id != POWER_NONE_MACRO)
	{
		if(power_on == on)
		{
			APS_LOG("ignore power control: %d\n", on);
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "TMD2772")) 
			{
				APS_ERR("power on fails!!\n");
			}
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "TMD2772")) 
			{
				APS_ERR("power off fail!!\n");   
			}
		}
	}
	power_on = on;
}
/********************************************************************/
int TMD2772_enable_ps(struct i2c_client *client, int enable)
{
	struct TMD2772_priv *obj = i2c_get_clientdata(client);
	int res;
	u8 databuf[3];
	int ps_value;
	
	databuf[0]= TMD2772_CMM_ENABLE;
	res = TMD2772_i2c_master_operate(client, databuf, 0x101, I2C_FLAG_READ);
	if(res < 0)
	{
		goto EXIT_ERR;
	}
	
	APS_LOG("TMD2772_CMM_ENABLE ps value = %x\n",databuf[0]);
	
	if(enable)
		{
			databuf[1] = databuf[0]|0x05;
			databuf[0] = TMD2772_CMM_ENABLE;
			APS_LOG("TMD2772_CMM_ENABLE enable ps value = %x\n",databuf[1]);	
			res = TMD2772_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res < 0)
			{
				goto EXIT_ERR;
			}
			atomic_set(&obj->ps_deb_on, 1);
			atomic_set(&obj->ps_deb_end, jiffies+atomic_read(&obj->ps_debounce)/(1000/HZ));
			set_bit(CMC_BIT_PS, &obj->enable);
			//tmd2772_set_ps_threshold(client, low, high);//set thd.
			msleep(120);
			TMD2772_read_ps(client, &obj->ps);
			ps_value = TMD2772_get_ps_value(obj, obj->ps);
			ps_report_interrupt_data(ps_value);
		#ifdef VENDOR_EDIT 	
		//ziqing.guo@BasicDrv.Sensor,2015/04/21, modify for QT 621670
      		 	mt_eint_unmask(CUST_EINT_ALS_NUM);
		#endif
			#ifdef VENDOR_EDIT 	
			//ziqing.guo@BasicDrv.Sensor,2015/04/12, add for PSD
			wake_up(&enable_ps);
			#endif
		}
	else{
		if(test_bit(CMC_BIT_ALS, &obj->enable))
			databuf[1] = databuf[0]&0xFB;
		else
			databuf[1] = databuf[0]&0xF8;
		
			databuf[0] = TMD2772_CMM_ENABLE;
			APS_LOG("TMD2772_CMM_ENABLE disable ps value = %x\n",databuf[1]);	
			res = TMD2772_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res < 0)
			{
				goto EXIT_ERR;
			}
			APS_LOG("%s  low:%d high:%d ",__func__, atomic_read(&obj->ps_thd_val_low), atomic_read(&obj->ps_thd_val_high));		
			
			clear_bit(CMC_BIT_PS, &obj->enable);
		#ifdef VENDOR_EDIT 	
		//ziqing.guo@BasicDrv.Sensor,2015/04/21, modify for QT 621670
      		 	mt_eint_mask(CUST_EINT_ALS_NUM);
		#endif
		}
	return 0;
	
EXIT_ERR:
	APS_ERR("TMD2772_enable_ps fail\n");
	return res;
}
/********************************************************************/
int TMD2772_enable_als(struct i2c_client *client, int enable)
{
	struct TMD2772_priv *obj = i2c_get_clientdata(client);
	int res;
	u8 databuf[3];

	databuf[0]= TMD2772_CMM_ENABLE;
	res = TMD2772_i2c_master_operate(client, databuf, 0x101, I2C_FLAG_READ);
	if(res < 0)
	{
		goto EXIT_ERR;
	}
	APS_LOG("TMD2772_CMM_ENABLE als value = %x\n",databuf[0]);
	
	if(enable)
		{
			databuf[1] = databuf[0]|0x03;
			databuf[0] = TMD2772_CMM_ENABLE;
			APS_LOG("TMD2772_CMM_ENABLE enable als value = %x\n",databuf[1]);
			res = TMD2772_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res < 0)
			{
				goto EXIT_ERR;
			}
			atomic_set(&obj->als_deb_on, 1);
			atomic_set(&obj->als_deb_end, jiffies+atomic_read(&obj->als_debounce)/(1000/HZ));
			set_bit(CMC_BIT_ALS, &obj->enable);
		}
	else {
		if(test_bit(CMC_BIT_PS, &obj->enable))
			databuf[1] = databuf[0]&0xFD;
		else
			databuf[1] = databuf[0]&0xF8;
		
			databuf[0] = TMD2772_CMM_ENABLE;
			APS_LOG("TMD2772_CMM_ENABLE disable als value = %x\n",databuf[1]);
			res = TMD2772_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res < 0)
			{
				goto EXIT_ERR;
			}
			clear_bit(CMC_BIT_ALS, &obj->enable);
		}
	return 0;
		
EXIT_ERR:
	APS_ERR("TMD2772_enable_als fail\n");
	return res;
}

/*----------------------------------------------------------------------------*/
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
static int TMD2772_check_and_clear_intr(struct i2c_client *client) 
{
	int res,intp,intl;
	u8 buffer[2];

	if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/  
	    return 0;

	buffer[0] = TMD2772_CMM_STATUS;
	res = TMD2772_i2c_master_operate(client, buffer, 0x101, I2C_FLAG_READ);
	if(res < 0)
	{
		goto EXIT_ERR;
	}
	
	res = 0;
	intp = 0;
	intl = 0;
	if(0 != (buffer[0] & 0x20))
	{
		res = 1;
		intp = 1;
	}
	if(0 != (buffer[0] & 0x10))
	{
		res = 1;
		intl = 1;		
	}

	if(1 == res)
	{
		if((1 == intp) && (0 == intl))
		{
			buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x05);
		}
		else if((0 == intp) && (1 == intl))
		{
			buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x06);
		}
		else
		{
			buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x07);
		}

		res = TMD2772_i2c_master_operate(client, buffer, 0x1, I2C_FLAG_WRITE);
		if(res < 0)
		{
			goto EXIT_ERR;
		}
		else
		{
			res = 0;
		}
	}

	return res;

EXIT_ERR:
	APS_ERR("TMD2772_check_and_clear_intr fail\n");
	return 1;
}

/********************************************************************/
long TMD2772_read_ps(struct i2c_client *client, u16 *data)
{
	long res;
	u8 databuf[2];
	struct TMD2772_priv *obj = i2c_get_clientdata(client);
#if 0//def CUSTOM_KERNEL_SENSORHUB
    SCP_SENSOR_HUB_DATA req;
    SCP_SENSOR_HUB_DATA_P pRsp = &req;
    TMD2772_CUST_DATA *pCustData;
    int len;
#endif

#if 0//def CUSTOM_KERNEL_SENSORHUB
    req.get_data_req.sensorType = ID_PROXIMITY;
    req.get_data_req.action = SENSOR_HUB_SET_CUST;
    
    pCustData = (TMD2772_CUST_DATA *)(&req.set_cust_req.custData);

    pCustData->getPSRawData.action = TMD2772_CUST_ACTION_GET_PS_RAW_DATA;
    len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(pCustData->getPSRawData);
    
    res = SCP_sensorHub_req_send(&req, &len, 1);
    if (0 == res)
    {
        if (len != (offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(pCustData->getPSRawData)) ||
            SENSOR_HUB_SET_CUST != pRsp->rsp.action || 0 != pRsp->rsp.errCode)
        {
            APS_ERR("SCP_sensorHub_req_send failed!\n");
            goto READ_PS_EXIT_ERR;
        }

        pCustData = (TMD2772_CUST_DATA *)(&pRsp->set_cust_rsp.custData);

        if (TMD2772_CUST_ACTION_GET_PS_RAW_DATA != pCustData->getPSRawData.action)
        {
            APS_ERR("SCP_sensorHub_req_send failed!\n");
            goto READ_PS_EXIT_ERR;
        }

        databuf[0] = pCustData->getPSRawData.ps;
    }
    else
    {
        APS_ERR("SCP_sensorHub_req_send failed!\n");
    }
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	databuf[0]=TMD2772_CMM_PDATA_L;
	res = TMD2772_i2c_master_operate(client, databuf, 0x201, I2C_FLAG_READ);
	if(res < 0)
	{
		goto READ_PS_EXIT_ERR;
	}

#endif //#ifdef CUSTOM_KERNEL_SENSORHUB
	
	APS_LOG("TMD2772_read_ps ps_data=%d, low:%d  high:%d", *data, databuf[0], databuf[1]);
	if((databuf[0] | (databuf[1]<<8)) < obj->ps_cali)
		*data = 0;
	else
		*data = (databuf[0] | (databuf[1]<<8)) - obj->ps_cali;

	APS_LOG("ps_data=%d\n", *data);
	
	return 0;	
	READ_PS_EXIT_ERR:
	return res;
}
/********************************************************************/
long TMD2772_read_als(struct i2c_client *client, u16 *data)
{
#if 0//def CUSTOM_KERNEL_SENSORHUB
    SCP_SENSOR_HUB_DATA req;
    SCP_SENSOR_HUB_DATA_P pRsp = &req;
    TMD2772_CUST_DATA *pCustData;
    int len;
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	u8 databuf[2];
	struct TMD2772_priv *obj = i2c_get_clientdata(client);
	u16 c0_value, c1_value;	 
	u32 c0_nf, c1_nf;
	//u16 atio;
	u32 atio = 0;
	int res = 0;
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

#if 0//def CUSTOM_KERNEL_SENSORHUB
    req.get_data_req.sensorType = ID_LIGHT;
    req.get_data_req.action = SENSOR_HUB_SET_CUST;
    
    pCustData = (TMD2772_CUST_DATA *)(&req.set_cust_req.custData);

    pCustData->getALSRawData.action = TMD2772_CUST_ACTION_GET_ALS_RAW_DATA;
    len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(pCustData->getALSRawData);
    
    res = SCP_sensorHub_req_send(&req, &len, 1);
    if (0 == res)
    {
        if (len != (offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(pCustData->getALSRawData)) ||
            SENSOR_HUB_SET_CUST != pRsp->rsp.action || 0 != pRsp->rsp.errCode)
        {
            APS_ERR("SCP_sensorHub_req_send failed!\n");
            goto READ_ALS_EXIT_ERR;
        }

        pCustData = (TMD2772_CUST_DATA *)(&pRsp->set_cust_rsp.custData);

        if (TMD2772_CUST_ACTION_GET_ALS_RAW_DATA != pCustData->getALSRawData.action)
        {
            APS_ERR("SCP_sensorHub_req_send failed!\n");
            goto READ_ALS_EXIT_ERR;
        }

        *data = pCustData->getALSRawData.als;
    }
    else
    {
        APS_ERR("SCP_sensorHub_req_send failed!\n");
    }
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	databuf[0]=TMD2772_CMM_C0DATA_L;
	res = TMD2772_i2c_master_operate(client, databuf, 0x201, I2C_FLAG_READ);
	if(res < 0)
	{
		goto READ_ALS_EXIT_ERR;
	}
	
	c0_value = (databuf[0] | (databuf[1]<<8));
	
	c0_nf = obj->als_modulus*c0_value/1000;
	APS_LOG("c0_value=%d, c0_nf=%d, als_modulus=%d\n", c0_value, c0_nf, obj->als_modulus);

	databuf[0]=TMD2772_CMM_C1DATA_L;
	res = TMD2772_i2c_master_operate(client, databuf, 0x201, I2C_FLAG_READ);
	if(res < 0)
	{
		goto READ_ALS_EXIT_ERR;
	}
	
	c1_value = (databuf[0] | (databuf[1]<<8));
	
	c1_nf = obj->als_modulus*c1_value/1000;	
	APS_LOG("c1_value=%d, c1_nf=%d, als_modulus=%d\n", c1_value, c1_nf, obj->als_modulus);
	
	if((c0_value > c1_value) &&(c0_value < 30000))
	{  	/*Lenovo-sw chenlj2 add 2011-06-03,add {*/
		atio = (c1_nf*100)/c0_nf;
      APS_LOG("c0_value=%d, c0_nf=%d, als_modulus=%d,atio = %d\n", c0_value, c0_nf, obj->als_modulus,atio);
	//APS_LOG("atio = %d\n", atio);
	if(atio<30)
	{
		//*data = (13*c0_nf - 24*c1_nf)/10000;
		*data = (13*c0_nf - 24*c1_nf)/10;
	}
	else if(atio>= 30 && atio<38) /*Lenovo-sw chenlj2 add 2011-06-03,modify > to >=*/
	{ 
		//*data = (16*c0_nf - 35*c1_nf)/10000;
		*data = (16*c0_nf - 35*c1_nf)/10;
	}
	else if(atio>= 38 && atio<45)  /*Lenovo-sw chenlj2 add 2011-06-03,modify > to >=*/
	{ 
		//*data = (9*c0_nf - 17*c1_nf)/10000;
		*data = (9*c0_nf - 17*c1_nf)/10;
	}
	else if(atio>= 45 && atio<54) /*Lenovo-sw chenlj2 add 2011-06-03,modify > to >=*/
	{ 
		//*data = (6*c0_nf - 10*c1_nf)/10000;
		*data = (6*c0_nf - 10*c1_nf)/10;
	}
	else
		*data = 2*c1_value;
	/*Lenovo-sw chenlj2 add 2011-06-03,add }*/
    }
	else if (c0_value >30000)
	{
		*data = 65535;
	}
        else
	{
		*data = c1_value;
		//APS_DBG("TMD2772_read_als als_value is invalid!!\n");
		//return -1;
	}

	APS_LOG("c0_value=%d, c1_value=%d, als_modulus=%d,atio = %d,als_value=%d\n", c0_value, c1_value, obj->als_modulus,atio,*data);

	*data=(*data)*2/5;

	//if(*data > 40) *data = *data * 2;
		
	if(*data <=3)
        {
           *data = 0;
        }
	
	 
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB
		APS_LOG("als_value=%d\n", *data);

	return 0;
READ_ALS_EXIT_ERR:
	return res;
}

//int TMD2772_read_als_ch0(struct i2c_client *client, u16 *data)
int TMD2772_read_als_ch0(struct i2c_client *client)
{	 
	//struct TMD2772_priv *obj = i2c_get_clientdata(client);
	u16 c0_value;	 
	u8 buffer[2];
	int res = 0;
	
	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

//get adc channel 0 value
	buffer[0]=TMD2772_CMM_C0DATA_L;
	res = TMD2772_i2c_master_operate(client, buffer, 0x201, I2C_FLAG_READ);
	if(res < 0)
	{
		goto EXIT_ERR;
	}
	c0_value = buffer[0] | (buffer[1]<<8);
	//*data = c0_value;
	APS_LOG("c0_value=%d\n", c0_value);

	if(c0_value<8000)
		return 0;

	else 
		return 1;
	
EXIT_ERR:
	APS_ERR("TMD2772_read_ps fail\n");
	return res;
}

/********************************************************************/
static int TMD2772_get_ps_value(struct TMD2772_priv *obj, u16 ps)
{
	int val=1;// mask = atomic_read(&obj->ps_mask);
	//int invalid = 0;
	static int val_temp=1;

	APS_LOG("TMD2772_get_ps_value val 111 = %d  low:%d high:%d ",ps, atomic_read(&obj->ps_thd_val_low), atomic_read(&obj->ps_thd_val_high));
			
	if(ps_cali.valid == 1)
	{
			if((ps >ps_cali.close))
			{
				val = 0;  /*close*/
				val_temp = 0;
				intr_flag_value = 1;
			}
			
			else if((ps < ps_cali.far_away))
			{
				val = 1;  /*far away*/
				val_temp = 1;
				intr_flag_value = 0;
			}
			else
				val = val_temp;

			APS_LOG("TMD2772_get_ps_value val  222= %d",val);
	}
	else
	{
			//if(ps  > 500)
			if((ps  > atomic_read(&obj->ps_thd_val_high)))
			{
				val = 0;  /*close*/
				val_temp = 0;
				intr_flag_value = 1;
			}
			//else if(ps  < 400)
			else if((ps  < atomic_read(&obj->ps_thd_val_low)))	
			{
				val = 1;  /*far away*/
				val_temp = 1;
				intr_flag_value = 0;
			}
//			else
//			       val = val_temp;	
			
	}

	APS_DBG("PS:  status = %d\n",val);
	return val_temp;
#if 0	
	/*if(atomic_read(&obj->ps_suspend))
	{
		invalid = 1;
	}*/
	/*else*/ if(1 == atomic_read(&obj->ps_deb_on))
	{
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->ps_deb_on, 0);
		}
		
		/*if (1 == atomic_read(&obj->ps_deb_on))
		{
			invalid = 1;
		}*/
	}
	else if (obj->als > 45000)
	{
		//invalid = 1;
		APS_DBG("ligh too high will result to failt proximiy\n");
		return 1;  /*far away*/
	}

	if(!invalid)
	{
		APS_DBG("PS:  %05d => %05d\n", ps, val);
		return val;
	}	
	else
	{
		APS_LOG("TMD2772_get_ps_value val 333 = %d",ps);
		return -1;
	}	
#endif
}
/********************************************************************/
static int TMD2772_get_als_value(struct TMD2772_priv *obj, u16 als)
{
	int idx;
	int invalid = 0;

	APS_LOG("TMD2772_get_ps_value val");
	
	for(idx = 0; idx < obj->als_level_num; idx++)
	{
		if(als < obj->hw->als_level[idx])
		{
			break;
		}
	}
	
	if(idx >= obj->als_value_num)
	{
		APS_ERR("TMD2772_get_als_value exceed range\n"); 
		idx = obj->als_value_num - 1;
	}
	
	if(1 == atomic_read(&obj->als_deb_on))
	{
		unsigned long endt = atomic_read(&obj->als_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->als_deb_on, 0);
		}
		
		if(1 == atomic_read(&obj->als_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
		APS_ERR("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);	
		return obj->hw->als_value[idx];
	}
	else
	{
		APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);    
		return -1;
	}
}


/*-------------------------------attribute file for debugging----------------------------------*/

/******************************************************************************
 * Sysfs attributes
*******************************************************************************/
static ssize_t TMD2772_show_config(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	
	if(!TMD2772_obj)
	{
		APS_ERR("TMD2772_obj is null!!\n");
		return 0;
	}
	
	res = snprintf(buf, PAGE_SIZE, "(%d %d %d %d %d)\n", 
		atomic_read(&TMD2772_obj->i2c_retry), atomic_read(&TMD2772_obj->als_debounce), 
		atomic_read(&TMD2772_obj->ps_mask), atomic_read(&TMD2772_obj->ps_thd_val), atomic_read(&TMD2772_obj->ps_debounce));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t TMD2772_store_config(struct device_driver *ddri, const char *buf, size_t count)
{
	int retry, als_deb, ps_deb, mask, thres;
	if(!TMD2772_obj)
	{
		APS_ERR("TMD2772_obj is null!!\n");
		return 0;
	}
	
	if(5 == sscanf(buf, "%d %d %d %d %d", &retry, &als_deb, &mask, &thres, &ps_deb))
	{ 
		atomic_set(&TMD2772_obj->i2c_retry, retry);
		atomic_set(&TMD2772_obj->als_debounce, als_deb);
		atomic_set(&TMD2772_obj->ps_mask, mask);
		atomic_set(&TMD2772_obj->ps_thd_val, thres);        
		atomic_set(&TMD2772_obj->ps_debounce, ps_deb);
	}
	else
	{
		APS_ERR("invalid content: '%s'\n", buf);
	}
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t TMD2772_show_trace(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	if(!TMD2772_obj)
	{
		APS_ERR("TMD2772_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&TMD2772_obj->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t TMD2772_store_trace(struct device_driver *ddri, const char *buf, size_t count)
{
    int trace;
    if(!TMD2772_obj)
	{
		APS_ERR("TMD2772_obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&TMD2772_obj->trace, trace);
	}
	else 
	{
		APS_ERR("invalid content: '%s'\n", buf);
	}
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t TMD2772_show_als(struct device_driver *ddri, char *buf)
{
	int res;
	
	if(!TMD2772_obj)
	{
		APS_ERR("TMD2772_obj is null!!\n");
		return 0;
	}
	if((res = TMD2772_read_als(TMD2772_obj->client, &TMD2772_obj->als)))
	{
		return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	}
	else
	{
		return snprintf(buf, PAGE_SIZE, "0x%04X\n", TMD2772_obj->als);     
	}
}
/*----------------------------------------------------------------------------*/
static ssize_t TMD2772_show_ps(struct device_driver *ddri, char *buf)
{
	int res;
	if(!TMD2772_obj)
	{
		APS_ERR("cm3623_obj is null!!\n");
		return 0;
	}

	//APS_ERR("TMD2772_show_ps\n");
	if((res = TMD2772_read_ps(TMD2772_obj->client, &TMD2772_obj->ps)))
	{
		return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	}
	else
	{
		//return snprintf(buf, PAGE_SIZE, "0x%04X\n", TMD2772_obj->ps);     
		#ifndef VENDOR_EDIT //ziqing.guo@BasicDrv.Sensor, 2015/02/12, Modify  for factory mode 
			return scnprintf(buf, PAGE_SIZE, "0x%04X\n", TMD2772_obj->ps); 
		#else/* VENDOR_EDIT */
			return scnprintf(buf, PAGE_SIZE, "%d\n", TMD2772_obj->ps);   
		#endif/* VENDOR_EDIT */	
	}
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t TMD2772_show_reg(struct device_driver *ddri, char *buf)
{
	int res = 0;
	u8 databuf[3];

	if(!TMD2772_obj)
	{
		APS_ERR("TMD2772_obj is null!!\n");
		return 0;
	}

	databuf[0]=0x9e;
	res = TMD2772_i2c_master_operate(TMD2772_obj->client, databuf, 0x101, I2C_FLAG_READ);
       return scnprintf(buf, PAGE_SIZE, "0x%04X\n", databuf[0]); 
	
	//APS_LOG("TMD2772_read_register reg:%x val:%x \n",i, databuf[0]);
	//}

	
	
	//return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t TMD2772_show_send(struct device_driver *ddri, char *buf)
{
    return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t TMD2772_store_send(struct device_driver *ddri, const char *buf, size_t count)
{
	int addr, cmd;
	u8 dat;

	if(!TMD2772_obj)
	{
		APS_ERR("TMD2772_obj is null!!\n");
		return 0;
	}
	else if(2 != sscanf(buf, "%x %x", &addr, &cmd))
	{
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}

	dat = (u8)cmd;
	//****************************
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t TMD2772_show_recv(struct device_driver *ddri, char *buf)
{
    return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t TMD2772_store_recv(struct device_driver *ddri, const char *buf, size_t count)
{
	int addr;
	//u8 dat;
	if(!TMD2772_obj)
	{
		APS_ERR("TMD2772_obj is null!!\n");
		return 0;
	}
	else if(1 != sscanf(buf, "%x", &addr))
	{
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}

	//****************************
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t TMD2772_show_status(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	
	if(!TMD2772_obj)
	{
		APS_ERR("TMD2772_obj is null!!\n");
		return 0;
	}
	
	if(TMD2772_obj->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d, (%d %d)\n", 
			TMD2772_obj->hw->i2c_num, TMD2772_obj->hw->power_id, TMD2772_obj->hw->power_vol);
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}
	
	len += snprintf(buf+len, PAGE_SIZE-len, "REGS: %02X %02X %02X %02lX %02lX\n", 
				atomic_read(&TMD2772_obj->als_cmd_val), atomic_read(&TMD2772_obj->ps_cmd_val), 
				atomic_read(&TMD2772_obj->ps_thd_val),TMD2772_obj->enable, TMD2772_obj->pending_intr);
	
	len += snprintf(buf+len, PAGE_SIZE-len, "MISC: %d %d\n", atomic_read(&TMD2772_obj->als_suspend), atomic_read(&TMD2772_obj->ps_suspend));

	return len;
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
#define IS_SPACE(CH) (((CH) == ' ') || ((CH) == '\n'))
/*----------------------------------------------------------------------------*/
static int read_int_from_buf(struct TMD2772_priv *obj, const char* buf, size_t count, u32 data[], int len)
{
	int idx = 0;
	char *cur = (char*)buf, *end = (char*)(buf+count);

	while(idx < len)
	{
		while((cur < end) && IS_SPACE(*cur))
		{
			cur++;        
		}

		if(1 != sscanf(cur, "%d", &data[idx]))
		{
			break;
		}

		idx++; 
		while((cur < end) && !IS_SPACE(*cur))
		{
			cur++;
		}
	}
	return idx;
}
/*----------------------------------------------------------------------------*/
static ssize_t TMD2772_show_alslv(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;
	if(!TMD2772_obj)
	{
		APS_ERR("TMD2772_obj is null!!\n");
		return 0;
	}
	
	for(idx = 0; idx < TMD2772_obj->als_level_num; idx++)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "%d ", TMD2772_obj->hw->als_level[idx]);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;    
}
/*----------------------------------------------------------------------------*/
static ssize_t TMD2772_store_alslv(struct device_driver *ddri, const char *buf, size_t count)
{
	if(!TMD2772_obj)
	{
		APS_ERR("TMD2772_obj is null!!\n");
		return 0;
	}
	else if(!strcmp(buf, "def"))
	{
		memcpy(TMD2772_obj->als_level, TMD2772_obj->hw->als_level, sizeof(TMD2772_obj->als_level));
	}
	else if(TMD2772_obj->als_level_num != read_int_from_buf(TMD2772_obj, buf, count, 
			TMD2772_obj->hw->als_level, TMD2772_obj->als_level_num))
	{
		APS_ERR("invalid format: '%s'\n", buf);
	}    
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t TMD2772_show_alsval(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;
	if(!TMD2772_obj)
	{
		APS_ERR("TMD2772_obj is null!!\n");
		return 0;
	}
	
	for(idx = 0; idx < TMD2772_obj->als_value_num; idx++)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "%d ", TMD2772_obj->hw->als_value[idx]);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;    
}
/*----------------------------------------------------------------------------*/
static ssize_t TMD2772_store_alsval(struct device_driver *ddri, const char *buf, size_t count)
{
	if(!TMD2772_obj)
	{
		APS_ERR("TMD2772_obj is null!!\n");
		return 0;
	}
	else if(!strcmp(buf, "def"))
	{
		memcpy(TMD2772_obj->als_value, TMD2772_obj->hw->als_value, sizeof(TMD2772_obj->als_value));
	}
	else if(TMD2772_obj->als_value_num != read_int_from_buf(TMD2772_obj, buf, count, 
			TMD2772_obj->hw->als_value, TMD2772_obj->als_value_num))
	{
		APS_ERR("invalid format: '%s'\n", buf);
	}    
	return count;
}
/*---------------------------------------------------------------------------------------*/
static DRIVER_ATTR(als,     S_IWUSR | S_IRUGO, TMD2772_show_als, NULL);
static DRIVER_ATTR(ps,      S_IWUSR | S_IRUGO, TMD2772_show_ps, NULL);
static DRIVER_ATTR(config,  S_IWUSR | S_IRUGO, TMD2772_show_config,	TMD2772_store_config);
static DRIVER_ATTR(alslv,   S_IWUSR | S_IRUGO, TMD2772_show_alslv, TMD2772_store_alslv);
static DRIVER_ATTR(alsval,  S_IWUSR | S_IRUGO, TMD2772_show_alsval, TMD2772_store_alsval);
static DRIVER_ATTR(trace,   S_IWUSR | S_IRUGO, TMD2772_show_trace,		TMD2772_store_trace);
static DRIVER_ATTR(status,  S_IWUSR | S_IRUGO, TMD2772_show_status, NULL);
static DRIVER_ATTR(send,    S_IWUSR | S_IRUGO, TMD2772_show_send, TMD2772_store_send);
static DRIVER_ATTR(recv,    S_IWUSR | S_IRUGO, TMD2772_show_recv, TMD2772_store_recv);
static DRIVER_ATTR(reg,     S_IWUSR | S_IRUGO, TMD2772_show_reg, NULL);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *TMD2772_attr_list[] = {
    &driver_attr_als,
    &driver_attr_ps,    
    &driver_attr_trace,        /*trace log*/
    &driver_attr_config,
    &driver_attr_alslv,
    &driver_attr_alsval,
    &driver_attr_status,
    &driver_attr_send,
    &driver_attr_recv,
    &driver_attr_reg,
};

/*----------------------------------------------------------------------------*/
static int TMD2772_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(TMD2772_attr_list)/sizeof(TMD2772_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, TMD2772_attr_list[idx])))
		{            
			APS_ERR("driver_create_file (%s) = %d\n", TMD2772_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
	static int TMD2772_delete_attr(struct device_driver *driver)
	{
	int idx ,err = 0;
	int num = (int)(sizeof(TMD2772_attr_list)/sizeof(TMD2772_attr_list[0]));

	if (!driver)
	return -EINVAL;

	for (idx = 0; idx < num; idx++) 
	{
		driver_remove_file(driver, TMD2772_attr_list[idx]);
	}
	
	return err;
}
/*----------------------------------------------------------------------------*/
/************************************************************/
static int set_psensor_threshold(struct i2c_client *client)
{
	struct TMD2772_priv *obj = i2c_get_clientdata(client);
	int res = 0;

	u8 databuf[3];
#ifdef CUSTOM_KERNEL_SENSORHUB

#else //#ifdef CUSTOM_KERNEL_SENSORHUB
    //u8 databuf[3];
/*
	APS_ERR("set_psensor_threshold function high: 0x%x, low:0x%x\n",atomic_read(&obj->ps_thd_val_high),atomic_read(&obj->ps_thd_val_low));
	databuf[0] = TMD2772_CMM_INT_LOW_THD_LOW;
	databuf[1] = atomic_read(&obj->ps_thd_val_low)& 0x00ff;
	databuf[2] = atomic_read(&obj->ps_thd_val_low)>>8;//threshold value need to confirm
	res = TMD2772_i2c_master_operate(client, databuf, 0x3, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		APS_ERR("i2c_master_send function err\n");
		return -1;
	}

	databuf[0] = TMD2772_CMM_INT_HIGH_THD_LOW;
	databuf[1] = atomic_read(&obj->ps_thd_val_high)& 0x00ff;
	databuf[2] = atomic_read(&obj->ps_thd_val_high)>>8;//threshold value need to confirm
	res = TMD2772_i2c_master_operate(client, databuf, 0x3, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		APS_ERR("i2c_master_send function err\n");
		return -1;
	}
*/
	/*singal interrupt function add*/
    			  //APS_LOG("tmd enter intr = %d obj->ps_thd_val_low=%d,obj->ps_thd_val_low=%d\r\n",intr_flag_value,obj->ps_thd_val_low,obj->ps_thd_val_high);
			if(!intr_flag_value)
			{
			    //printk("tmd enter 55555\r\n");
				databuf[0] = TMD2772_CMM_INT_LOW_THD_LOW;	
				databuf[1] = 0;

				//printk("tmd enter 1 databuf[1]=%d\r\n",databuf[1]);
				
				res = TMD2772_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
				if(res < 0)
				{
					goto EXIT_INTR_ERR;
				}
				
				databuf[0] = TMD2772_CMM_INT_LOW_THD_HIGH;	
				databuf[1] = 0;

				//printk("tmd enter 2 databuf[1]=%d\r\n",databuf[1]);
				res = TMD2772_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
				if(res < 0)
				{
					goto EXIT_INTR_ERR;
				}
				databuf[0] = TMD2772_CMM_INT_HIGH_THD_LOW;	
				databuf[1] = (u8)(0x00FF&(atomic_read(&obj->ps_thd_val_high)));

				//printk("tmd enter 3 databuf[1]=%d\r\n",databuf[1]);
				res = TMD2772_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
				if(res < 0)
				{
					goto EXIT_INTR_ERR;
				}
				
				databuf[0] = TMD2772_CMM_INT_HIGH_THD_HIGH; 
				databuf[1] = (u8)((0xFF00&atomic_read(&obj->ps_thd_val_high))>>8);

				//printk("tmd enter 4 databuf[1]=%d\r\n",databuf[1]);
				res = TMD2772_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
				if(res < 0)
				{
					goto EXIT_INTR_ERR;
				}
							
			}
			else
				{	
				   // printk("tmd enter 66666\r\n");
					databuf[0] = TMD2772_CMM_INT_LOW_THD_LOW;	
					databuf[1] = (u8)(atomic_read(&obj->ps_thd_val_low) & 0x00FF);

					//printk("tmd enter 5 databuf[1]=%d\r\n",databuf[1]);
					res = TMD2772_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
					if(res < 0)
					{
						goto EXIT_INTR_ERR;
					}
					
					databuf[0] = TMD2772_CMM_INT_LOW_THD_HIGH;	
					databuf[1] = (u8)((0xFF00&atomic_read(&obj->ps_thd_val_low))>>8);

					//printk("tmd enter 6 databuf[1]=%d\r\n",databuf[1]);
					res = TMD2772_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
					if(res < 0)
					{
						goto EXIT_INTR_ERR;
					}
					
					databuf[0] = TMD2772_CMM_INT_HIGH_THD_LOW;	
					databuf[1] = (u8)(0x00FF);
					
					//printk("tmd enter 7 databuf[1]=%d\r\n",databuf[1]);
					res = TMD2772_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
					if(res < 0)
					{
						goto EXIT_INTR_ERR;
					}
				
					databuf[0] = TMD2772_CMM_INT_HIGH_THD_HIGH; 
					databuf[1] = (u8)(0xFF00>> 8);

					//printk("tmd enter 8 databuf[1]=%d\r\n",databuf[1]);
					res = TMD2772_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
					if(res < 0)
					{
						goto EXIT_INTR_ERR;
					}
					
				}

			
	return 0;
EXIT_INTR_ERR:			
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB
	return res;

}

/*----------------------------------------------------------------------------*/
static void tmd2772_set_ps_threshold(struct i2c_client *client, int low_threshold, int high_threshold)
{
    struct TMD2772_priv *obj = i2c_get_clientdata(client);
    atomic_set(&obj->ps_thd_val_high,  high_threshold);
    atomic_set(&obj->ps_thd_val_low,  low_threshold);//need to confirm
    printk("enter 4");
    set_psensor_threshold(obj->client);
}
/*----------------------------------interrupt functions--------------------------------*/

/*----------------------------------------------------------------------------*/
#ifndef CUSTOM_KERNEL_SENSORHUB
static int TMD2772_check_intr(struct i2c_client *client) 
{
	int res,intp,intl;
	u8 buffer[2];

	if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/  
	return 0;

	buffer[0] = TMD2772_CMM_STATUS;
	res = TMD2772_i2c_master_operate(client, buffer, 0x101, I2C_FLAG_READ);
	if(res < 0)
	{
		goto EXIT_ERR;
	}
	res = 0;
	intp = 0;
	intl = 0;
	if(0 != (buffer[0] & 0x20))
	{
		res = 0;
		intp = 1;
	}
	if(0 != (buffer[0] & 0x10))
	{
		res = 0;
		intl = 1;		
	}

	return res;

EXIT_ERR:
	APS_ERR("TMD2772_check_intr fail\n");
	return 1;

}
#endif //#ifndef CUSTOM_KERNEL_SENSORHUB

static int TMD2772_clear_intr(struct i2c_client *client) 
{
	int res;
	u8 buffer[2];

	
	
	buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x07);
	res = TMD2772_i2c_master_operate(client, buffer, 0x1, I2C_FLAG_WRITE);
	if(res < 0)
	{
		goto EXIT_ERR;
	}
	else
	{
		res = 0;
	}

	APS_ERR("TMD2772_check_and_clear_intr pass\n");
	return res;

EXIT_ERR:
	APS_ERR("TMD2772_check_and_clear_intr fail\n");
	return 1;
}

/*----------------------------------------------------------------------------*/
static void TMD2772_eint_work(struct work_struct *work)
{
#ifdef CUSTOM_KERNEL_SENSORHUB
    int res = 0;
    
    res = ps_report_interrupt_data(intr_flag);
    if(res != 0)
    {
        APS_ERR("TMD2772_eint_work err: %d\n", res);
    }
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	struct TMD2772_priv *obj = (struct TMD2772_priv *)container_of(work, struct TMD2772_priv, eint_work);
	int res = 0;
	//u8 databuf[3];
	//u8 i;   
	//u8 val;    
	APS_LOG("TMD2772 int top half time = %lld\n", int_top_time);
	
	/*for (i = 0x80; i <= 0x9e; i += 0x01){        
		databuf[0]=i;
		res = TMD2772_i2c_master_operate(obj->client, databuf, 0x101, I2C_FLAG_READ);
	if(res < 0)
	{
		goto EXIT_INTR_ERR;
	}
	
	APS_LOG("TMD2772_read_register reg:%x val:%x \n",i, databuf[0]);
	}*/
	
	if((res = TMD2772_check_intr(obj->client)))
	{
		APS_ERR("TMD2772_eint_work check intrs: %d\n", res);
	}
	else
	{
		//get raw data
		TMD2772_read_ps(obj->client, &obj->ps);
//		TMD2772_read_als_ch0(obj->client, &obj->als);
//		APS_LOG("TMD2772_eint_work rawdata ps=%d als_ch0=%d!\n",obj->ps,obj->als);
		
//		if(obj->als > 40000)
//		{
//			APS_LOG("TMD2772_eint_work ALS too large may under lighting als_ch0=%d!\n",obj->als);
//			return;
//		}
		intr_flag = TMD2772_get_ps_value(obj, obj->ps);
		APS_LOG("TMD2772_eint_work intr_flag = %d\n",intr_flag);
		tmd2772_set_ps_threshold(obj->client,atomic_read(&obj->ps_thd_val_low), atomic_read(&obj->ps_thd_val_high));
						
			//let up layer to know
			if(intr_flag != -1)
			{
				//prox_last_value = intr_flag;
				APS_LOG("tmd2772 interrupt value = %d\n", intr_flag);
				res = ps_report_interrupt_data(intr_flag);
		        }

	
	}
  TMD2772_clear_intr(obj->client);


#ifdef CUST_EINT_ALS_TYPE
	mt_eint_unmask(CUST_EINT_ALS_NUM);
#else
	mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
#endif

	return;
//EXIT_INTR_ERR:
	
#ifdef CUST_EINT_ALS_TYPE
	mt_eint_unmask(CUST_EINT_ALS_NUM);
#else
	mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
#endif
	APS_ERR("TMD2772_eint_work err: %d\n", res);
#endif
}
/*----------------------------------------------------------------------------*/
#ifdef CUSTOM_KERNEL_SENSORHUB
static void TMD2772_init_done_work(struct work_struct *work)
{
    struct TMD2772_priv *obj = TMD2772_obj;
    TMD2772_CUST_DATA *p_cust_data;
    SCP_SENSOR_HUB_DATA data;
    int max_cust_data_size_per_packet;
    int i;
    uint sizeOfCustData;
    uint len;
    char *p = (char *)obj->hw;

    APS_FUN();

    p_cust_data = (TMD2772_CUST_DATA *)data.set_cust_req.custData;
    sizeOfCustData = sizeof(*(obj->hw));
    max_cust_data_size_per_packet = sizeof(data.set_cust_req.custData) - offsetof(TMD2772_SET_CUST, data);
    
    for (i=0;sizeOfCustData>0;i++)
    {
        data.set_cust_req.sensorType = ID_PROXIMITY;
        data.set_cust_req.action = SENSOR_HUB_SET_CUST;
        p_cust_data->setCust.action = TMD2772_CUST_ACTION_SET_CUST;
        p_cust_data->setCust.part = i;
        
        if (sizeOfCustData > max_cust_data_size_per_packet)
        {
            len = max_cust_data_size_per_packet;
        }
        else
        {
            len = sizeOfCustData;
        }

        memcpy(p_cust_data->setCust.data, p, len);
        sizeOfCustData -= len;
        p += len;
        
        len += offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + offsetof(TMD2772_SET_CUST, data);
        SCP_sensorHub_req_send(&data, &len, 1);
    }

    data.set_cust_req.sensorType = ID_PROXIMITY;
    data.set_cust_req.action = SENSOR_HUB_SET_CUST;
    p_cust_data->setEintInfo.action = TMD2772_CUST_ACTION_SET_EINT_INFO;
    p_cust_data->setEintInfo.gpio_mode = GPIO_ALS_EINT_PIN_M_EINT;
    p_cust_data->setEintInfo.gpio_pin = GPIO_ALS_EINT_PIN;
    p_cust_data->setEintInfo.eint_num = CUST_EINT_ALS_NUM;
    p_cust_data->setEintInfo.eint_is_deb_en = CUST_EINT_ALS_DEBOUNCE_EN;
    p_cust_data->setEintInfo.eint_type = CUST_EINT_ALS_TYPE;
    len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(p_cust_data->setEintInfo);
    SCP_sensorHub_req_send(&data, &len, 1);

}
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB
/*----------------------------------------------------------------------------*/
static void TMD2772_eint_func(void)
{
	struct TMD2772_priv *obj = TMD2772_obj;
	if(!obj)
	{
		return;
	}	
	int_top_time = sched_clock();
	schedule_work(&obj->eint_work);
}
/*----------------------------------------------------------------------------*/
#ifdef CUSTOM_KERNEL_SENSORHUB
static int TMD2772_irq_handler(void* data, uint len)
{
	struct TMD2772_priv *obj = TMD2772_obj;
    SCP_SENSOR_HUB_DATA_P rsp = (SCP_SENSOR_HUB_DATA_P)data;
    
	if(!obj)
	{
		return -1;
	}

    APS_ERR("len = %d, type = %d, action = %d, errCode = %d\n", len, rsp->rsp.sensorType, rsp->rsp.action, rsp->rsp.errCode);

    switch(rsp->rsp.action)
    {
        case SENSOR_HUB_NOTIFY:
            switch(rsp->notify_rsp.event)
            {
                case SCP_INIT_DONE:
                    schedule_work(&obj->init_done_work);
                    //schedule_delayed_work(&obj->init_done_work, HZ);
                    break;
                case SCP_NOTIFY:
                    if (TMD2772_NOTIFY_PROXIMITY_CHANGE == rsp->notify_rsp.data[0])
                    {
                        intr_flag = rsp->notify_rsp.data[1];
                        TMD2772_eint_func();
                    }
                    else
                    {
                        APS_ERR("Unknow notify");
                    }
                    break;
                default:
                    APS_ERR("Error sensor hub notify");
                    break;
            }
            break;
        default:
            APS_ERR("Error sensor hub action");
            break;
    }

    return 0;
}
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB
/*----------------------------------------------------------------------------*/
int TMD2772_setup_eint(struct i2c_client *client)
{
#ifdef CUSTOM_KERNEL_SENSORHUB
    int err = 0;

    err = SCP_sensorHub_rsp_registration(ID_PROXIMITY, TMD2772_irq_handler);
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	//struct TMD2772_priv *obj = i2c_get_clientdata(client);        

	mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, TRUE);
	mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);

#ifdef CUST_EINT_ALS_TYPE
	mt_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_TYPE, TMD2772_eint_func, 0);
#else
	mt65xx_eint_set_sens(CUST_EINT_ALS_NUM, CUST_EINT_ALS_SENSITIVE);
	mt65xx_eint_set_polarity(CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY);
	mt65xx_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt65xx_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_ALS_POLARITY, TMD2772_eint_func, 0);
#endif

#ifdef CUST_EINT_ALS_TYPE
	#ifdef VENDOR_EDIT 	
	//ziqing.guo@BasicDrv.Sensor,2015/04/21, modify for QT 621670
	//mt_eint_unmask(CUST_EINT_ALS_NUM);
      		 mt_eint_mask(CUST_EINT_ALS_NUM);
	#endif
#else
	mt65xx_eint_unmask(CUST_EINT_ALS_NUM); 
#endif

#endif //#ifdef CUSTOM_KERNEL_SENSORHUB
    return 0;
}
/*-------------------------------MISC device related------------------------------------------*/



/************************************************************/
static int TMD2772_open(struct inode *inode, struct file *file)
{
	file->private_data = TMD2772_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}
/************************************************************/
static int TMD2772_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}


/*----------------------------------------------------------------------------*/
static long TMD2772_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
		struct i2c_client *client = (struct i2c_client*)file->private_data;
		struct TMD2772_priv *obj = i2c_get_clientdata(client);  
		long err = 0;
		void __user *ptr = (void __user*) arg;
		int dat;
		uint32_t enable;
		int ps_result;
		int ps_cali;
		int threshold[2];
		struct alsps_hw *hw = NULL;
    		struct set_ps_thd_para set_ps_thd_para;
#ifdef CUSTOM_KERNEL_SENSORHUB
        SCP_SENSOR_HUB_DATA data;
        TMD2772_CUST_DATA *pCustData;
        int len;

        data.set_cust_req.sensorType = ID_PROXIMITY;
        data.set_cust_req.action = SENSOR_HUB_SET_CUST;
        pCustData = (TMD2772_CUST_DATA *)(&data.set_cust_req.custData);
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB
		
		switch (cmd)
		{
			case ALSPS_SET_PS_MODE:
				printk("set TMD enable =  %d",enable) ;
				if(copy_from_user(&enable, ptr, sizeof(enable)))
				{
					err = -EFAULT;
					goto err_out;
				}
				if(enable)
				{
					if((err = TMD2772_enable_ps(obj->client, 1)))
					{
						APS_ERR("enable ps fail: %ld\n", err); 
						goto err_out;
					}
					
					set_bit(CMC_BIT_PS, &obj->enable);
				}
				else
				{
					if((err = TMD2772_enable_ps(obj->client, 0)))
					{
						APS_ERR("disable ps fail: %ld\n", err); 
						goto err_out;
					}
					clear_bit(CMC_BIT_PS, &obj->enable);
				}
				break;
	
			case ALSPS_GET_PS_MODE:
				#ifdef VENDOR_EDIT 	
				//ziqing.guo@BasicDrv.Sensor,2015/04/12, add for PSD
				enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
				#endif
				
				printk("TMD enable =  %d",enable) ;
				if(copy_to_user(ptr, &enable, sizeof(enable)))
				{
					err = -EFAULT;
					goto err_out;
				}
				break;
	
			case ALSPS_GET_PS_DATA:    
				APS_LOG("enter 1\n"); 
				if((err = TMD2772_read_ps(obj->client, &obj->ps)))
				{
					goto err_out;
				}
				
				dat = TMD2772_get_ps_value(obj, obj->ps);
				if(copy_to_user(ptr, &dat, sizeof(dat)))
				{
					err = -EFAULT;
					goto err_out;
				}  
				break;
	
			case ALSPS_GET_PS_RAW_DATA:  
				APS_LOG("enter 2\n"); 
				if((err = TMD2772_read_ps(obj->client, &obj->ps)))
				{
					goto err_out;
				}
				
				dat = obj->ps;
				if(copy_to_user(ptr, &dat, sizeof(dat)))
				{
					err = -EFAULT;
					goto err_out;
				}  
				break;			  
	
			case ALSPS_SET_ALS_MODE:
	
				if(copy_from_user(&enable, ptr, sizeof(enable)))
				{
					err = -EFAULT;
					goto err_out;
				}
				if(enable)
				{
					if((err = TMD2772_enable_als(obj->client, 1)))
					{
						APS_ERR("enable als fail: %ld\n", err); 
						goto err_out;
					}
					set_bit(CMC_BIT_ALS, &obj->enable);
				}
				else
				{
					if((err = TMD2772_enable_als(obj->client, 0)))
					{
						APS_ERR("disable als fail: %ld\n", err); 
						goto err_out;
					}
					clear_bit(CMC_BIT_ALS, &obj->enable);
				}
				break;
	
			case ALSPS_GET_ALS_MODE:
				enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
				if(copy_to_user(ptr, &enable, sizeof(enable)))
				{
					err = -EFAULT;
					goto err_out;
				}
				break;
	
			case ALSPS_GET_ALS_DATA: 
				if((err = TMD2772_read_als(obj->client, &obj->als)))
				{
					goto err_out;
				}
	
				dat = TMD2772_get_als_value(obj, obj->als);
				if(copy_to_user(ptr, &dat, sizeof(dat)))
				{
					err = -EFAULT;
					goto err_out;
				}			   
				break;
	
			case ALSPS_GET_ALS_RAW_DATA:	
				if((err = TMD2772_read_als(obj->client, &obj->als)))
				{
					goto err_out;
				}
	
				dat = obj->als;
				if(copy_to_user(ptr, &dat, sizeof(dat)))
				{
					err = -EFAULT;
					goto err_out;
				}			   
				break;

			/*----------------------------------for factory mode test---------------------------------------*/
			case ALSPS_GET_PS_TEST_RESULT:
				APS_LOG("enter 3\n"); 
				if((err = TMD2772_read_ps(obj->client, &obj->ps)))
				{
					goto err_out;
				}
				if(obj->ps > atomic_read(&obj->ps_thd_val_high))
					{
						ps_result = 0;
					}
				else	
					ps_result = 1;
				
				if(copy_to_user(ptr, &ps_result, sizeof(ps_result)))
				{
					err = -EFAULT;
					goto err_out;
				}			   
				break;

			case ALSPS_IOCTL_CLR_CALI:
				if(copy_from_user(&dat, ptr, sizeof(dat)))
				{
					err = -EFAULT;
					goto err_out;
				}
				if(dat == 0)
					obj->ps_cali = 0;

#ifdef CUSTOM_KERNEL_SENSORHUB
                pCustData->clearCali.action = TMD2772_CUST_ACTION_CLR_CALI;
                len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(pCustData->clearCali);
                
                err = SCP_sensorHub_req_send(&data, &len, 1);
#endif

				break;

			case ALSPS_IOCTL_GET_CALI:
				ps_cali = obj->ps_cali ;
				if(copy_to_user(ptr, &ps_cali, sizeof(ps_cali)))
				{
					err = -EFAULT;
					goto err_out;
				}
				break;

			case ALSPS_IOCTL_SET_CALI:
				if(copy_from_user(&ps_cali, ptr, sizeof(ps_cali)))
				{
					err = -EFAULT;
					goto err_out;
				}

				obj->ps_cali = ps_cali;

#ifdef CUSTOM_KERNEL_SENSORHUB
                pCustData->setCali.action = TMD2772_CUST_ACTION_SET_CALI;
                pCustData->setCali.cali = ps_cali;
                len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(pCustData->setCali);
                
                err = SCP_sensorHub_req_send(&data, &len, 1);
#endif

				break;

			case ALSPS_SET_PS_THRESHOLD:
				if(copy_from_user(threshold, ptr, sizeof(threshold)))
				{
					err = -EFAULT;
					goto err_out;
				}
				APS_ERR("%s set threshold high: 0x%x, low: 0x%x\n", __func__, threshold[0],threshold[1]); 
				atomic_set(&obj->ps_thd_val_high,  (threshold[0]+obj->ps_cali));
				atomic_set(&obj->ps_thd_val_low,  (threshold[1]+obj->ps_cali));//need to confirm

				set_psensor_threshold(obj->client);
				
				break;
				
			case ALSPS_GET_PS_THRESHOLD_HIGH:
				threshold[0] = atomic_read(&obj->ps_thd_val_high) - obj->ps_cali;
				APS_ERR("%s get threshold high: 0x%x\n", __func__, threshold[0]); 
				if(copy_to_user(ptr, &threshold[0], sizeof(threshold[0])))
				{
					err = -EFAULT;
					goto err_out;
				}
				break;
				
			case ALSPS_GET_PS_THRESHOLD_LOW:
				threshold[0] = atomic_read(&obj->ps_thd_val_low) - obj->ps_cali;
				APS_ERR("%s get threshold low: 0x%x\n", __func__, threshold[0]); 
				if(copy_to_user(ptr, &threshold[0], sizeof(threshold[0])))
				{
					err = -EFAULT;
					goto err_out;
				}
				break;
			/*------------------------------------------------------------------------------------------*/

			case ALSPS_SET_PS_THRESHOLD_OPPO: //0x21 lycan add for 
           			 if(copy_from_user(&set_ps_thd_para, ptr, sizeof(set_ps_thd_para)))
            			{
                			err = -EFAULT;
                			goto err_out;
            			}

	            	switch (set_ps_thd_para.algo_state) {
	                case PS_ADJUST_TREND_STATE : 
	                    APS_LOG("set_ps_threshold ps average:%d state: TREND\n", set_ps_thd_para.ps_average);
	                    break;
	                case PS_ADJUST_NOTREND_STATE :
	                    APS_LOG("set_ps_threshold ps average:%d state: NOTREND\n", set_ps_thd_para.ps_average);
	                    break;
	                case PS_ADJUST_HIGHLIGHT_STATE :
	                    APS_LOG("set_ps_threshold ps average:%d state: HIGHLIGHT\n", set_ps_thd_para.ps_average);
	                    break;
	                case PS_ADJUST_AVOID_DIRTY_STATE : 
	                    APS_LOG("set_ps_threshold ps average:%d state: AVOID_DIRTY\n", set_ps_thd_para.ps_average);
	                    break;
	                default:
	                    APS_LOG("set_ps_threshold ps average:%d state: impossible\n", set_ps_thd_para.ps_average);
	                    break;
	            }

	            tmd2772_set_ps_threshold(obj->client,
	                    set_ps_thd_para.low_threshold, 
	                    set_ps_thd_para.high_threshold);
		/*tmd2772_set_ps_threshold(obj->client,
	                    400, 
	                   500);*/
	            break;

	        	case ALSPS_GET_CUST_PS_ADJUST_PARA: //0x22 lycan add for
	           	 APS_LOG(" %s()->case ALSPS_GET_CUST_PS_ADJUST_PARA \n", __FUNCTION__);
	            	hw = tmd_get_cust_alsps_hw();
	            	if(copy_to_user(ptr, hw->p_ps_adjust_para, sizeof( struct ps_adjust_para)))
	           	 {
	               	 err = -EFAULT;
	                	goto err_out;
	            	}              
	            break;

			case ALSPS_GET_HIGHLIGHT_DATA: 
			dat=TMD2772_read_als_ch0(obj->client);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;

#ifdef VENDOR_EDIT 	
			//ziqing.guo@BasicDrv.Sensor,2015/04/12, add for PSD
		case ALSPS_GET_WAKEUP_STATUS:			
			wait_event_interruptible(enable_ps, test_bit(CMC_BIT_PS, &obj->enable)); 
			dat=1;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
           		 {
                		err = -EFAULT;
                		goto err_out;
            		}              
            break;
#endif
				
				default:
					APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
					err = -ENOIOCTLCMD;
					break;
			}
	
		err_out:
		return err;    
	}
/********************************************************************/
/*------------------------------misc device related operation functions------------------------------------*/
static struct file_operations TMD2772_fops = {
	.owner = THIS_MODULE,
	.open = TMD2772_open,
	.release = TMD2772_release,
	.unlocked_ioctl = TMD2772_unlocked_ioctl,
};

static struct miscdevice TMD2772_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &TMD2772_fops,
};

/*--------------------------------------------------------------------------------------*/
static void TMD2772_early_suspend(struct early_suspend *h)
{
		struct TMD2772_priv *obj = container_of(h, struct TMD2772_priv, early_drv);	
		int err;
		APS_FUN();	  
	
		if(!obj)
		{
			APS_ERR("null pointer!!\n");
			return;
		}
		
		atomic_set(&obj->als_suspend, 1);
		if((err = TMD2772_enable_als(obj->client, 0)))
		{
			APS_ERR("disable als fail: %d\n", err); 
		}
}

static void TMD2772_late_resume(struct early_suspend *h) 
{
		struct TMD2772_priv *obj = container_of(h, struct TMD2772_priv, early_drv);		  
		int err;
		hwm_sensor_data sensor_data;
		memset(&sensor_data, 0, sizeof(sensor_data));
		APS_FUN();
		if(!obj)
		{
			APS_ERR("null pointer!!\n");
			return;
		}
	
		atomic_set(&obj->als_suspend, 0);
		if(test_bit(CMC_BIT_ALS, &obj->enable))
		{
			if((err = TMD2772_enable_als(obj->client, 1)))
			{
				APS_ERR("enable als fail: %d\n", err);		  
	
			}
		}
}
/*--------------------------------------------------------------------------------*/
static int TMD2772_init_client(struct i2c_client *client)
{
	struct TMD2772_priv *obj = i2c_get_clientdata(client);
	u8 databuf[3];    
	int res = 0;
	APS_FUN();

		databuf[0] = TMD2772_CMM_ID;
	res = TMD2772_i2c_master_operate(client, databuf, 0x101, I2C_FLAG_READ);
	if(res < 0)
	{
		goto EXIT_ERR;
	}
	APS_LOG(" TMD2772_CMM_ID = 0x%x \n",databuf[0]);		
	
	databuf[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x00);
	res = TMD2772_i2c_master_operate(client, databuf, 0x1, I2C_FLAG_WRITE);
	if(res < 0)
	{
		APS_LOG(" TMD2771_init_client function 1 err!\n");
		goto EXIT_ERR;
	}
	
	databuf[0] = TMD2772_CMM_ENABLE;
	if(obj->hw->polling_mode_ps == 1)
	databuf[1] = 0x08;
	if(obj->hw->polling_mode_ps == 0)
	databuf[1] = 0x28;
	res = TMD2772_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res < 0)
	{
		APS_LOG(" TMD2771_init_client function 2 err!\n");		
		goto EXIT_ERR;
	}
	
	databuf[0] = TMD2772_CMM_ATIME;    
	databuf[1] = 0xDB;
	res = TMD2772_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res < 0)
	{
		APS_LOG(" TMD2771_init_client function 3 err!\n");
		goto EXIT_ERR;
	}

	databuf[0] = TMD2772_CMM_PTIME;    
	databuf[1] = 0xFF;
	res = TMD2772_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res < 0)
	{
		goto EXIT_ERR;
	}

	databuf[0] = TMD2772_CMM_WTIME;    
	databuf[1] = 0xFC;
	res = TMD2772_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res < 0)
	{
		goto EXIT_ERR;
	}
	/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
	if(0 == obj->hw->polling_mode_ps)
	{
		if(1 == ps_cali.valid)
		{
			if (set_psensor_threshold(client))//set thd.
				goto EXIT_ERR;
		}
		else
		{
			if (set_psensor_threshold(client))//set thd.
				goto EXIT_ERR;

		}

		databuf[0] = TMD2772_CMM_Persistence;
		databuf[1] = 0x10;
		res = TMD2772_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if(res < 0)
		{
			goto EXIT_ERR;
		}

	}

	databuf[0] = TMD2772_CMM_CONFIG;    
	databuf[1] = 0x00;
	res = TMD2772_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res < 0)
	{
		goto EXIT_ERR;
	}

       /*Lenovo-sw chenlj2 add 2011-06-03,modified pulse 2  to 4 */
	databuf[0] = TMD2772_CMM_PPCOUNT;    
	databuf[1] = TMD2772_CMM_PPCOUNT_VALUE;
	res = TMD2772_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res < 0)
	{
		goto EXIT_ERR;
	}

        /*Lenovo-sw chenlj2 add 2011-06-03,modified gain 16  to 1 */
	databuf[0] = TMD2772_CMM_CONTROL;    
	databuf[1] = TMD2772_CMM_CONTROL_VALUE;
	res = TMD2772_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res < 0)
	{
		goto EXIT_ERR;
	}
	/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
	if((res = TMD2772_setup_eint(client))!=0)
	{
		APS_ERR("setup eint: %d\n", res);
		return res;
	}
	if((res = TMD2772_check_and_clear_intr(client)))
	{
		APS_ERR("check/clear intr: %d\n", res);
	    return res;
	}
	
	return TMD2772_SUCCESS;

	EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;
}
/*--------------------------------------------------------------------------------*/

// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int als_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	APS_FUN();
	return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int als_enable_nodata(int en)
{
	int res = 0;
#ifdef CUSTOM_KERNEL_SENSORHUB
    SCP_SENSOR_HUB_DATA req;
    int len;
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

    APS_LOG("TMD2772_obj als enable value = %d\n", en);

#ifdef CUSTOM_KERNEL_SENSORHUB
    req.activate_req.sensorType = ID_LIGHT;
    req.activate_req.action = SENSOR_HUB_ACTIVATE;
    req.activate_req.enable = en;
    len = sizeof(req.activate_req);
    res = SCP_sensorHub_req_send(&req, &len, 1);
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	if(!TMD2772_obj)
	{
		APS_ERR("TMD2772_obj is null!!\n");
		return -1;
	}
	res=	TMD2772_enable_als(TMD2772_obj->client, en);
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB
	if(res){
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}
	return 0;
}

static int als_set_delay(u64 ns)
{
	APS_FUN();
	return 0;
}

static int als_get_data(int* value, int* status)
{
	int err = 0;
#ifdef CUSTOM_KERNEL_SENSORHUB
    SCP_SENSOR_HUB_DATA req;
    int len;
#else
    struct TMD2772_priv *obj = NULL;
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

#ifdef CUSTOM_KERNEL_SENSORHUB
    req.get_data_req.sensorType = ID_LIGHT;
    req.get_data_req.action = SENSOR_HUB_GET_DATA;
    len = sizeof(req.get_data_req);
    err = SCP_sensorHub_req_send(&req, &len, 1);
    if (err)
    {
        APS_ERR("SCP_sensorHub_req_send fail!\n");
    }
    else
    {
        *value = req.get_data_rsp.int16_Data[0];
        *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    }

    if(atomic_read(&TMD2772_obj->trace) & CMC_TRC_PS_DATA)
	{
        APS_LOG("value = %d\n", *value);
        //show data
	}
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	if(!TMD2772_obj)
	{
		APS_ERR("TMD2772_obj is null!!\n");
		return -1;
	}
	obj = TMD2772_obj;
	
	APS_LOG("als_get_data\n");
	
	if((err = TMD2772_read_als(obj->client, &obj->als)))
	{
		err = -1;
	}
	else
	{
		APS_LOG("have get als_get_data\n");
		
		*value = obj->als;//TMD2772_get_als_value(obj, obj->als);
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

	return err;
}

// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int ps_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL

static int ps_enable_nodata(int en)
{
	int res = 0;
#ifdef CUSTOM_KERNEL_SENSORHUB
    SCP_SENSOR_HUB_DATA req;
    int len;
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

    APS_LOG("TMD2772_obj ps enable value = %d\n", en);

#ifdef CUSTOM_KERNEL_SENSORHUB
    req.activate_req.sensorType = ID_PROXIMITY;
    req.activate_req.action = SENSOR_HUB_ACTIVATE;
    req.activate_req.enable = en;
    len = sizeof(req.activate_req);
    res = SCP_sensorHub_req_send(&req, &len, 1);
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	if(!TMD2772_obj)
	{
		APS_ERR("TMD2772_obj is null!!\n");
		return -1;
	}
	res=	TMD2772_enable_ps(TMD2772_obj->client, en);
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB
    
	if(res){
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}
	return 0;

}

static int ps_set_delay(u64 ns)
{
	return 0;
}

static int ps_get_data(int* value, int* status)
{
    int err = 0;
#ifdef CUSTOM_KERNEL_SENSORHUB
    SCP_SENSOR_HUB_DATA req;
    int len;
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

#ifdef CUSTOM_KERNEL_SENSORHUB
    req.get_data_req.sensorType = ID_PROXIMITY;
    req.get_data_req.action = SENSOR_HUB_GET_DATA;
    len = sizeof(req.get_data_req);
    err = SCP_sensorHub_req_send(&req, &len, 1);
    if (err)
    {
        APS_ERR("SCP_sensorHub_req_send fail!\n");
    }
    else
    {
        *value = req.get_data_rsp.int16_Data[0];
        *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    }

    if(atomic_read(&TMD2772_obj->trace) & CMC_TRC_PS_DATA)
	{
        APS_LOG("value = %d\n", *value);
        //show data
	}
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
    if(!TMD2772_obj)
	{
		APS_ERR("TMD2772_obj is null!!\n");
		return -1;
	}

		APS_LOG("ps_get_data \n");

    if((err = TMD2772_read_ps(TMD2772_obj->client, &TMD2772_obj->ps)))
    {
    		APS_LOG("ps_get_data err  = %d\n",err);
        err = -1;
    }
    else
    {
   				 APS_LOG("ps_get_data val  = %d\n",TMD2772_obj->ps);
					*value = TMD2772_get_ps_value(TMD2772_obj, TMD2772_obj->ps);  				 
//        *value = TMD2772_obj->ps;//TMD2772_get_ps_value(TMD2772_obj, TMD2772_obj->ps);

//        res = TMD2772_get_ps_value(TMD2772_obj, TMD2772_obj->ps);
//        ps_report_interrupt_data(res);	
        
        *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    }
    
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB
    
	return 0;
}


/*-----------------------------------i2c operations----------------------------------*/
static int TMD2772_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct TMD2772_priv *obj;

	int err = 0;
	struct als_control_path als_ctl={0};
	struct als_data_path als_data={0};
	struct ps_control_path ps_ctl={0};
	struct ps_data_path ps_data={0};

	#ifdef VENDOR_EDIT 	
	//ziqing.guo@BasicDrv.Sensor,2015/04/12, add for PSD
	init_waitqueue_head(&enable_ps);
	#endif

	APS_FUN();
	
	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	
	memset(obj, 0, sizeof(*obj));
	TMD2772_obj = obj;
	
	obj->hw = tmd_get_cust_alsps_hw();//get custom file data struct
	
	INIT_WORK(&obj->eint_work, TMD2772_eint_work);
#ifdef CUSTOM_KERNEL_SENSORHUB
    INIT_WORK(&obj->init_done_work, TMD2772_init_done_work);
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

	obj->client = client;
	i2c_set_clientdata(client, obj);
	
	/*-----------------------------value need to be confirmed-----------------------------------------*/
	atomic_set(&obj->als_debounce, 200);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 200);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->als_cmd_val, 0xDF);
	atomic_set(&obj->ps_cmd_val,  0xC1);
	atomic_set(&obj->ps_thd_val_high,  obj->hw->ps_threshold_high);
	atomic_set(&obj->ps_thd_val_low,  obj->hw->ps_threshold_low);
	atomic_set(&obj->als_thd_val_high,  obj->hw->als_threshold_high);
	atomic_set(&obj->als_thd_val_low,  obj->hw->als_threshold_low);
	
	obj->enable = 0;
	obj->pending_intr = 0;
	obj->ps_cali = 0;
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);
	/*-----------------------------value need to be confirmed-----------------------------------------*/
	/*Lenovo-sw chenlj2 add 2011-06-03,modified gain 16 to 1/5 accoring to actual thing */
	obj->als_modulus = (400*100*TMD2772_ZOOM_TIME)/(1*50);//(1/Gain)*(400/Tine), this value is fix after init ATIME and CONTROL register value
										//(400)/16*2.72 here is amplify *100 //16
										
	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);
	//set_bit(CMC_BIT_ALS, &obj->enable);
	//set_bit(CMC_BIT_PS, &obj->enable);

	TMD2772_i2c_client = client;
		
	if((err = TMD2772_init_client(client)))
	{
		goto exit_init_failed;
	}
	APS_LOG("TMD2772_init_client() OK!\n");

	if((err = misc_register(&TMD2772_device)))
	{
		APS_ERR("TMD2772_device register failed\n");
		goto exit_misc_device_register_failed;
	}
	APS_LOG("TMD2772_device misc_register OK!\n");

	/*------------------------TMD2772 attribute file for debug--------------------------------------*/
	if((err = TMD2772_create_attr(&(TMD2772_init_info.platform_diver_addr->driver))))
	{
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
	/*------------------------TMD2772 attribute file for debug--------------------------------------*/
	als_ctl.open_report_data= als_open_report_data;
	als_ctl.enable_nodata = als_enable_nodata;
	als_ctl.set_delay  = als_set_delay;
	als_ctl.is_report_input_direct = false;
#ifdef CUSTOM_KERNEL_SENSORHUB
	als_ctl.is_support_batch = obj->hw->is_batch_supported_als;
#else
    als_ctl.is_support_batch = false;
#endif
	
	err = als_register_control_path(&als_ctl);
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	als_data.get_data = als_get_data;
	als_data.vender_div = 100;
	err = als_register_data_path(&als_data);	
	if(err)
	{
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	
	ps_ctl.open_report_data= ps_open_report_data;
	ps_ctl.enable_nodata = ps_enable_nodata;
	ps_ctl.set_delay  = ps_set_delay;
	ps_ctl.is_report_input_direct = true;
#ifdef CUSTOM_KERNEL_SENSORHUB
	ps_ctl.is_support_batch = obj->hw->is_batch_supported_ps;
#else
    ps_ctl.is_support_batch = false;
#endif
	
	err = ps_register_control_path(&ps_ctl);
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	ps_data.get_data = ps_get_data;
	ps_data.vender_div = 100;
	err = ps_register_data_path(&ps_data);	
	if(err)
	{
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	err = batch_register_support_info(ID_LIGHT,als_ctl.is_support_batch, 100, 0);
	if(err)
	{
		APS_ERR("register light batch support err = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}
	
	err = batch_register_support_info(ID_PROXIMITY,ps_ctl.is_support_batch, 100, 0);
	if(err)
	{
		APS_ERR("register proximity batch support err = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	#if defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_EARLYSUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 2,
	obj->early_drv.suspend  = TMD2772_early_suspend,
	obj->early_drv.resume   = TMD2772_late_resume,    
	register_early_suspend(&obj->early_drv);
	#endif

	TMD2772_init_flag =0;
	alsps_dev = ALSPS_TMD_27723;
	APS_LOG("%s: OK\n", __func__);
	return 0;

	exit_create_attr_failed:
	exit_sensor_obj_attach_fail:
	exit_misc_device_register_failed:
		misc_deregister(&TMD2772_device);
	exit_init_failed:
		kfree(obj);
	exit:
	TMD2772_i2c_client = NULL;           
	APS_ERR("%s: err = %d\n", __func__, err);
	TMD2772_init_flag =-1;
	return err;
}

static int TMD2772_i2c_remove(struct i2c_client *client)
{
	int err;	
	/*------------------------TMD2772 attribute file for debug--------------------------------------*/	
	if((err = TMD2772_delete_attr(&(TMD2772_init_info.platform_diver_addr->driver))))
	{
		APS_ERR("TMD2772_delete_attr fail: %d\n", err);
	} 
	/*----------------------------------------------------------------------------------------*/
	
	if((err = misc_deregister(&TMD2772_device)))
	{
		APS_ERR("misc_deregister fail: %d\n", err);    
	}
		
	TMD2772_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;

}

static int TMD2772_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, TMD2772_DEV_NAME);
	return 0;

}

static int TMD2772_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
	APS_FUN();
	return 0;
}

static int TMD2772_i2c_resume(struct i2c_client *client)
{
	APS_FUN();
	return 0;
}

/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int TMD2772_remove(void)
{
	//APS_FUN(); 
	struct alsps_hw *hw = tmd_get_cust_alsps_hw();
	
	TMD2772_power(hw, 0);//*****************  
	
	i2c_del_driver(&TMD2772_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/

static int  TMD2772_local_init(void)
{
    struct alsps_hw *hw = tmd_get_cust_alsps_hw();
	//printk("fwq loccal init+++\n");

	TMD2772_power(hw, 1);
	if(i2c_add_driver(&TMD2772_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	}
	if(-1 == TMD2772_init_flag)
	{
	   return -1;
	}
	//printk("fwq loccal init---\n");
	return 0;
}


/*----------------------------------------------------------------------------*/
static int __init TMD2772_init(void)
{
	struct alsps_hw *hw = tmd_get_cust_alsps_hw();
	APS_FUN();
	APS_LOG("%s: i2c_number=%d, i2c_addr: 0x%x\n", __func__, hw->i2c_num, hw->i2c_addr[0]);
	i2c_register_board_info(hw->i2c_num, &i2c_TMD2772, 1);
	alsps_driver_add(&TMD2772_init_info);
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit TMD2772_exit(void)
{
	APS_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(TMD2772_init);
module_exit(TMD2772_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("yucong xiong");
MODULE_DESCRIPTION("TMD2772 driver");
MODULE_LICENSE("GPL");

