/* 
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
/*
 * Definitions for TMD2772 als/ps sensor chip.
 */
#ifndef __TMD2772_H__
#define __TMD2772_H__

#include <linux/ioctl.h>

extern int TMD2772_CMM_PPCOUNT_VALUE;
extern int TMD2772_ZOOM_TIME;
extern int TMD2772_CMM_CONTROL_VALUE;


#define TMD2772_REG_CS_CONF 		0X00
#define TMD2772_REG_ALS_THDH 		0X01
#define TMD2772_REG_ALS_THDL 		0X02
#define TMD2772_REG_PS_CONF1_2		0X03

#define TMD2772_REG_PS_CANC			0X06
#define TMD2772_REG_PS_DATA			0X07
#define TMD2772_REG_ALS_DATA		0X09
#define TMD2772_REG_INT_FLAG		0X0C
#define TMD2772_REG_ID_MODE			0X0D

#define TMD2772_CMM_ENABLE 		0X80
#define TMD2772_CMM_ATIME 		0X81
#define TMD2772_CMM_PTIME 		0X82
#define TMD2772_CMM_WTIME 		0X83
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
#define TMD2772_CMM_INT_LOW_THD_LOW   0X88
#define TMD2772_CMM_INT_LOW_THD_HIGH  0X89
#define TMD2772_CMM_INT_HIGH_THD_LOW  0X8A
#define TMD2772_CMM_INT_HIGH_THD_HIGH 0X8B
#define TMD2772_CMM_Persistence       0X8C
#define TMD2772_CMM_STATUS            0X93
#define TMD2772_CMM_ID           					0X92

#define TAOS_TRITON_CMD_REG           0X80
#define TAOS_TRITON_CMD_SPL_FN        0x60

#define TMD2772_CMM_CONFIG 			0X8D
#define TMD2772_CMM_PPCOUNT 		0X8E
#define TMD2772_CMM_CONTROL 		0X8F

#define TMD2772_CMM_PDATA_L 		0X98
#define TMD2772_CMM_PDATA_H 		0X99
#define TMD2772_CMM_C0DATA_L 	0X94
#define TMD2772_CMM_C0DATA_H 	0X95
#define TMD2772_CMM_C1DATA_L 	0X96
#define TMD2772_CMM_C1DATA_H 	0X97

/*TMD2772 related driver tag macro*/
#define TMD2772_SUCCESS				 		 0
#define TMD2772_ERR_I2C						-1
#define TMD2772_ERR_STATUS					-3
#define TMD2772_ERR_SETUP_FAILURE			-4
#define TMD2772_ERR_GETGSENSORDATA			-5
#define TMD2772_ERR_IDENTIFICATION			-6

/*----------------------------------------------------------------------------*/
typedef enum{
    TMD2772_NOTIFY_PROXIMITY_CHANGE = 1,
}TMD2772_NOTIFY_TYPE;
/*----------------------------------------------------------------------------*/
typedef enum{
    TMD2772_CUST_ACTION_SET_CUST = 1,
    TMD2772_CUST_ACTION_CLR_CALI,
    TMD2772_CUST_ACTION_SET_CALI,
    TMD2772_CUST_ACTION_SET_PS_THRESHODL,
    TMD2772_CUST_ACTION_SET_EINT_INFO,
    TMD2772_CUST_ACTION_GET_ALS_RAW_DATA,
    TMD2772_CUST_ACTION_GET_PS_RAW_DATA,
}TMD2772_CUST_ACTION;
/*----------------------------------------------------------------------------*/
typedef struct
{
    uint16_t    action;
}TMD2772_CUST;
/*----------------------------------------------------------------------------*/
typedef struct
{
    uint16_t    action;
    uint16_t    part;
    int32_t    data[0];
}TMD2772_SET_CUST;
/*----------------------------------------------------------------------------*/
typedef TMD2772_CUST TMD2772_CLR_CALI;
/*----------------------------------------------------------------------------*/
typedef struct
{
    uint16_t    action;
    int32_t     cali;
}TMD2772_SET_CALI;
/*----------------------------------------------------------------------------*/
typedef struct
{
    uint16_t    action;
    int32_t     threshold[2];
}TMD2772_SET_PS_THRESHOLD;
/*----------------------------------------------------------------------------*/
typedef struct
{
    uint16_t    action;
    uint32_t    gpio_pin;
    uint32_t    gpio_mode;
    uint32_t    eint_num;
    uint32_t    eint_is_deb_en;
    uint32_t    eint_type;
}TMD2772_SET_EINT_INFO;
/*----------------------------------------------------------------------------*/
typedef struct
{
    uint16_t    action;
    uint16_t    als;
}TMD2772_GET_ALS_RAW_DATA;
/*----------------------------------------------------------------------------*/
typedef struct
{
    uint16_t    action;
    uint16_t    ps;
} TMD2772_GET_PS_RAW_DATA;
/*----------------------------------------------------------------------------*/
typedef union
{
    uint32_t                    data[10];
    TMD2772_CUST                cust;
    TMD2772_SET_CUST            setCust;
    TMD2772_CLR_CALI            clearCali;
    TMD2772_SET_CALI            setCali;
    TMD2772_SET_PS_THRESHOLD    setPSThreshold;
    TMD2772_SET_EINT_INFO       setEintInfo;
    TMD2772_GET_ALS_RAW_DATA    getALSRawData;
    TMD2772_GET_PS_RAW_DATA     getPSRawData;
}TMD2772_CUST_DATA;
/*----------------------------------------------------------------------------*/

extern struct alsps_hw* tmd_get_cust_alsps_hw(void);

#endif

