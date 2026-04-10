#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_alsps.h>

static struct ps_adjust_para cust_ps_adjust_para_tmd2772 = {
	.ps_up = 60,
	.ps_thd_low_notrend = 80,
	.ps_thd_high_notrend = 120,
	.ps_thd_low_trend = 40,
	.ps_thd_high_trend = 60,
	.ps_thd_low_highlight = 600,
	.ps_thd_high_highlight = 650,
	.ps_adjust_min = 0,
	.ps_adjust_max = 850,
	.highlight_limit = 8000,
	.sampling_time = 60,
	.sampling_count = 5,
	.dirty_adjust_limit = 900,
	.dirty_adjust_low_thd = 250,
	.dirty_adjust_high_thd = 300

};

static struct alsps_hw cust_alsps_hw = {
    .i2c_num    = 0,
    .polling_mode_ps =0,
    .polling_mode_als =1,
    .power_id   = MT65XX_POWER_NONE,    /*LDO is not used*/
    .power_vol  = VOL_DEFAULT,          /*LDO is not used*/
    //.i2c_addr   = {0x0C, 0x48, 0x78, 0x00},
    .als_level  = {0, 471, 1058, 1703, 3883, 10171, 10443, 15445, 28499, 35153, 41421, 59194, 65535, 65535, 65535},
    .als_value  = {0, 133, 303, 501, 1002, 2001, 3355, 5001, 8008, 10010, 12000, 16010, 20010, 20010, 20010, 20010},
    .ps_threshold_high =650,
    .ps_threshold_low = 600,
    .p_ps_adjust_para = &cust_ps_adjust_para_tmd2772,
    .is_batch_supported_ps = false,
    .is_batch_supported_als = false,
};
struct alsps_hw *tmd_get_cust_alsps_hw(void) {
    cust_alsps_hw.p_ps_adjust_para = &cust_ps_adjust_para_tmd2772;
    return &cust_alsps_hw;
}

int TMD2772_CMM_PPCOUNT_VALUE = 0x06;//20;//0x09;
int TMD2772_ZOOM_TIME = 10;
int TMD2772_CMM_CONTROL_VALUE = 0x20;
