/*
 * sec_battery.h
 *
 * Battery measurement code for samsung platform.
 *
 * Copyright (C) 2010 Samsung Electronics.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __SEC_BATTERY_H
#define __SEC_BATTERY_H

/*
 * Battery Table
 */
#define BATT_CAL		2447	/* 3.60V */

#define BATT_MAXIMUM		406	/* 4.176V */
#define BATT_FULL		353	/* 4.10V  */
#define BATT_SAFE_RECHARGE	353	/* 4.10V */
#define BATT_ALMOST_FULL	188	/* 3.8641V */
#define BATT_HIGH		112	/* 3.7554V */
#define BATT_MED		66	/* 3.6907V */
#define BATT_LOW		43	/* 3.6566V */
#define BATT_CRITICAL		8	/* 3.6037V */
#define BATT_MINIMUM		(-28)	/* 3.554V */
#define BATT_OFF		(-128)	/* 3.4029V */


#define SAMSUNG_LPM_MODE
//#define __FUEL_GAUGES_IC__ 


enum cable_type_t {
	CABLE_TYPE_NONE = 0,
	CABLE_TYPE_USB,
	CABLE_TYPE_AC,
	CABLE_TYPE_IMPROPER_AC,
};

enum charging_status_type_t {
	CHARGING_STATUS_ERROR = -1,
	CHARGING_STATUS_NONE = 0,
	CHARGING_STATUS_ACTIVE,
	CHARGING_STATUS_FULL,
};

/**
 * sec_battery_adc_table_data
 * @adc_value : temperture adc value
 * @temperature : temperature(C) * 10
 */
struct sec_battery_adc_table_data {
	int adc_value;
	int temperature;
};

struct sec_battery_callbacks {
	void (*set_cable)(struct sec_battery_callbacks *ptr,
		enum cable_type_t status);
	void (*set_status)(struct sec_battery_callbacks *ptr,
		enum charging_status_type_t status);
	void (*force_update)(struct sec_battery_callbacks *ptr);
};

struct charger_device {
	int (*charging_control) (int en, int cable_status);
	int (*get_charging_status) (void);
	void (*set_charging_status) (int status);
	int (*get_connection_status) (void);
};

struct sec_battery_platform_data {
	struct power_supply *psy_fuelgauge;
	struct charger_device *pmic_charger;
	struct charger_device *external_charger;
	void (*register_callbacks)(struct sec_battery_callbacks *ptr);
	int (*fuelgauge_cb)(int request_mode, int arg1, int arg2);
	bool (*get_jig_status)(void);
	struct sec_battery_adc_table_data *adc_table;
	int adc_array_size;
};


/*
 * ADC channel
 */
enum adc_channel_type {
	ADC_TEMPERATURE = 0,
	ADC_CH1,
	ADC_LCD_ID,
	ADC_EAR_ADC_35,
	ADC_ACCESSORY_ID,
	ADC_REMOTE_SENSE,
	ADC_AP_CHECK_2,
	ADC_AP_CHECK_1,
	ADC_CH8,
	ADC_CH9,
	ENDOFADC
};


enum {
	BATT_VOL = 0,
	BATT_TEMP,
	BATT_CHARGING_SOURCE,
	BATT_FG_SOC,
#ifdef CONFIG_BATTERY_MAX17042
	BATT_FG_CHECK,
	BATT_RESET_SOC,
	BATT_RESET_CAP,
	BATT_FG_REG,
	BATT_BATT_TYPE,
#endif
#ifdef SAMSUNG_LPM_MODE
	CHARGING_MODE_BOOTING,
	BATT_TEMP_CHECK,
	BATT_FULL_CHECK,
#endif
};


#define TOTAL_CHARGING_TIME	(6*60*60)	/* 6 hours */
#define TOTAL_RECHARGING_TIME	  (90*60)	/* 1.5 hours */

#define COMPENSATE_VIBRATOR		19
#define COMPENSATE_CAMERA		25
#define COMPENSATE_MP3			17
#define COMPENSATE_VIDEO			28
#define COMPENSATE_VOICE_CALL_2G	13
#define COMPENSATE_VOICE_CALL_3G	14
#define COMPENSATE_DATA_CALL		25
#define COMPENSATE_LCD			0
#define COMPENSATE_TA				0
#define COMPENSATE_CAM_FALSH		0
#define COMPENSATE_BOOTING		52

#define SOC_LB_FOR_POWER_OFF		27

#define RECHARGE_COND_VOLTAGE	4135000
#define RECHARGE_COND_TIME		(30*1000)	/* 30 seconds */

#endif
