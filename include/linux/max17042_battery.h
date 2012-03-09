/*
 *  Copyright (C) 2011 Samsung Electronics
 *  Ikkeun Kim <iks.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MAX17042_BATTERY_H_
#define __MAX17042_BATTERY_H_

enum {
	REQ_FULL_CHARGE_COMPENSATION = 0,
	REQ_VF_FULLCAP_RANGE_CHECK,
	REQ_CAP_CORRUPTION_CHECK,
	REQ_LOW_BATTERY_COMPENSATION,
	REQ_ADJUST_CAPACITY_RESTART,
	REQ_TEST_MODE_INTERFACE,
};

typedef enum {
	TEST_MODE_FUEL_GAUGE_CHECK = 0,
	TEST_MODE_RESET_CAPACITY,
	TEST_MODE_QUICK_START_CMD,
	TEST_MODE_DUMP_FG_REGISTER,
	TEST_MODE_BATTERY_TYPE_CHECK,
} max17042_test_mode_type_t ;

typedef enum {
	UNKNOWN_TYPE = 0,
	SDI_BATTERY_TYPE,
	ATL_BATTERY_TYPE
} battery_type_t;

struct max17042_callbacks {
	void (*full_charge_comp)(struct max17042_callbacks *ptr,
		u32 is_recharging, u32 pre_update);
	void (*vf_fullcap_check)(struct max17042_callbacks *ptr);
	int (*corruption_check)(struct max17042_callbacks *ptr);
	void (*low_batt_comp)(struct max17042_callbacks *ptr, int enable);
	int (*adjust_capacity)(struct max17042_callbacks *ptr);
	int (*test_mode_request)(struct max17042_callbacks *ptr,
		max17042_test_mode_type_t mode, int arg);
};

struct max17042_platform_data {
	int (*battery_online)(void);
	int (*charger_online)(void);
	int (*charger_enable)(void);
	int (*power_supply_register)(struct device *parent,
		struct power_supply *psy);
	void (*power_supply_unregister)(struct power_supply *psy);
	void (*force_update_status)(void);
	void (*register_callbacks)(struct max17042_callbacks *ptr);
};

#endif
