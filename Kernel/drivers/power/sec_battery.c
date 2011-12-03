/*
 * linux/drivers/power/sec_battery.c
 *
 * Battery measurement code for samsung platform.
 *
 * based on palmtx_battery.c
 *
 * Copyright (C) 2010 Samsung Electronics.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <asm/mach-types.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/sec_battery.h>
#include <linux/max17042_battery.h>
#include <linux/fsa9480.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <mach/battery.h>
#include <mach/hardware.h>
#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/regs-gpio.h>
#include <mach/adc.h>
#include <mach/gpio-p1.h>
#include <plat/gpio-cfg.h>
#include <linux/android_alarm.h>
#include <mach/regs-clock.h>
#include <asm/io.h>

#define TEMPERATURE_FROM_FUELGAUGE

#define POLLING_INTERVAL	1000
#define ADC_TOTAL_COUNT		10
#define ADC_DATA_ARR_SIZE	6

#define OFFSET_VIBRATOR_ON		(0x1 << 0)
#define OFFSET_CAMERA_ON		(0x1 << 1)
#define OFFSET_MP3_PLAY			(0x1 << 2)
#define OFFSET_VIDEO_PLAY		(0x1 << 3)
#define OFFSET_VOICE_CALL_2G		(0x1 << 4)
#define OFFSET_VOICE_CALL_3G		(0x1 << 5)
#define OFFSET_DATA_CALL		(0x1 << 6)
#define OFFSET_LCD_ON			(0x1 << 7)
#define OFFSET_TA_ATTACHED		(0x1 << 8)
#define OFFSET_CAM_FLASH		(0x1 << 9)
#define OFFSET_BOOTING			(0x1 << 10)

#define FAST_POLL			(1 * 40)
#define SLOW_POLL			(10 * 60)

#define DISCONNECT_BAT_FULL		0x1
#define DISCONNECT_TEMP_OVERHEAT	0x2
#define DISCONNECT_TEMP_FREEZE		0x4
#define DISCONNECT_OVER_TIME		0x8

#define HIGH_BLOCK_TEMP			550
#define HIGH_RECOVER_TEMP		450
#define LOW_BLOCK_TEMP			0
#define LOW_RECOVER_TEMP		30

struct battery_info {
	s32 batt_vol;		/* Battery voltage from ADC */
	u32 batt_temp;		/* Battery Temperature (C) from ADC */
	u32 batt_temp_adc;	/* Battery Temperature ADC value */
	u32 batt_health;	/* Battery Health (Authority) */
	u32 dis_reason;
	u32 batt_vcell;
	u32 batt_soc;
	u32 charging_status;
	bool batt_is_full;      /* 0 : Not full 1: Full */
	bool batt_improper_ta;  /* 1: improper ta */	
};

struct adc_sample_info {
	unsigned int cnt;
	int total_adc;
	int average_adc;
	int adc_arr[ADC_TOTAL_COUNT];
	int index;
};

struct chg_data {
	struct device		*dev;
	struct work_struct	bat_work;
	struct sec_battery_platform_data *pdata;

	struct power_supply	psy_bat;
	struct power_supply	psy_usb;
	struct power_supply	psy_ac;
	struct alarm		alarm;
	struct workqueue_struct *monitor_wqueue;
	struct wake_lock	vbus_wake_lock;
	struct wake_lock	work_wake_lock;
	struct adc_sample_info	adc_sample[ENDOFADC];
	struct battery_info	bat_info;
	struct mutex		mutex;

	enum cable_type_t	cable_status;
	enum charging_status_type_t	charging_status;
	bool			charging;
	bool			is_recharging;
	bool			set_charge_timeout;
	int			present;
	int			timestamp;
	int			set_batt_full;
	unsigned long		discharging_time;
	unsigned int		polling_interval;
	int			slow_poll;
	ktime_t			last_poll;
	struct sec_battery_callbacks callbacks;
#ifdef CONFIG_BATTERY_MAX17042
	battery_type_t		battery_type;
	bool			low_batt_boot_flag;
	struct delayed_work	full_chg_work;
	int			check_start_vol;
#endif
#ifdef SAMSUNG_LPM_MODE
	u32 charging_mode_booting;
#endif	
};

static char *supply_list[] = {
	"battery",
};

static enum power_supply_property sec_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TECHNOLOGY,
};

static enum power_supply_property sec_power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static ssize_t sec_bat_show_attrs(struct device *dev,
				  struct device_attribute *attr, char *buf);

static ssize_t sec_bat_store_attrs(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count);

#define SEC_BATTERY_ATTR(_name)								\
{											\
	.attr = { .name = #_name, .mode = 0664, .owner = THIS_MODULE },	\
	.show = sec_bat_show_attrs,							\
	.store = sec_bat_store_attrs,								\
}

static struct device_attribute sec_battery_attrs[] = {
  	SEC_BATTERY_ATTR(batt_vol),
	SEC_BATTERY_ATTR(batt_temp),
	SEC_BATTERY_ATTR(charging_source),
	SEC_BATTERY_ATTR(fg_soc),
#ifdef CONFIG_BATTERY_MAX17042
	SEC_BATTERY_ATTR(fg_check),
	SEC_BATTERY_ATTR(reset_soc),
	SEC_BATTERY_ATTR(reset_cap),
	SEC_BATTERY_ATTR(fg_reg),
	SEC_BATTERY_ATTR(batt_type),
#endif
#ifdef SAMSUNG_LPM_MODE
	SEC_BATTERY_ATTR(charging_mode_booting),
	SEC_BATTERY_ATTR(batt_temp_check),
	SEC_BATTERY_ATTR(batt_full_check),
#endif
};


#ifdef SAMSUNG_LPM_MODE
void charging_mode_set(struct chg_data *chg, u32 val)
{
	chg->charging_mode_booting = val;
}

int get_charging_status(struct chg_data *chg)
{
	if(chg->cable_status==CABLE_TYPE_USB || chg->cable_status==CABLE_TYPE_AC)
		return 1;
	else
		return 0;
}

int get_boot_charger_info(void)
{
	return readl(S5P_INFORM5);
}

void p1_lpm_mode_check(struct chg_data *chg)
{
	
	if(get_boot_charger_info())
	{
		printk("%s : VDCIN (%d)\n", __func__, get_charging_status(chg));
		if(get_charging_status(chg)) {
			charging_mode_set(chg, 1);
		}
		else{
			if (pm_power_off)
				pm_power_off();
		}
	}
	else
	{
		charging_mode_set(chg, 0);
	}
}
#endif

static void sec_bat_set_cable(struct sec_battery_callbacks *ptr,
	enum cable_type_t status)
{
	printk("%s : cable_type = %d ( 0 : NONE, 1: USB, 2: TA) \n",__func__,status);
	struct chg_data *chg = container_of(ptr, struct chg_data, callbacks);
	chg->cable_status = status;
	power_supply_changed(&chg->psy_ac);
	power_supply_changed(&chg->psy_usb);
	wake_lock(&chg->work_wake_lock);
	queue_work(chg->monitor_wqueue, &chg->bat_work);
}

static void sec_bat_set_status(struct sec_battery_callbacks *ptr,
	enum charging_status_type_t status)
{
	printk("%s : charging_status = %d ( -1 : ERROR, 0 : NONE, 1: ACTIVE, 2: FULL) \n",__func__,status);
	struct chg_data *chg = container_of(ptr, struct chg_data, callbacks);
	chg->charging_status = status;
	int vdc_status = 0;

	if(chg->pdata && chg->pdata->pmic_charger &&
			chg->pdata->pmic_charger->get_connection_status)
		vdc_status = chg->pdata->pmic_charger->get_connection_status();

	if (vdc_status && chg->charging_status == CHARGING_STATUS_FULL) {  // Charger is connected
		/*	Normal case : SOC >= 70 && Full charge interrupt occured
			Absolute timer expiration : SOC condition is not considered */
		if(chg->bat_info.batt_soc >= 70 || chg->set_charge_timeout) {
			chg->set_batt_full = 1;
			chg->bat_info.batt_is_full = true;

#ifdef CONFIG_BATTERY_MAX17042
			// full charge compensation - pre update.
			if(!chg->set_charge_timeout) {
				if(chg->pdata && chg->pdata->fuelgauge_cb)
					chg->pdata->fuelgauge_cb(REQ_FULL_CHARGE_COMPENSATION,
						chg->is_recharging, 1);  // pre_update

				cancel_delayed_work(&chg->full_chg_work);
				schedule_delayed_work(&chg->full_chg_work, msecs_to_jiffies(1000));  // after 1sec
			}
#endif
		}
	}

	if(!chg->set_charge_timeout) {  // bat_work is already running in case of charging timeout.
		wake_lock(&chg->work_wake_lock);
		queue_work(chg->monitor_wqueue, &chg->bat_work);
	}
}

static void sec_bat_force_update(struct sec_battery_callbacks *ptr)
{
	printk("%s : force update called\n",__func__);
	struct chg_data *chg = container_of(ptr, struct chg_data, callbacks);

	// Just update now.
	wake_lock(&chg->work_wake_lock);
	queue_work(chg->monitor_wqueue, &chg->bat_work);
}

#ifdef CONFIG_BATTERY_MAX17042
static bool check_UV_charging_case(struct chg_data *chg)
{
	union power_supply_propval value;
	int vcell = 0;
	int current_now = 0;
	int threshold = 0;

	if (chg->pdata && chg->pdata->psy_fuelgauge &&
	    chg->pdata->psy_fuelgauge->get_property) {
		chg->pdata->psy_fuelgauge->get_property(chg->pdata->psy_fuelgauge,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &value);
		vcell = (value.intval / 1000);

		chg->pdata->psy_fuelgauge->get_property(chg->pdata->psy_fuelgauge,
			POWER_SUPPLY_PROP_CURRENT_NOW, &value);
		current_now = value.intval;
	}

	if(chg->battery_type == SDI_BATTERY_TYPE)
		threshold = 3300 + ((current_now * 17) / 100);
	else if(chg->battery_type == ATL_BATTERY_TYPE)
		threshold = 3300 + ((current_now * 13) / 100);

	printk("%s: current(%dmA), vcell(%dmV), threshold(%dmV)\n", __func__, current_now, vcell, threshold);
	
	if(vcell <= threshold)
		return true;
	else
		return false;
}

static void full_comp_work_handler(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct chg_data *chg = container_of(dw, struct chg_data, full_chg_work);
	union power_supply_propval value;
	int avg_current = 0;

	if(chg->pdata && chg->pdata->psy_fuelgauge && chg->pdata->psy_fuelgauge->get_property)
		chg->pdata->psy_fuelgauge->get_property(chg->pdata->psy_fuelgauge,
			POWER_SUPPLY_PROP_CURRENT_AVG, &value);
	avg_current = value.intval;

	if(avg_current >= 25) {  // Real threshold is 25.625mA
		cancel_delayed_work(&chg->full_chg_work);
		schedule_delayed_work(&chg->full_chg_work, msecs_to_jiffies(1000));  // after 1sec
	}
	else {
		printk("%s : full charge compensation start (avg_current : %d)\n", __func__, avg_current);
		
		// full charge compensation - final update.
		if(chg->pdata && chg->pdata->fuelgauge_cb)
			chg->pdata->fuelgauge_cb(REQ_FULL_CHARGE_COMPENSATION,
				chg->is_recharging, 0);  // final update
	}
}
#endif

static int sec_bat_get_property(struct power_supply *bat_ps,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct chg_data *chg = container_of(bat_ps,
				struct chg_data, psy_bat);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = chg->bat_info.charging_status;
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = chg->bat_info.batt_health;
		break;

	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = chg->present;
		break;

	case POWER_SUPPLY_PROP_TEMP:
		val->intval = chg->bat_info.batt_temp;
		break;

	case POWER_SUPPLY_PROP_ONLINE:
		/* battery is always online */
		val->intval = 1;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (chg->pdata && chg->pdata->psy_fuelgauge &&
			chg->pdata->psy_fuelgauge->get_property &&
			chg->pdata->psy_fuelgauge->get_property(
				chg->pdata->psy_fuelgauge, psp, val) < 0)
			return -EINVAL;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		if (chg->pdata && chg->pdata->psy_fuelgauge &&
			chg->pdata->psy_fuelgauge->get_property &&
			chg->pdata->psy_fuelgauge->get_property(
				chg->pdata->psy_fuelgauge, psp, val) < 0)
			return -EINVAL;

		// Capacity cannot exceed 100%.
		if(val->intval >= 100)
			val->intval = 100;

		// Update 100% only full charged with TA Charger.
		if (chg->bat_info.batt_is_full &&
		    (chg->cable_status == CABLE_TYPE_AC && !chg->bat_info.batt_improper_ta))
			val->intval = 100;
		else {
			if(val->intval == 100)
				val->intval = 99;
		}

#ifdef CONFIG_BATTERY_MAX17042
		if(chg->low_batt_boot_flag)
			val->intval = 0;
#endif
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

static int sec_usb_get_property(struct power_supply *ps,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct chg_data *chg = container_of(ps, struct chg_data, psy_usb);

	if (psp != POWER_SUPPLY_PROP_ONLINE)
		return -EINVAL;

	/* Set enable=1 only if the USB charger is connected */
	val->intval = ((chg->cable_status == CABLE_TYPE_USB) &&
			chg->pdata->pmic_charger->get_connection_status());

	return 0;
}

static int sec_ac_get_property(struct power_supply *ps,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct chg_data *chg = container_of(ps, struct chg_data, psy_ac);

	if (psp != POWER_SUPPLY_PROP_ONLINE)
		return -EINVAL;

	/* Set enable=1 only if the AC charger is connected */
	val->intval = (chg->cable_status == CABLE_TYPE_AC);

	return 0;
}

static int sec_bat_get_adc_data(enum adc_channel_type adc_ch)
{
	int adc_data;
	int adc_max = 0;
	int adc_min = 0;
	int adc_total = 0;
	int i;

	for (i = 0; i < ADC_DATA_ARR_SIZE; i++) {
		adc_data = s3c_adc_get_adc_data(adc_ch);

		if (i != 0) {
			if (adc_data > adc_max)
				adc_max = adc_data;
			else if (adc_data < adc_min)
				adc_min = adc_data;
		} else {
			adc_max = adc_data;
			adc_min = adc_data;
		}
		adc_total += adc_data;
	}

	return (adc_total - adc_max - adc_min) / (ADC_DATA_ARR_SIZE - 2);
}

static unsigned long calculate_average_adc(enum adc_channel_type channel,
			int adc, struct chg_data *chg)
{
	unsigned int cnt = 0;
	int total_adc = 0;
	int average_adc = 0;
	int index = 0;

	cnt = chg->adc_sample[channel].cnt;
	total_adc = chg->adc_sample[channel].total_adc;

	if (adc <= 0) {
		pr_err("%s : invalid adc : %d\n", __func__, adc);
		adc = chg->adc_sample[channel].average_adc;
	}

	if (cnt < ADC_TOTAL_COUNT) {
		chg->adc_sample[channel].adc_arr[cnt] = adc;
		chg->adc_sample[channel].index = cnt;
		chg->adc_sample[channel].cnt = ++cnt;

		total_adc += adc;
		average_adc = total_adc / cnt;
	} else {
		index = chg->adc_sample[channel].index;
		if (++index >= ADC_TOTAL_COUNT)
			index = 0;

		total_adc = total_adc - chg->adc_sample[channel].adc_arr[index]
				+ adc;
		average_adc = total_adc / ADC_TOTAL_COUNT;

		chg->adc_sample[channel].adc_arr[index] = adc;
		chg->adc_sample[channel].index = index;
	}

	chg->adc_sample[channel].total_adc = total_adc;
	chg->adc_sample[channel].average_adc = average_adc;

	chg->bat_info.batt_temp_adc = average_adc;

	return average_adc;
}

static unsigned long sec_read_temp(struct chg_data *chg)
{
	int adc = 0;

	adc = sec_bat_get_adc_data(ADC_TEMPERATURE);

	return calculate_average_adc(ADC_TEMPERATURE, adc, chg);
}

static int sec_get_bat_temp(struct chg_data *chg)
{
	int temp = 0;
	int temp_adc = sec_read_temp(chg);
	int health = chg->bat_info.batt_health;
	int left_side = 0;
	int right_side = chg->pdata->adc_array_size - 1;
	int mid;
	union power_supply_propval value;

	while (left_side <= right_side) {
		mid = (left_side + right_side) / 2 ;
		if (mid == 0 || mid == chg->pdata->adc_array_size - 1 ||
				(chg->pdata->adc_table[mid-1].adc_value >= temp_adc &&
				chg->pdata->adc_table[mid].adc_value < temp_adc)) {
			temp = chg->pdata->adc_table[mid-1].temperature;
			break;
		}
		else if (temp_adc - chg->pdata->adc_table[mid].adc_value > 0)
			right_side = mid - 1;
		else
			left_side = mid + 1;
	}

#ifdef TEMPERATURE_FROM_FUELGAUGE
	if (chg->pdata && chg->pdata->psy_fuelgauge &&
			 chg->pdata->psy_fuelgauge->get_property) {
		chg->pdata->psy_fuelgauge->get_property(chg->pdata->psy_fuelgauge,
		POWER_SUPPLY_PROP_TEMP, &value);
	}

	chg->bat_info.batt_temp = value.intval;
#else
	chg->bat_info.batt_temp = temp;
#endif

	if (chg->bat_info.batt_temp >= HIGH_BLOCK_TEMP) {
		if (health != POWER_SUPPLY_HEALTH_OVERHEAT &&
		    health != POWER_SUPPLY_HEALTH_UNSPEC_FAILURE)
			chg->bat_info.batt_health =
					POWER_SUPPLY_HEALTH_OVERHEAT;
	} else if (chg->bat_info.batt_temp <= HIGH_RECOVER_TEMP &&
	    chg->bat_info.batt_temp >= LOW_RECOVER_TEMP) {
		if (health == POWER_SUPPLY_HEALTH_OVERHEAT ||
		    health == POWER_SUPPLY_HEALTH_COLD)
			chg->bat_info.batt_health =
					POWER_SUPPLY_HEALTH_GOOD;
	} else if (chg->bat_info.batt_temp <= LOW_BLOCK_TEMP) {
		if (health != POWER_SUPPLY_HEALTH_COLD &&
		    health != POWER_SUPPLY_HEALTH_UNSPEC_FAILURE)
			chg->bat_info.batt_health =
				POWER_SUPPLY_HEALTH_COLD;
	}

	pr_debug("%s : batt_temp = %d, temp = %d, adc = %d\n", __func__, chg->bat_info.batt_temp, temp, temp_adc);

	return chg->bat_info.batt_temp;
}

static int chk_cnt = 0;
static void sec_bat_discharge_reason(struct chg_data *chg)
{
	int discharge_reason;
	ktime_t ktime;
	struct timespec cur_time;
	union power_supply_propval value;
	bool vdc_status;
	int recover_flag = 0;

	if (chg->pdata && chg->pdata->psy_fuelgauge &&
	     chg->pdata->psy_fuelgauge->get_property) {
		chg->pdata->psy_fuelgauge->get_property(chg->pdata->psy_fuelgauge,
		POWER_SUPPLY_PROP_VOLTAGE_NOW, &value);
		chg->bat_info.batt_vcell = value.intval;

		chg->pdata->psy_fuelgauge->get_property(chg->pdata->psy_fuelgauge,
		POWER_SUPPLY_PROP_CAPACITY, &value);
		chg->bat_info.batt_soc = value.intval;
	}

#ifdef CONFIG_BATTERY_MAX17042
	// Corruption check algorithm
	if(chg->pdata && chg->pdata->get_jig_status &&
	    !chg->pdata->get_jig_status()) {  // Not using Jig.
		if(chg->pdata && chg->pdata->fuelgauge_cb)
			recover_flag = chg->pdata->fuelgauge_cb(REQ_CAP_CORRUPTION_CHECK, 0, 0);
	}

	// Get recoverd values if recover flag is set.
	if(recover_flag) {
		if (chg->pdata && chg->pdata->psy_fuelgauge &&
		     chg->pdata->psy_fuelgauge->get_property) {
			chg->pdata->psy_fuelgauge->get_property(chg->pdata->psy_fuelgauge,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &value);
			chg->bat_info.batt_vcell = value.intval;
		
			chg->pdata->psy_fuelgauge->get_property(chg->pdata->psy_fuelgauge,
			POWER_SUPPLY_PROP_CAPACITY, &value);
			chg->bat_info.batt_soc = value.intval;
		}
	}

	// VF FullCap range check algorithm.
	if(!(chk_cnt++ % 10)) {  // every 5 minutes
		if(chg->pdata && chg->pdata->fuelgauge_cb)
			recover_flag = chg->pdata->fuelgauge_cb(REQ_VF_FULLCAP_RANGE_CHECK, 0, 0);
		chk_cnt = 1;
	}

	// SOC gauging restart algorithm
	if(chg->low_batt_boot_flag) {
		// 1. Check charger connection status
		if(chg->pdata && chg->pdata->pmic_charger &&
		    chg->pdata->pmic_charger->get_connection_status)
			vdc_status = chg->pdata->pmic_charger->get_connection_status();

		// 2. Restart SOC gauging after (vcell >= threshold) condition satisfied.
		if(vdc_status &&  !check_UV_charging_case(chg)) {
			if(chg->pdata && chg->pdata->fuelgauge_cb)
				chg->pdata->fuelgauge_cb(REQ_ADJUST_CAPACITY_RESTART, 0, 0);
			chg->low_batt_boot_flag = false;
		}

		// If charger removed, then clear flag.
		if(!vdc_status)
			chg->low_batt_boot_flag = false;
	}

	// Start low battery compensation algorithm
	if(!chg->charging && ((chg->bat_info.batt_vcell/1000) <= chg->check_start_vol)) {
		if(chg->pdata && chg->pdata->get_jig_status &&
		    !chg->pdata->get_jig_status()) {  // Not using Jig.
			if (chg->pdata && chg->pdata->fuelgauge_cb)
				chg->pdata->fuelgauge_cb(REQ_LOW_BATTERY_COMPENSATION, 1, 0);
		}
	}
#endif

	discharge_reason = chg->bat_info.dis_reason & 0xf;

	if (discharge_reason == DISCONNECT_BAT_FULL &&
			chg->bat_info.batt_vcell < RECHARGE_COND_VOLTAGE) {
		chg->bat_info.dis_reason &= ~DISCONNECT_BAT_FULL;
		chg->is_recharging = true;
	}

	if (discharge_reason == DISCONNECT_TEMP_OVERHEAT &&
			chg->bat_info.batt_temp <=
			HIGH_RECOVER_TEMP)
		chg->bat_info.dis_reason &= ~DISCONNECT_TEMP_OVERHEAT;

	if (discharge_reason == DISCONNECT_TEMP_FREEZE &&
			chg->bat_info.batt_temp >=
			LOW_RECOVER_TEMP)
		chg->bat_info.dis_reason &= ~DISCONNECT_TEMP_FREEZE;

	if (discharge_reason == DISCONNECT_OVER_TIME &&
			chg->bat_info.batt_vcell < RECHARGE_COND_VOLTAGE)
		chg->bat_info.dis_reason &= ~DISCONNECT_OVER_TIME;

	if (chg->set_batt_full)
		chg->bat_info.dis_reason |= DISCONNECT_BAT_FULL;

	if (chg->bat_info.batt_health != POWER_SUPPLY_HEALTH_GOOD)
		chg->bat_info.dis_reason |= chg->bat_info.batt_health ==
			POWER_SUPPLY_HEALTH_OVERHEAT ?
			DISCONNECT_TEMP_OVERHEAT : DISCONNECT_TEMP_FREEZE;

	ktime = alarm_get_elapsed_realtime();
	cur_time = ktime_to_timespec(ktime);

	if (chg->discharging_time &&
			cur_time.tv_sec > chg->discharging_time) {
		pr_info("%s : cur_time = %d, timeout = %d\n", __func__, cur_time.tv_sec, chg->discharging_time);
		chg->set_charge_timeout = true;
//		chg->bat_info.dis_reason |= DISCONNECT_OVER_TIME;  // for GED (crespo).
		chg->bat_info.dis_reason |= DISCONNECT_BAT_FULL;
		// set battery full (expiration of absolute timer)
		sec_bat_set_status(&chg->callbacks, CHARGING_STATUS_FULL);
	}

	pr_debug("%s : Current Voltage : %d\n			\
		Current time : %ld  discharging_time : %ld\n	\
		discharging reason : %d\n",			\
		__func__, chg->bat_info.batt_vcell, cur_time.tv_sec,
		chg->discharging_time, chg->bat_info.dis_reason);
}

static bool check_samsung_charger(void)
{
	int adc_1, adc_2, vol_1, vol_2;
	int i = 0;

	adc_1 = adc_2 = vol_1 = vol_2 = 0;

	fsa9480_manual_switching(SWITCH_Audio_Port);

	for ( i=0; i<3; i++)
	{
		//1.Read ADC value
		adc_1 = sec_bat_get_adc_data(ADC_AP_CHECK_1);
		adc_2 = sec_bat_get_adc_data(ADC_AP_CHECK_2);

		//2. Change ADC value to Voltage
		vol_1= adc_1* 3300 / 4095;
		vol_2= adc_2* 3300 / 4095;
		pr_info("%s: vol_1 = %d, vol_2 = %d!!\n", __func__, vol_1, vol_2);

		//3. Check range of the voltage
		if( (vol_1 < 800) || (vol_1 > 1470) || (vol_2 < 800) || (vol_2 > 1470) )
		{
			pr_info("%s: Improper charger is connected!!!\n", __func__);
			fsa9480_manual_switching(AUTO_SWITCH);
			return false;
		}	
	}
	
	pr_info("%s: Samsung charger is connected!!!\n", __func__);
	fsa9480_manual_switching(AUTO_SWITCH);
	return true;

}

static int sec_bat_charging_control(struct chg_data *chg)
{
	static enum cable_type_t prev_cable_status = CABLE_TYPE_NONE;
	static bool prev_charging = false;
	int ret;

	if(chg->cable_status != prev_cable_status || chg->charging != prev_charging)
	{
		/* disable max8998 charer */
		if(chg->pdata->pmic_charger && chg->pdata->pmic_charger->charging_control) {
			ret = chg->pdata->pmic_charger->charging_control(0, chg->cable_status);
			if (ret < 0)
				goto err;
		}

		if (!chg->charging) {
			/* disable charging */
			gpio_set_value(GPIO_TA_EN, 1);	// External charger disable
		} else {
			/* enable charging */
			if(chg->pdata->external_charger && chg->pdata->external_charger->charging_control)
			{
				if ((chg->cable_status == CABLE_TYPE_AC) && !check_samsung_charger()) {
					chg->pdata->external_charger->charging_control(1, CABLE_TYPE_IMPROPER_AC);  // Improper charger
					chg->bat_info.batt_improper_ta = true;  // Improper charger
				}
				else {
					chg->pdata->external_charger->charging_control(1, chg->cable_status);  // TA or USB cable
				}
			}

			gpio_set_value(GPIO_TA_EN, 0);	// External charger enable

			// Stop low battery compensation algorithm
			if(chg->pdata && chg->pdata->fuelgauge_cb)
				chg->pdata->fuelgauge_cb(REQ_LOW_BATTERY_COMPENSATION, 0, 0);
		}

		/* update previous status values */
		prev_cable_status = chg->cable_status;
		prev_charging = chg->charging;
	}
	else
	{
		// do nothing... status is same as previous one.
	}

	return 0;
err:
	pr_err("max8998_read_reg error\n");
	return ret;
}

static int sec_cable_status_update(struct chg_data *chg)
{
	int ret;
	bool vdc_status = false;
	ktime_t ktime;
	struct timespec cur_time;

	/* if max8998 has detected vdcin */
	if (chg->pdata && chg->pdata->pmic_charger &&
	    chg->pdata->pmic_charger->get_connection_status &&
	    chg->pdata->pmic_charger->get_connection_status())
	{
		if (!chg->pdata->get_jig_status())
			vdc_status = true;
		if (chg->bat_info.dis_reason) {
			pr_info("%s : battery status discharging : %d\n",
				__func__, chg->bat_info.dis_reason);
			/* have vdcin, but cannot charge */
			chg->charging = false;
			ret = sec_bat_charging_control(chg);
			if (ret < 0)
				goto err;
			chg->bat_info.charging_status =
				chg->bat_info.batt_is_full ?
				POWER_SUPPLY_STATUS_FULL :
				POWER_SUPPLY_STATUS_NOT_CHARGING;
			chg->discharging_time = 0;
			chg->set_batt_full = 0;
			goto update;
		} else if (chg->discharging_time == 0) {
			ktime = alarm_get_elapsed_realtime();
			cur_time = ktime_to_timespec(ktime);
			chg->discharging_time =
				chg->bat_info.batt_is_full ||
				chg->set_charge_timeout ?
				cur_time.tv_sec + TOTAL_RECHARGING_TIME :
				cur_time.tv_sec + TOTAL_CHARGING_TIME;
		}

		/* able to charge */
		chg->charging = true;
		/* if we have vdcin but we cannot detect the cable type,
		force to AC so we can charge anyway */
		if (chg->cable_status == CABLE_TYPE_NONE)
			chg->cable_status = CABLE_TYPE_AC;
		ret = sec_bat_charging_control(chg);
		if (ret < 0)
			goto err;

		chg->bat_info.charging_status = chg->bat_info.batt_is_full ?
			POWER_SUPPLY_STATUS_FULL : POWER_SUPPLY_STATUS_CHARGING;
	} 
	else {
		/* no vdc in, not able to charge */
		vdc_status = false;
		chg->charging = false;
		ret = sec_bat_charging_control(chg);
		if (ret < 0)
			goto err;

		chg->bat_info.charging_status = POWER_SUPPLY_STATUS_DISCHARGING;

		chg->bat_info.batt_is_full = false;
		chg->set_charge_timeout = false;
		chg->set_batt_full = 0;
		chg->bat_info.dis_reason = 0;
		chg->discharging_time = 0;
		chg->bat_info.batt_improper_ta = false;
		chg->is_recharging = false;
	}

update:
	if(chg->bat_info.charging_status == POWER_SUPPLY_STATUS_FULL ||
	    chg->bat_info.charging_status == POWER_SUPPLY_STATUS_CHARGING) {
		/* Update DISCHARGING status in case of USB cable or Improper charger */
		if(chg->cable_status==CABLE_TYPE_USB || chg->bat_info.batt_improper_ta)
			chg->bat_info.charging_status = POWER_SUPPLY_STATUS_DISCHARGING;
	}

	if ((chg->cable_status != CABLE_TYPE_NONE) && vdc_status)
		wake_lock(&chg->vbus_wake_lock);
	else
		wake_lock_timeout(&chg->vbus_wake_lock, HZ / 2);

	return 0;
err:
	return ret;
}

static void sec_program_alarm(struct chg_data *chg, int seconds)
{
	ktime_t low_interval = ktime_set(seconds - 10, 0);
	ktime_t slack = ktime_set(20, 0);
	ktime_t next;

	alarm_cancel(&chg->alarm);
	next = ktime_add(chg->last_poll, low_interval);
	alarm_start_range(&chg->alarm, next, ktime_add(next, slack));
}

static void sec_bat_work(struct work_struct *work)
{
	struct chg_data *chg =
		container_of(work, struct chg_data, bat_work);
	int ret;
	struct timespec ts;
	unsigned long flags;

	mutex_lock(&chg->mutex);

	sec_get_bat_temp(chg);
	sec_bat_discharge_reason(chg);

	ret = sec_cable_status_update(chg);
	if (ret < 0)
		goto err;

	mutex_unlock(&chg->mutex);

	power_supply_changed(&chg->psy_bat);

	chg->last_poll = alarm_get_elapsed_realtime();
	ts = ktime_to_timespec(chg->last_poll);
	chg->timestamp = ts.tv_sec;

	/* prevent suspend before starting the alarm */
	local_irq_save(flags);
	wake_unlock(&chg->work_wake_lock);
	sec_program_alarm(chg, FAST_POLL);
	local_irq_restore(flags);
	return;
err:
	mutex_unlock(&chg->mutex);
	wake_unlock(&chg->work_wake_lock);
	pr_err("battery workqueue fail\n");
}

static void sec_battery_alarm(struct alarm *alarm)
{
	struct chg_data *chg =
			container_of(alarm, struct chg_data, alarm);

	wake_lock(&chg->work_wake_lock);
	queue_work(chg->monitor_wqueue, &chg->bat_work);
}

static ssize_t sec_bat_show_attrs(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct chg_data *chg = container_of(psy, struct chg_data, psy_bat);
	int i = 0;
	const ptrdiff_t off = attr - sec_battery_attrs;
	union power_supply_propval value;
	u8 batt_str[5];

	switch (off) {
	case BATT_VOL:
		if (chg->pdata &&
		    chg->pdata->psy_fuelgauge &&
		    chg->pdata->psy_fuelgauge->get_property) {
			chg->pdata->psy_fuelgauge->get_property(
				chg->pdata->psy_fuelgauge,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &value);
			chg->bat_info.batt_vcell = value.intval;
		}
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", (chg->bat_info.batt_vcell) / 1000);
		break;

	case BATT_TEMP:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", chg->bat_info.batt_temp);
		break;

	case BATT_CHARGING_SOURCE:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", chg->cable_status);
		break;

	case BATT_FG_SOC:
		if (chg->pdata &&
		    chg->pdata->psy_fuelgauge &&
		    chg->pdata->psy_fuelgauge->get_property) {
			chg->pdata->psy_fuelgauge->get_property(
				chg->pdata->psy_fuelgauge,
				POWER_SUPPLY_PROP_CAPACITY, &value);
			chg->bat_info.batt_soc = value.intval;
		}
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", chg->bat_info.batt_soc);
		break;

#ifdef CONFIG_BATTERY_MAX17042
	case BATT_FG_CHECK:
		if(chg->pdata &&
		    chg->pdata->fuelgauge_cb)
			value.intval = chg->pdata->fuelgauge_cb(REQ_TEST_MODE_INTERFACE,
						TEST_MODE_FUEL_GAUGE_CHECK, 0);
		else
			value.intval = 0;
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", value.intval);
		break;

	case BATT_BATT_TYPE:
		if(chg->battery_type== SDI_BATTERY_TYPE)
			sprintf(batt_str, "SDI");
		else if(chg->battery_type== ATL_BATTERY_TYPE)
			sprintf(batt_str, "ATL");
		else
			sprintf(batt_str, "XXX");
		i += scnprintf(buf + i, PAGE_SIZE - i, "%s_%s\n", batt_str, batt_str);
		break;
#endif

#ifdef SAMSUNG_LPM_MODE
	case CHARGING_MODE_BOOTING:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", chg->charging_mode_booting);
		break;

	case BATT_TEMP_CHECK:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", chg->bat_info.batt_health);
		break;

	case BATT_FULL_CHECK:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", chg->bat_info.batt_is_full);
		break;
#endif
	default:
		i = -EINVAL;
	}

	return i;
}

static ssize_t sec_bat_store_attrs(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct chg_data *chg = container_of(psy, struct chg_data, psy_bat);
	int x = 0;
	int ret = 0;
	const ptrdiff_t off = attr - sec_battery_attrs;
	bool vdc_status = false;
	bool jig_status = false;

	switch (off) {
#ifdef CONFIG_BATTERY_MAX17042
	case BATT_RESET_SOC:
		if (sscanf(buf, "%d\n", &x) == 1) {
			if (x == 1) {
				// Get DCIN status from PMIC.
				if(chg->pdata && chg->pdata->pmic_charger &&
				    chg->pdata->pmic_charger->get_connection_status)
					vdc_status = chg->pdata->pmic_charger->get_connection_status();

				// Get Jig status from Switch.
				if(chg->pdata && chg->pdata->get_jig_status)
					jig_status = chg->pdata->get_jig_status();

				if(jig_status && !vdc_status) {  // Using Jig with no DCIN source.
					if(chg->pdata && chg->pdata->fuelgauge_cb)
						chg->pdata->fuelgauge_cb(REQ_TEST_MODE_INTERFACE,
							TEST_MODE_QUICK_START_CMD, 0);
				}
				else
					printk("Quickstart canceled by Jig(%d) or DCIN(%d)\n", jig_status, vdc_status);
			}
			ret = count;
		}
		dev_info(dev, "%s: Reset SOC:%d\n", __func__, x);
		break;

	case BATT_RESET_CAP:
		if (sscanf(buf, "%d\n", &x) == 1 || x==2 || x==3 || x==4) {
			if (x==1 || x== 2 || x==3 || x==4) {
				if(chg->pdata &&
				    chg->pdata->fuelgauge_cb)
					chg->pdata->fuelgauge_cb(REQ_TEST_MODE_INTERFACE,
						TEST_MODE_RESET_CAPACITY, x);
			}
			ret = count;
		}
		dev_info(dev, "%s: Reset CAP:%d\n", __func__, x);
		break;

	case BATT_FG_REG:
		if (sscanf(buf, "%d\n", &x) == 1) {
			if (x == 1) {
				if(chg->pdata &&
				    chg->pdata->fuelgauge_cb)
					chg->pdata->fuelgauge_cb(REQ_TEST_MODE_INTERFACE,
						TEST_MODE_DUMP_FG_REGISTER, 0);
			}
			ret = count;
		}
		dev_info(dev, "%s: FG Register:%d\n", __func__, x);
		break;
#endif

#ifdef SAMSUNG_LPM_MODE
	case CHARGING_MODE_BOOTING:
		if (sscanf(buf, "%d\n", &x) == 1) {
			chg->charging_mode_booting = x;
			ret = count;
		}
		break;
#endif

	default:
		ret = -EINVAL;
	}

	return ret;
}

static int sec_bat_create_attrs(struct device *dev)
{
	int i, rc;

	for (i = 0; i < ARRAY_SIZE(sec_battery_attrs); i++) {
		rc = device_create_file(dev, &sec_battery_attrs[i]);
		if (rc)
			goto sec_attrs_failed;
	}
	goto succeed;

sec_attrs_failed:
	while (i--)
		device_remove_file(dev, &sec_battery_attrs[i]);
succeed:
	return rc;
}

static __devinit int sec_battery_probe(struct platform_device *pdev)
{
	struct sec_battery_platform_data *pdata = pdev->dev.platform_data;
	struct chg_data *chg;
	int ret = 0;

	pr_info("%s : Samsung Battery Driver Loading\n", __func__);

	chg = kzalloc(sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->pdata = pdata;

	if (!chg->pdata || !chg->pdata->adc_table) {
		pr_err("%s : No platform data & adc_table supplied\n", __func__);
		ret = -EINVAL;
		goto err_bat_table;
	}

	chg->psy_bat.name = "battery",
	chg->psy_bat.type = POWER_SUPPLY_TYPE_BATTERY,
	chg->psy_bat.properties = sec_battery_props,
	chg->psy_bat.num_properties = ARRAY_SIZE(sec_battery_props),
	chg->psy_bat.get_property = sec_bat_get_property,

	chg->psy_usb.name = "usb",
	chg->psy_usb.type = POWER_SUPPLY_TYPE_USB,
	chg->psy_usb.supplied_to = supply_list,
	chg->psy_usb.num_supplicants = ARRAY_SIZE(supply_list),
	chg->psy_usb.properties = sec_power_properties,
	chg->psy_usb.num_properties = ARRAY_SIZE(sec_power_properties),
	chg->psy_usb.get_property = sec_usb_get_property,

	chg->psy_ac.name = "ac",
	chg->psy_ac.type = POWER_SUPPLY_TYPE_MAINS,
	chg->psy_ac.supplied_to = supply_list,
	chg->psy_ac.num_supplicants = ARRAY_SIZE(supply_list),
	chg->psy_ac.properties = sec_power_properties,
	chg->psy_ac.num_properties = ARRAY_SIZE(sec_power_properties),
	chg->psy_ac.get_property = sec_ac_get_property,

	chg->present = 1;
	chg->polling_interval = POLLING_INTERVAL;
	chg->bat_info.batt_health = POWER_SUPPLY_HEALTH_GOOD;
	chg->bat_info.batt_is_full = false;
	chg->set_charge_timeout = false;
	chg->bat_info.batt_improper_ta = false;
	chg->is_recharging = false;
#ifdef CONFIG_BATTERY_MAX17042
	// Get battery type from fuelgauge driver.
	if(chg->pdata && chg->pdata->fuelgauge_cb)
		chg->battery_type = (battery_type_t)chg->pdata->fuelgauge_cb(
					REQ_TEST_MODE_INTERFACE, TEST_MODE_BATTERY_TYPE_CHECK, 0);

	// Check UV charging case.
	if(chg->pdata && chg->pdata->pmic_charger &&
	    chg->pdata->pmic_charger->get_connection_status) {
		if(chg->pdata->pmic_charger->get_connection_status() &&
		    check_UV_charging_case(chg))
			chg->low_batt_boot_flag = true;
	}
	else
		chg->low_batt_boot_flag = false;

	// init delayed work
	INIT_DELAYED_WORK(&chg->full_chg_work, full_comp_work_handler);

	 // Init low batt check threshold values.
	if(chg->battery_type == SDI_BATTERY_TYPE)
		chg->check_start_vol = 3550;  // Under 3.55V
	else if(chg->battery_type == ATL_BATTERY_TYPE)
		chg->check_start_vol = 3450;  // Under 3.45V
#endif

	chg->cable_status = CABLE_TYPE_NONE;
	chg->charging_status = CHARGING_STATUS_NONE;

	mutex_init(&chg->mutex);

	platform_set_drvdata(pdev, chg);

	wake_lock_init(&chg->vbus_wake_lock, WAKE_LOCK_SUSPEND,
		"vbus_present");
	wake_lock_init(&chg->work_wake_lock, WAKE_LOCK_SUSPEND,
		"sec_battery_work");

	INIT_WORK(&chg->bat_work, sec_bat_work);

	chg->monitor_wqueue =
		create_freezeable_workqueue(dev_name(&pdev->dev));
	if (!chg->monitor_wqueue) {
		pr_err("Failed to create freezeable workqueue\n");
		ret = -ENOMEM;
		goto err_wake_lock;
	}

	chg->last_poll = alarm_get_elapsed_realtime();
	alarm_init(&chg->alarm, ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
		sec_battery_alarm);

	/* init power supplier framework */
	ret = power_supply_register(&pdev->dev, &chg->psy_bat);
	if (ret) {
		pr_err("Failed to register power supply psy_bat\n");
		goto err_wqueue;
	}

	ret = power_supply_register(&pdev->dev, &chg->psy_usb);
	if (ret) {
		pr_err("Failed to register power supply psy_usb\n");
		goto err_supply_unreg_bat;
	}

	ret = power_supply_register(&pdev->dev, &chg->psy_ac);
	if (ret) {
		pr_err("Failed to register power supply psy_ac\n");
		goto err_supply_unreg_usb;
	}

	sec_bat_create_attrs(chg->psy_bat.dev);

	chg->callbacks.set_cable = sec_bat_set_cable;
	chg->callbacks.set_status = sec_bat_set_status;
	chg->callbacks.force_update = sec_bat_force_update;
	if (chg->pdata->register_callbacks)
		chg->pdata->register_callbacks(&chg->callbacks);

	wake_lock(&chg->work_wake_lock);
	queue_work(chg->monitor_wqueue, &chg->bat_work);

	p1_lpm_mode_check(chg);

	return 0;

err_supply_unreg_ac:
	power_supply_unregister(&chg->psy_ac);
err_supply_unreg_usb:
	power_supply_unregister(&chg->psy_usb);
err_supply_unreg_bat:
	power_supply_unregister(&chg->psy_bat);
err_wqueue:
	destroy_workqueue(chg->monitor_wqueue);
	cancel_work_sync(&chg->bat_work);
	alarm_cancel(&chg->alarm);
err_wake_lock:
	wake_lock_destroy(&chg->work_wake_lock);
	wake_lock_destroy(&chg->vbus_wake_lock);
	mutex_destroy(&chg->mutex);
err_bat_table:
	kfree(chg);
	return ret;
}

static int __devexit sec_battery_remove(struct platform_device *pdev)
{
	struct chg_data *chg = platform_get_drvdata(pdev);

	alarm_cancel(&chg->alarm);
	flush_workqueue(chg->monitor_wqueue);
	destroy_workqueue(chg->monitor_wqueue);
	power_supply_unregister(&chg->psy_bat);
	power_supply_unregister(&chg->psy_usb);
	power_supply_unregister(&chg->psy_ac);
	wake_lock_destroy(&chg->work_wake_lock);
	wake_lock_destroy(&chg->vbus_wake_lock);
	mutex_destroy(&chg->mutex);
	kfree(chg);

	return 0;
}

static int sec_battery_suspend(struct device *dev)
{

	struct chg_data *chg = dev_get_drvdata(dev);
	if (!chg->charging) {
		sec_program_alarm(chg, SLOW_POLL);
		chg->slow_poll = 1;
	}

	return 0;
}

static void sec_battery_resume(struct device *dev)
{
	struct chg_data *chg = dev_get_drvdata(dev);
	/* We might be on a slow sample cycle.  If we're
	 * resuming we should resample the battery state
	 * if it's been over a minute since we last did
	 * so, and move back to sampling every minute until
	 * we suspend again.
	 */
	if (chg->slow_poll) {
		sec_program_alarm(chg, FAST_POLL);
		chg->slow_poll = 0;
	}
}

static const struct dev_pm_ops sec_battery_pm_ops = {
	.prepare        = sec_battery_suspend,
	.complete       = sec_battery_resume,
};

static struct platform_driver sec_battery_driver = {
	.driver = {
		.name = "sec_battery",
		.owner = THIS_MODULE,
		.pm = &sec_battery_pm_ops,
	},
	.probe = sec_battery_probe,
	.remove = __devexit_p(sec_battery_remove),
};

static int __init sec_battery_init(void)
{
	return platform_driver_register(&sec_battery_driver);
}

static void __exit sec_battery_exit(void)
{
	platform_driver_register(&sec_battery_driver);
}

late_initcall(sec_battery_init);
module_exit(sec_battery_exit);

MODULE_AUTHOR("Ikkeun Kim <iks.kim@samsung.com>");
MODULE_DESCRIPTION("Samsung battery driver");
MODULE_LICENSE("GPL");
