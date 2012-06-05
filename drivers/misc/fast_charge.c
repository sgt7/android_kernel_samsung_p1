/*
 * Author: Humberto Borba <humberos@gmail.com>
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
 * The original idea was created by Chad Froebel <chadfroebel@gmail.com>
 *
 * This idea is a simple sysfs interface to enable adapters that
 * are detected as USB to charge as AC.
 *
 */

#include <linux/init.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/fast_charge.h>

int enable_fast_charge;

static ssize_t fast_charge_read( struct device * dev, 
    struct device_attribute * attr, char * buf )
{
    return sprintf( buf, "%d\n", enable_fast_charge );
}

static ssize_t fast_charge_write( struct device * dev, 
    struct device_attribute * attr, const char * buf, size_t count )
{
    unsigned int value;

    if ( sscanf( buf, "%du", &value ) == 1 ) {
    
        if ( value >= 0 && value <= 1 ) {

            enable_fast_charge = value;
            pr_info( "[Fast Charge] - value: %u\n", enable_fast_charge );

        } else {

            pr_info( "%s: invalid input range (0,1): %u\n", __FUNCTION__, value );

        }
    } else {
        pr_info( "%s: invalid input: \n", __FUNCTION__ );
    }

    return count;
}

static DEVICE_ATTR( enable_fast_charge, S_IRUGO | S_IWUGO, 
    fast_charge_read, fast_charge_write );

static struct attribute *fast_charge_attributes[] = {
	&dev_attr_enable_fast_charge.attr,
    NULL,
};

static struct attribute_group fast_charger_group = {
    .attrs = fast_charge_attributes,
};

static struct miscdevice fast_charger_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "fast_charger",
};

int enable_fast_charge_init( void )
{
    int ret;

    pr_info( "%s misc_register(%s)\n", __FUNCTION__, fast_charger_device.name );

    ret = misc_register( &fast_charger_device );

    if (ret) {

	    pr_err( "%s misc_register(%s) fail\n", __FUNCTION__, 
            fast_charger_device.name );

	    return 1;
	}

    if ( sysfs_create_group( &fast_charger_device.this_device->kobj, 
        &fast_charger_group ) < 0 ) {

	    pr_err( "%s sysfs_create_group fail\n", __FUNCTION__ );
	    pr_err( "Failed to create sysfs group for device (%s)!\n", 
            fast_charger_device.name );
	}

    return 0;

}

device_initcall( enable_fast_charge_init );
