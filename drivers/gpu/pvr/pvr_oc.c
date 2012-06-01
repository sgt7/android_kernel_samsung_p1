/*
* Author: Jason Toney <jt1134@gmail.com>
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
*/

#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/sysfs.h>
#include <linux/pvr_oc.h>

int pvr_oc;
unsigned int pvr_clk_val;

static ssize_t pvr_oc_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", pvr_oc);
}

static ssize_t pvr_oc_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%du", &pvr_oc);

	switch(pvr_oc) {
	    case 1:
            pvr_clk_val = 260000000;
		    break;

	    case 2:
            pvr_clk_val = 320000000;
		    break;

	    case 3:
            pvr_clk_val = 370000000;
		    break;

	    default:
            pvr_clk_val = 200000000;
		    break;
	}

	printk("## [GPU OC] : Clock Speed == %d MHz ##\n", pvr_clk_val / 1000000);

	return count;
}
static struct kobj_attribute pvr_oc_attribute = __ATTR(pvr_oc, 0666, pvr_oc_show, pvr_oc_store);

static struct attribute *attrs[] = {
	&pvr_oc_attribute.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

static struct kobject *pvr_oc_kobj;

int pvr_oc_init(void)
{
	int retval;

	pvr_oc = 0;

        pvr_oc_kobj = kobject_create_and_add("pvr_oc", kernel_kobj);

        if (!pvr_oc_kobj) {
                return -ENOMEM;
        }
        retval = sysfs_create_group(pvr_oc_kobj, &attr_group);

        if (retval)
                kobject_put(pvr_oc_kobj);

        return retval;
}

void pvr_oc_exit(void)
{
	kobject_put(pvr_oc_kobj);
}

module_init(pvr_oc_init);
module_exit(pvr_oc_exit);

MODULE_AUTHOR("Toney, Jason <jt1134@gmail.com>");
MODULE_DESCRIPTION("sysfs interface to control GPU (pvr) clock speed");
MODULE_LICENSE("GPL");
