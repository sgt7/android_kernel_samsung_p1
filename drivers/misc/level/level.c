#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/kernel_sec_common.h>

#include <asm/uaccess.h>

#define LEVEL_DEV_NAME	"level"

#define LEVEL_DEV_IOCTL_CMD   0xee

#define LEVEL_DEV_UNSET_UPLOAD    _IO(LEVEL_DEV_IOCTL_CMD, 0x1)
#define LEVEL_DEV_SET_AUTOTEST    _IO(LEVEL_DEV_IOCTL_CMD, 0x2)
#define LEVEL_DEV_SET_DEBUGLEVEL    _IO(LEVEL_DEV_IOCTL_CMD, 0x3)
#define LEVEL_DEV_GET_DEBUGLEVEL    _IO(LEVEL_DEV_IOCTL_CMD, 0x4)

static void set_debug_level(void);
static unsigned int get_debug_level(void);

static ssize_t show_control(struct device *d,
		struct device_attribute *attr, char *buf);
static ssize_t store_control(struct device *d,
		struct device_attribute *attr, const char *buf, size_t count);
		
static DEVICE_ATTR(control, 0664, show_control, store_control);

static struct attribute *levelctl_attributes[] = {
	&dev_attr_control.attr,
	NULL
};

static const struct attribute_group levelctl_group = {
	.attrs = levelctl_attributes,
};

static ssize_t show_control(struct device *d,
		struct device_attribute *attr, char *buf)
{
	char *p = buf;
	unsigned int val;

	val = get_debug_level();
	
	p += sprintf(p, "0x%4x\n",val);

	return p - buf;
}

static ssize_t store_control(struct device *d,
		struct device_attribute *attr, const char *buf, size_t count)
{

	if(!strncmp(buf, "clear", 5)) {
		// clear upload magic number
		kernel_sec_clear_upload_magic_number();
		return count;
	}

	if(!strncmp(buf, "autotest", 8)) {
		// set auto test
		kernel_sec_set_autotest();
		return count;
	}

	if(!strncmp(buf, "set", 3)) {
		// set debug level
		set_debug_level();
		return count;
	}

return count;
}

static int level_open(struct inode *inode, struct file *filp)
{
	printk("level Device open\n");

	return 0;
}

static int level_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	unsigned int val;

	switch (cmd) {
		case LEVEL_DEV_UNSET_UPLOAD:
			kernel_sec_clear_upload_magic_number();
			return 0;

		case LEVEL_DEV_SET_AUTOTEST:
			kernel_sec_set_autotest();
			return 0;

		case LEVEL_DEV_SET_DEBUGLEVEL:
			set_debug_level();
			return 0;
			
		case LEVEL_DEV_GET_DEBUGLEVEL:
		{
			val = get_debug_level();
			return copy_to_user((unsigned int *)arg, &val, sizeof(val));
		}
		default:
			printk("Unknown Cmd: %x\n", cmd);
			break;
		}
	return -ENOTSUPP;
}

static void set_debug_level()
{
	switch(kernel_sec_get_debug_level_from_param())
	{
		case KERNEL_SEC_DEBUG_LEVEL_LOW:
			kernel_sec_set_debug_level(KERNEL_SEC_DEBUG_LEVEL_MID);
			break;
		case KERNEL_SEC_DEBUG_LEVEL_MID:
			kernel_sec_set_debug_level(KERNEL_SEC_DEBUG_LEVEL_HIGH);
			break;
		case KERNEL_SEC_DEBUG_LEVEL_HIGH:
			kernel_sec_set_debug_level(KERNEL_SEC_DEBUG_LEVEL_LOW);
			break;
		default:
			break;
	}
}

static unsigned int get_debug_level()
{
	unsigned int val = 0;
	
	switch(kernel_sec_get_debug_level_from_param())
	{
		case KERNEL_SEC_DEBUG_LEVEL_LOW:
			val = 0xA0A0; 
			break;
		case KERNEL_SEC_DEBUG_LEVEL_MID:
			val = 0xB0B0;
			break;
		case KERNEL_SEC_DEBUG_LEVEL_HIGH:
			val = 0xC0C0;
			break;
		default:
			val = 0xFFFF;
			break;
	}

	return val;
}

static struct file_operations level_fops = 
{
	.owner = THIS_MODULE,
	.open = level_open,
	.ioctl = level_ioctl,
};

static struct miscdevice level_device = {
	.minor  = MISC_DYNAMIC_MINOR,
	.name   = LEVEL_DEV_NAME,
	.fops   = &level_fops,
};

/* init & cleanup. */
static int __init level_init(void)
{
	int result;

	printk("level device init\n");

	result = misc_register(&level_device);
	if (result <0) 
		return result;
	
  result = sysfs_create_group(&level_device.this_device->kobj, &levelctl_group);
	if (result < 0) {
		printk("failed to create sysfs files\n");
	}

	return 0;
}

static void __exit level_exit(void)
{
	printk("level device exit\n");
	misc_deregister(&level_device);
}

module_init(level_init);
module_exit(level_exit);

