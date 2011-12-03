#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/device.h>

#include <asm/uaccess.h>
#include <asm/io.h>

#define HOTSPOT_EVENT_MONITORING_DEV		"hotspot_event_monitoring"
#define HOTSPOT_EVENT_MONITORING_MAJOR		123
#define HOTSPOT_MAX_QUEUE_CNT			128

enum
{
	HOTSPOT_SLEEP,
	HOTSPOT_WAKE
}hotspot_status;

int hotspot_event_detect_complete(char *msg);
static struct class *hotspot_dev_class;
unsigned int hotspot_detect_condition = 0;
char hotspot_str_buf[HOTSPOT_MAX_QUEUE_CNT];
int hotspot_event_flag;
int hotspot_polling_flag = 0;


DECLARE_WAIT_QUEUE_HEAD(Hotspot_Waitqueue);



void hotspot_event_monitoring_exit(void)
{
	unregister_chrdev(HOTSPOT_EVENT_MONITORING_MAJOR, HOTSPOT_EVENT_MONITORING_DEV);

	device_destroy( hotspot_dev_class, MKDEV(HOTSPOT_EVENT_MONITORING_MAJOR, 0) );
	class_destroy( hotspot_dev_class );	
}

int hotspot_event_detect_complete(char *msg)
{
	unsigned long flags;

	if(!hotspot_polling_flag)
		return 0;

	local_save_flags(flags);
	local_irq_disable();
	memset(hotspot_str_buf, 0, sizeof(hotspot_str_buf));
	
	strcpy(hotspot_str_buf, msg);
	hotspot_event_flag = 1;

	hotspot_detect_condition = HOTSPOT_WAKE;
	
	local_irq_restore(flags);
	wake_up_interruptible(&Hotspot_Waitqueue);
	return 0;
}


int hotspot_event_monitoring_open(struct inode *inode, struct file *flip)
{
	int num = MINOR(inode->i_rdev);

	return 0;
}

int hotspot_event_monitoring_release(struct inode *inode, struct file *flip)
{
	printk("hotspot_event_monitoring_release");
	return 0;
}

ssize_t hotspot_event_monitoring_read(struct file *flip, char *buf, size_t count, loff_t *f_pos)
{
	unsigned long flags;
	int realmax;
	int loop;
	int retstate;

	if((!hotspot_detect_condition) && (flip->f_flags & O_NONBLOCK))
		return -EAGAIN;

	retstate = wait_event_interruptible(Hotspot_Waitqueue, hotspot_detect_condition);

	if(retstate)
		return retstate;

	local_save_flags(flags);
	local_irq_disable();

	hotspot_detect_condition = HOTSPOT_SLEEP;

	copy_to_user(buf, (char *)hotspot_str_buf, sizeof(hotspot_str_buf));
	
	local_irq_restore(flags);
}

ssize_t hotspot_event_monitoring_write(struct file *flip, char *buf, size_t count, loff_t *f_pos)
{
	printk("hotspot_event_monitoring_write");
}
	
unsigned int hotspot_event_monitoring_poll(struct file *flip, poll_table *wait)
{
	unsigned int mask = 0;
	hotspot_polling_flag = 1;
	poll_wait(flip, &Hotspot_Waitqueue, wait);

	if(hotspot_event_flag)
	{
		mask |= POLLIN | POLLRDNORM;
		hotspot_event_flag = 0;
	}

	return mask;
}

struct file_operations hotspot_event_monitoring_fops =
{
	.owner    =     THIS_MODULE,
	.read	  =		hotspot_event_monitoring_read,
	.write    =		hotspot_event_monitoring_write,
	.open     =		hotspot_event_monitoring_open,
	.poll     =		hotspot_event_monitoring_poll,
	.release  =		hotspot_event_monitoring_release,
};

int hotspot_event_monitoring_init(void)
{
	int result;

	result = register_chrdev(HOTSPOT_EVENT_MONITORING_MAJOR, HOTSPOT_EVENT_MONITORING_DEV,
								&hotspot_event_monitoring_fops);

	if(result < 0)
		return result;

	hotspot_dev_class = class_create(THIS_MODULE, HOTSPOT_EVENT_MONITORING_DEV);
	if (IS_ERR(hotspot_dev_class))
		return;

	device_create(hotspot_dev_class, NULL, MKDEV(HOTSPOT_EVENT_MONITORING_MAJOR, 0), NULL, HOTSPOT_EVENT_MONITORING_DEV);
	

	return 0;
}

EXPORT_SYMBOL(hotspot_event_detect_complete);

module_init(hotspot_event_monitoring_init);
module_exit(hotspot_event_monitoring_exit);

MODULE_LICENSE("GPL");

