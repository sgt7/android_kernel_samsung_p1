
#include "p1_keyboard.h"

static struct dock_keyboard_data *g_data;
static bool remap_state = false;
static unsigned long connected_time=0;

/* to contol the gpio in sleep mode */
bool keyboard_enable = false;
EXPORT_SYMBOL(keyboard_enable);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void keyboard_early_suspend(struct early_suspend *);
static void keyboard_late_resume(struct early_suspend *);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

extern struct class *sec_class;
extern void dock_keyboard_tx(u8 val);
extern int change_console_baud_rate(int baud);

static void keyboard_timer(unsigned long _data)
{
    struct dock_keyboard_data *data = (struct dock_keyboard_data *)_data;
    if(data->kl == UNKOWN_KEYLAYOUT )
    {
        s3c_gpio_cfgpin(ACCESSORY_EN, S3C_GPIO_OUTPUT);
        s3c_gpio_setpull(ACCESSORY_EN, S3C_GPIO_PULL_UP);
        gpio_direction_output(ACCESSORY_EN, 0);
        keyboard_enable = false;
    }
}

static void remapkey_timer(unsigned long data)
{
    unsigned int keycode = 0;
    if((g_data->pressed[0x45])||(g_data->pressed[0x48]))
    {
        remap_state = REMAPKEY_PRESSED;
        keycode = g_data->keycode[data];
        input_report_key(g_data->input_dev,keycode, 1);
        input_sync(g_data->input_dev);
    }
    else
    {
        remap_state = REMAPKEY_RELEASED;

        if(data == 0x48)
        {
            keycode = KEY_NEXTSONG;
        }
        else
        {
            keycode = KEY_PREVIOUSSONG;
        }
        input_report_key(g_data->input_dev, keycode, 1);
        input_report_key(g_data->input_dev, keycode, 0);
        input_sync(g_data->input_dev);
    }
}

static void release_all_keys(void)
{
    int i;
    //printk(KERN_DEBUG "[Keyboard] Release the pressed keys.\n");
    for(i = 0; i < KEYBOARD_MAX; i++)
    {
        if(g_data->pressed[i])
        {
            input_report_key(g_data->input_dev, g_data->keycode[i], 0);
            g_data->pressed[i] = false;
        }
        input_sync(g_data->input_dev);
    }
}

#if defined(KBD_THREAD)
static int p1_keyboard_thread(void *pdata)
{
    struct dock_keyboard_data *data = (struct dock_keyboard_data *)pdata;
#elif defined(KBD_POLL)
static void key_event_timer(unsigned long _data)
{
    struct dock_keyboard_data *data = (struct dock_keyboard_data *)_data;
#elif defined(KBD_WORK)
static void key_event_work(struct work_struct *work)
{
struct dock_keyboard_data *data = container_of(work,
struct dock_keyboard_data, work_msg);
#endif

    bool press;
    unsigned int keycode;
    unsigned char scan_code;

#if defined(KBD_THREAD)
    while (!kthread_should_stop())
    {
        wait_event_timeout(data->wait_queue, data->buf_front != data->buf_rear, data->timeout);
#endif
        mutex_lock(&data->mutex);
        if(data->buf_front != data->buf_rear)
        {
            scan_code = data->key_buf[data->buf_front];
            //data->buf_front = (data->buf_front == MAX_BUF) ? 0 : data->buf_front + 1;
            data->buf_front++;
            if(data->buf_front > MAX_BUF)
                data->buf_front = 0;
            /* keyboard driver need the contry code*/
            if(data->kl == UNKOWN_KEYLAYOUT)
            {
                switch(scan_code)
                {
                    case US_KEYBOARD:
                        data->kl = US_KEYLAYOUT;
                        data->keycode[49] = KEY_BACKSLASH;
                        /* for the wakeup state*/
                        data->pre_kl = data->kl;
                        printk(KERN_DEBUG "[Keyboard] US keyboard is attacted.\n");
                        break;

                    case UK_KEYBOARD:
                        data->kl = UK_KEYLAYOUT;
                        data->keycode[49] = KEY_NUMERIC_POUND;
                        /* for the wakeup state*/
                        data->pre_kl = data->kl;
                        printk(KERN_DEBUG "[Keyboard] UK keyboard is attacted.\n");
                        break;

                    default:
                        printk(KERN_DEBUG "[Keyboard] Unkown key layout : %x\n", scan_code);
                        break;
                }
            }
            else
            {
                /* Caps lock led on/off */
                if(scan_code == 0xca || scan_code == 0xcb || scan_code == 0xeb || scan_code == 0xec)
                {
                    // Ignore
                }
                else
                {
                    press = ((scan_code & 0x80) != 0x80);
                    keycode = (scan_code & 0x7f);

                    if(keycode >= KEYBOARD_MIN || keycode <= KEYBOARD_MAX)
                    {
                        if(press)
                        {
                            data->pressed[keycode] = true;
                        }
                        else
                        {
                            data->pressed[keycode] = false;
                        }

                        /* for the remap keys*/
                        if(keycode == 0x45 || keycode == 0x48)
                        {
                            if(press)
                            {
                                data->key_timer.data = (unsigned long) keycode;
                                mod_timer(&data->key_timer, jiffies + HZ/3);
                            }
                            else
                            {
                                if(remap_state == REMAPKEY_PRESSED)
                                {
                                    remap_state = REMAPKEY_RELEASED;
                                    input_report_key(data->input_dev, data->keycode[keycode], press);
                                    input_sync(data->input_dev);
                                }
                            }
                        }
                        else
                        {
                            input_report_key(data->input_dev, data->keycode[keycode], press);
                            input_sync(data->input_dev);
                        }
                    }
                }
            }
        }
        mutex_unlock(&data->mutex);
        input_sync(data->input_dev);
#if defined(KBD_THREAD)
    }
    return 0;
#elif defined(KBD_POLL)
    mod_timer(&g_data->timer_poll, jiffies + HZ/5);
#endif
}

static void led_on(void)
{

    if(g_data->led_on)
    {
        //caps lock led on
        dock_keyboard_tx(0xca);
    }
    else
    {
        //caps lock led off
        dock_keyboard_tx(0xcb);
    }
}

int check_keyboard_dock(int val)
{
    static bool dockconnected = false;
    static bool pre_connected =false;
    static bool pre_uart_path =false;
    static unsigned long disconnected_time=0;
    int try_cnt = 0;
    int error = 0;
    int max_cnt = 10;

    if(val)
    {
        dockconnected = false;
    }
    else
    {
        pre_connected = true;

        /*for checking handshake*/
        connected_time = jiffies_to_msecs(jiffies);

         /* check the keyboard is connected durring the boot*/
        if((connected_time - disconnected_time) < 1000)
        {
            g_data->kl = g_data->pre_kl;
            printk(KERN_DEBUG "[Keyboard] kl : %d\n", g_data->pre_kl);
        }
        else
        {
            g_data->pre_kl = UNKOWN_KEYLAYOUT;
        }

        if(!keyboard_enable)
        {
            if(gpio_is_valid(ACCESSORY_EN))
            {
                gpio_request(ACCESSORY_EN,"ACCESSORY_EN");
                s3c_gpio_cfgpin(ACCESSORY_EN, S3C_GPIO_OUTPUT);
                s3c_gpio_setpull(ACCESSORY_EN, S3C_GPIO_PULL_UP);
                gpio_direction_output(ACCESSORY_EN, 1);
                keyboard_enable = true;
            }
        }

        /* if the uart is set as cp path, the path should be switched to ap path.*/
        pre_uart_path = gpio_get_value(GPIO_UART_SEL);
        if(!pre_uart_path)
        {
            gpio_direction_output(GPIO_UART_SEL, 1);
            printk(KERN_DEBUG "[Keyboard] console uart path is switched to AP.\n");
        }

        /* Set baud rate for the keyboard uart */
        error = change_console_baud_rate(9600);
        if(error<0)
        {
            printk(KERN_ERR "[Keyboard] Couldn't modify the baud rate.\n");
        }
        else
        {
//            wake_up(&data->wait_queue);
//            wake_up_process(g_data->task);
#if defined(KBD_THREAD)
            g_data->task = kthread_run(p1_keyboard_thread, g_data, "p1_keyboard_thread");
            if(g_data->task == NULL)
            {
                printk(KERN_ERR "[Keyboard]unabled to create touch thread\n");
                return 0;
            }
#endif
            wake_lock_timeout(&g_data->wake_lock, HZ);
        }

#if defined(KBD_POLL)
        mod_timer(&g_data->timer_poll, jiffies + HZ/10);
#endif

        if(!dockconnected)
        {
            /* try to get handshake data */
            for(try_cnt=0; try_cnt<max_cnt; try_cnt++)
            {
                msleep(100);
                if(g_data->kl != UNKOWN_KEYLAYOUT)
                {
                    dockconnected = true;
                    break;
                }

                /* the accessory is dettached. */
                if(gpio_get_value(g_data->gpio))
                {
                    dockconnected = false;
                    break;
                }
            }
        }
    }

    if(dockconnected)
    {
        return 1;
    }
    else
    {
        if(pre_connected)
        {
            error = change_console_baud_rate(115200);
            if(error<0)
            {
                printk(KERN_ERR "[Keyboard] Couldn't modify the baud rate.\n");
            }

            /* stop the thread and clear the buffer*/
//            g_data->timeout = MAX_SCHEDULE_TIMEOUT;
            g_data->buf_front = g_data->buf_rear = 0;
            if(g_data->task != NULL)
            {
                kthread_stop(g_data->task);
            }

            dockconnected = false;
            gpio_direction_output(GPIO_UART_SEL, pre_uart_path);
            mod_timer(&g_data->timer, jiffies + HZ);

            g_data->kl = UNKOWN_KEYLAYOUT;
            pre_connected = false;
            disconnected_time = jiffies_to_msecs(jiffies);
            release_all_keys();
        }
        return 0;
    }
}

EXPORT_SYMBOL(check_keyboard_dock);

#if defined(ACC_INT_KBD)
static void kbd_int_work(struct work_struct *work)
{
    check_keyboard_dock();
}

static irqreturn_t accessory_interrupt(int irq, void *dev_id)
{
    struct dock_keyboard_data *data = dev_id;
//    printk("[Keyboard] %s \n", __func__);

    if (!work_pending(&data->work_int))
    {
        schedule_work(&data->work_int);
    }
    return IRQ_HANDLED;
}
#endif

void send_keyevent(unsigned int key_code)
{
    g_data->key_buf[g_data->buf_rear]  = key_code;
    //g_data->buf_rear = (g_data->buf_rear == MAX_BUF) ? 0 : g_data->buf_rear;
    g_data->buf_rear++;
    if(g_data->buf_rear > MAX_BUF)
        g_data->buf_rear = 0;
#if defined(KBD_WORK)
    if (!work_pending(&g_data->work_msg))
    {
        schedule_work(&g_data->work_msg);
    }
#endif
}

EXPORT_SYMBOL(send_keyevent);

static ssize_t caps_lock_led(struct device *dev, struct device_attribute *attr, char *buf, size_t size)
{
//    struct dock_keyboard_data *data = dev->platform_data;
    int i=0;
    //printk(KERN_DEBUG "[Keyboard] Caps lock led : %d.\n", g_data->led_on);
    if(sscanf(buf,"%d",&i)==1)
    {
        if(i == 1)
        {
            g_data->led_on = true;
        }
        else
        {
            g_data->led_on = false;
        }
    }
    else
    {
        printk(KERN_ERR "[Keyboard] Couldn't get led state.\n");
    }

    led_on();

    return size;
}
static DEVICE_ATTR(keyboard_led, S_IRUGO | S_IWUSR | S_IWGRP, NULL, caps_lock_led);

static int __devinit dock_keyboard_probe(struct platform_device *pdev)
{
//    struct dock_keyboard_data *data = pdev->dev.platform_data;
    struct dock_keyboard_data *data;
    struct input_dev *input;
    int i, error;
#if defined(ACC_INT_KBD)
    int gpio, irq;
#endif
    struct device *keyboard_dev;

    data = kzalloc(sizeof(struct dock_keyboard_data), GFP_KERNEL);
    if(NULL == data)
    {
        error = -ENOMEM;
        goto err_free_mem;
    }

#if defined(KBD_THREAD)
    data->timeout = 50;
    mutex_init(&data->mutex);
    init_waitqueue_head(&data->wait_queue);
#elif defined(KBD_POLL)
    init_timer(&data->timer_poll);
    data->timer_poll.function = key_event_timer;
    data->timer_poll.data = (unsigned long)data;
#elif defined(KBD_WORK)
    INIT_WORK(&data->work_msg, key_event_work);
#endif

#if defined(ACC_INT_KBD)
    INIT_WORK(&data->work_int, kbd_int_work);
#endif

    wake_lock_init(&data->wake_lock, WAKE_LOCK_SUSPEND, "kbd_wake_lock");

    input = input_allocate_device();
    if (!input)
    {
        printk(KERN_ERR "[Keyboard] Fail to allocate input device.\n");
        error = -ENOMEM;
        goto err_free_mem;
    }

    data->input_dev = input;
    data->kl = UNKOWN_KEYLAYOUT;
    memcpy(data->keycode, dock_keycodes, sizeof(dock_keycodes));

    input->name = pdev->name;
    input->dev.parent = &pdev->dev;
    input->id.bustype = BUS_RS232;

    set_bit(EV_SYN, input->evbit);
//    set_bit(EV_REP, input->evbit);
    set_bit(EV_KEY, input->evbit);

    for(i = 0; i < KEYBOARD_SIZE; i++)
    {
        if( KEY_RESERVED != data->keycode[i])
        {
            input_set_capability(input, EV_KEY, data->keycode[i]);
        }
    }

    /* for the UK keyboard */
    input_set_capability(input, EV_KEY, KEY_NUMERIC_POUND);

    /* for the remaped keys */
    input_set_capability(input, EV_KEY, KEY_NEXTSONG);
    input_set_capability(input, EV_KEY, KEY_PREVIOUSSONG);

    error = input_register_device(data->input_dev);
    if(error<0)
    {
        printk(KERN_ERR "[Keyboard] Fail to register input device.\n");
        error = -ENOMEM;
        goto err_input_allocate_device;
    }

    data->buf_front = data->buf_rear = 0;

    data->gpio = GPIO_ACCESSORY_INT;

    /* Accessory detect pin is used by dock accessory driver. */
#if defined(ACC_INT_KBD)
    gpio = data->gpio;
    s3c_gpio_cfgpin(gpio, S3C_GPIO_INPUT);
    s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
    irq = IRQ_EINT5;

    error = request_irq(irq, accessory_interrupt,
                    IRQF_SAMPLE_RANDOM|IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING,
			"p1_keyboard", data);
    if(error)
    {
        printk(KERN_ERR "[Keyboard] Fail to request irq : %d\n", error);
        error = -EINTR;
        goto err_free_mem;
    }
    data->gpio = gpio;
#endif
    g_data = data;

    keyboard_dev = device_create(sec_class, NULL, 0, NULL, "keyboard");
    if (IS_ERR(keyboard_dev))
        pr_err("Failed to create device(ts)!\n");

    if (device_create_file(keyboard_dev, &dev_attr_keyboard_led) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_keyboard_led.attr.name);

#ifdef CONFIG_HAS_EARLYSUSPEND
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->early_suspend.suspend = keyboard_early_suspend;
	data->early_suspend.resume = keyboard_late_resume;
	register_early_suspend(&data->early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

    init_timer(&data->timer);
    data->timer.expires = jiffies + HZ * 5;
    data->timer.function = keyboard_timer;	/* timer handler */
    data->timer.data = (unsigned long)data;

    init_timer(&data->key_timer);
    data->key_timer.expires = jiffies + HZ/2;
    data->key_timer.function = remapkey_timer;

    return 0;
err_input_allocate_device:
    input_free_device(input);
err_free_mem:
    kfree(data);
    return error;

}

static int __devexit dock_keyboard_remove(struct platform_device *pdev)
{
//    struct dock_keyboard_data *pdata = pdev->dev.platform_data;
    input_unregister_device(g_data->input_dev);
    kthread_stop(g_data->task);
    return 0;
}

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void keyboard_early_suspend(struct early_suspend *early_sus)
{
    if(keyboard_enable)
    {
        dock_keyboard_tx(0x10);
    }
}

static void keyboard_late_resume(struct early_suspend *early_sus)
{
//    struct dock_keyboard_data *data = container_of(early_sus,
//        struct dock_keyboard_data, early_suspend);

    fiq_glue_resume();
    if(keyboard_enable)
    {
        led_on();
        if(change_console_baud_rate(9600))
        {
            printk(KERN_ERR "[Keyboard] Couldn't modify the baud rate.\n");
        }
    }
}
#endif	// End of CONFIG_HAS_EARLYSUSPEND


#if defined(CONFIG_PM)
static int dock_keyboard_suspend(struct platform_device *pdev, pm_message_t state)
{
//    struct dock_keyboard_data *data = pdev->dev.platform_data;
    return 0;
}

static int dock_keyboard_resume(struct platform_device *pdev)
{
//    struct dock_keyboard_data *data = pdev->dev.platform_data;
    return 0;
}
#endif

static struct platform_driver dock_keyboard_device_driver =
{
    .probe		= dock_keyboard_probe,
    .remove	= __devexit_p(dock_keyboard_remove),
#if defined(CONFIG_PM)
    .suspend	= dock_keyboard_suspend,
    .resume	= dock_keyboard_resume,
#endif
    .driver		=
    {
    	.name	= "p1_keyboard",
    	.owner	= THIS_MODULE,
    }
};

static int __init dock_keyboard_init(void)
{
    return platform_driver_register(&dock_keyboard_device_driver);
}

static void __exit dock_keyboard_exit(void)
{
    platform_driver_unregister(&dock_keyboard_device_driver);
}

late_initcall(dock_keyboard_init);
module_exit(dock_keyboard_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SEC P series Dock Keyboard driver");
