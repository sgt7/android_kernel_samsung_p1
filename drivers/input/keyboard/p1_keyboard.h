
#ifndef _P1_KEYBOARD_H_
#define _P1_KEYBOARD_H_

#include <linux/input.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/earlysuspend.h>
#include <linux/vmalloc.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <asm/gpio.h>
#include <plat/gpio-cfg.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <asm/fiq_glue.h>
#include <linux/kthread.h>
#include <linux/wakelock.h>

#define KEYBOARD_SIZE   128
#define US_KEYBOARD     0xeb
#define UK_KEYBOARD     0xec

#define KEYBOARD_MIN   0x4
#define KEYBOARD_MAX   0x7f

#define MAX_BUF     255

//for the remap key
#define REMAPKEY_RELEASED    0x0
#define REMAPKEY_PRESSED      0x1

//#define ACC_INT_KBD

//#define KBD_POLL
#define KBD_THREAD
//#define KBD_WORK

#define ACCESSORY_EN                    S5PV210_GPJ1(4)
#if !defined(GPIO_UART_SEL)
#define GPIO_UART_SEL			  S5PV210_MP05(7)
#endif
#if !defined(GPIO_ACCESSORY_INT)
#define GPIO_ACCESSORY_INT	  S5PV210_GPH0(5)
#endif

enum KEY_LAYOUT
{
    UNKOWN_KEYLAYOUT = 0,
    US_KEYLAYOUT,
    UK_KEYLAYOUT,
};

  /* Each client has this additional data */
struct dock_keyboard_data
{
    struct input_dev *input_dev;
#if defined(KBD_THREAD)
    struct task_struct *task;
    struct mutex mutex;
    spinlock_t spin_lock;
    wait_queue_head_t wait_queue;
    signed int timeout;
#elif defined(KBD_POLL)
    struct timer_list timer_poll;
#elif defined(KBD_WORK)
    struct work_struct work_msg;
#endif
#if defined(ACC_INT_KBD)
    struct work_struct work_int;
#endif
    struct early_suspend	early_suspend;
    struct timer_list timer;
    struct timer_list key_timer;
    struct wake_lock wake_lock;
    unsigned short keycode[KEYBOARD_SIZE];
    bool pressed[KEYBOARD_SIZE];
    bool led_on;
    int gpio;
    int buf_front;
    int buf_rear;
    unsigned int kl;
    unsigned int pre_kl;
    unsigned char key_buf[MAX_BUF+1];
};

static const unsigned short dock_keycodes[KEYBOARD_SIZE] =
{
     // keycode              ,         decimal     hex
    KEY_RESERVED,       		//	0		0
    KEY_RESERVED,       		//	1		1
    KEY_RESERVED,       		//	2		2
    KEY_RESERVED,       		//	3		3
    KEY_A,              		//	4		4
    KEY_B,              		//	5		5
    KEY_C,              		//	6		6
    KEY_D,              		//	7		7
    KEY_E,              		//	8		8
    KEY_F,              		//	9		9
    KEY_G,              		//	10		0A
    KEY_H,              		//	11		0B
    KEY_I,              		//	12		0C
    KEY_J,              		//	13		0D
    KEY_K,              		//	14		0E
    KEY_L,              		//	15		0F
    KEY_M,              		//	16		10
    KEY_N,              		//	17		11
    KEY_O,              		//	18		12
    KEY_P,              		//	19		13
    KEY_Q,              		//	20		14
    KEY_R,              		//	21		15
    KEY_S,              		//	22		16
    KEY_T,              		//	23		17
    KEY_U,              		//	24		18
    KEY_V,              		//	25		19
    KEY_W,              		//	26		1A
    KEY_X,              		//	27		1B
    KEY_Y,              		//	28		1C
    KEY_Z,              		//	29		1D
    KEY_1,              		//	30		1E
    KEY_2,              		//	31		1F
    KEY_3,              		//	32		20
    KEY_4,              		//	33		21
    KEY_5,              		//	34		22
    KEY_6,              		//	35		23
    KEY_7,              		//	36		24
    KEY_8,              		//	37		25
    KEY_9,              		//	38		26
    KEY_0,              		//	39		27
    KEY_ENTER,          		//	40		28
    KEY_SCREENLOCK,            	//	41		29
    KEY_BACKSPACE,      		//	42		2A
    KEY_TAB,            		//	43		2B
    KEY_SPACE,          		//	44		2C
    KEY_MINUS,          		//	45		2D
    KEY_EQUAL,          		//	46		2E
    KEY_LEFTBRACE,      		//	47		2F
    KEY_RIGHTBRACE,     		//	48		30
    KEY_BACKSLASH,      		//	49		31
    KEY_RESERVED,       		//	50		32
    KEY_SEMICOLON,      		//	51		33
    KEY_APOSTROPHE,     		//	52		34
    KEY_GRAVE,          		//	53		35
    KEY_COMMA,          		//	54		36
    KEY_DOT,            		//	55		37
    KEY_SLASH,          		//	56		38
    KEY_CAPSLOCK,       		//	57		39
    KEY_TIME,           		//	58		3A
    KEY_BRIGHTNESSDOWN, 		//	59		3B
    KEY_BRIGHTNESSUP,   		//	60		3C
    KEY_WWW,            		//	61		3D
    KEY_MENU,           		//	62		3E
    KEY_HOME,           		//	63		3F
    KEY_BACK,           		//	64		40
    KEY_SEARCH,         		//	65		41
    KEY_MP3,            		//	66		42
    KEY_VIDEO,          		//	67		43
    KEY_PLAY,           		//	68		44
    KEY_REWIND,         		//	69		45
    KEY_MUTE,           		//	70		46
    KEY_RESERVED,       		//	71		47
    KEY_FASTFORWARD,    		//	72		48
    KEY_VOLUMEDOWN,     		//	73		49
    KEY_RESERVED,       		//	4A
    KEY_RESERVED,       		//	75		4B
    KEY_VOLUMEUP,       		//	76		4C
    KEY_RESERVED,       		//	77		4D
    KEY_RESERVED,       		//	78		4E
    KEY_RIGHT,          		//	79		4F
    KEY_LEFT,           		//	80		50
    KEY_DOWN,  	 	        //81		51
    KEY_UP,       	 	        //82		52
    KEY_NUMLOCK,        		//	83		53
    KEY_KPSLASH,        		//	84		54
    KEY_APOSTROPHE,     		//	85		55
    KEY_KPMINUS,        		//	86		56
    KEY_KPPLUS,         		//	87		57
    KEY_KPENTER,        		//	88		58
    KEY_KP1,            		//	89		59
    KEY_KP2,            		//	90		5A
    KEY_KP3,            		//	91		5B
    KEY_KP4,            		//	92		5C
    KEY_KP5,            		//	93		5D
    KEY_KP6,            		//	94		5E
    KEY_KP7,            		//	95		5F
    KEY_KP8,            		//	96		60
    KEY_KP9,            		//	97		61
    KEY_KPDOT,          		//	98		62
    KEY_RESERVED,       		//	99		63
    KEY_BACKSLASH,      		//	100		64      //For the UK keyboard
    KEY_F18,           		//	101		65
    KEY_RESERVED,       		//	102		66
    KEY_RESERVED,       		//	103		67
    KEY_RESERVED,       		//	104		68
    KEY_RESERVED,       		//	105		69
    KEY_RESERVED,       		//	106		6A
    KEY_RESERVED,       		//	107		6B
    KEY_RESERVED,       		//	108		6C
    KEY_RESERVED,       		//	109		6D
    KEY_RESERVED,       		//	110		6E
    KEY_RESERVED,       		//	111		6F
    KEY_HANGEUL,        		//	112		70
    KEY_HANJA,          		//	113		71
    KEY_F13,            		//	114		72
    KEY_LEFTSHIFT,      		//	115		73
    KEY_F16,            		//	116		74
    KEY_F17,            		//	75		Left GUI (Windows Key)
    KEY_F19,            		//	118		76
    KEY_RIGHTSHIFT,     		//	119		77
    KEY_F15,            		//	120		78
    KEY_RESERVED,       		//	121		79		Right GUI (Windows Key)
    KEY_RESERVED,       		//	122		7A
    KEY_RESERVED,       		//	123		7B
    KEY_RESERVED,       		//	124		7C
    KEY_RESERVED,       		//	125		7D
    KEY_RESERVED,       		//	126		7E
    KEY_F14,            		//	127		7F
};

#endif  //_P1_KEYBOARD_H_