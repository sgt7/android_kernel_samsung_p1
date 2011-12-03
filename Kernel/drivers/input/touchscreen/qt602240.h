
#ifndef _QT602240_H_
#define _QT602240_H_

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <asm/uaccess.h>
#include <asm/gpio.h>
#include <plat/gpio-cfg.h>
#include <linux/leds.h>
#include <linux/vmalloc.h>
#if defined(CONFIG_DVFS_LIMIT)
#include <mach/cpu-freq-v210.h>
#endif
#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

//#define MDNIE_TUNING
//#define CAMERA_FLASH_CONTROL
#define KEY_LED_CONTROL
#define KEY_LED_SELF
#define ENABLE_NOISE_TEST_MODE
#if defined(CONFIG_TARGET_LOCALE_KOR) || defined (CONFIG_TARGET_LOCALE_USAGSM)
#define DRIVER_FILTER
#endif

#if defined (KEY_LED_CONTROL)
// Key LED controller
#define KEYLED_EN               S5PV210_GPD0(0)

#define KEYLED_ADDRESS_CURRENT      17
#define KEYLED_ADDRESS_MAX              20
#define KEYLED_ADDRESS_LOW              21
#define KEYLED_ADDRESS_ONOFF          22

#define KEYLED_DATA_20MAX                1
#define KEYLED_DATA_30MAX                2
#define KEYLED_DATA_15MAX                3
#define KEYLED_DATA_LOW                   4
#define LED_SWITCH

#endif

#if defined(CONFIG_MACH_P1_LTN)
#define IRQ_TOUCH_INT       IRQ_EINT_GROUP(14, 2)	// group 14 : G0
#else
#define IRQ_TOUCH_INT       IRQ_EINT_GROUP(18, 5)	// group 18 : J0
#endif

#ifndef __GPIO_P1_H_
#define GPIO_TOUCH_EN       S5PV210_GPH2(1)
#if defined(CONFIG_MACH_P1_LTN)
#define GPIO_TOUCH_INT     S5PV210_GPG0(2)
#else
#define GPIO_TOUCH_INT     S5PV210_GPJ0(5)
#endif
#define GPIO_INPUT		0
#define GPIO_OUTPUT		1
#endif

#define MAX_USING_FINGER_NUM	      10

#define I2C_M_WR 0 /* for i2c */
#define I2C_MAX_SEND_LENGTH     300

 /* TSP Key */
#define NUMOFKEYS       4

#define KEY_PRESS        1
#define KEY_RELEASE     0

static const int tsp_keycodes[NUMOFKEYS] ={

        KEY_MENU,
        KEY_HOME,
        KEY_BACK,
        KEY_SEARCH
};

static const char *tsp_keyname[NUMOFKEYS] ={

        "Menu",
        "Home",
        "Back",
        "Search"
};

static bool tsp_keystatus[NUMOFKEYS];

 /* Version */
#define QT602240_VER_22             0x16

 /* Slave addresses */
#define QT602240_APP_LOW		0x4a
#define QT602240_APP_HIGH		0x4b
#define QT602240_BOOT_LOW		0x24
#define QT602240_BOOT_HIGH		0x25

 /* Firmware */
#define QT602240_FW_NAME		"qt602240.fw"
#define FW_PATH                           "/system/etc/TSP_FIRMWARE/qt602240.bin"

 /* Registers */
#define QT602240_FAMILY_ID		0x00
#define QT602240_VARIANT_ID		0x01
#define QT602240_VERSION		0x02
#define QT602240_BUILD			0x03
#define QT602240_MATRIX_X_SIZE		0x04
#define QT602240_MATRIX_Y_SIZE		0x05
#define QT602240_OBJECT_NUM		0x06
#define QT602240_OBJECT_START		0x07

#define QT602240_OBJECT_SIZE		6

 /* Object types */
#define QT602240_DEBUG_DELTAS		2	/* firmware ver 18 only */
#define QT602240_DEBUG_REFERENCES	3	/* firmware ver 18 only */
#define QT602240_DEBUG_CTERANGE		26	/* firmware ver 18 only */
#define QT602240_DEBUG_DIAGNOSTIC	37	/* firmware ver 20 only */
#define QT602240_GEN_MESSAGE		5
#define QT602240_GEN_COMMAND		6
#define QT602240_GEN_POWER		7
#define QT602240_GEN_ACQUIRE		8
#define QT602240_TOUCH_MULTI		9
#define QT602240_TOUCH_KEYARRAY		15
#define QT602240_TOUCH_PROXIMITY	23	/* firmware ver 20 only */
#define QT602240_PROCI_GRIPFACE		20
#define QT602240_PROCG_NOISE		22
#define QT602240_PROCI_ONETOUCH		24
#define QT602240_PROCI_TWOTOUCH		27
#define QT602240_SPT_GPIOPWM		19
#define QT602240_SPT_SELFTEST		25
#define QT602240_SPT_CTECONFIG		28

 /* QT602240_GEN_COMMAND field */
#define QT602240_COMMAND_RESET		0
#define QT602240_COMMAND_BACKUPNV	1
#define QT602240_COMMAND_CALIBRATE	2
#define QT602240_COMMAND_REPORTALL	3
#define QT602240_COMMAND_DIAGNOSTIC	5	/* firmware 20 ver only */

 /* QT602240_GEN_POWER field */
#define QT602240_POWER_IDLEACQINT	0
#define QT602240_POWER_ACTVACQINT	1
#define QT602240_POWER_ACTV2IDLETO	2

 /* QT602240_GEN_ACQUIRE field */
#define QT602240_ACQUIRE_CHRGTIME	0
#define QT602240_ACQUIRE_TCHDRIFT	2
#define QT602240_ACQUIRE_DRIFTST	3
#define QT602240_ACQUIRE_TCHAUTOCAL	4
#define QT602240_ACQUIRE_SYNC		5
#define QT602240_ACQUIRE_ATCHCALST	6	/* firmware 20 ver only */
#define QT602240_ACQUIRE_ATCHCALSTHR	7	/* firmware 20 ver only */

 /* QT602240_TOUCH_MULTI field */
#define QT602240_TOUCH_CTRL		0
#define QT602240_TOUCH_XSIZE		3
#define QT602240_TOUCH_YSIZE		4
#define QT602240_TOUCH_BLEN		6
#define QT602240_TOUCH_TCHTHR		7
#define QT602240_TOUCH_TCHDI		8
#define QT602240_TOUCH_ORIENT		9
#define QT602240_TOUCH_MOVHYSTI		11
#define QT602240_TOUCH_MOVHYSTN		12
#define QT602240_TOUCH_MOVFILTER		13
#define QT602240_TOUCH_NUMTOUCH		14
#define QT602240_TOUCH_MRGHYST		15
#define QT602240_TOUCH_MRGTHR		16
#define QT602240_TOUCH_AMPHYST		17
#define QT602240_TOUCH_XRANGE_LSB	18
#define QT602240_TOUCH_XRANGE_MSB	19
#define QT602240_TOUCH_YRANGE_LSB	20
#define QT602240_TOUCH_YRANGE_MSB	21
#define QT602240_TOUCH_XLOCLIP		22
#define QT602240_TOUCH_XHICLIP		23
#define QT602240_TOUCH_YLOCLIP		24
#define QT602240_TOUCH_YHICLIP		25
#define QT602240_TOUCH_XEDGECTRL	26	/* firmware 20 ver only */
#define QT602240_TOUCH_XEDGEDIST	27	/* firmware 20 ver only */
#define QT602240_TOUCH_YEDGECTRL	28	/* firmware 20 ver only */
#define QT602240_TOUCH_YEDGEDIST	29	/* firmware 20 ver only */

 /* QT602240_PROCI_GRIPFACE field */
#define QT602240_GRIPFACE_CTRL		0
#define QT602240_GRIPFACE_XLOGRIP	1
#define QT602240_GRIPFACE_XHIGRIP	2
#define QT602240_GRIPFACE_YLOGRIP	3
#define QT602240_GRIPFACE_YHIGRIP	4
#define QT602240_GRIPFACE_MAXTCHS	5
#define QT602240_GRIPFACE_SZTHR1	7
#define QT602240_GRIPFACE_SZTHR2	8
#define QT602240_GRIPFACE_SHPTHR1	9
#define QT602240_GRIPFACE_SHPTHR2	10
#define QT602240_GRIPFACE_SUPEXTTO	11	/* firmware 20 ver only */

 /* QT602240_PROCI_NOISE field */
#define QT602240_NOISE_CTRL		0
#define QT602240_NOISE_OUTFLEN		1
#define QT602240_NOISE_GCAFUL_LSB	3
#define QT602240_NOISE_GCAFUL_MSB	4
#define QT602240_NOISE_GCAFLL_LSB	5
#define QT602240_NOISE_GCAFLL_MSB	6
#define QT602240_NOISE_ACTVGCAFVALID	7
#define QT602240_NOISE_NOISETHR		8
#define QT602240_NOISE_FREQHOPSCALE	10
#define QT602240_NOISE_FREQ0		11
#define QT602240_NOISE_FREQ1		12
#define QT602240_NOISE_FREQ2		13
#define QT602240_NOISE_FREQ3		14	/* firmware 20 ver only */
#define QT602240_NOISE_FREQ4		15	/* firmware 20 ver only */
#define QT602240_NOISE_IDLEGCAFVALID	16	/* firmware 20 ver only */

 /* QT602240_SPT_CTECONFIG field */
#define QT602240_CTE_CTRL		0
#define QT602240_CTE_CMD		1
#define QT602240_CTE_MODE		2
#define QT602240_CTE_IDLEGCAFDEPTH	3
#define QT602240_CTE_ACTVGCAFDEPTH	4
#define QT602240_CTE_VOLTAGE	5

 /* Define for QT602240_GEN_COMMAND */
#define QT602240_BOOT_VALUE		0xa5
#define QT602240_BACKUP_VALUE		0x55

 /* Command to unlock bootloader */
#define QT602240_UNLOCK_CMD_MSB		0xaa
#define QT602240_UNLOCK_CMD_LSB		0xdc

 /* Bootloader mode status */
#define QT602240_WAITING_BOOTLOAD_CMD	0xc0	/* valid 7 6 bit only */
#define QT602240_WAITING_FRAME_DATA	0x80	/* valid 7 6 bit only */
#define QT602240_FRAME_CRC_CHECK	0x02
#define QT602240_FRAME_CRC_FAIL		0x03
#define QT602240_FRAME_CRC_PASS		0x04
#define QT602240_APP_CRC_FAIL		0x40	/* valid 7 8 bit only */
#define QT602240_BOOT_STATUS_MASK	0x3f

 /* Touch status */
#define QT602240_SUPPRESS		(1 << 1)
#define QT602240_AMP			(1 << 2)
#define QT602240_VECTOR			(1 << 3)
#define QT602240_MOVE			(1 << 4)
#define QT602240_RELEASE		(1 << 5)
#define QT602240_PRESS			(1 << 6)
#define QT602240_DETECT			(1 << 7)

 /* Touchscreen absolute values */
#define QT602240_MAX_XC			1024
#define QT602240_MAX_YC			600
#define QT602240_MAX_PRESSURE		255
#define QT602240_MAX_SIZE                30
#define QT602240_MAX_ID                 MAX_USING_FINGER_NUM

 //*****************************************************************************
 //
 //
 //      std_objects_driver
 //
 //
 //*****************************************************************************

#define RESERVED_T0                               0u
#define RESERVED_T1                               1u
#define DEBUG_DELTAS_T2                           2u
#define DEBUG_REFERENCES_T3                       3u
#define DEBUG_SIGNALS_T4                          4u
#define GEN_MESSAGEPROCESSOR_T5                   5u
#define GEN_COMMANDPROCESSOR_T6                   6u
#define GEN_POWERCONFIG_T7                        7u
#define GEN_ACQUISITIONCONFIG_T8                  8u
#define TOUCH_MULTITOUCHSCREEN_T9                 9u
#define TOUCH_SINGLETOUCHSCREEN_T10               10u
#define TOUCH_XSLIDER_T11                         11u
#define TOUCH_YSLIDER_T12                         12u
#define TOUCH_XWHEEL_T13                          13u
#define TOUCH_YWHEEL_T14                          14u
#define TOUCH_KEYARRAY_T15                        15u
#define PROCG_SIGNALFILTER_T16                    16u
#define PROCI_LINEARIZATIONTABLE_T17              17u
#define SPT_COMCONFIG_T18                         18u
#define SPT_GPIOPWM_T19                           19u
#define PROCI_GRIPFACESUPPRESSION_T20             20u
#define RESERVED_T21                              21u
#define PROCG_NOISESUPPRESSION_T22                22u
#define TOUCH_PROXIMITY_T23                       23u
#define PROCI_ONETOUCHGESTUREPROCESSOR_T24        24u
#define SPT_SELFTEST_T25                          25u
#define DEBUG_CTERANGE_T26                        26u
#define PROCI_TWOTOUCHGESTUREPROCESSOR_T27        27u
#define SPT_CTECONFIG_T28                         28u
#define SPT_GPI_T29                               29u
#define SPT_GATE_T30                              30u
#define TOUCH_KEYSET_T31                          31u
#define TOUCH_XSLIDERSET_T32                      32u
#define RESERVED_T33                              33u
#define GEN_MESSAGEBLOCK_T34                      34u
#define SPT_GENERICDATA_T35                       35u
#define RESERVED_T36                              36u
#define DEBUG_DIAGNOSTIC_T37                      37u
#define SPARE_T38                                 38u
#define SPARE_T39                                 39u
#define SPARE_T40                                 40u
#define SPARE_T41                                 41u
#define SPARE_T42                                 42u
#define SPARE_T43                                 43u
#define SPARE_T44                                 44u
#define SPARE_T45                                 45u
#define SPARE_T46                                 46u
#define SPARE_T47                                 47u
#define SPARE_T48                                 48u
#define SPARE_T49                                 49u
#define SPARE_T50                                 50u
 /*
  * All entries spare up to 255
 */
#define RESERVED_T255                             255u

 // report ID
#define REPORTID_COMMANDPROECESSOR 1
#define REPORTID_TSP_MIN                 2
#define REPORTID_TSP_MAX                11
#define REPORTID_TSPKEY_MIN           12
#define REPORTID_TSPKEY_MAX           13
#define REPORTID_PALM                       14
#define REPORTID_NOISESUPPRESSION 15

 /*----------------------------------------------------------------------------
   type definitions
 ----------------------------------------------------------------------------*/

 typedef struct
 {
    uint8_t reset;       /*!< Force chip reset             */
    uint8_t backupnv;    /*!< Force backup to eeprom/flash */
    uint8_t calibrate;   /*!< Force recalibration          */
    uint8_t reportall;   /*!< Force all objects to report  */
    uint8_t reserve;   /*!< Turn on output of debug data */
    uint8_t diagnostic;  /*!< Controls the diagnostic object */
 }__packed gen_commandprocessor_t6_config_t;


 typedef struct
 {
    uint8_t idleacqint;    /*!< Idle power mode sleep length in ms           */
    uint8_t actvacqint;    /*!< Active power mode sleep length in ms         */
    uint8_t actv2idleto;   /*!< Active to idle power mode delay length in units of 0.2s*/

 }__packed gen_powerconfig_t7_config_t;

 typedef struct
 {
    uint8_t chrgtime;          /*!< Charge-transfer dwell time             */
    uint8_t reserved;          /*!< reserved                               */
    uint8_t tchdrift;          /*!< Touch drift compensation period        */
    uint8_t driftst;           /*!< Drift suspend time                     */
    uint8_t tchautocal;        /*!< Touch automatic calibration delay in units of 0.2s*/
    uint8_t sync;              /*!< Measurement synchronisation control    */
    uint8_t atchcalst;         /*!< recalibration suspend time after last detection */
    uint8_t atchcalsthr;       /*!< Anti-touch calibration suspend threshold */
    uint8_t atchcalfrcthr;
    uint8_t atchcalfrcratio;
 }__packed gen_acquisitionconfig_t8_config_t;

 typedef struct
 {
    /* Screen Configuration */
    uint8_t ctrl;            /*!< ACENABLE LCENABLE Main configuration field  */

    /* Physical Configuration */
    uint8_t xorigin;         /*!< LCMASK ACMASK Object x start position on matrix  */
    uint8_t yorigin;         /*!< LCMASK ACMASK Object y start position on matrix  */
    uint8_t xsize;           /*!< LCMASK ACMASK Object x size (i.e. width)         */
    uint8_t ysize;           /*!< LCMASK ACMASK Object y size (i.e. height)        */

    /* Detection Configuration */
    uint8_t akscfg;          /*!< Adjacent key suppression config     */
    uint8_t blen;            /*!< Sets the gain of the analog circuits in front of the ADC. The gain should be set in
                             conjunction with the burst length to optimize the signal acquisition. Maximum gain values for
                             a given object/burst length can be obtained following a full calibration of the system. GAIN
                             has a maximum setting of 4; settings above 4 are capped at 4.*/
    uint8_t tchthr;          /*!< ACMASK Threshold for all object channels   */
    uint8_t tchdi;           /*!< Detect integration config           */

    uint8_t orient;  /*!< LCMASK Controls flipping and rotating of touchscreen
                         *   object */
    uint8_t mrgtimeout; /*!< Timeout on how long a touch might ever stay
                         *   merged - units of 0.2s, used to tradeoff power
                         *   consumption against being able to detect a touch
                         *   de-merging early */

    /* Position Filter Configuration */
    uint8_t movhysti;   /*!< Movement hysteresis setting used after touchdown */
    uint8_t movhystn;   /*!< Movement hysteresis setting used once dragging   */
    uint8_t movfilter;  /*!< Position filter setting controlling the rate of  */

    /* Multitouch Configuration */
    uint8_t numtouch;   /*!< The number of touches that the screen will attempt
                         *   to track */
    uint8_t mrghyst;    /*!< The hysteresis applied on top of the merge threshold
                         *   to stop oscillation */
    uint8_t mrgthr;     /*!< The threshold for the point when two peaks are
                         *   considered one touch */

    uint8_t amphyst;          /*!< TBD */

   /* Resolution Controls */
   uint8_t xrange1;       /*!< LCMASK */
   uint8_t xrange2;       /*!< LCMASK */
   uint8_t yrange1;       /*!< LCMASK */
   uint8_t yrange2;       /*!< LCMASK */
   uint8_t xloclip;       /*!< LCMASK */
   uint8_t xhiclip;       /*!< LCMASK */
   uint8_t yloclip;       /*!< LCMASK */
   uint8_t yhiclip;       /*!< LCMASK */
   /* edge correction controls */
   uint8_t xedgectrl;     /*!< LCMASK */
   uint8_t xedgedist;     /*!< LCMASK */
   uint8_t yedgectrl;     /*!< LCMASK */
   uint8_t yedgedist;     /*!< LCMASK */
   uint8_t jumplimit;
 }__packed touch_multitouchscreen_t9_config_t;

 typedef struct
 {
    /* Key Array Configuration */
    uint8_t ctrl;               /*!< ACENABLE LCENABLE Main configuration field           */

    /* Physical Configuration */
    uint8_t xorigin;           /*!< ACMASK LCMASK Object x start position on matrix  */
    uint8_t yorigin;           /*!< ACMASK LCMASK Object y start position on matrix  */
    uint8_t xsize;             /*!< ACMASK LCMASK Object x size (i.e. width)         */
    uint8_t ysize;             /*!< ACMASK LCMASK Object y size (i.e. height)        */

    /* Detection Configuration */
    uint8_t akscfg;             /*!< Adjacent key suppression config     */
    uint8_t blen;               /*!< ACMASK Burst length for all object channels*/
    uint8_t tchthr;             /*!< ACMASK LCMASK Threshold for all object channels   */
    uint8_t tchdi;              /*!< Detect integration config           */
    uint8_t reserved[2];        /*!< Spare x2 */
 }__packed touch_keyarray_t15_config_t;

 typedef struct
 {
  uint8_t ctrl;
  uint8_t xoffset1;
  uint8_t xoffset2;
  uint8_t  xsegment[16];
  uint8_t yoffset1;
  uint8_t yoffset2;
  uint8_t  ysegment[16];
 }__packed proci_linearizationtable_t17_config_t;

 typedef struct
 {
    uint8_t ctrl;
    uint8_t xlogrip;
    uint8_t xhigrip;
    uint8_t ylogrip;
    uint8_t yhigrip;
    uint8_t maxtchs;
    uint8_t reserved;
    uint8_t szthr1;
    uint8_t szthr2;
    uint8_t shpthr1;
    uint8_t shpthr2;
    uint8_t supextto;
 }__packed proci_gripfacesuppression_t20_config_t;

 typedef struct
 {
    uint8_t ctrl;
    uint8_t reserved;
    uint8_t reserved1;
    uint8_t gcaful1;
    uint8_t gcaful2;
    uint8_t gcafll1;
    uint8_t gcafll2;
    uint8_t actvgcafvalid;        /* LCMASK */
    uint8_t noisethr;
    uint8_t reserved2;
    uint8_t freqhopscale;
    uint8_t freq[5u];             /* LCMASK ACMASK */
    uint8_t idlegcafvalid;        /* LCMASK */
 }__packed procg_noisesuppression_t22_config_t;

 typedef struct
 {
    uint8_t ctrl;          /*!< Ctrl field reserved for future expansion */
    uint8_t cmd;           /*!< Cmd field for sending CTE commands */
    uint8_t mode;          /*!< LCMASK CTE mode configuration field */
    uint8_t idlegcafdepth; /*!< LCMASK The global gcaf number of averages when idle */
    uint8_t actvgcafdepth; /*!< LCMASK The global gcaf number of averages when active */
    uint8_t voltage;
 }__packed spt_cteconfig_t28_config_t;

 struct qt602240_info {
     u8 family_id;
     u8 variant_id;
     u8 version;
     u8 build;
     u8 matrix_xsize;
     u8 matrix_ysize;
     u8 object_num;
 };

 struct qt602240_object {
     u8 type;
     u16 start_address;
     u8 size;
     u8 instances;
     u8 num_report_ids;

     /* to map object and message */
     u8 max_reportid;
 };

 struct qt602240_message {
     u8 reportid;
     u8 message[7];
     u8 checksum;
 };

 typedef struct
 {
     uint16_t id;         /*!< id */
     uint16_t size;         /*!< size */
     int16_t pressure;      /*!< dn=40, up=0, none=-1 */
     int16_t x;          /*!< X */
     int16_t y;          /*!< Y */
 } report_finger_info_t;

  /* Each client has this additional data */
struct qt602240_data
{
    unsigned int irq;
    struct i2c_client *client;
    struct input_dev *input_dev;
    struct work_struct ta_work;
    struct qt602240_platform_data *pdata;
    struct qt602240_info *info;
    struct qt602240_object *object_table;
    struct qt602240_message *object_message;
    struct early_suspend	early_suspend;
};

enum Touch_Update_Statue{
    FW_UPDATE_READY = 0,
    FW_UPDATE_DOWNLOADING,
    FW_UPDATE_DONE,
    FW_UPDATE_FAIL,
};

typedef enum
{
    QT_PAGE_UP         = 0x01,
    QT_PAGE_DOWN       = 0x02,
    QT_DELTA_MODE      = 0x10,
    QT_REFERENCE_MODE  = 0x11,
    QT_CTE_MODE        = 0x31
}diagnostic_debug_command;

#if defined(KEY_LED_CONTROL)
void led_control(int data);
void init_led(void);
void touh_led_on(int type);
#endif

#endif  /* _QT602240_H_ */

/* Module information */
MODULE_DESCRIPTION("AT42QT602240 Touchscreen driver");
MODULE_LICENSE("GPL");
