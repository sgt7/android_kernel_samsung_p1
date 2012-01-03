/*
** =========================================================================
** File:
**     ImmVibeSPI.c
**
** Description:
**     Device-dependent functions called by Immersion TSP API
**     to control PWM duty cycle, amp enable/disable, save IVT file, etc...
**
** Portions Copyright (c) 2008-2009 Immersion Corporation. All Rights Reserved.
**
** This file contains Original Code and/or Modifications of Original Code
** as defined in and that are subject to the GNU Public License v2 -
** (the 'License'). You may not use this file except in compliance with the
** License. You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software Foundation, Inc.,
** 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or contact
** TouchSenseSales@immersion.com.
**
** The Original Code and all software distributed under the License are
** distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER
** EXPRESS OR IMPLIED, AND IMMERSION HEREBY DISCLAIMS ALL SUCH WARRANTIES,
** INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY, FITNESS
** FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT. Please see
** the License for the specific language governing rights and limitations
** under the License.
** =========================================================================
*/
#include <linux/pwm.h>
#include <plat/gpio-cfg.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include "tspdrv.h"
#include <linux/regulator/consumer.h>

#ifdef CONFIG_TARGET_LOCALE_KOR
#define COUNT_OF_MOTOR
#endif

#ifdef IMMVIBESPIAPI
#undef IMMVIBESPIAPI
#endif
#define IMMVIBESPIAPI static

/*
** This SPI supports only one actuator.
*/
#define NUM_ACTUATORS 1
//#define PWM_DUTY_MAX    579 /* 13MHz / (579 + 1) = 22.4kHz */
//#define FREQ_COUNT		44640	/*89284*/
#define PWM_DEVICE	1

extern unsigned int g_PWM_duty_max;

struct pwm_device	*Immvib_pwm;
static bool g_bAmpEnabled = false;

#ifdef COUNT_OF_MOTOR
struct class *vibe_class;
struct device *vibe_dev;
static unsigned int motor_cnt = 0;

static void vibecnt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned int retcnt=0;
	printk(KERN_DEBUG "vibecnt_show\n");

	retcnt=motor_cnt;
	motor_cnt=0;

	return sprintf(buf, "%d\n", retcnt);

}

static DEVICE_ATTR(vibecnt, S_IRUGO |S_IRUSR, vibecnt_show, NULL);
#endif

/*
** Called to disable amp (disable output force)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpDisable(VibeUInt8 nActuatorIndex)
{
#if 0
#error Please review the code between the #if and #endif

    if (g_bAmpEnabled)
    {
        DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_AmpDisable.\n"));

        g_bAmpEnabled = false;

#if 0
        mhn_gpio_set_level(GPIO_EN, GPIO_LEVEL_LOW);
	    mz_ops.bstat &= ~HN_BATTERY_MOTOR;
#endif
    }
#endif
    if (g_bAmpEnabled)
    {
        g_bAmpEnabled = false;
		s3c_gpio_cfgpin(VIB_PWM, S3C_GPIO_OUTPUT);
        pwm_config(Immvib_pwm, 0, g_PWM_duty_max/2);
        pwm_disable(Immvib_pwm);
        if(gpio_get_value(VIB_EN))
        {
            regulator_disable(regulator_motor);
        }
        gpio_direction_output(VIB_EN, 0);
#ifdef COUNT_OF_MOTOR
        motor_cnt++;
#endif
    }

    return VIBE_S_SUCCESS;
}

/*
** Called to enable amp (enable output force)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpEnable(VibeUInt8 nActuatorIndex)
{
#if 0
#error Please review the code between the #if and #endif

    if (!g_bAmpEnabled)
    {
        DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_AmpEnable.\n"));

        g_bAmpEnabled = true;

#if 0
        /*
        ** Ensure the PWM frequency is at the expected value. These 2 lines of code
        ** can be removed if no other application alters the PWM frequency.
        */
        PWM_CTRL  = 0;                  /* 13Mhz / (0 + 1) = 13MHz */
        PWM_PERIOD = PWM_DUTY_MAX;      /* 13Mhz / (PWM_DUTY_MAX + 1) = 22.4kHz */

        /* Set duty cycle to 50% */
        PWM_DUTY = (PWM_DUTY_MAX+1)>>1; /* Duty cycle range = [0, PWM_DUTY_MAX] */

        /* Enable amp */
        mhn_gpio_set_level(GPIO_EN, GPIO_LEVEL_HIGH);
        mz_ops.bstat |= HN_BATTERY_MOTOR;
#endif
    }
#endif

    if (!g_bAmpEnabled)
    {
        g_bAmpEnabled = true;
		s3c_gpio_cfgpin(VIB_PWM, S3C_GPIO_SFN(2));
        regulator_enable(regulator_motor);
        pwm_enable(Immvib_pwm);
        gpio_direction_output(VIB_EN, 1);
    }

    return VIBE_S_SUCCESS;
}

/*
** Called at initialization time to set PWM freq, disable amp, etc...
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Initialize(void)
{
#if 0
#error Please review the code between the #if and #endif

    DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_Initialize.\n"));

    g_bAmpEnabled = true;   /* to force ImmVibeSPI_ForceOut_AmpDisable disabling the amp */

    /*
    ** Disable amp.
    ** If multiple actuators are supported, please make sure to call ImmVibeSPI_ForceOut_AmpDisable
    ** for each actuator (provide the actuator index as input argument).
    */
    ImmVibeSPI_ForceOut_AmpDisable(0);

#if 0
    /*
    ** PWM frequency:
    ** The PWM frequency must be set to a fixed value and shouldn't change
    ** during the lifetime of the app. The ideal solution would be to use a
    ** frequency value between 20kHz and 50kHz. A frequency value slightly
    ** outside of the above limits should still work and be compliant with
    ** TSP requirements (please refer to the TSP integration guide for
    ** further information).
    */

    /* 22.4kHz PWM, duty cycle 50% */
    PWM_CTRL = 0;                   /* 13Mhz / (0 + 1) = 13MHz */
    PWM_PERIOD = PWM_DUTY_MAX;      /* 13Mhz / (PWM_DUTY_MAX + 1) = 22.4kHz */
    PWM_DUTY = (PWM_DUTY_MAX+1)>>1; /* Duty cycle range = [0, PWM_DUTY_MAX] */
#endif
#endif
    g_bAmpEnabled = true;

    Immvib_pwm = pwm_request(PWM_DEVICE, "Immvibtonz");
    pwm_config(Immvib_pwm, g_PWM_duty_max/2, g_PWM_duty_max);
    ImmVibeSPI_ForceOut_AmpDisable(0);

#ifdef COUNT_OF_MOTOR
    vibe_class = class_create(THIS_MODULE, "vibe");
    if (IS_ERR(vibe_class))
        return PTR_ERR(vibe_class);

    vibe_dev = device_create(vibe_class, NULL, 0, NULL, "vibetonz");

    if (device_create_file(vibe_dev, &dev_attr_vibecnt) < 0)
        printk("[Vibetonz] Failed to create device file(%s)!\n", dev_attr_vibecnt.attr.name);
#endif

    return VIBE_S_SUCCESS;
}

/*
** Called at termination time to set PWM freq, disable amp, etc...
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Terminate(void)
{
//#error Please review the code between the #if and #endif

 //   DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_Terminate.\n"));

    /*
    ** Disable amp.
    ** If multiple actuators are supported, please make sure to call ImmVibeSPI_ForceOut_AmpDisable
    ** for each actuator (provide the actuator index as input argument).
    */
    ImmVibeSPI_ForceOut_AmpDisable(0);

#if 0
    /* Set PWM frequency */
    PWM_CTRL  = 0;                  /* 13Mhz / (0 + 1) = 13MHz */
    PWM_PERIOD = PWM_DUTY_MAX;      /* 13Mhz / (PWM_DUTY_MAX + 1) = 22.4kHz */

    /* Set duty cycle to 50% */
    PWM_DUTY = (PWM_DUTY_MAX+1)>>1; /* Duty cycle range = [0, PWM_DUTY_MAX] */
#endif

    return VIBE_S_SUCCESS;
}

/*
** Called by the real-time loop to set PWM duty cycle, and enable amp if required
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Set(VibeUInt8 nActuatorIndex, VibeInt8 nForce)
{
#if 0
#error Please review the code between the #if and #endif

#if 0
    VibeUInt32 nTmp;
#endif

    if (nForce == 0)
    {
#if 0
		/* Set 50% duty cycle */
        PWM_DUTY = (PWM_DUTY_MAX+1)>>1;
#endif
    }
    else
    {
#if 0
        /* Map force from [-127, 127] to [0, PWM_DUTY_MAX] */
        nTmp = ((nForce + 127) * 9) >> 2;

        /*
        ** The above mapping will output PWM duty cycle in [0, PWM_DUTY_MAX] range.
        ** For PWM_DUTY_MAX == 579, max duty cycle is 98.6%.
        ** As long as we don't hit the 100% mark, we're OK
        */
        PWM_DUTY = nTmp;
#endif
    }
#endif

	int pwm_duty=g_PWM_duty_max/2 + ((g_PWM_duty_max/2 - 2) * nForce)/127;

	if (nForce == 0)
	{
		ImmVibeSPI_ForceOut_AmpDisable(0);

	}
	else
	{
		pwm_config(Immvib_pwm, pwm_duty, g_PWM_duty_max);
		ImmVibeSPI_ForceOut_AmpEnable(0);
	}

//	printk(KERN_DEBUG "[Vibtonz] %s nForce: %d\n",__func__, nForce);

	return VIBE_S_SUCCESS;
}

/*
** Called by the real-time loop to set force output, and enable amp if required
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetSamples(VibeUInt8 nActuatorIndex, VibeUInt16 nOutputSignalBitDepth, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer)
{
    /* This function is not called for LRA device */
    return VIBE_S_SUCCESS;
}

/*
** Called to set force output frequency parameters
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetFrequency(VibeUInt8 nActuatorIndex, VibeUInt16 nFrequencyParameterID, VibeUInt32 nFrequencyParameterValue)
{
//#error Please review the code between the #if and #endif

#if 0
    #error  "The OEM must handle different frequency parameters here"
#endif

    return VIBE_S_SUCCESS;
}

/*
** Called to get the device name (device name must be returned as ANSI char)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_Device_GetName(VibeUInt8 nActuatorIndex, char *szDevName, int nSize)
{
//#error Please review the code between the #if and #endif

#if 0   /* The following code is provided as a sample. Please modify as required. */
    if ((!szDevName) || (nSize < 1)) return VIBE_E_FAIL;

    DbgOut((KERN_DEBUG "ImmVibeSPI_Device_GetName.\n"));

    strncpy(szDevName, "Generic Linux Device", nSize-1);
    szDevName[nSize - 1] = '\0';    /* make sure the string is NULL terminated */
#endif

    return VIBE_S_SUCCESS;
}
