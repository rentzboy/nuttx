#include "fs.h"
#include "mutex.h"
#include <stdint.h>   // C: defines uint8_t, int32_t, etc.
#include <stdbool.h>
#include <time.h>
#include <stdio.h>
#include <malloc.h>
#include <string.h>

/* from board.h*/
#define STM32_HSI_FREQUENCY     8000000ul
#define STM32_LSI_FREQUENCY     40000            /* Between 30kHz and 60kHz */
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768            /* X2 on board => NO instalado en la Discovery_3 */

/* Calculados para los HSI/LSI de la Discovery_3 => Inputclock / (asyncPrediv * syncPrediv) = 1Hz */
#define STM32_RTC_LSE_ASYNC_PREDIV    (0x80) /* 7-bit asynchronous clock divider -128- */
#define STM32_RTC_LSE_SYNC_PREDIV     (0x100) /* 15-bit asynchronous clock divider -256- */
#define STM32_RTC_LSI_ASYNC_PREDIV    (0x64) /* 7-bit asynchronous clock divider -100- */
#define STM32_RTC_LSI_SYNC_PREDIV     (0x190) /* 15-bit asynchronous clock divider -400- */

#define STM32_RTC_BASE                0x40002800
#define STM32_PWR_BASE                0x40007000

/* RTC control register */
#define STM32_RTC_CR                  STM32_RTC_BASE + 0x08
#define STM32_RTC_CR_TSEDGE           (u_int32_t)(1 << 10)
#define STM32_RTC_WUCKSEL_RTC/16      (u_int32_t)(0 << 2)
#define STM32_RTC_WUCKSEL_RTC/8       (u_int32_t)(1 << 2)
#define STM32_RTC_WUCKSEL_RTC/4       (u_int32_t)(2 << 2)
#define STM32_RTC_WUCKSEL_RTC/2       (u_int32_t)(3 << 2)
#define STM32_RTC_WUCKSEL_1HZ         (u_int32_t)(4 << 2)
#define STM32_RTC_CR_REFCKON          (u_int32_t)(1 << 4)
#define STM32_RTC_CR_BYPASS           (u_int32_t)(1 << 5)
#define STM32_RTC_CR_FMT              (u_int32_t)(1 << 6)
#define STM32_RTC_CR_ALRAE            (u_int32_t)(1 << 8)
#define STM32_RTC_CR_WUTE             (u_int32_t)(1 << 10)
#define STM32_RTC_CR_ALRAIE           (u_int32_t)(1 << 12)
#define STM32_RTC_CR_POL              (u_int32_t)(1 << 20) /* Output polarity */
#define STM32_RTC_CR_OSEL             (u_int32_t)(1 << 21) /* Alarma A Output selection */
#define STM32_RTC_CR_OSEL             (u_int32_t)(2 << 21) /* Alarma B Output selection */
#define STM32_RTC_CR_OSEL             (u_int32_t)(3 << 21) /* Wakeup Output selection */
#define STM32_RTC_CR_PM               (u_int32_t)(1 << 22) /* 0: 24h mode, 1: 12h mode*/
#define STM32_RTC_CR_COE              (u_int32_t)(1 << 23) /* Calibration output enable */

/* RTC write protection register */
#define STM32_RTC_WPR                 STM32_RTC_BASE + 0x24

/* PWR control register */
#define STM32_PWR_CR                  STM32_PWR_BASE + 0x00
#define STM32_PWR_CR_DBP              (1 << 8)  /* Bit 8: Disable Backup Domain write protection */

/* stm32_rtc_config_t values */
#define STM32_RTC_OUTPUT_POLARITY_LOW     0   /* 0: pin is high when ALRAF/ALRBF/WUTF is asserted */
#define STM32_RTC_OUTPUT_POLARITY_HIGH    1   /* 1: pin is low when ALRAF/ALRBF/WUTF is asserted */

#define STM32_RTC_OUTPUT_TYPE_OPEN_DRAIN  0   /* 0: pin is open drain when ALRAF/ALRBF/WUTF is asserted */ 
#define STM32_RTC_OUTPUT_TYPE_PUSH_PULL   1   /* 1: pin is push-pull when ALRAF/ALRBF/WUTF is asserted */
 
#define STM32_RTC_INPUT_TS_RISE_EDGE      0   /* 0: Timestamp event occurs on the rising edge of the event */
#define STM32_RTC_INPUT_TS_FALL_EDGE      1   /* 1: Timestamp event occurs on the falling edge of the event input */

#define STM32_RTC_CALIBRATION_512HZ       0   /* 0: Calibration output is 512 Hz */
#define STM32_RTC_CALIBRATION_1HZ         1   /* 1: Calibration output is 1 Hz */

#define STM32_RTC_WAKEUP_CLOCK_RTCCLK_DIV16   0   /* 0: RTCCLK / 16 */
#define STM32_RTC_WAKEUP_CLOCK_RTCCLK_DIV8    1   /* 1: RTCCLK / 8 */
#define STM32_RTC_WAKEUP_CLOCK_RTCCLK_DIV4    2   /* 2: RTCCLK / 4 */
#define STM32_RTC_WAKEUP_CLOCK_RTCCLK_DIV2    3   /* 3: RTCCLK / 2 */
#define STM32_RTC_WAKEUP_CLOCK_1HZ            4   /* 4: 1 Hz */

/* Initialization values TODO: move to board.h */
#define STM32_RTC_OUTPUT_POLARITY        STM32_RTC_OUTPUT_POLARITY_LOW
#define STM32_RTC_OUTPUT_TYPE            STM32_RTC_OUTPUT_TYPE_PUSH_PULL
#define STM32_RTC_INPUT_TS_EDGE          STM32_RTC_INPUT_TS_FALL_EDGE
#define STM32_RTC_CALIBRATION            STM32_RTC_CALIBRATION_1HZ
#define STM32_RTC_WAKEUP_CLOCK           STM32_RTC_WAKEUP_CLOCK_1HZ 

/****************************************************************************
 * Private Types
 ****************************************************************************/
typedef struct rtc_clock_config_s {
    uint8_t clock_source;
    uint16_t asyncPrescaler;
    uint16_t syncPrescaler;
} rtc_clock_config_t;

#if defined(CONFIG_RTC_ALARM) || defined(CONFIG_RTC_TIMESTAMP)
typedef struct rtc_calendar_config_s {
    uint8_t format; /* 0: 24h mode, 1: 12h mode*/
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    /* calendar date */
    uint8_t day;
    uint8_t month;
    uint8_t year;
    uint8_t weekDay;
} rtc_calendar_config_t;
#endif

#if defined(CONFIG_RTC_WAKEUP)
typedef struct rtc_wakeup_config_s {
    uint8_t wakeupClock;
    uint16_t wakeupCounter;
} rtc_wakeup_config_t;
#endif

#if defined(CONFIG_RTC_TAMPER)
typedef struct rtc_tamper_config_s {
    uint8_t filter;
    uint8_t sampleFrequence;
    uint8_t precharge;
    uint8_t tamper_PullUp;
    uint8_t timeStamp;
    uint8_t triggerEdge;
} rtc_tamper_config_t;
#endif

/* stm32_rtc_config_t definition */
typedef struct stm32_rtc_config_s {
    /* clocks*/
    rtc_clock_config_t clocks;

#if defined(CONFIG_RTC_ALARM_OUTPUT) || defined(CONFIG_RTC_WAKEUP_OUTPUT)
    uint16_t outputPolarity;
    uint16_t outputType;
#endif
#if defined(CONFIG_RTC_TIMESTAMP)
    uint8_t triggerEdge;
#endif
    //En la union solo se puede tener un miembro activo: si un miembro tiene varios campos hay que crear una struct
    /* La union no es necesaria, pues con los if/else unicamente hay un miembro activo */
    union {
#if defined(CONFIG_RTC_ALARM) || defined(CONFIG_RTC_TIMESTAMP)
        rtc_calendar_config_t calendar;
#endif
#if defined(CONFIG_RTC_WAKEUP)
        rtc_wakeup_config_t wakeup;
#endif
#if defined(CONFIG_RTC_CALIB)
    uint8_t outputFrequence; /* calibration 512 Hz or 1 Hz */
#endif
#if defined(CONFIG_RTC_TAMPER)
        rtc_tamper_config_t tamper;
#endif
    } u;
} stm32_rtc_config_t;

//TODO: revisar/terminar/eliminar ?
typedef struct stm32_rtc_ops_s {
    //int (*rtc_open)(FAR struct file *filep);
    //int (*settime)(FAR struct rtc_lowerhalf_s *lower, FAR const struct tm *tm);
    //int (*rdtime)(FAR struct rtc_lowerhalf_s *lower, FAR struct tm *tm);
}stm32_rtc_ops_t;

/* Private RTC driver */ /* TODO: Añadir los campos necesarios para hacer funcionar el driver */
typedef struct stm32_rtc_priv_s
{
    const struct stm32_rtc_config_t *config;
    int refs;                         /* Reference count */
    mutex_t lock;                     /* Mutual exclusion mutex */
}stm32_rtc_priv_t;

/* Lower RTC driver*/
typedef struct rtc_lowerhalf_s {
    stm32_rtc_ops_t *ops;
    stm32_rtc_priv_t *priv;
}rtc_lowerhalf_t;

/* Upper RTC driver */
typedef struct rtc_upper_half_s {
    stm32_rtc_ops_t *ops;
} rtc_upperhalf_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/
static stm32_rtc_config_t rtc_config_init = {
    .clocks = {
    #if defined(CONFIG_STM32_RTC_LSICLOCK)
            .clock_source = STM32_LSI_FREQUENCY,
            .asyncPrescaler = STM32_RTC_LSI_ASYNC_PREDIV,
            .syncPrescaler = STM32_RTC_LSI_SYNC_PREDIV,
    #elif defined(CONFIG_STM32_RTC_LSECLOCK)
            #error "Unsupported clock source: Xtal not available in Discovery_3"
            .clock_source = STM32_LSE_FREQUENCY,
            .asyncPrescaler = RTC_LSE_ASYNC_PREDIV,
            .syncPrescaler = RTC_LSE_SYNC_PREDIV,
    #elif defined(CONFIG_STM32_RTC_HSECLOCK)
            .clock_source = STM32_HSE_FREQUENCY, /* Wake-up no available for HSE clock */
    #else
            #error "Unsupported clock source"
    #endif
        }, /* END clocks */
    #if defined(CONFIG_RTC_ALARM_OUTPUT) || defined(CONFIG_RTC_WAKEUP_OUTPUT)
        .outputPolarity = STM32_RTC_OUTPUT_POLARITY,  /* TODO:Revisar si opendrain y polarity */
        .outputType = STM32_RTC_OUTPUT_TYPE, /* or RTC_OUTPUT_POLARITY_PUSH_PULL */
    #endif
    #if defined(CONFIG_RTC_TIMESTAMP)
        uint8_t triggerEdge = STM32_RTC_INPUT_TS_EDGE,
    #endif
    /* Only one output can be enabled at once: RTC_ALARM, RTC_CALIB, RTC_WAKEUP, RTC_TIMESTAMP */
        .u = {
    #if defined(CONFIG_RTC_ALARM) || defined(CONFIG_RTC_TIMESTAMP)
            .calendar = {
                .format = 0,
                .hours = 0,
                .minutes = 0,
                .seconds = 0,
                .day = 1,
                .month = 1,
                .year = 2000,
                .weekDay = 0
            },
    #elif defined(CONFIG_RTC_WAKEUP)
            .wakeup = {
                .wakeupClock = STM32_RTC_WAKEUP_CLOCK,
                .wakeupCounter = 0x00
            },  
    #elif defined(CONFIG_RTC_CALIB)
            outputFrequence = STM32_RTC_CALIBRATION,
    #elif defined(CONFIG_RTC_TAMPER)
            .tamper = {
                .filter = 0,
                .sampleFrequence = 0,
                .precharge = 0,
                .tamper_PullUp = 0,
                .timeStamp = 0,
                .triggerEdge = 0
            }
    #endif
        }
};

//TODO: añadir campos necesarios para el driver + esto tiene que ir en stm32_rtc_initialize()
static stm32_rtc_priv_t rtc_priv_init = {
    .config = &rtc_config_init,
    .refs = 0,
    .lock = 0
};

//TODO: move to rtc_upperhalf.c
static const struct file_operations g_rtc_ops = {
/* Hay que mantener siempre este mismo orden */
  rtc_open,    /* open */
  rtc_close,   /* close */
  rtc_read,    /* read */
  rtc_write,   /* write */
  NULL,        /* seek */
  rtc_ioctl,   /* ioctl */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static int rtc_open(FAR struct file *filep);
static int rtc_close(FAR struct file *filep);
static ssize_t rtc_read(FAR struct file *filep, FAR char *buffer, size_t buflen);
static ssize_t rtc_write(FAR struct file *filep, FAR const char *buffer, size_t buflen);
static int rtc_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

int stm32_rtc_init(stm32_rtc_priv_t *priv);
void stm32_rtc_deinit(void);


/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
 rtc_upperhalf_t* stm32_rtc_initialize(void)
{
    stm32_rtc_priv_t *priv = NULL;
    rtc_lowerhalf_t *inst = NULL;

    priv = (stm32_rtc_priv_t *)&rtc_priv_init;

    if (!(inst = kmm_malloc(sizeof(rtc_lowerhalf_t))))
    {
      return NULL;
    }

    inst->ops = &g_rtc_ops;
    inst->priv = priv;

    //GPIOs & Clocks & RTC registers initialization from RM
    stm32_rtc_init(priv);

    return (rtc_upperhalf_t*)inst; //solo devuelvo la parte inicial (upper_half)
}

int stm32_rtc_register(void)
{
    int ret;
    rtc_upperhalf_t *priv = stm32_rtc_initialize();
    if (priv == NULL)
    {
        return -ENOMEM;
    }

    ret = register_driver("/dev/rtc", &g_rtc_ops, 0666, priv);
    if (ret == NULL)
    {
        return -ENOMEM;
    }
    return OK;
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/
int stm32_rtc_init(stm32_rtc_priv_t *priv)
{
    /* Calendar initialization and configuration
    1- Set INIT bit to 1 in the RTC_ISR register to enter initialization mode. 
    2- Poll INITF bit to 1 in the RTC_ISR register (takes 2 cycles)
    3- Program both the prescaler factors in RTC_PRER register to get 1 Hz clock for the calendar counter
    4- Load the initial time and date values in the shadow registers (RTC_TR and RTC_DR),
       and configure the time format (12 or 24 hours) through the FMT bit in the RTC_CR register.
    5- Exit the initialization mode by clearing the INIT bit (takes 4 RTCCLK clock cycles) */
    stm32_rtc_calendar_config();

   /* RTC registers write protection disable pag. 780*/
   stm32_rtc_write_enable();

    /* Clocks & prescalers configuration 778 */
    stm32_rtc_clocks_config();
    /* GPIO configuration & Output control pag.776*/
    stm32_rtc_gpios_config();
    /* Once the RTC_ALARM output is enabled, it has priority over RTC_CALIB */
    /* Interrupts */

    return OK;
}

 void stm32_rtc_wakeup_write(time_t interval)
{
#if defined(CONFIG_RTC_WAKEUP)
    DEBUGASSERT(interval <= 65535);

    //TODO: Check for RTC write protection removed

    //Check if wakeup is enabled
    uint32_t regval;
    regval = getreg32(STM32_RTC_CR);
    if(!regval &= RTC_CR_WUTE)
    {
        fprintf(stderr, "Wakeup not enabled\n");
        return 0;
    }

    if (interval < 1)
    {
        //TODO: Al pasarle solamente un parametro, interval siempre > 1 sec
    }
    else
    {
        /* Clock source from 1Hz internal clock */
        rtc_config.u.wakeup.wakeupClock = STM32_RTC_WUCKSEL_1HZ;
        rtc_config.u.wakeup.wakeupCounter = interval;
    }
#endif
}

/* After reset all RTC registers are write protected. Function removes write protection pag. 780 */ 
int stm32_rtc_write_enable(void)
{
    /* 1- Set DBP bit in PWR_CR (Power Control Register)*/
    uint32_t regval;
    regval = getreg32(STM32_PWR_CR);
    regval |= STM32_PWR_CR_DBP;
    putreg32(regval, STM32_PWR_CR);

    /* 2- Enable write access to RTC registers */
    regval = (u_int32_t)0xCA;
    putreg32(regval, STM32_RTC_WPR);
    regval = (u_int32_t)0x53;
    putreg32(regval, STM32_RTC_WPR);
    return OK;
}

int stm32_rtc_clocks_config(void)
{

    return OK;
    
}
/* Enables or disables the RTC wake-up feature */
void stm32_rtc_wakeup_Cmd(bool enable)
{
    uint32_t regval;
    regval = getreg32(STM32_RTC_CR);
    
    if (enable)
    {
        regval |= STM32_RTC_CR_WUTE;
        putreg32(regval, STM32_RTC_CR);
    }else
    {
        regval &= ~STM32_RTC_CR_WUTE;
        putreg32(regval, STM32_RTC_CR);
    } 
}