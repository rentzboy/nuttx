#include "fs.h"
#include "mutex.h"

/* Upper RTC driver*/
struct rtc_upperhalf_s {
    struct rtc_ops_s *ops;          /* Pointer to the operations structure */
    struct rtc_lowerhalf_s *lower;  /* Contain lower half driver */
    mutex_t lock;                   /* Mutex to protect access to this structure */
};

struct file_operations g_rtc_ops = {
    .open = rtc_open,
    .close = rtc_close,
    .read = rtc_read,
    .write = rtc_write,
    .ioctl = rtc_ioctl
};

//Todo: upper or lower to register_driver ??
int rtc_register(struct rtc_upperhalf_s *upper) {
    const char *devName = "/dev/rtc0";
    register_driver(devName, &g_rtc_ops, 0666, upper);
    return 0;
}

//Todo: upper or lower to register_driver ??
int stm32_rtc_initialize(int minor, FAR struct rtc_lowerhalf_s *lower) {
    return 0;
}

struct rtc_ops_s {
    int (*settime)(FAR struct rtc_lowerhalf_s *lower, FAR const struct tm *tm);
    int (*rdtime)(FAR struct rtc_lowerhalf_s *lower, FAR struct tm *tm);
}

/* Lower RTC driver*/
struct rtc_lowerhalf_s {
    struct rtc_ops_s *ops; /* Pointer to the operations structure */
    struct rtc_config_s *config;
}

struct rtc_config_s {

#if CONFIG_STM32_RTC_LSICLOCK 

#elif CONFIG_STM32_RTC_LSECLOCK

#elif CONFIG_STM32_RTC_HSECLOCK

#else
    #error "Unsupported clock source"
#endif

//Clock source
    uint8_t clock_source;

//Clock prescaler
    uint16_t prescaler;

//Alarm prescaler
    uint16_t alarm_prescaler;

//Alarm mask
    uint8_t alarm_mask;

//Alarm trigger
    uint8_t alarm_trigger;

//Alarm event
    uint8_t alarm_event;

}