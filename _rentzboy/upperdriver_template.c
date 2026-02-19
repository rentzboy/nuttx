#include "fs.h"
#include "mutex.h"
#include <stdint.h>   // C: defines uint8_t, int32_t, etc.
#include <stdbool.h>
#include <time.h>
#include <stdio.h>
#include <malloc.h>
#include <string.h>

/*
1-Funcionalidades p.774
Clock -time of the day-
Calendar -including compensation 28-29-30-31days
Programmable alarm with interrupt function. The alarm can be triggered by any combination of the calendar fields
Configuracion: source clock, prescaler
Digital calibration
Input tamper detection
Input time-stamp function

***** lower half driver *****
RTC_RD_TIME  | Mostrar un reloj y/o calendario     | int (*readTime) (id_rtc, struct rtc_time *tm)
RTC_SET_TIME | Inicializar el reloj y/o calendario | int (*setTime) (id_rtc, struct rtc_time *tm)

RTC_RD_ALARM  | readAlarm (id_alarm, struct rtc_time *tm)
RTC_SET_ALARM | Programar una alarma from calendar fields -> event/interrupt | setAlarm (id_alarm, struct rtc_time *tm, callback)
RTC_CANCEL_ALARM (id_alarm)

RTC_SET_PERIODIC | Programar un periodic wakeup trigger -> event/interrupt
RTC_CANCEL_PERIODIC

RTC_SET_RELATIVE

***** upper half driver *****
callback(pid_t pid, int signum, FAR void *arg); //task to notify, signum=SIGALRM, arg=alarm time ?? => depende de signals.h (o como se llame)

*/

/*************************************
rtc.h
**************************************/
//IOCTL Commands for RTC driver
#define RTC_RD_TIME        _RTCIOC(0x0001) 

//Types definitions for this driver (customs struct)
struct rtc_time{}; struct rtc_rdalarm_s{};

//Se llama desde µC_lowerhalf.c
struct rtc_lowerhalf_s {
  /* This is the contained reference to the read-only, lower-half operations vtable
     En el I2C driver, se utiliza directamente struct rtc_ops_s *ops en lugar de struct i2c_lowerhalf_s *lower */
  FAR const struct rtc_ops_s *ops;
  /* Data following this can vary from RTC driver-to-driver */
};

//Se declara en rtc.h, pues se definen en stm32_rtc_lowerhalf.c
struct rtc_ops_s {
    /*  Funciones específicas de este driver, se definen en stm32_rtc_lowerhalf.c
        Cada #define ioctl(#) llama a una de estas funciones, mediante un switch/case en rtc_ioctl()
        Defines the callable interface from the RTC common upper-half driver into lower half implementation.
            -> Each method in this structure corresponds to one of the RTC IOCTL commands.  
            -> All IOCTL command logic is implemented in the lower-half driver.
            -> la struct es una lista de punteros a funciones
            -> No incluye las file_operations (open, close, read, ...), solo las del RTC driver (bajo nivel).
    */

    int (*settime)(FAR struct rtc_lowerhalf_s *lower, FAR const struct tm *tm);
    int (*rdtime)(FAR struct rtc_lowerhalf_s *lower, FAR struct tm *tm);

    int (*setalarm)(FAR struct rtc_lowerhalf_s *lower, FAR const struct tm *tm);
    int (*rdalarm)(FAR struct rtc_lowerhalf_s *lower, FAR struct tm *tm);

    int (*setperiodic)(FAR struct rtc_lowerhalf_s *lower, FAR const struct tm *tm);
    int (*rdperiodic)(FAR struct rtc_lowerhalf_s *lower, FAR struct tm *tm);
};

//Se declara en .h, se define en .c y se llama desde stm32_bringup.c
int rtc_initialize(int minor, FAR struct rtc_lowerhalf_s *lower)
{
    /*  -> rtc_upperhalf_s filled with return value from stm32_rtc_initialize() & casted to upper
        -> Encapasula la llamada a register_driver(devpath, &g_rtc_fops, 0666, upper); //&g_rtc_fops a nivel de rtc.c
        -> struct rtc_upperhalf_s *upper = (struct rtc_upperhalf_s *) lower;
    */

}
/*************************************
rtc.c
**************************************/
//Libro de file operations disponibles para el Upper RTC driver. 
struct file_operations g_rtc_fops = {
    /* 
    > No es una declaración de struct file_operations => estamos creando una instancia
    > Se utiliza en register_driver(devpath, &g_rtc_fops, 0666, upper);
    > Lista estática de f()´s disponibles del driver. Compartida por todos los RTC del µC. Genérico. No data.
    > Hay que mantener la ordenación, si alguna f() no está disponible: NULL
    > Realmente es un placeholder, con un orden concreto. Es la manera que tiene Nuttx/POSIX de asociar las file_operations del OS con las funciones correpondientes del driver.
    From open(/dev/rt0) to rtc_open(struct file *filep):
        -> nx_open() encuentra el inode: struct inode *inode = inode_find(path);
        -> llama a la función registrada de g_rtc_fops, según el orden de la struct.
        -> open(*filep) =>  filep->f_inode->i_ops->rtc_open
        -> rtc_open y rtc_close: la funcionalidad principal se ejecuta en open() y close(),
        aquí unicamente incrementamos/decrementamos el contador de instancias inode -> i_crefs
        -> rtc_read, rtc_write: no se usan en este driver
        -> rtc_unlink: lleva la cuenta de i_crefs y cuando se llega a 0 se elimina el driver
        -> rtc_ioctl: mediante switch(), llama a las funciones del lower half driver: settime, rdtime, ...
    >NO CONFUNDIR file_operations con rtc_ops, que es una struct interna del driver. */
    rtc_open,      /* open */
    rtc_close,     /* close */
    rtc_read,      /* read */
    rtc_write,     /* write */
    NULL,          /* seek */
    rtc_ioctl,     /* ioctl */
};

struct rtc_upperhalf_s {
    /*  -> Se pasa como arg a register_driver(....., struct rtc_upperhalf_s *upper); //void *priv
        -> Define sobre qué objeto se ejecutan las funciones de g_rtc_fops
        -> Es específico de la instancia creada en el lower half: cada RTC tiene su propio upper
        -> Antes de registrar el upper half driver, se vincula al rtc_lowerhalf_s
        -> open("/dev/rtc0", ...) => filep->f_inode->i_private = upper
    */
    struct rtc_lowerhalf_s *lower;
};

//rtc_open/close/read/write/ioctl se definen en el rtc.c, pues no se utilizan fuera de este archivo
int rtc_open(FAR struct file *filep) {
    //open() ya ha encontrado el inode de nuestro driver, y devuelve una llamada a esta función
    //aqui simplemente añadimos si hay que hacer alguna cosa más ....
    //Realmente lo único que faltaría según Gregory es añadir contar la nueva instancia: upper->crefs++;
    return OK;
}
int rtc_close(FAR struct file *filep) {
    //en este caso habría que descontar simplemente la instancia: upper->crefs--;
    return 0;
}
ssize_t rtc_read(FAR struct file *filep, FAR char *buffer, size_t buflen) {
    //No se define, pues no se utiliza para RTC
    return buflen;
}
ssize_t rtc_write(FAR struct file *filep, FAR const char *buffer, size_t buflen) {
    //No se define, pues no se utiliza para RTC
    return buflen;
}
int rtc_ioctl(FAR struct file *filep, int cmd, unsigned long arg) {
    //Sin terminar este archivo es solo una plantilla para crear otros drivers
    struct inode *node; 
    node = filep->f_inode;

    struct rtc_upperhalf_s *upper = (struct rtc_upperhalf_s *) node->i_private;
    struct rtc_lowerhalf_s *lower;


    switch (cmd)
    {
#ifdef CONFIG_RTC_ALARM
    case RTC_SET_ALARM:
        upper->lower->ops->setalarm(lower, (FAR const struct tm *)arg);
        break;

    case RTC_CANCEL_ALARM:
        /* code */
        break;
    case RTC_RD_ALARM:
        /* code */
        break;
    case RTC_CANCEL_PERIODIC:
        /* code */
        break;
#endif
#ifdef CONFIG_RTC_DATETIME
    case RTC_RD_TIME:
        /* code */
        break;
    case RTC_SET_TIME:
        /* code */
        break;
#endif
    default:
        break;
    }
    return 0;
}

/*************************************
stm32_rtc_lowerhalf.c
**************************************/
//PENDING
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


};
