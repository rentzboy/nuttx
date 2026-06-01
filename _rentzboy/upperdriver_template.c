#include "fs.h"
#include "mutex.h"
#include <stdint.h>   // C: defines uint8_t, int32_t, etc.
#include <stdbool.h>
#include <time.h>
#include <stdio.h>
#include <malloc.h>
#include <string.h>


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
struct rtc_ops_s
{
  /* rdtime() returns the current RTC time. */
  CODE int (*rdtime)(FAR struct rtc_lowerhalf_s *lower, FAR struct rtc_time *rtctime);

  /* settime sets the RTC's time */
  CODE int (*settime)(FAR struct rtc_lowerhalf_s *lower, FAR const struct rtc_time *rtctime);

  /* havesettime checks if RTC time have been set */
  CODE bool (*havesettime)(FAR struct rtc_lowerhalf_s *lower);

#ifdef CONFIG_RTC_ALARM
  /* setalarm sets up a new alarm. */
  CODE int (*setalarm)(FAR struct rtc_lowerhalf_s *lower, FAR const struct lower_setalarm_s *alarminfo);

  /* setalarm sets up a new alarm relative to the current time. */
  CODE int (*setrelative)(FAR struct rtc_lowerhalf_s *lower, FAR const struct lower_setrelative_s *alarminfo);

  /* cancelalarm cancels the current alarm. */
  CODE int (*cancelalarm)(FAR struct rtc_lowerhalf_s *lower, int alarmid);

  /* rdalarm query the current alarm. */
  CODE int (*rdalarm)(FAR struct rtc_lowerhalf_s *lower, FAR struct lower_rdalarm_s *alarminfo);
#endif
#ifdef CONFIG_RTC_PERIODIC
  /* setperiodic sets up a new periodic wakeup. */
  CODE int (*setperiodic)(FAR struct rtc_lowerhalf_s *lower, FAR const struct lower_setperiodic_s *alarminfo);

  /* cancelperiodic cancels the current periodic wakeup. */
  CODE int (*cancelperiodic)(FAR struct rtc_lowerhalf_s *lower, int alarmid);
#endif
#ifdef CONFIG_RTC_IOCTL
  /* Support for architecture-specific RTC operations */
  CODE int (*ioctl)(FAR struct rtc_lowerhalf_s *lower, int cmd, unsigned long arg);
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  /* The driver has been unlinked and there are no further open references to the driver */
  CODE int (*destroy)(FAR struct rtc_lowerhalf_s *lower);
#endif
};



//Se declara en .h, se define en .c y se llama desde stm32_bringup.c
int rtc_initialize(int minor, FAR struct rtc_lowerhalf_s *lower)
{
    /*  -> rtc_upperhalf_s filled with return from stm32_rtc_initialize (=rtc_lowerhalf_s *lower) => binding upper/lower half driver
        -> Encapasula la llamada a register_driver(devpath, &g_rtc_fops, 0666, upper); //&g_rtc_fops a nivel de rtc.c
        -> El resto de valores de struct rtc_upperhalf_s (crefs, unlinked) se inicializan a 0 al no definirlos el usuario
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
    /*  -> Antes de registrar el upper half driver, se vincula al rtc_lowerhalf_s
        -> Se pasa como arg a register_driver(....., struct rtc_upperhalf_s *upper); //void *priv
        -> Define sobre qué objeto se ejecutan las funciones de g_rtc_fops
        -> Es específico de la instancia creada en el lower half: cada RTC tiene su propio upper
        -> Permite llevar un registro de las instancias creadas -crefs-
        -> open("/dev/rtc0", ...) => filep->f_inode->i_private = upper
    */
  FAR struct rtc_lowerhalf_s *lower;  /* Contained lower half driver */
  mutex_t lock;                       /* Supports mutual exclusion */

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  uint8_t crefs;                      /* Number of open references */
  bool unlinked;                      /* True if the driver has been unlinked */
#endif

#ifdef CONFIG_RTC_ALARM
  /* This is an array, indexed by the alarm ID, that provides information
   * needed to map an alarm expiration to a signal event.
   */

  struct rtc_alarminfo_s alarminfo[CONFIG_RTC_NALARMS];
#endif

#ifdef CONFIG_RTC_PERIODIC
  /* Currently only one periodic wakeup is supported. */

  struct rtc_alarminfo_s periodicinfo;
#endif
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

    struct rtc_upperhalf_s *upper;
    struct rtc_lowerhalf_s *lower;

    upper = (struct rtc_upperhalf_s *) node->i_private;
    lower = upper->lower; //lower = rtc_ops_s


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


