
#include <unistd.h>
/*  Upper-drive -igual para todos los µc-
    Relaciona /dev/i2c/xyz con open(), close(), read(), write()
    i2c_master es 
    pending: priv 
*/
rtc_register(struct rtc_upper_half *i2c, int bus)
{
    register_driver("/dev/rtc0", &g_rtc_fops, 0666, *priv); // priv es *i2c_master
}

struct rtc_upper_half 
{
    struct file_operations *f_ops;;
};

/*  Upper-drive interface:Sin implementación
    llama a la f() correspondiente para cada µc: stm32_i2c_open(), .....
*/
static int rtc_open(struct file *fp);
static int rtc_close(struct file *fp);
static int rtc_ioctl(FAR struct file *filep, int cmd, unsigned long arg);


//Creamos una instancia de struct file_operations
struct file_operations g_rt_fops =
{
    .open = rtc_open,
    .close = rtc_close,
    .read = NULL,
    .write = NULL,
    .ioctl = rtc_ioctl
};

/*  Configura el lower-driver y devuelve un puntero i2c_master
    que se utiliza en i2c_register()
*/
struct rtc__master stm32_i2c_initialize(int bus)
{
    struct stm32_i2c_configure i2c_config;
    struct stm32_i2c_private i2c_priv;

    switch (bus)
    {
    case 1:
        i2c_confiG = I2C_CONFIGURATION_BUS_1;
        break;
    case 2:
        i2c_confiG = I2C_CONFIGURATION_BUS_2;
        break;
    }
}