/*  Upper-drive -igual para todos los µc-
    Relaciona /dev/i2c/xyz con open(), close(), read(), write()
    i2c_master es 
    pending: priv 
*/
i2c_register(struct i2c_master *i2c, int bus)
{
    register_driver("/dev/i2c1", &g_i2c_fops, 0666, *priv); // priv es *i2c_master
}

struct i2c_master 
{
    struct file_operations *f_ops;
    struct i2c_private i2c_priv;
};

/*  Upper-drive interface:Sin implementación
    llama a la f() correspondiente para cada µc: stm32_i2c_open(), .....
*/
static int i2c_open(struct file *fp);
static int i2c_close(struct file *fp);
static int i2c_read(struct file *fp, char *buffer, size_t buflen);
static int i2c_write(struct file *fp, char *buffer, size_t buflen);

//Creamos una instancia de struct file_operations
struct file_operations g_i2c_fops =
{
    .open = i2c_open,
    .close = i2c_close,
    .read = i2c_read,
    .write = i2c_write
};

/*  Configura el lower-driver y devuelve un puntero i2c_master
    que se utiliza en i2c_register()
*/
struct i2c_master stm32_i2c_initialize(int bus)
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