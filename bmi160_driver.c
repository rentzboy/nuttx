#include <stdio.h>
#include <stdint.h>
#include <stddef.h>

//--------------FS --------------//
struct file {
    int flags; //O_READ, O_WRITE, O_RDWR, O_CREAT, O_TRUNC, O_APPEND
    int pos;
    struct inode *i_node;
    char *priv;// private data for the file, e.g., a device structure
};

//Upper half driver interface
struct file_ops_s {
    int (*open)(struct file* filep); //filep = i_node->i_priv +flags + pos
    int (*close)(struct file* filep);
    int (*read)(struct file* filep, char *buffer, int buflen);
    int (*write)(struct file* filep, char *buffer, int buflen);
};

union i_ops_u
{
    struct file_ops_s *ops;
    //El resto no se incluyen pues para I2C no se van a utilizar
};

struct inode {
    struct inode *parent;
    struct inode *child;
    struct inode *sibling;
    int crefs;
    int flags;

    union i_ops_u *u;
    
    char *i_priv; //xxx_dev_s, i2c_master_s, etc.
    char i_name[1]; //dev/xxx/yxz => al registrar el driver habrá que amplicar el array y copiar el nombre
};

int register_driver(char *devName, struct file_operations *f_ops, int mode, void *priv) {
    //Hay que crear el inode al registrar el driver y completarlo con los datos del driver
    struct inode *i_node = kmalloc(sizeof(struct inode)); 
    i_node->i_priv = priv; //i2c_master_s, etc.
    i_node->u->ops = f_ops; //&g_i2c_drvr_ops
    i_node->flags = mode;     //0666 for char devices
    strncpy(i_node->i_name, devName, strlen(devName)); //e.g., "/dev/i2c1" => no está bien, solo se va a copiar 1 char en i_name[1]
}

//-------------- I2C --------------//
struct i2c_drvr_s
{
    struct i2c_master_s *i2c;
    int cref;
    mutex_t lock;
    bool unlinked;
};

struct file_ops_s g_i2c_drvr_ops = {
    //Hay que mantener este orden para que funcione
    i2c_drvr_open,
    i2c_drvr_close,
    i2c_drvr_read,
    i2c_drvr_write
};

i2c_drvr_open(struct file* filep) {
    /* nuttx/drivers/i2c/i2c_driver.c
    Find filep->i_node->i_name (= /dev/xyz/xxx) in the inode->i_name tree and returns fd
    if matching. Otherwise, returns the error ENOENT.
    En NuttX únicamente se pueden crear device files con register_driver() */
    
    struct i2c_drvr_s *i2c_drvr;

  /* Get our private data structure */
  i2c_drvr = filep->i_node->i_priv;

  /* Get exclusive access to the I2C driver state structure */
    nxmutex_lock(&i2c_drvr->lock); //OJO: NO libero el lock hasta llamar a i2c_drvr_close()

  /* I2c master initialize */
  if (i2c_drvr->i2c->ops->setup != NULL && i2c_drvr->cref == 0)
  {
      I2C_SETUP(i2c_drvr->i2c);
  }

  /* Increment the count of open references on the driver */
  i2c_drvr->cref++;
}
int i2c_drvr_close(struct file* filep) {
    /* nuttx/drivers/i2c/i2c_driver.c */
    
    struct i2c_drvr_s *i2c_drvr;

  /* Get our private data structure */
    i2c_drvr = filep->i_node->i_priv;

    /* Get exclusive access to the I2C driver state structure */
    nxmutex_lock(&i2c_drvr->lock);

    /* I2c master uninitialize */
    if (i2c_drvr->i2c->ops->shutdown != NULL && i2c_drvr->cref == 1)
    {
        I2C_SHUTDOWN(i2c_drvr->i2c);
    }

  /* Decrement the count of open references on the driver */
  i2c_drvr->cref--;

  /* If the count has decremented to zero and the driver has been unlinked, kill-it */
  if (i2c_drvr->cref <= 0 && i2c_drvr->unlinked)
    {
      nxmutex_destroy(&i2c_drvr->lock);
      kmm_free(i2c_drvr);
      filep->i_node->i_priv = NULL;
      return;
    }

}
int i2c_drvr_read(struct file* filep, char *buffer, int buflen) {
    /* No hace falta definirla, pues es unicamente una interfaz de lectura.
    En su lugar se llamará a la funcion read del sensor */
}
int i2c_drvr_write(struct file* filep, char *buffer, int buflen) {
    /* No hace falta definirla, pues es unicamente una interfaz de escritura.
    En su lugar se llamará a la funcion write del sensor */
}

//-------------- I2C-master --------------//
//Lower half driver interface
struct i2c_ops_s {
    //Estas funciones son propias de cada µC, y se implementan en el I2C driver de STM32 -stm32_i2c_v2.c-
    (*transfer) (struct i2c_master_s *dev, struct i2c_msg_s *msgs, int count);
    (*setup)    (struct i2c_master_s *dev, struct config_s *config);
    (*reset)    (struct i2c_master_s *dev);
    (*shutdown) (struct i2c_master_s *dev);
};

struct i2c_master_s {
    struct i2c_ops_s *ops;
};

struct i2c_config_s
{
    //nuttx/include/nuttx/i2c/i2c_master.h
    uint32_t frequency;          /* I2C frequency */
    uint16_t address;            /* I2C address (7- or 10-bit) */
    uint8_t addrlen;             /* I2C address length (7 or 10 bits) */
};

struct i2c_msg_s {
    //nuttx/include/nuttx/i2c/i2c_master.h
    uint32_t frequency; //a partir de i2c_config_s
    uint16_t addr;      //a partir de i2c_config_s
    uint16_t flags;     //Flags for read/write, etc.=> Transfer direction bit (0 = WRITE, 1 = READ)
    uint8_t *buffer;    //Buffer to read/write data
    long length;        //Length of the buffer
};

int32_t transfer (struct i2c_master_s *dev, struct i2c_msg_s *msgs, int count) {
    for(;;) {
        //nuttx/drivers/i2c/i2c_driver.c
        //1-Check if dev is NULL
        //2-Check if msgs is NULL
        //3-Check if count > 0
        //4-Call the transfer function of the I2C driver (e.g., stm32_i2c_v2.c)
        //5-Return the result of the transfer
    }

}

int32_t i2c_read (struct i2c_master_s *dev, struct i2c_config_s *config, uint8_t *buffer, int buflen) {
    /* Implementada en  nuttx/drivers/i2c/i2c_read.c => f() propia de I2C, no del µC
    La función I2C_READ() se encarga de LEER UN BLOQUE DE DATOS DESDE un dispositivo I2C (aka motor, sensor) */

  struct i2c_msg_s msg;
  unsigned int flags;
  int ret;
    
  flags = (config->addrlen == 10) ? I2C_M_TEN : 0; //7- or 10-bit address

  /* Setup for the transfer */
  msg.frequency = config->frequency,
  msg.addr      = config->address,
  msg.flags     = (flags | I2C_M_READ);
  msg.buffer    = buffer;
  msg.length    = buflen;

  /* Then perform the transfer. */
  ret = I2C_TRANSFER(dev, &msg, 1);
  return (ret >= 0) ? OK : ret;

  /* GARBAGE BY MYSELF 

    struct i2c_msg_s msg[1]; //[0]: address del device, [1]: register address to read from
    msg[0].frequency = config->frequency; //I2C bus frequency
    msg[0].addr = config->address;         //I2C slave address
    msg[0].flags = I2C_M_READ;             //Read data from slave to master -Transfer direction bit-
    msg[0].buffer = buffer;                //Buffer to read data -empty ??-
    msg[0].length = 1;                     //Length of the buffer

    msg[1].frequency = config->frequency; //I2C bus frequency
    msg[1].addr = config->address;         //I2C slave address
    msg[1].flags = I2C_M_READ;             //Read data from slave to master
    msg[1].buffer = buffer;                //Buffer to read data -register address to read from-
    msg[1].length = buflen;                //Length of the buffer
    */
}

int32_t i2c_write (struct i2c_master_s *dev, struct i2c_config_s *config, uint8_t *buffer, int buflen) {
{
    /* Implementada en  nuttx/drivers/i2c/i2c_write.c => f() propia de I2C, no del µC
    La función I2C_WRITE() se encarga de ESCRIBIR UN BLOQUE DE DATOS A un dispositivo I2C (aka motor, sensor) */;

  struct i2c_msg_s msg;
  int ret;

  /* Setup for the transfer */
  msg.frequency = config->frequency,
  msg.addr      = config->address;
  msg.flags     = (config->addrlen == 10) ? I2C_M_TEN : 0;
  msg.buffer    = (uint8_t *)buffer;  /* Override const */
  msg.length    = buflen;

  /* Then perform the transfer. */
  ret = I2C_TRANSFER(dev, &msg, 1);
  return (ret >= 0) ? OK : ret;
}


int i2c_register(struct i2c_master_s *i2c, int bus){
    //nuttx/drivers/i2c/i2c_driver.c
    struct i2c_drvr_s *priv;

    /* Allocate a I2C character device structure */
    priv = kmm_zalloc(sizeof(struct i2c_drvr_s));

    /* Initialize the I2C character device structure */
    priv->i2c = i2c; //La struct i2c_master_s *i2c ha sido creada e inicializada previamente ...
    
    //Ejemplo: "/dev/i2c/1" para el bus 1, "/dev/i2c/2" para el bus 2, etc.
    const char *devName = "/dev/i2c/xyz"; //generar el devPath o hardcodearlo
    
    //register_driver(char *devName, struct file_operations *f_ops, int mode, char *priv)
    register_driver(devName, &g_i2c_drvr_ops, 0666, priv /* i2c_drvr_s */); 
}

//-------------- BMI160 --------------//
struct bmi160_dev_s {
    struct i2c_master_s *i2c;
    int address;
    int frq;
};

struct bmi160_fops_s {
    int (*bmi160_open)(struct file* filep);
    int (*bmi160_close)(struct file* filep);
    int (*bmi160_read)(struct file* filep, char *buffer, int buflen);
    int (*bmi160_write)(struct file* filep, char *buffer, int buflen);
};

int bmi160_register(const char *devpath, struct i2c_master_s *dev){
    //Hay que crear el file device + el inode asociado
    struct inode *i_node = kmalloc(sizeof(struct inode));
    i_node->i_name = devpath;
    i_node->u = &file_ops_s;

    struct bmi160_dev_s *dev;
    
    

    //driver_register(char *devName, struct file_operations *f_ops, int mode, char *priv)
    driver_register(devpath, &g_bmi160_fops, 0666, dev);

}

int bmi160_open(char* devPath, int flags){
    //create a connection we can use to read/write/...

    //set-up sensor

}

int bmi160_read(int fd, char* buffer, int buflen){
    struct file *filep;
    read(fd, *buffer, buflen);
}

int bmi160_write(int fd, char* buffer, int buflen){
    //
}