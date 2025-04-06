//pruebas driver bmi160

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <stddef.h>

#define FAR 

/*  1-device struct: bmi_dev_s {i2c_master, addr, freq}. Una instancia de este struct se registra con driver_register()
    2-file operations struct for the device/sensor: file_ops {bmi_read, bmi_write, ...}
    3-file functions definition: open, close, read, write, ...
    4-private/auxiliary functions to read/write the sensor´s registers {get_reg8, set_reg8, get_reg16, set_reg16, ...}
    5-register function: bmi_register()....call driver_register()
    y al utilizar IC2 además .....
    1-struct ic2_master_s{bus, ops, priv}
    2-struct i2c_msg_s {frequency, addr, flags, buffer, length}
    3-struct i2c_ops_s {transfer}
    4-struct i2c_transfer_s {msgv, msgc}
    5-struct i2c_config_s {address, addrlen, frequency}
    6-register function: i2c_register()    
    */


//SENSOR Forward declarations *************************
struct bmi_dev_s {
    struct ic2_dev_master_s *i2c_master;
    uint8_t addr; //I2C address
    int freq;     //I2C frequence
};
struct file_ops {
    //Hay que mantener este orden para que funcione
    bmi160_open,
    bmi160_close,
    bmi160_read,
    NULL,   //write
    NULL,   //seek
    bmi160_ioctl
};

//No hay que pasarle bmi_dev_s pues desde la función open() ya se llama al open registrado para este sensor.
int bmi160_open (char devPath, int mode)
{
    //basic configurations: get from pdf using read_8bits, read_16bits, put_8bits, put_16ibts
}

int bmi160_read(int fd, char* buffer, int bufferLenght)
{
    
}



//I2C Forward declarations **************************
struct i2c_msg_s {
    uint32_t frequency;
    uint16_t addr;     
    uint16_t flags;    
    uint8_t *buffer;
    ssize_t length;
};
struct i2c_ops_s {
    int (*transfer)(FAR struct i2c_master_s *dev, FAR struct i2c_msg_s *msgv, int msgc);
};
struct i2c_transfer_s {
    struct i2c_msg_s *msgv;
    int msgc;
};
struct i2c_config_s {
    uint16_t address;
    uint8_t addrlen;
    uint32_t frequency;
};

//END Forward declarations


static int i2cdrvr_open(FAR struct file *filepath)
{
    struct inode *node = filepath->f_inode;
    struct i2c_driver_s *priv = node->i_private;
}


/* Character driver methods */
static int bmi160_open_prueba(FAR struct file *filepath /* /dev/accel0 */)
{
    //Permite linkar nuestro device con un inode
    //Se trabaja con punteros, la memoria se asigna en el momento de registrar el driver

    //Aunque el usuario únicamente le pasa pues no se le ha asignado antes ningún valor ....
    FAR struct inode *node = filepath->f_inode;
    //TODO: Esta linea no la tengo claro......
    FAR struct bmi_dev_s *dev = node->i_private;

    //Preparar el sensor
    bmi160_set_normal_imu(dev);

    return OK;
}

static int bmi160_close_prueba(FAR struct file *filepath)
{
    //Se asigna el path a nuestro nodo
    FAR struct inode *node = filepath->f_inode;
    //TODO: Esta linea no la tengo claro......
    FAR struct bmi_dev_s *dev = node->i_private;

  /* Set suspend mode to each sensors. */
  bmi160_putreg8(dev, BMI160_CMD, ACCEL_PM_SUSPEND);
  up_mdelay(30);

  bmi160_putreg8(dev, BMI160_CMD, GYRO_PM_SUSPEND);
  up_mdelay(30);

    return OK;
}

static ssize_t bmi160_read_prueba(FAR struct file *filepath, FAR char *buffer, size_t len)
{
    //Se asigna el path a nuestro nodo
    FAR struct inode *node = filepath->f_inode;
    //TODO: Esta linea no la tengo claro......
    FAR struct bmi_dev_s *dev = node->i_private;

    struct i2c_msg_s msg;
    msg.frequency = dev->freq;
    msg.addr = dev->addr;
    msg.buffer = buffer;
    msg.length = len;

    //i2c_read(FAR struct i2c_master_s *dev, FAR const struct i2c_config_s *config, FAR uint8_t *buffer, int buflen);
    return i2c_read(dev->i2c_master, FAR const struct i2c_config_s *config, FAR uint8_t *buffer, int buflen);
}

static int bmi160_ioctl_prueba(FAR struct file *filep, int cmd, unsigned long arg);

struct bmi_ops_s fops {
    CODE (bmi_open*)(const char filepath*, const char mode*);
    CODE (bmi_close*)(const char filepath*););
    CODE (bmi_read*)(void);
    CODE (bmi_write*)(void);
    CODE (bmi_ioctl*)(void);
    CODE (bmi_poll*)(void);
    CODE (bmi_unlink*)(void);
};

