#include <stdio.h>
#include <stdint.h>
#include <stddef.h>

#include "arm_internal.h"
#include "stm32f30xxx_rcc.h"
#include "stm32f30xxx_pinmap_legacy.h"
#include "stm32_gpio.h"
#include "stm32_i2c.h"

#define SETUPI2C(i2c, a, b, c)       \
    do                            \
    {                             \
        (i2c)->hz = (a);          \
        (i2c)->address = (b);     \
        i2c->addressLenght = (c); \
    } while (0); //execute 1 time only
    
#define TRANSFERI2C(i2c, a, b, c)    \
    do                            \
    {                             \
        (i2c)->hz = (a);          \
        (i2c)->address = (b);     \
        i2c->addressLenght = (c); \
    } while (0); //execute 1 time only



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
    strncpy(i_node->i_name, devName, strlen(devName)); //e.g., "/dev/i2c1" => error, solo se va a copiar 1 char en i_name[1]
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
        I2C_SETUP(i2c_drvr->i2c); //lower half driver, defined in the STM32 I2C driver (stm32_i2c_v2.c)
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
    uint16_t flags;     //Flags for read/write, etc (i2c_master.h)=> Transfer direction bit (0 = WRITE, 1 = READ)
    uint8_t *buffer;    //Buffer to read/write data
    long length;        //Length of the buffer
};

struct i2c_transfer_s {
    struct i2c_msg_s *msgv;
    int msgc;
};

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
    msg.flags     = (config->addrlen == 10) ? I2C_M_TEN : 0; //Transfer direction bit (0 = WRITE)
    msg.buffer    = (uint8_t *)buffer;  /* Override const */
    msg.length    = buflen;

    /* Then perform the transfer. */
    ret = I2C_TRANSFER(dev, &msg, 1);
    return (ret >= 0) ? OK : ret;
}

int i2c_writeread(FAR struct i2c_master_s *dev, const struct i2c_config_s *config,
                  FAR const uint8_t *wbuffer, int wbuflen, uint8_t *rbuffer, int rbuflen)
{
    //nuttx/drivers/i2c/i2c_writeread.c
    struct i2c_msg_s msg[2];
    unsigned int flags;
    int ret;

    /* 7- or 10-bit address? */
    DEBUGASSERT(config->addrlen == 10 || config->addrlen == 7);
    flags = (config->addrlen == 10) ? I2C_M_TEN : 0;

    /* Format two messages: The first is a write which is never terminated
    * with STOP condition.
    */

    msg[0].frequency  = config->frequency,
    msg[0].addr       = config->address;
    msg[0].flags      = flags | I2C_M_NOSTOP;
    msg[0].buffer     = (FAR uint8_t *)wbuffer;  /* Override const */
    msg[0].length     = wbuflen;

    /* The second is either a read (rbuflen > 0) with a repeated start or a
    * write (rbuflen < 0) with no restart.
    */

    if (rbuflen > 0)
        {
        msg[1].flags  = (flags | I2C_M_READ);
        }
    else
        {
        msg[1].flags  = (flags | I2C_M_NOSTART);
        rbuflen       = -rbuflen;
        }

    msg[1].frequency  = config->frequency,
    msg[1].addr       = config->address;
    msg[1].buffer     = rbuffer;
    msg[1].length     = rbuflen;

    /* Then perform the transfer. */
    ret = I2C_TRANSFER(dev, msg, 2); //msg al ser un array se pasa sin el &
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

//-------------- STM32 I2C --------------//
struct stm32_i2c_config_s
{
    uint32_t base;              /* I2C base address */
    uint32_t clk_bit;           /* Clock enable bit */
    uint32_t reset_bit;         /* Reset bit */
    uint32_t scl_pin;           /* GPIO configuration for SCL as SCL */
    uint32_t sda_pin;           /* GPIO configuration for SDA as SDA */
 #ifndef CONFIG_I2C_POLLED
    uint32_t ev_irq;            /* Event IRQ */
    uint32_t er_irq;            /* Error IRQ */
 #endif
};

struct stm32_i2c_priv_s
{
    /* IMPORTANTE: Multiple instances (shared I2Cbus) => utilizamos locks y semaforos */

    /* Port configuration */
    const struct stm32_i2c_config_s *config;

    int refs;                    /* Reference count */
    mutex_t lock;                /* Mutual exclusion mutex */
 #ifndef CONFIG_I2C_POLLED
    sem_t sem_isr;               /* Interrupt wait semaphore */
 #endif
    volatile uint8_t intstate;   /* Interrupt handshake/state (see enum stm32_intstate_e) */
    uint8_t msgc;                /* Message count */
    struct i2c_msg_s *msgv;      /* Message list */
    uint8_t *ptr;                /* Current message buffer */
    uint32_t frequency;          /* Current I2C frequency */
    int dcnt;                    /* Current message bytes remaining to transfer */
    uint16_t flags;              /* Current message flags */
    bool astart;                 /* START sent */

    /* I2C trace support */
 #ifdef CONFIG_I2C_TRACE
    int tndx;                    /* Trace array index */
    clock_t start_time;          /* Time when the trace was started */

    /* The actual trace data */
    struct stm32_trace_s trace[CONFIG_I2C_NTRACE];
 #endif
    uint32_t status;             /* End of transfer SR2|SR1 status */
};

struct stm32_i2c_inst_s
{
    const struct i2c_ops_s  *ops;  /* Standard I2C operations */
    struct stm32_i2c_priv_s *priv; /* Common driver private data structure */
};


//-------------- STM32 I2C BY MYSELF --------------//
static struct i2cInit {
    int filterAN;   //Analog filter
    int filterDI;   //Digital filter
    int addressLen; //7-10bits
    int addressDev; //Device datasheet
    int timing;

}g_i2cInit;

int i2c_register(struct i2c_master_s *i2c_master_dev, int bus) {
    //Clocks config.
    i2c_Clocks_Initialization(bus);
    //GPIO config.
    i2c_GPIO_Initialization(bus);
    //Driver register
    register_driver("devName", &g_i2c_drvr_ops, 0666, i2c_master_dev);
    //I2C config.
    i2c_Initialization(int bus);
}

int i2c_Clocks_Initialization(int bus) {
    //Clock enable APB1 -I2C bus clock-
    uint32_t regval;
    
    //Reset I2C2 peripheral clock first
    regval = getreg32(STM32_RCC_APB1RSTR);
    regval |= RCC_APB1RSTR_I2C2RST;
    putreg32(regval, STM32_RCC_APB1RSTR)

    //Then Enable I2C2 peripheral clock
    regval = getreg32(STM32_RCC_APB1ENR);
    regval |= RCC_APB1ENR_I2C2EN;
    putreg32(regval, STM32_RCC_APB1RSTR)

    //GPIOA clocks
    //Primero reseteo
    regval = getreg32(STM32_RCC_AHBRSTR);
    regval |= RCC_AHBRSTR_IOPARST;
    putreg32(regval, STM32_RCC_AHBENR);
    //Ahora lo activo
    regval = getreg32(STM32_RCC_AHBENR);
    regval |= RCC_AHBENR_IOPAEN;
    putreg32(regval, STM32_RCC_AHBENR);
}

int i2c_GPIO_Initialization(int bus) {
    //GPIO configuration: SDA pin 11 | SCL pin 9
    stm32_configgpio(GPIO_I2C2_SCL_1); //(GPIO_ALT|GPIO_AF4|GPIO_SPEED_50MHz|GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9) -pinmap legacy-
    stm32_configgpio(GPIO_I2C2_SDA_1); //(GPIO_ALT|GPIO_AF4|GPIO_SPEED_50MHz|GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN10) -pinmap legacy-
}

int i2c_Initialization(int bus) {
    /* Master initialization (ver table 295. I2C initialization flowchart pág. 816 Reference Manual)
    [parámetros dependientes del usuario: únicamente la frequencia]
    1- Clear PE bit in I2C_CR1 register
    2- Configure Analog filter ANOFF and Digital filter DNF bits in I2C_CR1 register
    3- Configure PRESC[3:0], SDADEL[3:0], SCLDEL[3:0] SCLL[7:0], SCLH[7:0] bits in I2C_TIMINGR register
       utilizar STM32 Excel tool calculator/table with predefined values
    4- Configure NOSTRETCH bit in I2C_CR1 register
    5- Set PE bit in I2C_CR1 register

    //Acknowledge: ACK bit from Control register 2                      => initTypeDef
    //NACK: NACK bit from Control register 2                            => initTypeDef
    //PEC: PEC bit from Control register 2                              => initTypeDef
    //Enable Interrupts and/or DMA in I2C_CR1 register */

    if(g_i2cInit.addressLen = 7) 
    {
        //
    }
    else
    {
        //I2C_CR2_ADD10
    }
    if(g_i2cInit.filterAN)
    {

    }
    if(g_i2cInit.filterDI)
    {

    }

    //Set the PE bit in the I2C_CR1 register and interrupts
    regval = getreg32(STM32_I2C_CR1);
    regval |= (I2C_CR1_TXIE | I2C_CR1_ADDRIE | I2C_CR1_PE); //TX, RX Interrup Enable, Peripheral Enable
    putreg32(regval, STM32_RCC_AHBENR);
}

int i2c_setup(struct i2c_cfg, struct i2c_master_s *dev) {
    //POR EL MOMENTO ES LO MISMO QUE I2C_Initialization
}

int32_t transfer (struct i2c_master_s *dev, struct i2c_msg_s *msgs, int count) {
    /* CALL from: i2c_read(), i2c_write(), i2c_writeRead(). 
    count: el # de mensajes a transferir -each msg contains a buffer and a length- */
    /* Master PRE-transfer:
    1- Clear PE: PE bit from I2C_CR1 register                            => from code
    2- Config Analog/Digital filters: ANOFF and DNF in I2C_CR1 register  => from code
    3- Transfer mode (MHz): I2C_TIMINGR register                         => i2c_msg_s->frequency
    4- [optional]Configure NOSTRETCH bit in I2C_CR1 register             => from code
    5- Addressing mode (7-bit or 10-bit): ADD10 in I2C_CR2 register      => i2c_msg_s->flags
    6- Slave address to be sent: SADD[9:0] in I2C_CR2 register           => i2c_msg_s->addr
    7- Transfer direction: RD_WRN bit from Control register 2            => i2c_msg_s->flags
    8- Number of bits to transfer: NBYTES[7:0] from Control register 2   => i2c_msg_s->length
    If the number of bytes is equal to or greater than 255 bytes, NBYTES[7:0] = 0xFF
    9- Set PE: PE bit from I2C_CR1 register                            => from code
    10- Start: START bit from Control register 2                          => from code
    11- STOP: STOP bit from Control register 2                            => from code 
    */
   
    for(;;) {
        //nuttx/drivers/i2c/i2c_driver.c
        //1-Check if dev is NULL
        //2-Check if msgs is NULL
        //3-Check if count > 0
        //4-Call the transfer function of the I2C driver (e.g., stm32_i2c_v2.c)
        //5-Return the result of the transfer
    }

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

    struct bmi160_dev_s *priv;
    priv = (FAR struct bmi160_dev_s *)kmm_malloc(sizeof(struct bmi160_dev_s));
    priv->i2c = dev;
    priv->addr = BMI160_I2C_ADDR;
    priv->freq = BMI160_I2C_FREQ;

    register_driver(devpath, &g_bmi160_fops_s, 0666, priv); /* priv = bmi160_dev_s */
}

//Lower half driver interface for BMI160 character device
int bmi160_open(struct file *filep){
    //Call type: open("/dev/bmi160", O_RDONLY)
    FAR struct bmi160_dev_s *priv  = filep->i_node->i_priv;//xxx_dev_s

    /* Set accel & gyro as normal mode. */
    bmi160_putreg8(priv, BMI160_CMD, ACCEL_PM_NORMAL);
    up_mdelay(30);
    bmi160_putreg8(priv, BMI160_CMD, GYRO_PM_NORMAL);
    up_mdelay(30);

    /* Set accel & gyro output data rate. */
    bmi160_putreg8(priv, BMI160_ACCEL_CONFIG,
                    ACCEL_NORMAL_AVG4 | ACCEL_ODR_100HZ);
    bmi160_putreg8(priv, BMI160_GYRO_CONFIG,
                    GYRO_NORMAL_MODE | GYRO_ODR_100HZ);

}

int bmi160_read(int fd, char* buffer, int buflen){
    //Call type: read(fd, &data, sizeof(struct accel_gyro_st_s));
    FAR struct bmi160_dev_s *priv  = filep->i_node->i_priv;//xxx_dev_s
    FAR struct accel_gyro_st_s *p = (FAR struct accel_gyro_st_s *)buffer;

    bmi160_getregs(priv, BMI160_DATA_8, (FAR uint8_t *)buffer, 15); //va a leer 15 registros desde el 

    /* Adjust sensing time into 24 bit */
    p->sensor_time >>= 8;

    return len;
}

int bmi160_write(int fd, char* buffer, int buflen){
    //
}

//Auxiliary functions for BMI160 communication
void bmi160_putreg8(FAR struct bmi160_dev_s *priv, uint8_t regaddr, uint8_t regval)
{
    /* Porqué no se utiliza i2c_write, parece lo lógico ... ver código de la IA al final */
    struct i2c_msg_s msg[2];
    int ret;
    uint8_t txbuffer[2];

    txbuffer[0] = regaddr;
    txbuffer[1] = regval;

    msg[0].frequency = priv->freq;
    msg[0].addr      = priv->addr;
    msg[0].flags     = 0; //I2C_M_WRITE=0
    msg[0].buffer    = txbuffer;
    msg[0].length    = 2;

    ret = I2C_TRANSFER(priv->i2c, msg, 1);

    /* INICIO del código generado por IA. Revisar si podría funcionar .... */
    struct i2c_config_s config;
    config.frequency = priv->freq;
    config.address = priv->addr;
    config.addrlen = 7; //7-bit address

    //Write the value to the register
    i2c_write(priv->i2c, &config, &value, 1);
    /* FIN codigo generado por IA */
}

uint8_t bmi160_getreg8(struct bmi160_dev_s *priv, uint8_t regaddr)
{
    /* Porqué no se utiliza i2c_writeread, parece lo lógico .....
    quizá el sensor es demasiado complejo y hay que programarlo ad-hoc */
    /* Para enterder las flags: /nuttx/include/nuttx/i2c/i2c_master.h */
    uint8_t regval = 0;
    struct i2c_msg_s msg[2];
    int ret;
    //First block: Write the register address
    msg[0].frequency = priv->freq;
    msg[0].addr      = priv->addr;
    msg[0].flags     = I2C_M_NOSTOP; //WRITE=0, no hace falta indicarlo
    msg[0].buffer    = &regaddr;
    msg[0].length    = 1;
    //Second block: Read the register value
    msg[1].frequency = priv->freq;
    msg[1].addr      = priv->addr;
    msg[1].flags     = I2C_M_READ;
    msg[1].buffer    = &regval;
    msg[1].length    = 1;

    ret = I2C_TRANSFER(priv->i2c, msg, 2);
    return regval;
}
int bmi160_checkid(struct bmi160_dev_s *priv) {
    uint8_t id;

    id = bmi160_getreg8(priv, BMI160_CHIP_ID);
    if (id != BMI160_CHIP_ID_VALUE) {
        snerr("Wrong Device ID: %02x\n", id);
        return -ENODEV;
    }

    sninfo("Device ID: %02x\n", id);
    return OK;
}
