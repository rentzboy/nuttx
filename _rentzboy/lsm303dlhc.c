#include <lsm303dlhc_driver.h>
#include <lsm303dlhc_gemini.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include "stm32_i2c.h"
#include <sys/types.h>
#include <kmalloc.h>


#include <nuttx/config.h>

#define LSM303DLHC_ACC_ADDRESS 0x19
#define LSM303DLHC_MAG_ADDRESS 0x1E

enum bus
{
    I2C1,
    I2C2,
    I2C3,
    I2C4
};

struct lsm303dlhc_s
{
    uint8_t ACC_address;
    uint8_t MAG_address;
    uint16_t frequency;
    struct i2c_master_s *i2c; //{}
};

struct g_lsm303dlhc_s
{
    lsm303dlhc_open, //setup sensor
    lsm303dlhc_close,
    lsm303dlhc_read,
    lsm303dlhc_write,
    lsm303dlch_ioctl
};


int8_t board_lsm303dlhc_initialize(uint8_t bus)
{
    DEBUGASSERT(bus >= I2C1 && bus <= I2C4);
    
    //FIRST: Initialize the I2C bus
    struct i2c_master_s *i2c;

    syslog(LOG_INFO, "Initializing LSM303DLHC!\n");

    i2c = stm32_i2cbus_initialize(bus);
    if (i2c == NULL)
    {
        syslog(LOG_ERR, "ERROR: Failed to get I2C%d interface\n", bus);
    }
    else
    {
        ret = i2c_register(i2c, bus);
        if (ret < 0)
        {
            syslog(LOG_ERR, "ERROR: Failed to register I2C%d driver: %d\n",
                   bus, ret);
            stm32_i2cbus_uninitialize(i2c);
        }
    }

    
    return 0;
}

/**
 * @brief Register the LSM303DLHC device driver.
 *
 * This function allocates memory for the LSM303DLHC device structure,
 * initializes the device addresses and frequency, and registers the
 * device driver with the system using the provided I2C interface.
 * @note Hay que llamar desde board_lsm303dlhc_initialize(),
 * que es donde se inicializa el bus I2C que vamos a pasar como parámetro

 * @param i2c Pointer to the I2C master interface used for communication.
 * @return Returns 0 on successful registration, or -1 on failure.
 */

int8_t register_lsm303dlhc(struct i2c_master_s *i2c)
{
    DEBUGASSERT(i2c != NULL);

    struct lsm303dlhc_s dev*;
    dev = kmm_zalloc(sizeof(struct lsm303dlhc_s));
    if (dev == NULL)
    {
        syslog(LOG_ERR, "malloc for lsm303dlhc failed");
        return -1;
    }

    dev->ACC_address = LSM303DLHC_ACC_ADDRESS;
    dev->MAG_address = LSM303DLHC_MAG_ADDRESS;
    dev->frequency = LSM303DLHC_I2C_FREQUENCY; //from menuconfig

    //PENDING
    switch (bus)
    {
    case I2C1:
        dev->i2c = (struct i2c_master_s *)stm32_i2c1v2_initialize(1);
        break;
    
    case I2C2:
        /* code */
        break;
    
    case I2C3:
        /* code */
        break;
    
    case I2C4:
        /* code */
        break;
    
    default:
        break;
    }

    dev->i2c = bus;
 
    int8_t ret;
    ret =register_driver("/dev/lsm303dlhc", &g_lsm303dlhc_s, 0666, dev);
    if(ret)
    {
        syslog(LOG_ERR, "Driver notregistered");
        return -1;
    }
};

/* 
solo para ayuda
struct i2c_msg_s
{
  uint32_t frequency;
  uint16_t addr; 
  uint16_t flags;
  FAR uint8_t *buffer;
  ssize_t length;
};
*/

/**
 * @brief Reads one byte from the specified register of the device at the 
 * provided address.
 * 
 * @param devAddress The I2C address of the device to be read. 
 * Valid addresses are LSM303DLHC_ACC_ADDRESS and LSM303DLHC_MAG_ADDRESS.
 * @param register The register to read.
 * 
 * @return The value of the register, or -1 if the address is invalid.
 */
int8_t read_8bits_LSM303DLHC(uint8_t devAddress, uint8_t register)
{
    if(devAddress != LSM303DLHC_ACC_ADDRESS && devAddress != LSM303DLHC_MAG_ADDRESS)
    {
        syslog(LOG_ERR, "Invalid device address");
        return -1;  
    }
    
    struct lsm303dlhc_s *dev; //new instancia, los campos están vacíos
    struct i2c_msg_s m[1]; //array de 1 posición
    uint8_t buffer[1];

    buffer[0] = register;

    m[0].frequency = LSM303DLHC_I2C_FREQUENCY;
    m[0].addr = devAddress; //LSM303DLHC_ACC_ADDRESS || LSM303DLHC_MAG_ADDRESS
    m[0].flags = I2C_M_READ;
    m[0].buffer = buffer;
    m[0].length = 1;

    I2C_TRANSFER(d, m, 1)
};

uint16_t read_16bits_LSM303DLHC(int register){};

int8_t write_8bits_LSM303DLHC(uint8_t devAddress, uint8_t register, uint8_t value)
{
    if(devAddress != LSM303DLHC_ACC_ADDRESS && devAddress != LSM303DLHC_MAG_ADDRESS)
    {
        syslog(LOG_ERR, "Invalid device address");
        return -1;  
    }

    struct lsm303dlhc_s *dev;
    struct i2c_msg_s m[1]; //array de 1 posición
    uint8_t buffer[2];

    buffer[0] = register;
    buffer[1] = value;
    
    m[0].frequency = LSM303DLHC_I2C_FREQUENCY;
    m[0].addr = devAddress; //LSM303DLHC_ACC_ADDRESS || LSM303DLHC_MAG_ADDRESS
    m[0].flags = 0; //Write
    m[0].buffer = buffer;
    m[0].length = 2;

    I2C_TRANSFER(d, m, 1)
};

int8_t write_n_registers_LSM303DLHC(int8_t register, int8_t buffer*, int8_t counter){};

int read_n_registers_LSM303DLHC(register, buffer*, counter){};




/*
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
*/