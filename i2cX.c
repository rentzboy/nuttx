#include "arm_internal.h"
#include "stm32f30xxx_rcc.h"
#include "stm32f30xxx_pinmap_legacy.h"
#include "stm32_gpio.h"
#include "stm32_i2c.h"

#define SETUPI2C(i2c, b, c)       \
    do                            \
    {                             \
        (i2c)->hz = (a);          \
        (i2c)->address = (b);     \
        i2c->addressLenght = (c); \
    } while (0); //execute 1 time only
    
#define TRANSFERI2C(i2c, b, c)    \
    do                            \
    {                             \
        (i2c)->hz = (a);          \
        (i2c)->address = (b);     \
        i2c->addressLenght = (c); \
    } while (0); //execute 1 time only

//Struct declaration
struct cfg {
    int address;
    int addressLenght;
    int frequence;
};

struct msg {
    int *buffer;
    int len;
};

struct i2c {
    struct cfg;
    struct msg;
};

//Struct initialization type1
struct cfg cfg = {
    .address = 0x87,
    .addressLenght = 8,
    .frequence = 400
};

//Struct initialization type2
struct cfg cfg = {0x87, 8, 400};

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

int i2c_transfer(uint8_t *data, int8_t register) {
    /* Master PRE-transfer:
    [parámetros dependientes del usuario: addressing mode, slave address, ¿transfer direction?, number of bytes]
    1- Addressing mode (7-bit or 10-bit): ADD10 in I2C_CR2 register      => i2c_cfg
    2- Slave address to be sent: SADD[9:0] in I2C_CR2 register           => i2c_cfg
    3- Transfer direction: RD_WRN bit from Control register 2            => f(read | write)
    4- Number of bits to transfer: NBYTES[7:0] from Control register 2   => i2c_msg
       If the number of bytes is equal to or greater than 255 bytes, NBYTES[7:0] = 0xFF,
       You must then set the START bit in I2C_CR2 register.
    6- Start: START bit from Control register 2                          => from code
    7- PE Enable: PE bit from Control register 2                         => from code
    8- STOP: STOP bit from Control register 2                            => from code 
    */
}

int i2c_read(int8_t addr, int8_t reg, uint8_t *data, int8_t len) {
    //Master receiver mode
    
}

int i2c_write() {
    //Master transmitter mode

}