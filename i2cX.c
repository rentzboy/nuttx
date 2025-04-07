#include "arm_internal.h"
#include "stm32f30xxx_rcc.h"
#include "stm32f30xxx_pinmap_legacy.h"
#include "stm32_gpio.h"
#include "stm32_i2c.h"

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

static struct i2cInit {
    int filterAN;   //Analog filter
    int filterDI;   //Digital filter
    int addressLen; //7-10bits
    int addressDev; //Device datasheet
    int timing;

}g_i2cInit; //still accessible using pointers from outside file ? no creo .... habría que declararla en el header

g_i2cInit = {0, 0, 7, 0, 0}; //Default values

//Struct initialization type2
struct cfg cfg = {0x87, 8, 400};

int i2c_register(struct i2c_master_s *device, int bus) {
    //GPIO config.
    i2c2_initialization(bus);
    //Driver register
    register_driver("devName", &g_i2c_drvr_ops, 0666, device);
    //I2C config.



    i2c_setup(cfg_s, dev);
}

int i2c2_clocksInitialization(int bus) {
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

int i2c2_initialization(int bus) {
    uint32_t regval;
    //GPIO configuration: SDA pin 11 | SCL pin 9
    stm32_configgpio(GPIO_I2C2_SCL_1); //(GPIO_ALT|GPIO_AF4|GPIO_SPEED_50MHz|GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9) -pinmap legacy-
    stm32_configgpio(GPIO_I2C2_SDA_1); //(GPIO_ALT|GPIO_AF4|GPIO_SPEED_50MHz|GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN10) -pinmap legacy-

    //Mode, Timing, Filters, Address, ACK
    //Filters: I2C_CR1_ANFOFF & I2C_CR1_DNF from Control register 1
    //10 bits address: I2C_CR2_ADD10 from Control register 2
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
    
    //Timing: Timeout registers 1 & 2 ..... no se como se configura

}

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

int i2c_setup(struct i2c_cfg, struct i2c_master_s *dev) {
/*  Master initialization
    Addressing mode (7-bit or 10-bit): ADD10 in I2C_CR2 register
    Slave address to be sent: SADD[9:0] in I2C_CR2 register
    Transfer direction: RD_WRN in I2C_CR2 register => no entiendo por qué aqui .....
    The number of bytes to be transferred: NBYTES[7:0] in I2C_CR2 register
    If the number of bytes is equal to or greater than 255 bytes, NBYTES[7:0] = 0xFF
    You must then set the START bit in I2C_CR2 register.
    

    Enable Interrupts and/or DMA in I2C_CR1 register
    */

    //Set the PE bit in the I2C_CR1 register and interrupts
    regval = getreg32(STM32_I2C_CR1);
    regval |= (I2C_CR1_TXIE | I2C_CR1_ADDRIE | I2C_CR1_PE); //TX, RX Interrup Enable, Peripheral Enable
    putreg32(regval, STM32_RCC_AHBENR);

};

int i2c_transfer(uint8_t *data, int8_t register) {
    //
}

int i2c_read(int8_t addr, int8_t reg, uint8_t *data, int8_t len) {
    //Always from master (µC) to slave (sensor)


}

int i2c_write() {
    //

}