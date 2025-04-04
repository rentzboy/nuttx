#include "arm_internal.h"
#include "stm32f30xxx_rcc.h"
#include "stm32f30xxx_pinmap_legacy.h"
#include "stm32_gpio.h"
#include "stm32_i2c.h"


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
    //Timing: Timeout registers 1 & 2 ..... no se como se configura

    //Interrupts

    //DMA

    //Set the PE bit in the I2C_CR1 register and interrupts
    regval = getreg32(STM32_I2C_CR1);
    regval |= (I2C_CR1_TXIE | I2C_CR1_ADDRIE | I2C_CR1_PE); //TX, RX Interrup Enable, Peripheral Enable
    putreg32(regval, STM32_RCC_AHBENR);

};




int i2c_read(int8_t addr, int8_t reg, uint8_t *data, int8_t len) {
    //Always from master (µC) to slave (sensor)


};

int i2c_write() {
    //

};