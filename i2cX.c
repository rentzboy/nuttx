#include "arm_internal.h"
#include "stm32f30xxx_rcc.h"
#include "stm32f30xxx_pinmap_legacy.h"
#include "stm32_gpio.h"


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
    //GPIO configuration: SDA pin 11 | SCL pin 9
    stm32_configgpio(GPIO_I2C2_SCL_1); //(GPIO_ALT|GPIO_AF4|GPIO_SPEED_50MHz|GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9) -pinmap legacy-
    stm32_configgpio(GPIO_I2C2_SDA_1); //(GPIO_ALT|GPIO_AF4|GPIO_SPEED_50MHz|GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN10) -pinmap legacy-





        





    





    

    //GPIO configuration -Alternate function-

    //Mode, Timing, Filters, Address, ACK

    //Interrupts

    //DMA

    //Enable I2C
};

int i2c_read() {
    //

};

int i2c_write() {
    //

};