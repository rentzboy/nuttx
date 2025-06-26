/* Wait semaphore for a transfer to complete */
static inline int stm32_i2c_sem_waitdone(struct stm32_i2c_priv_s *priv)

/* Wait semaphore untill STOP interrupt */
static inline void stm32_i2c_sem_waitstop(struct stm32_i2c_priv_s *priv)

/* Common interrupt service routine (ISR) that handles I2C protocol logic . 
This ISR is activated and deactivated by: stm32_i2c_process and stm32_i2c_sem_waitdone */
static int stm32_i2c_isr_process(struct stm32_i2c_priv_s *priv)

/* Wraper for stm32_i2c_isr_process */
static int stm32_i2c_isr(int irq, void *context, void *arg)

/*Initiates a master mode transaction on the I2C bus to transfer the provided messages to and from the slave devices */
static int stm32_i2c_process(struct i2c_master_s *dev, struct i2c_msg_s *msgs, int count)

/* Wraper for stm32_i2c_process */
static int stm32_i2c_transfer(struct i2c_master_s *dev, struct i2c_msg_s *msgs, int count)


stm32_i2c_transfer [wraper-lower half driver] [activa el lock y llama a ...]
    --> stm32_i2c_process [stm32_i2c_sem_waitstop (looping hasta que el bit I2C_CR2_STOP=0), stm32_i2c_clearinterrupts (all I2C interrupts),
                           stm32_i2c_setclock (trigger a RESET condition), stm32_i2c_enableinterrupts (I2C_CR1_TXRX | I2C_CR1_NACKIE), 
                           stm32_i2c_sendstart (R/W direction, # of bits, 7-10 bits @, flags), 
                           stm32_i2c_sem_waitdone (crital_section(), enable all I2C interrupts -I2C_CR1_ALLINTS-, looping untill transfer completed
                           using nxsem_tickwait_uninterruptible -wait delay hasta desistir si el sem no queda libre-, disable interrupts )]

stm32_i2c_isr [wraper for stm32_i2c_isr_process]
    --> stm32_i2c_isr_process [This ISR is activated and deactivated by: stm32_i2c_process and stm32_i2c_waitdone -in poll mode- ]

/* Setup the I2C hardware, ready for operation with defaults */
static int stm32_i2c_init(struct stm32_i2c_priv_s *priv)

struct i2c_master_s *stm32_i2cbus_initialize(int port) --> stm32_i2c_init


Configurar I2C bus en STM32: sda, sck, pins, mhz, .....
Registrar el I2C driver en el kernel
Funciones upper half driver: open, close, read, write, ioctl, ...
Funciones lower half driver, en f(tipo device): transfer, setup, reset.
Funciones auxiliares  del I2C -static-
Driver del sensor BMI160 para i2c_master_s

