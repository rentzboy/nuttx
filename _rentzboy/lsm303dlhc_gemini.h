/**
 * @file lsm303dlhc.h
 * @version 2.0
 * @brief Archivo de cabecera para el driver del sensor 3D acelerómetro y 3D magnetómetro LSM303DLHC.
 *
 * Este archivo define la interfaz del driver para el sensor LSM303DLHC. Incluye las
 * direcciones de los registros, máscaras de bits, y las estructuras de datos necesarias
 * para la comunicación y la interpretación de los datos del sensor a través de I2C.
 * Los comentarios están en español para facilitar su comprensión.
 */

#ifndef LSM303DLHC_H
#define LSM303DLHC_H

#include <stdint.h>

/*******************************************************************************
 ******************* DEFINICIONES Y DIRECCIONES DEL ACELERÓMETRO ***************
 ******************************************************************************/

/**
 * @brief Dirección I2C de 7 bits del acelerómetro.
 * La hoja de datos especifica la dirección de 7 bits como 0011001b.
 * Las direcciones de lectura/escritura se construyen a partir de esta base.
 */
#define ACC_I2C_ADDRESS         0x19 // 0011001b

/* Tabla 17. Mapa de registros del acelerómetro */
#define ACC_CTRL_REG1_A         0x20 // Registro de control 1
#define ACC_CTRL_REG2_A         0x21 // Registro de control 2
#define ACC_CTRL_REG3_A         0x22 // Registro de control 3
#define ACC_CTRL_REG4_A         0x23 // Registro de control 4
#define ACC_CTRL_REG5_A         0x24 // Registro de control 5
#define ACC_CTRL_REG6_A         0x25 // Registro de control 6
#define ACC_REFERENCE_A         0x26 // Referencia para la generación de interrupciones
#define ACC_STATUS_REG_A        0x27 // Registro de estado
#define ACC_OUT_X_L_A           0x28 // Dato LSB (byte menos significativo) del eje X
#define ACC_OUT_X_H_A           0x29 // Dato MSB (byte más significativo) del eje X
#define ACC_OUT_Y_L_A           0x2A // Dato LSB del eje Y
#define ACC_OUT_Y_H_A           0x2B // Dato MSB del eje Y
#define ACC_OUT_Z_L_A           0x2C // Dato LSB del eje Z
#define ACC_OUT_Z_H_A           0x2D // Dato MSB del eje Z
#define ACC_FIFO_CTRL_REG_A     0x2E // Registro de control del buffer FIFO
#define ACC_FIFO_SRC_REG_A      0x2F // Registro de estado/fuente del FIFO
#define ACC_INT1_CFG_A          0x30 // Configuración de la interrupción 1
#define ACC_INT1_SRC_A          0x31 // Registro de fuente de la interrupción 1
#define ACC_INT1_THS_A          0x32 // Umbral de la interrupción 1
#define ACC_INT1_DURATION_A     0x33 // Duración de la interrupción 1
#define ACC_INT2_CFG_A          0x34 // Configuración de la interrupción 2
#define ACC_INT2_SRC_A          0x35 // Registro de fuente de la interrupción 2
#define ACC_INT2_THS_A          0x36 // Umbral de la interrupción 2
#define ACC_INT2_DURATION_A     0x37 // Duración de la interrupción 2
#define ACC_CLICK_CFG_A         0x38 // Configuración de la detección de Click/Doble-Click
#define ACC_CLICK_SRC_A         0x39 // Registro de fuente de Click/Doble-Click
#define ACC_CLICK_THS_A         0x3A // Umbral para la detección de Click
#define ACC_TIME_LIMIT_A        0x3B // Límite de tiempo para la detección de Click
#define ACC_TIME_LATENCY_A      0x3C // Latencia de tiempo para la detección de Doble-Click
#define ACC_TIME_WINDOW_A       0x3D // Ventana de tiempo para la detección de Doble-Click


#define ACC_SPEED_400_KHZ       (uint8_t*)0x70 // 400 KHz
#define ACC_LOW_POWER_EN        (uint8_t*)0x08 // Habilita el modo bajo consumo, desactivado por defecto
#define ACC_XEN                 (uint8_t*)0x01 // Habilita el eje X, desactivado por defecto
#define ACC_YEN                 (uint8_t*)0x02 // Habilita el eje Y, desactivado por defecto
#define ACC_ZEN                 (uint8_t*)0x04 // Habilita el eje Z, desactivado por defecto

#define ACC_BDU_EN              (uint8_t*)0x80 // Habilita el modo BDU (block data update), desactivado por defecto
#define ACC_SCALE_2G            (uint8_t*)0x00 // Escala 2g, activada por defecto
#define ACC_SCALE_4G            (uint8_t*)0x10 // Escala 4g, descactivada por defecto
#define ACC_SCALE_8G            (uint8_t*)0x20 // Escala 8g, desactivada por defecto
#define ACC_SCALE_16G           (uint8_t*)0x30 // Escala 16g, desactivada por defecto
#define ACC_HIGH_RES_EN         (uint8_t*)0x08 // Habilita la resolución alta, desactivado por defecto

#define ACC_FIFO_EN             (uint8_t*)0x40 // Habilita el buffer FIFO, desactivado por defecto



/*******************************************************************************
 ******************* DEFINICIONES Y DIRECCIONES DEL MAGNETÓMETRO ***************
 ******************************************************************************/

/**
 * @brief Dirección I2C de 7 bits del magnetómetro.
 * La hoja de datos especifica la dirección de 7 bits como 0011110b.
 */
#define MAG_I2C_ADDRESS         0x1E // 0011110b

/* Tabla 17. Mapa de registros del magnetómetro */
#define MAG_CRA_REG_M           0x00 // Registro de configuración A
#define MAG_CRB_REG_M           0x01 // Registro de configuración B (ganancia)
#define MAG_MR_REG_M            0x02 // Registro de modo de operación
#define MAG_OUT_X_H_M           0x03 // Dato MSB del eje X
#define MAG_OUT_X_L_M           0x04 // Dato LSB del eje X
#define MAG_OUT_Z_H_M           0x05 // Dato MSB del eje Z
#define MAG_OUT_Z_L_M           0x06 // Dato LSB del eje Z
#define MAG_OUT_Y_H_M           0x07 // Dato MSB del eje Y
#define MAG_OUT_Y_L_M           0x08 // Dato LSB del eje Y
#define MAG_SR_REG_M            0x09 // Registro de estado
#define MAG_IRA_REG_M           0x0A // Registro de identificación A
#define MAG_IRB_REG_M           0x0B // Registro de identificación B
#define MAG_IRC_REG_M           0x0C // Registro de identificación C
#define MAG_TEMP_OUT_H_M        0x31 // Dato MSB del sensor de temperatura
#define MAG_TEMP_OUT_L_M        0x32 // Dato LSB del sensor de temperatura

/*******************************************************************************
 ******************************* ESTRUCTURAS DE DATOS **************************
 ******************************************************************************/

/**
 * @brief Estructura para almacenar los datos crudos del acelerómetro.
 * Los valores son enteros de 16 bits en formato de complemento a dos.
 * Estos valores deben ser convertidos a unidades de 'g' usando la sensibilidad
 * apropiada según la escala de rango completo (full-scale) configurada.
 */
typedef struct {
    int16_t x; // Valor crudo de aceleración en el eje X
    int16_t y; // Valor crudo de aceleración en el eje Y
    int16_t z; // Valor crudo de aceleración en el eje Z
} lsm303dlhc_acc_data_t;

/**
 * @brief Estructura para almacenar los datos crudos del magnetómetro.
 * Los valores son enteros de 16 bits en formato de complemento a dos.
 * Estos valores deben ser convertidos a unidades de 'Gauss' usando la ganancia
 * configurada en el registro CRB_REG_M.
 */
typedef struct {
    int16_t x; // Valor crudo de campo magnético en el eje X
    int16_t y; // Valor crudo de campo magnético en el eje Y
    int16_t z; // Valor crudo de campo magnético en el eje Z
} lsm303dlhc_mag_data_t;

/**
 * @brief Estructura para almacenar los datos crudos de temperatura.
 * El valor es un entero de 12 bits en formato de complemento a dos.
 * La hoja de datos indica una sensibilidad de 8 LSB/°C.
 */
typedef struct {
    int16_t temp; // Valor crudo del sensor de temperatura
} lsm303dlhc_temp_data_t;

/*******************************************************************************
 **************************** INTERFAZ DEL DRIVER ******************************
 ******************************************************************************/

/**
 * @brief Puntero a una función para escribir datos en el bus I2C.
 * Debe ser implementado por el usuario para su plataforma específica.
 * @param handle   Puntero a la configuración del bus I2C (opcional, específico de la plataforma).
 * @param dev_addr Dirección I2C del dispositivo esclavo.
 * @param reg_addr Dirección del registro en el que se va a escribir.
 * @param data     Puntero al buffer con los datos a escribir.
 * @param len      Número de bytes a escribir.
 * @return 0 en caso de éxito, un valor negativo en caso de error.
 */
typedef int32_t (*lsm303dlhc_write_ptr_t)(void *handle, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);

/**
 * @brief Puntero a una función para leer datos del bus I2C.
 * Debe ser implementado por el usuario para su plataforma específica.
 * @param handle   Puntero a la configuración del bus I2C (opcional, específico de la plataforma).
 * @param dev_addr Dirección I2C del dispositivo esclavo.
 * @param reg_addr Dirección del registro desde el que se va a leer.
 * @param data     Puntero al buffer donde se almacenarán los datos leídos.
 * @param len      Número de bytes a leer.
 * @return 0 en caso de éxito, un valor negativo en caso de error.
 */
typedef int32_t (*lsm303dlhc_read_ptr_t)(void *handle, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);

/**
 * @brief Estructura del dispositivo sensor.
 * Esta estructura agrupa los punteros a las funciones de comunicación I2C
 * y un handle opcional para la configuración del bus, haciendo el driver
 * independiente de la plataforma.
 */
typedef struct {
    lsm303dlhc_write_ptr_t write; // Puntero a la función de escritura I2C.
    lsm303dlhc_read_ptr_t  read;  // Puntero a la función de lectura I2C.
    void                  *handle; // Handle para la configuración del bus I2C (ej. puntero a I2C_HandleTypeDef en STM32).
} lsm303dlhc_dev_t;

/*******************************************************************************
 **************************** PROTOTIPOS DE FUNCIONES **************************
 ******************************************************************************/

/**
 * @brief Inicializa el sensor con una configuración básica recomendada.
 * @param dev Puntero a la estructura del dispositivo.
 * @return 0 en caso de éxito, un valor negativo en caso de error.
 */
int32_t lsm303dlhc_init(const lsm303dlhc_dev_t *dev);

/**
 * @brief Lee los datos crudos de los tres ejes del acelerómetro.
 * @param dev Puntero a la estructura del dispositivo.
 * @param acc_data Puntero a la estructura donde se guardarán los datos.
 * @return 0 en caso de éxito, un valor negativo en caso de error.
 */
int32_t lsm303dlhc_get_acc_raw(const lsm303dlhc_dev_t *dev, lsm303dlhc_acc_data_t *acc_data);

/**
 * @brief Lee los datos crudos de los tres ejes del magnetómetro.
 * @param dev Puntero a la estructura del dispositivo.
 * @param mag_data Puntero a la estructura donde se guardarán los datos.
 * @return 0 en caso de éxito, un valor negativo en caso de error.
 */
int32_t lsm303dlhc_get_mag_raw(const lsm303dlhc_dev_t *dev, lsm303dlhc_mag_data_t *mag_data);

/**
 * @brief Lee los datos crudos del sensor de temperatura.
 * @param dev Puntero a la estructura del dispositivo.
 * @param temp_data Puntero a la estructura donde se guardarán los datos.
 * @return 0 en caso de éxito, un valor negativo en caso de error.
 */
int32_t lsm303dlhc_get_temp_raw(const lsm303dlhc_dev_t *dev, lsm303dlhc_temp_data_t *temp_data);

#endif // LSM303DLHC_H
