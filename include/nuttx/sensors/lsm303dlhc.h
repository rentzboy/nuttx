/****************************************************************************
 * include/nuttx/sensors/lsm303dlhc.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_SENSORS_LSM303DLHC_H
#define __INCLUDE_NUTTX_SENSORS_LSM303DLHC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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


#define ACC_SPEED_400_KHZ       (uint8_t)0x70 // 400 KHz
#define ACC_SPEED_200_KHZ       (uint8_t)0x60 // 200 KHz
#define ACC_SPEED_100_KHZ       (uint8_t)0x50 // 100 KHz
#define ACC_LOW_POWER_EN        (uint8_t)0x08 // Habilita el modo bajo consumo, desactivado por defecto
#define ACC_XEN                 (uint8_t)0x01 // Habilita el eje X, activado por defecto
#define ACC_YEN                 (uint8_t)0x02 // Habilita el eje Y, activado por defecto
#define ACC_ZEN                 (uint8_t)0x04 // Habilita el eje Z, activado por defecto

#define ACC_BDU_EN              (uint8_t)0x80 // Habilita el modo BDU (block data update), desactivado por defecto
#define ACC_SCALE_2G            (uint8_t)0x00 // Escala 2g, activada por defecto
#define ACC_SCALE_4G            (uint8_t)0x10 // Escala 4g, descactivada por defecto
#define ACC_SCALE_8G            (uint8_t)0x20 // Escala 8g, desactivada por defecto
#define ACC_SCALE_16G           (uint8_t)0x30 // Escala 16g, desactivada por defecto
#define ACC_HIGH_RES_EN         (uint8_t)0x08 // Habilita la resolución alta, desactivado por defecto

#define ACC_FIFO_EN             (uint8_t)0x40 // Habilita el buffer FIFO, desactivado por defecto

#define LSM303DLHC_READ_MULTIPLE_BYTES(n)  \
 (n | (1 << 7)) // Bit de autoincremento del pointer to read

#define LSM303DLHC_I2C_FREQUENCY(n)   \
 (n == 400000 ? ACC_SPEED_400_KHZ :   \
  (n == 100000 ? ACC_SPEED_100_KHZ :  \
    (n == 200000 ? ACC_SPEED_200_KHZ :\
       ACC_SPEED_400_KHZ)))


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

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct lsm303dlhc_acc_data_s {
  int16_t x_axis_acc;
  int16_t y_axis_acc;
  int16_t z_axis_acc;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: lsm303dlhc_register
 *
 * Description:
 *   Register the LSM303DLHC device as a character driver.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/accel0".
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             the LSM303DLHC.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/
int lsm303dlhc_register(FAR const char *devpath,
                        FAR struct i2c_master_s *i2c);
#endif /* __INCLUDE_NUTTX_SENSORS_LSM303DLHC_H */
