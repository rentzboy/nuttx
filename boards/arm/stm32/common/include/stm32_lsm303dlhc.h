/****************************************************************************
 * boards/arm/stm32/common/include/stm32_lsm303dlhc.h
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

#ifndef __BOARDS_ARM_STM32_COMMON_INCLUDE_STM32_LSM303DLHC_H
#define __BOARDS_ARM_STM32_COMMON_INCLUDE_STM32_LSM303DLHC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * stm32_lsm303dlhc_initialize - Initialize the LSM303DLHC sensor using the I2C
 * interface of the specified bus
 *
 * @param bus The bus number of the I2C interface to use
 *
 * @return 0 on success, -1 on failure
 *
 * This function allocates memory for the LSM303DLHC device structure, initializes
 * the device addresses and frequency, and registers the device driver with the
 * system using the provided I2C interface.
 */
 int stm32_lsm303dlhc_initialize(int bus);

#endif /* __BOARDS_ARM_STM32_COMMON_INCLUDE_STM32_BMP180_H */
