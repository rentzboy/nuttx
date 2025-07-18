/****************************************************************************
 * nuttx/drivers/sensors/lsm303dlhc.h
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

#ifndef __NUTTX_RENTZBOY_LSM303DLHC_H
#define __NUTTX_RENTZBOY_LSM303DLHC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: register_lsm303dlhc
 *
 * Description:
 *   Register the LSM303DLHC device driver.
 *
 * Input Parameters:
 *   i2c - An instance of the I2C interface to use to communicate with
 *         LSM303DLHC.
 *   bus - The I2C bus number.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int8_t register_lsm303dlhc(struct i2c_master_s *i2c, uint8_t bus);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __NUTTX_RENTZBOY_LSM303DLHC_H */
